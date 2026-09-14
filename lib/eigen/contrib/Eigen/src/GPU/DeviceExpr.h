// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// Lightweight expression types for DeviceMatrix operations.
//
// These are NOT Eigen expression templates. Each type maps 1:1 to a single
// NVIDIA library call (cuBLAS or cuSOLVER). There is no coefficient-level
// evaluation, no lazy fusion, no packet operations.
//
// Expression types:
//   AdjointView<S>   — d_A.adjoint()  → marks ConjTrans for GEMM
//   TransposeView<S> — d_A.transpose() → marks Trans for GEMM
//   Scaled<Expr>     — alpha * expr    → carries scalar factor
//   gpu::GemmExpr<Lhs, Rhs> — lhs * rhs    → dispatches to cublasXgemm

#ifndef EIGEN_GPU_DEVICE_EXPR_H
#define EIGEN_GPU_DEVICE_EXPR_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

#include "./CuBlasSupport.h"
#include "./type_traits.h"

namespace Eigen {
namespace gpu {
namespace internal {
// SFINAE gate for scalar factors: any type convertible to the expression's
// scalar (so `2 * d_A` and `2.0 * d_cplx` work), except DeviceScalar.
template <typename T, typename S>
using require_host_scalar_convertible_t =
    typename std::enable_if<std::is_convertible<T, S>::value && !is_device_scalar<typename std::decay<T>::type>::value,
                            int>::type;

}  // namespace internal

/** \brief View returned by DeviceMatrix::adjoint(); maps to the cuBLAS conjugate-transpose operand flag. */
template <typename Scalar_>
class AdjointView {
 public:
  using Scalar = Scalar_;
  explicit AdjointView(const DeviceMatrix<Scalar>& m) : mat_(m) {}
  const DeviceMatrix<Scalar>& matrix() const { return mat_; }

 private:
  const DeviceMatrix<Scalar>& mat_;
};

/** \brief View returned by DeviceMatrix::transpose(); maps to the cuBLAS transpose operand flag. */
template <typename Scalar_>
class TransposeView {
 public:
  using Scalar = Scalar_;
  explicit TransposeView(const DeviceMatrix<Scalar>& m) : mat_(m) {}
  const DeviceMatrix<Scalar>& matrix() const { return mat_; }

 private:
  const DeviceMatrix<Scalar>& mat_;
};

/** \brief Expression returned by operator*(Scalar, DeviceMatrix/View), carrying the scalar factor.
 *
 * \c Inner names the scaled operand, decayed, for callers that need to reason
 * about what is being scaled. The is_scaled_* predicates in type_traits.h
 * deliberately do not read it: naming a member instantiates Scaled, and
 * Scaled<GemmExpr<...>>::Scalar is ill-formed. They deduce the operand from the
 * template-id instead.
 */
template <typename Inner_>
class Scaled {
 public:
  using Inner = std::decay_t<Inner_>;
  using Scalar = internal::scalar_type_t<Inner>;
  Scaled(Scalar alpha, const Inner_& inner) : alpha_(alpha), inner_(inner) {}
  Scalar scalar() const { return alpha_; }
  const Inner_& inner() const { return inner_; }

 private:
  Scalar alpha_;
  const Inner_& inner_;
};

/** \brief Expression returned by operator*(lhs_expr, rhs_expr), dispatched to cuBLAS GEMM. */
template <typename Lhs, typename Rhs>
class GemmExpr {
 public:
  using Scalar = internal::scalar_type_t<Lhs>;
  static_assert(std::is_same<Scalar, internal::scalar_type_t<Rhs>>::value,
                "DeviceMatrix GEMM: LHS and RHS must have the same scalar type");

  GemmExpr(const Lhs& lhs, const Rhs& rhs) : lhs_(lhs), rhs_(rhs) {}
  const Lhs& lhs() const { return lhs_; }
  const Rhs& rhs() const { return rhs_; }

 private:
  // Stored by reference — like Eigen's CPU expression templates, these must
  // not be captured with auto (the references will dangle). Assign to (or
  // construct) a DeviceMatrix immediately.
  const Lhs& lhs_;
  const Rhs& rhs_;
};

// Defined after device_expr_traits so it can accept any supported view pair.

// The scalar factor accepts any type convertible to the matrix scalar (int
// and double literals included), in either operand order. Division by a
// scalar and unary minus fold into the same Scaled wrapper.

template <typename T, typename S, internal::require_host_scalar_convertible_t<T, S> = 0>
Scaled<DeviceMatrix<S>> operator*(T alpha, const DeviceMatrix<S>& m) {
  return {static_cast<S>(alpha), m};
}

template <typename T, typename S, internal::require_host_scalar_convertible_t<T, S> = 0>
Scaled<DeviceMatrix<S>> operator*(const DeviceMatrix<S>& m, T alpha) {
  return {static_cast<S>(alpha), m};
}

template <typename T, typename S, internal::require_host_scalar_convertible_t<T, S> = 0>
Scaled<DeviceMatrix<S>> operator/(const DeviceMatrix<S>& m, T alpha) {
  return {S(1) / static_cast<S>(alpha), m};
}

template <typename S>
Scaled<DeviceMatrix<S>> operator-(const DeviceMatrix<S>& m) {
  return {S(-1), m};
}

template <typename T, typename S, internal::require_host_scalar_convertible_t<T, S> = 0>
Scaled<AdjointView<S>> operator*(T alpha, const AdjointView<S>& m) {
  return {static_cast<S>(alpha), m};
}

template <typename T, typename S, internal::require_host_scalar_convertible_t<T, S> = 0>
Scaled<AdjointView<S>> operator*(const AdjointView<S>& m, T alpha) {
  return {static_cast<S>(alpha), m};
}

template <typename T, typename S, internal::require_host_scalar_convertible_t<T, S> = 0>
Scaled<TransposeView<S>> operator*(T alpha, const TransposeView<S>& m) {
  return {static_cast<S>(alpha), m};
}

template <typename T, typename S, internal::require_host_scalar_convertible_t<T, S> = 0>
Scaled<TransposeView<S>> operator*(const TransposeView<S>& m, T alpha) {
  return {static_cast<S>(alpha), m};
}

// Rescale / negate an already-scaled expression: T * (alpha * m), -(alpha * m).
template <typename T, typename Inner,
          internal::require_host_scalar_convertible_t<T, internal::scalar_type_t<Inner>> = 0>
Scaled<Inner> operator*(T alpha, const Scaled<Inner>& s) {
  using S = internal::scalar_type_t<Inner>;
  return {static_cast<S>(alpha) * s.scalar(), s.inner()};
}

template <typename Inner>
Scaled<Inner> operator-(const Scaled<Inner>& s) {
  using S = internal::scalar_type_t<Inner>;
  return {S(-1) * s.scalar(), s.inner()};
}

namespace internal {
// Default: a DeviceMatrix is NoTrans. Documented on the FwdDecl.h forward declaration.
template <typename T>
struct device_expr_traits {
  static constexpr bool is_device_expr = false;
};

template <typename Scalar>
struct device_expr_traits<DeviceMatrix<Scalar>> {
  using scalar_type = Scalar;
  static constexpr GpuOp op = GpuOp::NoTrans;
  static constexpr bool is_device_expr = true;
  static const DeviceMatrix<Scalar>& matrix(const DeviceMatrix<Scalar>& x) { return x; }
  static Scalar alpha(const DeviceMatrix<Scalar>&) { return Scalar(1); }
};

template <typename Scalar>
struct device_expr_traits<AdjointView<Scalar>> {
  using scalar_type = Scalar;
  static constexpr GpuOp op = GpuOp::ConjTrans;
  static constexpr bool is_device_expr = true;
  static const DeviceMatrix<Scalar>& matrix(const AdjointView<Scalar>& x) { return x.matrix(); }
  static Scalar alpha(const AdjointView<Scalar>&) { return Scalar(1); }
};

template <typename Scalar>
struct device_expr_traits<TransposeView<Scalar>> {
  using scalar_type = Scalar;
  static constexpr GpuOp op = GpuOp::Trans;
  static constexpr bool is_device_expr = true;
  static const DeviceMatrix<Scalar>& matrix(const TransposeView<Scalar>& x) { return x.matrix(); }
  static Scalar alpha(const TransposeView<Scalar>&) { return Scalar(1); }
};

template <typename Inner>
struct device_expr_traits<Scaled<Inner>> {
  using scalar_type = scalar_type_t<Inner>;
  static constexpr GpuOp op = device_expr_traits<Inner>::op;
  static constexpr bool is_device_expr = true;
  static const DeviceMatrix<scalar_type>& matrix(const Scaled<Inner>& x) {
    return device_expr_traits<Inner>::matrix(x.inner());
  }
  static scalar_type alpha(const Scaled<Inner>& x) { return x.scalar() * device_expr_traits<Inner>::alpha(x.inner()); }
};
}  // namespace internal

template <typename Lhs, typename Rhs,
          std::enable_if_t<internal::device_expr_traits<Lhs>::is_device_expr &&
                               internal::device_expr_traits<Rhs>::is_device_expr,
                           int> = 0>
GemmExpr<Lhs, Rhs> operator*(const Lhs& a, const Rhs& b) {
  return {a, b};
}

/**
 * \brief Expression that scales a device matrix by a DeviceScalar.
 *
 * Unlike Scaled, this expression carries a device pointer. operator+= dispatches to cuBLAS AXPY with device pointer
 * mode.
 */
template <typename Scalar_>
class DeviceScaledDevice {
 public:
  using Scalar = Scalar_;
  DeviceScaledDevice(const DeviceScalar<Scalar>& alpha, const DeviceMatrix<Scalar>& mat) : alpha_(alpha), mat_(mat) {}
  const DeviceScalar<Scalar>& alpha() const { return alpha_; }
  const DeviceMatrix<Scalar>& matrix() const { return mat_; }

 private:
  const DeviceScalar<Scalar>& alpha_;
  const DeviceMatrix<Scalar>& mat_;
};

// DeviceScalar * DeviceMatrix → DeviceScaledDevice
template <typename S>
DeviceScaledDevice<S> operator*(const DeviceScalar<S>& alpha, const DeviceMatrix<S>& m) {
  return {alpha, m};
}

// Captures `DeviceMatrix + Scaled<DeviceMatrix>` (and reverse).
// Dispatched to geam: C = alpha * A + beta * B.
//
// Note: These operator+/- overloads are intentionally free functions on
// DeviceMatrix, not Eigen expression templates. DeviceMatrix does not inherit
// from MatrixBase, so there is no ambiguity with Eigen's own operator+/-.
// If DeviceMatrix is ever made an Eigen expression type, these would need to
// be revisited.

/** \brief Linear combination of two device matrices. */
template <typename Scalar_>
class DeviceAddExpr {
 public:
  using Scalar = Scalar_;
  DeviceAddExpr(Scalar alpha, const DeviceMatrix<Scalar>& A, Scalar beta, const DeviceMatrix<Scalar>& B)
      : alpha_(alpha), A_(A), beta_(beta), B_(B) {}
  Scalar alpha() const { return alpha_; }
  Scalar beta() const { return beta_; }
  const DeviceMatrix<Scalar>& A() const { return A_; }
  const DeviceMatrix<Scalar>& B() const { return B_; }

 private:
  Scalar alpha_;
  const DeviceMatrix<Scalar>& A_;
  Scalar beta_;
  const DeviceMatrix<Scalar>& B_;
};

// DeviceMatrix + DeviceMatrix → DeviceAddExpr (alpha=1, beta=1)
template <typename S>
DeviceAddExpr<S> operator+(const DeviceMatrix<S>& a, const DeviceMatrix<S>& b) {
  return {S(1), a, S(1), b};
}

// DeviceMatrix + Scaled<DeviceMatrix> → DeviceAddExpr (alpha=1, beta=scaled)
template <typename S>
DeviceAddExpr<S> operator+(const DeviceMatrix<S>& a, const Scaled<DeviceMatrix<S>>& b) {
  return {S(1), a, b.scalar(), b.inner()};
}

// Scaled<DeviceMatrix> + DeviceMatrix → DeviceAddExpr (alpha=scaled, beta=1)
template <typename S>
DeviceAddExpr<S> operator+(const Scaled<DeviceMatrix<S>>& a, const DeviceMatrix<S>& b) {
  return {a.scalar(), a.inner(), S(1), b};
}

// DeviceMatrix - DeviceMatrix → DeviceAddExpr (alpha=1, beta=-1)
template <typename S>
DeviceAddExpr<S> operator-(const DeviceMatrix<S>& a, const DeviceMatrix<S>& b) {
  return {S(1), a, S(-1), b};
}

// DeviceMatrix - Scaled<DeviceMatrix> → DeviceAddExpr (alpha=1, beta=-scaled)
template <typename S>
DeviceAddExpr<S> operator-(const DeviceMatrix<S>& a, const Scaled<DeviceMatrix<S>>& b) {
  return {S(1), a, -b.scalar(), b.inner()};
}

// Scaled<DeviceMatrix> - DeviceMatrix → DeviceAddExpr (alpha=scaled, beta=-1)
template <typename S>
DeviceAddExpr<S> operator-(const Scaled<DeviceMatrix<S>>& a, const DeviceMatrix<S>& b) {
  return {a.scalar(), a.inner(), S(-1), b};
}

// Scaled<DeviceMatrix> ± Scaled<DeviceMatrix> → DeviceAddExpr
template <typename S>
DeviceAddExpr<S> operator+(const Scaled<DeviceMatrix<S>>& a, const Scaled<DeviceMatrix<S>>& b) {
  return {a.scalar(), a.inner(), b.scalar(), b.inner()};
}

template <typename S>
DeviceAddExpr<S> operator-(const Scaled<DeviceMatrix<S>>& a, const Scaled<DeviceMatrix<S>>& b) {
  return {a.scalar(), a.inner(), -b.scalar(), b.inner()};
}
}  // namespace gpu
}  // namespace Eigen

#endif  // EIGEN_GPU_DEVICE_EXPR_H
