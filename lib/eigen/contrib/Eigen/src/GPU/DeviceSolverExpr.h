// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// Solver expression types for gpu::DeviceMatrix. Each maps 1:1 onto a pair of
// cuSOLVER calls and factors afresh on every assignment; use the gpu::LLT /
// gpu::LU classes when a factorization should be cached across solves.

#ifndef EIGEN_GPU_DEVICE_SOLVER_EXPR_H
#define EIGEN_GPU_DEVICE_SOLVER_EXPR_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

#include <functional>

#include "./FwdDecl.h"

namespace Eigen {
namespace gpu {

/** d_A.llt().solve(d_B), dispatched to cusolverDnXpotrf + cusolverDnXpotrs. */
template <typename Scalar_, int UpLo_ = Lower>
class LltSolveExpr {
 public:
  using Scalar = Scalar_;
  static constexpr int UpLo = UpLo_;

  LltSolveExpr(const DeviceMatrix<Scalar>& A, const DeviceMatrix<Scalar>& B) : A_(A), B_(B) {}
  const DeviceMatrix<Scalar>& matrix() const { return A_; }
  const DeviceMatrix<Scalar>& rhs() const { return B_; }

 private:
  std::reference_wrapper<const DeviceMatrix<Scalar>> A_;
  std::reference_wrapper<const DeviceMatrix<Scalar>> B_;
};

/** d_A.lu().solve(d_B), dispatched to cusolverDnXgetrf + cusolverDnXgetrs. */
template <typename Scalar_>
class LuSolveExpr {
 public:
  using Scalar = Scalar_;

  LuSolveExpr(const DeviceMatrix<Scalar>& A, const DeviceMatrix<Scalar>& B) : A_(A), B_(B) {}
  const DeviceMatrix<Scalar>& matrix() const { return A_; }
  const DeviceMatrix<Scalar>& rhs() const { return B_; }

 private:
  std::reference_wrapper<const DeviceMatrix<Scalar>> A_;
  std::reference_wrapper<const DeviceMatrix<Scalar>> B_;
};

/** d_A.llt(), whose solve() builds an LltSolveExpr. */
template <typename Scalar_, int UpLo_ = Lower>
class LLTView {
 public:
  using Scalar = Scalar_;

  explicit LLTView(const DeviceMatrix<Scalar>& m) : mat_(m) {}

  /** The expression is evaluated on assignment to a gpu::DeviceMatrix. */
  LltSolveExpr<Scalar, UpLo_> solve(const DeviceMatrix<Scalar>& rhs) const { return {mat_, rhs}; }

 private:
  std::reference_wrapper<const DeviceMatrix<Scalar>> mat_;
};

/** d_A.lu(), whose solve() builds a LuSolveExpr. */
template <typename Scalar_>
class LUView {
 public:
  using Scalar = Scalar_;

  explicit LUView(const DeviceMatrix<Scalar>& m) : mat_(m) {}

  /** The expression is evaluated on assignment to a gpu::DeviceMatrix. */
  LuSolveExpr<Scalar> solve(const DeviceMatrix<Scalar>& rhs) const { return {mat_, rhs}; }

 private:
  std::reference_wrapper<const DeviceMatrix<Scalar>> mat_;
};

}  // namespace gpu
}  // namespace Eigen

#endif  // EIGEN_GPU_DEVICE_SOLVER_EXPR_H
