// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Pavel Guzenfeld
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_STRUCTURED_BINDINGS_H
#define EIGEN_STRUCTURED_BINDINGS_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

#if EIGEN_MAX_CPP_VER >= 17 && EIGEN_COMP_CXXVER >= 17

#include <tuple>
#include <type_traits>

// Structured bindings support for fixed-size Eigen vectors and matrices.
//
// Enables:
//   Eigen::Vector3d v(1, 2, 3);
//   auto [x, y, z] = v;
//
//   Eigen::Array3i a(4, 5, 6);
//   auto& [a0, a1, a2] = a;
//
// Decomposition order follows storage order: column-major by default,
// so Matrix2d decomposes as (0,0), (1,0), (0,1), (1,1). Only fixed-size
// column-major Matrix and Array are tuple-like; Map, Ref, and fixed-size
// Block intentionally do not participate.

namespace Eigen {
namespace internal {

// Bases supplying value/type only for fixed-size shapes, leaving the std
// specializations below memberless for dynamic sizes: generic tuple-like
// detection probes std::tuple_size<T>::value in a SFINAE context (fmt's range
// formatter, issue #3103) and needs a substitution failure, not a hard error.
// An enable_if_t base-clause cannot replace this: base-class-specifier
// substitution is not a SFINAE context.
template <int Rows_, int Cols_, bool IsFixedSize_ = (Rows_ != Dynamic && Cols_ != Dynamic)>
struct structured_binding_size {};

template <int Rows_, int Cols_>
struct structured_binding_size<Rows_, Cols_, true>
    : std::integral_constant<size_t, static_cast<size_t>(Rows_) * static_cast<size_t>(Cols_)> {};

// Note: uses Idx_ instead of I to avoid conflict with Eigen's test framework macro.
template <size_t Idx_, typename Scalar_, int Rows_, int Cols_,
          bool IsFixedSize_ = (Rows_ != Dynamic && Cols_ != Dynamic)>
struct structured_binding_element {};

template <size_t Idx_, typename Scalar_, int Rows_, int Cols_>
struct structured_binding_element<Idx_, Scalar_, Rows_, Cols_, true> {
  static_assert(Idx_ < static_cast<size_t>(Rows_) * static_cast<size_t>(Cols_), "Index out of range.");
  using type = Scalar_;
};

}  // namespace internal
}  // namespace Eigen

namespace std {

// std::tuple_size / std::tuple_element for Matrix.
template <typename Scalar_, int Rows_, int Cols_, int Options_, int MaxRows_, int MaxCols_>
struct tuple_size<Eigen::Matrix<Scalar_, Rows_, Cols_, Options_, MaxRows_, MaxCols_>>
    : Eigen::internal::structured_binding_size<Rows_, Cols_> {};

template <size_t Idx_, typename Scalar_, int Rows_, int Cols_, int Options_, int MaxRows_, int MaxCols_>
struct tuple_element<Idx_, Eigen::Matrix<Scalar_, Rows_, Cols_, Options_, MaxRows_, MaxCols_>>
    : Eigen::internal::structured_binding_element<Idx_, Scalar_, Rows_, Cols_> {};

// std::tuple_size / std::tuple_element for Array.
template <typename Scalar_, int Rows_, int Cols_, int Options_, int MaxRows_, int MaxCols_>
struct tuple_size<Eigen::Array<Scalar_, Rows_, Cols_, Options_, MaxRows_, MaxCols_>>
    : Eigen::internal::structured_binding_size<Rows_, Cols_> {};

template <size_t Idx_, typename Scalar_, int Rows_, int Cols_, int Options_, int MaxRows_, int MaxCols_>
struct tuple_element<Idx_, Eigen::Array<Scalar_, Rows_, Cols_, Options_, MaxRows_, MaxCols_>>
    : Eigen::internal::structured_binding_element<Idx_, Scalar_, Rows_, Cols_> {};

}  // namespace std

namespace Eigen {

// Until the decomposition order for genuinely 2D RowMajor storage is agreed
// upon, reject row-major matrices at the get<I> level. coeffRef(Index) is
// linear in storage order, so permitting both orientations for 2D types would
// silently flip the binding order between Matrix<T,R,C> and Matrix<T,R,C,RowMajor>.
// Vector types (Rows==1 or Cols==1) are unaffected because storage order
// does not change element order for a 1×N or N×1 shape — and Eigen forces
// row-major on 1×N regardless, so we must allow it for row vectors.
#define EIGEN_STRUCTURED_BINDINGS_ASSERT_COL_MAJOR(ROWS, COLS, OPTIONS)  \
  static_assert((ROWS <= 1 || COLS <= 1) || ((OPTIONS & RowMajor) == 0), \
                "Structured bindings on 2D RowMajor Eigen matrices are disabled to prevent storage-order confusion.");

// get<Idx_> free functions for Matrix.
template <size_t Idx_, typename Scalar_, int Rows_, int Cols_, int Options_, int MaxRows_, int MaxCols_>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar_& get(
    Matrix<Scalar_, Rows_, Cols_, Options_, MaxRows_, MaxCols_>& m) noexcept {
  static_assert(Rows_ != Dynamic && Cols_ != Dynamic, "Structured bindings require fixed-size Eigen types.");
  static_assert(Idx_ < static_cast<size_t>(Rows_ * Cols_), "Index out of range.");
  EIGEN_STRUCTURED_BINDINGS_ASSERT_COL_MAJOR(Rows_, Cols_, Options_);
  return m.coeffRef(static_cast<Index>(Idx_));
}

template <size_t Idx_, typename Scalar_, int Rows_, int Cols_, int Options_, int MaxRows_, int MaxCols_>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const Scalar_& get(
    const Matrix<Scalar_, Rows_, Cols_, Options_, MaxRows_, MaxCols_>& m) noexcept {
  static_assert(Rows_ != Dynamic && Cols_ != Dynamic, "Structured bindings require fixed-size Eigen types.");
  static_assert(Idx_ < static_cast<size_t>(Rows_ * Cols_), "Index out of range.");
  EIGEN_STRUCTURED_BINDINGS_ASSERT_COL_MAJOR(Rows_, Cols_, Options_);
  return m.coeffRef(static_cast<Index>(Idx_));
}

template <size_t Idx_, typename Scalar_, int Rows_, int Cols_, int Options_, int MaxRows_, int MaxCols_>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar_&& get(
    Matrix<Scalar_, Rows_, Cols_, Options_, MaxRows_, MaxCols_>&& m) noexcept {
  static_assert(Rows_ != Dynamic && Cols_ != Dynamic, "Structured bindings require fixed-size Eigen types.");
  static_assert(Idx_ < static_cast<size_t>(Rows_ * Cols_), "Index out of range.");
  EIGEN_STRUCTURED_BINDINGS_ASSERT_COL_MAJOR(Rows_, Cols_, Options_);
  return std::move(m.coeffRef(static_cast<Index>(Idx_)));
}

// get<Idx_> free functions for Array.
template <size_t Idx_, typename Scalar_, int Rows_, int Cols_, int Options_, int MaxRows_, int MaxCols_>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar_& get(
    Array<Scalar_, Rows_, Cols_, Options_, MaxRows_, MaxCols_>& a) noexcept {
  static_assert(Rows_ != Dynamic && Cols_ != Dynamic, "Structured bindings require fixed-size Eigen types.");
  static_assert(Idx_ < static_cast<size_t>(Rows_ * Cols_), "Index out of range.");
  EIGEN_STRUCTURED_BINDINGS_ASSERT_COL_MAJOR(Rows_, Cols_, Options_);
  return a.coeffRef(static_cast<Index>(Idx_));
}

template <size_t Idx_, typename Scalar_, int Rows_, int Cols_, int Options_, int MaxRows_, int MaxCols_>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const Scalar_& get(
    const Array<Scalar_, Rows_, Cols_, Options_, MaxRows_, MaxCols_>& a) noexcept {
  static_assert(Rows_ != Dynamic && Cols_ != Dynamic, "Structured bindings require fixed-size Eigen types.");
  static_assert(Idx_ < static_cast<size_t>(Rows_ * Cols_), "Index out of range.");
  EIGEN_STRUCTURED_BINDINGS_ASSERT_COL_MAJOR(Rows_, Cols_, Options_);
  return a.coeffRef(static_cast<Index>(Idx_));
}

template <size_t Idx_, typename Scalar_, int Rows_, int Cols_, int Options_, int MaxRows_, int MaxCols_>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar_&& get(
    Array<Scalar_, Rows_, Cols_, Options_, MaxRows_, MaxCols_>&& a) noexcept {
  static_assert(Rows_ != Dynamic && Cols_ != Dynamic, "Structured bindings require fixed-size Eigen types.");
  static_assert(Idx_ < static_cast<size_t>(Rows_ * Cols_), "Index out of range.");
  EIGEN_STRUCTURED_BINDINGS_ASSERT_COL_MAJOR(Rows_, Cols_, Options_);
  return std::move(a.coeffRef(static_cast<Index>(Idx_)));
}

}  // namespace Eigen

#endif  // C++17

#endif  // EIGEN_STRUCTURED_BINDINGS_H
