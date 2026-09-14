// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_GPU_TYPE_TRAITS_H
#define EIGEN_GPU_TYPE_TRAITS_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

#include "./FwdDecl.h"
#include "./Meta.h"

#include <complex>
#include <type_traits>

namespace Eigen {
namespace gpu {

/**
 * @name internal:: type traits
 *
 * Each one keys off the *name* of a forward-declared class. None of them
 * inspect members so they parse cleanly with FwdDecl.h alone. is_scaled_leaf
 * and is_scaled_gemm compose with an inner predicate, but still by matching the
 * template-id rather than by reading a member.
 */

namespace internal {

///@{

template <typename Expr>
using scalar_type_t = typename device_expr_traits<Expr>::scalar_type;

template <typename T>
struct is_device_buffer : Eigen::internal::bool_constant<false> {};
template <>
struct is_device_buffer<DeviceBuffer> : Eigen::internal::bool_constant<true> {};

template <typename T>
struct is_device_matrix : Eigen::internal::bool_constant<false> {};
template <typename Scalar>
struct is_device_matrix<DeviceMatrix<Scalar>> : Eigen::internal::bool_constant<true> {};

template <typename T>
struct is_gemm_expr : Eigen::internal::bool_constant<false> {};
template <typename Lhs, typename Rhs>
struct is_gemm_expr<GemmExpr<Lhs, Rhs>> : Eigen::internal::bool_constant<true> {};

template <typename T>
struct is_adjoint_view : Eigen::internal::bool_constant<false> {};
template <typename Scalar>
struct is_adjoint_view<AdjointView<Scalar>> : Eigen::internal::bool_constant<true> {};

template <typename T>
struct is_transpose_view : Eigen::internal::bool_constant<false> {};
template <typename Scalar>
struct is_transpose_view<TransposeView<Scalar>> : Eigen::internal::bool_constant<true> {};

template <typename T>
struct is_scaled : Eigen::internal::bool_constant<false> {};
template <typename Inner>
struct is_scaled<Scaled<Inner>> : Eigen::internal::bool_constant<true> {};

/**
 * @brief Detects a Scaled directly over a leaf DeviceMatrix (no view in between).
 * @tparam T The type to test.
 * @note Defers to is_device_matrix on the scaled operand rather than restating
 * the nested pattern, so it tracks whatever is_device_matrix accepts.
 * @note The operand is deduced from the template-id, not read from Scaled::Inner:
 * naming a member would instantiate Scaled, and Scaled<GemmExpr<...>>::Scalar is
 * ill-formed because GemmExpr has no device_expr_traits specialization. Matching
 * the template-id keeps this predicate usable on any Scaled, complete or not.
 * @note Such a node arriving as a donation (owned rvalue) is materialized by
 * stealing the leaf and applying the scalar in place (scal) instead of a geam
 * into a fresh buffer.
 */
template <typename T>
struct is_scaled_leaf : Eigen::internal::bool_constant<false> {};
template <typename Inner>
struct is_scaled_leaf<Scaled<Inner>> : is_device_matrix<std::decay_t<Inner>> {};

/**
 * @brief Detects a Scaled directly over a GemmExpr (a product carrying ONE deferred scalar).
 * @tparam T The type to test.
 * @note Defers to is_gemm_expr on the scaled operand, deduced the same way as in
 * is_scaled_leaf and for the same reason.
 * @note Such a summand routes through the GEMM epilogue with its factor as the
 * gemm's alpha_scale (no temporary).
 */
template <typename T>
struct is_scaled_gemm : Eigen::internal::bool_constant<false> {};
template <typename Inner>
struct is_scaled_gemm<Scaled<Inner>> : is_gemm_expr<std::decay_t<Inner>> {};

template <typename T>
struct is_triangular_view : Eigen::internal::bool_constant<false> {};
template <typename Inner, int UpLo>
struct is_triangular_view<TriangularView<Inner, UpLo>> : Eigen::internal::bool_constant<true> {};

template <typename T>
struct is_trsm_expr : Eigen::internal::bool_constant<false> {};
template <typename Scalar, int UpLo>
struct is_trsm_expr<TrsmExpr<Scalar, UpLo>> : Eigen::internal::bool_constant<true> {};

template <typename T>
struct is_selfadjoint_view : Eigen::internal::bool_constant<false> {};
template <typename Scalar, int UpLo>
struct is_selfadjoint_view<SelfAdjointView<Scalar, UpLo>> : Eigen::internal::bool_constant<true> {};

template <typename T>
struct is_const_selfadjoint_view : Eigen::internal::bool_constant<false> {};
template <typename Inner, int UpLo>
struct is_const_selfadjoint_view<ConstSelfAdjointView<Inner, UpLo>> : Eigen::internal::bool_constant<true> {};

template <typename T>
struct is_symm_expr : Eigen::internal::bool_constant<false> {};
template <typename Scalar, int UpLo>
struct is_symm_expr<SymmExpr<Scalar, UpLo>> : Eigen::internal::bool_constant<true> {};

template <typename T>
struct is_syrk_expr : Eigen::internal::bool_constant<false> {};
template <typename A, int UpLo>
struct is_syrk_expr<SyrkExpr<A, UpLo>> : Eigen::internal::bool_constant<true> {};

template <typename T>
struct is_llt_solve_expr : Eigen::internal::bool_constant<false> {};
template <typename Scalar, int UpLo>
struct is_llt_solve_expr<LltSolveExpr<Scalar, UpLo>> : Eigen::internal::bool_constant<true> {};

template <typename T>
struct is_lu_solve_expr : Eigen::internal::bool_constant<false> {};
template <typename Scalar>
struct is_lu_solve_expr<LuSolveExpr<Scalar>> : Eigen::internal::bool_constant<true> {};

template <typename T>
struct is_llt_view : Eigen::internal::bool_constant<false> {};
template <typename Scalar, int UpLo>
struct is_llt_view<LLTView<Scalar, UpLo>> : Eigen::internal::bool_constant<true> {};

template <typename T>
struct is_lu_view : Eigen::internal::bool_constant<false> {};
template <typename Scalar>
struct is_lu_view<LUView<Scalar>> : Eigen::internal::bool_constant<true> {};

template <typename T>
struct is_device_add_expr : Eigen::internal::bool_constant<false> {};
template <typename Scalar>
struct is_device_add_expr<DeviceAddExpr<Scalar>> : Eigen::internal::bool_constant<true> {};

template <typename T>
struct is_device_scaled_device : Eigen::internal::bool_constant<false> {};
template <typename Inner>
struct is_device_scaled_device<DeviceScaledDevice<Inner>> : Eigen::internal::bool_constant<true> {};

template <typename T>
struct is_device_scalar : Eigen::internal::bool_constant<false> {};
template <typename S>
struct is_device_scalar<DeviceScalar<S>> : Eigen::internal::bool_constant<true> {};

///@}

}  // namespace internal

/**
 * @defgroup gpu_type_traits GPU type traits
 * @name Public is_* / is_*_v / require_* / require_all_* wrappers
 *
 * Type traits for the Eigen GPU library
 */
///@{

/**
 * Detect if a type is a @ref Eigen::gpu::internal::DeviceBuffer
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
struct is_device_buffer : internal::is_device_buffer<std::decay_t<T>> {};

/**
 * True iff a type is a @ref Eigen::gpu::internal::DeviceBuffer
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
constexpr bool is_device_buffer_v = is_device_buffer<T>::value;

/**
 * Require a type is a @ref Eigen::gpu::internal::DeviceBuffer
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
using require_device_buffer = internal::require_t<is_device_buffer<T>>;

/**
 * Detect if a type is a @ref Eigen::gpu::DeviceMatrix
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
struct is_device_matrix : internal::is_device_matrix<std::decay_t<T>> {};

/**
 * True iff a type is a @ref Eigen::gpu::DeviceMatrix
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
constexpr bool is_device_matrix_v = is_device_matrix<T>::value;

/**
 * Require a type is a @ref Eigen::gpu::DeviceMatrix
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
using require_device_matrix = internal::require_t<is_device_matrix<T>>;

/**
 * Require a type is not a @ref Eigen::gpu::DeviceMatrix
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
using require_not_device_matrix = internal::require_not_t<is_device_matrix<T>>;

/**
 * Require all types are @ref Eigen::gpu::DeviceMatrix types.
 * @tparam Types The types to test.
 * @ingroup gpu_type_traits
 */
template <typename... Types>
using require_all_device_matrix = internal::require_all_t<is_device_matrix_v<Types>...>;

/**
 * Detect if a type is a @ref Eigen::gpu::GemmExpr
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
struct is_gemm_expr : internal::is_gemm_expr<std::decay_t<T>> {};

/**
 * True iff a type is a @ref Eigen::gpu::GemmExpr
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
constexpr bool is_gemm_expr_v = is_gemm_expr<T>::value;

/**
 * Require a type is a @ref Eigen::gpu::GemmExpr
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
using require_gemm_expr = internal::require_t<is_gemm_expr<T>>;

/**
 * Require all types are @ref Eigen::gpu::GemmExpr types.
 * @tparam Types The types to test.
 * @ingroup gpu_type_traits
 */
template <typename... Types>
using require_all_gemm_expr = internal::require_all_t<is_gemm_expr_v<Types>...>;

/**
 * Detect if a type is an @ref Eigen::gpu::AdjointView
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
struct is_adjoint_view : internal::is_adjoint_view<std::decay_t<T>> {};

/**
 * True iff a type is an @ref Eigen::gpu::AdjointView
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
constexpr bool is_adjoint_view_v = is_adjoint_view<T>::value;

/**
 * Require a type is an @ref Eigen::gpu::AdjointView
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
using require_adjoint_view = internal::require_t<is_adjoint_view<T>>;

/**
 * Require all types are @ref Eigen::gpu::AdjointView types.
 * @tparam Types The types to test.
 * @ingroup gpu_type_traits
 */
template <typename... Types>
using require_all_adjoint_view = internal::require_all_t<is_adjoint_view_v<Types>...>;

/**
 * Detect if a type is a @ref Eigen::gpu::TransposeView
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
struct is_transpose_view : internal::is_transpose_view<std::decay_t<T>> {};

/**
 * True iff a type is a @ref Eigen::gpu::TransposeView
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
constexpr bool is_transpose_view_v = is_transpose_view<T>::value;

/**
 * Require a type is a @ref Eigen::gpu::TransposeView
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
using require_transpose_view = internal::require_t<is_transpose_view<T>>;

/**
 * Require all types are @ref Eigen::gpu::TransposeView types.
 * @tparam Types The types to test.
 * @ingroup gpu_type_traits
 */
template <typename... Types>
using require_all_transpose_view = internal::require_all_t<is_transpose_view_v<Types>...>;

/**
 * Detect if a type is a @ref Eigen::gpu::Scaled expression.
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
struct is_scaled : internal::is_scaled<std::decay_t<T>> {};

/**
 * True iff a type is a @ref Eigen::gpu::Scaled expression.
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
constexpr bool is_scaled_v = is_scaled<T>::value;

/**
 * Require a type is a @ref Eigen::gpu::Scaled expression.
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
using require_scaled = internal::require_t<is_scaled<T>>;

/**
 * Detect if a type is a @ref Eigen::gpu::Scaled expression directly over a @ref Eigen::gpu::DeviceMatrix
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
struct is_scaled_leaf : internal::is_scaled_leaf<std::decay_t<T>> {};

/**
 * True iff a type is a @ref Eigen::gpu::Scaled expression directly over a @ref Eigen::gpu::DeviceMatrix
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
constexpr bool is_scaled_leaf_v = is_scaled_leaf<T>::value;

/**
 * Require a type is a @ref Eigen::gpu::Scaled expression directly over a @ref Eigen::gpu::DeviceMatrix
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
using require_scaled_leaf = internal::require_t<is_scaled_leaf<T>>;

/**
 * Detect if a type is a @ref Eigen::gpu::Scaled expression directly over a @ref Eigen::gpu::GemmExpr
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
struct is_scaled_gemm : internal::is_scaled_gemm<std::decay_t<T>> {};

/**
 * True iff a type is a @ref Eigen::gpu::Scaled expression directly over a @ref Eigen::gpu::GemmExpr
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
constexpr bool is_scaled_gemm_v = is_scaled_gemm<T>::value;

/**
 * Require a type is a @ref Eigen::gpu::Scaled expression directly over a @ref Eigen::gpu::GemmExpr
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
using require_scaled_gemm = internal::require_t<is_scaled_gemm<T>>;

/**
 * True iff a type is a @ref Eigen::gpu::GemmExpr or a @ref Eigen::gpu::Scaled expression directly over one.
 * @tparam T The type to test.
 * @note Such a product may carry one deferred scalar, which becomes the GEMM alpha scale.
 * @ingroup gpu_type_traits
 */
template <typename T>
constexpr bool is_gemm_like_v = is_gemm_expr_v<T> || is_scaled_gemm_v<T>;

/**
 * Require a type is an @ref Eigen::gpu::AdjointView or a @ref Eigen::gpu::TransposeView
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
using require_adjoint_or_transpose_view =
    internal::require_t<Eigen::internal::bool_constant<is_adjoint_view_v<T> || is_transpose_view_v<T>>>;

/**
 * Detect if a type is a @ref Eigen::gpu::TriangularView
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
struct is_triangular_view : internal::is_triangular_view<std::decay_t<T>> {};

/**
 * True iff a type is a @ref Eigen::gpu::TriangularView
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
constexpr bool is_triangular_view_v = is_triangular_view<T>::value;

/**
 * Require a type is a @ref Eigen::gpu::TriangularView
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
using require_triangular_view = internal::require_t<is_triangular_view<T>>;

/**
 * Require all types are @ref Eigen::gpu::TriangularView types.
 * @tparam Types The types to test.
 * @ingroup gpu_type_traits
 */
template <typename... Types>
using require_all_triangular_view = internal::require_all_t<is_triangular_view_v<Types>...>;

/**
 * Detect if a type is a @ref Eigen::gpu::TrsmExpr
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
struct is_trsm_expr : internal::is_trsm_expr<std::decay_t<T>> {};

/**
 * True iff a type is a @ref Eigen::gpu::TrsmExpr
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
constexpr bool is_trsm_expr_v = is_trsm_expr<T>::value;

/**
 * Require a type is a @ref Eigen::gpu::TrsmExpr
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
using require_trsm_expr = internal::require_t<is_trsm_expr<T>>;

/**
 * Require all types are @ref Eigen::gpu::TrsmExpr types.
 * @tparam Types The types to test.
 * @ingroup gpu_type_traits
 */
template <typename... Types>
using require_all_trsm_expr = internal::require_all_t<is_trsm_expr_v<Types>...>;

/**
 * Detect if a type is a @ref Eigen::gpu::SelfAdjointView
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
struct is_selfadjoint_view : internal::is_selfadjoint_view<std::decay_t<T>> {};

/**
 * True iff a type is a @ref Eigen::gpu::SelfAdjointView
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
constexpr bool is_selfadjoint_view_v = is_selfadjoint_view<T>::value;

/**
 * Require a type is a @ref Eigen::gpu::SelfAdjointView
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
using require_selfadjoint_view = internal::require_t<is_selfadjoint_view<T>>;

/**
 * Require all types are @ref Eigen::gpu::SelfAdjointView types.
 * @tparam Types The types to test.
 * @ingroup gpu_type_traits
 */
template <typename... Types>
using require_all_selfadjoint_view = internal::require_all_t<is_selfadjoint_view_v<Types>...>;

/**
 * Detect if a type is a @ref Eigen::gpu::ConstSelfAdjointView
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
struct is_const_selfadjoint_view : internal::is_const_selfadjoint_view<std::decay_t<T>> {};

/**
 * True iff a type is a @ref Eigen::gpu::ConstSelfAdjointView
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
constexpr bool is_const_selfadjoint_view_v = is_const_selfadjoint_view<T>::value;

/**
 * Require a type is a @ref Eigen::gpu::ConstSelfAdjointView
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
using require_const_selfadjoint_view = internal::require_t<is_const_selfadjoint_view<T>>;

/**
 * Require all types are @ref Eigen::gpu::ConstSelfAdjointView types.
 * @tparam Types The types to test.
 * @ingroup gpu_type_traits
 */
template <typename... Types>
using require_all_const_selfadjoint_view = internal::require_all_t<is_const_selfadjoint_view_v<Types>...>;

/**
 * Detect if a type is a @ref Eigen::gpu::SymmExpr
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
struct is_symm_expr : internal::is_symm_expr<std::decay_t<T>> {};

/**
 * True iff a type is a @ref Eigen::gpu::SymmExpr
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
constexpr bool is_symm_expr_v = is_symm_expr<T>::value;

/**
 * Require a type is a @ref Eigen::gpu::SymmExpr
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
using require_symm_expr = internal::require_t<is_symm_expr<T>>;

/**
 * Require all types are @ref Eigen::gpu::SymmExpr types.
 * @tparam Types The types to test.
 * @ingroup gpu_type_traits
 */
template <typename... Types>
using require_all_symm_expr = internal::require_all_t<is_symm_expr_v<Types>...>;

/**
 * Detect if a type is a @ref Eigen::gpu::SyrkExpr
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
struct is_syrk_expr : internal::is_syrk_expr<std::decay_t<T>> {};

/**
 * True iff a type is a @ref Eigen::gpu::SyrkExpr
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
constexpr bool is_syrk_expr_v = is_syrk_expr<T>::value;

/**
 * Require a type is a @ref Eigen::gpu::SyrkExpr
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
using require_syrk_expr = internal::require_t<is_syrk_expr<T>>;

/**
 * Require all types are @ref Eigen::gpu::SyrkExpr types.
 * @tparam Types The types to test.
 * @ingroup gpu_type_traits
 */
template <typename... Types>
using require_all_syrk_expr = internal::require_all_t<is_syrk_expr_v<Types>...>;

/**
 * Detect if a type is a @ref Eigen::gpu::LltSolveExpr
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
struct is_llt_solve_expr : internal::is_llt_solve_expr<std::decay_t<T>> {};

/**
 * True iff a type is a @ref Eigen::gpu::LltSolveExpr
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
constexpr bool is_llt_solve_expr_v = is_llt_solve_expr<T>::value;

/**
 * Require a type is a @ref Eigen::gpu::LltSolveExpr
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
using require_llt_solve_expr = internal::require_t<is_llt_solve_expr<T>>;

/**
 * Require all types are @ref Eigen::gpu::LltSolveExpr types.
 * @tparam Types The types to test.
 * @ingroup gpu_type_traits
 */
template <typename... Types>
using require_all_llt_solve_expr = internal::require_all_t<is_llt_solve_expr_v<Types>...>;

/**
 * Detect if a type is a @ref Eigen::gpu::LuSolveExpr
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
struct is_lu_solve_expr : internal::is_lu_solve_expr<std::decay_t<T>> {};

/**
 * True iff a type is a @ref Eigen::gpu::LuSolveExpr
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
constexpr bool is_lu_solve_expr_v = is_lu_solve_expr<T>::value;

/**
 * Require a type is a @ref Eigen::gpu::LuSolveExpr
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
using require_lu_solve_expr = internal::require_t<is_lu_solve_expr<T>>;

/**
 * Require all types are @ref Eigen::gpu::LuSolveExpr types.
 * @tparam Types The types to test.
 * @ingroup gpu_type_traits
 */
template <typename... Types>
using require_all_lu_solve_expr = internal::require_all_t<is_lu_solve_expr_v<Types>...>;

/**
 * Detect if a type is an @ref Eigen::gpu::LLTView
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
struct is_llt_view : internal::is_llt_view<std::decay_t<T>> {};

/**
 * True iff a type is an @ref Eigen::gpu::LLTView
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
constexpr bool is_llt_view_v = is_llt_view<T>::value;

/**
 * Require a type is an @ref Eigen::gpu::LLTView
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
using require_llt_view = internal::require_t<is_llt_view<T>>;

/**
 * Require all types are @ref Eigen::gpu::LLTView types.
 * @tparam Types The types to test.
 * @ingroup gpu_type_traits
 */
template <typename... Types>
using require_all_llt_view = internal::require_all_t<is_llt_view_v<Types>...>;

/**
 * Detect if a type is an @ref Eigen::gpu::LUView
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
struct is_lu_view : internal::is_lu_view<std::decay_t<T>> {};

/**
 * True iff a type is an @ref Eigen::gpu::LUView
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
constexpr bool is_lu_view_v = is_lu_view<T>::value;

/**
 * Require a type is an @ref Eigen::gpu::LUView
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
using require_lu_view = internal::require_t<is_lu_view<T>>;

/**
 * Require all types are @ref Eigen::gpu::LUView types.
 * @tparam Types The types to test.
 * @ingroup gpu_type_traits
 */
template <typename... Types>
using require_all_lu_view = internal::require_all_t<is_lu_view_v<Types>...>;

/**
 * True iff a type is an @ref Eigen::gpu::LLTView or an @ref Eigen::gpu::LUView
 * @tparam T The type to test.
 * @note Arithmetic operators exclude factorization handles so they cannot be lowered as matrix operands.
 * @ingroup gpu_type_traits
 */
template <typename T>
constexpr bool is_factor_expr_v = is_lu_view_v<T> || is_llt_view_v<T>;

/**
 * Detect if a type is a @ref Eigen::gpu::DeviceAddExpr
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
struct is_device_add_expr : internal::is_device_add_expr<std::decay_t<T>> {};

/**
 * True iff a type is a @ref Eigen::gpu::DeviceAddExpr
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
constexpr bool is_device_add_expr_v = is_device_add_expr<T>::value;

/**
 * Require a type is a @ref Eigen::gpu::DeviceAddExpr
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
using require_device_add_expr = internal::require_t<is_device_add_expr<T>>;

/**
 * Require all types are @ref Eigen::gpu::DeviceAddExpr types.
 * @tparam Types The types to test.
 * @ingroup gpu_type_traits
 */
template <typename... Types>
using require_all_device_add_expr = internal::require_all_t<is_device_add_expr_v<Types>...>;

/**
 * Detect if a type is a @ref Eigen::gpu::DeviceScaledDevice
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
struct is_device_scaled_device : internal::is_device_scaled_device<std::decay_t<T>> {};

/**
 * True iff a type is a @ref Eigen::gpu::DeviceScaledDevice
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
constexpr bool is_device_scaled_device_v = is_device_scaled_device<T>::value;

/**
 * Require a type is a @ref Eigen::gpu::DeviceScaledDevice
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
using require_device_scaled_device = internal::require_t<is_device_scaled_device<T>>;

/**
 * Require all types are @ref Eigen::gpu::DeviceScaledDevice types.
 * @tparam Types The types to test.
 * @ingroup gpu_type_traits
 */
template <typename... Types>
using require_all_device_scaled_device = internal::require_all_t<is_device_scaled_device_v<Types>...>;

/**
 * Detect if a type is a @ref Eigen::gpu::DeviceScalar
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
struct is_device_scalar : internal::is_device_scalar<std::decay_t<T>> {};

/**
 * True iff a type is a @ref Eigen::gpu::DeviceScalar
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
constexpr bool is_device_scalar_v = is_device_scalar<T>::value;

/**
 * Require a type is a @ref Eigen::gpu::DeviceScalar
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
using require_device_scalar = internal::require_t<is_device_scalar<T>>;

/**
 * Require all types are @ref Eigen::gpu::DeviceScalar types.
 * @tparam Types The types to test.
 * @ingroup gpu_type_traits
 */
template <typename... Types>
using require_all_device_scalar = internal::require_all_t<is_device_scalar_v<Types>...>;

/**
 * @name device_expr_traits query aliases
 *
 * Lazy readers of the member-inspecting internal::device_expr_traits, whose
 * specializations live in DeviceExpr.h. Declared from the FwdDecl.h forward
 * declaration; they resolve only when instantiated on a complete specialization.
 */
///@{

template <typename T>
using scalar_type_t = internal::scalar_type_t<std::decay_t<T>>;

/**
 * Detect if a type is marked as a device expression by
 * @ref Eigen::gpu::internal::device_expr_traits
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
struct is_device_expr : Eigen::internal::bool_constant<internal::device_expr_traits<std::decay_t<T>>::is_device_expr> {
};

/**
 * True iff a type is marked as a device expression by
 * @ref Eigen::gpu::internal::device_expr_traits
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
constexpr bool is_device_expr_v = is_device_expr<T>::value;

/**
 * Require a type is marked as a device expression by
 * @ref Eigen::gpu::internal::device_expr_traits
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
using require_device_expr = internal::require_t<is_device_expr<T>>;

/**
 * Require all types are marked as device expressions by
 * @ref Eigen::gpu::internal::device_expr_traits
 * @tparam Types The types to test.
 * @ingroup gpu_type_traits
 */
template <typename... Types>
using require_all_device_expr = internal::require_all_t<is_device_expr_v<Types>...>;

template <typename T>
constexpr GpuOp trans_op = internal::device_expr_traits<std::decay_t<T>>::op;

///@}

namespace internal {
template <typename T>
struct is_complex : Eigen::internal::bool_constant<false> {};

template <typename T>
struct is_complex<std::complex<T>> : Eigen::internal::bool_constant<true> {};
}  // namespace internal

/**
 * Detect if a type is a `std::complex` specialization.
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
struct is_complex : internal::is_complex<std::decay_t<T>> {};

/**
 * True iff a type is a `std::complex` specialization.
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
constexpr bool is_complex_v = is_complex<T>::value;

/**
 * Detect if a type is an integral, floating-point, or `std::complex` host scalar.
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
struct is_host_scalar : Eigen::internal::bool_constant<std::is_floating_point<std::decay_t<T>>::value ||
                                                       std::is_integral<std::decay_t<T>>::value || is_complex_v<T>> {};

/**
 * Require a type is an integral, floating-point, or `std::complex` host scalar.
 * @tparam T The type to test.
 * @ingroup gpu_type_traits
 */
template <typename T>
using require_host_scalar = internal::require_t<is_host_scalar<T>>;

///@}

}  // namespace gpu
}  // namespace Eigen

#endif  // EIGEN_GPU_TYPE_TRAITS_H
