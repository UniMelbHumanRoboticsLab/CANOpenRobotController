// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

// Compile-time tests for the contrib GPU type-trait surface: name-keyed
// predicates plus the device_expr_traits query aliases (scalar_type_t, is_device_expr_v, trans_op).

#define EIGEN_USE_GPU
#include "main.h"
#include <type_traits>
#include <contrib/Eigen/GPU>

using namespace Eigen;

namespace {
using TraitDeviceMatrix = gpu::DeviceMatrix<double>;
using TraitAdjointView = gpu::AdjointView<double>;
using TraitTransposeView = gpu::TransposeView<double>;
using TraitScaledMatrix = gpu::Scaled<TraitDeviceMatrix>;
using TraitGemmExpr = gpu::GemmExpr<TraitDeviceMatrix, TraitDeviceMatrix>;
using TraitTriangularView = gpu::TriangularView<double, Lower>;
using TraitTrsmExpr = gpu::TrsmExpr<double, Lower>;
using TraitSelfAdjointView = gpu::SelfAdjointView<double, Lower>;
using TraitConstSelfAdjointView = gpu::ConstSelfAdjointView<double, Lower>;
using TraitSymmExpr = gpu::SymmExpr<double, Lower>;
using TraitSyrkExpr = gpu::SyrkExpr<double, Lower>;
using TraitLltSolveExpr = gpu::LltSolveExpr<double, Lower>;
using TraitLuSolveExpr = gpu::LuSolveExpr<double>;
using TraitLltView = gpu::LLTView<double, Lower>;
using TraitLuView = gpu::LUView<double>;
using TraitDeviceAddExpr = gpu::DeviceAddExpr<double>;
using TraitDeviceScaledDevice = gpu::DeviceScaledDevice<double>;
using TraitDeviceScalar = gpu::DeviceScalar<double>;
using TraitDeviceBuffer = gpu::internal::DeviceBuffer;

#define EIGEN_GPU_STATIC_ASSERT_TRAIT(Trait, Require, RequireAll, Type, NegativeType)       \
  static_assert(gpu::Trait<Type>::value, #Trait " should accept the exact type");           \
  static_assert(gpu::Trait<const Type&>::value, #Trait " should decay const references");   \
  static_assert(gpu::Trait##_v<Type&&>, #Trait "_v should decay rvalue references");        \
  static_assert(!gpu::Trait<NegativeType>::value, #Trait " should reject unrelated types"); \
  static_assert(std::is_same<gpu::Require<Type>, int>::value, #Require " should compile");  \
  static_assert(std::is_same<gpu::RequireAll<Type, const Type&>, int>::value, #RequireAll " should compile")

EIGEN_GPU_STATIC_ASSERT_TRAIT(is_device_matrix, require_device_matrix, require_all_device_matrix, TraitDeviceMatrix,
                              TraitAdjointView);
EIGEN_GPU_STATIC_ASSERT_TRAIT(is_adjoint_view, require_adjoint_view, require_all_adjoint_view, TraitAdjointView,
                              TraitDeviceMatrix);
EIGEN_GPU_STATIC_ASSERT_TRAIT(is_transpose_view, require_transpose_view, require_all_transpose_view, TraitTransposeView,
                              TraitDeviceMatrix);
EIGEN_GPU_STATIC_ASSERT_TRAIT(is_gemm_expr, require_gemm_expr, require_all_gemm_expr, TraitGemmExpr, TraitDeviceMatrix);
EIGEN_GPU_STATIC_ASSERT_TRAIT(is_triangular_view, require_triangular_view, require_all_triangular_view,
                              TraitTriangularView, TraitDeviceMatrix);
EIGEN_GPU_STATIC_ASSERT_TRAIT(is_trsm_expr, require_trsm_expr, require_all_trsm_expr, TraitTrsmExpr, TraitDeviceMatrix);
EIGEN_GPU_STATIC_ASSERT_TRAIT(is_selfadjoint_view, require_selfadjoint_view, require_all_selfadjoint_view,
                              TraitSelfAdjointView, TraitDeviceMatrix);
EIGEN_GPU_STATIC_ASSERT_TRAIT(is_const_selfadjoint_view, require_const_selfadjoint_view,
                              require_all_const_selfadjoint_view, TraitConstSelfAdjointView, TraitDeviceMatrix);
EIGEN_GPU_STATIC_ASSERT_TRAIT(is_symm_expr, require_symm_expr, require_all_symm_expr, TraitSymmExpr, TraitDeviceMatrix);
EIGEN_GPU_STATIC_ASSERT_TRAIT(is_syrk_expr, require_syrk_expr, require_all_syrk_expr, TraitSyrkExpr, TraitDeviceMatrix);
EIGEN_GPU_STATIC_ASSERT_TRAIT(is_llt_solve_expr, require_llt_solve_expr, require_all_llt_solve_expr, TraitLltSolveExpr,
                              TraitDeviceMatrix);
EIGEN_GPU_STATIC_ASSERT_TRAIT(is_lu_solve_expr, require_lu_solve_expr, require_all_lu_solve_expr, TraitLuSolveExpr,
                              TraitDeviceMatrix);
EIGEN_GPU_STATIC_ASSERT_TRAIT(is_llt_view, require_llt_view, require_all_llt_view, TraitLltView, TraitDeviceMatrix);
EIGEN_GPU_STATIC_ASSERT_TRAIT(is_lu_view, require_lu_view, require_all_lu_view, TraitLuView, TraitDeviceMatrix);
EIGEN_GPU_STATIC_ASSERT_TRAIT(is_device_add_expr, require_device_add_expr, require_all_device_add_expr,
                              TraitDeviceAddExpr, TraitDeviceMatrix);
EIGEN_GPU_STATIC_ASSERT_TRAIT(is_device_scaled_device, require_device_scaled_device, require_all_device_scaled_device,
                              TraitDeviceScaledDevice, TraitDeviceMatrix);
EIGEN_GPU_STATIC_ASSERT_TRAIT(is_device_scalar, require_device_scalar, require_all_device_scalar, TraitDeviceScalar,
                              TraitDeviceMatrix);

#undef EIGEN_GPU_STATIC_ASSERT_TRAIT

// is_device_buffer has no require_all_ variant, so it is checked directly.
static_assert(gpu::is_device_buffer_v<TraitDeviceBuffer>, "internal::DeviceBuffer is a device buffer");
static_assert(gpu::is_device_buffer_v<const TraitDeviceBuffer&>, "is_device_buffer_v should decay cv/ref");
static_assert(!gpu::is_device_buffer_v<TraitDeviceMatrix>, "a DeviceMatrix is not a DeviceBuffer");
static_assert(std::is_same<gpu::require_device_buffer<TraitDeviceBuffer>, int>::value,
              "require_device_buffer should compile");

// is_scaled is name-keyed; is_scaled_leaf / is_scaled_gemm compose it with a
// predicate over Scaled::Inner. None of the three has a require_all_ variant.
static_assert(gpu::is_scaled_v<TraitScaledMatrix>, "Scaled is a scaled node");
static_assert(gpu::is_scaled_v<const TraitScaledMatrix&>, "is_scaled_v should decay cv/ref");
static_assert(!gpu::is_scaled_v<TraitDeviceMatrix>, "a DeviceMatrix is not a scaled node");
static_assert(std::is_same<gpu::require_scaled<TraitScaledMatrix>, int>::value, "require_scaled should compile");

// Scaled names its operand, which is what the two composed predicates read.
static_assert(std::is_same<TraitScaledMatrix::Inner, TraitDeviceMatrix>::value, "Scaled exposes its inner type");
static_assert(std::is_same<gpu::Scaled<const TraitDeviceMatrix&>::Inner, TraitDeviceMatrix>::value,
              "Scaled::Inner decays the operand");

static_assert(gpu::is_scaled_leaf_v<TraitScaledMatrix>, "Scaled<DeviceMatrix> is a scaled leaf");
static_assert(gpu::is_scaled_leaf_v<TraitScaledMatrix&&>, "is_scaled_leaf_v should decay refs");
static_assert(gpu::is_scaled_leaf_v<gpu::Scaled<const TraitDeviceMatrix&>>,
              "a scaled leaf is still a leaf through Scaled::Inner's decay");
static_assert(!gpu::is_scaled_leaf_v<gpu::Scaled<TraitAdjointView>>, "Scaled over a view is not a scaled leaf");
static_assert(!gpu::is_scaled_leaf_v<gpu::Scaled<TraitTransposeView>>, "Scaled over a view is not a scaled leaf");
static_assert(!gpu::is_scaled_leaf_v<gpu::Scaled<TraitGemmExpr>>, "Scaled over a composite is not a scaled leaf");
static_assert(!gpu::is_scaled_leaf_v<TraitDeviceMatrix>, "a bare leaf is not a scaled leaf");

static_assert(gpu::is_scaled_gemm_v<gpu::Scaled<TraitGemmExpr>>, "Scaled<GemmExpr> is a scaled product");
static_assert(!gpu::is_scaled_gemm_v<TraitGemmExpr>, "a bare GemmExpr is not a scaled product");
static_assert(!gpu::is_scaled_gemm_v<TraitScaledMatrix>, "Scaled<leaf> is not a scaled product");
static_assert(!gpu::is_scaled_gemm_v<gpu::Scaled<TraitAdjointView>>, "Scaled<view> is not a scaled product");

// The gate short-circuits: a non-Scaled operand must answer false, not fail to
// compile on the missing ::Inner member.
static_assert(!gpu::is_scaled_leaf_v<TraitAdjointView>, "a view has no ::Inner and is not a scaled leaf");
static_assert(!gpu::is_scaled_gemm_v<TraitDeviceScalar>, "a device scalar has no ::Inner and is not a scaled product");

static_assert(gpu::is_gemm_like_v<TraitGemmExpr>, "a GemmExpr is gemm-like");
static_assert(gpu::is_gemm_like_v<gpu::Scaled<TraitGemmExpr>>, "Scaled<GemmExpr> is gemm-like");
static_assert(!gpu::is_gemm_like_v<TraitDeviceMatrix>, "a leaf is not gemm-like");
static_assert(!gpu::is_gemm_like_v<TraitDeviceAddExpr>, "a sum is not gemm-like");

// A factorization handle of either kind satisfies the combined operator gate.
static_assert(gpu::is_factor_expr_v<TraitLuView>, "LUView is a factorization handle");
static_assert(gpu::is_factor_expr_v<TraitLltView>, "LLTView is a factorization handle");
static_assert(!gpu::is_factor_expr_v<TraitDeviceMatrix>, "a leaf is not a factorization handle");

// Query aliases over device_expr_traits (specializations from DeviceExpr.h).
static_assert(gpu::is_device_expr_v<TraitDeviceMatrix>, "a leaf is a device-expression operand");
static_assert(gpu::is_device_expr_v<const TraitAdjointView&>, "an adjoint view is a device-expression operand");
static_assert(gpu::is_device_expr_v<TraitTransposeView&&>, "a transpose view is a device-expression operand");
static_assert(gpu::is_device_expr_v<TraitScaledMatrix>, "a scaled leaf is a device-expression operand");
static_assert(!gpu::is_device_expr_v<TraitGemmExpr>, "a GemmExpr is dispatched, not an operand");
static_assert(std::is_same<gpu::scalar_type_t<const TraitScaledMatrix&>, double>::value,
              "scalar_type_t decays cv/ref and folds the operand scalar");
static_assert(std::is_same<gpu::scalar_type_t<TraitAdjointView>, double>::value, "scalar_type_t reads the view scalar");
static_assert(gpu::trans_op<TraitDeviceMatrix> == gpu::GpuOp::NoTrans, "a leaf is NoTrans");
static_assert(gpu::trans_op<TraitAdjointView> == gpu::GpuOp::ConjTrans, "an adjoint view is ConjTrans");
static_assert(gpu::trans_op<TraitTransposeView> == gpu::GpuOp::Trans, "a transpose view is Trans");
static_assert(std::is_same<gpu::require_all_device_expr<TraitDeviceMatrix, const TraitAdjointView&, TraitScaledMatrix>,
                           int>::value,
              "require_all_device_expr should compile for device-expression operands");
}  // namespace

EIGEN_DECLARE_TEST(device_traits) {}
