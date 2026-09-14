// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_GPU_FWD_DECL_H
#define EIGEN_GPU_FWD_DECL_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {
namespace gpu {

class Context;

enum class GpuOp;

namespace internal {
class DeviceBuffer;

/** \brief Describes GPU device expression types.
 *
 * Specializations report the expression's scalar type, its transpose op, and
 * how to reach the underlying matrix and deferred scalar. They live in
 * DeviceExpr.h; this declaration is what the query aliases in type_traits.h
 * are written against.
 */
template <typename Expr>
struct device_expr_traits;
}  // namespace internal

template <typename Scalar_>
class DeviceMatrix;
template <typename Scalar_>
class DeviceScalar;

template <typename Scalar_, int UpLo_>
class LLT;
template <typename Scalar_>
class LU;
template <typename Scalar_>
class QR;

template <typename Scalar_>
class AdjointView;
template <typename Scalar_>
class TransposeView;
template <typename Scalar_>
class Assignment;
template <typename Lhs, typename Rhs>
class GemmExpr;
template <typename Inner>
class Scaled;
template <typename Scalar_>
class DeviceAddExpr;
template <typename Scalar_>
class DeviceScaledDevice;

template <typename Scalar_, int UpLo_>
class LltSolveExpr;
template <typename Scalar_>
class LuSolveExpr;
template <typename Scalar_, int UpLo_>
class LLTView;
template <typename Scalar_>
class LUView;

template <typename Scalar_, int UpLo_>
class TriangularView;
template <typename Scalar_, int UpLo_>
class SelfAdjointView;
template <typename Scalar_, int UpLo_>
class ConstSelfAdjointView;
template <typename Scalar_, int UpLo_>
class TrsmExpr;
template <typename Scalar_, int UpLo_>
class SymmExpr;
template <typename Scalar_, int UpLo_>
class SyrkExpr;

template <typename Scalar_>
class SparseContext;
template <typename Scalar_>
class DeviceSparseView;
template <typename Scalar_>
class SpMVExpr;

}  // namespace gpu
}  // namespace Eigen

#endif  // EIGEN_GPU_FWD_DECL_H
