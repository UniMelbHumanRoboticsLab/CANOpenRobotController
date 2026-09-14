// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2009 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2006-2008 Benoit Jacob <jacob.benoit.1@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// This file is included into the body of the base classes supporting matrix specific coefficient-wise functions.
// These are MatrixBase, ArrayBase, and SparseMatrixBase.

/// \returns an expression of the coefficient-wise absolute value of \c *this
///
/// Example: \include MatrixBase_cwiseAbs.cpp
/// Output: \verbinclude MatrixBase_cwiseAbs.out
///
EIGEN_DOC_UNARY_ADDONS(cwiseAbs, absolute value)
///
/// \sa cwiseAbs2()
///
EIGEN_MAKE_CWISE_UNARY_OP(cwiseAbs, internal::scalar_abs_op, CwiseAbsReturnType)

/// \returns an expression of the coefficient-wise squared absolute value of \c *this
///
/// Example: \include MatrixBase_cwiseAbs2.cpp
/// Output: \verbinclude MatrixBase_cwiseAbs2.out
///
EIGEN_DOC_UNARY_ADDONS(cwiseAbs2, squared absolute value)
///
/// \sa cwiseAbs()
///
EIGEN_MAKE_CWISE_UNARY_OP(cwiseAbs2, internal::scalar_abs2_op, CwiseAbs2ReturnType)

/// \returns an expression of the coefficient-wise square root of *this.
///
/// Example: \include MatrixBase_cwiseSqrt.cpp
/// Output: \verbinclude MatrixBase_cwiseSqrt.out
///
EIGEN_DOC_UNARY_ADDONS(cwiseSqrt, square - root)
///
/// \sa cwisePow(), cwiseSquare(), cwiseCbrt()
///
EIGEN_MAKE_CWISE_UNARY_OP(cwiseSqrt, internal::scalar_sqrt_op, CwiseSqrtReturnType)

/// \returns an expression of the coefficient-wise cube root of *this.
///
EIGEN_DOC_UNARY_ADDONS(cwiseCbrt, cube - root)
///
/// \sa cwiseSqrt(), cwiseSquare(), cwisePow()
///
EIGEN_MAKE_CWISE_UNARY_OP(cwiseCbrt, internal::scalar_cbrt_op, CwiseCbrtReturnType)

/// \returns an expression of the coefficient-wise square of *this.
///
EIGEN_DOC_UNARY_ADDONS(cwiseSquare, square)
///
/// \sa cwisePow(), cwiseSqrt(), cwiseCbrt()
///
EIGEN_MAKE_CWISE_UNARY_OP(cwiseSquare, internal::scalar_square_op, CwiseSquareReturnType)

/// \returns an expression of the coefficient-wise signum of *this.
///
/// Example: \include MatrixBase_cwiseSign.cpp
/// Output: \verbinclude MatrixBase_cwiseSign.out
///
EIGEN_DOC_UNARY_ADDONS(cwiseSign, sign function)
///
EIGEN_MAKE_CWISE_UNARY_OP(cwiseSign, internal::scalar_sign_op, CwiseSignReturnType)

/// \returns an expression of the coefficient-wise inverse of *this.
///
/// Example: \include MatrixBase_cwiseInverse.cpp
/// Output: \verbinclude MatrixBase_cwiseInverse.out
///
EIGEN_DOC_UNARY_ADDONS(cwiseInverse, inverse)
///
/// \sa cwiseProduct()
///
EIGEN_MAKE_CWISE_UNARY_OP(cwiseInverse, internal::scalar_inverse_op, CwiseInverseReturnType)

/// \returns an expression of the coefficient-wise phase angle of \c *this
///
/// Example: \include MatrixBase_cwiseArg.cpp
/// Output: \verbinclude MatrixBase_cwiseArg.out
///
EIGEN_DOC_UNARY_ADDONS(cwiseArg, arg)

EIGEN_MAKE_CWISE_UNARY_OP(cwiseArg, internal::scalar_arg_op, CwiseArgReturnType)

EIGEN_MAKE_CWISE_UNARY_OP(cwiseCArg, internal::scalar_carg_op, CwiseCArgReturnType)

template <typename ScalarExponent>
using CwisePowReturnType =
    std::enable_if_t<internal::is_arithmetic<typename NumTraits<ScalarExponent>::Real>::value,
                     CwiseUnaryOp<internal::scalar_unary_pow_op<Scalar, ScalarExponent>, const Derived>>;

template <typename ScalarExponent>
EIGEN_DEVICE_FUNC constexpr inline const CwisePowReturnType<ScalarExponent> cwisePow(
    const ScalarExponent& exponent) const {
  return CwisePowReturnType<ScalarExponent>(derived(), internal::scalar_unary_pow_op<Scalar, ScalarExponent>(exponent));
}
