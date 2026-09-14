// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_TEST_RANDOM_FOR_ARITHMETIC_H
#define EIGEN_TEST_RANDOM_FOR_ARITHMETIC_H

#include <Eigen/Core>

namespace Eigen {

// Small integer operands leave room for the sums and products in arithmetic tests.
// Floating-point tests retain the usual Random() distribution.
template <typename Scalar>
Scalar random_scalar_for_arithmetic() {
  EIGEN_IF_CONSTEXPR (NumTraits<Scalar>::IsInteger) {
    return internal::random<Scalar>(Scalar(NumTraits<Scalar>::IsSigned ? -7 : 0), Scalar(7));
  }
  return internal::random<Scalar>();
}

template <typename MatrixType, typename... Sizes>
MatrixType random_for_arithmetic(Sizes... sizes) {
  return MatrixType::NullaryExpr(sizes...,
                                 []() { return random_scalar_for_arithmetic<typename MatrixType::Scalar>(); });
}

}  // namespace Eigen

#endif  // EIGEN_TEST_RANDOM_FOR_ARITHMETIC_H
