// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_TEST_PRODUCT_TEST_HELPERS_H
#define EIGEN_TEST_PRODUCT_TEST_HELPERS_H

#include <iostream>

#include <Eigen/Core>

namespace Eigen {

// Rounding error bounds for matrix products, based on:
//
//   Deterministic: Higham, "Accuracy and Stability of Numerical Algorithms",
//     Thm 3.5: |fl(A*B) - A*B| <= gamma_k * |A| * |B|,  gamma_k ~ k * epsilon.
//
//   Probabilistic: Higham & Mary, "A New Approach to Probabilistic Rounding
//     Error Analysis", SISC 2019, Thm 3.4: under the assumption that rounding
//     errors are independent with mean zero:
//       |fl(A*B) - A*B| <= gamma_tilde_k * |A| * |B|,
//       gamma_tilde_k ~ lambda * sqrt(k) * epsilon,
//     holding with probability >= 1 - 2*exp(-lambda^2/2) per inner product.
//
// The bound is absolute rather than relative on purpose: || |A|*|B| ||_F /
// ||A*B||_F is unbounded, so no multiple of epsilon bounds the relative error
// of a product whose result cancels. Random operands do reach that regime — for
// A = m1*m1^T, B = m2, the result vanishes as m1^T*m2 does.
//
// Parameters:
//   num_products: number of independent products contributing error (default 1).
//                 Use 2 when comparing two different evaluations of A*B.
//   lambda:       probability parameter; P(lambda) = 1 - 2*exp(-lambda^2/2).
//                 lambda=5 gives P > 0.9999 per inner product.

// Returns lambda * sqrt(k) * epsilon * num_products * || |A|*|B| ||_F, with epsilon that of the
// operand scalar but the value kept in double: narrowed to half, a triple-product bound already
// overflows at k = 100 for all-one operands.
//
// || |A|*|B| ||_F is accumulated in double with an explicit loop rather than formed as
// (A.cwiseAbs() * B.cwiseAbs()).norm(): a bound built with the product implementation would inherit
// that implementation's defects, and one that overflowed to infinity would admit any result at all.
template <typename DerivedA, typename DerivedB>
double product_error_bound(const MatrixBase<DerivedA>& A, const MatrixBase<DerivedB>& B, int num_products = 1,
                           double lambda = 5) {
  using Scalar = typename DerivedA::Scalar;
  const Index k = A.cols();
  double squared_norm = 0.0;
  for (Index i = 0; i < A.rows(); ++i)
    for (Index j = 0; j < B.cols(); ++j) {
      double sum = 0.0;
      for (Index l = 0; l < k; ++l)
        sum += static_cast<double>(numext::abs(A.coeff(i, l))) * static_cast<double>(numext::abs(B.coeff(l, j)));
      squared_norm += sum * sum;
    }
  return lambda * numext::sqrt(double(k)) * static_cast<double>(NumTraits<Scalar>::epsilon()) * double(num_products) *
         numext::sqrt(squared_norm);
}

// Verify that two computations of A*B agree within the Higham-Mary bound.
// Returns true if ||actual - expected||_F <= product_error_bound(A, B, ...).
//
// The error norm is accumulated in double like the bound, so that it overflows only when the
// difference itself does. A non-finite error or bound fails: inf <= inf would otherwise accept
// an overflowed result.
template <typename D1, typename D2, typename DA, typename DB>
inline bool verifyProduct(const MatrixBase<D1>& actual, const MatrixBase<D2>& expected, const MatrixBase<DA>& A,
                          const MatrixBase<DB>& B, int num_products = 2, double lambda = 5) {
  const double bound = product_error_bound(A, B, num_products, lambda);
  const typename D1::PlainObject diff = actual - expected;
  double squared_error = 0.0;
  for (Index j = 0; j < diff.cols(); ++j)
    for (Index i = 0; i < diff.rows(); ++i) {
      const double d = static_cast<double>(numext::abs(diff.coeff(i, j)));
      squared_error += d * d;
    }
  const double error = numext::sqrt(squared_error);
  if (!((numext::isfinite)(bound) && error <= bound)) {
    std::cerr << "Product verification failed: error " << error << " exceeds bound " << bound << std::endl;
    return false;
  }
  return true;
}

}  // namespace Eigen

#endif  // EIGEN_TEST_PRODUCT_TEST_HELPERS_H
