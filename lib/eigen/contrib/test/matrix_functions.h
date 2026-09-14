// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009-2011 Jitse Niesen <jitse@maths.leeds.ac.uk>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_UNSUPPORTED_TEST_MATRIX_FUNCTIONS_H
#define EIGEN_UNSUPPORTED_TEST_MATRIX_FUNCTIONS_H

#include "main.h"
#include <contrib/Eigen/MatrixFunctions>

// For complex matrices, any matrix is fine.
template <typename MatrixType, int IsComplex = NumTraits<typename internal::traits<MatrixType>::Scalar>::IsComplex>
struct processTriangularMatrix {
  static void run(MatrixType&, MatrixType&, const MatrixType&) {}
};

// For real matrices, ensure all eigenvalues have positive real parts (needed for
// the matrix logarithm and fractional powers to stay off the branch cut on the
// negative real axis) and cap the condition number. The quasi-triangular Schur
// factor T is modified block by block: a 2x2 block must stay a genuine complex
// conjugate pair, so it is negated and scaled as a unit. Touching a single
// diagonal entry of a 2x2 block could flip its discriminant and split it into
// real -- possibly negative -- eigenvalues, which would reintroduce the branch
// cut this routine is meant to avoid.
template <typename MatrixType>
struct processTriangularMatrix<MatrixType, 0> {
  typedef typename MatrixType::Scalar Scalar;
  static void run(MatrixType& m, MatrixType& T, const MatrixType& U) {
    using std::abs;
    using std::sqrt;
    const Index size = m.cols();

    // First pass: give every eigenvalue a positive real part and record the
    // largest eigenvalue magnitude.
    Scalar maxMag(0);
    for (Index i = 0; i < size; ++i) {
      if (i == size - 1 || numext::is_exactly_zero(T.coeff(i + 1, i))) {
        // 1x1 block (real eigenvalue): make it positive.
        T.coeffRef(i, i) = abs(T.coeff(i, i));
        maxMag = (std::max)(maxMag, T.coeff(i, i));
      } else {
        // 2x2 block (complex conjugate pair a ± bi, real part
        // a = (T(i,i) + T(i+1,i+1)) / 2). Negate the whole block when its real
        // part is negative; this keeps it a valid conjugate pair while moving it
        // off the branch cut. Checking a single diagonal entry (as before) misses
        // blocks whose real part is negative only because the other entry is.
        if (T.coeff(i, i) + T.coeff(i + 1, i + 1) < Scalar(0)) {
          T.template block<2, 2>(i, i) *= Scalar(-1);
        }
        // |eigenvalue|^2 equals the block determinant a^2 + b^2.
        const Scalar det = T.template block<2, 2>(i, i).determinant();
        maxMag = (std::max)(maxMag, sqrt(abs(det)));
        ++i;
      }
    }

    // Second pass: lift eigenvalues that are too small to cap the condition
    // number; tests lose too many digits on ill-conditioned matrices. A 2x2 block
    // is scaled as a unit so it remains a conjugate pair.
    if (maxMag > Scalar(0)) {
      const Scalar minAllowed = maxMag / Scalar(100);
      for (Index i = 0; i < size; ++i) {
        if (i == size - 1 || numext::is_exactly_zero(T.coeff(i + 1, i))) {
          if (T.coeff(i, i) < minAllowed) T.coeffRef(i, i) = minAllowed;
        } else {
          const Scalar det = T.template block<2, 2>(i, i).determinant();
          const Scalar mag = sqrt(abs(det));
          if (mag > Scalar(0) && mag < minAllowed) {
            const Scalar s = minAllowed / mag;
            T.template block<2, 2>(i, i) *= s;
          }
          ++i;
        }
      }
    }
    m = U * T * U.transpose();
  }
};

template <typename MatrixType, int IsComplex = NumTraits<typename internal::traits<MatrixType>::Scalar>::IsComplex>
struct generateTestMatrix;

template <typename MatrixType>
struct generateTestMatrix<MatrixType, 0> {
  static void run(MatrixType& result, typename MatrixType::Index size) {
    result = MatrixType::Random(size, size);
    RealSchur<MatrixType> schur(result);
    MatrixType T = schur.matrixT();
    processTriangularMatrix<MatrixType>::run(result, T, schur.matrixU());
  }
};

template <typename MatrixType>
struct generateTestMatrix<MatrixType, 1> {
  static void run(MatrixType& result, typename MatrixType::Index size) { result = MatrixType::Random(size, size); }
};

template <typename Derived, typename OtherDerived>
typename Derived::RealScalar relerr(const MatrixBase<Derived>& A, const MatrixBase<OtherDerived>& B) {
  return std::sqrt((A - B).cwiseAbs2().sum() / (std::min)(A.cwiseAbs2().sum(), B.cwiseAbs2().sum()));
}

#endif  // EIGEN_UNSUPPORTED_TEST_MATRIX_FUNCTIONS_H
