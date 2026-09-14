// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2012 Alexey Korepanov <kaikaikai@yandex.ru>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#define EIGEN_RUNTIME_NO_MALLOC
#include "main.h"
#include <limits>
#include <Eigen/Eigenvalues>

template <typename MatrixType>
void real_qz(const MatrixType& m) {
  /* this test covers the following files:
     RealQZ.h
  */
  using std::abs;

  Index dim = m.cols();

  MatrixType A = MatrixType::Random(dim, dim), B = MatrixType::Random(dim, dim);

  // Regression test for bug 985: Randomly set rows or columns to zero
  Index k = internal::random<Index>(0, dim - 1);
  switch (internal::random<int>(0, 10)) {
    case 0:
      A.row(k).setZero();
      break;
    case 1:
      A.col(k).setZero();
      break;
    case 2:
      B.row(k).setZero();
      break;
    case 3:
      B.col(k).setZero();
      break;
    default:
      break;
  }

  RealQZ<MatrixType> qz(dim);
  // TODO enable full-prealocation of required memory, this probably requires an in-place mode for
  // HessenbergDecomposition
  // Eigen::internal::set_is_malloc_allowed(false);
  qz.compute(A, B);
  // Eigen::internal::set_is_malloc_allowed(true);

  VERIFY_IS_EQUAL(qz.info(), Success);
  // check for zeros
  bool all_zeros = true;
  for (Index i = 0; i < A.cols(); i++)
    for (Index j = 0; j < i; j++) {
      if (!numext::is_exactly_zero(abs(qz.matrixT()(i, j)))) {
        std::cerr << "Error: T(" << i << "," << j << ") = " << qz.matrixT()(i, j) << std::endl;
        all_zeros = false;
      }
      if (j < i - 1 && !numext::is_exactly_zero(abs(qz.matrixS()(i, j)))) {
        std::cerr << "Error: S(" << i << "," << j << ") = " << qz.matrixS()(i, j) << std::endl;
        all_zeros = false;
      }
      if (j == i - 1 && j > 0 && !numext::is_exactly_zero(abs(qz.matrixS()(i, j))) &&
          !numext::is_exactly_zero(abs(qz.matrixS()(i - 1, j - 1)))) {
        std::cerr << "Error: S(" << i << "," << j << ") = " << qz.matrixS()(i, j) << " && S(" << i - 1 << "," << j - 1
                  << ") = " << qz.matrixS()(i - 1, j - 1) << std::endl;
        all_zeros = false;
      }
    }
  VERIFY_IS_EQUAL(all_zeros, true);
  VERIFY_IS_APPROX(qz.matrixQ() * qz.matrixS() * qz.matrixZ(), A);
  VERIFY_IS_APPROX(qz.matrixQ() * qz.matrixT() * qz.matrixZ(), B);
  VERIFY_IS_APPROX(qz.matrixQ() * qz.matrixQ().adjoint(), MatrixType::Identity(dim, dim));
  VERIFY_IS_APPROX(qz.matrixZ() * qz.matrixZ().adjoint(), MatrixType::Identity(dim, dim));
}

// Regression test for the iteration-cap accounting bug in
// RealQZ::compute. On a 3x3 all-zero pencil (a degenerate case observed
// in production), every iteration of the inner while loop took the
// pushDownZero branch and made no progress: pushDownZero(2, 0, 2) is a
// no-op because its inner for(zz=z; zz<l; ++zz) loop runs zero times
// and the trailing Givens rotation operates on already-zero entries.
// Before the fix, local_iter was not incremented on the pushDownZero
// branch, so the m_maxIters cap never engaged and compute() looped
// indefinitely. After the fix, every "no convergence yet" iteration
// counts toward the cap, so the loop exits with NoConvergence.
template <typename MatrixType>
void real_qz_iteration_cap_regression() {
  MatrixType A = MatrixType::Zero(3, 3);
  MatrixType B = MatrixType::Zero(3, 3);
  RealQZ<MatrixType> qz(3);
  qz.setMaxIterations(50);
  qz.compute(A, B);
  VERIFY_IS_EQUAL(qz.info(), NoConvergence);
}

EIGEN_DECLARE_TEST(real_qz) {
  int s = 0;
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(real_qz(Matrix4f()));
    s = internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 4);
    CALL_SUBTEST_2(real_qz(MatrixXd(s, s)));

    // some trivial but implementation-wise tricky cases
    CALL_SUBTEST_2(real_qz(MatrixXd(1, 1)));
    CALL_SUBTEST_2(real_qz(MatrixXd(2, 2)));
    CALL_SUBTEST_3(real_qz(Matrix<double, 1, 1>()));
    CALL_SUBTEST_4(real_qz(Matrix2d()));

    CALL_SUBTEST_2(real_qz_iteration_cap_regression<MatrixXd>());
  }

  TEST_SET_BUT_UNUSED_VARIABLE(s);
}
