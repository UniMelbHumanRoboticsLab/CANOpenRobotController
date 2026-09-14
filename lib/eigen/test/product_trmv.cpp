// This file is triangularView of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2009 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "main.h"
#include "product_test_helpers.h"

template <typename MatrixType>
void trmv(const MatrixType& m) {
  typedef typename MatrixType::Scalar Scalar;
  typedef Matrix<Scalar, MatrixType::RowsAtCompileTime, 1> VectorType;

  Index rows = m.rows();
  Index cols = m.cols();

  MatrixType m1 = MatrixType::Random(rows, cols), m3(rows, cols);
  VectorType v1 = VectorType::Random(rows);

  Scalar s1 = internal::random<Scalar>();

  m1 = MatrixType::Random(rows, cols);

  // check with a column-major matrix
  m3 = m1.template triangularView<Eigen::Lower>();
  VERIFY(verifyProduct(m3 * v1, m1.template triangularView<Eigen::Lower>() * v1, m3, v1));
  m3 = m1.template triangularView<Eigen::Upper>();
  VERIFY(verifyProduct(m3 * v1, m1.template triangularView<Eigen::Upper>() * v1, m3, v1));
  m3 = m1.template triangularView<Eigen::UnitLower>();
  VERIFY(verifyProduct(m3 * v1, m1.template triangularView<Eigen::UnitLower>() * v1, m3, v1));
  m3 = m1.template triangularView<Eigen::UnitUpper>();
  VERIFY(verifyProduct(m3 * v1, m1.template triangularView<Eigen::UnitUpper>() * v1, m3, v1));

  // check conjugated and scalar multiple expressions (col-major)
  m3 = m1.template triangularView<Eigen::Lower>();
  VERIFY(verifyProduct((s1 * m3).conjugate() * v1, (s1 * m1).conjugate().template triangularView<Eigen::Lower>() * v1,
                       (s1 * m3).conjugate(), v1));
  m3 = m1.template triangularView<Eigen::Upper>();
  VERIFY(verifyProduct(m3.conjugate() * v1.conjugate(),
                       m1.conjugate().template triangularView<Eigen::Upper>() * v1.conjugate(), m3.conjugate(),
                       v1.conjugate()));

  // check with a row-major matrix
  m3 = m1.template triangularView<Eigen::Upper>();
  VERIFY(verifyProduct(m3.transpose() * v1, m1.transpose().template triangularView<Eigen::Lower>() * v1, m3.transpose(),
                       v1));
  m3 = m1.template triangularView<Eigen::Lower>();
  VERIFY(verifyProduct(m3.transpose() * v1, m1.transpose().template triangularView<Eigen::Upper>() * v1, m3.transpose(),
                       v1));
  m3 = m1.template triangularView<Eigen::UnitUpper>();
  VERIFY(verifyProduct(m3.transpose() * v1, m1.transpose().template triangularView<Eigen::UnitLower>() * v1,
                       m3.transpose(), v1));
  m3 = m1.template triangularView<Eigen::UnitLower>();
  VERIFY(verifyProduct(m3.transpose() * v1, m1.transpose().template triangularView<Eigen::UnitUpper>() * v1,
                       m3.transpose(), v1));

  // check conjugated and scalar multiple expressions (row-major)
  m3 = m1.template triangularView<Eigen::Upper>();
  VERIFY(verifyProduct(m3.adjoint() * v1, m1.adjoint().template triangularView<Eigen::Lower>() * v1, m3.adjoint(), v1));
  m3 = m1.template triangularView<Eigen::Lower>();
  VERIFY(verifyProduct(m3.adjoint() * (s1 * v1.conjugate()),
                       m1.adjoint().template triangularView<Eigen::Upper>() * (s1 * v1.conjugate()), m3.adjoint(),
                       (s1 * v1.conjugate()).eval()));
  m3 = m1.template triangularView<Eigen::UnitUpper>();

  // check transposed cases:
  m3 = m1.template triangularView<Eigen::Lower>();
  VERIFY(verifyProduct(v1.transpose() * m3, v1.transpose() * m1.template triangularView<Eigen::Lower>(), v1.transpose(),
                       m3));
  VERIFY(verifyProduct(v1.adjoint() * m3, v1.adjoint() * m1.template triangularView<Eigen::Lower>(), v1.adjoint(), m3));
  VERIFY(verifyProduct(v1.adjoint() * m3.adjoint(), v1.adjoint() * m1.template triangularView<Eigen::Lower>().adjoint(),
                       v1.adjoint(), m3.adjoint()));

  // TODO check with sub-matrices
}

EIGEN_DECLARE_TEST(product_trmv) {
  int s = 0;
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(trmv(Matrix<float, 1, 1>()));
    CALL_SUBTEST_2(trmv(Matrix<float, 2, 2>()));
    CALL_SUBTEST_3(trmv(Matrix3d()));

    s = internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2);
    CALL_SUBTEST_4(trmv(MatrixXcf(s, s)));
    CALL_SUBTEST_5(trmv(MatrixXcd(s, s)));
    TEST_SET_BUT_UNUSED_VARIABLE(s);

    s = internal::random<int>(1, EIGEN_TEST_MAX_SIZE);
    CALL_SUBTEST_6(trmv(Matrix<float, Dynamic, Dynamic, RowMajor>(s, s)));
    TEST_SET_BUT_UNUSED_VARIABLE(s);
  }
}
