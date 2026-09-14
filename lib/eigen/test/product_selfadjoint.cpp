// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2009 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "main.h"

template <typename MatrixType>
void product_selfadjoint(const MatrixType& m) {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, MatrixType::RowsAtCompileTime, 1> VectorType;
  typedef Matrix<Scalar, 1, MatrixType::RowsAtCompileTime> RowVectorType;

  typedef Matrix<Scalar, MatrixType::RowsAtCompileTime, Dynamic, RowMajor> RhsMatrixType;

  Index rows = m.rows();
  Index cols = m.cols();

  MatrixType m1 = MatrixType::Random(rows, cols), m2 = MatrixType::Random(rows, cols), m3;
  VectorType v1 = VectorType::Random(rows), v2 = VectorType::Random(rows), v3(rows);
  RowVectorType r1 = RowVectorType::Random(rows), r2 = RowVectorType::Random(rows);
  RhsMatrixType m4 = RhsMatrixType::Random(rows, 10);

  Scalar s1 = internal::random<Scalar>(), s2 = internal::random<Scalar>(), s3 = internal::random<Scalar>();

  m1 = (m1.adjoint() + m1).eval();

  // Dense selfadjoint assignment is documented as writing only the referenced triangle.
  m2.setRandom();
  m3 = m2;
  m2.template selfadjointView<Upper>() = m1 + m1.adjoint();
  m3.template triangularView<Upper>() = m1 + m1.adjoint();
  VERIFY_IS_APPROX(m2, m3);

  m2.setRandom();
  m3 = m2;
  m2.template selfadjointView<Upper>().setZero();
  m3.template triangularView<Upper>().setZero();
  VERIFY_IS_APPROX(m2, m3);

  m2.setRandom();
  m3 = m2;
  m2.template selfadjointView<Lower>().setOnes();
  m3.template triangularView<Lower>().setOnes();
  VERIFY_IS_APPROX(m2, m3);

  m2.setRandom();
  m3 = m2;
  m2.template selfadjointView<Upper>().setConstant(s1);
  m3.template triangularView<Upper>().setConstant(s1);
  VERIFY_IS_APPROX(m2, m3);

  m2.setRandom();
  m3 = m2;
  m2.template selfadjointView<Lower>().fill(s2);
  m3.template triangularView<Lower>().fill(s2);
  VERIFY_IS_APPROX(m2, m3);

  m2.setRandom();
  m3 = m2;
  m2.template selfadjointView<Upper>().setIdentity();
  m3.template triangularView<Upper>().setIdentity();
  VERIFY_IS_APPROX(m2, m3);

  m2.setRandom();
  m3 = m2;
  m2.template selfadjointView<Lower>().setRandom();
  VERIFY_IS_APPROX(m2.template triangularView<StrictlyUpper>().toDenseMatrix(),
                   m3.template triangularView<StrictlyUpper>().toDenseMatrix());

  m2.setRandom();
  m3 = m2;
  m2.template selfadjointView<Upper>() += m1;
  m3.template triangularView<Upper>() += m1;
  VERIFY_IS_APPROX(m2, m3);

  m2.setRandom();
  m3 = m2;
  m2.template selfadjointView<Lower>() -= m1;
  m3.template triangularView<Lower>() -= m1;
  VERIFY_IS_APPROX(m2, m3);

  // SelfAdjointView in-place scaling/division accept real scalars only: scaling only the
  // stored triangle by a non-real factor would leave conj(other) on the unstored half.
  const RealScalar real_s1 = numext::real(s1);
  const RealScalar real_divisor = numext::real(s2) + RealScalar(2);
  m2.setRandom();
  m3 = m2;
  m2.template selfadjointView<Upper>() *= real_s1;
  m3.template triangularView<Upper>() *= real_s1;
  VERIFY_IS_APPROX(m2, m3);

  m2.setRandom();
  m3 = m2;
  m2.template selfadjointView<Lower>() /= real_divisor;
  m3.template triangularView<Lower>() /= real_divisor;
  VERIFY_IS_APPROX(m2, m3);

  m2.setRandom();
  m2.template selfadjointView<Lower>() = m1.template selfadjointView<Lower>();
  m3 = m1.template selfadjointView<Lower>().toDenseMatrix();
  VERIFY_IS_APPROX(m2, m3);

  m2.setRandom();
  m2.template selfadjointView<Lower>() = m1.template selfadjointView<Upper>();
  m3 = m1.template selfadjointView<Upper>().toDenseMatrix();
  VERIFY_IS_APPROX(m2, m3);

  m2.setRandom();
  m2.template selfadjointView<Lower>() = m1.template triangularView<Upper>();
  m3 = m1.template triangularView<Upper>().toDenseMatrix();
  VERIFY_IS_APPROX(m2, m3);

  m2.setRandom();
  m2.template selfadjointView<Lower>() = m1.template triangularView<UnitUpper>();
  m3 = m1.template triangularView<UnitUpper>().toDenseMatrix();
  VERIFY_IS_APPROX(m2, m3);

  m2.setRandom();
  m2.template selfadjointView<Upper>() = m1.template triangularView<StrictlyLower>();
  m3 = m1.template triangularView<StrictlyLower>().toDenseMatrix();
  VERIFY_IS_APPROX(m2, m3);

  // rank2 update
  m2 = m1.template triangularView<Lower>();
  m2.template selfadjointView<Lower>().rankUpdate(v1, v2);
  VERIFY_IS_APPROX(m2, (m1 + v1 * v2.adjoint() + v2 * v1.adjoint()).template triangularView<Lower>().toDenseMatrix());

  m2 = m1.template triangularView<Upper>();
  m2.template selfadjointView<Upper>().rankUpdate(-v1, s2 * v2, s3);
  VERIFY_IS_APPROX(m2, (m1 + (s3 * (-v1) * (s2 * v2).adjoint() + numext::conj(s3) * (s2 * v2) * (-v1).adjoint()))
                           .template triangularView<Upper>()
                           .toDenseMatrix());

  m2 = m1.template triangularView<Upper>();
  m2.template selfadjointView<Upper>().rankUpdate(-s2 * r1.adjoint(), r2.adjoint() * s3, s1);
  VERIFY_IS_APPROX(m2, (m1 + s1 * (-s2 * r1.adjoint()) * (r2.adjoint() * s3).adjoint() +
                        numext::conj(s1) * (r2.adjoint() * s3) * (-s2 * r1.adjoint()).adjoint())
                           .template triangularView<Upper>()
                           .toDenseMatrix());

  if (rows > 1) {
    m2 = m1.template triangularView<Lower>();
    m2.block(1, 1, rows - 1, cols - 1)
        .template selfadjointView<Lower>()
        .rankUpdate(v1.tail(rows - 1), v2.head(cols - 1));
    m3 = m1;
    m3.block(1, 1, rows - 1, cols - 1) +=
        v1.tail(rows - 1) * v2.head(cols - 1).adjoint() + v2.head(cols - 1) * v1.tail(rows - 1).adjoint();
    VERIFY_IS_APPROX(m2, m3.template triangularView<Lower>().toDenseMatrix());
  }

  // matrix-vector
  m2 = m1.template triangularView<Lower>();
  VERIFY_IS_APPROX(m1 * m4, m2.template selfadjointView<Lower>() * m4);
}

// Test selfadjoint products at blocking boundary sizes.
// The existing test uses random sizes; this tests deterministic sizes
// at transitions (especially around the GEBP early-return threshold of 48).
template <int>
void product_selfadjoint_boundary() {
  typedef double Scalar;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;
  typedef Matrix<Scalar, Dynamic, 1> Vec;

  const int sizes[] = {1, 2, 3, 4, 8, 16, 47, 48, 49, 64, 96, 128};
  for (int si = 0; si < 12; ++si) {
    int n = sizes[si];
    Mat m1 = Mat::Random(n, n);
    m1 = (m1 + m1.transpose()).eval();  // make symmetric

    Vec v1 = Vec::Random(n);
    Mat rhs = Mat::Random(n, 5);

    // Lower selfadjointView * vector
    Mat m2 = m1.triangularView<Lower>();
    VERIFY_IS_APPROX(m2.selfadjointView<Lower>() * v1, m1 * v1);

    // Upper selfadjointView * vector
    m2 = m1.triangularView<Upper>();
    VERIFY_IS_APPROX(m2.selfadjointView<Upper>() * v1, m1 * v1);

    // selfadjointView * matrix
    m2 = m1.triangularView<Lower>();
    VERIFY_IS_APPROX(m2.selfadjointView<Lower>() * rhs, m1 * rhs);

    // rankUpdate
    Vec v2 = Vec::Random(n);
    m2 = m1.triangularView<Lower>();
    m2.selfadjointView<Lower>().rankUpdate(v1, v2);
    VERIFY_IS_APPROX(m2, (m1 + v1 * v2.transpose() + v2 * v1.transpose()).triangularView<Lower>().toDenseMatrix());
  }
}

// Same test for complex type (tests conjugation logic).
template <int>
void product_selfadjoint_boundary_complex() {
  typedef std::complex<float> Scalar;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;
  typedef Matrix<Scalar, Dynamic, 1> Vec;

  const int sizes[] = {1, 8, 47, 48, 49, 64};
  for (int si = 0; si < 6; ++si) {
    int n = sizes[si];
    Mat m1 = Mat::Random(n, n);
    m1 = (m1 + m1.adjoint()).eval();                               // make Hermitian
    m1.diagonal() = m1.diagonal().real().template cast<Scalar>();  // real diagonal

    Vec v1 = Vec::Random(n);
    Mat rhs = Mat::Random(n, 3);

    Mat m2 = m1.triangularView<Lower>();
    VERIFY_IS_APPROX(m2.selfadjointView<Lower>() * v1, m1 * v1);
    VERIFY_IS_APPROX(m2.selfadjointView<Lower>() * rhs, m1 * rhs);

    m2 = m1.triangularView<Upper>();
    VERIFY_IS_APPROX(m2.selfadjointView<Upper>() * v1, m1 * v1);
  }
}

EIGEN_DECLARE_TEST(product_selfadjoint) {
  int s = 0;
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(product_selfadjoint(Matrix<float, 1, 1>()));
    CALL_SUBTEST_2(product_selfadjoint(Matrix<float, 2, 2>()));
    CALL_SUBTEST_3(product_selfadjoint(Matrix3d()));

    s = internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2);
    CALL_SUBTEST_4(product_selfadjoint(MatrixXcf(s, s)));
    TEST_SET_BUT_UNUSED_VARIABLE(s);

    s = internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2);
    CALL_SUBTEST_5(product_selfadjoint(MatrixXcd(s, s)));
    TEST_SET_BUT_UNUSED_VARIABLE(s);

    s = internal::random<int>(1, EIGEN_TEST_MAX_SIZE);
    CALL_SUBTEST_6(product_selfadjoint(MatrixXd(s, s)));
    TEST_SET_BUT_UNUSED_VARIABLE(s);

    s = internal::random<int>(1, EIGEN_TEST_MAX_SIZE);
    CALL_SUBTEST_7(product_selfadjoint(Matrix<float, Dynamic, Dynamic, RowMajor>(s, s)));
    TEST_SET_BUT_UNUSED_VARIABLE(s);
  }

  // Deterministic blocking boundary tests (outside g_repeat).
  CALL_SUBTEST_8(product_selfadjoint_boundary<0>());
  CALL_SUBTEST_9(product_selfadjoint_boundary_complex<0>());
}
