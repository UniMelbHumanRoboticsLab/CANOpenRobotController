// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "main.h"

template <typename MatrixType>
void replicate(const MatrixType& m) {
  /* this test covers the following files:
     Replicate.cpp
  */
  typedef typename MatrixType::Scalar Scalar;
  typedef Matrix<Scalar, MatrixType::RowsAtCompileTime, 1> VectorType;
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixX;
  typedef Matrix<Scalar, Dynamic, 1> VectorX;

  Index rows = m.rows();
  Index cols = m.cols();

  MatrixType m1 = MatrixType::Random(rows, cols), m2 = MatrixType::Random(rows, cols);

  VectorType v1 = VectorType::Random(rows);

  MatrixX x1, x2;
  VectorX vx1;

  int f1 = internal::random<int>(1, 10), f2 = internal::random<int>(1, 10);

  x1.resize(rows * f1, cols * f2);
  for (int j = 0; j < f2; j++)
    for (int i = 0; i < f1; i++) x1.block(i * rows, j * cols, rows, cols) = m1;
  VERIFY_IS_APPROX(x1, m1.replicate(f1, f2));

  x2.resize(2 * rows, 3 * cols);
  x2 << m2, m2, m2, m2, m2, m2;
  VERIFY_IS_APPROX(x2, (m2.template replicate<2, 3>()));

  x2.resize(rows, 3 * cols);
  x2 << m2, m2, m2;
  VERIFY_IS_APPROX(x2, (m2.template replicate<1, 3>()));

  vx1.resize(3 * rows, cols);
  vx1 << m2, m2, m2;
  VERIFY_IS_APPROX(vx1 + vx1, vx1 + (m2.template replicate<3, 1>()));

  vx1 = m2 + (m2.colwise().replicate(1));

  if (m2.cols() == 1) VERIFY_IS_APPROX(m2.coeff(0), (m2.template replicate<3, 1>().coeff(m2.rows())));

  x2.resize(rows, f1);
  for (int j = 0; j < f1; ++j) x2.col(j) = v1;
  VERIFY_IS_APPROX(x2, v1.rowwise().replicate(f1));

  vx1.resize(rows * f2);
  for (int j = 0; j < f2; ++j) vx1.segment(j * rows, rows) = v1;
  VERIFY_IS_APPROX(vx1, v1.colwise().replicate(f2));
}

// A Replicate packet cannot cross a replication boundary, so the evaluator may serve packets
// exactly when the inner (storage-order) direction is not replicated; without the flag, colwise
// and rowwise broadcast operations silently fall back to scalar traversal.
template <typename Scalar>
void check_replicate_evaluator_flags() {
  // Storage orders are pinned so the checks keep their meaning under EIGEN_DEFAULT_TO_ROW_MAJOR.
  typedef Matrix<Scalar, Dynamic, Dynamic, ColMajor> Mat;
  typedef Matrix<Scalar, Dynamic, Dynamic, RowMajor> RowMat;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, 1, Dynamic> RowVec;
  enum { BasePacket = int(internal::evaluator<Mat>::Flags) & PacketAccessBit };

  // Inner direction not replicated: packets serve from a single copy of the argument. The first
  // two are the shapes colwise (col-major) and rowwise (row-major) operations expand to.
  STATIC_CHECK((int(internal::evaluator<Replicate<Vec, 1, Dynamic> >::Flags) & PacketAccessBit) == int(BasePacket));
  STATIC_CHECK((int(internal::evaluator<Replicate<RowVec, Dynamic, 1> >::Flags) & PacketAccessBit) == int(BasePacket));
  STATIC_CHECK((int(internal::evaluator<Replicate<Mat, 1, 3> >::Flags) & PacketAccessBit) == int(BasePacket));
  STATIC_CHECK((int(internal::evaluator<Replicate<RowMat, 3, 1> >::Flags) & PacketAccessBit) == int(BasePacket));
  STATIC_CHECK(int(internal::evaluator<Replicate<Vec, 1, Dynamic> >::Alignment) ==
               int(internal::evaluator<Vec>::Alignment));

  // Replicated inner direction (or factors unknown at compile time): a packet could cross a copy
  // boundary, so there is no packet access.
  STATIC_CHECK((int(internal::evaluator<Replicate<Mat, 3, 1> >::Flags) & PacketAccessBit) == 0);
  STATIC_CHECK((int(internal::evaluator<Replicate<Vec, Dynamic, 1> >::Flags) & PacketAccessBit) == 0);
  STATIC_CHECK((int(internal::evaluator<Replicate<Mat, Dynamic, Dynamic> >::Flags) & PacketAccessBit) == 0);
}

// Exercise the (possibly vectorized) broadcast kernels with sizes that have partial-packet tails.
template <typename Scalar>
void replicate_broadcasts(Index rows, Index cols) {
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, 1, Dynamic> RowVec;

  Mat m = Mat::Random(rows, cols);
  Vec v = Vec::Random(rows);
  RowVec rv = RowVec::Random(cols);

  Mat c = m;
  c.colwise() += v;
  for (Index j = 0; j < cols; ++j)
    for (Index i = 0; i < rows; ++i) VERIFY_IS_EQUAL(c(i, j), Scalar(m(i, j) + v(i)));

  Mat r = m;
  r.rowwise() += rv;
  for (Index j = 0; j < cols; ++j)
    for (Index i = 0; i < rows; ++i) VERIFY_IS_EQUAL(r(i, j), Scalar(m(i, j) + rv(j)));

  Mat h = m.template replicate<1, 3>();
  for (Index j = 0; j < 3 * cols; ++j)
    for (Index i = 0; i < rows; ++i) VERIFY_IS_EQUAL(h(i, j), m(i, j % cols));

  Mat ver = m.template replicate<3, 1>();
  for (Index j = 0; j < cols; ++j)
    for (Index i = 0; i < 3 * rows; ++i) VERIFY_IS_EQUAL(ver(i, j), m(i % rows, j));

  // A replicate nested inside a larger coefficient-wise expression.
  Mat sum = m + v.rowwise().replicate(cols);
  for (Index j = 0; j < cols; ++j)
    for (Index i = 0; i < rows; ++i) VERIFY_IS_EQUAL(sum(i, j), Scalar(m(i, j) + v(i)));
}

EIGEN_DECLARE_TEST(array_replicate) {
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(replicate(Matrix<float, 1, 1>()));
    CALL_SUBTEST_2(replicate(Vector2f()));
    CALL_SUBTEST_3(replicate(Vector3d()));
    CALL_SUBTEST_4(replicate(Vector4f()));
    CALL_SUBTEST_5(replicate(VectorXf(16)));
    CALL_SUBTEST_6(replicate(VectorXcd(10)));
    CALL_SUBTEST_7(check_replicate_evaluator_flags<float>());
    CALL_SUBTEST_7(replicate_broadcasts<float>(internal::random<Index>(1, 64), internal::random<Index>(1, 64)));
    CALL_SUBTEST_7(replicate_broadcasts<float>(17, 19));
    CALL_SUBTEST_8(check_replicate_evaluator_flags<double>());
    CALL_SUBTEST_8(replicate_broadcasts<double>(internal::random<Index>(1, 64), internal::random<Index>(1, 64)));
    CALL_SUBTEST_8(replicate_broadcasts<std::complex<float> >(9, 5));
  }
}
