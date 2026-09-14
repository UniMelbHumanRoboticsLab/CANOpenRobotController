// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2006-2010 Benoit Jacob <jacob.benoit.1@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "main.h"
#include "random_for_arithmetic.h"

template <typename MatrixType>
void diagonal(const MatrixType& m) {
  typedef typename MatrixType::Scalar Scalar;

  Index rows = m.rows();
  Index cols = m.cols();

  MatrixType m1 = random_for_arithmetic<MatrixType>(rows, cols), m2 = random_for_arithmetic<MatrixType>(rows, cols);

  Scalar s1 = random_scalar_for_arithmetic<Scalar>();

  // check diagonal()
  VERIFY_IS_APPROX(m1.diagonal(), m1.transpose().diagonal());
  m2.diagonal() = 2 * m1.diagonal();
  m2.diagonal()[0] *= 3;

  if (rows > 2) {
    enum { N1 = MatrixType::RowsAtCompileTime > 2 ? 2 : 0, N2 = MatrixType::RowsAtCompileTime > 1 ? -1 : 0 };

    // check sub/super diagonal
    if (MatrixType::SizeAtCompileTime != Dynamic) {
      VERIFY(m1.template diagonal<N1>().RowsAtCompileTime == m1.diagonal(N1).size());
      VERIFY(m1.template diagonal<N2>().RowsAtCompileTime == m1.diagonal(N2).size());
    }

    m2.template diagonal<N1>() = 2 * m1.template diagonal<N1>();
    VERIFY_IS_APPROX(m2.template diagonal<N1>(), static_cast<Scalar>(2) * m1.diagonal(N1));
    m2.template diagonal<N1>()[0] *= 3;
    VERIFY_IS_APPROX(m2.template diagonal<N1>()[0], static_cast<Scalar>(6) * m1.template diagonal<N1>()[0]);

    m2.template diagonal<N2>() = 2 * m1.template diagonal<N2>();
    m2.template diagonal<N2>()[0] *= 3;
    VERIFY_IS_APPROX(m2.template diagonal<N2>()[0], static_cast<Scalar>(6) * m1.template diagonal<N2>()[0]);

    m2.diagonal(N1) = 2 * m1.diagonal(N1);
    VERIFY_IS_APPROX(m2.template diagonal<N1>(), static_cast<Scalar>(2) * m1.diagonal(N1));
    m2.diagonal(N1)[0] *= 3;
    VERIFY_IS_APPROX(m2.diagonal(N1)[0], static_cast<Scalar>(6) * m1.diagonal(N1)[0]);

    m2.diagonal(N2) = 2 * m1.diagonal(N2);
    VERIFY_IS_APPROX(m2.template diagonal<N2>(), static_cast<Scalar>(2) * m1.diagonal(N2));
    m2.diagonal(N2)[0] *= 3;
    VERIFY_IS_APPROX(m2.diagonal(N2)[0], static_cast<Scalar>(6) * m1.diagonal(N2)[0]);

    m2.diagonal(N2).x() = s1;
    VERIFY_IS_APPROX(m2.diagonal(N2).x(), s1);
    m2.diagonal(N2).coeffRef(0) = Scalar(2) * s1;
    VERIFY_IS_APPROX(m2.diagonal(N2).coeff(0), Scalar(2) * s1);
  }

  VERIFY(m1.diagonal(cols).size() == 0);
  VERIFY(m1.diagonal(-rows).size() == 0);
}

template <typename MatrixType>
void diagonal_assert(const MatrixType& m) {
  Index rows = m.rows();
  Index cols = m.cols();

  MatrixType m1 = MatrixType::Random(rows, cols);

  if (rows >= 2 && cols >= 2) {
    VERIFY_RAISES_ASSERT(m1 += m1.diagonal());
    VERIFY_RAISES_ASSERT(m1 -= m1.diagonal());
    VERIFY_RAISES_ASSERT(m1.array() *= m1.diagonal().array());
    VERIFY_RAISES_ASSERT(m1.array() /= m1.diagonal().array());
  }

  VERIFY_RAISES_ASSERT(m1.diagonal(cols + 1));
  VERIFY_RAISES_ASSERT(m1.diagonal(-(rows + 1)));
}

// Test that (A * B).diagonal() gives the same result as (A * B).eval().diagonal().
// The diagonal-of-product path uses LazyProduct evaluation (see ProductEvaluators.h),
// which avoids computing the full product. Verify this optimization is correct.
template <typename Scalar>
void diagonal_of_product() {
  const Index PS = internal::packet_traits<Scalar>::size;
  const Index sizes[] = {1, 2, 3, PS - 1, PS, PS + 1, 2 * PS - 1, 2 * PS, 2 * PS + 1, 4 * PS, 4 * PS + 1};
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;
  typedef Matrix<Scalar, Dynamic, 1> Vec;

  for (int si = 0; si < 11; ++si) {
    Index n = sizes[si];
    if (n <= 0) continue;

    Mat A = random_for_arithmetic<Mat>(n, n);
    Mat B = random_for_arithmetic<Mat>(n, n);

    // Lazy diagonal vs explicit product diagonal
    Vec diag_lazy = (A * B).diagonal();
    Vec diag_explicit = (A * B).eval().diagonal();
    VERIFY_IS_APPROX(diag_lazy, diag_explicit);

    // Also test non-square: A is m×k, B is k×n
    for (int k : {1, 3, (int)n}) {
      if (k <= 0) continue;
      Mat C = random_for_arithmetic<Mat>(n, k);
      Mat D = random_for_arithmetic<Mat>(k, n);
      Vec diag_lazy2 = (C * D).diagonal();
      Vec diag_explicit2 = (C * D).eval().diagonal();
      VERIFY_IS_APPROX(diag_lazy2, diag_explicit2);
    }
  }
}

// Test .select() at vectorization boundary sizes.
// select() uses CwiseTernaryOp which has packet-level evaluation with remainder handling.
template <typename Scalar>
void select_boundary() {
  const Index PS = internal::packet_traits<Scalar>::size;
  const Index sizes[] = {1, 2, 3, PS - 1, PS, PS + 1, 2 * PS - 1, 2 * PS, 2 * PS + 1, 4 * PS, 4 * PS + 1};
  typedef Array<Scalar, Dynamic, 1> Arr;

  for (int si = 0; si < 11; ++si) {
    Index n = sizes[si];
    if (n <= 0) continue;

    Arr a = Arr::Random(n);
    Arr b = Arr::Random(n);
    auto cond = (a > Scalar(0));

    // select with two arrays
    Arr result = cond.select(a, b);
    for (Index k = 0; k < n; ++k) {
      Scalar expected = (a(k) > Scalar(0)) ? a(k) : b(k);
      VERIFY_IS_APPROX(result(k), expected);
    }

    // select with scalar else
    Arr result2 = cond.select(a, Scalar(0));
    for (Index k = 0; k < n; ++k) {
      Scalar expected = (a(k) > Scalar(0)) ? a(k) : Scalar(0);
      VERIFY_IS_APPROX(result2(k), expected);
    }

    // select with scalar then
    Arr result3 = cond.select(Scalar(42), b);
    for (Index k = 0; k < n; ++k) {
      Scalar expected = (a(k) > Scalar(0)) ? Scalar(42) : b(k);
      VERIFY_IS_APPROX(result3(k), expected);
    }
  }
}

EIGEN_DECLARE_TEST(diagonal) {
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(diagonal(Matrix<float, 1, 1>()));
    CALL_SUBTEST_1(diagonal(Matrix<float, 4, 9>()));
    CALL_SUBTEST_1(diagonal(Matrix<float, 7, 3>()));
    CALL_SUBTEST_2(diagonal(Matrix4d()));
    CALL_SUBTEST_2(diagonal(
        MatrixXcf(internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_2(diagonal(
        MatrixXi(internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_2(diagonal(
        MatrixXcd(internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_1(diagonal(
        MatrixXf(internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_1(diagonal(Matrix<float, Dynamic, 4>(3, 4)));
    CALL_SUBTEST_1(diagonal_assert(
        MatrixXf(internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
  }

  // Diagonal-of-product optimization (deterministic, outside g_repeat).
  CALL_SUBTEST_3(diagonal_of_product<float>());
  CALL_SUBTEST_3(diagonal_of_product<double>());
  CALL_SUBTEST_3(diagonal_of_product<std::complex<float>>());

  // Select at vectorization boundaries (deterministic, outside g_repeat).
  CALL_SUBTEST_4(select_boundary<float>());
  CALL_SUBTEST_4(select_boundary<double>());
  CALL_SUBTEST_4(select_boundary<int>());
}
