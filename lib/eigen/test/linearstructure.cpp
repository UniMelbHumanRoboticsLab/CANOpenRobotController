// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2006-2008 Benoit Jacob <jacob.benoit.1@gmail.com>
// Copyright (C) 2014 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

static bool g_called;
#define EIGEN_SCALAR_BINARY_OP_PLUGIN \
  { g_called |= (!std::is_same<LhsScalar, RhsScalar>::value); }

#include "main.h"
#include "random_for_arithmetic.h"
#include "fp_control.h"

template <typename MatrixType>
void linearStructure(const MatrixType& m) {
  using std::abs;
  /* this test covers the following files:
     CwiseUnaryOp.h, CwiseBinaryOp.h, SelfCwiseBinaryOp.h
  */
  typedef typename MatrixType::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;

  Index rows = m.rows();
  Index cols = m.cols();

  // this test relies a lot on Random.h, and there's not much more that we can do
  // to test it, hence I consider that we will have tested Random.h
  MatrixType m1 = random_for_arithmetic<MatrixType>(rows, cols), m2 = random_for_arithmetic<MatrixType>(rows, cols),
             m3(rows, cols);

  Scalar s1 = random_scalar_for_arithmetic<Scalar>();
  if (s1 == Scalar(0)) s1 = Scalar(1);

  Index r = internal::random<Index>(0, rows - 1), c = internal::random<Index>(0, cols - 1);

  VERIFY_IS_APPROX(-(-m1), m1);
  VERIFY_IS_APPROX(m1 + m1, 2 * m1);
  VERIFY_IS_APPROX(m1 + m2 - m1, m2);
  VERIFY_IS_APPROX(-m2 + m1 + m2, m1);
  VERIFY_IS_APPROX(m1 * s1, s1 * m1);
  if (NumTraits<Scalar>::IsInteger) {
    // Bounded integer arithmetic is exactly distributive.
    VERIFY_IS_APPROX((m1 + m2) * s1, s1 * m1 + s1 * m2);
    VERIFY_IS_APPROX((-m1 + m2) * s1, -s1 * m1 + s1 * m2);
  } else {
    // Both forms of the distributive law round to within eps * |s1| * (|m1| + |m2|) of the exact value,
    // which is unbounded relative to a result formed by cancellation.
    const RealScalar scale = numext::abs(s1) * (max_abs_coeff(m1) + max_abs_coeff(m2));
    VERIFY_IS_APPROX_SCALED((m1 + m2) * s1, s1 * m1 + s1 * m2, scale);
    VERIFY_IS_APPROX_SCALED((-m1 + m2) * s1, -s1 * m1 + s1 * m2, scale);
  }
  m3 = m2;
  m3 += m1;
  VERIFY_IS_APPROX(m3, m1 + m2);
  m3 = m2;
  m3 -= m1;
  VERIFY_IS_APPROX(m3, m2 - m1);
  m3 = m2;
  m3 *= s1;
  VERIFY_IS_APPROX(m3, s1 * m2);
  if (!NumTraits<Scalar>::IsInteger) {
    m3 = m2;
    m3 /= s1;
    VERIFY_IS_APPROX(m3, m2 / s1);
  }

  // again, test operator() to check const-qualification
  VERIFY_IS_APPROX((-m1)(r, c), -(m1(r, c)));
  VERIFY_IS_APPROX((m1 - m2)(r, c), (m1(r, c)) - (m2(r, c)));
  VERIFY_IS_APPROX((m1 + m2)(r, c), (m1(r, c)) + (m2(r, c)));
  VERIFY_IS_APPROX((s1 * m1)(r, c), s1 * (m1(r, c)));
  VERIFY_IS_APPROX((m1 * s1)(r, c), (m1(r, c)) * s1);
  if (!NumTraits<Scalar>::IsInteger) VERIFY_IS_APPROX((m1 / s1)(r, c), (m1(r, c)) / s1);

  // use .block to disable vectorization and compare to the vectorized version
  VERIFY_IS_APPROX(m1 + m1.block(0, 0, rows, cols), m1 + m1);
  VERIFY_IS_APPROX(m1.cwiseProduct(m1.block(0, 0, rows, cols)), m1.cwiseProduct(m1));
  VERIFY_IS_APPROX(m1 - m1.block(0, 0, rows, cols), m1 - m1);
  VERIFY_IS_APPROX(m1.block(0, 0, rows, cols) * s1, m1 * s1);
}

// Make sure that complex * real and real * complex are properly optimized
template <typename MatrixType>
void real_complex(DenseIndex rows = MatrixType::RowsAtCompileTime, DenseIndex cols = MatrixType::ColsAtCompileTime) {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename MatrixType::RealScalar RealScalar;

  RealScalar s = internal::random<RealScalar>();
  MatrixType m1 = MatrixType::Random(rows, cols);

  g_called = false;
  VERIFY_IS_APPROX(s * m1, Scalar(s) * m1);
  VERIFY(g_called && "real * matrix<complex> not properly optimized");

  g_called = false;
  VERIFY_IS_APPROX(m1 * s, m1 * Scalar(s));
  VERIFY(g_called && "matrix<complex> * real not properly optimized");

  g_called = false;
  VERIFY_IS_APPROX(m1 / s, m1 / Scalar(s));
  VERIFY(g_called && "matrix<complex> / real not properly optimized");

  g_called = false;
  VERIFY_IS_APPROX(s + m1.array(), Scalar(s) + m1.array());
  VERIFY(g_called && "real + matrix<complex> not properly optimized");

  g_called = false;
  VERIFY_IS_APPROX(m1.array() + s, m1.array() + Scalar(s));
  VERIFY(g_called && "matrix<complex> + real not properly optimized");

  g_called = false;
  VERIFY_IS_APPROX(s - m1.array(), Scalar(s) - m1.array());
  VERIFY(g_called && "real - matrix<complex> not properly optimized");

  g_called = false;
  VERIFY_IS_APPROX(m1.array() - s, m1.array() - Scalar(s));
  VERIFY(g_called && "matrix<complex> - real not properly optimized");
}

// Test linear structure operations between matrices with different storage orders.
// When storage orders differ, vectorization is disabled (StorageOrdersAgree=false in
// AssignEvaluator.h), exercising the scalar fallback path.
template <typename Scalar>
void linearStructure_mixed_storage() {
  const Index PS = internal::packet_traits<Scalar>::size;
  // Sizes at vectorization boundaries to expose any mismatch in traversal
  const Index sizes[] = {1, PS, PS + 1, 2 * PS, 2 * PS + 1, 4 * PS + 1, 16};
  typedef Matrix<Scalar, Dynamic, Dynamic, ColMajor> ColMat;
  typedef Matrix<Scalar, Dynamic, Dynamic, RowMajor> RowMat;

  for (int si = 0; si < 7; ++si) {
    Index n = sizes[si];
    if (n <= 0) continue;
    ColMat mc = random_for_arithmetic<ColMat>(n, n);
    RowMat mr = random_for_arithmetic<RowMat>(n, n);

    // ColMajor + RowMajor → ColMajor
    ColMat sum_c = mc + mr;
    for (Index j = 0; j < n; ++j)
      for (Index i = 0; i < n; ++i) VERIFY_IS_APPROX(sum_c(i, j), mc(i, j) + mr(i, j));

    // ColMajor - RowMajor → ColMajor
    ColMat diff_c = mc - mr;
    for (Index j = 0; j < n; ++j)
      for (Index i = 0; i < n; ++i) VERIFY_IS_APPROX(diff_c(i, j), mc(i, j) - mr(i, j));

    // RowMajor + ColMajor → RowMajor
    RowMat sum_r = mr + mc;
    for (Index j = 0; j < n; ++j)
      for (Index i = 0; i < n; ++i) VERIFY_IS_APPROX(sum_r(i, j), mr(i, j) + mc(i, j));

    // Assignment between storage orders
    ColMat from_row = mr;
    VERIFY_IS_APPROX(from_row, mr);
    RowMat from_col = mc;
    VERIFY_IS_APPROX(from_col, mc);

    // cwiseProduct with mixed storage
    ColMat cwp = mc.cwiseProduct(mr);
    for (Index j = 0; j < n; ++j)
      for (Index i = 0; i < n; ++i) VERIFY_IS_APPROX(cwp(i, j), mc(i, j) * mr(i, j));

    // += with mixed storage
    ColMat mc2 = mc;
    mc2 += mr;
    VERIFY_IS_APPROX(mc2, sum_c);
  }
}

// Regression test for the seed-dependent failure of the distributive law checks above: when m1 and m2
// agree to all but the last few bits, m1 + m2 (resp. -m1 + m2) is formed by cancellation while the
// rounding error of s1 * m1 + s1 * m2 stays proportional to |s1| * (|m1| + |m2|).
template <typename Scalar>
void linearStructure_cancellation() {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, 1, 1> MatrixType;

  const RealScalar close = RealScalar(1) - RealScalar(64) * NumTraits<RealScalar>::epsilon();
  const Scalar s1 = Scalar(RealScalar(-0.465));
  MatrixType m1, m2;
  m1(0, 0) = Scalar(RealScalar(0.35));

  for (int i = 0; i < 2; ++i) {
    // i == 0 cancels in m1 + m2, i == 1 cancels in -m1 + m2.
    m2(0, 0) = (i == 0 ? -m1(0, 0) : m1(0, 0)) * close;
    const RealScalar scale = numext::abs(s1) * (max_abs_coeff(m1) + max_abs_coeff(m2));
    VERIFY_IS_APPROX_SCALED((m1 + m2) * s1, s1 * m1 + s1 * m2, scale);
    VERIFY_IS_APPROX_SCALED((-m1 + m2) * s1, -s1 * m1 + s1 * m2, scale);
  }
}

template <int>
void linearstructure_overflow() {
  // make sure that /=scalar and /scalar do not overflow
  // rational: 1.0/4.94e-320 overflow, but m/4.94e-320 should not
  //
  // The claim is about Eigen: that it divides rather than multiplying by a
  // reciprocal.  An environment that flushes subnormal operands, or whose
  // compiler performs that rewrite itself, cannot answer it -- the quotient is
  // infinite either way, so a failure would not distinguish Eigen's arithmetic
  // from the compiler's.  NVHPC is such an environment by default, through
  // -Knoieee.
  if (!subnormalDivisionIsExact<double>()) {
    const char* reason = ScopedFlushToZero::hardwareFlushesSubnormalInputs() ? "the hardware flushes subnormal inputs"
                                                                             : "the compiler relaxed the division";
    std::cout << "SKIP: linearstructure_overflow needs an environment that divides by subnormals per IEEE 754 ("
              << reason << ")." << std::endl;
    return;
  }

  Matrix4d m2, m3;
  m3 = m2 = Matrix4d::Random() * 1e-20;
  m2 = m2 / 4.9e-320;
  VERIFY_IS_APPROX(m2.cwiseQuotient(m2), Matrix4d::Ones());
  m3 /= 4.9e-320;
  VERIFY_IS_APPROX(m3.cwiseQuotient(m3), Matrix4d::Ones());
}

EIGEN_DECLARE_TEST(linearstructure) {
  g_called = true;
  VERIFY(g_called);  // avoid `unneeded-internal-declaration` warning.
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(linearStructure(Matrix<float, 1, 1>()));
    CALL_SUBTEST_2(linearStructure(Matrix2f()));
    CALL_SUBTEST_3(linearStructure(Vector3d()));
    CALL_SUBTEST_4(linearStructure(Matrix4d()));
    CALL_SUBTEST_5(linearStructure(MatrixXcf(internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2),
                                             internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2))));
    CALL_SUBTEST_6(linearStructure(
        MatrixXf(internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_7(linearStructure(
        MatrixXi(internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_8(linearStructure(MatrixXcd(internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2),
                                             internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2))));
    CALL_SUBTEST_9(linearStructure(
        ArrayXXf(internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_10(linearStructure(
        ArrayXXcf(internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));

    CALL_SUBTEST_11(real_complex<Matrix4cd>());
    CALL_SUBTEST_11(real_complex<MatrixXcf>(10, 10));
    CALL_SUBTEST_11(real_complex<ArrayXXcf>(10, 10));
  }
  CALL_SUBTEST_4(linearstructure_overflow<0>());

  // Deterministic tests, outside g_repeat.
  CALL_SUBTEST_12(linearStructure_cancellation<float>());
  CALL_SUBTEST_12(linearStructure_cancellation<double>());
  CALL_SUBTEST_12(linearStructure_cancellation<std::complex<float>>());

  // Mixed storage order tests.
  CALL_SUBTEST_12(linearStructure_mixed_storage<float>());
  CALL_SUBTEST_12(linearStructure_mixed_storage<double>());
  CALL_SUBTEST_12(linearStructure_mixed_storage<std::complex<float>>());
}
