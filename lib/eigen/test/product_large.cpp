// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2006-2008 Benoit Jacob <jacob.benoit.1@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "product.h"
#include <Eigen/LU>

#if (defined(EIGEN_VECTORIZE_ALTIVEC) || defined(EIGEN_VECTORIZE_VSX)) && defined(__unix__)
#include <sys/mman.h>
#include <unistd.h>

template <int>
void gemv_bfloat16_strided_tail() {
  using MatrixType = Matrix<bfloat16, Dynamic, Dynamic, RowMajor>;
  using VectorType = Matrix<bfloat16, Dynamic, 1>;
  const long page_size = sysconf(_SC_PAGESIZE);
  VERIFY(page_size > 0);
  char* storage =
      static_cast<char*>(mmap(nullptr, 2 * page_size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0));
  VERIFY(storage != MAP_FAILED);
  VERIFY_IS_EQUAL(mprotect(storage + page_size, page_size, PROT_NONE), 0);

  // Put the last result coefficient against an inaccessible page: even one excess gather must fail.
  for (Index rows : {1, 2, 3, 4, 5, 7, 8, 9, 12, 15, 16, 17, 31, 32, 33}) {
    for (Index stride : {1, 2, 3, 17}) {
      const Index span = (rows - 1) * stride + 1;
      VERIFY(span * Index(sizeof(bfloat16)) <= page_size);
      bfloat16* values = reinterpret_cast<bfloat16*>(storage + page_size - span * sizeof(bfloat16));
      for (Index i = 0; i < rows; ++i) new (values + i * stride) bfloat16(1);
      Map<VectorType, 0, InnerStride<Dynamic>> result(values, rows, InnerStride<Dynamic>(stride));
      const MatrixType A = MatrixType::Ones(rows, 65);
      const VectorType x = VectorType::Ones(65);
      result.noalias() += A * x;
      VERIFY_IS_CWISE_EQUAL(result, VectorType::Constant(rows, bfloat16(66)));
    }
  }
  VERIFY_IS_EQUAL(munmap(storage, 2 * page_size), 0);
}
#endif

template <typename T>
void test_aliasing() {
  int rows = internal::random<int>(1, 12);
  int cols = internal::random<int>(1, 12);
  typedef Matrix<T, Dynamic, Dynamic> MatrixType;
  typedef Matrix<T, Dynamic, 1> VectorType;
  VectorType x(cols);
  x.setRandom();
  VectorType z(x);
  VectorType y(rows);
  y.setZero();
  MatrixType A(rows, cols);
  A.setRandom();
  // CwiseBinaryOp
  VERIFY_IS_APPROX(x = y + A * x, A * z);  // OK because "y + A*x" is marked as "assume-aliasing"
  x = z;
  // CwiseUnaryOp
  VERIFY_IS_APPROX(x = T(1.) * (A * x),
                   A * z);  // OK because 1*(A*x) is replaced by (1*A*x) which is a Product<> expression
  x = z;
  // VERIFY_IS_APPROX(x = y-A*x, -A*z);   // Not OK in 3.3 because x is resized before A*x gets evaluated
  x = z;
}

template <int>
void product_large_regressions() {
  {
    // test a specific issue in DiagonalProduct
    int N = 1000000;
    VectorXf v = VectorXf::Ones(N);
    MatrixXf m = MatrixXf::Ones(N, 3);
    m = (v + v).asDiagonal() * m;
    VERIFY_IS_APPROX(m, MatrixXf::Constant(N, 3, 2));
  }

  {
    // test deferred resizing in Matrix::operator=
    MatrixXf a = MatrixXf::Random(10, 4), b = MatrixXf::Random(4, 10), c = a;
    VERIFY_IS_APPROX((a = a * b), (c * b).eval());
  }

  {
    // check the functions to setup blocking sizes compile and do not segfault
    // FIXME check they do what they are supposed to do !!
    std::ptrdiff_t old_l1 = l1CacheSize();
    std::ptrdiff_t old_l2 = l2CacheSize();
    std::ptrdiff_t old_l3 = l3CacheSize();
    std::ptrdiff_t l1 = internal::random<int>(10000, 20000);
    std::ptrdiff_t l2 = internal::random<int>(100000, 200000);
    std::ptrdiff_t l3 = internal::random<int>(1000000, 2000000);
    setCpuCacheSizes(l1, l2, l3);
    VERIFY(l1 == l1CacheSize());
    VERIFY(l2 == l2CacheSize());
    std::ptrdiff_t k1 = internal::random<int>(10, 100) * 16;
    std::ptrdiff_t m1 = internal::random<int>(10, 100) * 16;
    std::ptrdiff_t n1 = internal::random<int>(10, 100) * 16;
    // only makes sure it compiles fine
    internal::computeProductBlockingSizes<float, float, std::ptrdiff_t>(k1, m1, n1, 1);

    // Regression test for small synthetic caches with AVX512 and multiple threads.
    // The L2 model can report that fewer than one nr-wide RHS panel fits.
    setCpuCacheSizes(10000, 100000, 1000000);
    std::ptrdiff_t k2 = 5000;
    std::ptrdiff_t m2 = 5000;
    std::ptrdiff_t n2 = 256;
    internal::computeProductBlockingSizes<double, double, std::ptrdiff_t>(k2, m2, n2, 32);
    VERIFY(k2 > 0);
    VERIFY(m2 > 0);
    VERIFY(n2 > 0);

    // An explicit override has to govern blocking on its own. The detected per-CPU L3 share also
    // feeds the rhs-panel budget, so leaving it set would silently overrule callers that force
    // small cache sizes to exercise multi-pass blocking (contrib/test/tensor_contraction.cpp
    // does exactly that).
    setCpuCacheSizes(896, 1920, 2944);
    std::ptrdiff_t forced_l1, forced_l2, forced_l3, forced_l3_per_cpu = -1;
    internal::manage_caching_sizes(GetAction, &forced_l1, &forced_l2, &forced_l3, &forced_l3_per_cpu);
    VERIFY(forced_l1 == 896 && forced_l2 == 1920 && forced_l3 == 2944);
    VERIFY(forced_l3_per_cpu == 0);

    setCpuCacheSizes(old_l1, old_l2, old_l3);
  }

  {
    // test regression in row-vector by matrix (bad Map type)
    MatrixXf mat1(10, 32);
    mat1.setRandom();
    MatrixXf mat2(32, 32);
    mat2.setRandom();
    MatrixXf r1 = mat1.row(2) * mat2.transpose();
    VERIFY_IS_APPROX(r1, (mat1.row(2) * mat2.transpose()).eval());

    MatrixXf r2 = mat1.row(2) * mat2;
    VERIFY_IS_APPROX(r2, (mat1.row(2) * mat2).eval());
  }

  {
    Eigen::MatrixXd A(10, 10), B, C;
    A.setRandom();
    C = A;
    for (int k = 0; k < 79; ++k) C = C * A;
    B.noalias() =
        (((A * A) * (A * A)) * ((A * A) * (A * A)) * ((A * A) * (A * A)) * ((A * A) * (A * A)) * ((A * A) * (A * A)) *
         ((A * A) * (A * A)) * ((A * A) * (A * A)) * ((A * A) * (A * A)) * ((A * A) * (A * A)) * ((A * A) * (A * A))) *
        (((A * A) * (A * A)) * ((A * A) * (A * A)) * ((A * A) * (A * A)) * ((A * A) * (A * A)) * ((A * A) * (A * A)) *
         ((A * A) * (A * A)) * ((A * A) * (A * A)) * ((A * A) * (A * A)) * ((A * A) * (A * A)) * ((A * A) * (A * A)));
    VERIFY_IS_APPROX(B, C);
  }
}

// Regression test: row-major GEMV with stride*sizeof > 32000 disables the
// 8-row main loop (n8=0). The cleanup must use `for` loops (not `if`) to
// process all remaining rows. Without the fix, only 7 out of `rows` results
// are computed. This manifests as loss of orthogonality in QR of tall-skinny
// matrices, since the Householder application uses row-major GEMV internally.
template <int>
void bug_gemv_rowmajor_large_stride() {
  // Direct GEMV test: row-major A with stride (= cols) triggering n8=0.
  // The threshold is stride * sizeof(Scalar) > 32000.
  // For double: cols > 4000. For float: cols > 8000.
  {
    const int rows = 100;
    const int cols = 5000;  // cols * sizeof(double) = 40000 > 32000
    Matrix<double, Dynamic, Dynamic, RowMajor> A(rows, cols);
    A.setRandom();
    VectorXd x = VectorXd::Random(cols);
    VectorXd y = A * x;
    VectorXd y_ref = VectorXd::Zero(rows);
    for (int i = 0; i < rows; ++i)
      for (int j = 0; j < cols; ++j) y_ref(i) += A(i, j) * x(j);
    VERIFY_IS_APPROX(y, y_ref);
  }

  // QR orthogonality test: this is the high-level symptom.
  // HouseholderQR of a col-major (m x n) matrix with m > 4000
  // uses row-major GEMV internally during Householder application.
  {
    const int m = 5000;
    const int n = 50;
    MatrixXd A = MatrixXd::Random(m, n);
    MatrixXd Q = A.householderQr().householderQ() * MatrixXd::Identity(m, n);
    MatrixXd QtQ = Q.adjoint() * Q;
    VERIFY_IS_APPROX(QtQ, MatrixXd::Identity(n, n));
  }
}

// Regression test for row-major GEMV run_small_cols bug.
// When cols is small (e.g., 2), and loop variables (like n8) are 0 due
// to row or stride limits, the remainder loops previously used `if` checks
// like `if (i < n4)`. This incorrectly skips rows if multiple remainder
// blocks are needed (e.g., 9 rows).
template <int>
void bug_gemv_run_small_cols() {
  const int rows = 9;       // > 8, covers 8-row loop step but tests remainder cleanup
  const int cols = 2;       // triggers run_small_cols (cols < PacketSize)
  const int stride = 5000;  // 5000 * sizeof(double) > 32000, forces n8 = 0

  Matrix<double, Dynamic, Dynamic, RowMajor> A_full(rows, stride);
  A_full.setRandom();
  auto A = A_full.leftCols(cols);

  VectorXd x = VectorXd::Random(cols);
  VectorXd y = A * x;
  VectorXd y_ref = A.eval() * x;  // No stride.

  VERIFY_IS_APPROX(y, y_ref);
}

// Systematic test of row-major GEMV run_small_cols and main run() remainder paths.
// Varies cols from 1-7 (covers float PacketSize=8 and double PacketSize=4 boundaries)
// and rows across values that exercise all n8/n4/n2/n1 remainder combinations.
template <int>
void gemv_small_cols_systematic() {
  const int test_cols[] = {1, 2, 3, 4, 5, 6, 7};
  const int test_rows[] = {1, 2, 3, 4, 5, 7, 8, 9, 11, 13, 15, 16, 17, 25};

  // Large stride forces n8=0, exercising all remainder-only paths.
  {
    const int stride = 5000;  // 5000 * sizeof(double) = 40000 > 32000
    for (int ci = 0; ci < 7; ++ci) {
      for (int ri = 0; ri < 14; ++ri) {
        int rows = test_rows[ri], cols = test_cols[ci];
        Matrix<double, Dynamic, Dynamic, RowMajor> A_full(rows, stride);
        A_full.setRandom();
        auto A = A_full.leftCols(cols);
        VectorXd x = VectorXd::Random(cols);
        VectorXd y = A * x;
        VectorXd y_ref = VectorXd::Zero(rows);
        for (int i = 0; i < rows; ++i)
          for (int j = 0; j < cols; ++j) y_ref(i) += A(i, j) * x(j);
        VERIFY_IS_APPROX(y, y_ref);
      }
    }
  }

  // Normal stride (n8 active) to cover the 8-row main loop + remainders.
  for (int ci = 0; ci < 7; ++ci) {
    for (int ri = 0; ri < 14; ++ri) {
      int rows = test_rows[ri], cols = test_cols[ci];
      Matrix<double, Dynamic, Dynamic, RowMajor> A(rows, cols);
      A.setRandom();
      VectorXd x = VectorXd::Random(cols);
      VectorXd y = A * x;
      VectorXd y_ref = VectorXd::Zero(rows);
      for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j) y_ref(i) += A(i, j) * x(j);
      VERIFY_IS_APPROX(y, y_ref);
    }
  }

  // Float with large stride: 9000 * sizeof(float) = 36000 > 32000
  {
    const int stride = 9000;
    for (int ci = 0; ci < 7; ++ci) {
      for (int ri = 0; ri < 14; ++ri) {
        int rows = test_rows[ri], cols = test_cols[ci];
        Matrix<float, Dynamic, Dynamic, RowMajor> A_full(rows, stride);
        A_full.setRandom();
        auto A = A_full.leftCols(cols);
        VectorXf x = VectorXf::Random(cols);
        VectorXf y = A * x;
        VectorXf y_ref = VectorXf::Zero(rows);
        for (int i = 0; i < rows; ++i)
          for (int j = 0; j < cols; ++j) y_ref(i) += A(i, j) * x(j);
        VERIFY_IS_APPROX(y, y_ref);
      }
    }
  }
}

// Test the main row-major GEMV n8=0 path (not run_small_cols) with varied row counts.
// The n8 threshold is stride*sizeof(Scalar) > 32000.
template <int>
void gemv_rowmajor_large_stride_varied_rows() {
  const int test_rows[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 15, 16, 17, 25, 100};
  // Double: cols=5000 (5000*8 > 32000), enough cols to stay on main run() path.
  {
    const int cols = 5000;
    for (int ri = 0; ri < 16; ++ri) {
      int rows = test_rows[ri];
      Matrix<double, Dynamic, Dynamic, RowMajor> A(rows, cols);
      A.setRandom();
      VectorXd x = VectorXd::Random(cols);
      VectorXd y = A * x;
      VectorXd y_ref = VectorXd::Zero(rows);
      for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j) y_ref(i) += A(i, j) * x(j);
      VERIFY_IS_APPROX(y, y_ref);
    }
  }
  // Float: cols=9000 (9000*4 > 32000).
  {
    const int cols = 9000;
    for (int ri = 0; ri < 16; ++ri) {
      int rows = test_rows[ri];
      Matrix<float, Dynamic, Dynamic, RowMajor> A(rows, cols);
      A.setRandom();
      VectorXf x = VectorXf::Random(cols);
      VectorXf y = A * x;
      VectorXf y_ref = VectorXf::Zero(rows);
      for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j) y_ref(i) += A(i, j) * x(j);
      VERIFY_IS_APPROX(y, y_ref);
    }
  }
}

// Test extreme aspect ratios that exercise GEMV, outer-product, and thin-GEMM dispatch.
template <int>
void product_extreme_aspect_ratios() {
  const int sizes[] = {1, 2, 3, 4, 8, 16, 48, 64, 128};
  for (int si = 0; si < 9; ++si) {
    int s = sizes[si];
    for (int ki = 0; ki < 9; ++ki) {
      int k = sizes[ki];
      // Thin result: s x k * k x 2  (2-column GEMM)
      {
        MatrixXd A = MatrixXd::Random(s, k);
        MatrixXd B = MatrixXd::Random(k, 2);
        MatrixXd C = A * B;
        MatrixXd Cref = MatrixXd::Zero(s, 2);
        for (int i = 0; i < s; ++i)
          for (int j = 0; j < 2; ++j)
            for (int kk = 0; kk < k; ++kk) Cref(i, j) += A(i, kk) * B(kk, j);
        VERIFY_IS_APPROX(C, Cref);
      }
      // Wide result: 2 x k * k x s  (2-row GEMM)
      {
        MatrixXd A = MatrixXd::Random(2, k);
        MatrixXd B = MatrixXd::Random(k, s);
        MatrixXd C = A * B;
        MatrixXd Cref = MatrixXd::Zero(2, s);
        for (int i = 0; i < 2; ++i)
          for (int j = 0; j < s; ++j)
            for (int kk = 0; kk < k; ++kk) Cref(i, j) += A(i, kk) * B(kk, j);
        VERIFY_IS_APPROX(C, Cref);
      }
      // GEMV: s x k * k x 1
      {
        MatrixXd A = MatrixXd::Random(s, k);
        VectorXd x = VectorXd::Random(k);
        VectorXd y = A * x;
        VectorXd yref = VectorXd::Zero(s);
        for (int i = 0; i < s; ++i)
          for (int kk = 0; kk < k; ++kk) yref(i) += A(i, kk) * x(kk);
        VERIFY_IS_APPROX(y, yref);
      }
      // Vec-mat: 1 x k * k x s
      {
        RowVectorXd v = RowVectorXd::Random(k);
        MatrixXd B = MatrixXd::Random(k, s);
        RowVectorXd r = v * B;
        RowVectorXd rref = RowVectorXd::Zero(s);
        for (int j = 0; j < s; ++j)
          for (int kk = 0; kk < k; ++kk) rref(j) += v(kk) * B(kk, j);
        VERIFY_IS_APPROX(r, rref);
      }
    }
  }
}

template <int>
void bug_1622() {
  typedef Matrix<double, 2, -1, 0, 2, -1> Mat2X;
  Mat2X x(2, 2);
  x.setRandom();
  MatrixXd y(2, 2);
  y.setRandom();
  const Mat2X K1 = x * y.inverse();
  const Matrix2d K2 = x * y.inverse();
  VERIFY_IS_APPROX(K1, K2);
}

EIGEN_DECLARE_TEST(product_large) {
#if (defined(EIGEN_VECTORIZE_ALTIVEC) || defined(EIGEN_VECTORIZE_VSX)) && defined(__unix__)
  CALL_SUBTEST_11(gemv_bfloat16_strided_tail<0>());
#endif
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(product(
        MatrixXf(internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_2(product(
        MatrixXd(internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_2(product(MatrixXd(internal::random<int>(1, 10), internal::random<int>(1, 10))));

    CALL_SUBTEST_3(product(
        MatrixXi(internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_4(product(MatrixXcf(internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2),
                                     internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2))));
    CALL_SUBTEST_5(product(Matrix<float, Dynamic, Dynamic, RowMajor>(internal::random<int>(1, EIGEN_TEST_MAX_SIZE),
                                                                     internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));

    CALL_SUBTEST_1(test_aliasing<float>());

    CALL_SUBTEST_6(bug_1622<1>());

    CALL_SUBTEST_7(product(MatrixXcd(internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2),
                                     internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2))));
    CALL_SUBTEST_8(product(Matrix<double, Dynamic, Dynamic, RowMajor>(internal::random<int>(1, EIGEN_TEST_MAX_SIZE),
                                                                      internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_9(product(Matrix<std::complex<float>, Dynamic, Dynamic, RowMajor>(
        internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_10(product(Matrix<std::complex<double>, Dynamic, Dynamic, RowMajor>(
        internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_11(product(Matrix<bfloat16, Dynamic, Dynamic, RowMajor>(
        internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_12(product(Matrix<Eigen::half, Dynamic, Dynamic>(internal::random<int>(1, EIGEN_TEST_MAX_SIZE),
                                                                  internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
  }

  CALL_SUBTEST_6(product_large_regressions<0>());
  CALL_SUBTEST_6(bug_gemv_rowmajor_large_stride<0>());
  CALL_SUBTEST_6(bug_gemv_run_small_cols<0>());
  CALL_SUBTEST_6(gemv_small_cols_systematic<0>());
  CALL_SUBTEST_6(gemv_rowmajor_large_stride_varied_rows<0>());
  CALL_SUBTEST_6(product_extreme_aspect_ratios<0>());

  // Regression test for bug 714:
#if defined EIGEN_HAS_OPENMP
  omp_set_dynamic(1);
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_6(product(Matrix<float, Dynamic, Dynamic>(internal::random<int>(1, EIGEN_TEST_MAX_SIZE),
                                                           internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
  }
#endif
}
