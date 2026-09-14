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

// regression test for bug 447
template <int>
void product1x1() {
  Matrix<float, 1, 3> matAstatic;
  Matrix<float, 3, 1> matBstatic;
  matAstatic.setRandom();
  matBstatic.setRandom();
  VERIFY_IS_APPROX((matAstatic * matBstatic).coeff(0, 0), matAstatic.cwiseProduct(matBstatic.transpose()).sum());

  MatrixXf matAdynamic(1, 3);
  MatrixXf matBdynamic(3, 1);
  matAdynamic.setRandom();
  matBdynamic.setRandom();
  VERIFY_IS_APPROX((matAdynamic * matBdynamic).coeff(0, 0), matAdynamic.cwiseProduct(matBdynamic.transpose()).sum());
}

template <typename TC, typename TA, typename TB>
const TC &ref_prod(TC &C, const TA &A, const TB &B) {
  for (Index i = 0; i < C.rows(); ++i)
    for (Index j = 0; j < C.cols(); ++j)
      for (Index k = 0; k < A.cols(); ++k) C.coeffRef(i, j) += A.coeff(i, k) * B.coeff(k, j);
  return C;
}

// The sweeps below compare an optimized product against ref_prod() with verifyProduct() rather than
// VERIFY_IS_APPROX. Both sides accumulate k terms in the working precision, so each carries rounding
// proportional to the magnitude of the summands, i.e. to |A|*|B| -- not to the result. Measuring the
// error relative to the result, as VERIFY_IS_APPROX does, is meaningless once the sum cancels: at
// bfloat16 precision a 10-term dot product can lose every significant digit, and the apparent
// relative error then swings by several times the tolerance from one seed to the next while the
// underlying discrepancy stays around one epsilon of |A|*|B|.

template <typename T, int Rows, int Cols, int Depth, int OC, int OA, int OB>
std::enable_if_t<!((Rows == 1 && Depth != 1 && OA == ColMajor) || (Depth == 1 && Rows != 1 && OA == RowMajor) ||
                   (Cols == 1 && Depth != 1 && OB == RowMajor) || (Depth == 1 && Cols != 1 && OB == ColMajor) ||
                   (Rows == 1 && Cols != 1 && OC == ColMajor) || (Cols == 1 && Rows != 1 && OC == RowMajor)),
                 void>
test_lazy_single(int rows, int cols, int depth) {
  Matrix<T, Rows, Depth, OA> A(rows, depth);
  A.setRandom();
  Matrix<T, Depth, Cols, OB> B(depth, cols);
  B.setRandom();
  Matrix<T, Rows, Cols, OC> C(rows, cols);
  C.setRandom();
  Matrix<T, Rows, Cols, OC> D(C);
  VERIFY_IS_APPROX(C += A.lazyProduct(B), ref_prod(D, A, B));
}

void test_dynamic_bool() {
  int rows = internal::random<int>(1, 64);
  int cols = internal::random<int>(1, 64);
  int depth = internal::random<int>(1, 65);

  typedef Matrix<bool, Dynamic, Dynamic> MatrixX;
  MatrixX A(rows, depth);
  A.setRandom();
  MatrixX B(depth, cols);
  B.setRandom();
  MatrixX C(rows, cols);
  C.setRandom();
  MatrixX D(C);
  for (Index i = 0; i < C.rows(); ++i)
    for (Index j = 0; j < C.cols(); ++j)
      for (Index k = 0; k < A.cols(); ++k) D.coeffRef(i, j) |= (A.coeff(i, k) && B.coeff(k, j));
  C += A * B;
  VERIFY_IS_EQUAL(C, D);

  MatrixX E = B.transpose();
  for (Index i = 0; i < B.rows(); ++i)
    for (Index j = 0; j < B.cols(); ++j) VERIFY_IS_EQUAL(B(i, j), E(j, i));
}

template <typename T, int Rows, int Cols, int Depth, int OC, int OA, int OB>
std::enable_if_t<((Rows == 1 && Depth != 1 && OA == ColMajor) || (Depth == 1 && Rows != 1 && OA == RowMajor) ||
                  (Cols == 1 && Depth != 1 && OB == RowMajor) || (Depth == 1 && Cols != 1 && OB == ColMajor) ||
                  (Rows == 1 && Cols != 1 && OC == ColMajor) || (Cols == 1 && Rows != 1 && OC == RowMajor)),
                 void>
test_lazy_single(int, int, int) {}

template <typename T, int Rows, int Cols, int Depth>
void test_lazy_all_layout(int rows = Rows, int cols = Cols, int depth = Depth) {
  CALL_SUBTEST((test_lazy_single<T, Rows, Cols, Depth, ColMajor, ColMajor, ColMajor>(rows, cols, depth)));
  CALL_SUBTEST((test_lazy_single<T, Rows, Cols, Depth, RowMajor, ColMajor, ColMajor>(rows, cols, depth)));
  CALL_SUBTEST((test_lazy_single<T, Rows, Cols, Depth, ColMajor, RowMajor, ColMajor>(rows, cols, depth)));
  CALL_SUBTEST((test_lazy_single<T, Rows, Cols, Depth, RowMajor, RowMajor, ColMajor>(rows, cols, depth)));
  CALL_SUBTEST((test_lazy_single<T, Rows, Cols, Depth, ColMajor, ColMajor, RowMajor>(rows, cols, depth)));
  CALL_SUBTEST((test_lazy_single<T, Rows, Cols, Depth, RowMajor, ColMajor, RowMajor>(rows, cols, depth)));
  CALL_SUBTEST((test_lazy_single<T, Rows, Cols, Depth, ColMajor, RowMajor, RowMajor>(rows, cols, depth)));
  CALL_SUBTEST((test_lazy_single<T, Rows, Cols, Depth, RowMajor, RowMajor, RowMajor>(rows, cols, depth)));
}

template <typename T>
void test_lazy_l1() {
  int rows = internal::random<int>(1, 12);
  int cols = internal::random<int>(1, 12);
  int depth = internal::random<int>(1, 12);

  // Inner
  CALL_SUBTEST((test_lazy_all_layout<T, 1, 1, 1>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 1, 1, 2>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 1, 1, 3>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 1, 1, 8>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 1, 1, 9>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 1, 1, -1>(1, 1, depth)));

  // Outer
  CALL_SUBTEST((test_lazy_all_layout<T, 2, 1, 1>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 1, 2, 1>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 2, 2, 1>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 3, 3, 1>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 4, 4, 1>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 4, 8, 1>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 4, -1, 1>(4, cols)));
  CALL_SUBTEST((test_lazy_all_layout<T, 7, -1, 1>(7, cols)));
  CALL_SUBTEST((test_lazy_all_layout<T, -1, 8, 1>(rows)));
  CALL_SUBTEST((test_lazy_all_layout<T, -1, 3, 1>(rows)));
  CALL_SUBTEST((test_lazy_all_layout<T, -1, -1, 1>(rows, cols)));
}

template <typename T>
void test_lazy_l2() {
  int rows = internal::random<int>(1, 12);
  int cols = internal::random<int>(1, 12);
  int depth = internal::random<int>(1, 12);

  // mat-vec
  CALL_SUBTEST((test_lazy_all_layout<T, 2, 1, 2>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 2, 1, 4>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 4, 1, 2>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 4, 1, 4>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 5, 1, 4>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 4, 1, 5>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 4, 1, 6>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 6, 1, 4>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 8, 1, 8>()));
  CALL_SUBTEST((test_lazy_all_layout<T, -1, 1, 4>(rows)));
  CALL_SUBTEST((test_lazy_all_layout<T, 4, 1, -1>(4, 1, depth)));
  CALL_SUBTEST((test_lazy_all_layout<T, -1, 1, -1>(rows, 1, depth)));

  // vec-mat
  CALL_SUBTEST((test_lazy_all_layout<T, 1, 2, 2>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 1, 2, 4>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 1, 4, 2>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 1, 4, 4>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 1, 5, 4>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 1, 4, 5>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 1, 4, 6>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 1, 6, 4>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 1, 8, 8>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 1, -1, 4>(1, cols)));
  CALL_SUBTEST((test_lazy_all_layout<T, 1, 4, -1>(1, 4, depth)));
  CALL_SUBTEST((test_lazy_all_layout<T, 1, -1, -1>(1, cols, depth)));
}

template <typename T>
void test_lazy_l3() {
  int rows = internal::random<int>(1, 12);
  int cols = internal::random<int>(1, 12);
  int depth = internal::random<int>(1, 12);
  // mat-mat
  CALL_SUBTEST((test_lazy_all_layout<T, 2, 4, 2>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 2, 6, 4>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 4, 3, 2>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 4, 8, 4>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 5, 6, 4>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 4, 2, 5>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 4, 7, 6>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 6, 8, 4>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 8, 3, 8>()));
  CALL_SUBTEST((test_lazy_all_layout<T, -1, 6, 4>(rows)));
  CALL_SUBTEST((test_lazy_all_layout<T, 4, 3, -1>(4, 3, depth)));
  CALL_SUBTEST((test_lazy_all_layout<T, -1, 6, -1>(rows, 6, depth)));
  CALL_SUBTEST((test_lazy_all_layout<T, 8, 2, 2>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 5, 2, 4>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 4, 4, 2>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 8, 4, 4>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 6, 5, 4>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 4, 4, 5>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 3, 4, 6>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 2, 6, 4>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 7, 8, 8>()));
  CALL_SUBTEST((test_lazy_all_layout<T, 8, -1, 4>(8, cols)));
  CALL_SUBTEST((test_lazy_all_layout<T, 3, 4, -1>(3, 4, depth)));
  CALL_SUBTEST((test_lazy_all_layout<T, 4, -1, -1>(4, cols, depth)));
}

template <typename T, int N, int M, int K>
void test_linear_but_not_vectorizable() {
  // Check tricky cases for which the result of the product is a vector and thus must exhibit the LinearBit flag,
  // but is not vectorizable along the linear dimension.
  Index n = N == Dynamic ? internal::random<Index>(1, 32) : N;
  Index m = M == Dynamic ? internal::random<Index>(1, 32) : M;
  Index k = K == Dynamic ? internal::random<Index>(1, 32) : K;

  {
    Matrix<T, N, M + 1> A;
    A.setRandom(n, m + 1);
    Matrix<T, M * 2, K> B;
    B.setRandom(m * 2, k);
    Matrix<T, 1, K> C;
    Matrix<T, 1, K> R;

    C.noalias() = A.template topLeftCorner<1, M>() * (B.template topRows<M>() + B.template bottomRows<M>());
    R.noalias() = A.template topLeftCorner<1, M>() * (B.template topRows<M>() + B.template bottomRows<M>()).eval();
    VERIFY_IS_APPROX(C, R);
  }

  {
    Matrix<T, M + 1, N, RowMajor> A;
    A.setRandom(m + 1, n);
    Matrix<T, K, M * 2, RowMajor> B;
    B.setRandom(k, m * 2);
    Matrix<T, K, 1> C;
    Matrix<T, K, 1> R;

    C.noalias() = (B.template leftCols<M>() + B.template rightCols<M>()) * A.template topLeftCorner<M, 1>();
    R.noalias() = (B.template leftCols<M>() + B.template rightCols<M>()).eval() * A.template topLeftCorner<M, 1>();
    VERIFY_IS_APPROX(C, R);
  }
}

template <int Rows>
void bug_1311() {
  Matrix<double, Rows, 2> A;
  A.setRandom();
  Vector2d b = Vector2d::Random();
  Matrix<double, Rows, 1> res;
  res.noalias() = 1. * (A * b);
  VERIFY_IS_APPROX(res, A * b);
  res.noalias() = 1. * A * b;
  VERIFY_IS_APPROX(res, A * b);
  res.noalias() = (1. * A).lazyProduct(b);
  VERIFY_IS_APPROX(res, A * b);
  res.noalias() = (1. * A).lazyProduct(1. * b);
  VERIFY_IS_APPROX(res, A * b);
  res.noalias() = (A).lazyProduct(1. * b);
  VERIFY_IS_APPROX(res, A * b);
}

template <int>
void product_small_regressions() {
  {
    // test compilation of (outer_product) * vector
    Vector3f v = Vector3f::Random();
    VERIFY_IS_APPROX((v * v.transpose()) * v, (v * v.transpose()).eval() * v);
  }

  {
    // regression test for pull-request #93
    Eigen::Matrix<double, 1, 1> A;
    A.setRandom();
    Eigen::Matrix<double, 18, 1> B;
    B.setRandom();
    Eigen::Matrix<double, 1, 18> C;
    C.setRandom();
    VERIFY_IS_APPROX(B * A.inverse(), B * A.inverse()[0]);
    VERIFY_IS_APPROX(A.inverse() * C, A.inverse()[0] * C);
  }

  {
    Eigen::Matrix<double, 10, 10> A, B, C;
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

  {
    // A square fixed-size product takes the coeff-based path while
    // Rows + Cols + Depth stays below the threshold, so the boundary is derived
    // rather than hard-coded. It is read back off product_type itself, not off
    // the macro: the effective threshold is per scalar pair (SME raises it only
    // for the pairs its kernel claims), so a build where double falls back to
    // the generic kernel would otherwise disagree with whichever value the macro
    // holds -- silently, since no SME job builds this test.
    typedef Eigen::Matrix<double, 10, 10> Matrix10;
    constexpr int kGemm = (internal::product_type<Matrix10, Matrix10>::FixedSizeThreshold + 2) / 3;
    typedef Eigen::Matrix<double, kGemm - 1, kGemm - 1> MatrixBelow;
    typedef Eigen::Matrix<double, kGemm, kGemm> MatrixAt;
    VERIFY((internal::product_type<Matrix10, Matrix10>::value == CoeffBasedProductMode));
    VERIFY((internal::product_type<MatrixBelow, MatrixBelow>::value == CoeffBasedProductMode));
    VERIFY((internal::product_type<MatrixAt, MatrixAt>::value == GemmProduct));

    Matrix10 A = Matrix10::Random();
    Matrix10 B = Matrix10::Random();
    Matrix10 C = Matrix10::Random();
    Matrix10 ref = C;

    C = A * B;
    ref = A.lazyProduct(B);
    VERIFY_IS_APPROX(C, ref);

    C.setRandom();
    ref = C;
    C.noalias() += A * B;
    ref.noalias() += A.lazyProduct(B);
    VERIFY_IS_APPROX(C, ref);

    C.setRandom();
    ref = C;
    C.noalias() -= A * B;
    ref.noalias() -= A.lazyProduct(B);
    VERIFY_IS_APPROX(C, ref);

    C.setRandom();
    ref = C;
    C.noalias() += (2.0 * A) * (B * 3.0);
    ref.noalias() += 6.0 * A.lazyProduct(B);
    VERIFY_IS_APPROX(C, ref);
  }
}

// Test products at sizes near critical code-path transitions:
//  - EIGEN_CACHEFRIENDLY_PRODUCT_THRESHOLD = 8 (coeff-based vs GEBP)
//  - Blocking early-return at max(k,m,n) < 48
// Uses a sparse set of sizes so total count is 14^3 = 2744 (fast).
template <typename T>
void product_transition_sizes() {
  using Matrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
  const int critical[] = {1, 2, 4, 7, 8, 9, 12, 16, 24, 32, 47, 48, 49, 64};
  for (int im = 0; im < 14; ++im) {
    for (int in = 0; in < 14; ++in) {
      Matrix C = Matrix::Zero(critical[im], critical[in]);
      Matrix Cref = Matrix::Zero(critical[im], critical[in]);
      for (int ik = 0; ik < 14; ++ik) {
        int m = critical[im], n = critical[in], k = critical[ik];
        Matrix A = Matrix::Random(m, k);
        Matrix B = Matrix::Random(k, n);
        C = A * B;
        Cref.setZero();
        ref_prod(Cref, A, B);
        VERIFY(verifyProduct(C, Cref, A, B));
      }
    }
  }
}

template <typename T>
void product_sweep(int max_m, int max_k, int max_n) {
  using Matrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
  for (int m = 1; m < max_m; ++m) {
    for (int n = 1; n < max_n; ++n) {
      Matrix C = Matrix::Zero(m, n);
      Matrix Cref = Matrix::Zero(m, n);
      for (int k = 1; k < max_k; ++k) {
        Matrix A = Matrix::Random(m, k);
        Matrix B = Matrix::Random(k, n);
        C = A * B;
        Cref.setZero();
        ref_prod(Cref, A, B);
        VERIFY(verifyProduct(C, Cref, A, B));
      }
    }
  }
}

// The bound for S * (S * S)^T is |S| * (|S| * |S|)^T, not |S|^3: transposition keeps every entry's
// magnitude but reorders the chain. For the nilpotent S below the two differ in kind — |S|^3 is
// identically zero while the two evaluation orders still disagree by a rounding error.
template <int>
void product_transposed_triple_bound() {
  Matrix3d square;
  square << 0, 0.3, 0, 0, 0, 0.7, 0, 0, 0;
  const Matrix3d abs_sq = square.cwiseAbs();
  const Matrix3d abs_sq2 = abs_sq * abs_sq;
  VERIFY((abs_sq2 * abs_sq).isZero(0));
  VERIFY(product_error_bound(abs_sq, abs_sq2.transpose(), 4) > 0);
  VERIFY(verifyProduct(square * (square * square).transpose(), square * square.transpose() * square.transpose(), abs_sq,
                       abs_sq2.transpose(), 4));
}

// verifyProduct() must not accept an overflowed result. Narrowed to half, the triple-product bound
// for all-one 100x100 operands is already infinite, and inf <= inf would then pass any result.
template <int>
void product_verify_rejects_nonfinite() {
  using MatrixXh = Matrix<half, Dynamic, Dynamic>;
  const Index n = 100;
  const MatrixXh abs_a = MatrixXh::Constant(n, n, half(float(n)));
  const MatrixXh abs_b = MatrixXh::Constant(n, n, half(1.0f));
  const double bound = product_error_bound(abs_a, abs_b, 4);
  VERIFY((numext::isfinite)(bound));
  VERIFY(bound > static_cast<double>(NumTraits<half>::highest()));

  const MatrixXh zero = MatrixXh::Zero(n, n);
  const MatrixXh inf = MatrixXh::Constant(n, n, half(std::numeric_limits<float>::infinity()));
  const MatrixXh nan = MatrixXh::Constant(n, n, half(std::numeric_limits<float>::quiet_NaN()));
  VERIFY(verifyProduct(zero, zero, abs_a, abs_b, 4));
  VERIFY(!verifyProduct(inf, zero, abs_a, abs_b, 4));
  VERIFY(!verifyProduct(nan, zero, abs_a, abs_b, 4));
}

EIGEN_DECLARE_TEST(product_small) {
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(product(Matrix<float, 3, 2>()));
    CALL_SUBTEST_2(product(Matrix<int, 3, 17>()));
    CALL_SUBTEST_8(product(Matrix<double, 3, 17>()));
    CALL_SUBTEST_3(product(Matrix3d()));
    CALL_SUBTEST_4(product(Matrix4d()));
    CALL_SUBTEST_5(product(Matrix4f()));
    CALL_SUBTEST_10(product(Matrix<bfloat16, 3, 2>()));
    CALL_SUBTEST_6(product1x1<0>());

    CALL_SUBTEST_11(test_lazy_l1<float>());
    CALL_SUBTEST_12(test_lazy_l2<float>());
    CALL_SUBTEST_13(test_lazy_l3<float>());

    CALL_SUBTEST_21(test_lazy_l1<double>());
    CALL_SUBTEST_22(test_lazy_l2<double>());
    CALL_SUBTEST_23(test_lazy_l3<double>());

    CALL_SUBTEST_31(test_lazy_l1<std::complex<float> >());
    CALL_SUBTEST_32(test_lazy_l2<std::complex<float> >());
    CALL_SUBTEST_33(test_lazy_l3<std::complex<float> >());

    CALL_SUBTEST_41(test_lazy_l1<std::complex<double> >());
    CALL_SUBTEST_42(test_lazy_l2<std::complex<double> >());
    CALL_SUBTEST_43(test_lazy_l3<std::complex<double> >());

    CALL_SUBTEST_7((test_linear_but_not_vectorizable<float, 2, 1, Dynamic>()));
    CALL_SUBTEST_7((test_linear_but_not_vectorizable<float, 3, 1, Dynamic>()));
    CALL_SUBTEST_7((test_linear_but_not_vectorizable<float, 2, 1, 16>()));

    CALL_SUBTEST_6(bug_1311<3>());
    CALL_SUBTEST_6(bug_1311<5>());

    CALL_SUBTEST_9(test_dynamic_bool());

    // Commonly specialized vectorized types.
    CALL_SUBTEST_50(product_sweep<float>(10, 10, 10));
    CALL_SUBTEST_51(product_sweep<double>(10, 10, 10));
    CALL_SUBTEST_52(product_sweep<Eigen::half>(10, 10, 10));
    CALL_SUBTEST_53(product_sweep<Eigen::bfloat16>(10, 10, 10));
  }

  CALL_SUBTEST_6(product_small_regressions<0>());
  CALL_SUBTEST_6(product_transposed_triple_bound<0>());
  CALL_SUBTEST_6(product_verify_rejects_nonfinite<0>());

  // Deterministic sweep at transition boundaries (outside g_repeat).
  CALL_SUBTEST_54(product_transition_sizes<float>());
  CALL_SUBTEST_55(product_transition_sizes<double>());
}
