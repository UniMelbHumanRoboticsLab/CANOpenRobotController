// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2009 Benoit Jacob <jacob.benoit.1@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "main.h"
#include <Eigen/LU>
#include <Eigen/QR>
#include "solverbase.h"
using namespace std;

template <typename MatrixType>
typename MatrixType::RealScalar matrix_l1_norm(const MatrixType& m) {
  return m.cwiseAbs().colwise().sum().maxCoeff();
}

template <typename MatrixType>
void lu_non_invertible() {
  typedef typename MatrixType::RealScalar RealScalar;
  /* this test covers the following files:
     LU.h
  */
  Index rows, cols, cols2;
  if (MatrixType::RowsAtCompileTime == Dynamic) {
    rows = internal::random<Index>(2, EIGEN_TEST_MAX_SIZE);
  } else {
    rows = MatrixType::RowsAtCompileTime;
  }
  if (MatrixType::ColsAtCompileTime == Dynamic) {
    cols = internal::random<Index>(2, EIGEN_TEST_MAX_SIZE);
    cols2 = internal::random<int>(2, EIGEN_TEST_MAX_SIZE);
  } else {
    cols2 = cols = MatrixType::ColsAtCompileTime;
  }

  enum { RowsAtCompileTime = MatrixType::RowsAtCompileTime, ColsAtCompileTime = MatrixType::ColsAtCompileTime };
  typedef typename internal::kernel_retval_base<FullPivLU<MatrixType> >::ReturnType KernelMatrixType;
  typedef typename internal::image_retval_base<FullPivLU<MatrixType> >::ReturnType ImageMatrixType;
  typedef Matrix<typename MatrixType::Scalar, ColsAtCompileTime, ColsAtCompileTime> CMatrixType;
  typedef Matrix<typename MatrixType::Scalar, RowsAtCompileTime, RowsAtCompileTime> RMatrixType;

  Index rank = internal::random<Index>(1, (std::min)(rows, cols) - 1);

  // The image of the zero matrix should consist of a single (zero) column vector
  VERIFY((MatrixType::Zero(rows, cols).fullPivLu().image(MatrixType::Zero(rows, cols)).cols() == 1));

  // The kernel of the zero matrix is the entire space, and thus is an invertible matrix of dimensions cols.
  KernelMatrixType kernel = MatrixType::Zero(rows, cols).fullPivLu().kernel();
  VERIFY((kernel.fullPivLu().isInvertible()));

  MatrixType m1(rows, cols), m3(rows, cols2);
  CMatrixType m2(cols, cols2);
  createRandomPIMatrixOfRank(rank, rows, cols, m1);

  FullPivLU<MatrixType> lu;

  // The special value 0.01 below works well in tests. Keep in mind that we're only computing the rank
  // of singular values are either 0 or 1.
  // So it's not clear at all that the epsilon should play any role there.
  lu.setThreshold(RealScalar(0.01));
  lu.compute(m1);

  MatrixType u(rows, cols);
  u = lu.matrixLU().template triangularView<Upper>();
  RMatrixType l = RMatrixType::Identity(rows, rows);
  l.block(0, 0, rows, (std::min)(rows, cols)).template triangularView<StrictlyLower>() =
      lu.matrixLU().block(0, 0, rows, (std::min)(rows, cols));

  VERIFY_IS_APPROX(lu.permutationP() * m1 * lu.permutationQ(), l * u);

  KernelMatrixType m1kernel = lu.kernel();
  ImageMatrixType m1image = lu.image(m1);

  VERIFY_IS_APPROX(m1, lu.reconstructedMatrix());
  VERIFY(rank == lu.rank());
  VERIFY(cols - lu.rank() == lu.dimensionOfKernel());
  VERIFY(!lu.isInjective());
  VERIFY(!lu.isInvertible());
  VERIFY(!lu.isSurjective());
  VERIFY_IS_MUCH_SMALLER_THAN((m1 * m1kernel), m1);
  VERIFY(m1image.fullPivLu().rank() == rank);
  VERIFY_IS_APPROX(m1 * m1.adjoint() * m1image, m1image);

  check_solverbase<CMatrixType, MatrixType>(m1, lu, rows, cols, cols2);

  m2 = CMatrixType::Random(cols, cols2);
  m3 = m1 * m2;
  m2 = CMatrixType::Random(cols, cols2);
  // test that the code, which does resize(), may be applied to an xpr
  m2.block(0, 0, m2.rows(), m2.cols()) = lu.solve(m3);
  VERIFY_IS_APPROX(m3, m1 * m2);
}

template <typename MatrixType>
void lu_invertible() {
  /* this test covers the following files:
     FullPivLU.h
  */
  typedef typename NumTraits<typename MatrixType::Scalar>::Real RealScalar;
  Index size = MatrixType::RowsAtCompileTime;
  if (size == Dynamic) size = internal::random<Index>(1, EIGEN_TEST_MAX_SIZE);

  MatrixType m1(size, size), m2(size, size), m3(size, size);
  FullPivLU<MatrixType> lu;
  lu.setThreshold(RealScalar(0.01));
  // Create a random diagonally dominant (thus invertible) matrix.
  m1 = MatrixType::Random(size, size);
  m1.diagonal().array() += RealScalar(2 * size);
  lu.compute(m1);

  VERIFY_IS_APPROX(m1, lu.reconstructedMatrix());
  VERIFY(0 == lu.dimensionOfKernel());
  VERIFY(lu.kernel().cols() == 1);  // the kernel() should consist of a single (zero) column vector
  VERIFY(size == lu.rank());
  VERIFY(lu.isInjective());
  VERIFY(lu.isSurjective());
  VERIFY(lu.isInvertible());
  VERIFY(lu.image(m1).fullPivLu().isInvertible());

  check_solverbase<MatrixType, MatrixType>(m1, lu, size, size, size);

  MatrixType m1_inverse = lu.inverse();
  m3 = MatrixType::Random(size, size);
  m2 = lu.solve(m3);
  VERIFY_IS_APPROX(m2, m1_inverse * m3);

  RealScalar rcond = (RealScalar(1) / matrix_l1_norm(m1)) / matrix_l1_norm(m1_inverse);
  const RealScalar rcond_est = lu.rcond();
  // Verify that the estimated condition number is within a factor of 10 of the
  // truth.
  VERIFY(rcond_est > rcond / 10 && rcond_est < rcond * 10);

  // Regression test for Bug 302
  MatrixType m4 = MatrixType::Random(size, size);
  VERIFY_IS_APPROX(lu.solve(m3 * m4), lu.solve(m3) * m4);
}

template <typename MatrixType>
void lu_partial_piv(Index size = MatrixType::ColsAtCompileTime) {
  /* this test covers the following files:
     PartialPivLU.h
  */
  typedef typename NumTraits<typename MatrixType::Scalar>::Real RealScalar;

  MatrixType m1(size, size), m2(size, size), m3(size, size);
  m1.setRandom();
  PartialPivLU<MatrixType> plu(m1);

  VERIFY_IS_APPROX(m1, plu.reconstructedMatrix());

  check_solverbase<MatrixType, MatrixType>(m1, plu, size, size, size);

  MatrixType m1_inverse = plu.inverse();
  m3 = MatrixType::Random(size, size);
  m2 = plu.solve(m3);
  VERIFY_IS_APPROX(m2, m1_inverse * m3);

  RealScalar rcond = (RealScalar(1) / matrix_l1_norm(m1)) / matrix_l1_norm(m1_inverse);
  const RealScalar rcond_est = plu.rcond();
  // Verify that the estimate is within a factor of 10 of the truth.
  VERIFY(rcond_est > rcond / 10 && rcond_est < rcond * 10);
}

// Regression test: FullPivLU took the maximum of an empty column-sum vector when computing its l1 norm,
// so it could not be constructed at all from a matrix with zero columns. PartialPivLU already guarded the
// same reduction. The determinant of an empty matrix is the empty product, 1.
template <typename MatrixType>
void lu_empty() {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;
  const Index n = 5;

  FullPivLU<MatrixType> lu{MatrixType(0, 0)};
  VERIFY_IS_EQUAL(lu.rank(), Index(0));
  VERIFY(lu.isInvertible());
  VERIFY_IS_EQUAL(lu.determinant(), Scalar(1));
  VERIFY_IS_EQUAL(lu.absDeterminant(), RealScalar(1));
  VERIFY_IS_EQUAL(lu.logAbsDeterminant(), RealScalar(0));
  VERIFY_IS_EQUAL(lu.signDeterminant(), Scalar(1));

  lu.compute(MatrixType(0, n));
  VERIFY_IS_EQUAL(lu.rank(), Index(0));
  VERIFY_IS_EQUAL(lu.dimensionOfKernel(), n);
  VERIFY(!lu.isInjective());

  lu.compute(MatrixType(n, 0));
  VERIFY_IS_EQUAL(lu.rank(), Index(0));
  VERIFY_IS_EQUAL(lu.dimensionOfKernel(), Index(0));
  VERIFY(!lu.isSurjective());

  PartialPivLU<MatrixType> plu{MatrixType(0, 0)};
  VERIFY_IS_EQUAL(plu.determinant(), Scalar(1));
  VERIFY_IS_EQUAL(plu.absDeterminant(), RealScalar(1));
  VERIFY_IS_EQUAL(plu.logAbsDeterminant(), RealScalar(0));
  VERIFY_IS_EQUAL(plu.signDeterminant(), Scalar(1));
}

// A = Q D Q^*, with Q unitary, has det(A) = det(D) because det(Q) det(Q^*) = |det(Q)|^2 = 1.
template <typename MatrixType>
void lu_determinant(Index size) {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;

  MatrixType d = MatrixType::Zero(size, size);
  setRandomWellConditionedDiagonal(d);
  const MatrixType q = MatrixType::Random(size, size).householderQr().householderQ();
  const MatrixType a = q * d * q.adjoint();

  const Scalar det = d.diagonal().prod();
  const RealScalar logabsdet = d.diagonal().cwiseAbs().array().log().sum();

  check_determinant(FullPivLU<MatrixType>(a), det, logabsdet);
  check_determinant(PartialPivLU<MatrixType>(a), det, logabsdet);
}

// logAbsDeterminant() exists to survive the range where the determinant itself does not: with n = 200 and
// a diagonal of 10^4, det = 10^800 overflows every supported float type while log|det| = 800 log 10 does not.
template <typename MatrixType>
void lu_determinant_overflow() {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;

  const Index size = 200;
  for (bool overflow : {true, false}) {
    const RealScalar scale = overflow ? RealScalar(1e4) : RealScalar(1e-4);
    const MatrixType a = MatrixType::Identity(size, size) * Scalar(scale);
    const RealScalar logabsdet = RealScalar(size) * numext::log(scale);

    PartialPivLU<MatrixType> plu(a);
    VERIFY(determinant_out_of_range(plu.absDeterminant(), overflow));
    VERIFY_IS_APPROX(plu.logAbsDeterminant(), logabsdet);
    VERIFY_IS_EQUAL(plu.signDeterminant(), Scalar(1));

    FullPivLU<MatrixType> lu(a);
    VERIFY(determinant_out_of_range(lu.absDeterminant(), overflow));
    VERIFY_IS_APPROX(lu.logAbsDeterminant(), logabsdet);
    VERIFY_IS_EQUAL(lu.signDeterminant(), Scalar(1));
  }
}

// A rank-deficient decomposition has no determinant worth reporting, and FullPivLU is the one LU that
// knows it. The three accessors that can express that gate on rank, so they agree with the rank-revealing
// QR decompositions; determinant() keeps its documented behaviour of returning the pivot product.
template <typename MatrixType>
void lu_determinant_rank_deficient(Index size) {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;

  const Index rank = internal::random<Index>(1, size - 1);
  MatrixType a(size, size);
  createRandomPIMatrixOfRank(rank, size, size, a);

  // The generated singular values are 0 or 1, so any threshold well inside that gap recovers the rank.
  const RealScalar threshold(0.01);

  FullPivLU<MatrixType> lu;
  lu.setThreshold(threshold);
  lu.compute(a);
  VERIFY_IS_EQUAL(lu.rank(), rank);
  VERIFY(!lu.isInvertible());
  VERIFY_IS_EQUAL(lu.absDeterminant(), RealScalar(0));
  VERIFY_IS_EQUAL(lu.logAbsDeterminant(), -NumTraits<RealScalar>::infinity());
  VERIFY_IS_EQUAL(lu.signDeterminant(), Scalar(0));

  ColPivHouseholderQR<MatrixType> qr;
  qr.setThreshold(threshold);
  qr.compute(a);
  VERIFY_IS_EQUAL(qr.rank(), rank);
  VERIFY_IS_EQUAL(qr.absDeterminant(), lu.absDeterminant());
  VERIFY_IS_EQUAL(qr.logAbsDeterminant(), lu.logAbsDeterminant());
  VERIFY_IS_EQUAL(qr.signDeterminant(), lu.signDeterminant());

  // determinant() is deliberately not gated. Pin that on a matrix whose smallest pivot is below the
  // threshold but nonzero, where the two answers are visibly different rather than both roundoff.
  MatrixType b = MatrixType::Identity(size, size);
  b(size - 1, size - 1) = Scalar(RealScalar(1e-30));
  FullPivLU<MatrixType> blu;
  blu.setThreshold(RealScalar(1e-3));
  blu.compute(b);
  VERIFY_IS_EQUAL(blu.rank(), size - 1);
  VERIFY_IS_EQUAL(blu.absDeterminant(), RealScalar(0));
  VERIFY_IS_EQUAL(blu.logAbsDeterminant(), -NumTraits<RealScalar>::infinity());
  VERIFY_IS_EQUAL(blu.signDeterminant(), Scalar(0));
  VERIFY_IS_APPROX(numext::abs(blu.determinant()), RealScalar(1e-30));
}

template <typename MatrixType>
void lu_verify_assert() {
  MatrixType tmp;

  FullPivLU<MatrixType> lu;
  VERIFY_RAISES_ASSERT(lu.matrixLU())
  VERIFY_RAISES_ASSERT(lu.permutationP())
  VERIFY_RAISES_ASSERT(lu.permutationQ())
  VERIFY_RAISES_ASSERT(lu.kernel())
  VERIFY_RAISES_ASSERT(lu.image(tmp))
  VERIFY_RAISES_ASSERT(lu.solve(tmp))
  VERIFY_RAISES_ASSERT(lu.transpose().solve(tmp))
  VERIFY_RAISES_ASSERT(lu.adjoint().solve(tmp))
  VERIFY_RAISES_ASSERT(lu.determinant())
  VERIFY_RAISES_ASSERT(lu.absDeterminant())
  VERIFY_RAISES_ASSERT(lu.logAbsDeterminant())
  VERIFY_RAISES_ASSERT(lu.signDeterminant())
  VERIFY_RAISES_ASSERT(lu.rank())
  VERIFY_RAISES_ASSERT(lu.dimensionOfKernel())
  VERIFY_RAISES_ASSERT(lu.isInjective())
  VERIFY_RAISES_ASSERT(lu.isSurjective())
  VERIFY_RAISES_ASSERT(lu.isInvertible())
  VERIFY_RAISES_ASSERT(lu.inverse())

  PartialPivLU<MatrixType> plu;
  VERIFY_RAISES_ASSERT(plu.matrixLU())
  VERIFY_RAISES_ASSERT(plu.permutationP())
  VERIFY_RAISES_ASSERT(plu.solve(tmp))
  VERIFY_RAISES_ASSERT(plu.transpose().solve(tmp))
  VERIFY_RAISES_ASSERT(plu.adjoint().solve(tmp))
  VERIFY_RAISES_ASSERT(plu.determinant())
  VERIFY_RAISES_ASSERT(plu.absDeterminant())
  VERIFY_RAISES_ASSERT(plu.logAbsDeterminant())
  VERIFY_RAISES_ASSERT(plu.signDeterminant())
  VERIFY_RAISES_ASSERT(plu.inverse())
}

// Rank-deficient matrix returns 0.
// https://gitlab.com/libeigen/eigen/-/issues/2889
void test_2889() {
  Eigen::MatrixXd A =
      Eigen::MatrixXd({{0.0000000000000000, 0.0000000000000000, 1.0000000000000000, 0.0000000000000000,
                        0.34149999916553497, 0.0000000000000000, 0.79877008515664061},
                       {0.0000000000000000, 1.0000000000000000, 0.0000000000000000, 0.29200000315904617,
                        0.0000000000000000, -0.37149999849498272, -0.16425902650844920},
                       {0.0000000000000000, 0.0000000000000000, 1.0000000000000000, 0.0000000000000000,
                        0.34149999916553497, 0.0000000000000000, 0.79877008515664061},
                       {0.0000000000000000, 1.0000000000000000, 0.0000000000000000, 0.040500000119209290,
                        0.0000000000000000, -0.30099999904632568, -0.081170580429391403},
                       {1.0000000000000000, 0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                        0.0000000000000000, 0.0000000000000000, -0.0000000000000000},
                       {0.0000000000000000, 0.70710689672598170, 0.70710666564709435, 0.027000000700354562,
                        0.025455838867477515, -0.025455847186317101, -0.0068972271572272821},
                       {1.0000000000000000, 0.0000000000000000, 0.0000000000000000, 0.0000000000000000,
                        0.0000000000000000, 0.0000000000000000, -0.0000000000000000}});
  Eigen::FullPivLU<Eigen::MatrixXd> lu_factorization(A);
  double rcond = lu_factorization.rcond();
  VERIFY_IS_EQUAL(rcond, 0.0);
}

// Test LU decomposition at blocking and vectorization boundaries.
// PartialPivLU uses blocks of size max(8, min(size/8 rounded to 16, 256)).
// Sizes near these boundaries exercise transitions between full blocks
// and remainder tails, including pivot propagation across block edges.
template <typename Scalar>
void lu_blocking_boundary() {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixType;

  const Index PS = internal::packet_traits<Scalar>::size;
  const Index sizes[] = {1, 2, 3,  PS - 1, PS, PS + 1, 2 * PS - 1, 2 * PS, 2 * PS + 1, 4 * PS, 4 * PS + 1, 7,
                         8, 9, 15, 16,     17, 31,     32,         33,     63,         64,     65};
  for (Index si = 0; si < Index(sizeof(sizes) / sizeof(sizes[0])); ++si) {
    Index n = sizes[si];
    if (n < 1) continue;

    // Create a diagonally dominant (invertible) matrix.
    MatrixType m = MatrixType::Random(n, n);
    m.diagonal().array() += RealScalar(2 * n);

    // PartialPivLU
    PartialPivLU<MatrixType> plu(m);
    VERIFY_IS_APPROX(m, plu.reconstructedMatrix());
    MatrixType rhs = MatrixType::Random(n, 3);
    MatrixType x = plu.solve(rhs);
    VERIFY_IS_APPROX(m * x, rhs);

    // FullPivLU
    FullPivLU<MatrixType> flu(m);
    VERIFY_IS_APPROX(m, flu.reconstructedMatrix());
    VERIFY(flu.isInvertible());
    x = flu.solve(rhs);
    VERIFY_IS_APPROX(m * x, rhs);
  }

  // Non-square matrices at boundary sizes for FullPivLU.
  const Index rect_sizes[][2] = {{PS, 2 * PS}, {2 * PS, PS}, {15, 33}, {33, 15}, {1, 5}, {5, 1}};
  for (Index si = 0; si < Index(sizeof(rect_sizes) / sizeof(rect_sizes[0])); ++si) {
    Index rows = rect_sizes[si][0];
    Index cols = rect_sizes[si][1];
    MatrixType m = MatrixType::Random(rows, cols);
    FullPivLU<MatrixType> flu(m);
    VERIFY_IS_APPROX(m, flu.reconstructedMatrix());
  }
}

// Test PartialPivLU with RowMajor storage order at blocking boundaries.
template <typename Scalar>
void lu_rowmajor_boundary() {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, Dynamic, RowMajor> RowMatrixType;

  const Index sizes[] = {7, 8, 9, 15, 16, 17, 31, 32, 33};
  for (Index si = 0; si < Index(sizeof(sizes) / sizeof(sizes[0])); ++si) {
    Index n = sizes[si];
    RowMatrixType m = RowMatrixType::Random(n, n);
    m.diagonal().array() += RealScalar(2 * n);
    PartialPivLU<RowMatrixType> plu(m);
    VERIFY_IS_APPROX(m, plu.reconstructedMatrix());
  }
}

template <typename Scalar, int StorageOrder>
void lu_strided_pivots() {
  using Mat = Matrix<Scalar, Dynamic, Dynamic, StorageOrder>;
  using RealScalar = typename NumTraits<Scalar>::Real;
  for (Index n : {17, 32, 33, 127, 128, 129, 256, 512}) {
    Mat a = Mat::Random(n, n);
    // A cyclic permutation forces overlapping row swaps across panel boundaries.
    const Index shift = n / 2 + 1;
    a.bottomLeftCorner(n - shift, n - shift).diagonal().array() += RealScalar(2 * n);
    a.topRightCorner(shift, shift).diagonal().array() += RealScalar(2 * n);
    for (Index padding : {0, 7}) {
      const Index stride = n + padding;
      Matrix<Scalar, Dynamic, 1> storage = Matrix<Scalar, Dynamic, 1>::Constant(stride * n + 2, Scalar(7));
      Map<Mat, 0, OuterStride<>> work(storage.data() + 1, n, n, OuterStride<>(stride));
      work = a;
      PartialPivLU<Ref<Mat>> lu(work);
      VERIFY_IS_EQUAL(lu.matrixLU().data(), work.data());
      const RealScalar bound = RealScalar(32 * n) * NumTraits<RealScalar>::epsilon() * a.norm();
      const Mat lower = work.template triangularView<UnitLower>();
      const Mat upper = work.template triangularView<Upper>();
      VERIFY((lu.permutationP() * a - lower * upper).norm() <= bound);
      VERIFY_IS_EQUAL(lu.permutationP().indices()[0], n - shift);
      VERIFY_IS_EQUAL(storage[0], Scalar(7));
      VERIFY_IS_EQUAL(storage[storage.size() - 1], Scalar(7));
      for (Index j = 0; j < n; ++j)
        for (Index i = n; i < stride; ++i) VERIFY_IS_EQUAL(storage[1 + j * stride + i], Scalar(7));
    }
  }
}

EIGEN_DECLARE_TEST(lu) {
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(lu_non_invertible<Matrix3f>());
    CALL_SUBTEST_1(lu_invertible<Matrix3f>());
    CALL_SUBTEST_1(lu_verify_assert<Matrix3f>());
    CALL_SUBTEST_1(lu_partial_piv<Matrix3f>());

    CALL_SUBTEST_2((lu_non_invertible<Matrix<double, 4, 6> >()));
    CALL_SUBTEST_2((lu_verify_assert<Matrix<double, 4, 6> >()));
    CALL_SUBTEST_2(lu_partial_piv<Matrix2d>());
    CALL_SUBTEST_2(lu_partial_piv<Matrix4d>());
    CALL_SUBTEST_2((lu_partial_piv<Matrix<double, 6, 6> >()));

    CALL_SUBTEST_3(lu_non_invertible<MatrixXf>());
    CALL_SUBTEST_3(lu_invertible<MatrixXf>());
    CALL_SUBTEST_3(lu_verify_assert<MatrixXf>());
    CALL_SUBTEST_3(lu_determinant<MatrixXf>(internal::random<int>(1, 30)));

    CALL_SUBTEST_4(lu_non_invertible<MatrixXd>());
    CALL_SUBTEST_4(lu_invertible<MatrixXd>());
    CALL_SUBTEST_4(lu_partial_piv<MatrixXd>(internal::random<int>(1, EIGEN_TEST_MAX_SIZE)));
    CALL_SUBTEST_4(lu_verify_assert<MatrixXd>());
    CALL_SUBTEST_4(lu_empty<MatrixXd>());
    CALL_SUBTEST_4(lu_determinant<MatrixXd>(internal::random<int>(1, 30)));
    CALL_SUBTEST_4(lu_determinant_rank_deficient<MatrixXd>(internal::random<int>(2, 30)));
    CALL_SUBTEST_3(lu_determinant_overflow<MatrixXf>());

    CALL_SUBTEST_5(lu_non_invertible<MatrixXcf>());
    CALL_SUBTEST_5(lu_invertible<MatrixXcf>());
    CALL_SUBTEST_5(lu_verify_assert<MatrixXcf>());
    CALL_SUBTEST_5(lu_determinant<MatrixXcf>(internal::random<int>(1, 30)));

    CALL_SUBTEST_6(lu_non_invertible<MatrixXcd>());
    CALL_SUBTEST_6(lu_invertible<MatrixXcd>());
    CALL_SUBTEST_6(lu_partial_piv<MatrixXcd>(internal::random<int>(1, EIGEN_TEST_MAX_SIZE)));
    CALL_SUBTEST_6(lu_verify_assert<MatrixXcd>());
    CALL_SUBTEST_6(lu_empty<MatrixXcd>());
    CALL_SUBTEST_6(lu_determinant<MatrixXcd>(internal::random<int>(1, 30)));
    CALL_SUBTEST_6(lu_determinant_rank_deficient<MatrixXcd>(internal::random<int>(2, 30)));

    CALL_SUBTEST_7((lu_non_invertible<Matrix<float, Dynamic, 16> >()));

    // Test problem size constructors
    CALL_SUBTEST_8(PartialPivLU<MatrixXf>(10));
    CALL_SUBTEST_8(FullPivLU<MatrixXf>(10, 20););

    CALL_SUBTEST_9(test_2889());
  }

  CALL_SUBTEST_10((lu_strided_pivots<float, ColMajor>()));
  CALL_SUBTEST_11((lu_strided_pivots<double, ColMajor>()));
  CALL_SUBTEST_12((lu_strided_pivots<std::complex<float>, ColMajor>()));
  CALL_SUBTEST_13((lu_strided_pivots<std::complex<double>, ColMajor>()));
  CALL_SUBTEST_14((lu_strided_pivots<double, RowMajor>()));
  CALL_SUBTEST_15((lu_strided_pivots<std::complex<double>, RowMajor>()));

  // Blocking and vectorization boundary tests (deterministic, outside g_repeat).
  CALL_SUBTEST_3(lu_blocking_boundary<float>());
  CALL_SUBTEST_4(lu_blocking_boundary<double>());
  CALL_SUBTEST_5(lu_blocking_boundary<std::complex<float> >());
  CALL_SUBTEST_6(lu_blocking_boundary<std::complex<double> >());
  CALL_SUBTEST_4(lu_rowmajor_boundary<double>());
  CALL_SUBTEST_5(lu_rowmajor_boundary<std::complex<float> >());
}
