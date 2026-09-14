// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include "sparse_solver.h"
#include <Eigen/IterativeLinearSolvers>

// Sparse square and least-squares solving with the default (identity) and the
// least-squares diagonal preconditioner, in both storage orders.
template <typename T>
void test_lsmr_T() {
  LSMR<SparseMatrix<T> > lsmr_colmajor_I;
  LSMR<SparseMatrix<T>, LeastSquareDiagonalPreconditioner<T> > lsmr_colmajor_diag;
  LSMR<SparseMatrix<T, RowMajor> > lsmr_rowmajor_I;
  LSMR<SparseMatrix<T, RowMajor>, LeastSquareDiagonalPreconditioner<T> > lsmr_rowmajor_diag;

  CALL_SUBTEST(check_sparse_square_solving(lsmr_colmajor_I));
  CALL_SUBTEST(check_sparse_square_solving(lsmr_colmajor_diag));

  CALL_SUBTEST(check_sparse_leastsquare_solving(lsmr_colmajor_I));
  CALL_SUBTEST(check_sparse_leastsquare_solving(lsmr_colmajor_diag));

  CALL_SUBTEST(check_sparse_square_solving(lsmr_rowmajor_I));
  CALL_SUBTEST(check_sparse_leastsquare_solving(lsmr_rowmajor_diag));
}

// LSMR on dense, rectangular, overdetermined problems: the solution must match
// the dense QR least-squares solution.
template <typename T>
void test_lsmr_dense() {
  typedef typename NumTraits<T>::Real RealScalar;
  typedef Matrix<T, Dynamic, Dynamic> DenseMatrix;
  typedef Matrix<T, Dynamic, 1> DenseVector;

  for (int k = 0; k < g_repeat; ++k) {
    Index rows = internal::random<Index>(20, 80);
    Index cols = internal::random<Index>(4, rows);
    DenseMatrix A = DenseMatrix::Random(rows, cols);
    DenseVector b = DenseVector::Random(rows);

    LSMR<DenseMatrix> lsmr(A);
    lsmr.setTolerance(NumTraits<RealScalar>::epsilon() * RealScalar(100));
    DenseVector x = lsmr.solve(b);
    VERIFY_IS_EQUAL(lsmr.info(), Success);

    DenseVector xref = A.householderQr().solve(b);
    VERIFY_IS_APPROX(x, xref);
  }
}

// solveWithGuess() must actually consume the initial guess. Starting from the
// exact solution, the normal-equation residual A^T(b - A x0) is ~0, so LSMR
// converges in very few iterations -- far fewer than from x = 0. Were the guess
// silently dropped, both runs would start from x = 0 and use the same count, so
// the strict iteration-count comparison below is load-bearing. We use cols >= 10
// so that the from-zero run needs many iterations, giving a wide margin.
template <typename T>
void test_lsmr_guess() {
  typedef typename NumTraits<T>::Real RealScalar;
  typedef Matrix<T, Dynamic, Dynamic> DenseMatrix;
  typedef Matrix<T, Dynamic, 1> DenseVector;

  for (int k = 0; k < g_repeat; ++k) {
    Index cols = internal::random<Index>(10, 20);
    Index rows = cols + internal::random<Index>(10, 40);
    DenseMatrix A = DenseMatrix::Random(rows, cols);
    DenseVector b = DenseVector::Random(rows);
    DenseVector xref = A.householderQr().solve(b);

    LSMR<DenseMatrix> lsmr(A);
    lsmr.setTolerance(NumTraits<RealScalar>::epsilon() * RealScalar(100));

    DenseVector x0 = lsmr.solve(b);
    VERIFY_IS_EQUAL(lsmr.info(), Success);
    VERIFY_IS_APPROX(x0, xref);
    const Index iters_from_zero = lsmr.iterations();

    DenseVector xg = lsmr.solveWithGuess(b, xref);
    VERIFY_IS_EQUAL(lsmr.info(), Success);
    VERIFY_IS_APPROX(xg, xref);
    VERIFY(lsmr.iterations() < iters_from_zero);
  }
}

// On rank-deficient or underdetermined (rows < cols) problems the least-squares
// solution is not unique. Started from x = 0, LSMR returns the *minimum-norm*
// least-squares solution -- exactly what completeOrthogonalDecomposition
// returns, whereas householderQr would return some other valid LS solution.
template <typename T>
void test_lsmr_minnorm() {
  typedef typename NumTraits<T>::Real RealScalar;
  typedef Matrix<T, Dynamic, Dynamic> DenseMatrix;
  typedef Matrix<T, Dynamic, 1> DenseVector;

  for (int k = 0; k < g_repeat; ++k) {
    // (a) Underdetermined, full row rank: rows < cols, many exact solutions.
    {
      Index rows = internal::random<Index>(10, 30);
      Index cols = rows + internal::random<Index>(5, 30);
      DenseMatrix A = DenseMatrix::Random(rows, cols);
      DenseVector b = DenseVector::Random(rows);

      LSMR<DenseMatrix> lsmr(A);
      lsmr.setTolerance(NumTraits<RealScalar>::epsilon() * RealScalar(100));
      DenseVector x = lsmr.solve(b);
      VERIFY_IS_EQUAL(lsmr.info(), Success);

      DenseVector xref = A.completeOrthogonalDecomposition().solve(b);
      VERIFY_IS_APPROX(x, xref);
    }
    // (b) Explicitly rank-deficient, overdetermined: a duplicated column.
    {
      Index rows = internal::random<Index>(30, 60);
      Index cols = internal::random<Index>(5, 15);
      DenseMatrix A = DenseMatrix::Random(rows, cols);
      A.col(1) = A.col(0);  // make A rank-deficient
      DenseVector b = DenseVector::Random(rows);

      LSMR<DenseMatrix> lsmr(A);
      lsmr.setTolerance(NumTraits<RealScalar>::epsilon() * RealScalar(100));
      DenseVector x = lsmr.solve(b);
      VERIFY_IS_EQUAL(lsmr.info(), Success);

      DenseVector xref = A.completeOrthogonalDecomposition().solve(b);
      VERIFY_IS_APPROX(x, xref);
    }
  }
}

// With damping lambda > 0 LSMR solves the regularized problem
//   min ||Ax-b||^2 + lambda^2 ||x||^2,
// whose solution satisfies the (well-conditioned) normal equation
//   (A^T A + lambda^2 I) x = A^T b.
template <typename T>
void test_lsmr_damping() {
  typedef typename NumTraits<T>::Real RealScalar;
  typedef Matrix<T, Dynamic, Dynamic> DenseMatrix;
  typedef Matrix<T, Dynamic, 1> DenseVector;

  for (int k = 0; k < g_repeat; ++k) {
    Index rows = internal::random<Index>(20, 60);
    Index cols = internal::random<Index>(4, rows);
    DenseMatrix A = DenseMatrix::Random(rows, cols);
    DenseVector b = DenseVector::Random(rows);
    RealScalar lambda = internal::random<RealScalar>(RealScalar(0.2), RealScalar(2));

    LSMR<DenseMatrix> lsmr(A);
    lsmr.setDamping(lambda);
    lsmr.setTolerance(NumTraits<RealScalar>::epsilon() * RealScalar(100));
    DenseVector x = lsmr.solve(b);
    VERIFY_IS_EQUAL(lsmr.info(), Success);

    DenseMatrix normalMat = A.adjoint() * A + (lambda * lambda) * DenseMatrix::Identity(cols, cols);
    DenseVector xref = normalMat.ldlt().solve(A.adjoint() * b);
    VERIFY_IS_APPROX(x, xref);
  }
}

// Robustness against extreme right-hand-side scaling: LSMR normalizes by ||b||
// immediately, so the solution should scale linearly with b.
void test_lsmr_extreme_rhs() {
  const Matrix2d mat = Matrix2d::Identity();
  const Vector2d direction = (Vector2d() << 1, -1).finished();
  LSMR<Matrix2d> solver(mat);
  solver.setTolerance(1e-12);

  for (double scale : {1e-200, 1e200}) {
    const Vector2d rhs = scale * direction;
    Vector2d x = solver.solve(rhs);
    VERIFY_IS_EQUAL(solver.info(), Success);
    VERIFY(x.allFinite());
    VERIFY_IS_APPROX(x / scale, direction);
  }
}

// The atol/btol stopping tolerances can be set independently. setTolerance()
// sets both; setToleranceA()/setToleranceB() override each one, and an unset
// component falls back to tolerance(). A tight atol must still reach the QR
// least-squares solution even when btol is left loose.
template <typename T>
void test_lsmr_tolerances() {
  typedef typename NumTraits<T>::Real RealScalar;
  typedef Matrix<T, Dynamic, Dynamic> DenseMatrix;
  typedef Matrix<T, Dynamic, 1> DenseVector;

  const RealScalar def = NumTraits<RealScalar>::epsilon();
  const RealScalar tight = def * RealScalar(100);

  // By default both tolerances track tolerance().
  LSMR<DenseMatrix> lsmr;
  VERIFY_IS_EQUAL(lsmr.toleranceA(), def);
  VERIFY_IS_EQUAL(lsmr.toleranceB(), def);

  // setTolerance() sets both.
  lsmr.setTolerance(tight);
  VERIFY_IS_EQUAL(lsmr.toleranceA(), tight);
  VERIFY_IS_EQUAL(lsmr.toleranceB(), tight);

  // An explicit override wins; the other component still tracks tolerance().
  lsmr.setToleranceA(def);
  VERIFY_IS_EQUAL(lsmr.toleranceA(), def);
  VERIFY_IS_EQUAL(lsmr.toleranceB(), tight);
  lsmr.setToleranceB(def);
  VERIFY_IS_EQUAL(lsmr.toleranceB(), def);

  // Use a strongly overdetermined, inconsistent system (rows >> cols) so the
  // minimum-residual ratio ||A x_LS - b|| / ||b|| stays well above btol. That
  // keeps the residual stopping rule (istop 1, governed by btol) from firing,
  // leaving the tight atol to drive convergence to the least-squares solution.
  // A near-square system would let ||r||/||b|| dip below btol first, stopping
  // LSMR early on an iterate that is nowhere near the QR solution.
  for (int k = 0; k < g_repeat; ++k) {
    Index rows = internal::random<Index>(40, 80);
    Index cols = internal::random<Index>(4, rows / 4);
    DenseMatrix A = DenseMatrix::Random(rows, cols);
    DenseVector b = DenseVector::Random(rows);
    DenseVector xref = A.householderQr().solve(b);

    // Tight atol drives the least-squares stopping rule, so the solution must
    // match QR even with btol left loose.
    LSMR<DenseMatrix> solver(A);
    solver.setToleranceA(tight).setToleranceB(RealScalar(0.1));
    DenseVector x = solver.solve(b);
    VERIFY_IS_EQUAL(solver.info(), Success);
    VERIFY_IS_APPROX(x, xref);
  }
}

EIGEN_DECLARE_TEST(lsmr) {
  CALL_SUBTEST_1(test_lsmr_T<double>());
  CALL_SUBTEST_2(test_lsmr_T<std::complex<double> >());
  CALL_SUBTEST_3(test_lsmr_dense<double>());
  CALL_SUBTEST_4(test_lsmr_dense<std::complex<double> >());
  CALL_SUBTEST_5(test_lsmr_damping<double>());
  CALL_SUBTEST_6(test_lsmr_extreme_rhs());
  CALL_SUBTEST_7(test_lsmr_minnorm<double>());
  CALL_SUBTEST_8(test_lsmr_damping<std::complex<double> >());
  CALL_SUBTEST_9(test_lsmr_minnorm<std::complex<double> >());
  CALL_SUBTEST_10(test_lsmr_guess<double>());
  CALL_SUBTEST_11(test_lsmr_tolerances<double>());
}
