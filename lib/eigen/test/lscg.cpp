// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2011 Gael Guennebaud <g.gael@free.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "sparse_solver.h"
#include <Eigen/IterativeLinearSolvers>

template <typename T>
void test_least_square_diagonal_preconditioner_zero_columns() {
  SparseMatrix<T, RowMajor> mat(3, 3);
  mat.insert(0, 0) = T(2);
  mat.insert(2, 2) = T(4);
  mat.makeCompressed();

  LeastSquareDiagonalPreconditioner<T> precond(mat);
  Matrix<T, 3, 1> rhs = Matrix<T, 3, 1>::Ones();
  Matrix<T, 3, 1> expected;
  expected << T(0.25), T(1), T(0.0625);

  VERIFY_IS_APPROX(precond.solve(rhs), expected);
}

template <typename T>
void test_lscg_T() {
  LeastSquaresConjugateGradient<SparseMatrix<T> > lscg_colmajor_diag;
  LeastSquaresConjugateGradient<SparseMatrix<T>, IdentityPreconditioner> lscg_colmajor_I;
  LeastSquaresConjugateGradient<SparseMatrix<T, RowMajor> > lscg_rowmajor_diag;
  LeastSquaresConjugateGradient<SparseMatrix<T, RowMajor>, IdentityPreconditioner> lscg_rowmajor_I;

  CALL_SUBTEST(check_sparse_square_solving(lscg_colmajor_diag));
  CALL_SUBTEST(check_sparse_square_solving(lscg_colmajor_I));

  CALL_SUBTEST(check_sparse_leastsquare_solving(lscg_colmajor_diag));
  CALL_SUBTEST(check_sparse_leastsquare_solving(lscg_colmajor_I));

  CALL_SUBTEST(check_sparse_square_solving(lscg_rowmajor_diag));
  CALL_SUBTEST(check_sparse_square_solving(lscg_rowmajor_I));

  CALL_SUBTEST(check_sparse_leastsquare_solving(lscg_rowmajor_diag));
  CALL_SUBTEST(check_sparse_leastsquare_solving(lscg_rowmajor_I));
}

void test_lscg_extreme_rhs() {
  const Matrix2d mat = Matrix2d::Identity();
  const Vector2d direction = (Vector2d() << 1, -1).finished();
  LeastSquaresConjugateGradient<Matrix2d, IdentityPreconditioner> solver(mat);
  solver.setTolerance(1e-12);

  for (double scale : {1e-200, 1e200}) {
    const Vector2d rhs = scale * direction;
    const Vector2d guess = 0.5 * rhs;
    Vector2d x = solver.solve(rhs);
    VERIFY_IS_EQUAL(solver.info(), Success);
    VERIFY(x.allFinite());
    VERIFY_IS_APPROX(x / scale, direction);

    x = solver.solveWithGuess(rhs, guess);
    VERIFY_IS_EQUAL(solver.info(), Success);
    VERIFY(x.allFinite());
    VERIFY_IS_APPROX(x / scale, direction);
  }
}

EIGEN_DECLARE_TEST(lscg) {
  CALL_SUBTEST_1(test_lscg_T<double>());
  CALL_SUBTEST_2(test_lscg_T<std::complex<double> >());
  CALL_SUBTEST_3(test_least_square_diagonal_preconditioner_zero_columns<double>());
  CALL_SUBTEST_4(test_least_square_diagonal_preconditioner_zero_columns<std::complex<double> >());
  CALL_SUBTEST_5(test_lscg_extreme_rhs());
}
