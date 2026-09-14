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

template <typename T, typename I_>
void test_conjugate_gradient_T() {
  typedef SparseMatrix<T, 0, I_> SparseMatrixType;
  ConjugateGradient<SparseMatrixType, Lower> cg_colmajor_lower_diag;
  ConjugateGradient<SparseMatrixType, Upper> cg_colmajor_upper_diag;
  ConjugateGradient<SparseMatrixType, Lower | Upper> cg_colmajor_loup_diag;
  ConjugateGradient<SparseMatrixType, Lower, IdentityPreconditioner> cg_colmajor_lower_I;
  ConjugateGradient<SparseMatrixType, Upper, IdentityPreconditioner> cg_colmajor_upper_I;

  CALL_SUBTEST(check_sparse_spd_solving(cg_colmajor_lower_diag));
  CALL_SUBTEST(check_sparse_spd_solving(cg_colmajor_upper_diag));
  CALL_SUBTEST(check_sparse_spd_solving(cg_colmajor_loup_diag));
  CALL_SUBTEST(check_sparse_spd_solving(cg_colmajor_lower_I));
  CALL_SUBTEST(check_sparse_spd_solving(cg_colmajor_upper_I));
}

// Regression for issue #1704: default-constructing an iterative solver
// templated on a fixed-size MatrixType tripped a resize(0,0) size assertion.
template <typename MatrixType>
void test_default_construct_fixed_size() {
  ConjugateGradient<MatrixType> cg;
  BiCGSTAB<MatrixType> bicg;
  LeastSquaresConjugateGradient<MatrixType> lscg;
}

void test_conjugate_gradient_extreme_rhs() {
  const Matrix2d mat = Matrix2d::Identity();
  const Vector2d direction = (Vector2d() << 1, -1).finished();
  ConjugateGradient<Matrix2d, Lower | Upper, IdentityPreconditioner> solver(mat);
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

EIGEN_DECLARE_TEST(conjugate_gradient) {
  CALL_SUBTEST_1((test_conjugate_gradient_T<double, int>()));
  CALL_SUBTEST_2((test_conjugate_gradient_T<std::complex<double>, int>()));
  CALL_SUBTEST_3((test_conjugate_gradient_T<double, long int>()));
  CALL_SUBTEST_4(test_default_construct_fixed_size<Matrix3d>());
  CALL_SUBTEST_5(test_conjugate_gradient_extreme_rhs());
}
