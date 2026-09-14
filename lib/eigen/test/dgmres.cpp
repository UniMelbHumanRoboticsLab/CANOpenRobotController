// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2011 Gael Guennebaud <g.gael@free.fr>
// Copyright (C) 2012 desire Nuentsa <desire.nuentsa_wakam@inria.fr
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "sparse_solver.h"
#include <Eigen/IterativeLinearSolvers>

template <typename T>
void test_dgmres_T() {
  DGMRES<SparseMatrix<T>, DiagonalPreconditioner<T> > dgmres_colmajor_diag;
  DGMRES<SparseMatrix<T>, IdentityPreconditioner> dgmres_colmajor_I;
  DGMRES<SparseMatrix<T>, IncompleteLUT<T> > dgmres_colmajor_ilut;
  // GMRES<SparseMatrix<T>, SSORPreconditioner<T> >     dgmres_colmajor_ssor;

  CALL_SUBTEST(check_sparse_square_solving(dgmres_colmajor_diag));
  //   CALL_SUBTEST( check_sparse_square_solving(dgmres_colmajor_I)     );
  CALL_SUBTEST(check_sparse_square_solving(dgmres_colmajor_ilut));
  // CALL_SUBTEST( check_sparse_square_solving(dgmres_colmajor_ssor)     );
}

// Regression: Arnoldi breakdown used to divide by zero (producing NaN in the
// Krylov basis) and solve a singular triangular system, silently returning
// Inf with info() == Success. Exercise both the pathological (rank-deficient
// pivot) and benign (exact Krylov subspace) breakdown paths.
template <typename T>
void test_dgmres_breakdown_T() {
  typedef SparseMatrix<T> Mat;
  typedef Matrix<T, 2, 1> Vec;

  // Nilpotent A with singular Hessenberg pivot on the first step.
  Mat A(2, 2);
  A.insert(0, 1) = T(1);
  A.makeCompressed();
  Vec b;
  b << T(1), T(0);

  DGMRES<Mat, IdentityPreconditioner> solver;
  solver.compute(A);
  Vec x = solver.solve(b);
  VERIFY(x.allFinite());
  VERIFY(solver.info() != Success);

  // Diagonal A with b in an eigenspace: Arnoldi converges after one step.
  Mat D(2, 2);
  D.insert(0, 0) = T(2);
  D.insert(1, 1) = T(2);
  D.makeCompressed();
  Vec d;
  d << T(2), T(2);

  DGMRES<Mat, DiagonalPreconditioner<T> > solver2;
  solver2.compute(D);
  Vec y = solver2.solve(d);
  VERIFY_IS_EQUAL(solver2.info(), Success);
  VERIFY_IS_APPROX(y, (Vec() << T(1), T(1)).finished());
}

// Regression: dgmres() used m_iterations only as the iteration cap and never
// wrote the performed count back, so iterations() returned maxIterations()
// after every solve, however quickly it converged.
template <typename T>
void test_dgmres_iterations_T() {
  using Mat = SparseMatrix<T>;
  using DenseMat = Matrix<T, Dynamic, Dynamic>;
  using Vec = Matrix<T, Dynamic, 1>;
  using RealScalar = typename NumTraits<T>::Real;

  // Well-conditioned tridiagonal system. Its size stays below the default
  // restart length of 30, so a converged solve ends inside the first cycle.
  const Index n = 20;
  Mat A(n, n);
  A.reserve(3 * n);
  for (Index i = 0; i < n; ++i) {
    if (i > 0) A.insert(i, i - 1) = T(-1);
    A.insert(i, i) = T(4);
    if (i + 1 < n) A.insert(i, i + 1) = T(-1);
  }
  A.makeCompressed();

  const Index max_iters = 500;
  // The system is well conditioned, so the true residual tracks the tolerance
  // the solver converged to, which defaults to NumTraits<Scalar>::epsilon().
  const RealScalar res_bound = RealScalar(64) * NumTraits<RealScalar>::epsilon();

  Vec b = Vec::Constant(n, T(1));
  DGMRES<Mat, DiagonalPreconditioner<T> > solver;
  solver.setMaxIterations(max_iters);
  solver.compute(A);
  Vec x = solver.solve(b);
  VERIFY_IS_EQUAL(solver.info(), Success);
  VERIFY(solver.iterations() > 0);
  VERIFY(solver.iterations() < solver.maxIterations());
  VERIFY((A * x - b).norm() <= res_bound * b.norm());

  // Zero right hand side: the early return reports no iteration at all.
  Vec zero = Vec::Zero(n);
  Vec x0 = solver.solve(zero);
  VERIFY(x0.isZero());
  VERIFY_IS_EQUAL(solver.iterations(), Index(0));

  // Several right hand sides: the cap is restored for every column, so a cheap
  // first column must not throttle a costlier second one.
  DenseMat B = DenseMat::Zero(n, 2);
  B.col(0).setConstant(T(1));
  B(0, 1) = T(1);
  DGMRES<Mat, DiagonalPreconditioner<T> > multi;
  multi.setMaxIterations(max_iters);
  multi.compute(A);
  DenseMat X = multi.solve(B);
  VERIFY_IS_EQUAL(multi.info(), Success);
  VERIFY(multi.iterations() > 0);
  VERIFY(multi.iterations() < multi.maxIterations());
  VERIFY((A * X - B).norm() <= res_bound * B.norm());

  // Non-converging direction: GMRES stagnates on a cyclic shift matrix, so the
  // reported count saturates at the cap and never exceeds it.
  const Index m = 16;
  Mat S(m, m);
  S.reserve(m);
  for (Index i = 0; i < m; ++i) S.insert((i + 1) % m, i) = T(1);
  S.makeCompressed();
  Vec e = Vec::Zero(m);
  e(0) = T(1);
  for (Index k = 1; k <= 3; ++k) {
    DGMRES<Mat, IdentityPreconditioner> stalled;
    stalled.setMaxIterations(k);
    stalled.compute(S);
    Vec xs = stalled.solve(e);
    VERIFY(xs.allFinite());
    VERIFY_IS_EQUAL(stalled.info(), NoConvergence);
    VERIFY(stalled.iterations() <= stalled.maxIterations());
    VERIFY_IS_EQUAL(stalled.iterations(), k);
  }
}

EIGEN_DECLARE_TEST(dgmres) {
  CALL_SUBTEST_1(test_dgmres_T<double>());
  CALL_SUBTEST_2(test_dgmres_T<std::complex<double> >());
  CALL_SUBTEST_3(test_dgmres_breakdown_T<double>());
  CALL_SUBTEST_4(test_dgmres_breakdown_T<std::complex<double> >());
  CALL_SUBTEST_5(test_dgmres_iterations_T<double>());
  CALL_SUBTEST_6(test_dgmres_iterations_T<std::complex<double> >());
}
