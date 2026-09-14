// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2025 Johannes Zipfel <johzip1010@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "sparse_solver.h"
#include <Eigen/IterativeLinearSolvers>

template <typename T, typename I_>
void test_incompleteLUT_T() {
  IncompleteLUT<T, I_> ilut;
  ilut.setDroptol(NumTraits<T>::epsilon() * 4);
}

template <typename T>
void test_extract_LU() {
  typedef Eigen::SparseMatrix<T> SparseMatrix;

  SparseMatrix A(5, 5);
  std::vector<Eigen::Triplet<T>> triplets;
  triplets.push_back({0, 0, 4});
  triplets.push_back({0, 1, -1});
  triplets.push_back({0, 4, -1});
  triplets.push_back({1, 0, -1});
  triplets.push_back({1, 1, 4});
  triplets.push_back({1, 2, -1});
  triplets.push_back({2, 1, -1});
  triplets.push_back({2, 2, 4});
  triplets.push_back({2, 3, -1});
  triplets.push_back({3, 2, -1});
  triplets.push_back({3, 3, 4});
  triplets.push_back({3, 4, -1});
  triplets.push_back({4, 0, -1});
  triplets.push_back({4, 3, -1});
  triplets.push_back({4, 4, 4});

  A.setFromTriplets(triplets.begin(), triplets.end());

  IncompleteLUT<T> ilut;
  ilut.compute(A);

  Eigen::SparseMatrix<T> matL = ilut.matrixL();  // Extract L
  Eigen::SparseMatrix<T> matU = ilut.matrixU();  // Extract U

  Eigen::SparseMatrix<T> expectedMatL(5, 5);
  std::vector<Eigen::Triplet<T>> tripletsExL;
  tripletsExL.emplace_back(0, 0, T(1));
  tripletsExL.emplace_back(1, 0, T(-0.25));
  tripletsExL.emplace_back(1, 1, T(1));
  tripletsExL.emplace_back(2, 0, T(-0.25));
  tripletsExL.emplace_back(2, 1, T(-0.0666667));
  tripletsExL.emplace_back(2, 2, T(1));
  tripletsExL.emplace_back(3, 2, T(-0.25));
  tripletsExL.emplace_back(3, 3, T(1));
  tripletsExL.emplace_back(4, 1, T(-0.266667));
  tripletsExL.emplace_back(4, 3, T(-0.266667));
  tripletsExL.emplace_back(4, 4, T(1));
  expectedMatL.setFromTriplets(tripletsExL.begin(), tripletsExL.end());

  Eigen::SparseMatrix<T> expectedMatU(5, 5);
  std::vector<Eigen::Triplet<T>> tripletsExU;
  tripletsExU.emplace_back(0, 0, T(4));
  tripletsExU.emplace_back(0, 1, T(-1));
  tripletsExU.emplace_back(1, 1, T(3.75));
  tripletsExU.emplace_back(1, 4, T(-1));
  tripletsExU.emplace_back(2, 2, T(4));
  tripletsExU.emplace_back(2, 3, T(-1));
  tripletsExU.emplace_back(3, 3, T(3.75));
  tripletsExU.emplace_back(3, 4, T(-1));
  tripletsExU.emplace_back(4, 4, T(3.46667));
  expectedMatU.setFromTriplets(tripletsExU.begin(), tripletsExU.end());

  VERIFY_IS_APPROX(expectedMatL, matL);
  VERIFY_IS_APPROX(expectedMatU, matU);
}

// https://gitlab.com/libeigen/eigen/-/issues/2626
// IncompleteLUT must apply a static row permutation so that matrices whose
// natural diagonal has zeros (but admit a full bipartite matching on the
// sparsity pattern) can still be factorized. Without it, ILUT silently
// produces a useless preconditioner.
template <typename T>
void test_zero_diagonal_2626() {
  typedef Eigen::SparseMatrix<T> SparseMatrixT;
  typedef Eigen::Matrix<T, Eigen::Dynamic, 1> Vector;

  const int n = 50;
  // 1D Laplacian: SPD, fully nonzero diagonal.
  std::vector<Eigen::Triplet<T>> triplets;
  for (int i = 0; i < n; ++i) {
    triplets.emplace_back(i, i, T(2));
    if (i > 0) triplets.emplace_back(i, i - 1, T(-1));
    if (i + 1 < n) triplets.emplace_back(i, i + 1, T(-1));
  }
  SparseMatrixT base(n, n);
  base.setFromTriplets(triplets.begin(), triplets.end());

  // Cyclic-shift row permutation by 2: separates the natural diagonal from
  // the tridiagonal nonzero band, so all diagonal entries of (Pshift * base)
  // are zero. A valid row matching must find the inverse shift.
  Eigen::PermutationMatrix<Eigen::Dynamic> Pshift(n);
  for (int i = 0; i < n; ++i) Pshift.indices()(i) = (i + 2) % n;
  SparseMatrixT A = Pshift * base;

  // Verify the test setup: every diagonal is zero.
  int nz_diag = 0;
  for (int i = 0; i < n; ++i)
    if (A.coeff(i, i) != T(0)) ++nz_diag;
  VERIFY(nz_diag == 0);

  // BiCGSTAB + IncompleteLUT must converge to a reasonable residual.
  Vector b = Vector::Random(n);
  Eigen::BiCGSTAB<SparseMatrixT, Eigen::IncompleteLUT<T>> solver;
  solver.setTolerance(typename Eigen::NumTraits<T>::Real(16) * Eigen::NumTraits<T>::epsilon());
  solver.compute(A);
  VERIFY(solver.preconditioner().info() == Eigen::Success);
  Vector x = solver.solve(b);
  VERIFY(solver.info() == Eigen::Success);
  Vector residual = b - A * x;
  // Solver was set to tol = 16*eps; allow some slack for the residual check.
  typename Eigen::NumTraits<T>::Real residual_bound =
      typename Eigen::NumTraits<T>::Real(1024) * Eigen::NumTraits<T>::epsilon() * b.norm();
  VERIFY(residual.norm() < residual_bound);
}

// A structurally singular matrix (empty row) cannot be made diagonal-nonzero
// by any row permutation; the factorization must report NumericalIssue.
// This covers the rownorm == 0 early-return path.
template <typename T>
void test_structurally_singular() {
  typedef Eigen::SparseMatrix<T> SparseMatrixT;
  std::vector<Eigen::Triplet<T>> triplets;
  triplets.emplace_back(0, 0, T(2));
  triplets.emplace_back(0, 1, T(1));
  triplets.emplace_back(1, 0, T(1));
  triplets.emplace_back(1, 1, T(2));
  // row 2 is intentionally empty
  triplets.emplace_back(3, 0, T(1));
  triplets.emplace_back(3, 3, T(2));
  SparseMatrixT A(4, 4);
  A.setFromTriplets(triplets.begin(), triplets.end());

  Eigen::IncompleteLUT<T> ilut;
  ilut.compute(A);
  VERIFY(ilut.info() == Eigen::NumericalIssue);
}

// A matrix where every row is structurally non-empty (so rownorm != 0) but
// no full bipartite matching exists, forcing at least one pivot to be shifted.
// Rows 0 and 1 only have entries in column 0, so columns 1 and 2 cannot both
// be matched. This exercises the shifted-pivot path that flips info() to
// NumericalIssue.
template <typename T>
void test_zero_pivot_numerical_issue() {
  typedef Eigen::SparseMatrix<T> SparseMatrixT;
  SparseMatrixT A(3, 3);
  std::vector<Eigen::Triplet<T>> triplets;
  triplets.emplace_back(0, 0, T(1));
  triplets.emplace_back(1, 0, T(2));  // row 1 competes with row 0 for column 0
  triplets.emplace_back(2, 2, T(3));
  A.setFromTriplets(triplets.begin(), triplets.end());

  Eigen::IncompleteLUT<T> ilut;
  ilut.compute(A);
  VERIFY(ilut.info() == Eigen::NumericalIssue);
}

// analyzePattern() must depend only on the stored sparsity pattern and not on
// numerical values. Otherwise, the two-step API contract breaks: the same
// analysis would no longer be reusable for any matrix sharing this stored
// pattern. Repro: analyze a pattern with all-zero placeholder values, then
// factorize a numerically nonzero matrix sharing that pattern.
template <typename T>
void test_pattern_value_separation() {
  typedef Eigen::SparseMatrix<T> SparseMatrixT;
  SparseMatrixT pattern(2, 2);
  pattern.insert(0, 1) = T(0);
  pattern.insert(1, 0) = T(0);
  pattern.makeCompressed();

  SparseMatrixT A(2, 2);
  A.insert(0, 1) = T(1);
  A.insert(1, 0) = T(1);
  A.makeCompressed();

  Eigen::IncompleteLUT<T> ilut;
  ilut.analyzePattern(pattern);
  ilut.factorize(A);
  VERIFY(ilut.info() == Eigen::Success);
}

EIGEN_DECLARE_TEST(incomplete_LUT) {
  CALL_SUBTEST_1((test_incompleteLUT_T<double, int>()));
  CALL_SUBTEST_1((test_incompleteLUT_T<float, int>()));
  CALL_SUBTEST_2((test_incompleteLUT_T<std::complex<double>, int>()));
  CALL_SUBTEST_3((test_incompleteLUT_T<double, long int>()));

  CALL_SUBTEST_4(test_extract_LU<double>());
  CALL_SUBTEST_4(test_extract_LU<float>());

  CALL_SUBTEST_5(test_zero_diagonal_2626<double>());
  CALL_SUBTEST_5(test_zero_diagonal_2626<float>());
  CALL_SUBTEST_5(test_structurally_singular<double>());
  CALL_SUBTEST_5(test_zero_pivot_numerical_issue<double>());
  CALL_SUBTEST_5(test_pattern_value_separation<double>());
}
