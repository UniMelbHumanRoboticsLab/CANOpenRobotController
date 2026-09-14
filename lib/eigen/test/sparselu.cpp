// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2012 Désiré Nuentsa-Wakam <desire.nuentsa_wakam@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// SparseLU solve does not accept column major matrices for the destination.
// However, as expected, the generic check_sparse_square_solving routines produces row-major
// rhs and destination matrices when compiled with EIGEN_DEFAULT_TO_ROW_MAJOR

#ifdef EIGEN_DEFAULT_TO_ROW_MAJOR
#undef EIGEN_DEFAULT_TO_ROW_MAJOR
#endif

#include "sparse_solver.h"
#include <Eigen/SparseLU>

template <typename T>
void test_sparselu_T() {
  SparseLU<SparseMatrix<T, ColMajor> /*, COLAMDOrdering<int>*/> sparselu_colamd;  // COLAMDOrdering is the default
  SparseLU<SparseMatrix<T, ColMajor>, AMDOrdering<int> > sparselu_amd;
  SparseLU<SparseMatrix<T, ColMajor, long int>, NaturalOrdering<long int> > sparselu_natural;

  check_sparse_square_solving(sparselu_colamd, 300, 100000, true);
  check_sparse_square_solving(sparselu_amd, 300, 10000, true);
  check_sparse_square_solving(sparselu_natural, 300, 2000, true);

  check_sparse_square_abs_determinant(sparselu_colamd);
  check_sparse_square_abs_determinant(sparselu_amd);

  check_sparse_square_determinant(sparselu_colamd);
  check_sparse_square_determinant(sparselu_amd);
}

template <typename T>
void test_sparselu_rowmajor_compressed_input() {
  typedef SparseMatrix<T, RowMajor> RowMajorSparseMatrix;
  typedef Matrix<T, Dynamic, 1> Vector;

  Vector b(2);
  b << T(1.1), T(3.14);

  Vector expected(2);
  expected << T(1.1 - 0.0001 * 3.14), T(3.14);

  RowMajorSparseMatrix compressed(2, 2);
  compressed.insert(0, 0) = T(1.0);
  compressed.insert(0, 1) = T(0.0001);
  compressed.insert(1, 1) = T(1.0);
  compressed.makeCompressed();

  RowMajorSparseMatrix uncompressed(2, 2);
  uncompressed.insert(0, 0) = T(1.0);
  uncompressed.insert(0, 1) = T(0.0001);
  uncompressed.insert(1, 1) = T(1.0);

  SparseLU<RowMajorSparseMatrix> compressed_solver;
  compressed_solver.compute(compressed);
  VERIFY_IS_EQUAL(compressed_solver.info(), Success);
  VERIFY_IS_APPROX(compressed_solver.solve(b), expected);

  SparseLU<RowMajorSparseMatrix> two_step_solver;
  two_step_solver.analyzePattern(compressed);
  two_step_solver.factorize(compressed);
  VERIFY_IS_EQUAL(two_step_solver.info(), Success);
  VERIFY_IS_APPROX(two_step_solver.solve(b), expected);

  SparseLU<RowMajorSparseMatrix> uncompressed_solver;
  uncompressed_solver.compute(uncompressed);
  VERIFY_IS_EQUAL(uncompressed_solver.info(), Success);
  VERIFY_IS_APPROX(uncompressed_solver.solve(b), expected);
}

template <typename T>
void test_sparselu_colmajor_uncompressed_input() {
  typedef SparseMatrix<T, ColMajor> ColMajorSparseMatrix;
  typedef Matrix<T, Dynamic, 1> Vector;

  Vector b(2);
  b << T(1.1), T(3.14);

  Vector expected(2);
  expected << T(1.1 - 0.0001 * 3.14), T(3.14);

  ColMajorSparseMatrix uncompressed(2, 2);
  uncompressed.insert(0, 0) = T(1.0);
  uncompressed.insert(0, 1) = T(0.0001);
  uncompressed.insert(1, 1) = T(1.0);

  SparseLU<ColMajorSparseMatrix> uncompressed_solver;
  uncompressed_solver.compute(uncompressed);
  VERIFY_IS_EQUAL(uncompressed_solver.info(), Success);
  VERIFY_IS_APPROX(uncompressed_solver.solve(b), expected);
}

// Regression for issue #1908: lastErrorMessage() must not carry over from a
// previous failed factorize() once a subsequent factorize() succeeds.
template <typename T>
void test_sparselu_clear_error_state() {
  typedef SparseMatrix<T, ColMajor> ColMajorSparseMatrix;

  ColMajorSparseMatrix singular(3, 3);  // structurally singular: no nonzeros

  ColMajorSparseMatrix non_singular(3, 3);
  non_singular.insert(0, 0) = T(1);
  non_singular.insert(1, 1) = T(1);
  non_singular.insert(2, 2) = T(1);
  non_singular.makeCompressed();

  SparseLU<ColMajorSparseMatrix> solver;
  solver.compute(singular);
  VERIFY(solver.info() != Success);
  VERIFY(!solver.lastErrorMessage().empty());

  solver.compute(non_singular);
  VERIFY_IS_EQUAL(solver.info(), Success);
  VERIFY(solver.lastErrorMessage().empty());
}

EIGEN_DECLARE_TEST(sparselu) {
  CALL_SUBTEST_1(test_sparselu_T<float>());
  CALL_SUBTEST_2(test_sparselu_T<double>());
  CALL_SUBTEST_3(test_sparselu_T<std::complex<float> >());
  CALL_SUBTEST_4(test_sparselu_T<std::complex<double> >());
  CALL_SUBTEST_5(test_sparselu_rowmajor_compressed_input<float>());
  CALL_SUBTEST_6(test_sparselu_rowmajor_compressed_input<double>());
  CALL_SUBTEST_7(test_sparselu_colmajor_uncompressed_input<float>());
  CALL_SUBTEST_8(test_sparselu_colmajor_uncompressed_input<double>());
  CALL_SUBTEST_9(test_sparselu_clear_error_state<float>());
  CALL_SUBTEST_10(test_sparselu_clear_error_state<double>());
}
