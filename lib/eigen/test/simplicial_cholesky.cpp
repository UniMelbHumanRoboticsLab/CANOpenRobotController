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

template <typename Solver>
void verify_determinant_asserts(const Solver& solver) {
  VERIFY_RAISES_ASSERT(solver.determinant());
  VERIFY_RAISES_ASSERT(solver.absDeterminant());
  VERIFY_RAISES_ASSERT(solver.logAbsDeterminant());
  VERIFY_RAISES_ASSERT(solver.signDeterminant());
}

// factorize_preordered() breaks out of its column loop on a bad pivot but still sets m_factorizationIsOk,
// leaving the tail of D -- and of L's diagonal -- unwritten. The determinant accessors read exactly those,
// so they check info() too, as the dense LLT ones do.
template <typename SparseMatrixType>
void test_simplicial_cholesky_determinant_asserts() {
  typedef typename SparseMatrixType::Scalar Scalar;
  const Index n = 6;

  // A zero pivot stops all four classes; SimplicialLLT alone also rejects a negative one.
  SparseMatrixType zero_pivot(n, n), negative_pivot(n, n);
  for (Index i = 0; i < n; ++i) {
    zero_pivot.insert(i, i) = (i == 0) ? Scalar(0) : Scalar(1);
    negative_pivot.insert(i, i) = (i == n - 1) ? Scalar(-1) : Scalar(1);
  }
  zero_pivot.makeCompressed();
  negative_pivot.makeCompressed();

  SimplicialLLT<SparseMatrixType> llt(zero_pivot);
  VERIFY(llt.info() == NumericalIssue);
  verify_determinant_asserts(llt);

  SimplicialLLT<SparseMatrixType> indefinite_llt(negative_pivot);
  VERIFY(indefinite_llt.info() == NumericalIssue);
  verify_determinant_asserts(indefinite_llt);

  SimplicialLDLT<SparseMatrixType> ldlt(zero_pivot);
  VERIFY(ldlt.info() == NumericalIssue);
  verify_determinant_asserts(ldlt);

  SimplicialNonHermitianLLT<SparseMatrixType> nhllt(zero_pivot);
  VERIFY(nhllt.info() == NumericalIssue);
  verify_determinant_asserts(nhllt);

  SimplicialNonHermitianLDLT<SparseMatrixType> nhldlt(zero_pivot);
  VERIFY(nhldlt.info() == NumericalIssue);
  verify_determinant_asserts(nhldlt);

  // The deprecated SimplicialCholesky exposes determinant() alone, and reads the same tail.
  SimplicialCholesky<SparseMatrixType> deprecated(zero_pivot);
  VERIFY(deprecated.info() == NumericalIssue);
  VERIFY_RAISES_ASSERT(deprecated.determinant());

  // info() alone would not reject an unfactorized decomposition: it is constructed Success.
  SimplicialLDLT<SparseMatrixType> unfactorized;
  verify_determinant_asserts(unfactorized);
}

template <typename T, typename I_, int flag>
void test_simplicial_cholesky_T() {
  typedef SparseMatrix<T, flag, I_> SparseMatrixType;
  SimplicialCholesky<SparseMatrixType, Lower> chol_colmajor_lower_amd;
  SimplicialCholesky<SparseMatrixType, Upper> chol_colmajor_upper_amd;
  SimplicialLLT<SparseMatrixType, Lower> llt_colmajor_lower_amd;
  SimplicialLLT<SparseMatrixType, Upper> llt_colmajor_upper_amd;
  SimplicialLDLT<SparseMatrixType, Lower> ldlt_colmajor_lower_amd;
  SimplicialLDLT<SparseMatrixType, Upper> ldlt_colmajor_upper_amd;
  SimplicialLDLT<SparseMatrixType, Lower, NaturalOrdering<I_> > ldlt_colmajor_lower_nat;
  SimplicialLDLT<SparseMatrixType, Upper, NaturalOrdering<I_> > ldlt_colmajor_upper_nat;
  SimplicialNonHermitianLLT<SparseMatrixType, Lower> nhllt_colmajor_lower_amd;
  SimplicialNonHermitianLLT<SparseMatrixType, Upper> nhllt_colmajor_upper_amd;
  SimplicialNonHermitianLDLT<SparseMatrixType, Lower> nhldlt_colmajor_lower_amd;
  SimplicialNonHermitianLDLT<SparseMatrixType, Upper> nhldlt_colmajor_upper_amd;
  SimplicialNonHermitianLDLT<SparseMatrixType, Lower, NaturalOrdering<I_> > nhldlt_colmajor_lower_nat;
  SimplicialNonHermitianLDLT<SparseMatrixType, Upper, NaturalOrdering<I_> > nhldlt_colmajor_upper_nat;

  check_sparse_spd_solving(chol_colmajor_lower_amd);
  check_sparse_spd_solving(chol_colmajor_upper_amd);
  check_sparse_spd_solving(llt_colmajor_lower_amd);
  check_sparse_spd_solving(llt_colmajor_upper_amd);
  check_sparse_spd_solving(ldlt_colmajor_lower_amd);
  check_sparse_spd_solving(ldlt_colmajor_upper_amd);
  check_sparse_nonhermitian_solving(nhllt_colmajor_lower_amd);
  check_sparse_nonhermitian_solving(nhllt_colmajor_upper_amd);
  check_sparse_nonhermitian_solving(nhldlt_colmajor_lower_amd);
  check_sparse_nonhermitian_solving(nhldlt_colmajor_upper_amd);

  check_sparse_spd_determinant(chol_colmajor_lower_amd);
  check_sparse_spd_determinant(chol_colmajor_upper_amd);
  check_sparse_spd_determinant(llt_colmajor_lower_amd);
  check_sparse_spd_determinant(llt_colmajor_upper_amd);
  check_sparse_spd_determinant(ldlt_colmajor_lower_amd);
  check_sparse_spd_determinant(ldlt_colmajor_upper_amd);
  check_sparse_nonhermitian_determinant(nhllt_colmajor_lower_amd);
  check_sparse_nonhermitian_determinant(nhllt_colmajor_upper_amd);
  check_sparse_nonhermitian_determinant(nhldlt_colmajor_lower_amd);
  check_sparse_nonhermitian_determinant(nhldlt_colmajor_upper_amd);

  check_sparse_spd_log_abs_determinant(llt_colmajor_lower_amd);
  check_sparse_spd_log_abs_determinant(llt_colmajor_upper_amd);
  check_sparse_spd_log_abs_determinant(ldlt_colmajor_lower_amd);
  check_sparse_spd_log_abs_determinant(ldlt_colmajor_upper_amd);
  check_sparse_nonhermitian_log_abs_determinant(nhllt_colmajor_lower_amd);
  check_sparse_nonhermitian_log_abs_determinant(nhllt_colmajor_upper_amd);
  check_sparse_nonhermitian_log_abs_determinant(nhldlt_colmajor_lower_amd);
  check_sparse_nonhermitian_log_abs_determinant(nhldlt_colmajor_upper_amd);

  test_simplicial_cholesky_determinant_asserts<SparseMatrixType>();

  check_sparse_spd_solving(ldlt_colmajor_lower_nat, (std::min)(300, EIGEN_TEST_MAX_SIZE), 1000);
  check_sparse_spd_solving(ldlt_colmajor_upper_nat, (std::min)(300, EIGEN_TEST_MAX_SIZE), 1000);
  check_sparse_nonhermitian_solving(nhldlt_colmajor_lower_nat, (std::min)(300, EIGEN_TEST_MAX_SIZE), 1000);
  check_sparse_nonhermitian_solving(nhldlt_colmajor_upper_nat, (std::min)(300, EIGEN_TEST_MAX_SIZE), 1000);
}

EIGEN_DECLARE_TEST(simplicial_cholesky) {
  CALL_SUBTEST_11((test_simplicial_cholesky_T<double, int, ColMajor>()));
  CALL_SUBTEST_12((test_simplicial_cholesky_T<std::complex<double>, int, ColMajor>()));
  CALL_SUBTEST_13((test_simplicial_cholesky_T<double, long int, ColMajor>()));
  CALL_SUBTEST_21((test_simplicial_cholesky_T<double, int, RowMajor>()));
  CALL_SUBTEST_22((test_simplicial_cholesky_T<std::complex<double>, int, RowMajor>()));
  CALL_SUBTEST_23((test_simplicial_cholesky_T<double, long int, RowMajor>()));
}
