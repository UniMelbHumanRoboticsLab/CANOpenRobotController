// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2012 Desire Nuentsa Wakam <desire.nuentsa_wakam@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// SPDX-License-Identifier: MPL-2.0

#define EIGEN_NO_DEBUG_SMALL_PRODUCT_BLOCKS
#include "sparse.h"
#include <Eigen/SPQRSupport>

template <typename MatrixType, typename DenseMat>
int generate_sparse_rectangular_problem(MatrixType& A, DenseMat& dA, int maxRows = 300, int maxCols = 300) {
  eigen_assert(maxRows >= maxCols);
  typedef typename MatrixType::Scalar Scalar;
  int rows = internal::random<int>(1, maxRows);
  int cols = internal::random<int>(1, rows);
  double density = (std::max)(8. / (rows * cols), 0.01);

  A.resize(rows, cols);
  dA.resize(rows, cols);
  initSparse<Scalar>(density, dA, A, ForceNonZeroDiag);
  A.makeCompressed();
  return rows;
}

template <typename Scalar>
void test_spqr_scalar() {
  typedef SparseMatrix<Scalar, ColMajor> MatrixType;
  MatrixType A;
  Matrix<Scalar, Dynamic, Dynamic> dA;
  typedef Matrix<Scalar, Dynamic, 1> DenseVector;
  DenseVector refX, x, b;
  SPQR<MatrixType> solver;
  generate_sparse_rectangular_problem(A, dA);

  Index m = A.rows();
  b = DenseVector::Random(m);
  solver.compute(A);
  if (solver.info() != Success) {
    std::cerr << "sparse QR factorization failed\n";
    exit(0);
    return;
  }
  x = solver.solve(b);
  if (solver.info() != Success) {
    std::cerr << "sparse QR factorization failed\n";
    exit(0);
    return;
  }
  // Compare with a dense solver
  refX = dA.colPivHouseholderQr().solve(b);
  VERIFY(x.isApprox(refX, test_precision<Scalar>()));
}

void test_spqr_fixed_ordering_uses_identity_permutation() {
  typedef SparseMatrix<double, ColMajor> MatrixType;
  typedef Matrix<double, Dynamic, Dynamic> DenseMatrix;
  typedef Matrix<double, Dynamic, 1> DenseVector;

  DenseMatrix dA(6, 4);
  dA << 4.0, 1.0, 0.0, 0.0,  //
      1.0, 0.0, 2.0, 0.0,    //
      -2.0, 3.0, 0.0, 6.0,   //
      0.0, 5.0, -1.0, 0.0,   //
      0.0, 0.0, 7.0, 2.0,    //
      0.0, 0.0, 0.0, 3.0;

  MatrixType A = dA.sparseView();
  A.makeCompressed();

  DenseVector b(6);
  b << 1.0, -2.0, 0.5, 4.0, -1.0, 3.0;

  SPQR<MatrixType> solver;
  solver.setSPQROrdering(SPQR_ORDERING_FIXED);
  solver.setPivotThreshold(SPQR_NO_TOL);
  solver.compute(A);

  VERIFY_IS_EQUAL(solver.info(), Success);
  VERIFY_IS_EQUAL(solver.rank(), A.cols());

  const auto permutation = solver.colsPermutation();
  VERIFY_IS_EQUAL(permutation.size(), A.cols());
  for (Index i = 0; i < permutation.size(); ++i) {
    VERIFY_IS_EQUAL(permutation.indices()(i), i);
  }

  const DenseVector refX = dA.colPivHouseholderQr().solve(b);
  DenseVector x = solver.solve(b);
  VERIFY_IS_EQUAL(solver.info(), Success);
  VERIFY_IS_APPROX(x, refX);
}

void test_spqr_matrix_q_times_identity_expression() {
  typedef SparseMatrix<double, ColMajor> MatrixType;
  typedef Matrix<double, Dynamic, Dynamic> DenseMatrix;
  typedef SPQR<MatrixType> SolverType;
  typedef typename SolverType::MatrixType SolverSparseMatrix;
  typedef Matrix<double, Dynamic, 1> DenseVector;

  DenseMatrix dA(6, 4);
  dA << 4.0, 1.0, 0.0, 0.0,  //
      1.0, 0.0, 2.0, 0.0,    //
      -2.0, 3.0, 0.0, 6.0,   //
      0.0, 5.0, -1.0, 0.0,   //
      0.0, 0.0, 7.0, 2.0,    //
      0.0, 0.0, 0.0, 3.0;

  MatrixType A = dA.sparseView();
  A.makeCompressed();

  SolverType solver;
  solver.compute(A);

  VERIFY_IS_EQUAL(solver.info(), Success);

  const DenseMatrix Q = solver.matrixQ() * DenseMatrix::Identity(A.rows(), A.rows());
  const DenseMatrix denseIdentity = DenseMatrix::Identity(A.rows(), A.rows());
  const auto qProduct = solver.matrixQ() * denseIdentity;
  VERIFY_IS_EQUAL(qProduct.rows(), A.rows());
  VERIFY_IS_EQUAL(qProduct.cols(), A.rows());
  const DenseMatrix qFromDenseIdentity = qProduct;
  DenseMatrix denseAssignedQ;
  denseAssignedQ = solver.matrixQ();
  VERIFY_IS_EQUAL(Q.rows(), A.rows());
  VERIFY_IS_EQUAL(Q.cols(), A.rows());
  VERIFY_IS_APPROX(Q.transpose() * Q, DenseMatrix::Identity(A.rows(), A.rows()));
  VERIFY_IS_APPROX(qFromDenseIdentity, Q);
  VERIFY_IS_APPROX(denseAssignedQ, Q);

  const DenseMatrix R = DenseMatrix(solver.matrixR().template triangularView<Upper>());
  const auto sparseR = solver.matrixR().template triangularView<Upper>();
  SolverSparseMatrix sparseIdentity(A.rows(), A.rows());
  sparseIdentity.setIdentity();
  SolverSparseMatrix sparseAssignedQ(A.rows(), A.rows());
  sparseAssignedQ = solver.matrixQ();
  SolverSparseMatrix sparseProductQ(A.rows(), A.rows());
  sparseProductQ = solver.matrixQ() * sparseIdentity;
  const DenseVector rhs = DenseVector::LinSpaced(A.cols(), 1.0, double(A.cols()));
  const DenseVector x = sparseR.solve(rhs);
  const DenseVector expected = R.template triangularView<Upper>().solve(rhs);
  const DenseMatrix recoveredA = Q.leftCols(A.cols()) * R * solver.colsPermutation().transpose();
  VERIFY_IS_APPROX(x, expected);
  VERIFY_IS_APPROX(DenseMatrix(sparseAssignedQ), denseAssignedQ);
  VERIFY_IS_APPROX(DenseMatrix(sparseProductQ), denseAssignedQ);
  VERIFY_IS_APPROX(recoveredA, dA);
}

EIGEN_DECLARE_TEST(spqr_support) {
  CALL_SUBTEST_1(test_spqr_scalar<double>());
  CALL_SUBTEST_2(test_spqr_scalar<std::complex<double> >());
  CALL_SUBTEST_3(test_spqr_fixed_ordering_uses_identity_permutation());
  CALL_SUBTEST_3(test_spqr_matrix_q_times_identity_expression());
}
