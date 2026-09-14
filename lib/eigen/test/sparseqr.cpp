// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2012 Desire Nuentsa Wakam <desire.nuentsa_wakam@inria.fr>
// Copyright (C) 2014 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// SPDX-License-Identifier: MPL-2.0
#include "sparse.h"
#include <Eigen/SparseQR>

template <typename MatrixType, typename DenseMat>
int generate_sparse_rectangular_problem(MatrixType& A, DenseMat& dA, int maxRows = 300, int maxCols = 150) {
  eigen_assert(maxRows >= maxCols);
  typedef typename MatrixType::Scalar Scalar;
  int rows = internal::random<int>(1, maxRows);
  int cols = internal::random<int>(1, maxCols);
  double density = (std::max)(8. / (rows * cols), 0.01);

  A.resize(rows, cols);
  dA.resize(rows, cols);
  initSparse<Scalar>(density, dA, A, ForceNonZeroDiag);
  A.makeCompressed();
  int nop = internal::random<int>(0, internal::random<double>(0, 1) > 0.5 ? cols / 2 : 0);
  for (int k = 0; k < nop; ++k) {
    int j0 = internal::random<int>(0, cols - 1);
    int j1 = internal::random<int>(0, cols - 1);
    Scalar s = internal::random<Scalar>();
    A.col(j0) = s * A.col(j1);
    dA.col(j0) = s * dA.col(j1);
  }

  //   if(rows<cols) {
  //     A.conservativeResize(cols,cols);
  //     dA.conservativeResize(cols,cols);
  //     dA.bottomRows(cols-rows).setZero();
  //   }

  return rows;
}

template <typename Scalar>
void test_sparseqr_scalar() {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef SparseMatrix<Scalar, ColMajor> MatrixType;
  typedef Matrix<Scalar, Dynamic, Dynamic> DenseMat;
  typedef Matrix<Scalar, Dynamic, 1> DenseVector;
  MatrixType A;
  DenseMat dA;
  DenseVector refX, x, b;
  SparseQR<MatrixType, COLAMDOrdering<int> > solver;
  generate_sparse_rectangular_problem(A, dA);

  b = dA * DenseVector::Random(A.cols());
  solver.compute(A);

  // Q should be MxM
  VERIFY_IS_EQUAL(solver.matrixQ().rows(), A.rows());
  VERIFY_IS_EQUAL(solver.matrixQ().cols(), A.rows());

  // R should be MxN
  VERIFY_IS_EQUAL(solver.matrixR().rows(), A.rows());
  VERIFY_IS_EQUAL(solver.matrixR().cols(), A.cols());

  // Q and R can be multiplied
  DenseMat recoveredA = solver.matrixQ() * DenseMat(solver.matrixR().template triangularView<Upper>()) *
                        solver.colsPermutation().transpose();
  VERIFY_IS_EQUAL(recoveredA.rows(), A.rows());
  VERIFY_IS_EQUAL(recoveredA.cols(), A.cols());

  // and in the full rank case the original matrix is recovered
  if (solver.rank() == A.cols()) {
    VERIFY_IS_APPROX(A, recoveredA);
  }

  if (internal::random<float>(0, 1) > 0.5f)
    solver.factorize(A);  // this checks that calling analyzePattern is not needed if the pattern do not change.
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

  // Compare with a dense QR solver
  ColPivHouseholderQR<DenseMat> dqr(dA);
  refX = dqr.solve(b);

  bool rank_deficient = A.cols() > A.rows() || dqr.rank() < A.cols();
  if (rank_deficient) {
    // rank deficient problem -> we might have to increase the threshold
    // to get a correct solution.
    RealScalar th =
        RealScalar(20) * dA.colwise().norm().maxCoeff() * (A.rows() + A.cols()) * NumTraits<RealScalar>::epsilon();
    for (Index k = 0; (k < 16) && !test_isApprox(A * x, b); ++k) {
      th *= RealScalar(10);
      solver.setPivotThreshold(th);
      solver.compute(A);
      x = solver.solve(b);
    }
  }

  VERIFY_IS_APPROX(A * x, b);

  // For rank deficient problem, the estimated rank might
  // be slightly off, so let's only raise a warning in such cases.
  if (rank_deficient) ++g_test_level;
  VERIFY_IS_EQUAL(solver.rank(), dqr.rank());
  if (rank_deficient) --g_test_level;

  if (solver.rank() == A.cols())  // full rank
    VERIFY_IS_APPROX(x, refX);
  //   else
  //     VERIFY((dA * refX - b).norm() * 2 > (A * x - b).norm() );

  // Compute explicitly the matrix Q
  MatrixType Q, QtQ, idM;
  Q = solver.matrixQ();
  // Check  ||Q' * Q - I ||
  QtQ = Q * Q.adjoint();
  idM.resize(Q.rows(), Q.rows());
  idM.setIdentity();
  VERIFY(idM.isApprox(QtQ));

  // Q to dense
  DenseMat dQ;
  dQ = solver.matrixQ();
  VERIFY_IS_APPROX(Q, dQ);
}

void test_sparseqr_factorize_uncompressed_input() {
  typedef SparseMatrix<double, ColMajor> MatrixType;
  typedef VectorXd Vector;
  typedef SparseQR<MatrixType, NaturalOrdering<int> > Solver;

  MatrixType uncompressed(2, 2);
  VectorXi reserve(2);
  reserve << 2, 3;
  uncompressed.reserve(reserve);
  uncompressed.insert(0, 0) = 1.0;
  uncompressed.insert(0, 1) = 0.5;
  uncompressed.insert(1, 1) = 1.0;

  // Poison inactive capacity so factorize() must ignore unused slots in the
  // uncompressed input instead of treating reserved space as structural nnz.
  uncompressed.innerIndexPtr()[1] = 1;
  uncompressed.valuePtr()[1] = 7.0;

  MatrixType compressed = uncompressed;
  compressed.makeCompressed();

  Vector b(2);
  b << 1.0, 2.0;
  Vector expected(2);
  expected << 0.0, 2.0;

  Solver compressed_solver;
  compressed_solver.compute(compressed);
  VERIFY_IS_EQUAL(compressed_solver.info(), Success);
  VERIFY_IS_APPROX(compressed_solver.solve(b), expected);

  Solver two_step_solver;
  two_step_solver.analyzePattern(compressed);
  two_step_solver.factorize(uncompressed);
  VERIFY_IS_EQUAL(two_step_solver.info(), Success);
  VERIFY_IS_APPROX(two_step_solver.solve(b), expected);
}

template <typename OrderingType, typename DenseMat, typename DenseRhs>
void verify_sparseqr_solves(const DenseMat& denseA, const DenseRhs& b, Index expectedRank) {
  typedef typename DenseMat::Scalar Scalar;
  typedef SparseMatrix<Scalar, ColMajor> MatrixType;
  typedef Matrix<Scalar, Dynamic, 1> SolutionType;

  MatrixType A = denseA.sparseView();
  A.makeCompressed();

  SparseQR<MatrixType, OrderingType> solver(A);
  VERIFY_IS_EQUAL(solver.info(), Success);
  VERIFY_IS_EQUAL(solver.rank(), expectedRank);
  VERIFY(!solver.lastPivotLookAheadSkipped());

  SolutionType x = solver.solve(b);
  VERIFY_IS_EQUAL(solver.info(), Success);
  VERIFY_IS_APPROX(denseA * x, b);
}

void test_sparseqr_lookahead_rejects_replaceable_weak_pivot() {
  Matrix<double, 4, 5> denseA;
  denseA << 10.875, 0.0, 0.0, 0.0, 0.0, -0.397597, 12.1403, 0.0, 0.0, 0.0317254, -0.851737, -0.0269339, 11.3113,
      0.0130592, 0.0, -0.676106, 0.0, 0.138752, 8.57745, 0.0;
  Matrix<double, 4, 1> b;
  b << 10.3612, -2.27836, -10.3179, -7.49344;

  verify_sparseqr_solves<COLAMDOrdering<int> >(denseA, b, 4);
}

void test_sparseqr_tiny_independent_column() {
  Matrix<double, 2, 3> denseA;
  Matrix<double, 2, 1> b;

  denseA << 0.5, 0.0, 0.0, 0.0, 1e12, 0.0;
  b << 0.5, 0.0;
  verify_sparseqr_solves<COLAMDOrdering<int> >(denseA, b, 2);

  denseA << 1e12, 1e12, 0.0, 0.0, 0.5, 0.0;
  b << 0.0, 0.5;
  verify_sparseqr_solves<COLAMDOrdering<int> >(denseA, b, 2);
}

void test_sparseqr_lookahead_preserves_needed_weak_direction() {
  Matrix<double, 3, 4> denseA;
  Matrix<double, 3, 1> b;

  denseA << 1.0, 1.0, 0.0, 0.0, 0.0, 5e-13, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
  b << 0.0, 5.0, 0.0;
  verify_sparseqr_solves<NaturalOrdering<int> >(denseA, b, 3);

  denseA << 1.0, 1.0, 0.0, 0.0, 0.0, 5e-13, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0;
  b << 0.0, 5.0, 0.0;
  verify_sparseqr_solves<NaturalOrdering<int> >(denseA, b, 3);
}

void test_sparseqr_explicit_threshold_disables_lookahead() {
  typedef SparseMatrix<double, ColMajor> MatrixType;
  typedef SparseQR<MatrixType, NaturalOrdering<int> > Solver;

  Matrix<double, 2, 3> denseA;
  const double weak_pivot = 5e-13;
  denseA << 1.0, 1.0, 0.0, 0.0, weak_pivot, 1.0;

  MatrixType A = denseA.sparseView();
  A.makeCompressed();

  Solver default_solver(A);
  VERIFY_IS_EQUAL(default_solver.info(), Success);
  VERIFY_IS_EQUAL(default_solver.rank(), 2);
  VERIFY(!default_solver.lastPivotLookAheadSkipped());

  const Solver::PermutationType::IndicesType& defaultPerm = default_solver.colsPermutation().indices();
  VERIFY_IS_EQUAL(defaultPerm.size(), 3);
  for (Index j = 0; j < defaultPerm.size(); ++j) {
    VERIFY(defaultPerm(j) >= 0);
    VERIFY(defaultPerm(j) < defaultPerm.size());
    for (Index k = j + 1; k < defaultPerm.size(); ++k) {
      VERIFY_IS_NOT_EQUAL(defaultPerm(j), defaultPerm(k));
    }
  }
  Matrix<double, 2, 1> b;
  b << 1.0, 1.0;
  VectorXd x = default_solver.solve(b);
  VERIFY_IS_EQUAL(default_solver.info(), Success);
  VERIFY_IS_APPROX(denseA * x, b);

  Solver explicit_threshold_solver;
  explicit_threshold_solver.setPivotThreshold(1e-14);
  explicit_threshold_solver.compute(A);
  VERIFY_IS_EQUAL(explicit_threshold_solver.info(), Success);
  VERIFY_IS_EQUAL(explicit_threshold_solver.rank(), 2);
  VERIFY(!explicit_threshold_solver.lastPivotLookAheadSkipped());
  VERIFY_IS_EQUAL(explicit_threshold_solver.colsPermutation().indices()(0), 0);
  VERIFY_IS_EQUAL(explicit_threshold_solver.colsPermutation().indices()(1), 1);
  VERIFY_IS_EQUAL(explicit_threshold_solver.colsPermutation().indices()(2), 2);
}

template <typename Scalar>
void test_sparseqr_16bit_default_threshold(int n, float tol) {
  // Factorize a well-conditioned, full-rank, diagonally dominant tridiagonal
  // (4 on the diagonal, -1 off-diagonal) system with the default pivot
  // threshold. The default threshold 20*(m+n)*max2Norm*epsilon used to be
  // computed entirely in the 16-bit scalar type, where 20*(m+n)*epsilon
  // exceeds 1 for m+n as small as 52 (half) or 7 (bfloat16) -- so every pivot
  // was rejected and the reported rank collapsed to 0 with info()==Success.
  typedef SparseMatrix<Scalar, ColMajor> MatrixType;
  typedef Matrix<Scalar, Dynamic, 1> DenseVector;

  MatrixType A(n, n);
  A.reserve(VectorXi::Constant(n, 3));
  for (int j = 0; j < n; ++j) {
    if (j > 0) A.insert(j - 1, j) = Scalar(-1);
    A.insert(j, j) = Scalar(4);
    if (j + 1 < n) A.insert(j + 1, j) = Scalar(-1);
  }
  A.makeCompressed();

  SparseQR<MatrixType, COLAMDOrdering<int> > solver(A);
  VERIFY_IS_EQUAL(solver.info(), Success);
  VERIFY_IS_EQUAL(solver.rank(), Index(n));

  const DenseVector b = A * DenseVector::Ones(n);
  const DenseVector x = solver.solve(b);
  VERIFY_IS_EQUAL(solver.info(), Success);
  // Check the relative residual in float: 16-bit norms of length-n vectors are
  // unreliable. A is diagonally dominant with cond(A) <= 3, so a small
  // residual also implies an accurate solution.
  const VectorXf residual = (A * x - b).template cast<float>();
  VERIFY(residual.norm() <= tol * b.template cast<float>().norm());
}

EIGEN_DECLARE_TEST(sparseqr) {
  for (int i = 0; i < g_repeat; ++i) {
    CALL_SUBTEST_1(test_sparseqr_scalar<double>());
    CALL_SUBTEST_2(test_sparseqr_scalar<std::complex<double> >());
  }
  CALL_SUBTEST_3(test_sparseqr_factorize_uncompressed_input());
  CALL_SUBTEST_4(test_sparseqr_lookahead_rejects_replaceable_weak_pivot());
  CALL_SUBTEST_5(test_sparseqr_tiny_independent_column());
  CALL_SUBTEST_6(test_sparseqr_explicit_threshold_disables_lookahead());
  CALL_SUBTEST_7(test_sparseqr_lookahead_preserves_needed_weak_direction());
  // The relative residual is a small multiple of the scalar's epsilon; bfloat16's coarser mantissa (7 bits vs
  // half's 10) loses more per operation, so it needs the larger factor despite the smaller system.
  CALL_SUBTEST_8(
      test_sparseqr_16bit_default_threshold<Eigen::half>(400, static_cast<float>(NumTraits<Eigen::half>::epsilon())));
  CALL_SUBTEST_8(test_sparseqr_16bit_default_threshold<Eigen::bfloat16>(
      64, 4 * static_cast<float>(NumTraits<Eigen::bfloat16>::epsilon())));
}
