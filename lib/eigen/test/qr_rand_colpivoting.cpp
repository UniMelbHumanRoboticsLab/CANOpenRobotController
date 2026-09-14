// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include "main.h"
#include <Eigen/QR>
#include <Eigen/SVD>
#include "solverbase.h"

// Use a small fixed block size in the tests so the blocked path actually
// triggers on the modest matrix sizes the unit tests exercise.
template <typename QRType>
void configure_small(QRType& qr) {
  qr.setBlockSize(4).setOversampling(2);
}

template <typename MatrixType>
void rqr() {
  using std::sqrt;

  Index rows = internal::random<Index>(2, EIGEN_TEST_MAX_SIZE), cols = internal::random<Index>(2, EIGEN_TEST_MAX_SIZE),
        cols2 = internal::random<Index>(2, EIGEN_TEST_MAX_SIZE);
  Index rank = internal::random<Index>(1, (std::min)(rows, cols) - 1);

  typedef typename MatrixType::Scalar Scalar;
  typedef typename MatrixType::RealScalar RealScalar;
  typedef Matrix<Scalar, MatrixType::RowsAtCompileTime, MatrixType::RowsAtCompileTime> MatrixQType;
  MatrixType m1;
  createRandomPIMatrixOfRank(rank, rows, cols, m1);
  RandColPivHouseholderQR<MatrixType> qr;
  configure_small(qr);
  qr.compute(m1);
  VERIFY_IS_EQUAL(rank, qr.rank());
  VERIFY_IS_EQUAL(cols - qr.rank(), qr.dimensionOfKernel());
  VERIFY(!qr.isInjective());
  VERIFY(!qr.isInvertible());
  VERIFY(!qr.isSurjective());

  MatrixQType q = qr.householderQ();
  VERIFY_IS_UNITARY(q);

  MatrixType r = qr.matrixQR().template triangularView<Upper>();
  MatrixType c = q * r * qr.colsPermutation().inverse();
  VERIFY_IS_APPROX(m1, c);

  // BQRRP-style randomized QRCP picks pivots on a Gaussian sketch with
  // partial-pivoted LU and then runs *unpivoted* QR on the chosen
  // panel; the diagonal of R is therefore not strictly monotonic, even
  // within a block. Verify the weaker rank-revealing property: every
  // R(i, i) above the rank threshold dominates the smallest later
  // diagonal entry by at most an O(sqrt(rows)) slack factor.
  RealScalar threshold = sqrt(RealScalar(rows)) * numext::abs(r(0, 0)) * NumTraits<Scalar>::epsilon();
  RealScalar slack = sqrt(RealScalar(rows));
  for (Index i = 0; i < (std::min)(rows, cols) - 1; ++i) {
    RealScalar x = numext::abs(r(i, i));
    RealScalar y_min = x;
    for (Index j = i + 1; j < (std::min)(rows, cols); ++j) y_min = (std::min)(y_min, numext::abs(r(j, j)));
    if (x < threshold && y_min < threshold) continue;
    VERIFY_IS_APPROX_OR_LESS_THAN(y_min, slack * x);
  }

  check_solverbase<MatrixType, MatrixType>(m1, qr, rows, cols, cols2);

  {
    MatrixType m2, m3;
    Index size = rows;
    m1 = MatrixType::Random(size, size);
    m1.diagonal().array() += Scalar(2 * size);
    qr.compute(m1);
    MatrixType m1_inv = qr.inverse();
    m3 = m1 * MatrixType::Random(size, cols2);
    m2 = qr.solve(m3);
    VERIFY_IS_APPROX(m2, m1_inv * m3);
  }
}

template <typename MatrixType, int Cols2>
void rqr_fixedsize() {
  enum { Rows = MatrixType::RowsAtCompileTime, Cols = MatrixType::ColsAtCompileTime };
  typedef typename MatrixType::Scalar Scalar;
  int rank = internal::random<int>(1, (std::min)(int(Rows), int(Cols)) - 1);
  MatrixType m1;
  createRandomPIMatrixOfRank(rank, Rows, Cols, m1);
  RandColPivHouseholderQR<MatrixType> qr;
  configure_small(qr);
  qr.compute(m1);
  VERIFY_IS_EQUAL(rank, qr.rank());
  VERIFY_IS_EQUAL(Cols - qr.rank(), qr.dimensionOfKernel());
  VERIFY_IS_EQUAL(qr.isInjective(), (rank == Rows));
  VERIFY_IS_EQUAL(qr.isSurjective(), (rank == Cols));
  VERIFY_IS_EQUAL(qr.isInvertible(), (qr.isInjective() && qr.isSurjective()));

  MatrixType r = qr.matrixQR().template triangularView<Upper>();
  MatrixType c = qr.householderQ() * r * qr.colsPermutation().inverse();
  VERIFY_IS_APPROX(m1, c);

  check_solverbase<Matrix<Scalar, Cols, Cols2>, Matrix<Scalar, Rows, Cols2>>(m1, qr, Rows, Cols, Cols2);
}

// Round-trip verification on the Kahan matrix (the canonical
// counter-example for naive QR pivoting). The randomized strategy does
// not promise diagonal monotonicity across block boundaries, so we check
// reconstruction only.
template <typename MatrixType>
void rqr_kahan_matrix() {
  using std::sqrt;
  typedef typename MatrixType::RealScalar RealScalar;

  Index rows = 200, cols = rows;

  MatrixType m1;
  m1.setZero(rows, cols);
  RealScalar s = std::pow(NumTraits<RealScalar>::epsilon(), 1.0 / rows);
  RealScalar c_kahan = std::sqrt(1 - s * s);
  RealScalar pow_s_i(1.0);
  for (Index i = 0; i < rows; ++i) {
    m1(i, i) = pow_s_i;
    m1.row(i).tail(rows - i - 1) = -pow_s_i * c_kahan * MatrixType::Ones(1, rows - i - 1);
    pow_s_i *= s;
  }
  m1 = (m1 + m1.transpose()).eval();

  RandColPivHouseholderQR<MatrixType> qr;
  configure_small(qr);
  qr.setSeed(0xC0FFEE);  // fix seed for reproducibility
  qr.compute(m1);
  MatrixType r = qr.matrixQR().template triangularView<Upper>();

  // Reconstruction round-trip is the strongest correctness check.
  MatrixType q = qr.householderQ();
  MatrixType reconstructed = q * r * qr.colsPermutation().inverse();
  VERIFY_IS_APPROX(m1, reconstructed);
}

template <typename MatrixType>
void rqr_invertible() {
  using std::abs;
  using std::log;
  typedef typename NumTraits<typename MatrixType::Scalar>::Real RealScalar;
  typedef typename MatrixType::Scalar Scalar;

  int size = internal::random<int>(10, 50);

  MatrixType m1(size, size), m2(size, size), m3(size, size);
  m1 = MatrixType::Random(size, size);

  if (std::is_same<RealScalar, float>::value) {
    MatrixType a = MatrixType::Random(size, size * 2);
    m1 += a * a.adjoint();
  }

  RandColPivHouseholderQR<MatrixType> qr;
  configure_small(qr);
  qr.compute(m1);

  check_solverbase<MatrixType, MatrixType>(m1, qr, size, size, size);

  // Now construct a matrix with prescribed determinant and verify det/sign.
  m1.setZero();
  setRandomWellConditionedDiagonal(m1);
  Scalar det = m1.diagonal().prod();
  RealScalar absdet = abs(det);
  m3 = qr.householderQ();
  m1 = m3 * m1 * m3.adjoint();
  qr.compute(m1);
  VERIFY_IS_APPROX(det, qr.determinant());
  VERIFY_IS_APPROX(absdet, qr.absDeterminant());
  VERIFY_IS_APPROX(log(absdet), qr.logAbsDeterminant());
  VERIFY_IS_APPROX(numext::sign(det), qr.signDeterminant());
}

template <typename MatrixType>
void rqr_verify_assert() {
  MatrixType tmp;

  RandColPivHouseholderQR<MatrixType> qr;
  VERIFY_RAISES_ASSERT(qr.matrixQR())
  VERIFY_RAISES_ASSERT(qr.solve(tmp))
  VERIFY_RAISES_ASSERT(qr.transpose().solve(tmp))
  VERIFY_RAISES_ASSERT(qr.adjoint().solve(tmp))
  VERIFY_RAISES_ASSERT(qr.householderQ())
  VERIFY_RAISES_ASSERT(qr.dimensionOfKernel())
  VERIFY_RAISES_ASSERT(qr.isInjective())
  VERIFY_RAISES_ASSERT(qr.isSurjective())
  VERIFY_RAISES_ASSERT(qr.isInvertible())
  VERIFY_RAISES_ASSERT(qr.inverse())
  VERIFY_RAISES_ASSERT(qr.determinant())
  VERIFY_RAISES_ASSERT(qr.absDeterminant())
  VERIFY_RAISES_ASSERT(qr.logAbsDeterminant())
  VERIFY_RAISES_ASSERT(qr.signDeterminant())
}

// Regression: compute() on degenerate empty inputs (0-row or 0-col) must
// not trigger integer division-by-zero in the auto-block-size heuristic
// (UBSan caught one on MatrixXd(0, 3) at MR SHA c2860f27). The expected
// behavior is that the unblocked path runs with no work and rank() == 0.
template <typename MatrixType>
void rqr_empty_input() {
  const std::pair<Index, Index> cases[] = {{0, 3}, {3, 0}, {0, 0}};
  for (const auto& dims : cases) {
    const Index rows = dims.first;
    const Index cols = dims.second;
    MatrixType a(rows, cols);
    RandColPivHouseholderQR<MatrixType> qr;
    qr.compute(a);
    VERIFY_IS_EQUAL(qr.rank(), Index(0));
    VERIFY_IS_EQUAL(qr.rows(), rows);
    VERIFY_IS_EQUAL(qr.cols(), cols);
  }
}

template <typename MatrixType>
void rcod() {
  Index rows = internal::random<Index>(2, EIGEN_TEST_MAX_SIZE);
  Index cols = internal::random<Index>(2, EIGEN_TEST_MAX_SIZE);
  Index cols2 = internal::random<Index>(2, EIGEN_TEST_MAX_SIZE);
  Index rank = internal::random<Index>(1, (std::min)(rows, cols) - 1);

  typedef typename MatrixType::Scalar Scalar;
  typedef typename MatrixType::RealScalar RealScalar;
  typedef Matrix<Scalar, MatrixType::RowsAtCompileTime, MatrixType::RowsAtCompileTime> MatrixQType;
  MatrixType matrix;
  createRandomPIMatrixOfRank(rank, rows, cols, matrix);
  RandCompleteOrthogonalDecomposition<MatrixType> cod;
  cod.setBlockSize(4).setSeed(0x5eed);
  cod.compute(matrix);
  VERIFY(rank == cod.rank());
  VERIFY(cols - cod.rank() == cod.dimensionOfKernel());
  VERIFY(!cod.isInjective());
  VERIFY(!cod.isInvertible());
  VERIFY(!cod.isSurjective());

  MatrixQType q = cod.householderQ();
  VERIFY_IS_UNITARY(q);

  MatrixType z = cod.matrixZ();
  VERIFY_IS_UNITARY(z);

  MatrixType t;
  t.setZero(rows, cols);
  t.topLeftCorner(rank, rank) = cod.matrixT().topLeftCorner(rank, rank).template triangularView<Upper>();

  MatrixType c = q * t * z * cod.colsPermutation().inverse();
  VERIFY_IS_APPROX(matrix, c);

  check_solverbase<MatrixType, MatrixType>(matrix, cod, rows, cols, cols2);

  MatrixType exact_solution = MatrixType::Random(cols, cols2);
  MatrixType rhs = matrix * exact_solution;
  MatrixType cod_solution = cod.solve(rhs);
  JacobiSVD<MatrixType, ComputeThinU | ComputeThinV> svd(matrix);
  MatrixType svd_solution = svd.solve(rhs);
  VERIFY_IS_APPROX(cod_solution, svd_solution);

  MatrixType pinv = cod.pseudoInverse();
  VERIFY_IS_APPROX(cod_solution, pinv * rhs);

  Index size = internal::random<Index>(2, 20);
  matrix.setZero(size, size);
  setRandomWellConditionedDiagonal(matrix);
  Scalar det = matrix.diagonal().prod();
  RealScalar absdet = numext::abs(det);
  RandCompleteOrthogonalDecomposition<MatrixType> cod2;
  cod2.setBlockSize(4).setSeed(0xc0d2);
  cod2.compute(matrix);
  q = cod2.householderQ();
  matrix = q * matrix * q.adjoint();
  cod2.compute(matrix);
  VERIFY_IS_APPROX(det, cod2.determinant());
  VERIFY_IS_APPROX(absdet, cod2.absDeterminant());
  VERIFY_IS_APPROX(numext::log(absdet), cod2.logAbsDeterminant());
  VERIFY_IS_APPROX(numext::sign(det), cod2.signDeterminant());
}

template <typename MatrixType, int Cols2>
void rcod_fixedsize() {
  enum { Rows = MatrixType::RowsAtCompileTime, Cols = MatrixType::ColsAtCompileTime };
  typedef typename MatrixType::Scalar Scalar;
  typedef RandCompleteOrthogonalDecomposition<MatrixType> COD;
  int rank = internal::random<int>(1, (std::min)(int(Rows), int(Cols)) - 1);
  MatrixType matrix;
  createRandomPIMatrixOfRank(rank, Rows, Cols, matrix);
  COD cod;
  cod.setBlockSize(4).setSeed(0xfed1);
  cod.compute(matrix);
  VERIFY(rank == cod.rank());
  VERIFY(Cols - cod.rank() == cod.dimensionOfKernel());
  VERIFY(cod.isInjective() == (rank == Rows));
  VERIFY(cod.isSurjective() == (rank == Cols));
  VERIFY(cod.isInvertible() == (cod.isInjective() && cod.isSurjective()));

  check_solverbase<Matrix<Scalar, Cols, Cols2>, Matrix<Scalar, Rows, Cols2>>(matrix, cod, Rows, Cols, Cols2);

  Matrix<Scalar, Cols, Cols2> exact_solution;
  exact_solution.setRandom(Cols, Cols2);
  Matrix<Scalar, Rows, Cols2> rhs = matrix * exact_solution;
  Matrix<Scalar, Cols, Cols2> cod_solution = cod.solve(rhs);
  JacobiSVD<MatrixType, ComputeFullU | ComputeFullV> svd(matrix);
  Matrix<Scalar, Cols, Cols2> svd_solution = svd.solve(rhs);
  VERIFY_IS_APPROX(cod_solution, svd_solution);

  typename Inverse<COD>::PlainObject pinv = cod.pseudoInverse();
  VERIFY_IS_APPROX(cod_solution, pinv * rhs);
}

template <typename MatrixType>
void rcod_verify_assert() {
  MatrixType tmp;

  RandCompleteOrthogonalDecomposition<MatrixType> cod;
  VERIFY_RAISES_ASSERT(cod.matrixQTZ())
  VERIFY_RAISES_ASSERT(cod.solve(tmp))
  VERIFY_RAISES_ASSERT(cod.transpose().solve(tmp))
  VERIFY_RAISES_ASSERT(cod.adjoint().solve(tmp))
  VERIFY_RAISES_ASSERT(cod.householderQ())
  VERIFY_RAISES_ASSERT(cod.dimensionOfKernel())
  VERIFY_RAISES_ASSERT(cod.isInjective())
  VERIFY_RAISES_ASSERT(cod.isSurjective())
  VERIFY_RAISES_ASSERT(cod.isInvertible())
  VERIFY_RAISES_ASSERT(cod.pseudoInverse())
  VERIFY_RAISES_ASSERT(cod.determinant())
  VERIFY_RAISES_ASSERT(cod.absDeterminant())
  VERIFY_RAISES_ASSERT(cod.logAbsDeterminant())
  VERIFY_RAISES_ASSERT(cod.signDeterminant())
}

// Exercise the blocked path by using a matrix sufficiently larger than 2*b.
// At b=4 we need at least 8 columns to enter the blocked branch.
// Stress the rank-detection threshold under the blocked path. With b=4
// we run multiple blocks; the rank-revealing column lands at index 8, 12,
// or 20 — i.e. in the third block or later. A panel-local threshold
// would deflate as norms shrink across blocks and could miscount the
// rank. Only a global threshold (precomputed from the original column
// norms, as ColPivHouseholderQR does) is correct.
template <typename MatrixType>
void rqr_rank_in_late_block() {
  const Index rows = 30;
  const Index cols = 24;
  for (Index target_rank : {Index(8), Index(12), Index(20)}) {
    MatrixType m1;
    createRandomPIMatrixOfRank(target_rank, rows, cols, m1);

    RandColPivHouseholderQR<MatrixType> qr;
    configure_small(qr);
    qr.compute(m1);
    VERIFY_IS_EQUAL(target_rank, qr.rank());

    MatrixType r = qr.matrixQR().template triangularView<Upper>();
    MatrixType q = qr.householderQ();
    VERIFY_IS_APPROX(m1, MatrixType(q * r * qr.colsPermutation().inverse()));

    // Cross-check against classical column pivoting on the same input.
    ColPivHouseholderQR<MatrixType> cpqr(m1);
    VERIFY_IS_EQUAL(qr.rank(), cpqr.rank());
  }
}

template <typename MatrixType>
void rqr_blocked_path() {
  Index rows = 40, cols = 30;
  MatrixType m1 = MatrixType::Random(rows, cols);
  RandColPivHouseholderQR<MatrixType> qr;
  configure_small(qr);
  qr.setSeed(42);
  qr.compute(m1);

  MatrixType r = qr.matrixQR().template triangularView<Upper>();
  MatrixType q = qr.householderQ();
  VERIFY_IS_UNITARY(q);
  MatrixType reconstructed = q * r * qr.colsPermutation().inverse();
  VERIFY_IS_APPROX(m1, reconstructed);

  // Same input + same seed must reproduce the exact same factorization.
  RandColPivHouseholderQR<MatrixType> qr2;
  configure_small(qr2);
  qr2.setSeed(42);
  qr2.compute(m1);
  VERIFY_IS_EQUAL(qr.colsPermutation().indices(), qr2.colsPermutation().indices());
  VERIFY_IS_APPROX(qr.matrixQR(), qr2.matrixQR());
}

// Mirrors qr_rank_detection_stress from qr_colpivoting.cpp: many random
// partial-isometry trials across aspect ratios. With configure_small (b=4)
// most of these matrices engage the blocked path.
template <typename MatrixType>
void rqr_rank_detection_stress() {
  const Index sizes[][2] = {{10, 10}, {20, 20}, {50, 50}, {100, 100}, {40, 10}, {100, 10}, {10, 40}, {10, 100}};
  for (const auto& sz : sizes) {
    const Index rows = sz[0], cols = sz[1];
    const Index min_dim = (std::min)(rows, cols);
    for (Index rank : {Index(1), (std::max)(Index(1), min_dim / 2), min_dim - 1}) {
      if (rank >= min_dim) continue;
      for (int trial = 0; trial < 10; ++trial) {
        MatrixType m1;
        createRandomPIMatrixOfRank(rank, rows, cols, m1);
        RandColPivHouseholderQR<MatrixType> qr;
        configure_small(qr);
        qr.compute(m1);
        VERIFY_IS_EQUAL(rank, qr.rank());
      }
    }
  }
}

// Mirrors qr_threshold_efficiency: matrices with smallest SV well above
// the rank-detection threshold must come back as full-rank.
template <typename MatrixType>
void rqr_threshold_efficiency() {
  typedef typename MatrixType::RealScalar RealScalar;
  typedef Matrix<RealScalar, Dynamic, 1> RealVectorType;
  const Index sizes[][2] = {{10, 10}, {50, 50}, {100, 100}, {40, 10}, {10, 40}};
  for (const auto& sz : sizes) {
    const Index rows = sz[0], cols = sz[1];
    const Index min_dim = (std::min)(rows, cols);
    RealScalar sigma_min = RealScalar(400) * RealScalar(min_dim) * NumTraits<RealScalar>::epsilon();
    RealVectorType svs = setupRangeSvs<RealVectorType>(min_dim, sigma_min, RealScalar(1));
    MatrixType m1;
    generateRandomMatrixSvs(svs, rows, cols, m1);
    RandColPivHouseholderQR<MatrixType> qr;
    configure_small(qr);
    qr.compute(m1);
    VERIFY_IS_EQUAL(min_dim, qr.rank());
  }
}

// Mirrors qr_rank_gap_test: geometric signal SVs decaying to sigma_rank,
// then noise SVs near eps. The clear gap at `rank` should be detected.
template <typename MatrixType>
void rqr_rank_gap_test() {
  typedef typename MatrixType::RealScalar RealScalar;
  typedef Matrix<RealScalar, Dynamic, 1> RealVectorType;
  const Index sizes[][2] = {{20, 20}, {50, 50}, {100, 100}, {50, 20}, {20, 50}};
  for (const auto& sz : sizes) {
    const Index rows = sz[0], cols = sz[1];
    const Index min_dim = (std::min)(rows, cols);
    const Index rank = (std::max)(Index(1), min_dim / 2);
    RealScalar sigma_rank = RealScalar(0.1);
    RealScalar eps_level = NumTraits<RealScalar>::epsilon();
    RealVectorType svs(min_dim);
    for (Index i = 0; i < rank; ++i) {
      RealScalar t = (rank > 1) ? RealScalar(i) / RealScalar(rank - 1) : RealScalar(0);
      svs(i) = std::pow(sigma_rank, t);
    }
    for (Index i = rank; i < min_dim; ++i) svs(i) = eps_level * RealScalar(min_dim - i);
    MatrixType m1;
    generateRandomMatrixSvs(svs, rows, cols, m1);
    RandColPivHouseholderQR<MatrixType> qr;
    configure_small(qr);
    qr.compute(m1);
    VERIFY_IS_EQUAL(rank, qr.rank());
  }
}

// SV-decay classes from the RQRCP/HQRRP papers' section 4.2: slow linear
// decay, fast exponential decay, and a low-rank case. The papers' central
// empirical claim is that the randomized strategy reports the same rank as
// classical column pivoting on these distributions; we verify that here.
template <typename MatrixType>
void rqr_sv_decay_classes() {
  typedef typename MatrixType::RealScalar RealScalar;
  typedef Matrix<RealScalar, Dynamic, 1> RealVectorType;
  const Index n = 60;

  // Slow decay: linear from 1 to 0.01.
  {
    RealVectorType svs(n);
    for (Index i = 0; i < n; ++i) svs(i) = RealScalar(1) - (RealScalar(0.99) * RealScalar(i)) / RealScalar(n - 1);
    MatrixType m1;
    generateRandomMatrixSvs(svs, n, n, m1);
    RandColPivHouseholderQR<MatrixType> qr;
    configure_small(qr);
    qr.compute(m1);
    ColPivHouseholderQR<MatrixType> cp(m1);
    VERIFY_IS_EQUAL(qr.rank(), cp.rank());
  }

  // Fast exponential decay: sigma_i = exp(-c * i), c chosen so the tail
  // hits roughly e^-20.
  {
    RealVectorType svs(n);
    RealScalar c = RealScalar(20) / RealScalar(n);
    for (Index i = 0; i < n; ++i) svs(i) = std::exp(-c * RealScalar(i));
    MatrixType m1;
    generateRandomMatrixSvs(svs, n, n, m1);
    RandColPivHouseholderQR<MatrixType> qr;
    configure_small(qr);
    qr.compute(m1);
    ColPivHouseholderQR<MatrixType> cp(m1);
    VERIFY_IS_EQUAL(qr.rank(), cp.rank());
  }

  // Low-rank: rank-12 signal block at [1, 0.5], rest at machine eps.
  {
    const Index r = 12;
    RealVectorType svs(n);
    for (Index i = 0; i < r; ++i) svs(i) = RealScalar(1) - (RealScalar(0.5) * i) / RealScalar(r - 1);
    for (Index i = r; i < n; ++i) svs(i) = NumTraits<RealScalar>::epsilon();
    MatrixType m1;
    generateRandomMatrixSvs(svs, n, n, m1);
    RandColPivHouseholderQR<MatrixType> qr;
    configure_small(qr);
    qr.compute(m1);
    VERIFY_IS_EQUAL(r, qr.rank());
  }
}

// Suite of well-known matrices from the numerical linear algebra
// literature that are commonly used to stress rank-revealing QR:
//   - Hilbert    H[i,j] = 1/(i+j+1)               (Hilbert 1894; cond ~ exp(3.5n))
//   - Vandermonde V[i,j] = x_i^j with x_i = i/n   (cond grows exponentially)
//   - Kahan A = S K                               (Kahan 1966 counterexample;
//                                                  HQRRP paper's "Matrix 4")
//   - HQRRP Matrix 1 (fast exponential SV decay,  d_j = β^((j-1)/(n-1)),
//                                                  β = 10^-5)
//   - HQRRP Matrix 2 (S-shaped SV decay; high
//                     plateau, knee, low plateau at 10^-6)
//
// Reconstruction must hold within a forgiving tolerance proportional to the
// matrix conditioning. Where the matrix is unambiguously full-rank or has a
// clear rank cutoff, we cross-check the reported rank against
// ColPivHouseholderQR on the same input — the central empirical claim of
// the RQRCP/HQRRP papers.
template <typename MatrixType>
void rqr_literature_matrices() {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename MatrixType::RealScalar RealScalar;
  typedef Matrix<RealScalar, Dynamic, 1> RealVectorType;

  // 1. Hilbert (n = 10): full rank in theory; cond ~ 1e13 in double. The
  // Q*R*P^-1 reconstruction is backward stable, so the round-trip error is
  // O(eps) independent of the conditioning.
  {
    const Index n = 10;
    MatrixType H(n, n);
    for (Index i = 0; i < n; ++i)
      for (Index j = 0; j < n; ++j) H(i, j) = Scalar(RealScalar(1) / RealScalar(i + j + 1));
    RandColPivHouseholderQR<MatrixType> qr;
    configure_small(qr);
    qr.compute(H);
    MatrixType R = qr.matrixQR().template triangularView<Upper>();
    MatrixType Q = qr.householderQ();
    RealScalar err = (H - Q * R * qr.colsPermutation().inverse()).norm() / H.norm();
    VERIFY(err < RealScalar(16) * NumTraits<RealScalar>::epsilon());
    ColPivHouseholderQR<MatrixType> cp(H);
    VERIFY_IS_EQUAL(qr.rank(), cp.rank());
  }

  // 2. Vandermonde with equispaced nodes (n = 12): also classically ill-
  // conditioned. Same round-trip + rank-match check.
  {
    const Index n = 12;
    MatrixType V(n, n);
    for (Index i = 0; i < n; ++i) {
      Scalar xi = Scalar(RealScalar(i + 1) / RealScalar(n));
      Scalar p(1);
      for (Index j = 0; j < n; ++j) {
        V(i, j) = p;
        p *= xi;
      }
    }
    RandColPivHouseholderQR<MatrixType> qr;
    configure_small(qr);
    qr.compute(V);
    MatrixType R = qr.matrixQR().template triangularView<Upper>();
    MatrixType Q = qr.householderQ();
    RealScalar err = (V - Q * R * qr.colsPermutation().inverse()).norm() / V.norm();
    VERIFY(err < RealScalar(16) * NumTraits<RealScalar>::epsilon());
    ColPivHouseholderQR<MatrixType> cp(V);
    VERIFY_IS_EQUAL(qr.rank(), cp.rank());
  }

  // 3. Canonical (non-symmetrized) Kahan A = S K. HQRRP paper "Matrix 4",
  // designed specifically to trip up classical column pivoting. Full rank;
  // classical QRP gets the rank-k truncation error wrong, but the
  // reconstruction round-trip and reported rank are still well-defined.
  {
    const Index n = 32;
    RealScalar zeta = RealScalar(0.99999);
    RealScalar phi = std::sqrt(RealScalar(1) - zeta * zeta);
    MatrixType S = MatrixType::Zero(n, n);
    MatrixType K = MatrixType::Zero(n, n);
    RealScalar zpow(1);
    for (Index i = 0; i < n; ++i) {
      S(i, i) = Scalar(zpow);
      zpow *= zeta;
      K(i, i) = Scalar(1);
      for (Index j = i + 1; j < n; ++j) K(i, j) = Scalar(-phi);
    }
    MatrixType A = S * K;
    RandColPivHouseholderQR<MatrixType> qr;
    configure_small(qr);
    qr.compute(A);
    MatrixType R = qr.matrixQR().template triangularView<Upper>();
    MatrixType Q = qr.householderQ();
    RealScalar err = (A - Q * R * qr.colsPermutation().inverse()).norm() / A.norm();
    VERIFY(err < RealScalar(16) * NumTraits<RealScalar>::epsilon());
    ColPivHouseholderQR<MatrixType> cp(A);
    VERIFY_IS_EQUAL(qr.rank(), cp.rank());
  }

  // 4. HQRRP "Matrix 1": exponential SV decay d_j = beta^((j-1)/(n-1))
  // with beta = 10^-5. All SVs are well above eps so rank should be n.
  {
    const Index n = 40;
    RealVectorType svs(n);
    RealScalar beta = RealScalar(1e-5);
    for (Index j = 0; j < n; ++j) svs(j) = std::pow(beta, RealScalar(j) / RealScalar(n - 1));
    MatrixType A;
    generateRandomMatrixSvs(svs, n, n, A);
    RandColPivHouseholderQR<MatrixType> qr;
    configure_small(qr);
    qr.compute(A);
    ColPivHouseholderQR<MatrixType> cp(A);
    VERIFY_IS_EQUAL(qr.rank(), cp.rank());
  }

  // 5. HQRRP "Matrix 2": S-shaped SV decay. High plateau (~1), sharp knee
  // around k = n/2, low plateau (~10^-6). The rank-revealing pivot strategy
  // should locate the knee.
  {
    const Index n = 40;
    const Index knee = n / 2;
    RealVectorType svs(n);
    RealScalar lo = RealScalar(1e-6);
    for (Index j = 0; j < n; ++j) {
      // sigmoid centered at `knee`, scaled to [lo, 1]
      RealScalar x = RealScalar(j - knee) * RealScalar(2);
      RealScalar s = RealScalar(1) / (RealScalar(1) + std::exp(x));
      svs(j) = lo + (RealScalar(1) - lo) * s;
    }
    MatrixType A;
    generateRandomMatrixSvs(svs, n, n, A);
    RandColPivHouseholderQR<MatrixType> qr;
    configure_small(qr);
    qr.compute(A);
    ColPivHouseholderQR<MatrixType> cp(A);
    VERIFY_IS_EQUAL(qr.rank(), cp.rank());
  }
}

EIGEN_DECLARE_TEST(qr_rand_colpivoting) {
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(rqr<MatrixXf>());
    CALL_SUBTEST_2(rqr<MatrixXd>());
    CALL_SUBTEST_3(rqr<MatrixXcd>());
    CALL_SUBTEST_4((rqr_fixedsize<Matrix<float, 8, 10>, 4>()));
    CALL_SUBTEST_5((rqr_fixedsize<Matrix<double, 12, 6>, 3>()));

    // The panel factorization wraps a block of the stored matrix in a Ref, so it has to follow
    // MatrixType's storage order rather than the workspaces' column-major one.
    CALL_SUBTEST_1((rqr<Matrix<float, Dynamic, Dynamic, RowMajor>>()));
    CALL_SUBTEST_2((rqr<Matrix<double, Dynamic, Dynamic, RowMajor>>()));
    CALL_SUBTEST_4((rqr_fixedsize<Matrix<float, 8, 10, RowMajor>, 4>()));

    CALL_SUBTEST_1(rcod<MatrixXf>());
    CALL_SUBTEST_2(rcod<MatrixXd>());
    CALL_SUBTEST_3(rcod<MatrixXcd>());
    CALL_SUBTEST_4((rcod_fixedsize<Matrix<float, 8, 10>, 4>()));
    CALL_SUBTEST_5((rcod_fixedsize<Matrix<double, 12, 6>, 3>()));

    CALL_SUBTEST_2((rcod<Matrix<double, Dynamic, Dynamic, RowMajor>>()));
    CALL_SUBTEST_4((rcod_fixedsize<Matrix<float, 8, 10, RowMajor>, 4>()));
  }

  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(rqr_invertible<MatrixXf>());
    CALL_SUBTEST_2(rqr_invertible<MatrixXd>());
    CALL_SUBTEST_6(rqr_invertible<MatrixXcf>());
    CALL_SUBTEST_3(rqr_invertible<MatrixXcd>());
  }

  CALL_SUBTEST_7(rqr_verify_assert<Matrix3f>());
  CALL_SUBTEST_8(rqr_verify_assert<Matrix3d>());
  CALL_SUBTEST_1(rqr_verify_assert<MatrixXf>());
  CALL_SUBTEST_2(rqr_verify_assert<MatrixXd>());
  CALL_SUBTEST_6(rqr_verify_assert<MatrixXcf>());
  CALL_SUBTEST_3(rqr_verify_assert<MatrixXcd>());

  CALL_SUBTEST_7(rcod_verify_assert<Matrix3f>());
  CALL_SUBTEST_8(rcod_verify_assert<Matrix3d>());
  CALL_SUBTEST_1(rcod_verify_assert<MatrixXf>());
  CALL_SUBTEST_2(rcod_verify_assert<MatrixXd>());
  CALL_SUBTEST_6(rcod_verify_assert<MatrixXcf>());
  CALL_SUBTEST_3(rcod_verify_assert<MatrixXcd>());

  // Test problem-size constructor.
  CALL_SUBTEST_9(RandColPivHouseholderQR<MatrixXf>(10, 20));

  CALL_SUBTEST_1(rqr_empty_input<MatrixXf>());
  CALL_SUBTEST_2(rqr_empty_input<MatrixXd>());
  CALL_SUBTEST_3(rqr_empty_input<MatrixXcd>());

  CALL_SUBTEST_1(rqr_kahan_matrix<MatrixXf>());
  CALL_SUBTEST_2(rqr_kahan_matrix<MatrixXd>());

  CALL_SUBTEST_2(rqr_blocked_path<MatrixXd>());
  CALL_SUBTEST_3(rqr_blocked_path<MatrixXcd>());

  CALL_SUBTEST_1(rqr_rank_in_late_block<MatrixXf>());
  CALL_SUBTEST_2(rqr_rank_in_late_block<MatrixXd>());
  CALL_SUBTEST_3(rqr_rank_in_late_block<MatrixXcd>());

  CALL_SUBTEST_1(rqr_rank_detection_stress<MatrixXf>());
  CALL_SUBTEST_2(rqr_rank_detection_stress<MatrixXd>());

  CALL_SUBTEST_1(rqr_threshold_efficiency<MatrixXf>());
  CALL_SUBTEST_2(rqr_threshold_efficiency<MatrixXd>());

  CALL_SUBTEST_1(rqr_rank_gap_test<MatrixXf>());
  CALL_SUBTEST_2(rqr_rank_gap_test<MatrixXd>());

  CALL_SUBTEST_2(rqr_sv_decay_classes<MatrixXd>());
  CALL_SUBTEST_3(rqr_sv_decay_classes<MatrixXcd>());

  CALL_SUBTEST_2(rqr_literature_matrices<MatrixXd>());
}
