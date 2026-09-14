// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// Enable Eigen's runtime malloc tracking so bunchkaufman_no_malloc() can assert that compute()
// performs no heap allocation when the workspace is pre-allocated. (Malloc stays allowed by default;
// only that one subtest toggles it off.) Must be defined before any Eigen header is included.
#define EIGEN_RUNTIME_NO_MALLOC

#include "main.h"
#include <Eigen/Cholesky>
#include <Eigen/QR>
#include <Eigen/Eigenvalues>
#include "fp_control.h"
#include "solverbase.h"

template <typename MatrixType, int UpLo>
typename MatrixType::RealScalar matrix_l1_norm(const MatrixType& m) {
  if (m.cols() == 0) return typename MatrixType::RealScalar(0);
  MatrixType symm = m.template selfadjointView<UpLo>();
  return symm.cwiseAbs().colwise().sum().maxCoeff();
}

// Reconstruct the block-diagonal D from vectorD() / subDiagonal() and check that
// P^T L D L^* P == A and that matrixL()/matrixU() are consistent.
template <typename MatrixType, typename BKType>
void verify_factorization(const MatrixType& A, const BKType& bk) {
  typedef typename MatrixType::Scalar Scalar;
  const Index n = A.rows();

  VERIFY_IS_APPROX(A, bk.reconstructedMatrix());

  // The diagonal of D must be exactly real (not merely up to roundoff): the factorization
  // re-realifies the trailing diagonal after each Hermitian update, like LAPACK xHETF2/xHETRF.
  for (Index k = 0; k < n; ++k) {
    VERIFY(numext::is_exactly_zero(numext::imag(bk.vectorD().coeff(k))));
  }

  // Build D explicitly from vectorD() and subDiagonal() and reconstruct manually.
  MatrixType D = MatrixType::Zero(n, n);
  D.diagonal() = bk.vectorD();
  for (Index k = 0; k + 1 < n; ++k) {
    Scalar s = bk.subDiagonal()(k);
    if (!numext::is_exactly_zero(s)) {
      D(k + 1, k) = s;
      D(k, k + 1) = numext::conj(s);
    }
  }
  // matrixU() should be the adjoint of matrixL().
  MatrixType L = bk.matrixL();
  MatrixType U = bk.matrixU();
  VERIFY_IS_APPROX(L.adjoint(), U);

  // P^T L D L^* P
  MatrixType PtL = bk.transpositionsP().transpose() * L;
  MatrixType recon = PtL * D * PtL.adjoint();
  VERIFY_IS_APPROX(A, recon);
}

// Core test on a Hermitian indefinite matrix `symm` (full, self-adjoint).
template <typename MatrixType>
void bunchkaufman_solve_and_reconstruct(const MatrixType& symm) {
  typedef typename MatrixType::Scalar Scalar;
  typedef Matrix<Scalar, MatrixType::RowsAtCompileTime, MatrixType::RowsAtCompileTime> SquareMatrixType;
  typedef Matrix<Scalar, MatrixType::RowsAtCompileTime, 1> VectorType;

  const Index rows = symm.rows();
  const Index cols = symm.cols();

  SquareMatrixType symmLo = symm.template triangularView<Lower>();
  SquareMatrixType symmUp = symm.template triangularView<Upper>();

  BunchKaufman<SquareMatrixType, Lower> bk_lo(symmLo);
  VERIFY(bk_lo.info() == Success);
  verify_factorization(SquareMatrixType(symm), bk_lo);
  check_solverbase<VectorType, VectorType>(symm, bk_lo, rows, rows, 1);
  check_solverbase<MatrixType, MatrixType>(symm, bk_lo, rows, cols, rows);

  BunchKaufman<SquareMatrixType, Upper> bk_up(symmUp);
  VERIFY(bk_up.info() == Success);
  verify_factorization(SquareMatrixType(symm), bk_up);
  check_solverbase<VectorType, VectorType>(symm, bk_up, rows, rows, 1);

  // MatrixBase / SelfAdjointView entry points.
  verify_factorization(SquareMatrixType(symm), SquareMatrixType(symm).bunchKaufman());
  verify_factorization(SquareMatrixType(symm), symm.template selfadjointView<Lower>().bunchKaufman());
  verify_factorization(SquareMatrixType(symm), symm.template selfadjointView<Upper>().bunchKaufman());

  // rcond() within a factor of 10 of the true reciprocal 1-norm condition number.
  if (rows > 0) {
    const SquareMatrixType inv = bk_lo.solve(SquareMatrixType::Identity(rows, rows));
    using RealScalar = typename MatrixType::RealScalar;
    RealScalar rcond = (RealScalar(1) / matrix_l1_norm<SquareMatrixType, Lower>(symmLo)) /
                       matrix_l1_norm<SquareMatrixType, Lower>(inv);
    RealScalar rcond_est = bk_lo.rcond();
    VERIFY(rcond_est >= rcond / 10 && rcond_est <= rcond * 10);
  }
}

// Build a Hermitian matrix with a prescribed (real) eigenvalue spectrum and check inertia + stability.
template <typename MatrixType>
void bunchkaufman_inertia_and_conditioning(Index n) {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename MatrixType::RealScalar RealScalar;
  typedef Matrix<RealScalar, Dynamic, 1> RealVectorType;

  // Random orthonormal/unitary U via QR.
  MatrixType R = MatrixType::Random(n, n);
  HouseholderQR<MatrixType> qr(R);
  MatrixType U = qr.householderQ();

  // Eigenvalues spanning a wide magnitude range, with mixed signs (indefinite).
  const RealScalar s = (std::min)(RealScalar(6), RealScalar(std::numeric_limits<RealScalar>::max_exponent10) / 8);
  RealVectorType d(n);
  Index expect_pos = 0, expect_neg = 0;
  for (Index k = 0; k < n; ++k) {
    RealScalar mag = numext::pow(RealScalar(10), internal::random<RealScalar>(-s, s));
    RealScalar sign = internal::random<bool>() ? RealScalar(1) : RealScalar(-1);
    d(k) = sign * mag;
    if (d(k) > 0)
      ++expect_pos;
    else
      ++expect_neg;
  }
  MatrixType A = U * d.asDiagonal() * U.adjoint();
  // Force exact Hermitian symmetry (kill round-off asymmetry).
  A = (A + A.adjoint()).eval() * Scalar(RealScalar(0.5));

  BunchKaufman<MatrixType, Lower> bk(A);
  VERIFY(bk.info() == Success);
  VERIFY_IS_APPROX(A, bk.reconstructedMatrix());

  // isPositive()/isNegative() agree with the true inertia.
  VERIFY(bk.isPositive() == (expect_neg == 0));
  VERIFY(bk.isNegative() == (expect_pos == 0));

  // Backward-stable solve: relative residual is small even for ill-conditioned A.
  Matrix<Scalar, Dynamic, 1> b = Matrix<Scalar, Dynamic, 1>::Random(n);
  Matrix<Scalar, Dynamic, 1> x = bk.solve(b);
  RealScalar res = (A * x - b).norm() / b.norm();
  RealScalar tol = sqrt(test_precision<RealScalar>());
  VERIFY(res <= tol);
}

// Definiteness / 2x2-pivot regression cases.
template <typename Scalar>
void bunchkaufman_small_cases() {
  typedef Matrix<Scalar, 2, 2> Mat2;
  typedef Matrix<Scalar, 2, 1> Vec2;

  // Indefinite with zero diagonal -> requires a 2x2 pivot.
  {
    Mat2 A;
    A << Scalar(0), Scalar(1), Scalar(1), Scalar(0);
    BunchKaufman<Mat2> bk(A);
    VERIFY(bk.info() == Success);
    VERIFY_IS_APPROX(A, bk.reconstructedMatrix());
    VERIFY(!bk.isPositive());
    VERIFY(!bk.isNegative());
    Vec2 b(Scalar(3), Scalar(5));
    Vec2 x = bk.solve(b);
    VERIFY_IS_APPROX(A * x, b);
  }
  // Diagonal indefinite [[1,0],[0,-1]].
  {
    Mat2 A;
    A << Scalar(1), Scalar(0), Scalar(0), Scalar(-1);
    BunchKaufman<Mat2> bk(A);
    VERIFY(bk.info() == Success);
    VERIFY_IS_APPROX(A, bk.reconstructedMatrix());
    VERIFY(!bk.isPositive());
    VERIFY(!bk.isNegative());
  }
  // 1x1.
  {
    Matrix<Scalar, 1, 1> A;
    A << Scalar(-3);
    BunchKaufman<Matrix<Scalar, 1, 1> > bk(A);
    VERIFY(bk.info() == Success);
    VERIFY_IS_APPROX(A, bk.reconstructedMatrix());
    VERIFY(!bk.isPositive());
    VERIFY(bk.isNegative());
  }
}

// Exactly-singular matrix: factorization succeeds structurally but reports NumericalIssue,
// while still reconstructing the input exactly. Exercised at a small size (unblocked kernel) and a
// size larger than the panel width (blocked kernel, where the zero pivot lands inside a panel).
template <typename Scalar>
void bunchkaufman_singular(Index n) {
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixType;
  // A zero column/row makes the matrix exactly singular with an exact zero pivot. Place it in the
  // interior so that, for n > blocksize, it falls inside a panel of the blocked algorithm.
  const Index z = n / 2;
  MatrixType M = MatrixType::Random(n, n);
  MatrixType A = M + M.adjoint();
  A.col(z).setZero();
  A.row(z).setZero();
  BunchKaufman<MatrixType, Lower> lo(A);
  VERIFY(lo.info() == NumericalIssue);
  VERIFY_IS_APPROX(A, lo.reconstructedMatrix());
  // matrixL() must be a well-formed unit lower triangular factor even on the singular column.
  VERIFY((lo.matrixL().toDenseMatrix().diagonal().array() == Scalar(1)).all());
  BunchKaufman<MatrixType, Upper> up(A);
  VERIFY(up.info() == NumericalIssue);
  VERIFY_IS_APPROX(A, up.reconstructedMatrix());
}

// A matrix containing a NaN must be reported as a numerical failure (matching LAPACK's DISNAN guard),
// not silently accepted.
template <typename Scalar>
void bunchkaufman_nan() {
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixType;
  for (Index n : {5, 100}) {
    MatrixType M = MatrixType::Random(n, n);
    MatrixType A = M + M.adjoint();
    A(n / 2, n / 2) = std::numeric_limits<typename NumTraits<Scalar>::Real>::quiet_NaN();
    BunchKaufman<MatrixType> bk(A);
    VERIFY(bk.info() == NumericalIssue);
  }
}

// Rank-deficient PSD: A = a a^* with a of rank r < n. Reconstruct must match.
template <typename Scalar>
void bunchkaufman_rank_deficient() {
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixType;
  const Index n = 16;
  const Index r = internal::random<Index>(1, n - 1);
  MatrixType a = MatrixType::Random(n, r);
  MatrixType A = a * a.adjoint();
  BunchKaufman<MatrixType> bk(A);
  VERIFY_IS_APPROX(A, bk.reconstructedMatrix());
  VERIFY(!bk.isNegative());  // PSD -> no negative eigenvalues
}

// Blocking and 2x2-panel-boundary stress across sizes that straddle the panel width.
template <typename Scalar>
void bunchkaufman_blocking_boundary() {
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixType;
  typedef typename NumTraits<Scalar>::Real RealScalar;
  const Index PS = internal::packet_traits<Scalar>::size;
  const Index sizes[] = {1,  2,  3,  PS - 1, PS,  PS + 1, 2 * PS, 31,  32,         33,
                         63, 64, 65, 96,     127, 128,    129,    192, 2 * 64 + 3, 200};
  for (Index n : sizes) {
    if (n <= 0) continue;
    MatrixType M = MatrixType::Random(n, n);
    MatrixType A = M + M.adjoint();
    // Force several 2x2 pivots by shrinking the diagonal.
    A.diagonal() *= Scalar(RealScalar(1e-2));

    BunchKaufman<MatrixType, Lower> lo(A);
    VERIFY(lo.info() == Success);
    VERIFY_IS_APPROX(A, lo.reconstructedMatrix());

    BunchKaufman<MatrixType, Upper> up(A);
    VERIFY(up.info() == Success);
    VERIFY_IS_APPROX(A, up.reconstructedMatrix());

    // Lower and Upper must yield the same (Hermitian) decomposition of A.
    VERIFY_IS_APPROX(lo.reconstructedMatrix(), up.reconstructedMatrix());

    Matrix<Scalar, Dynamic, 1> b = Matrix<Scalar, Dynamic, 1>::Random(n);
    Matrix<Scalar, Dynamic, 1> x = lo.solve(b);
    const Matrix<Scalar, Dynamic, 1> residual = A * x - b;
    // The diagonal shrink above intentionally makes the pivot-boundary cases harder than the generic solve tests.
    VERIFY(residual.norm() <= RealScalar(10) * NumTraits<Scalar>::epsilon() * (A.norm() * x.norm() + b.norm()));
  }
}

// A = Q D Q^*, with Q unitary and D real, is Hermitian with det(A) = prod(D_ii). Mixed signs make A
// indefinite, so the factorization mixes 1x1 and 2x2 blocks of D; drawing the |D_ii| from an annulus keeps
// A well conditioned, hence the inertia -- and with it signDeterminant() -- unambiguous.
template <typename MatrixType>
void bunchkaufman_determinant(Index size) {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<RealScalar, Dynamic, 1> RealVectorType;

  MatrixType q = MatrixType::Random(size, size).householderQr().householderQ();
  RealVectorType d(size);
  for (Index i = 0; i < size; ++i) {
    d(i) = internal::random<RealScalar>(RealScalar(1.2), RealScalar(2.8));
    if (internal::random<bool>()) d(i) = -d(i);
  }
  const MatrixType a = q * d.template cast<Scalar>().asDiagonal() * q.adjoint();

  const RealScalar det = d.prod();
  const RealScalar logabsdet = d.array().abs().log().sum();

  BunchKaufman<MatrixType, Lower> bklo(a);
  VERIFY(bklo.info() == Success);
  check_determinant(bklo, Scalar(det), logabsdet);
  // Unlike the other decompositions, this sign is read off the inertia rather than accumulated from a
  // product of signs, so it is exact.
  VERIFY_IS_EQUAL(bklo.signDeterminant(), Scalar(numext::sign(det)));

  BunchKaufman<MatrixType, Upper> bkup(a);
  VERIFY(bkup.info() == Success);
  check_determinant(bkup, Scalar(det), logabsdet);
  // Unlike the other decompositions, this sign is read off the inertia rather than accumulated from a
  // product of signs, so it is exact.
  VERIFY_IS_EQUAL(bkup.signDeterminant(), Scalar(numext::sign(det)));
}

// The determinant of an empty matrix is the empty product, 1.
template <typename MatrixType>
void bunchkaufman_determinant_empty() {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;

  BunchKaufman<MatrixType> bk{MatrixType(0, 0)};
  VERIFY_IS_EQUAL(bk.determinant(), Scalar(1));
  VERIFY_IS_EQUAL(bk.absDeterminant(), RealScalar(1));
  VERIFY_IS_EQUAL(bk.logAbsDeterminant(), RealScalar(0));
  VERIFY_IS_EQUAL(bk.signDeterminant(), Scalar(1));
}

// det(D) can be representable while the block determinants it is built from are not. The blocks below are
// [[s/2, s], [s, s/2]] with det = -3s^2/4 at s = 2^600 and s = 2^-600, one overflowing and one
// underflowing, whose product is exactly 9/16. Multiplying the blocks directly gives inf * 0 = NaN.
void bunchkaufman_determinant_mixed_scale() {
  MatrixXd a = MatrixXd::Zero(4, 4);
  for (int b = 0; b < 2; ++b) {
    const double s = numext::ldexp(1.0, b == 0 ? 600 : -600);
    a(2 * b, 2 * b) = a(2 * b + 1, 2 * b + 1) = 0.5 * s;
    a(2 * b + 1, 2 * b) = a(2 * b, 2 * b + 1) = s;
  }

  // (-3/4)^2 (2^600 2^-600)^2 = 9/16, exactly, both factors being powers of two.
  const double det = 0.5625;

  BunchKaufman<MatrixXd, Lower> bklo(a);
  VERIFY(bklo.info() == Success);
  check_determinant(bklo, det, numext::log(det));

  BunchKaufman<MatrixXd, Upper> bkup(a);
  VERIFY(bkup.info() == Success);
  check_determinant(bkup, det, numext::log(det));
}

// Bunch-Kaufman selects a 2x2 block only where |d11 d22| <= alpha^2 |d21|^2, alpha = (1+sqrt(17))/8 < 1, so
// det D_k = d11 d22 - |d21|^2 < 0 always. Subnormal |d21| underflows that determinant to zero, leaving the
// log and the sign as the only accessors that can report it; both need det D_k / |d21|^2 = O(1), which
// 1/|d21| overflows on, and the resulting infinity flips the block's inertia.
template <typename MatrixType>
void bunchkaufman_determinant_subnormal_block() {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;

  if (!subnormalDivisionIsExact<RealScalar>()) {
    const char* reason = ScopedFlushToZero::hardwareFlushesSubnormalInputs() ? "the hardware flushes subnormal inputs"
                                                                             : "the compiler relaxed the division";
    std::cout << "SKIP: bunchkaufman_determinant_subnormal_block needs an environment that divides by subnormals "
                 "per IEEE 754 ("
              << reason << ")." << std::endl;
    return;
  }

  // Entries are exact integer multiples of the smallest subnormal u, so det A = (k11 k22 - k21^2) u^2 with
  // the bracket an exact int, and log|det A| = log|k11 k22 - k21^2| + 2 log u -- a reference that shares no
  // expression with the code under test. The off-diagonal stays real to keep the entries exact; the complex
  // instantiation still reaches numext::abs() on a subnormal.
  const RealScalar u = (std::numeric_limits<RealScalar>::denorm_min)();
  const int k11[] = {0, 1, 2, 100};
  const int k22[] = {0, 1, 2, 100};
  const int k21[] = {1, 4, 8, 300};

  for (int c = 0; c < 4; ++c) {
    const int idet = k11[c] * k22[c] - k21[c] * k21[c];
    VERIFY(idet < 0);
    const RealScalar logabsdet = numext::log(RealScalar(-idet)) + RealScalar(2) * numext::log(u);

    MatrixType a(2, 2);
    a << Scalar(RealScalar(k11[c]) * u), Scalar(RealScalar(k21[c]) * u), Scalar(RealScalar(k21[c]) * u),
        Scalar(RealScalar(k22[c]) * u);
    // Flushing subnormal results to zero empties the matrix even where the division probe passed.
    if (numext::is_exactly_zero(numext::abs(a.coeff(1, 0)))) return;

    BunchKaufman<MatrixType, Lower> bk(a);
    VERIFY(bk.info() == Success);

    VERIFY_IS_EQUAL(bk.signDeterminant(), Scalar(-1));
    VERIFY_IS_APPROX(bk.logAbsDeterminant(), logabsdet);
    // det A is of order u^2, so zero is all these two can report.
    VERIFY_IS_EQUAL(bk.absDeterminant(), RealScalar(0));
    VERIFY_IS_EQUAL(bk.determinant(), Scalar(0));
    VERIFY(!bk.isPositive());
    VERIFY(!bk.isNegative());
  }
}

// The criterion bounds |d11| against alpha|d21| but bounds |d22| only against the largest entry of its own
// row, which can dwarf |d21|. So d22/d21 can overflow, and the scaled determinant comes out +-inf, or 0*inf
// = NaN where d11 is zero; +inf and NaN count the block as definite with the sign of its trace, which is
// positive here. The criterion puts the true magnitude below 1, which is what rejects all three.
void bunchkaufman_inertia_wide_2x2_block() {
  // The 2x2 block is (d11, d21, d22) = (a00, 1e-10, 1e299), determinant a00*1e299 - 1e-20 < 0 for every
  // a00 below, so it contributes one eigenvalue of each sign. The block's inverse has entries of order
  // d22/det ~ 1e319, so L = U D^{-1} is not representable and the factorization reports NumericalIssue
  // (its own scaled determinant sees d22/d21 overflow); the inertia is still exact.
  for (double a00 : {0.0, 1e-320, -1e-320}) {
    MatrixXd a = MatrixXd::Zero(4, 4);
    a(0, 0) = a00;
    a(1, 0) = a(0, 1) = 1e-10;
    a(1, 1) = 1e299;
    a(3, 1) = a(1, 3) = 2e299;

    BunchKaufman<MatrixXd> bk(a);
    VERIFY(bk.info() == NumericalIssue);
    VERIFY(!bk.isPositive());
    VERIFY(!bk.isNegative());
  }
}

// The scaling above divides by numext::abs(d21), so it relies on that magnitude not underflowing: for
// |3u + 4iu| Eigen's hypot scales by the larger component instead of summing squares, giving 5u exactly
// where u^2 would be zero. Not template code, because building the off-diagonal needs a complex literal.
void bunchkaufman_determinant_subnormal_block_complex() {
  typedef std::complex<double> Scalar;

  if (!subnormalDivisionIsExact<double>()) return;

  const double u = (std::numeric_limits<double>::denorm_min)();
  MatrixXcd a(2, 2);
  a << Scalar(u, 0), Scalar(3 * u, 4 * u), Scalar(3 * u, -4 * u), Scalar(u, 0);
  BunchKaufman<MatrixXcd, Lower> bk(a);

  // det = u^2 - |3u + 4iu|^2 = -24 u^2.
  VERIFY_IS_EQUAL(bk.signDeterminant(), Scalar(-1));
  VERIFY_IS_APPROX(bk.logAbsDeterminant(), numext::log(24.0) + 2.0 * numext::log(u));
  VERIFY(!bk.isPositive());
  VERIFY(!bk.isNegative());
}

template <typename MatrixType>
void bunchkaufman_verify_assert() {
  MatrixType tmp;
  BunchKaufman<MatrixType> bk;
  VERIFY_RAISES_ASSERT(bk.matrixL())
  VERIFY_RAISES_ASSERT(bk.matrixU())
  VERIFY_RAISES_ASSERT(bk.vectorD())
  VERIFY_RAISES_ASSERT(bk.subDiagonal())
  VERIFY_RAISES_ASSERT(bk.transpositionsP())
  VERIFY_RAISES_ASSERT(bk.isPositive())
  VERIFY_RAISES_ASSERT(bk.isNegative())
  VERIFY_RAISES_ASSERT(bk.matrixLDLT())
  VERIFY_RAISES_ASSERT(bk.reconstructedMatrix())
  VERIFY_RAISES_ASSERT(bk.solve(tmp))
  VERIFY_RAISES_ASSERT(bk.determinant())
  VERIFY_RAISES_ASSERT(bk.absDeterminant())
  VERIFY_RAISES_ASSERT(bk.logAbsDeterminant())
  VERIFY_RAISES_ASSERT(bk.signDeterminant())
}

// Build a random Hermitian (real symmetric) indefinite matrix of the same type/size as `m`.
template <typename MatrixType>
MatrixType make_hermitian_indefinite(const MatrixType& m) {
  MatrixType a = MatrixType::Random(m.rows(), m.cols());
  return MatrixType(a + a.adjoint());
}

template <typename MatrixType>
void bunchkaufman(const MatrixType& m) {
  // General Hermitian indefinite.
  bunchkaufman_solve_and_reconstruct(make_hermitian_indefinite(m));
  // Zero-diagonal Hermitian -> forces 2x2 pivots throughout (needs n >= 2 to stay non-singular).
  if (m.rows() >= 2) {
    MatrixType A = make_hermitian_indefinite(m);
    A.diagonal().setZero();
    bunchkaufman_solve_and_reconstruct(A);
  }
}

// Extreme-scale 2x2 pivot: an off-diagonal-only Hermitian 2x2 forces a single 2x2 pivot whose
// determinant is det = -|off|^2. The scaled (det-free) 2x2 formulas must stay finite and correct when
// |off| is huge or tiny. Regression for forming det = d11*d22 - |d21|^2 directly, which overflows to
// +-inf for off=1e200 (solve then returns 0, residual 1) and underflows to 0 for off=1e-200 (solve
// returns NaN/inf), and which also misclassifies the inertia. Requires a type that can hold 1e+-200,
// so this is exercised for double / complex<double> only.
template <typename Scalar>
void bunchkaufman_extreme_scale() {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, 2, 2> Mat2;
  typedef Matrix<Scalar, 2, 1> Vec2;
  const RealScalar tol = numext::sqrt(test_precision<RealScalar>());
  for (RealScalar mag : {numext::pow(RealScalar(10), RealScalar(200)), numext::pow(RealScalar(10), RealScalar(-200))}) {
    const Scalar off = Scalar(mag);
    Mat2 A;
    A << Scalar(0), numext::conj(off), off, Scalar(0);
    BunchKaufman<Mat2> bk(A);
    VERIFY(bk.info() == Success);
    VERIFY(!bk.isPositive());  // det < 0 => one positive and one negative eigenvalue
    VERIFY(!bk.isNegative());
    // Use the max-abs (infinity) relative norm throughout: the Frobenius norm (.norm()) squares the
    // ~1e200 entries and would overflow/underflow even for a correct factorization.
    VERIFY((A - bk.reconstructedMatrix()).cwiseAbs().maxCoeff() <= tol * A.cwiseAbs().maxCoeff());
    // A x = b with b = [1,1]; the product A*x stays O(1), so its residual is safe to measure.
    const Vec2 b(Scalar(1), Scalar(1));
    const Vec2 x = bk.solve(b);
    VERIFY((A * x - b).cwiseAbs().maxCoeff() <= tol);
  }
}

// Extreme-scale factorization at the matrix level: a zero-diagonal Hermitian matrix (2x2 pivots
// throughout, exercising both the unblocked and -- for n > blocksize -- the blocked trailing update)
// scaled to an extreme magnitude. The factorization is scale-equivariant and must remain overflow-free;
// the solve must stay backward stable. double / complex<double> only (1e+-175 overflows float).
template <typename Scalar>
void bunchkaufman_extreme_scale_large(Index n) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixType;
  typedef Matrix<Scalar, Dynamic, 1> VectorType;
  const RealScalar tol = numext::sqrt(test_precision<RealScalar>());
  MatrixType M = MatrixType::Random(n, n);
  MatrixType A = M + M.adjoint();
  A.diagonal().setZero();
  // 1e+-175 is past the squaring threshold (|entry|^2 over/underflows double), so the pre-fix code
  // (which forms det = d11*d22 - |d21|^2) produces NaN/Inf here, while the scaled formulas stay exact.
  for (RealScalar sigma :
       {numext::pow(RealScalar(10), RealScalar(175)), numext::pow(RealScalar(10), RealScalar(-175))}) {
    const MatrixType As = A * Scalar(sigma);
    BunchKaufman<MatrixType, Lower> bk(As);
    VERIFY(bk.info() == Success);
    // Max-abs relative reconstruction error (avoid .norm(), whose squaring overflows the ~1e175 entries).
    VERIFY((As - bk.reconstructedMatrix()).cwiseAbs().maxCoeff() <= tol * As.cwiseAbs().maxCoeff());
    // As*x stays O(1) for a unit-scale rhs, so its residual norm is safe to form.
    const VectorType b = VectorType::Random(n);
    const VectorType x = bk.solve(b);
    VERIFY((As * x - b).norm() <= tol * b.norm());
  }
}

// The off-diagonal of the subnormal 2x2 pivot block below: 4s, or (3 + 4i) s with the same |.| = 5s structure as
// the determinant test above. Two overloads because a complex literal cannot be spelled in template code.
template <typename Scalar>
Scalar subnormal_block_off_diagonal(const Scalar& s, std::false_type /*IsComplex*/) {
  return Scalar(4) * s;
}
template <typename Scalar>
Scalar subnormal_block_off_diagonal(const typename NumTraits<Scalar>::Real& s, std::true_type /*IsComplex*/) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  return Scalar(RealScalar(3) * s, RealScalar(4) * s);
}

// Issue #3142: a 2x2 pivot block with a subnormal off-diagonal. D_k = s [[1, 4], [4, 1]] (d21 = (3+4i) s when
// complex) is an ordinary 2x2 pivot, det D_k = -15 s^2 (-24 s^2), but 1/d21 overflows once d21 is subnormal, and a
// reciprocal hoisted out of the factor-column and D-solve loops made both 0*inf. A is the identity apart from D_k
// at rows p, p+1 and the m rows below it, coupled to it by small-integer multiples of s, so P = I, L - I is the
// O(1) block C D_k^{-1}, and the Schur complement I - C D_k^{-1} C^* is I up to O(s) off-diagonal entries.
template <typename MatrixType, typename BKType>
void verify_subnormal_2x2_block(const MatrixType& A, const BKType& bk,
                                const Matrix<typename MatrixType::Scalar, Dynamic, 1>& b,
                                const Matrix<typename MatrixType::Scalar, Dynamic, 1>& x_true, Index m,
                                const typename MatrixType::RealScalar& s) {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename MatrixType::RealScalar RealScalar;
  const RealScalar u = (std::numeric_limits<RealScalar>::denorm_min)();
  const RealScalar eps = NumTraits<RealScalar>::epsilon();

  VERIFY(bk.info() == Success);
  VERIFY(bk.matrixLDLT().allFinite());
  VERIFY_IS_EQUAL(bk.signDeterminant(), Scalar(-1));
  // The O(1) entries reconstruct exactly; an entry of order s is rebuilt from a few products that each round at
  // the quantum u.
  VERIFY((A - bk.reconstructedMatrix()).cwiseAbs().maxCoeff() <= RealScalar(8) * u);
  // The D solve divides by d21 before anything else and is then a few roundings of O(1) values up to 2 in
  // magnitude. A factor entry t*(ak*u0 - u1)/conj(d21) forms its numerator at the scale of s, about three
  // roundings at the quantum u, so it is off by up to 3u/|d21| <= 3u/(4s); the substitution adds m*max|x| = 4
  // of those to x.
  const Matrix<Scalar, Dynamic, 1> x = bk.solve(b);
  VERIFY(x.allFinite());
  const RealScalar quantum = m > 0 ? u / s : RealScalar(0);
  const RealScalar tol = RealScalar(16) * eps + RealScalar(4) * quantum;
  VERIFY((x - x_true).cwiseAbs().maxCoeff() <= tol);
}

template <typename MatrixType>
void bunchkaufman_subnormal_2x2_block(Index n, Index p, Index m, const typename MatrixType::RealScalar& s) {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename MatrixType::RealScalar RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> VectorType;

  MatrixType A = MatrixType::Identity(n, n);
  A(p, p) = A(p + 1, p + 1) = Scalar(s);
  const Scalar d21 = subnormal_block_off_diagonal<Scalar>(s, internal::bool_constant<NumTraits<Scalar>::IsComplex>());
  A(p + 1, p) = d21;
  A(p, p + 1) = numext::conj(d21);
  // Flushing subnormal results to zero empties the block even where the division probe passed.
  if (numext::is_exactly_zero(numext::abs(A(p + 1, p)))) return;
  // Couplings of magnitude at most 3s stay below |d21|, so the pivot search keeps row p+1 as the column
  // maximum and selects the 2x2 block without an interchange.
  for (Index i = 0; i < m; ++i) {
    const Index r = p + 2 + i;
    A(r, p) = A(p, r) = Scalar(RealScalar(i + 1) * s);
    A(r, p + 1) = A(p + 1, r) = Scalar(RealScalar(2 * i - 3) * s);
  }
  VectorType x_true(n);
  for (Index i = 0; i < n; ++i) x_true(i) = Scalar(RealScalar(i % 5) - RealScalar(2));
  // Rows of order s are exact small-integer multiples of s; the others round to the integer x_true(i).
  const VectorType b = A * x_true;

  BunchKaufman<MatrixType, Lower> bk_lo(A);
  verify_subnormal_2x2_block(A, bk_lo, b, x_true, m, s);
  BunchKaufman<MatrixType, Upper> bk_up(A);
  verify_subnormal_2x2_block(A, bk_up, b, x_true, m, s);
}

template <typename MatrixType>
void bunchkaufman_subnormal_2x2_block() {
  typedef typename MatrixType::RealScalar RealScalar;
  if (!subnormalDivisionIsExact<RealScalar>()) {
    const char* reason = ScopedFlushToZero::hardwareFlushesSubnormalInputs() ? "the hardware flushes subnormal inputs"
                                                                             : "the compiler relaxed the division";
    std::cout << "SKIP: bunchkaufman_subnormal_2x2_block needs an environment that divides by subnormals per "
                 "IEEE 754 ("
              << reason << ")." << std::endl;
    return;
  }
  // s = min/1024 is subnormal with digits-10 significant bits, and 1/(4s) = 256/min overflows in every IEEE
  // format. The coupled cases need those bits: their factor columns round at the quantum u relative to s.
  const RealScalar s = (std::numeric_limits<RealScalar>::min)() / RealScalar(1024);
  // The issue's two shapes -- the block last (no trailing update) and first -- at denorm_min itself where the
  // block is real: every quotient is then dyadic and exact. A complex quotient of denorm_min-scale operands is
  // exact only under a complex division that scales both operands, which the C++ runtime may or may not do, so
  // the complex instantiations use s, where 3s*0.75 and 2.25s are representable under any of the algorithms.
  const RealScalar s0 =
      NumTraits<typename MatrixType::Scalar>::IsComplex ? s : (std::numeric_limits<RealScalar>::denorm_min)();
  bunchkaufman_subnormal_2x2_block<MatrixType>(4, 2, 0, s0);
  bunchkaufman_subnormal_2x2_block<MatrixType>(4, 0, 0, s0);
  bunchkaufman_subnormal_2x2_block<MatrixType>(4, 0, 2, s);
  // Above the panel width the blocked driver runs: block in the first panel, straddling the panel boundary
  // (deferred to the next panel), opening the second panel, and last in the unblocked tail.
  const Index n = 2 * internal::bunch_kaufman_blocksize<typename MatrixType::Scalar>() + 2;
  for (Index p : {Index(0), n / 2 - 2, n / 2, n - 2}) {
    bunchkaufman_subnormal_2x2_block<MatrixType>(n, p, p + 2 < n ? 2 : 0, s);
  }
}

template <typename BKType, typename VectorType>
void verify_near_max_scale(const BKType& bk, const VectorType& b, const VectorType& x_true) {
  typedef typename VectorType::RealScalar RealScalar;
  VERIFY(bk.info() == Success);
  VERIFY(bk.matrixLDLT().allFinite());
  // x_true = e_2 comes back through cancellations (6/5 - 6/5 and -3/5 + 3/5, or 18/25 - 18/25 and
  // -9/25 + 9/25) of values that each carry a few roundings.
  VERIFY((bk.solve(b) - x_true).cwiseAbs().maxCoeff() <= RealScalar(16) * NumTraits<RealScalar>::epsilon());
}

// A 2x2 pivot block at the top of the representable range: A = M [[1/2, 1, 0], [1, s/2, 9/10], [0, 9/10, 0]]
// with M = max and s = +-1: d21 = M, ak = s/2, akm1 = 1/2, denom = s/4 - 1, and the factor row below the
// block is (6/5, -3/5) for s = 1 and (18/25, -9/25) for s = -1. Scaling ak*u0 - u1 = -9M/10 by t before
// dividing by d21 overflows although the entries are finite; for s = -1 so does denom*d21, which would zero
// the row with info() == Success. n = 3 runs unblocked(), the identity wider than the panel partial_factor().
// L D L^* is not checked: (L D)(2, 1) = 6M/5 - 3M/10 overflows in the middle of its own evaluation.
template <typename MatrixType>
void bunchkaufman_near_max_scale() {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename MatrixType::RealScalar RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> VectorType;
  const RealScalar M = (std::numeric_limits<RealScalar>::max)();
  for (Index n : {Index(3), 2 * internal::bunch_kaufman_blocksize<Scalar>() + 2}) {
    for (RealScalar s : {RealScalar(1), RealScalar(-1)}) {
      MatrixType A = MatrixType::Identity(n, n);
      A(0, 0) = Scalar(M / RealScalar(2));
      A(1, 1) = Scalar(s * M / RealScalar(2));
      A(1, 0) = A(0, 1) = Scalar(M);
      A(2, 1) = A(1, 2) = Scalar(RealScalar(0.9) * M);
      A(2, 2) = Scalar(0);
      const VectorType b = A.col(2);
      VectorType e2 = VectorType::Zero(n);
      e2(2) = Scalar(1);
      verify_near_max_scale(BunchKaufman<MatrixType, Lower>(A), b, e2);
      verify_near_max_scale(BunchKaufman<MatrixType, Upper>(A), b, e2);
    }
  }
}

// Regression: the size constructor must pre-allocate the panel workspace so that a subsequent compute()
// on a problem of that size performs no heap allocation. n is chosen above the panel width so the
// blocked path (the one that uses the workspace) runs. (Uses the default stack-allocation limit so the
// trailing-update GEMM's small blocking buffers stay on the stack rather than the heap.)
template <typename Scalar>
void bunchkaufman_no_malloc() {
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixType;
  const Index n = internal::bunch_kaufman_blocksize<Scalar>() + 36;
  const MatrixType M = MatrixType::Random(n, n);
  const MatrixType A = M + M.adjoint();
  BunchKaufman<MatrixType> bk(n);  // pre-allocates m_matrix, m_transpositions, m_subdiag, m_workspace
  internal::set_is_malloc_allowed(false);
  bk.compute(A);
  internal::set_is_malloc_allowed(true);
  VERIFY(bk.info() == Success);
}

EIGEN_DECLARE_TEST(bunchkaufman) {
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(bunchkaufman(Matrix<double, 1, 1>()));
    CALL_SUBTEST_2(bunchkaufman(Matrix2d()));
    CALL_SUBTEST_3(bunchkaufman(Matrix3f()));
    CALL_SUBTEST_4(bunchkaufman(Matrix4d()));

    int s = internal::random<int>(1, EIGEN_TEST_MAX_SIZE);
    CALL_SUBTEST_5(bunchkaufman(MatrixXd(s, s)));
    TEST_SET_BUT_UNUSED_VARIABLE(s);

    s = internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2);
    CALL_SUBTEST_6(bunchkaufman(MatrixXcd(s, s)));
    TEST_SET_BUT_UNUSED_VARIABLE(s);

    // Bounded so that the determinant itself, not just its logarithm, stays in range.
    s = internal::random<int>(1, 30);
    CALL_SUBTEST_5(bunchkaufman_determinant<MatrixXd>(s));
    CALL_SUBTEST_6(bunchkaufman_determinant<MatrixXcd>(s));
    TEST_SET_BUT_UNUSED_VARIABLE(s);

    s = internal::random<int>(2, EIGEN_TEST_MAX_SIZE);
    CALL_SUBTEST_5(bunchkaufman_inertia_and_conditioning<MatrixXd>(s));
    s = internal::random<int>(2, EIGEN_TEST_MAX_SIZE / 2);
    CALL_SUBTEST_6(bunchkaufman_inertia_and_conditioning<MatrixXcd>(s));

    CALL_SUBTEST_7(bunchkaufman_small_cases<double>());
    CALL_SUBTEST_7(bunchkaufman_small_cases<std::complex<double> >());
    // Singular matrices in both the unblocked (n=8) and blocked (n=100 > blocksize) regimes.
    CALL_SUBTEST_5(bunchkaufman_singular<double>(8));
    CALL_SUBTEST_5(bunchkaufman_singular<double>(100));
    CALL_SUBTEST_6(bunchkaufman_singular<std::complex<double> >(8));
    CALL_SUBTEST_6(bunchkaufman_singular<std::complex<double> >(100));
    CALL_SUBTEST_5(bunchkaufman_nan<double>());
    CALL_SUBTEST_6(bunchkaufman_nan<std::complex<double> >());
    CALL_SUBTEST_5(bunchkaufman_rank_deficient<double>());
    CALL_SUBTEST_6(bunchkaufman_rank_deficient<std::complex<double> >());
  }

  // Empty-matrix edge case.
  CALL_SUBTEST_5(bunchkaufman(MatrixXd(0, 0)));
  CALL_SUBTEST_5(bunchkaufman_determinant_empty<MatrixXd>());

  // Subnormal 2x2 block: the determinant underflows, its log and sign do not.
  CALL_SUBTEST_5(bunchkaufman_determinant_subnormal_block<MatrixXd>());
  CALL_SUBTEST_6(bunchkaufman_determinant_subnormal_block<MatrixXcd>());
  CALL_SUBTEST_6(bunchkaufman_determinant_subnormal_block_complex());
  CALL_SUBTEST_8(bunchkaufman_determinant_subnormal_block<MatrixXf>());
  CALL_SUBTEST_5(bunchkaufman_inertia_wide_2x2_block());

  // Mixed-scale 2x2 blocks: the block determinants leave the representable range, their product does not.
  CALL_SUBTEST_5(bunchkaufman_determinant_mixed_scale());

  // Problem-size constructors.
  CALL_SUBTEST_8(BunchKaufman<MatrixXf>(10));
  CALL_SUBTEST_8(BunchKaufman<MatrixXcd>(10));

  // Assertion checks on an uninitialized decomposition.
  CALL_SUBTEST_3(bunchkaufman_verify_assert<Matrix3f>());
  CALL_SUBTEST_5(bunchkaufman_verify_assert<MatrixXd>());
  CALL_SUBTEST_6(bunchkaufman_verify_assert<MatrixXcd>());

  // Deterministic blocking / panel-boundary tests (outside g_repeat).
  CALL_SUBTEST_8(bunchkaufman_blocking_boundary<double>());
  CALL_SUBTEST_8(bunchkaufman_blocking_boundary<float>());
  CALL_SUBTEST_8(bunchkaufman_blocking_boundary<std::complex<double> >());

  // Extreme-scale 2x2 pivots: the scaled (det-free) 2x2 formulas must not over/underflow.
  CALL_SUBTEST_7(bunchkaufman_extreme_scale<double>());
  CALL_SUBTEST_7(bunchkaufman_extreme_scale<std::complex<double> >());
  CALL_SUBTEST_5(bunchkaufman_extreme_scale_large<double>(8));
  CALL_SUBTEST_5(bunchkaufman_extreme_scale_large<double>(100));
  CALL_SUBTEST_6(bunchkaufman_extreme_scale_large<std::complex<double> >(8));
  CALL_SUBTEST_6(bunchkaufman_extreme_scale_large<std::complex<double> >(100));

  // Issue #3142: subnormal 2x2 pivot blocks through the unblocked and blocked paths and the D solve.
  CALL_SUBTEST_5(bunchkaufman_subnormal_2x2_block<MatrixXd>());
  CALL_SUBTEST_6(bunchkaufman_subnormal_2x2_block<MatrixXcd>());
  CALL_SUBTEST_8(bunchkaufman_subnormal_2x2_block<MatrixXf>());
  CALL_SUBTEST_8(bunchkaufman_subnormal_2x2_block<MatrixXcf>());
  CALL_SUBTEST_5(bunchkaufman_near_max_scale<MatrixXd>());
  CALL_SUBTEST_6(bunchkaufman_near_max_scale<MatrixXcd>());
  CALL_SUBTEST_8(bunchkaufman_near_max_scale<MatrixXf>());
  CALL_SUBTEST_8(bunchkaufman_near_max_scale<MatrixXcf>());

  // No-malloc regression: the size constructor pre-allocates the panel workspace.
  CALL_SUBTEST_8(bunchkaufman_no_malloc<double>());
  CALL_SUBTEST_8(bunchkaufman_no_malloc<std::complex<double> >());
}
