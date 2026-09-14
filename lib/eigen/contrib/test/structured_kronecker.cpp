// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include "main.h"

#include <contrib/Eigen/StructuredMatrices>

using namespace Eigen;

// Reference dense Kronecker product, built entry-wise and independently of both
// the operator under test and the KroneckerProduct module.
template <typename Scalar>
Matrix<Scalar, Dynamic, Dynamic> reference_kron(const Matrix<Scalar, Dynamic, Dynamic>& A,
                                                const Matrix<Scalar, Dynamic, Dynamic>& B) {
  Matrix<Scalar, Dynamic, Dynamic> K(A.rows() * B.rows(), A.cols() * B.cols());
  for (Index i = 0; i < K.rows(); ++i)
    for (Index j = 0; j < K.cols(); ++j) K(i, j) = A(i / B.rows(), j / B.cols()) * B(i % B.rows(), j % B.cols());
  return K;
}

template <typename Scalar>
void test_kron_product(Index m1, Index n1, Index m2, Index n2) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Mat A = Mat::Random(m1, n1), B = Mat::Random(m2, n2);
  KroneckerOperator<Mat, Mat> K(A, B);
  Mat dense = reference_kron<Scalar>(A, B);
  VERIFY_IS_EQUAL(K.rows(), m1 * m2);
  VERIFY_IS_EQUAL(K.cols(), n1 * n2);

  // Dense assignment through the evaluator and coefficient access.
  Mat Kd = K;
  VERIFY_IS_APPROX(Kd, dense);
  for (Index t = 0; t < 5; ++t) {
    Index i = internal::random<Index>(0, K.rows() - 1), j = internal::random<Index>(0, K.cols() - 1);
    VERIFY_IS_APPROX(K.coeff(i, j), dense(i, j));
  }

  // Fast matrix-vector and matrix-matrix products via the vec identity.
  Vec x = Vec::Random(n1 * n2);
  VERIFY_IS_APPROX((K * x).eval(), (dense * x).eval());
  Mat X = Mat::Random(n1 * n2, 3);
  VERIFY_IS_APPROX((K * X).eval(), (dense * X).eval());

  // A matrix product on the right must be materialized once before addProduct()
  // takes column expressions from it. Keep the outer expression alive past the
  // full expression that created the nested product as a lifetime check too.
  Mat P = Mat::Random(n1 * n2, 4), Q = Mat::Random(4, 3);
  auto nestedProduct = K * (P * Q);
  Mat nestedResult = nestedProduct;
  VERIFY_IS_APPROX(nestedResult, (dense * (P * Q)).eval());

  // Accumulation form exercised by the iterative solvers.
  Vec y = Vec::Random(m1 * m2);
  Vec y0 = y;
  y.noalias() += K * x;
  VERIFY_IS_APPROX(y, (y0 + dense * x).eval());
}

template <typename Scalar>
void test_kron_transpose(Index m1, Index n1, Index m2, Index n2) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Mat A = Mat::Random(m1, n1), B = Mat::Random(m2, n2);
  KroneckerOperator<Mat, Mat> K(A, B);
  Mat dense = reference_kron<Scalar>(A, B);

  Mat Td = K.transpose();
  VERIFY_IS_APPROX(Td, Mat(dense.transpose()));
  Mat Ad = K.adjoint();
  VERIFY_IS_APPROX(Ad, Mat(dense.adjoint()));
  Mat Cd = K.conjugate();
  VERIFY_IS_APPROX(Cd, Mat(dense.conjugate()));

  Vec y = Vec::Random(m1 * m2);
  VERIFY_IS_APPROX((K.transpose() * y).eval(), (dense.transpose() * y).eval());
  VERIFY_IS_APPROX((K.adjoint() * y).eval(), (dense.adjoint() * y).eval());
}

// The products carry the default product tag, so plain assignment materializes a
// temporary exactly like a dense product would. That must resolve every aliasing
// form between the destination and the right-hand side: the same object, an
// expression referencing the destination, and overlapping views -- cases the old
// AliasFreeProduct same-object check could not see.
template <typename Scalar>
void test_kron_aliased_product(Index n1, Index n2) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  // Square factors so that the destination and the right-hand side can alias.
  Mat A = Mat::Random(n1, n1), B = Mat::Random(n2, n2);
  KroneckerOperator<Mat, Mat> K(A, B);
  Mat dense = reference_kron<Scalar>(A, B);

  Vec x = Vec::Random(n1 * n2);
  Vec y = x;
  y = K * y;
  VERIFY_IS_APPROX(y, (dense * x).eval());

  y = x;
  y += K * y;
  VERIFY_IS_APPROX(y, (x + dense * x).eval());

  y = x;
  y -= K * y;
  VERIFY_IS_APPROX(y, (x - dense * x).eval());

  Mat X = Mat::Random(n1 * n2, 3);
  Mat Y = X;
  Y = K * Y;
  VERIFY_IS_APPROX(Y, (dense * X).eval());

  // Right-hand-side expression referencing the destination.
  y = x;
  y = K * (y + Vec::Ones(n1 * n2));
  VERIFY_IS_APPROX(y, (dense * (x + Vec::Ones(n1 * n2))).eval());

  // Overlapping views: the destination shares all but one entry with the
  // right-hand side.
  Vec buf = Vec::Random(n1 * n2 + 1);
  const Vec expected = dense * buf.tail(n1 * n2);
  buf.head(n1 * n2) = K * buf.tail(n1 * n2);
  VERIFY_IS_APPROX(buf.head(n1 * n2).eval(), expected);
}

// Rectangular operators resize an aliased destination: x = K * x must evaluate
// into a temporary before x is resized from cols() to rows() entries.
template <typename Scalar>
void test_kron_aliased_resize(Index m1, Index n1, Index m2, Index n2) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Mat A = Mat::Random(m1, n1), B = Mat::Random(m2, n2);
  KroneckerOperator<Mat, Mat> K(A, B);
  Mat dense = reference_kron<Scalar>(A, B);

  Vec x = Vec::Random(n1 * n2);
  const Vec xref = x;
  x = K * x;
  VERIFY_IS_EQUAL(x.size(), m1 * m2);
  VERIFY_IS_APPROX(x, (dense * xref).eval());

  Mat X = Mat::Random(n1 * n2, 2);
  const Mat Xref = X;
  X = K * X;
  VERIFY_IS_EQUAL(X.rows(), m1 * m2);
  VERIFY_IS_APPROX(X, (dense * Xref).eval());
}

// transpose()/conjugate()/adjoint() and makeKroneckerOperator() return owning
// temporaries, so the product expression must nest the structured operand by
// value: a delayed-evaluated expression has to outlive the temporary operator it
// was built from. The static check pins the value nesting; the behavioral check
// would read freed memory if the product held a reference instead.
template <typename Scalar>
void test_kron_delayed_product(Index m1, Index n1, Index m2, Index n2) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  STATIC_CHECK((!std::is_reference<typename internal::ref_selector<KroneckerOperator<Mat, Mat>>::type>::value));

  Mat A = Mat::Random(m1, n1), B = Mat::Random(m2, n2);
  KroneckerOperator<Mat, Mat> K(A, B);
  Mat dense = reference_kron<Scalar>(A, B);

  Vec y = Vec::Random(m1 * m2);
  auto expr = K.adjoint() * y;                          // the adjoint temporary dies with the full expression
  Vec scribble = Vec::Random(2 * (m1 * m2 + n1 * n2));  // reuses the temporary's freed heap storage
  Vec x = expr;
  VERIFY_IS_APPROX(x, (dense.adjoint() * y).eval());
  VERIFY_IS_EQUAL(scribble.size(), 2 * (m1 * m2 + n1 * n2));  // keep the scribble alive across the evaluation

  Vec xin = Vec::Random(n1 * n2);
  auto expr2 = makeKroneckerOperator(Mat(A), Mat(B)) * xin;  // factory temporary
  Vec scribble2 = Vec::Random(m1 * m2 + n1 * n2);
  Vec y2 = expr2;
  VERIFY_IS_APPROX(y2, (dense * xin).eval());
  VERIFY_IS_EQUAL(scribble2.size(), m1 * m2 + n1 * n2);
}

// Mixed-scalar products: a real operator applied to a complex right-hand side
// (and a complex operator applied to a real one) promotes to the complex product
// scalar, so alpha, the vec-trick workspaces and the accumulation must run in the
// promoted type rather than the operator scalar.
template <typename RealScalar>
void test_kron_mixed_scalar(Index m1, Index n1, Index m2, Index n2) {
  typedef std::complex<RealScalar> Complex;
  typedef Matrix<RealScalar, Dynamic, 1> RVec;
  typedef Matrix<RealScalar, Dynamic, Dynamic> RMat;
  typedef Matrix<Complex, Dynamic, 1> CVec;
  typedef Matrix<Complex, Dynamic, Dynamic> CMat;

  RMat A = RMat::Random(m1, n1), B = RMat::Random(m2, n2);
  KroneckerOperator<RMat, RMat> K(A, B);
  CMat dense = reference_kron<RealScalar>(A, B).template cast<Complex>();

  CVec x = CVec::Random(n1 * n2);
  CVec y = K * x;
  VERIFY_IS_APPROX(y, (dense * x).eval());

  CVec y0 = CVec::Random(m1 * m2);
  y = y0;
  y.noalias() += K * x;
  VERIFY_IS_APPROX(y, (y0 + dense * x).eval());

  CMat Ac = CMat::Random(m1, n1), Bc = CMat::Random(m2, n2);
  KroneckerOperator<CMat, CMat> Kc(Ac, Bc);
  CMat denseC = reference_kron<Complex>(Ac, Bc);
  RVec xr = RVec::Random(n1 * n2);
  CVec z = Kc * xr;
  VERIFY_IS_APPROX(z, (denseC * xr).eval());
}

template <typename Scalar>
void test_kron_solve(Index n1, Index n2) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  // Diagonal boosts keep both factors (hence the product) well conditioned.
  Mat A = Mat::Random(n1, n1) + RealScalar(2 * n1) * Mat::Identity(n1, n1);
  Mat B = Mat::Random(n2, n2) + RealScalar(2 * n2) * Mat::Identity(n2, n2);
  KroneckerOperator<Mat, Mat> K(A, B);
  Mat dense = reference_kron<Scalar>(A, B);

  Vec b = Vec::Random(n1 * n2);
  Vec x = K.solve(b);
  VERIFY_IS_APPROX((dense * x).eval(), b);
  VERIFY_IS_APPROX(x, dense.partialPivLu().solve(b).eval());

  Mat Bm = Mat::Random(n1 * n2, 3);
  Mat Xm = K.solve(Bm);
  VERIFY_IS_APPROX(Xm, dense.partialPivLu().solve(Bm).eval());
}

template <typename Scalar>
void test_kron_least_squares(Index m1, Index n1, Index m2, Index n2) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Mat A = Mat::Random(m1, n1), B = Mat::Random(m2, n2);
  KroneckerOperator<Mat, Mat> K(A, B);
  Mat dense = reference_kron<Scalar>(A, B);

  // The minimum-norm least-squares solution is unique, so the factored
  // pseudo-inverse must match the dense complete orthogonal decomposition.
  Vec b = Vec::Random(m1 * m2);
  Vec x = K.leastSquaresSolve(b);
  VERIFY_IS_APPROX(x, dense.completeOrthogonalDecomposition().solve(b).eval());
}

template <typename Scalar>
void test_kron_least_squares_rank_deficient(Index m1, Index n1, Index m2, Index n2) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  // A rank-one left factor and a rank-two right factor make the product rank
  // deficient; the minimum-norm least-squares solution stays unique and
  // comparable against the dense complete orthogonal decomposition.
  Mat A = Vec::Random(m1) * Vec::Random(n1).transpose();
  Mat B = Vec::Random(m2) * Vec::Random(n2).transpose() + Vec::Random(m2) * Vec::Random(n2).transpose();
  KroneckerOperator<Mat, Mat> K(A, B);
  Mat dense = reference_kron<Scalar>(A, B);
  VERIFY_IS_EQUAL(K.rank(), dense.completeOrthogonalDecomposition().rank());

  Vec b = Vec::Random(m1 * m2);
  Vec x = K.leastSquaresSolve(b);
  VERIFY_IS_APPROX(x, dense.completeOrthogonalDecomposition().solve(b).eval());
}

// Near-rank-deficiency must be judged at the product level: with
// A = B = diag(1, 1e-8) each factor is full rank against its own threshold, but
// the smallest singular value of the product, 1e-16, falls below the
// product-level threshold min(rows,cols) * eps * sigma_max(A) * sigma_max(B).
// The rank is 3 -- matching the dense complete orthogonal decomposition -- and
// leastSquaresSolve() must truncate the 1e-16 mode instead of inverting it.
void test_kron_product_level_rank() {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef Matrix<double, Dynamic, Dynamic> Mat;

  const double small = 1e-8;  // sqrt(eps)-ish: kept per factor, truncated as a pairwise product
  Mat A(2, 2);
  A << 1, 0, 0, small;
  Mat B = A;
  KroneckerOperator<Mat, Mat> K(A, B);
  Mat dense = reference_kron<double>(A, B);

  CompleteOrthogonalDecomposition<Mat> cod(dense);
  VERIFY_IS_EQUAL(cod.rank(), 3);
  VERIFY_IS_EQUAL(K.rank(), cod.rank());

  Vec b = Vec::Random(4);
  Vec x = K.leastSquaresSolve(b);
  VERIFY_IS_APPROX(x, cod.solve(b).eval());
  // Direct encoding of "the 1e-16 mode is not inverted": the solution is bounded
  // by the reciprocal of the smallest kept singular value (with slack), far below
  // the 1e16 blow-up an inverted product mode would produce.
  const double invertedModeBound = double(10) / small;  // smallest kept product singular value is `small`
  VERIFY(x.norm() <= invertedModeBound * b.norm());
}

// det(A (x) B) = det(A)^n2 * det(B)^n1 must be accumulated with exponent
// balancing: det(A)^n2 or det(B)^n1 can overflow or underflow on their own even
// when the result is representable.
void test_kron_determinant_scaling() {
  typedef Matrix<double, Dynamic, Dynamic> Mat;

  // Reviewer repro: det(A)^2 = 1e400 overflows on its own, yet the determinant
  // of K = diag(1e100, 1e100) is 1e200.
  Mat A(1, 1);
  A << 1e200;
  Mat B = Mat::Zero(2, 2);
  B(0, 0) = B(1, 1) = 1e-100;
  KroneckerOperator<Mat, Mat> K(A, B);
  VERIFY_IS_APPROX(K.determinant(), 1e200);
  VERIFY_IS_APPROX(K.determinant(), reference_kron<double>(A, B).determinant());

  // Underflow-side analogue: det(A2)^2 = 1e-400 underflows on its own, yet the
  // determinant of K2 = diag(1e-100, 1e-100) is 1e-200.
  Mat A2(1, 1);
  A2 << 1e-200;
  Mat B2 = Mat::Zero(2, 2);
  B2(0, 0) = B2(1, 1) = 1e100;
  KroneckerOperator<Mat, Mat> K2(A2, B2);
  VERIFY_IS_APPROX(K2.determinant(), 1e-200);
  VERIFY_IS_APPROX(K2.determinant(), reference_kron<double>(A2, B2).determinant());

  // A genuinely overflowing determinant (1e800) must still give infinity ...
  Mat B3 = Mat::Zero(2, 2);
  B3(0, 0) = B3(1, 1) = 1e200;
  KroneckerOperator<Mat, Mat> K3(A, B3);
  VERIFY((numext::isinf)(K3.determinant()));
  VERIFY(K3.determinant() > 0);

  // ... and a genuinely underflowing one (1e-800) must flush to zero.
  Mat B4 = Mat::Zero(2, 2);
  B4(0, 0) = B4(1, 1) = 1e-200;
  KroneckerOperator<Mat, Mat> K4(A2, B4);
  VERIFY_IS_EQUAL(K4.determinant(), 0.0);
}

// Reviewer repro: (A (x) B) with A = [1e-200] and B = [1e200] is exactly the
// identity, but evaluating (B X) A^T without protection overflows in
// B X = 1e400 and returns Inf. The power-of-two pre-scaling must keep every
// intermediate finite whenever the true result is representable -- and powers of
// two shift only exponents, so no accuracy may be lost.
void test_kron_extreme_scale_product() {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef Matrix<double, Dynamic, Dynamic> Mat;

  Mat A(1, 1), B(1, 1);
  A << 1e-200;
  B << 1e200;
  KroneckerOperator<Mat, Mat> K(A, B);
  Vec x(1);
  x << 1e200;
  Vec y = K * x;
  VERIFY(y.allFinite());
  VERIFY_IS_APPROX(y[0], 1e200);

  // Same magnitudes through solve(): with the factors swapped the operator is
  // still the identity, and the intermediate B^{-1} mat(b) = 1e400 overflows;
  // the normalized-frame solve must return the exact solution.
  KroneckerOperator<Mat, Mat> Ks(B, A);
  Vec b(1);
  b << 1e200;
  Vec xs = Ks.solve(b);
  VERIFY(xs.allFinite());
  VERIFY_IS_APPROX(xs[0], 1e200);

  // Identity built from scaled unit matrices: the first GEMM is ~1e400 while
  // K * x == x exactly.
  Mat A2 = 1e-200 * Mat::Identity(2, 2), B2 = 1e200 * Mat::Identity(3, 3);
  KroneckerOperator<Mat, Mat> K2(A2, B2);
  Vec x2 = 1e200 * Vec::Random(6);
  Vec y2 = K2 * x2;
  VERIFY(y2.allFinite());
  VERIFY_IS_APPROX(y2, x2);

  // Multi-dimensional overflow boundary: the intermediate B X ~ 2^1103 overflows
  // while the true result ~ 2^904 is representable -- and the dense reference can
  // compute it, because the materialized product has moderate entries ~ 2^400.
  Mat A3 = std::ldexp(1.0, -200) * Mat::Random(2, 2);
  Mat B3 = std::ldexp(1.0, 600) * Mat::Random(2, 2);
  KroneckerOperator<Mat, Mat> K3(A3, B3);
  Mat dense3 = reference_kron<double>(A3, B3);
  Vec x3 = std::ldexp(1.0, 500) * Vec::Random(4);
  Vec y3 = K3 * x3;
  VERIFY(y3.allFinite());
  VERIFY_IS_APPROX(y3, (dense3 * x3).eval());

  // A genuinely overflowing result must still saturate to infinity.
  KroneckerOperator<Mat, Mat> K4(B, B);  // [1e200] (x) [1e200]
  Vec y4 = K4 * x;                       // true result 1e600
  VERIFY((numext::isinf)(y4[0]));
  VERIFY(y4[0] > 0);
}

// Complex analogue: the exponent bounds must come from the component-wise
// magnitudes. A finite complex value near the overflow threshold has a
// non-representable modulus, which would silently disable the scaling (and a
// result with components near the threshold has one too, which must not be
// destroyed on the way back).
void test_kron_extreme_scale_complex() {
  typedef std::complex<double> Complex;
  typedef Matrix<Complex, Dynamic, 1> Vec;
  typedef Matrix<Complex, Dynamic, Dynamic> Mat;

  Mat A(1, 1), B(1, 1);
  A << Complex(1e-300, 0.0);
  B << Complex(1e308, 1e308);  // |B(0,0)| overflows; the components are finite
  KroneckerOperator<Mat, Mat> K(A, B);
  Vec x(1);
  x << Complex(1e300, 0.0);
  Vec y = K * x;
  VERIFY(y.allFinite());
  // Exact scalar reference, evaluated in an order that never leaves the
  // representable range: (A * B) * x = (1e8 + 1e8 i) * 1e300.
  const Complex expected = (A(0, 0) * B(0, 0)) * x[0];
  VERIFY_IS_APPROX(y[0], expected);

  Mat A2(1, 1), B2(1, 1);
  A2 << Complex(1e-200, 0.0);
  B2 << Complex(1.2e200, -1.3e200);
  KroneckerOperator<Mat, Mat> K2(A2, B2);
  Vec x2(1);
  x2 << Complex(1.1e200, -0.7e200);
  Vec y2 = K2 * x2;
  VERIFY(y2.allFinite());
  VERIFY_IS_APPROX(y2[0], ((A2(0, 0) * B2(0, 0)) * x2[0]));
}

// Reviewer repro: with A = [1e-310] (subnormal) the absolute rank threshold
// min(rows,cols) * eps * sa_max underflows to zero before the sb_max
// multiplication, so every pairwise singular-value product used to pass and
// rank() returned 8 where the dense SVD says 1. The comparison must happen in
// ratio space: (sa_i/sa_0) * (sb_j/sb_0) against min(rows,cols) * eps.
void test_kron_rank_ratio_threshold() {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef Matrix<double, Dynamic, Dynamic> Mat;

  Mat A(1, 1);
  A << 1e-310;  // subnormal
  Mat B = Mat::Zero(8, 8);
  for (Index i = 0; i < 8; ++i) B(i, i) = std::pow(10.0, 308.0 - 15.0 * double(i));  // 1e308, 1e293, ...
  KroneckerOperator<Mat, Mat> K(A, B);

  // The materialized product has moderate entries (1e-2 down to 1e-107), so the
  // dense SVD is a trustworthy reference: the singular-value ratios decay by
  // 1e-15 per mode, below the 8 * eps ~ 1.8e-15 threshold, leaving rank one.
  Mat dense = reference_kron<double>(A, B);
  JacobiSVD<Mat, ComputeThinU | ComputeThinV> svd(dense);
  VERIFY_IS_EQUAL(svd.rank(), 1);
  VERIFY_IS_EQUAL(K.rank(), svd.rank());

  // leastSquaresSolve must truncate the seven sub-threshold modes instead of
  // inverting them: inverting even the second one would blow the solution up by
  // ~1e17 (the last by ~1e107). The kept mode has sigma = 1e-2.
  Vec b = Vec::Random(8);
  Vec x = K.leastSquaresSolve(b);
  VERIFY(x.allFinite());
  VERIFY_IS_APPROX(x, svd.solve(b).eval());
  VERIFY(x.norm() <= 1e3 * b.norm());

  // An exactly zero factor zeroes the whole operator: rank 0 and a zero
  // pseudo-inverse (instead of 0/0 singular-value ratios).
  Mat Z = Mat::Zero(2, 2);
  KroneckerOperator<Mat, Mat> Kz(Z, B.topLeftCorner(2, 2).eval());
  VERIFY_IS_EQUAL(Kz.rank(), 0);
  Vec bz = Vec::Random(4);
  VERIFY_IS_EQUAL(Kz.leastSquaresSolve(bz).norm(), 0.0);
}

// The ratio test alone dropped SVDBase's smallest-normal clamp: a subnormal
// product singular value, whose reciprocal overflows, must count as an exact
// zero. A = [1], B = [DBL_MIN/2] has the singular value DBL_MIN/2: the dense
// rank is 0 and the pseudo-inverse is zero, but without the clamp the
// structured rank was 1 and leastSquaresSolve returned ~9e307. The boundary
// DBL_MIN itself stays rank one, matching SVDBase::rank()'s >= convention.
void test_kron_rank_min_normal_clamp() {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef Matrix<double, Dynamic, Dynamic> Mat;
  const double mn = (std::numeric_limits<double>::min)();

  Mat A(1, 1), B(1, 1);
  A << 1.0;
  B << mn / 2;  // subnormal
  KroneckerOperator<Mat, Mat> K(A, B);
  Mat dense = reference_kron<double>(A, B);
  JacobiSVD<Mat, ComputeThinU | ComputeThinV> svd(dense);
  VERIFY_IS_EQUAL(svd.rank(), 0);
  VERIFY_IS_EQUAL(K.rank(), svd.rank());
  Vec b(1);
  b << 1.0;
  VERIFY_IS_EQUAL(K.leastSquaresSolve(b).norm(), 0.0);
  VERIFY_IS_EQUAL(svd.solve(b).norm(), 0.0);

  // Boundary: the smallest normal number is the smallest kept singular value.
  Mat B2(1, 1);
  B2 << mn;
  KroneckerOperator<Mat, Mat> K2(A, B2);
  JacobiSVD<Mat> svd2(reference_kron<double>(A, B2));
  VERIFY_IS_EQUAL(svd2.rank(), 1);
  VERIFY_IS_EQUAL(K2.rank(), svd2.rank());
  Vec x2 = K2.leastSquaresSolve(b);
  VERIFY(x2.allFinite());
  VERIFY_IS_APPROX(x2[0], 1.0 / mn);
}

// solve() must run in a normalized frame; the previous detect-and-retry missed
// (a) silent underflow -- the intermediate B^-1 mat(b) flushes to zero, which is
// finite, so nothing triggered a retry -- and (b) factors whose raw inverse is
// not representable even against a unit right-hand side.
void test_kron_solve_normalized() {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef Matrix<double, Dynamic, Dynamic> Mat;

  // (a) Identity operator, tiny right-hand side: B^-1 * b = 1e-400 used to
  // underflow silently to zero.
  Mat A(1, 1), B(1, 1);
  A << 1e-200;
  B << 1e200;
  KroneckerOperator<Mat, Mat> K(A, B);
  Vec b(1);
  b << 1e-200;
  Vec x = K.solve(b);
  VERIFY_IS_APPROX(x[0], 1e-200);

  // (b) B^-1 ~ 1e310 is not representable, yet the operator is ~0.01 and the
  // solution ~100: in the normalized frame every intermediate is bounded by the
  // conditioning of the factors.
  Mat A2(1, 1), B2(1, 1);
  A2 << 1e308;
  B2 << 1e-310;  // subnormal
  KroneckerOperator<Mat, Mat> K2(A2, B2);
  Vec b2(1);
  b2 << 1.0;
  Vec x2 = K2.solve(b2);
  VERIFY(x2.allFinite());
  VERIFY_IS_APPROX(x2[0], 1.0 / (A2(0, 0) * B2(0, 0)));
}

// Moderate inputs must be bit-identical to the unnormalized evaluation: partial
// pivoting is invariant under the uniform power-of-two normalization and every
// substitution step scales exactly with it.
template <typename Scalar>
void test_kron_solve_bit_identity(Index n1, Index n2) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;
  // The reference below reproduces the member-internal evaluation, which pins
  // its workspaces to column-major storage.
  typedef Matrix<Scalar, Dynamic, Dynamic, ColMajor> CmMat;

  Mat A = Mat::Random(n1, n1) + RealScalar(2 * n1) * Mat::Identity(n1, n1);
  Mat B = Mat::Random(n2, n2) + RealScalar(2 * n2) * Mat::Identity(n2, n2);
  KroneckerOperator<Mat, Mat> K(A, B);

  Mat Bm = Mat::Random(n1 * n2, 2);
  Mat X = K.solve(Bm);

  // Raw, unnormalized reference evaluation.
  PartialPivLU<CmMat> luA(A), luB(B);
  for (Index k = 0; k < Bm.cols(); ++k) {
    const Vec bc = Bm.col(k);
    const Map<const CmMat> Bmat(bc.data(), n2, n1);
    const CmMat Xref = luA.solve(luB.solve(Bmat).transpose()).transpose();
    const Map<const Vec> xref(Xref.data(), Xref.size());
    VERIFY_IS_EQUAL(X.col(k).eval(), Vec(xref));
  }
}

template <typename Scalar>
void test_kron_eigen(Index n1, Index n2) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef std::complex<RealScalar> Complex;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;
  typedef Matrix<Complex, Dynamic, Dynamic> CMat;
  typedef Matrix<Complex, Dynamic, 1> CVec;

  Mat A = Mat::Random(n1, n1), B = Mat::Random(n2, n2);
  KroneckerOperator<Mat, Mat> K(A, B);
  CMat dense = reference_kron<Scalar>(A, B).template cast<Complex>();

  CVec lambda = K.eigenvalues();
  CMat V = K.eigenvectors();  // materializes the Kronecker eigenvector operator
  VERIFY_IS_APPROX((dense * V).eval(), (V * lambda.asDiagonal()).eval());
}

template <typename Scalar>
void test_kron_svd(Index m1, Index n1, Index m2, Index n2) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Mat A = Mat::Random(m1, n1), B = Mat::Random(m2, n2);
  KroneckerOperator<Mat, Mat> K(A, B);
  Mat dense = reference_kron<Scalar>(A, B);

  Matrix<RealScalar, Dynamic, 1> sv = K.singularValues();
  Mat U = K.matrixU(), V = K.matrixV();
  const Index k = sv.size();
  VERIFY_IS_EQUAL(U.cols(), k);
  VERIFY_IS_EQUAL(V.cols(), k);

  // Thin SVD in Kronecker order: U and V have orthonormal columns and
  // U * diag(sv) * V^H reconstructs the matrix.
  VERIFY_IS_APPROX((U.adjoint() * U).eval(), Mat(Mat::Identity(k, k)));
  VERIFY_IS_APPROX((V.adjoint() * V).eval(), Mat(Mat::Identity(k, k)));
  VERIFY_IS_APPROX((U * sv.template cast<Scalar>().asDiagonal() * V.adjoint()).eval(), dense);

  // The value multiset matches the dense SVD (compare sorted).
  Matrix<RealScalar, Dynamic, 1> svSorted = sv;
  std::sort(svSorted.data(), svSorted.data() + svSorted.size(), std::greater<RealScalar>());
  JacobiSVD<Mat> svd(dense);
  VERIFY_IS_APPROX(svSorted, svd.singularValues().head(k).eval());
  if (svd.singularValues().size() > k)  // rectangular shapes pad with structural zeros
    VERIFY(svd.singularValues().tail(svd.singularValues().size() - k).norm() <=
           RealScalar(100) * NumTraits<RealScalar>::epsilon() * svd.singularValues()[0]);
}

template <typename Scalar>
void test_kron_inverse_determinant(Index n1, Index n2) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  // Modest diagonal boost: invertible, with the determinant's magnitude tame
  // enough for a relative comparison in double precision.
  Mat A = Mat::Random(n1, n1) + RealScalar(2) * Mat::Identity(n1, n1);
  Mat B = Mat::Random(n2, n2) + RealScalar(2) * Mat::Identity(n2, n2);
  KroneckerOperator<Mat, Mat> K(A, B);
  Mat dense = reference_kron<Scalar>(A, B);

  Mat inv = K.inverse();
  VERIFY_IS_APPROX((inv * dense).eval(), Mat(Mat::Identity(dense.rows(), dense.cols())));
  VERIFY_IS_APPROX(K.determinant(), dense.determinant());
}

template <typename Scalar>
void test_kron_matrix_free_cg(Index n1, Index n2) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  // SPD factors make an SPD Kronecker product.
  Mat Ar = Mat::Random(n1, n1), Br = Mat::Random(n2, n2);
  Mat A = Ar * Ar.adjoint() + RealScalar(n1) * Mat::Identity(n1, n1);
  Mat B = Br * Br.adjoint() + RealScalar(n2) * Mat::Identity(n2, n2);
  KroneckerOperator<Mat, Mat> K(A, B);
  Mat dense = reference_kron<Scalar>(A, B);

  Vec b = Vec::Random(n1 * n2);
  ConjugateGradient<KroneckerOperator<Mat, Mat>, Lower | Upper, IdentityPreconditioner> cg;
  cg.compute(K);
  Vec x = cg.solve(b);
  VERIFY(cg.info() == Success);
  VERIFY_IS_APPROX((dense * x).eval(), b);
}

template <typename Scalar, int M1, int N1, int M2, int N2>
void test_kron_fixed() {
  typedef Matrix<Scalar, M1, N1> MatA;
  typedef Matrix<Scalar, M2, N2> MatB;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  MatA A = MatA::Random();
  MatB B = MatB::Random();
  KroneckerOperator<MatA, MatB> K(A, B);
  STATIC_CHECK((KroneckerOperator<MatA, MatB>::RowsAtCompileTime == M1 * M2));
  STATIC_CHECK((KroneckerOperator<MatA, MatB>::ColsAtCompileTime == N1 * N2));
  STATIC_CHECK((internal::remove_all_t<decltype(makeKroneckerOperator(A, B))>::RowsAtCompileTime == M1 * M2));
  STATIC_CHECK((internal::remove_all_t<decltype(K.transpose())>::RowsAtCompileTime == N1 * N2));
  STATIC_CHECK((internal::remove_all_t<decltype(K.transpose())>::ColsAtCompileTime == M1 * M2));

  Mat dense = reference_kron<Scalar>(Mat(A), Mat(B));
  Matrix<Scalar, M1 * M2, N1 * N2> Kd = K;
  VERIFY_IS_APPROX(Mat(Kd), dense);

  Matrix<Scalar, N1 * N2, 1> x = Matrix<Scalar, N1 * N2, 1>::Random();
  Matrix<Scalar, M1 * M2, 1> y = K * x;
  VERIFY_IS_APPROX(y, (dense * x).eval());

  // The solvers must propagate the compile-time product dimension instead of
  // collapsing to Dynamic rows.
  typedef Matrix<Scalar, M1 * M2, 1> FixedB;
  STATIC_CHECK((internal::remove_all_t<decltype(std::declval<const KroneckerOperator<MatA, MatB>&>().leastSquaresSolve(
                    std::declval<const FixedB&>()))>::RowsAtCompileTime == N1 * N2));
}

// solve()/leastSquaresSolve() on fixed-size square factors: the returned type
// carries the fixed row count, and the values match the dense references.
template <typename Scalar, int N1, int N2>
void test_kron_fixed_solve() {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, N1, N1> MatA;
  typedef Matrix<Scalar, N2, N2> MatB;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  MatA A = MatA::Random() + RealScalar(2 * N1) * MatA::Identity();
  MatB B = MatB::Random() + RealScalar(2 * N2) * MatB::Identity();
  KroneckerOperator<MatA, MatB> K(A, B);
  Mat dense = reference_kron<Scalar>(Mat(A), Mat(B));

  const Matrix<Scalar, N1 * N2, 1> b = Matrix<Scalar, N1 * N2, 1>::Random();
  const auto x = K.solve(b);
  STATIC_CHECK((internal::remove_all_t<decltype(x)>::RowsAtCompileTime == N1 * N2));
  VERIFY_IS_APPROX((dense * Vec(x)).eval(), Vec(b));

  const auto xls = K.leastSquaresSolve(b);
  STATIC_CHECK((internal::remove_all_t<decltype(xls)>::RowsAtCompileTime == N1 * N2));
  VERIFY_IS_APPROX(Vec(xls), Vec(x));
}

// ---- Diagonal and identity factors -----------------------------------------

// Diagonal factors: D (x) B, B (x) D and D1 (x) D2 must match the densified
// reference through dense assignment, coefficient access and every product
// path, while storing only the diagonals.
template <typename Scalar>
void test_kron_diag_product(Index n1, Index m2, Index n2) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;
  typedef DiagonalMatrix<Scalar, Dynamic> Diag;

  const Diag D1(Vec::Random(n1)), D2(Vec::Random(n2));
  Mat B = Mat::Random(m2, n2);
  const Mat D1d = D1.toDenseMatrix(), D2d = D2.toDenseMatrix();

  KroneckerOperator<Diag, Mat> KL(D1, B);    // D1 (x) B
  KroneckerOperator<Mat, Diag> KR(B, D2);    // B (x) D2
  KroneckerOperator<Diag, Diag> KD(D1, D2);  // D1 (x) D2

  Mat refL = reference_kron<Scalar>(D1d, B);
  Mat refR = reference_kron<Scalar>(B, D2d);
  Mat refD = reference_kron<Scalar>(D1d, D2d);

  // Dense assignment through evalTo (a diagonal factor writes zero blocks off
  // its diagonal) and coefficient access.
  VERIFY_IS_APPROX(Mat(KL), refL);
  VERIFY_IS_APPROX(Mat(KR), refR);
  VERIFY_IS_APPROX(Mat(KD), refD);
  for (Index t = 0; t < 5; ++t) {
    Index i = internal::random<Index>(0, KL.rows() - 1), j = internal::random<Index>(0, KL.cols() - 1);
    VERIFY_IS_APPROX(KL.coeff(i, j), refL(i, j));
    i = internal::random<Index>(0, KD.rows() - 1), j = internal::random<Index>(0, KD.cols() - 1);
    VERIFY_IS_APPROX(KD.coeff(i, j), refD(i, j));
  }

  // Dense accumulation through addTo/subTo (a diagonal factor accumulates only
  // the block diagonals).
  Mat acc = Mat::Random(refL.rows(), refL.cols());
  const Mat acc0 = acc;
  acc += KL;
  VERIFY_IS_APPROX(acc, (acc0 + refL).eval());
  acc -= KL;
  VERIFY_IS_APPROX(acc, acc0);

  // Vector, matrix and accumulated products via the vec identity.
  Vec x = Vec::Random(refL.cols());
  VERIFY_IS_APPROX((KL * x).eval(), (refL * x).eval());
  Mat X = Mat::Random(refL.cols(), 3);
  VERIFY_IS_APPROX((KL * X).eval(), (refL * X).eval());
  Vec y = Vec::Random(refL.rows());
  Vec y0 = y;
  y.noalias() += KL * x;
  VERIFY_IS_APPROX(y, (y0 + refL * x).eval());

  Vec xr = Vec::Random(refR.cols());
  VERIFY_IS_APPROX((KR * xr).eval(), (refR * xr).eval());
  Vec xd = Vec::Random(refD.cols());
  VERIFY_IS_APPROX((KD * xd).eval(), (refD * xd).eval());

  // Factory deduction from a diagonal expression maps to an owning
  // DiagonalMatrix factor.
  auto KF = makeKroneckerOperator(D1.diagonal().asDiagonal(), B);
  STATIC_CHECK((std::is_same<decltype(KF), KroneckerOperator<Diag, Mat>>::value));
  VERIFY_IS_APPROX((KF * x).eval(), (refL * x).eval());
}

// Identity factors, the I (x) A and A (x) I operators of finite-difference
// discretizations: a unit-diagonal factor stores O(p) data and applies as a
// no-op scaling instead of a GEMM. Checked against the dense reference and
// against the vec identity (I (x) A) vec(X) = vec(A X) directly.
template <typename Scalar>
void test_kron_identity(Index p, Index m2, Index n2) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;
  typedef Matrix<Scalar, Dynamic, Dynamic, ColMajor> CmMat;  // reshape frame of the vec identity
  typedef DiagonalMatrix<Scalar, Dynamic> Diag;

  Mat A = Mat::Random(m2, n2);
  auto KI = makeKroneckerOperator(Vec::Ones(p).asDiagonal(), A);  // I_p (x) A
  auto IK = makeKroneckerOperator(A, Vec::Ones(p).asDiagonal());  // A (x) I_p
  STATIC_CHECK((std::is_same<decltype(KI), KroneckerOperator<Diag, Mat>>::value));
  STATIC_CHECK((std::is_same<decltype(IK), KroneckerOperator<Mat, Diag>>::value));

  const Mat Ip = Mat::Identity(p, p);
  Mat refI = reference_kron<Scalar>(Ip, A);
  Mat refK = reference_kron<Scalar>(A, Ip);

  Vec x = Vec::Random(p * n2);
  VERIFY_IS_APPROX((KI * x).eval(), (refI * x).eval());
  VERIFY_IS_APPROX((IK * x).eval(), (refK * x).eval());

  // (I_p (x) A) vec(X) = vec(A X) with X of size n2 x p: block-wise application
  // of A to each length-n2 segment.
  const Map<const CmMat> Xmat(x.data(), n2, p);
  CmMat Y = A * Xmat;
  VERIFY_IS_APPROX((KI * x).eval(), Vec(Map<const Vec>(Y.data(), Y.size())));
}

// The transposition family of a diagonal factor stays diagonal, in type and in
// value.
template <typename Scalar>
void test_kron_diag_transpose(Index n1, Index m2, Index n2) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;
  typedef DiagonalMatrix<Scalar, Dynamic> Diag;

  const Diag D(Vec::Random(n1));
  Mat B = Mat::Random(m2, n2);
  KroneckerOperator<Diag, Mat> K(D, B);
  Mat dense = reference_kron<Scalar>(D.toDenseMatrix(), B);

  STATIC_CHECK((std::is_same<typename internal::remove_all_t<decltype(K.transpose())>,
                             KroneckerOperator<Diag, Matrix<Scalar, Dynamic, Dynamic>>>::value));
  STATIC_CHECK((std::is_same<typename internal::remove_all_t<decltype(K.adjoint())>,
                             KroneckerOperator<Diag, Matrix<Scalar, Dynamic, Dynamic>>>::value));
  STATIC_CHECK(
      (std::is_same<typename internal::remove_all_t<decltype(K.conjugate())>, KroneckerOperator<Diag, Mat>>::value));

  Mat Td = K.transpose();
  VERIFY_IS_APPROX(Td, Mat(dense.transpose()));
  Mat Ad = K.adjoint();
  VERIFY_IS_APPROX(Ad, Mat(dense.adjoint()));
  Mat Cd = K.conjugate();
  VERIFY_IS_APPROX(Cd, Mat(dense.conjugate()));

  Vec y = Vec::Random(dense.rows());
  VERIFY_IS_APPROX((K.transpose() * y).eval(), (dense.transpose() * y).eval());
  VERIFY_IS_APPROX((K.adjoint() * y).eval(), (dense.adjoint() * y).eval());
}

// solve() through diagonal factors: a diagonal side is an entrywise division
// instead of an LU substitution. All three factor-kind mixes against the dense
// LU solve.
template <typename Scalar>
void test_kron_diag_solve(Index n1, Index n2) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;
  typedef DiagonalMatrix<Scalar, Dynamic> Diag;

  // Diagonal entries bounded away from zero and diagonally boosted dense
  // factors keep everything well conditioned.
  const Diag D1(Vec::Random(n1) + Vec::Constant(n1, Scalar(2)));
  const Diag D2(Vec::Random(n2) + Vec::Constant(n2, Scalar(2)));
  Mat A = Mat::Random(n1, n1) + RealScalar(2 * n1) * Mat::Identity(n1, n1);
  Mat B = Mat::Random(n2, n2) + RealScalar(2 * n2) * Mat::Identity(n2, n2);

  KroneckerOperator<Diag, Mat> KL(D1, B);
  KroneckerOperator<Mat, Diag> KR(A, D2);
  KroneckerOperator<Diag, Diag> KD(D1, D2);

  Mat denseL = reference_kron<Scalar>(D1.toDenseMatrix(), B);
  Mat denseR = reference_kron<Scalar>(A, D2.toDenseMatrix());
  Mat denseD = reference_kron<Scalar>(D1.toDenseMatrix(), D2.toDenseMatrix());

  Vec b = Vec::Random(n1 * n2);
  VERIFY_IS_APPROX((denseL * KL.solve(b)).eval(), b);
  VERIFY_IS_APPROX(KL.solve(b), denseL.partialPivLu().solve(b).eval());
  VERIFY_IS_APPROX(KR.solve(b), denseR.partialPivLu().solve(b).eval());
  VERIFY_IS_APPROX(KD.solve(b), denseD.partialPivLu().solve(b).eval());

  Mat Bm = Mat::Random(n1 * n2, 3);
  VERIFY_IS_APPROX(KL.solve(Bm), denseL.partialPivLu().solve(Bm).eval());
  VERIFY_IS_APPROX(KR.solve(Bm), denseR.partialPivLu().solve(Bm).eval());
}

// inverse() of a diagonal factor stays diagonal (entrywise reciprocals);
// determinant() accumulates directly from the diagonal, skipping the LU.
template <typename Scalar>
void test_kron_diag_inverse_determinant(Index n1, Index n2) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;
  typedef DiagonalMatrix<Scalar, Dynamic> Diag;

  const Diag D(Vec::Random(n1) + Vec::Constant(n1, Scalar(2)));
  Mat B = Mat::Random(n2, n2) + RealScalar(2) * Mat::Identity(n2, n2);
  KroneckerOperator<Diag, Mat> K(D, B);
  Mat dense = reference_kron<Scalar>(D.toDenseMatrix(), B);

  auto Kinv = K.inverse();
  STATIC_CHECK((std::is_same<typename internal::remove_all_t<decltype(Kinv)>,
                             KroneckerOperator<Diag, Matrix<Scalar, Dynamic, Dynamic, ColMajor>>>::value));
  Mat inv = Kinv;
  VERIFY_IS_APPROX((inv * dense).eval(), Mat(Mat::Identity(dense.rows(), dense.cols())));
  VERIFY_IS_APPROX(K.determinant(), dense.determinant());

  const Diag D2(Vec::Random(n2) + Vec::Constant(n2, Scalar(2)));
  KroneckerOperator<Diag, Diag> KD(D, D2);
  Mat denseD = reference_kron<Scalar>(D.toDenseMatrix(), D2.toDenseMatrix());
  VERIFY_IS_APPROX(KD.determinant(), denseD.determinant());
  Mat invD = KD.inverse();
  VERIFY_IS_APPROX((invD * denseD).eval(), Mat(Mat::Identity(denseD.rows(), denseD.cols())));
}

// The decomposition family on diagonal factors (materialized densely for the
// factor decompositions): eigen-residual, SVD reconstruction, rank and
// minimum-norm least squares against the dense references.
template <typename Scalar>
void test_kron_diag_decompositions(Index n1, Index n2) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef std::complex<RealScalar> Complex;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;
  typedef Matrix<Complex, Dynamic, Dynamic> CMat;
  typedef Matrix<Complex, Dynamic, 1> CVec;
  typedef DiagonalMatrix<Scalar, Dynamic> Diag;

  const Diag D(Vec::Random(n1));
  Mat B = Mat::Random(n2, n2);
  KroneckerOperator<Diag, Mat> K(D, B);
  Mat dense = reference_kron<Scalar>(D.toDenseMatrix(), B);

  CVec lambda = K.eigenvalues();
  CMat V = K.eigenvectors();
  VERIFY_IS_APPROX((dense.template cast<Complex>() * V).eval(), (V * lambda.asDiagonal()).eval());

  Matrix<RealScalar, Dynamic, 1> sv = K.singularValues();
  Mat U = K.matrixU(), W = K.matrixV();
  VERIFY_IS_APPROX((U * sv.template cast<Scalar>().asDiagonal() * W.adjoint()).eval(), dense);

  VERIFY_IS_EQUAL(K.rank(), dense.completeOrthogonalDecomposition().rank());
  Vec b = Vec::Random(n1 * n2);
  VERIFY_IS_APPROX(K.leastSquaresSolve(b), dense.completeOrthogonalDecomposition().solve(b).eval());
}

// The overflow-hardening contract must hold through diagonal factors: products
// and solves with extreme diagonal magnitudes stay finite whenever the true
// result is representable, and the balanced determinant survives partial
// products that overflow on their own.
void test_kron_diag_extreme_scale() {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef Matrix<double, Dynamic, Dynamic> Mat;
  typedef DiagonalMatrix<double, Dynamic> Diag;

  // diag(1e-200) (x) [1e200] is the identity; the unprotected application
  // B X = 1e400 would overflow.
  Diag D(Vec::Constant(1, 1e-200));
  Mat B(1, 1);
  B << 1e200;
  KroneckerOperator<Diag, Mat> K(D, B);
  Vec x(1);
  x << 1e200;
  Vec y = K * x;
  VERIFY(y.allFinite());
  VERIFY_IS_APPROX(y[0], 1e200);

  // Same magnitudes through solve(), diagonal on the divided side.
  KroneckerOperator<Mat, Diag> Ks(B, D);
  Vec b(1);
  b << 1e200;
  Vec xs = Ks.solve(b);
  VERIFY(xs.allFinite());
  VERIFY_IS_APPROX(xs[0], 1e200);

  // Identity built from extreme diagonal scales: products and the entrywise
  // divisions of the diagonal-diagonal solve run in the normalized frame.
  Diag Da(Vec::Constant(2, 1e-200)), Db(Vec::Constant(3, 1e200));
  KroneckerOperator<Diag, Diag> K2(Da, Db);
  Vec x2 = 1e200 * Vec::Random(6);
  Vec y2 = K2 * x2;
  VERIFY(y2.allFinite());
  VERIFY_IS_APPROX(y2, x2);
  Vec b2 = 1e-200 * Vec::Random(6);
  Vec xd = K2.solve(b2);
  VERIFY(xd.allFinite());
  VERIFY_IS_APPROX(xd, b2);

  // det(diag(1e200))^2 = 1e400 overflows on its own, yet the determinant of
  // diag(1e200) (x) diag(1e-100, 1e-100) is 1e200 -- and the diagonal path
  // must balance exactly like the LU path.
  Diag A1(Vec::Constant(1, 1e200)), B1(Vec::Constant(2, 1e-100));
  KroneckerOperator<Diag, Diag> Kd(A1, B1);
  VERIFY_IS_APPROX(Kd.determinant(), 1e200);
}

// Mixed-scalar promotion through a diagonal factor: a real operator applied to
// a complex right-hand side runs in the promoted scalar.
template <typename RealScalar>
void test_kron_diag_mixed_scalar(Index n1, Index m2, Index n2) {
  typedef std::complex<RealScalar> Complex;
  typedef Matrix<RealScalar, Dynamic, 1> RVec;
  typedef Matrix<RealScalar, Dynamic, Dynamic> RMat;
  typedef Matrix<Complex, Dynamic, 1> CVec;
  typedef Matrix<Complex, Dynamic, Dynamic> CMat;
  typedef DiagonalMatrix<RealScalar, Dynamic> RDiag;

  const RDiag D(RVec::Random(n1));
  RMat B = RMat::Random(m2, n2);
  KroneckerOperator<RDiag, RMat> K(D, B);
  CMat dense = reference_kron<RealScalar>(D.toDenseMatrix(), B).template cast<Complex>();

  CVec x = CVec::Random(n1 * n2);
  CVec y = K * x;
  VERIFY_IS_APPROX(y, (dense * x).eval());
  CVec y0 = CVec::Random(n1 * m2);
  y = y0;
  y.noalias() += K * x;
  VERIFY_IS_APPROX(y, (y0 + dense * x).eval());
}

// Fixed-size diagonal factors propagate compile-time dimensions.
template <typename Scalar, int N1, int M2, int N2>
void test_kron_diag_fixed() {
  typedef DiagonalMatrix<Scalar, N1> Diag;
  typedef Matrix<Scalar, M2, N2> MatB;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  const Diag D(Matrix<Scalar, N1, 1>::Random());
  MatB B = MatB::Random();
  KroneckerOperator<Diag, MatB> K(D, B);
  STATIC_CHECK((KroneckerOperator<Diag, MatB>::RowsAtCompileTime == N1 * M2));
  STATIC_CHECK((KroneckerOperator<Diag, MatB>::ColsAtCompileTime == N1 * N2));

  Mat dense = reference_kron<Scalar>(Mat(D.toDenseMatrix()), Mat(B));
  Matrix<Scalar, N1 * M2, N1 * N2> Kd = K;
  VERIFY_IS_APPROX(Mat(Kd), dense);

  Matrix<Scalar, N1 * N2, 1> x = Matrix<Scalar, N1 * N2, 1>::Random();
  Matrix<Scalar, N1 * M2, 1> y = K * x;
  VERIFY_IS_APPROX(y, (dense * x).eval());
}

// Matrix-free CG through a diagonal-factor operator: a positive diagonal
// tensored with an SPD dense factor is SPD.
void test_kron_diag_matrix_free_cg(Index n1, Index n2) {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef Matrix<double, Dynamic, Dynamic> Mat;
  typedef DiagonalMatrix<double, Dynamic> Diag;

  const Diag D(Vec::Random(n1).cwiseAbs() + Vec::Constant(n1, 1.0));
  Mat Br = Mat::Random(n2, n2);
  Mat B = Br * Br.adjoint() + double(n2) * Mat::Identity(n2, n2);
  KroneckerOperator<Diag, Mat> K(D, B);
  Mat dense = reference_kron<double>(D.toDenseMatrix(), B);

  Vec b = Vec::Random(n1 * n2);
  ConjugateGradient<KroneckerOperator<Diag, Mat>, Lower | Upper, IdentityPreconditioner> cg;
  cg.compute(K);
  Vec x = cg.solve(b);
  VERIFY(cg.info() == Success);
  VERIFY_IS_APPROX((dense * x).eval(), b);
}

EIGEN_DECLARE_TEST(structured_kronecker) {
  for (int i = 0; i < g_repeat; ++i) {
    // Products, dense assignment, coefficient access across factor shapes.
    CALL_SUBTEST_1((test_kron_product<double>(1, 1, 1, 1)));
    CALL_SUBTEST_1((test_kron_product<double>(2, 3, 3, 2)));
    CALL_SUBTEST_1((test_kron_product<double>(4, 4, 5, 5)));
    CALL_SUBTEST_1((test_kron_product<double>(8, 3, 2, 7)));
    CALL_SUBTEST_1((test_kron_product<double>(1, 6, 7, 1)));  // vector-shaped factors
    CALL_SUBTEST_1((test_kron_product<float>(5, 4, 4, 5)));
    CALL_SUBTEST_1((test_kron_product<std::complex<double>>(3, 5, 4, 2)));
    CALL_SUBTEST_1((test_kron_product<std::complex<float>>(4, 3, 3, 4)));

    // Transposition family.
    CALL_SUBTEST_2((test_kron_transpose<double>(4, 6, 5, 3)));
    CALL_SUBTEST_2((test_kron_transpose<std::complex<double>>(3, 4, 5, 2)));

    // Direct solves through factor LU.
    CALL_SUBTEST_3((test_kron_solve<double>(1, 1)));
    CALL_SUBTEST_3((test_kron_solve<double>(4, 7)));
    CALL_SUBTEST_3((test_kron_solve<double>(9, 5)));
    CALL_SUBTEST_3((test_kron_solve<float>(5, 4)));
    CALL_SUBTEST_3((test_kron_solve<std::complex<double>>(6, 4)));
    CALL_SUBTEST_3(test_kron_solve_normalized());
    CALL_SUBTEST_3((test_kron_solve_bit_identity<double>(4, 3)));
    CALL_SUBTEST_3((test_kron_solve_bit_identity<std::complex<double>>(3, 4)));

    // Minimum-norm least squares through factor pseudo-inverses.
    CALL_SUBTEST_3((test_kron_least_squares<double>(8, 5, 7, 4)));  // tall x tall
    CALL_SUBTEST_3((test_kron_least_squares<double>(8, 5, 3, 6)));  // tall x wide
    CALL_SUBTEST_3((test_kron_least_squares<std::complex<double>>(6, 4, 5, 3)));
    CALL_SUBTEST_3((test_kron_least_squares_rank_deficient<double>(6, 4, 5, 3)));
    CALL_SUBTEST_3((test_kron_least_squares_rank_deficient<std::complex<double>>(5, 3, 4, 4)));

    // Eigendecomposition, SVD, inverse, determinant.
    CALL_SUBTEST_4((test_kron_eigen<double>(4, 5)));
    CALL_SUBTEST_4((test_kron_eigen<double>(1, 6)));
    CALL_SUBTEST_4((test_kron_eigen<std::complex<double>>(3, 4)));
    CALL_SUBTEST_4((test_kron_svd<double>(4, 4, 5, 5)));
    CALL_SUBTEST_4((test_kron_svd<double>(6, 4, 3, 5)));  // rectangular: structural zeros
    CALL_SUBTEST_4((test_kron_svd<std::complex<double>>(4, 3, 4, 4)));
    CALL_SUBTEST_4((test_kron_svd<float>(4, 4, 4, 4)));
    CALL_SUBTEST_4((test_kron_inverse_determinant<double>(4, 5)));
    CALL_SUBTEST_4((test_kron_inverse_determinant<std::complex<double>>(3, 4)));

    // Matrix-free iterative solve and fixed-size factors.
    CALL_SUBTEST_5((test_kron_matrix_free_cg<double>(6, 7)));
    CALL_SUBTEST_5((test_kron_fixed<double, 3, 4, 2, 5>()));
    CALL_SUBTEST_5((test_kron_fixed<std::complex<float>, 2, 3, 3, 2>()));
    CALL_SUBTEST_5((test_kron_fixed_solve<double, 2, 3>()));
    CALL_SUBTEST_5((test_kron_fixed_solve<std::complex<double>, 3, 2>()));

    // Lifetime and aliasing of the product expression, mixed-scalar promotion.
    CALL_SUBTEST_6((test_kron_aliased_product<double>(3, 4)));
    CALL_SUBTEST_6((test_kron_aliased_product<std::complex<double>>(4, 3)));
    CALL_SUBTEST_6((test_kron_aliased_resize<double>(4, 3, 2, 5)));
    CALL_SUBTEST_6((test_kron_aliased_resize<std::complex<double>>(2, 4, 5, 3)));
    CALL_SUBTEST_6((test_kron_delayed_product<double>(4, 3, 2, 5)));
    CALL_SUBTEST_6((test_kron_delayed_product<std::complex<float>>(3, 3, 4, 2)));
    CALL_SUBTEST_6((test_kron_mixed_scalar<double>(4, 3, 3, 5)));
    CALL_SUBTEST_6((test_kron_mixed_scalar<float>(3, 4, 2, 3)));

    // Numerical boundaries: product-level rank truncation, balanced determinant,
    // overflow-safe product scaling and ratio-space rank thresholds.
    CALL_SUBTEST_7(test_kron_product_level_rank());
    CALL_SUBTEST_7(test_kron_determinant_scaling());
    CALL_SUBTEST_7(test_kron_extreme_scale_product());
    CALL_SUBTEST_7(test_kron_extreme_scale_complex());
    CALL_SUBTEST_7(test_kron_rank_ratio_threshold());
    CALL_SUBTEST_7(test_kron_rank_min_normal_clamp());

    // Diagonal and identity factors: products, transposition family,
    // mixed-scalar promotion, fixed sizes.
    CALL_SUBTEST_8((test_kron_diag_product<double>(4, 5, 3)));
    CALL_SUBTEST_8((test_kron_diag_product<float>(3, 4, 4)));
    CALL_SUBTEST_8((test_kron_diag_product<std::complex<double>>(3, 4, 2)));
    CALL_SUBTEST_8((test_kron_identity<double>(5, 4, 3)));
    CALL_SUBTEST_8((test_kron_identity<std::complex<double>>(3, 3, 4)));
    CALL_SUBTEST_8((test_kron_diag_transpose<double>(4, 5, 3)));
    CALL_SUBTEST_8((test_kron_diag_transpose<std::complex<double>>(3, 4, 5)));
    CALL_SUBTEST_8((test_kron_diag_mixed_scalar<double>(4, 3, 5)));
    CALL_SUBTEST_8((test_kron_diag_mixed_scalar<float>(3, 4, 2)));
    CALL_SUBTEST_8((test_kron_diag_fixed<double, 3, 2, 4>()));
    CALL_SUBTEST_8((test_kron_diag_fixed<std::complex<float>, 2, 3, 3>()));

    // Diagonal factors through the solvers, inverse/determinant, the
    // decomposition family, extreme scales and matrix-free CG.
    CALL_SUBTEST_9((test_kron_diag_solve<double>(4, 7)));
    CALL_SUBTEST_9((test_kron_diag_solve<float>(5, 4)));
    CALL_SUBTEST_9((test_kron_diag_solve<std::complex<double>>(6, 4)));
    CALL_SUBTEST_9((test_kron_diag_inverse_determinant<double>(4, 5)));
    CALL_SUBTEST_9((test_kron_diag_inverse_determinant<std::complex<double>>(3, 4)));
    CALL_SUBTEST_9((test_kron_diag_decompositions<double>(4, 5)));
    CALL_SUBTEST_9((test_kron_diag_decompositions<std::complex<double>>(3, 4)));
    CALL_SUBTEST_9(test_kron_diag_extreme_scale());
    CALL_SUBTEST_9(test_kron_diag_matrix_free_cg(6, 7));
  }
}
