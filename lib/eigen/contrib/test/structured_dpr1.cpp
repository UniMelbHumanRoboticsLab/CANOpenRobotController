// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include "main.h"
#include "fp_control.h"

#include <contrib/Eigen/StructuredMatrices>

using namespace Eigen;

template <typename Scalar>
EIGEN_DONT_INLINE Scalar dpr1_subnormal_input_probe() {
  volatile Scalar denorm = (std::numeric_limits<Scalar>::denorm_min)();
  volatile Scalar highest = (std::numeric_limits<Scalar>::max)();
  return denorm * highest;
}

template <typename Scalar>
EIGEN_DONT_INLINE Scalar dpr1_underflow_probe() {
  volatile Scalar normalMin = (std::numeric_limits<Scalar>::min)();
  volatile Scalar oneHalf = Scalar(0.5);
  return normalMin * oneHalf;
}

template <typename Scalar>
bool dpr1_preserves_subnormal_inputs() {
  return std::numeric_limits<Scalar>::has_denorm == std::denorm_present &&
         dpr1_subnormal_input_probe<Scalar>() != Scalar(0);
}

template <typename Scalar>
bool dpr1_has_gradual_underflow() {
  return std::numeric_limits<Scalar>::has_denorm == std::denorm_present && dpr1_underflow_probe<Scalar>() != Scalar(0);
}

// Full quality check of one decomposition: eigenvalues against a dense
// reference, residual ||A*V - V*Lambda||, and -- the litmus test for the
// Gu-Eisenstat machinery -- orthogonality ||V^T V - I||, all relative to
// epsilon-scaled bounds.
template <typename Scalar>
void check_dpr1(const Matrix<Scalar, Dynamic, 1>& d, Scalar rho, const Matrix<Scalar, Dynamic, 1>& z) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  const Index n = d.size();
  Mat dense = Mat(d.asDiagonal()) + rho * z * z.transpose();
  // The natural backward-error scale of the secular formulation is the data
  // scale ||D|| + |rho| ||z||^2 (as for LAPACK's merged D&C problems), which can
  // exceed ||A|| when d + rho*z*z^T cancels; bounds are relative to it.
  Scalar scale = d.cwiseAbs().maxCoeff() + numext::abs(rho) * z.squaredNorm();
  if (scale == Scalar(0)) scale = Scalar(1);  // zero matrix: absolute bounds
  const Scalar tol = Scalar(100) * Scalar(n) * NumTraits<Scalar>::epsilon();

  DPR1EigenSolver<Scalar> es(d, rho, z);
  VERIFY(es.info() == Success);
  const Vec& lambda = es.eigenvalues();
  const Mat& V = es.eigenvectors();

  // Eigenvalues ascending and matching the dense reference to eps * scale.
  for (Index i = 0; i + 1 < n; ++i) VERIFY(lambda[i] <= lambda[i + 1]);
  SelfAdjointEigenSolver<Mat> ref(dense, EigenvaluesOnly);
  VERIFY((lambda - ref.eigenvalues()).cwiseAbs().maxCoeff() <= tol * scale);

  // Residual and orthogonality (the latter is scale-free and holds at O(n*eps)).
  VERIFY((dense * V - V * lambda.asDiagonal()).norm() <= tol * scale * Scalar(numext::sqrt(Scalar(n))));
  VERIFY((V.transpose() * V - Mat::Identity(n, n)).norm() <= tol);

  // The eigenvalues-only path agrees exactly with the full path.
  DPR1EigenSolver<Scalar> esv(d, rho, z, EigenvaluesOnly);
  VERIFY_IS_EQUAL(esv.eigenvalues(), lambda);
}

template <typename Scalar>
void test_dpr1_random(Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  Vec d = Vec::Random(n), z = Vec::Random(n);
  check_dpr1<Scalar>(d, Scalar(1), z);
  check_dpr1<Scalar>(d, Scalar(-1), z);              // negative rho
  check_dpr1<Scalar>(d, Scalar(1e-3), z);            // weak update
  check_dpr1<Scalar>(d, Scalar(50) * Scalar(n), z);  // dominant update
}

// Tight clusters and exact duplicates in the diagonal force the Givens
// deflation path (the secular equation requires strictly separated poles).
template <typename Scalar>
void test_dpr1_clustered(Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  Vec d(n), z = Vec::Random(n);
  for (Index i = 0; i < n; ++i) {
    const Index cluster = i / 3, offsetInCluster = i % 3;
    d[i] = Scalar(1) + Scalar(cluster) + Scalar(1e-15) * Scalar(offsetInCluster);
  }
  check_dpr1<Scalar>(d, Scalar(2), z);

  // All diagonal entries exactly equal: A = c*I + rho*z*z^T has closed-form
  // spectrum {c, ..., c, c + rho*||z||^2}.
  Vec dc = Vec::Constant(n, Scalar(3));
  check_dpr1<Scalar>(dc, Scalar(1), z);
  DPR1EigenSolver<Scalar> es(dc, Scalar(1), z);
  VERIFY(numext::abs(es.eigenvalues()[n - 1] - (Scalar(3) + z.squaredNorm())) <=
         Scalar(100) * Scalar(n) * NumTraits<Scalar>::epsilon() * (Scalar(3) + z.squaredNorm()));
}

// Zero and tiny z entries exercise the z-deflation path.
template <typename Scalar>
void test_dpr1_sparse_z(Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  Vec d = Vec::Random(n);
  d.array() += Scalar(2);
  Vec z = Vec::Zero(n);
  z[1] = Scalar(0.7);
  z[n / 2] = Scalar(-0.3);
  z[n - 1] = Scalar(1e-30);  // far below the deflation threshold
  check_dpr1<Scalar>(d, Scalar(1), z);

  check_dpr1<Scalar>(d, Scalar(1), Vec(Vec::Zero(n)));    // pure diagonal
  check_dpr1<Scalar>(d, Scalar(0), Vec(Vec::Random(n)));  // rho = 0
}

// Edge sizes and the classic textbook case.
template <typename Scalar>
void test_dpr1_edges() {
  typedef Matrix<Scalar, Dynamic, 1> Vec;

  Vec d1(1), z1(1);
  d1 << Scalar(2);
  z1 << Scalar(3);
  DPR1EigenSolver<Scalar> es1(d1, Scalar(2), z1);
  VERIFY(numext::abs(es1.eigenvalues()[0] - Scalar(20)) <= Scalar(100) * NumTraits<Scalar>::epsilon() * Scalar(20));

  Vec d2(2), z2(2);
  d2 << Scalar(1), Scalar(2);
  z2 << Scalar(1), Scalar(1);
  check_dpr1<Scalar>(d2, Scalar(1), z2);

  // d_i = i, z = ones/sqrt(n): every interval holds exactly one root.
  const Index n = 24;
  Vec d(n), z(n);
  for (Index i = 0; i < n; ++i) {
    d[i] = Scalar(i);
    z[i] = Scalar(1);
  }
  z /= z.norm();
  check_dpr1<Scalar>(d, Scalar(1), z);
  check_dpr1<Scalar>(d, Scalar(-1), z);
}

// Unsorted input: the solver must sort internally and undo the permutation.
template <typename Scalar>
void test_dpr1_unsorted(Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  Vec d = Vec::Random(n), z = Vec::Random(n);
  d = d.reverse().eval();
  check_dpr1<Scalar>(d, Scalar(1.5), z);
}

// Rank-one matrix (all-zero diagonal): spectrum {0,...,0, rho*||z||^2}.
template <typename Scalar>
void test_dpr1_rank_one(Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  Vec z = Vec::Random(n);
  check_dpr1<Scalar>(Vec(Vec::Zero(n)), Scalar(2), z);
}

// Wide dynamic range in d and rho scaling.
template <typename Scalar>
void test_dpr1_scaling(Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  Vec d = Vec::Random(n), z = Vec::Random(n);
  d = d.cwiseProduct(Vec::LinSpaced(n, Scalar(1e-8), Scalar(1e8)));
  check_dpr1<Scalar>(d, Scalar(1e6), z);
  check_dpr1<Scalar>(d, Scalar(-1e-6), z);
}

// The Gu-Eisenstat litmus test: poles close enough that naive eigenvectors lose
// orthogonality, but far enough apart (well above the deflation threshold) that
// the secular solver must handle them. Gaps span eps^1 to eps^(1/2) scales.
template <typename Scalar>
void test_dpr1_close_undeflated(Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  const Scalar eps = NumTraits<Scalar>::epsilon();
  Vec d(n), z(n);
  Scalar gap = Scalar(100) * eps;
  for (Index i = 0; i < n; ++i) {
    d[i] = Scalar(1) + Scalar(i) * gap;
    gap *= Scalar(4);  // gaps sweep 1e2*eps ... ~1e6*eps
    if (gap > numext::sqrt(eps)) gap = Scalar(100) * eps;
    z[i] = Scalar(0.3) + Scalar(0.5) * Scalar(i % 7) / Scalar(7);
  }
  check_dpr1<Scalar>(d, Scalar(1), z);
  check_dpr1<Scalar>(d, Scalar(-1), z);
}

// stableNorm keeps huge z representable: with float z ~ 2e19, z.squaredNorm()
// would overflow but rho*||z||^2 is a perfectly ordinary number.
void test_dpr1_huge_z() {
  typedef Matrix<float, Dynamic, 1> Vec;
  Vec d(1), z(1);
  d << 1.0f;
  z << 2e19f;
  DPR1EigenSolver<float> es(d, 1e-30f, z);
  VERIFY(es.info() == Success);
  // Form rho*z^2 = 4e8 one factor at a time: the literal 4e38 already overflows
  // float (an infinite `expected` would make the bound below vacuous).
  const float expected = 1.0f + (1e-30f * 2e19f) * 2e19f;
  VERIFY((numext::isfinite)(expected));
  // Loose bound: expected ~ 4e8 is dominated by the rank-one term, so only its
  // leading digits survive in float.
  const float tol = 1000.0f * NumTraits<float>::epsilon();  // ~1.2e-4
  VERIFY(numext::abs(es.eigenvalues()[0] - expected) <= tol * expected);

  // A spectrum that genuinely overflows (top eigenvalue ~1e320 here) must be
  // reported, not silently deflated. Note rho*||z||^2 overflowing is *not* by
  // itself invalid (see test_dpr1_overflowing_update_representable_spectrum);
  // this input is rejected because the eigenvalue itself is not representable.
  DPR1EigenSolver<double> ov((Matrix<double, Dynamic, 1>(1) << 1.0).finished(), 1e200,
                             (Matrix<double, Dynamic, 1>(1) << 1e60).finished());
  VERIFY(ov.info() == InvalidInput);
}

// Non-finite input must be rejected up front (it would otherwise break the
// sorting comparator and silently deflate everything).
void test_dpr1_nonfinite() {
  typedef Matrix<double, Dynamic, 1> Vec;
  Vec d = Vec::Random(6), z = Vec::Random(6);
  Vec dn = d;
  dn[2] = std::numeric_limits<double>::quiet_NaN();
  DPR1EigenSolver<double> e1(dn, 1.0, z);
  VERIFY(e1.info() == InvalidInput);
  Vec zn = z;
  zn[4] = std::numeric_limits<double>::infinity();
  DPR1EigenSolver<double> e2(d, 1.0, zn);
  VERIFY(e2.info() == InvalidInput);
  DPR1EigenSolver<double> e3(d, std::numeric_limits<double>::quiet_NaN(), z);
  VERIFY(e3.info() == InvalidInput);

  const Vec empty;
  DPR1EigenSolver<double> e4(empty, std::numeric_limits<double>::quiet_NaN(), empty);
  VERIFY(e4.info() == InvalidInput);
  DPR1EigenSolver<double> e5(empty, std::numeric_limits<double>::infinity(), empty);
  VERIFY(e5.info() == InvalidInput);
}

void test_dpr1_invalid_options() {
  typedef Matrix<double, Dynamic, 1> Vec;
  const Vec d = Vec::LinSpaced(3, 1.0, 3.0);
  const Vec z = Vec::Ones(3);
  DPR1EigenSolver<double> es;
  VERIFY_RAISES_ASSERT((es.compute(d, 1.0, z, ComputeEigenvectors | EigenvaluesOnly)));
  VERIFY_RAISES_ASSERT((es.compute(d, 1.0, z, ComputeFullU)));
}

// Regression (MR 2694 review): poles spanning the full exponent range with a huge but representable
// update. Pole differences (d_1 - d_0 = 1.5*DBL_MAX) overflow unless the problem is rescaled by an
// exact power of two; the solver used to report Success with an infinite eigenvalue. The reference is
// a dense solve of the rescaled problem 2^-e * (d, rho), whose spectrum is exactly 2^-e times the
// original.
void test_dpr1_full_range_poles() {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef Matrix<double, Dynamic, Dynamic> Mat;
  const double dmax = (std::numeric_limits<double>::max)();
  Vec d(2), z(2);
  d << -0.75 * dmax, 0.75 * dmax;
  z << 1.0, 1.0;
  const double rho = 1e300;
  DPR1EigenSolver<double> es(d, rho, z);
  VERIFY(es.info() == Success);
  VERIFY(es.eigenvalues().allFinite());

  const int e = 1024;  // 2^-e maps 0.75*DBL_MAX into [0.5, 1)
  Vec dS(2);
  dS << std::ldexp(d[0], -e), std::ldexp(d[1], -e);
  const double rhoS = std::ldexp(rho, -e);
  Mat dense = Mat(dS.asDiagonal()) + rhoS * z * z.transpose();
  SelfAdjointEigenSolver<Mat> ref(dense, EigenvaluesOnly);
  const double scaleS = dS.cwiseAbs().maxCoeff() + rhoS * z.squaredNorm();
  const double tol = 100.0 * NumTraits<double>::epsilon();
  Vec lambdaS(2);
  lambdaS << std::ldexp(es.eigenvalues()[0], -e), std::ldexp(es.eigenvalues()[1], -e);
  VERIFY((lambdaS - ref.eigenvalues()).cwiseAbs().maxCoeff() <= tol * scaleS);
  // Eigenvectors are scale-invariant: check them on the rescaled matrix.
  const Mat& V = es.eigenvectors();
  VERIFY((dense * V - V * lambdaS.asDiagonal()).norm() <= tol * scaleS);
  VERIFY((V.transpose() * V - Mat::Identity(2, 2)).norm() <= tol);
}

// Regression (MR 2694 review): z entries at the top of the range make ||z||
// itself overflow, but the update rho*z*z^T (rank-one eigenvalue ~6.46e306) is
// perfectly representable; the solver used to return InvalidInput. It must
// instead normalize z by an exact power of two, 2^-p z with rho 2^(2p), which
// leaves the matrix identical -- the same rewrite gives the dense reference.
void test_dpr1_huge_z_norm() {
  if (!dpr1_preserves_subnormal_inputs<double>()) return;

  typedef Matrix<double, Dynamic, 1> Vec;
  typedef Matrix<double, Dynamic, Dynamic> Mat;
  const double dmax = (std::numeric_limits<double>::max)();
  Vec d(2), z(2);
  d << 0.0, 0.0;
  z << dmax, dmax;
  const double rho = 1e-310;  // subnormal, so rho * 2^(2p) stays finite
  DPR1EigenSolver<double> es(d, rho, z);
  VERIFY(es.info() == Success);

  const int p = 1024;  // 2^-p maps DBL_MAX into [0.5, 1)
  Vec zS(2);
  zS << std::ldexp(z[0], -p), std::ldexp(z[1], -p);
  const double rhoS = std::ldexp(rho, 2 * p);  // exact: subnormal scaled up
  Mat dense = Mat(d.asDiagonal()) + rhoS * zS * zS.transpose();
  SelfAdjointEigenSolver<Mat> ref(dense, EigenvaluesOnly);
  const double scale = rhoS * zS.squaredNorm();  // the rank-one eigenvalue, ~6.46e306
  const double tol = 100.0 * NumTraits<double>::epsilon();
  VERIFY((es.eigenvalues() - ref.eigenvalues()).cwiseAbs().maxCoeff() <= tol * scale);
  VERIFY(numext::abs(es.eigenvalues()[1] - scale) <= tol * scale);
}

// Regression (MR 2694 review): a z entry just above the deflation threshold
// puts the secular root ~rho*z_0^2 ~ 6.3e-30 from its pole; bisecting the O(1)
// bracket down to it takes ~150 halvings, beyond the old 2*digits+32 iteration
// cap, and used to report NoConvergence (check_dpr1 requires Success).
void test_dpr1_near_deflation_root() {
  typedef Matrix<double, Dynamic, 1> Vec;
  Vec d(2), z(2);
  d << 0.0, 1.0;
  z << 16.0 * NumTraits<double>::epsilon(), 1.0;
  check_dpr1<double>(d, 1.0, z);
}

// Regression (MR 2694 review): finite 1x1 input whose only eigenvalue
// d + rho*z^2 = 2*DBL_MAX overflows. The documented contract for a spectrum
// that is not representable is InvalidInput, not Success carrying infinity.
void test_dpr1_overflowing_spectrum() {
  typedef Matrix<double, Dynamic, 1> Vec;
  const double dmax = (std::numeric_limits<double>::max)();
  Vec d(1), z(1);
  d << dmax;
  z << 1.0;
  DPR1EigenSolver<double> es(d, dmax, z);
  VERIFY(es.info() == InvalidInput);
  VERIFY((numext::isnan)(es.eigenvalues()[0]));
  // Same with the overflow amplified through z (eigenvalue = 5*DBL_MAX): the
  // internal rescaling saturates the unrepresentable eigenvalue to infinity,
  // which the contract maps to InvalidInput + NaN rather than Success + inf.
  z << 2.0;
  DPR1EigenSolver<double> es2(d, dmax, z);
  VERIFY(es2.info() == InvalidInput);
  VERIFY((numext::isnan)(es2.eigenvalues()[0]));
}

// Regression (MR 2694 review): the normalized secular problem can round a root across the
// maximum-finite boundary before it is scaled back, so that root's finiteness cannot classify the
// exact spectrum. The first update has lambda = DBL_MAX - 19/256 ulp, which rounds to DBL_MAX; the
// second has lambda = DBL_MAX + 3739/2048 ulp and genuinely overflows. Constants are assembled from
// their exact significands, hexadecimal float literals not being in the C++14 baseline.
void test_dpr1_maximum_range_rounding() {
  typedef Matrix<double, Dynamic, 1> Vec;
  const double dmax = (std::numeric_limits<double>::max)();
  Vec d(1), z(1);

  d << -dmax;
  z << 31.0 / 8.0;
  const double rhoRepresentable = std::ldexp(4798840810018093.0, 969);
  DPR1EigenSolver<double> representable(d, rhoRepresentable, z);
  VERIFY(representable.info() == Success);
  VERIFY_IS_EQUAL(representable.eigenvalues()[0], dmax);

  d << dmax;
  DPR1EigenSolver<double> representableNegative(d, -rhoRepresentable, z);
  VERIFY(representableNegative.info() == Success);
  VERIFY_IS_EQUAL(representableNegative.eigenvalues()[0], -dmax);

  d << -dmax;
  z << 65.0 / 2.0;
  const double rhoOverflow = std::ldexp(8732186543767835.0, 962);
  DPR1EigenSolver<double> overflow(d, rhoOverflow, z);
  VERIFY(overflow.info() == InvalidInput);
  VERIFY((numext::isnan)(overflow.eigenvalues()[0]));

  d << dmax;
  DPR1EigenSolver<double> overflowNegative(d, -rhoOverflow, z);
  VERIFY(overflowNegative.info() == InvalidInput);
  VERIFY((numext::isnan)(overflowNegative.eigenvalues()[0]));
}

// Multiple exactly representable endpoint terms must be certifiable together:
// rho*z_i^2/(max-d_i) = 1/2 for each i, so the spectrum is exactly {0, max}.
template <typename Scalar>
void test_dpr1_exact_endpoint_sum() {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  using std::nextafter;
  const Scalar highest = (std::numeric_limits<Scalar>::max)();
  const Scalar boundaryRho = highest / Scalar(2);
  const Vec d = Vec::Zero(2);
  const Vec z = Vec::Ones(2);

  DPR1EigenSolver<Scalar> positive(d, boundaryRho, z);
  VERIFY(positive.info() == Success);
  VERIFY_IS_EQUAL(positive.eigenvalues()[0], Scalar(0));
  VERIFY_IS_EQUAL(positive.eigenvalues()[1], highest);

  DPR1EigenSolver<Scalar> negative(d, -boundaryRho, z);
  VERIFY(negative.info() == Success);
  VERIFY_IS_EQUAL(negative.eigenvalues()[0], -highest);
  VERIFY_IS_EQUAL(negative.eigenvalues()[1], Scalar(0));

  const Vec d4 = Vec::Zero(4);
  const Vec z4 = Vec::Ones(4);
  DPR1EigenSolver<Scalar> fourTerms(d4, highest / Scalar(4), z4);
  VERIFY(fourTerms.info() == Success);
  VERIFY_IS_EQUAL(fourTerms.eigenvalues()[3], highest);

  DPR1EigenSolver<Scalar> below(d, nextafter(boundaryRho, Scalar(0)), z);
  VERIFY(below.info() == Success);
  VERIFY(below.eigenvalues().allFinite());

  DPR1EigenSolver<Scalar> above(d, nextafter(boundaryRho, highest), z);
  VERIFY(above.info() == InvalidInput);

  // The exact top eigenvalue is highest + min. Normalizing the endpoint
  // denominator drops that tiny difference, so this case must remain
  // conservatively unresolved instead of reusing the exact-equality path.
  const Vec minDiagonal = Vec::Constant(2, (std::numeric_limits<Scalar>::min)());
  DPR1EigenSolver<Scalar> unresolved(minDiagonal, boundaryRho, z);
  VERIFY(unresolved.info() != Success);
}

// Regression (MR 2694 review): rho*||z||^2 = 2*max overflows, but the single eigenvalue
// d + rho*z^2 = -max + 2*max = max is representable. The rescaling must come from component exponents
// alone, never materializing rho*||z||^2 at full scale. The secular root sits exactly on the far end
// of its bisection bracket, so the computed eigenvalue lands within a couple of ulps of max rather
// than exactly at it.
template <typename Scalar>
void test_dpr1_overflowing_update_representable_spectrum() {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  const Scalar dmax = (std::numeric_limits<Scalar>::max)();
  const Scalar eps = NumTraits<Scalar>::epsilon();
  Vec d(1), z(1);
  d << -dmax;
  z << Scalar(2);
  DPR1EigenSolver<Scalar> es(d, dmax / Scalar(2), z);
  VERIFY(es.info() == Success);
  const Scalar lam = es.eigenvalues()[0];
  VERIFY((numext::isfinite)(lam));
  VERIFY(numext::abs(lam - dmax) <= Scalar(4) * eps * dmax);
  // n = 1: the eigenvector is the coordinate axis, exactly (up to sign).
  VERIFY(numext::abs(es.eigenvectors()(0, 0)) == Scalar(1));
}

// Regression (MR 2694 review), n > 1 version: poles near -0.75*DBL_MAX with
// rho*||z||^2 ~ 1.2*DBL_MAX (overflowing if materialized) but every eigenvalue
// representable -- the top root lands near +0.4*DBL_MAX, the rest interlace
// the poles. Verified against a dense solve of the exactly rescaled problem,
// whose spectrum is exactly 2^-e times the original.
void test_dpr1_huge_representable_spectrum() {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef Matrix<double, Dynamic, Dynamic> Mat;
  const Index n = 8;
  const double dmax = (std::numeric_limits<double>::max)();
  Vec d(n), z(n);
  for (Index i = 0; i < n; ++i) {
    d[i] = -dmax * (0.70 + 0.01 * double(i));
    z[i] = 1.0 + double(i) / 8.0;  // ||z||^2 ~ 18.6
  }
  const double rho = dmax / 16;
  DPR1EigenSolver<double> es(d, rho, z);
  VERIFY(es.info() == Success);
  VERIFY(es.eigenvalues().allFinite());

  const int e = 1025;  // 2^-e maps every |d_i| and rho*||z||^2 below 1
  Vec dS(n), lambdaS(n);
  for (Index i = 0; i < n; ++i) {
    dS[i] = std::ldexp(d[i], -e);
    lambdaS[i] = std::ldexp(es.eigenvalues()[i], -e);
  }
  const double rhoS = std::ldexp(rho, -e);
  Mat dense = Mat(dS.asDiagonal()) + rhoS * z * z.transpose();
  SelfAdjointEigenSolver<Mat> ref(dense, EigenvaluesOnly);
  const double scaleS = dS.cwiseAbs().maxCoeff() + rhoS * z.squaredNorm();
  const double tol = 100.0 * double(n) * NumTraits<double>::epsilon();
  VERIFY((lambdaS - ref.eigenvalues()).cwiseAbs().maxCoeff() <= tol * scaleS);
  // Eigenvectors are scale-invariant: check them on the rescaled matrix.
  const Mat& V = es.eigenvectors();
  VERIFY((dense * V - V * lambdaS.asDiagonal()).norm() <= tol * scaleS * numext::sqrt(double(n)));
  VERIFY((V.transpose() * V - Mat::Identity(n, n)).norm() <= tol);
}

// Mirror case at the bottom of the range: d and rho near DBL_MIN. The same
// exact power-of-two machinery scales *up* (scaleExp < 0), which needs no
// separate underflow handling -- scaling up from anywhere at or above the
// normal range is exact, and the eigenvalues here are all normal. Verified
// against a dense solve of the exactly upscaled problem.
void test_dpr1_tiny_scale() {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef Matrix<double, Dynamic, Dynamic> Mat;
  const Index n = 8;
  const double dmin = (std::numeric_limits<double>::min)();
  Vec d(n), z(n);
  for (Index i = 0; i < n; ++i) {
    d[i] = dmin * double(1 + i);
    z[i] = 1.0 + double(i) / 8.0;
  }
  const double rho = 3 * dmin;
  DPR1EigenSolver<double> es(d, rho, z);
  VERIFY(es.info() == Success);

  const int e = -1020;  // upscale by 2^1020: exact, brings the data to O(1)
  Vec dS(n), lambdaS(n);
  for (Index i = 0; i < n; ++i) {
    dS[i] = std::ldexp(d[i], -e);
    lambdaS[i] = std::ldexp(es.eigenvalues()[i], -e);  // exact: all normal
  }
  const double rhoS = std::ldexp(rho, -e);
  Mat dense = Mat(dS.asDiagonal()) + rhoS * z * z.transpose();
  SelfAdjointEigenSolver<Mat> ref(dense, EigenvaluesOnly);
  const double scaleS = dS.cwiseAbs().maxCoeff() + rhoS * z.squaredNorm();
  const double tol = 100.0 * double(n) * NumTraits<double>::epsilon();
  VERIFY((lambdaS - ref.eigenvalues()).cwiseAbs().maxCoeff() <= tol * scaleS);
  VERIFY((dense * es.eigenvectors() - es.eigenvectors() * lambdaS.asDiagonal()).norm() <=
         tol * scaleS * numext::sqrt(double(n)));

  if (dpr1_preserves_subnormal_inputs<double>() && dpr1_has_gradual_underflow<double>()) {
    // Deep-subnormal rank-one update: spectrum {0 x7, rho*||z||^2 = 2^-1065}.
    // The result is rounded onto the subnormal grid on the way back down, so the
    // bound is a few subnormal ulps (denorm_min), not a relative one.
    const double dn = (std::numeric_limits<double>::denorm_min)();
    Vec z1 = Vec::Ones(n);
    const double rho1 = 64 * dn;
    DPR1EigenSolver<double> sub(Vec(Vec::Zero(n)), rho1, z1);
    VERIFY(sub.info() == Success);
    VERIFY(numext::abs(sub.eigenvalues()[n - 1] - double(n) * rho1) <= 8 * dn);
    VERIFY(sub.eigenvalues().head(n - 1).cwiseAbs().maxCoeff() == 0.0);
  }
}

void test_dpr1_ftz_mode() {
  const double underflowBefore = dpr1_underflow_probe<double>();
  bool flushToZeroSupported = false;
  {
    ScopedFlushToZero flushToZero;
    flushToZeroSupported = flushToZero.isSupported();
    if (flushToZeroSupported) {
      VERIFY_IS_EQUAL(dpr1_underflow_probe<double>(), 0.0);
      test_dpr1_edges<double>();
    }
  }
  if (flushToZeroSupported) VERIFY_IS_EQUAL(dpr1_underflow_probe<double>(), underflowBefore);
}

// A huge diagonal spread makes every z entry individually negligible even though
// rho itself is not: the whole update deflates and m == 0.
void test_dpr1_all_deflated() {
  typedef Matrix<double, Dynamic, 1> Vec;
  Vec d(4), z(4);
  d << 0.0, 1e17, 2e17, 3e17;
  z << 0.5, 0.5, 0.5, 0.5;
  check_dpr1<double>(d, 1.0, z);
}

EIGEN_DECLARE_TEST(structured_dpr1) {
  for (int i = 0; i < g_repeat; ++i) {
    CALL_SUBTEST_1((test_dpr1_random<double>(1)));
    CALL_SUBTEST_1((test_dpr1_random<double>(2)));
    CALL_SUBTEST_1((test_dpr1_random<double>(3)));
    CALL_SUBTEST_1((test_dpr1_random<double>(10)));
    CALL_SUBTEST_1((test_dpr1_random<double>(33)));
    CALL_SUBTEST_1((test_dpr1_random<double>(64)));
    CALL_SUBTEST_1((test_dpr1_random<float>(12)));
    CALL_SUBTEST_1((test_dpr1_random<float>(40)));
    CALL_SUBTEST_1((test_dpr1_random<long double>(12)));

    CALL_SUBTEST_2((test_dpr1_clustered<double>(12)));
    CALL_SUBTEST_2((test_dpr1_clustered<double>(30)));
    CALL_SUBTEST_2((test_dpr1_clustered<float>(15)));
    CALL_SUBTEST_2((test_dpr1_sparse_z<double>(16)));
    CALL_SUBTEST_2((test_dpr1_sparse_z<float>(9)));
    CALL_SUBTEST_2((test_dpr1_rank_one<double>(20)));
    CALL_SUBTEST_2((test_dpr1_rank_one<float>(11)));
    CALL_SUBTEST_2((test_dpr1_clustered<long double>(12)));
    CALL_SUBTEST_2((test_dpr1_sparse_z<long double>(9)));

    CALL_SUBTEST_3((test_dpr1_edges<double>()));
    CALL_SUBTEST_3((test_dpr1_edges<float>()));
    CALL_SUBTEST_3((test_dpr1_unsorted<double>(17)));
    CALL_SUBTEST_3((test_dpr1_scaling<double>(14)));
    CALL_SUBTEST_3((test_dpr1_edges<long double>()));
    CALL_SUBTEST_3((test_dpr1_scaling<long double>(14)));
    CALL_SUBTEST_3(test_dpr1_invalid_options());

    // Gu-Eisenstat litmus (close but undeflated poles), orthogonality growth at
    // larger n, extreme scales, non-finite rejection, full-deflation pin.
    CALL_SUBTEST_4((test_dpr1_close_undeflated<double>(24)));
    CALL_SUBTEST_4((test_dpr1_close_undeflated<float>(16)));
    CALL_SUBTEST_4((test_dpr1_random<double>(128)));
    CALL_SUBTEST_4((test_dpr1_close_undeflated<long double>(16)));
    CALL_SUBTEST_4(test_dpr1_huge_z());
    CALL_SUBTEST_4(test_dpr1_nonfinite());
    CALL_SUBTEST_4(test_dpr1_all_deflated());

    // MR 2694 review regressions: exponent-range extremes and deep bisection.
    CALL_SUBTEST_5(test_dpr1_full_range_poles());
    CALL_SUBTEST_5(test_dpr1_huge_z_norm());
    CALL_SUBTEST_5(test_dpr1_near_deflation_root());
    CALL_SUBTEST_5(test_dpr1_overflowing_spectrum());
    CALL_SUBTEST_5(test_dpr1_maximum_range_rounding());
    CALL_SUBTEST_5((test_dpr1_exact_endpoint_sum<float>()));
    CALL_SUBTEST_5((test_dpr1_exact_endpoint_sum<double>()));
    CALL_SUBTEST_5((test_dpr1_exact_endpoint_sum<long double>()));
    CALL_SUBTEST_5((test_dpr1_overflowing_update_representable_spectrum<double>()));
    CALL_SUBTEST_5((test_dpr1_overflowing_update_representable_spectrum<float>()));
    CALL_SUBTEST_5((test_dpr1_overflowing_update_representable_spectrum<long double>()));
    CALL_SUBTEST_5(test_dpr1_huge_representable_spectrum());
    CALL_SUBTEST_5(test_dpr1_tiny_scale());
    CALL_SUBTEST_5(test_dpr1_ftz_mode());
  }
}
