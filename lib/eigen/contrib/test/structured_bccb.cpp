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

// Dense inverse DFT matrix, used to synthesize generators independently of the
// FFT implementation under test.
template <typename RealScalar>
Matrix<std::complex<RealScalar>, Dynamic, Dynamic> inverse_dft_matrix(Index n) {
  typedef std::complex<RealScalar> Complex;
  Matrix<Complex, Dynamic, Dynamic> F(n, n);
  for (Index a = 0; a < n; ++a)
    for (Index c = 0; c < n; ++c)
      F(a, c) =
          std::polar(RealScalar(1) / RealScalar(n), RealScalar(2 * EIGEN_PI) * RealScalar((a * c) % n) / RealScalar(n));
  return F;
}

// Reference dense BCCB built entry-wise from the generating array, independently
// of the operator under test: entry (i,j) with i = b1*n2+i2, j = c1*n2+j2 is
// G((i2-j2) mod n2, (b1-c1) mod n1).
template <typename Scalar>
Matrix<Scalar, Dynamic, Dynamic> reference_bccb(const Matrix<Scalar, Dynamic, Dynamic>& G) {
  const Index n2 = G.rows(), n1 = G.cols(), N = n1 * n2;
  Matrix<Scalar, Dynamic, Dynamic> dense(N, N);
  for (Index j = 0; j < N; ++j)
    for (Index i = 0; i < N; ++i) {
      Index k2 = i % n2 - j % n2;
      if (k2 < 0) k2 += n2;
      Index k1 = i / n2 - j / n2;
      if (k1 < 0) k1 += n1;
      dense(i, j) = G(k2, k1);
    }
  return dense;
}

template <typename Scalar>
void test_bccb_product(Index n2, Index n1) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Mat G = Mat::Random(n2, n1);
  Bccb<Scalar> C(G);
  Mat dense = reference_bccb<Scalar>(G);
  const Index N = n1 * n2;
  VERIFY_IS_EQUAL(C.rows(), N);

  Mat Cd = C;
  VERIFY_IS_APPROX(Cd, dense);
  Mat accumulated = Mat::Random(N, N);
  const Mat initial = accumulated;
  accumulated += C;
  VERIFY_IS_APPROX(accumulated, (initial + dense).eval());
  accumulated = initial;
  accumulated -= C;
  VERIFY_IS_APPROX(accumulated, (initial - dense).eval());
  for (Index t = 0; t < 5; ++t) {
    Index i = internal::random<Index>(0, N - 1), j = internal::random<Index>(0, N - 1);
    VERIFY_IS_APPROX(C.coeff(i, j), dense(i, j));
  }

  Vec x = Vec::Random(N);
  VERIFY_IS_APPROX((C * x).eval(), (dense * x).eval());

  Mat X = Mat::Random(N, 3);
  VERIFY_IS_APPROX((C * X).eval(), (dense * X).eval());

  // Accumulation form exercised by the iterative solvers.
  Vec y = Vec::Random(N);
  Vec y0 = y;
  y.noalias() += C * x;
  VERIFY_IS_APPROX(y, (y0 + dense * x).eval());
}

template <typename Scalar>
void test_bccb_transpose(Index n2, Index n1) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Mat G = Mat::Random(n2, n1);
  Bccb<Scalar> C(G);
  Mat dense = reference_bccb<Scalar>(G);
  const Index N = n1 * n2;

  Mat Td = C.transpose();
  VERIFY_IS_APPROX(Td, Mat(dense.transpose()));
  Mat Ad = C.adjoint();
  VERIFY_IS_APPROX(Ad, Mat(dense.adjoint()));
  Mat Kd = C.conjugate();
  VERIFY_IS_APPROX(Kd, Mat(dense.conjugate()));

  Vec x = Vec::Random(N);
  VERIFY_IS_APPROX((C.transpose() * x).eval(), (dense.transpose() * x).eval());
  VERIFY_IS_APPROX((C.adjoint() * x).eval(), (dense.adjoint() * x).eval());

  // Exact round trips: generators and symbols are pure permutations/conjugations.
  Bccb<Scalar> Ctt = C.transpose().transpose();
  VERIFY_IS_EQUAL(Ctt.generator(), G);
  VERIFY_IS_EQUAL(Ctt.symbol(), C.symbol());
  Bccb<Scalar> Caa = C.adjoint().adjoint();
  VERIFY_IS_EQUAL(Caa.generator(), G);
  VERIFY_IS_EQUAL(Caa.symbol(), C.symbol());
}

// transpose()/conjugate()/adjoint() return owning temporaries, so the product
// expression must nest the structured operand by value: a delayed-evaluated
// expression has to outlive the temporary operator it was built from. The static
// check pins the value nesting; the behavioral check would read freed memory if
// the product held a reference instead.
template <typename Scalar>
void test_bccb_delayed_product(Index n2, Index n1) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  STATIC_CHECK(!std::is_reference<typename internal::ref_selector<Bccb<Scalar>>::type>::value);

  Mat G = Mat::Random(n2, n1);
  Bccb<Scalar> C(G);
  Mat dense = reference_bccb<Scalar>(G);
  const Index N = n1 * n2;
  Vec x = Vec::Random(N);

  auto expr = C.adjoint() * x;        // the adjoint temporary dies with the full expression
  Vec scribble = Vec::Random(2 * N);  // reuses the temporary's freed heap storage
  Vec y = expr;
  VERIFY_IS_APPROX(y, (dense.adjoint() * x).eval());
  VERIFY_IS_EQUAL(scribble.size(), 2 * N);  // keep the scribble alive across the evaluation
}

// The products carry the default product tag, so plain assignment materializes
// a temporary exactly like a dense product: x = C * x and x += C * x get the
// ordinary dense-product aliasing semantics.
template <typename Scalar>
void test_bccb_aliased_product(Index n2, Index n1) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Mat G = Mat::Random(n2, n1);
  Bccb<Scalar> C(G);
  Mat dense = reference_bccb<Scalar>(G);
  const Index N = n1 * n2;

  Vec x = Vec::Random(N);
  Vec y = x;
  y = C * y;
  VERIFY_IS_APPROX(y, (dense * x).eval());

  y = x;
  y += C * y;
  VERIFY_IS_APPROX(y, (x + dense * x).eval());

  y = x;
  y -= C * y;
  VERIFY_IS_APPROX(y, (x - dense * x).eval());

  Mat X = Mat::Random(N, 3);
  Mat Y = X;
  Y = C * Y;
  VERIFY_IS_APPROX(Y, (dense * X).eval());
}

// Aliasing beyond the same-object case: the default-product temporary must also
// resolve right-hand-side expressions that reference the destination and
// overlapping views of one buffer, neither of which is_same_dense can see.
template <typename Scalar>
void test_bccb_aliased_expression(Index n2, Index n1) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Mat G = Mat::Random(n2, n1);
  Bccb<Scalar> C(G);
  Mat dense = reference_bccb<Scalar>(G);
  const Index N = n1 * n2;

  // Right-hand-side expression referencing the destination.
  Vec x = Vec::Random(N), x0 = x;
  x = C * (x + Vec::Ones(N));
  VERIFY_IS_APPROX(x, (dense * (x0 + Vec::Ones(N))).eval());

  // Overlapping (shifted) segments of one buffer.
  Vec buf = Vec::Random(N + 1);
  Vec expected = dense * buf.tail(N);
  buf.head(N) = C * buf.tail(N);
  VERIFY_IS_APPROX(buf.head(N).eval(), expected);
}

// Mixed-scalar products: a real operator applied to a complex right-hand side
// (and a complex operator applied to a real one) promotes to the complex product
// scalar, so alpha and the accumulation must run in the promoted type rather than
// the operator scalar.
template <typename RealScalar>
void test_bccb_mixed_scalar(Index n2, Index n1) {
  typedef std::complex<RealScalar> Complex;
  typedef Matrix<RealScalar, Dynamic, 1> RVec;
  typedef Matrix<RealScalar, Dynamic, Dynamic> RMat;
  typedef Matrix<Complex, Dynamic, 1> CVec;
  typedef Matrix<Complex, Dynamic, Dynamic> CMat;

  RMat G = RMat::Random(n2, n1);
  Bccb<RealScalar> C(G);
  CMat dense = reference_bccb<RealScalar>(G).template cast<Complex>();
  const Index N = n1 * n2;

  CVec x = CVec::Random(N);
  CVec y = C * x;
  VERIFY_IS_APPROX(y, (dense * x).eval());

  CVec y0 = CVec::Random(N);
  y = y0;
  y.noalias() += C * x;
  VERIFY_IS_APPROX(y, (y0 + dense * x).eval());

  CMat Gc = CMat::Random(n2, n1);
  Bccb<Complex> Cc(Gc);
  CMat denseC = reference_bccb<Complex>(Gc);
  RVec xr = RVec::Random(N);
  CVec z = Cc * xr;
  VERIFY_IS_APPROX(z, (denseC * xr).eval());
}

// Non-5-smooth block dimensions: products transform on the padded per-axis
// circulant-embedding grid (kissfft's generic butterfly is quadratic in prime
// factors), while the spectral operations keep the exact-size symbol. Cover
// each padded combination -- n2 awkward, n1 awkward, both -- against the dense
// reference, including the transposition family (which reuses the padded
// symbol through the reversal/conjugation rules), solve, and inverse (which
// rebuilds the padded symbol for the inverse operator).
template <typename Scalar>
void test_bccb_prime_dimensions(Index n2, Index n1) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;
  const Index N = n1 * n2;

  Mat G = Mat::Random(n2, n1);
  Bccb<Scalar> C(G);
  Mat dense = reference_bccb<Scalar>(G);

  Vec x = Vec::Random(N);
  VERIFY_IS_APPROX((C * x).eval(), (dense * x).eval());
  Mat X = Mat::Random(N, 3);
  VERIFY_IS_APPROX((C * X).eval(), (dense * X).eval());

  VERIFY_IS_APPROX((C.transpose() * x).eval(), (dense.transpose() * x).eval());
  VERIFY_IS_APPROX((C.adjoint() * x).eval(), (dense.adjoint() * x).eval());
  VERIFY_IS_APPROX((C.conjugate() * x).eval(), (dense.conjugate() * x).eval());

  // Exact symbol round trip through the transposition family: both cached
  // symbols are pure permutations/conjugations of the originals.
  Bccb<Scalar> Ctt = C.transpose().transpose();
  VERIFY_IS_EQUAL(Ctt.generator(), G);
  VERIFY_IS_EQUAL(Ctt.symbol(), C.symbol());

  // Solve (exact-size spectral path) and inverse (whose product rebuilds the
  // padded symbol): diagonal dominance keeps the symbol away from zero.
  Mat Gd = Mat::Random(n2, n1);
  Gd(0, 0) += Scalar(RealScalar(2 * N));
  Bccb<Scalar> Cd(Gd);
  Mat densed = reference_bccb<Scalar>(Gd);
  Vec b = Vec::Random(N);
  Vec xs = Cd.solve(b);
  VERIFY_IS_APPROX((densed * xs).eval(), b);
  VERIFY_IS_APPROX((Cd.inverse() * b).eval(), xs);
}

// Dynamic dimension mismatches must trip the runtime assertion when the product
// or solve expression is built. (Incompatible *fixed* sizes are rejected at
// compile time by a static assertion in operator* / solve, which a runtime test
// cannot exercise.)
void test_bccb_dimension_asserts() {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef Matrix<double, Dynamic, Dynamic> Mat;

  Mat G = Mat::Random(3, 4);
  Bccb<double> C(G);
  Vec bad = Vec::Random(11);
  VERIFY_RAISES_ASSERT(C * bad);
  VERIFY_RAISES_ASSERT(C.solve(bad));
}

template <typename Scalar>
void test_bccb_solve(Index n2, Index n1) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  // Diagonal dominance (through the (0,0) generator entry) keeps the symbol away
  // from zero, so the direct 2-D FFT solve is exact up to roundoff.
  Mat G = Mat::Random(n2, n1);
  G(0, 0) += Scalar(RealScalar(2 * n1 * n2));
  Bccb<Scalar> C(G);
  Mat dense = reference_bccb<Scalar>(G);
  const Index N = n1 * n2;

  Vec b = Vec::Random(N);
  Vec x = C.solve(b);
  VERIFY_IS_APPROX((dense * x).eval(), b);

  Mat B = Mat::Random(N, 3);
  Mat Xs = C.solve(B);
  VERIFY_IS_APPROX((dense * Xs).eval(), B);
}

// Rank-deficient BCCB synthesized by zeroing 2-D symbol entries: the rank counts
// the surviving entries and solve() matches the SVD pseudo-inverse.
template <typename Scalar>
void test_bccb_rank_deficient(Index n2, Index n1, Index defect) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef std::complex<RealScalar> Complex;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;
  typedef Matrix<Complex, Dynamic, Dynamic> CMat;

  const Index N = n1 * n2;
  CMat S = CMat::Random(n2, n1);
  S.array() += Complex(2);  // keep the surviving moduli away from the threshold
  for (Index k = 0; k < defect; ++k) S((2 * k + 1) % n2, (3 * k) % n1) = Complex(0);

  const CMat F2 = inverse_dft_matrix<RealScalar>(n2), F1 = inverse_dft_matrix<RealScalar>(n1);
  Mat G = (F2 * S * F1.transpose()).eval();  // Scalar is complex here
  Bccb<Scalar> C(G);
  VERIFY_IS_EQUAL(C.rank(), N - defect);

  Mat dense = reference_bccb<Scalar>(G);
  JacobiSVD<Mat> svd(dense, ComputeThinU | ComputeThinV);
  Vec b = Vec::Random(N);
  VERIFY_IS_APPROX(C.solve(b), svd.solve(b).eval());
}

template <typename Scalar>
void test_bccb_zero(Index n2, Index n1) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Bccb<Scalar> C(Mat(Mat::Zero(n2, n1)));
  VERIFY_IS_EQUAL(C.rank(), 0);
  VERIFY(C.singularValues().isZero());
  Vec b = Vec::Random(n1 * n2);
  VERIFY(C.solve(b).isZero());
}

void test_bccb_nan_propagation(Index n2, Index n1) {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef Matrix<double, Dynamic, Dynamic> Mat;

  Mat G = Mat::Random(n2, n1);
  G(n2 / 2, n1 / 2) = std::numeric_limits<double>::quiet_NaN();
  Bccb<double> C(G);
  VERIFY_IS_EQUAL(C.rank(), n1 * n2);
  Vec b = Vec::Random(n1 * n2);
  Vec x = C.solve(b);
  VERIFY(!(x.array() == x.array()).all());
}

// Inputs at the very top (and bottom) of the exponent range: the transforms sum
// up to N terms, so without scaling the FFT intermediates overflow even though
// every entry of the true result is representable. The apply path rescales the
// symbol and each right-hand-side column by exact powers of two and folds the
// exponent back at the end.
void test_bccb_finite_overflow() {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef Matrix<double, Dynamic, Dynamic> Mat;

  const double huge = (std::numeric_limits<double>::max)() / 16;
  // The 2-D FFT round trip costs a few ulps per entry; a norm-based comparison
  // would itself overflow at these magnitudes, hence the entrywise bound.
  const double kFftRoundTripTol = 100 * NumTraits<double>::epsilon();

  // Reviewer repro: a 36x36 identity BCCB applied to a vector of DBL_MAX/16
  // must return the vector, not 36 NaNs.
  Mat G = Mat::Zero(6, 6);
  G(0, 0) = 1.0;
  Bccb<double> C(G);
  Vec x = Vec::Constant(36, huge);
  Vec y = C * x;
  VERIFY(y.allFinite());
  VERIFY(((y - x).array().abs() <= kFftRoundTripTol * x.array().abs()).all());

  // A huge generator makes a huge symbol: the scaling must come from the symbol
  // side as well.
  Mat Gh = Mat::Zero(6, 6);
  Gh(0, 0) = huge;  // C == huge * Identity
  Bccb<double> Ch(Gh);
  Vec ones = Vec::Ones(36);
  Vec z = Ch * ones;
  VERIFY(z.allFinite());
  VERIFY(((z.array() - huge).abs() <= kFftRoundTripTol * huge).all());

  // The inverse symbol of a tiny operator is huge; solve() shares the scaled
  // apply path.
  Mat Gt = Mat::Zero(6, 6);
  Gt(0, 0) = 1.0 / huge;
  Bccb<double> Ct(Gt);
  Vec w = Ct.solve(ones);
  VERIFY(w.allFinite());
  VERIFY(((w.array() - huge).abs() <= kFftRoundTripTol * huge).all());

  // An exactly zero column maps to an exactly zero column (short-circuited
  // before any scaling), while nonzero columns are still transformed.
  Mat B(36, 2);
  B.col(0).setConstant(huge);
  B.col(1).setZero();
  Mat Y = C * B;
  VERIFY(Y.col(1).isZero());
  VERIFY(((Y.col(0) - x).array().abs() <= kFftRoundTripTol * x.array().abs()).all());

  // Genuine NaN and infinity inputs keep propagating; such right-hand sides
  // take the direct kernel (see test_bccb_nonfinite_product for the entrywise
  // semantics).
  Vec xn = x;
  xn[7] = std::numeric_limits<double>::quiet_NaN();
  Vec yn = C * xn;
  VERIFY(!(yn.array() == yn.array()).all());
  Vec xi = Vec::Ones(36);
  xi[3] = std::numeric_limits<double>::infinity();
  Vec yi = C * xi;
  VERIFY(!yi.allFinite());
}

// The scaling exponents are derived from component-wise magnitudes: a finite
// complex value near the overflow threshold has a non-representable modulus,
// which would otherwise disable the scaling and turn an exactly representable
// product into NaN.
template <typename RealScalar>
void test_bccb_fft_complex_boundary(Index n2, Index n1) {
  typedef std::complex<RealScalar> Complex;
  typedef Matrix<RealScalar, Dynamic, Dynamic> RMat;
  typedef Matrix<Complex, Dynamic, 1> CVec;
  typedef Matrix<Complex, Dynamic, Dynamic> CMat;
  const RealScalar kFftRoundTripTol = RealScalar(100) * NumTraits<RealScalar>::epsilon();
  const RealScalar big = RealScalar(0.75) * (std::numeric_limits<RealScalar>::max)();
  const Index N = n1 * n2;

  // Identity BCCB with a complex generator: the product returns the right-hand
  // side unchanged even though |x_k| overflows.
  CMat G = CMat::Zero(n2, n1);
  G(0, 0) = Complex(1);
  Bccb<Complex> C(G);
  CVec x = CVec::Constant(N, Complex(big, big));
  CVec y = C * x;
  VERIFY(y.allFinite());
  VERIFY(((y - x).cwiseAbs() / big).maxCoeff() <= kFftRoundTripTol);

  // A real identity operator applied to the same complex right-hand side takes
  // the mixed-scalar product path.
  RMat Gr = RMat::Zero(n2, n1);
  Gr(0, 0) = RealScalar(1);
  Bccb<RealScalar> Cr(Gr);
  y = Cr * x;
  VERIFY(y.allFinite());
  VERIFY(((y - x).cwiseAbs() / big).maxCoeff() <= kFftRoundTripTol);
}

// Entrywise IEEE comparison for the non-finite tests: NaNs match NaNs,
// infinities match by value (sign included), finite entries match to roundoff.
// VERIFY_IS_APPROX would reject any output containing NaN.
template <typename D1, typename D2>
bool ieee_entrywise_match(const D1& a, const D2& b) {
  if (a.rows() != b.rows() || a.cols() != b.cols()) return false;
  for (Index j = 0; j < a.cols(); ++j)
    for (Index i = 0; i < a.rows(); ++i) {
      const typename D1::Scalar x = a(i, j), y = b(i, j);
      if (x == y) continue;                                    // finite match or same-signed infinities
      if ((numext::isnan)(x) && (numext::isnan)(y)) continue;  // both NaN
      if (!test_isApprox(x, y)) return false;                  // finite roundoff
    }
  return true;
}

// Scalar-loop product: the mathematically transparent IEEE reference for the
// non-finite tests. Eigen's own vectorized complex kernels can smear a single
// infinity into NaN (Inf - Inf across the split real/imaginary accumulators), so
// the dense product is not a faithful entrywise reference for non-finite data.
template <typename Scalar>
Matrix<Scalar, Dynamic, 1> reference_product_ieee(const Matrix<Scalar, Dynamic, Dynamic>& A,
                                                  const Matrix<Scalar, Dynamic, 1>& x) {
  Matrix<Scalar, Dynamic, 1> y(A.rows());
  for (Index i = 0; i < A.rows(); ++i) {
    Scalar acc(0);
    for (Index j = 0; j < A.cols(); ++j) acc += A(i, j) * x[j];
    y[i] = acc;
  }
  return y;
}

// A single Inf or NaN in the data must propagate like the reference product --
// through the dot products that touch it -- instead of being smeared into NaNs
// across the whole output by the transforms.
template <typename Scalar>
void test_bccb_nonfinite_product(Index n2, Index n1) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;
  const RealScalar inf = std::numeric_limits<RealScalar>::infinity();
  const RealScalar nan = std::numeric_limits<RealScalar>::quiet_NaN();
  const Index N = n1 * n2;

  Mat G = Mat::Random(n2, n1);
  Bccb<Scalar> C(G);
  Mat dense = reference_bccb<Scalar>(G);

  // Inf in the right-hand side.
  Vec x = Vec::Random(N);
  x[N / 2] = Scalar(inf);
  VERIFY(ieee_entrywise_match((C * x).eval(), reference_product_ieee(dense, x)));

  // NaN in the right-hand side.
  Vec xn = Vec::Random(N);
  xn[N - 1] = Scalar(nan);
  VERIFY(ieee_entrywise_match((C * xn).eval(), reference_product_ieee(dense, xn)));

  // Mixed multi-column right-hand side: the non-finite column falls back to the
  // direct kernel individually while the finite column keeps the FFT path.
  Mat Xm(N, 2);
  Xm.col(0) = Vec::Random(N);
  Xm.col(1) = x;
  Mat Ym = C * Xm;
  VERIFY_IS_APPROX(Ym.col(0).eval(), (dense * Xm.col(0)).eval());
  VERIFY(ieee_entrywise_match(Ym.col(1).eval(), reference_product_ieee(dense, Vec(Xm.col(1)))));

  // A zero column carrying a single NaN must not take the zero-column shortcut:
  // the fast-max routing scan can miss a NaN among zeros (an Inf always
  // surfaces), so the shortcut rechecks exactly and such a column falls back.
  Vec xz = Vec::Zero(N);
  xz[0] = Scalar(nan);
  VERIFY(ieee_entrywise_match((C * xz).eval(), reference_product_ieee(dense, xz)));

  // Inf in the generating array: the operator itself is non-finite, whatever the
  // right-hand side.
  Mat G2 = Mat::Random(n2, n1);
  G2(n2 / 2, n1 / 2) = Scalar(-inf);
  Bccb<Scalar> C2(G2);
  Mat dense2 = reference_bccb<Scalar>(G2);
  Vec x2 = Vec::Random(N);
  VERIFY(ieee_entrywise_match((C2 * x2).eval(), reference_product_ieee(dense2, x2)));

  // Non-finite right-hand sides of solve() apply the pseudo-inverse -- itself a
  // BCCB operator -- through the direct kernel, so the Inf propagates entrywise
  // instead of NaN-ing the whole output through the transforms. The operator is
  // diagonally dominant, so no symbol entry is thresholded and the pseudo-inverse
  // coincides with inverse(), whose dense form serves as the reference matrix.
  Mat Gd = Mat::Random(n2, n1);
  Gd(0, 0) += Scalar(RealScalar(2 * N));
  Bccb<Scalar> Cd(Gd);
  Mat pinv = Mat(Cd.inverse());
  Vec binf = Vec::Random(N);
  binf[N / 3] = Scalar(inf);
  VERIFY(ieee_entrywise_match(Cd.solve(binf), reference_product_ieee(pinv, binf)));
}

// An all-zero right-hand side must not short-circuit to an exact zero when the
// operator holds non-finite data: every row of a BCCB touches every generator
// entry, so each result entry is a dot product with an Inf coefficient times
// zero -- NaN under IEEE, not zero.
void test_bccb_nonfinite_zero_rhs(Index n2, Index n1) {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef Matrix<double, Dynamic, Dynamic> Mat;
  const Index N = n1 * n2;

  Mat G = Mat::Random(n2, n1);
  G(n2 / 2, n1 / 2) = std::numeric_limits<double>::infinity();
  Bccb<double> C(G);
  Mat dense = reference_bccb<double>(G);

  const Vec z = Vec::Zero(N);
  Vec y = C * z;
  VERIFY(y.hasNaN());
  VERIFY(ieee_entrywise_match(y, reference_product_ieee(dense, z)));

  // The same holds for solve(): the symbol of a non-finite operator holds NaN,
  // so the pseudo-inverse applied to a zero right-hand side is NaN, not zero.
  Vec xs = C.solve(z);
  VERIFY(xs.hasNaN());
}

// Spectra with a wide dynamic range: the determinant is representable, but a
// plain product of the eigenvalues in FFT order overflows to infinity (or
// underflows to an exact zero) partway through. Pins the balanced accumulation
// in determinant().
void test_bccb_determinant_scaled() {
  typedef std::complex<double> Complex;
  typedef Matrix<Complex, Dynamic, Dynamic> CMat;

  const Index n2 = 25, n1 = 40;  // N = 1000
  const Index nLead = 653;

  // Symbol magnitudes `lead` on the first nLead column-major-flattened indices
  // (the accumulation order of determinant()) and `rest` elsewhere, so the
  // naive running product leaves the representable range partway through while
  // the true determinant lead^nLead * rest^(N - nLead) is representable. The
  // generator is recovered through the dense inverse 2-D DFT matrices,
  // independently of the implementation under test.
  auto makeOperator = [n2, n1, nLead](double lead, double rest) {
    CMat S(n2, n1);
    for (Index k1 = 0; k1 < n1; ++k1)
      for (Index k2 = 0; k2 < n2; ++k2) S(k2, k1) = Complex(k1 * n2 + k2 < nLead ? lead : rest);
    const CMat F2 = inverse_dft_matrix<double>(n2), F1 = inverse_dft_matrix<double>(n1);
    return Bccb<Complex>((F2 * S * F1.transpose()).eval());
  };

  // The spectrum is only reproduced up to the FFT round trip's forward error,
  // and the determinant multiplies ~1e3 such factors.
  const double kSpectrumRoundTripTol = 1e8 * NumTraits<double>::epsilon();
  {
    // det = 10^653 * 10^-347 = 1e306; the naive partial product reaches 1e327.
    Bccb<Complex> C = makeOperator(10.0, 0.1);
    const Complex det = C.determinant();
    VERIFY((numext::isfinite)(numext::abs(det)));
    VERIFY(numext::abs(det / 1e306 - Complex(1)) <= kSpectrumRoundTripTol);
  }
  {
    // det = 10^-653 * 10^347 = 1e-306; the naive partial product reaches
    // 1e-327, well below the smallest subnormal, and flushes to an exact zero.
    Bccb<Complex> C = makeOperator(0.1, 10.0);
    const Complex det = C.determinant();
    VERIFY(det != Complex(0));
    VERIFY(numext::abs(det / 1e-306 - Complex(1)) <= kSpectrumRoundTripTol);
  }
  {
    // A genuinely overflowing determinant must still saturate to infinity.
    typedef Matrix<double, Dynamic, Dynamic> Mat;
    Mat G = Mat::Zero(6, 6);
    G(0, 0) = 1e308;  // the symbol is 1e308 everywhere: det = 10^11088
    Bccb<double> C(G);
    VERIFY((numext::isinf)(C.determinant()));
  }
}

// The rank decision at the clamped threshold (the smallest normal number): a
// smallest-normal symbol entry is still inverted -- the comparison is strict,
// matching SVDBase::rank(), which likewise reports rank one -- a subnormal entry
// is treated as an exact zero, and non-finite entries count as non-zero.
void test_bccb_rank_boundaries() {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef Matrix<double, Dynamic, Dynamic> Mat;

  const double mn = (std::numeric_limits<double>::min)();
  const Vec b = Vec::Ones(1);
  {
    Bccb<double> C(Mat(Mat::Constant(1, 1, mn)));
    VERIFY_IS_EQUAL(C.rank(), 1);
    Vec x = C.solve(b);
    VERIFY((numext::isfinite)(x[0]));
    VERIFY_IS_APPROX(x[0], 1.0 / mn);
  }
  {
    Bccb<double> C(Mat(Mat::Constant(1, 1, mn / 2)));  // subnormal
    VERIFY_IS_EQUAL(C.rank(), 0);
    VERIFY(C.solve(b).isZero());
  }
  {
    Bccb<double> C(Mat(Mat::Constant(1, 1, std::numeric_limits<double>::infinity())));
    VERIFY_IS_EQUAL(C.rank(), 1);
    VERIFY((numext::isinf)(C.determinant()));
  }
}

// A finite complex symbol entry near the overflow threshold has a
// non-representable modulus. The rank threshold used to be computed from the raw
// moduli, turning it into infinity: the rank was under-reported and solve()
// zeroed valid Fourier modes. Both are now evaluated in an exactly rescaled
// frame.
void test_bccb_rank_complex_boundary() {
  typedef std::complex<double> Complex;
  typedef Matrix<Complex, Dynamic, 1> CVec;
  typedef Matrix<Complex, Dynamic, Dynamic> CMat;
  const double mx = (std::numeric_limits<double>::max)();

  // n2 = 2, n1 = 1: the symbol is exactly [g0 + g1, g0 - g1], so pick the
  // generator from the desired spectrum. |s0| overflows while both of its
  // components are finite.
  const Complex s0(0.75 * mx, 0.75 * mx), s1(1e300, 0.0);
  CMat G(2, 1);
  G(0, 0) = (s0 + s1) * 0.5;
  G(1, 0) = (s0 - s1) * 0.5;
  Bccb<Complex> C(G);
  VERIFY_IS_EQUAL(C.rank(), 2);

  // The second Fourier mode must be inverted, not zeroed: a product of a small
  // vector solves back to that vector (the accuracy is limited by the condition
  // number |s0| / |s1| ~ 2.5e8).
  CVec x0(2);
  x0[0] = Complex(1e-10, -2e-10);
  x0[1] = Complex(-3e-10, 1e-10);
  CVec b = C * x0;
  VERIFY(b.allFinite());
  CVec x = C.solve(b);
  VERIFY(((x - x0).cwiseAbs().maxCoeff() / x0.cwiseAbs().maxCoeff()) <= 1e-6);

  // A genuinely negligible second entry still truncates in the scaled frame.
  CMat G2(2, 1);
  G2(0, 0) = (s0 + Complex(1)) * 0.5;
  G2(1, 0) = (s0 - Complex(1)) * 0.5;
  VERIFY_IS_EQUAL(Bccb<Complex>(G2).rank(), 1);
}

// A single 2^-e frame factor is itself subnormal once the frame exceeds the
// exponent range, and reads as zero under flush-to-zero: every scaled modulus
// and the threshold collapse together, the rank is over-reported, and solve()
// inverts a mode it should have truncated. Hence the two exact factors.
void test_bccb_rank_flush_to_zero() {
  ScopedFlushToZero flush_to_zero;
  if (!flush_to_zero.isSupported()) return;

  using Complex = std::complex<double>;
  using CMat = Matrix<Complex, Dynamic, Dynamic>;
  const double mx = (std::numeric_limits<double>::max)();

  // Spectrum [s0, 1] with |s0| at the overflow boundary: the second mode is
  // negligible against it, so the operator is rank one.
  const Complex s0(0.75 * mx, 0.75 * mx);
  CMat G(2, 1);
  G(0, 0) = (s0 + Complex(1)) * 0.5;
  G(1, 0) = (s0 - Complex(1)) * 0.5;
  VERIFY_IS_EQUAL(Bccb<Complex>(G).rank(), 1);
}

template <typename Scalar>
void test_bccb_eigen(Index n2, Index n1) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef std::complex<RealScalar> Complex;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;
  typedef Matrix<Complex, Dynamic, Dynamic> CMat;

  Mat G = Mat::Random(n2, n1);
  Bccb<Scalar> C(G);
  CMat denseC = reference_bccb<Scalar>(G).template cast<Complex>();
  const Index N = n1 * n2;

  Matrix<Complex, Dynamic, 1> lam = C.eigenvalues();
  CMat V = C.eigenvectors();
  VERIFY_IS_APPROX((denseC * V).eval(), (V * lam.asDiagonal()).eval());
  VERIFY_IS_APPROX((V.adjoint() * V).eval(), CMat(CMat::Identity(N, N)));
}

template <typename Scalar>
void test_bccb_svd(Index n2, Index n1) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef std::complex<RealScalar> Complex;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;
  typedef Matrix<Complex, Dynamic, Dynamic> CMat;

  Mat G = Mat::Random(n2, n1);
  Bccb<Scalar> C(G);
  Mat dense = reference_bccb<Scalar>(G);
  const Index N = n1 * n2;

  Matrix<RealScalar, Dynamic, 1> sv = C.singularValues();
  JacobiSVD<Mat> svd(dense);
  VERIFY_IS_APPROX(sv, svd.singularValues());

  CMat U = C.matrixU(), V = C.matrixV();
  VERIFY_IS_APPROX((U * sv.template cast<Complex>().asDiagonal() * V.adjoint()).eval(),
                   CMat(dense.template cast<Complex>()));
  VERIFY_IS_APPROX((U.adjoint() * U).eval(), CMat(CMat::Identity(N, N)));
  VERIFY_IS_APPROX((V.adjoint() * V).eval(), CMat(CMat::Identity(N, N)));
}

template <typename Scalar>
void test_bccb_inverse(Index n2, Index n1) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Mat G = Mat::Random(n2, n1);
  G(0, 0) += Scalar(RealScalar(2 * n1 * n2));  // safely invertible, tame determinant scale
  Bccb<Scalar> C(G);
  Mat dense = reference_bccb<Scalar>(G);
  const Index N = n1 * n2;

  Mat inv = C.inverse();
  VERIFY_IS_APPROX((inv * dense).eval(), Mat(Mat::Identity(N, N)));

  Vec b = Vec::Random(N);
  VERIFY_IS_APPROX((C.inverse() * b).eval(), C.solve(b));
}

template <typename Scalar>
void test_bccb_determinant(Index n2, Index n1) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Mat G = Mat::Random(n2, n1);
  G(0, 0) += Scalar(RealScalar(2 * n1 * n2));
  Bccb<Scalar> C(G);
  Mat dense = reference_bccb<Scalar>(G);
  VERIFY_IS_APPROX(C.determinant(), dense.determinant());
}

template <typename Scalar>
void test_bccb_matrix_free_cg(Index n2, Index n1) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  // Symmetrizing the generator under the two-dimensional index reversal makes
  // the BCCB matrix symmetric; diagonal dominance then makes it positive
  // definite.
  Mat G = Mat::Random(n2, n1);
  Mat Gs = G;
  for (Index k1 = 0; k1 < n1; ++k1)
    for (Index k2 = 0; k2 < n2; ++k2) Gs(k2, k1) = (G(k2, k1) + G((n2 - k2) % n2, (n1 - k1) % n1)) / Scalar(2);
  Gs(0, 0) += Scalar(RealScalar(2 * n1 * n2));
  Bccb<Scalar> C(Gs);
  Mat dense = reference_bccb<Scalar>(Gs);
  VERIFY_IS_APPROX(dense, Mat(dense.transpose()));

  const Index N = n1 * n2;
  Vec b = Vec::Random(N);
  ConjugateGradient<Bccb<Scalar>, Lower | Upper, IdentityPreconditioner> cg;
  cg.compute(C);
  Vec x = cg.solve(b);
  VERIFY(cg.info() == Success);
  VERIFY_IS_APPROX((dense * x).eval(), b);
}

template <typename Scalar, int N2, int N1>
void test_bccb_fixed() {
  typedef Matrix<Scalar, N2, N1> GenMat;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  GenMat G = GenMat::Random();
  Bccb<Scalar, N2, N1> C(G);
  STATIC_CHECK((Bccb<Scalar, N2, N1>::RowsAtCompileTime == N1 * N2));
  STATIC_CHECK((internal::remove_all_t<decltype(makeBccb(G))>::RowsAtCompileTime == N1 * N2));

  Mat dense = reference_bccb<Scalar>(Mat(G));
  Matrix<Scalar, N1 * N2, N1 * N2> Cd = C;
  VERIFY_IS_APPROX(Mat(Cd), dense);

  Matrix<Scalar, N1 * N2, 1> x = Matrix<Scalar, N1 * N2, 1>::Random();
  Matrix<Scalar, N1 * N2, 1> y = C * x;
  VERIFY_IS_APPROX(y, (dense * x).eval());
}

template <typename RealScalar>
void test_structured_symbol_reciprocals() {
  using Complex = std::complex<RealScalar>;
  using CVector = Matrix<Complex, Dynamic, 1>;
  using RVector = Matrix<RealScalar, Dynamic, 1>;
  constexpr Index kPacketSize = internal::packet_traits<Complex>::size;
  const Index count = 3 * kPacketSize + 1;
  const RealScalar epsilon = NumTraits<RealScalar>::epsilon();
  const RealScalar lower = numext::ldexp(RealScalar(1), std::numeric_limits<RealScalar>::min_exponent / 2);
  const RealScalar upper = numext::ldexp(RealScalar(1), std::numeric_limits<RealScalar>::max_exponent / 2 - 1);
  const RealScalar minimum = (std::numeric_limits<RealScalar>::min)();
  const RealScalar maximum = (std::numeric_limits<RealScalar>::max)();
  const RealScalar infinity = NumTraits<RealScalar>::infinity();
  const RealScalar nan = NumTraits<RealScalar>::quiet_NaN();
  const RealScalar values[] = {RealScalar(0),
                               -RealScalar(0),
                               RealScalar(1),
                               RealScalar(-1),
                               lower / 2,
                               lower,
                               upper,
                               upper * 2,
                               minimum,
                               minimum / 2,
                               std::numeric_limits<RealScalar>::denorm_min(),
                               RealScalar(0.75) * maximum,
                               infinity,
                               nan};
  CVector symbol(count + 2), inverse(count + 2), unthresholded(count + 2);
  RVector moduli(count + 2);
  // Slide each special value through every lane, with unaligned inputs and a scalar tail.
  for (Index offset = 0; offset <= 1; ++offset) {
    for (Index lane = 0; lane < kPacketSize; ++lane) {
      for (RealScalar real : values) {
        for (RealScalar imag : values) {
          symbol.setConstant(Complex(3, 4));
          moduli.setOnes();
          symbol(offset + lane) = Complex(real, imag);
          symbol(offset + count - 1) = Complex(real, imag);
          inverse.setConstant(Complex(9));
          internal::structured_symbol_reciprocal<RealScalar>(symbol.data() + offset, moduli.data() + offset,
                                                             RealScalar(1), inverse.data() + offset, count);
          unthresholded = symbol;
          internal::structured_symbol_reciprocal(unthresholded.data() + offset, count);
          VERIFY_IS_EQUAL(unthresholded(offset + count), symbol(offset + count));
          if (offset) VERIFY_IS_EQUAL(unthresholded(0), symbol(0));
          VERIFY_IS_EQUAL(inverse(offset + count), Complex(9));
          if (offset) VERIFY_IS_EQUAL(inverse(0), Complex(9));
          for (Index k = offset; k < offset + count; ++k) {
            const Complex expected = internal::structured_scaled_reciprocal(symbol(k));
            for (const CVector* result : {&inverse, &unthresholded}) {
              for (Index component = 0; component < 2; ++component) {
                const RealScalar actual = component ? (*result)(k).imag() : (*result)(k).real();
                const RealScalar reference = component ? expected.imag() : expected.real();
                if ((numext::isnan)(reference)) {
                  VERIFY((numext::isnan)(actual));
                } else if ((numext::isinf)(reference) || reference == RealScalar(0)) {
                  VERIFY_IS_EQUAL(actual, reference);
                  VERIFY_IS_EQUAL(std::signbit(actual), std::signbit(reference));
                } else {
                  VERIFY((numext::isfinite)(actual));
                  VERIFY(numext::abs(actual / reference - RealScalar(1)) <= RealScalar(8) * epsilon);
                }
              }
            }
          }
        }
      }
    }
  }

  // Positive denominator: two squares, their sum and a division, plus reference rounding.
  for (Index trial = 0; trial < 100; ++trial) {
    for (Index k = 0; k < count; ++k) {
      const int exponent = internal::random<int>(std::numeric_limits<RealScalar>::min_exponent / 2,
                                                 std::numeric_limits<RealScalar>::max_exponent / 2 - 1);
      const RealScalar re = numext::ldexp(internal::random<RealScalar>(RealScalar(0.5), RealScalar(1)), exponent);
      const RealScalar im = numext::ldexp(internal::random<RealScalar>(RealScalar(-1), RealScalar(-0.5)), exponent);
      symbol(k) = Complex(re, im);
    }
    moduli.setOnes();
    internal::structured_symbol_reciprocal<RealScalar>(symbol.data(), moduli.data(), RealScalar(1), inverse.data(),
                                                       count);
    for (Index k = 0; k < count; ++k) {
      const long double re = static_cast<long double>(symbol(k).real());
      const long double im = static_cast<long double>(symbol(k).imag());
      const long double scale = (std::max)(std::abs(re), std::abs(im));
      const long double a = re / scale, b = im / scale;
      const long double denom = a * a + b * b;
      const long double expected[] = {(a / denom) / scale, (-b / denom) / scale};
      for (Index component = 0; component < 2; ++component) {
        const long double actual = static_cast<long double>(component ? inverse(k).imag() : inverse(k).real());
        VERIFY(std::abs(actual / expected[component] - 1) <= 8 * static_cast<long double>(epsilon));
      }
    }
  }

  symbol.setConstant(Complex(3, 4));
  moduli.setOnes();
  for (Index k = 0; k < count; k += 2) {
    symbol(k) = Complex(0);
    moduli(k) = RealScalar(0);
  }
  std::fenv_t saved_environment;
  const bool environment_saved = std::feholdexcept(&saved_environment) == 0;
  internal::structured_symbol_reciprocal<RealScalar>(symbol.data(), moduli.data(), RealScalar(1), inverse.data(),
                                                     count);
  if (environment_saved) {
    const int exceptions = std::fetestexcept(FE_INVALID | FE_DIVBYZERO | FE_OVERFLOW);
    std::fesetenv(&saved_environment);
    VERIFY_IS_EQUAL(exceptions, 0);
  }
  for (Index k = 0; k < count; ++k) {
    if (k % 2 == 0) {
      VERIFY_IS_EQUAL(inverse(k), Complex(0));
      VERIFY(!std::signbit(inverse(k).real()));
      VERIFY(!std::signbit(inverse(k).imag()));
    } else {
      const Complex expected(RealScalar(3) / RealScalar(25), -RealScalar(4) / RealScalar(25));
      VERIFY(numext::abs(inverse(k) - expected) <= RealScalar(8) * epsilon);
    }
  }
  // NaN moduli, like exact equality above, must remain in the inverted set.
  symbol.setConstant(Complex(3, 4));
  moduli.setConstant(nan);
  internal::structured_symbol_reciprocal<RealScalar>(symbol.data(), moduli.data(), RealScalar(1), inverse.data(),
                                                     count);
  VERIFY((inverse.head(count).array().abs() > RealScalar(0)).all());
  internal::structured_symbol_reciprocal<RealScalar>(nullptr, nullptr, RealScalar(1), nullptr, 0);
  internal::structured_symbol_reciprocal<RealScalar>(nullptr, 0);
}

template <typename RealScalar>
void test_structured_packet_reciprocals() {
  using Complex = std::complex<RealScalar>;
  using CMatrix = Matrix<Complex, Dynamic, Dynamic>;
  CMatrix generator = CMatrix::Zero(3, 5);
  generator(0, 0) = Complex(3, 4);  // Constant non-real symbol exercises packets and the tail.
  const Bccb<Complex> op(generator);
  const CMatrix expected = CMatrix::Random(15, 2);
  const CMatrix rhs = generator(0, 0) * expected;
  CMatrix actual = op.solve(rhs);
  VERIFY(actual.allFinite());
  // Forward/inverse 3-by-5 transforms and one reciprocal contribute rounding.
  VERIFY((actual - expected).norm() <= RealScalar(64) * NumTraits<RealScalar>::epsilon() * expected.norm());
  Matrix<Complex, Dynamic, 1> column = Matrix<Complex, Dynamic, 1>::Zero(15);
  column(0) = generator(0, 0);
  const Circulant<Complex> circulant(column);
  actual = circulant.solve(rhs);
  VERIFY(actual.allFinite());
  VERIFY((actual - expected).norm() <= RealScalar(64) * NumTraits<RealScalar>::epsilon() * expected.norm());
  actual = circulant.inverse() * rhs;
  VERIFY(actual.allFinite());
  VERIFY((actual - expected).norm() <= RealScalar(64) * NumTraits<RealScalar>::epsilon() * expected.norm());
}

template <typename RealScalar>
void test_circulant_inverse_modes() {
  using Complex = std::complex<RealScalar>;
  using CVector = Matrix<Complex, Dynamic, 1>;
  const RealScalar epsilon = NumTraits<RealScalar>::epsilon();
  // The small mode is representable but below the solve's rank threshold.
  CVector column(2);
  column << Complex(1), Complex(1 - epsilon);
  const Circulant<Complex> circulant(column);
  VERIFY_IS_EQUAL(circulant.rank(), 1);
  const Complex circulant_mode = circulant.inverse().symbol()(1);
  VERIFY(numext::abs(circulant_mode * epsilon - Complex(1)) <= RealScalar(8) * epsilon);

  // 1/(a+ia) = (0.5/a)(1-i), even when an unscaled denominator overflows.
  const RealScalar a = RealScalar(0.75) * (std::numeric_limits<RealScalar>::max)();
  for (Index n : {Index(1), Index(48)}) {
    column.setZero(n);
    column(0) = Complex(a, a);
    const CVector circulant_inverse = Circulant<Complex>(column).inverse().symbol();
    VERIFY(circulant_inverse.allFinite());
    volatile RealScalar minimum = (std::numeric_limits<RealScalar>::min)();
    if (!numext::is_exactly_zero(minimum * RealScalar(0.5))) {
      for (Index k = 0; k < n; ++k) {
        VERIFY(numext::abs(circulant_inverse(k).real() * a - RealScalar(0.5)) <= RealScalar(64) * epsilon);
        VERIFY(numext::abs(circulant_inverse(k).imag() * a + RealScalar(0.5)) <= RealScalar(64) * epsilon);
      }
    }
  }
}

EIGEN_DECLARE_TEST(structured_bccb) {
  STATIC_CHECK((!internal::structured_reciprocal_packet_traits<long double>::Vectorizable));
#ifdef EIGEN_VECTORIZE_SSE2
  STATIC_CHECK((internal::structured_reciprocal_packet_traits<float>::Vectorizable));
  STATIC_CHECK((internal::structured_reciprocal_packet_traits<double>::Vectorizable));
#else
  STATIC_CHECK((!internal::structured_reciprocal_packet_traits<float>::Vectorizable));
  STATIC_CHECK((!internal::structured_reciprocal_packet_traits<double>::Vectorizable));
#endif
  for (int i = 0; i < g_repeat; ++i) {
    // Products, dense assignment, coefficient access: scalar tier (N <= 32) and
    // 2-D FFT tier, including single-row/column-of-blocks degenerate shapes.
    CALL_SUBTEST_1((test_bccb_product<double>(1, 1)));
    CALL_SUBTEST_1((test_bccb_product<double>(3, 4)));
    CALL_SUBTEST_1((test_bccb_product<double>(5, 5)));
    CALL_SUBTEST_1((test_bccb_product<double>(8, 6)));
    CALL_SUBTEST_1((test_bccb_product<double>(7, 5)));  // odd prime dimensions
    CALL_SUBTEST_1((test_bccb_product<double>(1, 40)));
    CALL_SUBTEST_1((test_bccb_product<double>(40, 1)));
    CALL_SUBTEST_1((test_bccb_product<float>(6, 8)));
    CALL_SUBTEST_1((test_bccb_product<std::complex<double>>(4, 3)));
    CALL_SUBTEST_1((test_bccb_product<std::complex<double>>(9, 6)));
    CALL_SUBTEST_1((test_bccb_product<std::complex<float>>(6, 7)));

    // Transposition family with exact symbol round trips.
    CALL_SUBTEST_2((test_bccb_transpose<double>(3, 4)));
    CALL_SUBTEST_2((test_bccb_transpose<double>(8, 6)));
    CALL_SUBTEST_2((test_bccb_transpose<double>(1, 12)));
    CALL_SUBTEST_2((test_bccb_transpose<std::complex<double>>(6, 8)));
    CALL_SUBTEST_2((test_bccb_transpose<std::complex<float>>(5, 7)));

    // Direct and pseudo-inverse solves, degenerate operators.
    CALL_SUBTEST_3((test_bccb_solve<double>(1, 1)));
    CALL_SUBTEST_3((test_bccb_solve<double>(6, 8)));
    CALL_SUBTEST_3((test_bccb_solve<float>(5, 6)));
    CALL_SUBTEST_3((test_bccb_solve<std::complex<double>>(6, 6)));
    CALL_SUBTEST_3((test_bccb_rank_deficient<std::complex<double>>(6, 8, 4)));
    CALL_SUBTEST_3((test_bccb_rank_deficient<std::complex<float>>(4, 5, 2)));
    CALL_SUBTEST_3((test_bccb_zero<double>(4, 5)));
    CALL_SUBTEST_3(test_bccb_nan_propagation(6, 7));

    // Closed-form eigendecomposition, SVD, inverse, determinant.
    CALL_SUBTEST_4((test_bccb_eigen<double>(4, 5)));
    CALL_SUBTEST_4((test_bccb_eigen<double>(1, 8)));
    CALL_SUBTEST_4((test_bccb_eigen<std::complex<double>>(3, 5)));
    CALL_SUBTEST_4((test_bccb_svd<double>(4, 5)));
    CALL_SUBTEST_4((test_bccb_svd<std::complex<double>>(4, 4)));
    CALL_SUBTEST_4((test_bccb_svd<float>(3, 4)));
    CALL_SUBTEST_4((test_bccb_inverse<double>(6, 8)));
    CALL_SUBTEST_4((test_bccb_inverse<std::complex<double>>(5, 5)));
    CALL_SUBTEST_4((test_bccb_determinant<double>(3, 4)));
    CALL_SUBTEST_4((test_bccb_determinant<std::complex<double>>(3, 3)));

    // Matrix-free iterative solve and fixed-size generators.
    CALL_SUBTEST_5((test_bccb_matrix_free_cg<double>(8, 10)));
    CALL_SUBTEST_5((test_bccb_fixed<double, 3, 4>()));
    CALL_SUBTEST_5((test_bccb_fixed<std::complex<float>, 4, 3>()));

    // Non-5-smooth (prime) block dimensions: the padded product-embedding grid,
    // in each axis combination (n2 awkward, n1 awkward, both).
    CALL_SUBTEST_5((test_bccb_prime_dimensions<double>(7, 6)));
    CALL_SUBTEST_5((test_bccb_prime_dimensions<double>(6, 7)));
    CALL_SUBTEST_5((test_bccb_prime_dimensions<double>(7, 7)));
    CALL_SUBTEST_5((test_bccb_prime_dimensions<std::complex<double>>(11, 7)));
    CALL_SUBTEST_5((test_bccb_prime_dimensions<float>(7, 6)));

    // Product lifetime, aliasing, mixed-scalar promotion, dimension mismatches,
    // across the scalar-loop and FFT dispatch tiers.
    CALL_SUBTEST_6((test_bccb_delayed_product<double>(4, 5)));
    CALL_SUBTEST_6((test_bccb_delayed_product<std::complex<double>>(6, 8)));
    CALL_SUBTEST_6((test_bccb_aliased_product<double>(3, 4)));
    CALL_SUBTEST_6((test_bccb_aliased_product<double>(8, 6)));
    CALL_SUBTEST_6((test_bccb_aliased_product<std::complex<double>>(6, 8)));
    CALL_SUBTEST_6((test_bccb_aliased_expression<double>(3, 4)));
    CALL_SUBTEST_6((test_bccb_aliased_expression<double>(6, 8)));
    CALL_SUBTEST_6((test_bccb_aliased_expression<std::complex<double>>(6, 8)));
    CALL_SUBTEST_6((test_bccb_mixed_scalar<double>(3, 4)));
    CALL_SUBTEST_6((test_bccb_mixed_scalar<double>(6, 8)));
    CALL_SUBTEST_6((test_bccb_mixed_scalar<float>(8, 8)));
    CALL_SUBTEST_6(test_bccb_dimension_asserts());

    // Finite-range robustness: scaled transforms, the complex overflow boundary,
    // balanced determinant accumulation, and the rank threshold boundary.
    CALL_SUBTEST_7(test_bccb_finite_overflow());
    CALL_SUBTEST_7((test_bccb_fft_complex_boundary<double>(6, 8)));
    CALL_SUBTEST_7((test_bccb_fft_complex_boundary<float>(6, 8)));
    CALL_SUBTEST_7(test_bccb_determinant_scaled());
    CALL_SUBTEST_7(test_bccb_rank_boundaries());
    CALL_SUBTEST_7(test_bccb_rank_complex_boundary());
    CALL_SUBTEST_7(test_bccb_rank_flush_to_zero());

    // Entrywise Inf/NaN propagation: FFT-sized operators must fall back to the
    // direct kernel; small ones are IEEE-exact already. A zero right-hand side
    // under a non-finite operator yields NaN, not zero.
    CALL_SUBTEST_7((test_bccb_nonfinite_product<double>(6, 8)));
    CALL_SUBTEST_7((test_bccb_nonfinite_product<double>(3, 4)));
    CALL_SUBTEST_7((test_bccb_nonfinite_product<std::complex<double>>(6, 8)));
    CALL_SUBTEST_7(test_bccb_nonfinite_zero_rhs(6, 8));

    CALL_SUBTEST_8(test_structured_symbol_reciprocals<float>());
    CALL_SUBTEST_8(test_structured_symbol_reciprocals<double>());
    CALL_SUBTEST_8(test_structured_symbol_reciprocals<long double>());
    CALL_SUBTEST_8(test_structured_packet_reciprocals<float>());
    CALL_SUBTEST_8(test_structured_packet_reciprocals<double>());
    CALL_SUBTEST_8(test_circulant_inverse_modes<float>());
    CALL_SUBTEST_8(test_circulant_inverse_modes<double>());
  }
}
