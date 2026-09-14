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

// Reference dense circulant built straight from the generating column, so that the
// fast (FFT) product is validated against an independent construction of the matrix.
template <typename Scalar>
Matrix<Scalar, Dynamic, Dynamic> reference_circulant(const Matrix<Scalar, Dynamic, 1>& c) {
  const Index n = c.size();
  Matrix<Scalar, Dynamic, Dynamic> dense(n, n);
  for (Index j = 0; j < n; ++j)
    for (Index i = 0; i < n; ++i) {
      Index k = i - j;
      if (k < 0) k += n;
      dense(i, j) = c[k];
    }
  return dense;
}

template <typename Scalar>
Matrix<Scalar, Dynamic, Dynamic> reference_toeplitz(const Matrix<Scalar, Dynamic, 1>& c,
                                                    const Matrix<Scalar, Dynamic, 1>& r) {
  const Index m = c.size(), n = r.size();
  Matrix<Scalar, Dynamic, Dynamic> dense(m, n);
  for (Index j = 0; j < n; ++j)
    for (Index i = 0; i < m; ++i) dense(i, j) = (i >= j) ? c[i - j] : r[j - i];
  return dense;
}

template <typename Scalar>
void test_circulant_product(Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec c = Vec::Random(n);
  Circulant<Scalar> C(c);
  Mat dense = reference_circulant<Scalar>(c);

  // The operator agrees with the independently-built dense matrix, both through
  // coeff access and assigned to a dense matrix via its evaluator.
  Mat Cd = C;
  VERIFY_IS_APPROX(Cd, dense);
  for (Index t = 0; t < (std::min)(n, Index(5)); ++t) {
    Index i = internal::random<Index>(0, n - 1), j = internal::random<Index>(0, n - 1);
    VERIFY_IS_APPROX(C.coeff(i, j), dense(i, j));
  }

  // Fast matrix-vector and matrix-matrix products.
  Vec x = Vec::Random(n);
  VERIFY_IS_APPROX((C * x).eval(), (dense * x).eval());

  Mat X = Mat::Random(n, 3);
  VERIFY_IS_APPROX((C * X).eval(), (dense * X).eval());

  // Accumulation forms exercised by the iterative solvers.
  Vec y = Vec::Random(n);
  Vec y0 = y;
  y.noalias() += C * x;
  VERIFY_IS_APPROX(y, (y0 + dense * x).eval());
}

template <typename Scalar>
void test_circulant_solve(Index n) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  // Diagonally dominant => well conditioned, so the direct FFT solve is accurate.
  Vec c = Vec::Random(n);
  c[0] += Scalar(RealScalar(2 * n));
  Circulant<Scalar> C(c);
  Mat dense = reference_circulant<Scalar>(c);

  Vec b = Vec::Random(n);
  Vec x = C.solve(b);
  VERIFY_IS_APPROX((dense * x).eval(), b);

  // Multiple right-hand sides at once.
  Mat B = Mat::Random(n, 4);
  Mat Xs = C.solve(B);
  VERIFY_IS_APPROX((dense * Xs).eval(), B);
}

// The precomputed symbol holds the eigenvalues of the circulant matrix:
// C * f_k = symbol[k] * f_k, with f_k the k-th column of the inverse DFT matrix.
template <typename Scalar>
void test_circulant_symbol(Index n) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef std::complex<RealScalar> Complex;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Complex, Dynamic, 1> ComplexVec;

  Vec c = Vec::Random(n);
  Circulant<Scalar> C(c);
  VERIFY_IS_EQUAL(C.symbol().size(), n);
  Matrix<Complex, Dynamic, Dynamic> denseC = reference_circulant<Scalar>(c).template cast<Complex>();

  const Index step = numext::maxi<Index>(n / 4, 1);
  for (Index k = 0; k < n; k += step) {
    ComplexVec f(n);
    for (Index j = 0; j < n; ++j) f[j] = std::polar(RealScalar(1), RealScalar(2 * EIGEN_PI * j * k) / RealScalar(n));
    VERIFY_IS_APPROX((denseC * f).eval(), (C.symbol()[k] * f).eval());
  }
}

template <typename Scalar>
void test_toeplitz_product(Index m, Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec c = Vec::Random(m), r = Vec::Random(n);
  r[0] = c[0];  // diagonal entry; r[0] is ignored anyway
  Toeplitz<Scalar> T(c, r);
  Mat dense = reference_toeplitz<Scalar>(c, r);

  Mat Td = T;
  VERIFY_IS_APPROX(Td, dense);

  Vec x = Vec::Random(n);
  VERIFY_IS_APPROX((T * x).eval(), (dense * x).eval());

  Mat X = Mat::Random(n, 3);
  VERIFY_IS_APPROX((T * X).eval(), (dense * X).eval());
}

template <typename Scalar>
Matrix<Scalar, Dynamic, Dynamic> reference_hankel(const Matrix<Scalar, Dynamic, 1>& h, Index m, Index n) {
  Matrix<Scalar, Dynamic, Dynamic> dense(m, n);
  for (Index j = 0; j < n; ++j)
    for (Index i = 0; i < m; ++i) dense(i, j) = h[i + j];
  return dense;
}

template <typename Scalar>
void test_hankel_product(Index m, Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec h = Vec::Random(m + n - 1);
  Hankel<Scalar> H(h.head(m), h.tail(n));
  Mat dense = reference_hankel<Scalar>(h, m, n);

  // The stored generating sequence is rebuilt exactly from column + last row.
  VERIFY_IS_EQUAL(H.generator(), h);
  VERIFY_IS_EQUAL(Vec(H.column()), Vec(h.head(m)));
  VERIFY_IS_EQUAL(Vec(H.lastRow()), Vec(h.tail(n)));

  Mat Hd = H;
  VERIFY_IS_APPROX(Hd, dense);
  for (Index t = 0; t < (std::min)(m, Index(5)); ++t) {
    Index i = internal::random<Index>(0, m - 1), j = internal::random<Index>(0, n - 1);
    VERIFY_IS_APPROX(H.coeff(i, j), dense(i, j));
  }

  Vec x = Vec::Random(n);
  VERIFY_IS_APPROX((H * x).eval(), (dense * x).eval());

  Mat X = Mat::Random(n, 3);
  VERIFY_IS_APPROX((H * X).eval(), (dense * X).eval());

  // Accumulation forms exercised by the iterative solvers.
  Vec y = Vec::Random(m);
  Vec y0 = y;
  y.noalias() += H * x;
  VERIFY_IS_APPROX(y, (y0 + dense * x).eval());
}

// Reviewer regression (MR 2688): applying an exact 40x40 exchange matrix (as a
// Hankel operator, FFT tier) to a vector of huge finite entries must return the
// unchanged finite values, not NaNs: the FFT intermediates overflow unless each
// column is scaled by an exact power of two derived from the column's and the
// symbol's magnitudes.
void test_hankel_fft_overflow() {
  typedef Matrix<double, Dynamic, 1> Vec;
  const Index n = 40;
  const double huge = (std::numeric_limits<double>::max)() / 16;

  // H(i,j) = h[i+j] with h = e_{n-1} is the exchange matrix (anti-identity).
  Vec h = Vec::Zero(2 * n - 1);
  h[n - 1] = 1.0;
  Hankel<double> H(h.head(n), h.tail(n));

  Vec x = Vec::Constant(n, huge);
  Vec y = H * x;
  VERIFY(y.allFinite());
  VERIFY_IS_APPROX((y / huge).eval(), Vec::Ones(n).eval());  // y == x.reverse() == x

  // A huge generator makes a huge symbol; its exponent must be scaled out too.
  Vec hh = Vec::Zero(2 * n - 1);
  hh[n - 1] = huge;
  Hankel<double> Hh(hh.head(n), hh.tail(n));
  Vec z = Hh * Vec::Ones(n);
  VERIFY(z.allFinite());
  VERIFY_IS_APPROX((z / huge).eval(), Vec::Ones(n).eval());

  // A genuine NaN input still propagates -- through the direct kernel, entrywise
  // (see test_hankel_nonfinite_product); the scaling must not launder non-finite
  // inputs into finite outputs.
  Vec xn = Vec::Random(n);
  xn[n / 2] = std::numeric_limits<double>::quiet_NaN();
  Vec yn = H * xn;
  VERIFY(yn.hasNaN());
}

// The scaling exponents are derived from component-wise magnitudes: a finite
// complex value near the overflow threshold has a non-representable modulus,
// which would otherwise disable the scaling and turn an exactly representable
// product into NaN.
template <typename RealScalar>
void test_hankel_fft_complex_boundary(Index n) {
  typedef std::complex<RealScalar> Complex;
  typedef Matrix<RealScalar, Dynamic, 1> RVec;
  typedef Matrix<Complex, Dynamic, 1> CVec;
  const RealScalar kFftRoundTripTol = RealScalar(100) * NumTraits<RealScalar>::epsilon();
  const RealScalar big = RealScalar(0.75) * (std::numeric_limits<RealScalar>::max)();

  // Exchange matrix with a complex generator: the product returns the reversed
  // right-hand side -- the input itself, being constant -- even though |x_k|
  // overflows.
  CVec h = CVec::Zero(2 * n - 1);
  h[n - 1] = Complex(1);
  Hankel<Complex> H(h.head(n), h.tail(n));
  CVec x = CVec::Constant(n, Complex(big, big));
  CVec y = H * x;
  VERIFY(y.allFinite());
  VERIFY(((y - x).cwiseAbs() / big).maxCoeff() <= kFftRoundTripTol);

  // A real exchange operator applied to the same complex right-hand side takes
  // the mixed-scalar product path.
  RVec hr = RVec::Zero(2 * n - 1);
  hr[n - 1] = RealScalar(1);
  Hankel<RealScalar> Hr(hr.head(n), hr.tail(n));
  y = Hr * x;
  VERIFY(y.allFinite());
  VERIFY(((y - x).cwiseAbs() / big).maxCoeff() <= kFftRoundTripTol);
}

// transpose()/conjugate()/adjoint() return owning temporaries (and so does
// makeHankel), so the product expression must nest the structured operand by
// value: a delayed-evaluated expression has to outlive the temporary operator it
// was built from. The static check pins the value nesting; the behavioral checks
// would read freed memory if the product held a reference instead.
template <typename Scalar>
void test_hankel_delayed_product(Index m, Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  STATIC_CHECK(!std::is_reference<typename internal::ref_selector<Hankel<Scalar>>::type>::value);

  Vec h = Vec::Random(m + n - 1);
  Hankel<Scalar> H(h.head(m), h.tail(n));
  Mat dense = reference_hankel<Scalar>(h, m, n);

  Vec y = Vec::Random(m);
  auto expr = H.adjoint() * y;        // the adjoint temporary dies with the full expression
  Vec scribble = Vec::Random(m + n);  // reuses the temporary's freed heap storage
  Vec x = expr;
  VERIFY_IS_APPROX(x, (dense.adjoint() * y).eval());
  VERIFY_IS_EQUAL(scribble.size(), m + n);

  Vec x2 = Vec::Random(n);
  auto expr2 = makeHankel(h.head(m).eval(), h.tail(n).eval()) * x2;  // factory-returned temporary
  Vec scribble2 = Vec::Random(m + n);
  Vec y2 = expr2;
  VERIFY_IS_APPROX(y2, (dense * x2).eval());
  VERIFY_IS_EQUAL(scribble2.size(), m + n);
}

// The products carry the default product tag, so plain assignment materializes a
// temporary exactly like a dense product and an aliased right-hand side is safe:
// without it, x = H * x would read a zeroed right-hand side and x += H * x would
// interleave destination writes with right-hand-side reads.
template <typename Scalar>
void test_hankel_aliased_product(Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  // Square so that the destination and the right-hand side can alias.
  Vec h = Vec::Random(2 * n - 1);
  Hankel<Scalar> H(h.head(n), h.tail(n));
  Mat dense = reference_hankel<Scalar>(h, n, n);

  Vec x = Vec::Random(n);
  Vec y = x;
  y = H * y;
  VERIFY_IS_APPROX(y, (dense * x).eval());

  y = x;
  y += H * y;
  VERIFY_IS_APPROX(y, (x + dense * x).eval());

  y = x;
  y -= H * y;
  VERIFY_IS_APPROX(y, (x - dense * x).eval());

  Mat X = Mat::Random(n, 3);
  Mat Y = X;
  Y = H * Y;
  VERIFY_IS_APPROX(Y, (dense * X).eval());
}

// Aliasing beyond the same-object case: the default-product temporary must also
// resolve right-hand-side expressions that reference the destination, overlapping
// views of one buffer, and rectangular self-assignments where the destination is
// resized by the assignment.
template <typename Scalar>
void test_hankel_aliased_expression(Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec h = Vec::Random(2 * n - 1);
  Hankel<Scalar> H(h.head(n), h.tail(n));
  Mat dense = reference_hankel<Scalar>(h, n, n);

  // Right-hand-side expression referencing the destination.
  Vec x = Vec::Random(n), x0 = x;
  x = H * (x + Vec::Ones(n));
  VERIFY_IS_APPROX(x, (dense * (x0 + Vec::Ones(n))).eval());

  // Overlapping (shifted) segments of one buffer.
  Vec buf = Vec::Random(n + 1);
  Vec expected = dense * buf.tail(n);
  buf.head(n) = H * buf.tail(n);
  VERIFY_IS_APPROX(buf.head(n).eval(), expected);

  // Rectangular self-assignment: z = H * z resizes the destination, so the
  // product must be captured before the destination storage is touched.
  const Index m = n + 3;
  Vec hr = Vec::Random(m + n - 1);
  Hankel<Scalar> Hr(hr.head(m), hr.tail(n));
  Mat denseR = reference_hankel<Scalar>(hr, m, n);
  Vec z = Vec::Random(n), z0 = z;
  z = Hr * z;
  VERIFY_IS_EQUAL(z.size(), m);
  VERIFY_IS_APPROX(z, (denseR * z0).eval());
}

// Mixed-scalar products: a real operator applied to a complex right-hand side
// (and a complex operator applied to a real one) promotes to the complex product
// scalar, so alpha and the accumulation must run in the promoted type rather than
// the operator scalar.
template <typename RealScalar>
void test_hankel_mixed_scalar(Index m, Index n) {
  typedef std::complex<RealScalar> Complex;
  typedef Matrix<RealScalar, Dynamic, 1> RVec;
  typedef Matrix<Complex, Dynamic, 1> CVec;
  typedef Matrix<Complex, Dynamic, Dynamic> CMat;

  RVec h = RVec::Random(m + n - 1);
  Hankel<RealScalar> H(h.head(m), h.tail(n));
  CMat dense = reference_hankel<RealScalar>(h, m, n).template cast<Complex>();

  CVec x = CVec::Random(n);
  CVec y = H * x;
  VERIFY_IS_APPROX(y, (dense * x).eval());

  CVec y0 = CVec::Random(m);
  y = y0;
  y.noalias() += H * x;
  VERIFY_IS_APPROX(y, (y0 + dense * x).eval());

  CVec hc = CVec::Random(m + n - 1);
  Hankel<Complex> Hc(hc.head(m), hc.tail(n));
  CMat denseC = reference_hankel<Complex>(hc, m, n);
  RVec xr = RVec::Random(n);
  CVec z = Hc * xr;
  VERIFY_IS_APPROX(z, (denseC * xr).eval());
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
      if (x == y) continue;                    // finite match or same-signed infinities
      if ((x != x) && (y != y)) continue;      // both NaN
      if (!test_isApprox(x, y)) return false;  // finite roundoff
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
// across the whole output by the transforms. The reviewer reproducer (MR 2688):
// the 40x40 exchange matrix, as a Hankel operator on the FFT tier, applied to a
// vector with one +Inf must return +Inf in the mirrored entry; the remaining
// entries are NaN through the IEEE 0*Inf terms of the dense dot products.
template <typename Scalar>
void test_hankel_nonfinite_product(Index n) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;
  const RealScalar inf = std::numeric_limits<RealScalar>::infinity();
  const RealScalar nan = std::numeric_limits<RealScalar>::quiet_NaN();

  // The exchange matrix is the Hankel operator generated by h = e_{n-1}.
  Vec h = Vec::Zero(2 * n - 1);
  h[n - 1] = Scalar(1);
  Hankel<Scalar> H(h.head(n), h.tail(n));
  Mat dense = reference_hankel<Scalar>(h, n, n);

  // Inf in the right-hand side: the mirrored entry must be +Inf (for a complex
  // operator its real part; the 0*Inf cross term makes the imaginary part NaN).
  Vec x = Vec::Random(n);
  x[n / 4] = Scalar(inf);
  Vec y = H * x;
  VERIFY(ieee_entrywise_match(y, reference_product_ieee(dense, x)));
  VERIFY(numext::real(y[n - 1 - n / 4]) == inf);

  // NaN in the right-hand side.
  Vec xn = Vec::Random(n);
  xn[n - 1] = Scalar(nan);
  VERIFY(ieee_entrywise_match((H * xn).eval(), reference_product_ieee(dense, xn)));

  // Mixed multi-column right-hand side: the non-finite column falls back to the
  // direct kernel individually while the finite column keeps the FFT path.
  Mat Xm(n, 2);
  Xm.col(0) = Vec::Random(n);
  Xm.col(1) = x;
  Mat Ym = H * Xm;
  VERIFY_IS_APPROX(Ym.col(0).eval(), (dense * Xm.col(0)).eval());
  VERIFY(ieee_entrywise_match(Ym.col(1).eval(), reference_product_ieee(dense, Vec(Xm.col(1)))));

  // Inf in the generating sequence: the operator itself is non-finite, whatever
  // the right-hand side.
  Vec h2 = Vec::Random(2 * n - 1);
  h2[1] = Scalar(-inf);
  Hankel<Scalar> H2(h2.head(n), h2.tail(n));
  Mat dense2 = reference_hankel<Scalar>(h2, n, n);
  Vec x2 = Vec::Random(n);
  VERIFY(ieee_entrywise_match((H2 * x2).eval(), reference_product_ieee(dense2, x2)));

  // The Circulant and Toeplitz operators share the fallback pattern.
  Vec c = Vec::Random(n);
  Circulant<Scalar> C(c);
  Mat denseC = reference_circulant<Scalar>(c);
  VERIFY(ieee_entrywise_match((C * x).eval(), reference_product_ieee(denseC, x)));

  Vec tc = Vec::Random(n), tr = Vec::Random(n);
  tr[0] = tc[0];
  tr[n / 2] = Scalar(inf);
  Toeplitz<Scalar> T(tc, tr);
  Mat denseT = reference_toeplitz<Scalar>(tc, tr);
  VERIFY(ieee_entrywise_match((T * x2).eval(), reference_product_ieee(denseT, x2)));

  // Non-finite right-hand sides of Circulant::solve take the direct inverse
  // application; on the 1x1 operator this is a single scalar multiply by the
  // inverse coefficient, checked against the same multiply on the dense inverse.
  Vec b1(1);
  b1[0] = Scalar(inf);
  Circulant<Scalar> C1(Vec(Vec::Constant(1, Scalar(2))));
  Mat inv1(1, 1);
  inv1(0, 0) = Scalar(1) / Scalar(2);
  VERIFY(ieee_entrywise_match(C1.solve(b1), reference_product_ieee(inv1, b1)));
}

// The transposed / adjoint / conjugated Hankel operators agree with the dense
// references, both materialized and through their fast products. For rectangular
// operators on the FFT tier this validates the phase-multiplication symbol reuse
// (the DFT shift theorem), including its sign.
template <typename Scalar>
void test_hankel_transpose(Index m, Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec h = Vec::Random(m + n - 1);
  Hankel<Scalar> H(h.head(m), h.tail(n));
  Mat dense = reference_hankel<Scalar>(h, m, n);

  Mat Td = H.transpose();
  VERIFY_IS_APPROX(Td, Mat(dense.transpose()));
  Mat Ad = H.adjoint();
  VERIFY_IS_APPROX(Ad, Mat(dense.adjoint()));
  Mat Kd = H.conjugate();
  VERIFY_IS_APPROX(Kd, Mat(dense.conjugate()));

  Vec y = Vec::Random(m);
  VERIFY_IS_APPROX((H.transpose() * y).eval(), (dense.transpose() * y).eval());
  VERIFY_IS_APPROX((H.adjoint() * y).eval(), (dense.adjoint() * y).eval());
  Vec x = Vec::Random(n);
  VERIFY_IS_APPROX((H.conjugate() * x).eval(), (dense.conjugate() * x).eval());

  // The generating sequence round-trips exactly; the symbol round-trips to
  // rounding (the phase factors cancel only approximately when m != n).
  Hankel<Scalar> Htt = H.transpose().transpose();
  VERIFY_IS_EQUAL(Htt.generator(), h);
  VERIFY_IS_APPROX(Htt.symbol(), H.symbol());
  Mat Httd = Htt;
  VERIFY_IS_APPROX(Httd, dense);
}

// A real square Hankel matrix is symmetric: transpose() is an exact fixed point.
template <typename Scalar>
void test_hankel_symmetry(Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec h = Vec::Random(2 * n - 1);
  Hankel<Scalar> H(h.head(n), h.tail(n));
  Hankel<Scalar> Ht = H.transpose();
  VERIFY_IS_EQUAL(Ht.generator(), h);
  VERIFY_IS_EQUAL(Ht.symbol(), H.symbol());  // square: no phase multiplication at all
  Mat dense = H;
  VERIFY_IS_APPROX(dense, Mat(dense.transpose()));
}

// The column-reversed Toeplitz equivalent: toToeplitz() == H * E.
template <typename Scalar>
void test_hankel_to_toeplitz(Index m, Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec h = Vec::Random(m + n - 1);
  Hankel<Scalar> H(h.head(m), h.tail(n));
  Mat dense = reference_hankel<Scalar>(h, m, n);
  Mat Td = H.toToeplitz();
  VERIFY_IS_APPROX(Td, Mat(dense.rowwise().reverse()));
}

// Direct O(n^2) solve of a square Hankel system through the Toeplitz equivalent.
template <typename Scalar>
void test_hankel_solve(Index n) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  // h[n-1] is the constant anti-diagonal that becomes the diagonal of the
  // Toeplitz equivalent; boosting it makes the system well conditioned.
  Vec h = Vec::Random(2 * n - 1);
  h[n - 1] += Scalar(RealScalar(2 * n));
  Hankel<Scalar> H(h.head(n), h.tail(n));
  Mat dense = reference_hankel<Scalar>(h, n, n);

  Vec b = Vec::Random(n);
  Vec x = H.solve(b);
  VERIFY_IS_APPROX((dense * x).eval(), b);
  VERIFY_IS_APPROX(x, dense.fullPivLu().solve(b).eval());

  Mat B = Mat::Random(n, 3);
  Mat X = H.solve(B);
  VERIFY_IS_APPROX(X, dense.fullPivLu().solve(B).eval());
}

// A zero constant anti-diagonal makes the Toeplitz equivalent's diagonal zero, so
// its 1x1 leading minor is singular and the solve must go through the look-ahead
// init/block-step machinery. Verified through the residual (the look-ahead
// Levinson algorithm is weakly stable).
template <typename Scalar>
void test_hankel_solve_lookahead(Index n) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec h = Vec::Random(2 * n - 1);
  h[n - 1] = Scalar(0);
  Hankel<Scalar> H(h.head(n), h.tail(n));
  Mat dense = reference_hankel<Scalar>(h, n, n);

  Vec b = dense * Vec::Ones(n);
  Vec x = H.solve(b);
  const RealScalar tol = RealScalar(5e6) * NumTraits<RealScalar>::epsilon();  // ~1e-9 in double
  VERIFY((dense * x - b).norm() <= tol * b.norm());
}

// The Hilbert matrix is the canonical ill-conditioned Hankel matrix: H(i,j) =
// 1/(i+j+1), i.e. h[k] = 1/(k+1). Deterministic, so the tolerances below are
// calibrated with a ~100x margin against measured values. The look-ahead Levinson
// solver is weakly stable: the forward error grows like eps * cond, but the
// residual stays small far beyond the point where the solution itself is lost.
void test_hankel_hilbert() {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef Matrix<double, Dynamic, Dynamic> Mat;

  auto residual = [](Index n) {
    Vec h(2 * n - 1);
    for (Index k = 0; k < 2 * n - 1; ++k) h[k] = 1.0 / double(k + 1);
    Hankel<double> H(h.head(n), h.tail(n));
    Mat dense = reference_hankel<double>(h, n, n);
    Vec b = dense * Vec::Ones(n);
    Vec x = H.solve(b);
    return (dense * x - b).norm() / b.norm();
  };
  const double eps = NumTraits<double>::epsilon();
  VERIFY(residual(8) <= 5e6 * eps);    // ~1e-9;  cond ~ 1.5e10, measured residual ~ 1.4e-11
  VERIFY(residual(12) <= 5e10 * eps);  // ~1e-5; cond ~ 1.7e16, measured residual ~ 1.1e-7

  // The condition estimate of the Toeplitz equivalent must see the
  // ill-conditioning (measured ~ 2.7e15 at n = 12).
  Vec h(23);
  for (Index k = 0; k < 23; ++k) h[k] = 1.0 / double(k + 1);
  Hankel<double> H(h.head(12), h.tail(12));
  LookAheadLevinson<double> lev(H.toToeplitz());
  VERIFY(lev.conditionEstimate() >= 1e14);
}

// With adjoint() available, a rectangular Hankel feeds the matrix-free
// least-squares solvers end to end.
template <typename Scalar>
void test_hankel_least_squares(Index m, Index n) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec h = Vec::Random(m + n - 1);
  Hankel<Scalar> H(h.head(m), h.tail(n));
  Mat dense = reference_hankel<Scalar>(h, m, n);

  Vec b = Vec::Random(m);

  LSMR<Hankel<Scalar>, IdentityPreconditioner> lsmr;
  lsmr.setTolerance(RealScalar(1e-12)).setMaxIterations(20 * n);
  lsmr.compute(H);
  Vec x = lsmr.solve(b);
  VERIFY(lsmr.info() == Success);
  // Check LSMR's own convergence criterion -- the normal-equations residual --
  // rather than the forward error, whose accuracy degrades with cond^2 and would
  // make the test flaky for unlucky random operators.
  Vec r = b - dense * x;
  const RealScalar tol = RealScalar(5e7) * NumTraits<RealScalar>::epsilon();     // ~1e-8, relative
  const RealScalar absTol = RealScalar(5e3) * NumTraits<RealScalar>::epsilon();  // ~1e-12, floor
  VERIFY((dense.adjoint() * r).norm() <= tol * dense.norm() * r.norm() + absTol * b.norm());
}

template <typename Scalar, int M, int N>
void test_hankel_fixed() {
  typedef Matrix<Scalar, M, 1> ColVec;
  typedef Matrix<Scalar, N, 1> RowVec;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, M, N> MatMN;

  Vec h = Vec::Random(M + N - 1);
  ColVec c = h.template head<M>();
  RowVec r = h.template tail<N>();
  Hankel<Scalar, M, N> H(c, r);
  STATIC_CHECK((Hankel<Scalar, M, N>::RowsAtCompileTime == M));
  STATIC_CHECK((Hankel<Scalar, M, N>::ColsAtCompileTime == N));
  STATIC_CHECK((internal::remove_all_t<decltype(makeHankel(c, r))>::ColsAtCompileTime == N));
  STATIC_CHECK((internal::remove_all_t<decltype(H.transpose())>::RowsAtCompileTime == N));
  STATIC_CHECK((internal::remove_all_t<decltype(H.transpose())>::ColsAtCompileTime == M));

  MatMN dense = H;
  VERIFY_IS_APPROX(dense, MatMN(reference_hankel<Scalar>(h, M, N)));

  RowVec x = RowVec::Random();
  Matrix<Scalar, M, 1> y = H * x;
  VERIFY_IS_APPROX(y, (dense * x).eval());

  ColVec w = ColVec::Random();
  RowVec tw = H.transpose() * w;
  VERIFY_IS_APPROX(tw, (dense.transpose() * w).eval());
}

// solve() on a fixed-size operator returns a fixed-size result (reviewer
// regression for MR 2688: the return type used to have Dynamic rows). Either
// fixed dimension of the square operator determines the compile-time row count.
template <typename Scalar, int N>
void test_hankel_fixed_solve() {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, N, 1> VecN;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, N, N> MatN;

  STATIC_CHECK((internal::remove_all_t<decltype(std::declval<const Hankel<Scalar, N, N>&>().solve(
                    std::declval<const VecN&>()))>::RowsAtCompileTime == N));
  STATIC_CHECK((internal::remove_all_t<decltype(std::declval<const Hankel<Scalar, Dynamic, N>&>().solve(
                    std::declval<const Vec&>()))>::RowsAtCompileTime == N));
  STATIC_CHECK((internal::remove_all_t<decltype(std::declval<const Hankel<Scalar, N, Dynamic>&>().solve(
                    std::declval<const Vec&>()))>::RowsAtCompileTime == N));

  // h[N-1] is the constant anti-diagonal that becomes the diagonal of the
  // Toeplitz equivalent; boosting it makes the system well conditioned.
  Vec h = Vec::Random(2 * N - 1);
  h[N - 1] += Scalar(RealScalar(2 * N));
  Hankel<Scalar, N, N> H(h.template head<N>(), h.template tail<N>());
  MatN dense = H;

  VecN b = VecN::Random();
  VecN x = H.solve(b);
  VERIFY_IS_APPROX((dense * x).eval(), b);

  Matrix<Scalar, N, 2> B = Matrix<Scalar, N, 2>::Random();
  STATIC_CHECK((internal::remove_all_t<decltype(H.solve(B))>::ColsAtCompileTime == 2));
  Matrix<Scalar, N, 2> X = H.solve(B);
  VERIFY_IS_APPROX((dense * X).eval(), B);
}

// The transposed / adjoint / conjugated operators agree with the dense references,
// both materialized and through their fast products, which reuse the cached symbol
// (index reversal / conjugation) instead of computing new FFTs.
template <typename Scalar>
void test_circulant_transpose(Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec c = Vec::Random(n);
  Circulant<Scalar> C(c);
  Mat dense = reference_circulant<Scalar>(c);

  Mat Td = C.transpose();
  VERIFY_IS_APPROX(Td, Mat(dense.transpose()));
  Mat Ad = C.adjoint();
  VERIFY_IS_APPROX(Ad, Mat(dense.adjoint()));
  Mat Kd = C.conjugate();
  VERIFY_IS_APPROX(Kd, Mat(dense.conjugate()));

  Vec x = Vec::Random(n);
  VERIFY_IS_APPROX((C.transpose() * x).eval(), (dense.transpose() * x).eval());
  VERIFY_IS_APPROX((C.adjoint() * x).eval(), (dense.adjoint() * x).eval());
  VERIFY_IS_APPROX((C.conjugate() * x).eval(), (dense.conjugate() * x).eval());

  // The transposition family round-trips exactly: generators and symbols are pure
  // permutations / conjugations, so no FFT is recomputed and no roundoff accrues.
  Circulant<Scalar> Ctt = C.transpose().transpose();
  VERIFY_IS_EQUAL(Ctt.column(), c);
  VERIFY_IS_EQUAL(Ctt.symbol(), C.symbol());
  Circulant<Scalar> Caa = C.adjoint().adjoint();
  VERIFY_IS_EQUAL(Caa.column(), c);
  VERIFY_IS_EQUAL(Caa.symbol(), C.symbol());
}

template <typename Scalar>
void test_toeplitz_transpose(Index m, Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  // r[0] is deliberately left random (it is documented as ignored): the transposed
  // operator must take its diagonal from c[0], not from r[0].
  Vec c = Vec::Random(m), r = Vec::Random(n);
  Toeplitz<Scalar> T(c, r);
  Mat dense = reference_toeplitz<Scalar>(c, r);

  Mat Td = T.transpose();
  VERIFY_IS_APPROX(Td, Mat(dense.transpose()));
  Mat Ad = T.adjoint();
  VERIFY_IS_APPROX(Ad, Mat(dense.adjoint()));
  Mat Kd = T.conjugate();
  VERIFY_IS_APPROX(Kd, Mat(dense.conjugate()));

  Vec y = Vec::Random(m);
  VERIFY_IS_APPROX((T.transpose() * y).eval(), (dense.transpose() * y).eval());
  VERIFY_IS_APPROX((T.adjoint() * y).eval(), (dense.adjoint() * y).eval());
  Vec x = Vec::Random(n);
  VERIFY_IS_APPROX((T.conjugate() * x).eval(), (dense.conjugate() * x).eval());

  // Exact round trip (row[0] is normalized to the diagonal value on the way).
  Toeplitz<Scalar> Ttt = T.transpose().transpose();
  VERIFY_IS_EQUAL(Ttt.column(), c);
  if (n > 1) VERIFY_IS_EQUAL(Vec(Ttt.row().tail(n - 1)), Vec(r.tail(n - 1)));
  VERIFY_IS_EQUAL(Ttt.symbol(), T.symbol());
}

// transpose()/conjugate()/adjoint() return owning temporaries, so the product
// expression must nest the structured operand by value: a delayed-evaluated
// expression has to outlive the temporary operator it was built from. The static
// check pins the value nesting; the behavioral check would read freed memory if
// the product held a reference instead.
template <typename Scalar>
void test_circulant_delayed_product(Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  STATIC_CHECK(!std::is_reference<typename internal::ref_selector<Circulant<Scalar>>::type>::value);

  Vec c = Vec::Random(n), x = Vec::Random(n);
  Circulant<Scalar> C(c);
  Mat dense = reference_circulant<Scalar>(c);

  auto expr = C.adjoint() * x;        // the adjoint temporary dies with the full expression
  Vec scribble = Vec::Random(2 * n);  // reuses the temporary's freed heap storage
  Vec y = expr;
  VERIFY_IS_APPROX(y, (dense.adjoint() * x).eval());
  VERIFY_IS_EQUAL(scribble.size(), 2 * n);  // keep the scribble alive across the evaluation
}

template <typename Scalar>
void test_toeplitz_delayed_product(Index m, Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  STATIC_CHECK(!std::is_reference<typename internal::ref_selector<Toeplitz<Scalar>>::type>::value);

  Vec c = Vec::Random(m), r = Vec::Random(n), y = Vec::Random(m);
  Toeplitz<Scalar> T(c, r);
  Mat dense = reference_toeplitz<Scalar>(c, r);

  auto expr = T.adjoint() * y;
  Vec scribble = Vec::Random(m + n);
  Vec x = expr;
  VERIFY_IS_APPROX(x, (dense.adjoint() * y).eval());
  VERIFY_IS_EQUAL(scribble.size(), m + n);
}

// The products are tagged AliasFreeProduct, so no temporary shields an aliased
// right-hand side: the shared product implementation must copy it instead.
// Without the copy, x = C * x reads a zeroed right-hand side and x += C * x
// interleaves destination writes with right-hand-side reads.
template <typename Scalar>
void test_circulant_aliased_product(Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec c = Vec::Random(n);
  Circulant<Scalar> C(c);
  Mat dense = reference_circulant<Scalar>(c);

  Vec x = Vec::Random(n);
  Vec y = x;
  y = C * y;
  VERIFY_IS_APPROX(y, (dense * x).eval());

  y = x;
  y += C * y;
  VERIFY_IS_APPROX(y, (x + dense * x).eval());

  y = x;
  y -= C * y;
  VERIFY_IS_APPROX(y, (x - dense * x).eval());

  Mat X = Mat::Random(n, 3);
  Mat Y = X;
  Y = C * Y;
  VERIFY_IS_APPROX(Y, (dense * X).eval());
}

template <typename Scalar>
void test_toeplitz_aliased_product(Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  // Square so that the destination and the right-hand side can alias.
  Vec c = Vec::Random(n), r = Vec::Random(n);
  r[0] = c[0];
  Toeplitz<Scalar> T(c, r);
  Mat dense = reference_toeplitz<Scalar>(c, r);

  Vec x = Vec::Random(n);
  Vec y = x;
  y = T * y;
  VERIFY_IS_APPROX(y, (dense * x).eval());

  y = x;
  y += T * y;
  VERIFY_IS_APPROX(y, (x + dense * x).eval());
}

// Aliasing beyond the same-object case: the default-product temporary must also
// resolve right-hand-side expressions that reference the destination, overlapping
// views of one buffer, and rectangular self-assignments where the destination is
// resized by the assignment.
template <typename Scalar>
void test_structured_aliased_expression(Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec c = Vec::Random(n);
  Circulant<Scalar> C(c);
  Mat dense = reference_circulant<Scalar>(c);

  // Right-hand-side expression referencing the destination.
  Vec x = Vec::Random(n), x0 = x;
  x = C * (x + Vec::Ones(n));
  VERIFY_IS_APPROX(x, (dense * (x0 + Vec::Ones(n))).eval());

  // Overlapping (shifted) segments of one buffer.
  Vec buf = Vec::Random(n + 1);
  Vec expected = dense * buf.tail(n);
  buf.head(n) = C * buf.tail(n);
  VERIFY_IS_APPROX(buf.head(n).eval(), expected);

  // Rectangular self-assignment: x = T * x resizes the destination, so the
  // product must be captured before the destination storage is touched.
  const Index m = n + 3;
  Vec tc = Vec::Random(m), tr = Vec::Random(n);
  tr[0] = tc[0];
  Toeplitz<Scalar> T(tc, tr);
  Mat denseT = reference_toeplitz<Scalar>(tc, tr);
  Vec z = Vec::Random(n), z0 = z;
  z = T * z;
  VERIFY_IS_EQUAL(z.size(), m);
  VERIFY_IS_APPROX(z, (denseT * z0).eval());
}

// Mixed-scalar products: a real operator applied to a complex right-hand side
// (and a complex operator applied to a real one) promotes to the complex product
// scalar, so alpha and the accumulation must run in the promoted type rather than
// the operator scalar.
template <typename RealScalar>
void test_circulant_mixed_scalar(Index n) {
  typedef std::complex<RealScalar> Complex;
  typedef Matrix<RealScalar, Dynamic, 1> RVec;
  typedef Matrix<Complex, Dynamic, 1> CVec;
  typedef Matrix<Complex, Dynamic, Dynamic> CMat;

  RVec c = RVec::Random(n);
  Circulant<RealScalar> C(c);
  CMat dense = reference_circulant<RealScalar>(c).template cast<Complex>();

  CVec x = CVec::Random(n);
  CVec y = C * x;
  VERIFY_IS_APPROX(y, (dense * x).eval());

  CVec y0 = CVec::Random(n);
  y = y0;
  y.noalias() += C * x;
  VERIFY_IS_APPROX(y, (y0 + dense * x).eval());

  CVec cc = CVec::Random(n);
  Circulant<Complex> Cc(cc);
  CMat denseC = reference_circulant<Complex>(cc);
  RVec xr = RVec::Random(n);
  CVec z = Cc * xr;
  VERIFY_IS_APPROX(z, (denseC * xr).eval());
}

template <typename RealScalar>
void test_toeplitz_mixed_scalar(Index m, Index n) {
  typedef std::complex<RealScalar> Complex;
  typedef Matrix<RealScalar, Dynamic, 1> RVec;
  typedef Matrix<Complex, Dynamic, 1> CVec;
  typedef Matrix<Complex, Dynamic, Dynamic> CMat;

  RVec c = RVec::Random(m), r = RVec::Random(n);
  r[0] = c[0];
  Toeplitz<RealScalar> T(c, r);
  CMat dense = reference_toeplitz<RealScalar>(c, r).template cast<Complex>();

  CVec x = CVec::Random(n);
  CVec y = T * x;
  VERIFY_IS_APPROX(y, (dense * x).eval());
}

// The FFT product path accumulates up to p addends per transform, so a finite
// right-hand side near the overflow threshold (or a huge generator) used to
// overflow inside the transforms and return NaN for a representable result. The
// power-of-two scaled path must stay finite and accurate.
template <typename RealScalar>
void test_structured_fft_overflow(Index n) {
  typedef Matrix<RealScalar, Dynamic, 1> Vec;
  // FFT-transform roundoff bound for the identity round trip.
  const RealScalar kFftRoundTripTol = RealScalar(100) * NumTraits<RealScalar>::epsilon();
  const RealScalar huge = (std::numeric_limits<RealScalar>::max)() / RealScalar(16);

  // Identity circulant applied to a huge finite vector returns it unchanged.
  Vec c = Vec::Zero(n);
  c[0] = RealScalar(1);
  Circulant<RealScalar> C(c);
  Vec x = Vec::Constant(n, huge);
  Vec y = C * x;
  VERIFY(y.allFinite());
  VERIFY(((y - x).cwiseAbs() / huge).maxCoeff() <= kFftRoundTripTol);

  // A huge generator applied to a moderate vector: every output entry is huge
  // but representable.
  Circulant<RealScalar> Ch(Vec(huge * c));
  Vec ones = Vec::Ones(n);
  Vec z = Ch * ones;
  VERIFY(z.allFinite());
  VERIFY(((z.array() - huge).abs() / huge).maxCoeff() <= kFftRoundTripTol);

  // Identity Toeplitz, same huge right-hand side.
  Vec r = Vec::Zero(n);
  r[0] = c[0];
  Toeplitz<RealScalar> T(c, r);
  Vec w = T * x;
  VERIFY(w.allFinite());
  VERIFY(((w - x).cwiseAbs() / huge).maxCoeff() <= kFftRoundTripTol);
}

// The scaling exponents are derived from component-wise magnitudes: a finite
// complex value near the overflow threshold has a non-representable modulus,
// which would otherwise disable the scaling and turn an exactly representable
// product into NaN.
template <typename RealScalar>
void test_structured_fft_complex_boundary(Index n) {
  typedef std::complex<RealScalar> Complex;
  typedef Matrix<RealScalar, Dynamic, 1> RVec;
  typedef Matrix<Complex, Dynamic, 1> CVec;
  const RealScalar kFftRoundTripTol = RealScalar(100) * NumTraits<RealScalar>::epsilon();
  const RealScalar big = RealScalar(0.75) * (std::numeric_limits<RealScalar>::max)();

  // Identity circulant with a complex generator: the product returns the
  // right-hand side unchanged even though |x_k| overflows.
  CVec c = CVec::Zero(n);
  c[0] = Complex(1);
  Circulant<Complex> C(c);
  CVec x = CVec::Constant(n, Complex(big, big));
  CVec y = C * x;
  VERIFY(y.allFinite());
  VERIFY(((y - x).cwiseAbs() / big).maxCoeff() <= kFftRoundTripTol);

  // A real identity operator applied to the same complex right-hand side takes
  // the mixed-scalar product path.
  RVec cr = RVec::Zero(n);
  cr[0] = RealScalar(1);
  Circulant<RealScalar> Cr(cr);
  y = Cr * x;
  VERIFY(y.allFinite());
  VERIFY(((y - x).cwiseAbs() / big).maxCoeff() <= kFftRoundTripTol);
}

// A single Inf or NaN in the data must propagate like the reference product --
// through the dot products that touch it -- instead of being smeared into NaNs
// across the whole output by the transforms.
template <typename Scalar>
void test_structured_nonfinite_product(Index n) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;
  const RealScalar inf = std::numeric_limits<RealScalar>::infinity();
  const RealScalar nan = std::numeric_limits<RealScalar>::quiet_NaN();

  Vec c = Vec::Random(n);
  Circulant<Scalar> C(c);
  Mat dense = reference_circulant<Scalar>(c);

  // Inf in the right-hand side.
  Vec x = Vec::Random(n);
  x[n / 2] = Scalar(inf);
  VERIFY(ieee_entrywise_match((C * x).eval(), reference_product_ieee(dense, x)));

  // NaN in the right-hand side.
  Vec xn = Vec::Random(n);
  xn[n - 1] = Scalar(nan);
  VERIFY(ieee_entrywise_match((C * xn).eval(), reference_product_ieee(dense, xn)));

  // Mixed multi-column right-hand side: the non-finite column falls back to the
  // direct kernel individually while the finite column keeps the FFT path.
  Mat Xm(n, 2);
  Xm.col(0) = Vec::Random(n);
  Xm.col(1) = x;
  Mat Ym = C * Xm;
  VERIFY_IS_APPROX(Ym.col(0).eval(), (dense * Xm.col(0)).eval());
  VERIFY(ieee_entrywise_match(Ym.col(1).eval(), reference_product_ieee(dense, Vec(Xm.col(1)))));

  // Inf in the generator: the operator itself is non-finite, whatever the
  // right-hand side.
  Vec c2 = Vec::Random(n);
  c2[1] = Scalar(-inf);
  Circulant<Scalar> C2(c2);
  Mat dense2 = reference_circulant<Scalar>(c2);
  Vec x2 = Vec::Random(n);
  VERIFY(ieee_entrywise_match((C2 * x2).eval(), reference_product_ieee(dense2, x2)));

  // Toeplitz with an Inf in the row generator.
  Vec tc = Vec::Random(n), tr = Vec::Random(n);
  tr[0] = tc[0];
  tr[n / 2] = Scalar(inf);
  Toeplitz<Scalar> T(tc, tr);
  Mat denseT = reference_toeplitz<Scalar>(tc, tr);
  VERIFY(ieee_entrywise_match((T * x2).eval(), reference_product_ieee(denseT, x2)));

  // Non-finite right-hand sides of solve() take the direct pseudo-inverse
  // application; on the 1x1 operator this is a single scalar multiply by the
  // inverse coefficient, checked against the same multiply on the dense inverse.
  Vec b1(1);
  b1[0] = Scalar(inf);
  Circulant<Scalar> C1(Vec(Vec::Constant(1, Scalar(2))));
  Mat pinv1 = Mat(C1.inverse());
  VERIFY(ieee_entrywise_match(C1.solve(b1), reference_product_ieee(pinv1, b1)));
}

// Closed-form eigendecomposition: C * V = V * diag(eigenvalues) with V unitary.
template <typename Scalar>
void test_circulant_eigen(Index n) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef std::complex<RealScalar> Complex;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Complex, Dynamic, Dynamic> CMat;

  Vec c = Vec::Random(n);
  Circulant<Scalar> C(c);
  CMat denseC = reference_circulant<Scalar>(c).template cast<Complex>();

  Matrix<Complex, Dynamic, 1> lam = C.eigenvalues();
  CMat V = C.eigenvectors();
  VERIFY_IS_APPROX((denseC * V).eval(), (V * lam.asDiagonal()).eval());
  VERIFY_IS_APPROX((V.adjoint() * V).eval(), CMat(CMat::Identity(n, n)));
}

// Closed-form SVD: the singular values match JacobiSVD, and U * S * V^H
// reconstructs the matrix with unitary factors.
template <typename Scalar>
void test_circulant_svd(Index n) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef std::complex<RealScalar> Complex;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;
  typedef Matrix<Complex, Dynamic, Dynamic> CMat;

  Vec c = Vec::Random(n);
  Circulant<Scalar> C(c);
  Mat dense = reference_circulant<Scalar>(c);

  Matrix<RealScalar, Dynamic, 1> sv = C.singularValues();
  JacobiSVD<Mat> svd(dense);
  VERIFY_IS_APPROX(sv, svd.singularValues());

  // The symbol of a real operator is conjugate-symmetric, so every mode but the
  // self-conjugate ones (k = 0 and, for even n, k = n/2) is tied with its mirror.
  // The tie must be exact rather than accurate to roundoff: singularValues(),
  // matrixU() and matrixV() each order the modes from their own symbol
  // computation, and a pair ranked by its last bits can come out ordered one way
  // in one of them and the other way in another -- pairing a left singular vector
  // with the wrong right one. Runs of exactly equal values must therefore all be
  // even, apart from those self-conjugate moduli (fewer runs are odd when a
  // modulus coincidentally repeats).
  if (!NumTraits<Scalar>::IsComplex) {
    Index odd = 0;
    for (Index t = 0; t < n;) {
      Index run = t;
      while (run < n && numext::equal_strict(sv[run], sv[t])) ++run;
      if ((run - t) % 2 == 1) ++odd;
      t = run;
    }
    VERIFY(odd <= (n % 2 == 0 ? 2 : 1));
  }

  CMat U = C.matrixU(), V = C.matrixV();
  VERIFY_IS_APPROX((U * sv.template cast<Complex>().asDiagonal() * V.adjoint()).eval(),
                   CMat(dense.template cast<Complex>()));
  VERIFY_IS_APPROX((U.adjoint() * U).eval(), CMat(CMat::Identity(n, n)));
  VERIFY_IS_APPROX((V.adjoint() * V).eval(), CMat(CMat::Identity(n, n)));
}

// A rank-one circulant (the all-ones matrix): solve() must return the
// minimum-norm least-squares solution -- the SVD pseudo-inverse applied to b --
// for both consistent and inconsistent right-hand sides.
template <typename Scalar>
void test_circulant_minnorm_solve(Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec c = Vec::Ones(n);
  Circulant<Scalar> C(c);
  VERIFY_IS_EQUAL(C.rank(), 1);
  Mat dense = reference_circulant<Scalar>(c);
  JacobiSVD<Mat, ComputeThinU | ComputeThinV> svd(dense);

  Vec b = dense * Vec::Random(n);  // consistent right-hand side
  VERIFY_IS_APPROX(C.solve(b), svd.solve(b).eval());
  Vec b2 = Vec::Random(n);  // generally inconsistent; keep the projection non-zero
  b2.array() += Scalar(2);
  VERIFY_IS_APPROX(C.solve(b2), svd.solve(b2).eval());
}

// General rank-deficient circulant, synthesized by zeroing symbol entries. The
// numerical rank must count exactly the surviving entries and solve() must match
// the SVD pseudo-inverse. Scalar must be complex (the generator is an inverse DFT).
template <typename Scalar>
void test_circulant_rank_deficient(Index n, Index defect) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef std::complex<RealScalar> Complex;
  typedef Matrix<Complex, Dynamic, 1> CVec;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  CVec s = CVec::Random(n);
  s.array() += Complex(2);  // keep the surviving moduli away from the threshold
  for (Index k = 0; k < defect; ++k) s[2 * k + 1] = Complex(0);
  CVec ct(n);
  FFT<RealScalar> fft;
  fft.inv(ct, s, n);
  Vec c = ct;
  Circulant<Scalar> C(c);
  VERIFY_IS_EQUAL(C.rank(), n - defect);

  Mat dense = reference_circulant<Scalar>(c);
  JacobiSVD<Mat, ComputeThinU | ComputeThinV> svd(dense);
  Vec b = Vec::Random(n);
  VERIFY_IS_APPROX(C.solve(b), svd.solve(b).eval());

  // The SVD factors must stay unitary and reconstruct the matrix even with zero
  // singular values (exercising the arbitrary-phase completion of U and the
  // tie-handling of the shared sort).
  typedef Matrix<Complex, Dynamic, Dynamic> CMat;
  Matrix<RealScalar, Dynamic, 1> sv = C.singularValues();
  VERIFY((sv.tail(defect).array() <= RealScalar(n) * NumTraits<RealScalar>::epsilon() * sv[0]).all());
  CMat U = C.matrixU(), V = C.matrixV();
  VERIFY_IS_APPROX((U * sv.template cast<Complex>().asDiagonal() * V.adjoint()).eval(),
                   CMat(dense.template cast<Complex>()));
  VERIFY_IS_APPROX((U.adjoint() * U).eval(), CMat(CMat::Identity(n, n)));
  VERIFY_IS_APPROX((V.adjoint() * V).eval(), CMat(CMat::Identity(n, n)));
}

// The zero operator: rank 0, pseudo-inverse solve identically zero, all singular
// values zero. Pins the clamped-threshold boundary behavior.
template <typename Scalar>
void test_circulant_zero(Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;

  Circulant<Scalar> C(Vec(Vec::Zero(n)));
  VERIFY_IS_EQUAL(C.rank(), 0);
  VERIFY(C.singularValues().isZero());
  Vec b = Vec::Random(n);
  VERIFY(C.solve(b).isZero());
}

// A NaN in the generator must propagate to the solution (and count as non-zero in
// rank()) instead of being silently laundered into zeros by the rank threshold.
void test_circulant_nan_propagation(Index n) {
  typedef Matrix<double, Dynamic, 1> Vec;

  Vec c = Vec::Random(n);
  c[n / 2] = std::numeric_limits<double>::quiet_NaN();
  Circulant<double> C(c);
  VERIFY_IS_EQUAL(C.rank(), n);
  Vec b = Vec::Random(n);
  Vec x = C.solve(b);
  VERIFY(!(x.array() == x.array()).all());
}

template <typename Scalar>
void test_circulant_inverse(Index n) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec c = Vec::Random(n);
  c[0] += Scalar(RealScalar(2 * n));  // diagonally dominant => safely invertible
  Circulant<Scalar> C(c);
  Mat dense = reference_circulant<Scalar>(c);

  Mat inv = C.inverse();
  VERIFY_IS_APPROX((inv * dense).eval(), Mat(Mat::Identity(n, n)));

  // The inverse operator's fast product acts as a solve.
  Vec b = Vec::Random(n);
  VERIFY_IS_APPROX((C.inverse() * b).eval(), C.solve(b));
}

template <typename Scalar>
void test_circulant_determinant(Index n) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  // Diagonal dominance keeps every eigenvalue away from zero, so the determinant
  // is well conditioned and comparable against the dense LU-based value.
  Vec c = Vec::Random(n);
  c[0] += Scalar(RealScalar(2 * n));
  Circulant<Scalar> C(c);
  Mat dense = reference_circulant<Scalar>(c);
  VERIFY_IS_APPROX(C.determinant(), dense.determinant());
}

// Spectra with a wide dynamic range: the determinant is representable, but a
// plain product of the eigenvalues in FFT order overflows to infinity (or
// underflows to an exact zero) partway through. Pins the balanced accumulation
// in determinant().
void test_circulant_determinant_scaled() {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef Matrix<std::complex<double>, Dynamic, 1> CVec;
  const Index n = 1000;

  // 653 eigenvalues `lead` and 347 eigenvalues `rest`, arranged symmetrically
  // (s[k] == s[n-k]) so the generator is real, with the leading 327 FFT-order
  // indices all equal to `lead` so the plain running product leaves the
  // representable range partway through.
  auto generator = [n](double lead, double rest) {
    Vec mag = Vec::Constant(n, rest);
    mag[0] = lead;
    for (Index k = 1; k <= 326; ++k) mag[k] = mag[n - k] = lead;
    CVec s = mag.cast<std::complex<double>>();
    CVec ct(n);
    FFT<double> fft;
    fft.inv(ct, s, n);
    return Vec(ct.real());
  };

  // The spectrum is only reproduced up to the FFT round trip's forward error,
  // and the determinant multiplies ~1e3 such factors.
  const double tol = 1e8 * NumTraits<double>::epsilon();
  {
    // det = 10^653 * 10^-347 = 1e306; the naive partial product reaches 1e327.
    Circulant<double> C(generator(10.0, 0.1));
    const double det = C.determinant();
    VERIFY((numext::isfinite)(det));
    VERIFY(numext::abs(det / 1e306 - 1.0) <= tol);
  }
  {
    // det = 10^-653 * 10^347 = 1e-306; the naive partial product reaches 1e-327,
    // well below the smallest subnormal, and flushes to an exact zero.
    Circulant<double> C(generator(0.1, 10.0));
    const double det = C.determinant();
    VERIFY(det != 0.0);
    VERIFY(numext::abs(det / 1e-306 - 1.0) <= tol);
  }
}

// The rank decision at the clamped threshold (the smallest normal number): a
// smallest-normal symbol entry is still inverted -- the comparison is strict,
// matching SVDBase::rank(), which likewise reports rank one -- a subnormal entry
// is treated as an exact zero, and non-finite entries count as non-zero.
void test_circulant_rank_boundaries() {
  typedef Matrix<double, Dynamic, 1> Vec;
  const double mn = (std::numeric_limits<double>::min)();
  const Vec b = Vec::Ones(1);
  {
    Circulant<double> C(Vec(Vec::Constant(1, mn)));
    VERIFY_IS_EQUAL(C.rank(), 1);
    Vec x = C.solve(b);
    VERIFY((numext::isfinite)(x[0]));
    VERIFY_IS_APPROX(x[0], 1.0 / mn);
  }
  {
    Circulant<double> C(Vec(Vec::Constant(1, mn / 2)));  // subnormal
    VERIFY_IS_EQUAL(C.rank(), 0);
    VERIFY(C.solve(b).isZero());
  }
  {
    Circulant<double> C(Vec(Vec::Constant(1, std::numeric_limits<double>::infinity())));
    VERIFY_IS_EQUAL(C.rank(), 1);
    VERIFY((numext::isinf)(C.determinant()));
  }
}

// True where a subnormal result survives: under flush-to-zero -- NVHPC's default
// -fast, for one -- it is an exact zero instead. The volatile operands keep the
// compiler from folding the probe under IEEE semantics the run time does not use.
bool subnormals_survive() {
  volatile double vtiny = (std::numeric_limits<double>::min)();
  volatile double vhalf = 0.5;
  return !numext::is_exactly_zero(vtiny * vhalf);
}

// A finite complex symbol entry near the overflow threshold has a
// non-representable modulus. The rank threshold used to be computed from the raw
// moduli, turning it into infinity: the rank was under-reported and solve()
// zeroed valid Fourier modes. Both are now evaluated in an exactly rescaled
// frame.
void test_circulant_rank_complex_boundary() {
  typedef std::complex<double> Complex;
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef Matrix<Complex, Dynamic, 1> CVec;
  const double mx = (std::numeric_limits<double>::max)();

  // The length-one shortcut has no FFTs, but its pointwise division still needs
  // scaling: a huge right-hand side over the balanced O(1) symbol overflows before
  // the symbol exponent is folded back, though the result itself is moderate.
  const Vec oneCol = Vec::Constant(1, 0.75 * mx);
  const Vec oneB = Vec::Constant(1, 0.25 * mx);
  const Vec oneX = Circulant<double>(oneCol).solve(oneB);
  const double oneExpected = oneB[0] / oneCol[0];
  VERIFY(oneX.allFinite());
  VERIFY(numext::abs(oneX[0] / oneExpected - 1.0) <= 16 * NumTraits<double>::epsilon());

  // n = 2: the symbol is exactly [c0 + c1, c0 - c1], so pick the generator from
  // the desired spectrum. |s0| overflows while both of its components are finite.
  const Complex s0(0.75 * mx, 0.75 * mx), s1(1e300, 0.0);
  CVec c(2);
  c[0] = (s0 + s1) * 0.5;
  c[1] = (s0 - s1) * 0.5;
  Circulant<Complex> C(c);
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

  // A large right-hand side in the smaller retained mode must be scaled for the
  // division's amplification as well as for the transforms: the quotient is
  // moderate, but the balanced-frame division overflows without it.
  const double h = 0.25 * mx;
  CVec largeB(2);
  largeB << Complex(h), Complex(-h);
  const CVec largeX = C.solve(largeB);
  const double expected = h / numext::real(C.symbol()[1]);
  CVec expectedX(2);
  expectedX << Complex(expected), Complex(-expected);
  VERIFY(largeX.allFinite());
  VERIFY(((largeX - expectedX).cwiseAbs().maxCoeff() / expected) <= 32 * NumTraits<double>::epsilon());

  // The reciprocal of an entry near the overflow boundary is subnormal but
  // representable, and must not collapse to zero: formed as conj(z)/|z|^2, or by
  // Smith's algorithm, it overflows to exactly zero and drops the mode. Gated on
  // flush-to-zero, where that entry is not representable at all -- which is why
  // solve() divides by the symbol rather than multiplying by its reciprocal.
  const CVec sinv = C.inverse().symbol();
  VERIFY(sinv.allFinite());
  if (subnormals_survive()) VERIFY_IS_EQUAL((sinv.array() == Complex(0)).count(), 0);

  // A genuinely negligible second entry still truncates in the scaled frame.
  CVec c2(2);
  c2[0] = (s0 + Complex(1)) * 0.5;
  c2[1] = (s0 - Complex(1)) * 0.5;
  VERIFY_IS_EQUAL(Circulant<Complex>(c2).rank(), 1);
}

// Fixed-size operators: generators are stored in fixed-size vectors, products and
// solves return fixed-size results, and small sizes go through the coeff-based
// product dispatch.
template <typename Scalar, int N>
void test_circulant_fixed() {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, N, 1> VecN;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, N, N> MatN;

  VecN c = VecN::Random();
  c[0] += Scalar(RealScalar(2 * N));  // well conditioned for the solve below
  Circulant<Scalar, N> C(c);
  STATIC_CHECK((Circulant<Scalar, N>::RowsAtCompileTime == N));
  STATIC_CHECK((internal::remove_all_t<decltype(makeCirculant(c))>::RowsAtCompileTime == N));

  MatN dense = C;
  VERIFY_IS_APPROX(dense, MatN(reference_circulant<Scalar>(Vec(c))));

  VecN x = VecN::Random();
  VecN y = C * x;
  VERIFY_IS_APPROX(y, (dense * x).eval());

  VecN b = VecN::Random();
  VecN xs = C.solve(b);
  VERIFY_IS_APPROX((dense * xs).eval(), b);

  STATIC_CHECK((internal::remove_all_t<decltype(C.transpose())>::RowsAtCompileTime == N));
  VecN xt = C.transpose() * x;
  VERIFY_IS_APPROX(xt, (dense.transpose() * x).eval());
}

template <typename Scalar, int M, int N>
void test_toeplitz_fixed() {
  typedef Matrix<Scalar, M, 1> ColVec;
  typedef Matrix<Scalar, N, 1> RowVec;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, M, N> MatMN;

  ColVec c = ColVec::Random();
  RowVec r = RowVec::Random();
  r[0] = c[0];
  Toeplitz<Scalar, M, N> T(c, r);
  STATIC_CHECK((Toeplitz<Scalar, M, N>::RowsAtCompileTime == M));
  STATIC_CHECK((Toeplitz<Scalar, M, N>::ColsAtCompileTime == N));
  STATIC_CHECK((internal::remove_all_t<decltype(makeToeplitz(c, r))>::ColsAtCompileTime == N));

  MatMN dense = T;
  VERIFY_IS_APPROX(dense, MatMN(reference_toeplitz<Scalar>(Vec(c), Vec(r))));

  RowVec x = RowVec::Random();
  Matrix<Scalar, M, 1> y = T * x;
  VERIFY_IS_APPROX(y, (dense * x).eval());

  STATIC_CHECK((internal::remove_all_t<decltype(T.transpose())>::RowsAtCompileTime == N));
  STATIC_CHECK((internal::remove_all_t<decltype(T.transpose())>::ColsAtCompileTime == M));
  ColVec w = ColVec::Random();
  RowVec tw = T.transpose() * w;
  VERIFY_IS_APPROX(tw, (dense.transpose() * w).eval());
}

template <typename Scalar>
void test_matrix_free_cg(Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  // Symmetric, strongly diagonally dominant circulant => SPD.
  Vec c = Vec::Zero(n);
  c[0] = Scalar(4);
  c[1] = Scalar(-1);
  c[n - 1] = Scalar(-1);
  Circulant<Scalar> C(c);
  Mat dense = reference_circulant<Scalar>(c);

  Vec b = Vec::Random(n);
  ConjugateGradient<Circulant<Scalar>, Lower | Upper, IdentityPreconditioner> cg;
  cg.compute(C);
  Vec x = cg.solve(b);
  VERIFY(cg.info() == Success);
  VERIFY_IS_APPROX((dense * x).eval(), b);
}

template <typename Scalar>
void test_matrix_free_gmres(Index n) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  // Strongly diagonally dominant (nonsymmetric) Toeplitz.
  Vec c = Vec::Random(n) * Scalar(RealScalar(0.1));
  Vec r = Vec::Random(n) * Scalar(RealScalar(0.1));
  c[0] = Scalar(3);
  r[0] = Scalar(3);
  Toeplitz<Scalar> T(c, r);
  Mat dense = reference_toeplitz<Scalar>(c, r);

  Vec b = Vec::Random(n);
  GMRES<Toeplitz<Scalar>, IdentityPreconditioner> gmres;
  gmres.compute(T);
  Vec x = gmres.solve(b);
  VERIFY(gmres.info() == Success);
  VERIFY_IS_APPROX((dense * x).eval(), b);
}

// With adjoint() available, the rectangular operators feed Eigen's matrix-free
// least-squares solvers end to end: both LSMR and LeastSquaresConjugateGradient
// evaluate mat.adjoint() * v through the fast product.
template <typename Scalar>
void test_matrix_free_least_squares(Index m, Index n) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec c = Vec::Random(m), r = Vec::Random(n);
  c[0] += Scalar(RealScalar(4));  // boost the diagonal to bound the conditioning
  Toeplitz<Scalar> T(c, r);
  Mat dense = reference_toeplitz<Scalar>(c, r);

  Vec b = Vec::Random(m);
  Vec xref = dense.colPivHouseholderQr().solve(b);

  // Forward-error bound for the diagonally-boosted (well-conditioned) operator.
  const RealScalar tol = RealScalar(5e8) * NumTraits<RealScalar>::epsilon();  // ~1e-7 in double

  LSMR<Toeplitz<Scalar>, IdentityPreconditioner> lsmr;
  lsmr.setTolerance(RealScalar(1e-12)).setMaxIterations(10 * n);
  lsmr.compute(T);
  Vec x = lsmr.solve(b);
  VERIFY(lsmr.info() == Success);
  VERIFY((x - xref).norm() <= tol * xref.norm());

  LeastSquaresConjugateGradient<Toeplitz<Scalar>, IdentityPreconditioner> lscg;
  lscg.setTolerance(RealScalar(1e-12));
  lscg.setMaxIterations(10 * n);
  lscg.compute(T);
  Vec x2 = lscg.solve(b);
  VERIFY(lscg.info() == Success);
  VERIFY((x2 - xref).norm() <= tol * xref.norm());
}

// Diagonally dominant (well-conditioned) Toeplitz: the look-ahead solver must agree
// with a dense LU solve.
template <typename Scalar>
void test_levinson_wellcond(Index n) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec c = Vec::Random(n), r = Vec::Random(n);
  c[0] = r[0] = Scalar(RealScalar(2 * n));
  Toeplitz<Scalar> T(c, r);
  Mat dense = T;

  Vec b = Vec::Random(n);
  LookAheadLevinson<Scalar> lev(T);
  VERIFY(lev.info() == Success);
  Vec x = lev.solve(b);
  VERIFY_IS_APPROX(x, dense.fullPivLu().solve(b).eval());

  // Multiple right-hand sides.
  Mat B = Mat::Random(n, 3);
  VERIFY_IS_APPROX(lev.solve(B), dense.fullPivLu().solve(B).eval());

  // Transposed and adjoint systems reuse the same factorization (persymmetry).
  Vec xt = lev.transpose().solve(b);
  VERIFY_IS_APPROX(xt, dense.transpose().fullPivLu().solve(b).eval());
  Vec xa = lev.adjoint().solve(b);
  VERIFY_IS_APPROX(xa, dense.adjoint().fullPivLu().solve(b).eval());
  Mat Xt = lev.transpose().solve(B);
  VERIFY_IS_APPROX(Xt, dense.transpose().fullPivLu().solve(B).eval());
}

// Indefinite / ill-conditioned matrices that force look-ahead block steps. The
// generators and required block sizes are from Chan & Hansen's test set; the true
// solution is the all-ones vector.
void test_levinson_lookahead() {
  typedef Matrix<double, Dynamic, 1> Vec;
  // Loose bound for these deliberately ill-conditioned look-ahead cases (~1e-9);
  // the look-ahead Levinson recursion is weakly stable, so the forward error is a
  // large but bounded multiple of epsilon.
  const double tol = 5e6 * NumTraits<double>::epsilon();
  auto check = [tol](const Vec& c, const Vec& r, Index pmax) {
    Toeplitz<double> T(c, r);
    Vec xt = Vec::Ones(c.size());
    Vec b = T * xt;
    LookAheadLevinson<double> lev;
    lev.setMaxBlockSize(pmax).compute(T);
    VERIFY(lev.info() == Success);
    Vec x = lev.solve(b);
    VERIFY((x - xt).norm() <= tol * xt.norm());

    // The transposed solve must track the same look-ahead block steps.
    Matrix<double, Dynamic, Dynamic> dense = T;
    Vec bt = dense.transpose() * xt;
    Vec y = lev.transpose().solve(bt);
    VERIFY((y - xt).norm() <= tol * xt.norm());
  };

  Vec c1(6), r1(6);  // Sweet-1 (block size 2)
  c1 << 4, 6, 71.0 / 15 + 5e-8, 5, 3, 1;
  r1 << 4, 8, 1, 6, 2, 3;
  check(c1, r1, 3);

  Vec c2(6), r2(6);  // Sweet-2 (block size 2)
  c2 << 8, 4, -34 + 5e-13, 5, 3, 1;
  r2 << 8, 4, 1, 6, 2, 3;
  check(c2, r2, 3);

  Vec c3(13), r3(13);  // Sweet-3 (block size 6)
  c3 << 5, 1, -3, 12.755, -19.656, 28.361, -7, -1, 2, 1, -6, 1, -0.5;
  r3 << 5, -1, 6, 2, 5.697, 5.850, 3, -5, -2, -7, 1, 10, -15;
  check(c3, r3, 6);

  // shifted KMS: leading submatrices T_k with k = 1,4,7,... are singular, so the
  // full order n must be a multiple of 3 for T_n itself to be non-singular.
  for (Index n : {15, 30, 60}) {
    Vec c(n), r(n);
    c[0] = r[0] = 1e-14;
    for (Index i = 1; i < n; ++i) c[i] = r[i] = std::pow(0.5, double(i - 1));
    check(c, r, 3);
  }
}

// Fixed-size Toeplitz operators also feed the solver.
void test_levinson_fixed() {
  typedef Matrix<double, 12, 1> Vec12;
  Vec12 c = Vec12::Random(), r = Vec12::Random();
  c[0] = r[0] = 24.0;
  Toeplitz<double, 12, 12> T(c, r);
  Matrix<double, 12, 12> dense = T;
  Vec12 b = Vec12::Random();
  LookAheadLevinson<double> lev(T);
  VERIFY(lev.info() == Success);
  VERIFY_IS_APPROX(lev.solve(b), dense.fullPivLu().solve(b).eval());
}

// Evaluating the Solve expression inside a larger expression (rather than
// assigning it straight to a plain matrix) goes through evaluator<Solve>, which
// caches pointers into its preallocated result before _solve_impl runs:
// _solve_impl must copy into the destination coefficient-wise, never move-assign
// it (a move steals the buffer and leaves the cached pointers dangling). Only a
// matrix right-hand side is affected: a vector one has a different plain type,
// which cannot select the move-assignment. Hankel::solve wraps the Levinson
// solve in exactly such an expression (colwise().reverse()).
template <typename Scalar>
void test_levinson_solve_in_expression(Index n) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec c = Vec::Random(n), r = Vec::Random(n);
  r[0] = c[0] += Scalar(RealScalar(2 * n));  // diagonally dominant => well conditioned
  Toeplitz<Scalar> T(c, r);
  LookAheadLevinson<Scalar> lev(T);
  VERIFY(lev.info() == Success);

  Mat B = Mat::Random(n, 3);
  Mat direct = lev.solve(B);  // plain assignment: solves straight into `direct`
  Mat inExpr = lev.solve(B).colwise().reverse();
  VERIFY_IS_APPROX(inExpr, direct.colwise().reverse().eval());

  Vec b = Vec::Random(n);
  Vec x = lev.solve(b).colwise().reverse();
  VERIFY_IS_APPROX(x, Vec(Vec(lev.solve(b)).reverse()));
}

// A numerically singular Toeplitz must be reported through info().
void test_levinson_singular() {
  typedef Matrix<double, Dynamic, 1> Vec;
  for (Index n : {4, 9}) {
    Vec c = Vec::Ones(n), r = Vec::Ones(n);  // all-ones Toeplitz is rank 1 for n >= 2
    LookAheadLevinson<double> lev(Toeplitz<double>(c, r));
    VERIFY(lev.info() == NumericalIssue);
  }
}

EIGEN_DECLARE_TEST(structured_matrices) {
  for (int i = 0; i < g_repeat; ++i) {
    // Circulant: direct path (small), FFT path (composite and prime sizes), edge cases.
    CALL_SUBTEST_1((test_circulant_product<double>(1)));
    CALL_SUBTEST_1((test_circulant_product<double>(2)));
    CALL_SUBTEST_1((test_circulant_product<double>(8)));
    CALL_SUBTEST_1((test_circulant_product<double>(64)));
    CALL_SUBTEST_1((test_circulant_product<double>(97)));  // prime: padded embedding product
    CALL_SUBTEST_1((test_circulant_product<std::complex<double>>(97)));
    CALL_SUBTEST_1((test_circulant_product<float>(48)));
    CALL_SUBTEST_1((test_circulant_product<std::complex<double>>(7)));  // direct path, complex
    CALL_SUBTEST_1((test_circulant_product<std::complex<double>>(50)));
    CALL_SUBTEST_1((test_circulant_product<std::complex<float>>(40)));
    CALL_SUBTEST_1((test_circulant_solve<double>(1)));  // degenerate 1x1 solve
    CALL_SUBTEST_1((test_circulant_solve<double>(8)));
    CALL_SUBTEST_1((test_circulant_solve<double>(50)));
    CALL_SUBTEST_1((test_circulant_solve<double>(97)));  // prime: solve keeps the exact-size symbol
    CALL_SUBTEST_1((test_circulant_solve<std::complex<double>>(40)));
    CALL_SUBTEST_1((test_circulant_solve<float>(32)));
    CALL_SUBTEST_1((test_circulant_symbol<double>(16)));
    CALL_SUBTEST_1((test_circulant_symbol<std::complex<double>>(12)));

    // Toeplitz: square, tall, wide, small (direct), single row/column, real and complex.
    CALL_SUBTEST_2((test_toeplitz_product<double>(1, 1)));
    CALL_SUBTEST_2((test_toeplitz_product<double>(2, 2)));
    CALL_SUBTEST_2((test_toeplitz_product<double>(10, 10)));
    CALL_SUBTEST_2((test_toeplitz_product<double>(64, 64)));
    CALL_SUBTEST_2((test_toeplitz_product<double>(96, 48)));
    CALL_SUBTEST_2((test_toeplitz_product<double>(48, 96)));
    CALL_SUBTEST_2((test_toeplitz_product<double>(1, 40)));  // single row, FFT path
    CALL_SUBTEST_2((test_toeplitz_product<double>(40, 1)));  // single column, FFT path
    CALL_SUBTEST_2((test_toeplitz_product<float>(50, 50)));
    CALL_SUBTEST_2((test_toeplitz_product<std::complex<double>>(5, 7)));  // direct path, complex
    CALL_SUBTEST_2((test_toeplitz_product<std::complex<double>>(48, 64)));
    CALL_SUBTEST_2((test_toeplitz_product<std::complex<float>>(40, 40)));

    // Matrix-free iterative solves through the existing solvers.
    CALL_SUBTEST_3((test_matrix_free_cg<double>(80)));
    CALL_SUBTEST_3((test_matrix_free_gmres<double>(80)));
    CALL_SUBTEST_3((test_matrix_free_least_squares<double>(60, 40)));
    CALL_SUBTEST_3((test_matrix_free_least_squares<std::complex<double>>(48, 32)));

    // Fixed-size operators: small (coeff-based dispatch) and above the FFT threshold.
    CALL_SUBTEST_4((test_circulant_fixed<double, 4>()));
    CALL_SUBTEST_4((test_circulant_fixed<std::complex<float>, 4>()));
    CALL_SUBTEST_4((test_circulant_fixed<double, 48>()));
    CALL_SUBTEST_4((test_toeplitz_fixed<double, 4, 6>()));
    CALL_SUBTEST_4((test_toeplitz_fixed<double, 40, 24>()));
    CALL_SUBTEST_4((test_toeplitz_fixed<std::complex<float>, 6, 4>()));

    // Transposition family across the dispatch tiers (scalar, direct, FFT incl. prime).
    CALL_SUBTEST_6((test_circulant_transpose<double>(1)));
    CALL_SUBTEST_6((test_circulant_transpose<double>(8)));
    CALL_SUBTEST_6((test_circulant_transpose<double>(24)));
    CALL_SUBTEST_6((test_circulant_transpose<double>(97)));
    CALL_SUBTEST_6((test_circulant_transpose<float>(48)));
    CALL_SUBTEST_6((test_circulant_transpose<std::complex<double>>(40)));
    CALL_SUBTEST_6((test_circulant_transpose<std::complex<float>>(33)));
    CALL_SUBTEST_6((test_toeplitz_transpose<double>(1, 1)));
    CALL_SUBTEST_6((test_toeplitz_transpose<double>(12, 7)));
    CALL_SUBTEST_6((test_toeplitz_transpose<double>(64, 40)));
    CALL_SUBTEST_6((test_toeplitz_transpose<double>(40, 64)));
    CALL_SUBTEST_6((test_toeplitz_transpose<double>(1, 40)));
    CALL_SUBTEST_6((test_toeplitz_transpose<std::complex<double>>(48, 48)));
    CALL_SUBTEST_6((test_toeplitz_transpose<std::complex<float>>(20, 36)));

    // Closed-form eigendecomposition, SVD, pseudo-inverse solve, inverse, determinant.
    CALL_SUBTEST_7((test_circulant_eigen<double>(1)));
    CALL_SUBTEST_7((test_circulant_eigen<double>(16)));
    CALL_SUBTEST_7((test_circulant_eigen<double>(40)));
    CALL_SUBTEST_7((test_circulant_eigen<std::complex<double>>(21)));
    CALL_SUBTEST_7((test_circulant_svd<double>(1)));
    CALL_SUBTEST_7((test_circulant_svd<double>(18)));  // conjugate-pair moduli routinely differ in the last bits
    CALL_SUBTEST_7((test_circulant_svd<double>(24)));
    CALL_SUBTEST_7((test_circulant_svd<std::complex<double>>(18)));
    CALL_SUBTEST_7((test_circulant_svd<float>(12)));
    CALL_SUBTEST_7((test_circulant_minnorm_solve<double>(20)));
    CALL_SUBTEST_7((test_circulant_minnorm_solve<std::complex<double>>(15)));
    CALL_SUBTEST_7((test_circulant_rank_deficient<std::complex<double>>(24, 3)));
    CALL_SUBTEST_7((test_circulant_rank_deficient<std::complex<double>>(50, 5)));
    CALL_SUBTEST_7((test_circulant_rank_deficient<std::complex<float>>(16, 2)));
    CALL_SUBTEST_7((test_circulant_zero<double>(12)));
    CALL_SUBTEST_7((test_circulant_zero<std::complex<double>>(7)));
    CALL_SUBTEST_7(test_circulant_nan_propagation(20));
    CALL_SUBTEST_7((test_circulant_inverse<double>(1)));
    CALL_SUBTEST_7((test_circulant_inverse<double>(50)));
    CALL_SUBTEST_7((test_circulant_inverse<double>(97)));  // prime: inverse rebuilds the padded product symbol
    CALL_SUBTEST_7((test_circulant_inverse<std::complex<double>>(20)));
    CALL_SUBTEST_7((test_circulant_inverse<float>(16)));
    CALL_SUBTEST_7((test_circulant_determinant<double>(1)));
    CALL_SUBTEST_7((test_circulant_determinant<double>(12)));
    CALL_SUBTEST_7((test_circulant_determinant<std::complex<double>>(9)));

    // Numerical and lifetime boundaries: value-nested (owning) delayed products,
    // aliased products across the dispatch tiers (scalar, direct, FFT), the
    // balanced determinant accumulation, and the rank threshold boundary.
    CALL_SUBTEST_8((test_circulant_delayed_product<double>(24)));
    CALL_SUBTEST_8((test_circulant_delayed_product<std::complex<double>>(48)));
    CALL_SUBTEST_8((test_toeplitz_delayed_product<double>(40, 24)));
    CALL_SUBTEST_8((test_toeplitz_delayed_product<std::complex<float>>(12, 8)));
    CALL_SUBTEST_8((test_circulant_aliased_product<double>(8)));
    CALL_SUBTEST_8((test_circulant_aliased_product<double>(64)));
    CALL_SUBTEST_8((test_circulant_aliased_product<std::complex<double>>(40)));
    CALL_SUBTEST_8((test_toeplitz_aliased_product<double>(12)));
    CALL_SUBTEST_8((test_toeplitz_aliased_product<double>(48)));
    CALL_SUBTEST_8((test_toeplitz_aliased_product<std::complex<float>>(36)));
    CALL_SUBTEST_8((test_circulant_mixed_scalar<double>(8)));
    CALL_SUBTEST_8((test_circulant_mixed_scalar<double>(24)));
    CALL_SUBTEST_8((test_circulant_mixed_scalar<double>(64)));
    CALL_SUBTEST_8((test_circulant_mixed_scalar<float>(97)));
    CALL_SUBTEST_8((test_toeplitz_mixed_scalar<double>(8, 8)));
    CALL_SUBTEST_8((test_toeplitz_mixed_scalar<double>(64, 40)));
    CALL_SUBTEST_8(test_circulant_determinant_scaled());
    CALL_SUBTEST_8(test_circulant_rank_boundaries());
    CALL_SUBTEST_8(test_circulant_rank_complex_boundary());
    CALL_SUBTEST_8((test_structured_fft_overflow<double>(40)));
    CALL_SUBTEST_8((test_structured_fft_overflow<double>(97)));  // prime, no 5-smooth padding
    CALL_SUBTEST_8((test_structured_fft_overflow<float>(40)));
    CALL_SUBTEST_8((test_structured_fft_complex_boundary<double>(40)));
    CALL_SUBTEST_8((test_structured_fft_complex_boundary<float>(40)));

    // Aliasing beyond the same-object case, across the dispatch tiers.
    CALL_SUBTEST_8((test_structured_aliased_expression<double>(8)));
    CALL_SUBTEST_8((test_structured_aliased_expression<double>(24)));
    CALL_SUBTEST_8((test_structured_aliased_expression<double>(48)));
    CALL_SUBTEST_8((test_structured_aliased_expression<std::complex<double>>(40)));

    // Entrywise Inf/NaN propagation: FFT-sized operators must fall back to the
    // direct kernel; small ones are IEEE-exact already.
    CALL_SUBTEST_8((test_structured_nonfinite_product<double>(40)));
    CALL_SUBTEST_8((test_structured_nonfinite_product<double>(12)));
    CALL_SUBTEST_8((test_structured_nonfinite_product<std::complex<double>>(40)));

    // Look-ahead Levinson direct Toeplitz solver, one subtest per scalar type:
    // each type pulls in its own FullPivLU reference solves for vector and matrix
    // right-hand sides, which together made this the most memory-hungry
    // translation unit in the test suite.
    CALL_SUBTEST_5((test_levinson_wellcond<double>(1)));
    CALL_SUBTEST_5((test_levinson_wellcond<double>(2)));
    CALL_SUBTEST_5((test_levinson_wellcond<double>(20)));
    CALL_SUBTEST_5((test_levinson_wellcond<double>(60)));
    CALL_SUBTEST_11((test_levinson_wellcond<float>(40)));
    CALL_SUBTEST_12((test_levinson_wellcond<std::complex<double>>(30)));
    CALL_SUBTEST_12((test_levinson_solve_in_expression<std::complex<double>>(16)));
    CALL_SUBTEST_13(test_levinson_lookahead());
    CALL_SUBTEST_13(test_levinson_fixed());
    CALL_SUBTEST_13(test_levinson_singular());
    CALL_SUBTEST_13((test_levinson_solve_in_expression<double>(1)));
    CALL_SUBTEST_13((test_levinson_solve_in_expression<double>(24)));
    CALL_SUBTEST_14((test_levinson_wellcond<std::complex<float>>(24)));

    // Hankel: products across dispatch tiers, transposition family (validating the
    // phase-multiplication symbol reuse on rectangular FFT-tier operators), the
    // Toeplitz equivalence, direct solves, matrix-free least squares, fixed sizes.
    CALL_SUBTEST_9((test_hankel_product<double>(1, 1)));
    CALL_SUBTEST_9((test_hankel_product<double>(2, 3)));
    CALL_SUBTEST_9((test_hankel_product<double>(8, 8)));
    CALL_SUBTEST_9((test_hankel_product<double>(12, 7)));
    CALL_SUBTEST_9((test_hankel_product<double>(17, 16)));  // just above the scalar tier
    CALL_SUBTEST_9((test_hankel_product<double>(20, 24)));  // direct segment tier
    CALL_SUBTEST_9((test_hankel_product<double>(32, 33)));  // first FFT-tier size
    CALL_SUBTEST_9((test_hankel_product<double>(64, 40)));
    CALL_SUBTEST_9((test_hankel_product<double>(40, 64)));
    CALL_SUBTEST_9((test_hankel_product<double>(97, 50)));
    CALL_SUBTEST_9((test_hankel_product<double>(1, 40)));   // single row: direct O(n) path
    CALL_SUBTEST_9((test_hankel_product<double>(40, 1)));   // single column: direct O(n) path
    CALL_SUBTEST_9((test_hankel_product<double>(1, 400)));  // skinny far beyond the FFT threshold
    CALL_SUBTEST_9((test_hankel_product<double>(400, 1)));
    CALL_SUBTEST_9((test_hankel_product<float>(50, 50)));
    CALL_SUBTEST_9((test_hankel_product<std::complex<double>>(5, 7)));
    CALL_SUBTEST_9((test_hankel_product<std::complex<double>>(48, 64)));
    CALL_SUBTEST_9((test_hankel_product<std::complex<float>>(40, 40)));
    CALL_SUBTEST_9((test_hankel_transpose<double>(1, 1)));
    CALL_SUBTEST_9((test_hankel_transpose<double>(12, 7)));
    CALL_SUBTEST_9((test_hankel_transpose<double>(20, 24)));  // direct tier, empty symbol
    CALL_SUBTEST_9((test_hankel_transpose<double>(64, 40)));
    CALL_SUBTEST_9((test_hankel_transpose<double>(40, 64)));
    CALL_SUBTEST_9((test_hankel_transpose<double>(1, 40)));
    CALL_SUBTEST_9((test_hankel_transpose<std::complex<double>>(33, 20)));
    CALL_SUBTEST_9((test_hankel_transpose<std::complex<float>>(20, 36)));
    CALL_SUBTEST_9((test_hankel_symmetry<double>(48)));
    CALL_SUBTEST_9((test_hankel_to_toeplitz<double>(12, 7)));
    CALL_SUBTEST_9((test_hankel_to_toeplitz<std::complex<double>>(40, 56)));
    CALL_SUBTEST_9((test_hankel_solve<double>(1)));
    CALL_SUBTEST_9((test_hankel_solve<double>(20)));
    CALL_SUBTEST_9((test_hankel_solve<double>(45)));
    CALL_SUBTEST_9((test_hankel_solve<std::complex<double>>(24)));
    CALL_SUBTEST_9((test_hankel_solve<float>(16)));
    CALL_SUBTEST_9((test_hankel_solve_lookahead<double>(20)));
    CALL_SUBTEST_9((test_hankel_solve_lookahead<std::complex<double>>(15)));
    CALL_SUBTEST_9(test_hankel_hilbert());
    CALL_SUBTEST_9((test_hankel_least_squares<double>(60, 40)));
    CALL_SUBTEST_9((test_hankel_least_squares<std::complex<double>>(48, 32)));
    CALL_SUBTEST_9((test_hankel_fixed<double, 4, 6>()));
    CALL_SUBTEST_9((test_hankel_fixed<double, 40, 24>()));
    CALL_SUBTEST_9((test_hankel_fixed<std::complex<float>, 6, 4>()));
    CALL_SUBTEST_9((test_hankel_fixed_solve<double, 8>()));
    CALL_SUBTEST_9((test_hankel_fixed_solve<std::complex<double>, 12>()));

    // MR 2688 review regressions: finite-overflow scaling in the FFT tier,
    // delayed (value-nested) products, aliased right-hand sides across the
    // dispatch tiers, and mixed real/complex products.
    CALL_SUBTEST_10(test_hankel_fft_overflow());
    CALL_SUBTEST_10((test_hankel_fft_complex_boundary<double>(40)));
    CALL_SUBTEST_10((test_hankel_fft_complex_boundary<float>(40)));
    CALL_SUBTEST_10((test_hankel_delayed_product<double>(24, 12)));
    CALL_SUBTEST_10((test_hankel_delayed_product<double>(40, 64)));
    CALL_SUBTEST_10((test_hankel_delayed_product<std::complex<double>>(48, 33)));
    CALL_SUBTEST_10((test_hankel_aliased_product<double>(8)));   // scalar tier
    CALL_SUBTEST_10((test_hankel_aliased_product<double>(24)));  // direct segment tier
    CALL_SUBTEST_10((test_hankel_aliased_product<double>(64)));  // FFT tier
    CALL_SUBTEST_10((test_hankel_aliased_product<std::complex<double>>(40)));

    // Aliasing beyond the same-object case, across the dispatch tiers.
    CALL_SUBTEST_10((test_hankel_aliased_expression<double>(8)));
    CALL_SUBTEST_10((test_hankel_aliased_expression<double>(24)));
    CALL_SUBTEST_10((test_hankel_aliased_expression<double>(48)));
    CALL_SUBTEST_10((test_hankel_aliased_expression<std::complex<double>>(40)));

    // Entrywise Inf/NaN propagation: FFT-sized operators must fall back to the
    // direct kernels; small ones are IEEE-exact already.
    CALL_SUBTEST_10((test_hankel_nonfinite_product<double>(40)));
    CALL_SUBTEST_10((test_hankel_nonfinite_product<double>(12)));
    CALL_SUBTEST_10((test_hankel_nonfinite_product<std::complex<double>>(40)));

    CALL_SUBTEST_10((test_hankel_mixed_scalar<double>(8, 8)));    // scalar tier
    CALL_SUBTEST_10((test_hankel_mixed_scalar<double>(24, 20)));  // direct segment tier
    CALL_SUBTEST_10((test_hankel_mixed_scalar<double>(64, 40)));  // FFT tier
    CALL_SUBTEST_10((test_hankel_mixed_scalar<double>(1, 40)));   // skinny direct paths
    CALL_SUBTEST_10((test_hankel_mixed_scalar<double>(40, 1)));
    CALL_SUBTEST_10((test_hankel_mixed_scalar<float>(48, 64)));
  }
}
