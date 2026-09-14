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

// Reference dense Vandermonde built entry-wise from the nodes.
template <typename Scalar>
Matrix<Scalar, Dynamic, Dynamic> reference_vandermonde(const Matrix<Scalar, Dynamic, 1>& x, Index n) {
  const Index m = x.size();
  Matrix<Scalar, Dynamic, Dynamic> dense(m, n);
  for (Index i = 0; i < m; ++i) {
    Scalar p(1);
    for (Index j = 0; j < n; ++j) {
      dense(i, j) = p;
      p *= x[i];
    }
  }
  return dense;
}

template <typename Scalar>
void test_vandermonde_product(Index m, Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec x = Vec::Random(m);
  Vandermonde<Scalar> V(x, n);
  Mat dense = reference_vandermonde<Scalar>(x, n);

  Mat Vd = V;
  VERIFY_IS_APPROX(Vd, dense);
  for (Index t = 0; t < 5; ++t) {
    Index i = internal::random<Index>(0, m - 1), j = internal::random<Index>(0, n - 1);
    VERIFY_IS_APPROX(V.coeff(i, j), dense(i, j));
  }

  Vec a = Vec::Random(n);
  VERIFY_IS_APPROX((V * a).eval(), (dense * a).eval());

  Mat A = Mat::Random(n, 3);
  VERIFY_IS_APPROX((V * A).eval(), (dense * A).eval());

  // Accumulation form exercised by the iterative solvers.
  Vec y = Vec::Random(m);
  Vec y0 = y;
  y.noalias() += V * a;
  VERIFY_IS_APPROX(y, (y0 + dense * a).eval());
}

// Well-conditioned interpolation: Chebyshev-like nodes in [-1,1] keep the
// conditioning around (1+sqrt(2))^n, so for moderate n a modest tolerance wide
// of roundoff verifies the primal and dual solves against the exact solution.
template <typename Scalar>
void test_bjorck_pereyra(Index n) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec x(n);
  for (Index i = 0; i < n; ++i)
    x[i] = Scalar(RealScalar(std::cos(double(EIGEN_PI) * double(2 * i + 1) / double(2 * n))));  // Chebyshev nodes
  Vandermonde<Scalar> V(x);
  Mat dense = reference_vandermonde<Scalar>(x, n);

  BjorckPereyra<Scalar> bp(V);
  VERIFY(bp.info() == Success);

  // Forward-error bound for the (moderately conditioned) Chebyshev-node system.
  const RealScalar tol = RealScalar(5e7) * NumTraits<RealScalar>::epsilon();  // ~1e-8 in double

  // Primal: interpolate values of a known polynomial and recover its coefficients.
  Vec aTrue = Vec::Random(n);
  Vec f = dense * aTrue;
  Vec a = bp.solve(f);
  VERIFY((a - aTrue).norm() <= tol * aTrue.norm());

  // Dual (moment) system through the SolverBase transpose idiom.
  Vec wTrue = Vec::Random(n);
  Vec b = dense.transpose() * wTrue;
  Vec w = bp.transpose().solve(b);
  VERIFY((w - wTrue).norm() <= tol * wTrue.norm());

  // Adjoint solve.
  Vec c = dense.adjoint() * wTrue;
  Vec u = bp.adjoint().solve(c);
  VERIFY((u - wTrue).norm() <= tol * wTrue.norm());

  // Multiple right-hand sides.
  Mat F = Mat::Random(n, 3);
  Mat A = bp.solve(F);
  VERIFY_IS_APPROX((dense * A).eval(), F);
}

// The n-th roots of unity make V/sqrt(n) exactly unitary. Verify against the
// analytic inverse a = V^H f / n to catch catastrophic Newton-basis growth.
template <typename RealScalar>
void test_bjorck_pereyra_roots_of_unity(Index n) {
  typedef std::complex<RealScalar> Scalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;

  Vec x(n);
  for (Index i = 0; i < n; ++i)
    x[i] = std::polar(RealScalar(1), RealScalar(2 * EIGEN_PI) * RealScalar(i) / RealScalar(n));
  Vandermonde<Scalar> V(x);

  BjorckPereyra<Scalar> bp(V);
  VERIFY(bp.info() == Success);

  // The first cardinal polynomial has every coefficient equal to 1/n. This
  // deterministic case exposes the severe Newton-basis growth of cyclic node
  // order while remaining analytically checkable.
  Vec f = Vec::Zero(n);
  f[0] = Scalar(1);
  Vec a = bp.solve(f);
  Vec aRef = Vec::Constant(n, Scalar(RealScalar(1) / RealScalar(n)));
  // The recurrence performs O(n^2) complex operations; the fixed factor also
  // covers growth in its intermediate Newton coefficients.
  const RealScalar tol = RealScalar(128) * RealScalar(n) * RealScalar(n) * NumTraits<RealScalar>::epsilon();
  VERIFY((a - aRef).norm() <= tol * aRef.norm());
}

// Eigen::half has explicit conversion from the wider type returned by the
// standard scaling functions. Instantiate both product and solver paths so the
// range-protection helpers retain support for narrow floating-point scalars.
void test_vandermonde_half() {
  typedef Matrix<half, 2, 1> Vec;
  Vec nodes, coefficients;
  nodes << half(0), half(2);
  coefficients << half(3), half(2);

  Vandermonde<half, 2, 2> V(nodes);
  Vec values = V * coefficients;
  VERIFY_IS_EQUAL(float(values[0]), 3.0f);
  VERIFY_IS_EQUAL(float(values[1]), 7.0f);

  BjorckPereyra<half> bp(V);
  Vec recovered = bp.solve(values);
  VERIFY_IS_EQUAL(float(recovered[0]), 3.0f);
  VERIFY_IS_EQUAL(float(recovered[1]), 2.0f);
}

// Björck-Pereyra's celebrated accuracy property (Higham, ASNA ch. 22): for
// monotone nodes and an alternating-sign right-hand side the forward error is
// tiny even though the matrix conditioning is astronomical. Deterministic.
void test_bjorck_pereyra_higham() {
  typedef Matrix<double, Dynamic, 1> Vec;

  const Index n = 20;
  Vec x(n);
  for (Index i = 0; i < n; ++i) x[i] = double(i + 1) / 32.0;  // exactly represented, monotone in (0,1)
  Vandermonde<double> V(x);

  // f_i = (-1)^i: alternating signs.
  Vec f(n);
  for (Index i = 0; i < n; ++i) f[i] = (i % 2 == 0) ? 1.0 : -1.0;
  BjorckPereyra<double> bp(V);
  Vec a = bp.solve(f);

  // Independent long-double coefficient reference. For equally spaced nodes
  // with h=1/32, the Newton weights are Delta^k f_0 / (k! h^k) = (-64)^k/k!.
  // Accumulate weight_k * product_{j<k}(z-x_j) directly in the monomial basis.
  Matrix<long double, Dynamic, 1> aRef = Matrix<long double, Dynamic, 1>::Zero(n);
  Matrix<long double, Dynamic, 1> basis = Matrix<long double, Dynamic, 1>::Zero(n);
  basis[0] = 1.0L;
  long double weight = 1.0L;
  for (Index k = 0; k < n; ++k) {
    aRef += weight * basis;
    if (k + 1 == n) break;
    const long double node = static_cast<long double>(x[k]);
    basis[k + 1] = basis[k];
    for (Index j = k; j > 0; --j) basis[j] = basis[j - 1] - node * basis[j];
    basis[0] *= -node;
    weight *= -64.0L / static_cast<long double>(k + 1);
  }

  const Vec roundedRef = aRef.cast<double>();
  const double maxRelativeError = ((a - roundedRef).cwiseAbs().array() / roundedRef.cwiseAbs().array()).maxCoeff();
  const double coefficientTol = 8.0 * double(n) * double(n) * NumTraits<double>::epsilon();
  VERIFY(maxRelativeError <= coefficientTol);
}

// The transpose and adjoint recurrences must be distinguished for genuinely
// complex nodes. A rotated root-of-unity grid keeps both systems well
// conditioned while avoiding the unrotated grid's extra symmetries.
void test_bjorck_pereyra_complex_transpose_adjoint(Index n) {
  typedef std::complex<double> Scalar;
  typedef Matrix<double, Dynamic, 1> RealVec;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec x(n);
  for (Index i = 0; i < n; ++i) x[i] = std::polar(1.0, double(2 * EIGEN_PI) * (double(i) + 0.25) / double(n));
  Vandermonde<Scalar> V(x);
  Mat dense = reference_vandermonde<Scalar>(x, n);
  BjorckPereyra<Scalar> bp(V);

  Vec expected = Vec::Random(n);
  Vec transposed = bp.transpose().solve(dense.transpose() * expected);
  Vec adjoint = bp.adjoint().solve(dense.adjoint() * expected);
  const double tol = 256.0 * double(n) * double(n) * NumTraits<double>::epsilon();
  const double scale = numext::maxi(1.0, expected.norm());
  VERIFY((transposed - expected).norm() <= tol * scale);
  VERIFY((adjoint - expected).norm() <= tol * scale);

  // A real right-hand side is compatible with complex nodes and promotes the
  // solve result to complex. The rotated Fourier matrix has V^H V = n I, so
  // all three analytic inverse forms are available without another solver.
  RealVec realRhs = RealVec::Random(n);
  Vec primalMixed = bp.solve(realRhs);
  Vec transposedMixed = bp.transpose().solve(realRhs);
  Vec adjointMixed = bp.adjoint().solve(realRhs);
  Vec primalRef = dense.adjoint() * realRhs / Scalar(double(n));
  Vec transposedRef = dense.conjugate() * realRhs / Scalar(double(n));
  Vec adjointRef = dense * realRhs / Scalar(double(n));
  const double mixedScale = numext::maxi(1.0, realRhs.norm());
  VERIFY((primalMixed - primalRef).norm() <= tol * mixedScale);
  VERIFY((transposedMixed - transposedRef).norm() <= tol * mixedScale);
  VERIFY((adjointMixed - adjointRef).norm() <= tol * mixedScale);

  // The reverse promotion direction remains valid as well: real nodes with a
  // complex right-hand side produce complex work values and output.
  RealVec realNodes(2);
  realNodes << -1.0, 1.0;
  BjorckPereyra<double> realBp{Vandermonde<double>(realNodes)};
  Vec complexRhs(2), complexExpected(2);
  complexRhs << Scalar(1.0, 1.0), Scalar(3.0, -2.0);
  complexExpected << Scalar(2.0, -0.5), Scalar(1.0, -1.5);
  Vec realNodeMixed = realBp.solve(complexRhs);
  VERIFY((realNodeMixed - complexExpected).norm() <= 4.0 * NumTraits<double>::epsilon() * complexExpected.norm());

  // Materializing an aliased RHS must precede the internal Leja permutation.
  Vec aliased = Vec::Random(n);
  const Vec aliasInput = (aliased + Vec::Ones(n)).eval();
  const Vec aliasReference = bp.solve(aliasInput);
  aliased = bp.solve(aliased + Vec::Ones(n));
  VERIFY((aliased - aliasReference).norm() <= tol * numext::maxi(1.0, aliasReference.norm()));
}

// Repeated nodes make the matrix exactly singular, while non-finite nodes make
// the solver input invalid. Both must be reported before solve evaluation.
void test_bjorck_pereyra_singular() {
  typedef Matrix<double, Dynamic, 1> Vec;
  Vec x(5);
  x << 0.1, 0.7, 0.3, 0.7, 0.9;
  Vandermonde<double> V(x);
  BjorckPereyra<double> bp(V);
  VERIFY(bp.info() == NumericalIssue);

  x[3] = std::numeric_limits<double>::quiet_NaN();
  bp.compute(Vandermonde<double>(x));
  VERIFY(bp.info() == InvalidInput);
}

// Aliased products: the products carry the default product tag, so assignment
// materializes a temporary exactly like a dense product and x = V * x,
// x += V * x must come out as if the right-hand side had been copied first.
template <typename Scalar>
void test_vandermonde_aliased_product(Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec x = Vec::Random(n);
  Vandermonde<Scalar> V(x);  // square, so x = V * x type-checks
  Mat dense = reference_vandermonde<Scalar>(x, n);

  Vec b = Vec::Random(n);
  Vec y = b;
  y = V * y;
  VERIFY_IS_APPROX(y, (dense * b).eval());

  y = b;
  y += V * y;
  VERIFY_IS_APPROX(y, (b + dense * b).eval());

  y = b;
  y -= V * y;
  VERIFY_IS_APPROX(y, (b - dense * b).eval());

  Mat B = Mat::Random(n, 3);
  Mat Y = B;
  Y = V * Y;
  VERIFY_IS_APPROX(Y, (dense * B).eval());
}

// Aliasing beyond the same-object case: the default-product temporary must also
// resolve right-hand-side expressions that reference the destination, overlapping
// views of one buffer, and rectangular self-assignments where the destination is
// resized by the assignment.
template <typename Scalar>
void test_vandermonde_aliased_expression(Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec nodes = Vec::Random(n);
  Vandermonde<Scalar> V(nodes);  // square
  Mat dense = reference_vandermonde<Scalar>(nodes, n);

  // Right-hand-side expression referencing the destination.
  Vec x = Vec::Random(n), x0 = x;
  x = V * (x + Vec::Ones(n));
  VERIFY_IS_APPROX(x, (dense * (x0 + Vec::Ones(n))).eval());

  // Overlapping (shifted) segments of one buffer.
  Vec buf = Vec::Random(n + 1);
  Vec expected = dense * buf.tail(n);
  buf.head(n) = V * buf.tail(n);
  VERIFY_IS_APPROX(buf.head(n).eval(), expected);

  // Rectangular self-assignment (the reviewer's 5x4 case): x = V * x resizes
  // the destination from 4 to 5, so the product must be captured before the
  // destination storage is touched.
  Vec tallNodes = Vec::Random(5);
  Vandermonde<Scalar> Vt(tallNodes, 4);
  Mat denseT = reference_vandermonde<Scalar>(tallNodes, 4);
  Vec z = Vec::Random(4), z0 = z;
  z = Vt * z;
  VERIFY_IS_EQUAL(z.size(), 5);
  VERIFY_IS_APPROX(z, (denseT * z0).eval());
}

// A delayed-evaluated product expression must keep its structured factor alive:
// the makeVandermonde() factory returns an owning temporary, so Product has to
// nest the operator by value (no NestByRefBit) or `expr` dangles.
template <typename Scalar>
void test_vandermonde_delayed_product(Index m, Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  STATIC_CHECK(!std::is_reference<typename internal::ref_selector<Vandermonde<Scalar>>::type>::value);

  Vec x = Vec::Random(m), a = Vec::Random(n);
  Mat dense = reference_vandermonde<Scalar>(x, n);

  auto expr = makeVandermonde(x, n) * a;  // the factory temporary dies with the full expression
  Vec scribble = Vec::Random(2 * m);      // reuses the temporary's freed heap storage
  Vec y = expr;
  VERIFY_IS_APPROX(y, (dense * a).eval());
  VERIFY_IS_EQUAL(scribble.size(), 2 * m);  // keep the scribble alive across the evaluation
}

// Mixed-scalar products: a real operator applied to a complex right-hand side
// (and a complex operator applied to a real one) promotes to the complex
// product scalar, so alpha and the Horner accumulation must run in the promoted
// type rather than the operator scalar.
template <typename RealScalar>
void test_vandermonde_mixed_scalar(Index m, Index n) {
  typedef std::complex<RealScalar> Complex;
  typedef Matrix<RealScalar, Dynamic, 1> RVec;
  typedef Matrix<Complex, Dynamic, 1> CVec;
  typedef Matrix<Complex, Dynamic, Dynamic> CMat;

  RVec x = RVec::Random(m);
  Vandermonde<RealScalar> V(x, n);
  CMat dense = reference_vandermonde<RealScalar>(x, n).template cast<Complex>();

  CVec a = CVec::Random(n);
  CVec y = V * a;
  VERIFY_IS_APPROX(y, (dense * a).eval());

  CVec y0 = CVec::Random(m);
  y = y0;
  y.noalias() += V * a;
  VERIFY_IS_APPROX(y, (y0 + dense * a).eval());

  CVec xc = CVec::Random(m);
  Vandermonde<Complex> Vc(xc, n);
  CMat denseC = reference_vandermonde<Complex>(xc, n);
  RVec ar = RVec::Random(n);
  CVec z = Vc * ar;
  VERIFY_IS_APPROX(z, (denseC * ar).eval());
}

template <typename Scalar>
void test_vandermonde_determinant(Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec x = Vec::Random(n);
  Vandermonde<Scalar> V(x);
  Mat dense = reference_vandermonde<Scalar>(x, n);
  VERIFY_IS_APPROX(V.determinant(), dense.determinant());
}

// Reference for the wide-dynamic-range determinant tests: the same factor sequence with explicit
// exponent tracking, halving any factor whose node difference overflows (exact, since a difference
// only overflows for huge normal operands). Power-of-two rescaling is exact, so this reproduces the
// exact product up to one rounding per multiplication, representable where naive partial products
// leave the double range. Finite nodes only.
double reference_vandermonde_det(const Matrix<double, Dynamic, 1>& x) {
  double mantissa = 1.0;
  Index exponent = 0;
  for (Index j = 1; j < x.size(); ++j)
    for (Index i = 0; i < j; ++i) {
      double diff = x[j] - x[i];
      if (!(numext::isfinite)(diff)) {
        diff = 0.5 * x[j] - 0.5 * x[i];
        ++exponent;
      }
      int e;
      mantissa *= std::frexp(diff, &e);
      exponent += e;
      mantissa = std::frexp(mantissa, &e);
      exponent += e;
    }
  return std::ldexp(mantissa, static_cast<int>(exponent));
}

// Balanced determinant accumulation (reviewer repro on MR 2691): the running
// product is kept as mantissa * 2^e with exact frexp/ldexp renormalization, so
// partial products that leave the representable range cannot destroy a
// representable determinant, while zeros and genuine overflow still propagate.
void test_vandermonde_determinant_scaled() {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef std::complex<double> Complex;
  // The implementation accumulates the identical factor sequence as the
  // reference above, so the two sides differ only by the exactness of the
  // power-of-two renormalization.
  const double kBalancedTol = 16 * NumTraits<double>::epsilon();
  // The analytic values below additionally absorb the representation error of
  // the decimal node literals and of pow(10, 27.5), amplified by the 15-factor
  // product.
  const double kAnalyticTol = 100 * NumTraits<double>::epsilon();

  const double a = 1e-110, M = std::pow(10.0, 27.5);
  {
    // Underflow side (the reviewer's exact node set): the a-spaced factors alone
    // reach 2e-330, below the smallest subnormal, so the naive running product
    // flushes to an exact zero -- yet the determinant is 864 * a^3 * M^12 ~ 864.
    Vec x(6);
    x << 0.0, a, 2 * a, M, 2 * M, 3 * M;
    Vandermonde<double> V(x);
    const double det = V.determinant();
    const double ref = reference_vandermonde_det(x);
    VERIFY((numext::isfinite)(det));
    VERIFY(numext::abs(det / ref - 1.0) <= kBalancedTol);
    VERIFY(numext::abs(det / 864.0 - 1.0) <= kAnalyticTol);
  }
  {
    // Overflow-side analogue: with the large nodes first the naive running
    // product tops out at ~4.3e327 (infinity) three factors before the end, yet
    // the determinant is a representable -864 * B^12 * s^3 = -8.64e305.
    const double B = 1e28, s = 1e-11;
    Vec x(6);
    x << B, 2 * B, 3 * B, 0.0, s, 2 * s;
    Vandermonde<double> V(x);
    const double det = V.determinant();
    const double ref = reference_vandermonde_det(x);
    VERIFY((numext::isfinite)(det));
    VERIFY(numext::abs(det / ref - 1.0) <= kBalancedTol);
    VERIFY(numext::abs(det / -8.64e305 - 1.0) <= kAnalyticTol);
  }
  {
    // Genuinely overflowing determinant, ~3.5e424: must saturate to +infinity.
    const double T = 1e28;
    Vec x(6);
    x << 0.0, T, 2 * T, 3 * T, 4 * T, 5 * T;
    Vandermonde<double> V(x);
    const double det = V.determinant();
    VERIFY((numext::isinf)(det) && det > 0.0);
  }
  {
    // Genuinely underflowing determinant: subnormal-spaced nodes give 15 factors
    // below 2^-1071, a product near 2^-16000, which must saturate to zero with
    // the sign of the factored form: +0 for ascending nodes (all factors
    // positive), -0 for the descending permutation (15 negative factors).
    const double d = std::numeric_limits<double>::denorm_min();
    Vec x(6);
    x << 0.0, d, 2 * d, 3 * d, 4 * d, 5 * d;
    const double det = Vandermonde<double>(x).determinant();
    VERIFY(det == 0.0 && !std::signbit(det));
    const double detNeg = Vandermonde<double>(Vec(x.reverse())).determinant();
    VERIFY(detNeg == 0.0 && std::signbit(detNeg));
  }
  {
    // A repeated node makes the matrix exactly singular: the zero factor must
    // propagate to an exact 0 even though the naive running product overflows
    // beforehand (which would end in 0 * inf = NaN).
    Vec x(5);
    x << 0.0, 1e300, 1e-300, 1e300, 5.0;
    Vandermonde<double> V(x);
    VERIFY(V.determinant() == 0.0);
  }
  {
    // Complex path of the balanced accumulation: purely imaginary nodes i*x
    // turn every factor of the underflow-side case into i*(x_j - x_i), so
    // det = i^15 * 864 = -864i.
    Vec xr(6);
    xr << 0.0, a, 2 * a, M, 2 * M, 3 * M;
    Matrix<Complex, Dynamic, 1> x = Complex(0, 1) * xr.cast<Complex>();
    Vandermonde<Complex> V(x);
    const Complex det = V.determinant();
    const Complex ref(0.0, -reference_vandermonde_det(xr));
    VERIFY((numext::isfinite)(numext::abs(det)));
    VERIFY(numext::abs(det - ref) <= kBalancedTol * numext::abs(ref));
  }
}

// Reviewer reproducer on MR 2691: a node difference can overflow while the determinant stays
// representable. For nodes [-DBL_MAX, DBL_MAX, d, 2d, ..., 5d] with d the smallest subnormal,
// DBL_MAX - (-DBL_MAX) = 2^1025 overflows, yet det = -576 DBL_MAX^11 d^10 ~ -3.1633e160. The
// overflowing difference must be halved (exact for these huge normal operands) and the factor of
// two carried in the running exponent.
void test_vandermonde_determinant_overflowing_differences() {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef std::complex<double> Complex;
  const double M = (std::numeric_limits<double>::max)();
  const double d = std::numeric_limits<double>::denorm_min();
  const double kBalancedTol = 16 * NumTraits<double>::epsilon();
  const double kAnalyticTol = 100 * NumTraits<double>::epsilon();

  Vec x(7);
  x << -M, M, d, 2 * d, 3 * d, 4 * d, 5 * d;
  const double det = Vandermonde<double>(x).determinant();
  VERIFY((numext::isfinite)(det));

  // Scaled reference computation with the identical factor sequence.
  const double ref = reference_vandermonde_det(x);
  VERIFY(numext::abs(det / ref - 1.0) <= kBalancedTol);

  // Analytic factored form, accumulated with the same exponent bookkeeping:
  // every huge factor rounds to +-DBL_MAX (and the halved leading factor is
  // exactly DBL_MAX * 2), the d-spaced block contributes exactly 288 * 2^-10740,
  // so det = -2 * 288 * DBL_MAX^11 * 2^-10740.
  double mantissa = -576.0;
  Index exponent = -10740;
  for (int t = 0; t < 11; ++t) {
    int e;
    mantissa *= std::frexp(M, &e);
    exponent += e;
    mantissa = std::frexp(mantissa, &e);
    exponent += e;
  }
  const double expected = std::ldexp(mantissa, static_cast<int>(exponent));
  VERIFY(numext::abs(det / expected - 1.0) <= kAnalyticTol);
  VERIFY(numext::abs(det / -3.1633e160 - 1.0) <= 1e-3);  // the reviewer's quoted value

  // Complex nodes i*x: every factor becomes i*(x_j - x_i) -- the overflow now
  // sits in the imaginary components, which the component-wise finiteness check
  // must catch -- and det picks up i^21 = i.
  Matrix<Complex, Dynamic, 1> xc = Complex(0, 1) * x.cast<Complex>();
  const Complex detc = Vandermonde<Complex>(xc).determinant();
  const Complex refc = Complex(0, 1) * Complex(ref);
  VERIFY((numext::isfinite)(detc));
  VERIFY(numext::abs(detc - refc) <= kBalancedTol * numext::abs(refc));
}

// Reviewer reproducer on MR 2691: Horner intermediates can overflow while the value stays
// representable. At node 1/2 with coefficients [0, DBL_MAX, DBL_MAX] the intermediate 1.5 DBL_MAX
// is Inf, but the value is exactly fl(0.75 DBL_MAX). The scaled path must return it bit-exactly,
// the plain path must match the naive loop for moderate data, and genuinely unrepresentable values
// must still saturate.
void test_vandermonde_scaled_horner() {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef std::complex<double> Complex;
  typedef Matrix<Complex, Dynamic, 1> CVec;
  const double M = (std::numeric_limits<double>::max)();

  {
    // The reviewer's exact reproducer, in each accumulation form.
    Vec x(1);
    x << 0.5;
    Vandermonde<double> V(x, 3);
    Vec a(3);
    a << 0.0, M, M;
    Vec y = V * a;
    VERIFY_IS_EQUAL(y[0], 0.75 * M);
    y.setZero();
    y.noalias() += V * a;
    VERIFY_IS_EQUAL(y[0], 0.75 * M);
    // Two identical columns exercise the per-column screening.
    Matrix<double, Dynamic, Dynamic> A(3, 2);
    A.col(0) = a;
    A.col(1) = a;
    Matrix<double, Dynamic, Dynamic> Y = V * A;
    VERIFY_IS_EQUAL(Y(0, 0), 0.75 * M);
    VERIFY_IS_EQUAL(Y(0, 1), 0.75 * M);
  }
  {
    // Complex path: the same coefficients rotated by i keep the overflow in a
    // single component; the value is i * fl(0.75 * DBL_MAX).
    CVec xc(1);
    xc << Complex(0.5, 0.0);
    Vandermonde<Complex> Vc(xc, 3);
    CVec ac(3);
    ac << Complex(0), Complex(0, M), Complex(0, M);
    CVec yc = Vc * ac;
    VERIFY_IS_EQUAL(numext::real(yc[0]), 0.0);
    VERIFY_IS_EQUAL(numext::imag(yc[0]), 0.75 * M);
  }
  {
    // Genuine overflow must still saturate: value 3 * DBL_MAX at the node 2.
    Vec x(1);
    x << 2.0;
    Vandermonde<double> V(x, 2);
    Vec a(2);
    a << M, M;
    Vec y = V * a;
    VERIFY((numext::isinf)(y[0]) && y[0] > 0.0);
    a << M, -M;  // value -DBL_MAX: representable again, sign preserved
    y = V * a;
    VERIFY_IS_EQUAL(y[0], -M);
  }
  {
    // A vanished accumulator has no scale (reviewer repro): after the running
    // value becomes exactly zero -- a zero node annihilating it, or an exact
    // cancellation -- the frame exponent must reset, or the trailing small
    // coefficient underflows in the stale huge frame and the value 0 is
    // returned instead of the coefficient itself.
    const double mn = (std::numeric_limits<double>::min)();
    const double dm = std::numeric_limits<double>::denorm_min();
    Vec x(1);
    x << 0.0;  // zero node: the value is exactly a[0]
    Vandermonde<double> V0(x, 3);
    Vec a(3);
    a << mn, M, M;
    Vec y = V0 * a;
    VERIFY_IS_EQUAL(y[0], mn);

    x << 1.0;  // exact cancellation: M - M + dm = dm
    Vandermonde<double> V1(x, 3);
    a << dm, -M, M;
    y = V1 * a;
    VERIFY_IS_EQUAL(y[0], dm);
  }
  {
    // Complex variant: the real components cancel exactly while the tiny
    // imaginary coefficient must survive in a fresh frame.
    const double dm = std::numeric_limits<double>::denorm_min();
    CVec xc(1);
    xc << Complex(1.0, 0.0);
    Vandermonde<Complex> Vc(xc, 3);
    CVec ac(3);
    ac << Complex(0.0, dm), Complex(-M, 0.0), Complex(M, 0.0);
    CVec yc = Vc * ac;
    VERIFY_IS_EQUAL(numext::real(yc[0]), 0.0);
    VERIFY_IS_EQUAL(numext::imag(yc[0]), dm);
  }
  {
    // Near-cancellation: M - (M - 2^971) leaves the tiny nonzero intermediate
    // 2^971 (mantissa 2^-53 in the huge frame), which the frexp renormalization
    // must rebase; the trailing coefficient 1 then rounds away exactly as in
    // real arithmetic: fl(2^971 + 1) = 2^971.
    const double big = std::ldexp(1.0, 971);  // ulp(DBL_MAX), and M - big is exact
    Vec x(1);
    x << 1.0;
    Vandermonde<double> V(x, 3);
    Vec a(3);
    a << 1.0, -(M - big), M;
    Vec y = V * a;
    VERIFY_IS_EQUAL(y[0], big);
  }
  {
    // Moderate data keeps the plain path, so this matches a naive Horner loop to Horner's own forward
    // error bound (Higham, ASNA 2nd ed., section 5.1): |p(x) - fl(p(x))| <= gamma_{2n} sum_j |a_j| |x|^j
    // <= 2n eps sum_j |a_j| for |x| <= 1. Not bit-identical, since the compiler may contract
    // acc * x[i] + a[j] into an FMA in one of the two loops and not the other.
    const Index m = 7, n = 6;
    Vec x = Vec::Random(m), a = Vec::Random(n);
    Vandermonde<double> V(x, n);
    Vec y = V * a;
    const double bound = double(2 * n) * NumTraits<double>::epsilon() * a.cwiseAbs().sum();
    for (Index i = 0; i < m; ++i) {
      double acc = a[n - 1];
      for (Index j = n - 2; j >= 0; --j) acc = acc * x[i] + a[j];
      VERIFY(numext::abs(y[i] - acc) <= bound);
    }
  }
}

template <typename Scalar, int M, int N>
void test_vandermonde_fixed() {
  typedef Matrix<Scalar, M, 1> NodeVec;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, M, N> MatMN;

  NodeVec x = NodeVec::Random();
  Vandermonde<Scalar, M, N> V(x, N);
  STATIC_CHECK((Vandermonde<Scalar, M, N>::RowsAtCompileTime == M));
  STATIC_CHECK((Vandermonde<Scalar, M, N>::ColsAtCompileTime == N));
  STATIC_CHECK((internal::remove_all_t<decltype(makeVandermonde(x))>::ColsAtCompileTime == M));

  MatMN dense = V;
  VERIFY_IS_APPROX(dense, MatMN(reference_vandermonde<Scalar>(Vec(x), N)));

  Matrix<Scalar, N, 1> a = Matrix<Scalar, N, 1>::Random();
  Matrix<Scalar, M, 1> y = V * a;
  VERIFY_IS_APPROX(y, (dense * a).eval());
}

// Core rewrites alpha * (V * a) as (alpha * V) * a. Keep the two scaled
// expression forms together because they share the same structured wrapper.
void test_vandermonde_expression_regressions() {
  typedef Matrix<double, 1, 1> Vec1;
  typedef Matrix<double, 3, 1> Vec3;
  typedef Matrix<double, 1, 3> Row3;
  typedef Matrix<double, 3, 3> Mat3;
  typedef Matrix<double, 3, 2> Mat32;

  Vec1 x;
  x << 0.5;
  Vandermonde<double, 1, 3> V(x, 3);
  Row3 dense = V;
  Row3 scaledDense = 2.0 * V;
  VERIFY_IS_APPROX(scaledDense, (2.0 * dense).eval());

  Vec3 a;
  a << 1.0, 2.0, 3.0;
  Vec1 scaledProduct = 0.5 * (V * a);
  VERIFY_IS_APPROX(scaledProduct, (0.5 * (dense * a)).eval());

  // A product on the right must be evaluated before the structured Horner
  // evaluator asks it for coefficients.
  Mat3 B = Mat3::Random();
  Mat32 C = Mat32::Random();
  VERIFY_IS_APPROX((V * (B * C)).eval(), (dense * (B * C)).eval());
}

void test_vandermonde_dimension_checks() {
  VectorXd x(2);
  x << 0.0, 1.0;
  VERIFY_RAISES_ASSERT((Vandermonde<double, Dynamic, 3>(x)));

  Vandermonde<double> rectangular(x, 3);
  VERIFY_RAISES_ASSERT((BjorckPereyra<double>(rectangular)));
}

// Fixed-size Vandermonde expressions own the operator by value so temporaries
// returned from factories remain alive. Their node storage must therefore avoid
// over-alignment: generic Product and cwise wrapper types do not supply aligned
// operator new under C++14.
void test_vandermonde_fixed_heap_expressions() {
  typedef Matrix<double, 4, 1> Vec4;
  typedef Matrix<double, 4, 4> Mat4;

  Vec4 x;
  x << -1.0, -0.25, 0.5, 1.0;
  Vec4 a;
  a << 1.0, 2.0, 3.0, 4.0;

  typedef decltype(makeVandermonde(x) * a) ProductExpression;
  typedef decltype(2.0 * makeVandermonde(x)) ScaledExpression;
  STATIC_CHECK(alignof(ProductExpression) <= alignof(std::max_align_t));
  STATIC_CHECK(alignof(ScaledExpression) <= alignof(std::max_align_t));

  ProductExpression* delayedProduct = new ProductExpression(makeVandermonde(x) * a);
  ScaledExpression* delayedScaled = new ScaledExpression(2.0 * makeVandermonde(x));

  const Mat4 dense = reference_vandermonde<double>(Matrix<double, Dynamic, 1>(x), 4);
  Vec4 y = *delayedProduct;
  Mat4 scaled = *delayedScaled;
  VERIFY_IS_APPROX(y, (dense * a).eval());
  VERIFY_IS_APPROX(scaled, (2.0 * dense).eval());
  delete delayedProduct;
  delete delayedScaled;
}

EIGEN_DECLARE_TEST(structured_vandermonde) {
  for (int i = 0; i < g_repeat; ++i) {
    // Horner products, dense assignment, coefficient access.
    CALL_SUBTEST_1((test_vandermonde_product<double>(1, 1)));
    CALL_SUBTEST_1((test_vandermonde_product<double>(8, 8)));
    CALL_SUBTEST_1((test_vandermonde_product<double>(20, 12)));  // tall
    CALL_SUBTEST_1((test_vandermonde_product<double>(12, 20)));  // wide
    CALL_SUBTEST_1((test_vandermonde_product<float>(10, 10)));
    CALL_SUBTEST_1((test_vandermonde_product<std::complex<double>>(9, 7)));
    CALL_SUBTEST_1((test_vandermonde_product<std::complex<float>>(7, 9)));

    // Björck-Pereyra primal/dual/adjoint solves.
    CALL_SUBTEST_2((test_bjorck_pereyra<double>(1)));
    CALL_SUBTEST_2((test_bjorck_pereyra<double>(2)));
    CALL_SUBTEST_2((test_bjorck_pereyra<double>(10)));
    CALL_SUBTEST_2((test_bjorck_pereyra<double>(14)));
    CALL_SUBTEST_2((test_bjorck_pereyra<std::complex<double>>(10)));
    CALL_SUBTEST_2((test_bjorck_pereyra_roots_of_unity<double>(64)));
    CALL_SUBTEST_2((test_bjorck_pereyra_roots_of_unity<float>(32)));
    CALL_SUBTEST_2(test_vandermonde_half());
    CALL_SUBTEST_2(test_bjorck_pereyra_higham());
    CALL_SUBTEST_2(test_bjorck_pereyra_complex_transpose_adjoint(12));
    CALL_SUBTEST_2(test_bjorck_pereyra_singular());

    // Closed-form determinant and fixed sizes.
    CALL_SUBTEST_3((test_vandermonde_determinant<double>(8)));
    CALL_SUBTEST_3((test_vandermonde_determinant<std::complex<double>>(7)));
    CALL_SUBTEST_3(test_vandermonde_determinant_scaled());
    CALL_SUBTEST_3(test_vandermonde_determinant_overflowing_differences());
    CALL_SUBTEST_3(test_vandermonde_scaled_horner());
    CALL_SUBTEST_3((test_vandermonde_fixed<double, 6, 4>()));
    CALL_SUBTEST_3((test_vandermonde_fixed<double, 5, 5>()));
    CALL_SUBTEST_3((test_vandermonde_fixed<std::complex<float>, 4, 4>()));
    CALL_SUBTEST_3(test_vandermonde_expression_regressions());
    CALL_SUBTEST_3(test_vandermonde_fixed_heap_expressions());

    // Product regressions: aliasing, operand lifetime, mixed scalars.
    CALL_SUBTEST_4((test_vandermonde_aliased_product<double>(8)));
    CALL_SUBTEST_4((test_vandermonde_aliased_product<double>(17)));
    CALL_SUBTEST_4((test_vandermonde_aliased_product<std::complex<double>>(9)));
    CALL_SUBTEST_4((test_vandermonde_aliased_expression<double>(8)));
    CALL_SUBTEST_4((test_vandermonde_aliased_expression<std::complex<double>>(9)));
    CALL_SUBTEST_4((test_vandermonde_delayed_product<double>(12, 9)));
    CALL_SUBTEST_4((test_vandermonde_delayed_product<std::complex<double>>(8, 8)));
    CALL_SUBTEST_4((test_vandermonde_mixed_scalar<double>(10, 8)));
    CALL_SUBTEST_4((test_vandermonde_mixed_scalar<float>(7, 9)));
  }

  CALL_SUBTEST_3(test_vandermonde_dimension_checks());
}
