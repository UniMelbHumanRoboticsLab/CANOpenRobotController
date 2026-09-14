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

// Reference dense Cauchy built entry-wise from the node vectors.
template <typename Scalar>
Matrix<Scalar, Dynamic, Dynamic> reference_cauchy(const Matrix<Scalar, Dynamic, 1>& x,
                                                  const Matrix<Scalar, Dynamic, 1>& y) {
  Matrix<Scalar, Dynamic, Dynamic> dense(x.size(), y.size());
  for (Index j = 0; j < y.size(); ++j)
    for (Index i = 0; i < x.size(); ++i) dense(i, j) = Scalar(1) / (x[i] - y[j]);
  return dense;
}

// Reference dense Cauchy built entry-wise through the guarded reciprocal, the
// single helper every coefficient evaluation must agree with at the overflow
// boundary.
template <typename Scalar>
Matrix<Scalar, Dynamic, Dynamic> reference_cauchy_guarded(const Matrix<Scalar, Dynamic, 1>& x,
                                                          const Matrix<Scalar, Dynamic, 1>& y) {
  Matrix<Scalar, Dynamic, Dynamic> dense(x.size(), y.size());
  for (Index j = 0; j < y.size(); ++j)
    for (Index i = 0; i < x.size(); ++i) dense(i, j) = internal::cauchy_reciprocal_diff(x[i], y[j]);
  return dense;
}

// Separated node sets: x in [2,3], y in [0,1], so all denominators are in [1,3].
template <typename Scalar>
void separated_nodes(Index m, Index n, Matrix<Scalar, Dynamic, 1>& x, Matrix<Scalar, Dynamic, 1>& y) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  x = Matrix<Scalar, Dynamic, 1>::Random(m);
  y = Matrix<Scalar, Dynamic, 1>::Random(n);
  x = (x * Scalar(RealScalar(0.5))).array() + Scalar(RealScalar(2.5));
  y = (y * Scalar(RealScalar(0.5))).array() + Scalar(RealScalar(0.5));
}

template <typename Scalar>
void test_cauchy_product(Index m, Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec x, y;
  separated_nodes<Scalar>(m, n, x, y);
  Cauchy<Scalar> C(x, y);
  Mat dense = reference_cauchy<Scalar>(x, y);

  Mat Cd = C;
  VERIFY_IS_APPROX(Cd, dense);
  Mat accumulated = Mat::Random(m, n);
  const Mat initial = accumulated;
  accumulated += C;
  VERIFY_IS_APPROX(accumulated, initial + dense);
  accumulated -= C;
  VERIFY_IS_APPROX(accumulated, initial);
  for (Index t = 0; t < 5; ++t) {
    Index i = internal::random<Index>(0, m - 1), j = internal::random<Index>(0, n - 1);
    VERIFY_IS_APPROX(C.coeff(i, j), dense(i, j));
  }

  Vec v = Vec::Random(n);
  VERIFY_IS_APPROX((C * v).eval(), (dense * v).eval());

  Mat V = Mat::Random(n, 3);
  VERIFY_IS_APPROX((C * V).eval(), (dense * V).eval());

  // Accumulation form exercised by the iterative solvers.
  Vec w = Vec::Random(m);
  Vec w0 = w;
  w.noalias() += C * v;
  VERIFY_IS_APPROX(w, (w0 + dense * v).eval());
}

template <typename Scalar>
void test_cauchy_transpose(Index m, Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec x, y;
  separated_nodes<Scalar>(m, n, x, y);
  Cauchy<Scalar> C(x, y);
  Mat dense = reference_cauchy<Scalar>(x, y);

  Mat Td = C.transpose();
  VERIFY_IS_APPROX(Td, Mat(dense.transpose()));
  Mat Ad = C.adjoint();
  VERIFY_IS_APPROX(Ad, Mat(dense.adjoint()));
  Mat Kd = C.conjugate();
  VERIFY_IS_APPROX(Kd, Mat(dense.conjugate()));

  Vec w = Vec::Random(m);
  VERIFY_IS_APPROX((C.transpose() * w).eval(), (dense.transpose() * w).eval());
  VERIFY_IS_APPROX((C.adjoint() * w).eval(), (dense.adjoint() * w).eval());
}

// GKO solve on separated random nodes, verified through residuals (Cauchy
// matrices are exponentially ill-conditioned, so forward-error comparisons
// between different algorithms would not be meaningful).
template <typename Scalar>
void test_cauchy_lu(Index n) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec x, y;
  separated_nodes<Scalar>(n, n, x, y);
  Cauchy<Scalar> C(x, y);
  Mat dense = reference_cauchy<Scalar>(x, y);

  CauchyLU<Scalar> lu(C);
  VERIFY(lu.info() == Success);

  // Backward-stability-style residual bound, scaled by the scalar's epsilon (the
  // separated node sets bound the GKO generators by 1, so no growth term).
  const RealScalar tol = RealScalar(1000) * RealScalar(n) * NumTraits<RealScalar>::epsilon();

  Vec b = Vec::Random(n);
  Vec u = lu.solve(b);
  VERIFY((dense * u - b).norm() <= tol * (dense.norm() * u.norm() + b.norm()));

  // Multiple right-hand sides.
  Mat B = Mat::Random(n, 3);
  Mat U = lu.solve(B);
  VERIFY((dense * U - B).norm() <= tol * (dense.norm() * U.norm() + B.norm()));

  // Transposed and adjoint systems reuse the same factorization.
  Vec vt = lu.transpose().solve(b);
  VERIFY((dense.transpose() * vt - b).norm() <= tol * (dense.norm() * vt.norm() + b.norm()));
  Vec va = lu.adjoint().solve(b);
  VERIFY((dense.adjoint() * va - b).norm() <= tol * (dense.norm() * va.norm() + b.norm()));
}

// The Hilbert matrix as a Cauchy matrix (x_i = i+1, y_j = -j): deterministic
// residual bounds far below what the astronomical conditioning would allow a
// forward-error test, plus the closed-form determinant against the dense LU one.
void test_cauchy_hilbert() {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef Matrix<double, Dynamic, Dynamic> Mat;

  auto residual = [](Index n) {
    Vec x(n), y(n);
    for (Index i = 0; i < n; ++i) {
      x[i] = double(i + 1);
      y[i] = -double(i);
    }
    Cauchy<double> C(x, y);
    Mat dense = reference_cauchy<double>(x, y);
    Vec b = dense * Vec::Ones(n);
    CauchyLU<double> lu(C);
    VERIFY(lu.info() == Success);
    Vec u = lu.solve(b);
    return (dense * u - b).norm() / b.norm();
  };
  // GKO partial pivoting is backward stable, so the residual stays a small
  // multiple of epsilon even as the Hilbert conditioning explodes with n.
  const double eps = NumTraits<double>::epsilon();
  VERIFY(residual(8) <= 5e5 * eps);   // ~1e-10
  VERIFY(residual(12) <= 5e7 * eps);  // ~1e-8

  Vec x(5), y(5);
  for (Index i = 0; i < 5; ++i) {
    x[i] = double(i + 1);
    y[i] = -double(i);
  }
  Cauchy<double> H5(x, y);
  Mat dense = reference_cauchy<double>(x, y);
  VERIFY_IS_APPROX(H5.determinant(), dense.determinant());
}

// Clustered row nodes make leading minors nearly singular: partial pivoting must
// keep the factorization backward stable (residual check).
void test_cauchy_lu_pivoting(Index n) {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef Matrix<double, Dynamic, Dynamic> Mat;

  Vec x(n), y(n);
  for (Index i = 0; i < n; ++i) {
    x[i] = 2.0 + 1e-13 * double(i * i);  // tight cluster
    y[i] = double(i) / double(n);        // spread out
  }
  Cauchy<double> C(x, y);
  Mat dense = reference_cauchy<double>(x, y);
  CauchyLU<double> lu(C);
  VERIFY(lu.info() == Success);
  Vec b = Vec::Random(n);
  Vec u = lu.solve(b);
  const double tol = 5e7 * NumTraits<double>::epsilon();  // ~1e-8
  VERIFY((dense * u - b).norm() <= tol * (dense.norm() * u.norm() + b.norm()));
}

// A duplicated row node makes two rows identical, hence the matrix exactly
// singular; a zero pivot must survive partial pivoting and be reported.
void test_cauchy_lu_singular() {
  typedef Matrix<double, Dynamic, 1> Vec;
  Vec x(5), y(5);
  x << 2.0, 2.5, 2.0, 2.75, 2.25;  // x[2] duplicates x[0]
  y << 0.1, 0.3, 0.5, 0.7, 0.9;
  Cauchy<double> C(x, y);
  CauchyLU<double> lu(C);
  VERIFY(lu.info() == NumericalIssue);
}

// Generator mantissas can overflow even when every matrix entry and LU factor
// is finite. Exponent-tracked generators must preserve the cancellation against
// the node reciprocal, and still expose an exact duplicate-row singularity.
template <typename Scalar>
void test_cauchy_lu_scaled_generators() {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec x(2), y(2), b(2);
  const RealScalar P = std::ldexp(RealScalar(1), 1000);
  const RealScalar t = std::ldexp(RealScalar(1), -1000);
  x << Scalar(0), Scalar(-P);
  y << Scalar(P), Scalar(-t);
  b << Scalar(1), Scalar(0.25);
  Cauchy<Scalar> C(x, y);
  Mat dense = C;
  VERIFY(dense.allFinite());
  CauchyLU<Scalar> lu(C);
  VERIFY(lu.info() == Success);
  Vec u = lu.solve(b);
  VERIFY(u.allFinite());
  const RealScalar tol = RealScalar(64) * NumTraits<RealScalar>::epsilon();
  VERIFY((dense * u - b).cwiseAbs().maxCoeff() <= tol * b.cwiseAbs().maxCoeff());

  Vec xs(2);
  xs << Scalar(0), Scalar(0);
  Cauchy<Scalar> Cs(xs, y);
  Mat denseSingular = Cs;
  VERIFY(denseSingular.allFinite());
  CauchyLU<Scalar> singularLu(Cs);
  VERIFY(singularLu.info() == NumericalIssue);

  Vec xi(1), yi(1);
  xi << Scalar(0);
  yi << Scalar(-std::numeric_limits<RealScalar>::denorm_min());
  Cauchy<Scalar> Ci(xi, yi);
  CauchyLU<Scalar> nonfiniteLu(Ci);
  VERIFY(nonfiniteLu.info() == NumericalIssue);
}

// y = -x gives the symmetric generalized Hilbert matrix 1/(x_i + x_j): verify
// the symmetry closure of the transpose through the node identity C^T = C(-y,-x).
template <typename Scalar>
void test_cauchy_symmetric(Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec x(n);
  for (Index i = 0; i < n; ++i) x[i] = Scalar(1) + Scalar(i);
  Vec negx = -x;
  Cauchy<Scalar> C(x, negx);
  Mat dense = reference_cauchy<Scalar>(x, negx);
  VERIFY_IS_APPROX(dense, Mat(dense.transpose()));
  Cauchy<Scalar> Ct = C.transpose();
  VERIFY_IS_EQUAL(Ct.rowNodes(), Vec(x));  // -(-x) round-trips exactly
  Mat Ctd = Ct;
  VERIFY_IS_APPROX(Ctd, dense);
}

// The closed form is a product of exact node differences (relative error
// O(n^2 eps)); the dense LU determinant it is compared against carries a
// cond(C)*eps error, so keep n small and the tolerance loose: the reference is
// the less accurate side of this comparison.
template <typename Scalar>
void test_cauchy_determinant(Index n) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec x, y;
  separated_nodes<Scalar>(n, n, x, y);
  Cauchy<Scalar> C(x, y);
  Mat dense = reference_cauchy<Scalar>(x, y);
  // The closed form is accurate to O(n^2 eps); the dense LU reference is the less
  // accurate side, its determinant carrying a relative error that grows like
  // cond(C)*eps (empirically ~0.1*cond*eps). Scale the bound by the SVD condition
  // number so the test is robust across random node draws.
  JacobiSVD<Mat> svd(dense);
  const RealScalar cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
  const RealScalar tol = RealScalar(100) * cond * NumTraits<RealScalar>::epsilon();
  VERIFY(numext::abs(C.determinant() - dense.determinant()) <= tol * numext::abs(dense.determinant()));
}

// The products carry the default product tag, so assignment materializes a
// temporary exactly like a dense product: x = C * x and x += C * x must see the
// pre-assignment right-hand side. Without the temporary, x = C * x would read a
// zeroed right-hand side and x += C * x would interleave destination writes
// with right-hand-side reads.
template <typename Scalar>
void test_cauchy_aliased_product(Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec x, y;
  separated_nodes<Scalar>(n, n, x, y);
  Cauchy<Scalar> C(x, y);
  Mat dense = reference_cauchy<Scalar>(x, y);

  Vec v = Vec::Random(n);
  Vec w = v;
  w = C * w;
  VERIFY_IS_APPROX(w, (dense * v).eval());

  w = v;
  w += C * w;
  VERIFY_IS_APPROX(w, (v + dense * v).eval());

  w = v;
  w -= C * w;
  VERIFY_IS_APPROX(w, (v - dense * v).eval());

  Mat V = Mat::Random(n, 3);
  Mat W = V;
  W = C * W;
  VERIFY_IS_APPROX(W, (dense * V).eval());
}

// Aliasing beyond the same-object case: the default-product temporary must also
// resolve right-hand-side expressions that reference the destination,
// overlapping views of one buffer, and rectangular self-assignment where the
// destination is resized by the assignment.
template <typename Scalar>
void test_cauchy_aliased_expression(Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec xn, yn;
  separated_nodes<Scalar>(n, n, xn, yn);
  Cauchy<Scalar> C(xn, yn);
  Mat dense = reference_cauchy<Scalar>(xn, yn);

  // Right-hand-side expression referencing the destination.
  Vec v = Vec::Random(n), v0 = v;
  v = C * (v + Vec::Ones(n));
  VERIFY_IS_APPROX(v, (dense * (v0 + Vec::Ones(n))).eval());

  // Overlapping (shifted) segments of one buffer.
  Vec buf = Vec::Random(n + 1);
  Vec expected = dense * buf.tail(n);
  buf.head(n) = C * buf.tail(n);
  VERIFY_IS_APPROX(buf.head(n).eval(), expected);

  // Rectangular self-assignment: z = R * z resizes the destination from n to m,
  // so the product must be captured before the destination storage is touched.
  const Index m = n + 3;
  Vec xr, yr;
  separated_nodes<Scalar>(m, n, xr, yr);
  Cauchy<Scalar> R(xr, yr);
  Mat denseR = reference_cauchy<Scalar>(xr, yr);
  Vec z = Vec::Random(n), z0 = z;
  z = R * z;
  VERIFY_IS_EQUAL(z.size(), m);
  VERIFY_IS_APPROX(z, (denseR * z0).eval());
}

// transpose()/conjugate()/adjoint() return owning temporaries, so the product
// expression must nest the structured operand by value: a delayed-evaluated
// expression has to outlive the temporary operator it was built from. The static
// check pins the value nesting; the behavioral check would read freed memory if
// the product held a reference instead.
template <typename Scalar>
void test_cauchy_delayed_product(Index m, Index n) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  STATIC_CHECK(!std::is_reference<typename internal::ref_selector<Cauchy<Scalar>>::type>::value);

  Vec x, y;
  separated_nodes<Scalar>(m, n, x, y);
  Cauchy<Scalar> C(x, y);
  Mat dense = reference_cauchy<Scalar>(x, y);

  Vec w = Vec::Random(m);
  auto expr = C.adjoint() * w;        // the adjoint temporary dies with the full expression
  Vec scribble = Vec::Random(m + n);  // reuses the temporary's freed heap storage
  Vec u = expr;
  VERIFY_IS_APPROX(u, (dense.adjoint() * w).eval());
  VERIFY_IS_EQUAL(scribble.size(), m + n);  // keep the scribble alive across the evaluation
}

// Mixed-scalar products: a real operator applied to a complex right-hand side
// (and a complex operator applied to a real one) promotes to the complex product
// scalar, so alpha and the accumulation must run in the promoted type rather than
// the operator scalar.
template <typename RealScalar>
void test_cauchy_mixed_scalar(Index m, Index n) {
  typedef std::complex<RealScalar> Complex;
  typedef Matrix<RealScalar, Dynamic, 1> RVec;
  typedef Matrix<Complex, Dynamic, 1> CVec;
  typedef Matrix<Complex, Dynamic, Dynamic> CMat;

  RVec x, y;
  separated_nodes<RealScalar>(m, n, x, y);
  Cauchy<RealScalar> C(x, y);
  CMat dense = reference_cauchy<RealScalar>(x, y).template cast<Complex>();

  CVec v = CVec::Random(n);
  CVec w = C * v;
  VERIFY_IS_APPROX(w, (dense * v).eval());

  CVec w0 = CVec::Random(m);
  w = w0;
  w.noalias() += C * v;
  VERIFY_IS_APPROX(w, (w0 + dense * v).eval());

  CVec xc, yc;
  separated_nodes<Complex>(m, n, xc, yc);
  Cauchy<Complex> Cc(xc, yc);
  CMat denseC = reference_cauchy<Complex>(xc, yc);
  RVec vr = RVec::Random(n);
  CVec z = Cc * vr;
  VERIFY_IS_APPROX(z, (denseC * vr).eval());
}

// Wide-dynamic-range determinants: the closed-form factors overflow or underflow
// individually while the determinant itself is representable, so the balanced
// m * 2^e accumulation must carry the exponent past the intermediate extremes.
void test_cauchy_determinant_range() {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef Matrix<double, Dynamic, Dynamic> Mat;

  // Reviewer repro: x = [2s, 3s], y = [0, s] with s = 1e160. The determinant is
  // -1/(12 s^2) ~ -8.35e-322, a subnormal, while the naive numerator product
  // -s^2 = -1e320 already overflows. Both the closed form and the dense LU
  // reference round once into the subnormal range (spacing denorm_min), so the
  // comparison needs an absolute term of a few subnormal spacings on top of a
  // relative term; the relative term alone would be far below denorm_min.
  {
    const double s = 1e160;
    Vec x(2), y(2);
    x << 2 * s, 3 * s;
    y << 0.0, s;
    Cauchy<double> C(x, y);
    Mat dense = reference_cauchy<double>(x, y);
    const double det = C.determinant();
    const double ref = dense.partialPivLu().determinant();
    const double kRelTol = 16 * NumTraits<double>::epsilon();
    const double kAbsTol = 4 * std::numeric_limits<double>::denorm_min();
    VERIFY((numext::isfinite)(det));
    VERIFY(numext::abs(det - ref) <= kRelTol * numext::abs(ref) + kAbsTol);
  }

  // Overflow-side analogue: x = [2s, 3s, 4s], y = [0, s/2, s] with s = 1e-60.
  // The determinant is -1/(3780 s^3) ~ -2.65e176, huge but representable, while
  // the naive numerator product -s^6/2 = -5e-361 underflows to zero. The dense
  // LU reference is the less accurate side (its error carries the conditioning
  // of the matrix, measured ~64 eps here); the deterministic nodes make 1000 eps
  // comfortable headroom.
  {
    const double s = 1e-60;
    Vec x(3), y(3);
    x << 2 * s, 3 * s, 4 * s;
    y << 0.0, s / 2, s;
    Cauchy<double> C(x, y);
    Mat dense = reference_cauchy<double>(x, y);
    const double det = C.determinant();
    const double ref = dense.partialPivLu().determinant();
    const double kRelTol = 1000 * NumTraits<double>::epsilon();
    VERIFY((numext::isfinite)(det));
    VERIFY(numext::abs(det - ref) <= kRelTol * numext::abs(ref));
  }

  // Genuinely infinite determinant: x[0] == y[0] makes entry (0,0) infinite (a
  // zero denominator factor with a non-zero numerator); the signed limit as
  // x[0] -> y[0] from above is -inf, and division by the exact zero factor must
  // produce it rather than a balanced-away finite value.
  {
    Vec x(2), y(2);
    x << 1.0, 2.0;
    y << 1.0, 3.0;
    Cauchy<double> C(x, y);
    const double det = C.determinant();
    VERIFY((numext::isinf)(det));
    VERIFY(det < 0.0);
  }

  // Coincident row nodes: two identical rows, so the matrix is exactly singular
  // and the zero numerator factor must propagate to an exact zero.
  {
    Vec x(3), y(3);
    x << 2.0, 3.0, 2.0;  // x[2] duplicates x[0]
    y << 0.0, 0.5, 1.0;
    Cauchy<double> C(x, y);
    VERIFY_IS_EQUAL(C.determinant(), 0.0);
  }
}

// Node differences at the overflow boundary: forming x_j - x_i or x_i - y_j can
// overflow to Inf even though every matrix entry is finite and unexceptional.
// Such factors must enter the balanced accumulation through the exact
// halved-operand recomputation, so only the determinant's own overflow or
// underflow is visible in the result -- saturated to a zero or infinity of the
// mathematically correct sign.
void test_cauchy_determinant_overflow_boundary() {
  typedef Matrix<double, Dynamic, 1> Vec;
  const double M = 0.6 * (std::numeric_limits<double>::max)();

  // Reviewer reproducer: every coefficient 1/(x_i - y_j) is finite and the exact
  // determinant (2M)(-0.8M) / (0.7056 M^4) underflows, but the numerator
  // difference x_1 - x_0 = 1.2 * DBL_MAX overflows if formed naively (the old
  // code returned -Inf). The result must be a zero of the correct sign: one
  // negative numerator factor against two negative denominator factors.
  {
    Vec x(2), y(2);
    x << -M, M;
    y << -0.4 * M, 0.4 * M;
    const double det = Cauchy<double>(x, y).determinant();
    VERIFY(det == 0.0 && std::signbit(det));
  }

  // Sign flip of the same configuration: swapping the y nodes negates the
  // determinant, so the underflow must land on +0.
  {
    Vec x(2), y(2);
    x << -M, M;
    y << 0.4 * M, -0.4 * M;
    const double det = Cauchy<double>(x, y).determinant();
    VERIFY(det == 0.0 && !std::signbit(det));
  }

  // Purely imaginary nodes of the same magnitudes: the halved-operand
  // recomputation applies componentwise to complex nodes. Scaling every node by
  // i multiplies the 2x2 determinant by i^2 / i^4 = -1, so this underflow lands
  // on a real part of +0 (the value is real: the four denominator divisions
  // rotate the accumulation back onto the real axis).
  {
    typedef std::complex<double> Cplx;
    Matrix<Cplx, Dynamic, 1> x(2), y(2);
    x << Cplx(0.0, -M), Cplx(0.0, M);
    y << Cplx(0.0, -0.4 * M), Cplx(0.0, 0.4 * M);
    const Cplx det = Cauchy<Cplx>(x, y).determinant();
    VERIFY(numext::real(det) == 0.0 && numext::imag(det) == 0.0);
    VERIFY(!std::signbit(numext::real(det)));
  }

  // A representable determinant whose evaluation crosses the boundary, guard on
  // a denominator factor (accumulated exponent decremented): with P = 2^1023,
  // x = [-P, 0], y = [P, c], the difference x_0 - y_0 = -2^1024 overflows, yet
  // det = (P - c) / (2 P c (P + c)), which for c = 2^-60 is 2^-964 up to a
  // relative correction c/P ~ 2^-1083, far below roundoff. Every rounded factor
  // is a power of two, so the balanced accumulation is exact here.
  {
    const double P = std::ldexp(1.0, 1023);
    const double c = std::ldexp(1.0, -60);
    Vec x(2), y(2);
    x << -P, 0.0;
    y << P, c;
    const double det = Cauchy<double>(x, y).determinant();
    VERIFY_IS_APPROX(det, std::ldexp(1.0, -964));
  }

  // Guard on a numerator factor (accumulated exponent incremented):
  // x = [-P, P, 0], y = [c, -c, d] with c = 2^1000 and d = 2^-1050. The
  // numerator difference x_1 - x_0 = 2^1024 overflows; the determinant
  // -4 P^3 (c^2 - d^2) / ((P^2 - c^2)^2 (P^2 - d^2) c d) equals
  // -2^-1017 / (1 - 2^-46)^2 up to relative corrections of order 2^-2050.
  {
    const double P = std::ldexp(1.0, 1023);
    const double c = std::ldexp(1.0, 1000);
    const double d = std::ldexp(1.0, -1050);
    Vec x(3), y(3);
    x << -P, P, 0.0;
    y << c, -c, d;
    const double det = Cauchy<double>(x, y).determinant();
    const double r = 1.0 - std::ldexp(1.0, -46);  // 1 - (c/P)^2
    VERIFY_IS_APPROX(det, -std::ldexp(1.0, -1017) / (r * r));
  }

  // Genuine overflow: clustered tiny nodes push the determinant past the
  // representable range while every factor stays finite; the accumulated
  // exponent must saturate to a correctly signed infinity, in both signs.
  {
    const double t = std::ldexp(1.0, -537);
    Vec x(2), y(2);
    x << 0.0, 3.0 * t;
    y << t, 2.0 * t;  // det = -3 / (4 t^2) ~ -1.6e323
    const double det = Cauchy<double>(x, y).determinant();
    VERIFY((numext::isinf)(det) && std::signbit(det));
  }
  {
    const double t = std::ldexp(1.0, -537);
    Vec x(2), y(2);
    x << 0.0, 3.0 * t;
    y << 2.0 * t, t;  // swapped y nodes: det = +3 / (4 t^2)
    const double det = Cauchy<double>(x, y).determinant();
    VERIFY((numext::isinf)(det) && !std::signbit(det));
  }
}

// Boundary nodes in every coefficient evaluation: when x_i - y_j overflows for
// finite nodes, a naively formed coefficient collapses to 1/Inf = 0 while the
// true value is a representable subnormal. coeff(), the dense materialization,
// the products and the CauchyLU factorization must all produce the guarded
// value -- and agree with determinant(), which derives the same quantity
// through its balanced accumulation.
void test_cauchy_boundary_coefficients() {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef Matrix<double, Dynamic, Dynamic> Mat;
  const double X = (std::numeric_limits<double>::max)();
  const double e = std::ldexp(1.0, -1025);  // 1/(2*DBL_MAX), correctly rounded (subnormal)

  // 1x1 reviewer case: the single coefficient is 1/(2*DBL_MAX) = 2^-1025
  // (2.7813423231340017e-309). Every step is correctly rounded IEEE arithmetic
  // with a power-of-two result, so the checks are exact equalities.
  {
    Vec x(1), y(1);
    x << X;
    y << -X;
    Cauchy<double> C(x, y);
    VERIFY_IS_EQUAL(C.coeff(0, 0), e);
    Mat d = C;
    VERIFY_IS_EQUAL(d(0, 0), e);
    Mat acc = Mat::Zero(1, 1);
    acc += C;  // addTo
    VERIFY_IS_EQUAL(acc(0, 0), e);
    acc -= C;  // subTo
    VERIFY_IS_EQUAL(acc(0, 0), 0.0);
    Vec p = C * Vec::Ones(1);
    VERIFY_IS_EQUAL(p[0], e);
    // determinant() reaches 2^-1025 through the balanced accumulation; the
    // guarded coefficient matches it exactly.
    VERIFY_IS_EQUAL(C.determinant(), e);
    // The old code materialized a zero coefficient and reported the (regular)
    // matrix as singular; the guarded pivot is subnormal but non-zero.
    CauchyLU<double> lu(C);
    VERIFY(lu.info() == Success);
    Vec u = lu.solve(p);  // p = C * [1]; recovered exactly (e / e = 1)
    VERIFY_IS_EQUAL(u[0], 1.0);
  }

  // A non-power-of-two boundary difference catches double rounding: computing
  // (1/t)*0.5 is one subnormal ULP above the correctly rounded 0.5/t. MPFR gives
  // the expected value below for x=1.1777865466541931e308 and
  // y=-1.7435684109415042e308, represented by their exact double bit patterns.
  {
    Vec x(1), y(1);
    x << numext::bit_cast<double>(numext::uint64_t(0x7fe4f71daafe5a86ull));
    y << numext::bit_cast<double>(numext::uint64_t(0xffef095b3493b755ull));
    const double expected = numext::bit_cast<double>(numext::uint64_t(0x00027621a9baa547ull));
    Cauchy<double> C(x, y);
    VERIFY_IS_EQUAL(C.coeff(0, 0), expected);
    Mat dense = C;
    VERIFY_IS_EQUAL(dense(0, 0), expected);
    Vec product = C * Vec::Ones(1);
    VERIFY_IS_EQUAL(product[0], expected);
    VERIFY_IS_EQUAL(C.determinant(), expected);
    CauchyLU<double> lu(C);
    VERIFY(lu.info() == Success);
    Vec rhs(1);
    rhs << expected;
    Vec solution = lu.solve(rhs);
    VERIFY_IS_EQUAL(solution[0], 1.0);
  }

  // 2x2 with boundary pairs among moderate ones: three of the four differences
  // overflow (2X and two 1.5X), one stays finite (X). All APIs agree with the
  // reference built from the guarded reciprocal; scaled by its magnitude the
  // matrix is well conditioned (cond ~ 38), so the GKO solve -- whose column
  // generation and generator updates cross the boundary too -- recovers the
  // solution accurately.
  {
    Vec x(2), y(2);
    x << X, X / 2;
    y << -X, -X / 2;
    Cauchy<double> C(x, y);
    Mat ref = reference_cauchy_guarded<double>(x, y);
    VERIFY_IS_EQUAL(ref(0, 0), e);                       // 1/(2*DBL_MAX)
    VERIFY_IS_EQUAL(ref(1, 1), std::ldexp(1.0, -1024));  // 1/DBL_MAX
    VERIFY(ref.allFinite());
    VERIFY((ref.array() != 0.0).all());
    for (Index j = 0; j < 2; ++j)
      for (Index i = 0; i < 2; ++i) VERIFY_IS_EQUAL(C.coeff(i, j), ref(i, j));
    Mat d = C;
    for (Index j = 0; j < 2; ++j)
      for (Index i = 0; i < 2; ++i) VERIFY_IS_EQUAL(d(i, j), ref(i, j));
    Vec v(2);
    v << 0.75, -0.5;
    VERIFY_IS_APPROX((C * v).eval(), (ref * v).eval());
    CauchyLU<double> lu(C);
    VERIFY(lu.info() == Success);
    Vec b = C * Vec::Ones(2);
    Vec u = lu.solve(b);
    VERIFY_IS_APPROX(u, Vec::Ones(2).eval());
  }

  // Complex nodes: purely imaginary boundary nodes overflow in the imaginary
  // component of the difference; the guard applies componentwise. The paths
  // sharing the helper agree exactly; the value is -i/(2*DBL_MAX) and the
  // determinant derives it independently, both to within a few subnormal
  // spacings (complex division rounds per component).
  {
    typedef std::complex<double> Cplx;
    const double tiny = 4.0 * std::numeric_limits<double>::denorm_min();
    Matrix<Cplx, Dynamic, 1> xc(1), yc(1);
    xc << Cplx(0.0, X);
    yc << Cplx(0.0, -X);
    Cauchy<Cplx> C(xc, yc);
    const Cplx cval = C.coeff(0, 0);
    Matrix<Cplx, Dynamic, Dynamic> dc = C;
    VERIFY_IS_EQUAL(dc(0, 0), cval);
    Matrix<Cplx, Dynamic, 1> pc = C * Matrix<Cplx, Dynamic, 1>::Ones(1);
    VERIFY_IS_EQUAL(pc[0], cval);
    VERIFY(numext::abs(numext::real(cval)) <= tiny);
    VERIFY(numext::abs(numext::imag(cval) + e) <= tiny);
    const Cplx det = C.determinant();
    VERIFY(numext::abs(numext::real(det) - numext::real(cval)) <= tiny);
    VERIFY(numext::abs(numext::imag(det) - numext::imag(cval)) <= tiny);
  }
}

template <typename Scalar, int M, int N>
void test_cauchy_fixed() {
  typedef Matrix<Scalar, M, 1> XVec;
  typedef Matrix<Scalar, N, 1> YVec;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, M, N> MatMN;

  Vec xd, yd;
  separated_nodes<Scalar>(M, N, xd, yd);
  XVec x = xd;
  YVec y = yd;
  typedef Cauchy<Scalar, M, N> CauchyType;
  CauchyType C(x, y);
  STATIC_CHECK((Cauchy<Scalar, M, N>::RowsAtCompileTime == M));
  STATIC_CHECK((Cauchy<Scalar, M, N>::ColsAtCompileTime == N));
  STATIC_CHECK((internal::remove_all_t<decltype(makeCauchy(x, y))>::ColsAtCompileTime == N));
  STATIC_CHECK((internal::remove_all_t<decltype(C.transpose())>::RowsAtCompileTime == N));
  STATIC_CHECK((internal::remove_all_t<decltype(C.transpose())>::ColsAtCompileTime == M));

  MatMN dense = C;
  VERIFY_IS_APPROX(dense, MatMN(reference_cauchy<Scalar>(xd, yd)));

  CauchyType* heapC = new CauchyType(x, y);
  VERIFY(std::uintptr_t(heapC) % std::alignment_of<CauchyType>::value == 0);
  MatMN heapDense = *heapC;
  delete heapC;
  VERIFY_IS_APPROX(heapDense, dense);

#if EIGEN_MAX_ALIGN_BYTES > 0 && !EIGEN_HAS_CXX17_OVERALIGN
  void* raw = (CauchyType::operator new)(sizeof(CauchyType));
  VERIFY(std::uintptr_t(raw) % std::alignment_of<CauchyType>::value == 0);
  (CauchyType::operator delete)(raw);
#endif

  YVec v = YVec::Random();
  Matrix<Scalar, M, 1> w = C * v;
  VERIFY_IS_APPROX(w, (dense * v).eval());

  // .noalias() keeps the direct (temporary-free) path of the default product tag;
  // the matching fixed dimensions also pin the compile-time product check.
  Matrix<Scalar, M, 1> w2;
  w2.noalias() = C * v;
  VERIFY_IS_APPROX(w2, (dense * v).eval());
}

EIGEN_DECLARE_TEST(structured_cauchy) {
  for (int i = 0; i < g_repeat; ++i) {
    // Products, dense assignment, coefficient access.
    CALL_SUBTEST_1((test_cauchy_product<double>(1, 1)));
    CALL_SUBTEST_1((test_cauchy_product<double>(8, 8)));
    CALL_SUBTEST_1((test_cauchy_product<double>(20, 12)));  // tall
    CALL_SUBTEST_1((test_cauchy_product<double>(12, 20)));  // wide
    CALL_SUBTEST_1((test_cauchy_product<float>(10, 10)));
    CALL_SUBTEST_1((test_cauchy_product<std::complex<double>>(9, 7)));
    CALL_SUBTEST_1((test_cauchy_product<std::complex<float>>(7, 9)));
    CALL_SUBTEST_1((test_cauchy_transpose<double>(10, 14)));
    CALL_SUBTEST_1((test_cauchy_transpose<std::complex<double>>(8, 6)));

    // GKO pivoted LU solves.
    CALL_SUBTEST_2((test_cauchy_lu<double>(1)));
    CALL_SUBTEST_2((test_cauchy_lu<double>(2)));
    CALL_SUBTEST_2((test_cauchy_lu<double>(12)));
    CALL_SUBTEST_2((test_cauchy_lu<double>(30)));
    CALL_SUBTEST_2((test_cauchy_lu<std::complex<double>>(16)));
    CALL_SUBTEST_2((test_cauchy_lu<float>(10)));
    CALL_SUBTEST_2(test_cauchy_hilbert());
    CALL_SUBTEST_2(test_cauchy_lu_pivoting(20));
    CALL_SUBTEST_2(test_cauchy_lu_singular());
    CALL_SUBTEST_2((test_cauchy_lu_scaled_generators<double>()));
    CALL_SUBTEST_2((test_cauchy_lu_scaled_generators<std::complex<double>>()));

    // Closed-form determinant, symmetric generalized Hilbert, fixed sizes.
    CALL_SUBTEST_3((test_cauchy_determinant<double>(4)));
    CALL_SUBTEST_3((test_cauchy_determinant<std::complex<double>>(4)));
    CALL_SUBTEST_3((test_cauchy_symmetric<double>(9)));
    CALL_SUBTEST_3((test_cauchy_fixed<double, 6, 4>()));
    CALL_SUBTEST_3((test_cauchy_fixed<std::complex<float>, 4, 5>()));

    // Numerical and lifetime boundaries: aliased and value-nested (owning)
    // delayed products, mixed-scalar products, wide-dynamic-range determinants.
    CALL_SUBTEST_4((test_cauchy_aliased_product<double>(11)));
    CALL_SUBTEST_4((test_cauchy_aliased_product<std::complex<double>>(8)));
    CALL_SUBTEST_4((test_cauchy_aliased_expression<double>(11)));
    CALL_SUBTEST_4((test_cauchy_aliased_expression<std::complex<double>>(8)));
    CALL_SUBTEST_4((test_cauchy_delayed_product<double>(12, 9)));
    CALL_SUBTEST_4((test_cauchy_delayed_product<std::complex<double>>(7, 10)));
    CALL_SUBTEST_4((test_cauchy_mixed_scalar<double>(10, 13)));
    CALL_SUBTEST_4((test_cauchy_mixed_scalar<float>(9, 6)));
    CALL_SUBTEST_4(test_cauchy_determinant_range());
    CALL_SUBTEST_4(test_cauchy_determinant_overflow_boundary());
    CALL_SUBTEST_4(test_cauchy_boundary_coefficients());
  }
}
