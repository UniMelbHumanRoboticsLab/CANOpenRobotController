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

// Well-conditioned random operator: diagonal bounded away from zero and a
// contractive correction, so the capacitance matrix is safely invertible.
template <typename Scalar>
void random_dplr(Index n, Index k, Matrix<Scalar, Dynamic, 1>& d, Matrix<Scalar, Dynamic, Dynamic>& U,
                 Matrix<Scalar, Dynamic, Dynamic>& V) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  d = Matrix<Scalar, Dynamic, 1>::Random(n);
  d.array() += Scalar(RealScalar(3));
  U = Scalar(RealScalar(0.5)) * Matrix<Scalar, Dynamic, Dynamic>::Random(n, k);
  V = Scalar(RealScalar(0.5)) * Matrix<Scalar, Dynamic, Dynamic>::Random(n, k);
}

// Reference dense matrix built independently, entry by entry.
template <typename Scalar>
Matrix<Scalar, Dynamic, Dynamic> reference_dplr(const Matrix<Scalar, Dynamic, 1>& d,
                                                const Matrix<Scalar, Dynamic, Dynamic>& U,
                                                const Matrix<Scalar, Dynamic, Dynamic>& V) {
  const Index n = d.size();
  Matrix<Scalar, Dynamic, Dynamic> dense(n, n);
  for (Index j = 0; j < n; ++j)
    for (Index i = 0; i < n; ++i) {
      Scalar s = (i == j) ? d[i] : Scalar(0);
      for (Index t = 0; t < U.cols(); ++t) s += U(i, t) * numext::conj(V(j, t));
      dense(i, j) = s;
    }
  return dense;
}

template <typename Scalar>
void test_dplr_product(Index n, Index k) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec d;
  Mat U, V;
  random_dplr<Scalar>(n, k, d, U, V);
  DiagonalPlusLowRank<Scalar> A(d, U, V);
  Mat dense = reference_dplr<Scalar>(d, U, V);
  VERIFY_IS_EQUAL(A.correctionRank(), k);

  Mat Ad = A;
  VERIFY_IS_APPROX(Ad, dense);
  for (Index t = 0; t < 5; ++t) {
    Index i = internal::random<Index>(0, n - 1), j = internal::random<Index>(0, n - 1);
    VERIFY_IS_APPROX(A.coeff(i, j), dense(i, j));
  }

  Vec x = Vec::Random(n);
  VERIFY_IS_APPROX((A * x).eval(), (dense * x).eval());

  Mat X = Mat::Random(n, 3);
  VERIFY_IS_APPROX((A * X).eval(), (dense * X).eval());

  // Accumulation form exercised by the iterative solvers.
  Vec y = Vec::Random(n);
  Vec y0 = y;
  y.noalias() += A * x;
  VERIFY_IS_APPROX(y, (y0 + dense * x).eval());

  // Dimension mismatches are caught when the product expression is formed.
  VERIFY_RAISES_ASSERT(A * Vec(Vec::Random(n + 1)));
}

template <typename Scalar>
void test_dplr_transpose(Index n, Index k) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec d;
  Mat U, V;
  random_dplr<Scalar>(n, k, d, U, V);
  DiagonalPlusLowRank<Scalar> A(d, U, V);
  Mat dense = reference_dplr<Scalar>(d, U, V);

  Mat Td = A.transpose();
  VERIFY_IS_APPROX(Td, Mat(dense.transpose()));
  Mat Ad = A.adjoint();
  VERIFY_IS_APPROX(Ad, Mat(dense.adjoint()));
  Mat Kd = A.conjugate();
  VERIFY_IS_APPROX(Kd, Mat(dense.conjugate()));

  Vec x = Vec::Random(n);
  VERIFY_IS_APPROX((A.transpose() * x).eval(), (dense.transpose() * x).eval());
  VERIFY_IS_APPROX((A.adjoint() * x).eval(), (dense.adjoint() * x).eval());
}

template <typename Scalar>
void test_dplr_solve(Index n, Index k) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec d;
  Mat U, V;
  random_dplr<Scalar>(n, k, d, U, V);
  DiagonalPlusLowRank<Scalar> A(d, U, V);
  Mat dense = reference_dplr<Scalar>(d, U, V);

  Vec b = Vec::Random(n);
  Vec x = A.solve(b);
  VERIFY_IS_APPROX((dense * x).eval(), b);
  VERIFY_IS_APPROX(x, dense.partialPivLu().solve(b).eval());

  Mat B = Mat::Random(n, 3);
  Mat X = A.solve(B);
  VERIFY_IS_APPROX(X, dense.partialPivLu().solve(B).eval());

  Mat empty_rhs(n, 0);
  Mat empty_solution = A.solve(empty_rhs);
  VERIFY_IS_EQUAL(empty_solution.rows(), n);
  VERIFY_IS_EQUAL(empty_solution.cols(), 0);
}

template <typename Scalar>
void test_dplr_inverse_determinant(Index n, Index k) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec d;
  Mat U, V;
  random_dplr<Scalar>(n, k, d, U, V);
  DiagonalPlusLowRank<Scalar> A(d, U, V);
  Mat dense = reference_dplr<Scalar>(d, U, V);

  // The inverse is itself diagonal-plus-low-rank; materialize and check.
  DiagonalPlusLowRank<Scalar> Ainv = A.inverse();
  VERIFY_IS_EQUAL(Ainv.correctionRank(), k);
  Mat invd = Ainv;
  VERIFY_IS_APPROX((invd * dense).eval(), Mat(Mat::Identity(n, n)));

  // Inverting twice returns to the original operator.
  Mat back = Ainv.inverse();
  VERIFY_IS_APPROX(back, dense);

  VERIFY_IS_APPROX(A.determinant(), dense.determinant());
}

template <typename Scalar>
void test_dplr_matrix_free_gmres(Index n, Index k) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec d;
  Mat U, V;
  random_dplr<Scalar>(n, k, d, U, V);
  DiagonalPlusLowRank<Scalar> A(d, U, V);
  Mat dense = reference_dplr<Scalar>(d, U, V);

  Vec b = Vec::Random(n);
  GMRES<DiagonalPlusLowRank<Scalar>, IdentityPreconditioner> gmres;
  gmres.setTolerance(RealScalar(1e-12));
  gmres.compute(A);
  Vec x = gmres.solve(b);
  VERIFY(gmres.info() == Success);
  const RealScalar tol = RealScalar(5e6) * NumTraits<RealScalar>::epsilon();  // ~1e-9 in double
  VERIFY((dense * x - b).norm() <= tol * b.norm());
}

// The products carry the default product tag, so an assignment whose right-hand
// side aliases the destination materializes the product into a temporary exactly
// like a dense product would -- including the aliasing that the former
// is_same_dense check could not see: overlapping views and right-hand-side
// expressions referencing the destination.
template <typename Scalar>
void test_dplr_aliased_product(Index n, Index k) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  // Pin the routing: the structured products must dispatch through the shared
  // structured_product_impl.
  STATIC_CHECK(
      (std::is_base_of<internal::structured_product_impl<DiagonalPlusLowRank<Scalar>, Vec>,
                       internal::generic_product_impl<DiagonalPlusLowRank<Scalar>, Vec, internal::StructuredShape,
                                                      DenseShape, GemvProduct>>::value));

  Vec d;
  Mat U, V;
  random_dplr<Scalar>(n, k, d, U, V);
  DiagonalPlusLowRank<Scalar> A(d, U, V);
  Mat dense = reference_dplr<Scalar>(d, U, V);

  Vec x = Vec::Random(n);
  Vec y = x;
  y = A * y;
  VERIFY_IS_APPROX(y, (dense * x).eval());

  y = x;
  y += A * y;
  VERIFY_IS_APPROX(y, (x + dense * x).eval());

  // Right-hand-side expression referencing the destination.
  y = x;
  y = A * (y + Vec::Ones(n));
  VERIFY_IS_APPROX(y, (dense * (x + Vec::Ones(n))).eval());

  // Overlapping views of a shared buffer, in both directions.
  Vec w = Vec::Random(n + 1);
  Vec w0 = w;
  w.tail(n) = A * w.head(n);
  VERIFY_IS_APPROX(w.tail(n).eval(), (dense * w0.head(n)).eval());
  w = w0;
  w.head(n) = A * w.tail(n);
  VERIFY_IS_APPROX(w.head(n).eval(), (dense * w0.tail(n)).eval());
}

// Product expressions must nest the structured operand by value: a
// delayed-evaluated expression has to outlive the temporary operator it was
// built from. The static check pins the value nesting; the behavioral check
// would read freed memory if the product held a reference instead.
template <typename Scalar>
void test_dplr_delayed_product(Index n, Index k) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  STATIC_CHECK(!std::is_reference<typename internal::ref_selector<DiagonalPlusLowRank<Scalar>>::type>::value);

  Vec d;
  Mat U, V;
  random_dplr<Scalar>(n, k, d, U, V);
  DiagonalPlusLowRank<Scalar> A(d, U, V);
  Mat dense = reference_dplr<Scalar>(d, U, V);

  Vec x = Vec::Random(n);
  auto expr = A.adjoint() * x;        // the adjoint temporary dies with the full expression
  Vec scribble = Vec::Random(2 * n);  // reuses the temporary's freed heap storage
  Vec y = expr;
  VERIFY_IS_APPROX(y, (dense.adjoint() * x).eval());
  VERIFY_IS_EQUAL(scribble.size(), 2 * n);  // keep the scribble alive across the evaluation
}

// Mixed-scalar products: a real operator applied to a complex right-hand side
// (and a complex operator applied to a real one) promotes to the complex product
// scalar, so alpha, the workspaces and the accumulation must run in the promoted
// type rather than the operator scalar.
template <typename RealScalar>
void test_dplr_mixed_scalar(Index n, Index k) {
  typedef std::complex<RealScalar> Complex;
  typedef Matrix<RealScalar, Dynamic, 1> RVec;
  typedef Matrix<RealScalar, Dynamic, Dynamic> RMat;
  typedef Matrix<Complex, Dynamic, 1> CVec;
  typedef Matrix<Complex, Dynamic, Dynamic> CMat;

  RVec d;
  RMat U, V;
  random_dplr<RealScalar>(n, k, d, U, V);
  DiagonalPlusLowRank<RealScalar> A(d, U, V);
  CMat dense = reference_dplr<RealScalar>(d, U, V).template cast<Complex>();

  CVec x = CVec::Random(n);
  CVec y = A * x;
  VERIFY_IS_APPROX(y, (dense * x).eval());

  CVec y0 = CVec::Random(n);
  y = y0;
  y.noalias() += A * x;
  VERIFY_IS_APPROX(y, (y0 + dense * x).eval());

  CVec dc;
  CMat Uc, Vc;
  random_dplr<Complex>(n, k, dc, Uc, Vc);
  DiagonalPlusLowRank<Complex> Ac(dc, Uc, Vc);
  CMat denseC = reference_dplr<Complex>(dc, Uc, Vc);
  RVec xr = RVec::Random(n);
  CVec z = Ac * xr;
  VERIFY_IS_APPROX(z, (denseC * xr).eval());
}

// Wide-dynamic-range determinants: the diagonal product is accumulated in the
// balanced form m * 2^e, so partial products neither overflow nor underflow
// when the determinant itself is representable, whatever the ordering of large
// and small diagonal entries. Genuinely non-representable determinants must
// still saturate to infinity / zero.
void test_dplr_determinant_scaled() {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef Matrix<double, Dynamic, Dynamic> Mat;
  typedef std::complex<double> Complex;
  typedef Matrix<Complex, Dynamic, 1> CVec;
  typedef Matrix<Complex, Dynamic, Dynamic> CMat;

  {
    // The reviewer's magnitude regime: det(D) = 1e200 * 1e200 * 1e-100 = 1e300 is
    // finite, but the plain running product hits 1e400 partway. k = 0 at runtime.
    Vec d(3);
    d << 1e200, 1e200, 1e-100;
    DiagonalPlusLowRank<double> A(d, Mat::Zero(3, 0), Mat::Zero(3, 0));
    const double det = A.determinant();
    VERIFY((numext::isfinite)(det));
    VERIFY_IS_APPROX(det, 1e300);
  }
  {
    // Underflow-side analogue: the partial product flushes to zero at 1e-400
    // while det(D) = 1e-300 is representable.
    Vec d(3);
    d << 1e-200, 1e-200, 1e100;
    DiagonalPlusLowRank<double> A(d, Mat::Zero(3, 0), Mat::Zero(3, 0));
    const double det = A.determinant();
    VERIFY(det != 0.0);
    VERIFY_IS_APPROX(det, 1e-300);
  }
  {
    // The capacitance determinant enters the same balanced accumulation: with
    // 450 entries of 10 first, the running diagonal product tops out at 1e450
    // while det(D) = 10^450 * 10^-150 = 1e300; the rank-1 correction
    // U = d[0] e_0, V = e_0 gives the capacitance 1 + d[0]/d[0] = 2, so
    // det(A) = 2e300 by the determinant lemma.
    const Index n = 600;
    Vec d(n);
    d.head(450).setConstant(10.0);
    d.tail(150).setConstant(0.1);
    Mat U = Mat::Zero(n, 1), V = Mat::Zero(n, 1);
    U(0, 0) = d[0];
    V(0, 0) = 1.0;
    DiagonalPlusLowRank<double> A(d, U, V);
    const double det = A.determinant();
    // One rounding multiply per balanced factor, and the decimal constants 0.1
    // and their 150-fold product are themselves inexact: a few n eps total.
    const double det_tol = 4.0 * double(n) * NumTraits<double>::epsilon();
    VERIFY((numext::isfinite)(det));
    VERIFY(numext::abs(det / 2e300 - 1.0) <= det_tol);
  }
  {
    // Complex scalars balance on max(|re|, |im|): det = i * 1e200 * 1e100 = 1e300 i.
    CVec d(3);
    d << Complex(0, 1e200), Complex(1e200, 0), Complex(1e-100, 0);
    DiagonalPlusLowRank<Complex> A(d, CMat::Zero(3, 0), CMat::Zero(3, 0));
    VERIFY_IS_APPROX(A.determinant(), Complex(0, 1e300));
  }
  {
    // A genuinely overflowing determinant still reports infinity...
    Vec d(2);
    d << 1e200, 1e200;
    DiagonalPlusLowRank<double> A(d, Mat::Zero(2, 0), Mat::Zero(2, 0));
    VERIFY((numext::isinf)(A.determinant()));
  }
  {
    // ... and a genuinely vanishing one an exact zero.
    Vec d(2);
    d << 1e-200, 1e-200;
    DiagonalPlusLowRank<double> A(d, Mat::Zero(2, 0), Mat::Zero(2, 0));
    VERIFY_IS_EQUAL(A.determinant(), 0.0);
  }
}

// The two factors of the determinant lemma can sit at opposite ends of the exponent range even when
// the determinant is ~1, since D^{-1} scales the capacitance entries by the reciprocal diagonal. Both
// det(D) and det(capacitance) must therefore accumulate in the balanced mantissa * 2^e form, the
// latter from the LU pivots rather than the plain pivot product .determinant() would form.
void test_dplr_determinant_capacitance() {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef Matrix<double, Dynamic, Dynamic> Mat;
  typedef std::complex<double> Complex;
  typedef Matrix<Complex, Dynamic, Dynamic> CMat;
  const double eps = NumTraits<double>::epsilon();

  {
    // The reviewer's reproducer: D + I I^H = diag(1 + 1e-200) is essentially the
    // identity, but det(D) = 1e-400 underflows and det(capacitance) ~ 1e400
    // overflows when either is formed on its own.
    Vec d(2);
    d << 1e-200, 1e-200;
    DiagonalPlusLowRank<double> A(d, Mat::Identity(2, 2), Mat::Identity(2, 2));
    const double det = A.determinant();
    VERIFY(numext::abs(det - 1.0) <= 16.0 * eps);
  }
  {
    // Mirrored, in exact powers of two: d = 2^60 and U = -(2^60 - 2^8) I with
    // V = I represent diag(2^8), so det(A) = 2^176 exactly. Every intermediate
    // is a power-of-two computation (the capacitance is exactly 2^-52 I by
    // Sterbenz subtraction), yet det(D) = 2^1342-ish overflows and the plain
    // capacitance pivot product 2^-1144 underflows.
    const Index n = 22;
    Vec d = Vec::Constant(n, std::ldexp(1.0, 60));
    Mat U = (std::ldexp(1.0, 8) - std::ldexp(1.0, 60)) * Mat::Identity(n, n);
    Mat V = Mat::Identity(n, n);
    DiagonalPlusLowRank<double> A(d, U, V);
    VERIFY_IS_EQUAL(A.determinant(), std::ldexp(1.0, 176));
  }
  {
    // Complex analogue of the reproducer: the mantissa stays complex and is
    // balanced on max(|re|, |im|). D + I I^H = diag(1 + 1e-200 i), det ~ 1.
    Matrix<Complex, Dynamic, 1> d(2);
    d << Complex(0, 1e-200), Complex(0, 1e-200);
    DiagonalPlusLowRank<Complex> A(d, CMat::Identity(2, 2), CMat::Identity(2, 2));
    VERIFY_IS_APPROX(A.determinant(), Complex(1, 0));
  }
  {
    // Sign correctness through the capacitance path at extreme scales:
    // D + (-I) I^H = diag(1e-200 - 1) ~ -I_3, det ~ -1; the balanced pivot
    // product of the capacitance ~ -1e200 I_3 carries the sign.
    Vec d = Vec::Constant(3, 1e-200);
    DiagonalPlusLowRank<double> A(d, (-Mat::Identity(3, 3)).eval(), Mat::Identity(3, 3));
    VERIFY_IS_APPROX(A.determinant(), -1.0);
  }
  {
    // Sign correctness across mixed extreme diagonal scales (rank 0).
    Vec d(4);
    d << -1e200, 1e-200, 1e200, 1e-200;
    DiagonalPlusLowRank<double> A(d, Mat::Zero(4, 0), Mat::Zero(4, 0));
    VERIFY_IS_APPROX(A.determinant(), -1.0);
  }
  {
    // Genuine overflow saturates to infinity of the correct sign...
    Vec d(3);
    d << -1e200, 1e200, 1e200;
    DiagonalPlusLowRank<double> A(d, Mat::Zero(3, 0), Mat::Zero(3, 0));
    const double det = A.determinant();
    VERIFY((numext::isinf)(det) && det < 0.0);
  }
  {
    // ... and genuine underflow to a zero of the correct sign.
    Vec d(3);
    d << -1e-200, 1e-200, 1e-200;
    DiagonalPlusLowRank<double> A(d, Mat::Zero(3, 0), Mat::Zero(3, 0));
    const double det = A.determinant();
    VERIFY_IS_EQUAL(det, 0.0);
    VERIFY(std::signbit(det));
  }
}

// The capacitance triple product V^H D^{-1} U can overflow in its plain association while the
// capacitance itself is representable: D^{-1} pairs the reciprocal of a tiny diagonal with a huge
// factor before the other, tiny factor pulls the product back down. The Woodbury kernels form these
// products from exactly rescaled factors so no spurious Inf/NaN reaches determinant(), solve() or
// inverse().
void test_dplr_capacitance_overflow() {
  typedef Matrix<double, Dynamic, 1> Vec;
  typedef std::complex<double> Complex;
  typedef Matrix<Complex, Dynamic, 1> CVec;
  const double eps = NumTraits<double>::epsilon();

  {
    // The reviewer's case: D + UV^H = 1e-200 + 1e-200 * 1e200 ~ 1 and the
    // capacitance 1 + 1e200 is representable, but V^H D^{-1} = 1e400 is not;
    // determinant() used to return Inf and solve() NaN.
    Vec d(1), u(1), v(1);
    d << 1e-200;
    u << 1e-200;
    v << 1e200;
    DiagonalPlusLowRank<double> A(d, u, v);
    VERIFY((numext::isfinite)(A.capacitance()(0, 0)));
    VERIFY_IS_APPROX(A.capacitance()(0, 0), 1e200);
    VERIFY(numext::abs(A.determinant() - 1.0) <= 8.0 * eps);
    // The Woodbury splitting routes this solve through D^{-1} b ~ 1e200 and
    // cancels it back down to x ~ 1, so the amplified digits are lost to any
    // association (see the solve() documentation); the fix is that the solve
    // stays finite instead of turning into NaN through an infinite capacitance.
    Vec b(1);
    b << 1.0;
    VERIFY(A.solve(b).allFinite());

    // D^{-1} b is 1e400 here, though the operator is essentially the identity and the solution 1e200
    // is representable; without the power-of-two rescaling the first term is Inf and the correction
    // turns it into NaN, which is what this pins. It cannot pin finiteness: A ~ 1 makes the correction
    // cancel D^{-1} b over ~200 digits, and whether that leaves 0 or an ulp the undone scaling lifts
    // back to Inf is a codegen detail (x86-64 gives the first, aarch64 the second).
    Vec big_b(1);
    big_b << 1e200;
    VERIFY(!A.solve(big_b).array().isNaN().any());
  }
  {
    // The same overflow pattern in exact powers of two, where the Woodbury
    // arithmetic is exact end to end: d = 2^-522, U = 2^-1025, V = 2^503
    // represent A = [2^-521] exactly; V^H D^{-1} = 2^1025 still overflows, and
    // the capacitance is exactly 2. Determinant and solve must match the dense
    // results for the represented matrix exactly.
    Vec d(1), u(1), v(1);
    d << std::ldexp(1.0, -522);
    u << std::ldexp(1.0, -1025);
    v << std::ldexp(1.0, 503);
    DiagonalPlusLowRank<double> A(d, u, v);
    VERIFY_IS_EQUAL(A.capacitance()(0, 0), 2.0);
    VERIFY_IS_EQUAL(A.determinant(), std::ldexp(1.0, -521));
    Vec b(1);
    b << 1.0;
    VERIFY_IS_EQUAL(A.solve(b)(0), std::ldexp(1.0, 521));
  }
  {
    // Symmetric mirror (U huge, V tiny): the plain capacitance association is
    // safe here, but the inverse's D^{-1} U product overflows spuriously while
    // the inverse factor -1e400 / (1 + 1e200) ~ -1e200 is representable.
    Vec d(1), u(1), v(1);
    d << 1e-200;
    u << 1e200;
    v << 1e-200;
    DiagonalPlusLowRank<double> A(d, u, v);
    VERIFY(numext::abs(A.determinant() - 1.0) <= 8.0 * eps);
    DiagonalPlusLowRank<double> Ainv = A.inverse();
    VERIFY(Ainv.factorU().allFinite());
    VERIFY_IS_APPROX(Ainv.factorU()(0, 0), -1e200);
    Vec b(1);
    b << 1.0;
    VERIFY(A.solve(b).allFinite());
  }
  {
    // Exact-power-of-two mirror: d = 2^-522, U = 2^502, V = 2^-1025 represent
    // A = [1.5 * 2^-522]; D^{-1} U = 2^1024 overflows, yet the inverse factor
    // -2^1024 / 1.5 is representable and the solve is exact to rounding.
    Vec d(1), u(1), v(1);
    d << std::ldexp(1.0, -522);
    u << std::ldexp(1.0, 502);
    v << std::ldexp(1.0, -1025);
    DiagonalPlusLowRank<double> A(d, u, v);
    VERIFY_IS_EQUAL(A.capacitance()(0, 0), 1.5);
    VERIFY_IS_EQUAL(A.determinant(), 1.5 * std::ldexp(1.0, -522));
    DiagonalPlusLowRank<double> Ainv = A.inverse();
    VERIFY_IS_APPROX(Ainv.factorU()(0, 0), -std::ldexp(1.0 / 1.5, 1024));
    Vec b(1);
    b << 3.0;
    VERIFY_IS_EQUAL(A.solve(b)(0), std::ldexp(1.0, 523));
  }
  {
    // Complex analogue of the reviewer's case: the operator is essentially
    // [i], the capacitance 1 + 1e200 is representable, and the balancing and
    // rescaling act on component magnitudes.
    CVec d(1), u(1), v(1);
    d << Complex(0, 1e-200);
    u << Complex(1e-200, 0);
    v << Complex(0, -1e200);
    DiagonalPlusLowRank<Complex> A(d, u, v);
    VERIFY((numext::isfinite)(numext::real(A.capacitance()(0, 0))));
    VERIFY_IS_APPROX(A.capacitance()(0, 0), Complex(1e200, 0));
    VERIFY_IS_APPROX(A.determinant(), Complex(0, 1));
  }
}

// Moderate data must keep the plain product association: the normalized
// Woodbury kernels only engage when a conservative exponent bound says a
// product could overflow, so for well-scaled operators the capacitance and the
// solve are bit-identical to the unnormalized evaluation.
template <typename Scalar>
void test_dplr_capacitance_fast_path(Index n, Index k) {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

  Vec d;
  Mat U, V;
  random_dplr<Scalar>(n, k, d, U, V);
  DiagonalPlusLowRank<Scalar> A(d, U, V);

  // Match the stored reciprocal: scalar and packet complex division can round differently.
  const Vec dinv = d.cwiseInverse();
  Mat capRef = Mat::Identity(k, k);
  capRef.noalias() += V.adjoint() * dinv.asDiagonal() * U;
  VERIFY_IS_CWISE_EQUAL(A.capacitance(), capRef);

  Vec b = Vec::Random(n);
  PartialPivLU<Mat> capLU(capRef);
  Vec xRef = dinv.asDiagonal() * b;
  xRef.noalias() -= dinv.asDiagonal() * (U * capLU.solve(V.adjoint() * xRef));
  VERIFY_IS_CWISE_EQUAL(A.solve(b), xRef);
}

// A fixed correction rank of zero is a compile-level regression: the operator
// must instantiate -- the capacitance solve is compile-time dispatched away, so
// PartialPivLU<Matrix<Scalar, 0, 0>> is never formed -- and behave as the plain
// diagonal it is.
template <typename Scalar, int N>
void test_dplr_fixed_rank0() {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, N, 1> VecN;
  typedef Matrix<Scalar, N, 0> FacN0;
  typedef Matrix<Scalar, N, N> MatN;

  VecN d = VecN::Random();
  d.array() += Scalar(RealScalar(3));
  FacN0 U, V;
  DiagonalPlusLowRank<Scalar, N, 0> A(d, U, V);
  VERIFY_IS_EQUAL(A.correctionRank(), Index(0));

  MatN dense = MatN::Zero();
  dense.diagonal() = d;
  MatN Ad = A;
  VERIFY_IS_APPROX(Ad, dense);

  VecN x = VecN::Random();
  VecN y = A * x;
  VERIFY_IS_APPROX(y, (d.asDiagonal() * x).eval());

  VecN b = VecN::Random();
  VecN xs = A.solve(b);
  VERIFY_IS_APPROX(xs, (b.array() / d.array()).matrix().eval());

  VERIFY_IS_APPROX(A.determinant(), d.prod());

  DiagonalPlusLowRank<Scalar, N, 0> Ainv = A.inverse();
  MatN invd = Ainv;
  VERIFY_IS_APPROX((invd * dense).eval(), MatN(MatN::Identity()));

  // Dynamic size with the rank still fixed at zero takes the same dispatch.
  Matrix<Scalar, Dynamic, 1> dd = d;
  Matrix<Scalar, Dynamic, 0> Ud(N, 0), Vd(N, 0);
  DiagonalPlusLowRank<Scalar, Dynamic, 0> B(dd, Ud, Vd);
  Matrix<Scalar, Dynamic, 1> yb = B * Matrix<Scalar, Dynamic, 1>(x);
  VERIFY_IS_APPROX(yb, (d.asDiagonal() * x).eval());
  VERIFY_IS_APPROX(B.solve(Matrix<Scalar, Dynamic, 1>(b)).eval(), (b.array() / d.array()).matrix().eval());
  VERIFY_IS_APPROX(B.determinant(), d.prod());
}

template <typename Scalar, int N, int K>
void test_dplr_fixed() {
  typedef Matrix<Scalar, N, 1> VecN;
  typedef Matrix<Scalar, N, K> FacNK;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;
  typedef typename NumTraits<Scalar>::Real RealScalar;

  VecN d = VecN::Random();
  d.array() += Scalar(RealScalar(3));
  FacNK U = FacNK::Random(), V = FacNK::Random();
  DiagonalPlusLowRank<Scalar, N, K> A(d, U, V);
  STATIC_CHECK((DiagonalPlusLowRank<Scalar, N, K>::RowsAtCompileTime == N));
  STATIC_CHECK((internal::remove_all_t<decltype(makeDiagonalPlusLowRank(d, U, V))>::RowsAtCompileTime == N));

  Mat dense = reference_dplr<Scalar>(Vec(d), Mat(U), Mat(V));
  Matrix<Scalar, N, N> Adense = A;
  VERIFY_IS_APPROX(Mat(Adense), dense);

  VecN x = VecN::Random();
  VecN y = A * x;
  VERIFY_IS_APPROX(y, (dense * x).eval());

  VecN b = VecN::Random();
  VecN xs = A.solve(b);
  VERIFY_IS_APPROX((dense * xs).eval(), b);

  // A dynamic right-hand side against the fixed-size operator: the compile-time
  // product-dimension check must accept the Dynamic/fixed mix.
  Vec xd = Vec(x);
  Vec yd = A * xd;
  VERIFY_IS_APPROX(yd, (dense * xd).eval());
}

EIGEN_DECLARE_TEST(structured_dplr) {
  for (int i = 0; i < g_repeat; ++i) {
    // Products, dense assignment, coefficient access; k = 0 is a plain diagonal
    // and k = n a full-rank correction.
    CALL_SUBTEST_1((test_dplr_product<double>(1, 0)));
    CALL_SUBTEST_1((test_dplr_product<double>(10, 0)));
    CALL_SUBTEST_1((test_dplr_product<double>(10, 1)));
    CALL_SUBTEST_1((test_dplr_product<double>(20, 3)));
    CALL_SUBTEST_1((test_dplr_product<double>(8, 8)));
    CALL_SUBTEST_1((test_dplr_product<float>(12, 2)));
    CALL_SUBTEST_1((test_dplr_product<std::complex<double>>(9, 2)));
    CALL_SUBTEST_1((test_dplr_product<std::complex<float>>(7, 3)));
    CALL_SUBTEST_1((test_dplr_transpose<double>(10, 2)));
    CALL_SUBTEST_1((test_dplr_transpose<std::complex<double>>(8, 3)));

    // Woodbury solves against the dense LU.
    CALL_SUBTEST_2((test_dplr_solve<double>(1, 0)));
    CALL_SUBTEST_2((test_dplr_solve<double>(10, 0)));
    CALL_SUBTEST_2((test_dplr_solve<double>(10, 1)));
    CALL_SUBTEST_2((test_dplr_solve<double>(30, 4)));
    CALL_SUBTEST_2((test_dplr_solve<double>(8, 8)));
    CALL_SUBTEST_2((test_dplr_solve<float>(12, 2)));
    CALL_SUBTEST_2((test_dplr_solve<std::complex<double>>(14, 3)));
    CALL_SUBTEST_2((test_dplr_solve<std::complex<float>>(8, 2)));

    // Inverse closure, determinant lemma, matrix-free GMRES, fixed sizes.
    CALL_SUBTEST_3((test_dplr_inverse_determinant<double>(10, 2)));
    CALL_SUBTEST_3((test_dplr_inverse_determinant<double>(6, 0)));
    CALL_SUBTEST_3((test_dplr_inverse_determinant<std::complex<double>>(8, 3)));
    CALL_SUBTEST_3((test_dplr_matrix_free_gmres<double>(24, 3)));
    CALL_SUBTEST_3((test_dplr_fixed<double, 6, 2>()));
    CALL_SUBTEST_3((test_dplr_fixed<std::complex<float>, 5, 1>()));

    // Review regressions: aliased and value-nested (owning) delayed products,
    // mixed-scalar products, the balanced determinant accumulation, and the
    // fixed rank-0 instantiation.
    CALL_SUBTEST_4((test_dplr_aliased_product<double>(11, 3)));
    CALL_SUBTEST_4((test_dplr_aliased_product<std::complex<double>>(9, 2)));
    CALL_SUBTEST_4((test_dplr_delayed_product<double>(13, 2)));
    CALL_SUBTEST_4((test_dplr_delayed_product<std::complex<double>>(10, 3)));
    CALL_SUBTEST_4((test_dplr_mixed_scalar<double>(12, 3)));
    CALL_SUBTEST_4((test_dplr_mixed_scalar<float>(9, 2)));
    CALL_SUBTEST_4(test_dplr_determinant_scaled());
    CALL_SUBTEST_4(test_dplr_determinant_capacitance());
    CALL_SUBTEST_4(test_dplr_capacitance_overflow());
    CALL_SUBTEST_4((test_dplr_capacitance_fast_path<double>(11, 3)));
    CALL_SUBTEST_4((test_dplr_capacitance_fast_path<std::complex<double>>(9, 2)));
    CALL_SUBTEST_4((test_dplr_fixed_rank0<double, 5>()));
    CALL_SUBTEST_4((test_dplr_fixed_rank0<std::complex<float>, 4>()));
  }
}
