// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2009 Benoit Jacob <jacob.benoit.1@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "main.h"
#include "fp_control.h"
#include <Eigen/SVD>

template <typename MatrixType, typename JacobiScalar>
void jacobi(const MatrixType& m = MatrixType()) {
  Index rows = m.rows();
  Index cols = m.cols();

  enum { RowsAtCompileTime = MatrixType::RowsAtCompileTime, ColsAtCompileTime = MatrixType::ColsAtCompileTime };

  typedef Matrix<JacobiScalar, 2, 1> JacobiVector;

  const MatrixType a(MatrixType::Random(rows, cols));

  JacobiVector v = JacobiVector::Random().normalized();
  JacobiScalar c = v.x(), s = v.y();
  JacobiRotation<JacobiScalar> rot(c, s);

  {
    Index p = internal::random<Index>(0, rows - 1);
    Index q;
    do {
      q = internal::random<Index>(0, rows - 1);
    } while (q == p);

    MatrixType b = a;
    b.applyOnTheLeft(p, q, rot);
    VERIFY_IS_APPROX(b.row(p), c * a.row(p) + numext::conj(s) * a.row(q));
    VERIFY_IS_APPROX(b.row(q), -s * a.row(p) + numext::conj(c) * a.row(q));
  }

  {
    Index p = internal::random<Index>(0, cols - 1);
    Index q;
    do {
      q = internal::random<Index>(0, cols - 1);
    } while (q == p);

    MatrixType b = a;
    b.applyOnTheRight(p, q, rot);
    VERIFY_IS_APPROX(b.col(p), c * a.col(p) - s * a.col(q));
    VERIFY_IS_APPROX(b.col(q), numext::conj(s) * a.col(p) + numext::conj(c) * a.col(q));
  }
}

// Verify that JacobiRotation::makeGivens(p, q, &r) produces a rotation that
// zeros out q, even when (p, q) straddle the over-/underflow thresholds
// where the direct formula r = p * sqrt(1 + (q/p)^2) would over- or
// underflow.  Eigen's convention is r >= 0 with sign carried in c.
template <typename Scalar>
void verify_makeGivens(const Scalar& p, const Scalar& q) {
  using std::abs;
  Scalar r;
  JacobiRotation<Scalar> rot;
  rot.makeGivens(p, q, &r);

  // Eigen's J^T * [p; q] = [r; 0] with J = [c s; -s c]. Verify the homogeneous relation after scaling the inputs so
  // the check itself does not overflow or flush intermediate products to zero.
  const Scalar scale = numext::maxi(numext::maxi(abs(p), abs(q)), (std::numeric_limits<Scalar>::min)());
  const Scalar scaledP = p / scale;
  const Scalar scaledQ = q / scale;
  const Scalar scaledR = r / scale;
  const Scalar rotated0 = rot.c() * scaledP - rot.s() * scaledQ;
  const Scalar rotated1 = rot.s() * scaledP + rot.c() * scaledQ;

  const Scalar tol = NumTraits<Scalar>::epsilon() * (abs(scaledR) + (std::numeric_limits<Scalar>::min)()) * Scalar(64);
  VERIFY(abs(rotated0 - scaledR) <= tol);
  VERIFY(abs(rotated1) <= tol);
  VERIFY(r >= Scalar(0));
  VERIFY_IS_APPROX(numext::abs2(rot.c()) + numext::abs2(rot.s()), Scalar(1));
}

template <typename Scalar>
void jacobi_makegivens_safe_scaling() {
  using std::sqrt;
  const Scalar safmin = (std::numeric_limits<Scalar>::min)();
  const Scalar safmax = Scalar(1) / safmin;
  const Scalar rtmin = sqrt(safmin);
  const Scalar rtmax = sqrt(safmax / Scalar(2));
  const Scalar one(1);
  const Scalar two(2);
  const Scalar half(0.5);

  // Safe-range cases (regression — must keep existing fast path working).
  verify_makeGivens<Scalar>(Scalar(0), Scalar(0));
  verify_makeGivens<Scalar>(Scalar(3), Scalar(4));
  verify_makeGivens<Scalar>(Scalar(-3), Scalar(4));
  verify_makeGivens<Scalar>(Scalar(3), Scalar(-4));
  verify_makeGivens<Scalar>(Scalar(-3), Scalar(-4));

  // Both inputs near overflow: direct formula r = p * sqrt(1+(q/p)^2) would
  // overflow because sqrt(1+1) > 1.  Prescaling avoids this.
  verify_makeGivens<Scalar>(rtmax * two, rtmax);
  verify_makeGivens<Scalar>(-rtmax * two, rtmax);
  verify_makeGivens<Scalar>(rtmax, rtmax);
  verify_makeGivens<Scalar>(rtmax * Scalar(1.5), rtmax * Scalar(1.5));

  // Both inputs near underflow / subnormal: direct (q/p)^2 underflows to 0.
  verify_makeGivens<Scalar>(rtmin * half, rtmin * half);
  verify_makeGivens<Scalar>(safmin, safmin);
  verify_makeGivens<Scalar>(-safmin, safmin);

  // Mixed: one near overflow, one normal.
  verify_makeGivens<Scalar>(rtmax * Scalar(1.5), one);
  verify_makeGivens<Scalar>(one, rtmax * Scalar(1.5));
  verify_makeGivens<Scalar>(-rtmax * Scalar(1.5), one);

  // Mixed: one near underflow, one normal.
  verify_makeGivens<Scalar>(safmin, one);
  verify_makeGivens<Scalar>(one, safmin);

  // Mixed: subnormal and near-overflow simultaneously.
  verify_makeGivens<Scalar>(safmin, rtmax);
  verify_makeGivens<Scalar>(rtmax, safmin);
}

template <typename Scalar>
void jacobi_makejacobi_large_tau() {
  using std::abs;
  using std::sqrt;

  const Scalar rtmax = sqrt((std::numeric_limits<Scalar>::max)());
  for (int factor = 1; factor <= 2; ++factor) {
    const Scalar deno = Scalar(1) / (Scalar(factor) * rtmax);
    const Scalar y = deno * Scalar(0.5);
    for (int delta_sign = -1; delta_sign <= 1; delta_sign += 2) {
      const Scalar x = delta_sign > 0 ? Scalar(1) : Scalar(0);
      const Scalar z = delta_sign > 0 ? Scalar(0) : Scalar(1);
      JacobiRotation<Scalar> rotation;
      rotation.makeJacobi(x, y, z);

      const Scalar offdiag =
          rotation.c() * rotation.s() * (x - z) + (rotation.c() * rotation.c() - rotation.s() * rotation.s()) * y;
      VERIFY(!numext::is_exactly_zero(rotation.s()));
      VERIFY(abs(offdiag) <= NumTraits<Scalar>::epsilon() * abs(y));
    }
  }
}

template <typename Scalar>
void jacobi_makejacobi_complex() {
  using RealScalar = typename NumTraits<Scalar>::Real;
  using std::abs;
  using std::sqrt;

  const RealScalar rtmax = sqrt((std::numeric_limits<RealScalar>::max)());
  // Allow rounding in the rotation and in the complex off-diagonal residual.
  const RealScalar tolerance = RealScalar(16) * NumTraits<RealScalar>::epsilon();
  for (const RealScalar magnitude :
       {RealScalar(0.5) / rtmax, RealScalar(0.25) / rtmax, RealScalar(0.25), RealScalar(0.5), RealScalar(1)}) {
    for (int real_sign : {-1, 1}) {
      for (int imag_sign : {-1, 1}) {
        const Scalar y(RealScalar(real_sign) * RealScalar(0.6) * magnitude,
                       RealScalar(imag_sign) * RealScalar(0.8) * magnitude);
        for (int delta_sign : {-1, 0, 1}) {
          const RealScalar x = RealScalar(delta_sign);
          const RealScalar z = RealScalar(0);
          JacobiRotation<Scalar> rotation;
          VERIFY(rotation.makeJacobi(x, y, z));

          const Scalar c = rotation.c();
          const Scalar s = rotation.s();
          VERIFY_IS_EQUAL(numext::imag(c), RealScalar(0));
          VERIFY(numext::real(c) > RealScalar(0));
          VERIFY(abs(s) > RealScalar(0));
          VERIFY(abs(numext::abs2(c) + numext::abs2(s) - RealScalar(1)) <= tolerance);

          // (J* B J)(0,1), with J = [c conj(s); -s c] and B = [x y; conj(y) z].
          const Scalar conjugate_s = numext::conj(s);
          const Scalar offdiag = c * conjugate_s * (x - z) + c * c * y - conjugate_s * conjugate_s * numext::conj(y);
          VERIFY(abs(offdiag) <= tolerance * abs(y));
        }
      }
    }
  }
}

template <typename Scalar>
void jacobi_makejacobi_extreme_phase() {
  const Scalar tolerance = Scalar(8) * NumTraits<Scalar>::epsilon();
  const Scalar expected = numext::sqrt(Scalar(0.5));
  ScopedFlushToZero flushToZero;
  for (const Scalar magnitude :
       {(std::numeric_limits<Scalar>::max)() / Scalar(4), (std::numeric_limits<Scalar>::min)() * Scalar(4)}) {
    for (const Scalar sign : {Scalar(-1), Scalar(1)}) {
      JacobiRotation<Scalar> rotation;
      rotation.makeJacobi(Scalar(0), sign * magnitude, Scalar(0));
      VERIFY(numext::abs(rotation.c() - expected) <= tolerance);
      VERIFY(numext::abs(rotation.s() - sign * expected) <= tolerance);
    }
  }
}

template <typename Scalar>
void jacobi_makejacobi_ratio_boundaries() {
  const Scalar eps = NumTraits<Scalar>::epsilon();
  // Allow rounding in the rotation and the scaled 2x2 residual.
  const Scalar tolerance = Scalar(8) * eps;
  for (const Scalar scale : {(std::numeric_limits<Scalar>::min)() * Scalar(4), Scalar(1),
                             (std::numeric_limits<Scalar>::max)() / Scalar(8)}) {
    for (const Scalar ratio : {Scalar(0), eps, Scalar(0.5), Scalar(1) - eps, Scalar(1), Scalar(1) + eps, Scalar(2)}) {
      for (const Scalar sign : {Scalar(-1), Scalar(1)}) {
        JacobiRotation<Scalar> rotation;
        VERIFY(rotation.makeJacobi(scale * ratio, sign * scale * Scalar(0.5), Scalar(0)));
        const Scalar c = rotation.c();
        const Scalar s = rotation.s();
        VERIFY(c > Scalar(0));
        VERIFY(numext::abs(c * c + s * s - Scalar(1)) <= tolerance);
        const Scalar residual = c * s * ratio + (c * c - s * s) * sign * Scalar(0.5);
        VERIFY(numext::abs(residual) <= tolerance);
      }
    }
  }
}

EIGEN_DECLARE_TEST(jacobi) {
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_7((jacobi_makegivens_safe_scaling<float>()));
    CALL_SUBTEST_7((jacobi_makegivens_safe_scaling<double>()));
    CALL_SUBTEST_7((jacobi_makejacobi_large_tau<float>()));
    CALL_SUBTEST_7((jacobi_makejacobi_large_tau<double>()));
    CALL_SUBTEST_7((jacobi_makejacobi_extreme_phase<float>()));
    CALL_SUBTEST_7((jacobi_makejacobi_extreme_phase<double>()));
    CALL_SUBTEST_7((jacobi_makejacobi_ratio_boundaries<float>()));
    CALL_SUBTEST_7((jacobi_makejacobi_ratio_boundaries<double>()));
    CALL_SUBTEST_7((jacobi_makejacobi_ratio_boundaries<long double>()));
    CALL_SUBTEST_7((jacobi_makejacobi_ratio_boundaries<half>()));
    CALL_SUBTEST_7((jacobi_makejacobi_ratio_boundaries<bfloat16>()));
    CALL_SUBTEST_8((jacobi_makejacobi_complex<std::complex<float>>()));
    CALL_SUBTEST_8((jacobi_makejacobi_complex<std::complex<double>>()));

    CALL_SUBTEST_1((jacobi<Matrix3f, float>()));
    CALL_SUBTEST_2((jacobi<Matrix4d, double>()));
    CALL_SUBTEST_3((jacobi<Matrix4cf, float>()));
    CALL_SUBTEST_3((jacobi<Matrix4cf, std::complex<float> >()));

    CALL_SUBTEST_1((jacobi<Matrix<float, 3, 3, RowMajor>, float>()));
    CALL_SUBTEST_2((jacobi<Matrix<double, 4, 4, RowMajor>, double>()));
    CALL_SUBTEST_3((jacobi<Matrix<std::complex<float>, 4, 4, RowMajor>, float>()));
    CALL_SUBTEST_3((jacobi<Matrix<std::complex<float>, 4, 4, RowMajor>, std::complex<float> >()));

    int r = internal::random<int>(2, internal::random<int>(1, EIGEN_TEST_MAX_SIZE) / 2),
        c = internal::random<int>(2, internal::random<int>(1, EIGEN_TEST_MAX_SIZE) / 2);
    CALL_SUBTEST_4((jacobi<MatrixXf, float>(MatrixXf(r, c))));
    CALL_SUBTEST_5((jacobi<MatrixXcd, double>(MatrixXcd(r, c))));
    CALL_SUBTEST_5((jacobi<MatrixXcd, std::complex<double> >(MatrixXcd(r, c))));
    // complex<float> is really important to test as it is the only way to cover conjugation issues in certain unaligned
    // paths
    CALL_SUBTEST_6((jacobi<MatrixXcf, float>(MatrixXcf(r, c))));
    CALL_SUBTEST_6((jacobi<MatrixXcf, std::complex<float> >(MatrixXcf(r, c))));

    TEST_SET_BUT_UNUSED_VARIABLE(r);
    TEST_SET_BUT_UNUSED_VARIABLE(c);
  }
}
