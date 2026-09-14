// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2008 Benoit Jacob <jacob.benoit.1@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "main.h"
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/QR>

// A complex plane equation may be scaled by any nonzero gamma without moving its zero set, which
// carries (normal, offset) to (conj(gamma) * normal, gamma * offset) because dot() is
// conjugate-linear in the normal. Neither |gamma| = 1 nor a real gamma is required; i and 3 - i
// are both far enough from 1 for isApprox() to separate the two equations.
template <typename HyperplaneType>
void hyperplane_complex_scaling(const HyperplaneType &, std::false_type) {}

template <typename HyperplaneType>
void hyperplane_complex_scaling(const HyperplaneType &plane, std::true_type) {
  using Scalar = typename HyperplaneType::Scalar;
  using VectorType = typename HyperplaneType::VectorType;

  for (const Scalar gamma : {Scalar(0, 1), Scalar(3, -1)}) {
    HyperplaneType rescaled(numext::conj(gamma) * plane.normal(), gamma * plane.offset());
    VERIFY(plane.isCoincident(rescaled));
    VERIFY(rescaled.isCoincident(plane));
    VERIFY(!plane.isApprox(rescaled));

    // The scaling is exactly what signedDistance() reports, so the two zero sets agree.
    const VectorType p = VectorType::Random(plane.dim());
    VERIFY_IS_APPROX(rescaled.signedDistance(p), gamma * plane.signedDistance(p));
  }
}

template <typename HyperplaneType>
void hyperplane(const HyperplaneType &_plane) {
  /* this test covers the following files:
     Hyperplane.h
  */
  using std::abs;
  const Index dim = _plane.dim();
  enum { Options = HyperplaneType::Options };
  typedef typename HyperplaneType::Scalar Scalar;
  typedef typename HyperplaneType::RealScalar RealScalar;
  typedef Matrix<Scalar, HyperplaneType::AmbientDimAtCompileTime, 1> VectorType;
  typedef Matrix<Scalar, HyperplaneType::AmbientDimAtCompileTime, HyperplaneType::AmbientDimAtCompileTime> MatrixType;

  VectorType p0 = VectorType::Random(dim);
  VectorType p1 = VectorType::Random(dim);

  VectorType n0 = VectorType::Random(dim).normalized();
  VectorType n1 = VectorType::Random(dim).normalized();

  HyperplaneType pl0(n0, p0);
  HyperplaneType pl1(n1, p1);
  HyperplaneType pl2 = pl1;

  Scalar s0 = internal::random<Scalar>();
  Scalar s1 = internal::random<Scalar>();

  VERIFY_IS_APPROX(n1.dot(n1), Scalar(1));

  VERIFY_IS_MUCH_SMALLER_THAN(pl0.absDistance(p0), Scalar(1));
  if (numext::abs2(s0) > RealScalar(1e-6))
    VERIFY_IS_APPROX(pl1.signedDistance(p1 + n1 * s0), s0);
  else
    VERIFY_IS_MUCH_SMALLER_THAN(abs(pl1.signedDistance(p1 + n1 * s0) - s0), Scalar(1));
  VERIFY_IS_MUCH_SMALLER_THAN(pl1.signedDistance(pl1.projection(p0)), Scalar(1));
  VERIFY_IS_MUCH_SMALLER_THAN(pl1.absDistance(p1 + pl1.normal().unitOrthogonal() * s1), Scalar(1));

  // isCoincident() compares hyperplanes as point sets, so it ignores the orientation that
  // isApprox() distinguishes (issue #2033).
  {
    HyperplaneType flipped(-pl1.normal(), -pl1.offset());
    VERIFY(pl1.isCoincident(pl1));
    VERIFY(pl1.isCoincident(flipped));
    VERIFY(flipped.isCoincident(pl1));
    VERIFY(!pl1.isApprox(flipped));
    // Same normal, shifted by one unit along it: still a hyperplane, no longer the same one.
    HyperplaneType shifted(pl1.normal(), pl1.offset() + Scalar(1));
    VERIFY(!pl1.isCoincident(shifted));
    VERIFY(!shifted.isCoincident(pl1));

    // Rescaling an equation by a nonzero factor does not move its zero set, so neither the
    // verdict nor its independence of the argument order may depend on the coefficient
    // magnitudes.
    HyperplaneType hundredfold = pl1;
    hundredfold.coeffs() *= Scalar(100);
    HyperplaneType thousandth = pl1;
    thousandth.coeffs() *= Scalar(RealScalar(1) / RealScalar(1024));
    VERIFY(pl1.isCoincident(hundredfold));
    VERIFY(hundredfold.isCoincident(pl1));
    VERIFY(pl1.isCoincident(thousandth));
    VERIFY(thousandth.isCoincident(pl1));
    VERIFY(hundredfold.isCoincident(thousandth));
    VERIFY(thousandth.isCoincident(hundredfold));

    // Nearly coincident, with equations a decade apart in magnitude: what decides is the sine of
    // the angle between them, which is at most `tilt` here and is symmetric, so a precision above
    // it accepts and one below it rejects in both argument orders. Projecting one normal onto the
    // other instead makes the tolerance scale with the operand order, and this pair is accepted
    // one way and rejected the other.
    const RealScalar tilt(0.05);
    HyperplaneType tilted = pl1;
    tilted.coeffs() *= Scalar(RealScalar(0.1));
    tilted.normal() += pl1.normal().unitOrthogonal() * Scalar(RealScalar(0.1) * tilt);
    VERIFY(pl1.isCoincident(tilted, RealScalar(2) * tilt));
    VERIFY(tilted.isCoincident(pl1, RealScalar(2) * tilt));
    VERIFY(!pl1.isCoincident(tilted, RealScalar(0.1) * tilt));
    VERIFY(!tilted.isCoincident(pl1, RealScalar(0.1) * tilt));

    // Rescaling the ambient coordinates by s carries (n, d) to (n, s * d) without moving anything,
    // so the verdict may not depend on it, and a distant hyperplane may not relax the comparison of
    // the normals: comparing the coefficient vectors as a whole gets both of these wrong once |d|
    // dominates ||n||, and reports even perpendicular hyperplanes as coincident.
    for (const RealScalar coordinate_scale : {RealScalar(1), RealScalar(1000), RealScalar(1000000)}) {
      HyperplaneType distant(pl1.normal(), Scalar(coordinate_scale));
      HyperplaneType perpendicular(pl1.normal().unitOrthogonal(), Scalar(coordinate_scale));
      HyperplaneType nudged(pl1.normal(), Scalar(coordinate_scale * RealScalar(1.001)));
      VERIFY(distant.isCoincident(distant));
      VERIFY(!distant.isCoincident(perpendicular));
      VERIFY(!perpendicular.isCoincident(distant));
      VERIFY(!distant.isCoincident(nudged));
      VERIFY(!nudged.isCoincident(distant));
    }

    hyperplane_complex_scaling(pl1, internal::bool_constant<NumTraits<Scalar>::IsComplex>());
  }

  // transform
  if (!NumTraits<Scalar>::IsComplex) {
    MatrixType rot = MatrixType::Random(dim, dim).householderQr().householderQ();
    DiagonalMatrix<Scalar, HyperplaneType::AmbientDimAtCompileTime> scaling(VectorType::Random());
    Translation<Scalar, HyperplaneType::AmbientDimAtCompileTime> translation(VectorType::Random());

    while (scaling.diagonal().cwiseAbs().minCoeff() < RealScalar(1e-4)) scaling.diagonal() = VectorType::Random();

    pl2 = pl1;
    VERIFY_IS_MUCH_SMALLER_THAN(pl2.transform(rot).absDistance(rot * p1), Scalar(1));
    pl2 = pl1;
    VERIFY_IS_MUCH_SMALLER_THAN(pl2.transform(rot, Isometry).absDistance(rot * p1), Scalar(1));
    pl2 = pl1;
    VERIFY_IS_MUCH_SMALLER_THAN(pl2.transform(rot * scaling).absDistance((rot * scaling) * p1), Scalar(1));
    VERIFY_IS_APPROX(pl2.normal().norm(), RealScalar(1));
    pl2 = pl1;
    VERIFY_IS_MUCH_SMALLER_THAN(
        pl2.transform(rot * scaling * translation).absDistance((rot * scaling * translation) * p1), Scalar(1));
    VERIFY_IS_APPROX(pl2.normal().norm(), RealScalar(1));
    pl2 = pl1;
    VERIFY_IS_MUCH_SMALLER_THAN(pl2.transform(rot * translation, Isometry).absDistance((rot * translation) * p1),
                                Scalar(1));
    VERIFY_IS_APPROX(pl2.normal().norm(), RealScalar(1));
  }

  // casting
  const int Dim = HyperplaneType::AmbientDimAtCompileTime;
  typedef typename GetDifferentType<Scalar>::type OtherScalar;
  Hyperplane<OtherScalar, Dim, Options> hp1f = pl1.template cast<OtherScalar>();
  VERIFY_IS_APPROX(hp1f.template cast<Scalar>(), pl1);
  Hyperplane<Scalar, Dim, Options> hp1d = pl1.template cast<Scalar>();
  VERIFY_IS_APPROX(hp1d.template cast<Scalar>(), pl1);
}

template <typename Scalar>
void lines() {
  using std::abs;
  typedef Hyperplane<Scalar, 2> HLine;
  typedef ParametrizedLine<Scalar, 2> PLine;
  typedef Matrix<Scalar, 2, 1> Vector;
  typedef Matrix<Scalar, 3, 1> CoeffsType;

  for (int i = 0; i < 10; i++) {
    Vector center = Vector::Random();
    Vector u = Vector::Random();
    Vector v = Vector::Random();
    Scalar a = internal::random<Scalar>();
    if (abs(a - 1) < Scalar(1e-4)) a = Scalar(0);
    if (u.norm() < Scalar(1e-4)) u = Vector::Unit(0);
    if (v.norm() < Scalar(1e-4)) v = Vector::Unit(1);

    HLine line_u = HLine::Through(center + u, center + a * u);
    HLine line_v = HLine::Through(center + v, center + a * v);

    // the line equations should be normalized so that a^2+b^2=1
    VERIFY_IS_APPROX(line_u.normal().norm(), Scalar(1));
    VERIFY_IS_APPROX(line_v.normal().norm(), Scalar(1));

    Vector result = line_u.intersection(line_v);

    // the lines should intersect at the point we called "center"
    if (abs(a - 1) > Scalar(1e-2) && abs(v.normalized().dot(u.normalized())) < Scalar(0.9))
      VERIFY_IS_APPROX(result, center);

    // check conversions between two types of lines
    PLine pl(line_u);  // gcc 3.3 will crash if we don't name this variable.
    HLine line_u2(pl);
    CoeffsType converted_coeffs = line_u2.coeffs();
    if (line_u2.normal().dot(line_u.normal()) < Scalar(0)) converted_coeffs = -line_u2.coeffs();
    VERIFY(line_u.coeffs().isApprox(converted_coeffs));
  }
}

template <typename Scalar>
void planes() {
  using std::abs;
  typedef Hyperplane<Scalar, 3> Plane;
  typedef Matrix<Scalar, 3, 1> Vector;

  for (int i = 0; i < 10; i++) {
    Vector v0 = Vector::Random();
    Vector v1(v0), v2(v0);
    if (internal::random<double>(0, 1) > 0.25) v1 += Vector::Random();
    if (internal::random<double>(0, 1) > 0.25)
      v2 += v1 * std::pow(internal::random<Scalar>(0, 1), internal::random<int>(1, 16));
    if (internal::random<double>(0, 1) > 0.25)
      v2 += Vector::Random() * std::pow(internal::random<Scalar>(0, 1), internal::random<int>(1, 16));

    Plane p0 = Plane::Through(v0, v1, v2);

    VERIFY_IS_APPROX(p0.normal().norm(), Scalar(1));
    VERIFY_IS_MUCH_SMALLER_THAN(p0.absDistance(v0), Scalar(1));
    VERIFY_IS_MUCH_SMALLER_THAN(p0.absDistance(v1), Scalar(1));
    VERIFY_IS_MUCH_SMALLER_THAN(p0.absDistance(v2), Scalar(1));
  }
}

template <typename Scalar>
void hyperplane_alignment() {
  typedef Hyperplane<Scalar, 3, AutoAlign> Plane3a;
  typedef Hyperplane<Scalar, 3, DontAlign> Plane3u;

  EIGEN_ALIGN_MAX Scalar array1[4];
  EIGEN_ALIGN_MAX Scalar array2[4];
  EIGEN_ALIGN_MAX Scalar array3[4 + 1];
  Scalar *array3u = array3 + 1;

  Plane3a *p1 = ::new (reinterpret_cast<void *>(array1)) Plane3a;
  Plane3u *p2 = ::new (reinterpret_cast<void *>(array2)) Plane3u;
  Plane3u *p3 = ::new (reinterpret_cast<void *>(array3u)) Plane3u;

  p1->coeffs().setRandom();
  *p2 = *p1;
  *p3 = *p1;

  VERIFY_IS_APPROX(p1->coeffs(), p2->coeffs());
  VERIFY_IS_APPROX(p1->coeffs(), p3->coeffs());
}

EIGEN_DECLARE_TEST(geo_hyperplane) {
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(hyperplane(Hyperplane<float, 2>()));
    CALL_SUBTEST_2(hyperplane(Hyperplane<float, 3>()));
    CALL_SUBTEST_2(hyperplane(Hyperplane<float, 3, DontAlign>()));
    CALL_SUBTEST_2(hyperplane_alignment<float>());
    CALL_SUBTEST_3(hyperplane(Hyperplane<double, 4>()));
    CALL_SUBTEST_4(hyperplane(Hyperplane<std::complex<double>, 5>()));
    CALL_SUBTEST_1(lines<float>());
    CALL_SUBTEST_3(lines<double>());
    CALL_SUBTEST_2(planes<float>());
    CALL_SUBTEST_5(planes<double>());
  }
}
