// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2009 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2009 Mathieu Gautier <mathieu.gautier@cea.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "main.h"
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
#include "AnnoyingScalar.h"

template <typename T>
T bounded_acos(T v) {
  using std::acos;
  using std::max;
  using std::min;
  return acos((max)(T(-1), (min)(v, T(1))));
}

template <typename QuatType>
void check_slerp(const QuatType& q0, const QuatType& q1) {
  using std::abs;
  typedef typename QuatType::Scalar Scalar;
  typedef AngleAxis<Scalar> AA;

  Scalar largeEps = test_precision<Scalar>();

  Scalar theta_tot = AA(q1 * q0.inverse()).angle();
  if (theta_tot > Scalar(EIGEN_PI)) theta_tot = Scalar(2.) * Scalar(EIGEN_PI) - theta_tot;
  for (Scalar t = 0; t <= Scalar(1.001); t += Scalar(0.1)) {
    QuatType q = q0.slerp(t, q1);
    Scalar theta = AA(q * q0.inverse()).angle();
    VERIFY(abs(q.norm() - 1) < largeEps);
    if (theta_tot == 0)
      VERIFY(theta < largeEps);
    else
      VERIFY(abs(theta - t * theta_tot) < largeEps);
  }
}

template <typename Scalar, int Options>
void quaternion(void) {
  /* this test covers the following files:
     Quaternion.h
  */
  using std::abs;
  using std::cos;
  using std::sin;
  using std::sqrt;
  typedef Matrix<Scalar, 3, 1> Vector3;
  typedef Matrix<Scalar, 3, 3> Matrix3;
  typedef Quaternion<Scalar, Options> Quaternionx;
  typedef AngleAxis<Scalar> AngleAxisx;

  Scalar largeEps = test_precision<Scalar>();

  Scalar eps = internal::random<Scalar>() * Scalar(1e-2);

  Vector3 v0 = Vector3::Random(), v1 = Vector3::Random(), v2 = Vector3::Random(), v3 = Vector3::Random();

  Scalar a = internal::random<Scalar>(-Scalar(EIGEN_PI), Scalar(EIGEN_PI)),
         b = internal::random<Scalar>(-Scalar(EIGEN_PI), Scalar(EIGEN_PI));

  // Quaternion: Identity(), setIdentity();
  Quaternionx q1, q2;
  q2.setIdentity();
  VERIFY_IS_APPROX(Quaternionx(Quaternionx::Identity()).coeffs(), q2.coeffs());
  q1.coeffs().setRandom();
  VERIFY_IS_APPROX(q1.coeffs(), (q1 * q2).coeffs());

#ifndef EIGEN_NO_IO
  // Printing
  std::ostringstream ss;
  ss << q2;
  VERIFY(ss.str() == "0i + 0j + 0k + 1");
#endif

  // Consistent handling of scalar first/last conventions regardless of Eigen's own coefficient layout
  const Scalar w(a);
  const Vector3 xyz(v0);
  q1 = Quaternionx::FromCoeffsScalarFirst(w, xyz.x(), xyz.y(), xyz.z());
  q2 = Quaternionx::FromCoeffsScalarLast(xyz.x(), xyz.y(), xyz.z(), w);
  VERIFY_IS_EQUAL(q1, q2);

  VERIFY_IS_EQUAL(q1.coeffsScalarFirst()[0], w);
  VERIFY_IS_EQUAL(q1.coeffsScalarFirst()(seqN(1, 3)), xyz);

  VERIFY_IS_EQUAL(q1.coeffsScalarLast()[3], w);
  VERIFY_IS_EQUAL(q1.coeffsScalarLast()(seqN(0, 3)), xyz);

  // concatenation
  q1 *= q2;

  q1 = AngleAxisx(a, v0.normalized());
  q2 = AngleAxisx(a, v1.normalized());

  // angular distance
  Scalar refangle = abs(AngleAxisx(q1.inverse() * q2).angle());
  if (refangle > Scalar(EIGEN_PI)) refangle = Scalar(2) * Scalar(EIGEN_PI) - refangle;

  if ((q1.coeffs() - q2.coeffs()).norm() > Scalar(10) * largeEps) {
    VERIFY_IS_MUCH_SMALLER_THAN(abs(q1.angularDistance(q2) - refangle), Scalar(1));
  }

  // angular distance with non-unit quaternions (scale-invariance)
  {
    Quaternionx qunit(AngleAxisx(Scalar(EIGEN_PI / 2), Vector3::UnitX()));
    Quaternionx qscaled;
    qscaled.coeffs() = qunit.coeffs() * Scalar(2);
    VERIFY_IS_APPROX(qscaled.angularDistance(Quaternionx::Identity()), Scalar(EIGEN_PI / 2));
    VERIFY_IS_APPROX(Quaternionx::Identity().angularDistance(qscaled), Scalar(EIGEN_PI / 2));
  }

  // Action on vector by the q v q* formula
  VERIFY_IS_APPROX(q1 * v2, (q1 * Quaternionx(Scalar(0), v2) * q1.inverse()).vec());
  VERIFY_IS_APPROX(q1.inverse() * v2, (q1.inverse() * Quaternionx(Scalar(0), v2) * q1).vec());

  // rotation matrix conversion
  VERIFY_IS_APPROX(q1 * v2, q1.toRotationMatrix() * v2);
  VERIFY_IS_APPROX(q1 * q2 * v2, q1.toRotationMatrix() * q2.toRotationMatrix() * v2);

  VERIFY((q2 * q1).isApprox(q1 * q2, largeEps) ||
         !(q2 * q1 * v2).isApprox(q1.toRotationMatrix() * q2.toRotationMatrix() * v2));

  q2 = q1.toRotationMatrix();
  VERIFY_IS_APPROX(q1 * v1, q2 * v1);

  Matrix3 rot1(q1);
  VERIFY_IS_APPROX(q1 * v1, rot1 * v1);
  Quaternionx q3(rot1.transpose() * rot1);
  VERIFY_IS_APPROX(q3 * v1, v1);

  // angle-axis conversion
  AngleAxisx aa = AngleAxisx(q1);
  VERIFY_IS_APPROX(q1 * v1, Quaternionx(aa) * v1);

  // Do not execute the test if the rotation angle is almost zero, or
  // the rotation axis and v1 are almost parallel.
  if (abs(aa.angle()) > Scalar(5) * test_precision<Scalar>() && (aa.axis() - v1.normalized()).norm() < Scalar(1.99) &&
      (aa.axis() + v1.normalized()).norm() < Scalar(1.99)) {
    VERIFY_IS_NOT_APPROX(q1 * v1, Quaternionx(AngleAxisx(aa.angle() * 2, aa.axis())) * v1);
  }

  // from two vector creation
  VERIFY_IS_APPROX(v2.normalized(), (q2.setFromTwoVectors(v1, v2) * v1).normalized());
  VERIFY_IS_APPROX(v1.normalized(), (q2.setFromTwoVectors(v1, v1) * v1).normalized());
  VERIFY_IS_APPROX(-v1.normalized(), (q2.setFromTwoVectors(v1, -v1) * v1).normalized());
  if (std::is_same<Scalar, double>::value) {
    v3 = (v1.array() + eps).matrix();
    VERIFY_IS_APPROX(v3.normalized(), (q2.setFromTwoVectors(v1, v3) * v1).normalized());
    VERIFY_IS_APPROX(-v3.normalized(), (q2.setFromTwoVectors(v1, -v3) * v1).normalized());
  }

  // from two vector creation static function
  VERIFY_IS_APPROX(v2.normalized(), (Quaternionx::FromTwoVectors(v1, v2) * v1).normalized());
  VERIFY_IS_APPROX(v1.normalized(), (Quaternionx::FromTwoVectors(v1, v1) * v1).normalized());
  VERIFY_IS_APPROX(-v1.normalized(), (Quaternionx::FromTwoVectors(v1, -v1) * v1).normalized());
  if (std::is_same<Scalar, double>::value) {
    v3 = (v1.array() + eps).matrix();
    VERIFY_IS_APPROX(v3.normalized(), (Quaternionx::FromTwoVectors(v1, v3) * v1).normalized());
    VERIFY_IS_APPROX(-v3.normalized(), (Quaternionx::FromTwoVectors(v1, -v3) * v1).normalized());
  }

  // inverse and conjugate
  VERIFY_IS_APPROX(q1 * (q1.inverse() * v1), v1);
  VERIFY_IS_APPROX(q1 * (q1.conjugate() * v1), v1);

  // test casting
  Quaternion<float> q1f = q1.template cast<float>();
  VERIFY_IS_APPROX(q1f.template cast<Scalar>(), q1);
  Quaternion<double> q1d = q1.template cast<double>();
  VERIFY_IS_APPROX(q1d.template cast<Scalar>(), q1);

  // test bug 369 - improper alignment.
  Quaternionx* q = new Quaternionx;
  delete q;

  // ---- setFromScaledAxis / toScaledAxis (SO(3) exp/log) ----
  // Identity round-trip.
  VERIFY_IS_APPROX(Quaternionx::FromScaledAxis(Vector3::Zero()).coeffs(), Quaternionx::Identity().coeffs());
  VERIFY_IS_EQUAL(Quaternionx::Identity().toScaledAxis(), Vector3::Zero());

  // Canonical axis rotations: 90/180/60 degrees.
  {
    Vector3 sa = Vector3::UnitX() * Scalar(EIGEN_PI / 2);
    Scalar c = sqrt(Scalar(2)) * Scalar(0.5);
    VERIFY_IS_APPROX(Quaternionx::FromScaledAxis(sa), Quaternionx(c, c, Scalar(0), Scalar(0)));
    VERIFY_IS_APPROX(Quaternionx::FromScaledAxis(sa), Quaternionx(AngleAxisx(Scalar(EIGEN_PI / 2), Vector3::UnitX())));
  }
  VERIFY_IS_APPROX(Quaternionx(Scalar(0), Scalar(0), Scalar(1), Scalar(0)).toScaledAxis(),
                   Vector3(Scalar(0), Scalar(EIGEN_PI), Scalar(0)));
  VERIFY_IS_APPROX(Quaternionx(sqrt(Scalar(3)) / Scalar(2), Scalar(0), Scalar(0), Scalar(0.5)).toScaledAxis(),
                   Scalar(EIGEN_PI / 3) * Vector3::UnitZ());

  // Small but normal angle: agrees with the closed-form half-angle quaternion.
  {
    Scalar theta = Scalar(1e-2);
    Vector3 sa = Vector3::UnitX() * theta;
    VERIFY_IS_APPROX(Quaternionx::FromScaledAxis(sa),
                     Quaternionx(cos(theta / Scalar(2)), sin(theta / Scalar(2)), Scalar(0), Scalar(0)));
    VERIFY_IS_APPROX(Quaternionx(AngleAxisx(theta, Vector3::UnitX())).toScaledAxis(), sa);
  }

  // Tiny angle (sqrt of the smallest normal): the squared-norm formulation would have
  // flushed to zero here, so this exercises the stableNorm() path. The closed form is
  // still used (no Taylor expansion) and must agree with the math.
  {
    const Scalar theta_base = (numext::numeric_limits<Scalar>::min)();
    const Scalar theta = sqrt(theta_base);
    Vector3 sa = Vector3::UnitX() * theta;
    VERIFY_IS_APPROX(Quaternionx::FromScaledAxis(sa),
                     Quaternionx(cos(theta / Scalar(2)), sin(theta / Scalar(2)), Scalar(0), Scalar(0)));
    VERIFY_IS_APPROX(Quaternionx(AngleAxisx(theta, Vector3::UnitX())).toScaledAxis(), sa);
  }

  // Exact-zero handling: identity quaternion out / zero vector out, no NaN.
  {
    Quaternionx qid = Quaternionx::FromScaledAxis(Vector3::Zero());
    VERIFY_IS_EQUAL(qid.w(), Scalar(1));
    VERIFY_IS_EQUAL(qid.vec(), Vector3::Zero());
    VERIFY_IS_EQUAL(Quaternionx::Identity().toScaledAxis(), Vector3::Zero());
    // -1 quaternion (same rotation as identity) also maps to zero.
    Quaternionx qneg(Scalar(-1), Scalar(0), Scalar(0), Scalar(0));
    VERIFY_IS_EQUAL(qneg.toScaledAxis(), Vector3::Zero());
  }

  // Negative-w quaternion: the canonical magnitude of the rotation vector is in [0, pi].
  {
    Vector3 axis = Vector3::UnitX();
    Quaternionx q_pos(AngleAxisx(Scalar(EIGEN_PI / 3), axis));
    Quaternionx q_neg(-q_pos.w(), -q_pos.x(), -q_pos.y(), -q_pos.z());
    Vector3 sa_pos = q_pos.toScaledAxis();
    Vector3 sa_neg = q_neg.toScaledAxis();
    VERIFY(sa_pos.norm() <= Scalar(EIGEN_PI) + test_precision<Scalar>());
    VERIFY(sa_neg.norm() <= Scalar(EIGEN_PI) + test_precision<Scalar>());
    // Both quaternions represent the same rotation, so re-applying exp gives back q_pos
    // up to sign.
    Quaternionx q_round = Quaternionx::FromScaledAxis(sa_neg);
    VERIFY_IS_APPROX(numext::abs(q_round.dot(q_pos)), Scalar(1));
  }

  // Near-pi rotations: sin(theta/2) is near 1 and cos(theta/2) is near 0, so the
  // canonicalisation branch (the abs(w)/(w<0) split) is stress-tested. Stay in [0, pi]:
  // for theta > pi the canonical answer is (2*pi - theta) * (-axis), which is an
  // important behaviour but is covered by the "Negative-w quaternion" test.
  {
    Vector3 axis = Vector3(Scalar(1), Scalar(2), Scalar(3)).normalized();
    for (Scalar gap : {Scalar(1e-3), Scalar(1e-6)}) {
      Scalar theta = Scalar(EIGEN_PI) - gap;
      Quaternionx near_pi_quat(AngleAxisx(theta, axis));
      Vector3 sa = near_pi_quat.toScaledAxis();
      VERIFY_IS_APPROX(sa.stableNorm(), theta);
      VERIFY_IS_APPROX(Quaternionx::FromScaledAxis(sa), near_pi_quat);
    }
  }

  // w == -0.0 corner case (and its +0.0 counterpart).
  {
    // (w=+0, x=0, y=0, z=1) and (w=-0, ...) both represent rotation by pi about z.
    Quaternionx qp(Scalar(0), Scalar(0), Scalar(0), Scalar(1));
    Quaternionx qn(-Scalar(0), Scalar(0), Scalar(0), Scalar(1));
    Vector3 sap = qp.toScaledAxis();
    Vector3 san = qn.toScaledAxis();
    VERIFY_IS_APPROX(sap.stableNorm(), Scalar(EIGEN_PI));
    VERIFY_IS_APPROX(san.stableNorm(), Scalar(EIGEN_PI));
    VERIFY_IS_APPROX(Quaternionx::FromScaledAxis(sap), qp);
    // qp and qn represent the same rotation, so the recovered quaternions must agree up
    // to sign.
    VERIFY_IS_APPROX(numext::abs(Quaternionx::FromScaledAxis(san).dot(qp)), Scalar(1));
  }

  // Round-trip across the full canonical range. Includes near-pi by construction.
  {
    static const int kNumOctants = 8;
    const Scalar angle_base = internal::random<Scalar>(Scalar(0), Scalar(EIGEN_PI));
    for (int octant = 0; octant < kNumOctants; ++octant) {
      // Sweep through magnitude as well, including the antipodal boundary at pi.
      Scalar angle =
          angle_base * Scalar(octant) / Scalar(kNumOctants - 1);  // 0 .. angle_base, hits pi when angle_base = pi
      Scalar theta = -Scalar(EIGEN_PI) + Scalar(2 * EIGEN_PI) * Scalar(octant) / Scalar(kNumOctants);
      Scalar phi = Scalar(EIGEN_PI) * Scalar(octant + 1) / Scalar(kNumOctants + 1);
      Vector3 sa(angle * sin(phi) * cos(theta), angle * sin(phi) * sin(theta), angle * cos(phi));
      Quaternionx roundtrip_quat = Quaternionx::FromScaledAxis(sa);
      VERIFY_IS_APPROX(roundtrip_quat.norm(), Scalar(1));
      Vector3 sa_rt = roundtrip_quat.toScaledAxis();
      VERIFY_IS_APPROX(sa_rt, sa);
      // And the other direction (quaternion -> sa -> quaternion).
      Quaternionx roundtrip_quat2 = Quaternionx::FromScaledAxis(sa_rt);
      VERIFY_IS_APPROX(roundtrip_quat2, roundtrip_quat);
    }
  }

  q1 = Quaternionx::UnitRandom();
  q2 = Quaternionx::UnitRandom();
  check_slerp(q1, q2);

  q1 = AngleAxisx(b, v1.normalized());
  q2 = AngleAxisx(b + Scalar(EIGEN_PI), v1.normalized());
  check_slerp(q1, q2);

  q1 = AngleAxisx(b, v1.normalized());
  q2 = AngleAxisx(-b, -v1.normalized());
  check_slerp(q1, q2);

  q1 = Quaternionx::UnitRandom();
  q2.coeffs() = -q1.coeffs();
  check_slerp(q1, q2);
}

template <typename Scalar>
void mapQuaternion(void) {
  typedef Map<Quaternion<Scalar>, Aligned> MQuaternionA;
  typedef Map<const Quaternion<Scalar>, Aligned> MCQuaternionA;
  typedef Map<Quaternion<Scalar> > MQuaternionUA;
  typedef Map<const Quaternion<Scalar> > MCQuaternionUA;
  typedef Quaternion<Scalar> Quaternionx;
  typedef Matrix<Scalar, 3, 1> Vector3;
  typedef AngleAxis<Scalar> AngleAxisx;

  Vector3 v0 = Vector3::Random(), v1 = Vector3::Random();
  Scalar a = internal::random<Scalar>(-Scalar(EIGEN_PI), Scalar(EIGEN_PI));

  EIGEN_ALIGN_TO_BOUNDARY(Aligned) Scalar array1[4];
  EIGEN_ALIGN_TO_BOUNDARY(Aligned) Scalar array2[4];
  EIGEN_ALIGN_MAX Scalar array3[4 + 1];
  Scalar* array3unaligned = array3 + 1;

  MQuaternionA mq1(array1);
  MCQuaternionA mcq1(array1);
  MQuaternionA mq2(array2);
  MQuaternionUA mq3(array3unaligned);
  MCQuaternionUA mcq3(array3unaligned);

  //  std::cerr << array1 << " " << array2 << " " << array3 << "\n";
  mq1 = AngleAxisx(a, v0.normalized());
  mq2 = mq1;
  mq3 = mq1;

  Quaternionx q1 = mq1;
  Quaternionx q2 = mq2;
  Quaternionx q3 = mq3;
  Quaternionx q4 = MCQuaternionUA(array3unaligned);

  VERIFY_IS_APPROX(q1.coeffs(), q2.coeffs());
  VERIFY_IS_APPROX(q1.coeffs(), q3.coeffs());
  VERIFY_IS_APPROX(q4.coeffs(), q3.coeffs());

  VERIFY_IS_APPROX(mq1 * (mq1.inverse() * v1), v1);
  VERIFY_IS_APPROX(mq1 * (mq1.conjugate() * v1), v1);

  VERIFY_IS_APPROX(mcq1 * (mcq1.inverse() * v1), v1);
  VERIFY_IS_APPROX(mcq1 * (mcq1.conjugate() * v1), v1);

  VERIFY_IS_APPROX(mq3 * (mq3.inverse() * v1), v1);
  VERIFY_IS_APPROX(mq3 * (mq3.conjugate() * v1), v1);

  VERIFY_IS_APPROX(mcq3 * (mcq3.inverse() * v1), v1);
  VERIFY_IS_APPROX(mcq3 * (mcq3.conjugate() * v1), v1);

  VERIFY_IS_APPROX(mq1 * mq2, q1 * q2);
  VERIFY_IS_APPROX(mq3 * mq2, q3 * q2);
  VERIFY_IS_APPROX(mcq1 * mq2, q1 * q2);
  VERIFY_IS_APPROX(mcq3 * mq2, q3 * q2);

  // Bug 1461, compilation issue with Map<const Quat>::w(), and other reference/constness checks:
  VERIFY_IS_APPROX(mcq3.coeffs().x() + mcq3.coeffs().y() + mcq3.coeffs().z() + mcq3.coeffs().w(), mcq3.coeffs().sum());
  VERIFY_IS_APPROX(mcq3.x() + mcq3.y() + mcq3.z() + mcq3.w(), mcq3.coeffs().sum());
  mq3.w() = 1;
  const Quaternionx& cq3(q3);
  VERIFY(&cq3.x() == &q3.x());
  const MQuaternionUA& cmq3(mq3);
  VERIFY(&cmq3.x() == &mq3.x());
  // FIXME the following should be ok. The problem is that currently the LValueBit flag
  // is used to determine whether we can return a coeff by reference or not, which is not enough for Map<const ...>.
  // const MCQuaternionUA& cmcq3(mcq3);
  // VERIFY( &cmcq3.x() == &mcq3.x() );

  // test cast
  {
    Quaternion<float> q1f = mq1.template cast<float>();
    VERIFY_IS_APPROX(q1f.template cast<Scalar>(), mq1);
    Quaternion<double> q1d = mq1.template cast<double>();
    VERIFY_IS_APPROX(q1d.template cast<Scalar>(), mq1);
  }
}

template <typename Scalar>
void quaternionAlignment(void) {
  typedef Quaternion<Scalar, AutoAlign> QuaternionA;
  typedef Quaternion<Scalar, DontAlign> QuaternionUA;

  EIGEN_ALIGN_MAX Scalar array1[4];
  EIGEN_ALIGN_MAX Scalar array2[4];
  EIGEN_ALIGN_MAX Scalar array3[4 + 1];
  Scalar* arrayunaligned = array3 + 1;

  QuaternionA* q1 = ::new (reinterpret_cast<void*>(array1)) QuaternionA;
  QuaternionUA* q2 = ::new (reinterpret_cast<void*>(array2)) QuaternionUA;
  QuaternionUA* q3 = ::new (reinterpret_cast<void*>(arrayunaligned)) QuaternionUA;

  q1->coeffs().setRandom();
  *q2 = *q1;
  *q3 = *q1;

  VERIFY_IS_APPROX(q1->coeffs(), q2->coeffs());
  VERIFY_IS_APPROX(q1->coeffs(), q3->coeffs());
}

template <typename PlainObjectType>
void check_const_correctness(const PlainObjectType&) {
  // there's a lot that we can't test here while still having this test compile!
  // the only possible approach would be to run a script trying to compile stuff and checking that it fails.
  // CMake can help with that.

  // verify that map-to-const don't have LvalueBit
  typedef std::add_const_t<PlainObjectType> ConstPlainObjectType;
  VERIFY(!(internal::traits<Map<ConstPlainObjectType> >::Flags & LvalueBit));
  VERIFY(!(internal::traits<Map<ConstPlainObjectType, Aligned> >::Flags & LvalueBit));
  VERIFY(!(Map<ConstPlainObjectType>::Flags & LvalueBit));
  VERIFY(!(Map<ConstPlainObjectType, Aligned>::Flags & LvalueBit));
}

// Regression for bug 1573
struct MovableClass {
  static_assert(std::is_nothrow_move_constructible<Quaternionf>::value, "");
  MovableClass() = default;
  MovableClass(const MovableClass&) = default;
  MovableClass(MovableClass&&) noexcept = default;
  MovableClass& operator=(const MovableClass&) = default;
  MovableClass& operator=(MovableClass&&) = default;
  Quaternionf m_quat;
};

EIGEN_DECLARE_TEST(geo_quaternion) {
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1((quaternion<float, AutoAlign>()));
    CALL_SUBTEST_1(check_const_correctness(Quaternionf()));
    CALL_SUBTEST_1((quaternion<float, DontAlign>()));
    CALL_SUBTEST_1((quaternionAlignment<float>()));
    CALL_SUBTEST_1(mapQuaternion<float>());

    CALL_SUBTEST_2((quaternion<double, AutoAlign>()));
    CALL_SUBTEST_2(check_const_correctness(Quaterniond()));
    CALL_SUBTEST_2((quaternion<double, DontAlign>()));
    CALL_SUBTEST_2((quaternionAlignment<double>()));
    CALL_SUBTEST_2(mapQuaternion<double>());

#ifndef EIGEN_TEST_ANNOYING_SCALAR_DONT_THROW
    AnnoyingScalar::dont_throw = true;
#endif
    CALL_SUBTEST_3((quaternion<AnnoyingScalar, AutoAlign>()));
  }
}
