// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2025 Rasmus Munk Larsen <rmlarsen@google.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// Tests for Geometry module classes that lack dedicated test coverage:
//   - AngleAxis edge cases
//   - Rotation2D standalone tests
//   - Translation standalone tests
//   - Scaling standalone tests
//   - Quaternion fromRotationMatrix with non-orthogonal input
//   - Quaternion slerp edge cases
//   - Umeyama degenerate input
//   - Transform shear ground truth

#include "main.h"
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

template <typename Scalar>
void angleaxis_edge_cases() {
  typedef Matrix<Scalar, 3, 1> Vector3;
  typedef Matrix<Scalar, 3, 3> Matrix3;
  typedef AngleAxis<Scalar> AngleAxisx;
  typedef Quaternion<Scalar> Quaternionx;

  Scalar eps = test_precision<Scalar>();

  // Identity rotation (angle = 0)
  {
    AngleAxisx aa(Scalar(0), Vector3::UnitZ());
    Matrix3 m = aa.toRotationMatrix();
    VERIFY_IS_APPROX(m, Matrix3::Identity());
    // Verify orthogonality: R^T * R = I
    VERIFY_IS_APPROX(m.transpose() * m, Matrix3::Identity());
    VERIFY_IS_APPROX(m.determinant(), Scalar(1));
    Quaternionx q(aa);
    VERIFY_IS_APPROX(q.toRotationMatrix(), Matrix3::Identity());
    VERIFY_IS_APPROX(q.norm(), Scalar(1));
  }

  // 180-degree rotation around each axis
  {
    for (int axis = 0; axis < 3; ++axis) {
      Vector3 ax = Vector3::Zero();
      ax(axis) = Scalar(1);
      AngleAxisx aa(Scalar(EIGEN_PI), ax);
      Matrix3 m = aa.toRotationMatrix();
      // Should be its own inverse
      VERIFY_IS_APPROX(m * m, Matrix3::Identity());
      // Verify orthogonality
      VERIFY_IS_APPROX(m.transpose() * m, Matrix3::Identity());
      VERIFY_IS_APPROX(m.determinant(), Scalar(1));
      // Diagonal entry for the rotation axis should be +1, others -1
      VERIFY_IS_APPROX(m(axis, axis), Scalar(1));
      for (int j = 0; j < 3; ++j) {
        if (j != axis) VERIFY_IS_APPROX(m(j, j), Scalar(-1));
      }
    }
  }

  // Very small angle: verify Rodrigues formula accuracy
  {
    Scalar tiny = Scalar(10) * NumTraits<Scalar>::epsilon();
    AngleAxisx aa(tiny, Vector3::UnitX());
    Matrix3 m = aa.toRotationMatrix();
    VERIFY((m - Matrix3::Identity()).norm() < Scalar(4) * tiny);
    // Orthogonality should hold to machine precision even for tiny angles
    VERIFY_IS_APPROX(m.transpose() * m, Matrix3::Identity());
    VERIFY_IS_APPROX(m.determinant(), Scalar(1));
    // Round-trip through quaternion
    Quaternionx q(aa);
    AngleAxisx aa2(q);
    VERIFY(numext::abs(aa2.angle()) < Scalar(4) * tiny);
  }

  // Round-trip: AngleAxis -> Matrix -> AngleAxis
  {
    Scalar a = internal::random<Scalar>(Scalar(0.1), Scalar(EIGEN_PI) - Scalar(0.1));
    Vector3 axis = Vector3::Random().normalized();
    AngleAxisx aa(a, axis);
    Matrix3 m = aa.toRotationMatrix();
    VERIFY_IS_APPROX(m.transpose() * m, Matrix3::Identity());
    VERIFY_IS_APPROX(m.determinant(), Scalar(1));
    AngleAxisx aa2;
    aa2.fromRotationMatrix(m);
    VERIFY_IS_APPROX(aa.toRotationMatrix(), aa2.toRotationMatrix());
    // Verify angle is preserved
    VERIFY(numext::abs(aa2.angle() - a) < eps);
  }

  // Round-trip for near-180-degree rotation
  {
    Scalar a = Scalar(EIGEN_PI) - Scalar(10) * NumTraits<Scalar>::epsilon();
    Vector3 axis = Vector3::Random().normalized();
    AngleAxisx aa(a, axis);
    Matrix3 m = aa.toRotationMatrix();
    VERIFY_IS_APPROX(m.transpose() * m, Matrix3::Identity());
    VERIFY_IS_APPROX(m.determinant(), Scalar(1));
    AngleAxisx aa2(m);
    // The rotation should be equivalent even if axis flips sign
    VERIFY_IS_APPROX(aa.toRotationMatrix(), aa2.toRotationMatrix());
  }

  // Accessors
  {
    Vector3 axis = Vector3::Random().normalized();
    Scalar angle = internal::random<Scalar>(Scalar(-EIGEN_PI), Scalar(EIGEN_PI));
    AngleAxisx aa(angle, axis);
    VERIFY_IS_APPROX(aa.angle(), angle);
    VERIFY_IS_APPROX(aa.axis(), axis);

    // Mutable accessors
    aa.angle() = Scalar(1);
    aa.axis() = Vector3::UnitZ();
    VERIFY_IS_APPROX(aa.angle(), Scalar(1));
    VERIFY_IS_APPROX(aa.axis(), Vector3::UnitZ());
  }

  // inverse
  {
    Scalar a = internal::random<Scalar>(Scalar(0.1), Scalar(EIGEN_PI));
    Vector3 axis = Vector3::Random().normalized();
    AngleAxisx aa(a, axis);
    VERIFY_IS_APPROX((aa * aa.inverse()).toRotationMatrix(), Matrix3::Identity());
  }

  // isApprox
  {
    Scalar a = internal::random<Scalar>(Scalar(0.1), Scalar(EIGEN_PI));
    Vector3 axis = Vector3::Random().normalized();
    AngleAxisx aa1(a, axis);
    AngleAxisx aa2(a + eps / Scalar(10), axis);
    VERIFY(aa1.isApprox(aa2, eps));
  }

  // Quaternion round-trip: AngleAxis -> Quaternion -> AngleAxis
  {
    Scalar a = internal::random<Scalar>(Scalar(0.1), Scalar(EIGEN_PI) - Scalar(0.1));
    Vector3 axis = Vector3::Random().normalized();
    AngleAxisx aa(a, axis);
    Quaternionx q(aa);
    VERIFY_IS_APPROX(q.norm(), Scalar(1));
    AngleAxisx aa2(q);
    VERIFY(numext::abs(aa2.angle() - a) < eps);
    // Axis should agree (possibly with sign flip for angle > pi)
    VERIFY(aa2.axis().isApprox(axis, eps) || aa2.axis().isApprox(-axis, eps));
  }
}

template <typename Scalar>
void rotation2d_standalone() {
  typedef Matrix<Scalar, 2, 1> Vector2;
  typedef Matrix<Scalar, 2, 2> Matrix2;
  typedef Rotation2D<Scalar> Rotation2Dx;

  Scalar eps = test_precision<Scalar>();

  // Construction and basic properties
  {
    Rotation2Dx r(Scalar(0));
    VERIFY_IS_APPROX(r.toRotationMatrix(), Matrix2::Identity());
  }

  // 90-degree rotation
  {
    Rotation2Dx r(Scalar(EIGEN_PI) / Scalar(2));
    Vector2 v(1, 0);
    Vector2 rv = r * v;
    VERIFY_IS_APPROX(rv, Vector2(0, 1));
  }

  // 180-degree rotation
  {
    Rotation2Dx r(Scalar(EIGEN_PI));
    Vector2 v(1, 0);
    Vector2 rv = r * v;
    VERIFY_IS_APPROX(rv, Vector2(-1, 0));
  }

  // inverse
  {
    Scalar a = internal::random<Scalar>(-Scalar(EIGEN_PI), Scalar(EIGEN_PI));
    Rotation2Dx r(a);
    VERIFY_IS_APPROX((r * r.inverse()).toRotationMatrix(), Matrix2::Identity());
  }

  // composition
  {
    Scalar a = internal::random<Scalar>(-Scalar(EIGEN_PI), Scalar(EIGEN_PI));
    Scalar b = internal::random<Scalar>(-Scalar(EIGEN_PI), Scalar(EIGEN_PI));
    Rotation2Dx ra(a), rb(b);
    VERIFY_IS_APPROX((ra * rb).toRotationMatrix(), ra.toRotationMatrix() * rb.toRotationMatrix());
  }

  // fromRotationMatrix round-trip: angle should agree to tight tolerance
  {
    Scalar a = internal::random<Scalar>(-Scalar(EIGEN_PI), Scalar(EIGEN_PI));
    Rotation2Dx r(a);
    Matrix2 m = r.toRotationMatrix();
    // Verify orthogonality
    VERIFY_IS_APPROX(m.transpose() * m, Matrix2::Identity());
    VERIFY_IS_APPROX(m.determinant(), Scalar(1));
    Rotation2Dx r2(m);
    // Angles should agree mod 2*pi. Use rotation matrices for exact comparison.
    VERIFY_IS_APPROX(r2.toRotationMatrix(), r.toRotationMatrix());
  }

  // slerp at t=0.5 of inverse should give ~0 or ~pi
  {
    Scalar a = internal::random<Scalar>(Scalar(0.1), Scalar(EIGEN_PI) / Scalar(2));
    Rotation2Dx r(a);
    Rotation2Dx half = r.slerp(Scalar(0.5), r.inverse());
    // The midpoint between a rotation and its inverse via shortest path should be ~0 or ~pi
    Scalar halfAngle = half.smallestAngle();
    VERIFY(numext::abs(halfAngle) < eps || numext::abs(numext::abs(halfAngle) - Scalar(EIGEN_PI)) < eps);
  }

  // Rotation2D slerp interpolation: verify linearity of angle
  {
    Scalar a0 = internal::random<Scalar>(Scalar(-EIGEN_PI), Scalar(EIGEN_PI));
    Scalar a1 = a0 + internal::random<Scalar>(Scalar(0.1), Scalar(1.0));
    Rotation2Dx r0(a0), r1(a1);
    Scalar t = Scalar(0.3);
    Rotation2Dx rt = r0.slerp(t, r1);
    // Slerp for 2D rotations is just linear interpolation of angle along shortest path.
    Scalar expected = a0 + t * Rotation2Dx(a1 - a0).smallestAngle();
    VERIFY(numext::abs(Rotation2Dx(rt.angle() - expected).smallestAngle()) < eps);
  }

  // smallestAngle range
  for (int k = 0; k < 100; ++k) {
    Scalar a = internal::random<Scalar>(Scalar(-100), Scalar(100));
    Rotation2Dx r(a);
    VERIFY(r.smallestAngle() >= -Scalar(EIGEN_PI));
    VERIFY(r.smallestAngle() <= Scalar(EIGEN_PI));
    VERIFY(r.smallestPositiveAngle() >= Scalar(0));
    VERIFY(r.smallestPositiveAngle() <= Scalar(2) * Scalar(EIGEN_PI));
  }
}

template <typename Scalar>
void translation_standalone() {
  typedef Matrix<Scalar, 3, 1> Vector3;
  typedef Translation<Scalar, 3> Translation3;
  typedef Transform<Scalar, 3, Isometry> Isometry3;

  // Construction
  {
    Vector3 v(1, 2, 3);
    Translation3 t(v);
    VERIFY_IS_APPROX(t.vector(), v);

    Translation3 t2(Scalar(1), Scalar(2), Scalar(3));
    VERIFY_IS_APPROX(t.vector(), t2.vector());
  }

  // Identity
  {
    Translation3 t = Translation3::Identity();
    VERIFY_IS_APPROX(t.vector(), Vector3::Zero());
  }

  // Composition
  {
    Vector3 v1(1, 2, 3), v2(4, 5, 6);
    Translation3 t1(v1), t2(v2);
    Translation3 t3 = t1 * t2;
    VERIFY_IS_APPROX(t3.vector(), v1 + v2);
  }

  // Inverse
  {
    Vector3 v(1, 2, 3);
    Translation3 t(v);
    Translation3 tinv = t.inverse();
    VERIFY_IS_APPROX(tinv.vector(), -v);
    VERIFY_IS_APPROX((t * tinv).vector(), Vector3::Zero());
  }

  // Translation * vector (via transform)
  {
    Vector3 v(1, 2, 3), p(10, 20, 30);
    Translation3 t(v);
    VERIFY_IS_APPROX(t * p, v + p);
  }

  // Casting
  {
    Translation3 t(Vector3(1, 2, 3));
    Translation<float, 3> tf = t.template cast<float>();
    VERIFY_IS_APPROX(tf.template cast<Scalar>().vector(), t.vector());
  }

  // Convert to Isometry
  {
    Vector3 v(1, 2, 3);
    Translation3 t(v);
    Isometry3 iso(t);
    VERIFY_IS_APPROX(iso.translation(), v);
    typedef Matrix<Scalar, 3, 3> Matrix3;
    VERIFY_IS_APPROX(iso.linear(), Matrix3::Identity());
  }
}

template <typename Scalar>
void scaling_standalone() {
  typedef Matrix<Scalar, 3, 1> Vector3;
  typedef Translation<Scalar, 3> Translation3;
  typedef Transform<Scalar, 3, Affine> Affine3;
  typedef UniformScaling<Scalar> UniformScalingx;

  // UniformScaling construction
  {
    UniformScalingx s(Scalar(2));
    VERIFY_IS_APPROX(s.factor(), Scalar(2));
  }

  // UniformScaling * vector
  {
    UniformScalingx s(Scalar(3));
    Vector3 v(1, 2, 3);
    // UniformScaling * Translation
    Translation3 t(v);
    Affine3 result = s * t;
    VERIFY_IS_APPROX(result.translation(), v * Scalar(3));
  }

  // UniformScaling composition
  {
    UniformScalingx s1(Scalar(2)), s2(Scalar(3));
    UniformScalingx s3 = s1 * s2;
    VERIFY_IS_APPROX(s3.factor(), Scalar(6));
  }

  // UniformScaling inverse
  {
    UniformScalingx s(Scalar(4));
    UniformScalingx si = s.inverse();
    VERIFY_IS_APPROX(si.factor(), Scalar(1) / Scalar(4));
    VERIFY_IS_APPROX((s * si).factor(), Scalar(1));
  }

  // Scaling convenience functions
  {
    Scalar s = Scalar(3);
    auto su = Eigen::Scaling(s);
    VERIFY_IS_APPROX(su.factor(), s);

    Vector3 sv(1, 2, 3);
    auto sd = Eigen::Scaling(sv);
    VERIFY_IS_APPROX(Vector3(sd.diagonal()), sv);
  }
}

template <typename Scalar>
void quaternion_from_non_orthogonal_matrix() {
  typedef Matrix<Scalar, 3, 3> Matrix3;
  typedef Quaternion<Scalar> Quaternionx;

  // Slightly non-orthogonal matrix should not produce NaN
  {
    Matrix3 m = Matrix3::Identity();
    // Perturb to make slightly non-orthogonal
    m(0, 0) += Scalar(1e-6);
    m(1, 1) -= Scalar(1e-6);
    Quaternionx q(m);
    // Should not be NaN
    VERIFY(!(numext::isnan)(q.w()));
    VERIFY(!(numext::isnan)(q.x()));
    VERIFY(!(numext::isnan)(q.y()));
    VERIFY(!(numext::isnan)(q.z()));
    // Should still be approximately a rotation
    VERIFY(numext::abs(q.norm() - Scalar(1)) < Scalar(0.01));
  }

  // Negative trace case with non-orthogonal matrix
  {
    // 180-degree rotation around Z axis, slightly perturbed
    Matrix3 m;
    m << Scalar(-1), Scalar(0), Scalar(0), Scalar(0), Scalar(-1), Scalar(0), Scalar(0), Scalar(0), Scalar(1);
    m(0, 0) += Scalar(1e-6);
    Quaternionx q(m);
    VERIFY(!(numext::isnan)(q.w()));
    VERIFY(!(numext::isnan)(q.x()));
    VERIFY(!(numext::isnan)(q.y()));
    VERIFY(!(numext::isnan)(q.z()));
  }

  // Matrix with trace very close to -1 (worst case for positive trace branch)
  {
    Matrix3 m;
    m << Scalar(-1), Scalar(0), Scalar(0), Scalar(0), Scalar(-0.5), Scalar(0), Scalar(0), Scalar(0), Scalar(-0.5);
    // trace = -2, negative trace path
    Quaternionx q(m);
    VERIFY(!(numext::isnan)(q.w()));
    VERIFY(!(numext::isnan)(q.x()));
  }

  // Verify graceful handling of arbitrary non-rotation matrices.
  // Shoemake's argmax-diagonal selection makes the sqrt argument non-negative
  // for any real matrix, but the numext::maxi guard provides additional safety.
  {
    Matrix3 m;
    m << Scalar(-0.4), Scalar(0.5), Scalar(0), Scalar(-0.5), Scalar(-0.4), Scalar(0), Scalar(0), Scalar(0),
        Scalar(-0.4);
    Quaternionx q(m);
    VERIFY(!(numext::isnan)(q.w()));
    VERIFY(!(numext::isnan)(q.x()));
    VERIFY(!(numext::isnan)(q.y()));
    VERIFY(!(numext::isnan)(q.z()));
  }
}

template <typename Scalar>
void quaternion_slerp_edge_cases() {
  typedef Quaternion<Scalar> Quaternionx;
  typedef AngleAxis<Scalar> AngleAxisx;
  typedef Matrix<Scalar, 3, 1> Vector3;
  typedef Matrix<Scalar, 3, 3> Matrix3;

  // The slerp formula preserves unit norm to O(epsilon^2) for unit inputs.
  Scalar tight = Scalar(32) * NumTraits<Scalar>::epsilon();

  // slerp with identical quaternions
  {
    Quaternionx q = Quaternionx::UnitRandom();
    Quaternionx r = q.slerp(Scalar(0.5), q);
    VERIFY_IS_APPROX(r.coeffs(), q.coeffs());
    VERIFY(numext::abs(r.norm() - Scalar(1)) < tight);
  }

  // slerp at t=0 and t=1
  {
    Quaternionx q0 = Quaternionx::UnitRandom();
    Quaternionx q1 = Quaternionx::UnitRandom();
    Quaternionx r0 = q0.slerp(Scalar(0), q1);
    Quaternionx r1 = q0.slerp(Scalar(1), q1);
    VERIFY(numext::abs(r0.norm() - Scalar(1)) < tight);
    VERIFY(numext::abs(r1.norm() - Scalar(1)) < tight);
    VERIFY_IS_APPROX(r0.toRotationMatrix(), q0.toRotationMatrix());
    VERIFY_IS_APPROX(r1.toRotationMatrix(), q1.toRotationMatrix());
  }

  // slerp with antipodal quaternions (represent same rotation)
  {
    Quaternionx q = Quaternionx::UnitRandom();
    Quaternionx qn;
    qn.coeffs() = -q.coeffs();
    for (Scalar t = 0; t <= Scalar(1.001); t += Scalar(0.25)) {
      Quaternionx r = q.slerp(t, qn);
      VERIFY(numext::abs(r.norm() - Scalar(1)) < tight);
      // Should stay at the same rotation
      VERIFY_IS_APPROX(r.toRotationMatrix(), q.toRotationMatrix());
    }
  }

  // slerp with 180-degree apart quaternions
  {
    Vector3 axis1 = Vector3::UnitX();
    Quaternionx q0(AngleAxisx(Scalar(0), axis1));
    Quaternionx q1(AngleAxisx(Scalar(EIGEN_PI), axis1));
    Quaternionx mid = q0.slerp(Scalar(0.5), q1);
    VERIFY(numext::abs(mid.norm() - Scalar(1)) < tight);
    // Midpoint should be a 90-degree rotation
    AngleAxisx aa(mid);
    VERIFY(numext::abs(aa.angle() - Scalar(EIGEN_PI) / Scalar(2)) < test_precision<Scalar>());
  }

  // slerp unit-norm preservation and rotation matrix orthogonality
  {
    Quaternionx q0 = Quaternionx::UnitRandom();
    Quaternionx q1 = Quaternionx::UnitRandom();
    for (Scalar t = 0; t <= Scalar(1.001); t += Scalar(0.05)) {
      Quaternionx r = q0.slerp(t, q1);
      VERIFY(numext::abs(r.norm() - Scalar(1)) < tight);
      Matrix3 m = r.toRotationMatrix();
      VERIFY_IS_APPROX(m.transpose() * m, Matrix3::Identity());
      VERIFY_IS_APPROX(m.determinant(), Scalar(1));
    }
  }

  // slerp monotonicity: angle from q0 should increase monotonically
  {
    Quaternionx q0 = Quaternionx::UnitRandom();
    Quaternionx q1 = Quaternionx::UnitRandom();
    Scalar prev_angle = Scalar(0);
    for (Scalar t = Scalar(0); t <= Scalar(1.001); t += Scalar(0.05)) {
      Quaternionx r = q0.slerp(t, q1);
      Scalar angle = AngleAxisx(r * q0.inverse()).angle();
      if (angle > Scalar(EIGEN_PI)) angle = Scalar(2) * Scalar(EIGEN_PI) - angle;
      VERIFY(angle >= prev_angle - tight);
      prev_angle = angle;
    }
  }
}

template <typename Scalar>
void umeyama_degenerate() {
  typedef Matrix<Scalar, 3, Dynamic> Points3;
  typedef Matrix<Scalar, 3, 1> Vector3;
  typedef Matrix<Scalar, 3, 3> Matrix3;
  typedef Matrix<Scalar, 4, 4> Matrix4;

  // All source points identical
  {
    int n = 10;
    Points3 src(3, n);
    src.colwise() = Vector3(1, 2, 3);
    Points3 dst = Points3::Random(3, n);

    Matrix4 T = umeyama(src, dst);
    // Result should NOT be NaN
    VERIFY(!(numext::isnan)(T(0, 0)));
    VERIFY(!(numext::isnan)(T(0, 3)));
  }

  // All destination points identical
  {
    int n = 10;
    Points3 src = Points3::Random(3, n);
    Points3 dst(3, n);
    dst.colwise() = Vector3(4, 5, 6);

    Matrix4 T = umeyama(src, dst);
    VERIFY(!(numext::isnan)(T(0, 0)));
  }

  // Pure translation (no rotation or scaling)
  {
    int n = 20;
    Points3 src = Points3::Random(3, n);
    Vector3 offset(10, 20, 30);
    Points3 dst = src.colwise() + offset;

    Matrix4 T = umeyama(src, dst);
    // Translation should be recovered
    Vector3 t_recovered = T.template block<3, 1>(0, 3);
    VERIFY_IS_APPROX(t_recovered, offset);
    // Rotation should be identity
    Matrix3 R = T.template block<3, 3>(0, 0);
    VERIFY(R.isApprox(Matrix3::Identity(), test_precision<Scalar>()));
  }
}

template <typename Scalar>
void shear_ground_truth() {
  typedef Transform<Scalar, 2, Affine> Transform2;
  typedef Matrix<Scalar, 2, 1> Vector2;

  // Verify the actual matrix produced by shear()
  // Note: Eigen's shear(sx, sy) applies the matrix [[1, sy], [sx, 1]] to columns,
  // meaning sx controls the y-component shear and sy controls the x-component shear.
  {
    Transform2 t;
    t.setIdentity();
    t.shear(Scalar(2), Scalar(0));
    // After shear(2, 0), linear part should be [[1, 0], [2, 1]]
    VERIFY_IS_APPROX(t.linear()(0, 0), Scalar(1));
    VERIFY_IS_APPROX(t.linear()(0, 1), Scalar(0));
    VERIFY_IS_APPROX(t.linear()(1, 0), Scalar(2));
    VERIFY_IS_APPROX(t.linear()(1, 1), Scalar(1));
  }

  {
    Transform2 t;
    t.setIdentity();
    t.shear(Scalar(0), Scalar(3));
    // After shear(0, 3), linear part should be [[1, 3], [0, 1]]
    VERIFY_IS_APPROX(t.linear()(0, 0), Scalar(1));
    VERIFY_IS_APPROX(t.linear()(0, 1), Scalar(3));
    VERIFY_IS_APPROX(t.linear()(1, 0), Scalar(0));
    VERIFY_IS_APPROX(t.linear()(1, 1), Scalar(1));
  }

  // Verify shear on a point
  {
    Transform2 t;
    t.setIdentity();
    t.shear(Scalar(2), Scalar(3));
    Vector2 v(1, 0);
    Vector2 result = t.linear() * v;
    // [[1, 3], [2, 1]] * [1, 0]^T = [1, 2]
    VERIFY_IS_APPROX(result, Vector2(1, 2));

    v = Vector2(0, 1);
    result = t.linear() * v;
    // [[1, 3], [2, 1]] * [0, 1]^T = [3, 1]
    VERIFY_IS_APPROX(result, Vector2(3, 1));
  }
}

EIGEN_DECLARE_TEST(geo_misc) {
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(angleaxis_edge_cases<float>());
    CALL_SUBTEST_2(angleaxis_edge_cases<double>());

    CALL_SUBTEST_3(rotation2d_standalone<float>());
    CALL_SUBTEST_4(rotation2d_standalone<double>());

    CALL_SUBTEST_5(translation_standalone<float>());
    CALL_SUBTEST_5(translation_standalone<double>());

    CALL_SUBTEST_6(scaling_standalone<float>());
    CALL_SUBTEST_6(scaling_standalone<double>());

    CALL_SUBTEST_7(quaternion_from_non_orthogonal_matrix<float>());
    CALL_SUBTEST_7(quaternion_from_non_orthogonal_matrix<double>());

    CALL_SUBTEST_8(quaternion_slerp_edge_cases<float>());
    CALL_SUBTEST_8(quaternion_slerp_edge_cases<double>());

    CALL_SUBTEST_9(umeyama_degenerate<float>());
    CALL_SUBTEST_9(umeyama_degenerate<double>());

    CALL_SUBTEST_10(shear_ground_truth<float>());
    CALL_SUBTEST_10(shear_ground_truth<double>());
  }
}
