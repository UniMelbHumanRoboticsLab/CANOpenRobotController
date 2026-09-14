// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

using namespace Eigen;

// ============================================================================
// Quaternion operations
// ============================================================================

template <typename Scalar>
static void BM_QuatSlerp(benchmark::State& state) {
  Quaternion<Scalar> q0 = Quaternion<Scalar>::UnitRandom();
  Quaternion<Scalar> q1 = Quaternion<Scalar>::UnitRandom();
  Quaternion<Scalar> r;
  Scalar t = Scalar(0.5);
  for (auto _ : state) {
    r = q0.slerp(t, q1);
    benchmark::DoNotOptimize(r.coeffs().data());
  }
}

template <typename Scalar>
static void BM_QuatRotateVec(benchmark::State& state) {
  Quaternion<Scalar> q = Quaternion<Scalar>::UnitRandom();
  Matrix<Scalar, 3, 1> v = Matrix<Scalar, 3, 1>::Random();
  for (auto _ : state) {
    v = q * v;
    benchmark::DoNotOptimize(v.data());
  }
}

template <typename Scalar>
static void BM_QuatNormalize(benchmark::State& state) {
  Quaternion<Scalar> q(Scalar(1.1), Scalar(2.2), Scalar(3.3), Scalar(4.4));
  for (auto _ : state) {
    q.normalize();
    benchmark::DoNotOptimize(q.coeffs().data());
  }
}

template <typename Scalar>
static void BM_QuatInverse(benchmark::State& state) {
  Quaternion<Scalar> q = Quaternion<Scalar>::UnitRandom();
  Quaternion<Scalar> r;
  for (auto _ : state) {
    r = q.inverse();
    benchmark::DoNotOptimize(r.coeffs().data());
  }
}

template <typename Scalar>
static void BM_QuatToRotationMatrix(benchmark::State& state) {
  Quaternion<Scalar> q = Quaternion<Scalar>::UnitRandom();
  Matrix<Scalar, 3, 3> m;
  for (auto _ : state) {
    m = q.toRotationMatrix();
    benchmark::DoNotOptimize(m.data());
  }
}

template <typename Scalar>
static void BM_QuatFromRotationMatrix(benchmark::State& state) {
  Quaternion<Scalar> q = Quaternion<Scalar>::UnitRandom();
  Matrix<Scalar, 3, 3> m = q.toRotationMatrix();
  for (auto _ : state) {
    q = m;
    benchmark::DoNotOptimize(q.coeffs().data());
  }
}

template <typename Scalar>
static void BM_QuatAngularDistance(benchmark::State& state) {
  Quaternion<Scalar> q0 = Quaternion<Scalar>::UnitRandom();
  Quaternion<Scalar> q1 = Quaternion<Scalar>::UnitRandom();
  Scalar d;
  for (auto _ : state) {
    d = q0.angularDistance(q1);
    benchmark::DoNotOptimize(d);
  }
}

template <typename Scalar>
static void BM_QuatSetFromTwoVectors(benchmark::State& state) {
  Matrix<Scalar, 3, 1> v0 = Matrix<Scalar, 3, 1>::Random().normalized();
  Matrix<Scalar, 3, 1> v1 = Matrix<Scalar, 3, 1>::Random().normalized();
  Quaternion<Scalar> q;
  for (auto _ : state) {
    q.setFromTwoVectors(v0, v1);
    benchmark::DoNotOptimize(q.coeffs().data());
  }
}

template <typename Scalar>
static void BM_QuatSetFromTwoVectorsAntiparallel(benchmark::State& state) {
  Matrix<Scalar, 3, 1> v0 = Matrix<Scalar, 3, 1>::Random().normalized();
  Matrix<Scalar, 3, 1> v1 = -v0;  // exactly antiparallel
  Quaternion<Scalar> q;
  for (auto _ : state) {
    q.setFromTwoVectors(v0, v1);
    benchmark::DoNotOptimize(q.coeffs().data());
  }
}

// ============================================================================
// AngleAxis operations
// ============================================================================

template <typename Scalar>
static void BM_AngleAxisToRotationMatrix(benchmark::State& state) {
  AngleAxis<Scalar> aa(Scalar(1.0), Matrix<Scalar, 3, 1>::UnitX());
  Matrix<Scalar, 3, 3> m;
  for (auto _ : state) {
    m = aa.toRotationMatrix();
    benchmark::DoNotOptimize(m.data());
  }
}

template <typename Scalar>
static void BM_AngleAxisFromRotationMatrix(benchmark::State& state) {
  Quaternion<Scalar> q = Quaternion<Scalar>::UnitRandom();
  Matrix<Scalar, 3, 3> m = q.toRotationMatrix();
  AngleAxis<Scalar> aa;
  for (auto _ : state) {
    aa = m;
    benchmark::DoNotOptimize(aa);
  }
}

template <typename Scalar>
static void BM_AngleAxisRotateVec(benchmark::State& state) {
  AngleAxis<Scalar> aa(Scalar(1.0), Matrix<Scalar, 3, 1>::Random().normalized());
  Matrix<Scalar, 3, 1> v = Matrix<Scalar, 3, 1>::Random();
  for (auto _ : state) {
    v = aa * v;
    benchmark::DoNotOptimize(v.data());
  }
}

// ============================================================================
// Rotation2D operations
// ============================================================================

template <typename Scalar>
static void BM_Rotation2DRotateVec(benchmark::State& state) {
  Rotation2D<Scalar> r(Scalar(0.5));
  Matrix<Scalar, 2, 1> v = Matrix<Scalar, 2, 1>::Random();
  for (auto _ : state) {
    v = r * v;
    benchmark::DoNotOptimize(v.data());
  }
}

template <typename Scalar>
static void BM_Rotation2DSlerp(benchmark::State& state) {
  Rotation2D<Scalar> r0(Scalar(0.1)), r1(Scalar(2.5));
  Rotation2D<Scalar> r;
  for (auto _ : state) {
    r = r0.slerp(Scalar(0.5), r1);
    benchmark::DoNotOptimize(r);
  }
}

// ============================================================================
// Transform inverse
// ============================================================================

template <typename Scalar, int Mode>
static void BM_TransformInverse(benchmark::State& state) {
  typedef Transform<Scalar, 3, Mode> Trans;
  Trans t;
  t.setIdentity();
  t.rotate(Quaternion<Scalar>::UnitRandom());
  t.translate(Matrix<Scalar, 3, 1>::Random());
  Trans r;
  for (auto _ : state) {
    r = t.inverse();
    benchmark::DoNotOptimize(r.data());
  }
}

// ============================================================================
// Euler angles
// ============================================================================

template <typename Scalar>
static void BM_EulerAnglesExtract(benchmark::State& state) {
  Quaternion<Scalar> q = Quaternion<Scalar>::UnitRandom();
  Matrix<Scalar, 3, 3> m = q.toRotationMatrix();
  Matrix<Scalar, 3, 1> ea;
  for (auto _ : state) {
    ea = m.canonicalEulerAngles(0, 1, 2);
    benchmark::DoNotOptimize(ea.data());
  }
}

// ============================================================================
// Cross product
// ============================================================================

template <typename Scalar>
static void BM_CrossProduct(benchmark::State& state) {
  Matrix<Scalar, 3, 1> a = Matrix<Scalar, 3, 1>::Random();
  Matrix<Scalar, 3, 1> b = Matrix<Scalar, 3, 1>::Random();
  Matrix<Scalar, 3, 1> c;
  for (auto _ : state) {
    c = a.cross(b);
    benchmark::DoNotOptimize(c.data());
  }
}

template <typename Scalar>
static void BM_UnitOrthogonal(benchmark::State& state) {
  Matrix<Scalar, 3, 1> v = Matrix<Scalar, 3, 1>::Random();
  Matrix<Scalar, 3, 1> r;
  for (auto _ : state) {
    r = v.unitOrthogonal();
    benchmark::DoNotOptimize(r.data());
  }
}

// ============================================================================
// AlignedBox operations
// ============================================================================

template <typename Scalar>
static void BM_AlignedBoxContains(benchmark::State& state) {
  AlignedBox<Scalar, 3> box(Matrix<Scalar, 3, 1>(-1, -1, -1), Matrix<Scalar, 3, 1>(1, 1, 1));
  Matrix<Scalar, 3, 1> p = Matrix<Scalar, 3, 1>::Random();
  bool result;
  for (auto _ : state) {
    result = box.contains(p);
    benchmark::DoNotOptimize(result);
  }
}

template <typename Scalar>
static void BM_AlignedBoxIntersects(benchmark::State& state) {
  AlignedBox<Scalar, 3> box1(Matrix<Scalar, 3, 1>(-1, -1, -1), Matrix<Scalar, 3, 1>(1, 1, 1));
  AlignedBox<Scalar, 3> box2(Matrix<Scalar, 3, 1>(0, 0, 0), Matrix<Scalar, 3, 1>(2, 2, 2));
  bool result;
  for (auto _ : state) {
    result = box1.intersects(box2);
    benchmark::DoNotOptimize(result);
  }
}

template <typename Scalar>
static void BM_AlignedBoxTransform(benchmark::State& state) {
  AlignedBox<Scalar, 3> box(Matrix<Scalar, 3, 1>(-1, -1, -1), Matrix<Scalar, 3, 1>(1, 1, 1));
  Transform<Scalar, 3, Isometry> t;
  t.setIdentity();
  t.rotate(Quaternion<Scalar>::UnitRandom());
  t.translate(Matrix<Scalar, 3, 1>::Random());
  for (auto _ : state) {
    AlignedBox<Scalar, 3> result = box.transformed(t);
    benchmark::DoNotOptimize(result);
  }
}

// ============================================================================
// Hyperplane / ParametrizedLine
// ============================================================================

template <typename Scalar>
static void BM_HyperplaneSignedDistance(benchmark::State& state) {
  Hyperplane<Scalar, 3> plane = Hyperplane<Scalar, 3>::Through(
      Matrix<Scalar, 3, 1>(1, 0, 0), Matrix<Scalar, 3, 1>(0, 1, 0), Matrix<Scalar, 3, 1>(0, 0, 1));
  Matrix<Scalar, 3, 1> p = Matrix<Scalar, 3, 1>::Random();
  Scalar d;
  for (auto _ : state) {
    d = plane.signedDistance(p);
    benchmark::DoNotOptimize(d);
  }
}

template <typename Scalar>
static void BM_LinePointDistance(benchmark::State& state) {
  ParametrizedLine<Scalar, 3> line(Matrix<Scalar, 3, 1>::Zero(), Matrix<Scalar, 3, 1>::UnitX());
  Matrix<Scalar, 3, 1> p = Matrix<Scalar, 3, 1>::Random();
  Scalar d;
  for (auto _ : state) {
    d = line.distance(p);
    benchmark::DoNotOptimize(d);
  }
}

template <typename Scalar>
static void BM_LinePlaneIntersection(benchmark::State& state) {
  ParametrizedLine<Scalar, 3> line(Matrix<Scalar, 3, 1>::Zero(), Matrix<Scalar, 3, 1>::UnitZ());
  Hyperplane<Scalar, 3> plane(Matrix<Scalar, 3, 1>::UnitZ(), Scalar(-5));
  Scalar t;
  for (auto _ : state) {
    t = line.intersectionParameter(plane);
    benchmark::DoNotOptimize(t);
  }
}

// ============================================================================
// Umeyama
// ============================================================================

template <typename Scalar, int Dim>
static void BM_Umeyama(benchmark::State& state) {
  const int n = state.range(0);
  Matrix<Scalar, Dim, Dynamic> src = Matrix<Scalar, Dim, Dynamic>::Random(Dim, n);
  // Apply a known transform
  Matrix<Scalar, Dim, Dim> R = Quaternion<Scalar>::UnitRandom().toRotationMatrix();
  Matrix<Scalar, Dim, 1> t = Matrix<Scalar, Dim, 1>::Random() * Scalar(10);
  Matrix<Scalar, Dim, Dynamic> dst = (R * src).colwise() + t;
  for (auto _ : state) {
    auto T = umeyama(src, dst);
    benchmark::DoNotOptimize(T);
  }
}

// ============================================================================
// Registration
// ============================================================================

// Quaternion
BENCHMARK(BM_QuatSlerp<float>);
BENCHMARK(BM_QuatSlerp<double>);
BENCHMARK(BM_QuatRotateVec<float>);
BENCHMARK(BM_QuatRotateVec<double>);
BENCHMARK(BM_QuatNormalize<float>);
BENCHMARK(BM_QuatNormalize<double>);
BENCHMARK(BM_QuatInverse<float>);
BENCHMARK(BM_QuatInverse<double>);
BENCHMARK(BM_QuatToRotationMatrix<float>);
BENCHMARK(BM_QuatToRotationMatrix<double>);
BENCHMARK(BM_QuatFromRotationMatrix<float>);
BENCHMARK(BM_QuatFromRotationMatrix<double>);
BENCHMARK(BM_QuatAngularDistance<float>);
BENCHMARK(BM_QuatAngularDistance<double>);
BENCHMARK(BM_QuatSetFromTwoVectors<float>);
BENCHMARK(BM_QuatSetFromTwoVectors<double>);
BENCHMARK(BM_QuatSetFromTwoVectorsAntiparallel<float>);
BENCHMARK(BM_QuatSetFromTwoVectorsAntiparallel<double>);

// AngleAxis
BENCHMARK(BM_AngleAxisToRotationMatrix<float>);
BENCHMARK(BM_AngleAxisToRotationMatrix<double>);
BENCHMARK(BM_AngleAxisFromRotationMatrix<float>);
BENCHMARK(BM_AngleAxisFromRotationMatrix<double>);
BENCHMARK(BM_AngleAxisRotateVec<float>);
BENCHMARK(BM_AngleAxisRotateVec<double>);

// Rotation2D
BENCHMARK(BM_Rotation2DRotateVec<float>);
BENCHMARK(BM_Rotation2DRotateVec<double>);
BENCHMARK(BM_Rotation2DSlerp<float>);
BENCHMARK(BM_Rotation2DSlerp<double>);

// Transform inverse
BENCHMARK(BM_TransformInverse<float, Isometry>);
BENCHMARK(BM_TransformInverse<float, Affine>);
BENCHMARK(BM_TransformInverse<double, Isometry>);
BENCHMARK(BM_TransformInverse<double, Affine>);

// Euler angles
BENCHMARK(BM_EulerAnglesExtract<float>);
BENCHMARK(BM_EulerAnglesExtract<double>);

// Cross product & orthogonal
BENCHMARK(BM_CrossProduct<float>);
BENCHMARK(BM_CrossProduct<double>);
BENCHMARK(BM_UnitOrthogonal<float>);
BENCHMARK(BM_UnitOrthogonal<double>);

// AlignedBox
BENCHMARK(BM_AlignedBoxContains<float>);
BENCHMARK(BM_AlignedBoxContains<double>);
BENCHMARK(BM_AlignedBoxIntersects<float>);
BENCHMARK(BM_AlignedBoxIntersects<double>);
BENCHMARK(BM_AlignedBoxTransform<float>);
BENCHMARK(BM_AlignedBoxTransform<double>);

// Hyperplane / ParametrizedLine
BENCHMARK(BM_HyperplaneSignedDistance<float>);
BENCHMARK(BM_HyperplaneSignedDistance<double>);
BENCHMARK(BM_LinePointDistance<float>);
BENCHMARK(BM_LinePointDistance<double>);
BENCHMARK(BM_LinePlaneIntersection<float>);
BENCHMARK(BM_LinePlaneIntersection<double>);

// Umeyama
BENCHMARK(BM_Umeyama<float, 3>)->Arg(10)->Arg(100)->Arg(1000);
BENCHMARK(BM_Umeyama<double, 3>)->Arg(10)->Arg(100)->Arg(1000);
