// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2012, 2013 Chen-Pang He <jdh8@ms63.hinet.net>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "matrix_functions.h"

template <typename T>
void test2dRotation(const T& tol) {
  Matrix<T, 2, 2> A, B, C;
  T angle, c, s;

  A << 0, 1, -1, 0;
  MatrixPower<Matrix<T, 2, 2>> Apow(A);

  for (int i = 0; i <= 20; ++i) {
    angle = std::pow(T(10), T(i - 10) / T(5.));
    c = std::cos(angle);
    s = std::sin(angle);
    B << c, s, -s, c;

    C = Apow(std::ldexp(angle, 1) / T(EIGEN_PI));
    std::cout << "test2dRotation: i = " << i << "   error powerm = " << relerr(C, B) << '\n';
    VERIFY(C.isApprox(B, tol));
  }
}

template <typename T>
void test2dHyperbolicRotation(const T& tol) {
  Matrix<std::complex<T>, 2, 2> A, B, C;
  T angle, ch = std::cosh((T)1);
  std::complex<T> ish(0, std::sinh((T)1));

  A << ch, ish, -ish, ch;
  MatrixPower<Matrix<std::complex<T>, 2, 2>> Apow(A);

  for (int i = 0; i <= 20; ++i) {
    angle = std::ldexp(static_cast<T>(i - 10), -1);
    ch = std::cosh(angle);
    ish = std::complex<T>(0, std::sinh(angle));
    B << ch, ish, -ish, ch;

    C = Apow(angle);
    std::cout << "test2dHyperbolicRotation: i = " << i << "   error powerm = " << relerr(C, B) << '\n';
    VERIFY(C.isApprox(B, tol));
  }
}

template <typename T>
void test3dRotation(const T& tol) {
  Matrix<T, 3, 1> v;
  T angle;

  for (int i = 0; i <= 20; ++i) {
    v = Matrix<T, 3, 1>::Random();
    v.normalize();
    angle = std::pow(T(10), T(i - 10) / T(5.));
    VERIFY(AngleAxis<T>(angle, v).matrix().isApprox(AngleAxis<T>(1, v).matrix().pow(angle), tol));
  }
}

template <typename MatrixType>
void testGeneral(const MatrixType& m, const typename MatrixType::RealScalar& tol) {
  typedef typename MatrixType::RealScalar RealScalar;
  MatrixType m1, m2, m3, m4, m5;
  RealScalar x, y;

  // The identities A^(xy) = (A^x)^y and (cA)^y = c^y * A^y involve two
  // independent Schur decompositions. Each decomposition + Pade + squaring
  // chain introduces rounding that scales with matrix dimension.
  RealScalar tol2 = tol * RealScalar(m.rows() * m.rows());

  for (int i = 0; i < g_repeat; ++i) {
    generateTestMatrix<MatrixType>::run(m1, m.rows());
    MatrixPower<MatrixType> mpow(m1);

    x = internal::random<RealScalar>();
    y = internal::random<RealScalar>();
    m2 = mpow(x);
    m3 = mpow(y);

    m4 = mpow(x + y);
    m5.noalias() = m2 * m3;
    VERIFY(m4.isApprox(m5, tol));

    m4 = mpow(x * y);
    m5 = m2.pow(y);
    VERIFY(m4.isApprox(m5, tol2));

    m4 = (std::abs(x) * m1).pow(y);
    m5 = std::pow(std::abs(x), y) * m3;
    VERIFY(m4.isApprox(m5, tol2));
  }
}

template <typename MatrixType>
void testSingular(const MatrixType& m_const, const typename MatrixType::RealScalar& tol) {
  // we need to pass by reference in order to prevent errors with
  // MSVC for aligned data types ...
  MatrixType& m = const_cast<MatrixType&>(m_const);

  typedef typename MatrixType::RealScalar RealScalar;
  const int IsComplex = NumTraits<typename internal::traits<MatrixType>::Scalar>::IsComplex;
  typedef std::conditional_t<IsComplex, TriangularView<MatrixType, Upper>, const MatrixType&> TriangularType;
  std::conditional_t<IsComplex, ComplexSchur<MatrixType>, RealSchur<MatrixType>> schur;
  MatrixType T;

  // MatrixPower uses its own Schur decomposition while the reference uses
  // repeated sqrt of the caller-provided Schur form, so errors compound
  // from two independent algorithm paths and scale with matrix dimension.
  RealScalar tol2 = tol * RealScalar(m.rows() * m.rows());

  for (int i = 0; i < g_repeat; ++i) {
    m.setRandom();
    m.col(0).fill(0);

    schur.compute(m);
    T = schur.matrixT();
    const MatrixType& U = schur.matrixU();
    processTriangularMatrix<MatrixType>::run(m, T, U);
    MatrixPower<MatrixType> mpow(m);

    T = T.sqrt();
    VERIFY(mpow(0.5L).isApprox(U * (TriangularType(T) * U.adjoint()), tol2));

    T = T.sqrt();
    VERIFY(mpow(0.25L).isApprox(U * (TriangularType(T) * U.adjoint()), tol2));

    T = T.sqrt();
    VERIFY(mpow(0.125L).isApprox(U * (TriangularType(T) * U.adjoint()), tol2));
  }
}

template <typename MatrixType>
void testLogThenExp(const MatrixType& m_const, const typename MatrixType::RealScalar& tol) {
  // we need to pass by reference in order to prevent errors with
  // MSVC for aligned data types ...
  MatrixType& m = const_cast<MatrixType&>(m_const);

  typedef typename MatrixType::Scalar Scalar;
  typedef typename MatrixType::RealScalar RealScalar;
  Scalar x;

  // Computing exp(x*log(m)) chains three separate Pade-based algorithms
  // (matrix log, scaling, matrix exp). The accumulated error far exceeds
  // what a single matrix power computation achieves.
  const Index n = m.rows();
  RealScalar tol2 = (std::max)(tol * RealScalar(n * n), test_precision<RealScalar>() * RealScalar(n * n));

  for (int i = 0; i < g_repeat; ++i) {
    generateTestMatrix<MatrixType>::run(m, m.rows());
    x = internal::random<Scalar>();
    VERIFY(m.pow(x).isApprox((x * m.log()).exp(), tol2));
  }
}

typedef Matrix<double, 3, 3, RowMajor> Matrix3dRowMajor;
typedef Matrix<long double, 3, 3> Matrix3e;
typedef Matrix<long double, Dynamic, Dynamic> MatrixXe;

EIGEN_DECLARE_TEST(matrix_power) {
  // Factors are the smallest powers of two above the worst relative error seen over ~50k randomized trials per
  // instantiation (4x that maximum). testGeneral is dominated by the A^(x+y) = A^x A^y and (A^x)^y identities,
  // whose error has a heavy tail for near-defective random matrices, so those factors are large. testLogThenExp's
  // bound is floored by test_precision * n^2, so its factor only documents the typical error.
  CALL_SUBTEST_2(test2dRotation<double>(128 * NumTraits<double>::epsilon()));
  CALL_SUBTEST_1(test2dRotation<float>(256 * NumTraits<float>::epsilon()));
  CALL_SUBTEST_9(test2dRotation<long double>(128 * NumTraits<long double>::epsilon()));
  CALL_SUBTEST_2(test2dHyperbolicRotation<double>(32 * NumTraits<double>::epsilon()));
  CALL_SUBTEST_1(test2dHyperbolicRotation<float>(64 * NumTraits<float>::epsilon()));
  CALL_SUBTEST_9(test2dHyperbolicRotation<long double>(64 * NumTraits<long double>::epsilon()));

  CALL_SUBTEST_10(test3dRotation<double>(512 * NumTraits<double>::epsilon()));
  CALL_SUBTEST_11(test3dRotation<float>(512 * NumTraits<float>::epsilon()));
  CALL_SUBTEST_12(test3dRotation<long double>(512 * NumTraits<long double>::epsilon()));

  CALL_SUBTEST_2(testGeneral(Matrix2d(), 2048 * NumTraits<double>::epsilon()));
  CALL_SUBTEST_7(testGeneral(Matrix3dRowMajor(), 4096 * NumTraits<double>::epsilon()));
  CALL_SUBTEST_3(testGeneral(Matrix4cd(), 4096 * NumTraits<std::complex<double>>::epsilon()));
  CALL_SUBTEST_4(testGeneral(MatrixXd(8, 8), 16384 * NumTraits<double>::epsilon()));
  CALL_SUBTEST_1(testGeneral(Matrix2f(), 2048 * NumTraits<float>::epsilon()));
  CALL_SUBTEST_5(testGeneral(Matrix3cf(), 1024 * NumTraits<std::complex<float>>::epsilon()));
  CALL_SUBTEST_8(testGeneral(Matrix4f(), 16384 * NumTraits<float>::epsilon()));
  CALL_SUBTEST_6(testGeneral(MatrixXf(2, 2), 2048 * NumTraits<float>::epsilon()));  // see bug 614
  CALL_SUBTEST_9(testGeneral(MatrixXe(7, 7), 16384 * NumTraits<long double>::epsilon()));
  CALL_SUBTEST_10(testGeneral(Matrix3d(), 32768 * NumTraits<double>::epsilon()));
  CALL_SUBTEST_11(testGeneral(Matrix3f(), 32768 * NumTraits<float>::epsilon()));
  CALL_SUBTEST_12(testGeneral(Matrix3e(), 4096 * NumTraits<long double>::epsilon()));

  CALL_SUBTEST_2(testSingular(Matrix2d(), 8 * NumTraits<double>::epsilon()));
  CALL_SUBTEST_7(testSingular(Matrix3dRowMajor(), 512 * NumTraits<double>::epsilon()));
  CALL_SUBTEST_3(testSingular(Matrix4cd(), 8 * NumTraits<std::complex<double>>::epsilon()));
  CALL_SUBTEST_4(testSingular(MatrixXd(8, 8), 128 * NumTraits<double>::epsilon()));
  CALL_SUBTEST_1(testSingular(Matrix2f(), 8 * NumTraits<float>::epsilon()));
  CALL_SUBTEST_5(testSingular(Matrix3cf(), 8 * NumTraits<std::complex<float>>::epsilon()));
  CALL_SUBTEST_8(testSingular(Matrix4f(), 256 * NumTraits<float>::epsilon()));
  CALL_SUBTEST_6(testSingular(MatrixXf(2, 2), 8 * NumTraits<float>::epsilon()));
  CALL_SUBTEST_9(testSingular(MatrixXe(7, 7), 256 * NumTraits<long double>::epsilon()));
  CALL_SUBTEST_10(testSingular(Matrix3d(), 1024 * NumTraits<double>::epsilon()));
  CALL_SUBTEST_11(testSingular(Matrix3f(), 2048 * NumTraits<float>::epsilon()));
  CALL_SUBTEST_12(testSingular(Matrix3e(), 1024 * NumTraits<long double>::epsilon()));

  CALL_SUBTEST_2(testLogThenExp(Matrix2d(), 128 * NumTraits<double>::epsilon()));
  CALL_SUBTEST_7(testLogThenExp(Matrix3dRowMajor(), 128 * NumTraits<double>::epsilon()));
  CALL_SUBTEST_3(testLogThenExp(Matrix4cd(), 8 * NumTraits<std::complex<double>>::epsilon()));
  CALL_SUBTEST_4(testLogThenExp(MatrixXd(8, 8), 64 * NumTraits<double>::epsilon()));
  CALL_SUBTEST_1(testLogThenExp(Matrix2f(), 64 * NumTraits<float>::epsilon()));
  CALL_SUBTEST_5(testLogThenExp(Matrix3cf(), 8 * NumTraits<std::complex<float>>::epsilon()));
  CALL_SUBTEST_8(testLogThenExp(Matrix4f(), 64 * NumTraits<float>::epsilon()));
  CALL_SUBTEST_6(testLogThenExp(MatrixXf(2, 2), 64 * NumTraits<float>::epsilon()));
  CALL_SUBTEST_9(testLogThenExp(MatrixXe(7, 7), 64 * NumTraits<long double>::epsilon()));
  CALL_SUBTEST_10(testLogThenExp(Matrix3d(), 128 * NumTraits<double>::epsilon()));
  CALL_SUBTEST_11(testLogThenExp(Matrix3f(), 128 * NumTraits<float>::epsilon()));
  CALL_SUBTEST_12(testLogThenExp(Matrix3e(), 128 * NumTraits<long double>::epsilon()));
}
