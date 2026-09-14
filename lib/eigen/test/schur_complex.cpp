// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2010,2012 Jitse Niesen <jitse@maths.leeds.ac.uk>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "main.h"
#include "fp_control.h"
#include <limits>
#include <Eigen/Eigenvalues>

template <typename MatrixType>
void schur(int size = MatrixType::ColsAtCompileTime) {
  typedef typename ComplexSchur<MatrixType>::ComplexScalar ComplexScalar;
  typedef typename ComplexSchur<MatrixType>::ComplexMatrixType ComplexMatrixType;

  // Test basic functionality: T is triangular and A = U T U*
  for (int counter = 0; counter < g_repeat; ++counter) {
    MatrixType A = MatrixType::Random(size, size);
    ComplexSchur<MatrixType> schurOfA(A);
    VERIFY_IS_EQUAL(schurOfA.info(), Success);
    ComplexMatrixType U = schurOfA.matrixU();
    ComplexMatrixType T = schurOfA.matrixT();
    for (int row = 1; row < size; ++row) {
      for (int col = 0; col < row; ++col) {
        VERIFY(T(row, col) == (typename MatrixType::Scalar)0);
      }
    }
    VERIFY_IS_APPROX(A.template cast<ComplexScalar>(), U * T * U.adjoint());
  }

  // Test asserts when not initialized
  ComplexSchur<MatrixType> csUninitialized;
  VERIFY_RAISES_ASSERT(csUninitialized.matrixT());
  VERIFY_RAISES_ASSERT(csUninitialized.matrixU());
  VERIFY_RAISES_ASSERT(csUninitialized.info());

  // Test whether compute() and constructor returns same result
  MatrixType A = MatrixType::Random(size, size);
  ComplexSchur<MatrixType> cs1;
  cs1.compute(A);
  ComplexSchur<MatrixType> cs2(A);
  VERIFY_IS_EQUAL(cs1.info(), Success);
  VERIFY_IS_EQUAL(cs2.info(), Success);
  VERIFY_IS_EQUAL(cs1.matrixT(), cs2.matrixT());
  VERIFY_IS_EQUAL(cs1.matrixU(), cs2.matrixU());

  // Test maximum number of iterations
  ComplexSchur<MatrixType> cs3;
  cs3.setMaxIterations(ComplexSchur<MatrixType>::m_maxIterationsPerRow * size).compute(A);
  VERIFY_IS_EQUAL(cs3.info(), Success);
  VERIFY_IS_EQUAL(cs3.matrixT(), cs1.matrixT());
  VERIFY_IS_EQUAL(cs3.matrixU(), cs1.matrixU());
  cs3.setMaxIterations(1).compute(A);
  // The schur decomposition does often converge with a single iteration.
  // VERIFY_IS_EQUAL(cs3.info(), size > 1 ? NoConvergence : Success);
  VERIFY_IS_EQUAL(cs3.getMaxIterations(), 1);

  MatrixType Atriangular = A;
  Atriangular.template triangularView<StrictlyLower>().setZero();
  cs3.setMaxIterations(1).compute(Atriangular);  // triangular matrices do not need any iterations
  VERIFY_IS_EQUAL(cs3.info(), Success);
  VERIFY_IS_EQUAL(cs3.matrixT(), Atriangular.template cast<ComplexScalar>());
  VERIFY_IS_EQUAL(cs3.matrixU(), ComplexMatrixType::Identity(size, size));

  // Test computation of only T, not U
  ComplexSchur<MatrixType> csOnlyT(A, false);
  VERIFY_IS_EQUAL(csOnlyT.info(), Success);
  VERIFY_IS_EQUAL(cs1.matrixT(), csOnlyT.matrixT());
  VERIFY_RAISES_ASSERT(csOnlyT.matrixU());

  if (size > 1 && size < 20) {
    // Test matrix with NaN
    A(0, 0) = std::numeric_limits<typename MatrixType::RealScalar>::quiet_NaN();
    ComplexSchur<MatrixType> csNaN(A);
    VERIFY_IS_EQUAL(csNaN.info(), NoConvergence);
  }
}

// The Hessenberg reduction treats a subdiagonal tail whose squared norm underflows as already zero. ComplexSchur used
// to reduce the caller's matrix directly, so at this scale it returned the diagonal of the input as the Schur form.
template <typename MatrixType>
void schur_underflow_scale(Index size) {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename MatrixType::RealScalar RealScalar;
  typedef typename ComplexSchur<MatrixType>::ComplexScalar ComplexScalar;
  typedef typename ComplexSchur<MatrixType>::ComplexMatrixType ComplexMatrixType;

  // Representable, but every product of two coefficients underflows.
  const RealScalar scale = numext::sqrt((std::numeric_limits<RealScalar>::min)()) / RealScalar(1024);
  const MatrixType A = MatrixType::Random(size, size) * Scalar(scale);

  ComplexSchur<MatrixType> schurOfA(A);
  VERIFY_IS_EQUAL(schurOfA.info(), Success);
  const ComplexMatrixType& U = schurOfA.matrixU();
  const ComplexMatrixType& T = schurOfA.matrixT();

  // isApprox squares its operands, and every square here is subnormal: under flush-to-zero both sides of the
  // reconstruction collapse to zero and any T passes. scale is an exact power of two, so multiplying by its
  // reciprocal is lossless and puts the comparison back in the normal range.
  const ComplexScalar invScale = ComplexScalar(RealScalar(1) / scale);
  VERIFY_IS_APPROX(ComplexMatrixType(A.template cast<ComplexScalar>() * invScale),
                   ComplexMatrixType(U * T * U.adjoint() * invScale));
  VERIFY_IS_APPROX(U * U.adjoint(), ComplexMatrixType::Identity(size, size));
  for (Index row = 1; row < size; ++row)
    for (Index col = 0; col < row; ++col) VERIFY(numext::is_exactly_zero(T(row, col)));
}

// Under flush-to-zero, targets with runtime control see what ARMv7 NEON always does to subnormal SIMD results, and Arm
// flushes subnormal inputs as well. The decomposition runs inside the scope; callers verify outside it.
template <typename MatrixType>
bool compute_schur(ComplexSchur<MatrixType>& schur, const MatrixType& A, bool flushToZero) {
  if (!flushToZero) {
    schur.compute(A);
    return true;
  }
  ScopedFlushToZero scope;
  if (scope.isSupported()) schur.compute(A);
  return scope.isSupported();
}

// A matrix whose coefficients are all subnormal is still a matrix with a Schur decomposition, not the zero matrix.
// Scaling by a power of two moves it into the normal range exactly, so the factorization stays accurate to the
// quantization of the subnormal input.
template <typename MatrixType>
void schur_subnormal_scale(Index size, bool flushToZero) {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename MatrixType::RealScalar RealScalar;
  typedef typename ComplexSchur<MatrixType>::ComplexMatrixType ComplexMatrixType;
  using WideScalar = std::complex<long double>;
  using WideMatrix = Matrix<WideScalar, Dynamic, Dynamic>;

  const RealScalar denormalMin = std::numeric_limits<RealScalar>::denorm_min();
  const RealScalar normalMin = (std::numeric_limits<RealScalar>::min)();
  if (!(denormalMin < normalMin)) return;  // Target has no subnormals, or flushes them to zero.

  // Every coefficient lands in the top of the subnormal range: none of them is normal, yet each still carries most of
  // its mantissa, so an accurate decomposition remains possible. Maxima and reconstructions are taken in long double,
  // whose arithmetic is scalar: a SIMD unit that flushes subnormals (ARMv7 NEON) would zero them.
  const MatrixType A = MatrixType::Random(size, size) * Scalar(normalMin / RealScalar(2));
  const WideMatrix wideA = A.template cast<WideScalar>();
  const long double maxA = wideA.cwiseAbs().maxCoeff();
  VERIFY(maxA > 0 && maxA < static_cast<long double>(normalMin));

  ComplexSchur<MatrixType> schurOfA(size);
  if (!compute_schur(schurOfA, A, flushToZero)) return;
  VERIFY_IS_EQUAL(schurOfA.info(), Success);
  const ComplexMatrixType& U = schurOfA.matrixU();
  const ComplexMatrixType& T = schurOfA.matrixT();
  const WideMatrix wideU = U.template cast<WideScalar>();
  const WideMatrix wideT = T.template cast<WideScalar>();

  // Unitary invariance gives ||T||_F = ||A||_F, so the largest coefficient of T cannot be more than a factor of the
  // dimension below the largest coefficient of A. Returning the zero matrix, as the unscaled reduction did, fails
  // this; isZero() would not, because every coefficient here is much smaller than one.
  VERIFY(wideT.cwiseAbs().maxCoeff() >= maxA / static_cast<long double>(size));
  VERIFY_IS_APPROX(U * U.adjoint(), ComplexMatrixType::Identity(size, size));
  // T rounds back into the subnormal range, where the spacing is denorm_min rather than eps times the coefficient, so
  // bound the residual by that spacing instead of by a relative tolerance.
  const long double residual = (wideA - wideU * wideT * wideU.adjoint()).cwiseAbs().maxCoeff();
  VERIFY(residual <= static_cast<long double>(4 * size * size) * static_cast<long double>(denormalMin));

  // A triangular matrix is its own Schur form and power-of-two scaling is exact, so T reproduces it bit for bit.
  MatrixType triangular = A;
  triangular.template triangularView<StrictlyLower>().setZero();
  compute_schur(schurOfA, triangular, flushToZero);
  VERIFY_IS_EQUAL(schurOfA.info(), Success);
  VERIFY_IS_EQUAL(U, ComplexMatrixType::Identity(size, size));
  VERIFY_IS_EQUAL(WideMatrix(T.template cast<WideScalar>()), WideMatrix(triangular.template cast<WideScalar>()));
}

// Scaling divides by a power of two, so coefficients more than the exponent range below the largest one flush to zero.
// That perturbation is bounded by the smallest subnormal relative to the largest coefficient, far below eps, and the
// unscaled reduction squares such a matrix into overflow anyway.
template <typename MatrixType>
void schur_dynamic_range(Index size) {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename MatrixType::RealScalar RealScalar;
  typedef typename ComplexSchur<MatrixType>::ComplexScalar ComplexScalar;
  typedef typename ComplexSchur<MatrixType>::ComplexMatrixType ComplexMatrixType;

  const RealScalar huge = numext::sqrt((std::numeric_limits<RealScalar>::max)());
  MatrixType A = MatrixType::Random(size, size) * Scalar(huge);
  A(size - 1, 0) = Scalar(std::numeric_limits<RealScalar>::denorm_min());

  ComplexSchur<MatrixType> schurOfA(A);
  VERIFY_IS_EQUAL(schurOfA.info(), Success);
  const ComplexMatrixType& U = schurOfA.matrixU();
  const ComplexMatrixType& T = schurOfA.matrixT();

  VERIFY(U.allFinite() && T.allFinite());
  VERIFY_IS_APPROX(U * U.adjoint(), ComplexMatrixType::Identity(size, size));
  // Both Frobenius norms overflow at this scale, so isApprox would compare infinity against infinity and any T would
  // pass. Compare at unit scale instead; huge is not a power of two, so this costs one rounding per coefficient.
  const ComplexScalar invHuge = ComplexScalar(RealScalar(1) / huge);
  VERIFY_IS_APPROX(ComplexMatrixType(A.template cast<ComplexScalar>() * invHuge),
                   ComplexMatrixType(U * T * U.adjoint() * invHuge));
}

// A NaN must not be scaled away. maxCoeff has to propagate it, or a matrix that is zero apart from a NaN looks like
// the zero matrix and reports a successful decomposition. Which coefficient carries the NaN matters, because the
// default max reduction keeps whichever operand it visits first.
template <typename MatrixType>
void schur_nan_only(Index size) {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename MatrixType::RealScalar RealScalar;

  for (Index k = 0; k < size; ++k) {
    MatrixType A = MatrixType::Zero(size, size);
    A(k, k) = Scalar(std::numeric_limits<RealScalar>::quiet_NaN());

    ComplexSchur<MatrixType> schurOfA(A);
    VERIFY_IS_NOT_EQUAL(schurOfA.info(), Success);
  }
}

// The zero matrix has no exponent to normalize by; it must still decompose trivially.
template <typename MatrixType>
void schur_zero(Index size) {
  typedef typename ComplexSchur<MatrixType>::ComplexMatrixType ComplexMatrixType;

  const MatrixType A = MatrixType::Zero(size, size);
  ComplexSchur<MatrixType> schurOfA(A);
  VERIFY_IS_EQUAL(schurOfA.info(), Success);
  VERIFY(schurOfA.matrixT().isZero());
  VERIFY_IS_EQUAL(schurOfA.matrixU(), ComplexMatrixType::Identity(size, size));
}

EIGEN_DECLARE_TEST(schur_complex) {
  CALL_SUBTEST_1((schur<Matrix4cd>()));
  CALL_SUBTEST_2((schur<MatrixXcf>(internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 4))));
  CALL_SUBTEST_3((schur<Matrix<std::complex<float>, 1, 1> >()));
  CALL_SUBTEST_4((schur<Matrix<float, 3, 3, Eigen::RowMajor> >()));

  // Test problem size constructors
  CALL_SUBTEST_5(ComplexSchur<MatrixXf>(10));

  CALL_SUBTEST_6((schur_underflow_scale<Matrix4cd>(4)));
  CALL_SUBTEST_6((schur_underflow_scale<MatrixXcf>(8)));
  CALL_SUBTEST_6((schur_underflow_scale<MatrixXf>(8)));
  {
    // Every square this reconstruction compares is subnormal, so run it again with subnormals flushed to zero: that
    // is the configuration in which the unscaled reduction it guards against also passes.
    Eigen::ScopedFlushToZero flush_to_zero;
    if (flush_to_zero.isSupported()) {
      CALL_SUBTEST_6((schur_underflow_scale<Matrix4cd>(4)));
      CALL_SUBTEST_6((schur_underflow_scale<MatrixXcf>(8)));
      CALL_SUBTEST_6((schur_underflow_scale<MatrixXf>(8)));
    }
  }

  CALL_SUBTEST_7((schur_subnormal_scale<Matrix4cd>(4, false)));
  CALL_SUBTEST_7((schur_subnormal_scale<MatrixXcf>(8, false)));
  CALL_SUBTEST_7((schur_subnormal_scale<MatrixXf>(8, false)));
  CALL_SUBTEST_7((schur_subnormal_scale<Matrix4cd>(4, true)));
  CALL_SUBTEST_7((schur_subnormal_scale<MatrixXcf>(8, true)));
  CALL_SUBTEST_7((schur_subnormal_scale<MatrixXf>(8, true)));

  CALL_SUBTEST_8((schur_dynamic_range<Matrix4cd>(4)));
  CALL_SUBTEST_8((schur_dynamic_range<MatrixXcf>(8)));
  CALL_SUBTEST_8((schur_dynamic_range<MatrixXf>(8)));

  CALL_SUBTEST_9((schur_nan_only<Matrix4cd>(4)));
  CALL_SUBTEST_9((schur_nan_only<MatrixXcf>(8)));
  CALL_SUBTEST_9((schur_nan_only<MatrixXf>(8)));

  CALL_SUBTEST_10((schur_zero<Matrix4cd>(4)));
  CALL_SUBTEST_10((schur_zero<MatrixXcf>(8)));
  CALL_SUBTEST_10((schur_zero<MatrixXf>(8)));
}
