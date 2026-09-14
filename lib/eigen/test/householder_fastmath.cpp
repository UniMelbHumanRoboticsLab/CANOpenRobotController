// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include "main.h"
#include <Eigen/QR>

// -ffinite-math-only lets the compiler assume every result is finite, so isfinite(), isinf(), and isnan() fold to
// constants in this translation unit and cannot be used to check a result. A comparison against a finite bound is
// still evaluated, and rejects infinities and NaNs alike.
template <typename RealScalar>
bool is_representable(const RealScalar& x) {
  return numext::abs(x) <= NumTraits<RealScalar>::highest();
}

// makeHouseholder must reach the scaled path whenever the direct construction would square the input out of range.
// The dispatch has to be decided from the input magnitudes, because a test on the overflowed sum does not survive
// here.
template <typename RealScalar>
void householder_overflow(const Matrix<RealScalar, Dynamic, 1>& vector) {
  typedef Matrix<RealScalar, Dynamic, 1> VectorType;
  typedef Matrix<RealScalar, Dynamic, Dynamic> MatrixType;
  const Index size = vector.size();

  VectorType essential(size - 1);
  RealScalar tau;
  RealScalar beta;
  vector.makeHouseholder(essential, tau, beta);

  VERIFY(is_representable(tau));
  VERIFY(is_representable(beta));
  VERIFY(is_representable(essential.cwiseAbs().maxCoeff()));

  // tau and the essential vector are scale invariant and beta is homogeneous, so applying the reflector to a
  // power-of-two rescaling of the input is exact and keeps the check itself inside the range.
  int exponent = 0;
  (void)std::frexp(vector.cwiseAbs().maxCoeff(), &exponent);
  const RealScalar scale = std::ldexp(RealScalar(1), exponent);
  const VectorType scaled = vector / scale;

  VectorType v(size);
  v[0] = RealScalar(1);
  v.tail(size - 1) = essential;
  const MatrixType reflector = MatrixType::Identity(size, size) - tau * v * v.transpose();

  VERIFY_IS_APPROX(reflector.transpose() * reflector, MatrixType::Identity(size, size));
  const VectorType reflected = reflector * scaled;
  VERIFY_IS_APPROX(reflected[0], beta / scale);
  VERIFY_IS_MUCH_SMALLER_THAN(reflected.tail(size - 1).cwiseAbs().maxCoeff(), RealScalar(1));
}

template <typename RealScalar>
void householder_overflow_cases() {
  typedef Matrix<RealScalar, Dynamic, 1> VectorType;
  const RealScalar huge = numext::sqrt(NumTraits<RealScalar>::highest()) * RealScalar(4);

  // The head alone squares out of range.
  VectorType head(2);
  head << huge, huge / RealScalar(8);
  householder_overflow<RealScalar>(head);

  // A tail component alone squares out of range, with a zero head.
  VectorType tail(3);
  tail << RealScalar(0), huge * RealScalar(0.6), huge * RealScalar(0.8);
  householder_overflow<RealScalar>(tail);

  // Accumulated overflow: every square is representable, their sum is not.
  householder_overflow<RealScalar>(VectorType::Constant(65, huge / RealScalar(4)));

  // A QR factorization of a matrix at this scale has to compose from those reflectors.
  const Index rows = 6, cols = 4;
  Matrix<RealScalar, Dynamic, Dynamic> matrix(rows, cols);
  for (Index j = 0; j < cols; ++j)
    for (Index i = 0; i < rows; ++i) matrix(i, j) = huge * RealScalar(internal::random<double>(-1.0, 1.0));
  const HouseholderQR<Matrix<RealScalar, Dynamic, Dynamic> > qr(matrix);
  const Matrix<RealScalar, Dynamic, Dynamic> q =
      qr.householderQ() * Matrix<RealScalar, Dynamic, Dynamic>::Identity(rows, cols);
  const Matrix<RealScalar, Dynamic, Dynamic> r =
      qr.matrixQR().topRows(cols).template triangularView<Upper>().toDenseMatrix();
  const Matrix<RealScalar, Dynamic, Dynamic> scaledMatrix = matrix / huge;
  VERIFY_IS_APPROX((q * r) / huge, scaledMatrix);
}

EIGEN_DECLARE_TEST(householder_fastmath) {
  CALL_SUBTEST_1(householder_overflow_cases<float>());
  CALL_SUBTEST_2(householder_overflow_cases<double>());
}
