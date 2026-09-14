// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009-2010 Benoit Jacob <jacob.benoit.1@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "main.h"
#include "fp_control.h"
#include <Eigen/QR>

namespace reordered_complex {

template <typename Real>
struct Complex {
  Complex() : imag_value(0), real_value(0) {}
  Complex(Real real) : imag_value(0), real_value(real) {}
  Complex(Real real, Real imag) : imag_value(imag), real_value(real) {}

  Complex operator+(const Complex& other) const {
    return Complex(real_value + other.real_value, imag_value + other.imag_value);
  }
  Complex operator-() const { return Complex(-real_value, -imag_value); }
  Complex operator-(const Complex& other) const {
    return Complex(real_value - other.real_value, imag_value - other.imag_value);
  }
  Complex operator*(const Complex& other) const {
    return Complex(real_value * other.real_value - imag_value * other.imag_value,
                   real_value * other.imag_value + imag_value * other.real_value);
  }
  Complex operator/(const Complex& other) const {
    const Real denominator = other.real_value * other.real_value + other.imag_value * other.imag_value;
    return Complex((real_value * other.real_value + imag_value * other.imag_value) / denominator,
                   (imag_value * other.real_value - real_value * other.imag_value) / denominator);
  }

  bool operator==(const Complex& other) const {
    return real_value == other.real_value && imag_value == other.imag_value;
  }
  bool operator!=(const Complex& other) const { return !(*this == other); }

  friend Complex operator+(const Real& lhs, const Complex& rhs) {
    return Complex(lhs + rhs.real_value, rhs.imag_value);
  }
  friend Complex operator-(const Real& lhs, const Complex& rhs) {
    return Complex(lhs - rhs.real_value, -rhs.imag_value);
  }
  friend Complex operator*(const Real& lhs, const Complex& rhs) {
    return Complex(lhs * rhs.real_value, lhs * rhs.imag_value);
  }
  friend Complex operator*(const Complex& lhs, const Real& rhs) {
    return Complex(lhs.real_value * rhs, lhs.imag_value * rhs);
  }
  friend Complex operator/(const Complex& lhs, const Real& rhs) {
    return Complex(lhs.real_value / rhs, lhs.imag_value / rhs);
  }

  // The reversed order is intentional: algorithms must not infer component access from IsComplex.
  Real imag_value;
  Real real_value;
};

template <typename Real>
Real real(const Complex<Real>& value) {
  return value.real_value;
}
template <typename Real>
Real imag(const Complex<Real>& value) {
  return value.imag_value;
}
template <typename Real>
Complex<Real> conj(const Complex<Real>& value) {
  return Complex<Real>(value.real_value, -value.imag_value);
}
template <typename Real>
Real abs(const Complex<Real>& value) {
  return numext::hypot(value.real_value, value.imag_value);
}

}  // namespace reordered_complex

namespace Eigen {
template <typename Real>
struct NumTraits<reordered_complex::Complex<Real>> : NumTraits<Real> {
  static constexpr bool IsComplex = true;
};
}  // namespace Eigen

template <typename MatrixType>
void householder(const MatrixType& m) {
  static bool even = true;
  even = !even;
  /* this test covers the following files:
     Householder.h
  */
  Index rows = m.rows();
  Index cols = m.cols();

  typedef typename MatrixType::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, MatrixType::RowsAtCompileTime, 1> VectorType;
  typedef Matrix<Scalar, internal::decrement_size<MatrixType::RowsAtCompileTime>::value, 1> EssentialVectorType;
  typedef Matrix<Scalar, MatrixType::RowsAtCompileTime, MatrixType::RowsAtCompileTime> SquareMatrixType;
  typedef Matrix<Scalar, Dynamic, MatrixType::ColsAtCompileTime> HBlockMatrixType;
  typedef Matrix<Scalar, Dynamic, 1> HCoeffsVectorType;

  typedef Matrix<Scalar, MatrixType::ColsAtCompileTime, MatrixType::RowsAtCompileTime> TMatrixType;

  Matrix<Scalar, internal::max_size_prefer_dynamic(MatrixType::RowsAtCompileTime, MatrixType::ColsAtCompileTime), 1>
      _tmp((std::max)(rows, cols));
  Scalar* tmp = &_tmp.coeffRef(0, 0);

  Scalar beta;
  RealScalar alpha;
  EssentialVectorType essential;

  VectorType v1 = VectorType::Random(rows), v2;
  v2 = v1;
  v1.makeHouseholder(essential, beta, alpha);
  v1.applyHouseholderOnTheLeft(essential, beta, tmp);
  VERIFY_IS_APPROX(v1.norm(), v2.norm());
  if (rows >= 2) VERIFY_IS_MUCH_SMALLER_THAN(v1.tail(rows - 1).norm(), v1.norm());
  v1 = VectorType::Random(rows);
  v2 = v1;
  v1.applyHouseholderOnTheLeft(essential, beta, tmp);
  VERIFY_IS_APPROX(v1.norm(), v2.norm());

  // reconstruct householder matrix:
  SquareMatrixType id, H1, H2;
  id.setIdentity(rows, rows);
  H1 = H2 = id;
  VectorType vv(rows);
  vv << Scalar(1), essential;
  H1.applyHouseholderOnTheLeft(essential, beta, tmp);
  H2.applyHouseholderOnTheRight(essential, beta, tmp);
  VERIFY_IS_APPROX(H1, H2);
  VERIFY_IS_APPROX(H1, id - beta * vv * vv.adjoint());

  MatrixType m1(rows, cols), m2(rows, cols);

  v1 = VectorType::Random(rows);
  if (even) v1.tail(rows - 1).setZero();
  m1.colwise() = v1;
  m2 = m1;
  m1.col(0).makeHouseholder(essential, beta, alpha);
  m1.applyHouseholderOnTheLeft(essential, beta, tmp);
  VERIFY_IS_APPROX(m1.norm(), m2.norm());
  if (rows >= 2) VERIFY_IS_MUCH_SMALLER_THAN(m1.block(1, 0, rows - 1, cols).norm(), m1.norm());
  VERIFY_IS_MUCH_SMALLER_THAN(numext::imag(m1(0, 0)), numext::real(m1(0, 0)));
  VERIFY_IS_APPROX(numext::real(m1(0, 0)), alpha);

  v1 = VectorType::Random(rows);
  if (even) v1.tail(rows - 1).setZero();
  SquareMatrixType m3(rows, rows), m4(rows, rows);
  m3.rowwise() = v1.transpose();
  m4 = m3;
  m3.row(0).makeHouseholder(essential, beta, alpha);
  m3.applyHouseholderOnTheRight(essential.conjugate(), beta, tmp);
  VERIFY_IS_APPROX(m3.norm(), m4.norm());
  if (rows >= 2) VERIFY_IS_MUCH_SMALLER_THAN(m3.block(0, 1, rows, rows - 1).norm(), m3.norm());
  VERIFY_IS_MUCH_SMALLER_THAN(numext::imag(m3(0, 0)), numext::real(m3(0, 0)));
  VERIFY_IS_APPROX(numext::real(m3(0, 0)), alpha);

  // test householder sequence on the left with a shift

  Index shift = internal::random<Index>(0, std::max<Index>(rows - 2, 0));
  Index brows = rows - shift;
  m1.setRandom(rows, cols);
  HBlockMatrixType hbm = m1.block(shift, 0, brows, cols);
  HouseholderQR<HBlockMatrixType> qr(hbm);
  m2 = m1;
  m2.block(shift, 0, brows, cols) = qr.matrixQR();
  HCoeffsVectorType hc = qr.hCoeffs().conjugate();
  HouseholderSequence<MatrixType, HCoeffsVectorType> hseq(m2, hc);
  hseq.setLength(hc.size()).setShift(shift);
  VERIFY(hseq.length() == hc.size());
  VERIFY(hseq.shift() == shift);

  MatrixType m5 = m2;
  m5.block(shift, 0, brows, cols).template triangularView<StrictlyLower>().setZero();
  VERIFY_IS_APPROX(hseq * m5, m1);  // test applying hseq directly
  m3 = hseq;
  VERIFY_IS_APPROX(m3 * m5, m1);  // test evaluating hseq to a dense matrix, then applying

  SquareMatrixType hseq_mat = hseq;
  SquareMatrixType hseq_mat_conj = hseq.conjugate();
  SquareMatrixType hseq_mat_adj = hseq.adjoint();
  SquareMatrixType hseq_mat_trans = hseq.transpose();
  SquareMatrixType m6 = SquareMatrixType::Random(rows, rows);
  VERIFY_IS_APPROX(hseq_mat.adjoint(), hseq_mat_adj);
  VERIFY_IS_APPROX(hseq_mat.conjugate(), hseq_mat_conj);
  VERIFY_IS_APPROX(hseq_mat.transpose(), hseq_mat_trans);
  VERIFY_IS_APPROX(hseq * m6, hseq_mat * m6);
  VERIFY_IS_APPROX(hseq.adjoint() * m6, hseq_mat_adj * m6);
  VERIFY_IS_APPROX(hseq.conjugate() * m6, hseq_mat_conj * m6);
  VERIFY_IS_APPROX(hseq.transpose() * m6, hseq_mat_trans * m6);
  VERIFY_IS_APPROX(m6 * hseq, m6 * hseq_mat);
  VERIFY_IS_APPROX(m6 * hseq.adjoint(), m6 * hseq_mat_adj);
  VERIFY_IS_APPROX(m6 * hseq.conjugate(), m6 * hseq_mat_conj);
  VERIFY_IS_APPROX(m6 * hseq.transpose(), m6 * hseq_mat_trans);

  // test householder sequence on the right with a shift

  TMatrixType tm2 = m2.transpose();
  HouseholderSequence<TMatrixType, HCoeffsVectorType, OnTheRight> rhseq(tm2, hc);
  rhseq.setLength(hc.size()).setShift(shift);
  VERIFY_IS_APPROX(rhseq * m5, m1);  // test applying rhseq directly
  m3 = rhseq;
  VERIFY_IS_APPROX(m3 * m5, m1);  // test evaluating rhseq to a dense matrix, then applying
}

template <typename MatrixType>
void householder_update(const MatrixType& m) {
  // This test is covering the internal::householder_qr_inplace_update function.
  // At time of writing, there is not public API that exposes this update behavior directly,
  // so we are testing the internal implementation.

  const Index rows = m.rows();
  const Index cols = m.cols();

  typedef typename MatrixType::Scalar Scalar;
  typedef Matrix<Scalar, MatrixType::RowsAtCompileTime, 1> VectorType;
  typedef Matrix<Scalar, Dynamic, 1> HCoeffsVectorType;
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixX;
  typedef Matrix<Scalar, Dynamic, 1> VectorX;

  VectorX tmpOwner(cols);
  Scalar* tmp = tmpOwner.data();

  // The matrix to factorize.
  const MatrixType A = MatrixType::Random(rows, cols);

  // matQR and hCoeffs will hold the factorization of A,
  // built by a sequence of calls to `update`.
  MatrixType matQR(rows, cols);
  HCoeffsVectorType hCoeffs(cols);

  // householder_qr_inplace_update should be able to build a QR factorization one column at a time.
  // We verify this by starting with an empty factorization and 'updating' one column at a time.
  // After each call to update, we should have a QR factorization of the columns presented so far.

  const Index size = (std::min)(rows, cols);  // QR can only go up to 'size' b/c that's full rank.
  for (Index k = 0; k != size; ++k) {
    // Make a copy of the column to prevent any possibility of 'leaking' other parts of A.
    const VectorType newColumn = A.col(k);
    internal::householder_qr_inplace_update(matQR, hCoeffs, newColumn, k, tmp);

    // Verify Property:
    // matQR.leftCols(k+1) and hCoeffs.head(k+1) hold
    // a QR factorization of A.leftCols(k+1).
    // This is the fundamental guarantee of householder_qr_inplace_update.
    {
      const MatrixX matQR_k = matQR.leftCols(k + 1);
      const VectorX hCoeffs_k = hCoeffs.head(k + 1);
      MatrixX R = matQR_k.template triangularView<Upper>();
      MatrixX QxR = householderSequence(matQR_k, hCoeffs_k.conjugate()) * R;
      VERIFY_IS_APPROX(QxR, A.leftCols(k + 1));
    }

    // Verify Property:
    // A sequence of calls to 'householder_qr_inplace_update'
    // should produce the same result as 'householder_qr_inplace_unblocked'.
    // This is a property of the current implementation.
    // If these implementations diverge in the future,
    // then simply delete the test of this property.
    {
      MatrixX QR_at_once = A.leftCols(k + 1);
      VectorX hCoeffs_at_once(k + 1);
      internal::householder_qr_inplace_unblocked(QR_at_once, hCoeffs_at_once, tmp);
      VERIFY_IS_APPROX(QR_at_once, matQR.leftCols(k + 1));
      VERIFY_IS_APPROX(hCoeffs_at_once, hCoeffs.head(k + 1));
    }
  }

  // Verify Property:
  // We can go back and update any column to have a new value,
  // and get a QR factorization of the columns up to that one.
  {
    const Index k = internal::random<Index>(0, size - 1);
    VectorType newColumn = VectorType::Random(rows);
    internal::householder_qr_inplace_update(matQR, hCoeffs, newColumn, k, tmp);

    const MatrixX matQR_k = matQR.leftCols(k + 1);
    const VectorX hCoeffs_k = hCoeffs.head(k + 1);
    MatrixX R = matQR_k.template triangularView<Upper>();
    MatrixX QxR = householderSequence(matQR_k, hCoeffs_k.conjugate()) * R;
    VERIFY_IS_APPROX(QxR.leftCols(k), A.leftCols(k));
    VERIFY_IS_APPROX(QxR.col(k), newColumn);
  }
}

template <typename Scalar>
void householder_blocked_right_regression() {
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixX;
  typedef Matrix<Scalar, Dynamic, 1> VectorX;
  typedef HouseholderSequence<MatrixX, VectorX> LeftSequence;
  typedef HouseholderSequence<MatrixX, VectorX, OnTheRight> RightSequence;

  const Index rows = 256;
  const Index cols = 128;
  const Index shift = 17;

  // Force the blocked path added for right-side Householder application.
  VERIFY(cols >= 48 && rows - shift >= 4 * 48);

  MatrixX input = MatrixX::Random(rows, cols);
  MatrixX qr_input = input.block(shift, 0, rows - shift, cols);
  HouseholderQR<MatrixX> qr(qr_input);

  MatrixX packed = input;
  packed.block(shift, 0, rows - shift, cols) = qr.matrixQR();
  VectorX hcoeffs = qr.hCoeffs().conjugate();

  LeftSequence hseq(packed, hcoeffs);
  hseq.setLength(hcoeffs.size()).setShift(shift);
  MatrixX dense_left = MatrixX(hseq);

  MatrixX packed_transposed = packed.transpose();
  RightSequence rhseq(packed_transposed, hcoeffs);
  rhseq.setLength(hcoeffs.size()).setShift(shift);
  MatrixX dense_right = MatrixX(rhseq);

  MatrixX left_rhs = MatrixX::Random(rows, rows + 9);
  MatrixX right_lhs = MatrixX::Random(rows + 7, rows);

  VERIFY_IS_APPROX(hseq * left_rhs, dense_left * left_rhs);
  VERIFY_IS_APPROX(hseq.adjoint() * left_rhs, dense_left.adjoint() * left_rhs);
  VERIFY_IS_APPROX(right_lhs * hseq, right_lhs * dense_left);
  VERIFY_IS_APPROX(right_lhs * hseq.adjoint(), right_lhs * dense_left.adjoint());

  VERIFY_IS_APPROX(rhseq * left_rhs, dense_right * left_rhs);
  VERIFY_IS_APPROX(rhseq.adjoint() * left_rhs, dense_right.adjoint() * left_rhs);
  VERIFY_IS_APPROX(right_lhs * rhseq, right_lhs * dense_right);
  VERIFY_IS_APPROX(right_lhs * rhseq.adjoint(), right_lhs * dense_right.adjoint());
}

template <typename VectorType, typename EssentialType>
void verify_real_householder_result(const VectorType& vector, const EssentialType& essential,
                                    const typename VectorType::Scalar& tau,
                                    const typename VectorType::RealScalar& beta) {
  typedef typename VectorType::RealScalar RealScalar;
  const long double epsilon = static_cast<long double>(NumTraits<RealScalar>::epsilon());
  long double input_squared_norm = 0;
  long double householder_squared_norm = 1;
  long double inner_product = static_cast<long double>(vector.coeff(0));
  for (Index i = 0; i < vector.size(); ++i) {
    const long double value = static_cast<long double>(vector.coeff(i));
    input_squared_norm += value * value;
    if (i > 0) {
      const long double essential_value = static_cast<long double>(essential.coeff(i - 1));
      householder_squared_norm += essential_value * essential_value;
      inner_product += essential_value * value;
    }
  }

  const long double input_norm = std::sqrt(input_squared_norm);
  const long double tau_wide = static_cast<long double>(tau);
  const long double bound = 64 * epsilon * input_norm;
  const long double transformed_head = static_cast<long double>(vector.coeff(0)) - tau_wide * inner_product;
  VERIFY(std::abs(transformed_head - static_cast<long double>(beta)) <= bound);

  long double transformed_tail_squared_norm = 0;
  for (Index i = 1; i < vector.size(); ++i) {
    const long double essential_value = static_cast<long double>(essential.coeff(i - 1));
    const long double transformed =
        static_cast<long double>(vector.coeff(i)) - tau_wide * essential_value * inner_product;
    transformed_tail_squared_norm += transformed * transformed;
  }
  VERIFY(std::sqrt(transformed_tail_squared_norm) <= bound);
  VERIFY(std::abs(2 * tau_wide - tau_wide * tau_wide * householder_squared_norm) <= 64 * epsilon);
}

template <typename RealScalar>
void verify_complex_zero_tail(RealScalar coefficient) {
  typedef std::complex<RealScalar> Scalar;
  typedef std::complex<long double> WideScalar;
  Matrix<Scalar, 2, 1> vector;
  vector << Scalar(0, coefficient), Scalar(0, 0);
  Matrix<Scalar, 1, 1> essential;
  Scalar tau;
  RealScalar beta;

  vector.makeHouseholder(essential, tau, beta);

  const WideScalar x0(0, static_cast<long double>(coefficient));
  const WideScalar x1(0, 0);
  const WideScalar essential_wide(static_cast<long double>(numext::real(essential[0])),
                                  static_cast<long double>(numext::imag(essential[0])));
  const WideScalar tau_wide(static_cast<long double>(numext::real(tau)), static_cast<long double>(numext::imag(tau)));
  const WideScalar inner_product = x0 + std::conj(essential_wide) * x1;
  const WideScalar transformed_head = x0 - tau_wide * inner_product;
  const WideScalar transformed_tail = x1 - tau_wide * essential_wide * inner_product;
  const long double epsilon = static_cast<long double>(NumTraits<RealScalar>::epsilon());
  const long double bound = 16 * epsilon * std::abs(x0);

  VERIFY(std::abs(transformed_head - WideScalar(static_cast<long double>(beta), 0)) <= bound);
  VERIFY(std::abs(transformed_tail) <= bound);
  VERIFY(std::abs(2 * numext::real(tau_wide) - std::norm(tau_wide) * (1 + std::norm(essential_wide))) <= 16 * epsilon);
}

template <typename RealScalar>
void verify_low_precision_complex_zero_tail() {
  typedef std::complex<RealScalar> Scalar;
  const RealScalar zero = RealScalar(0);
  const RealScalar coefficient = RealScalar(0.25f);
  Matrix<Scalar, 2, 1> vector;
  vector << Scalar(zero, coefficient), Scalar(zero, zero);
  Matrix<Scalar, 1, 1> essential;
  Scalar tau;
  RealScalar beta;

  vector.makeHouseholder(essential, tau, beta);

  VERIFY_IS_EQUAL(numext::real(tau), RealScalar(1));
  VERIFY_IS_EQUAL(numext::imag(tau), RealScalar(-1));
  VERIFY_IS_EQUAL(beta, -coefficient);
  VERIFY_IS_EQUAL(numext::real(essential[0]), zero);
  VERIFY_IS_EQUAL(numext::imag(essential[0]), zero);
}

template <typename RealScalar>
void verify_complex_nan_head() {
  typedef std::complex<RealScalar> Scalar;
  const RealScalar nan = NumTraits<RealScalar>::quiet_NaN();
  Matrix<Scalar, 2, 1> vector;
  vector << Scalar(1, nan), Scalar(0, 0);
  Matrix<Scalar, 1, 1> essential;
  Scalar tau;
  RealScalar beta;

  vector.makeHouseholder(essential, tau, beta);

  VERIFY((numext::isnan)(beta));
}

void verify_custom_complex_small_tail() {
  typedef reordered_complex::Complex<float> Scalar;
  const float coefficient = 2e-16f;
  Matrix<Scalar, 2, 1> vector;
  vector << Scalar(0), Scalar(coefficient);
  Matrix<Scalar, 1, 1> essential;
  Scalar tau;
  float beta;

  vector.makeHouseholder(essential, tau, beta);

  VERIFY_IS_EQUAL(numext::real(tau), 1.0f);
  VERIFY_IS_EQUAL(numext::imag(tau), 0.0f);
  VERIFY_IS_EQUAL(beta, -coefficient);
  VERIFY_IS_EQUAL(numext::real(essential[0]), 1.0f);
  VERIFY_IS_EQUAL(numext::imag(essential[0]), 0.0f);
}

EIGEN_DONT_INLINE void verify_householder_flushed_tail() {
  volatile float normal_min_input = (std::numeric_limits<float>::min)();
  const float normal_min = normal_min_input;
  const float normal_root = numext::sqrt(normal_min);
  const float dominant = normal_root * numext::sqrt(1.1f);
  const float minor = normal_root * numext::sqrt(0.1f / 7.0f);
  VectorXf vector = VectorXf::Constant(9, minor);
  vector[0] = 0.0f;
  vector[1] = dominant;
  VectorXf essential(8);
  float tau;
  float beta;

  vector.makeHouseholder(essential, tau, beta);

  verify_real_householder_result(vector, essential, tau, beta);
}

EIGEN_DONT_INLINE void verify_householder_dimension_scaled_tail() {
  // The dominant square is just above the length-independent threshold, while the many minor squares are normal
  // inputs whose individually subnormal products become significant in aggregate.
  constexpr Index size = 262145;
  const double normal_min = static_cast<double>((std::numeric_limits<float>::min)());
  const double epsilon = static_cast<double>(NumTraits<float>::epsilon());
  const float dominant = static_cast<float>(std::sqrt(1.01 * normal_min / epsilon));
  const float minor = static_cast<float>(std::sqrt(0.5 * normal_min));
  VectorXf vector = VectorXf::Constant(size, minor);
  vector[0] = 0.0f;
  vector[1] = dominant;
  VectorXf essential(size - 1);
  float tau;
  float beta;

  vector.makeHouseholder(essential, tau, beta);

  verify_real_householder_result(vector, essential, tau, beta);
  const double expected_norm =
      std::sqrt(double(dominant) * double(dominant) + double(size - 2) * double(minor) * double(minor));
  VERIFY(std::abs(double(beta) + expected_norm) <= 8 * epsilon * expected_norm);
}

void householder_small_tail_layouts() {
  const float coefficient = 1e-20f;
  Vector4f column;
  column << 0.0f, coefficient, -2.0f * coefficient, 3.0f * coefficient;
  Vector3f column_essential;
  float tau;
  float beta;
  column.makeHouseholder(column_essential, tau, beta);
  verify_real_householder_result(column, column_essential, tau, beta);

  RowVector4f row = column.transpose();
  RowVector3f row_essential;
  row.makeHouseholder(row_essential, tau, beta);
  verify_real_householder_result(row, row_essential, tau, beta);

  Vector4f in_place = column;
  in_place.makeHouseholderInPlace(tau, beta);
  verify_real_householder_result(column, in_place.tail<3>(), tau, beta);

  Vector4f aliased = column;
  const Vector4f original = aliased;
  auto aliased_essential = aliased.tail<3>();
  aliased.makeHouseholder(aliased_essential, tau, beta);
  verify_real_householder_result(original, aliased_essential, tau, beta);

  float input_storage[8] = {0.0f, 11.0f, coefficient, 12.0f, -2.0f * coefficient, 13.0f, 3.0f * coefficient, 14.0f};
  float essential_storage[6] = {0.0f, 21.0f, 0.0f, 22.0f, 0.0f, 23.0f};
  typedef Map<VectorXf, Unaligned, InnerStride<2>> StridedVector;
  StridedVector strided_input(input_storage, 4, InnerStride<2>());
  StridedVector strided_essential(essential_storage, 3, InnerStride<2>());
  strided_input.makeHouseholder(strided_essential, tau, beta);
  verify_real_householder_result(strided_input, strided_essential, tau, beta);
  VERIFY_IS_EQUAL(input_storage[1], 11.0f);
  VERIFY_IS_EQUAL(input_storage[3], 12.0f);
  VERIFY_IS_EQUAL(input_storage[5], 13.0f);
  VERIFY_IS_EQUAL(input_storage[7], 14.0f);
  VERIFY_IS_EQUAL(essential_storage[1], 21.0f);
  VERIFY_IS_EQUAL(essential_storage[3], 22.0f);
  VERIFY_IS_EQUAL(essential_storage[5], 23.0f);
}

void householder_small_tail() {
  {
    const float coefficient = 1e-20f;
    Vector2f vector(0.0f, coefficient);
    VectorXf essential(1);
    float tau;
    float beta;

    vector.makeHouseholder(essential, tau, beta);

    VERIFY_IS_APPROX(tau, 1.0f);
    VERIFY_IS_APPROX(beta, -coefficient);
    VERIFY_IS_APPROX(essential[0], 1.0f);
  }

  {
    const float largest = (std::numeric_limits<float>::max)();
    Vector2f vector(largest, 1e-20f);
    VectorXf essential(1);
    float tau;
    float beta;

    vector.makeHouseholder(essential, tau, beta);

    VERIFY_IS_EQUAL(tau, 0.0f);
    VERIFY_IS_EQUAL(beta, largest);
    VERIFY_IS_EQUAL(essential[0], 0.0f);
  }

  {
    const float largest = (std::numeric_limits<float>::max)();
    const Vector2cf vector(std::complex<float>(0.0f, largest), std::complex<float>(1e-20f, 0.0f));
    VectorXcf essential(1);
    std::complex<float> tau;
    float beta;

    vector.makeHouseholder(essential, tau, beta);

    VERIFY_IS_EQUAL(tau, std::complex<float>(1.0f, -1.0f));
    VERIFY_IS_EQUAL(beta, -largest);
    VERIFY_IS_EQUAL(essential[0], std::complex<float>(0.0f, 0.0f));
    Vector2cf householder;
    householder << std::complex<float>(1.0f, 0.0f), essential;
    const Matrix2cf transform = Matrix2cf::Identity() - tau * householder * householder.adjoint();
    VERIFY_IS_APPROX(transform.adjoint() * transform, Matrix2cf::Identity());
  }

  {
    volatile float denormInput = (std::numeric_limits<float>::denorm_min)();
    const float denorm = denormInput;
    // Subnormal operands are indistinguishable from zero when the target flushes them.
    if (denorm > 0.0f && denorm + denorm > 0.0f) {
      const Vector2f vector = Vector2f::Constant(denorm);
      VectorXf essential(1);
      float tau;
      float beta;

      vector.makeHouseholder(essential, tau, beta);

      Vector2f householder;
      householder << 1.0f, essential;
      const Matrix2f transform = Matrix2f::Identity() - tau * householder * householder.transpose();
      VERIFY_IS_APPROX(transform.transpose() * transform, Matrix2f::Identity());
      const Vector2d transformed = transform.cast<double>() * vector.cast<double>();
      const Vector2d expected(-numext::sqrt(2.0) * double(denorm), 0.0);
      VERIFY((transformed - expected).norm() <= 4.0 * double(NumTraits<float>::epsilon()) * expected.norm());

      const std::complex<float> complexDenorm(denorm, denorm);
      const Vector2cf complexVector = Vector2cf::Constant(complexDenorm);
      VectorXcf complexEssential(1);
      std::complex<float> complexTau;
      float complexBeta;

      complexVector.makeHouseholder(complexEssential, complexTau, complexBeta);

      Vector2cf complexHouseholder;
      complexHouseholder[0] = std::complex<float>(1.0f, 0.0f);
      complexHouseholder[1] = complexEssential[0];
      const Matrix2cf complexTransform =
          Matrix2cf::Identity() - complexTau * complexHouseholder * complexHouseholder.adjoint();
      VERIFY_IS_APPROX(complexTransform.adjoint() * complexTransform, Matrix2cf::Identity());
      const Vector2cd complexTransformed =
          complexTransform.cast<std::complex<double>>() * complexVector.cast<std::complex<double>>();
      const Vector2cd complexExpected(std::complex<double>(-2.0 * double(denorm), 0.0), std::complex<double>(0.0, 0.0));
      VERIFY((complexTransformed - complexExpected).norm() <=
             8.0 * double(NumTraits<float>::epsilon()) * complexExpected.norm());
    }
  }

  {
    Vector2d vector(0.0, 1e-160);
    VectorXd essential(1);
    double tau;
    double beta;

    vector.makeHouseholder(essential, tau, beta);

    verify_real_householder_result(vector, essential, tau, beta);
  }

  verify_complex_zero_tail<float>(1e-20f);
  verify_complex_zero_tail<double>(1e-160);
  verify_low_precision_complex_zero_tail<half>();
  verify_low_precision_complex_zero_tail<bfloat16>();
  verify_complex_nan_head<float>();
  verify_complex_nan_head<double>();
  verify_custom_complex_small_tail();

  {
    const float coefficient = 1e-20f;
    const Matrix2cf matrix = std::complex<float>(0.0f, coefficient) * Matrix2cf::Identity();
    const HouseholderQR<Matrix2cf> qr(matrix);
    const Matrix2cf q = qr.householderQ();
    const Matrix2cf r = qr.matrixQR().template triangularView<Upper>();
    const Matrix2cd matrix_wide = matrix.cast<std::complex<double>>();
    const Matrix2cd reconstructed = (q * r).cast<std::complex<double>>();
    const double relative_residual = (reconstructed - matrix_wide).norm() / matrix_wide.norm();
    VERIFY(relative_residual <= 16 * double(NumTraits<float>::epsilon()));
  }

  verify_householder_flushed_tail();
  verify_householder_dimension_scaled_tail();
  {
    Eigen::ScopedFlushToZero flush_to_zero;
    if (flush_to_zero.isSupported()) {
      verify_householder_flushed_tail();
      verify_householder_dimension_scaled_tail();
    }
  }

  householder_small_tail_layouts();

  constexpr Index size = 65;
  const float coefficient = 1e-22f;
  VectorXf vector = VectorXf::Constant(size, coefficient);
  VectorXf essential(size - 1);
  float tau;
  float beta;

  vector.makeHouseholder(essential, tau, beta);

  VERIFY(!numext::is_exactly_zero(tau));
  VERIFY_IS_APPROX(beta / coefficient, -numext::sqrt(float(size)));
  VERIFY_IS_APPROX(2.0f * tau, tau * tau * (1.0f + essential.squaredNorm()));
}

// tau and the essential vector are scale invariant and beta is homogeneous, so rescaling by a power of two is exact.
// It keeps the long double reference below the squaring overflow that this path exists to avoid, which matters where
// long double is only as wide as double.
template <typename VectorType, typename EssentialType>
void verify_large_householder_result(const VectorType& vector, const EssentialType& essential,
                                     const typename VectorType::Scalar& tau,
                                     const typename VectorType::RealScalar& beta) {
  typedef typename VectorType::RealScalar RealScalar;
  int exponent = 0;
  (void)std::frexp(vector.cwiseAbs().maxCoeff(), &exponent);
  const RealScalar scale = std::ldexp(RealScalar(1), exponent);
  verify_real_householder_result((vector / scale).eval(), essential, tau, RealScalar(beta / scale));
}

void householder_large_components() {
  {
    // The head coefficient squares out of the float range; the tail coefficient does not. The direct
    // construction forms their sum regardless, so it produced beta = -inf and tau = NaN.
    Vector2f vector(4e19f, -5.2e18f);
    Matrix<float, 1, 1> essential;
    float tau;
    float beta;

    vector.makeHouseholder(essential, tau, beta);

    VERIFY((numext::isfinite)(tau));
    VERIFY((numext::isfinite)(beta));
    VERIFY((numext::isfinite)(essential[0]));
    verify_large_householder_result(vector, essential, tau, beta);
  }

  {
    VectorXf vector(3);
    vector << 1e20f, 2e20f, 3e20f;
    VectorXf essential(2);
    float tau;
    float beta;

    vector.makeHouseholder(essential, tau, beta);

    verify_large_householder_result(vector, essential, tau, beta);
  }

  {
    // Accumulated overflow: every square is representable, their sum is not.
    VectorXf vector = VectorXf::Constant(65, 1e19f);
    VectorXf essential(64);
    float tau;
    float beta;

    vector.makeHouseholder(essential, tau, beta);

    verify_large_householder_result(vector, essential, tau, beta);
  }

  {
    VectorXd vector(3);
    vector << 1e160, -2e160, 3e160;
    VectorXd essential(2);
    double tau;
    double beta;

    vector.makeHouseholder(essential, tau, beta);

    verify_large_householder_result(vector, essential, tau, beta);
  }

  {
    const float largest = (std::numeric_limits<float>::max)();
    const Vector2cf vector(std::complex<float>(largest, 0.0f), std::complex<float>(largest, 0.0f));
    Matrix<std::complex<float>, 1, 1> essential;
    std::complex<float> tau;
    float beta;

    vector.makeHouseholder(essential, tau, beta);

    // The true norm exceeds the float range, so beta cannot be represented; the reflector itself still must be.
    VERIFY((numext::isfinite)(numext::real(tau)));
    VERIFY((numext::isfinite)(numext::imag(tau)));
    VERIFY((numext::isfinite)(numext::real(essential[0])));
    VERIFY((numext::isfinite)(numext::imag(essential[0])));
    Vector2cf householder;
    householder << std::complex<float>(1.0f, 0.0f), essential;
    const Matrix2cf transform = Matrix2cf::Identity() - tau * householder * householder.adjoint();
    VERIFY_IS_APPROX(transform.adjoint() * transform, Matrix2cf::Identity());
  }

  {
    // Reflectors this large must still compose into a usable decomposition.
    MatrixXf matrix(6, 4);
    for (Index i = 0; i < matrix.rows(); ++i)
      for (Index j = 0; j < matrix.cols(); ++j) matrix(i, j) = 1e19f * float(internal::random<double>(-1.0, 1.0));
    const HouseholderQR<MatrixXf> qr(matrix);
    const MatrixXf q = qr.householderQ() * MatrixXf::Identity(6, 4);
    const MatrixXf r = qr.matrixQR().topRows(4).template triangularView<Upper>();
    const MatrixXd scaled = (matrix / 1e19f).cast<double>();
    const double relative_residual = ((q * r).cast<double>() / 1e19 - scaled).norm() / scaled.norm();
    VERIFY(relative_residual <= 64 * double(NumTraits<float>::epsilon()));
  }
}

template <typename Scalar, int Size, int StorageOrder>
void householder_short_strided() {
  using Mat = Matrix<Scalar, Dynamic, Dynamic, StorageOrder>;
  using RealScalar = typename NumTraits<Scalar>::Real;
  using Vector = Matrix<Scalar, Size, 1>;
  using Essential = Matrix<Scalar, Size - 1, 1>;
  const Vector vector = Vector::Random();
  Essential essential;
  Scalar tau;
  RealScalar beta;
  vector.makeHouseholder(essential, tau, beta);
  Vector v;
  v << Scalar(1), essential;
  const Matrix<Scalar, Size, Size> h = Matrix<Scalar, Size, Size>::Identity() - tau * v * v.adjoint();

  for (Index cols : {0, 1, 2, 17, 32, 33, 500, 512}) {
    for (Index innerStride : {1, 2}) {
      for (Index outerStride : {2048, 2055}) {
        Matrix<Scalar, Dynamic, 1> storage =
            Matrix<Scalar, Dynamic, 1>::Random(outerStride * (StorageOrder == ColMajor ? cols + 2 : Size + 2));
        Map<Mat, 0, Stride<Dynamic, Dynamic>> mapped(storage.data(), Size + 2, cols + 2,
                                                     Stride<Dynamic, Dynamic>(outerStride, innerStride));
        const auto originalStorage = storage.eval();
        const Mat original = mapped;
        auto block = mapped.block(1, 1, Size, cols);
        const Mat expected = h * original.block(1, 1, Size, cols);
        Matrix<Scalar, Dynamic, 1> workspace(cols + 2);
        workspace.setConstant(Scalar(7));
        // Fixed essential length selects the fused column-major path even with dynamic block dimensions.
        block.applyHouseholderOnTheLeft(essential, tau, workspace.data() + 1);
        const RealScalar bound = RealScalar(16 * Size) * NumTraits<RealScalar>::epsilon() * original.norm();
        VERIFY((block - expected).norm() <= bound);
        VERIFY_IS_EQUAL(workspace[0], Scalar(7));
        VERIFY_IS_EQUAL(workspace[cols + 1], Scalar(7));
        Mat expectedFull = original;
        expectedFull.block(1, 1, Size, cols) = block;
        const auto resultStorage = storage.eval();
        storage = originalStorage;
        mapped = expectedFull;
        VERIFY_IS_EQUAL(storage, resultStorage);

        block.applyHouseholderOnTheLeft(essential, Scalar(0), workspace.data() + 1);
        VERIFY_IS_EQUAL(storage, resultStorage);

        storage = originalStorage;
        const Matrix<Scalar, Dynamic, 1> dynamicEssential = essential;
        block.applyHouseholderOnTheLeft(dynamicEssential, tau, workspace.data() + 1);
        VERIFY((block - expected).norm() <= bound);

        if (cols > 0) {
          storage = originalStorage;
          const Scalar aliasedTau = block.coeff(0, 0);
          const Matrix<Scalar, Size, Size> aliasedH =
              Matrix<Scalar, Size, Size>::Identity() - aliasedTau * v * v.adjoint();
          const Mat aliasedExpected = aliasedH * original.block(1, 1, Size, cols);
          block.applyHouseholderOnTheLeft(essential, block.coeffRef(0, 0), workspace.data() + 1);
          VERIFY((block - aliasedExpected).norm() <= bound);
        }
      }
    }
  }
}

// Reference for H * a with v = [1; essential] and the operation order of applyHouseholderOnTheLeft, which a
// noncommutative scalar distinguishes: w_j = tau * (a_0j + sum_i conj(e_i) a_(i+1)j), a_0j -= w_j, a_(i+1)j -= e_i w_j.
template <typename MatrixType, typename EssentialType>
MatrixType householder_left_reference(const MatrixType& a, const EssentialType& essential,
                                      const typename MatrixType::Scalar& tau) {
  using Scalar = typename MatrixType::Scalar;
  MatrixType result = a;
  for (Index j = 0; j < a.cols(); ++j) {
    Scalar dot = a(0, j);
    for (Index i = 0; i < essential.size(); ++i) dot += numext::conj(essential(i)) * a(i + 1, j);
    const Scalar w = tau * dot;
    result(0, j) -= w;
    for (Index i = 0; i < essential.size(); ++i) result(i + 1, j) -= essential(i) * w;
  }
  return result;
}

// Whether applyHouseholderOnTheLeft on Derived with this essential type selects the fused column loop.
template <typename Derived, typename EssentialPart>
constexpr bool householder_fused() {
  return std::is_same<internal::householder_apply_left_impl<Derived, EssentialPart>,
                      internal::householder_apply_left_impl<Derived, EssentialPart, true>>::value;
}

template <bool ExpectFused, typename MatrixType, typename EssentialType>
void verify_householder_left(MatrixType a, const EssentialType& essential, const typename MatrixType::Scalar& tau) {
  using RealScalar = typename NumTraits<typename MatrixType::Scalar>::Real;
  STATIC_CHECK((householder_fused<MatrixType, EssentialType>() == ExpectFused));
  const MatrixType expected = householder_left_reference(a, essential.eval(), tau);
  // Applying a reflector is backward stable; both evaluations carry O(rows * eps * |a|) rounding.
  const RealScalar bound = RealScalar(16 * a.rows()) * NumTraits<RealScalar>::epsilon() * a.norm();
  Matrix<typename MatrixType::Scalar, Dynamic, 1> workspace(a.cols());
  a.applyHouseholderOnTheLeft(essential, tau, workspace.data());
  VERIFY((a - expected).norm() <= bound);
}

// The essential part may be any column-vector expression, not only a plain vector: a runtime one-column block
// (no compile-time vector shape), the adjoint of a row tail as CompleteOrthogonalDecomposition passes, or a
// fixed-size segment of either. Each must compile in C++14, where no branch of the dispatch is discarded, and
// only the fixed-size ones on column-major storage select the fused loop.
template <typename Scalar, int StorageOrder>
void householder_essential_expressions() {
  using Mat = Matrix<Scalar, Dynamic, Dynamic, StorageOrder>;
  using RowMajorMat = Matrix<Scalar, Dynamic, Dynamic, RowMajor>;
  using VectorType = Matrix<Scalar, Dynamic, 1>;
  constexpr bool kFusedIfFixed = StorageOrder == ColMajor;
  const Index cols = internal::random<Index>(1, 20);
  for (Index rows : {Index(2), Index(3), internal::random<Index>(4, 20)}) {
    const Mat a = Mat::Random(rows, cols);
    Scalar tau;
    typename NumTraits<Scalar>::Real beta;
    VectorType essential(rows - 1);
    VectorType::Random(rows).makeHouseholder(essential, tau, beta);

    Mat column(rows + 1, 3);
    column.col(1).tail(rows - 1) = essential;
    verify_householder_left<false>(a, column.block(2, 1, rows - 1, 1), tau);

    Mat row(3, rows + 1);
    row.row(1).tail(rows - 1) = essential.adjoint();
    verify_householder_left<false>(a, row.row(1).tail(rows - 1).adjoint(), tau);

    RowMajorMat rowMajorColumn(rows + 1, 3);
    rowMajorColumn.col(1).tail(rows - 1) = essential;
    verify_householder_left<false>(a, rowMajorColumn.col(1).tail(rows - 1), tau);

    if (rows == 2) {
      verify_householder_left<kFusedIfFixed>(a, row.row(1).template segment<1>(2).adjoint(), tau);
      verify_householder_left<kFusedIfFixed>(a, rowMajorColumn.col(1).template tail<1>(), tau);
    }
    if (rows == 3) {
      verify_householder_left<kFusedIfFixed>(a, row.row(1).template segment<2>(2).adjoint(), tau);
      verify_householder_left<kFusedIfFixed>(a, rowMajorColumn.col(1).template tail<2>(), tau);
    }
  }
}

namespace noncommutative_scalar {

// Quaternions: multiplication is associative and conjugation reverses products, but ab != ba in general.
struct Quaternion {
  double r, i, j, k;
  Quaternion(double real = 0, double x = 0, double y = 0, double z = 0) : r(real), i(x), j(y), k(z) {}
  Quaternion operator+(const Quaternion& b) const { return Quaternion(r + b.r, i + b.i, j + b.j, k + b.k); }
  Quaternion operator-(const Quaternion& b) const { return Quaternion(r - b.r, i - b.i, j - b.j, k - b.k); }
  Quaternion operator-() const { return Quaternion(-r, -i, -j, -k); }
  Quaternion operator*(const Quaternion& b) const {
    return Quaternion(r * b.r - i * b.i - j * b.j - k * b.k, r * b.i + i * b.r + j * b.k - k * b.j,
                      r * b.j - i * b.k + j * b.r + k * b.i, r * b.k + i * b.j - j * b.i + k * b.r);
  }
  Quaternion& operator+=(const Quaternion& b) { return *this = *this + b; }
  Quaternion& operator-=(const Quaternion& b) { return *this = *this - b; }
  Quaternion& operator*=(const Quaternion& b) { return *this = *this * b; }
  bool operator==(const Quaternion& b) const { return r == b.r && i == b.i && j == b.j && k == b.k; }
  bool operator!=(const Quaternion& b) const { return !(*this == b); }
};

inline Quaternion conj(const Quaternion& a) { return Quaternion(a.r, -a.i, -a.j, -a.k); }
inline double real(const Quaternion& a) { return a.r; }
inline double imag(const Quaternion& a) { return a.i; }

}  // namespace noncommutative_scalar

namespace Eigen {
template <>
struct NumTraits<noncommutative_scalar::Quaternion> : GenericNumTraits<noncommutative_scalar::Quaternion> {
  using Real = double;
  using Literal = double;
  static constexpr bool IsComplex = true;
  static constexpr bool RequireInitialization = true;
};
}  // namespace Eigen

// The fused loop must multiply in the order the general update's expressions spell out, essential * tmp with
// tmp = tau * (essential.adjoint() * column + top), which only a noncommutative scalar observes. Small integer
// components keep every operation exact. The general path is not a reference here: its product kernels reorder
// operands (the gemv selector transposes essential.adjoint() * bottom, the column-major outer-product selector
// forms tmp_j * essential).
template <int Size>
void householder_noncommutative_scalar() {
  using Scalar = noncommutative_scalar::Quaternion;
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using EssentialType = Matrix<Scalar, Size - 1, 1>;
  STATIC_CHECK((householder_fused<Mat, EssentialType>()));
  const Scalar i(0, 1), j(0, 0, 1), k(0, 0, 0, 1);
  {
    // With essential = [i, 0...]^T, tau = 1 and input [j, 0...]^T, the second entry becomes -i * j = -k.
    Mat a = Mat::Constant(Size, 1, Scalar(0));
    a(0, 0) = j;
    EssentialType essential = EssentialType::Constant(Scalar(0));
    essential(0) = i;
    Scalar workspace[1];
    a.applyHouseholderOnTheLeft(essential, Scalar(1), workspace);
    VERIFY(a(0, 0) == Scalar(0));
    VERIFY(a(1, 0) == -k);
  }
  const Index cols = internal::random<Index>(1, 7);
  Mat a(Size, cols);
  for (Index c = 0; c < cols; ++c)
    for (Index r = 0; r < Size; ++r)
      a(r, c) = Scalar(internal::random<int>(-2, 2), internal::random<int>(-2, 2), internal::random<int>(-2, 2),
                       internal::random<int>(-2, 2));
  EssentialType essential;
  for (Index r = 0; r + 1 < Size; ++r)
    essential(r) = Scalar(internal::random<int>(-2, 2), internal::random<int>(-2, 2), internal::random<int>(-2, 2),
                          internal::random<int>(-2, 2));
  const Scalar tau(internal::random<int>(-2, 2), internal::random<int>(-2, 2), internal::random<int>(-2, 2), 1);
  const Mat expected = householder_left_reference(a, essential, tau);
  Matrix<Scalar, Dynamic, 1> workspace(cols);
  a.applyHouseholderOnTheLeft(essential, tau, workspace.data());
  VERIFY(a == expected);
}

EIGEN_DECLARE_TEST(householder) {
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(householder(Matrix<double, 2, 2>()));
    CALL_SUBTEST_2(householder(Matrix<float, 2, 3>()));
    CALL_SUBTEST_3(householder(Matrix<double, 3, 5>()));
    CALL_SUBTEST_4(householder(Matrix<float, 4, 4>()));
    CALL_SUBTEST_5(householder(
        MatrixXd(internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_6(householder(
        MatrixXcf(internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_7(householder(
        MatrixXf(internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_8(householder(Matrix<double, 1, 1>()));

    CALL_SUBTEST_9(householder_update(Matrix<double, 3, 5>()));
    CALL_SUBTEST_9(householder_update(Matrix<float, 4, 2>()));
    CALL_SUBTEST_9(householder_update(
        MatrixXcf(internal::random<Index>(1, EIGEN_TEST_MAX_SIZE), internal::random<Index>(1, EIGEN_TEST_MAX_SIZE))));
  }

  CALL_SUBTEST_10(householder_blocked_right_regression<double>());
  CALL_SUBTEST_11(householder_blocked_right_regression<std::complex<double>>());
  CALL_SUBTEST_12(householder_small_tail());
  CALL_SUBTEST_13(householder_large_components());
  CALL_SUBTEST_14((householder_short_strided<float, 2, ColMajor>()));
  CALL_SUBTEST_14((householder_short_strided<float, 3, ColMajor>()));
  CALL_SUBTEST_15((householder_short_strided<double, 2, ColMajor>()));
  CALL_SUBTEST_15((householder_short_strided<double, 3, ColMajor>()));
  CALL_SUBTEST_16((householder_short_strided<std::complex<float>, 2, ColMajor>()));
  CALL_SUBTEST_16((householder_short_strided<std::complex<float>, 3, ColMajor>()));
  CALL_SUBTEST_17((householder_short_strided<std::complex<double>, 2, ColMajor>()));
  CALL_SUBTEST_17((householder_short_strided<std::complex<double>, 3, ColMajor>()));
  CALL_SUBTEST_18((householder_short_strided<double, 2, RowMajor>()));
  CALL_SUBTEST_18((householder_short_strided<double, 3, RowMajor>()));
  CALL_SUBTEST_19((householder_short_strided<std::complex<double>, 3, RowMajor>()));
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_20((householder_essential_expressions<double, ColMajor>()));
    CALL_SUBTEST_20((householder_essential_expressions<double, RowMajor>()));
    CALL_SUBTEST_21((householder_essential_expressions<std::complex<double>, ColMajor>()));
    CALL_SUBTEST_21((householder_essential_expressions<std::complex<double>, RowMajor>()));
    CALL_SUBTEST_22(householder_noncommutative_scalar<2>());
    CALL_SUBTEST_22(householder_noncommutative_scalar<3>());
  }
}
