// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2015 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "main.h"
#include <Eigen/QR>

void check(bool b, bool ref) {
  std::cout << b;
  if (b == ref)
    std::cout << " OK  ";
  else
    std::cout << " BAD ";
}

template <typename T>
void check_inf_nan(bool dryrun) {
  Matrix<T, Dynamic, 1> m(10);
  m.setRandom();
  m(3) = std::numeric_limits<T>::quiet_NaN();

  if (dryrun) {
    std::cout << "std::isfinite(" << m(3) << ") = ";
    check((std::isfinite)(m(3)), false);
    std::cout << "  ; numext::isfinite = ";
    check((numext::isfinite)(m(3)), false);
    std::cout << "\n";
    std::cout << "std::isinf(" << m(3) << ")    = ";
    check((std::isinf)(m(3)), false);
    std::cout << "  ; numext::isinf    = ";
    check((numext::isinf)(m(3)), false);
    std::cout << "\n";
    std::cout << "std::isnan(" << m(3) << ")    = ";
    check((std::isnan)(m(3)), true);
    std::cout << "  ; numext::isnan    = ";
    check((numext::isnan)(m(3)), true);
    std::cout << "\n";
    std::cout << "allFinite: ";
    check(m.allFinite(), 0);
    std::cout << "\n";
    std::cout << "hasNaN:    ";
    check(m.hasNaN(), 1);
    std::cout << "\n";
    std::cout << "\n";
  } else {
    if ((std::isfinite)(m(3))) {
      g_test_level = 1;
      VERIFY(!(numext::isfinite)(m(3)));
      g_test_level = 0;
    }
    if ((std::isinf)(m(3))) {
      g_test_level = 1;
      VERIFY(!(numext::isinf)(m(3)));
      g_test_level = 0;
    }
    if (!(std::isnan)(m(3))) {
      g_test_level = 1;
      VERIFY((numext::isnan)(m(3)));
      g_test_level = 0;
    }
    if ((std::isfinite)(m(3))) {
      g_test_level = 1;
      VERIFY(!m.allFinite());
      g_test_level = 0;
    }
    if (!(std::isnan)(m(3))) {
      g_test_level = 1;
      VERIFY(m.hasNaN());
      g_test_level = 0;
    }
  }
  T hidden_zero = (std::numeric_limits<T>::min)() * (std::numeric_limits<T>::min)();
  m(4) /= hidden_zero;
  if (dryrun) {
    std::cout << "std::isfinite(" << m(4) << ") = ";
    check((std::isfinite)(m(4)), false);
    std::cout << "  ; numext::isfinite = ";
    check((numext::isfinite)(m(4)), false);
    std::cout << "\n";
    std::cout << "std::isinf(" << m(4) << ")    = ";
    check((std::isinf)(m(4)), true);
    std::cout << "  ; numext::isinf    = ";
    check((numext::isinf)(m(4)), true);
    std::cout << "\n";
    std::cout << "std::isnan(" << m(4) << ")    = ";
    check((std::isnan)(m(4)), false);
    std::cout << "  ; numext::isnan    = ";
    check((numext::isnan)(m(4)), false);
    std::cout << "\n";
    std::cout << "allFinite: ";
    check(m.allFinite(), 0);
    std::cout << "\n";
    std::cout << "hasNaN:    ";
    check(m.hasNaN(), 1);
    std::cout << "\n";
    std::cout << "\n";
  } else {
    if ((std::isfinite)(m(3))) {
      g_test_level = 1;
      VERIFY(!(numext::isfinite)(m(4)));
      g_test_level = 0;
    }
    if (!(std::isinf)(m(3))) {
      g_test_level = 1;
      VERIFY((numext::isinf)(m(4)));
      g_test_level = 0;
    }
    if ((std::isnan)(m(3))) {
      g_test_level = 1;
      VERIFY(!(numext::isnan)(m(4)));
      g_test_level = 0;
    }
    if ((std::isfinite)(m(3))) {
      g_test_level = 1;
      VERIFY(!m.allFinite());
      g_test_level = 0;
    }
    if (!(std::isnan)(m(3))) {
      g_test_level = 1;
      VERIFY(m.hasNaN());
      g_test_level = 0;
    }
  }
  m(3) = 0;
  if (dryrun) {
    std::cout << "std::isfinite(" << m(3) << ") = ";
    check((std::isfinite)(m(3)), true);
    std::cout << "  ; numext::isfinite = ";
    check((numext::isfinite)(m(3)), true);
    std::cout << "\n";
    std::cout << "std::isinf(" << m(3) << ")    = ";
    check((std::isinf)(m(3)), false);
    std::cout << "  ; numext::isinf    = ";
    check((numext::isinf)(m(3)), false);
    std::cout << "\n";
    std::cout << "std::isnan(" << m(3) << ")    = ";
    check((std::isnan)(m(3)), false);
    std::cout << "  ; numext::isnan    = ";
    check((numext::isnan)(m(3)), false);
    std::cout << "\n";
    std::cout << "allFinite: ";
    check(m.allFinite(), 0);
    std::cout << "\n";
    std::cout << "hasNaN:    ";
    check(m.hasNaN(), 0);
    std::cout << "\n";
    std::cout << "\n\n";
  } else {
    if (!(std::isfinite)(m(3))) {
      g_test_level = 1;
      VERIFY((numext::isfinite)(m(3)));
      g_test_level = 0;
    }
    if ((std::isinf)(m(3))) {
      g_test_level = 1;
      VERIFY(!(numext::isinf)(m(3)));
      g_test_level = 0;
    }
    if ((std::isnan)(m(3))) {
      g_test_level = 1;
      VERIFY(!(numext::isnan)(m(3)));
      g_test_level = 0;
    }
    if ((std::isfinite)(m(3))) {
      g_test_level = 1;
      VERIFY(!m.allFinite());
      g_test_level = 0;
    }
    if ((std::isnan)(m(3))) {
      g_test_level = 1;
      VERIFY(!m.hasNaN());
      g_test_level = 0;
    }
  }
}

template <typename RealScalar>
void check_complex_rowmajor_adjoint_product() {
  typedef std::complex<RealScalar> Scalar;
  typedef Matrix<Scalar, Dynamic, Dynamic, RowMajor> RowMatrix;
  typedef Matrix<Scalar, Dynamic, Dynamic, ColMajor> ColMatrix;

  RowMatrix mat(2, 2);
  mat << Scalar(1, 2), Scalar(3, -4), Scalar(-5, 6), Scalar(7, 8);

  RowMatrix expected(2, 2);
  expected << Scalar(66, 0), Scalar(8, -92), Scalar(8, 92), Scalar(138, 0);

  const RowMatrix row_major_result = mat.adjoint() * mat;
  const ColMatrix col_major_result = mat.adjoint() * mat;

  VERIFY_IS_APPROX(mat.adjoint() * mat, expected);
  VERIFY_IS_APPROX(row_major_result, expected);
  VERIFY_IS_APPROX(col_major_result, expected);
}

template <typename RealScalar>
void check_complex_packet_arithmetic() {
  typedef std::complex<RealScalar> Scalar;
  typedef Matrix<Scalar, 2, 1> Vector2;

  Vector2 values;
  values << Scalar(RealScalar(0.53645928880954319), RealScalar(-0.60489662966980218)),
      Scalar(RealScalar(0.25774142970757641), RealScalar(0.10793998506041591));
  Scalar divisor(RealScalar(1.6611441458336193), RealScalar(-0.21123424512127231));
  Scalar factor(RealScalar(1.2499121678643004), RealScalar(0.36146968008699221));

  Vector2 quotient = values / divisor;
  Vector2 expected_quotient;
  expected_quotient << values.coeff(0) / divisor, values.coeff(1) / divisor;
  VERIFY_IS_APPROX(quotient, expected_quotient);

  Vector2 inverse = values.array().inverse();
  Vector2 expected_inverse;
  expected_inverse << Scalar(RealScalar(1)) / values.coeff(0), Scalar(RealScalar(1)) / values.coeff(1);
  VERIFY_IS_APPROX(inverse, expected_inverse);

  Vector2 product = values * factor;
  Vector2 expected_product;
  expected_product << values.coeff(0) * factor, values.coeff(1) * factor;
  VERIFY_IS_APPROX(product, expected_product);

  Vector2 conjugate_product = values.conjugate().cwiseProduct(Vector2::Constant(factor));
  Vector2 expected_conjugate_product;
  expected_conjugate_product << numext::conj(values.coeff(0)) * factor, numext::conj(values.coeff(1)) * factor;
  VERIFY_IS_APPROX(conjugate_product, expected_conjugate_product);

  // numext::divide guards against intermediate overflow (c^2 + d^2) near max representable bounds.
  const RealScalar max_val = (std::numeric_limits<RealScalar>::max)();
  const Scalar big_num(RealScalar(0.9) * max_val, RealScalar(0));
  const Scalar big_denom(max_val, RealScalar(0));
  const Scalar safe_div_result = numext::divide(big_num, big_denom);
  VERIFY((numext::isfinite)(safe_div_result.real()));
  VERIFY_IS_APPROX(safe_div_result, Scalar(RealScalar(0.9), RealScalar(0)));
  VERIFY_IS_APPROX(numext::divide(RealScalar(0.9) * max_val, max_val), RealScalar(0.9));

  // Verify vectorized expression division (vec / denom) on numbers that would cause
  // intermediate overflow (c^2 + d^2 > max) in naive complex division.
  const RealScalar intermediate_scale = numext::sqrt(max_val) * RealScalar(2);
  const Scalar intermediate_num(RealScalar(0.9) * intermediate_scale, RealScalar(0));
  const Scalar intermediate_denom(intermediate_scale, RealScalar(0));
  Matrix<Scalar, Dynamic, 1> vec(8);
  vec.fill(intermediate_num);
  Matrix<Scalar, Dynamic, 1> vec_div = vec / intermediate_denom;
  for (Index i = 0; i < vec.size(); ++i) {
    VERIFY((numext::isfinite)(vec_div(i).real()));
    VERIFY_IS_APPROX(vec_div(i), Scalar(RealScalar(0.9), RealScalar(0)));
  }
}

template <typename RealScalar>
void check_complex_packet_math_functions() {
  typedef std::complex<RealScalar> Scalar;
  typedef Matrix<Scalar, 4, 1> Vector4;

  Vector4 values;
  values << Scalar(RealScalar(0.53645928880954319), RealScalar(-0.60489662966980218)),
      Scalar(RealScalar(0.25774142970757641), RealScalar(0.10793998506041591)),
      Scalar(RealScalar(-0.83239073966000054), RealScalar(0.026801457199547407)),
      Scalar(RealScalar(1.6611441458336193), RealScalar(-0.21123424512127231));

  Vector4 sqrt_result = values.array().sqrt();
  Vector4 log_result = values.array().log();
  Vector4 exp_result = values.array().exp();

  Vector4 expected_sqrt, expected_log, expected_exp;
  for (Index i = 0; i < values.size(); ++i) {
    expected_sqrt[i] = std::sqrt(values[i]);
    expected_log[i] = std::log(values[i]);
    expected_exp[i] = std::exp(values[i]);
  }

  VERIFY_IS_APPROX(sqrt_result, expected_sqrt);
  VERIFY_IS_APPROX(log_result, expected_log);
  VERIFY_IS_APPROX(exp_result, expected_exp);
}

template <typename RealScalar>
void check_complex_householder_qr() {
  typedef std::complex<RealScalar> Scalar;
  typedef Matrix<Scalar, 3, 2> Matrix32;
  typedef Matrix<Scalar, 2, 2> Matrix22;

  Matrix32 mat;
  mat << Scalar(RealScalar(0.59688070592806186), RealScalar(-0.21123424512127231)),
      Scalar(RealScalar(0.83239073966000054), RealScalar(0.026801457199547407)),
      Scalar(RealScalar(0.53645928880954319), RealScalar(-0.60489662966980218)),
      Scalar(RealScalar(0.21393694076781777), RealScalar(0.43459380694554106)),
      Scalar(RealScalar(0.25774142970757641), RealScalar(0.10793998506041591)),
      Scalar(RealScalar(0.60835279200704262), RealScalar(-0.51422689790671194));

  HouseholderQR<Matrix32> qr(mat);
  Matrix32 q = qr.householderQ() * Matrix32::Identity();
  Matrix22 r = qr.matrixQR().template topRows<2>().template triangularView<Upper>();
  VERIFY_IS_APPROX(mat, q * r);
}

// The complex self-adjoint 1-norm takes sqrt(re^2 + im^2) in packets and falls back to the scalar
// abs when a component is too large to square. The decision must not depend on an infinity or
// NaN surviving fast-math: an approximate packet sqrt can turn the overflow into a NaN that a
// maximum then discards. Large entries both on the diagonal and inside a packet of a column.
template <typename RealScalar>
void check_complex_selfadjoint_l1norm() {
  typedef std::complex<RealScalar> Scalar;
  RealScalar big = numext::sqrt(NumTraits<RealScalar>::highest()) * RealScalar(1e3);
  Matrix<Scalar, 8, 8> m = Matrix<Scalar, 8, 8>::Identity() * big;
  VERIFY_IS_APPROX(m.template selfadjointView<Lower>().l1Norm(), big);
  VERIFY_IS_APPROX(m.template selfadjointView<Upper>().l1Norm(), big);
  m.setIdentity();
  m(2, 0) = m(0, 2) = Scalar(big, big);
  RealScalar expected = numext::abs(Scalar(big, big)) + RealScalar(1);
  VERIFY_IS_APPROX(m.template selfadjointView<Lower>().l1Norm(), expected);
  VERIFY_IS_APPROX(m.template selfadjointView<Upper>().l1Norm(), expected);
}

template <typename RealScalar>
void check_complex_fastmath() {
  check_complex_rowmajor_adjoint_product<RealScalar>();
  check_complex_packet_arithmetic<RealScalar>();
  check_complex_packet_math_functions<RealScalar>();
  check_complex_householder_qr<RealScalar>();
  check_complex_selfadjoint_l1norm<RealScalar>();
}

// The packet implementations of these functions manipulate signs of non-zero values through a
// -0.0 bitmask. Under fast-math flags compilers consider -0.0 and +0.0 interchangeable and may
// substitute one such constant for the other (GCC's value numbering does this on RISC-V), which
// silently zeroes the mask unless it is constructed from integer bits (see psignmask and
// https://gitlab.com/libeigen/eigen/-/merge_requests/2698). Sign handling of *non-zero* inputs
// and outputs is not relaxed by fast-math, so these checks must hold.
template <typename Scalar>
void check_sign_dependent_functions() {
  typedef Array<Scalar, Dynamic, 1> ArrayType;
  const Index n = 64;

  ArrayType ya(n), xa(n);
  for (Index i = 0; i < n; ++i) {
    Scalar s = Scalar(1) + Scalar(i) / Scalar(n);
    // All four quadrants.
    ya[i] = (i & 1) ? s : -s;
    xa[i] = (i & 2) ? Scalar(2) * s : Scalar(-2) * s;
  }
  // Cover the |x| == |y| special path of patan2 in all four quadrants.
  ya[0] = Scalar(1), xa[0] = Scalar(1);
  ya[1] = Scalar(1), xa[1] = Scalar(-1);
  ya[2] = Scalar(-1), xa[2] = Scalar(1);
  ya[3] = Scalar(-1), xa[3] = Scalar(-1);

  const ArrayType atan2_result = ya.atan2(xa);
  for (Index i = 0; i < n; ++i) {
    VERIFY_IS_APPROX(atan2_result[i], std::atan2(ya[i], xa[i]));
  }

  // Mixed-sign inputs, no exact zeros, |w| up to ~3 to hit the small- and large-|x| branches.
  const ArrayType w = ArrayType::LinSpaced(n, Scalar(-3), Scalar(3)) + Scalar(0.017);
  const ArrayType w_unit = w / Scalar(3.2);  // in (-1, 1) for atanh
  const ArrayType atan_result = w.atan();
  const ArrayType sinh_result = w.sinh();
  const ArrayType tanh_result = w.tanh();
  const ArrayType asinh_result = w.asinh();
  const ArrayType atanh_result = w_unit.atanh();
  const ArrayType cbrt_result = w.cbrt();
  for (Index i = 0; i < n; ++i) {
    VERIFY_IS_APPROX(atan_result[i], std::atan(w[i]));
    VERIFY_IS_APPROX(sinh_result[i], std::sinh(w[i]));
    VERIFY_IS_APPROX(tanh_result[i], std::tanh(w[i]));
    VERIFY_IS_APPROX(asinh_result[i], std::asinh(w[i]));
    VERIFY_IS_APPROX(atanh_result[i], std::atanh(w_unit[i]));
    VERIFY_IS_APPROX(cbrt_result[i], std::cbrt(w[i]));
  }
}

EIGEN_DECLARE_TEST(fastmath) {
  std::cout << "*** float *** \n\n";
  check_inf_nan<float>(true);
  std::cout << "*** double ***\n\n";
  check_inf_nan<double>(true);
  std::cout << "*** long double *** \n\n";
  check_inf_nan<long double>(true);

  check_inf_nan<float>(false);
  check_inf_nan<double>(false);
  check_inf_nan<long double>(false);

  CALL_SUBTEST_1(check_complex_fastmath<float>());
  CALL_SUBTEST_2(check_complex_fastmath<double>());
  CALL_SUBTEST_3(check_sign_dependent_functions<float>());
  CALL_SUBTEST_4(check_sign_dependent_functions<double>());
}
