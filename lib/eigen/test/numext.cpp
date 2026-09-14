// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2017 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "main.h"
#include "twoprod_helpers.h"

#if EIGEN_COMP_MSVC
#include <cfenv>
#endif

template <typename T, typename U>
bool check_if_equal_or_nans(const T& actual, const U& expected) {
  return (numext::equal_strict(actual, expected) || ((numext::isnan)(actual) && (numext::isnan)(expected)));
}

template <typename T, typename U>
bool check_if_equal_or_nans(const std::complex<T>& actual, const std::complex<U>& expected) {
  return check_if_equal_or_nans(numext::real(actual), numext::real(expected)) &&
         check_if_equal_or_nans(numext::imag(actual), numext::imag(expected));
}

template <typename T, typename U>
bool test_is_equal_or_nans(const T& actual, const U& expected) {
  if (check_if_equal_or_nans(actual, expected)) {
    return true;
  }

  // false:
  std::cerr << "\n    actual   = " << actual << "\n    expected = " << expected << "\n\n";
  return false;
}

#define VERIFY_IS_EQUAL_OR_NANS(a, b) VERIFY(test_is_equal_or_nans(a, b))

template <typename T>
struct check_copysign_impl {
  static void run() {
    const T pos_zero = T(0);
    const T pos_one = T(1);

    // Tests valid for all types.
    VERIFY_IS_EQUAL(numext::copysign(pos_one, pos_one), pos_one);
    VERIFY_IS_EQUAL(numext::copysign(pos_zero, pos_one), pos_zero);

    // Tests valid for all signed types (integer and floating-point).
    if (NumTraits<T>::IsSigned) {
      const T neg_one = numext::negate(pos_one);
      VERIFY_IS_EQUAL(numext::copysign(pos_one, neg_one), neg_one);
      VERIFY_IS_EQUAL(numext::copysign(neg_one, pos_one), pos_one);
      VERIFY_IS_EQUAL(numext::copysign(neg_one, neg_one), neg_one);
    }

    // Tests specific to floating-point types (negative zero, infinity, NaN).
    if (!NumTraits<T>::IsInteger) {
      const T neg_zero = numext::negate(pos_zero);
      const T neg_one = numext::negate(pos_one);
      const T pos_inf = std::numeric_limits<T>::infinity();
      const T neg_inf = numext::negate(pos_inf);
      const T pos_nan = std::numeric_limits<T>::quiet_NaN();
      const T neg_nan = numext::negate(pos_nan);
      // Sign transferred from zero.
      VERIFY_IS_EQUAL(numext::copysign(pos_one, pos_zero), pos_one);
      VERIFY_IS_EQUAL(numext::copysign(pos_one, neg_zero), neg_one);
      // Sign transferred from infinity.
      VERIFY_IS_EQUAL(numext::copysign(pos_one, pos_inf), pos_one);
      VERIFY_IS_EQUAL(numext::copysign(pos_one, neg_inf), neg_one);
      // Sign transferred from NaN.
      VERIFY_IS_EQUAL(numext::copysign(pos_one, pos_nan), pos_one);
      VERIFY_IS_EQUAL(numext::copysign(pos_one, neg_nan), neg_one);
    }

    for (int k = 0; k < 100; ++k) {
      // For signed integers avoid lowest() so that abs(a) does not overflow.
      const T a = (NumTraits<T>::IsSigned && NumTraits<T>::IsInteger)
                      ? internal::random<T>(numext::negate(NumTraits<T>::highest()), NumTraits<T>::highest())
                      : internal::random<T>();
      const T b = internal::random<T>();
      const T result = numext::copysign(a, b);
      // Magnitude is preserved.
      VERIFY_IS_EQUAL(numext::abs(result), numext::abs(a));
      // Sign matches sign source. Integers have no negative zero, so the sign
      // of the result is only meaningful when a != 0.
      if (!NumTraits<T>::IsInteger || a != T(0)) {
        VERIFY_IS_EQUAL(numext::copysign(pos_one, result), numext::copysign(pos_one, b));
      }
    }
  }
};

template <typename T>
struct check_copysign_impl<std::complex<T>> {
  static void run() {
    typedef std::complex<T> ComplexT;
    const T pos_one = T(1);
    const T neg_one = numext::negate(pos_one);

    // Complex copysign is applied component-wise.
    VERIFY_IS_EQUAL(numext::copysign(ComplexT(pos_one, pos_one), ComplexT(pos_one, neg_one)),
                    ComplexT(pos_one, neg_one));
    VERIFY_IS_EQUAL(numext::copysign(ComplexT(neg_one, pos_one), ComplexT(pos_one, neg_one)),
                    ComplexT(pos_one, neg_one));
    VERIFY_IS_EQUAL(numext::copysign(ComplexT(pos_one, neg_one), ComplexT(neg_one, pos_one)),
                    ComplexT(neg_one, pos_one));

    for (int k = 0; k < 100; ++k) {
      const ComplexT a = internal::random<ComplexT>();
      const ComplexT b = internal::random<ComplexT>();
      const ComplexT result = numext::copysign(a, b);
      // Each component is independently copysigned.
      VERIFY_IS_EQUAL(numext::real(result), numext::copysign(numext::real(a), numext::real(b)));
      VERIFY_IS_EQUAL(numext::imag(result), numext::copysign(numext::imag(a), numext::imag(b)));
    }
  }
};

template <typename T>
void check_copysign() {
  check_copysign_impl<T>::run();
}

template <>
void check_copysign<bool>() {
  for (bool a : {false, true}) {
    for (bool b : {false, true}) {
      VERIFY_IS_EQUAL(numext::copysign(a, b), a);
    }
  }
}

template <typename T>
void check_negate() {
  Index size = 1000;
  for (Index i = 0; i < size; i++) {
    T val = i == 0 ? T(0) : internal::random<T>(T(0), NumTraits<T>::highest());
    T neg_val = numext::negate(val);
    VERIFY_IS_EQUAL(T(val + neg_val), T(0));
    VERIFY_IS_EQUAL(numext::negate(neg_val), val);
  }
}

#if EIGEN_COMP_MSVC
#pragma float_control(precise, on, push)
#pragma fenv_access(on)
#endif

template <typename T>
void check_complex_exp() {
  using Complex = std::complex<T>;
  const T highest = (std::numeric_limits<T>::max)();
  const T inf = std::numeric_limits<T>::infinity();
  const T nan = std::numeric_limits<T>::quiet_NaN();

  const Complex finite_inf = numext::exp(Complex(T(1), inf));
  VERIFY((numext::isnan)(finite_inf.real()));
  VERIFY((numext::isnan)(finite_inf.imag()));

  const Complex finite_nan = numext::exp(Complex(T(1), nan));
  VERIFY((numext::isnan)(finite_nan.real()));
  VERIFY((numext::isnan)(finite_nan.imag()));

#if EIGEN_COMP_MSVC
  std::feclearexcept(FE_ALL_EXCEPT);
#endif
  const Complex highest_inf = numext::exp(Complex(highest, inf));
  VERIFY((numext::isnan)(highest_inf.real()));
  VERIFY((numext::isnan)(highest_inf.imag()));
#if EIGEN_COMP_MSVC
  VERIFY((std::fetestexcept(FE_INVALID) & FE_INVALID) != 0);
  std::feclearexcept(FE_ALL_EXCEPT);
#endif

  const Complex inf_nan = numext::exp(Complex(inf, nan));
  VERIFY((numext::isinf)(inf_nan.real()));
  VERIFY((numext::isnan)(inf_nan.imag()));
}

#if EIGEN_COMP_MSVC
#pragma fenv_access(off)
#pragma float_control(pop)
#endif

template <typename T>
std::enable_if_t<NumTraits<T>::IsInteger && NumTraits<T>::IsSigned, T> random_abs2_input() {
  const T safeAbs2Input = static_cast<T>(std::sqrt(static_cast<long double>(NumTraits<T>::highest())));
  return internal::random<T>(-safeAbs2Input, safeAbs2Input);
}

// Note: bool resolves here (IsInteger && !IsSigned), but check_abs<bool> is fully
// specialized and never calls random_abs2_input<bool>().
template <typename T>
std::enable_if_t<NumTraits<T>::IsInteger && !NumTraits<T>::IsSigned, T> random_abs2_input() {
  const T safeAbs2Input = static_cast<T>(std::sqrt(static_cast<long double>(NumTraits<T>::highest())));
  return internal::random<T>(T(0), safeAbs2Input);
}

template <typename T>
std::enable_if_t<!NumTraits<T>::IsInteger, T> random_abs2_input() {
  typedef typename NumTraits<T>::Real Real;
  return internal::random<T>() / Real(2);
}

template <typename T>
void check_abs() {
  typedef typename NumTraits<T>::Real Real;
  Real zero(0);

  if (NumTraits<T>::IsSigned) VERIFY_IS_EQUAL(numext::abs(numext::negate(T(1))), T(1));
  VERIFY_IS_EQUAL(numext::abs(T(0)), T(0));
  VERIFY_IS_EQUAL(numext::abs(T(1)), T(1));

  for (int k = 0; k < 100; ++k) {
    T x = random_abs2_input<T>();
    if (NumTraits<T>::IsSigned) {
      VERIFY_IS_EQUAL(numext::abs(x), numext::abs(numext::negate(x)));
      VERIFY(numext::abs(numext::negate(x)) >= zero);
    }
    VERIFY(numext::abs(x) >= zero);
    VERIFY_IS_APPROX(numext::abs2(x), numext::abs2(numext::abs(x)));
  }
}

template <>
void check_abs<bool>() {
  for (bool x : {true, false}) {
    VERIFY_IS_EQUAL(numext::abs(x), x);
    VERIFY(numext::abs(x) >= false);
    VERIFY_IS_EQUAL(numext::abs2(x), numext::abs2(numext::abs(x)));
  }
}

// numext::sign(z) = z / |z| for complex z. The interesting inputs are the ends of the range, where
// forming 1/|z| first would overflow (subnormal |z|) or itself be subnormal, hence inexact (|z| near max).
template <typename T>
void check_complex_sign() {
  typedef typename NumTraits<T>::Real Real;
  const Real zero(0), one(1);

  VERIFY_IS_EQUAL(numext::sign(T(zero, zero)), T(zero, zero));
  VERIFY_IS_EQUAL(numext::sign(T(one, zero)), T(one, zero));
  VERIFY_IS_EQUAL(numext::sign(T(zero, -one)), T(zero, -one));

  for (Real r : {std::numeric_limits<Real>::denorm_min(), (std::numeric_limits<Real>::min)(),
                 (std::numeric_limits<Real>::max)()}) {
    VERIFY_IS_EQUAL(numext::sign(T(r, zero)), T(one, zero));
    VERIFY_IS_EQUAL(numext::sign(T(-r, zero)), T(-one, zero));
    VERIFY_IS_EQUAL(numext::sign(T(zero, r)), T(zero, one));
  }

  // Off the axes the magnitude is only as accurate as abs() itself, which cannot resolve |z| for a z
  // whose components are at the bottom of the subnormal range; from the smallest normal upwards it can.
  for (Real r : {(std::numeric_limits<Real>::min)(), one, (std::numeric_limits<Real>::max)() / Real(2)}) {
    const T s = numext::sign(T(r, r));
    VERIFY_IS_APPROX(numext::abs(s), one);
    VERIFY_IS_EQUAL(numext::real(s), numext::imag(s));
  }
}

// The FMA implementation must retain the product's rounding error even when scalar madd is unfused.
template <typename T>
void check_twoprod_pair(const T& x, const T& y) {
  T hi, lo;
  internal::twoprod(x, y, hi, lo);
  VERIFY_IS_EQUAL(hi, x * y);
  // fma is exact by IEEE-754 contract, so this is the definition of the low word rather than an
  // independent approximation of it.
  EIGEN_USING_STD(fma);
  VERIFY_IS_EQUAL(lo, fma(x, y, -hi));
  VERIFY_IS_EQUAL(internal::twoprod_low(x, y, hi), fma(x, y, -hi));
}

template <typename T>
void check_twoprod() {
  // (1 + epsilon) * (1 - epsilon) = 1 - epsilon^2 exposes a lost low word in every IEEE type.
  const T epsilon = NumTraits<T>::epsilon();
  check_twoprod_pair(T(1) + epsilon, T(1) - epsilon);
  for (int k = 0; k < 100; ++k) {
    check_twoprod_pair(internal::random<T>(T(-1), T(1)), internal::random<T>(T(-1), T(1)));
  }
}

#ifdef EIGEN_VECTORIZE_FMA
// Custom scalars can return a lazy expression from unary minus.
struct TwoprodScalar {
  struct Negation {
    const TwoprodScalar& operand;
  };

  TwoprodScalar() = default;
  explicit TwoprodScalar(double v) : value(v) {}
  TwoprodScalar(const Negation& negation) : value(-negation.operand.value) {}

  Negation operator-() const { return {*this}; }
  TwoprodScalar operator*(const TwoprodScalar& other) const { return TwoprodScalar(value * other.value); }

  friend TwoprodScalar fma(const TwoprodScalar& x, const TwoprodScalar& y, const TwoprodScalar& z) {
    return TwoprodScalar(std::fma(x.value, y.value, z.value));
  }

  double value = 0;
};

void check_twoprod_negation_expression() {
  STATIC_CHECK(internal::is_scalar<TwoprodScalar>::value);
  STATIC_CHECK(internal::has_fma<TwoprodScalar>::value);
  STATIC_CHECK((!std::is_same<decltype(-std::declval<TwoprodScalar>()), TwoprodScalar>::value));
  const double epsilon = NumTraits<double>::epsilon();
  for (double sign : {-1.0, 1.0}) {
    const TwoprodScalar x(sign * (1 + epsilon)), y(1 - epsilon);
    TwoprodScalar hi, lo;
    internal::twoprod(x, y, hi, lo);
    VERIFY_IS_EQUAL(hi.value, sign);
    VERIFY_IS_EQUAL(lo.value, -sign * epsilon * epsilon);
    VERIFY_IS_EQUAL(internal::twoprod_low(x, y, hi).value, -sign * epsilon * epsilon);
  }
}
#endif

template <typename T>
void check_arg() {
  typedef typename NumTraits<T>::Real Real;
  VERIFY_IS_EQUAL(numext::abs(T(0)), T(0));
  VERIFY_IS_EQUAL(numext::abs(T(1)), T(1));

  for (int k = 0; k < 100; ++k) {
    T x = internal::random<T>();
    Real y = numext::arg(x);
    VERIFY_IS_APPROX(y, std::arg(x));
  }
}

template <typename T>
struct check_sqrt_impl {
  static void run() {
    for (int i = 0; i < 1000; ++i) {
      const T x = numext::abs(internal::random<T>());
      const T sqrtx = numext::sqrt(x);
      VERIFY_IS_APPROX(sqrtx * sqrtx, x);
    }

    // Corner cases.
    const T zero = T(0);
    const T one = T(1);
    const T inf = std::numeric_limits<T>::infinity();
    const T nan = std::numeric_limits<T>::quiet_NaN();
    VERIFY_IS_EQUAL(numext::sqrt(zero), zero);
    VERIFY_IS_EQUAL(numext::sqrt(inf), inf);
    VERIFY((numext::isnan)(numext::sqrt(nan)));
    VERIFY((numext::isnan)(numext::sqrt(-one)));
  }
};

template <typename T>
struct check_sqrt_impl<std::complex<T>> {
  static void run() {
    typedef std::complex<T> ComplexT;

    for (int i = 0; i < 1000; ++i) {
      const ComplexT x = internal::random<ComplexT>();
      const ComplexT sqrtx = numext::sqrt(x);
      VERIFY_IS_APPROX(sqrtx * sqrtx, x);
    }

    // Corner cases.
    const T zero = T(0);
    const T one = T(1);
    const T inf = std::numeric_limits<T>::infinity();
    const T nan = std::numeric_limits<T>::quiet_NaN();

    // Set of corner cases from https://en.cppreference.com/w/cpp/numeric/complex/sqrt
    const int kNumCorners = 20;
    const ComplexT corners[kNumCorners][2] = {
        {ComplexT(zero, zero), ComplexT(zero, zero)},  {ComplexT(-zero, zero), ComplexT(zero, zero)},
        {ComplexT(zero, -zero), ComplexT(zero, zero)}, {ComplexT(-zero, -zero), ComplexT(zero, zero)},
        {ComplexT(one, inf), ComplexT(inf, inf)},      {ComplexT(nan, inf), ComplexT(inf, inf)},
        {ComplexT(one, -inf), ComplexT(inf, -inf)},    {ComplexT(nan, -inf), ComplexT(inf, -inf)},
        {ComplexT(-inf, one), ComplexT(zero, inf)},    {ComplexT(inf, one), ComplexT(inf, zero)},
        {ComplexT(-inf, -one), ComplexT(zero, -inf)},  {ComplexT(inf, -one), ComplexT(inf, -zero)},
        {ComplexT(-inf, nan), ComplexT(nan, inf)},     {ComplexT(inf, nan), ComplexT(inf, nan)},
        {ComplexT(zero, nan), ComplexT(nan, nan)},     {ComplexT(one, nan), ComplexT(nan, nan)},
        {ComplexT(nan, zero), ComplexT(nan, nan)},     {ComplexT(nan, one), ComplexT(nan, nan)},
        {ComplexT(nan, -one), ComplexT(nan, nan)},     {ComplexT(nan, nan), ComplexT(nan, nan)},
    };

    for (int i = 0; i < kNumCorners; ++i) {
      const ComplexT& x = corners[i][0];
      const ComplexT sqrtx = corners[i][1];
      VERIFY_IS_EQUAL_OR_NANS(numext::sqrt(x), sqrtx);
    }
  }
};

template <typename T>
void check_sqrt() {
  check_sqrt_impl<T>::run();
}

template <typename T>
struct check_rsqrt_impl {
  static void run() {
    const T zero = T(0);
    const T one = T(1);
    const T inf = std::numeric_limits<T>::infinity();
    const T nan = std::numeric_limits<T>::quiet_NaN();

    for (int i = 0; i < 1000; ++i) {
      const T x = numext::abs(internal::random<T>());
      const T rsqrtx = numext::rsqrt(x);
      const T invx = one / x;
      VERIFY_IS_APPROX(rsqrtx * rsqrtx, invx);
    }

    // Corner cases.
    VERIFY_IS_EQUAL(numext::rsqrt(zero), inf);
    VERIFY_IS_EQUAL(numext::rsqrt(inf), zero);
    VERIFY((numext::isnan)(numext::rsqrt(nan)));
    VERIFY((numext::isnan)(numext::rsqrt(-one)));
  }
};

template <typename T>
struct check_rsqrt_impl<std::complex<T>> {
  static void run() {
    typedef std::complex<T> ComplexT;
    const T zero = T(0);
    const T one = T(1);
    const T inf = std::numeric_limits<T>::infinity();
    const T nan = std::numeric_limits<T>::quiet_NaN();

    for (int i = 0; i < 1000; ++i) {
      const ComplexT x = internal::random<ComplexT>();
      const ComplexT invx = ComplexT(one, zero) / x;
      const ComplexT rsqrtx = numext::rsqrt(x);
      VERIFY_IS_APPROX(rsqrtx * rsqrtx, invx);
    }

// GCC and MSVC differ in their treatment of 1/(0 + 0i)
//   GCC/clang = (inf, nan)
//   MSVC = (nan, nan)
// and 1 / (x + inf i)
//   GCC/clang = (0, 0)
//   MSVC = (nan, nan)
#if (EIGEN_COMP_GNUC)
    {
      const int kNumCorners = 20;
      const ComplexT corners[kNumCorners][2] = {
          // Only consistent across GCC, clang
          {ComplexT(zero, zero), ComplexT(zero, zero)},
          {ComplexT(-zero, zero), ComplexT(zero, zero)},
          {ComplexT(zero, -zero), ComplexT(zero, zero)},
          {ComplexT(-zero, -zero), ComplexT(zero, zero)},
          {ComplexT(one, inf), ComplexT(inf, inf)},
          {ComplexT(nan, inf), ComplexT(inf, inf)},
          {ComplexT(one, -inf), ComplexT(inf, -inf)},
          {ComplexT(nan, -inf), ComplexT(inf, -inf)},
          // Consistent across GCC, clang, MSVC
          {ComplexT(-inf, one), ComplexT(zero, inf)},
          {ComplexT(inf, one), ComplexT(inf, zero)},
          {ComplexT(-inf, -one), ComplexT(zero, -inf)},
          {ComplexT(inf, -one), ComplexT(inf, -zero)},
          {ComplexT(-inf, nan), ComplexT(nan, inf)},
          {ComplexT(inf, nan), ComplexT(inf, nan)},
          {ComplexT(zero, nan), ComplexT(nan, nan)},
          {ComplexT(one, nan), ComplexT(nan, nan)},
          {ComplexT(nan, zero), ComplexT(nan, nan)},
          {ComplexT(nan, one), ComplexT(nan, nan)},
          {ComplexT(nan, -one), ComplexT(nan, nan)},
          {ComplexT(nan, nan), ComplexT(nan, nan)},
      };

      for (int i = 0; i < kNumCorners; ++i) {
        const ComplexT& x = corners[i][0];
        const ComplexT rsqrtx = ComplexT(one, zero) / corners[i][1];
        VERIFY_IS_EQUAL_OR_NANS(numext::rsqrt(x), rsqrtx);
      }
    }
#endif
  }
};

template <typename T>
void check_rsqrt() {
  check_rsqrt_impl<T>::run();
}

template <typename T>
struct check_signbit_impl {
  static void run() {
    T true_mask;
    std::memset(static_cast<void*>(&true_mask), 0xff, sizeof(T));
    T false_mask;
    std::memset(static_cast<void*>(&false_mask), 0x00, sizeof(T));

    std::vector<T> negative_values;
    std::vector<T> non_negative_values;

    if (NumTraits<T>::IsInteger) {
      negative_values = {static_cast<T>(-1), static_cast<T>(NumTraits<T>::lowest())};
      non_negative_values = {static_cast<T>(0), static_cast<T>(1), static_cast<T>(NumTraits<T>::highest())};
    } else {
      // does not have sign bit
      const T pos_zero = static_cast<T>(0.0);
      const T pos_one = static_cast<T>(1.0);
      const T pos_inf = std::numeric_limits<T>::infinity();
      const T pos_nan = std::numeric_limits<T>::quiet_NaN();
      // has sign bit
      const T neg_zero = numext::negate(pos_zero);
      const T neg_one = numext::negate(pos_one);
      const T neg_inf = numext::negate(pos_inf);
      const T neg_nan = numext::negate(pos_nan);

      negative_values = {neg_zero, neg_one, neg_inf, neg_nan};
      non_negative_values = {pos_zero, pos_one, pos_inf, pos_nan};
    }

    auto check_all = [](auto values, auto expected) {
      bool all_pass = true;
      for (T val : values) {
        const T numext_val = numext::signbit(val);
        bool not_same = internal::predux_any(internal::bitwise_helper<T>::bitwise_xor(expected, numext_val));
        all_pass = all_pass && !not_same;
        if (not_same) std::cout << "signbit(" << val << ") = " << numext_val << " != " << expected << std::endl;
      }
      return all_pass;
    };

    bool check_all_pass = check_all(non_negative_values, false_mask);
    check_all_pass = check_all_pass && check_all(negative_values, (NumTraits<T>::IsSigned ? true_mask : false_mask));
    VERIFY(check_all_pass);
  }
};
template <typename T>
void check_signbit() {
  check_signbit_impl<T>::run();
}

template <typename T>
void check_nextafter() {
  const T zero(0);
  const T one(1);
  const T two(2);
  // NumTraits::epsilon() is the ulp at 1. std::numeric_limits reports the far smaller representational gap for the
  // non-IEEE IBM double-double `long double`, which is not the step nextafter takes.
  const T eps = NumTraits<T>::epsilon();
  const T denorm_min = std::numeric_limits<T>::denorm_min();
  const T inf = std::numeric_limits<T>::infinity();
  const T nan = std::numeric_limits<T>::quiet_NaN();
  const T max = (std::numeric_limits<T>::max)();

  // from == to returns to.
  VERIFY(numext::equal_strict(numext::nextafter(one, one), one));
  // Stepping up from 1 and back down returns 1.
  VERIFY(numext::equal_strict(numext::nextafter(numext::nextafter(one, two), zero), one));
  // One-ulp steps around 1.
  VERIFY(numext::equal_strict(numext::nextafter(one, two), one + eps));
  VERIFY(numext::equal_strict(numext::nextafter(one + eps, zero), one));
  // The neighbors of ±0 are the smallest subnormals, with the sign of the direction.
  VERIFY(numext::equal_strict(numext::nextafter(zero, one), denorm_min));
  VERIFY(numext::equal_strict(numext::nextafter(zero, -one), -denorm_min));
  VERIFY(numext::equal_strict(numext::nextafter(-zero, one), denorm_min));
  VERIFY(numext::equal_strict(numext::copysign(one, numext::nextafter(zero, -one)), -one));
  // Stepping the smallest subnormals toward the other sign lands on the zero
  // of the starting sign (IEEE-754 nextUp/nextDown).
  VERIFY(numext::equal_strict(numext::nextafter(denorm_min, -one), zero));
  VERIFY(numext::equal_strict(numext::copysign(one, numext::nextafter(denorm_min, -one)), one));
  VERIFY(numext::equal_strict(numext::copysign(one, numext::nextafter(-denorm_min, one)), -one));
  // Infinities saturate and unsaturate by one step.
  VERIFY(numext::equal_strict(numext::nextafter(max, inf), inf));
  VERIFY(numext::equal_strict(numext::nextafter(inf, zero), max));
  // NaNs propagate.
  VERIFY((numext::isnan)(numext::nextafter(nan, one)));
  VERIFY((numext::isnan)(numext::nextafter(one, nan)));
}

template <typename T>
void check_ceil_power_of_two() {
  const T zero(0);
  const T one(1);
  const T two(2);
  const T min = (std::numeric_limits<T>::min)();
  const T denormMin = std::numeric_limits<T>::denorm_min();
  const T inf = std::numeric_limits<T>::infinity();

  VERIFY_IS_EQUAL(numext::ceil_power_of_two(zero), zero);
  VERIFY_IS_EQUAL(numext::ceil_power_of_two(denormMin), min);
  VERIFY_IS_EQUAL(numext::ceil_power_of_two(min), min);
  VERIFY_IS_EQUAL(numext::ceil_power_of_two(T(0.75)), one);
  VERIFY_IS_EQUAL(numext::ceil_power_of_two(one), one);
  VERIFY_IS_EQUAL(numext::ceil_power_of_two(one + NumTraits<T>::epsilon()), two);
  VERIFY_IS_EQUAL(numext::ceil_power_of_two(T(3)), T(4));
  VERIFY_IS_EQUAL(numext::ceil_power_of_two((std::numeric_limits<T>::max)()), inf);
}

template <typename T>
void check_shift() {
  using SignedT = typename numext::get_integer_by_size<sizeof(T)>::signed_type;
  using UnsignedT = typename numext::get_integer_by_size<sizeof(T)>::unsigned_type;
  constexpr int kNumBits = CHAR_BIT * sizeof(T);
  for (int i = 0; i < 1000; ++i) {
    const T a = internal::random<T>();
    for (int s = 1; s < kNumBits; s++) {
      T a_bsll = numext::logical_shift_left(a, s);
      T a_bsll_ref = numext::bit_cast<T, UnsignedT>(numext::bit_cast<UnsignedT, T>(a) << s);
      VERIFY_IS_EQUAL(a_bsll, a_bsll_ref);
      T a_bsrl = numext::logical_shift_right(a, s);
      T a_bsrl_ref = numext::bit_cast<T, UnsignedT>(numext::bit_cast<UnsignedT, T>(a) >> s);
      VERIFY_IS_EQUAL(a_bsrl, a_bsrl_ref);
      T a_bsra = numext::arithmetic_shift_right(a, s);
      // An unsigned T has no sign bit to propagate, so the arithmetic shift must agree with the
      // logical one; this is what every backend's parithmetic_shift_right on unsigned packets does.
      T a_bsra_ref =
          NumTraits<T>::IsSigned ? numext::bit_cast<T, SignedT>(numext::bit_cast<SignedT, T>(a) >> s) : a_bsrl_ref;
      VERIFY_IS_EQUAL(a_bsra, a_bsra_ref);
    }
  }
}

EIGEN_DECLARE_TEST(numext) {
  for (int k = 0; k < g_repeat; ++k) {
    CALL_SUBTEST(check_copysign<half>());
    CALL_SUBTEST(check_copysign<bfloat16>());
    CALL_SUBTEST(check_copysign<float>());
    CALL_SUBTEST(check_copysign<double>());
    CALL_SUBTEST(check_copysign<long double>());
    CALL_SUBTEST(check_copysign<std::complex<float>>());
    CALL_SUBTEST(check_copysign<std::complex<double>>());

    CALL_SUBTEST(check_copysign<bool>());
    CALL_SUBTEST(check_copysign<int8_t>());
    CALL_SUBTEST(check_copysign<int16_t>());
    CALL_SUBTEST(check_copysign<int32_t>());
    CALL_SUBTEST(check_copysign<int64_t>());
    CALL_SUBTEST(check_copysign<uint8_t>());
    CALL_SUBTEST(check_copysign<uint16_t>());
    CALL_SUBTEST(check_copysign<uint32_t>());
    CALL_SUBTEST(check_copysign<uint64_t>());

    CALL_SUBTEST(check_negate<signed char>());
    CALL_SUBTEST(check_negate<unsigned char>());
    CALL_SUBTEST(check_negate<short>());
    CALL_SUBTEST(check_negate<unsigned short>());
    CALL_SUBTEST(check_negate<int>());
    CALL_SUBTEST(check_negate<unsigned int>());
    CALL_SUBTEST(check_negate<long>());
    CALL_SUBTEST(check_negate<unsigned long>());
    CALL_SUBTEST(check_negate<half>());
    CALL_SUBTEST(check_negate<bfloat16>());
    CALL_SUBTEST(check_negate<float>());
    CALL_SUBTEST(check_negate<double>());
    CALL_SUBTEST(check_negate<long double>());
    CALL_SUBTEST(check_negate<std::complex<float>>());
    CALL_SUBTEST(check_negate<std::complex<double>>());

    CALL_SUBTEST(check_complex_exp<float>());
    CALL_SUBTEST(check_complex_exp<double>());

    CALL_SUBTEST(check_abs<bool>());
    CALL_SUBTEST(check_abs<signed char>());
    CALL_SUBTEST(check_abs<unsigned char>());
    CALL_SUBTEST(check_abs<short>());
    CALL_SUBTEST(check_abs<unsigned short>());
    CALL_SUBTEST(check_abs<int>());
    CALL_SUBTEST(check_abs<unsigned int>());
    CALL_SUBTEST(check_abs<long>());
    CALL_SUBTEST(check_abs<unsigned long>());
    CALL_SUBTEST(check_abs<half>());
    CALL_SUBTEST(check_abs<bfloat16>());
    CALL_SUBTEST(check_abs<float>());
    CALL_SUBTEST(check_abs<double>());
    CALL_SUBTEST(check_abs<long double>());
    CALL_SUBTEST(check_abs<std::complex<float>>());
    CALL_SUBTEST(check_abs<std::complex<double>>());

    CALL_SUBTEST(check_complex_sign<std::complex<float>>());
    CALL_SUBTEST(check_complex_sign<std::complex<double>>());

    CALL_SUBTEST(check_twoprod<float>());
    CALL_SUBTEST(check_twoprod<double>());
    CALL_SUBTEST(check_twoprod<long double>());
    CALL_SUBTEST(check_twoprod<half>());
    CALL_SUBTEST(check_twoprod<bfloat16>());
    CALL_SUBTEST(check_twoprod_contraction<float>());
    CALL_SUBTEST(check_twoprod_contraction<double>());
#ifdef EIGEN_VECTORIZE_FMA
    CALL_SUBTEST(check_twoprod_negation_expression());
#endif

    CALL_SUBTEST(check_arg<std::complex<float>>());
    CALL_SUBTEST(check_arg<std::complex<double>>());

    CALL_SUBTEST(check_sqrt<float>());
    CALL_SUBTEST(check_sqrt<double>());
    CALL_SUBTEST(check_sqrt<std::complex<float>>());
    CALL_SUBTEST(check_sqrt<std::complex<double>>());

    CALL_SUBTEST(check_rsqrt<float>());
    CALL_SUBTEST(check_rsqrt<double>());
    CALL_SUBTEST(check_rsqrt<std::complex<float>>());
    CALL_SUBTEST(check_rsqrt<std::complex<double>>());

    CALL_SUBTEST(check_signbit<half>());
    CALL_SUBTEST(check_signbit<bfloat16>());
    CALL_SUBTEST(check_signbit<float>());
    CALL_SUBTEST(check_signbit<double>());

    CALL_SUBTEST(check_signbit<uint8_t>());
    CALL_SUBTEST(check_signbit<uint16_t>());
    CALL_SUBTEST(check_signbit<uint32_t>());
    CALL_SUBTEST(check_signbit<uint64_t>());

    CALL_SUBTEST(check_signbit<int8_t>());
    CALL_SUBTEST(check_signbit<int16_t>());
    CALL_SUBTEST(check_signbit<int32_t>());
    CALL_SUBTEST(check_signbit<int64_t>());

    CALL_SUBTEST(check_nextafter<half>());
    CALL_SUBTEST(check_nextafter<bfloat16>());
    CALL_SUBTEST(check_nextafter<float>());
    CALL_SUBTEST(check_nextafter<double>());
    CALL_SUBTEST(check_nextafter<long double>());

    CALL_SUBTEST(check_ceil_power_of_two<half>());
    CALL_SUBTEST(check_ceil_power_of_two<bfloat16>());
    CALL_SUBTEST(check_ceil_power_of_two<float>());
    CALL_SUBTEST(check_ceil_power_of_two<double>());
    CALL_SUBTEST(check_ceil_power_of_two<long double>());

    CALL_SUBTEST(check_shift<int8_t>());
    CALL_SUBTEST(check_shift<int16_t>());
    CALL_SUBTEST(check_shift<int32_t>());
    CALL_SUBTEST(check_shift<int64_t>());

    CALL_SUBTEST(check_shift<uint8_t>());
    CALL_SUBTEST(check_shift<uint16_t>());
    CALL_SUBTEST(check_shift<uint32_t>());
    CALL_SUBTEST(check_shift<uint64_t>());
  }
}
