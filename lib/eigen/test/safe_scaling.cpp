// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include "main.h"
#include "fp_control.h"
#define EIGEN_TEST_ANNOYING_SCALAR_DONT_THROW
#include "AnnoyingScalar.h"

template <typename T>
void check_power_of_two_scaling_factor() {
  static_assert(internal::supports_power_of_two_scaling<T>::value, "power-of-two scaling must be enabled");
  using Factors = internal::safe_scaling_factors<T>;

  const auto check_factor = [](const T& value) {
    const auto factors = internal::safe_scaling<T>::compute_floor_factors(value);
    VERIFY(factors.scale >= (std::numeric_limits<T>::min)());
    VERIFY(factors.invScale >= (std::numeric_limits<T>::min)());
    VERIFY_IS_EQUAL(factors.scale * factors.invScale, T(1));
    return factors;
  };

  const auto check_round_trip = [&](const T& value) {
    const auto factors = check_factor(value);
    Matrix<T, 1, 1> input;
    input(0) = value;
    Matrix<T, 1, 1> scaled;
    const auto scaleToFactors = internal::safe_scaling<T>::scale_to(scaled, input, value);
    VERIFY_IS_EQUAL(scaleToFactors.scale, factors.scale);
    VERIFY_IS_EQUAL(scaleToFactors.invScale, factors.invScale);
    Matrix<T, 1, 1> scaledInPlace;
    scaledInPlace(0) = value;
    internal::safe_scaling<T>::scale_in_place(scaledInPlace, value, factors);
    VERIFY_IS_EQUAL(scaledInPlace, scaled);

    T restored;
    internal::safe_scaling<T>::unscale_to(restored, scaled(0), factors);
    VERIFY_IS_EQUAL(restored, value);
    internal::safe_scaling<T>::unscale_in_place(scaled, factors);
    VERIFY_IS_EQUAL(scaled(0), value);
  };

  const auto denormFactors = check_factor(std::numeric_limits<T>::denorm_min());
  VERIFY_IS_EQUAL(denormFactors.scale, (std::numeric_limits<T>::min)());
  Factors normalReciprocalFactors;
  VERIFY(internal::safe_scaling<T>::try_compute_ceiling_factors_with_normal_reciprocal(
      std::numeric_limits<T>::denorm_min(), (std::numeric_limits<T>::min)(), normalReciprocalFactors));
  VERIFY_IS_EQUAL(normalReciprocalFactors.scale, denormFactors.scale);
  VERIFY_IS_EQUAL(normalReciprocalFactors.invScale, denormFactors.invScale);

  // bfloat16 arithmetic widens to float, whose subnormal inputs may be flushed before scaling can recover them.
  if (!std::is_same<T, bfloat16>::value || !ScopedFlushToZero::hardwareFlushesSubnormalInputs()) {
    using InputScalar = std::conditional_t<std::is_floating_point<T>::value, T, float>;
    volatile InputScalar denormInput = static_cast<InputScalar>(std::numeric_limits<T>::denorm_min());
    const T denormMin = static_cast<T>(denormInput);
    Matrix<T, 1, 1> denormInputMatrix;
    denormInputMatrix(0) = denormMin;
    Matrix<T, 1, 1> scaledDenorm;
    const auto factors = internal::safe_scaling<T>::scale_to(scaledDenorm, denormInputMatrix, denormMin);
    Matrix<T, 1, 1> scaledDenormInPlace;
    scaledDenormInPlace(0) = denormMin;
    internal::safe_scaling<T>::scale_in_place(scaledDenormInPlace, denormMin, factors);
    VERIFY_IS_EQUAL(scaledDenormInPlace, scaledDenorm);
    VERIFY(scaledDenorm(0) > T(0));
  }

  check_round_trip((std::numeric_limits<T>::min)());
  check_round_trip(T(0.75));
  check_round_trip(T(3));
  check_round_trip((std::numeric_limits<T>::max)());

  const auto floorFactors = internal::safe_scaling<T>::compute_floor_factors(T(1.5));
  const auto ceilingFactors = internal::safe_scaling<T>::compute_ceiling_factors_with_normal_reciprocal(T(1.5));
  VERIFY_IS_EQUAL(floorFactors.scale, T(1));
  VERIFY_IS_EQUAL(ceilingFactors.scale, T(2));

  const T highest = (std::numeric_limits<T>::max)();
  const auto highestCeilingFactors = internal::safe_scaling<T>::compute_ceiling_factors_with_normal_reciprocal(highest);
  VERIFY_IS_EQUAL(highestCeilingFactors.scale, check_factor(highest).scale);
  VERIFY_IS_EQUAL(highestCeilingFactors.scale * highestCeilingFactors.invScale, T(1));
  const T scaledHighest = highest * highestCeilingFactors.invScale;
  VERIFY(scaledHighest >= T(1));
  if (std::numeric_limits<T>::max_exponent + std::numeric_limits<T>::min_exponent <= 3) {
    VERIFY(scaledHighest < T(4));
  }
}

template <typename T>
void check_safe_scaling_special_values() {
  using Scaling = internal::safe_scaling<T>;
  using Factors = internal::safe_scaling_factors<T>;

  const Factors identity;
  VERIFY_IS_EQUAL(identity.scale, T(1));
  VERIFY_IS_EQUAL(identity.invScale, T(1));

  Matrix<T, 1, 1> zero = Matrix<T, 1, 1>::Zero();
  Matrix<T, 1, 1> scaledZero;
  const auto zeroFactors = Scaling::scale_to(scaledZero, zero, T(0));
  VERIFY_IS_EQUAL(zeroFactors.scale, T(1));
  VERIFY_IS_EQUAL(zeroFactors.invScale, T(1));
  VERIFY_IS_EQUAL(scaledZero(0), T(0));

  for (const T special : {std::numeric_limits<T>::infinity(), std::numeric_limits<T>::quiet_NaN()}) {
    Matrix<T, 2, 1> input;
    input << special, T(2);
    Matrix<T, 2, 1> scaled;
    const auto factors = Scaling::scale_to(scaled, input, special);
    VERIFY_IS_EQUAL(factors.scale, T(1));
    VERIFY_IS_EQUAL(factors.invScale, T(1));
    VERIFY_IS_EQUAL(scaled(1), T(2));
    if ((numext::isnan)(special)) {
      VERIFY((numext::isnan)(scaled(0)));
    } else {
      VERIFY((numext::isinf)(scaled(0)));
    }
  }
}

void check_safe_scaling_special_value_frontends() {
  using Scaling = internal::safe_scaling<float>;
  using Factors = internal::safe_scaling_factors<float>;

  for (const float special : {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()}) {
    Vector2f input(special, 2.0f);
    Vector2f scaledTo;
    const auto scaleToFactors = Scaling::scale_to(scaledTo, input, special);
    VERIFY_IS_EQUAL(scaleToFactors.scale, 1.0f);
    VERIFY_IS_EQUAL(scaleToFactors.invScale, 1.0f);

    Vector2f scaledExpression;
    const Factors expressionFactors =
        Scaling::with_scaled(input, special, [&](const auto& expression) { scaledExpression = expression; });
    VERIFY_IS_EQUAL(expressionFactors.scale, 1.0f);
    VERIFY_IS_EQUAL(expressionFactors.invScale, 1.0f);
    VERIFY_IS_EQUAL(scaledTo(1), 2.0f);
    VERIFY_IS_EQUAL(scaledExpression(1), 2.0f);
    if ((numext::isnan)(special)) {
      VERIFY((numext::isnan)(scaledTo(0)));
      VERIFY((numext::isnan)(scaledExpression(0)));
    } else {
      VERIFY((numext::isinf)(scaledTo(0)));
      VERIFY((numext::isinf)(scaledExpression(0)));
    }
  }
}

void check_arithmetic_safe_scaling_fallback() {
  using Scaling = internal::safe_scaling<double, false>;
  using Factors = internal::safe_scaling_factors<double>;

  Matrix<double, 2, 1> input;
  input << 3.0, 6.0;
  Matrix<double, 2, 1> scaled;
  const auto factors = Scaling::scale_to(scaled, input, 3.0);
  VERIFY_IS_EQUAL(factors.scale, 3.0);
  VERIFY_IS_EQUAL(factors.invScale, 1.0);
  VERIFY_IS_EQUAL(scaled(0), 1.0);
  VERIFY_IS_EQUAL(scaled(1), 2.0);
  Scaling::unscale_in_place(scaled, factors);
  VERIFY_IS_EQUAL(scaled, input);

  const auto check_extreme_unscale = [](const double value) {
    const Factors extremeFactors = Scaling::compute_floor_factors(value);
    double restored;
    Scaling::unscale_to(restored, 1.0, extremeFactors);
    VERIFY_IS_EQUAL(restored, value);
    restored = 1.0;
    Scaling::unscale_in_place(restored, extremeFactors);
    VERIFY_IS_EQUAL(restored, value);
  };
  check_extreme_unscale((std::numeric_limits<double>::max)());
  volatile double denormMinInput = std::numeric_limits<double>::denorm_min();
  const double denormMin = denormMinInput;
  if (denormMin > 0.0) check_extreme_unscale(denormMin);

  Factors normalReciprocalFactors;
  const double normalMin = (std::numeric_limits<double>::min)();
  VERIFY(Scaling::try_compute_ceiling_factors_with_normal_reciprocal(3.0, normalMin, normalReciprocalFactors));
  VERIFY_IS_EQUAL(normalReciprocalFactors.scale, 3.0);
  VERIFY_IS_EQUAL(normalReciprocalFactors.invScale, 1.0 / 3.0);
  VERIFY(!Scaling::try_compute_ceiling_factors_with_normal_reciprocal(std::numeric_limits<double>::denorm_min(),
                                                                      normalMin, normalReciprocalFactors));
  VERIFY(!Scaling::try_compute_ceiling_factors_with_normal_reciprocal((std::numeric_limits<double>::max)(), normalMin,
                                                                      normalReciprocalFactors));
}

void check_custom_scalar_scaling_exceptions() {
  using Scalar = AnnoyingScalar;
  using Scaling = internal::safe_scaling<Scalar>;
  static_assert(!internal::supports_power_of_two_scaling<Scalar>::value, "exercise arithmetic scaling");
  volatile float denormInput = std::numeric_limits<float>::denorm_min();
  const float denorm = denormInput;
  const float twiceDenorm = denorm + denorm;
  if (!(denorm > 0.0f && twiceDenorm > 0.0f)) return;  // No custom-scalar subnormal recovery is promised.

  Matrix<Scalar, 2, 1> input;
  input << Scalar(denorm), Scalar(twiceDenorm);
  Matrix<Scalar, 2, 1> scaled;
  std::fenv_t savedEnvironment;
  if (std::feholdexcept(&savedEnvironment) != 0) return;
  const auto factors = Scaling::scale_to(scaled, input, Scalar(twiceDenorm));
  const int overflow = std::fetestexcept(FE_OVERFLOW);
  std::fesetenv(&savedEnvironment);
  VERIFY_IS_EQUAL(overflow, 0);
  VERIFY_IS_EQUAL(scaled(0), Scalar(0.5));
  VERIFY_IS_EQUAL(scaled(1), Scalar(1));
  Scaling::unscale_in_place(scaled, factors);
  VERIFY_IS_EQUAL(scaled, input);
}

template <typename Scalar>
void check_scale_binary_by_power_of_two() {
  using Binary = internal::binary_floating_point_traits<Scalar>;
  using Bits = typename Binary::Bits;
  constexpr int kMinFactorExponent = 1 - std::numeric_limits<Scalar>::min_exponent - Binary::kFractionBits;
  constexpr int kMaxFactorExponent = 1 - std::numeric_limits<Scalar>::min_exponent;
  const Bits magnitudes[] = {Bits(0),
                             Bits(1),
                             Bits(3),
                             Binary::kFractionMask,
                             Binary::kExponentUnit,
                             Binary::kExponentUnit + (Binary::kExponentUnit >> 1)};

  ScopedFlushToZero flushToZero;
  // Cover the entire factor range selected by tiny-input recovery. Form the reference from integer significands,
  // not subnormal floating-point inputs, so it remains valid under FTZ/DAZ.
  for (int exponent = kMinFactorExponent; exponent <= kMaxFactorExponent; ++exponent) {
    const Scalar factor = numext::ldexp(Scalar(1), exponent);
    for (Bits magnitude : magnitudes) {
      const Scalar expected = numext::ldexp(Scalar(magnitude), std::numeric_limits<Scalar>::min_exponent -
                                                                   std::numeric_limits<Scalar>::digits + exponent);
      for (Bits sign : {Bits(0), Bits(Binary::kSignBit)}) {
        const Scalar value = numext::bit_cast<Scalar>(sign | magnitude);
        const Scalar actual = internal::scale_binary_by_power_of_two(value, factor);
        VERIFY_IS_EQUAL(Binary::bits(actual), sign | Binary::bits(expected));
      }
    }
  }
}

// Rounds a subnormal result exactly where hardware arithmetic may flush it: the exact product m * 2^-k denorm_min is
// normal in double, and its nearest integer, ties to even, is the expected magnitude's bit pattern (a carry into the
// exponent field encodes the smallest normal).
template <typename Scalar>
void check_scale_binary_by_power_of_two_rounding() {
  using Binary = internal::binary_floating_point_traits<Scalar>;
  using Bits = typename Binary::Bits;
  // Leading-bit significands in [kExponentUnit, 2 * kExponentUnit): ties and their neighbours at small shifts, and an
  // all-ones tail that carries into the smallest normal.
  const Bits significands[] = {Binary::kExponentUnit,
                               Binary::kExponentUnit | Bits(1),
                               Binary::kExponentUnit | Bits(2),
                               Binary::kExponentUnit | Bits(3),
                               Binary::kExponentUnit | Bits(5),
                               Binary::kExponentUnit + (Binary::kExponentUnit >> 1),
                               (Binary::kExponentUnit << 1) - Bits(1)};
  // A biased input exponent e scales m by 2^(e - 1); the factor 2^-(e - 1 + k) cancels it and shifts by k.
  const int exponentFields[] = {1, 2, Binary::kExponentBias - Binary::kFractionBits - 3};

  ScopedFlushToZero flushToZero;
  for (int field : exponentFields) {
    for (int k = 0; k <= Binary::kFractionBits + 3; ++k) {
      const Scalar factor = numext::ldexp(Scalar(1), -(field - 1 + k));
      for (Bits m : significands) {
        const Bits expected = Bits(std::nearbyint(std::ldexp(double(m), -k)));
        for (Bits sign : {Bits(0), Bits(Binary::kSignBit)}) {
          const Scalar value =
              numext::bit_cast<Scalar>(sign | (Bits(field) << Binary::kFractionBits) | (m & Binary::kFractionMask));
          VERIFY_IS_EQUAL(Binary::bits(internal::scale_binary_by_power_of_two(value, factor)), sign | expected);
        }
      }
    }
  }
  // Subnormal inputs normalize before they round.
  for (Bits magnitude : {Bits(1), Bits(3), Binary::kExponentUnit >> 1, Binary::kFractionMask}) {
    for (int k = 1; k <= Binary::kFractionBits + 2; ++k) {
      const Bits expected = Bits(std::nearbyint(std::ldexp(double(magnitude), -k)));
      for (Bits sign : {Bits(0), Bits(Binary::kSignBit)}) {
        const Scalar actual = internal::scale_binary_by_power_of_two(numext::bit_cast<Scalar>(sign | magnitude),
                                                                     numext::ldexp(Scalar(1), -k));
        VERIFY_IS_EQUAL(Binary::bits(actual), sign | expected);
      }
    }
  }
}

// Under DAZ a comparison reads a subnormal as zero, so compare representations.
template <typename RealScalar>
bool same_bits(const RealScalar& a, const RealScalar& b) {
  using Binary = internal::binary_floating_point_traits<RealScalar>;
  return Binary::bits(a) == Binary::bits(b);
}

template <typename RealScalar>
bool same_bits(const std::complex<RealScalar>& a, const std::complex<RealScalar>& b) {
  return same_bits(a.real(), b.real()) && same_bits(a.imag(), b.imag());
}

template <typename Scalar>
struct scaling_test_value {
  using RealScalar = typename NumTraits<Scalar>::Real;
  static Scalar run(RealScalar real, RealScalar) { return real; }
};

template <typename RealScalar>
struct scaling_test_value<std::complex<RealScalar>> {
  static std::complex<RealScalar> run(RealScalar real, RealScalar imag) { return std::complex<RealScalar>(real, imag); }
};

template <typename Scalar>
void check_arithmetic_scaling_expression() {
  using RealScalar = typename NumTraits<Scalar>::Real;
  using Scaling = internal::safe_scaling<RealScalar, false>;
  using Vector2 = Matrix<Scalar, 2, 1>;
  using Value = scaling_test_value<Scalar>;

  // Lazy scaling and its adjoint must retain the division policy even below the binary-recovery threshold.
  for (const RealScalar maxCoeff :
       {RealScalar(3), numext::ldexp(RealScalar(1.5), std::numeric_limits<RealScalar>::min_exponent + 7)}) {
    Vector2 input;
    input << Value::run(maxCoeff, RealScalar(0)), Value::run(maxCoeff / RealScalar(2), -maxCoeff / RealScalar(4));
    Vector2 expected;
    expected << Value::run(RealScalar(1), RealScalar(0)), Value::run(RealScalar(0.5), RealScalar(-0.25));
    Vector2 materialized;
    const auto factors = Scaling::scale_to(materialized, input, maxCoeff);
    Vector2 lazy;
    Matrix<Scalar, 1, 2> adjoint;
    const auto expressionFactors = Scaling::with_scaled(input, maxCoeff, [&](const auto& expression) {
      lazy = expression;
      adjoint = expression.adjoint();
    });
    VERIFY_IS_EQUAL(materialized, expected);
    VERIFY_IS_EQUAL(lazy, expected);
    VERIFY_IS_EQUAL(adjoint, expected.adjoint());
    VERIFY_IS_EQUAL(expressionFactors.scale, factors.scale);
    VERIFY_IS_EQUAL(expressionFactors.invScale, factors.invScale);
  }
}

template <typename Scalar>
void check_subnormal_preserving_scaling() {
  using RealScalar = typename NumTraits<Scalar>::Real;
  using Binary = internal::binary_floating_point_traits<RealScalar>;
  using Bits = typename Binary::Bits;
  const RealScalar maxCoeff = numext::ldexp((std::numeric_limits<RealScalar>::min)(), 8);
  // Arithmetic or narrowing conversions could flush these inputs before they reach the scaling helper.
  const RealScalar subnormalMaxCoeff = numext::bit_cast<RealScalar>(Bits(64));
  const auto factors = internal::safe_scaling<RealScalar>::compute_floor_factors(maxCoeff);

  Matrix<Scalar, 2, 1> input;
  input(0) = scaling_test_value<Scalar>::run(maxCoeff, -maxCoeff);
  input(1) = scaling_test_value<Scalar>::run(numext::bit_cast<RealScalar>(Binary::kExponentUnit >> 1),
                                             numext::bit_cast<RealScalar>(Binary::kExponentUnit >> 2));
  Matrix<Scalar, 1, 1> subnormalInput;
  subnormalInput(0) = scaling_test_value<Scalar>::run(subnormalMaxCoeff, RealScalar(0));
  ScopedFlushToZero flushToZero;
  Matrix<Scalar, 2, 1> scaled;
  internal::safe_scaling<RealScalar>::scale_to(scaled, input, maxCoeff, factors);

  Matrix<Scalar, 2, 1> scaledExpression;
  const auto expressionFactors = internal::safe_scaling<RealScalar>::with_scaled(
      input, maxCoeff, [&](const auto& expression) { scaledExpression = expression; });

  VERIFY_IS_EQUAL(scaled(0), scaling_test_value<Scalar>::run(RealScalar(1), RealScalar(-1)));
  VERIFY_IS_EQUAL(scaled(1),
                  scaling_test_value<Scalar>::run(RealScalar(1) / RealScalar(512), RealScalar(1) / RealScalar(1024)));
  VERIFY_IS_EQUAL(expressionFactors.scale, factors.scale);
  VERIFY_IS_EQUAL(expressionFactors.invScale, factors.invScale);
  VERIFY_IS_EQUAL(scaledExpression, scaled);

  Matrix<Scalar, 1, 1> scaledSubnormal;
  const auto subnormalFactors =
      internal::safe_scaling<RealScalar>::scale_to(scaledSubnormal, subnormalInput, subnormalMaxCoeff);
  VERIFY(subnormalFactors.scale != RealScalar(1) || subnormalFactors.invScale != RealScalar(1));
  VERIFY(numext::abs(scaledSubnormal(0)) > RealScalar(0));

  // Unscaling restores the subnormal components exactly, still under FTZ.
  internal::safe_scaling<RealScalar>::unscale_in_place(scaled, maxCoeff, factors);
  internal::safe_scaling<RealScalar>::unscale_in_place(scaledSubnormal, subnormalMaxCoeff, subnormalFactors);
  for (Index i = 0; i < input.size(); ++i) VERIFY(same_bits(scaled(i), input(i)));
  VERIFY(same_bits(scaledSubnormal(0), subnormalInput(0)));

  // A NaN, infinite or zero maxCoeff selects identity factors, and unscaling leaves the coefficients alone.
  for (const RealScalar special :
       {std::numeric_limits<RealScalar>::quiet_NaN(), std::numeric_limits<RealScalar>::infinity(), RealScalar(0)}) {
    Matrix<Scalar, 2, 1> unchanged;
    const auto specialFactors = internal::safe_scaling<RealScalar>::scale_to(unchanged, input, special);
    internal::safe_scaling<RealScalar>::unscale_in_place(unchanged, special, specialFactors);
    for (Index i = 0; i < input.size(); ++i) VERIFY(same_bits(unchanged(i), input(i)));
  }
}

// Unscaling rounds through integer significands exactly when 0 < maxCoeff < min / eps. With floor factors, eps / 2
// unscales to min / 2 at the threshold and to min / 4 one ulp below it; FTZ flushes only the former.
template <typename Scalar>
void check_unscale_recovery_threshold() {
  using Binary = internal::binary_floating_point_traits<Scalar>;
  using Scaling = internal::safe_scaling<Scalar>;
  const Scalar threshold = (std::numeric_limits<Scalar>::min)() / NumTraits<Scalar>::epsilon();
  const Scalar belowThreshold = numext::nextafter(threshold, Scalar(0));
  // Read through volatile so the products are formed at run time, under FTZ.
  volatile Scalar halfEpsilon = NumTraits<Scalar>::epsilon() / Scalar(2);
  Matrix<Scalar, 1, 1> atThreshold, below;
  atThreshold(0) = halfEpsilon;
  below(0) = halfEpsilon;

  ScopedFlushToZero flushToZero;
  Scaling::unscale_in_place(below, belowThreshold, Scaling::compute_floor_factors(belowThreshold));
  VERIFY(same_bits(below(0), numext::bit_cast<Scalar>(Binary::kExponentUnit >> 2)));
  Scaling::unscale_in_place(atThreshold, threshold, Scaling::compute_floor_factors(threshold));
  if (flushToZero.isSupported()) VERIFY(same_bits(atThreshold(0), Scalar(0)));
}

// A reduction that flushes subnormal inputs returns zero; the rescan reads the largest real or imaginary magnitude from
// the representation instead, and leaves a nonzero maximum alone.
template <typename Scalar>
void check_recover_flushed_max_coeff() {
  using RealScalar = typename NumTraits<Scalar>::Real;
  using Binary = internal::binary_floating_point_traits<RealScalar>;
  using Bits = typename Binary::Bits;
  using Scaling = internal::safe_scaling<RealScalar>;
  using Value = scaling_test_value<Scalar>;
  const RealScalar small = numext::bit_cast<RealScalar>(Bits(3));
  const RealScalar large = numext::bit_cast<RealScalar>(Binary::kFractionMask);
  Matrix<Scalar, 2, 2> m;
  m << Value::run(small, RealScalar(0)), Value::run(-small, -large), Value::run(RealScalar(0), small),
      Value::run(-small, RealScalar(0));
  const RealScalar expected = NumTraits<Scalar>::IsComplex ? large : small;
  VERIFY(same_bits(Scaling::recover_flushed_max_coeff(m, RealScalar(0)), expected));
  VERIFY(same_bits(Scaling::recover_flushed_max_coeff(m, RealScalar(1)), RealScalar(1)));
  VERIFY(same_bits(Scaling::recover_flushed_max_coeff(Matrix<Scalar, 2, 2>::Zero(), RealScalar(0)), RealScalar(0)));
}

EIGEN_DECLARE_TEST(safe_scaling) {
  CALL_SUBTEST(check_power_of_two_scaling_factor<half>());
  CALL_SUBTEST(check_power_of_two_scaling_factor<bfloat16>());
  CALL_SUBTEST(check_power_of_two_scaling_factor<float>());
  CALL_SUBTEST(check_power_of_two_scaling_factor<double>());
  CALL_SUBTEST(check_power_of_two_scaling_factor<long double>());
  CALL_SUBTEST(check_safe_scaling_special_values<half>());
  CALL_SUBTEST(check_safe_scaling_special_values<bfloat16>());
  CALL_SUBTEST(check_safe_scaling_special_values<float>());
  CALL_SUBTEST(check_safe_scaling_special_values<double>());
  CALL_SUBTEST(check_safe_scaling_special_value_frontends());
  CALL_SUBTEST(check_arithmetic_safe_scaling_fallback());
  CALL_SUBTEST(check_custom_scalar_scaling_exceptions());
  CALL_SUBTEST(check_arithmetic_scaling_expression<float>());
  CALL_SUBTEST(check_arithmetic_scaling_expression<double>());
  CALL_SUBTEST(check_arithmetic_scaling_expression<std::complex<float>>());
  CALL_SUBTEST(check_arithmetic_scaling_expression<std::complex<double>>());
  CALL_SUBTEST(check_scale_binary_by_power_of_two<float>());
  CALL_SUBTEST(check_scale_binary_by_power_of_two<double>());
  CALL_SUBTEST(check_scale_binary_by_power_of_two_rounding<float>());
  CALL_SUBTEST(check_scale_binary_by_power_of_two_rounding<double>());
  CALL_SUBTEST(check_subnormal_preserving_scaling<float>());
  CALL_SUBTEST(check_subnormal_preserving_scaling<std::complex<float>>());
  CALL_SUBTEST(check_subnormal_preserving_scaling<double>());
  CALL_SUBTEST(check_subnormal_preserving_scaling<std::complex<double>>());
  CALL_SUBTEST(check_unscale_recovery_threshold<float>());
  CALL_SUBTEST(check_unscale_recovery_threshold<double>());
  CALL_SUBTEST(check_recover_flushed_max_coeff<float>());
  CALL_SUBTEST(check_recover_flushed_max_coeff<std::complex<float>>());
  CALL_SUBTEST(check_recover_flushed_max_coeff<double>());
  CALL_SUBTEST(check_recover_flushed_max_coeff<std::complex<double>>());
}
