// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2006-2010 Benoit Jacob <jacob.benoit.1@gmail.com>
// Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_MATHFUNCTIONS_H
#define EIGEN_MATHFUNCTIONS_H

// TODO: consider moving these constants to NumTraits.
// Source: WolframAlpha
#define EIGEN_PI 3.141592653589793238462643383279502884197169399375105820974944592307816406L
#define EIGEN_LOG2E 1.442695040888963407359924681001892137426645954152985934135449406931109219L
#define EIGEN_LN2 0.693147180559945309417232121458176568075500134360255254120680009493393621L

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

namespace internal {

/** \internal \class global_math_functions_filtering_base
 *
 * What it does:
 * Defines a typedef 'type' as follows:
 * - if type T has a member typedef Eigen_BaseClassForSpecializationOfGlobalMathFuncImpl, then
 *   global_math_functions_filtering_base<T>::type is a typedef for it.
 * - otherwise, global_math_functions_filtering_base<T>::type is a typedef for T.
 *
 * How it's used:
 * To allow to defined the global math functions (like sin...) in certain cases, like the Array expressions.
 * When you do sin(array1+array2), the object array1+array2 has a complicated expression type, all what you want to know
 * is that it inherits ArrayBase. So we implement a partial specialization of sin_impl for ArrayBase<Derived>.
 * So we must make sure to use sin_impl<ArrayBase<Derived> > and not sin_impl<Derived>, otherwise our partial
 * specialization won't be used. How does sin know that? That's exactly what global_math_functions_filtering_base tells
 * it.
 *
 * How it's implemented:
 * SFINAE in the style of enable_if. Highly susceptible of breaking compilers. With GCC, it sure does work, but if you
 * replace the typename dummy by an integer template parameter, it doesn't work anymore!
 */

template <typename T, typename dummy = void>
struct global_math_functions_filtering_base {
  using type = T;
};

template <typename T>
struct global_math_functions_filtering_base<T,
                                            void_t<typename T::Eigen_BaseClassForSpecializationOfGlobalMathFuncImpl>> {
  using type = typename T::Eigen_BaseClassForSpecializationOfGlobalMathFuncImpl;
};

#define EIGEN_MATHFUNC_IMPL(func, scalar) \
  Eigen::internal::func##_impl<typename Eigen::internal::global_math_functions_filtering_base<scalar>::type>

/****************************************************************************
 * Implementation of real                                                 *
 ****************************************************************************/

template <typename Scalar, bool IsComplex = NumTraits<Scalar>::IsComplex>
struct real_default_impl {
  using RealScalar = typename NumTraits<Scalar>::Real;
  EIGEN_DEVICE_FUNC static constexpr RealScalar run(const Scalar& x) { return x; }
};

template <typename Scalar>
struct real_default_impl<Scalar, true> {
  using RealScalar = typename NumTraits<Scalar>::Real;
  EIGEN_DEVICE_FUNC static inline RealScalar run(const Scalar& x) {
    using std::real;
    return real(x);
  }
};

template <typename Scalar>
struct real_impl : real_default_impl<Scalar> {};

#if defined(EIGEN_GPU_COMPILE_PHASE)
template <typename T>
struct real_impl<std::complex<T>> {
  using RealScalar = T;
  EIGEN_DEVICE_FUNC static inline T run(const std::complex<T>& x) { return x.real(); }
};
#endif

/****************************************************************************
 * Implementation of imag                                                 *
 ****************************************************************************/

template <typename Scalar, bool IsComplex = NumTraits<Scalar>::IsComplex>
struct imag_default_impl {
  using RealScalar = typename NumTraits<Scalar>::Real;
  EIGEN_DEVICE_FUNC static inline RealScalar run(const Scalar&) { return RealScalar(0); }
};

template <typename Scalar>
struct imag_default_impl<Scalar, true> {
  using RealScalar = typename NumTraits<Scalar>::Real;
  EIGEN_DEVICE_FUNC static inline RealScalar run(const Scalar& x) {
    using std::imag;
    return imag(x);
  }
};

template <typename Scalar>
struct imag_impl : imag_default_impl<Scalar> {};

#if defined(EIGEN_GPU_COMPILE_PHASE)
template <typename T>
struct imag_impl<std::complex<T>> {
  using RealScalar = T;
  EIGEN_DEVICE_FUNC static inline T run(const std::complex<T>& x) { return x.imag(); }
};
#endif

/****************************************************************************
 * Implementation of real_ref                                             *
 ****************************************************************************/

template <typename Scalar>
struct real_ref_impl {
  using RealScalar = typename NumTraits<Scalar>::Real;
  EIGEN_DEVICE_FUNC static inline RealScalar& run(Scalar& x) { return reinterpret_cast<RealScalar*>(&x)[0]; }
  EIGEN_DEVICE_FUNC static inline const RealScalar& run(const Scalar& x) {
    return reinterpret_cast<const RealScalar*>(&x)[0];
  }
};

/****************************************************************************
 * Implementation of imag_ref                                             *
 ****************************************************************************/

template <typename Scalar, bool IsComplex>
struct imag_ref_default_impl {
  using RealScalar = typename NumTraits<Scalar>::Real;
  EIGEN_DEVICE_FUNC static inline RealScalar& run(Scalar& x) { return reinterpret_cast<RealScalar*>(&x)[1]; }
  EIGEN_DEVICE_FUNC static inline const RealScalar& run(const Scalar& x) {
    return reinterpret_cast<const RealScalar*>(&x)[1];
  }
};

template <typename Scalar>
struct imag_ref_default_impl<Scalar, false> {
  using RealScalar = typename NumTraits<Scalar>::Real;
  EIGEN_DEVICE_FUNC constexpr static inline RealScalar run(Scalar&) { return RealScalar(0); }
  EIGEN_DEVICE_FUNC constexpr static inline RealScalar run(const Scalar&) { return RealScalar(0); }
};

template <typename Scalar>
struct imag_ref_impl : imag_ref_default_impl<Scalar, NumTraits<Scalar>::IsComplex> {};

}  // namespace internal

namespace numext {

template <typename Scalar>
EIGEN_DEVICE_FUNC inline decltype(auto) real(const Scalar& x) {
  return EIGEN_MATHFUNC_IMPL(real, Scalar)::run(x);
}

template <typename Scalar>
EIGEN_DEVICE_FUNC inline auto real_ref(const Scalar& x)
    -> internal::add_const_on_value_type_t<decltype(internal::real_ref_impl<Scalar>::run(x))> {
  return internal::real_ref_impl<Scalar>::run(x);
}

template <typename Scalar>
EIGEN_DEVICE_FUNC inline auto real_ref(Scalar& x)
    -> decltype(EIGEN_MATHFUNC_IMPL(real_ref, std::remove_const_t<Scalar>)::run(x)) {
  return EIGEN_MATHFUNC_IMPL(real_ref, std::remove_const_t<Scalar>)::run(x);
}

template <typename Scalar>
EIGEN_DEVICE_FUNC inline decltype(auto) imag(const Scalar& x) {
  return EIGEN_MATHFUNC_IMPL(imag, Scalar)::run(x);
}

template <typename Scalar>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar select(const Scalar& mask, const Scalar& a, const Scalar& b) {
  return numext::is_exactly_zero(mask) ? b : a;
}

}  // namespace numext

namespace internal {

/****************************************************************************
 * Implementation of conj                                                 *
 ****************************************************************************/

template <typename Scalar, bool IsComplex = NumTraits<Scalar>::IsComplex>
struct conj_default_impl {
  EIGEN_DEVICE_FUNC static constexpr Scalar run(const Scalar& x) { return x; }
};

template <typename Scalar>
struct conj_default_impl<Scalar, true> {
  EIGEN_DEVICE_FUNC static inline Scalar run(const Scalar& x) {
    using std::conj;
    return conj(x);
  }
};

template <typename Scalar, bool IsComplex = NumTraits<Scalar>::IsComplex>
struct conj_impl : conj_default_impl<Scalar, IsComplex> {};

/****************************************************************************
 * Implementation of mul                                                  *
 ****************************************************************************/

// Unsigned operands narrower than int promote to int, where a product whose wrapped value is representable
// can still overflow: (unsigned short)0xffff squared is 0xfffe0001 > INT_MAX. Multiplying in unsigned int,
// wider than every such operand, wraps instead. Mixed operand types, signed types, and bool keep operator*.
template <typename Lhs, typename Rhs,
          bool NarrowUnsigned = std::is_same<Lhs, Rhs>::value && std::is_unsigned<Lhs>::value &&
                                !std::is_same<Lhs, bool>::value && (sizeof(Lhs) < sizeof(int))>
struct mul_impl {
  EIGEN_DEVICE_FUNC static constexpr EIGEN_ALWAYS_INLINE auto run(const Lhs& a, const Rhs& b) { return a * b; }
};

template <typename Scalar>
struct mul_impl<Scalar, Scalar, true> {
  EIGEN_DEVICE_FUNC static constexpr EIGEN_ALWAYS_INLINE Scalar run(const Scalar& a, const Scalar& b) {
    return static_cast<Scalar>(static_cast<unsigned int>(a) * static_cast<unsigned int>(b));
  }
};

/** \internal \returns a * b, wrapping rather than overflowing the signed type that integral promotion
 * would give narrow unsigned operands. The result keeps operator*'s type, which for an expression-template
 * Scalar is a lazy expression: consume it rather than binding it to auto. */
template <typename Lhs, typename Rhs>
EIGEN_DEVICE_FUNC constexpr EIGEN_ALWAYS_INLINE auto mul(const Lhs& a, const Rhs& b) {
  return mul_impl<Lhs, Rhs>::run(a, b);
}

/****************************************************************************
 * Implementation of abs2                                                 *
 ****************************************************************************/

template <typename Scalar, bool IsComplex>
struct abs2_impl_default {
  using RealScalar = typename NumTraits<Scalar>::Real;
  EIGEN_DEVICE_FUNC static inline RealScalar run(const Scalar& x) { return internal::mul(x, x); }
};

template <typename Scalar>
struct abs2_impl_default<Scalar, true>  // IsComplex
{
  using RealScalar = typename NumTraits<Scalar>::Real;
  EIGEN_DEVICE_FUNC static inline RealScalar run(const Scalar& x) {
    return numext::real(x) * numext::real(x) + numext::imag(x) * numext::imag(x);
  }
};

template <typename Scalar>
struct abs2_impl {
  using RealScalar = typename NumTraits<Scalar>::Real;
  EIGEN_DEVICE_FUNC static inline RealScalar run(const Scalar& x) {
    return abs2_impl_default<Scalar, NumTraits<Scalar>::IsComplex>::run(x);
  }
};

/****************************************************************************
 * Implementation of sqrt/rsqrt                                             *
 ****************************************************************************/

template <typename Scalar>
struct sqrt_impl {
  EIGEN_DEVICE_FUNC static EIGEN_ALWAYS_INLINE Scalar run(const Scalar& x) {
    EIGEN_USING_STD(sqrt);
    return sqrt(x);
  }
};

// Complex sqrt defined in MathFunctionsImpl.h.
template <typename ComplexT>
EIGEN_DEVICE_FUNC constexpr ComplexT complex_sqrt(const ComplexT& a_x);

// Custom implementation is faster than `std::sqrt`, works on
// GPU, and correctly handles special cases (unlike MSVC).
template <typename T>
struct sqrt_impl<std::complex<T>> {
  EIGEN_DEVICE_FUNC static EIGEN_ALWAYS_INLINE std::complex<T> run(const std::complex<T>& x) { return complex_sqrt(x); }
};

// Default implementation relies on numext::sqrt, at bottom of file.
template <typename T>
struct rsqrt_impl;

// Complex rsqrt defined in MathFunctionsImpl.h.
template <typename ComplexT>
EIGEN_DEVICE_FUNC constexpr ComplexT complex_rsqrt(const ComplexT& a_x);

template <typename T>
struct rsqrt_impl<std::complex<T>> {
  EIGEN_DEVICE_FUNC static EIGEN_ALWAYS_INLINE std::complex<T> run(const std::complex<T>& x) {
    return complex_rsqrt(x);
  }
};

/****************************************************************************
 * Implementation of norm1                                                *
 ****************************************************************************/

template <typename Scalar, bool IsComplex>
struct norm1_default_impl;

template <typename Scalar>
struct norm1_default_impl<Scalar, true> {
  using RealScalar = typename NumTraits<Scalar>::Real;
  EIGEN_DEVICE_FUNC static inline RealScalar run(const Scalar& x) {
    EIGEN_USING_STD(abs);
    return abs(numext::real(x)) + abs(numext::imag(x));
  }
};

template <typename Scalar>
struct norm1_default_impl<Scalar, false> {
  using RealScalar = typename NumTraits<Scalar>::Real;
  EIGEN_DEVICE_FUNC static inline RealScalar run(const Scalar& x) {
    EIGEN_USING_STD(abs);
    return abs(x);
  }
};

template <typename Scalar>
struct norm1_impl : norm1_default_impl<Scalar, NumTraits<Scalar>::IsComplex> {};

/****************************************************************************
 * Implementation of hypot                                                *
 ****************************************************************************/

template <typename Scalar>
struct hypot_impl;

/****************************************************************************
 * Implementation of cast                                                 *
 ****************************************************************************/

template <typename OldType, typename NewType, typename EnableIf = void>
struct cast_impl {
  EIGEN_DEVICE_FUNC static inline NewType run(const OldType& x) { return static_cast<NewType>(x); }
};

template <typename OldType>
struct cast_impl<OldType, bool> {
  EIGEN_DEVICE_FUNC static inline bool run(const OldType& x) { return x != OldType(0); }
};

// Casting from S -> Complex<T> leads to an implicit conversion from S to T,
// generating warnings on clang.  Here we explicitly cast the real component.
template <typename OldType, typename NewType>
struct cast_impl<OldType, NewType, std::enable_if_t<!NumTraits<OldType>::IsComplex && NumTraits<NewType>::IsComplex>> {
  EIGEN_DEVICE_FUNC static inline NewType run(const OldType& x) {
    using NewReal = typename NumTraits<NewType>::Real;
    return static_cast<NewType>(static_cast<NewReal>(x));
  }
};

// Returns NewType directly to avoid unintended intermediate conversions.

template <typename OldType, typename NewType>
EIGEN_DEVICE_FUNC inline NewType cast(const OldType& x) {
  return cast_impl<OldType, NewType>::run(x);
}

/****************************************************************************
 * Implementation of arg                                                     *
 ****************************************************************************/

// Visual Studio 2017 has a bug where arg(float) returns 0 for negative inputs.
// This seems to be fixed in VS 2019.
#if (!EIGEN_COMP_MSVC || EIGEN_COMP_MSVC >= 1920)
// std::arg is only defined for types of std::complex, or integer types or float/double/long double
template <typename Scalar, bool HasStdImpl = NumTraits<Scalar>::IsComplex || std::is_integral<Scalar>::value ||
                                             std::is_same<Scalar, float>::value ||
                                             std::is_same<Scalar, double>::value ||
                                             std::is_same<Scalar, long double>::value>
struct arg_default_impl;

template <typename Scalar>
struct arg_default_impl<Scalar, true> {
  using RealScalar = typename NumTraits<Scalar>::Real;
  EIGEN_DEVICE_FUNC static inline RealScalar run(const Scalar& x) {
    // There is no official ::arg on device in CUDA/HIP, so we always need to use std::arg.
    using std::arg;
    return static_cast<RealScalar>(arg(x));
  }
};

// Must be non-complex floating-point type (e.g. half/bfloat16).
template <typename Scalar>
struct arg_default_impl<Scalar, false> {
  using RealScalar = typename NumTraits<Scalar>::Real;
  EIGEN_DEVICE_FUNC static inline RealScalar run(const Scalar& x) {
    return (x < Scalar(0)) ? RealScalar(EIGEN_PI) : RealScalar(0);
  }
};
#else
template <typename Scalar, bool IsComplex = NumTraits<Scalar>::IsComplex>
struct arg_default_impl {
  using RealScalar = typename NumTraits<Scalar>::Real;
  EIGEN_DEVICE_FUNC static inline RealScalar run(const Scalar& x) {
    return (x < RealScalar(0)) ? RealScalar(EIGEN_PI) : RealScalar(0);
  }
};

template <typename Scalar>
struct arg_default_impl<Scalar, true> {
  using RealScalar = typename NumTraits<Scalar>::Real;
  EIGEN_DEVICE_FUNC static inline RealScalar run(const Scalar& x) {
    EIGEN_USING_STD(arg);
    return arg(x);
  }
};
#endif
template <typename Scalar>
struct arg_impl : arg_default_impl<Scalar> {};

/****************************************************************************
 * Implementation of expm1                                                   *
 ****************************************************************************/

// This implementation is based on GSL Math's expm1.
namespace std_fallback {
// fallback expm1 implementation in case there is no expm1(Scalar) function in namespace of Scalar,
// or that there is no suitable std::expm1 function available. Implementation
// attributed to Kahan. See: http://www.plunk.org/~hatch/rightway.php.
template <typename Scalar>
EIGEN_DEVICE_FUNC inline Scalar expm1(const Scalar& x) {
  EIGEN_STATIC_ASSERT_NON_INTEGER(Scalar)
  using RealScalar = typename NumTraits<Scalar>::Real;

  EIGEN_USING_STD(exp);
  Scalar u = exp(x);
  if (numext::equal_strict(u, Scalar(1))) {
    return x;
  }
  Scalar um1 = u - RealScalar(1);
  if (numext::equal_strict(um1, Scalar(-1))) {
    return RealScalar(-1);
  }

  EIGEN_USING_STD(log);
  Scalar logu = log(u);
  return numext::equal_strict(u, logu) ? u : (u - RealScalar(1)) * x / logu;
}
}  // namespace std_fallback

template <typename Scalar>
struct expm1_impl {
  EIGEN_DEVICE_FUNC static inline Scalar run(const Scalar& x) {
    EIGEN_STATIC_ASSERT_NON_INTEGER(Scalar)
    EIGEN_USING_STD(expm1);
    return expm1(x);
  }
};

/****************************************************************************
 * Implementation of log                                                     *
 ****************************************************************************/

// Complex log defined in MathFunctionsImpl.h.
template <typename ComplexT>
EIGEN_DEVICE_FUNC constexpr ComplexT complex_log(const ComplexT& z);

template <typename Scalar>
struct log_impl {
  EIGEN_DEVICE_FUNC static inline Scalar run(const Scalar& x) {
    EIGEN_USING_STD(log);
    return static_cast<Scalar>(log(x));
  }
};

template <typename Scalar>
struct log_impl<std::complex<Scalar>> {
  EIGEN_DEVICE_FUNC static inline std::complex<Scalar> run(const std::complex<Scalar>& z) { return complex_log(z); }
};

/****************************************************************************
 * Implementation of log1p                                                   *
 ****************************************************************************/

namespace std_fallback {
// fallback log1p implementation in case there is no log1p(Scalar) function in namespace of Scalar,
// or that there is no suitable std::log1p function available
template <typename Scalar>
EIGEN_DEVICE_FUNC inline Scalar log1p(const Scalar& x) {
  EIGEN_STATIC_ASSERT_NON_INTEGER(Scalar)
  using RealScalar = typename NumTraits<Scalar>::Real;
  EIGEN_USING_STD(log);
  Scalar x1p = RealScalar(1) + x;
  Scalar log_1p = log_impl<Scalar>::run(x1p);
  const bool is_small = numext::equal_strict(x1p, Scalar(1));
  const bool is_inf = numext::equal_strict(x1p, log_1p);
  return (is_small || is_inf) ? x : x * (log_1p / (x1p - RealScalar(1)));
}
}  // namespace std_fallback

template <typename Scalar>
struct log1p_impl {
  EIGEN_STATIC_ASSERT_NON_INTEGER(Scalar)

  EIGEN_DEVICE_FUNC static inline Scalar run(const Scalar& x) {
    EIGEN_USING_STD(log1p);
    return log1p(x);
  }
};

// Specialization for complex types that are not supported by std::log1p.
template <typename RealScalar>
struct log1p_impl<std::complex<RealScalar>> {
  EIGEN_STATIC_ASSERT_NON_INTEGER(RealScalar)

  EIGEN_DEVICE_FUNC static inline std::complex<RealScalar> run(const std::complex<RealScalar>& x) {
    return std_fallback::log1p(x);
  }
};

/****************************************************************************
 * Implementation of pow                                                  *
 ****************************************************************************/

template <typename ScalarX, typename ScalarY,
          bool IsInteger = NumTraits<ScalarX>::IsInteger && NumTraits<ScalarY>::IsInteger>
struct pow_impl {
  using result_type =
      typename ScalarBinaryOpTraits<ScalarX, ScalarY, internal::scalar_pow_op<ScalarX, ScalarY>>::ReturnType;
  static EIGEN_DEVICE_FUNC inline result_type run(const ScalarX& x, const ScalarY& y) {
    EIGEN_USING_STD(pow);
    return pow(x, y);
  }
};

template <typename ScalarX, typename ScalarY>
struct pow_impl<ScalarX, ScalarY, true> {
  using result_type = ScalarX;
  static EIGEN_DEVICE_FUNC inline ScalarX run(ScalarX x, ScalarY y) {
    ScalarX res(1);
    eigen_assert(!NumTraits<ScalarY>::IsSigned || y >= 0);
    if (y & 1) res = internal::mul(res, x);
    y >>= 1;
    while (y) {
      x = internal::mul(x, x);
      if (y & 1) res = internal::mul(res, x);
      y >>= 1;
    }
    return res;
  }
};

constexpr int floor_log2(unsigned int value) {
  int result = 0;
  while (value > 1) {
    value >>= 1;
    ++result;
  }
  return result;
}

template <typename BitsType, typename EnableIf = void>
struct count_bits_impl {
  static_assert(std::is_integral<BitsType>::value && std::is_unsigned<BitsType>::value,
                "BitsType must be an unsigned integer");
  static EIGEN_DEVICE_FUNC inline int clz(BitsType bits) {
    int n = CHAR_BIT * sizeof(BitsType);
    int shift = n / 2;
    while (bits > 0 && shift > 0) {
      BitsType y = bits >> shift;
      if (y > 0) {
        n -= shift;
        bits = y;
      }
      shift /= 2;
    }
    if (shift == 0) {
      --n;
    }
    return n;
  }

  static EIGEN_DEVICE_FUNC inline int ctz(BitsType bits) {
    int n = CHAR_BIT * sizeof(BitsType);
    int shift = n / 2;
    while (bits > 0 && shift > 0) {
      BitsType y = bits << shift;
      if (y > 0) {
        n -= shift;
        bits = y;
      }
      shift /= 2;
    }
    if (shift == 0) {
      --n;
    }
    return n;
  }

  static EIGEN_DEVICE_FUNC inline int popcount(BitsType bits) {
    int n = 0;
    while (bits) {
      bits &= bits - 1;
      ++n;
    }
    return n;
  }
};

// Count leading zeros.
template <typename BitsType>
EIGEN_DEVICE_FUNC inline int clz(BitsType bits) {
  return count_bits_impl<BitsType>::clz(bits);
}

// Count trailing zeros.
template <typename BitsType>
EIGEN_DEVICE_FUNC inline int ctz(BitsType bits) {
  return count_bits_impl<BitsType>::ctz(bits);
}

// Count set bits (population count).
template <typename BitsType>
EIGEN_DEVICE_FUNC inline int popcount(BitsType bits) {
  return count_bits_impl<BitsType>::popcount(bits);
}

#if EIGEN_COMP_GNUC || EIGEN_COMP_CLANG

template <typename BitsType>
struct count_bits_impl<
    BitsType, std::enable_if_t<std::is_integral<BitsType>::value && sizeof(BitsType) <= sizeof(unsigned int)>> {
  static constexpr int kNumBits = static_cast<int>(sizeof(BitsType) * CHAR_BIT);
  static EIGEN_DEVICE_FUNC inline int clz(BitsType bits) {
    static constexpr int kLeadingBitsOffset = (sizeof(unsigned int) - sizeof(BitsType)) * CHAR_BIT;
    return bits == 0 ? kNumBits : __builtin_clz(static_cast<unsigned int>(bits)) - kLeadingBitsOffset;
  }

  static EIGEN_DEVICE_FUNC inline int ctz(BitsType bits) {
    return bits == 0 ? kNumBits : __builtin_ctz(static_cast<unsigned int>(bits));
  }

  static EIGEN_DEVICE_FUNC inline int popcount(BitsType bits) {
    return __builtin_popcount(static_cast<unsigned int>(bits));
  }
};

template <typename BitsType>
struct count_bits_impl<BitsType,
                       std::enable_if_t<std::is_integral<BitsType>::value && sizeof(unsigned int) < sizeof(BitsType) &&
                                        sizeof(BitsType) <= sizeof(unsigned long)>> {
  static constexpr int kNumBits = static_cast<int>(sizeof(BitsType) * CHAR_BIT);
  static EIGEN_DEVICE_FUNC inline int clz(BitsType bits) {
    static constexpr int kLeadingBitsOffset = (sizeof(unsigned long) - sizeof(BitsType)) * CHAR_BIT;
    return bits == 0 ? kNumBits : __builtin_clzl(static_cast<unsigned long>(bits)) - kLeadingBitsOffset;
  }

  static EIGEN_DEVICE_FUNC inline int ctz(BitsType bits) {
    return bits == 0 ? kNumBits : __builtin_ctzl(static_cast<unsigned long>(bits));
  }

  static EIGEN_DEVICE_FUNC inline int popcount(BitsType bits) {
    return __builtin_popcountl(static_cast<unsigned long>(bits));
  }
};

template <typename BitsType>
struct count_bits_impl<BitsType,
                       std::enable_if_t<std::is_integral<BitsType>::value && sizeof(unsigned long) < sizeof(BitsType) &&
                                        sizeof(BitsType) <= sizeof(unsigned long long)>> {
  static constexpr int kNumBits = static_cast<int>(sizeof(BitsType) * CHAR_BIT);
  static EIGEN_DEVICE_FUNC inline int clz(BitsType bits) {
    static constexpr int kLeadingBitsOffset = (sizeof(unsigned long long) - sizeof(BitsType)) * CHAR_BIT;
    return bits == 0 ? kNumBits : __builtin_clzll(static_cast<unsigned long long>(bits)) - kLeadingBitsOffset;
  }

  static EIGEN_DEVICE_FUNC inline int ctz(BitsType bits) {
    return bits == 0 ? kNumBits : __builtin_ctzll(static_cast<unsigned long long>(bits));
  }

  static EIGEN_DEVICE_FUNC inline int popcount(BitsType bits) {
    return __builtin_popcountll(static_cast<unsigned long long>(bits));
  }
};

#elif EIGEN_COMP_MSVC

// `__popcnt`/`__popcnt64` require `POPCNT` hardware support, which MSVC cannot guarantee
// at its default baseline (unlike `_BitScanReverse`/`_BitScanForward`, which lower to
// baseline `bsr`/`bsf`).  Fall back to a portable count when building without SSE4.2
// enabled.
template <typename BitsType>
EIGEN_DEVICE_FUNC inline int popcount_fallback(BitsType bits) {
  int n = 0;
  while (bits) {
    bits &= bits - 1;
    ++n;
  }
  return n;
}

template <typename BitsType>
struct count_bits_impl<
    BitsType, std::enable_if_t<std::is_integral<BitsType>::value && sizeof(BitsType) <= sizeof(unsigned long)>> {
  static constexpr int kNumBits = static_cast<int>(sizeof(BitsType) * CHAR_BIT);
  static EIGEN_DEVICE_FUNC inline int clz(BitsType bits) {
    unsigned long out;
    _BitScanReverse(&out, static_cast<unsigned long>(bits));
    return bits == 0 ? kNumBits : (kNumBits - 1) - static_cast<int>(out);
  }

  static EIGEN_DEVICE_FUNC inline int ctz(BitsType bits) {
    unsigned long out;
    _BitScanForward(&out, static_cast<unsigned long>(bits));
    return bits == 0 ? kNumBits : static_cast<int>(out);
  }

  static EIGEN_DEVICE_FUNC inline int popcount(BitsType bits) {
#if defined(EIGEN_VECTORIZE_SSE4_2)
    return static_cast<int>(__popcnt(static_cast<unsigned int>(bits)));
#else
    return popcount_fallback(bits);
#endif
  }
};

#ifdef _WIN64

template <typename BitsType>
struct count_bits_impl<BitsType,
                       std::enable_if_t<std::is_integral<BitsType>::value && sizeof(unsigned long) < sizeof(BitsType) &&
                                        sizeof(BitsType) <= sizeof(__int64)>> {
  static constexpr int kNumBits = static_cast<int>(sizeof(BitsType) * CHAR_BIT);
  static EIGEN_DEVICE_FUNC inline int clz(BitsType bits) {
    unsigned long out;
    _BitScanReverse64(&out, static_cast<unsigned __int64>(bits));
    return bits == 0 ? kNumBits : (kNumBits - 1) - static_cast<int>(out);
  }

  static EIGEN_DEVICE_FUNC inline int ctz(BitsType bits) {
    unsigned long out;
    _BitScanForward64(&out, static_cast<unsigned __int64>(bits));
    return bits == 0 ? kNumBits : static_cast<int>(out);
  }

  static EIGEN_DEVICE_FUNC inline int popcount(BitsType bits) {
#if defined(EIGEN_VECTORIZE_SSE4_2)
    return static_cast<int>(__popcnt64(static_cast<unsigned __int64>(bits)));
#else
    return popcount_fallback(bits);
#endif
  }
};

#endif  // _WIN64

#endif  // EIGEN_COMP_GNUC || EIGEN_COMP_CLANG

template <typename BitsType>
struct log_2_impl {
  static constexpr int kTotalBits = sizeof(BitsType) * CHAR_BIT;
  static EIGEN_DEVICE_FUNC inline int run_ceil(const BitsType& x) {
    const int n = kTotalBits - clz(x);
    bool power_of_two = (x & (x - 1)) == 0;
    return x == 0 ? 0 : power_of_two ? (n - 1) : n;
  }
  static EIGEN_DEVICE_FUNC inline int run_floor(const BitsType& x) {
    const int n = kTotalBits - clz(x);
    return x == 0 ? 0 : n - 1;
  }
};

template <typename BitsType>
int log2_ceil(const BitsType& x) {
  return log_2_impl<BitsType>::run_ceil(x);
}

template <typename BitsType>
int log2_floor(const BitsType& x) {
  return log_2_impl<BitsType>::run_floor(x);
}

// Implementation of is* functions

template <typename T>
EIGEN_DEVICE_FUNC std::enable_if_t<!(std::numeric_limits<T>::has_infinity || std::numeric_limits<T>::has_quiet_NaN ||
                                     std::numeric_limits<T>::has_signaling_NaN),
                                   bool>
isfinite_impl(const T&) {
  return true;
}

template <typename T>
EIGEN_DEVICE_FUNC std::enable_if_t<(std::numeric_limits<T>::has_infinity || std::numeric_limits<T>::has_quiet_NaN ||
                                    std::numeric_limits<T>::has_signaling_NaN) &&
                                       (!NumTraits<T>::IsComplex),
                                   bool>
isfinite_impl(const T& x) {
  EIGEN_USING_STD(isfinite);
  return isfinite EIGEN_NOT_A_MACRO(x);
}

template <typename T>
EIGEN_DEVICE_FUNC std::enable_if_t<!std::numeric_limits<T>::has_infinity, bool> isinf_impl(const T&) {
  return false;
}

template <typename T>
EIGEN_DEVICE_FUNC std::enable_if_t<(std::numeric_limits<T>::has_infinity && !NumTraits<T>::IsComplex), bool> isinf_impl(
    const T& x) {
  EIGEN_USING_STD(isinf);
  return isinf EIGEN_NOT_A_MACRO(x);
}

template <typename T>
EIGEN_DEVICE_FUNC
    std::enable_if_t<!(std::numeric_limits<T>::has_quiet_NaN || std::numeric_limits<T>::has_signaling_NaN), bool>
    isnan_impl(const T&) {
  return false;
}

template <typename T>
EIGEN_DEVICE_FUNC std::enable_if_t<
    (std::numeric_limits<T>::has_quiet_NaN || std::numeric_limits<T>::has_signaling_NaN) && (!NumTraits<T>::IsComplex),
    bool>
isnan_impl(const T& x) {
  EIGEN_USING_STD(isnan);
  return isnan EIGEN_NOT_A_MACRO(x);
}

// The following overload are defined at the end of this file
template <typename T>
EIGEN_DEVICE_FUNC bool isfinite_impl(const std::complex<T>& x);
template <typename T>
EIGEN_DEVICE_FUNC bool isnan_impl(const std::complex<T>& x);
template <typename T>
EIGEN_DEVICE_FUNC bool isinf_impl(const std::complex<T>& x);
template <typename T>
EIGEN_DEFINE_FUNCTION_ALLOWING_MULTIPLE_DEFINITIONS T ptanh_float(const T& a_x);

/****************************************************************************
 * Implementation of sign                                                 *
 ****************************************************************************/
template <typename Scalar, bool IsComplex = (NumTraits<Scalar>::IsComplex != 0),
          bool IsInteger = (NumTraits<Scalar>::IsInteger != 0)>
struct sign_impl {
  EIGEN_DEVICE_FUNC static inline Scalar run(const Scalar& a) { return Scalar((a > Scalar(0)) - (a < Scalar(0))); }
};

template <typename Scalar>
struct sign_impl<Scalar, false, false> {
  EIGEN_DEVICE_FUNC static inline Scalar run(const Scalar& a) {
    return (isnan_impl<Scalar>)(a) ? a : Scalar((a > Scalar(0)) - (a < Scalar(0)));
  }
};

template <typename Scalar, bool IsInteger>
struct sign_impl<Scalar, true, IsInteger> {
  EIGEN_DEVICE_FUNC static inline Scalar run(const Scalar& a) {
    using real_type = typename NumTraits<Scalar>::Real;
    EIGEN_USING_STD(abs);
    real_type aa = abs(a);
    if (aa == real_type(0)) return Scalar(0);
    // Divide rather than multiply by 1/aa: the reciprocal overflows for subnormal aa and is itself
    // subnormal, hence inexact, for aa near the top of the range. psign_impl for complex packets
    // divides for the same reason.
    return Scalar(numext::real(a) / aa, numext::imag(a) / aa);
  }
};

// The sign function for bool is the identity.
template <>
struct sign_impl<bool, false, true> {
  EIGEN_DEVICE_FUNC static inline bool run(const bool& a) { return a; }
};

template <typename Scalar, bool IsComplex = (NumTraits<Scalar>::IsComplex != 0),
          bool IsInteger = (NumTraits<Scalar>::IsInteger != 0)>
struct copysign_impl {
  EIGEN_DEVICE_FUNC static inline Scalar run(const Scalar& a, const Scalar& b) {
    EIGEN_USING_STD(copysign);
    return Scalar(copysign(a, b));
  }
};

template <typename Scalar, bool IsInteger>
struct copysign_impl<Scalar, true, IsInteger> {
  EIGEN_DEVICE_FUNC static inline Scalar run(const Scalar& a, const Scalar& b) {
    EIGEN_USING_STD(copysign);
    return Scalar(copysign(numext::real(a), numext::real(b)), copysign(numext::imag(a), numext::imag(b)));
  }
};

template <typename Scalar>
struct copysign_impl<Scalar, false, true> {
  EIGEN_DEVICE_FUNC static inline Scalar run(const Scalar& a, const Scalar& b) {
    EIGEN_IF_CONSTEXPR (!NumTraits<Scalar>::IsSigned) return a;
    const Scalar abs_a = a < Scalar(0) ? -a : a;
    return b < Scalar(0) ? -abs_a : abs_a;
  }
};

template <>
struct copysign_impl<bool, false, true> {
  EIGEN_DEVICE_FUNC static inline bool run(const bool& a, const bool&) { return a; }
};

// suppress "unary minus operator applied to unsigned type, result still unsigned" warnings on MSVC
// note: `0 - a` is distinct from `-a` when Scalar is a floating point type and `a` is zero

template <typename Scalar, bool IsInteger = NumTraits<Scalar>::IsInteger>
struct negate_impl {
  static EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE Scalar run(const Scalar& a) { return -a; }
};

template <typename Scalar>
struct negate_impl<Scalar, true> {
  EIGEN_STATIC_ASSERT((!std::is_same<Scalar, bool>::value), NEGATE IS NOT DEFINED FOR BOOLEAN TYPES)
  static EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE Scalar run(const Scalar& a) { return Scalar(0) - a; }
};

template <typename Scalar, bool IsInteger = NumTraits<typename unpacket_traits<Scalar>::type>::IsInteger>
struct nearest_integer_impl {
  static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar run_floor(const Scalar& x) {
    EIGEN_USING_STD(floor) return floor(x);
  }
  static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar run_ceil(const Scalar& x) {
    EIGEN_USING_STD(ceil) return ceil(x);
  }
  static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar run_rint(const Scalar& x) {
    EIGEN_USING_STD(rint) return rint(x);
  }
  static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar run_round(const Scalar& x) {
    EIGEN_USING_STD(round) return round(x);
  }
  static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar run_trunc(const Scalar& x) {
    EIGEN_USING_STD(trunc) return trunc(x);
  }
};
template <typename Scalar>
struct nearest_integer_impl<Scalar, true> {
  static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar run_floor(const Scalar& x) { return x; }
  static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar run_ceil(const Scalar& x) { return x; }
  static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar run_rint(const Scalar& x) { return x; }
  static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar run_round(const Scalar& x) { return x; }
  static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar run_trunc(const Scalar& x) { return x; }
};

// Extra namespace to prevent leaking std::fma into Eigen::internal.
namespace has_fma_detail {

template <typename T, typename EnableIf = void>
struct has_fma_impl : public std::false_type {};

using std::fma;

template <typename T>
struct has_fma_impl<
    T, std::enable_if_t<std::is_same<T, decltype(fma(std::declval<T>(), std::declval<T>(), std::declval<T>()))>::value>>
    : public std::true_type {};

}  // namespace has_fma_detail

template <typename T>
struct has_fma : public has_fma_detail::has_fma_impl<T> {};

// Default implementation.
template <typename T, typename Enable = void>
struct fma_impl {
  static_assert(has_fma<T>::value, "No function fma(...) for type.  Please provide an implementation.");
};

// STD or ADL version if it exists.
template <typename T>
struct fma_impl<T, std::enable_if_t<has_fma<T>::value>> {
  static EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE T run(const T& a, const T& b, const T& c) {
    using std::fma;
    return fma(a, b, c);
  }
};

#if defined(EIGEN_GPUCC)
template <>
struct has_fma<float> : public std::true_type {};

template <>
struct fma_impl<float, void> {
  static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE float run(const float& a, const float& b, const float& c) {
    return ::fmaf(a, b, c);
  }
};

template <>
struct has_fma<double> : public std::true_type {};

template <>
struct fma_impl<double, void> {
  static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE double run(const double& a, const double& b, const double& c) {
    return ::fma(a, b, c);
  }
};
#endif

// Basic multiply-add.
template <typename Scalar, typename EnableIf = void>
struct madd_impl {
  static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar run(const Scalar& x, const Scalar& y, const Scalar& z) {
    return x * y + z;
  }
};

#if EIGEN_SCALAR_MADD_USE_FMA
template <typename Scalar>
struct madd_impl<Scalar, std::enable_if_t<has_fma<Scalar>::value>> {
  static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar run(const Scalar& x, const Scalar& y, const Scalar& z) {
    return fma_impl<Scalar>::run(x, y, z);
  }
};
#endif

// Representation properties shared by operations that inspect an IEEE-style binary floating-point value.
template <typename Scalar>
struct binary_floating_point_traits {
  using Bits = typename numext::get_integer_by_size<sizeof(Scalar)>::unsigned_type;
  static constexpr int kFractionBits = std::numeric_limits<Scalar>::digits - 1;
  static constexpr int kExponentBits = int(sizeof(Scalar) * CHAR_BIT) - std::numeric_limits<Scalar>::digits;
  static constexpr int kExponentBias = std::numeric_limits<Scalar>::max_exponent - 1;
  static constexpr Bits kSignBit = Bits(1) << (sizeof(Scalar) * CHAR_BIT - 1);
  static constexpr Bits kExponentUnit = Bits(1) << kFractionBits;
  static constexpr Bits kFractionMask = kExponentUnit - 1;
  static constexpr Bits kExponentMask = ((Bits(1) << kExponentBits) - 1) << kFractionBits;
  static constexpr Bits kMaxFiniteExponentBits = kExponentMask - kExponentUnit;

  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE Bits bits(const Scalar& value) { return numext::bit_cast<Bits>(value); }

  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE Bits magnitude(const Scalar& value) { return bits(value) & ~kSignBit; }
};

}  // end namespace internal

/****************************************************************************
 * Generic math functions                                                    *
 ****************************************************************************/

namespace numext {

#if !defined(EIGEN_GPUCC)
template <typename T>
EIGEN_DEVICE_FUNC constexpr EIGEN_ALWAYS_INLINE T mini(const T& x, const T& y) {
  EIGEN_USING_STD(min)
  return min EIGEN_NOT_A_MACRO(x, y);
}

template <typename T>
EIGEN_DEVICE_FUNC constexpr EIGEN_ALWAYS_INLINE T maxi(const T& x, const T& y) {
  EIGEN_USING_STD(max)
  return max EIGEN_NOT_A_MACRO(x, y);
}
#else
template <typename T>
EIGEN_DEVICE_FUNC constexpr EIGEN_ALWAYS_INLINE T mini(const T& x, const T& y) {
  return y < x ? y : x;
}
#if !defined(EIGEN_CONSTEXPR_ARE_DEVICE_FUNC)
// Without relaxed constexpr, numeric GPU scalars keep fmin/fmax's number-preferring
// NaN behavior. With relaxed constexpr, they use the constexpr ternary overloads,
// matching std::min/std::max behavior and supporting custom less-comparable scalars.
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float mini(const float& x, const float& y) {
  return fminf(x, y);
}
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double mini(const double& x, const double& y) {
  return fmin(x, y);
}

#ifndef EIGEN_GPU_COMPILE_PHASE
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE long double mini(const long double& x, const long double& y) {
#if defined(EIGEN_HIPCC)
  // no "fminl" on HIP yet
  return (x < y) ? x : y;
#else
  return fminl(x, y);
#endif
}
#endif
#endif

template <typename T>
EIGEN_DEVICE_FUNC constexpr EIGEN_ALWAYS_INLINE T maxi(const T& x, const T& y) {
  return x < y ? y : x;
}
#if !defined(EIGEN_CONSTEXPR_ARE_DEVICE_FUNC)
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float maxi(const float& x, const float& y) {
  return fmaxf(x, y);
}
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double maxi(const double& x, const double& y) {
  return fmax(x, y);
}
#ifndef EIGEN_GPU_COMPILE_PHASE
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE long double maxi(const long double& x, const long double& y) {
#if defined(EIGEN_HIPCC)
  // no "fmaxl" on HIP yet
  return (x > y) ? x : y;
#else
  return fmaxl(x, y);
#endif
}
#endif
#endif
#endif

#if defined(SYCL_DEVICE_ONLY)

#define SYCL_SPECIALIZE_SIGNED_INTEGER_TYPES_BINARY(NAME, FUNC) \
  SYCL_SPECIALIZE_BINARY_FUNC(NAME, FUNC, cl::sycl::cl_char)    \
  SYCL_SPECIALIZE_BINARY_FUNC(NAME, FUNC, cl::sycl::cl_short)   \
  SYCL_SPECIALIZE_BINARY_FUNC(NAME, FUNC, cl::sycl::cl_int)     \
  SYCL_SPECIALIZE_BINARY_FUNC(NAME, FUNC, cl::sycl::cl_long)
#define SYCL_SPECIALIZE_SIGNED_INTEGER_TYPES_UNARY(NAME, FUNC) \
  SYCL_SPECIALIZE_UNARY_FUNC(NAME, FUNC, cl::sycl::cl_char)    \
  SYCL_SPECIALIZE_UNARY_FUNC(NAME, FUNC, cl::sycl::cl_short)   \
  SYCL_SPECIALIZE_UNARY_FUNC(NAME, FUNC, cl::sycl::cl_int)     \
  SYCL_SPECIALIZE_UNARY_FUNC(NAME, FUNC, cl::sycl::cl_long)
#define SYCL_SPECIALIZE_UNSIGNED_INTEGER_TYPES_BINARY(NAME, FUNC) \
  SYCL_SPECIALIZE_BINARY_FUNC(NAME, FUNC, cl::sycl::cl_uchar)     \
  SYCL_SPECIALIZE_BINARY_FUNC(NAME, FUNC, cl::sycl::cl_ushort)    \
  SYCL_SPECIALIZE_BINARY_FUNC(NAME, FUNC, cl::sycl::cl_uint)      \
  SYCL_SPECIALIZE_BINARY_FUNC(NAME, FUNC, cl::sycl::cl_ulong)
#define SYCL_SPECIALIZE_UNSIGNED_INTEGER_TYPES_UNARY(NAME, FUNC) \
  SYCL_SPECIALIZE_UNARY_FUNC(NAME, FUNC, cl::sycl::cl_uchar)     \
  SYCL_SPECIALIZE_UNARY_FUNC(NAME, FUNC, cl::sycl::cl_ushort)    \
  SYCL_SPECIALIZE_UNARY_FUNC(NAME, FUNC, cl::sycl::cl_uint)      \
  SYCL_SPECIALIZE_UNARY_FUNC(NAME, FUNC, cl::sycl::cl_ulong)
#define SYCL_SPECIALIZE_INTEGER_TYPES_BINARY(NAME, FUNC)  \
  SYCL_SPECIALIZE_SIGNED_INTEGER_TYPES_BINARY(NAME, FUNC) \
  SYCL_SPECIALIZE_UNSIGNED_INTEGER_TYPES_BINARY(NAME, FUNC)
#define SYCL_SPECIALIZE_INTEGER_TYPES_UNARY(NAME, FUNC)  \
  SYCL_SPECIALIZE_SIGNED_INTEGER_TYPES_UNARY(NAME, FUNC) \
  SYCL_SPECIALIZE_UNSIGNED_INTEGER_TYPES_UNARY(NAME, FUNC)
#define SYCL_SPECIALIZE_FLOATING_TYPES_BINARY(NAME, FUNC)     \
  SYCL_SPECIALIZE_BINARY_FUNC(NAME, FUNC, cl::sycl::cl_float) \
  SYCL_SPECIALIZE_BINARY_FUNC(NAME, FUNC, cl::sycl::cl_double)
#define SYCL_SPECIALIZE_FLOATING_TYPES_UNARY(NAME, FUNC)     \
  SYCL_SPECIALIZE_UNARY_FUNC(NAME, FUNC, cl::sycl::cl_float) \
  SYCL_SPECIALIZE_UNARY_FUNC(NAME, FUNC, cl::sycl::cl_double)
#define SYCL_SPECIALIZE_FLOATING_TYPES_UNARY_FUNC_RET_TYPE(NAME, FUNC, RET_TYPE) \
  SYCL_SPECIALIZE_GEN_UNARY_FUNC(NAME, FUNC, RET_TYPE, cl::sycl::cl_float)       \
  SYCL_SPECIALIZE_GEN_UNARY_FUNC(NAME, FUNC, RET_TYPE, cl::sycl::cl_double)

#define SYCL_SPECIALIZE_GEN_UNARY_FUNC(NAME, FUNC, RET_TYPE, ARG_TYPE)     \
  template <>                                                              \
  EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE RET_TYPE NAME(const ARG_TYPE& x) { \
    return cl::sycl::FUNC(x);                                              \
  }

#define SYCL_SPECIALIZE_UNARY_FUNC(NAME, FUNC, TYPE) SYCL_SPECIALIZE_GEN_UNARY_FUNC(NAME, FUNC, TYPE, TYPE)

#define SYCL_SPECIALIZE_GEN1_BINARY_FUNC(NAME, FUNC, RET_TYPE, ARG_TYPE1, ARG_TYPE2)            \
  template <>                                                                                   \
  EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE RET_TYPE NAME(const ARG_TYPE1& x, const ARG_TYPE2& y) { \
    return cl::sycl::FUNC(x, y);                                                                \
  }

#define SYCL_SPECIALIZE_GEN2_BINARY_FUNC(NAME, FUNC, RET_TYPE, ARG_TYPE) \
  SYCL_SPECIALIZE_GEN1_BINARY_FUNC(NAME, FUNC, RET_TYPE, ARG_TYPE, ARG_TYPE)

#define SYCL_SPECIALIZE_BINARY_FUNC(NAME, FUNC, TYPE) SYCL_SPECIALIZE_GEN2_BINARY_FUNC(NAME, FUNC, TYPE, TYPE)

SYCL_SPECIALIZE_INTEGER_TYPES_BINARY(mini, min)
SYCL_SPECIALIZE_FLOATING_TYPES_BINARY(mini, fmin)
SYCL_SPECIALIZE_INTEGER_TYPES_BINARY(maxi, max)
SYCL_SPECIALIZE_FLOATING_TYPES_BINARY(maxi, fmax)

#endif

template <typename Scalar>
EIGEN_DEVICE_FUNC inline typename NumTraits<Scalar>::Real arg(const Scalar& x) {
  return EIGEN_MATHFUNC_IMPL(arg, Scalar)::run(x);
}

template <typename Scalar>
EIGEN_DEVICE_FUNC inline auto imag_ref(const Scalar& x)
    -> internal::add_const_on_value_type_t<decltype(internal::imag_ref_impl<Scalar>::run(x))> {
  return internal::imag_ref_impl<Scalar>::run(x);
}

template <typename Scalar>
EIGEN_DEVICE_FUNC inline auto imag_ref(Scalar& x)
    -> decltype(EIGEN_MATHFUNC_IMPL(imag_ref, std::remove_const_t<Scalar>)::run(x)) {
  return EIGEN_MATHFUNC_IMPL(imag_ref, std::remove_const_t<Scalar>)::run(x);
}

template <typename Scalar>
EIGEN_DEVICE_FUNC inline Scalar conj(const Scalar& x) {
  return EIGEN_MATHFUNC_IMPL(conj, Scalar)::run(x);
}

template <typename Scalar>
EIGEN_DEVICE_FUNC inline Scalar sign(const Scalar& x) {
  return EIGEN_MATHFUNC_IMPL(sign, Scalar)::run(x);
}

template <typename Scalar>
EIGEN_DEVICE_FUNC inline Scalar copysign(const Scalar& x, const Scalar& y) {
  return EIGEN_MATHFUNC_IMPL(copysign, Scalar)::run(x, y);
}

template <typename Scalar>
EIGEN_DEVICE_FUNC inline Scalar negate(const Scalar& x) {
  return EIGEN_MATHFUNC_IMPL(negate, Scalar)::run(x);
}

template <typename Scalar>
EIGEN_DEVICE_FUNC inline decltype(auto) abs2(const Scalar& x) {
  return EIGEN_MATHFUNC_IMPL(abs2, Scalar)::run(x);
}

EIGEN_DEVICE_FUNC inline bool abs2(bool x) { return x; }

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE T absdiff(const T& x, const T& y) {
  return x > y ? x - y : y - x;
}
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float absdiff(const float& x, const float& y) {
  return fabsf(x - y);
}
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double absdiff(const double& x, const double& y) {
  return fabs(x - y);
}

// HIP and CUDA do not support long double.
#ifndef EIGEN_GPU_COMPILE_PHASE
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE long double absdiff(const long double& x, const long double& y) {
  return fabsl(x - y);
}
#endif

template <typename Scalar>
EIGEN_DEVICE_FUNC inline typename NumTraits<Scalar>::Real norm1(const Scalar& x) {
  return EIGEN_MATHFUNC_IMPL(norm1, Scalar)::run(x);
}

template <typename Scalar>
EIGEN_DEVICE_FUNC inline typename NumTraits<Scalar>::Real hypot(const Scalar& x, const Scalar& y) {
  return EIGEN_MATHFUNC_IMPL(hypot, Scalar)::run(x, y);
}

#if defined(SYCL_DEVICE_ONLY)
SYCL_SPECIALIZE_FLOATING_TYPES_BINARY(hypot, hypot)
#endif

template <typename Scalar>
EIGEN_DEVICE_FUNC inline Scalar log1p(const Scalar& x) {
  return EIGEN_MATHFUNC_IMPL(log1p, Scalar)::run(x);
}

#if defined(SYCL_DEVICE_ONLY)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY(log1p, log1p)
#endif

#if defined(EIGEN_GPUCC)
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float log1p(const float& x) {
  return ::log1pf(x);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double log1p(const double& x) {
  return ::log1p(x);
}
#endif

template <typename ScalarX, typename ScalarY>
EIGEN_DEVICE_FUNC inline typename internal::pow_impl<ScalarX, ScalarY>::result_type pow(const ScalarX& x,
                                                                                        const ScalarY& y) {
  return internal::pow_impl<ScalarX, ScalarY>::run(x, y);
}

#if defined(SYCL_DEVICE_ONLY)
SYCL_SPECIALIZE_FLOATING_TYPES_BINARY(pow, pow)
#endif

template <typename T>
EIGEN_DEVICE_FUNC bool(isnan)(const T& x) {
  return internal::isnan_impl(x);
}
template <typename T>
EIGEN_DEVICE_FUNC bool(isinf)(const T& x) {
  return internal::isinf_impl(x);
}
template <typename T>
EIGEN_DEVICE_FUNC bool(isfinite)(const T& x) {
  return internal::isfinite_impl(x);
}

#if defined(SYCL_DEVICE_ONLY)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY_FUNC_RET_TYPE(isnan, isnan, bool)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY_FUNC_RET_TYPE(isinf, isinf, bool)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY_FUNC_RET_TYPE(isfinite, isfinite, bool)
#endif

template <typename Scalar>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar rint(const Scalar& x) {
  return internal::nearest_integer_impl<Scalar>::run_rint(x);
}

template <typename Scalar>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar round(const Scalar& x) {
  return internal::nearest_integer_impl<Scalar>::run_round(x);
}

template <typename Scalar>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar(floor)(const Scalar& x) {
  return internal::nearest_integer_impl<Scalar>::run_floor(x);
}

template <typename Scalar>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar(ceil)(const Scalar& x) {
  return internal::nearest_integer_impl<Scalar>::run_ceil(x);
}

template <typename Scalar>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar(trunc)(const Scalar& x) {
  return internal::nearest_integer_impl<Scalar>::run_trunc(x);
}

#if defined(SYCL_DEVICE_ONLY)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY(round, round)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY(floor, floor)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY(ceil, ceil)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY(trunc, trunc)
#endif

#if defined(EIGEN_GPUCC)
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float floor(const float& x) {
  return ::floorf(x);
}
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double floor(const double& x) {
  return ::floor(x);
}
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float ceil(const float& x) {
  return ::ceilf(x);
}
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double ceil(const double& x) {
  return ::ceil(x);
}
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float trunc(const float& x) {
  return ::truncf(x);
}
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double trunc(const double& x) {
  return ::trunc(x);
}
#endif

// Integer division with rounding up.
// T is assumed to be an integer type with a>=0, and b>0
template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE constexpr T div_ceil(T a, T b) {
  using UnsignedT = std::make_unsigned_t<T>;
  EIGEN_STATIC_ASSERT((NumTraits<T>::IsInteger), THIS FUNCTION IS FOR INTEGER TYPES)
  // Note: explicitly declaring a and b as non-negative values allows the compiler to use better optimizations
  const UnsignedT ua = UnsignedT(a);
  const UnsignedT ub = UnsignedT(b);
  // Note: This form is used because it cannot overflow.
  return ua == 0 ? 0 : (ua - 1) / ub + 1;
}

// Integer round down to nearest power of b
// T is assumed to be an integer type with a>=0, and b>0
template <typename T, typename U>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE constexpr T round_down(T a, U b) {
  using UnsignedT = std::make_unsigned_t<T>;
  using UnsignedU = std::make_unsigned_t<U>;
  EIGEN_STATIC_ASSERT((NumTraits<T>::IsInteger), THIS FUNCTION IS FOR INTEGER TYPES)
  EIGEN_STATIC_ASSERT((NumTraits<U>::IsInteger), THIS FUNCTION IS FOR INTEGER TYPES)
  // Note: explicitly declaring a and b as non-negative values allows the compiler to use better optimizations
  const UnsignedT ua = UnsignedT(a);
  const UnsignedU ub = UnsignedU(b);
  return ub * (ua / ub);
}

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE T log2(T x) {
  EIGEN_USING_STD(log2);
  return log2(x);
}

/** Log base 2 for 32 bits positive integers.
 * Conveniently returns 0 for x==0. */
constexpr int log2(int x) {
  unsigned int v(x);
  constexpr int table[32] = {0, 9,  1,  10, 13, 21, 2,  29, 11, 14, 16, 18, 22, 25, 3, 30,
                             8, 12, 20, 28, 15, 17, 24, 7,  19, 27, 23, 6,  26, 5,  4, 31};
  v |= v >> 1;
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;
  return table[(v * 0x07C4ACDDU) >> 27];
}

/** \returns the square root of \a x.
 *
 * It is essentially equivalent to
 * \code using std::sqrt; return sqrt(x); \endcode
 * but slightly faster for float/double and some compilers (e.g., gcc), thanks to
 * specializations when SSE is enabled.
 *
 * Its usage is justified in performance critical functions, like norm/normalize.
 */
template <typename Scalar>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE Scalar sqrt(const Scalar& x) {
  return EIGEN_MATHFUNC_IMPL(sqrt, Scalar)::run(x);
}

// Boolean specialization, avoids implicit float to bool conversion (-Wimplicit-conversion-floating-point-to-bool).
template <>
EIGEN_DEFINE_FUNCTION_ALLOWING_MULTIPLE_DEFINITIONS EIGEN_DEVICE_FUNC bool sqrt<bool>(const bool& x) {
  return x;
}

#if defined(SYCL_DEVICE_ONLY)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY(sqrt, sqrt)
#endif

/** \returns the cube root of \a x. **/
template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE std::enable_if_t<!NumTraits<T>::IsComplex, T> cbrt(const T& x) {
  EIGEN_USING_STD(cbrt);
  return static_cast<T>(cbrt(x));
}

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE std::enable_if_t<NumTraits<T>::IsComplex, T> cbrt(const T& x) {
  EIGEN_USING_STD(pow);
  return pow(x, typename NumTraits<T>::Real(1.0 / 3.0));
}

/** \returns the reciprocal square root of \a x. **/
template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE T rsqrt(const T& x) {
  return internal::rsqrt_impl<T>::run(x);
}

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE T log(const T& x) {
  return internal::log_impl<T>::run(x);
}

#if defined(SYCL_DEVICE_ONLY)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY(log, log)
#endif

#if defined(EIGEN_GPUCC)
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float log(const float& x) {
  return ::logf(x);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double log(const double& x) {
  return ::log(x);
}
#endif

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE
    std::enable_if_t<NumTraits<T>::IsSigned && !NumTraits<T>::IsComplex, typename NumTraits<T>::Real>
    abs(const T& x) {
  EIGEN_USING_STD(abs);
  return abs(x);
}

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE std::enable_if_t<NumTraits<T>::IsComplex, typename NumTraits<T>::Real> abs(
    const T& x) {
  return numext::hypot(numext::real(x), numext::imag(x));
}

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE
    std::enable_if_t<!NumTraits<T>::IsSigned && !NumTraits<T>::IsComplex, typename NumTraits<T>::Real>
    abs(const T& x) {
  return x;
}

#if defined(SYCL_DEVICE_ONLY)
SYCL_SPECIALIZE_INTEGER_TYPES_UNARY(abs, abs)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY(abs, fabs)
#endif

#if defined(EIGEN_GPUCC)
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float abs(const float& x) {
  return ::fabsf(x);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double abs(const double& x) {
  return ::fabs(x);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float abs(const std::complex<float>& x) {
  return ::hypotf(x.real(), x.imag());
}

template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double abs(const std::complex<double>& x) {
  return ::hypot(x.real(), x.imag());
}
#endif

template <typename Scalar, bool IsInteger = NumTraits<Scalar>::IsInteger, bool IsSigned = NumTraits<Scalar>::IsSigned>
struct signbit_impl;
template <typename Scalar>
struct signbit_impl<Scalar, false, true> {
  static constexpr size_t Size = sizeof(Scalar);
  static constexpr size_t Shift = (CHAR_BIT * Size) - 1;
  using intSize_t = typename get_integer_by_size<Size>::signed_type;
  EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE static Scalar run(const Scalar& x) {
    intSize_t a = bit_cast<intSize_t, Scalar>(x);
    a = a >> Shift;
    Scalar result = bit_cast<Scalar, intSize_t>(a);
    return result;
  }
};
template <typename Scalar>
struct signbit_impl<Scalar, true, true> {
  static constexpr size_t Size = sizeof(Scalar);
  static constexpr size_t Shift = (CHAR_BIT * Size) - 1;
  EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE static constexpr Scalar run(const Scalar& x) { return x >> Shift; }
};
template <typename Scalar>
struct signbit_impl<Scalar, true, false> {
  EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE static constexpr Scalar run(const Scalar&) { return Scalar(0); }
};
template <typename Scalar>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE static constexpr Scalar signbit(const Scalar& x) {
  return signbit_impl<Scalar>::run(x);
}

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE T exp(const T& x) {
  EIGEN_USING_STD(exp);
  return exp(x);
}

// MSVC through at least 19.51 mishandles some finite-real, non-finite-imaginary inputs to std::exp(complex).
#if EIGEN_COMP_MSVC
template <typename RealScalar>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE std::complex<RealScalar> exp(const std::complex<RealScalar>& x) {
  // Evaluate eagerly to preserve the floating-point exceptions raised by std::exp before correcting its result.
  const std::complex<RealScalar> result = std::exp(x);
  // If z is (x,±∞) (for any finite x), the result is (NaN,NaN) and FE_INVALID is raised.
  // If z is (x,NaN) (for any finite x), the result is (NaN,NaN) and FE_INVALID may be raised.
  if ((isfinite)(real_ref(x)) && !(isfinite)(imag_ref(x))) {
    return std::complex<RealScalar>(NumTraits<RealScalar>::quiet_NaN(), NumTraits<RealScalar>::quiet_NaN());
  }
#if EIGEN_COMP_MSVC < 1931
  // Before 19.31, MSVC also mishandles a positive-infinite real part with a non-finite imaginary part.
  // If z is (+∞,±∞), the result is (±∞,NaN) and FE_INVALID is raised (the sign of the real part is unspecified)
  // If z is (+∞,NaN), the result is (±∞,NaN) (the sign of the real part is unspecified)
  if ((real_ref(x) == NumTraits<RealScalar>::infinity() && !(isfinite)(imag_ref(x)))) {
    return std::complex<RealScalar>(NumTraits<RealScalar>::infinity(), NumTraits<RealScalar>::quiet_NaN());
  }
#endif
  return result;
}
#endif

#if defined(SYCL_DEVICE_ONLY)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY(exp, exp)
#endif

#if defined(EIGEN_GPUCC)
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float exp(const float& x) {
  return ::expf(x);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double exp(const double& x) {
  return ::exp(x);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE std::complex<float> exp(const std::complex<float>& x) {
  float com = ::expf(x.real());
  float res_real = com * ::cosf(x.imag());
  float res_imag = com * ::sinf(x.imag());
  return std::complex<float>(res_real, res_imag);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE std::complex<double> exp(const std::complex<double>& x) {
  double com = ::exp(x.real());
  double res_real = com * ::cos(x.imag());
  double res_imag = com * ::sin(x.imag());
  return std::complex<double>(res_real, res_imag);
}
#endif

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE T exp2(const T& x) {
  EIGEN_USING_STD(exp2);
  return exp2(x);
}

// MSVC screws up some edge-cases for std::exp2(complex).
#ifdef EIGEN_COMP_MSVC
template <typename RealScalar>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE std::complex<RealScalar> exp2(const std::complex<RealScalar>& x) {
  EIGEN_USING_STD(exp);
  // If z is (x,±∞) (for any finite x), the result is (NaN,NaN) and FE_INVALID is raised.
  // If z is (x,NaN) (for any finite x), the result is (NaN,NaN) and FE_INVALID may be raised.
  if ((isfinite)(real_ref(x)) && !(isfinite)(imag_ref(x))) {
    return std::complex<RealScalar>(NumTraits<RealScalar>::quiet_NaN(), NumTraits<RealScalar>::quiet_NaN());
  }
  // If z is (+∞,±∞), the result is (±∞,NaN) and FE_INVALID is raised (the sign of the real part is unspecified)
  // If z is (+∞,NaN), the result is (±∞,NaN) (the sign of the real part is unspecified)
  if ((real_ref(x) == NumTraits<RealScalar>::infinity() && !(isfinite)(imag_ref(x)))) {
    return std::complex<RealScalar>(NumTraits<RealScalar>::infinity(), NumTraits<RealScalar>::quiet_NaN());
  }
  return exp2(x);
}
#endif

#if defined(SYCL_DEVICE_ONLY)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY(exp2, exp2)
#endif

#if defined(EIGEN_GPUCC)
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float exp2(const float& x) {
  return ::exp2f(x);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double exp2(const double& x) {
  return ::exp2(x);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE std::complex<float> exp2(const std::complex<float>& x) {
  float com = ::exp2f(x.real());
  float res_real = com * ::cosf(static_cast<float>(EIGEN_LN2) * x.imag());
  float res_imag = com * ::sinf(static_cast<float>(EIGEN_LN2) * x.imag());
  return std::complex<float>(res_real, res_imag);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE std::complex<double> exp2(const std::complex<double>& x) {
  double com = ::exp2(x.real());
  double res_real = com * ::cos(static_cast<double>(EIGEN_LN2) * x.imag());
  double res_imag = com * ::sin(static_cast<double>(EIGEN_LN2) * x.imag());
  return std::complex<double>(res_real, res_imag);
}
#endif

// Exact scaling by 2^exponent, including denormals and unrepresentable scale factors.
// Results outside the finite range saturate to zero or infinity.
template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE T ldexp(const T& x, int exponent) {
  EIGEN_USING_STD(ldexp);
  return static_cast<T>(ldexp(x, exponent));
}

#if defined(SYCL_DEVICE_ONLY)
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE cl::sycl::cl_float ldexp(const cl::sycl::cl_float& x, int exponent) {
  return cl::sycl::ldexp(x, exponent);
}
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE cl::sycl::cl_double ldexp(const cl::sycl::cl_double& x, int exponent) {
  return cl::sycl::ldexp(x, exponent);
}
#endif

#if defined(EIGEN_GPUCC)
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float ldexp(const float& x, int exponent) {
  return ::ldexpf(x, exponent);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double ldexp(const double& x, int exponent) {
  return ::ldexp(x, exponent);
}
#endif

template <typename Scalar>
EIGEN_DEVICE_FUNC inline Scalar expm1(const Scalar& x) {
  return EIGEN_MATHFUNC_IMPL(expm1, Scalar)::run(x);
}

#if defined(SYCL_DEVICE_ONLY)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY(expm1, expm1)
#endif

#if defined(EIGEN_GPUCC)
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float expm1(const float& x) {
  return ::expm1f(x);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double expm1(const double& x) {
  return ::expm1(x);
}
#endif

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE T cos(const T& x) {
  EIGEN_USING_STD(cos);
  return cos(x);
}

#if defined(SYCL_DEVICE_ONLY)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY(cos, cos)
#endif

#if defined(EIGEN_GPUCC)
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float cos(const float& x) {
  return ::cosf(x);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double cos(const double& x) {
  return ::cos(x);
}
#endif

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE T sin(const T& x) {
  EIGEN_USING_STD(sin);
  return sin(x);
}

#if defined(SYCL_DEVICE_ONLY)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY(sin, sin)
#endif

#if defined(EIGEN_GPUCC)
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float sin(const float& x) {
  return ::sinf(x);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double sin(const double& x) {
  return ::sin(x);
}
#endif

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE T tan(const T& x) {
  EIGEN_USING_STD(tan);
  return tan(x);
}

#if defined(SYCL_DEVICE_ONLY)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY(tan, tan)
#endif

#if defined(EIGEN_GPUCC)
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float tan(const float& x) {
  return ::tanf(x);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double tan(const double& x) {
  return ::tan(x);
}
#endif

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE T acos(const T& x) {
  EIGEN_USING_STD(acos);
  return acos(x);
}

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE T acosh(const T& x) {
  EIGEN_USING_STD(acosh);
  return static_cast<T>(acosh(x));
}

#if defined(SYCL_DEVICE_ONLY)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY(acos, acos)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY(acosh, acosh)
#endif

#if defined(EIGEN_GPUCC)
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float acos(const float& x) {
  return ::acosf(x);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double acos(const double& x) {
  return ::acos(x);
}
#endif

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE T asin(const T& x) {
  EIGEN_USING_STD(asin);
  return asin(x);
}

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE T asinh(const T& x) {
  EIGEN_USING_STD(asinh);
  return static_cast<T>(asinh(x));
}

#if defined(SYCL_DEVICE_ONLY)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY(asin, asin)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY(asinh, asinh)
#endif

#if defined(EIGEN_GPUCC)
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float asin(const float& x) {
  return ::asinf(x);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double asin(const double& x) {
  return ::asin(x);
}
#endif

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE T atan(const T& x) {
  EIGEN_USING_STD(atan);
  return static_cast<T>(atan(x));
}

template <typename T, std::enable_if_t<!NumTraits<T>::IsComplex, int> = 0>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE T atan2(const T& y, const T& x) {
  EIGEN_USING_STD(atan2);
  return static_cast<T>(atan2(y, x));
}

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE T atanh(const T& x) {
  EIGEN_USING_STD(atanh);
  return static_cast<T>(atanh(x));
}

#if defined(SYCL_DEVICE_ONLY)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY(atan, atan)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY(atanh, atanh)
#endif

#if defined(EIGEN_GPUCC)
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float atan(const float& x) {
  return ::atanf(x);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double atan(const double& x) {
  return ::atan(x);
}
#endif

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE T cosh(const T& x) {
  EIGEN_USING_STD(cosh);
  return static_cast<T>(cosh(x));
}

#if defined(SYCL_DEVICE_ONLY)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY(cosh, cosh)
#endif

#if defined(EIGEN_GPUCC)
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float cosh(const float& x) {
  return ::coshf(x);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double cosh(const double& x) {
  return ::cosh(x);
}
#endif

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE T sinh(const T& x) {
  EIGEN_USING_STD(sinh);
  return static_cast<T>(sinh(x));
}

#if defined(SYCL_DEVICE_ONLY)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY(sinh, sinh)
#endif

#if defined(EIGEN_GPUCC)
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float sinh(const float& x) {
  return ::sinhf(x);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double sinh(const double& x) {
  return ::sinh(x);
}
#endif

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE T tanh(const T& x) {
  EIGEN_USING_STD(tanh);
  return tanh(x);
}

#if (!defined(EIGEN_GPUCC)) && EIGEN_FAST_MATH && !defined(SYCL_DEVICE_ONLY)
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float tanh(float x) { return internal::ptanh_float(x); }
#endif

#if defined(SYCL_DEVICE_ONLY)
SYCL_SPECIALIZE_FLOATING_TYPES_UNARY(tanh, tanh)
#endif

#if defined(EIGEN_GPUCC)
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float tanh(const float& x) {
  return ::tanhf(x);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double tanh(const double& x) {
  return ::tanh(x);
}
#endif

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE T fmod(const T& a, const T& b) {
  EIGEN_USING_STD(fmod);
  return fmod(a, b);
}

#if defined(SYCL_DEVICE_ONLY)
SYCL_SPECIALIZE_FLOATING_TYPES_BINARY(fmod, fmod)
#endif

#if defined(EIGEN_GPUCC)
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float fmod(const float& a, const float& b) {
  return ::fmodf(a, b);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double fmod(const double& a, const double& b) {
  return ::fmod(a, b);
}
#endif

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE T nextafter(const T& from, const T& to) {
  EIGEN_USING_STD(nextafter);
  return nextafter(from, to);
}

/** Returns the smallest normal power of two greater than or equal to \a value.
 *
 * \pre \a value is finite and nonnegative.
 * Zero is preserved. A value larger than the greatest finite power of two maps to infinity.
 */
template <typename Scalar,
          std::enable_if_t<!std::is_same<Scalar, long double>::value && std::numeric_limits<Scalar>::is_iec559 &&
                               std::numeric_limits<Scalar>::radix == 2 &&
                               (sizeof(Scalar) == sizeof(uint16_t) || sizeof(Scalar) == sizeof(uint32_t) ||
                                sizeof(Scalar) == sizeof(uint64_t)),
                           int> = 0>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar ceil_power_of_two(const Scalar& value) {
  using Binary = internal::binary_floating_point_traits<Scalar>;
  using Bits = typename Binary::Bits;
  const Bits valueBits = Binary::bits(value);
  const Bits resultBits = Bits((valueBits + Binary::kFractionMask) & Binary::kExponentMask);
  return bit_cast<Scalar>(resultBits);
}

// long double has no portable object representation: depending on the ABI it can be binary64, padded x87 extended
// precision, IEEE binary128, or IBM double-double. Construct the result arithmetically instead of inspecting its bits.
#if !defined(EIGEN_GPU_COMPILE_PHASE)
EIGEN_STRONG_INLINE long double ceil_power_of_two(const long double& value) {
  EIGEN_USING_STD(frexp);
  EIGEN_USING_STD(ldexp);
  int exponent = 0;
  const long double fraction = frexp(value, &exponent);
  if (fraction == 0.0L) return 0.0L;
  if (fraction == 0.5L) --exponent;
  if (exponent < std::numeric_limits<long double>::min_exponent - 1)
    exponent = std::numeric_limits<long double>::min_exponent - 1;
  return ldexp(1.0L, exponent);
}
#endif

#if defined(SYCL_DEVICE_ONLY)
SYCL_SPECIALIZE_FLOATING_TYPES_BINARY(nextafter, nextafter)
#endif

#if defined(EIGEN_GPUCC)
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float nextafter(const float& from, const float& to) {
  return ::nextafterf(from, to);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double nextafter(const double& from, const double& to) {
  return ::nextafter(from, to);
}
#endif

#if defined(SYCL_DEVICE_ONLY)
#undef SYCL_SPECIALIZE_SIGNED_INTEGER_TYPES_BINARY
#undef SYCL_SPECIALIZE_SIGNED_INTEGER_TYPES_UNARY
#undef SYCL_SPECIALIZE_UNSIGNED_INTEGER_TYPES_BINARY
#undef SYCL_SPECIALIZE_UNSIGNED_INTEGER_TYPES_UNARY
#undef SYCL_SPECIALIZE_INTEGER_TYPES_BINARY
#undef SYCL_SPECIALIZE_INTEGER_TYPES_UNARY
#undef SYCL_SPECIALIZE_FLOATING_TYPES_BINARY
#undef SYCL_SPECIALIZE_FLOATING_TYPES_UNARY
#undef SYCL_SPECIALIZE_FLOATING_TYPES_UNARY_FUNC_RET_TYPE
#undef SYCL_SPECIALIZE_GEN_UNARY_FUNC
#undef SYCL_SPECIALIZE_UNARY_FUNC
#undef SYCL_SPECIALIZE_GEN1_BINARY_FUNC
#undef SYCL_SPECIALIZE_GEN2_BINARY_FUNC
#undef SYCL_SPECIALIZE_BINARY_FUNC
#endif

// A logical shift is a shift of the bit pattern, so it goes through the unsigned type of the same
// width whatever Scalar's signedness. A Scalar narrower than int promotes to int for the shift
// itself, which the explicit truncation undoes: for the left shift the promoted value can carry set
// bits above Scalar's width, and dropping them is the operation rather than an accident of the cast.
template <typename Scalar, typename Enable = std::enable_if_t<std::is_integral<Scalar>::value>>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar logical_shift_left(const Scalar& a, int n) {
  using UnsignedScalar = typename numext::get_integer_by_size<sizeof(Scalar)>::unsigned_type;
  return bit_cast<Scalar, UnsignedScalar>(static_cast<UnsignedScalar>(bit_cast<UnsignedScalar, Scalar>(a) << n));
}

template <typename Scalar, typename Enable = std::enable_if_t<std::is_integral<Scalar>::value>>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar logical_shift_right(const Scalar& a, int n) {
  using UnsignedScalar = typename numext::get_integer_by_size<sizeof(Scalar)>::unsigned_type;
  return bit_cast<Scalar, UnsignedScalar>(static_cast<UnsignedScalar>(bit_cast<UnsignedScalar, Scalar>(a) >> n));
}

// An arithmetic shift propagates the sign bit, so it coincides with the logical shift when Scalar is
// unsigned and has none. Shifting through the signed type unconditionally would sign-extend an
// ordinary value bit, which is what every backend's parithmetic_shift_right on unsigned packets
// avoids, leaving the scalar and vectorized paths of one expression disagreeing. Scalar's own
// operator>> already selects on its signedness, so no reinterpretation is needed; the cast only
// undoes the integral promotion a Scalar narrower than int is subject to.
template <typename Scalar, typename Enable = std::enable_if_t<std::is_integral<Scalar>::value>>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar arithmetic_shift_right(const Scalar& a, int n) {
  return static_cast<Scalar>(a >> n);
}

template <typename Scalar>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar fma(const Scalar& x, const Scalar& y, const Scalar& z) {
  return internal::fma_impl<Scalar>::run(x, y, z);
}

// Multiply-add.
template <typename Scalar>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar madd(const Scalar& x, const Scalar& y, const Scalar& z) {
  return internal::madd_impl<Scalar>::run(x, y, z);
}

}  // end namespace numext

namespace internal {

template <typename T>
EIGEN_DEVICE_FUNC bool isfinite_impl(const std::complex<T>& x) {
  EIGEN_IF_CONSTEXPR ((std::is_floating_point<T>::value)) {
    // Eager classification lets coefficient-wise loops remain branch-free.
    return static_cast<unsigned int>((numext::isfinite)(numext::real(x))) & (numext::isfinite)(numext::imag(x));
  } else {
    return (numext::isfinite)(numext::real(x)) && (numext::isfinite)(numext::imag(x));
  }
}

template <typename T>
EIGEN_DEVICE_FUNC bool isnan_impl(const std::complex<T>& x) {
  EIGEN_IF_CONSTEXPR ((std::is_floating_point<T>::value)) {
    return static_cast<unsigned int>((numext::isnan)(numext::real(x))) | (numext::isnan)(numext::imag(x));
  } else {
    return (numext::isnan)(numext::real(x)) || (numext::isnan)(numext::imag(x));
  }
}

template <typename T>
EIGEN_DEVICE_FUNC bool isinf_impl(const std::complex<T>& x) {
  EIGEN_IF_CONSTEXPR ((std::is_floating_point<T>::value)) {
    const bool has_inf = static_cast<unsigned int>((numext::isinf)(numext::real(x))) | (numext::isinf)(numext::imag(x));
    const bool has_nan = (numext::isnan)(x);
    return has_inf & !has_nan;
  } else {
    return ((numext::isinf)(numext::real(x)) || (numext::isinf)(numext::imag(x))) && (!(numext::isnan)(x));
  }
}

/****************************************************************************
 * Implementation of fuzzy comparisons                                       *
 ****************************************************************************/

template <typename Scalar, bool IsComplex, bool IsInteger>
struct scalar_fuzzy_default_impl {};

template <typename Scalar>
struct scalar_fuzzy_default_impl<Scalar, false, false> {
  using RealScalar = typename NumTraits<Scalar>::Real;
  template <typename OtherScalar>
  EIGEN_DEVICE_FUNC static inline bool isMuchSmallerThan(const Scalar& x, const OtherScalar& y,
                                                         const RealScalar& prec) {
    return numext::abs(x) <= numext::abs(y) * prec;
  }
  EIGEN_DEVICE_FUNC static inline bool isApprox(const Scalar& x, const Scalar& y, const RealScalar& prec) {
    return numext::abs(x - y) <= numext::mini(numext::abs(x), numext::abs(y)) * prec;
  }
  EIGEN_DEVICE_FUNC static inline bool isApproxOrLessThan(const Scalar& x, const Scalar& y, const RealScalar& prec) {
    return x <= y || isApprox(x, y, prec);
  }
};

template <typename Scalar>
struct scalar_fuzzy_default_impl<Scalar, false, true> {
  using RealScalar = typename NumTraits<Scalar>::Real;
  template <typename OtherScalar>
  EIGEN_DEVICE_FUNC static inline bool isMuchSmallerThan(const Scalar& x, const Scalar&, const RealScalar&) {
    return x == Scalar(0);
  }
  EIGEN_DEVICE_FUNC static inline bool isApprox(const Scalar& x, const Scalar& y, const RealScalar&) { return x == y; }
  EIGEN_DEVICE_FUNC static inline bool isApproxOrLessThan(const Scalar& x, const Scalar& y, const RealScalar&) {
    return x <= y;
  }
};

template <typename Scalar>
struct scalar_fuzzy_default_impl<Scalar, true, false> {
  using RealScalar = typename NumTraits<Scalar>::Real;
  template <typename OtherScalar>
  EIGEN_DEVICE_FUNC static inline bool isMuchSmallerThan(const Scalar& x, const OtherScalar& y,
                                                         const RealScalar& prec) {
    return numext::abs2(x) <= numext::abs2(y) * prec * prec;
  }
  EIGEN_DEVICE_FUNC static inline bool isApprox(const Scalar& x, const Scalar& y, const RealScalar& prec) {
    return numext::abs2(x - y) <= numext::mini(numext::abs2(x), numext::abs2(y)) * prec * prec;
  }
};

template <typename Scalar>
struct scalar_fuzzy_impl
    : scalar_fuzzy_default_impl<Scalar, NumTraits<Scalar>::IsComplex, NumTraits<Scalar>::IsInteger> {};

template <typename Scalar, typename OtherScalar>
EIGEN_DEVICE_FUNC inline bool isMuchSmallerThan(
    const Scalar& x, const OtherScalar& y,
    const typename NumTraits<Scalar>::Real& precision = NumTraits<Scalar>::dummy_precision()) {
  return scalar_fuzzy_impl<Scalar>::template isMuchSmallerThan<OtherScalar>(x, y, precision);
}

template <typename Scalar>
EIGEN_DEVICE_FUNC inline bool isApprox(
    const Scalar& x, const Scalar& y,
    const typename NumTraits<Scalar>::Real& precision = NumTraits<Scalar>::dummy_precision()) {
  return scalar_fuzzy_impl<Scalar>::isApprox(x, y, precision);
}

template <typename Scalar>
EIGEN_DEVICE_FUNC inline bool isApproxOrLessThan(
    const Scalar& x, const Scalar& y,
    const typename NumTraits<Scalar>::Real& precision = NumTraits<Scalar>::dummy_precision()) {
  return scalar_fuzzy_impl<Scalar>::isApproxOrLessThan(x, y, precision);
}

/******************************************
***  The special case of the  bool type ***
******************************************/

template <>
struct scalar_fuzzy_impl<bool> {
  using RealScalar = bool;

  template <typename OtherScalar>
  EIGEN_DEVICE_FUNC static inline bool isMuchSmallerThan(const bool& x, const bool&, const bool&) {
    return !x;
  }

  EIGEN_DEVICE_FUNC static inline bool isApprox(bool x, bool y, bool) { return x == y; }

  EIGEN_DEVICE_FUNC static inline bool isApproxOrLessThan(const bool& x, const bool& y, const bool&) {
    return (!x) || y;
  }
};

}  // end namespace internal

// Default implementations that rely on other numext implementations
namespace internal {

// Specialization for complex types that are not supported by std::expm1.
template <typename RealScalar>
struct expm1_impl<std::complex<RealScalar>> {
  EIGEN_STATIC_ASSERT_NON_INTEGER(RealScalar)

  EIGEN_DEVICE_FUNC static inline std::complex<RealScalar> run(const std::complex<RealScalar>& x) {
    RealScalar xr = x.real();
    RealScalar xi = x.imag();
    // expm1(z) = exp(z) - 1
    //          = exp(x +  i * y) - 1
    //          = exp(x) * (cos(y) + i * sin(y)) - 1
    //          = exp(x) * cos(y) - 1 + i * exp(x) * sin(y)
    // Imag(expm1(z)) = exp(x) * sin(y)
    // Real(expm1(z)) = exp(x) * cos(y) - 1
    //          = exp(x) * cos(y) - 1.
    //          = expm1(x) + exp(x) * (cos(y) - 1)
    //          = expm1(x) + exp(x) * (2 * sin(y / 2) ** 2)
    RealScalar erm1 = numext::expm1<RealScalar>(xr);
    RealScalar er = erm1 + RealScalar(1.);
    RealScalar sin2 = numext::sin(xi / RealScalar(2.));
    sin2 = sin2 * sin2;
    RealScalar s = numext::sin(xi);
    RealScalar real_part = erm1 - RealScalar(2.) * er * sin2;
    return std::complex<RealScalar>(real_part, er * s);
  }
};

template <typename T>
struct rsqrt_impl {
// C4804: unsafe use of type 'bool' in operation. Unavoidable when instantiated with T=bool.
#if EIGEN_COMP_MSVC
#pragma warning(push)
#pragma warning(disable : 4804)
#endif
  EIGEN_DEVICE_FUNC static EIGEN_ALWAYS_INLINE T run(const T& x) { return T(1) / numext::sqrt(x); }
#if EIGEN_COMP_MSVC
#pragma warning(pop)
#endif
};

#if defined(EIGEN_GPU_COMPILE_PHASE)
template <typename T>
struct conj_impl<std::complex<T>, true> {
  EIGEN_DEVICE_FUNC static inline std::complex<T> run(const std::complex<T>& x) {
    return std::complex<T>(numext::real(x), -numext::imag(x));
  }
};
#endif

// Complex multiply and division operators.
// Note that these do not handle the case if inf+NaNi, which is considered an infinity.
// This is for consistency with our standard pmul, pdiv implementations.
template <typename T>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T> complex_multiply(const std::complex<T>& a,
                                                                       const std::complex<T>& b) {
  const T a_real = numext::real(a);
  const T a_imag = numext::imag(a);
  const T b_real = numext::real(b);
  const T b_imag = numext::imag(b);
  return std::complex<T>(a_real * b_real - a_imag * b_imag, a_imag * b_real + a_real * b_imag);
}

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T> complex_divide_fast(const std::complex<T>& a,
                                                                          const std::complex<T>& b) {
  const T a_real = numext::real(a);
  const T a_imag = numext::imag(a);
  const T b_real = numext::real(b);
  const T b_imag = numext::imag(b);
  const T norm = (b_real * b_real + b_imag * b_imag);
  return std::complex<T>((a_real * b_real + a_imag * b_imag) / norm, (a_imag * b_real - a_real * b_imag) / norm);
}

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T> complex_divide_smith(const std::complex<T>& a,
                                                                           const std::complex<T>& b) {
  const T a_real = numext::real(a);
  const T a_imag = numext::imag(a);
  const T b_real = numext::real(b);
  const T b_imag = numext::imag(b);
  // Smith's complex division (https://arxiv.org/pdf/1210.4539.pdf),
  // guards against over/under-flow.
  const bool scale_imag = numext::abs(b_imag) <= numext::abs(b_real);
  const T rscale = scale_imag ? T(1) : b_real / b_imag;
  const T iscale = scale_imag ? b_imag / b_real : T(1);
  const T denominator = b_real * rscale + b_imag * iscale;
  return std::complex<T>((a_real * rscale + a_imag * iscale) / denominator,
                         (a_imag * rscale - a_real * iscale) / denominator);
}

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T> complex_divide(const std::complex<T>& a,
                                                                     const std::complex<T>& b) {
#if EIGEN_FAST_MATH
  return complex_divide_fast(a, b);
#else
  return complex_divide_smith(a, b);
#endif
}

}  // end namespace internal

namespace numext {

/** \internal
 * \brief Scalar division that guards against intermediate overflow for complex types.
 *
 * For real types, this evaluates a / b.
 * For complex types, this evaluates a / b using Smith's method to prevent intermediate
 * overflow (c^2 + d^2) when b is near the boundary of the representable range, consistent
 * with the vectorized complex division path in GenericPacketMathComplex.h.
 */
template <typename LhsScalar, typename RhsScalar>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE auto divide(const LhsScalar& a, const RhsScalar& b) -> decltype(a / b) {
  return a / b;
}

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T> divide(const std::complex<T>& a, const std::complex<T>& b) {
  return internal::complex_divide_smith(a, b);
}

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T> divide(const T& a, const std::complex<T>& b) {
  return internal::complex_divide_smith(std::complex<T>(a, T(0)), b);
}

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T> divide(const std::complex<T>& a, const T& b) {
  return a / b;
}

}  // end namespace numext

}  // end namespace Eigen

#endif  // EIGEN_MATHFUNCTIONS_H
