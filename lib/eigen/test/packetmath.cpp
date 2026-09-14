// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2009 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2006-2008 Benoit Jacob <jacob.benoit.1@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include <utility>
#include "packetmath_test_shared.h"
#include "random_without_cast_overflow.h"
#include "fp_control.h"

using internal::unpacket_traits;

template <typename T, std::enable_if_t<!NumTraits<T>::IsInteger || !NumTraits<T>::IsSigned, int> = 0>
inline T REF_ADD(const T& a, const T& b) {
  return a + b;
}
template <typename T, std::enable_if_t<NumTraits<T>::IsInteger && NumTraits<T>::IsSigned, int> = 0>
inline T REF_ADD(const T& a, const T& b) {
  using UnsignedT = std::make_unsigned_t<T>;
  return static_cast<T>(static_cast<UnsignedT>(a) + static_cast<UnsignedT>(b));
}
template <typename T, std::enable_if_t<!NumTraits<T>::IsInteger || !NumTraits<T>::IsSigned, int> = 0>
inline T REF_SUB(const T& a, const T& b) {
  return a - b;
}
template <typename T, std::enable_if_t<NumTraits<T>::IsInteger && NumTraits<T>::IsSigned, int> = 0>
inline T REF_SUB(const T& a, const T& b) {
  using UnsignedT = std::make_unsigned_t<T>;
  return static_cast<T>(static_cast<UnsignedT>(a) - static_cast<UnsignedT>(b));
}
template <typename T, std::enable_if_t<!NumTraits<T>::IsInteger || std::is_same<T, bool>::value, int> = 0>
inline T REF_MUL(const T& a, const T& b) {
  return a * b;
}
template <typename T, std::enable_if_t<NumTraits<T>::IsInteger && !std::is_same<T, bool>::value, int> = 0>
inline T REF_MUL(const T& a, const T& b) {
  // Evaluate in an unsigned type at least as wide as int so that sub-int
  // operands are not promoted back to signed int (whose product can overflow);
  // the result then wraps modulo 2^bits just like pmul.
  using UnsignedT = std::common_type_t<std::make_unsigned_t<T>, unsigned>;
  return static_cast<T>(static_cast<UnsignedT>(a) * static_cast<UnsignedT>(b));
}

template <typename Scalar, typename EnableIf = void>
struct madd_impl {
  static EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE Scalar madd(const Scalar& a, const Scalar& b, const Scalar& c) {
    return a * b + c;
  }
  static EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE Scalar msub(const Scalar& a, const Scalar& b, const Scalar& c) {
    return a * b - c;
  }
  static EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE Scalar nmadd(const Scalar& a, const Scalar& b, const Scalar& c) {
    return c - a * b;
  }
  static EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE Scalar nmsub(const Scalar& a, const Scalar& b, const Scalar& c) {
    return Scalar(0) - (a * b + c);
  }
};

template <typename Scalar>
struct madd_impl<Scalar, std::enable_if_t<NumTraits<Scalar>::IsInteger && !std::is_same<Scalar, bool>::value>> {
  // Unsigned type at least as wide as int, so sub-int operands are not promoted
  // back to signed int (whose products/sums can overflow); results wrap modulo
  // 2^bits like the packet madd/msub ops.
  using UnsignedScalar = std::common_type_t<std::make_unsigned_t<Scalar>, unsigned>;

  static EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE Scalar madd(const Scalar& a, const Scalar& b, const Scalar& c) {
    return static_cast<Scalar>(static_cast<UnsignedScalar>(a) * static_cast<UnsignedScalar>(b) +
                               static_cast<UnsignedScalar>(c));
  }
  static EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE Scalar msub(const Scalar& a, const Scalar& b, const Scalar& c) {
    return static_cast<Scalar>(static_cast<UnsignedScalar>(a) * static_cast<UnsignedScalar>(b) -
                               static_cast<UnsignedScalar>(c));
  }
  static EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE Scalar nmadd(const Scalar& a, const Scalar& b, const Scalar& c) {
    return static_cast<Scalar>(static_cast<UnsignedScalar>(c) -
                               static_cast<UnsignedScalar>(a) * static_cast<UnsignedScalar>(b));
  }
  static EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE Scalar nmsub(const Scalar& a, const Scalar& b, const Scalar& c) {
    return static_cast<Scalar>(UnsignedScalar(0) - (static_cast<UnsignedScalar>(a) * static_cast<UnsignedScalar>(b) +
                                                    static_cast<UnsignedScalar>(c)));
  }
};

template <typename Scalar>
struct madd_impl<Scalar, std::enable_if_t<Eigen::internal::is_scalar<Scalar>::value &&
                                          Eigen::NumTraits<Scalar>::IsSigned && !NumTraits<Scalar>::IsInteger>> {
  static EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE Scalar madd(const Scalar& a, const Scalar& b, const Scalar& c) {
    return numext::madd(a, b, c);
  }
  static EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE Scalar msub(const Scalar& a, const Scalar& b, const Scalar& c) {
    return numext::madd(a, b, Scalar(-c));
  }
  static EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE Scalar nmadd(const Scalar& a, const Scalar& b, const Scalar& c) {
    return numext::madd(Scalar(-a), b, c);
  }
  static EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE Scalar nmsub(const Scalar& a, const Scalar& b, const Scalar& c) {
    return -Scalar(numext::madd(a, b, c));
  }
};

template <typename T>
inline T REF_MADD(const T& a, const T& b, const T& c) {
  return madd_impl<T>::madd(a, b, c);
}
template <typename T>
inline T REF_MSUB(const T& a, const T& b, const T& c) {
  return madd_impl<T>::msub(a, b, c);
}
template <typename T>
inline T REF_NMADD(const T& a, const T& b, const T& c) {
  return madd_impl<T>::nmadd(a, b, c);
}
template <typename T>
inline T REF_NMSUB(const T& a, const T& b, const T& c) {
  return madd_impl<T>::nmsub(a, b, c);
}
template <typename T>
inline T REF_DIV(const T& a, const T& b) {
  return a / b;
}
template <typename T>
inline T REF_RECIPROCAL(const T& a) {
  return T(1) / a;
}
template <typename T>
inline T REF_ABS_DIFF(const T& a, const T& b) {
  return a > b ? a - b : b - a;
}

template <typename Packet>
struct predux_reference_scalar {
  using Scalar = typename internal::unpacket_traits<Packet>::type;
  static constexpr bool WidensToFloat = std::is_same<Scalar, bfloat16>::value
#if defined(EIGEN_VECTORIZE_AVX) && !defined(EIGEN_VECTORIZE_AVX512FP16)
                                        || std::is_same<Packet, internal::Packet8h>::value
#endif
#if defined(EIGEN_VECTORIZE_AVX512) && !defined(EIGEN_VECTORIZE_AVX512FP16)
                                        || std::is_same<Packet, internal::Packet16h>::value
#endif
      ;
  using type = std::conditional_t<WidensToFloat, float, Scalar>;
};

template <typename Packet>
using predux_reference_scalar_t = typename predux_reference_scalar<Packet>::type;

// MacOS apple-clang has an issue with pcmp_eq for half when inlined,
// resulting in an ICE, but only in this specific test.
template <typename Packet>
EIGEN_DONT_INLINE Packet REF_PCMP_EQ(const Packet& a, const Packet& b) {
  return internal::pcmp_eq(a, b);
}

// Specializations for bool.
template <>
inline bool REF_ADD(const bool& a, const bool& b) {
  return a || b;
}
template <>
inline bool REF_SUB(const bool& a, const bool& b) {
  return a ^ b;
}
template <>
inline bool REF_MUL(const bool& a, const bool& b) {
  return a && b;
}
template <>
inline bool REF_MADD(const bool& a, const bool& b, const bool& c) {
  return (a && b) || c;
}
template <>
inline bool REF_DIV(const bool& a, const bool& b) {
  return a && b;
}
template <>
inline bool REF_RECIPROCAL(const bool& a) {
  return a;
}

template <typename T>
inline T REF_FREXP(const T& x, T& exp) {
  int iexp = 0;
  EIGEN_USING_STD(frexp)
  const T out = static_cast<T>(frexp(x, &iexp));
  exp = static_cast<T>(iexp);

  // The exponent value is unspecified if the input is inf or NaN, but MSVC
  // sets it to 1.  We need to set it back to zero for consistency.
  if (!(numext::isfinite)(x)) {
    exp = T(0);
  }
  return out;
}

template <typename T>
inline T REF_LDEXP(const T& x, const T& exp) {
  EIGEN_USING_STD(ldexp)
  return static_cast<T>(ldexp(x, static_cast<int>(exp)));
}

// provides a convenient function to take the absolute value of each component of a complex number to prevent
// catastrophic cancellation in randomly generated complex numbers
template <typename T, bool IsComplex = NumTraits<T>::IsComplex>
struct abs_helper_impl {
  static T run(T x) { return numext::abs(x); }
};
template <typename T>
struct abs_helper_impl<T, true> {
  static T run(T x) {
    T res = x;
    numext::real_ref(res) = numext::abs(numext::real(res));
    numext::imag_ref(res) = numext::abs(numext::imag(res));
    return res;
  }
};
template <typename T>
T abs_helper(T x) {
  return abs_helper_impl<T>::run(x);
}

// Uses pcast to cast from one array to another.
template <typename SrcPacket, typename TgtPacket, int SrcCoeffRatio, int TgtCoeffRatio>
struct pcast_array;

template <typename SrcPacket, typename TgtPacket, int TgtCoeffRatio>
struct pcast_array<SrcPacket, TgtPacket, 1, TgtCoeffRatio> {
  typedef typename internal::unpacket_traits<SrcPacket>::type SrcScalar;
  typedef typename internal::unpacket_traits<TgtPacket>::type TgtScalar;
  static void cast(const SrcScalar* src, size_t size, TgtScalar* dst) {
    static const int SrcPacketSize = internal::unpacket_traits<SrcPacket>::size;
    static const int TgtPacketSize = internal::unpacket_traits<TgtPacket>::size;
    size_t i;
    for (i = 0; i < size && i + SrcPacketSize <= size; i += TgtPacketSize) {
      internal::pstoreu(dst + i, internal::pcast<SrcPacket, TgtPacket>(internal::ploadu<SrcPacket>(src + i)));
    }
    // Leftovers that cannot be loaded into a packet.
    for (; i < size; ++i) {
      dst[i] = static_cast<TgtScalar>(src[i]);
    }
  }
};

template <typename SrcPacket, typename TgtPacket>
struct pcast_array<SrcPacket, TgtPacket, 2, 1> {
  static void cast(const typename internal::unpacket_traits<SrcPacket>::type* src, size_t size,
                   typename internal::unpacket_traits<TgtPacket>::type* dst) {
    static const int SrcPacketSize = internal::unpacket_traits<SrcPacket>::size;
    static const int TgtPacketSize = internal::unpacket_traits<TgtPacket>::size;
    for (size_t i = 0; i < size; i += TgtPacketSize) {
      SrcPacket a = internal::ploadu<SrcPacket>(src + i);
      SrcPacket b = internal::ploadu<SrcPacket>(src + i + SrcPacketSize);
      internal::pstoreu(dst + i, internal::pcast<SrcPacket, TgtPacket>(a, b));
    }
  }
};

template <typename SrcPacket, typename TgtPacket>
struct pcast_array<SrcPacket, TgtPacket, 4, 1> {
  static void cast(const typename internal::unpacket_traits<SrcPacket>::type* src, size_t size,
                   typename internal::unpacket_traits<TgtPacket>::type* dst) {
    static const int SrcPacketSize = internal::unpacket_traits<SrcPacket>::size;
    static const int TgtPacketSize = internal::unpacket_traits<TgtPacket>::size;
    for (size_t i = 0; i < size; i += TgtPacketSize) {
      SrcPacket a = internal::ploadu<SrcPacket>(src + i);
      SrcPacket b = internal::ploadu<SrcPacket>(src + i + SrcPacketSize);
      SrcPacket c = internal::ploadu<SrcPacket>(src + i + 2 * SrcPacketSize);
      SrcPacket d = internal::ploadu<SrcPacket>(src + i + 3 * SrcPacketSize);
      internal::pstoreu(dst + i, internal::pcast<SrcPacket, TgtPacket>(a, b, c, d));
    }
  }
};

template <typename SrcPacket, typename TgtPacket>
struct pcast_array<SrcPacket, TgtPacket, 8, 1> {
  static void cast(const typename internal::unpacket_traits<SrcPacket>::type* src, size_t size,
                   typename internal::unpacket_traits<TgtPacket>::type* dst) {
    static const int SrcPacketSize = internal::unpacket_traits<SrcPacket>::size;
    static const int TgtPacketSize = internal::unpacket_traits<TgtPacket>::size;
    for (size_t i = 0; i < size; i += TgtPacketSize) {
      SrcPacket a = internal::ploadu<SrcPacket>(src + i);
      SrcPacket b = internal::ploadu<SrcPacket>(src + i + SrcPacketSize);
      SrcPacket c = internal::ploadu<SrcPacket>(src + i + 2 * SrcPacketSize);
      SrcPacket d = internal::ploadu<SrcPacket>(src + i + 3 * SrcPacketSize);
      SrcPacket e = internal::ploadu<SrcPacket>(src + i + 4 * SrcPacketSize);
      SrcPacket f = internal::ploadu<SrcPacket>(src + i + 5 * SrcPacketSize);
      SrcPacket g = internal::ploadu<SrcPacket>(src + i + 6 * SrcPacketSize);
      SrcPacket h = internal::ploadu<SrcPacket>(src + i + 7 * SrcPacketSize);
      internal::pstoreu(dst + i, internal::pcast<SrcPacket, TgtPacket>(a, b, c, d, e, f, g, h));
    }
  }
};

template <typename SrcPacket, typename TgtPacket, int SrcCoeffRatio, int TgtCoeffRatio, bool CanCast = false>
struct test_cast_helper;

template <typename SrcPacket, typename TgtPacket, int SrcCoeffRatio, int TgtCoeffRatio>
struct test_cast_helper<SrcPacket, TgtPacket, SrcCoeffRatio, TgtCoeffRatio, false> {
  static void run() {}
};

template <typename SrcPacket, typename TgtPacket, int SrcCoeffRatio, int TgtCoeffRatio>
struct test_cast_helper<SrcPacket, TgtPacket, SrcCoeffRatio, TgtCoeffRatio, true> {
  static void run() {
    typedef typename internal::unpacket_traits<SrcPacket>::type SrcScalar;
    typedef typename internal::unpacket_traits<TgtPacket>::type TgtScalar;
    static const int SrcPacketSize = internal::unpacket_traits<SrcPacket>::size;
    static const int TgtPacketSize = internal::unpacket_traits<TgtPacket>::size;
    static const int BlockSize = SrcPacketSize * SrcCoeffRatio;
    eigen_assert(BlockSize == TgtPacketSize * TgtCoeffRatio && "Packet sizes and cast ratios are mismatched.");

    static const int DataSize = 10 * BlockSize;
    EIGEN_ALIGN_MAX SrcScalar data1[DataSize];
    EIGEN_ALIGN_MAX TgtScalar data2[DataSize];
    EIGEN_ALIGN_MAX TgtScalar ref[DataSize];

    // Construct a packet of scalars that will not overflow when casting
    for (int i = 0; i < DataSize; ++i) {
      data1[i] = internal::random_without_cast_overflow<SrcScalar, TgtScalar>::value();
    }

    for (int i = 0; i < DataSize; ++i) {
      ref[i] = static_cast<TgtScalar>(data1[i]);
    }

    pcast_array<SrcPacket, TgtPacket, SrcCoeffRatio, TgtCoeffRatio>::cast(data1, DataSize, data2);

    VERIFY(test::areApprox(ref, data2, DataSize) && "internal::pcast<>");

    // Test that pcast<SrcScalar, TgtScalar> generates the same result.
    for (int i = 0; i < DataSize; ++i) {
      data2[i] = internal::pcast<SrcScalar, TgtScalar>(data1[i]);
    }
    VERIFY(test::areApprox(ref, data2, DataSize) && "internal::pcast<>");
  }
};

template <typename SrcPacket, typename TgtPacket>
struct test_cast {
  static void run() {
    typedef typename internal::unpacket_traits<SrcPacket>::type SrcScalar;
    typedef typename internal::unpacket_traits<TgtPacket>::type TgtScalar;
    typedef typename internal::type_casting_traits<SrcScalar, TgtScalar> TypeCastingTraits;
    static const int SrcCoeffRatio = TypeCastingTraits::SrcCoeffRatio;
    static const int TgtCoeffRatio = TypeCastingTraits::TgtCoeffRatio;
    static const int SrcPacketSize = internal::unpacket_traits<SrcPacket>::size;
    static const int TgtPacketSize = internal::unpacket_traits<TgtPacket>::size;
    static const bool HasCast =
        internal::unpacket_traits<SrcPacket>::vectorizable && internal::unpacket_traits<TgtPacket>::vectorizable &&
        TypeCastingTraits::VectorizedCast && (SrcPacketSize * SrcCoeffRatio == TgtPacketSize * TgtCoeffRatio);
    test_cast_helper<SrcPacket, TgtPacket, SrcCoeffRatio, TgtCoeffRatio, HasCast>::run();
  }
};

template <typename SrcPacket, typename TgtScalar,
          typename TgtPacket = typename internal::packet_traits<TgtScalar>::type,
          bool Vectorized = internal::packet_traits<TgtScalar>::Vectorizable,
          bool HasHalf = !std::is_same<typename internal::unpacket_traits<TgtPacket>::half, TgtPacket>::value>
struct test_cast_runner;

template <typename SrcPacket, typename TgtScalar, typename TgtPacket>
struct test_cast_runner<SrcPacket, TgtScalar, TgtPacket, true, false> {
  static void run() { test_cast<SrcPacket, TgtPacket>::run(); }
};

template <typename SrcPacket, typename TgtScalar, typename TgtPacket>
struct test_cast_runner<SrcPacket, TgtScalar, TgtPacket, true, true> {
  static void run() {
    test_cast<SrcPacket, TgtPacket>::run();
    test_cast_runner<SrcPacket, TgtScalar, typename internal::unpacket_traits<TgtPacket>::half>::run();
  }
};

template <typename SrcPacket, typename TgtScalar, typename TgtPacket>
struct test_cast_runner<SrcPacket, TgtScalar, TgtPacket, false, false> {
  static void run() {}
};

template <typename Scalar, typename Packet, typename EnableIf = void>
struct packetmath_pcast_ops_runner {
  static void run() {
    test_cast_runner<Packet, float>::run();
    test_cast_runner<Packet, double>::run();
    test_cast_runner<Packet, int8_t>::run();
    test_cast_runner<Packet, uint8_t>::run();
    test_cast_runner<Packet, int16_t>::run();
    test_cast_runner<Packet, uint16_t>::run();
    test_cast_runner<Packet, int32_t>::run();
    test_cast_runner<Packet, uint32_t>::run();
    test_cast_runner<Packet, int64_t>::run();
    test_cast_runner<Packet, uint64_t>::run();
    test_cast_runner<Packet, bool>::run();
    test_cast_runner<Packet, std::complex<float>>::run();
    test_cast_runner<Packet, std::complex<double>>::run();
    test_cast_runner<Packet, half>::run();
    test_cast_runner<Packet, bfloat16>::run();
  }
};

// Only some types support cast from std::complex<>.
template <typename Scalar, typename Packet>
struct packetmath_pcast_ops_runner<Scalar, Packet, std::enable_if_t<NumTraits<Scalar>::IsComplex>> {
  static void run() {
    test_cast_runner<Packet, std::complex<float>>::run();
    test_cast_runner<Packet, std::complex<double>>::run();
    test_cast_runner<Packet, half>::run();
    test_cast_runner<Packet, bfloat16>::run();
  }
};

template <typename Scalar, typename Packet>
void packetmath_boolean_mask_ops() {
  using RealScalar = typename NumTraits<Scalar>::Real;
  const int PacketSize = internal::unpacket_traits<Packet>::size;
  const int size = 2 * PacketSize;
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data1[size];
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data2[size];
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar ref[size];

  for (int i = 0; i < size; ++i) {
    data1[i] = internal::random<Scalar>();
  }
  CHECK_CWISE1_MASK(internal::ptrue, internal::ptrue);
  CHECK_CWISE2_IF(true, internal::pandnot, internal::pandnot);
  for (int i = 0; i < PacketSize; ++i) {
    data1[i] = Scalar(RealScalar(i));
    data1[i + PacketSize] = internal::random<bool>() ? data1[i] : Scalar(0);
  }

  CHECK_CWISE2_MASK(REF_PCMP_EQ, internal::pcmp_eq);

  // Test (-0) == (0) for signed operations
  for (int i = 0; i < PacketSize; ++i) {
    data1[i] = Scalar(-0.0);
    data1[i + PacketSize] = internal::random<bool>() ? data1[i] : Scalar(0);
  }
  CHECK_CWISE2_MASK(REF_PCMP_EQ, internal::pcmp_eq);

  // Test NaN
  for (int i = 0; i < PacketSize; ++i) {
    data1[i] = NumTraits<Scalar>::quiet_NaN();
    data1[i + PacketSize] = internal::random<bool>() ? data1[i] : Scalar(0);
  }
  CHECK_CWISE2_MASK(REF_PCMP_EQ, internal::pcmp_eq);
}

template <typename Scalar, typename Packet>
void packetmath_boolean_mask_ops_real() {
  const int PacketSize = internal::unpacket_traits<Packet>::size;
  const int size = 2 * PacketSize;
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data1[size];
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data2[size];

  for (int i = 0; i < PacketSize; ++i) {
    data1[i] = internal::random<Scalar>();
    data1[i + PacketSize] = internal::random<bool>() ? data1[i] : Scalar(0);
  }

  CHECK_CWISE2_MASK(internal::pcmp_lt_or_nan, internal::pcmp_lt_or_nan);

  // Test (-0) <=/< (0) for signed operations
  for (int i = 0; i < PacketSize; ++i) {
    data1[i] = Scalar(-0.0);
    data1[i + PacketSize] = internal::random<bool>() ? data1[i] : Scalar(0);
  }
  CHECK_CWISE2_MASK(internal::pcmp_lt_or_nan, internal::pcmp_lt_or_nan);

  // Test NaN
  for (int i = 0; i < PacketSize; ++i) {
    data1[i] = NumTraits<Scalar>::quiet_NaN();
    data1[i + PacketSize] = internal::random<bool>() ? data1[i] : Scalar(0);
  }
  CHECK_CWISE2_MASK(internal::pcmp_lt_or_nan, internal::pcmp_lt_or_nan);
}

template <typename Scalar, typename Packet, typename EnableIf = void>
struct packetmath_boolean_mask_ops_notcomplex_test {
  static void run() {}
};

template <typename Scalar, typename Packet>
struct packetmath_boolean_mask_ops_notcomplex_test<
    Scalar, Packet, std::enable_if_t<internal::packet_traits<Scalar>::HasCmp && !std::is_same<Scalar, bool>::value>> {
  static void run() {
    const int PacketSize = internal::unpacket_traits<Packet>::size;
    const int size = 2 * PacketSize;
    EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data1[size];
    EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data2[size];

    for (int i = 0; i < PacketSize; ++i) {
      data1[i] = internal::random<Scalar>();
      data1[i + PacketSize] = internal::random<bool>() ? data1[i] : Scalar(0);
    }

    CHECK_CWISE2_MASK(internal::pcmp_le, internal::pcmp_le);
    CHECK_CWISE2_MASK(internal::pcmp_lt, internal::pcmp_lt);

    // Test (-0) <=/< (0) for signed operations
    for (int i = 0; i < PacketSize; ++i) {
      data1[i] = Scalar(-0.0);
      data1[i + PacketSize] = internal::random<bool>() ? data1[i] : Scalar(0);
    }
    CHECK_CWISE2_MASK(internal::pcmp_le, internal::pcmp_le);
    CHECK_CWISE2_MASK(internal::pcmp_lt, internal::pcmp_lt);

    // Test NaN
    for (int i = 0; i < PacketSize; ++i) {
      data1[i] = NumTraits<Scalar>::quiet_NaN();
      data1[i + PacketSize] = internal::random<bool>() ? data1[i] : Scalar(0);
    }
    CHECK_CWISE2_MASK(internal::pcmp_le, internal::pcmp_le);
    CHECK_CWISE2_MASK(internal::pcmp_lt, internal::pcmp_lt);
  }
};

template <typename Scalar, typename Packet, typename EnableIf = void>
struct packetmath_split_half_compare_test {
  static void run() {}
};

// An emulated wide compare that combines per-half compares lexicographically is correct only if the
// low half is ordered as unsigned. Operands sharing a high half are what reach that path, and
// packetmath_boolean_mask_ops_notcomplex_test builds none: it pairs each value with itself or zero.
template <typename Scalar, typename Packet>
struct packetmath_split_half_compare_test<
    Scalar, Packet,
    std::enable_if_t<std::is_integral<Scalar>::value && internal::packet_traits<Scalar>::HasCmp &&
                     !std::is_same<Scalar, bool>::value>> {
  // Class scope keeps these usable as array bounds inside a lambda, which MSVC otherwise treats as
  // captured and therefore non-constant.
  static constexpr int PacketSize = internal::unpacket_traits<Packet>::size;
  static constexpr int size = 2 * PacketSize;

  static void run() {
    using Unsigned = std::make_unsigned_t<Scalar>;
    constexpr int kHalfBits = 4 * int(sizeof(Scalar));
    constexpr Unsigned kHalfSignBit = Unsigned(Unsigned(1) << (kHalfBits - 1));
    constexpr Unsigned kLowMask = Unsigned(Unsigned(kHalfSignBit << 1) - Unsigned(1));

    const Unsigned low_parts[] = {Unsigned(0), Unsigned(1), Unsigned(kHalfSignBit - Unsigned(1)), kHalfSignBit,
                                  kLowMask};
    constexpr int kNumLow = int(sizeof(low_parts) / sizeof(low_parts[0]));
    // The second high half sets the lane's own sign bit, exercising the high compare for signed and
    // unsigned Scalar alike.
    const Unsigned high_parts[] = {Unsigned(0), kLowMask};
    constexpr int kNumHigh = int(sizeof(high_parts) / sizeof(high_parts[0]));

    EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data1[size];
    EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data2[size];
    EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar ref[size];

    const auto compose = [&](int high_index, int low_index) {
      return Scalar(Unsigned(Unsigned(high_parts[high_index] << kHalfBits) | low_parts[low_index]));
    };

    for (int high_index = 0; high_index < kNumHigh; ++high_index) {
      for (int lhs_low_index = 0; lhs_low_index < kNumLow; ++lhs_low_index) {
        for (int rhs_low_index = 0; rhs_low_index < kNumLow; ++rhs_low_index) {
          for (int lane = 0; lane < PacketSize; ++lane) {
            data1[lane] = compose(high_index, lhs_low_index);
            data1[lane + PacketSize] = compose(high_index, rhs_low_index);
          }
          CHECK_CWISE2_MASK(internal::pcmp_le, internal::pcmp_le);
          CHECK_CWISE2_MASK(internal::pcmp_lt, internal::pcmp_lt);
          CHECK_CWISE2_MASK(REF_PCMP_EQ, internal::pcmp_eq);
          CHECK_CWISE2_IF(internal::packet_traits<Scalar>::HasMin, (std::min), internal::pmin);
          CHECK_CWISE2_IF(internal::packet_traits<Scalar>::HasMax, (std::max), internal::pmax);
          CHECK_CWISE2_IF(internal::packet_traits<Scalar>::HasAbsDiff, REF_ABS_DIFF, internal::pabsdiff);
        }
      }
    }
  }
};

template <typename Scalar, typename Packet, typename EnableIf = void>
struct packetmath_minus_zero_add_test {
  static void run() {}
};

template <typename Scalar, typename Packet>
struct packetmath_minus_zero_add_test<Scalar, Packet, std::enable_if_t<!NumTraits<Scalar>::IsInteger>> {
  static void run() {
    const int PacketSize = internal::unpacket_traits<Packet>::size;
    const int size = 2 * PacketSize;
    EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data1[size] = {};
    EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data2[size] = {};
    EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar ref[size] = {};

    for (int i = 0; i < PacketSize; ++i) {
      data1[i] = Scalar(-0.0);
      data1[i + PacketSize] = Scalar(-0.0);
    }
    CHECK_CWISE2_IF(internal::packet_traits<Scalar>::HasAdd, REF_ADD, internal::padd);
  }
};

template <typename Scalar, typename Packet, typename EnableIf = void>
struct packetmath_integer_predicates_test {
  static void run() {}
};

// Integer scalars have no NaN or infinity: pisnan/pisinf must be all-false and pisfinite
// all-true for every input, including |a| == 2^(digits-1), whose bit pattern matches the
// constant synthesized by pinf<Packet>().
template <typename Scalar, typename Packet>
struct packetmath_integer_predicates_test<
    Scalar, Packet, std::enable_if_t<NumTraits<Scalar>::IsInteger && !std::is_same<Scalar, bool>::value>> {
  static void run() {
    const int PacketSize = internal::unpacket_traits<Packet>::size;
    EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data[PacketSize];
    EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar res[PacketSize];
    // "True" is Scalar(1) in the scalar mask convention and all-ones bits in the packet one;
    // ptrue of the tested Packet type yields the right one either way (the runner also
    // instantiates Packet = Scalar).
    const Scalar scalar_true = internal::ptrue(Scalar(0));
    EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar lane_true[PacketSize];
    internal::pstore(lane_true, internal::ptrue(internal::pset1<Packet>(Scalar(0))));
    const Scalar values[] = {Scalar(0),
                             Scalar(1),
                             static_cast<Scalar>(-1),
                             Scalar(Scalar(1) << (std::numeric_limits<Scalar>::digits - 1)),
                             NumTraits<Scalar>::highest(),
                             NumTraits<Scalar>::lowest()};
    const int num_values = sizeof(values) / sizeof(values[0]);
    for (int i = 0; i < num_values; ++i) {
      VERIFY(numext::is_exactly_zero(internal::pisnan(values[i])) && "scalar integer pisnan");
      VERIFY(numext::is_exactly_zero(internal::pisinf(values[i])) && "scalar integer pisinf");
      VERIFY(internal::pisfinite(values[i]) == scalar_true && "scalar integer pisfinite");
    }
    for (int i = 0; i < PacketSize; ++i) data[i] = values[i % num_values];
    internal::pstore(res, internal::pisnan(internal::pload<Packet>(data)));
    for (int i = 0; i < PacketSize; ++i) VERIFY(numext::is_exactly_zero(res[i]) && "integer pisnan");
    internal::pstore(res, internal::pisinf(internal::pload<Packet>(data)));
    for (int i = 0; i < PacketSize; ++i) VERIFY(numext::is_exactly_zero(res[i]) && "integer pisinf");
    internal::pstore(res, internal::pisfinite(internal::pload<Packet>(data)));
    for (int i = 0; i < PacketSize; ++i) VERIFY(res[i] == lane_true[i] && "integer pisfinite");
  }
};

template <typename Scalar, typename Packet, typename = void>
struct packetmath_64bit_boundary_test {
  static void run() {}
};

// Focused coverage for 64-bit lanes: non-ARM64 `pcmp_eq<Packet2{,u}l>` splits each lane into 32-bit
// halves and `AND`s the two half-comparisons together. The generic `packetmath_boolean_mask_ops`
// only feeds 0/1 values, whose high half is always zero, so a broken half-pairing/`AND` there can
// go undetected. Here lanes vary only the high half, only the low half, or neither, and boundary
// values cross the 2^32 seam.
template <typename Scalar, typename Packet>
struct packetmath_64bit_boundary_test<Scalar, Packet,
                                      std::enable_if_t<NumTraits<Scalar>::IsInteger && sizeof(Scalar) == 8>> {
  static constexpr int PacketSize = unpacket_traits<Packet>::size;
  static constexpr int size = 2 * PacketSize;

  static void run() {
    EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data1[size];
    EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data2[size];
    EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar ref[size];

    const auto ref_abs = [](const Scalar& x) { return x < Scalar(0) ? test::negate(x) : x; };
    const auto check_ops = [&] {
      CHECK_CWISE2_MASK(REF_PCMP_EQ, internal::pcmp_eq);
      CHECK_CWISE2_MASK(internal::pcmp_lt, internal::pcmp_lt);
      CHECK_CWISE2_MASK(internal::pcmp_le, internal::pcmp_le);
      CHECK_CWISE2_IF(internal::packet_traits<Scalar>::HasMin, (std::min), internal::pmin);
      CHECK_CWISE2_IF(internal::packet_traits<Scalar>::HasMax, (std::max), internal::pmax);
      CHECK_CWISE2_IF(internal::packet_traits<Scalar>::HasMul, REF_MUL, internal::pmul);
      CHECK_CWISE1_IF(internal::packet_traits<Scalar>::HasNegate, test::negate, internal::pnegate);
      CHECK_CWISE1(ref_abs, internal::pabs);
    };

    constexpr Scalar high = 0x11111111;
    constexpr Scalar low = 0x00000001;
    constexpr Scalar reference = (high << 32) | low;
    constexpr Scalar high_shifted = (high << 33) | low;
    constexpr Scalar low_shifted = (high << 32) | (low << 1);

    static constexpr Scalar half_lanes[] = {high_shifted, low_shifted, reference};
    constexpr int half_lanes_count = sizeof(half_lanes) / sizeof(half_lanes[0]);

    constexpr int half_lanes_chunks = numext::div_ceil(half_lanes_count, PacketSize);
    for (int chunk = 0; chunk < half_lanes_chunks; ++chunk) {
      Map<ArrayX<Scalar>>(data1, PacketSize).setConstant(reference);
      for (int i = 0; i < PacketSize; ++i)
        data1[i + PacketSize] = half_lanes[(chunk * PacketSize + i) % half_lanes_count];
      check_ops();

      for (int i = 0; i < PacketSize; ++i) std::swap(data1[i], data1[i + PacketSize]);
      check_ops();
    }

    const auto from_bits = [](unsigned long long bits) { return numext::bit_cast<Scalar>(bits); };
    const Scalar boundary_values[] = {
        Scalar(0),
        Scalar(1),
        from_bits(0xFFFFFFFFFFFFFFFFull),  // -1 (signed) / UINT64_MAX (unsigned)
        from_bits(0x8000000000000000ull),  // INT64_MIN (signed) / 2^63 (unsigned)
        from_bits(0x7FFFFFFFFFFFFFFFull),  // INT64_MAX
        from_bits(0x00000000FFFFFFFFull),  // 2^32 - 1
        from_bits(0x0000000100000000ull),  // 2^32
        from_bits(0x00000001FFFFFFFFull),  // 2^33 - 1
    };
    constexpr int num_boundary = sizeof(boundary_values) / sizeof(boundary_values[0]);

    // Test every distinct pair of `boundary_values` entries against each other.  Broadcast each
    // pair across all lanes; lane-position coverage is already exercised above and in the
    // self-value sweep below.
    for (int i = 0; i < num_boundary; ++i) {
      for (int j = i + 1; j < num_boundary; ++j) {
        for (int k = 0; k < PacketSize; ++k) {
          data1[k] = boundary_values[i];
          data1[k + PacketSize] = boundary_values[j];
        }
        check_ops();

        for (int k = 0; k < PacketSize; ++k) std::swap(data1[k], data1[k + PacketSize]);
        check_ops();
      }
    }

    constexpr int num_self_chunks = numext::div_ceil(num_boundary, PacketSize);
    for (int chunk = 0; chunk < num_self_chunks; ++chunk) {
      for (int i = 0; i < PacketSize; ++i) {
        const int idx = (chunk * PacketSize + i) % num_boundary;
        data1[i] = data1[i + PacketSize] = boundary_values[idx];
      }
      check_ops();
    }
  }
};

// Ensure optimization barrier compiles and doesn't modify contents.
// Only applies to raw types, so will not work for std::complex, Eigen::half
// or Eigen::bfloat16. For those you would need to refer to an underlying
// storage element.
template <typename Packet, typename EnableIf = void>
struct eigen_optimization_barrier_test {
  static void run() {}
};

template <typename Packet>
struct eigen_optimization_barrier_test<
    Packet, std::enable_if_t<!NumTraits<Packet>::IsComplex && !std::is_same<Packet, Eigen::half>::value &&
                             !std::is_same<Packet, Eigen::bfloat16>::value>> {
  static void run() {
    typedef typename internal::unpacket_traits<Packet>::type Scalar;
    Scalar s = internal::random<Scalar>();
    Packet barrier = internal::pset1<Packet>(s);
    EIGEN_OPTIMIZATION_BARRIER(barrier);
    eigen_assert(s == internal::pfirst(barrier) && "EIGEN_OPTIMIZATION_BARRIER");
  }
};

template <typename Scalar, typename Packet, bool HasNegate = internal::packet_traits<Scalar>::HasNegate>
struct negate_test_impl {
  static void run_negate(Scalar* data1, Scalar* data2, Scalar* ref, int PacketSize) {
    CHECK_CWISE1_IF(HasNegate, test::negate, internal::pnegate);
  }
  static void run_nmsub(Scalar* data1, Scalar* data2, Scalar* ref, int PacketSize) {
    CHECK_CWISE3_IF(HasNegate, REF_NMSUB, internal::pnmsub);
  }
};

template <typename Scalar, typename Packet>
struct negate_test_impl<Scalar, Packet, false> {
  static void run_negate(Scalar*, Scalar*, Scalar*, int) {}
  static void run_nmsub(Scalar*, Scalar*, Scalar*, int) {}
};

template <typename Scalar, typename Packet>
void negate_test(Scalar* data1, Scalar* data2, Scalar* ref, int size) {
  negate_test_impl<Scalar, Packet>::run_negate(data1, data2, ref, size);
}

template <typename Scalar, typename Packet>
void nmsub_test(Scalar* data1, Scalar* data2, Scalar* ref, int size) {
  negate_test_impl<Scalar, Packet>::run_nmsub(data1, data2, ref, size);
}

template <typename Scalar, typename Packet>
void packetmath() {
  typedef internal::packet_traits<Scalar> PacketTraits;
  const int PacketSize = internal::unpacket_traits<Packet>::size;
  typedef typename NumTraits<Scalar>::Real RealScalar;

  if (g_first_pass)
    std::cerr << "=== Testing packet of type '" << typeid(Packet).name() << "' and scalar type '"
              << typeid(Scalar).name() << "' and size '" << PacketSize << "' ===\n";

  constexpr int max_size = PacketSize > 4 ? PacketSize : 4;
  const int size = PacketSize * max_size;
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data1[size];
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data2[size];
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data3[size];
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar ref[size];
  RealScalar refvalue = RealScalar(0);

  eigen_optimization_barrier_test<Packet>::run();
  eigen_optimization_barrier_test<Scalar>::run();

  for (int i = 0; i < size; ++i) {
    data1[i] = internal::random<Scalar>();
    data2[i] = internal::random<Scalar>();
    refvalue = (std::max)(refvalue, numext::abs(data1[i]));
  }

  internal::pstore(data2, internal::pload<Packet>(data1));
  VERIFY(test::areApprox(data1, data2, PacketSize) && "aligned load/store");

  for (int offset = 0; offset < PacketSize; ++offset) {
    internal::pstore(data2, internal::ploadu<Packet>(data1 + offset));
    VERIFY(test::areApprox(data1 + offset, data2, PacketSize) && "internal::ploadu");
  }

  for (int offset = 0; offset < PacketSize; ++offset) {
    internal::pstoreu(data2 + offset, internal::pload<Packet>(data1));
    VERIFY(test::areApprox(data1, data2 + offset, PacketSize) && "internal::pstoreu");
  }

  for (int M = 0; M < PacketSize; ++M) {
    for (int N = 0; N <= PacketSize; ++N) {
      for (int j = 0; j < size; ++j) {
        data1[j] = internal::random<Scalar>();
        data2[j] = internal::random<Scalar>();
        refvalue = (std::max)(refvalue, numext::abs(data1[j]));
      }

      if (M == 0) {
        internal::pstore_partial(data2, internal::pload_partial<Packet>(data1, N), N);
        VERIFY(test::areApprox(data1, data2, N) && "aligned loadN/storeN");

        for (int offset = 0; offset < PacketSize; ++offset) {
          internal::pstore_partial(data2, internal::ploadu_partial<Packet>(data1 + offset, N), N);
          VERIFY(test::areApprox(data1 + offset, data2, N) && "internal::ploadu_partial");
        }

        for (int offset = 0; offset < PacketSize; ++offset) {
          internal::pstoreu_partial(data2 + offset, internal::pload_partial<Packet>(data1, N), N);
          VERIFY(test::areApprox(data1, data2 + offset, N) && "internal::pstoreu_partial");
        }
      }

      if (N + M > PacketSize) continue;  // Don't read or write past end of Packet

      internal::pstore_partial(data2, internal::pload_partial<Packet>(data1, N, M), N, M);
      VERIFY(test::areApprox(data1, data2, N) && "aligned offset loadN/storeN");
    }
  }

  if (internal::unpacket_traits<Packet>::masked_load_available) {
    test::packet_helper<internal::unpacket_traits<Packet>::masked_load_available, Packet> h;
    unsigned long long max_umask = (0x1ull << PacketSize);

    for (int offset = 0; offset < PacketSize; ++offset) {
      for (unsigned long long umask = 0; umask < max_umask; ++umask) {
        h.store(data2, h.load(data1 + offset, umask));
        for (int k = 0; k < PacketSize; ++k) data3[k] = ((umask & (0x1ull << k)) >> k) ? data1[k + offset] : Scalar(0);
        VERIFY(test::areApprox(data3, data2, PacketSize) && "internal::ploadu masked");
      }
    }
  }

  if (internal::unpacket_traits<Packet>::masked_store_available) {
    test::packet_helper<internal::unpacket_traits<Packet>::masked_store_available, Packet> h;
    unsigned long long max_umask = (0x1ull << PacketSize);

    for (int offset = 0; offset < PacketSize; ++offset) {
      for (unsigned long long umask = 0; umask < max_umask; ++umask) {
        internal::pstore(data2, internal::pset1<Packet>(Scalar(0)));
        h.store(data2, h.loadu(data1 + offset), umask);
        for (int k = 0; k < PacketSize; ++k) data3[k] = ((umask & (0x1ull << k)) >> k) ? data1[k + offset] : Scalar(0);
        VERIFY(test::areApprox(data3, data2, PacketSize) && "internal::pstoreu masked");
      }
    }
  }

  VERIFY((!PacketTraits::Vectorizable) || PacketTraits::HasAdd);
  VERIFY((!PacketTraits::Vectorizable) || PacketTraits::HasSub);
  VERIFY((!PacketTraits::Vectorizable) || PacketTraits::HasMul);

  CHECK_CWISE2_IF(PacketTraits::HasAdd, REF_ADD, internal::padd);
  CHECK_CWISE2_IF(PacketTraits::HasSub, REF_SUB, internal::psub);
  CHECK_CWISE2_IF(PacketTraits::HasMul, REF_MUL, internal::pmul);
  CHECK_CWISE2_IF(PacketTraits::HasDiv, REF_DIV, internal::pdiv);

  negate_test<Scalar, Packet>(data1, data2, ref, PacketSize);
  CHECK_CWISE1_IF(PacketTraits::HasReciprocal, REF_RECIPROCAL, internal::preciprocal);
  CHECK_CWISE1(numext::conj, internal::pconj);

  CHECK_CWISE1_IF(PacketTraits::HasSign, numext::sign, internal::psign);

  for (int offset = 0; offset < 3; ++offset) {
    for (int i = 0; i < PacketSize; ++i) ref[i] = data1[offset];
    internal::pstore(data2, internal::pset1<Packet>(data1[offset]));
    VERIFY(test::areApprox(ref, data2, PacketSize) && "internal::pset1");
  }

  {
    for (int i = 0; i < PacketSize * 4; ++i) ref[i] = data1[i / PacketSize];
    Packet A0, A1, A2, A3;
    internal::pbroadcast4<Packet>(data1, A0, A1, A2, A3);
    internal::pstore(data2 + 0 * PacketSize, A0);
    internal::pstore(data2 + 1 * PacketSize, A1);
    internal::pstore(data2 + 2 * PacketSize, A2);
    internal::pstore(data2 + 3 * PacketSize, A3);
    VERIFY(test::areApprox(ref, data2, 4 * PacketSize) && "internal::pbroadcast4");
  }

  {
    for (int i = 0; i < PacketSize * 2; ++i) ref[i] = data1[i / PacketSize];
    Packet A0, A1;
    internal::pbroadcast2<Packet>(data1, A0, A1);
    internal::pstore(data2 + 0 * PacketSize, A0);
    internal::pstore(data2 + 1 * PacketSize, A1);
    VERIFY(test::areApprox(ref, data2, 2 * PacketSize) && "internal::pbroadcast2");
  }

  VERIFY(internal::isApprox(data1[0], internal::pfirst(internal::pload<Packet>(data1))) && "internal::pfirst");

  if (PacketSize > 1) {
    // apply different offsets to check that ploaddup is robust to unaligned inputs
    for (int offset = 0; offset < 4; ++offset) {
      for (int i = 0; i < PacketSize / 2; ++i) ref[2 * i + 0] = ref[2 * i + 1] = data1[offset + i];
      internal::pstore(data2, internal::ploaddup<Packet>(data1 + offset));
      VERIFY(test::areApprox(ref, data2, PacketSize) && "ploaddup");
    }
  }

  if (PacketSize > 2) {
    // apply different offsets to check that ploadquad is robust to unaligned inputs
    for (int offset = 0; offset < 4; ++offset) {
      for (int i = 0; i < PacketSize / 4; ++i)
        ref[4 * i + 0] = ref[4 * i + 1] = ref[4 * i + 2] = ref[4 * i + 3] = data1[offset + i];
      internal::pstore(data2, internal::ploadquad<Packet>(data1 + offset));
      VERIFY(test::areApprox(ref, data2, PacketSize) && "ploadquad");
    }
  }

  // Match the packet reduction's accumulation precision. REF_ADD also preserves signed wrapping and Boolean OR
  // semantics and avoids MSVC C4804 for raw Boolean addition.
  using ReduxReferenceScalar = predux_reference_scalar_t<Packet>;
  ReduxReferenceScalar redux_ref(0);
  for (int i = 0; i < PacketSize; ++i) redux_ref = REF_ADD(redux_ref, static_cast<ReduxReferenceScalar>(data1[i]));
  ref[0] = static_cast<Scalar>(redux_ref);
  VERIFY(test::isApproxAbs(ref[0], internal::predux(internal::pload<Packet>(data1)), refvalue) && "internal::predux");

  if (!std::is_same<Packet, typename internal::unpacket_traits<Packet>::half>::value) {
    int HalfPacketSize = PacketSize > 4 ? PacketSize / 2 : PacketSize;
    for (int i = 0; i < HalfPacketSize; ++i) ref[i] = Scalar(0);
    for (int i = 0; i < PacketSize; ++i) ref[i % HalfPacketSize] = REF_ADD(ref[i % HalfPacketSize], data1[i]);
    internal::pstore(data2, internal::predux_half(internal::pload<Packet>(data1)));
    VERIFY(test::areApprox(ref, data2, HalfPacketSize) && "internal::predux_half");
  }

  // Avoid overflows.
  if (NumTraits<Scalar>::IsInteger && NumTraits<Scalar>::IsSigned &&
      Eigen::internal::unpacket_traits<Packet>::size > 1) {
    Scalar limit = static_cast<Scalar>(
        static_cast<RealScalar>(std::pow(static_cast<double>(numext::real(NumTraits<Scalar>::highest())),
                                         1.0 / static_cast<double>(Eigen::internal::unpacket_traits<Packet>::size))));
    for (int i = 0; i < PacketSize; ++i) {
      data1[i] = internal::random<Scalar>(Scalar(0) - limit, limit);
    }
  } else if (!NumTraits<Scalar>::IsInteger && !NumTraits<Scalar>::IsComplex && !std::is_same<Scalar, bool>::value) {
    // Prevent very small product results by adjusting range.  Otherwise,
    // we may end up with multiplying e.g. 32 Eigen::halfs with values < 1.
    for (int i = 0; i < PacketSize; ++i) {
      data1[i] = REF_MUL(internal::random<Scalar>(Scalar(0.5), Scalar(1)),
                         (internal::random<bool>() ? Scalar(-1) : Scalar(1)));
    }
  }
  ref[0] = Scalar(1);
  for (int i = 0; i < PacketSize; ++i) ref[0] = REF_MUL(ref[0], data1[i]);
  VERIFY(internal::isApprox(ref[0], internal::predux_mul(internal::pload<Packet>(data1))) && "internal::predux_mul");

  for (int i = 0; i < PacketSize; ++i) ref[i] = data1[PacketSize - i - 1];
  internal::pstore(data2, internal::preverse(internal::pload<Packet>(data1)));
  VERIFY(test::areApprox(ref, data2, PacketSize) && "internal::preverse");

  internal::PacketBlock<Packet> kernel;
  for (int i = 0; i < PacketSize; ++i) {
    kernel.packet[i] = internal::pload<Packet>(data1 + i * PacketSize);
  }
  ptranspose(kernel);
  for (int i = 0; i < PacketSize; ++i) {
    internal::pstore(data2, kernel.packet[i]);
    for (int j = 0; j < PacketSize; ++j) {
      VERIFY(test::isApproxAbs(data2[j], data1[i + j * PacketSize], refvalue) && "ptranspose");
    }
  }

  // GeneralBlockPanelKernel also checks PacketBlock<Packet,(PacketSize%4)==0?4:PacketSize>;
  if (PacketSize > 4 && PacketSize % 4 == 0) {
    internal::PacketBlock<Packet, PacketSize % 4 == 0 ? 4 : PacketSize> kernel2;
    for (int i = 0; i < 4; ++i) {
      kernel2.packet[i] = internal::pload<Packet>(data1 + i * PacketSize);
    }
    ptranspose(kernel2);
    int data_counter = 0;
    for (int i = 0; i < PacketSize; ++i) {
      for (int j = 0; j < 4; ++j) {
        data2[data_counter++] = data1[j * PacketSize + i];
      }
    }
    for (int i = 0; i < 4; ++i) {
      internal::pstore(data3, kernel2.packet[i]);
      for (int j = 0; j < PacketSize; ++j) {
        VERIFY(test::isApproxAbs(data3[j], data2[i * PacketSize + j], refvalue) && "ptranspose");
      }
    }
  }

  {
    for (int i = 0; i < PacketSize; ++i) {
      // "if" mask
      // Note: it's UB to load 0xFF directly into a `bool`.
      uint8_t v =
          internal::random<bool>() ? (std::is_same<Scalar, bool>::value ? static_cast<uint8_t>(true) : 0xff) : 0;
      // Avoid strict aliasing violation by using memset.
      memset(static_cast<void*>(data1 + i), v, sizeof(Scalar));
      // "then" packet
      data1[i + PacketSize] = internal::random<Scalar>();
      // "else" packet
      data1[i + 2 * PacketSize] = internal::random<Scalar>();
    }
    CHECK_CWISE3_IF(true, internal::pselect, internal::pselect);
  }

  for (int i = 0; i < size; ++i) {
    data1[i] = internal::random<Scalar>();
  }
  CHECK_CWISE1(internal::pzero, internal::pzero);
  CHECK_CWISE2_IF(true, internal::por, internal::por);
  CHECK_CWISE2_IF(true, internal::pxor, internal::pxor);
  CHECK_CWISE2_IF(true, internal::pand, internal::pand);

  packetmath_boolean_mask_ops<Scalar, Packet>();
  packetmath_pcast_ops_runner<Scalar, Packet>::run();
  packetmath_minus_zero_add_test<Scalar, Packet>::run();
  packetmath_integer_predicates_test<Scalar, Packet>::run();
  packetmath_64bit_boundary_test<Scalar, Packet>::run();

  CHECK_CWISE3_IF(true, REF_MADD, internal::pmadd);
  if (!std::is_same<Scalar, bool>::value && NumTraits<Scalar>::IsSigned) {
    nmsub_test<Scalar, Packet>(data1, data2, ref, PacketSize);
  }

  // For pmsub, pnmadd, the values can cancel each other to become near zero,
  // which can lead to very flaky tests. Here we ensure the signs are such that
  // they do not cancel.
  for (int i = 0; i < PacketSize; ++i) {
    data1[i] = abs_helper(internal::random<Scalar>());
    data1[i + PacketSize] = abs_helper(internal::random<Scalar>());
    data1[i + 2 * PacketSize] = Scalar(0) - abs_helper(internal::random<Scalar>());
  }
  if (!std::is_same<Scalar, bool>::value && NumTraits<Scalar>::IsSigned) {
    CHECK_CWISE3_IF(true, REF_MSUB, internal::pmsub);
    CHECK_CWISE3_IF(true, REF_NMADD, internal::pnmadd);
  }

  CHECK_CWISE1_IF(PacketTraits::HasSqrt, numext::sqrt, internal::psqrt);
  CHECK_CWISE1_IF(PacketTraits::HasRsqrt, numext::rsqrt, internal::prsqrt);
  CHECK_CWISE1_IF(PacketTraits::HasCbrt, numext::cbrt, internal::pcbrt);
}

// Notice that this definition works for complex types as well.
// std::log2 only supports real types, not complex.
template <typename Scalar>
Scalar log2(Scalar x) {
  return Scalar(EIGEN_LOG2E) * std::log(x);
}

template <typename Scalar, typename Packet>
void packetmath_real() {
  typedef internal::packet_traits<Scalar> PacketTraits;
  const int PacketSize = internal::unpacket_traits<Packet>::size;

  const int size = PacketSize * 4;
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data1[PacketSize * 4] = {};
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data2[PacketSize * 4] = {};
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar ref[PacketSize * 4] = {};

  // Negate with -0.
  if (PacketTraits::HasNegate) {
    test::packet_helper<PacketTraits::HasNegate, Packet> h;
    data1[0] = Scalar{-0};
    h.store(data2, internal::pnegate(h.load(data1)));
    typedef std::make_unsigned_t<typename internal::make_integer<Scalar>::type> Bits;
    Bits bits = numext::bit_cast<Bits>(data2[0]);
    VERIFY_IS_EQUAL(bits, static_cast<Bits>(Bits(1) << (sizeof(Scalar) * CHAR_BIT - 1)));
  }

  for (int i = 0; i < size; ++i) {
    data1[i] = Scalar(internal::random<double>(0, 1) * std::pow(10., internal::random<double>(-6, 6)));
    data2[i] = Scalar(internal::random<double>(0, 1) * std::pow(10., internal::random<double>(-6, 6)));
  }

  if (internal::random<float>(0, 1) < 0.1f) data1[internal::random<int>(0, PacketSize)] = Scalar(0);

  CHECK_CWISE1_IF(PacketTraits::HasLog, std::log, internal::plog);
  CHECK_CWISE1_IF(PacketTraits::HasLog, log2, internal::plog2);
  CHECK_CWISE1_IF(PacketTraits::HasLog10, std::log10, internal::plog10);
  CHECK_CWISE1_IF(PacketTraits::HasRsqrt, numext::rsqrt, internal::prsqrt);

  for (int i = 0; i < size; ++i) {
    data1[i] = Scalar(internal::random<double>(-1, 1) * std::pow(10., internal::random<double>(-3, 3)));
    data2[i] = Scalar(internal::random<double>(-1, 1) * std::pow(10., internal::random<double>(-3, 3)));
  }
  CHECK_CWISE1_IF(PacketTraits::HasSin, std::sin, internal::psin);
  CHECK_CWISE1_IF(PacketTraits::HasCos, std::cos, internal::pcos);
  CHECK_CWISE1_IF(PacketTraits::HasTan, std::tan, internal::ptan);

  CHECK_CWISE1_EXACT_IF(PacketTraits::HasRound, numext::round, internal::pround);
  CHECK_CWISE1_EXACT_IF(PacketTraits::HasRound, numext::ceil, internal::pceil);
  CHECK_CWISE1_EXACT_IF(PacketTraits::HasRound, numext::floor, internal::pfloor);
  CHECK_CWISE1_EXACT_IF(PacketTraits::HasRound, numext::rint, internal::print);
  CHECK_CWISE1_EXACT_IF(PacketTraits::HasRound, numext::trunc, internal::ptrunc);
  CHECK_CWISE1_IF(PacketTraits::HasSign, numext::sign, internal::psign);

  packetmath_boolean_mask_ops_real<Scalar, Packet>();

  // Rounding edge cases.
  if (PacketTraits::HasRound) {
    typedef typename internal::make_integer<Scalar>::type IntType;
    // Start with values that cannot fit inside an integer, work down to less than one.
    Scalar val =
        numext::mini(Scalar(2) * static_cast<Scalar>(NumTraits<IntType>::highest()), NumTraits<Scalar>::highest());
    std::vector<Scalar> values;
    while (val > Scalar(0.25)) {
      // Cover both even and odd, positive and negative cases.
      values.push_back(val);
      values.push_back(val + Scalar(0.3));
      values.push_back(val + Scalar(0.5));
      values.push_back(val + Scalar(0.8));
      values.push_back(val + Scalar(1));
      values.push_back(val + Scalar(1.3));
      values.push_back(val + Scalar(1.5));
      values.push_back(val + Scalar(1.8));
      values.push_back(-val);
      values.push_back(-val - Scalar(0.3));
      values.push_back(-val - Scalar(0.5));
      values.push_back(-val - Scalar(0.8));
      values.push_back(-val - Scalar(1));
      values.push_back(-val - Scalar(1.3));
      values.push_back(-val - Scalar(1.5));
      values.push_back(-val - Scalar(1.8));
      values.push_back(Scalar(-1.5) + val);  // Bug 1785.
      val = val / Scalar(2);
    }
    values.push_back(NumTraits<Scalar>::infinity());
    values.push_back(-NumTraits<Scalar>::infinity());
    values.push_back(NumTraits<Scalar>::quiet_NaN());

    for (size_t k = 0; k < values.size(); ++k) {
      data1[0] = values[k];
      CHECK_CWISE1_EXACT_IF(PacketTraits::HasRound, numext::round, internal::pround);
      CHECK_CWISE1_EXACT_IF(PacketTraits::HasRound, numext::ceil, internal::pceil);
      CHECK_CWISE1_EXACT_IF(PacketTraits::HasRound, numext::floor, internal::pfloor);
      CHECK_CWISE1_EXACT_IF(PacketTraits::HasRound, numext::rint, internal::print);
      CHECK_CWISE1_EXACT_IF(PacketTraits::HasRound, numext::trunc, internal::ptrunc);
    }
  }

  for (int i = 0; i < size; ++i) {
    data1[i] = Scalar(internal::random<double>(-1, 1));
    data2[i] = Scalar(internal::random<double>(-1, 1));
  }
  CHECK_CWISE1_IF(PacketTraits::HasASin, std::asin, internal::pasin);
  CHECK_CWISE1_IF(PacketTraits::HasACos, std::acos, internal::pacos);
  CHECK_CWISE1_IF(PacketTraits::HasATan, std::atan, internal::patan);
  CHECK_CWISE1_IF(PacketTraits::HasATanh, std::atanh, internal::patanh);

  for (int i = 0; i < size; ++i) {
    data1[i] = Scalar(internal::random<double>(-87, 88));
    data2[i] = Scalar(internal::random<double>(-87, 88));
    data1[0] = -NumTraits<Scalar>::infinity();
  }
  CHECK_CWISE1_IF(PacketTraits::HasExp, std::exp, internal::pexp);
  CHECK_CWISE1_IF(PacketTraits::HasExp, std::exp2, internal::pexp2);

  CHECK_CWISE1_BYREF1_IF(PacketTraits::HasExp, REF_FREXP, internal::pfrexp);
  if (PacketTraits::HasExp) {
// Check denormals:
#if !EIGEN_ARCH_ARM
    for (int j = 0; j < 3; ++j) {
      data1[0] = Scalar(std::ldexp(1, NumTraits<Scalar>::min_exponent() - j));
      CHECK_CWISE1_BYREF1_IF(PacketTraits::HasExp, REF_FREXP, internal::pfrexp);
      data1[0] = -data1[0];
      CHECK_CWISE1_BYREF1_IF(PacketTraits::HasExp, REF_FREXP, internal::pfrexp);
    }
#endif

    // zero
    data1[0] = Scalar(0);
    CHECK_CWISE1_BYREF1_IF(PacketTraits::HasExp, REF_FREXP, internal::pfrexp);

    // inf and NaN only compare output fraction, not exponent.
    test::packet_helper<PacketTraits::HasExp, Packet> h;
    Packet pout;
    Scalar sout;
    Scalar special[] = {NumTraits<Scalar>::infinity(), -NumTraits<Scalar>::infinity(), NumTraits<Scalar>::quiet_NaN()};
    for (int i = 0; i < 3; ++i) {
      data1[0] = special[i];
      ref[0] = Scalar(REF_FREXP(data1[0], ref[PacketSize]));
      h.store(data2, internal::pfrexp(h.load(data1), h.forward_reference(pout, sout)));
      VERIFY(test::areApprox(ref, data2, 1) && "internal::pfrexp");
    }
  }

  for (int i = 0; i < PacketSize; ++i) {
    data1[i] = Scalar(internal::random<double>(-1, 1));
    data2[i] = Scalar(internal::random<double>(-1, 1));
  }
  for (int i = 0; i < PacketSize; ++i) {
    data1[i + PacketSize] = Scalar(internal::random<int>(-4, 4));
    data2[i + PacketSize] = Scalar(internal::random<double>(-4, 4));
  }
  CHECK_CWISE2_IF(PacketTraits::HasExp, REF_LDEXP, internal::pldexp);
  if (PacketTraits::HasExp) {
    data1[0] = Scalar(-1);
    // underflow to zero
    data1[PacketSize] = Scalar(NumTraits<Scalar>::min_exponent() - 55);
    CHECK_CWISE2_IF(PacketTraits::HasExp, REF_LDEXP, internal::pldexp);
    // overflow to inf
    data1[PacketSize] = Scalar(NumTraits<Scalar>::max_exponent() + 10);
    CHECK_CWISE2_IF(PacketTraits::HasExp, REF_LDEXP, internal::pldexp);
    // NaN stays NaN
    data1[0] = NumTraits<Scalar>::quiet_NaN();
    CHECK_CWISE2_IF(PacketTraits::HasExp, REF_LDEXP, internal::pldexp);
    VERIFY((numext::isnan)(data2[0]));
    // inf stays inf
    data1[0] = NumTraits<Scalar>::infinity();
    data1[PacketSize] = Scalar(NumTraits<Scalar>::min_exponent() - 10);
    CHECK_CWISE2_IF(PacketTraits::HasExp, REF_LDEXP, internal::pldexp);
    // zero stays zero
    data1[0] = Scalar(0);
    data1[PacketSize] = Scalar(NumTraits<Scalar>::max_exponent() + 10);
    CHECK_CWISE2_IF(PacketTraits::HasExp, REF_LDEXP, internal::pldexp);
    // Small number big exponent.
    data1[0] = Scalar(std::ldexp(Scalar(1.0), NumTraits<Scalar>::min_exponent() - 1));
    data1[PacketSize] = Scalar(-NumTraits<Scalar>::min_exponent() + NumTraits<Scalar>::max_exponent());
    CHECK_CWISE2_IF(PacketTraits::HasExp, REF_LDEXP, internal::pldexp);
    // Big number small exponent.
    data1[0] = Scalar(std::ldexp(Scalar(1.0), NumTraits<Scalar>::max_exponent() - 1));
    data1[PacketSize] = Scalar(+NumTraits<Scalar>::min_exponent() - NumTraits<Scalar>::max_exponent());
    CHECK_CWISE2_IF(PacketTraits::HasExp, REF_LDEXP, internal::pldexp);
    // Near-max magnitude with small negative exponents.  Regression guard for
    // the 4-way scale-factor split: the remainder factor c2 = 2^(e-3*floor(e/4))
    // is > 1 for e in {-1, -2, -5, -6, ...}, so the multiply tree must apply
    // the downscale c1 before c2 -- otherwise (numext::abs(a)) * c2 spuriously
    // overflows to inf for finite results like ldexp((numext::numeric_limits)
    // <Scalar>::max(), -1).
    for (int i = 0; i < PacketSize; ++i) {
      data1[i] = (numext::numeric_limits<Scalar>::max)();
      data1[i + PacketSize] = Scalar(-1 - (i % 8));  // -1, -2, ..., -8
    }
    CHECK_CWISE2_IF(PacketTraits::HasExp, REF_LDEXP, internal::pldexp);
    // For |e| >= 2 * max_exponent, reassociated scale factors overflow and can
    // turn zero into NaN or finite denormal results into infinity.
#if !EIGEN_ARCH_ARM
    const Scalar tiny = std::numeric_limits<Scalar>::denorm_min();
#else
    // 32-bit ARM flushes denormal inputs to zero.
    const Scalar tiny = (std::numeric_limits<Scalar>::min)();
#endif
    for (int i = 0; i < PacketSize; ++i) {
      data1[i] = (i % 2) ? tiny : Scalar(0);
      data1[i + PacketSize] = Scalar(2 * NumTraits<Scalar>::max_exponent() + (i % 4));
    }
    CHECK_CWISE2_IF(PacketTraits::HasExp, REF_LDEXP, internal::pldexp);
    for (int i = 0; i < PacketSize; ++i) {
      data1[i] = (i % 2) ? Scalar(1) : Scalar(0);
      data1[i + PacketSize] = Scalar(-2 * NumTraits<Scalar>::max_exponent() - (i % 4));
    }
    CHECK_CWISE2_IF(PacketTraits::HasExp, REF_LDEXP, internal::pldexp);
  }

  for (int i = 0; i < size; ++i) {
    data1[i] = Scalar(internal::random<double>(-1, 1) * std::pow(10., internal::random<double>(-6, 6)));
    data2[i] = Scalar(internal::random<double>(-1, 1) * std::pow(10., internal::random<double>(-6, 6)));
  }
  data1[0] = Scalar(1e-20);
  CHECK_CWISE1_IF(PacketTraits::HasTanh, std::tanh, internal::ptanh);
  if (PacketTraits::HasExp && PacketSize >= 2) {
    const Scalar small = NumTraits<Scalar>::epsilon();
    data1[0] = NumTraits<Scalar>::quiet_NaN();
    data1[1] = small;
    test::packet_helper<PacketTraits::HasExp, Packet> h;
    h.store(data2, internal::pexp(h.load(data1)));
    VERIFY((numext::isnan)(data2[0]));
    VERIFY_IS_APPROX(std::exp(small), data2[1]);

    data1[0] = -small;
    data1[1] = Scalar(0);
    h.store(data2, internal::pexp(h.load(data1)));
    VERIFY_IS_APPROX(std::exp(-small), data2[0]);
    VERIFY_IS_EQUAL(std::exp(Scalar(0)), data2[1]);

    data1[0] = (std::numeric_limits<Scalar>::min)();
    data1[1] = -(std::numeric_limits<Scalar>::min)();
    h.store(data2, internal::pexp(h.load(data1)));
    VERIFY_IS_APPROX(std::exp((std::numeric_limits<Scalar>::min)()), data2[0]);
    VERIFY_IS_APPROX(std::exp(-(std::numeric_limits<Scalar>::min)()), data2[1]);

    data1[0] = std::numeric_limits<Scalar>::denorm_min();
    data1[1] = -std::numeric_limits<Scalar>::denorm_min();
    h.store(data2, internal::pexp(h.load(data1)));
    VERIFY_IS_APPROX(std::exp(std::numeric_limits<Scalar>::denorm_min()), data2[0]);
    VERIFY_IS_APPROX(std::exp(-std::numeric_limits<Scalar>::denorm_min()), data2[1]);

    // pexp must produce subnormal outputs for inputs in
    // [log(denorm_min), log(min)).
#if !EIGEN_ARCH_ARM  // 32-bit ARM flushes subnormals.
    if (std::numeric_limits<Scalar>::has_denorm == std::denorm_present) {
      const Scalar log_min = numext::log((std::numeric_limits<Scalar>::min)());
      const Scalar log_denorm_min = numext::log(std::numeric_limits<Scalar>::denorm_min());
      data1[0] = log_min - Scalar(0.5);                     // just inside subnormal cliff
      data1[1] = Scalar(0.5) * (log_min + log_denorm_min);  // mid-subnormal
      h.store(data2, internal::pexp(h.load(data1)));
      VERIFY_IS_APPROX(numext::exp(data1[0]), data2[0]);
      VERIFY_IS_APPROX(numext::exp(data1[1]), data2[1]);
    }
#endif
  }

  if (PacketTraits::HasTanh) {
    // NOTE this test might fail with GCC prior to 6.3, see MathFunctionsImpl.h for details.
    data1[0] = NumTraits<Scalar>::quiet_NaN();
    test::packet_helper<internal::packet_traits<Scalar>::HasTanh, Packet> h;
    h.store(data2, internal::ptanh(h.load(data1)));
    VERIFY((numext::isnan)(data2[0]));
  }

  if (PacketTraits::HasExp) {
    internal::scalar_logistic_op<Scalar> logistic;
    for (int i = 0; i < size; ++i) {
      data1[i] = Scalar(internal::random<double>(-20, 20));
    }

    test::packet_helper<PacketTraits::HasExp, Packet> h;
    h.store(data2, logistic.packetOp(h.load(data1)));
    for (int i = 0; i < PacketSize; ++i) {
      VERIFY_IS_APPROX(data2[i], logistic(data1[i]));
    }
  }

  data1[0] = NumTraits<Scalar>::infinity();
  data1[1] = Scalar(-1);
  CHECK_CWISE1_IF(PacketTraits::HasLog1p, std::log1p, internal::plog1p);
  data1[0] = NumTraits<Scalar>::infinity();
  data1[1] = -NumTraits<Scalar>::infinity();
  CHECK_CWISE1_IF(PacketTraits::HasExpm1, std::expm1, internal::pexpm1);

  if (PacketSize >= 2) {
    data1[0] = NumTraits<Scalar>::quiet_NaN();
    data1[1] = NumTraits<Scalar>::epsilon();
    if (PacketTraits::HasLog) {
      test::packet_helper<PacketTraits::HasLog, Packet> h;
      h.store(data2, internal::plog(h.load(data1)));
      VERIFY((numext::isnan)(data2[0]));
      // TODO(cantonios): Re-enable for bfloat16.
      if (!std::is_same<Scalar, bfloat16>::value) {
        VERIFY_IS_APPROX(std::log(data1[1]), data2[1]);
      }

      data1[0] = -NumTraits<Scalar>::epsilon();
      data1[1] = Scalar(0);
      h.store(data2, internal::plog(h.load(data1)));
      VERIFY((numext::isnan)(data2[0]));
      VERIFY_IS_EQUAL(std::log(Scalar(0)), data2[1]);

      data1[0] = (std::numeric_limits<Scalar>::min)();
      data1[1] = -(std::numeric_limits<Scalar>::min)();
      h.store(data2, internal::plog(h.load(data1)));
      // TODO(cantonios): Re-enable for bfloat16.
      if (!std::is_same<Scalar, bfloat16>::value) {
        VERIFY_IS_APPROX(std::log((std::numeric_limits<Scalar>::min)()), data2[0]);
      }
      VERIFY((numext::isnan)(data2[1]));

      // Note: 32-bit arm always flushes denorms to zero.
#if !EIGEN_ARCH_ARM
      if (std::numeric_limits<Scalar>::has_denorm == std::denorm_present) {
        data1[0] = std::numeric_limits<Scalar>::denorm_min();
        data1[1] = -std::numeric_limits<Scalar>::denorm_min();
        h.store(data2, internal::plog(h.load(data1)));
        // TODO(rmlarsen): Re-enable for bfloat16.
        if (!std::is_same<Scalar, bfloat16>::value) {
          VERIFY_IS_APPROX(std::log(std::numeric_limits<Scalar>::denorm_min()), data2[0]);
        }
        VERIFY((numext::isnan)(data2[1]));
      }
#endif

      data1[0] = Scalar(-1.0f);
      h.store(data2, internal::plog(h.load(data1)));
      VERIFY((numext::isnan)(data2[0]));

      data1[0] = NumTraits<Scalar>::infinity();
      h.store(data2, internal::plog(h.load(data1)));
      VERIFY((numext::isinf)(data2[0]));
    }
    if (PacketTraits::HasLog10) {
      test::packet_helper<PacketTraits::HasLog10, Packet> h;
      data1[0] = Scalar(0);
      data1[1] = NumTraits<Scalar>::infinity();
      h.store(data2, internal::plog10(h.load(data1)));
      VERIFY_IS_EQUAL(std::log10(Scalar(0)), data2[0]);
      VERIFY_IS_EQUAL(std::log10(NumTraits<Scalar>::infinity()), data2[1]);
    }
    if (PacketTraits::HasLog1p) {
      test::packet_helper<PacketTraits::HasLog1p, Packet> h;
      data1[0] = Scalar(-2);
      data1[1] = -NumTraits<Scalar>::infinity();
      h.store(data2, internal::plog1p(h.load(data1)));
      VERIFY((numext::isnan)(data2[0]));
      VERIFY((numext::isnan)(data2[1]));
    }

    // TODO(rmlarsen): Re-enable for half and bfloat16.
    if (PacketTraits::HasCos && !std::is_same<Scalar, half>::value && !std::is_same<Scalar, bfloat16>::value) {
      test::packet_helper<PacketTraits::HasCos, Packet> h;
      for (Scalar k = Scalar(1); k < Scalar(10000) / NumTraits<Scalar>::epsilon(); k *= Scalar(2)) {
        for (int k1 = 0; k1 <= 1; ++k1) {
          data1[0] = Scalar((2 * double(k) + k1) * double(EIGEN_PI) / 2 * internal::random<double>(0.8, 1.2));
          data1[1] = Scalar((2 * double(k) + 2 + k1) * double(EIGEN_PI) / 2 * internal::random<double>(0.8, 1.2));
          h.store(data2, internal::pcos(h.load(data1)));
          h.store(data2 + PacketSize, internal::psin(h.load(data1)));
          VERIFY(data2[0] <= Scalar(1.) && data2[0] >= Scalar(-1.));
          VERIFY(data2[1] <= Scalar(1.) && data2[1] >= Scalar(-1.));
          VERIFY(data2[PacketSize + 0] <= Scalar(1.) && data2[PacketSize + 0] >= Scalar(-1.));
          VERIFY(data2[PacketSize + 1] <= Scalar(1.) && data2[PacketSize + 1] >= Scalar(-1.));

          VERIFY_IS_APPROX(data2[0], std::cos(data1[0]));
          VERIFY_IS_APPROX(data2[1], std::cos(data1[1]));
          VERIFY_IS_APPROX(data2[PacketSize + 0], std::sin(data1[0]));
          VERIFY_IS_APPROX(data2[PacketSize + 1], std::sin(data1[1]));

          VERIFY_IS_APPROX(numext::abs2(data2[0]) + numext::abs2(data2[PacketSize + 0]), Scalar(1));
          VERIFY_IS_APPROX(numext::abs2(data2[1]) + numext::abs2(data2[PacketSize + 1]), Scalar(1));
        }
      }

      data1[0] = NumTraits<Scalar>::infinity();
      data1[1] = -NumTraits<Scalar>::infinity();
      h.store(data2, internal::psin(h.load(data1)));
      VERIFY((numext::isnan)(data2[0]));
      VERIFY((numext::isnan)(data2[1]));

      h.store(data2, internal::pcos(h.load(data1)));
      VERIFY((numext::isnan)(data2[0]));
      VERIFY((numext::isnan)(data2[1]));

      data1[0] = NumTraits<Scalar>::quiet_NaN();
      h.store(data2, internal::psin(h.load(data1)));
      VERIFY((numext::isnan)(data2[0]));
      h.store(data2, internal::pcos(h.load(data1)));
      VERIFY((numext::isnan)(data2[0]));

      data1[0] = -Scalar(0.);
      h.store(data2, internal::psin(h.load(data1)));
      VERIFY(test::biteq(data2[0], data1[0]));
      h.store(data2, internal::pcos(h.load(data1)));
      VERIFY_IS_EQUAL(data2[0], Scalar(1));
    }
  }
  if (PacketTraits::HasReciprocal && PacketSize >= 2) {
    test::packet_helper<PacketTraits::HasReciprocal, Packet> h;
    const Scalar inf = NumTraits<Scalar>::infinity();
    const Scalar zero = Scalar(0);
    data1[0] = zero;
    data1[1] = -zero;
    h.store(data2, internal::preciprocal(h.load(data1)));
    VERIFY_IS_EQUAL(data2[0], inf);
    VERIFY_IS_EQUAL(data2[1], -inf);

    data1[0] = inf;
    data1[1] = -inf;
    h.store(data2, internal::preciprocal(h.load(data1)));
    VERIFY_IS_EQUAL(data2[0], zero);
    VERIFY_IS_EQUAL(data2[1], -zero);
  }
}

template <typename Scalar>
Scalar propagate_nan_max(const Scalar& a, const Scalar& b) {
  if ((numext::isnan)(a)) return a;
  if ((numext::isnan)(b)) return b;
  return (numext::maxi)(a, b);
}

template <typename Scalar>
Scalar propagate_nan_min(const Scalar& a, const Scalar& b) {
  if ((numext::isnan)(a)) return a;
  if ((numext::isnan)(b)) return b;
  return (numext::mini)(a, b);
}

template <typename Scalar>
Scalar propagate_number_max(const Scalar& a, const Scalar& b) {
  if ((numext::isnan)(a)) return b;
  if ((numext::isnan)(b)) return a;
  return (numext::maxi)(a, b);
}

template <typename Scalar>
Scalar propagate_number_min(const Scalar& a, const Scalar& b) {
  if ((numext::isnan)(a)) return b;
  if ((numext::isnan)(b)) return a;
  return (numext::mini)(a, b);
}

// pmin/pmax<PropagateNaN> may differ from plain pmin/pmax only where a NaN is involved: on two
// ordered operands both must select the same one, down to the sign of a zero result. Signed
// zeros are the only operands that compare equal while differing in their bits, and isApprox
// cannot tell them apart, so compare the bits. Which operand a tie selects stays unspecified:
// it varies with the backend and with the packet width wherever the hardware min/max resolves
// a tie by sign rather than by position (issue #3116).
template <typename Scalar, typename Packet, typename EnableIf = void>
struct packetmath_minmax_propagation_test {
  static void run() {}
};

template <typename Scalar, typename Packet>
struct packetmath_minmax_propagation_test<Scalar, Packet, std::enable_if_t<!NumTraits<Scalar>::IsInteger>> {
  using PacketTraits = internal::packet_traits<Scalar>;
  using Bits = std::make_unsigned_t<typename internal::make_integer<Scalar>::type>;

  static bool same_bits(const Scalar& a, const Scalar& b) {
    return numext::bit_cast<Bits>(a) == numext::bit_cast<Bits>(b);
  }

  // NaN payloads are not pinned down across backends, so a NaN result only has to stay a NaN.
  static void verify_semantics(const Scalar& a, const Scalar& b, const Scalar& plain, const Scalar& fast,
                               const Scalar& nan, const Scalar& numbers) {
    const bool a_is_nan = (numext::isnan)(a), b_is_nan = (numext::isnan)(b);
    if (a_is_nan || b_is_nan) {
      VERIFY((numext::isnan)(nan));
      if (a_is_nan && b_is_nan) {
        VERIFY((numext::isnan)(numbers));
      } else {
        VERIFY(same_bits(numbers, a_is_nan ? b : a));
      }
    } else {
      VERIFY(same_bits(nan, plain));
      VERIFY(same_bits(fast, plain));
    }
  }

  static void run() {
    constexpr int PacketSize = internal::unpacket_traits<Packet>::size;
    const Scalar values[] = {Scalar(0),
                             Scalar(-0.0),
                             Scalar(1),
                             Scalar(-1),
                             NumTraits<Scalar>::infinity(),
                             -NumTraits<Scalar>::infinity(),
                             NumTraits<Scalar>::quiet_NaN()};
    constexpr int kNumValues = int(sizeof(values) / sizeof(values[0]));

    EIGEN_ALIGN_TO_BOUNDARY(internal::unpacket_traits<Packet>::alignment) Scalar lhs[PacketSize];
    EIGEN_ALIGN_TO_BOUNDARY(internal::unpacket_traits<Packet>::alignment) Scalar rhs[PacketSize];
    EIGEN_ALIGN_TO_BOUNDARY(internal::unpacket_traits<Packet>::alignment) Scalar plain[PacketSize];
    EIGEN_ALIGN_TO_BOUNDARY(internal::unpacket_traits<Packet>::alignment) Scalar fast[PacketSize];
    EIGEN_ALIGN_TO_BOUNDARY(internal::unpacket_traits<Packet>::alignment) Scalar nan[PacketSize];
    EIGEN_ALIGN_TO_BOUNDARY(internal::unpacket_traits<Packet>::alignment) Scalar numbers[PacketSize];

    // Without HasMin/HasMax the helper degrades to the scalar op and writes only one element.
    constexpr int kMinLanes = PacketTraits::HasMin ? PacketSize : 1;
    constexpr int kMaxLanes = PacketTraits::HasMax ? PacketSize : 1;

    test::packet_helper<PacketTraits::HasMin, Packet> hmin;
    test::packet_helper<PacketTraits::HasMax, Packet> hmax;
    for (int i = 0; i < kNumValues; ++i) {
      const Scalar& a = values[i];
      for (int j = 0; j < kNumValues; ++j) {
        const Scalar& b = values[j];
        verify_semantics(a, b, internal::pmin(a, b), internal::pmin<PropagateFast>(a, b),
                         internal::pmin<PropagateNaN>(a, b), internal::pmin<PropagateNumbers>(a, b));
        verify_semantics(a, b, internal::pmax(a, b), internal::pmax<PropagateFast>(a, b),
                         internal::pmax<PropagateNaN>(a, b), internal::pmax<PropagateNumbers>(a, b));

        for (int k = 0; k < PacketSize; ++k) {
          lhs[k] = a;
          rhs[k] = b;
        }
        hmin.store(plain, internal::pmin(hmin.load(lhs), hmin.load(rhs)));
        hmin.store(fast, internal::pmin<PropagateFast>(hmin.load(lhs), hmin.load(rhs)));
        hmin.store(nan, internal::pmin<PropagateNaN>(hmin.load(lhs), hmin.load(rhs)));
        hmin.store(numbers, internal::pmin<PropagateNumbers>(hmin.load(lhs), hmin.load(rhs)));
        for (int k = 0; k < kMinLanes; ++k) verify_semantics(a, b, plain[k], fast[k], nan[k], numbers[k]);

        hmax.store(plain, internal::pmax(hmax.load(lhs), hmax.load(rhs)));
        hmax.store(fast, internal::pmax<PropagateFast>(hmax.load(lhs), hmax.load(rhs)));
        hmax.store(nan, internal::pmax<PropagateNaN>(hmax.load(lhs), hmax.load(rhs)));
        hmax.store(numbers, internal::pmax<PropagateNumbers>(hmax.load(lhs), hmax.load(rhs)));
        for (int k = 0; k < kMaxLanes; ++k) verify_semantics(a, b, plain[k], fast[k], nan[k], numbers[k]);
      }
    }
  }
};

template <bool Cond, typename Scalar, typename Packet, bool SkipDenorms = EIGEN_ARCH_ARM, typename FunctorT>
std::enable_if_t<!Cond, void> run_ieee_cases(const FunctorT&) {}

template <bool Cond, typename Scalar, typename Packet, bool SkipDenorms = EIGEN_ARCH_ARM, typename FunctorT>
std::enable_if_t<Cond, void> run_ieee_cases(const FunctorT& fun) {
  const int PacketSize = internal::unpacket_traits<Packet>::size;
  const Scalar norm_min = (std::numeric_limits<Scalar>::min)();
  const Scalar norm_max = (std::numeric_limits<Scalar>::max)();
  const Scalar inf = (std::numeric_limits<Scalar>::infinity)();
  const Scalar nan = (std::numeric_limits<Scalar>::quiet_NaN)();
  std::vector<Scalar> values{Scalar(0), Scalar(1), norm_max, inf, nan};
  // On ARM, NEON flush-to-zero mode can flush intermediate subnormal results to zero,
  // causing functions like sin(norm_min) to return 0 instead of norm_min. Skip norm_min
  // in that case, along with truly subnormal values.
  if (!SkipDenorms) {
    values.push_back(norm_min);
    if (std::numeric_limits<Scalar>::has_denorm == std::denorm_present) {
      values.push_back(std::numeric_limits<Scalar>::denorm_min());
      values.push_back(norm_min / Scalar(2));
    }
  }

  constexpr int size = PacketSize * 2;
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data1[size];
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data2[size];
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar ref[size];
  for (int i = 0; i < size; ++i) {
    data1[i] = data2[i] = ref[i] = Scalar(0);
  }

  for (Scalar abs_value : values) {
    data1[0] = abs_value;
    data1[1] = -data1[0];
    g_test_stack.push_back("IEEE cases: " + fun.name);
    CHECK_CWISE1_IF(Cond, fun.expected, fun.actual);
    g_test_stack.pop_back();
  }
}

// Create a tester struct with the actual and the reference function
// as templated member functions.
#define CREATE_TESTER(NAME, ACTUAL, EXPECTED) \
  struct NAME {                               \
    template <typename T>                     \
    T actual(const T& val) const {            \
      return ACTUAL(val);                     \
    }                                         \
    template <typename T>                     \
    T expected(const T& val) const {          \
      return EXPECTED(val);                   \
    }                                         \
    const std::string name = #NAME;           \
  }

CREATE_TESTER(sqrt_fun, internal::psqrt, numext::sqrt);
CREATE_TESTER(rsqrt_fun, internal::prsqrt, numext::rsqrt);
CREATE_TESTER(cbrt_fun, internal::pcbrt, numext::cbrt);
CREATE_TESTER(exp_fun, internal::pexp, numext::exp);
CREATE_TESTER(exp2_fun, internal::pexp2, numext::exp2);
CREATE_TESTER(log_fun, internal::plog, numext::log);
CREATE_TESTER(log2_fun, internal::plog2, numext::log2);
CREATE_TESTER(expm1_fun, internal::pexpm1, numext::expm1);
CREATE_TESTER(log1p_fun, internal::plog1p, numext::log1p);
CREATE_TESTER(sin_fun, internal::psin, numext::sin);
CREATE_TESTER(cos_fun, internal::pcos, numext::cos);
CREATE_TESTER(tan_fun, internal::ptan, numext::tan);
CREATE_TESTER(asin_fun, internal::pasin, numext::asin);
CREATE_TESTER(acos_fun, internal::pacos, numext::acos);
CREATE_TESTER(atan_fun, internal::patan, numext::atan);
CREATE_TESTER(tanh_fun, internal::ptanh, numext::tanh);
CREATE_TESTER(atanh_fun, internal::patanh, numext::atanh);

template <typename Scalar, typename Packet>
std::enable_if_t<NumTraits<Scalar>::IsComplex, void> packetmath_ieee_special_values() {}

template <typename Scalar, typename Packet>
std::enable_if_t<!NumTraits<Scalar>::IsComplex, void> packetmath_ieee_special_values() {
  typedef internal::packet_traits<Scalar> PacketTraits;
  run_ieee_cases<PacketTraits::HasSqrt, Scalar, Packet>(sqrt_fun());
  // TODO(rmlarsen): See if we can fix rsqrt for denorms without wreaking performance.
  run_ieee_cases<PacketTraits::HasRsqrt, Scalar, Packet, true>(rsqrt_fun());
  run_ieee_cases<PacketTraits::HasCbrt, Scalar, Packet>(cbrt_fun());
  run_ieee_cases<PacketTraits::HasExp, Scalar, Packet>(exp_fun());
  run_ieee_cases<PacketTraits::HasExp, Scalar, Packet>(exp2_fun());
  run_ieee_cases<PacketTraits::HasLog, Scalar, Packet>(log_fun());
  run_ieee_cases<PacketTraits::HasLog, Scalar, Packet>(log2_fun());
  run_ieee_cases<PacketTraits::HasExpm1, Scalar, Packet>(expm1_fun());
  run_ieee_cases<PacketTraits::HasLog1p, Scalar, Packet>(log1p_fun());
  run_ieee_cases<PacketTraits::HasSin, Scalar, Packet>(sin_fun());
  run_ieee_cases<PacketTraits::HasCos, Scalar, Packet>(cos_fun());
  run_ieee_cases<PacketTraits::HasTan, Scalar, Packet>(tan_fun());
  run_ieee_cases<PacketTraits::HasASin, Scalar, Packet>(asin_fun());
  run_ieee_cases<PacketTraits::HasACos, Scalar, Packet>(acos_fun());
  run_ieee_cases<PacketTraits::HasATan, Scalar, Packet>(atan_fun());
  run_ieee_cases<PacketTraits::HasTanh, Scalar, Packet>(tanh_fun());
  run_ieee_cases<PacketTraits::HasATanh, Scalar, Packet>(atanh_fun());
}

template <typename Scalar, typename Packet>
void packetmath_redux_infinities() {
  const Scalar infinity = NumTraits<Scalar>::infinity();
  const Packet positive_infinity = internal::pset1<Packet>(infinity);
  VERIFY_IS_EQUAL(internal::predux_min(positive_infinity), infinity);
  VERIFY_IS_EQUAL(internal::predux_max(positive_infinity), infinity);

  const Packet negative_infinity = internal::pset1<Packet>(-infinity);
  VERIFY_IS_EQUAL(internal::predux_min(negative_infinity), -infinity);
  VERIFY_IS_EQUAL(internal::predux_max(negative_infinity), -infinity);
}

#if defined(EIGEN_VECTORIZE_SSE2)
void packetmath_packet16b_reductions() {
  const internal::Packet16b packet(_mm_setr_epi8(0, 1, -1, 2, 0, -128, 127, 0, 0, 0, 3, -2, 0, 42, 0, -1));
  VERIFY_IS_EQUAL(internal::predux_count(packet), 9);
  VERIFY(!internal::predux_all(packet));

  const internal::Packet16b all_nonzero(_mm_set1_epi8(-1));
  VERIFY(internal::predux_all(all_nonzero));

  EIGEN_ALIGN16 bool values[16];
  for (int i = 0; i < 16; ++i) values[i] = i % 3 != 0;
  const internal::Packet16b canonical_packet = internal::pload<internal::Packet16b>(values);
  VERIFY_IS_EQUAL(internal::predux_count_impl<internal::Packet16b>::run(canonical_packet), 10);
}

void packetmath_packet16b_select() {
  EIGEN_ALIGN16 bool condition[16];
  EIGEN_ALIGN16 bool then_values[16];
  EIGEN_ALIGN16 bool else_values[16];
  EIGEN_ALIGN16 bool actual[16];

  for (int i = 0; i < 16; ++i) {
    condition[i] = (i % 3) == 0;
    then_values[i] = (i % 2) == 0;
    else_values[i] = (i % 5) == 0;
  }

  const internal::Packet16b condition_packet = internal::pload<internal::Packet16b>(condition);
  const internal::Packet16b mask = internal::pcmp_eq(condition_packet, internal::pzero(condition_packet));
  const internal::Packet16b selected = internal::pselect(mask, internal::pload<internal::Packet16b>(else_values),
                                                         internal::pload<internal::Packet16b>(then_values));
  internal::pstore(actual, selected);

  for (int i = 0; i < 16; ++i) {
    VERIFY_IS_EQUAL(actual[i], condition[i] ? then_values[i] : else_values[i]);
  }
}
#endif

template <typename Scalar, typename Packet>
void packetmath_abs_bits() {
  using Bits = typename numext::get_integer_by_size<sizeof(Scalar)>::unsigned_type;
  constexpr int PacketSize = unpacket_traits<Packet>::size;
  constexpr Bits Sign = Bits(1) << (8 * sizeof(Scalar) - 1);
  constexpr Bits MinNormal = Bits(1) << (std::numeric_limits<Scalar>::digits - 1);
  constexpr Bits Inf = (Sign - 1) ^ (MinNormal - 1);
  // Move operands between the Scalar arrays and their bit patterns with memcpy only. Reading output[i] through
  // bit_cast lets GCC 14 on ppc64le extract the lane from the pabs result with the signaling xscvspdp conversion,
  // which quiets a signaling NaN before its bits are compared.
  Bits input_bits[PacketSize], output_bits[PacketSize];
  Scalar input[PacketSize], output[PacketSize];
  const auto check = [&] {
    std::memcpy(input, input_bits, sizeof(input));
    internal::pstoreu(output, internal::pabs(internal::ploadu<Packet>(input)));
    std::memcpy(output_bits, output, sizeof(output));
    for (int i = 0; i < PacketSize; ++i) {
      const Bits expected = input_bits[i] & (Sign - 1);
      const Bits actual = output_bits[i];
      if (std::is_same<Scalar, Packet>::value && std::is_floating_point<Scalar>::value && expected > Inf) {
        // Scalar floating-point loads/stores (notably x87) may quiet signaling NaNs. Preserve all other bits.
        VERIFY(actual == expected || actual == Bits(expected | (MinNormal >> 1)));
      } else {
        VERIFY_IS_EQUAL(actual, expected);
      }
    }
  };
  const auto run = [&] {
    EIGEN_IF_CONSTEXPR (sizeof(Scalar) == 2) {
      // Exhaust half/bfloat16 encodings, including signaling NaNs and their payloads.
      for (int first = 0; first < 65536; first += PacketSize) {
        for (int i = 0; i < PacketSize; ++i) input_bits[i] = Bits(first + i);
        check();
      }
    } else {
      const Bits samples[] = {Bits(0),
                              Bits(1),
                              Bits(2),
                              Bits(MinNormal - 1),
                              MinNormal,
                              Bits(MinNormal + 1),
                              numext::bit_cast<Bits>(Scalar(1)),
                              Bits(Inf - 1),
                              Inf,
                              Bits(Inf | 1),
                              Bits(Inf | (MinNormal >> 1)),
                              Bits(Sign - 1)};
      const int count = sizeof(samples) / sizeof(samples[0]);
      for (int offset = 0; offset < count; ++offset) {
        for (Bits sign : {Bits(0), Sign}) {
          for (int i = 0; i < PacketSize; ++i) input_bits[i] = Bits(samples[(offset + i) % count] | sign);
          check();
        }
      }
    }
  };
  run();
  ScopedFlushToZero flush_to_zero;
  run();
}

template <typename Scalar, typename Packet>
void packetmath_notcomplex() {
  packetmath_ieee_special_values<Scalar, Packet>();

  typedef internal::packet_traits<Scalar> PacketTraits;
  const int PacketSize = internal::unpacket_traits<Packet>::size;

  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data1[PacketSize * 4];
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data2[PacketSize * 4];
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar ref[PacketSize * 4];

  Array<Scalar, Dynamic, 1>::Map(data1, PacketSize * 4).setRandom();

  VERIFY((!PacketTraits::Vectorizable) || PacketTraits::HasMin);
  VERIFY((!PacketTraits::Vectorizable) || PacketTraits::HasMax);

  CHECK_CWISE2_IF(PacketTraits::HasMin, (std::min), internal::pmin);
  CHECK_CWISE2_IF(PacketTraits::HasMax, (std::max), internal::pmax);

  CHECK_CWISE2_IF(PacketTraits::HasMin, propagate_number_min, internal::pmin<PropagateNumbers>);
  CHECK_CWISE2_IF(PacketTraits::HasMax, propagate_number_max, internal::pmax<PropagateNumbers>);
  CHECK_CWISE1(numext::abs, internal::pabs);
  // Vectorized versions may give a different result in the case of signed int overflow,
  // which is undefined behavior (e.g. NEON).
  // Also note that unsigned integers with size < sizeof(int) may be implicitly converted to a signed
  // int, which can also trigger UB.
  if (Eigen::NumTraits<Scalar>::IsInteger) {
    for (int i = 0; i < 2 * PacketSize; ++i) {
      data1[i] = data1[i] / Scalar(2);
    }
  }
  CHECK_CWISE2_IF(PacketTraits::HasAbsDiff, REF_ABS_DIFF, internal::pabsdiff);

  ref[0] = data1[0];
  for (int i = 0; i < PacketSize; ++i) ref[0] = internal::pmin(ref[0], data1[i]);
  VERIFY(internal::isApprox(ref[0], internal::predux_min(internal::pload<Packet>(data1))) && "internal::predux_min");
  ref[0] = data1[0];
  for (int i = 0; i < PacketSize; ++i) ref[0] = internal::pmax(ref[0], data1[i]);
  VERIFY(internal::isApprox(ref[0], internal::predux_max(internal::pload<Packet>(data1))) && "internal::predux_max");

  for (int i = 0; i < PacketSize; ++i) ref[i] = data1[0] + Scalar(i);
  internal::pstore(data2, internal::plset<Packet>(data1[0]));
  VERIFY(test::areApprox(ref, data2, PacketSize) && "internal::plset");

  {
    unsigned char* data1_bits = reinterpret_cast<unsigned char*>(data1);
    for (unsigned int i = 0; i < PacketSize * sizeof(Scalar); ++i) data1_bits[i] = 0xff;
    VERIFY(internal::predux_all(internal::pload<Packet>(data1)) && "internal::predux_all(1111)");
    for (int k = 0; k < PacketSize; ++k) {
      for (unsigned int i = 0; i < sizeof(Scalar); ++i) data1_bits[k * sizeof(Scalar) + i] = 0x0;
      VERIFY((!internal::predux_all(internal::pload<Packet>(data1))) && "internal::predux_all(0101)");
      for (unsigned int i = 0; i < sizeof(Scalar); ++i) data1_bits[k * sizeof(Scalar) + i] = 0xff;
    }

    // predux_any
    for (unsigned int i = 0; i < PacketSize * sizeof(Scalar); ++i) data1_bits[i] = 0x0;
    VERIFY((!internal::predux_any(internal::pload<Packet>(data1))) && "internal::predux_any(0000)");
    for (int k = 0; k < PacketSize; ++k) {
      for (unsigned int i = 0; i < sizeof(Scalar); ++i) data1_bits[k * sizeof(Scalar) + i] = 0xff;
      VERIFY(internal::predux_any(internal::pload<Packet>(data1)) && "internal::predux_any(0101)");
      VERIFY_IS_EQUAL(internal::predux_count(internal::pload<Packet>(data1)), k + 1);
    }
    for (int k = 0; k < PacketSize; ++k) {
      for (unsigned int i = 0; i < sizeof(Scalar); ++i) data1_bits[k * sizeof(Scalar) + i] = 0x00;
    }
    data1[0] = Scalar(-0.0);
    VERIFY_IS_EQUAL(internal::predux_count(internal::pload<Packet>(data1)), 0);

    for (int k = 0; k < PacketSize; ++k) {
      data1[k] = Scalar(1);
      VERIFY_IS_EQUAL(internal::predux_count(internal::pload<Packet>(data1)), k + 1);
    }

    if (!NumTraits<Scalar>::IsInteger) {
      for (int k = 0; k < PacketSize; ++k) data2[k] = Scalar(0);
      data2[PacketSize - 1] = NumTraits<Scalar>::quiet_NaN();
      VERIFY_IS_EQUAL(internal::predux_count(internal::pload<Packet>(data2)), 1);
    }
  }

  // Test NaN propagation.
  if (!NumTraits<Scalar>::IsInteger) {
    packetmath_minmax_propagation_test<Scalar, Packet>::run();

    // Test reductions with no NaNs.
    ref[0] = data1[0];
    for (int i = 0; i < PacketSize; ++i) ref[0] = internal::pmin<PropagateNumbers>(ref[0], data1[i]);
    VERIFY(internal::isApprox(ref[0], internal::predux_min<PropagateNumbers>(internal::pload<Packet>(data1))) &&
           "internal::predux_min<PropagateNumbers>");
    ref[0] = data1[0];
    for (int i = 0; i < PacketSize; ++i) ref[0] = internal::pmin<PropagateNaN>(ref[0], data1[i]);
    VERIFY(internal::isApprox(ref[0], internal::predux_min<PropagateNaN>(internal::pload<Packet>(data1))) &&
           "internal::predux_min<PropagateNaN>");
    ref[0] = data1[0];
    for (int i = 0; i < PacketSize; ++i) ref[0] = internal::pmax<PropagateNumbers>(ref[0], data1[i]);
    VERIFY(internal::isApprox(ref[0], internal::predux_max<PropagateNumbers>(internal::pload<Packet>(data1))) &&
           "internal::predux_max<PropagateNumbers>");
    ref[0] = data1[0];
    for (int i = 0; i < PacketSize; ++i) ref[0] = internal::pmax<PropagateNaN>(ref[0], data1[i]);
    VERIFY(internal::isApprox(ref[0], internal::predux_max<PropagateNaN>(internal::pload<Packet>(data1))) &&
           "internal::predux_max<PropagateNumbers>");
    // A single NaN.
    const size_t index = std::numeric_limits<size_t>::quiet_NaN() % PacketSize;
    data1[index] = NumTraits<Scalar>::quiet_NaN();
    VERIFY(PacketSize == 1 || !(numext::isnan)(internal::predux_min<PropagateNumbers>(internal::pload<Packet>(data1))));
    VERIFY((numext::isnan)(internal::predux_min<PropagateNaN>(internal::pload<Packet>(data1))));
    VERIFY(PacketSize == 1 || !(numext::isnan)(internal::predux_max<PropagateNumbers>(internal::pload<Packet>(data1))));
    VERIFY((numext::isnan)(internal::predux_max<PropagateNaN>(internal::pload<Packet>(data1))));
    // All NaNs.
    for (int i = 0; i < 4 * PacketSize; ++i) data1[i] = NumTraits<Scalar>::quiet_NaN();
    VERIFY((numext::isnan)(internal::predux_min<PropagateNumbers>(internal::pload<Packet>(data1))));
    VERIFY((numext::isnan)(internal::predux_min<PropagateNaN>(internal::pload<Packet>(data1))));
    VERIFY((numext::isnan)(internal::predux_max<PropagateNumbers>(internal::pload<Packet>(data1))));
    VERIFY((numext::isnan)(internal::predux_max<PropagateNaN>(internal::pload<Packet>(data1))));

    // Test NaN propagation for coefficient-wise min and max.
    for (int i = 0; i < PacketSize; ++i) {
      data1[i] = internal::random<bool>() ? NumTraits<Scalar>::quiet_NaN() : Scalar(0);
      data1[i + PacketSize] = internal::random<bool>() ? NumTraits<Scalar>::quiet_NaN() : Scalar(0);
    }
    // Note: NaN propagation is implementation defined for pmin/pmax, so we do not test it here.
    CHECK_CWISE2_IF(PacketTraits::HasMin, propagate_number_min, (internal::pmin<PropagateNumbers>));
    CHECK_CWISE2_IF(PacketTraits::HasMax, propagate_number_max, internal::pmax<PropagateNumbers>);
    CHECK_CWISE2_IF(PacketTraits::HasMin, propagate_nan_min, (internal::pmin<PropagateNaN>));
    CHECK_CWISE2_IF(PacketTraits::HasMax, propagate_nan_max, internal::pmax<PropagateNaN>);
  }

  packetmath_boolean_mask_ops_notcomplex_test<Scalar, Packet>::run();
  packetmath_split_half_compare_test<Scalar, Packet>::run();
}

template <typename Scalar, typename Packet, bool ConjLhs, bool ConjRhs>
void test_conj_helper(Scalar* data1, Scalar* data2, Scalar* ref, Scalar* pval) {
  const int PacketSize = internal::unpacket_traits<Packet>::size;

  internal::conj_if<ConjLhs> cj0;
  internal::conj_if<ConjRhs> cj1;
  internal::conj_helper<Scalar, Scalar, ConjLhs, ConjRhs> cj;
  internal::conj_helper<Packet, Packet, ConjLhs, ConjRhs> pcj;

  for (int i = 0; i < PacketSize; ++i) {
    ref[i] = cj0(data1[i]) * cj1(data2[i]);
    VERIFY(internal::isApprox(ref[i], cj.pmul(data1[i], data2[i])) && "conj_helper pmul");
  }
  internal::pstore(pval, pcj.pmul(internal::pload<Packet>(data1), internal::pload<Packet>(data2)));
  VERIFY(test::areApprox(ref, pval, PacketSize) && "conj_helper pmul");

  for (int i = 0; i < PacketSize; ++i) {
    Scalar tmp = ref[i];
    ref[i] += cj0(data1[i]) * cj1(data2[i]);
    VERIFY(internal::isApprox(ref[i], cj.pmadd(data1[i], data2[i], tmp)) && "conj_helper pmadd");
  }
  internal::pstore(
      pval, pcj.pmadd(internal::pload<Packet>(data1), internal::pload<Packet>(data2), internal::pload<Packet>(pval)));
  VERIFY(test::areApprox(ref, pval, PacketSize) && "conj_helper pmadd");
}

template <typename Scalar, typename Packet, bool HasExp = internal::packet_traits<Scalar>::HasExp>
struct exp_complex_test_impl {
  typedef typename Scalar::value_type RealScalar;

  static Scalar pexp1(const Scalar& x) {
    Packet px = internal::pset1<Packet>(x);
    Packet py = internal::pexp(px);
    return internal::pfirst(py);
  }

  static Scalar cis(const RealScalar& x) { return Scalar(numext::cos(x), numext::sin(x)); }

  // Verify equality with signed zero.
  static bool is_exactly_equal(RealScalar a, RealScalar b) {
    // NaNs are always unsigned, and always compare not equal directly.
    if ((numext::isnan)(a)) {
      return (numext::isnan)(b);
    }

    RealScalar zero(0);
#ifdef EIGEN_ARCH_ARM
    // ARM automatically flushes denormals to zero.
    // Preserve sign by multiplying by +0.
    if (numext::abs(a) < (std::numeric_limits<RealScalar>::min)()) {
      a = a * zero;
    }
    if (numext::abs(b) < (std::numeric_limits<RealScalar>::min)()) {
      b = b * zero;
    }
#endif

    // Signed zero.
    if (a == zero) {
      // Signs are either 0 or NaN, so verify that their comparisons to zero are equal.
      return (a == b) && ((numext::signbit(a) == zero) == (numext::signbit(b) == zero));
    }
    // Allow _some_ tolerance.
    return verifyIsApprox(a, b);
  }

  // Verify equality with signed zero.
  static bool is_exactly_equal(const Scalar& a, const Scalar& b, bool quiet = false) {
    bool result = is_exactly_equal(numext::real_ref(a), numext::real_ref(b)) &&
                  is_exactly_equal(numext::imag_ref(a), numext::imag_ref(b));
    if (!result && !quiet) {
      std::cout << a << " != " << b << std::endl;
    }
    return result;
  }

  static bool is_sign_exp_unspecified(const Scalar& z) {
    const RealScalar inf = std::numeric_limits<RealScalar>::infinity();
    // If z is (-∞,±∞), the result is (±0,±0) (signs are unspecified)
    if (numext::real_ref(z) == -inf && (numext::isinf)(numext::imag_ref(z))) {
      return true;
    }
    // If z is (+∞,±∞), the result is (±∞,NaN) and FE_INVALID is raised (the sign of the real part is unspecified)
    if (numext::real_ref(z) == +inf && (numext::isinf)(numext::imag_ref(z))) {
      return true;
    }
    // If z is (-∞,NaN), the result is (±0,±0) (signs are unspecified)
    if (numext::real_ref(z) == -inf && (numext::isnan)(numext::imag_ref(z))) {
      return true;
    }
    // If z is (+∞,NaN), the result is (±∞,NaN) (the sign of the real part is unspecified)
    if (numext::real_ref(z) == +inf && (numext::isnan)(numext::imag_ref(z))) {
      return true;
    }
    // If exp(x) overflows to inf and y is finite nonzero, the result involves inf * cos(y) and
    // inf * sin(y). When cos(y) or sin(y) is near a zero crossing (e.g., cos(pi/2)), different
    // trig implementations may produce different signs, so the signs of the result are unspecified.
    if (!(numext::isinf)(numext::imag_ref(z)) && !(numext::isnan)(numext::imag_ref(z)) && numext::imag_ref(z) != 0 &&
        (numext::isinf)(std::exp(numext::real_ref(z)))) {
      return true;
    }
    return false;
  }

  static void run(Scalar* data1, Scalar* data2, Scalar* ref, int size) {
    const int PacketSize = internal::unpacket_traits<Packet>::size;

    for (int i = 0; i < size; ++i) {
      data1[i] = Scalar(internal::random<RealScalar>(), internal::random<RealScalar>());
    }
    CHECK_CWISE1_N(std::exp, internal::pexp, size);

    // Test all corner cases (and more).
    const RealScalar edges[] = {RealScalar(0),
                                RealScalar(1),
                                RealScalar(2),
                                RealScalar(EIGEN_PI / 2),
                                RealScalar(EIGEN_PI),
                                RealScalar(3 * EIGEN_PI / 2),
                                RealScalar(2 * EIGEN_PI),
                                numext::log(NumTraits<RealScalar>::highest()) - 1,
                                NumTraits<RealScalar>::highest(),
                                std::numeric_limits<RealScalar>::infinity(),
                                std::numeric_limits<RealScalar>::quiet_NaN(),
                                -RealScalar(0),
                                -RealScalar(1),
                                -RealScalar(2),
                                -RealScalar(EIGEN_PI / 2),
                                -RealScalar(EIGEN_PI),
                                -RealScalar(3 * EIGEN_PI / 2),
                                -RealScalar(2 * EIGEN_PI),
                                -numext::log(NumTraits<RealScalar>::highest()) + 1,
                                -NumTraits<RealScalar>::highest(),
                                -std::numeric_limits<RealScalar>::infinity(),
                                -std::numeric_limits<RealScalar>::quiet_NaN()};

    for (RealScalar x : edges) {
      for (RealScalar y : edges) {
        Scalar z = Scalar(x, y);
        Scalar w = pexp1(z);
        if (is_sign_exp_unspecified(z)) {
          Scalar abs_w = Scalar(numext::abs(numext::real_ref(w)), numext::abs(numext::imag_ref(w)));
          Scalar expected = numext::exp(z);
          Scalar abs_expected =
              Scalar(numext::abs(numext::real_ref(expected)), numext::abs(numext::imag_ref(expected)));
          VERIFY(is_exactly_equal(abs_w, abs_expected));
        } else {
          Scalar expected = numext::exp(z);
          // First try exact equality (handles NaN, signed zeros correctly).
          // Fall back to approximate comparison to allow for small differences
          // in trig functions near zero crossings (e.g., vectorized sincos may
          // compute cos(pi/2) = 0 while scalar std::exp gives ~6.12e-17).
          VERIFY(is_exactly_equal(w, expected, /*quiet=*/true) || verifyIsApprox(w, expected));
        }
      }
    }
  }
};

template <typename Scalar, typename Packet>
struct exp_complex_test_impl<Scalar, Packet, false> {
  typedef typename Scalar::value_type RealScalar;
  static void run(Scalar*, Scalar*, Scalar*, int){};
};

template <typename Scalar, typename Packet>
void exp_complex_test(Scalar* data1, Scalar* data2, Scalar* ref, int size) {
  exp_complex_test_impl<Scalar, Packet>::run(data1, data2, ref, size);
}

template <typename Scalar, typename Packet>
void packetmath_complex() {
  typedef internal::packet_traits<Scalar> PacketTraits;
  typedef typename Scalar::value_type RealScalar;
  const int PacketSize = internal::unpacket_traits<Packet>::size;

  const int size = PacketSize * 4;
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data1[PacketSize * 4];
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data2[PacketSize * 4];
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar ref[PacketSize * 4];
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar pval[PacketSize * 4];
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) RealScalar realdata[PacketSize * 4];
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) RealScalar realref[PacketSize * 4];

  for (int i = 0; i < size; ++i) {
    data1[i] = internal::random<Scalar>() * Scalar(1e2);
    data2[i] = internal::random<Scalar>() * Scalar(1e2);
  }

  test_conj_helper<Scalar, Packet, false, false>(data1, data2, ref, pval);
  test_conj_helper<Scalar, Packet, false, true>(data1, data2, ref, pval);
  test_conj_helper<Scalar, Packet, true, false>(data1, data2, ref, pval);
  test_conj_helper<Scalar, Packet, true, true>(data1, data2, ref, pval);

  // Test pcplxflip.
  {
    for (int i = 0; i < PacketSize; ++i) ref[i] = Scalar(std::imag(data1[i]), std::real(data1[i]));
    internal::pstore(pval, internal::pcplxflip(internal::pload<Packet>(data1)));
    VERIFY(test::areApprox(ref, pval, PacketSize) && "pcplxflip");
  }

  const RealScalar zero = RealScalar(0);
  const RealScalar one = RealScalar(1);
  const RealScalar inf = std::numeric_limits<RealScalar>::infinity();
  const RealScalar nan = std::numeric_limits<RealScalar>::quiet_NaN();

  // Test division by a denominator with equal real and imaginary magnitudes
  // to ensure pdiv scaling avoids division by zero (e.g. 1.0 - 1.0i).
  if (PacketTraits::HasDiv) {
    for (int i = 0; i < PacketSize; ++i) {
      data1[i] = Scalar(one, zero);
      RealScalar sign_re = (i & 1) ? -one : one;
      RealScalar sign_im = (i & 2) ? -one : one;
      data2[i] = Scalar(sign_re, sign_im);
    }
    internal::pstore(pval, internal::pdiv(internal::pload<Packet>(data1), internal::pload<Packet>(data2)));
    for (int i = 0; i < PacketSize; ++i) {
      Scalar expected = data1[i] / data2[i];
      VERIFY_IS_APPROX(pval[i], expected);
    }
  }

  // Test pisnan.
  {
    const Scalar values[4] = {Scalar(one, one), Scalar(nan, zero), Scalar(zero, nan), Scalar(nan, nan)};
    const bool expect_nan[4] = {false, true, true, true};
    // The scalar instantiation must remain callable with plain std::complex arguments.
    for (int i = 0; i < 4; ++i) {
      VERIFY(numext::is_exactly_zero(internal::pisnan(values[i])) == !expect_nan[i] && "scalar pisnan");
    }
    for (int i = 0; i < size; ++i) data1[i] = values[i % 4];
    for (int j = 0; j < size; j += PacketSize) {
      internal::pstore(data2 + j, internal::pisnan(internal::pload<Packet>(data1 + j)));
    }
    for (int i = 0; i < size; ++i) {
      VERIFY(numext::is_exactly_zero(data2[i]) == !expect_nan[i % 4] && "pisnan");
    }
  }

  // Multiplication and Division.
  {
    std::array<RealScalar, 8> special_values = {zero, one, inf, nan, -zero, -one, -inf, -nan};
    for (RealScalar a : special_values) {
      for (RealScalar b : special_values) {
        for (RealScalar c : special_values) {
          for (RealScalar d : special_values) {
            data1[0] = Scalar(a, b);
            data2[0] = Scalar(c, d);
            CHECK_CWISE2_IF(PacketTraits::HasMul, internal::complex_multiply, internal::pmul);
            CHECK_CWISE2_IF(PacketTraits::HasDiv, internal::complex_divide, internal::pdiv);
          }
        }
      }
    }
  }

  if (PacketTraits::HasSqrt) {
    for (int i = 0; i < size; ++i) {
      data1[i] = Scalar(internal::random<RealScalar>(), internal::random<RealScalar>());
    }
    CHECK_CWISE1_N(numext::sqrt, internal::psqrt, size);
    CHECK_CWISE1_IF(PacketTraits::HasSign, numext::sign, internal::psign);

    // Test misc. corner cases.
    data1[0] = Scalar(zero, zero);
    data1[1] = Scalar(-zero, zero);
    data1[2] = Scalar(one, zero);
    data1[3] = Scalar(zero, one);
    CHECK_CWISE1_N(numext::sqrt, internal::psqrt, 4);
    data1[0] = Scalar(-one, zero);
    data1[1] = Scalar(zero, -one);
    data1[2] = Scalar(one, one);
    data1[3] = Scalar(-one, -one);
    CHECK_CWISE1_N(numext::sqrt, internal::psqrt, 4);
    data1[0] = Scalar(inf, zero);
    data1[1] = Scalar(zero, inf);
    data1[2] = Scalar(-inf, zero);
    data1[3] = Scalar(zero, -inf);
    CHECK_CWISE1_N(numext::sqrt, internal::psqrt, 4);
    data1[0] = Scalar(inf, inf);
    data1[1] = Scalar(-inf, inf);
    data1[2] = Scalar(inf, -inf);
    data1[3] = Scalar(-inf, -inf);
    CHECK_CWISE1_N(numext::sqrt, internal::psqrt, 4);
    data1[0] = Scalar(nan, zero);
    data1[1] = Scalar(zero, nan);
    data1[2] = Scalar(nan, one);
    data1[3] = Scalar(one, nan);
    CHECK_CWISE1_N(numext::sqrt, internal::psqrt, 4);
    data1[0] = Scalar(nan, nan);
    data1[1] = Scalar(inf, nan);
    data1[2] = Scalar(nan, inf);
    data1[3] = Scalar(-inf, nan);
    CHECK_CWISE1_N(numext::sqrt, internal::psqrt, 4);
  }
  if (PacketTraits::HasLog) {
    for (int i = 0; i < size; ++i) {
      data1[i] = Scalar(internal::random<RealScalar>(), internal::random<RealScalar>());
    }
    CHECK_CWISE1_N(std::log, internal::plog, size);

    // Test misc. corner cases.
    for (RealScalar x : {zero, one, inf}) {
      for (RealScalar y : {zero, one, inf}) {
        data1[0] = Scalar(x, y);
        data1[1] = Scalar(-x, y);
        data1[2] = Scalar(x, -y);
        data1[3] = Scalar(-x, -y);
        CHECK_CWISE1_IM1ULP_N(std::log, internal::plog, 4);
      }
    }
    // Set reference results to nan.
    // Some architectures don't handle IEEE edge cases correctly
    ref[0] = Scalar(nan, nan);
    ref[1] = Scalar(nan, nan);
    ref[2] = Scalar(nan, nan);
    ref[3] = Scalar(nan, nan);
    for (RealScalar x : {zero, one}) {
      data1[0] = Scalar(x, nan);
      data1[1] = Scalar(-x, nan);
      data1[2] = Scalar(nan, x);
      data1[3] = Scalar(nan, -x);
      for (int j = 0; j < size; j += PacketSize)
        internal::pstore(data2 + j, internal::plog(internal::pload<Packet>(data1 + j)));
      VERIFY(test::areApprox(ref, data2, 4));
    }
    data1[0] = Scalar(inf, nan);
    data1[1] = Scalar(-inf, nan);
    data1[2] = Scalar(nan, inf);
    data1[3] = Scalar(nan, -inf);
    CHECK_CWISE1_IM1ULP_N(numext::log, internal::plog, 4);
  }
  exp_complex_test<Scalar, Packet>(data1, data2, ref, size);
}

template <typename Scalar, typename Packet>
void packetmath_scatter_gather() {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  const int PacketSize = internal::unpacket_traits<Packet>::size;
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data1[PacketSize];
  RealScalar refvalue = RealScalar(0);
  for (int i = 0; i < PacketSize; ++i) {
    data1[i] = internal::random<Scalar>();
  }

  int stride = internal::random<int>(1, 20);

  // Buffer of zeros.
  EIGEN_ALIGN_MAX Scalar buffer[PacketSize * 20] = {};

  Packet packet = internal::pload<Packet>(data1);
  internal::pscatter<Scalar, Packet>(buffer, packet, stride);

  for (int i = 0; i < PacketSize * 20; ++i) {
    if ((i % stride) == 0 && i < stride * PacketSize) {
      VERIFY(test::isApproxAbs(buffer[i], data1[i / stride], refvalue) && "pscatter");
    } else {
      VERIFY(test::isApproxAbs(buffer[i], Scalar(0), refvalue) && "pscatter");
    }
  }

  for (int i = 0; i < PacketSize * 7; ++i) {
    buffer[i] = internal::random<Scalar>();
  }
  packet = internal::pgather<Scalar, Packet>(buffer, 7);
  internal::pstore(data1, packet);
  for (int i = 0; i < PacketSize; ++i) {
    VERIFY(test::isApproxAbs(data1[i], buffer[i * 7], refvalue) && "pgather");
  }

  for (Index N = 0; N <= PacketSize; ++N) {
    for (Index i = 0; i < N; ++i) {
      data1[i] = internal::random<Scalar>();
    }

    for (Index i = 0; i < N * 20; ++i) {
      buffer[i] = Scalar(0);
    }

    packet = internal::pload_partial<Packet>(data1, N);
    internal::pscatter_partial<Scalar, Packet>(buffer, packet, stride, N);

    for (Index i = 0; i < N * 20; ++i) {
      if ((i % stride) == 0 && i < stride * N) {
        VERIFY(test::isApproxAbs(buffer[i], data1[i / stride], refvalue) && "pscatter_partial");
      } else {
        VERIFY(test::isApproxAbs(buffer[i], Scalar(0), refvalue) && "pscatter_partial");
      }
    }

    for (Index i = 0; i < N * 7; ++i) {
      buffer[i] = internal::random<Scalar>();
    }
    packet = internal::pgather_partial<Scalar, Packet>(buffer, 7, N);
    internal::pstore_partial(data1, packet, N);
    for (Index i = 0; i < N; ++i) {
      VERIFY(test::isApproxAbs(data1[i], buffer[i * 7], refvalue) && "pgather_partial");
    }
  }
}

// At saturated unsigned short operands the scalar pmul family must wrap, not overflow the int that
// integral promotion would otherwise multiply in.
void packetmath_unsigned_short() {
  // Volatile inputs keep sanitizer builds from folding away the promotions.
  volatile unsigned short values[] = {0, 1, 32768, 40232, 58075, 65535};
  const unsigned short c = 65535;
  for (unsigned short a : values) {
    for (unsigned short b : values) {
      VERIFY_IS_EQUAL(internal::pmul(a, b), REF_MUL(a, b));
      VERIFY_IS_EQUAL(internal::pmadd(a, b, c), REF_MADD(a, b, c));
      VERIFY_IS_EQUAL(internal::pmsub(a, b, c), REF_MSUB(a, b, c));
      VERIFY_IS_EQUAL(internal::pnmadd(a, b, c), REF_NMADD(a, b, c));
      VERIFY_IS_EQUAL(internal::pnmsub(a, b, c), REF_NMSUB(a, b, c));
    }
  }
}

void packetmath_bfloat16_abs_array() {
  // Keep public evaluator coverage with complete packets and a scalar tail.
  constexpr Index count = 2 * internal::packet_traits<bfloat16>::size + 1;
  const numext::uint16_t samples[] = {0x8000, 0x8001, 0x807f, 0x8080, 0xbf80, 0xff80, 0xff81, 0xffff};
  Array<bfloat16, Dynamic, 1> input(count), result(count);
  for (Index i = 0; i < count; ++i) {
    input(i) = numext::bit_cast<bfloat16>(samples[i % 8]);
  }
  input(count - 1) = numext::bit_cast<bfloat16>(numext::uint16_t(0xffff));
  result = input.abs();
  for (Index i = 0; i < count; ++i) {
    const auto expected = static_cast<numext::uint16_t>(numext::bit_cast<numext::uint16_t>(input(i)) & 0x7fff);
    VERIFY_IS_EQUAL(numext::bit_cast<numext::uint16_t>(result(i)), expected);
  }
}

namespace Eigen {
namespace test {

template <typename Scalar, typename PacketType>
struct runall<Scalar, PacketType, false, false> {  // i.e. float or double
  static void run() {
    if (g_first_pass) packetmath_abs_bits<Scalar, PacketType>();
    packetmath<Scalar, PacketType>();
    packetmath_scatter_gather<Scalar, PacketType>();
    packetmath_notcomplex<Scalar, PacketType>();
    packetmath_real<Scalar, PacketType>();
  }
};

template <typename Scalar, typename PacketType>
struct runall<Scalar, PacketType, false, true> {  // i.e. int
  static void run() {
    packetmath<Scalar, PacketType>();
    packetmath_scatter_gather<Scalar, PacketType>();
    packetmath_notcomplex<Scalar, PacketType>();
  }
};

template <typename Scalar, typename PacketType>
struct runall<Scalar, PacketType, true, false> {  // i.e. complex
  static void run() {
    packetmath<Scalar, PacketType>();
    packetmath_scatter_gather<Scalar, PacketType>();
    packetmath_complex<Scalar, PacketType>();
  }
};

}  // namespace test
}  // namespace Eigen

EIGEN_DECLARE_TEST(packetmath) {
  g_first_pass = true;
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(test::runner<float>::run());
    CALL_SUBTEST_2(test::runner<double>::run());
    CALL_SUBTEST_3(test::runner<int8_t>::run());
    CALL_SUBTEST_4(test::runner<uint8_t>::run());
    CALL_SUBTEST_5(test::runner<int16_t>::run());
    CALL_SUBTEST_6(test::runner<uint16_t>::run());
    CALL_SUBTEST_6(packetmath_unsigned_short());
    CALL_SUBTEST_7(test::runner<int32_t>::run());
    CALL_SUBTEST_8(test::runner<uint32_t>::run());
    CALL_SUBTEST_9(test::runner<int64_t>::run());
    CALL_SUBTEST_10(test::runner<uint64_t>::run());
    CALL_SUBTEST_11(test::runner<std::complex<float>>::run());
    CALL_SUBTEST_12(test::runner<std::complex<double>>::run());
    CALL_SUBTEST_13(test::runner<half>::run());
    CALL_SUBTEST_14((packetmath<bool, internal::packet_traits<bool>::type>()));
    CALL_SUBTEST_14((packetmath_scatter_gather<bool, internal::packet_traits<bool>::type>()));
#if defined(EIGEN_VECTORIZE_SSE2)
    CALL_SUBTEST_14(packetmath_packet16b_reductions());
    CALL_SUBTEST_14(packetmath_packet16b_select());
#endif
    CALL_SUBTEST_15(test::runner<bfloat16>::run());
    g_first_pass = false;
  }

  CALL_SUBTEST_15(packetmath_bfloat16_abs_array());
  CALL_SUBTEST_15({
    ScopedFlushToZero flush_to_zero;
    packetmath_bfloat16_abs_array();
  });

#if defined(EIGEN_VECTORIZE_RVV10)
  CALL_SUBTEST_1((packetmath_redux_infinities<float, internal::Packet1Xf>()));
  CALL_SUBTEST_1((packetmath_redux_infinities<float, internal::Packet2Xf>()));
  CALL_SUBTEST_1((packetmath_redux_infinities<float, internal::Packet4Xf>()));
  CALL_SUBTEST_2((packetmath_redux_infinities<double, internal::Packet1Xd>()));
  CALL_SUBTEST_2((packetmath_redux_infinities<double, internal::Packet2Xd>()));
  CALL_SUBTEST_2((packetmath_redux_infinities<double, internal::Packet4Xd>()));
#endif

#if defined(EIGEN_VECTORIZE_RVV10FP16)
  CALL_SUBTEST_13((packetmath_redux_infinities<half, internal::Packet1Xh>()));
  CALL_SUBTEST_13((packetmath_redux_infinities<half, internal::Packet2Xh>()));
#endif
}
