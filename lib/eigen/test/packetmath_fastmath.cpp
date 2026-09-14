// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <cstring>

#include "main.h"
#include "fp_control.h"

template <typename Scalar, typename Packet>
EIGEN_DONT_INLINE void store_ptrue(Scalar* output) {
  const Packet zero = Eigen::internal::pset1<Packet>(Scalar(0));
  Eigen::internal::pstoreu<Scalar, Packet>(output, Eigen::internal::ptrue(zero));
}

// The out-of-line boundary keeps the mask opaque at the packet-op call site, so
// the compiler must emit the mask test instead of folding it against the known
// lane values.
template <typename Scalar, typename Packet>
EIGEN_DONT_INLINE void select_with_mask(const Scalar* mask, const Scalar* a, const Scalar* b, Scalar* output) {
  const Packet selected = Eigen::internal::pselect(
      Eigen::internal::ploadu<Packet>(mask), Eigen::internal::ploadu<Packet>(a), Eigen::internal::ploadu<Packet>(b));
  Eigen::internal::pstoreu<Scalar, Packet>(output, selected);
}

template <typename Scalar, typename Packet>
EIGEN_DONT_INLINE bool mask_any(const Scalar* mask) {
  return Eigen::internal::predux_any(Eigen::internal::ploadu<Packet>(mask));
}

template <typename Scalar, typename Packet>
EIGEN_DONT_INLINE bool mask_all(const Scalar* mask) {
  return Eigen::internal::predux_all(Eigen::internal::ploadu<Packet>(mask));
}

template <typename Scalar, typename Packet>
void verify_mask_reduction_impl() {
  constexpr int packet_size = Eigen::internal::unpacket_traits<Packet>::size;
  Scalar mask[packet_size];

  std::memset(static_cast<void*>(mask), 0, sizeof(mask));
  VERIFY(!(mask_any<Scalar, Packet>(mask)));
  VERIFY(!(mask_all<Scalar, Packet>(mask)));
  VERIFY_IS_EQUAL(Eigen::internal::predux_count(Eigen::internal::ploadu<Packet>(mask)), 0);

  for (int lane = 0; lane < packet_size; ++lane) {
    std::memset(static_cast<void*>(mask), 0, sizeof(mask));
    std::memset(static_cast<void*>(mask + lane), 0xff, sizeof(Scalar));
    VERIFY((mask_any<Scalar, Packet>(mask)));
    VERIFY_IS_EQUAL((mask_all<Scalar, Packet>(mask)), packet_size == 1);
    VERIFY_IS_EQUAL(Eigen::internal::predux_count(Eigen::internal::ploadu<Packet>(mask)), 1);
  }

  std::memset(static_cast<void*>(mask), 0xff, sizeof(mask));
  VERIFY((mask_all<Scalar, Packet>(mask)));
}

template <typename Scalar, typename Packet>
void verify_mask_reduction() {
  verify_mask_reduction_impl<Scalar, Packet>();
  const Eigen::ScopedFlushToZero flush_to_zero;
  if (flush_to_zero.isSupported()) verify_mask_reduction_impl<Scalar, Packet>();
}

// For bit-test backends, a low-bit truth mask must survive FTZ even though its floating-point encoding is subnormal.
// Do not apply this to backends that consume only the sign bits of canonical comparison masks.
template <typename Scalar, typename Packet>
void verify_low_bit_mask_any_ftz() {
  const Eigen::ScopedFlushToZero flush_to_zero;
  VERIFY(flush_to_zero.isSupported());
  constexpr int packet_size = Eigen::internal::unpacket_traits<Packet>::size;
  using Bits = typename Eigen::numext::get_integer_by_size<sizeof(Scalar)>::unsigned_type;
  const Bits true_bits = 1;
  Scalar mask[packet_size];
  std::memset(static_cast<void*>(mask), 0, sizeof(mask));
  VERIFY(!(mask_any<Scalar, Packet>(mask)));
  for (int lane = 0; lane < packet_size; ++lane) {
    std::memset(static_cast<void*>(mask), 0, sizeof(mask));
    std::memcpy(static_cast<void*>(mask + lane), &true_bits, sizeof(Scalar));
    VERIFY((mask_any<Scalar, Packet>(mask)));
  }
  for (int lane = 0; lane < packet_size; ++lane) {
    std::memcpy(static_cast<void*>(mask + lane), &true_bits, sizeof(Scalar));
  }
  VERIFY((mask_any<Scalar, Packet>(mask)));
}

// Keep the finite inputs opaque while compiling the reduction itself with fast-math optimizations.
template <typename Scalar, typename Packet>
EIGEN_DONT_INLINE void reduce_minmax(const Scalar* input, Scalar* output) {
  const Packet packet = Eigen::internal::ploadu<Packet>(input);
  output[0] = Eigen::internal::predux_min(packet);
  output[1] = Eigen::internal::predux_max(packet);
}

template <typename Scalar, typename Packet>
void verify_minmax_reduction() {
  constexpr int packet_size = Eigen::internal::unpacket_traits<Packet>::size;
  Scalar input[packet_size];
  for (int i = 0; i < packet_size; ++i) {
    input[i] = Scalar(i + 1);
  }

  Scalar output[2];
  reduce_minmax<Scalar, Packet>(input, output);
  VERIFY_IS_EQUAL(output[0], Scalar(1));
  VERIFY_IS_EQUAL(output[1], Scalar(packet_size));
}

// Complementary to the opaque-mask calls: the all-ones mask flows straight
// from ptrue into the packet op, the way comparison-mask producers feed it,
// which gives constant folding under -ffinite-math-only a chance to see the
// NaN bit pattern.
template <typename Scalar, typename Packet>
EIGEN_DONT_INLINE bool ptrue_mask_any() {
  const Packet zero = Eigen::internal::pset1<Packet>(Scalar(0));
  return Eigen::internal::predux_any(Eigen::internal::ptrue(zero));
}

template <typename Scalar, typename Packet>
EIGEN_DONT_INLINE void select_with_ptrue_mask(const Scalar* a, const Scalar* b, Scalar* output) {
  const Packet zero = Eigen::internal::pset1<Packet>(Scalar(0));
  const Packet selected = Eigen::internal::pselect(Eigen::internal::ptrue(zero), Eigen::internal::ploadu<Packet>(a),
                                                   Eigen::internal::ploadu<Packet>(b));
  Eigen::internal::pstoreu<Scalar, Packet>(output, selected);
}

template <typename Scalar>
EIGEN_DONT_INLINE void store_extended_scalar_constants(Scalar* output) {
  output[0] = Eigen::internal::psignmask<Scalar>();
  output[1] = Eigen::internal::pinf<Scalar>();
  output[2] = Eigen::internal::pnan<Scalar>();
}

template <typename Scalar>
EIGEN_DONT_INLINE Eigen::numext::uint32_t extended_to_float_bits(const volatile Scalar* input) {
  const float narrowed = static_cast<float>(*input);
  return Eigen::numext::bit_cast<Eigen::numext::uint32_t>(narrowed);
}

template <typename Scalar, bool Vectorizable = Eigen::internal::packet_traits<Scalar>::Vectorizable>
struct packetmath_fastmath_runner {
  static void run() {}
};

template <typename Scalar>
struct packetmath_fastmath_runner<Scalar, true> {
  static void run() {
    typedef typename Eigen::internal::packet_traits<Scalar>::type Packet;
    const int packet_size = Eigen::internal::packet_traits<Scalar>::size;
    Scalar output[packet_size];
    for (int i = 0; i < packet_size; ++i) {
      output[i] = Scalar(0);
    }

    store_ptrue<Scalar, Packet>(output);

    for (int i = 0; i < packet_size; ++i) {
      const unsigned char* lane_bytes = reinterpret_cast<const unsigned char*>(output + i);
      bool has_nonzero_byte = false;
      for (std::size_t j = 0; j < sizeof(Scalar); ++j) {
        has_nonzero_byte = has_nonzero_byte || lane_bytes[j] != 0;
      }
      VERIFY(has_nonzero_byte);
    }

    // pselect and predux_any consume comparison masks whose "true" lanes are
    // all-ones bit patterns, i.e. NaN when reinterpreted as floating point. An
    // implementation that tests such a mask with a floating-point compare
    // invites -ffinite-math-only to drop the unordered case and mishandle the
    // "true" lanes, so exercise all-zero, all-ones, and single-lane masks here.
    Scalar mask[packet_size];
    Scalar a[packet_size];
    Scalar b[packet_size];
    Scalar selected[packet_size];
    for (int i = 0; i < packet_size; ++i) {
      a[i] = Scalar(i + 1);
      b[i] = Scalar(-(i + 1));
    }

    std::memset(static_cast<void*>(mask), 0, sizeof(mask));
    select_with_mask<Scalar, Packet>(mask, a, b, selected);
    for (int i = 0; i < packet_size; ++i) {
      VERIFY_IS_EQUAL(selected[i], b[i]);
    }
    VERIFY(!(mask_any<Scalar, Packet>(mask)));

    std::memset(static_cast<void*>(mask), 0xff, sizeof(mask));
    select_with_mask<Scalar, Packet>(mask, a, b, selected);
    for (int i = 0; i < packet_size; ++i) {
      VERIFY_IS_EQUAL(selected[i], a[i]);
    }
    VERIFY((mask_any<Scalar, Packet>(mask)));

    select_with_ptrue_mask<Scalar, Packet>(a, b, selected);
    for (int i = 0; i < packet_size; ++i) {
      VERIFY_IS_EQUAL(selected[i], a[i]);
    }
    VERIFY((ptrue_mask_any<Scalar, Packet>()));

    for (int lane = 0; lane < packet_size; ++lane) {
      std::memset(static_cast<void*>(mask), 0, sizeof(mask));
      std::memset(static_cast<void*>(mask + lane), 0xff, sizeof(Scalar));
      select_with_mask<Scalar, Packet>(mask, a, b, selected);
      for (int i = 0; i < packet_size; ++i) {
        VERIFY_IS_EQUAL(selected[i], i == lane ? a[i] : b[i]);
      }
      VERIFY((mask_any<Scalar, Packet>(mask)));
    }

    verify_mask_reduction<Scalar, Packet>();
    verify_minmax_reduction<Scalar, Packet>();
  }
};

template <typename Scalar,
          bool HasIntegerBits =
              !std::is_void<typename Eigen::numext::get_integer_by_size<sizeof(Scalar)>::unsigned_type>::value>
struct extended_scalar_constant_runner {
  static void run() {}
};

template <typename Scalar>
struct extended_scalar_constant_runner<Scalar, false> {
  static void run() {
    Scalar actual[3];
    store_extended_scalar_constants(actual);

    // Floating-point classification is optimized away under -ffinite-math-only. Narrow through a volatile pointer and
    // inspect the resulting integer bits instead; this also avoids indeterminate padding in x87 long double objects.
    typedef Eigen::numext::uint32_t Bits;
    const Bits sign_bits = extended_to_float_bits(actual + 0);
    const Bits inf_bits = extended_to_float_bits(actual + 1);
    const Bits nan_bits = extended_to_float_bits(actual + 2);
    VERIFY_IS_EQUAL(sign_bits, Bits(0x80000000u));
    VERIFY_IS_EQUAL(inf_bits, Bits(0x7f800000u));
    VERIFY_IS_EQUAL(nan_bits & Bits(0x7fc00000u), Bits(0x7fc00000u));
  }
};

EIGEN_DECLARE_TEST(packetmath_fastmath) {
  CALL_SUBTEST(packetmath_fastmath_runner<float>::run());
  CALL_SUBTEST(packetmath_fastmath_runner<double>::run());
  CALL_SUBTEST(packetmath_fastmath_runner<Eigen::half>::run());
  CALL_SUBTEST(packetmath_fastmath_runner<Eigen::bfloat16>::run());
  CALL_SUBTEST(extended_scalar_constant_runner<long double>::run());

#if defined(EIGEN_VECTORIZE_RVV10)
  CALL_SUBTEST((verify_mask_reduction<float, Eigen::internal::Packet1Xf>()));
  CALL_SUBTEST((verify_mask_reduction<float, Eigen::internal::Packet2Xf>()));
  CALL_SUBTEST((verify_mask_reduction<float, Eigen::internal::Packet4Xf>()));
  CALL_SUBTEST((verify_mask_reduction<double, Eigen::internal::Packet1Xd>()));
  CALL_SUBTEST((verify_mask_reduction<double, Eigen::internal::Packet2Xd>()));
  CALL_SUBTEST((verify_mask_reduction<double, Eigen::internal::Packet4Xd>()));
  CALL_SUBTEST((verify_minmax_reduction<float, Eigen::internal::Packet1Xf>()));
  CALL_SUBTEST((verify_minmax_reduction<float, Eigen::internal::Packet2Xf>()));
  CALL_SUBTEST((verify_minmax_reduction<float, Eigen::internal::Packet4Xf>()));
  CALL_SUBTEST((verify_minmax_reduction<double, Eigen::internal::Packet1Xd>()));
  CALL_SUBTEST((verify_minmax_reduction<double, Eigen::internal::Packet2Xd>()));
  CALL_SUBTEST((verify_minmax_reduction<double, Eigen::internal::Packet4Xd>()));
#endif

#if defined(EIGEN_VECTORIZE_RVV10FP16)
  CALL_SUBTEST((verify_mask_reduction<Eigen::half, Eigen::internal::Packet1Xh>()));
  CALL_SUBTEST((verify_mask_reduction<Eigen::half, Eigen::internal::Packet2Xh>()));
  CALL_SUBTEST((verify_minmax_reduction<Eigen::half, Eigen::internal::Packet1Xh>()));
  CALL_SUBTEST((verify_minmax_reduction<Eigen::half, Eigen::internal::Packet2Xh>()));
#endif

#if defined(EIGEN_VECTORIZE_RVV10BF16)
  CALL_SUBTEST((verify_mask_reduction<Eigen::bfloat16, Eigen::internal::Packet1Xbf>()));
  CALL_SUBTEST((verify_mask_reduction<Eigen::bfloat16, Eigen::internal::Packet2Xbf>()));
#endif

#if defined(EIGEN_VECTORIZE_NEON)
  CALL_SUBTEST((verify_mask_reduction<float, Eigen::internal::Packet2f>()));
  CALL_SUBTEST((verify_low_bit_mask_any_ftz<float, Eigen::internal::Packet2f>()));
  CALL_SUBTEST((verify_low_bit_mask_any_ftz<Eigen::bfloat16, Eigen::internal::Packet4bf>()));
#if EIGEN_ARCH_ARM64
  CALL_SUBTEST((verify_low_bit_mask_any_ftz<double, Eigen::internal::Packet2d>()));
#endif
#endif

#if defined(EIGEN_VECTORIZE_AVX512FP16)
  CALL_SUBTEST((verify_mask_reduction<Eigen::half, Eigen::internal::Packet16h>()));
  CALL_SUBTEST((verify_mask_reduction<Eigen::half, Eigen::internal::Packet8h>()));
  VERIFY((ptrue_mask_any<Eigen::half, Eigen::internal::Packet16h>()));
  VERIFY((ptrue_mask_any<Eigen::half, Eigen::internal::Packet8h>()));
  CALL_SUBTEST((verify_low_bit_mask_any_ftz<Eigen::half, Eigen::internal::Packet32h>()));
  CALL_SUBTEST((verify_low_bit_mask_any_ftz<Eigen::half, Eigen::internal::Packet16h>()));
  CALL_SUBTEST((verify_low_bit_mask_any_ftz<Eigen::half, Eigen::internal::Packet8h>()));
#endif
}
