// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

// isinf/isnan/isfinite of every bfloat16 encoding, scalar, packet and through the public array predicates. The
// inputs are built from raw bits and the expectations from the exponent and mantissa fields in integer arithmetic,
// so nothing here can fold under -ffast-math: bfloat16_classification_fastmath compiles this file with that flag,
// under which a classification of the widened float folds to "not NaN, not inf, finite".

#include "main.h"

namespace {

using Eigen::numext::uint16_t;

struct Classification {
  bool is_inf, is_nan, is_finite;
};

// An exponent field of all ones is an infinity with a zero mantissa and a NaN otherwise.
Classification classify(uint16_t bits) {
  const bool exponent_all_ones = (bits & 0x7f80) == 0x7f80;
  const bool mantissa_zero = (bits & 0x007f) == 0;
  return {exponent_all_ones && mantissa_zero, exponent_all_ones && !mantissa_zero, !exponent_all_ones};
}

bfloat16 from_bits(int bits) { return numext::bit_cast<bfloat16>(static_cast<uint16_t>(bits)); }
uint16_t bits_of(bfloat16 x) { return numext::bit_cast<uint16_t>(x); }

void bfloat16_scalar_classification() {
  for (int bits = 0; bits <= 0xffff; ++bits) {
    const Classification expected = classify(static_cast<uint16_t>(bits));
    const bfloat16 x = from_bits(bits);
    VERIFY_IS_EQUAL((numext::isinf)(x), expected.is_inf);
    VERIFY_IS_EQUAL((numext::isnan)(x), expected.is_nan);
    VERIFY_IS_EQUAL((numext::isfinite)(x), expected.is_finite);
  }
}

// A vectorized packet must return a full-bit mask per lane: all ones where the predicate holds, all zeros elsewhere.
template <typename Packet, bool Vectorized = internal::unpacket_traits<Packet>::vectorizable>
struct packet_classification {
  static void run() {
    constexpr int PacketSize = internal::unpacket_traits<Packet>::size;
    EIGEN_ALIGN_MAX bfloat16 in[PacketSize], is_inf[PacketSize], is_nan[PacketSize], is_finite[PacketSize];
    for (int start = 0; start <= 0xffff; start += PacketSize) {
      for (int lane = 0; lane < PacketSize; ++lane) in[lane] = from_bits(start + lane);
      const Packet a = internal::pload<Packet>(in);
      internal::pstore(is_inf, internal::pisinf(a));
      internal::pstore(is_nan, internal::pisnan(a));
      internal::pstore(is_finite, internal::pisfinite(a));
      for (int lane = 0; lane < PacketSize; ++lane) {
        const Classification expected = classify(static_cast<uint16_t>(start + lane));
        VERIFY_IS_EQUAL(bits_of(is_inf[lane]), expected.is_inf ? 0xffff : 0);
        VERIFY_IS_EQUAL(bits_of(is_nan[lane]), expected.is_nan ? 0xffff : 0);
        VERIFY_IS_EQUAL(bits_of(is_finite[lane]), expected.is_finite ? 0xffff : 0);
      }
    }
  }
};
template <typename Packet>
struct packet_classification<Packet, false> {
  static void run() {}
};

void bfloat16_packet_classification() {
  using Packet = internal::packet_traits<bfloat16>::type;
  using HalfPacket = internal::unpacket_traits<Packet>::half;
  packet_classification<Packet>::run();
  if (!std::is_same<Packet, HalfPacket>::value) packet_classification<HalfPacket>::run();
}

using ArrayType = Array<bfloat16, Dynamic, 1>;

// The bool predicates go through numext::is*; the typed ones through pis* on whole packets and numext::is* on the
// scalar tail, where a lane is the all-ones mask or Scalar(1), so only its truth value is checked. isFiniteTyped()
// is the public typed predicate; the isinf/isnan functors have no public entry point and are applied directly.
// (allFinite() is not checked: under __FINITE_MATH_ONLY__ it is true by definition, see all_finite_impl.)
void check_array_classification(const ArrayType& x) {
  const Array<bool, Dynamic, 1> is_inf = x.isInf(), is_nan = x.isNaN(), is_finite = x.isFinite();
  const ArrayType typed_inf = x.unaryExpr(internal::scalar_isinf_op<bfloat16, true>()),
                  typed_nan = x.unaryExpr(internal::scalar_isnan_op<bfloat16, true>()),
                  typed_finite = x.isFiniteTyped();
  for (Index i = 0; i < x.size(); ++i) {
    const Classification expected = classify(bits_of(x[i]));
    VERIFY_IS_EQUAL(is_inf[i], expected.is_inf);
    VERIFY_IS_EQUAL(is_nan[i], expected.is_nan);
    VERIFY_IS_EQUAL(is_finite[i], expected.is_finite);
    VERIFY_IS_EQUAL(bits_of(typed_inf[i]) != 0, expected.is_inf);
    VERIFY_IS_EQUAL(bits_of(typed_nan[i]) != 0, expected.is_nan);
    VERIFY_IS_EQUAL(bits_of(typed_finite[i]) != 0, expected.is_finite);
  }
}

void bfloat16_array_classification() {
  ArrayType all(0x10000);
  for (Index i = 0; i < all.size(); ++i) all[i] = from_bits(int(i));
  check_array_classification(all);
  // Lengths that are not a multiple of the packet size leave a scalar tail; over these lengths it crosses the
  // largest finite value, the infinity and the first NaNs of either sign.
  for (Index len = 1; len <= 33; ++len) {
    check_array_classification(all.segment(0x7f70, len));
    check_array_classification(all.segment(0xff70, len));
  }
}

}  // namespace

EIGEN_DECLARE_TEST(bfloat16_classification) {
  CALL_SUBTEST(bfloat16_scalar_classification());
  CALL_SUBTEST(bfloat16_packet_classification());
  CALL_SUBTEST(bfloat16_array_classification());
}
