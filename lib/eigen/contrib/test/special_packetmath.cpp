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

#include <limits>
#include "packetmath_test_shared.h"
#include "fp_control.h"
#include "../Eigen/SpecialFunctions"

using internal::unpacket_traits;

#if EIGEN_ARCH_ARM && defined(EIGEN_VECTORIZE_NEON)
template <typename Scalar, typename Packet>
void packet_erf_subnormals() {
  using Bits = typename numext::get_integer_by_size<sizeof(Scalar)>::unsigned_type;
  constexpr int PacketSize = unpacket_traits<Packet>::size;
  constexpr Bits Sign = Bits(1) << (8 * sizeof(Scalar) - 1);
  constexpr Bits MinNormal = Bits(1) << (std::numeric_limits<Scalar>::digits - 1);
  const Bits samples[] = {Bits(1), Bits(2), Bits(3), Bits(MinNormal / 2), Bits(MinNormal - 1)};
  // In this interval the cubic term of erf is negligible even relative to one subnormal ULP.
  const double slope = 2.0 / std::sqrt(std::acos(-1.0));
  for (Bits n : samples) {
    for (Bits sign : {Bits(0), Sign}) {
      for (int lane = 0; lane < PacketSize; ++lane) {
        Scalar input[PacketSize], output[PacketSize];
        for (int i = 0; i < PacketSize; ++i) input[i] = Scalar(0.5);
        input[lane] = numext::bit_cast<Scalar>(Bits(sign | n));
        internal::pstoreu(output, internal::perf(internal::ploadu<Packet>(input)));
        const Bits actual = numext::bit_cast<Bits>(output[lane]);
        const Bits expected = Bits(std::floor(double(n) * slope + 0.5));
        VERIFY_IS_EQUAL(Bits(actual & Sign), sign);
        const int error = int(actual & (Sign - 1)) - int(expected);
        VERIFY(std::abs(error) <= (sizeof(Scalar) == sizeof(float) ? 1 : 0));
        if (n == 1) VERIFY_IS_EQUAL(actual, Bits(sign | n));
        for (int i = 0; i < PacketSize; ++i) {
          if (i != lane) VERIFY_IS_APPROX(output[i], Scalar(std::erf(0.5)));
        }
      }
    }
  }

  // Recovering one lane must not change zeros, infinities, or NaNs in its neighbors.
  const Scalar neighbors[] = {Scalar(0), numext::bit_cast<Scalar>(Sign), std::numeric_limits<Scalar>::infinity(),
                              std::numeric_limits<Scalar>::quiet_NaN()};
  for (Scalar neighbor : neighbors) {
    Scalar values[PacketSize], before[PacketSize], after[PacketSize];
    for (int i = 0; i < PacketSize; ++i) values[i] = neighbor;
    internal::pstoreu(before, internal::perf(internal::ploadu<Packet>(values)));
    if ((numext::bit_cast<Bits>(neighbor) & (Sign - 1)) == 0) {
      for (int i = 0; i < PacketSize; ++i)
        VERIFY_IS_EQUAL(numext::bit_cast<Bits>(before[i]), numext::bit_cast<Bits>(neighbor));
    }
    values[0] = numext::bit_cast<Scalar>(Bits(1));
    internal::pstoreu(after, internal::perf(internal::ploadu<Packet>(values)));
    VERIFY_IS_EQUAL(numext::bit_cast<Bits>(after[0]), Bits(1));
    for (int i = 1; i < PacketSize; ++i)
      VERIFY_IS_EQUAL(numext::bit_cast<Bits>(after[i]), numext::bit_cast<Bits>(before[i]));
  }

  // Check the public evaluator with packet bodies and an ordinary scalar tail, also while scalar arithmetic flushes.
  Array<Scalar, Dynamic, 1> input(3 * internal::packet_traits<Scalar>::size + 1), output(input.size());
  for (Index i = 0; i + 1 < input.size(); ++i)
    input(i) = numext::bit_cast<Scalar>(Bits(Bits(1) | (i % 2 ? Sign : Bits(0))));
  input(input.size() - 1) = Scalar(0.5);
  output = input.erf();
  for (Index i = 0; i + 1 < input.size(); ++i)
    VERIFY_IS_EQUAL(numext::bit_cast<Bits>(output(i)), numext::bit_cast<Bits>(input(i)));
  VERIFY_IS_APPROX(output(output.size() - 1), Scalar(std::erf(0.5)));
}

void neon_erf_subnormals_float() {
  packet_erf_subnormals<float, internal::Packet2f>();
  packet_erf_subnormals<float, internal::Packet4f>();

  // Rounded values straddling the subnormal/normal output boundary and the largest subnormal input.
  const numext::uint32_t inputs[] = {0x00716fe1u, 0x00716fe2u, 0x00716fe3u, 0x007fffffu};
  const numext::uint32_t expected[] = {0x007fffffu, 0x00800000u, 0x00800001u, 0x00906eb9u};
  float input[4], output[4];
  for (int i = 0; i < 4; ++i) input[i] = numext::bit_cast<float>(inputs[i]);
  internal::pstoreu(output, internal::perf(internal::ploadu<internal::Packet4f>(input)));
  for (int i = 0; i < 4; ++i) {
    const int error = int(numext::bit_cast<numext::uint32_t>(output[i])) - int(expected[i]);
    VERIFY(std::abs(error) <= 1);
  }
}

void neon_erf_subnormals_bfloat16() {
  packet_erf_subnormals<bfloat16, internal::Packet4bf>();
  // Exhaustive bfloat16 subnormals, with both signs, checked without floating-point subnormal conversions.
  for (unsigned int n = 1; n < 128; ++n) {
    const numext::uint16_t expectedBits =
        numext::uint16_t(std::floor(double(n) * (2.0 / std::sqrt(std::acos(-1.0))) + 0.5));
    bfloat16 values[4], result[4];
    for (int i = 0; i < 4; ++i) values[i] = numext::bit_cast<bfloat16>(numext::uint16_t(n | (i % 2 ? 0x8000 : 0)));
    internal::pstoreu(result, internal::perf(internal::ploadu<internal::Packet4bf>(values)));
    for (int i = 0; i < 4; ++i)
      VERIFY_IS_EQUAL(numext::bit_cast<numext::uint16_t>(result[i]),
                      numext::uint16_t(expectedBits | (i % 2 ? 0x8000 : 0)));
  }
}
#endif

#if EIGEN_ARCH_ARM
// Note: 32-bit arm always flushes subnormals to zero.
#define MAYBE_FLUSH(op)                                                      \
  [](Scalar x) {                                                             \
    Scalar y = static_cast<Scalar>(op(x));                                   \
    if (Eigen::numext::abs(y) < (std::numeric_limits<decltype(y)>::min)()) { \
      y = y * decltype(y)(0); /* Preserve sign. */                           \
    }                                                                        \
    return y;                                                                \
  }
#else
#define MAYBE_FLUSH(op) op
#endif

template <typename Scalar, typename Packet>
void packetmath_real() {
  using std::abs;
  typedef internal::packet_traits<Scalar> PacketTraits;
  const int PacketSize = internal::unpacket_traits<Packet>::size;

  const int size = PacketSize * 4;
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data1[PacketSize * 4] = {};
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar data2[PacketSize * 4] = {};
  EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) Scalar ref[PacketSize * 4] = {};

  {
    data1[0] = std::numeric_limits<Scalar>::quiet_NaN();
    test::packet_helper<internal::packet_traits<Scalar>::HasLGamma, Packet> h;
    h.store(data2, internal::plgamma(h.load(data1)));
    VERIFY((numext::isnan)(data2[0]));
  }
  if (internal::packet_traits<Scalar>::HasErf) {
    data1[0] = std::numeric_limits<Scalar>::quiet_NaN();
    data1[1] = std::numeric_limits<Scalar>::infinity();
    data1[2] = -std::numeric_limits<Scalar>::infinity();
    data1[3] = (std::numeric_limits<Scalar>::max)();
    if (size >= 8) {
      data1[4] = -(std::numeric_limits<Scalar>::max)();
      if (sizeof(Scalar) >= 8) {
        data1[5] = Scalar(1e200);
        data1[6] = Scalar(-1e200);
      } else {
        data1[5] = Scalar(1e30f);
        data1[6] = Scalar(-1e30f);
      }
      data1[7] = std::numeric_limits<Scalar>::denorm_min();
    }
    test::packet_helper<internal::packet_traits<Scalar>::HasErf, Packet> h;
    for (int i = 0; i < size; i += PacketSize) {
      h.store(data2 + i, internal::perf(h.load(data1 + i)));
    }
    VERIFY((numext::isnan)(data2[0]));
    VERIFY_IS_EQUAL(data2[1], Scalar(1));
    VERIFY_IS_EQUAL(data2[2], Scalar(-1));
    VERIFY_IS_EQUAL(data2[3], Scalar(1));
    if (size >= 8) {
      VERIFY_IS_EQUAL(data2[4], Scalar(-1));
      VERIFY_IS_EQUAL(data2[5], Scalar(1));
      VERIFY_IS_EQUAL(data2[6], Scalar(-1));
      if (data1[7] > Scalar(0)) {
        VERIFY((data2[7] > Scalar(0)));
      }
    }
  }
  if (internal::packet_traits<Scalar>::HasErfc) {
    data1[0] = std::numeric_limits<Scalar>::quiet_NaN();
    data1[1] = std::numeric_limits<Scalar>::infinity();
    data1[2] = -std::numeric_limits<Scalar>::infinity();
    data1[3] = (std::numeric_limits<Scalar>::max)();
    if (size >= 8) {
      data1[4] = -(std::numeric_limits<Scalar>::max)();
      if (sizeof(Scalar) >= 8) {
        data1[5] = Scalar(1e200);
        data1[6] = Scalar(-1e200);
      } else {
        data1[5] = Scalar(1e30f);
        data1[6] = Scalar(-1e30f);
      }
    }
    test::packet_helper<internal::packet_traits<Scalar>::HasErfc, Packet> h;
    for (int i = 0; i < size; i += PacketSize) {
      h.store(data2 + i, internal::perfc(h.load(data1 + i)));
    }
    VERIFY((numext::isnan)(data2[0]));
    VERIFY_IS_EQUAL(data2[1], Scalar(0));
    VERIFY_IS_EQUAL(data2[2], Scalar(2));
    VERIFY_IS_EQUAL(data2[3], Scalar(0));
    if (size >= 8) {
      VERIFY_IS_EQUAL(data2[4], Scalar(2));
      VERIFY_IS_EQUAL(data2[5], Scalar(0));
      VERIFY_IS_EQUAL(data2[6], Scalar(2));
    }
  }
  {
    for (int i = 0; i < size; ++i) {
      data1[i] = internal::random<Scalar>(Scalar(0), Scalar(1));
    }
    CHECK_CWISE1_IF(internal::packet_traits<Scalar>::HasNdtri, numext::ndtri, internal::pndtri);
  }

  // For bessel_i*e and bessel_j*, the valid range is negative reals.
  {
    const int max_exponent = numext::mini(std::numeric_limits<Scalar>::max_exponent10 - 1, 6);
    for (int i = 0; i < size; ++i) {
      data1[i] = internal::random<Scalar>(Scalar(-1), Scalar(1)) *
                 Scalar(std::pow(Scalar(10), internal::random<Scalar>(Scalar(-max_exponent), Scalar(max_exponent))));
      data2[i] = internal::random<Scalar>(Scalar(-1), Scalar(1)) *
                 Scalar(std::pow(Scalar(10), internal::random<Scalar>(Scalar(-max_exponent), Scalar(max_exponent))));
    }

    CHECK_CWISE1_IF(PacketTraits::HasBessel, numext::bessel_i0e, internal::pbessel_i0e);
    CHECK_CWISE1_IF(PacketTraits::HasBessel, numext::bessel_i1e, internal::pbessel_i1e);
    CHECK_CWISE1_IF(PacketTraits::HasBessel, numext::bessel_j0, internal::pbessel_j0);
    CHECK_CWISE1_IF(PacketTraits::HasBessel, numext::bessel_j1, internal::pbessel_j1);
  }

  // Use a smaller data range for the bessel_i* as these can become very large.
  // Following #1693, we also restrict this range further to avoid inf's due to
  // differences in pexp and exp.
  for (int i = 0; i < size; ++i) {
    data1[i] = internal::random<Scalar>(Scalar(0.01), Scalar(1)) *
               Scalar(std::pow(Scalar(9), internal::random<Scalar>(Scalar(-1), Scalar(2))));
    data2[i] = internal::random<Scalar>(Scalar(0.01), Scalar(1)) *
               Scalar(std::pow(Scalar(9), internal::random<Scalar>(Scalar(-1), Scalar(2))));
  }
  CHECK_CWISE1_IF(PacketTraits::HasBessel, numext::bessel_i0, internal::pbessel_i0);
  CHECK_CWISE1_IF(PacketTraits::HasBessel, numext::bessel_i1, internal::pbessel_i1);

  // Boundary values for which a naive i0(x) = exp(|x|) * i0e(x) overflows even though the result is
  // finite.  CHECK_CWISE1_IF cannot detect that on its own: its reference is the scalar path through
  // the same generic_i0, so both sides would be +inf and compare equal.  Check finiteness explicitly.
  if (PacketTraits::HasBessel &&
      (internal::is_same<Scalar, float>::value || internal::is_same<Scalar, double>::value)) {
    Scalar boundary = internal::is_same<Scalar, float>::value ? Scalar(90) : Scalar(713);
    for (int i = 0; i < size; ++i) {
      data1[i] = (i % 2 == 0) ? boundary : -boundary;
    }
    CHECK_CWISE1_IF(PacketTraits::HasBessel, numext::bessel_i0, internal::pbessel_i0);
    for (int i = 0; i < PacketSize; ++i) VERIFY((numext::isfinite)(data2[i]));
    CHECK_CWISE1_IF(PacketTraits::HasBessel, numext::bessel_i1, internal::pbessel_i1);
    for (int i = 0; i < PacketSize; ++i) VERIFY((numext::isfinite)(data2[i]));
  }

  // y_i, and k_i are valid for x > 0.
  {
    const int max_exponent = numext::mini(std::numeric_limits<Scalar>::max_exponent10 - 1, 5);
    for (int i = 0; i < size; ++i) {
      data1[i] = internal::random<Scalar>(Scalar(0.01), Scalar(1)) *
                 Scalar(std::pow(Scalar(10), internal::random<Scalar>(Scalar(-2), Scalar(max_exponent))));
      data2[i] = internal::random<Scalar>(Scalar(0.01), Scalar(1)) *
                 Scalar(std::pow(Scalar(10), internal::random<Scalar>(Scalar(-2), Scalar(max_exponent))));
    }
  }

  // TODO(srvasude): Re-enable this test once properly investigated why the
  // scalar and vector paths differ.
  // CHECK_CWISE1_IF(PacketTraits::HasBessel, numext::bessel_y0, internal::pbessel_y0);
  CHECK_CWISE1_IF(PacketTraits::HasBessel, numext::bessel_y1, internal::pbessel_y1);
  CHECK_CWISE1_IF(PacketTraits::HasBessel, numext::bessel_k0e, internal::pbessel_k0e);
  CHECK_CWISE1_IF(PacketTraits::HasBessel, numext::bessel_k1e, internal::pbessel_k1e);

  // Following #1693, we restrict the range for exp to avoid zeroing out too
  // fast.
  for (int i = 0; i < size; ++i) {
    data1[i] = internal::random<Scalar>(Scalar(0.01), Scalar(1)) *
               Scalar(std::pow(Scalar(9), internal::random<Scalar>(Scalar(-1), Scalar(2))));
    data2[i] = internal::random<Scalar>(Scalar(0.01), Scalar(1)) *
               Scalar(std::pow(Scalar(9), internal::random<Scalar>(Scalar(-1), Scalar(2))));
  }
  CHECK_CWISE1_IF(PacketTraits::HasBessel, numext::bessel_k0, internal::pbessel_k0);
  CHECK_CWISE1_IF(PacketTraits::HasBessel, numext::bessel_k1, internal::pbessel_k1);

  for (int i = 0; i < size; ++i) {
    data1[i] = internal::random<Scalar>(Scalar(0.01), Scalar(1)) *
               Scalar(std::pow(Scalar(10), internal::random<Scalar>(Scalar(-1), Scalar(2))));
    data2[i] = internal::random<Scalar>(Scalar(0.01), Scalar(1)) *
               Scalar(std::pow(Scalar(10), internal::random<Scalar>(Scalar(-1), Scalar(2))));
  }

  CHECK_CWISE1_IF(internal::packet_traits<Scalar>::HasLGamma, std::lgamma, internal::plgamma);
  CHECK_CWISE1_IF(internal::packet_traits<Scalar>::HasErf, std::erf, internal::perf);
  // FIXME(rmlarsen): This test occasionally fails due to difference in tiny subnormal results
  // near the underflow boundary. I am not sure which version is correct.
  CHECK_CWISE1_IF(internal::packet_traits<Scalar>::HasErfc, MAYBE_FLUSH(std::erfc), internal::perfc);
}

namespace Eigen {
namespace test {

template <typename Scalar, typename PacketType, bool IsComplex, bool IsInteger>
struct runall {
  static void run() { packetmath_real<Scalar, PacketType>(); }
};

}  // namespace test
}  // namespace Eigen

EIGEN_DECLARE_TEST(special_packetmath) {
#if EIGEN_ARCH_ARM && defined(EIGEN_VECTORIZE_NEON)
  CALL_SUBTEST_1(neon_erf_subnormals_float());
  CALL_SUBTEST_4(neon_erf_subnormals_bfloat16());
  {
    const Eigen::ScopedFlushToZero flush_to_zero;
    CALL_SUBTEST_1(neon_erf_subnormals_float());
    CALL_SUBTEST_4(neon_erf_subnormals_bfloat16());
  }
#endif
  g_first_pass = true;
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(test::runner<float>::run());
    CALL_SUBTEST_2(test::runner<double>::run());
    CALL_SUBTEST_3(test::runner<Eigen::half>::run());
    CALL_SUBTEST_4(test::runner<Eigen::bfloat16>::run());
    g_first_pass = false;
  }
}
