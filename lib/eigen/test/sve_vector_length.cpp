// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

// SVE packet types are fixed-length vectors of exactly EIGEN_ARM64_SVE_VL bits. Running such a
// binary where the hardware vector length differs makes every packet type a lie and lets the
// rest of the suite pass vacuously. qemu is the common way in: above 512 bits it keeps the
// default 512-bit task vector length, so `-cpu max,sve2048=on` alone runs at 512 unless
// `sve-default-vector-length=<bytes>` is also given.

// The backend raises Eigen's own default to fit its blocked kernel panels, but an explicit limit
// is the caller's policy and must survive. 1 is below the requirement at every vector length, so
// the assertion is never vacuous, and nothing in this file allocates a fixed-size Eigen object.
#define EIGEN_STACK_ALLOCATION_LIMIT 1
#include "main.h"
static_assert(EIGEN_STACK_ALLOCATION_LIMIT == 1, "SVE must preserve an explicit stack allocation limit");

// Registered only under EIGEN_TEST_SVE (test/CMakeLists.txt). Without an SVE -march this would
// compile against NEON and trivially pass.
#if !defined(EIGEN_VECTORIZE_SVE)
#error \
    "sve_vector_length requires the SVE backend.  Build with an SVE -march (e.g. " \
    "-march=armv8.2-a+sve) and -DEIGEN_TEST_SVE=ON, which supplies -DEIGEN_ARM64_USE_SVE and " \
    "-msve-vector-bits."
#endif

// Must stay inline asm: under -msve-vector-bits the compiler folds svcntb() to the compile-time
// constant, making the comparison below a tautology that holds on a mismatched machine.
static EIGEN_DONT_INLINE int runtime_vector_length_bits() {
  numext::uint64_t vl_bytes;
  asm("rdvl %0, #1" : "=r"(vl_bytes));
  return int(vl_bytes) * CHAR_BIT;
}

void check_runtime_vector_length() { VERIFY_IS_EQUAL(runtime_vector_length_bits(), int(EIGEN_ARM64_SVE_VL)); }

// Cross-checked against RDVL rather than svcntw()/svcntd(), which fold as above.
void check_packet_sizes() {
  const int vl = runtime_vector_length_bits();
  VERIFY_IS_EQUAL(int(internal::packet_traits<numext::int32_t>::size), vl / (8 * int(sizeof(numext::int32_t))));
  VERIFY_IS_EQUAL(int(internal::packet_traits<float>::size), vl / (8 * int(sizeof(float))));
  VERIFY_IS_EQUAL(int(internal::packet_traits<double>::size), vl / (8 * int(sizeof(double))));
}

// The opaque barriers keep the store/reload from being constant-folded away; on a narrower
// machine the tail of the buffer then keeps its sentinel.
template <typename Scalar, typename Packet>
void check_full_width_store() {
  constexpr int PacketSize = internal::unpacket_traits<Packet>::size;
  EIGEN_ALIGN_MAX Scalar buffer[PacketSize];
  Scalar value = Scalar(1);

  Scalar* p = buffer;
  asm volatile("" : "+r"(p) : : "memory");
  for (int i = 0; i < PacketSize; ++i) p[i] = Scalar(-12345);
  asm volatile("" : "+r"(p), "+r"(value) : : "memory");

  internal::pstore(p, internal::pset1<Packet>(value));
  asm volatile("" : "+r"(p) : : "memory");

  for (int i = 0; i < PacketSize; ++i) {
    VERIFY_IS_EQUAL(p[i], Scalar(1));
  }
}

EIGEN_DECLARE_TEST(sve_vector_length) {
  CALL_SUBTEST(check_runtime_vector_length());
  CALL_SUBTEST(check_packet_sizes());
  CALL_SUBTEST((check_full_width_store<numext::int32_t, internal::PacketXi>()));
  CALL_SUBTEST((check_full_width_store<float, internal::PacketXf>()));
  CALL_SUBTEST((check_full_width_store<double, internal::PacketXd>()));
}
