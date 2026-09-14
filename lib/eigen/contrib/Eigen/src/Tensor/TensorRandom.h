// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2016 Benoit Steiner <benoit.steiner.goog@gmail.com>
// Copyright (C) 2018 Mehdi Goli <eigen@codeplay.com> Codeplay Software Ltd.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_TENSOR_TENSOR_RANDOM_H
#define EIGEN_TENSOR_TENSOR_RANDOM_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {
namespace internal {

EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE uint64_t get_random_seed() {
#if defined(EIGEN_GPU_COMPILE_PHASE)
  // We don't support 3d kernels since we currently only use 1 and
  // 2d kernels.
  eigen_assert(threadIdx.z == 0);
  return blockIdx.x * blockDim.x + threadIdx.x + gridDim.x * blockDim.x * (blockIdx.y * blockDim.y + threadIdx.y);
#else
  // Rely on Eigen's random implementation.
  return random<uint64_t>();
#endif
}

EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE unsigned PCG_XSH_RS_generator(uint64_t* state, uint64_t stream) {
  // TODO: Unify with the implementation in the non blocking thread pool.
  uint64_t current = *state;
  // Update the internal state
  *state = current * 6364136223846793005ULL + (stream << 1 | 1);
  // Generate the random output (using the PCG-XSH-RS scheme)
  return static_cast<unsigned>((current ^ (current >> 22)) >> (22 + (current >> 61)));
}

EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE uint64_t PCG_XSH_RS_state(uint64_t seed) {
  seed = seed ? seed : get_random_seed();
  return seed * 6364136223846793005ULL + 0xda3e39cb94b95bdbULL;
}

// The state that the PCG stream of element `index` starts from: the base state stepped along the SplitMix64 Weyl
// sequence and mixed (splitmix64_mix, RandomImpl.h), so it is a pure function of (seed, index) and unrelated between
// neighbouring indices. The mixing matters because PCG_XSH_RS_generator derives its first output from the state it
// is handed: without it, the low bits of base + index * increment would pass straight into that output.
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE uint64_t PCG_XSH_RS_state_for_index(uint64_t base_state, uint64_t index) {
  return splitmix64_mix(base_state + index * 0x9e3779b97f4a7c15ULL);
}

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE T RandomToTypeUniform(uint64_t* state, uint64_t stream) {
  unsigned rnd = PCG_XSH_RS_generator(state, stream);
  return static_cast<T>(rnd);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE bool RandomToTypeUniform<bool>(uint64_t* state, uint64_t stream) {
  unsigned rnd = PCG_XSH_RS_generator(state, stream);
  return (rnd & 0x1) != 0;
}

template <>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Eigen::half RandomToTypeUniform<Eigen::half>(uint64_t* state, uint64_t stream) {
  // Generate 10 random bits for the mantissa, merge with exponent.
  unsigned rnd = PCG_XSH_RS_generator(state, stream);
  const uint16_t half_bits = static_cast<uint16_t>(rnd & 0x3ffu) | (static_cast<uint16_t>(15) << 10);
  Eigen::half result = Eigen::numext::bit_cast<Eigen::half>(half_bits);
  return result - Eigen::half(1.0f);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Eigen::bfloat16 RandomToTypeUniform<Eigen::bfloat16>(uint64_t* state,
                                                                                           uint64_t stream) {
  // Generate 7 random bits for the mantissa, merge with exponent.
  unsigned rnd = PCG_XSH_RS_generator(state, stream);
  const uint16_t half_bits = static_cast<uint16_t>(rnd & 0x7fu) | (static_cast<uint16_t>(127) << 7);
  Eigen::bfloat16 result = Eigen::numext::bit_cast<Eigen::bfloat16>(half_bits);
  return result - Eigen::bfloat16(1.0f);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE float RandomToTypeUniform<float>(uint64_t* state, uint64_t stream) {
  typedef union {
    uint32_t raw;
    float fp;
  } internal;
  internal result;
  // Generate 23 random bits for the mantissa.
  const unsigned rnd = PCG_XSH_RS_generator(state, stream);
  result.raw = rnd & 0x7fffffu;
  // Set the exponent.
  result.raw |= (static_cast<uint32_t>(127) << 23);
  return result.fp - 1.0f;
}

template <>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE double RandomToTypeUniform<double>(uint64_t* state, uint64_t stream) {
  typedef union {
    uint64_t raw;
    double dp;
  } internal;
  internal result;
  result.raw = 0;
  // Generate 52 random bits for the mantissa
  // First generate the upper 20 bits
  unsigned rnd1 = PCG_XSH_RS_generator(state, stream) & 0xfffffu;
  // Then generate the lower 32 bits.
  unsigned rnd2 = PCG_XSH_RS_generator(state, stream);
  result.raw = (static_cast<uint64_t>(rnd1) << 32) | rnd2;
  // Set the exponent
  result.raw |= (static_cast<uint64_t>(1023) << 52);
  // Return the final result
  return result.dp - 1.0;
}

template <>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<float> RandomToTypeUniform<std::complex<float> >(uint64_t* state,
                                                                                                    uint64_t stream) {
  const float real = RandomToTypeUniform<float>(state, stream);
  const float imag = RandomToTypeUniform<float>(state, stream);
  return std::complex<float>(real, imag);
}
template <>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<double> RandomToTypeUniform<std::complex<double> >(uint64_t* state,
                                                                                                      uint64_t stream) {
  const double real = RandomToTypeUniform<double>(state, stream);
  const double imag = RandomToTypeUniform<double>(state, stream);
  return std::complex<double>(real, imag);
}

// Element i is drawn from its own PCG stream, seeded from (seed, i) by PCG_XSH_RS_state_for_index. The functor
// therefore holds no evolving state: a fill is the same on every device, thread count and packet width, sharing
// the functor between threads is race-free, and the copy every GPU thread receives cannot replay the same numbers.
template <typename T>
class UniformRandomGenerator {
 public:
  static constexpr bool PacketAccess = true;

  // Uses the given "seed" if non-zero, otherwise uses a random seed.
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE UniformRandomGenerator(uint64_t seed = 0) : m_state(PCG_XSH_RS_state(seed)) {}

  template <typename Index>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE T operator()(Index i) const {
    const uint64_t index = static_cast<uint64_t>(i);
    uint64_t state = PCG_XSH_RS_state_for_index(m_state, index);
    return RandomToTypeUniform<T>(&state, index);
  }

  template <typename Packet, typename Index>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Packet packetOp(Index i) const {
    const int packetSize = internal::unpacket_traits<Packet>::size;
    EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) T values[packetSize];
    EIGEN_UNROLL_LOOP
    for (int j = 0; j < packetSize; ++j) {
      values[j] = (*this)(i + j);
    }
    return internal::pload<Packet>(values);
  }

 private:
  uint64_t m_state;
};

template <typename Scalar>
struct functor_traits<UniformRandomGenerator<Scalar> > {
  enum {
    // Rough estimate for floating point, multiplied by ceil(sizeof(T) / sizeof(float)).
    Cost = 12 * NumTraits<Scalar>::AddCost * ((sizeof(Scalar) + sizeof(float) - 1) / sizeof(float)),
    PacketAccess = UniformRandomGenerator<Scalar>::PacketAccess,
    // Element i is a pure function of (seed, i), so re-evaluating the expression or visiting the indices in
    // block order reproduces the same fill. This is what lets the nullary evaluator serve blocks.
    IsRepeatable = true
  };
};

template <typename T>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE T RandomToTypeNormal(uint64_t* state, uint64_t stream) {
  // Use the ratio of uniform method to generate numbers following a normal
  // distribution. See for example Numerical Recipes chapter 7.3.9 for the
  // details.
  T u, v, q;
  do {
    u = RandomToTypeUniform<T>(state, stream);
    v = T(1.7156) * (RandomToTypeUniform<T>(state, stream) - T(0.5));
    const T x = u - T(0.449871);
    const T y = numext::abs(v) + T(0.386595);
    q = x * x + y * (T(0.196) * y - T(0.25472) * x);
  } while (q > T(0.27597) && (q > T(0.27846) || v * v > T(-4) * numext::log(u) * u * u));

  return v / u;
}

// For 16-bit types, compute the deviate in float and round once. Running the
// rejection algorithm above directly in 16-bit arithmetic truncates the tails
// (|v/u| is limited by the coarse uniform grid) and emits NaN/Inf: the 16-bit
// uniform draw is exactly 0 with probability 2^-10 (half) / 2^-7 (bfloat16),
// so log(u) = -inf poisons the acceptance test and v/u returns +/-inf.
template <>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Eigen::half RandomToTypeNormal<Eigen::half>(uint64_t* state, uint64_t stream) {
  return Eigen::half(RandomToTypeNormal<float>(state, stream));
}
template <>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Eigen::bfloat16 RandomToTypeNormal<Eigen::bfloat16>(uint64_t* state,
                                                                                          uint64_t stream) {
  return Eigen::bfloat16(RandomToTypeNormal<float>(state, stream));
}

template <>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<float> RandomToTypeNormal<std::complex<float> >(uint64_t* state,
                                                                                                   uint64_t stream) {
  const float real = RandomToTypeNormal<float>(state, stream);
  const float imag = RandomToTypeNormal<float>(state, stream);
  return std::complex<float>(real, imag);
}
template <>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<double> RandomToTypeNormal<std::complex<double> >(uint64_t* state,
                                                                                                     uint64_t stream) {
  const double real = RandomToTypeNormal<double>(state, stream);
  const double imag = RandomToTypeNormal<double>(state, stream);
  return std::complex<double>(real, imag);
}

// Per-element streams as in UniformRandomGenerator; the rejection loop of RandomToTypeNormal draws as many values
// as it needs from element i's own stream.
template <typename T>
class NormalRandomGenerator {
 public:
  static constexpr bool PacketAccess = true;

  // Uses the given "seed" if non-zero, otherwise uses a random seed.
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE NormalRandomGenerator(uint64_t seed = 0) : m_state(PCG_XSH_RS_state(seed)) {}

  template <typename Index>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE T operator()(Index i) const {
    const uint64_t index = static_cast<uint64_t>(i);
    uint64_t state = PCG_XSH_RS_state_for_index(m_state, index);
    return RandomToTypeNormal<T>(&state, index);
  }

  template <typename Packet, typename Index>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Packet packetOp(Index i) const {
    const int packetSize = internal::unpacket_traits<Packet>::size;
    EIGEN_ALIGN_TO_BOUNDARY(unpacket_traits<Packet>::alignment) T values[packetSize];
    EIGEN_UNROLL_LOOP
    for (int j = 0; j < packetSize; ++j) {
      values[j] = (*this)(i + j);
    }
    return internal::pload<Packet>(values);
  }

 private:
  uint64_t m_state;
};

template <typename Scalar>
struct functor_traits<NormalRandomGenerator<Scalar> > {
  enum {
    // On average, we need to generate about 3 random numbers
    // 15 mul, 8 add, 1.5 logs
    Cost = 3 * functor_traits<UniformRandomGenerator<Scalar> >::Cost + 15 * NumTraits<Scalar>::AddCost +
           8 * NumTraits<Scalar>::AddCost + 3 * functor_traits<scalar_log_op<Scalar> >::Cost / 2,
    PacketAccess = NormalRandomGenerator<Scalar>::PacketAccess,
    // Pure in (seed, index) as UniformRandomGenerator is: the rejection loop draws from element i's own stream.
    IsRepeatable = true
  };
};

}  // end namespace internal
}  // end namespace Eigen

#endif  // EIGEN_TENSOR_TENSOR_RANDOM_H
