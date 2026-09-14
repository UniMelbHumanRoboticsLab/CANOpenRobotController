// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_NEON_SPECIALFUNCTIONS_H
#define EIGEN_NEON_SPECIALFUNCTIONS_H

namespace Eigen {
namespace internal {

#if EIGEN_ARCH_ARM
// For x = n * 2^-149, n < 2^23, erf(x) = (2/sqrt(pi))*x to float precision.
// round((2/sqrt(pi))*2^31) = 2423175810; the coefficient error contributes < 0.000319 ULP.
// Integer rounding avoids ARMv7 NEON's input/output flushing. The result bits also cover the first normal binade.
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Packet2i perf_subnormal_significands(const Packet2i& n) {
  return vreinterpret_s32_u32(vrshrn_n_u64(vmull_u32(vreinterpret_u32_s32(n), vdup_n_u32(2423175810u)), 31));
}

EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Packet4i perf_subnormal_significands(const Packet4i& n) {
  return vcombine_s32(perf_subnormal_significands(Packet2i(vget_low_s32(n))),
                      perf_subnormal_significands(Packet2i(vget_high_s32(n))));
}

template <typename Packet>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Packet perf_neon(const Packet& x) {
  using PacketI = typename unpacket_traits<Packet>::integer_packet;
  const Packet result = generic_fast_erf<float>::run(x);
  const PacketI bits = preinterpret<PacketI>(x);
  const PacketI magnitude = pand(bits, pset1<PacketI>(0x7fffffff));
  const PacketI sign = pxor(bits, magnitude);
  const PacketI subnormal = pcmp_lt(magnitude, pset1<PacketI>(0x00800000));
  const PacketI recovered = por(perf_subnormal_significands(magnitude), sign);
  return pselect(preinterpret<Packet>(subnormal), preinterpret<Packet>(recovered), result);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Packet2f perf<Packet2f>(const Packet2f& x) {
  return perf_neon(x);
}

template <>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Packet4f perf<Packet4f>(const Packet4f& x) {
  return perf_neon(x);
}
#endif

#if EIGEN_ARCH_ARM64 && EIGEN_HAS_ARM64_FP16

#define NEON_HALF_TO_FLOAT_FUNCTIONS(METHOD)                                              \
  template <>                                                                             \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Packet8hf METHOD<Packet8hf>(const Packet8hf& x) { \
    const Packet4f lo = METHOD<Packet4f>(vcvt_f32_f16(vget_low_f16(x)));                  \
    const Packet4f hi = METHOD<Packet4f>(vcvt_f32_f16(vget_high_f16(x)));                 \
    return vcombine_f16(vcvt_f16_f32(lo), vcvt_f16_f32(hi));                              \
  }                                                                                       \
                                                                                          \
  template <>                                                                             \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Packet4hf METHOD<Packet4hf>(const Packet4hf& x) { \
    return vcvt_f16_f32(METHOD<Packet4f>(vcvt_f32_f16(x)));                               \
  }

NEON_HALF_TO_FLOAT_FUNCTIONS(perf)
NEON_HALF_TO_FLOAT_FUNCTIONS(pndtri)

#undef NEON_HALF_TO_FLOAT_FUNCTIONS
#endif  // EIGEN_ARCH_ARM64 && EIGEN_HAS_ARM64_FP16

EIGEN_INSTANTIATE_SPECIAL_FUNCS_BF16(Packet4f, Packet4bf)

}  // namespace internal
}  // namespace Eigen

#endif  // EIGEN_NEON_SPECIALFUNCTIONS_H
