// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2025 Rasmus Munk Larsen
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_COMPLEX_CLANG_H
#define EIGEN_COMPLEX_CLANG_H

// IWYU pragma: private
#include "../../InternalHeaderCheck.h"

namespace Eigen {
namespace internal {

template <typename RealScalar, int N>
struct complex_packet_wrapper {
  using RealPacketT = detail::VectorType<RealScalar, 2 * N>;
  complex_packet_wrapper() = default;
  EIGEN_STRONG_INLINE explicit complex_packet_wrapper(const RealPacketT& a) : v(a) {}
  EIGEN_STRONG_INLINE constexpr std::complex<RealScalar> operator[](Index i) const {
    return std::complex<RealScalar>(v[2 * i], v[2 * i + 1]);
  }
  RealPacketT v;
};

// --- Primary complex packet aliases ---
constexpr int kComplexFloatSize = kFloatPacketSize / 2;    // 2, 4, or 8
constexpr int kComplexDoubleSize = kDoublePacketSize / 2;  // 1, 2, or 4
using PacketXcf = complex_packet_wrapper<float, kComplexFloatSize>;
using PacketXcd = complex_packet_wrapper<double, kComplexDoubleSize>;

struct generic_complex_packet_traits : default_packet_traits {
  enum {
    Vectorizable = 1,
    AlignedOnScalar = 1,
    HasAdd = 1,
    HasSub = 1,
    HasMul = 1,
    HasDiv = 1,
    HasNegate = 1,
    HasAbs = 0,
    HasAbs2 = 0,
    HasMin = 0,
    HasMax = 0,
    HasArg = 0,
    HasSetLinear = 0,
    HasConj = 1,
    // Math functions
    HasLog = 1,
    HasExp = 1,
    HasSqrt = 1,
  };
};

template <>
struct packet_traits<std::complex<float>> : generic_complex_packet_traits {
  using type = PacketXcf;
  using half = PacketXcf;
  enum {
    size = kComplexFloatSize,
  };
};

template <>
struct unpacket_traits<PacketXcf> : generic_unpacket_traits {
  using type = std::complex<float>;
  using half = PacketXcf;
  using as_real = PacketXf;
  enum {
    size = kComplexFloatSize,
  };
};

template <>
struct packet_traits<std::complex<double>> : generic_complex_packet_traits {
  using type = PacketXcd;
  using half = PacketXcd;
  enum {
    size = kComplexDoubleSize,
  };
};

template <>
struct unpacket_traits<PacketXcd> : generic_unpacket_traits {
  using type = std::complex<double>;
  using half = PacketXcd;
  using as_real = PacketXd;
  enum {
    size = kComplexDoubleSize,
  };
};

// ------------ Load and store ops ----------
#define EIGEN_CLANG_COMPLEX_LOAD_STORE(PACKET_TYPE)                                                       \
  template <>                                                                                             \
  EIGEN_STRONG_INLINE PACKET_TYPE ploadu<PACKET_TYPE>(const unpacket_traits<PACKET_TYPE>::type* from) {   \
    return PACKET_TYPE(ploadu<typename unpacket_traits<PACKET_TYPE>::as_real>(&numext::real_ref(*from))); \
  }                                                                                                       \
  template <>                                                                                             \
  EIGEN_STRONG_INLINE PACKET_TYPE pload<PACKET_TYPE>(const unpacket_traits<PACKET_TYPE>::type* from) {    \
    return PACKET_TYPE(pload<typename unpacket_traits<PACKET_TYPE>::as_real>(&numext::real_ref(*from)));  \
  }                                                                                                       \
  template <>                                                                                             \
  EIGEN_STRONG_INLINE void pstoreu<typename unpacket_traits<PACKET_TYPE>::type, PACKET_TYPE>(             \
      typename unpacket_traits<PACKET_TYPE>::type * to, const PACKET_TYPE& from) {                        \
    pstoreu(&numext::real_ref(*to), from.v);                                                              \
  }                                                                                                       \
  template <>                                                                                             \
  EIGEN_STRONG_INLINE void pstore<typename unpacket_traits<PACKET_TYPE>::type, PACKET_TYPE>(              \
      typename unpacket_traits<PACKET_TYPE>::type * to, const PACKET_TYPE& from) {                        \
    pstore(&numext::real_ref(*to), from.v);                                                               \
  }

EIGEN_CLANG_COMPLEX_LOAD_STORE(PacketXcf);
EIGEN_CLANG_COMPLEX_LOAD_STORE(PacketXcd);
#undef EIGEN_CLANG_COMPLEX_LOAD_STORE

namespace detail {

// Index sequence over the real components -- two per complex value -- of a
// complex packet. Index Is names component Is % 2 of complex value Is / 2.
template <typename ComplexPacket>
using complex_real_indices = vector_indices<typename ComplexPacket::RealPacketT>;

template <typename ComplexPacket, std::size_t... Is>
EIGEN_STRONG_INLINE ComplexPacket complex_pset1_impl(const typename unpacket_traits<ComplexPacket>::type& from,
                                                     std::index_sequence<Is...>) {
  using RealPacket = typename ComplexPacket::RealPacketT;
  using RealScalar = scalar_type_of_vector_t<RealPacket>;
  const RealScalar re = numext::real(from);
  const RealScalar im = numext::imag(from);
  return ComplexPacket(RealPacket{(Is % 2 == 0 ? re : im)...});
}

// Negates the imaginary parts by flipping their sign bits with one xor; the
// negate-and-shuffle alternative costs an extra blend.
template <typename ComplexPacket, std::size_t... Is>
EIGEN_STRONG_INLINE ComplexPacket complex_pconj_impl(const ComplexPacket& a, std::index_sequence<Is...>) {
  using RealScalar = scalar_type_of_vector_t<typename ComplexPacket::RealPacketT>;
  using IntScalar = std::conditional_t<sizeof(RealScalar) == 4, int32_t, int64_t>;
  using IntPacket = VectorType<IntScalar, sizeof...(Is)>;
  const IntPacket sign_mask = {(Is % 2 == 0 ? IntScalar(0) : (std::numeric_limits<IntScalar>::min)())...};
  return ComplexPacket(
      reinterpret_cast<typename ComplexPacket::RealPacketT>(reinterpret_cast<IntPacket>(a.v) ^ sign_mask));
}

// {re, im} -> {im, re}.
template <typename ComplexPacket, std::size_t... Is>
EIGEN_STRONG_INLINE ComplexPacket complex_pcplxflip_impl(const ComplexPacket& a, std::index_sequence<Is...>) {
  return ComplexPacket(__builtin_shufflevector(a.v, a.v, (2 * (Is / 2) + (1 - Is % 2))...));
}

// {re, im} -> {re, re}.
template <typename ComplexPacket, std::size_t... Is>
EIGEN_STRONG_INLINE ComplexPacket complex_pdupreal_impl(const ComplexPacket& a, std::index_sequence<Is...>) {
  return ComplexPacket(__builtin_shufflevector(a.v, a.v, (2 * (Is / 2))...));
}

// {re, im} -> {im, im}.
template <typename ComplexPacket, std::size_t... Is>
EIGEN_STRONG_INLINE ComplexPacket complex_pdupimag_impl(const ComplexPacket& a, std::index_sequence<Is...>) {
  return ComplexPacket(__builtin_shufflevector(a.v, a.v, (2 * (Is / 2) + 1)...));
}

// (a + ib)(c + id) = (ac - bd) + i(ad + bc), as an even/odd merge of the
// difference and sum vectors. Both products are spelled inside one expression
// each so fp-contract may fuse them, and the merge of a same-operand sub and
// add is the pattern LLVM folds into a single fmaddsub-style instruction on
// targets that have one. This matches the handwritten x86 backends, unlike
// the conjugate-and-flip formulation of the generic pmul_complex.
template <typename ComplexPacket, std::size_t... Is>
EIGEN_STRONG_INLINE ComplexPacket complex_pmul_impl(const ComplexPacket& x, const ComplexPacket& y,
                                                    std::index_sequence<Is...> is) {
  using RealPacket = typename ComplexPacket::RealPacketT;
  const RealPacket x_re = complex_pdupreal_impl(x, is).v;
  const RealPacket t = complex_pdupimag_impl(x, is).v * complex_pcplxflip_impl(y, is).v;
  const RealPacket sub = x_re * y.v - t;
  const RealPacket add = x_re * y.v + t;
  return ComplexPacket(__builtin_shufflevector(sub, add, (Is % 2 == 0 ? Is : sizeof...(Is) + Is)...));
}

// Loads each complex value Repeat times in a row: Repeat == 2 implements
// ploaddup and Repeat == 4 implements ploadquad.
template <std::size_t Repeat, typename ComplexPacket, std::size_t... Is>
EIGEN_STRONG_INLINE ComplexPacket complex_loadrepeat_impl(const typename unpacket_traits<ComplexPacket>::type* from,
                                                          std::index_sequence<Is...>) {
  using RealPacket = typename ComplexPacket::RealPacketT;
  return ComplexPacket(
      RealPacket{(Is % 2 == 0 ? numext::real(from[Is / (2 * Repeat)]) : numext::imag(from[Is / (2 * Repeat)]))...});
}

// Reverses the complex values, keeping each real/imaginary pair together.
template <typename ComplexPacket, std::size_t... Is>
EIGEN_STRONG_INLINE ComplexPacket complex_preverse_impl(const ComplexPacket& a, std::index_sequence<Is...>) {
  constexpr std::size_t kLastValue = sizeof...(Is) - 2;
  return ComplexPacket(__builtin_shufflevector(a.v, a.v, (kLastValue - 2 * (Is / 2) + Is % 2)...));
}

}  // namespace detail

// --- pset1 for complex ---
#define EIGEN_CLANG_COMPLEX_SET1(PACKET_TYPE)                                                          \
  template <>                                                                                          \
  EIGEN_STRONG_INLINE PACKET_TYPE pset1<PACKET_TYPE>(const unpacket_traits<PACKET_TYPE>::type& from) { \
    return detail::complex_pset1_impl<PACKET_TYPE>(from, detail::complex_real_indices<PACKET_TYPE>{}); \
  }

EIGEN_CLANG_COMPLEX_SET1(PacketXcf)
EIGEN_CLANG_COMPLEX_SET1(PacketXcd)
#undef EIGEN_CLANG_COMPLEX_SET1

// ----------- Unary ops ------------------
#define DELEGATE_UNARY_TO_REAL_OP(PACKET_TYPE, OP)                        \
  template <>                                                             \
  EIGEN_STRONG_INLINE PACKET_TYPE OP<PACKET_TYPE>(const PACKET_TYPE& a) { \
    return PACKET_TYPE(OP(a.v));                                          \
  }

#define EIGEN_CLANG_COMPLEX_UNARY_CWISE_OPS(PACKET_TYPE)                                             \
  DELEGATE_UNARY_TO_REAL_OP(PACKET_TYPE, pnegate)                                                    \
  DELEGATE_UNARY_TO_REAL_OP(PACKET_TYPE, pzero)                                                      \
  template <>                                                                                        \
  EIGEN_STRONG_INLINE unpacket_traits<PACKET_TYPE>::type pfirst<PACKET_TYPE>(const PACKET_TYPE& a) { \
    return a[0];                                                                                     \
  }                                                                                                  \
  EIGEN_INSTANTIATE_COMPLEX_MATH_FUNCS(PACKET_TYPE)

EIGEN_CLANG_COMPLEX_UNARY_CWISE_OPS(PacketXcf);
EIGEN_CLANG_COMPLEX_UNARY_CWISE_OPS(PacketXcd);

#undef DELEGATE_UNARY_TO_REAL_OP
#undef EIGEN_CLANG_COMPLEX_UNARY_CWISE_OPS

// --- Operations that rearrange the real and imaginary lanes ---
#define EIGEN_CLANG_COMPLEX_LANE_OPS(PACKET_TYPE)                                          \
  template <>                                                                              \
  EIGEN_STRONG_INLINE PACKET_TYPE pconj<PACKET_TYPE>(const PACKET_TYPE& a) {               \
    return detail::complex_pconj_impl(a, detail::complex_real_indices<PACKET_TYPE>{});     \
  }                                                                                        \
  template <>                                                                              \
  EIGEN_STRONG_INLINE PACKET_TYPE pcplxflip<PACKET_TYPE>(const PACKET_TYPE& a) {           \
    return detail::complex_pcplxflip_impl(a, detail::complex_real_indices<PACKET_TYPE>{}); \
  }                                                                                        \
  template <>                                                                              \
  EIGEN_STRONG_INLINE PACKET_TYPE pdupreal<PACKET_TYPE>(const PACKET_TYPE& a) {            \
    return detail::complex_pdupreal_impl(a, detail::complex_real_indices<PACKET_TYPE>{});  \
  }                                                                                        \
  template <>                                                                              \
  EIGEN_STRONG_INLINE PACKET_TYPE pdupimag<PACKET_TYPE>(const PACKET_TYPE& a) {            \
    return detail::complex_pdupimag_impl(a, detail::complex_real_indices<PACKET_TYPE>{});  \
  }

EIGEN_CLANG_COMPLEX_LANE_OPS(PacketXcf)
EIGEN_CLANG_COMPLEX_LANE_OPS(PacketXcd)
#undef EIGEN_CLANG_COMPLEX_LANE_OPS

// --- ploaddup and ploadquad ---
#define EIGEN_CLANG_COMPLEX_LOAD_REPEAT(PACKET_TYPE)                                                           \
  template <>                                                                                                  \
  EIGEN_STRONG_INLINE PACKET_TYPE ploaddup<PACKET_TYPE>(const unpacket_traits<PACKET_TYPE>::type* from) {      \
    return detail::complex_loadrepeat_impl<2, PACKET_TYPE>(from, detail::complex_real_indices<PACKET_TYPE>{}); \
  }                                                                                                            \
  template <>                                                                                                  \
  EIGEN_STRONG_INLINE PACKET_TYPE ploadquad<PACKET_TYPE>(const unpacket_traits<PACKET_TYPE>::type* from) {     \
    return detail::complex_loadrepeat_impl<4, PACKET_TYPE>(from, detail::complex_real_indices<PACKET_TYPE>{}); \
  }

EIGEN_CLANG_COMPLEX_LOAD_REPEAT(PacketXcf)
EIGEN_CLANG_COMPLEX_LOAD_REPEAT(PacketXcd)
#undef EIGEN_CLANG_COMPLEX_LOAD_REPEAT

// --- preverse ---
template <>
EIGEN_STRONG_INLINE PacketXcf preverse<PacketXcf>(const PacketXcf& a) {
  return detail::complex_preverse_impl(a, detail::complex_real_indices<PacketXcf>{});
}
template <>
EIGEN_STRONG_INLINE PacketXcd preverse<PacketXcd>(const PacketXcd& a) {
  return detail::complex_preverse_impl(a, detail::complex_real_indices<PacketXcd>{});
}

// ----------- Binary ops ------------------
#define DELEGATE_BINARY_TO_REAL_OP(PACKET_TYPE, OP)                                             \
  template <>                                                                                   \
  EIGEN_STRONG_INLINE PACKET_TYPE OP<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b) { \
    return PACKET_TYPE(OP(a.v, b.v));                                                           \
  }

#define EIGEN_CLANG_COMPLEX_BINARY_CWISE_OPS(PACKET_TYPE)                                            \
  DELEGATE_BINARY_TO_REAL_OP(PACKET_TYPE, padd)                                                      \
  DELEGATE_BINARY_TO_REAL_OP(PACKET_TYPE, psub)                                                      \
  DELEGATE_BINARY_TO_REAL_OP(PACKET_TYPE, pand)                                                      \
  DELEGATE_BINARY_TO_REAL_OP(PACKET_TYPE, por)                                                       \
  DELEGATE_BINARY_TO_REAL_OP(PACKET_TYPE, pxor)                                                      \
  DELEGATE_BINARY_TO_REAL_OP(PACKET_TYPE, pandnot)                                                   \
  template <>                                                                                        \
  EIGEN_STRONG_INLINE PACKET_TYPE pdiv<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b) {    \
    return pdiv_complex(a, b);                                                                       \
  }                                                                                                  \
  template <>                                                                                        \
  EIGEN_STRONG_INLINE PACKET_TYPE pcmp_eq<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b) { \
    const PACKET_TYPE t = PACKET_TYPE(pcmp_eq(a.v, b.v));                                            \
    return PACKET_TYPE(pand(pdupreal(t).v, pdupimag(t).v));                                          \
  }

EIGEN_CLANG_COMPLEX_BINARY_CWISE_OPS(PacketXcf);
EIGEN_CLANG_COMPLEX_BINARY_CWISE_OPS(PacketXcd);

// The sub-packets complex_predux_mul in Reductions.h recurses through call
// detail::complex_pmul_impl directly, so only the full packets need pmul.
#define EIGEN_CLANG_COMPLEX_PMUL(PACKET_TYPE)                                                     \
  template <>                                                                                     \
  EIGEN_STRONG_INLINE PACKET_TYPE pmul<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b) { \
    return detail::complex_pmul_impl(a, b, detail::complex_real_indices<PACKET_TYPE>{});          \
  }

EIGEN_CLANG_COMPLEX_PMUL(PacketXcf);
EIGEN_CLANG_COMPLEX_PMUL(PacketXcd);

#define EIGEN_CLANG_PACKET_SCATTER_GATHER(PACKET_TYPE)                                                               \
  template <>                                                                                                        \
  EIGEN_STRONG_INLINE void pscatter(unpacket_traits<PACKET_TYPE>::type* to, const PACKET_TYPE& from, Index stride) { \
    constexpr int size = unpacket_traits<PACKET_TYPE>::size;                                                         \
    for (int i = 0; i < size; ++i) {                                                                                 \
      to[i * stride] = from[i];                                                                                      \
    }                                                                                                                \
  }                                                                                                                  \
  template <>                                                                                                        \
  EIGEN_STRONG_INLINE PACKET_TYPE pgather<typename unpacket_traits<PACKET_TYPE>::type, PACKET_TYPE>(                 \
      const unpacket_traits<PACKET_TYPE>::type* from, Index stride) {                                                \
    constexpr int size = unpacket_traits<PACKET_TYPE>::size;                                                         \
    PACKET_TYPE result;                                                                                              \
    for (int i = 0; i < size; ++i) {                                                                                 \
      const unpacket_traits<PACKET_TYPE>::type from_i = from[i * stride];                                            \
      result.v[2 * i] = numext::real(from_i);                                                                        \
      result.v[2 * i + 1] = numext::imag(from_i);                                                                    \
    }                                                                                                                \
    return result;                                                                                                   \
  }

EIGEN_CLANG_PACKET_SCATTER_GATHER(PacketXcf);
EIGEN_CLANG_PACKET_SCATTER_GATHER(PacketXcd);
#undef EIGEN_CLANG_PACKET_SCATTER_GATHER

#undef DELEGATE_BINARY_TO_REAL_OP
#undef EIGEN_CLANG_COMPLEX_BINARY_CWISE_OPS
#undef EIGEN_CLANG_COMPLEX_PMUL

// ------------ ternary ops -------------
// The mask duplicates each complex value's bits across both real lanes, so
// selecting per real lane is equivalent to selecting whole complex values.
template <>
EIGEN_STRONG_INLINE PacketXcf pselect<PacketXcf>(const PacketXcf& mask, const PacketXcf& a, const PacketXcf& b) {
  return PacketXcf(pselect(mask.v, a.v, b.v));
}
template <>
EIGEN_STRONG_INLINE PacketXcd pselect<PacketXcd>(const PacketXcd& mask, const PacketXcd& a, const PacketXcd& b) {
  return PacketXcd(pselect(mask.v, a.v, b.v));
}

// --- zip_in_place for complex ---
namespace detail {

// Complex packets interleave whole complex values, so their real and imaginary
// components move together.
template <>
EIGEN_ALWAYS_INLINE void zip_in_place<PacketXcf>(PacketXcf& p1, PacketXcf& p2) {
  zip_in_place_impl<2>(p1.v, p2.v, complex_real_indices<PacketXcf>{});
}

#if EIGEN_GENERIC_VECTOR_SIZE_BYTES >= 32
// PacketXcd holds a single complex value at 16 bytes, so there is nothing to interleave.
template <>
EIGEN_ALWAYS_INLINE void zip_in_place<PacketXcd>(PacketXcd& p1, PacketXcd& p2) {
  zip_in_place_impl<2>(p1.v, p2.v, complex_real_indices<PacketXcd>{});
}
#endif

}  // namespace detail

// --- ptranspose for complex ---
// PacketXcf: valid block sizes depend on kComplexFloatSize.
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void ptranspose(PacketBlock<PacketXcf, 2>& kernel) {
  detail::ptranspose_impl(kernel);
}
#if EIGEN_GENERIC_VECTOR_SIZE_BYTES >= 32
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void ptranspose(PacketBlock<PacketXcf, 4>& kernel) {
  detail::ptranspose_impl(kernel);
}
#endif
#if EIGEN_GENERIC_VECTOR_SIZE_BYTES >= 64
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void ptranspose(PacketBlock<PacketXcf, 8>& kernel) {
  detail::ptranspose_impl(kernel);
}
#endif

// PacketXcd: valid block sizes depend on kComplexDoubleSize.
#if EIGEN_GENERIC_VECTOR_SIZE_BYTES >= 32
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void ptranspose(PacketBlock<PacketXcd, 2>& kernel) {
  detail::ptranspose_impl(kernel);
}
#endif
#if EIGEN_GENERIC_VECTOR_SIZE_BYTES >= 64
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void ptranspose(PacketBlock<PacketXcd, 4>& kernel) {
  detail::ptranspose_impl(kernel);
}
#endif

EIGEN_MAKE_CONJ_HELPER_CPLX_REAL(PacketXcf, PacketXf)
EIGEN_MAKE_CONJ_HELPER_CPLX_REAL(PacketXcd, PacketXd)

}  // end namespace internal
}  // end namespace Eigen

#endif  // EIGEN_COMPLEX_CLANG_H
