// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2025 Rasmus Munk Larsen
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_PACKET_MATH_CLANG_H
#define EIGEN_PACKET_MATH_CLANG_H

// IWYU pragma: private
#include "../../InternalHeaderCheck.h"

namespace Eigen {
namespace internal {

namespace detail {
// namespace detail contains implementation details specific to this
// file, while namespace internal contains internal APIs used elsewhere
// in Eigen.
template <typename ScalarT, int n>
using VectorType = ScalarT __attribute__((ext_vector_type(n), aligned(n * sizeof(ScalarT))));
}  // namespace detail

// --- Naming Convention ---
// This backend uses size-independent type aliases so the same code works
// for EIGEN_GENERIC_VECTOR_SIZE_BYTES in {16, 32, 64}:
//
//   PacketXf  - float vector   (4, 8, or 16 elements)
//   PacketXd  - double vector  (2, 4, or 8 elements)
//   PacketXi  - int32_t vector (4, 8, or 16 elements)
//   PacketXl  - int64_t vector (2, 4, or 8 elements)
//   PacketXcf - complex<float> vector  (2, 4, or 8 elements)  [in Complex.h]
//   PacketXcd - complex<double> vector (1, 2, or 4 elements)  [in Complex.h]
//
// The "X" suffix indicates the element count is determined by the macro
// EIGEN_GENERIC_VECTOR_SIZE_BYTES at compile time. Operations that require
// compile-time constant indices (e.g. __builtin_shufflevector) obtain them by
// expanding detail::vector_indices<Packet>, so they need no per-size code.

static_assert(EIGEN_GENERIC_VECTOR_SIZE_BYTES == 16 || EIGEN_GENERIC_VECTOR_SIZE_BYTES == 32 ||
                  EIGEN_GENERIC_VECTOR_SIZE_BYTES == 64,
              "EIGEN_GENERIC_VECTOR_SIZE_BYTES must be 16, 32, or 64");

constexpr int kFloatPacketSize = EIGEN_GENERIC_VECTOR_SIZE_BYTES / sizeof(float);
constexpr int kDoublePacketSize = EIGEN_GENERIC_VECTOR_SIZE_BYTES / sizeof(double);
using PacketXf = detail::VectorType<float, kFloatPacketSize>;
using PacketXd = detail::VectorType<double, kDoublePacketSize>;
using PacketXi = detail::VectorType<int32_t, kFloatPacketSize>;
using PacketXl = detail::VectorType<int64_t, kDoublePacketSize>;

// --- packet_traits specializations ---
struct generic_float_packet_traits : default_packet_traits {
  enum {
    Vectorizable = 1,
    AlignedOnScalar = 1,
    HasAdd = 1,
    HasSub = 1,
    HasMul = 1,
    HasDiv = 1,
    HasNegate = 1,
    HasAbs = 1,
    HasRound = 1,
    HasMin = 1,
    HasMax = 1,
    HasCmp = 1,
    HasSet1 = 1,
    HasCast = 1,
    HasBitwise = 1,
    HasRedux = 1,
    HasSign = 1,
    HasArg = 0,
    HasConj = 1,
    // Math functions
    HasReciprocal = 1,
    HasSin = 1,
    HasCos = 1,
    HasTan = 1,
    HasACos = 1,
    HasASin = 1,
    HasATan = 1,
    HasATanh = 1,
    HasLog = 1,
    HasLog1p = 1,
    HasExpm1 = 1,
    HasExp = 1,
    HasPow = 1,
    HasNdtri = 1,
    HasBessel = 1,
    HasSqrt = 1,
    HasRsqrt = 1,
    HasCbrt = 1,
    HasTanh = 1,
    HasErf = 1,
    HasErfc = 1
  };
};

template <>
struct packet_traits<float> : generic_float_packet_traits {
  using type = PacketXf;
  using half = PacketXf;
  enum {
    size = kFloatPacketSize,
  };
};

template <>
struct packet_traits<double> : generic_float_packet_traits {
  using type = PacketXd;
  using half = PacketXd;
  // Generic double-precision acos/asin are not yet implemented in
  // GenericPacketMathFunctions.h (only float versions exist).
  enum { size = kDoublePacketSize, HasACos = 0, HasASin = 0 };
};

struct generic_integer_packet_traits : default_packet_traits {
  enum {
    Vectorizable = 1,
    AlignedOnScalar = 1,
    HasAdd = 1,
    HasSub = 1,
    HasMul = 1,
    HasDiv = 1,
    HasNegate = 1,
    HasAbs = 1,
    HasMin = 1,
    HasMax = 1,
    HasCmp = 1,
    HasSet1 = 1,
    HasCast = 1,
    HasBitwise = 1,
    HasRedux = 1,
    // Set remaining to 0
    HasRound = 1,
    HasSqrt = 0,
    HasRsqrt = 0,
    HasReciprocal = 0,
    HasArg = 0,
    HasConj = 1,
    HasExp = 0,
    HasLog = 0,
    HasSin = 0,
    HasCos = 0,
  };
};

template <>
struct packet_traits<int32_t> : generic_integer_packet_traits {
  using type = PacketXi;
  using half = PacketXi;
  enum {
    size = kFloatPacketSize,
  };
};

template <>
struct packet_traits<int64_t> : generic_integer_packet_traits {
  using type = PacketXl;
  using half = PacketXl;
  enum {
    size = kDoublePacketSize,
  };
};

// --- unpacket_traits specializations ---
struct generic_unpacket_traits : default_unpacket_traits {
  enum {
    alignment = EIGEN_GENERIC_VECTOR_SIZE_BYTES,
    vectorizable = true,
  };
};

template <>
struct unpacket_traits<PacketXf> : generic_unpacket_traits {
  using type = float;
  using half = PacketXf;
  using integer_packet = PacketXi;
  enum {
    size = kFloatPacketSize,
  };
};
template <>
struct unpacket_traits<PacketXd> : generic_unpacket_traits {
  using type = double;
  using half = PacketXd;
  using integer_packet = PacketXl;
  enum {
    size = kDoublePacketSize,
  };
};
template <>
struct unpacket_traits<PacketXi> : generic_unpacket_traits {
  using type = int32_t;
  using half = PacketXi;
  enum {
    size = kFloatPacketSize,
  };
};
template <>
struct unpacket_traits<PacketXl> : generic_unpacket_traits {
  using type = int64_t;
  using half = PacketXl;
  enum {
    size = kDoublePacketSize,
  };
};

namespace detail {
// --- vector type helpers ---
template <typename VectorT>
struct ScalarTypeOfVector {
  using type = std::remove_all_extents_t<std::remove_reference_t<decltype(VectorT()[0])>>;
};

template <typename VectorT>
using scalar_type_of_vector_t = typename ScalarTypeOfVector<VectorT>::type;

template <typename VectorType>
struct UnsignedVectorHelper {
  static VectorType v;
  static constexpr int n = __builtin_vectorelements(v);
  using UnsignedScalar = std::make_unsigned_t<scalar_type_of_vector_t<VectorType>>;
  using type = UnsignedScalar __attribute__((ext_vector_type(n), aligned(n * sizeof(UnsignedScalar))));
};

template <typename VectorT>
using unsigned_vector_t = typename UnsignedVectorHelper<VectorT>::type;

template <typename VectorT>
constexpr int vector_elements() {
  return static_cast<int>(sizeof(VectorT) / sizeof(scalar_type_of_vector_t<VectorT>));
}

// Signed integer vector with the same lane count and width, for sign-bit
// tests and bitwise manipulation of floating-point packets.
template <typename VectorT>
struct SignedVectorHelper {
  using SignedScalar = std::conditional_t<sizeof(scalar_type_of_vector_t<VectorT>) == 4, int32_t, int64_t>;
  using type = VectorType<SignedScalar, vector_elements<VectorT>()>;
};

template <typename VectorT>
using signed_vector_t = typename SignedVectorHelper<VectorT>::type;

template <typename VectorT>
using half_vector_t = VectorType<scalar_type_of_vector_t<VectorT>, vector_elements<VectorT>() / 2>;

template <typename VectorT>
using quarter_vector_t = VectorType<scalar_type_of_vector_t<VectorT>, vector_elements<VectorT>() / 4>;

template <typename VectorT>
using scalar_pair_t = std::pair<scalar_type_of_vector_t<VectorT>, scalar_type_of_vector_t<VectorT>>;

// Index sequence covering every element of VectorT. Expanding it inside a
// __builtin_shufflevector index list or a braced initializer is what keeps the
// operations below independent of EIGEN_GENERIC_VECTOR_SIZE_BYTES.
template <typename VectorT>
using vector_indices = std::make_index_sequence<vector_elements<VectorT>()>;

// load and store helpers.
template <typename VectorT>
EIGEN_STRONG_INLINE VectorT load_vector_unaligned(const scalar_type_of_vector_t<VectorT>* from) {
  VectorT to;
  __builtin_memcpy(&to, from, sizeof(VectorT));
  return to;
}

template <typename VectorT>
EIGEN_STRONG_INLINE VectorT load_vector_aligned(const scalar_type_of_vector_t<VectorT>* from) {
  eigen_assert((std::uintptr_t(from) % alignof(VectorT) == 0) && "load_vector_aligned");
  return *reinterpret_cast<const VectorT*>(assume_aligned<alignof(VectorT)>(from));
}

template <typename VectorT>
EIGEN_STRONG_INLINE void store_vector_unaligned(scalar_type_of_vector_t<VectorT>* to, const VectorT& from) {
  __builtin_memcpy(to, &from, sizeof(VectorT));
}

template <typename VectorT>
EIGEN_STRONG_INLINE void store_vector_aligned(scalar_type_of_vector_t<VectorT>* to, const VectorT& from) {
  eigen_assert((std::uintptr_t(to) % alignof(VectorT) == 0) && "store_vector_aligned");
  *reinterpret_cast<VectorT*>(assume_aligned<alignof(VectorT)>(to)) = from;
}

}  // namespace detail

// --- Intrinsic-like specializations ---

// --- Load/Store operations ---
#define EIGEN_CLANG_PACKET_LOAD_STORE_PACKET(PACKET_TYPE)                                                         \
  template <>                                                                                                     \
  EIGEN_STRONG_INLINE PACKET_TYPE ploadu<PACKET_TYPE>(const detail::scalar_type_of_vector_t<PACKET_TYPE>* from) { \
    return detail::load_vector_unaligned<PACKET_TYPE>(from);                                                      \
  }                                                                                                               \
  template <>                                                                                                     \
  EIGEN_STRONG_INLINE PACKET_TYPE pload<PACKET_TYPE>(const detail::scalar_type_of_vector_t<PACKET_TYPE>* from) {  \
    return detail::load_vector_aligned<PACKET_TYPE>(from);                                                        \
  }                                                                                                               \
  template <>                                                                                                     \
  EIGEN_STRONG_INLINE void pstoreu<detail::scalar_type_of_vector_t<PACKET_TYPE>, PACKET_TYPE>(                    \
      detail::scalar_type_of_vector_t<PACKET_TYPE> * to, const PACKET_TYPE& from) {                               \
    detail::store_vector_unaligned<PACKET_TYPE>(to, from);                                                        \
  }                                                                                                               \
  template <>                                                                                                     \
  EIGEN_STRONG_INLINE void pstore<detail::scalar_type_of_vector_t<PACKET_TYPE>, PACKET_TYPE>(                     \
      detail::scalar_type_of_vector_t<PACKET_TYPE> * to, const PACKET_TYPE& from) {                               \
    detail::store_vector_aligned<PACKET_TYPE>(to, from);                                                          \
  }

EIGEN_CLANG_PACKET_LOAD_STORE_PACKET(PacketXf)
EIGEN_CLANG_PACKET_LOAD_STORE_PACKET(PacketXd)
EIGEN_CLANG_PACKET_LOAD_STORE_PACKET(PacketXi)
EIGEN_CLANG_PACKET_LOAD_STORE_PACKET(PacketXl)
#undef EIGEN_CLANG_PACKET_LOAD_STORE_PACKET

// --- Broadcast operation ---
template <>
EIGEN_STRONG_INLINE PacketXf pset1frombits<PacketXf>(uint32_t from) {
  return PacketXf(numext::bit_cast<float>(from));
}

template <>
EIGEN_STRONG_INLINE PacketXd pset1frombits<PacketXd>(uint64_t from) {
  return PacketXd(numext::bit_cast<double>(from));
}

#define EIGEN_CLANG_PACKET_SET1(PACKET_TYPE)                                                            \
  template <>                                                                                           \
  EIGEN_STRONG_INLINE PACKET_TYPE pset1<PACKET_TYPE>(const unpacket_traits<PACKET_TYPE>::type& from) {  \
    return PACKET_TYPE(from);                                                                           \
  }                                                                                                     \
  template <>                                                                                           \
  EIGEN_STRONG_INLINE unpacket_traits<PACKET_TYPE>::type pfirst<PACKET_TYPE>(const PACKET_TYPE& from) { \
    return from[0];                                                                                     \
  }

EIGEN_CLANG_PACKET_SET1(PacketXf)
EIGEN_CLANG_PACKET_SET1(PacketXd)
EIGEN_CLANG_PACKET_SET1(PacketXi)
EIGEN_CLANG_PACKET_SET1(PacketXl)
#undef EIGEN_CLANG_PACKET_SET1

// --- Arithmetic operations ---
#define EIGEN_CLANG_PACKET_ARITHMETIC(PACKET_TYPE)                             \
  template <>                                                                  \
  EIGEN_STRONG_INLINE PACKET_TYPE pisnan<PACKET_TYPE>(const PACKET_TYPE& a) {  \
    return reinterpret_cast<PACKET_TYPE>(a != a);                              \
  }                                                                            \
  template <>                                                                  \
  EIGEN_STRONG_INLINE PACKET_TYPE pnegate<PACKET_TYPE>(const PACKET_TYPE& a) { \
    return -a;                                                                 \
  }

EIGEN_CLANG_PACKET_ARITHMETIC(PacketXf)
EIGEN_CLANG_PACKET_ARITHMETIC(PacketXd)
EIGEN_CLANG_PACKET_ARITHMETIC(PacketXi)
EIGEN_CLANG_PACKET_ARITHMETIC(PacketXl)
#undef EIGEN_CLANG_PACKET_ARITHMETIC

// --- Bitwise operations (via casting) ---

namespace detail {

// Reinterpret-cast helpers, equivalent to preinterpret<> but defined here
// because PacketMath.h is included before TypeCasting.h.
EIGEN_STRONG_INLINE PacketXi preinterpret_float_to_int(const PacketXf& a) { return reinterpret_cast<PacketXi>(a); }
EIGEN_STRONG_INLINE PacketXf preinterpret_int_to_float(const PacketXi& a) { return reinterpret_cast<PacketXf>(a); }
EIGEN_STRONG_INLINE PacketXl preinterpret_double_to_long(const PacketXd& a) { return reinterpret_cast<PacketXl>(a); }
EIGEN_STRONG_INLINE PacketXd preinterpret_long_to_double(const PacketXl& a) { return reinterpret_cast<PacketXd>(a); }

}  // namespace detail

// Bitwise ops for integer packets
#define EIGEN_CLANG_PACKET_BITWISE_INT(PACKET_TYPE)                                                  \
  template <>                                                                                        \
  constexpr EIGEN_STRONG_INLINE PACKET_TYPE pzero<PACKET_TYPE>(const PACKET_TYPE& /*unused*/) {      \
    return PACKET_TYPE(0);                                                                           \
  }                                                                                                  \
  template <>                                                                                        \
  constexpr EIGEN_STRONG_INLINE PACKET_TYPE ptrue<PACKET_TYPE>(const PACKET_TYPE& /*unused*/) {      \
    return numext::bit_cast<PACKET_TYPE>(PACKET_TYPE(0) == PACKET_TYPE(0));                          \
  }                                                                                                  \
  template <>                                                                                        \
  EIGEN_STRONG_INLINE PACKET_TYPE pand<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b) {    \
    return a & b;                                                                                    \
  }                                                                                                  \
  template <>                                                                                        \
  EIGEN_STRONG_INLINE PACKET_TYPE por<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b) {     \
    return a | b;                                                                                    \
  }                                                                                                  \
  template <>                                                                                        \
  EIGEN_STRONG_INLINE PACKET_TYPE pxor<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b) {    \
    return a ^ b;                                                                                    \
  }                                                                                                  \
  template <>                                                                                        \
  EIGEN_STRONG_INLINE PACKET_TYPE pandnot<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b) { \
    return a & ~b;                                                                                   \
  }                                                                                                  \
  template <int N>                                                                                   \
  EIGEN_STRONG_INLINE PACKET_TYPE parithmetic_shift_right(const PACKET_TYPE& a) {                    \
    return a >> N;                                                                                   \
  }                                                                                                  \
  template <int N>                                                                                   \
  EIGEN_STRONG_INLINE PACKET_TYPE plogical_shift_right(const PACKET_TYPE& a) {                       \
    using UnsignedT = detail::unsigned_vector_t<PACKET_TYPE>;                                        \
    return reinterpret_cast<PACKET_TYPE>(reinterpret_cast<UnsignedT>(a) >> N);                       \
  }                                                                                                  \
  template <int N>                                                                                   \
  EIGEN_STRONG_INLINE PACKET_TYPE plogical_shift_left(const PACKET_TYPE& a) {                        \
    return a << N;                                                                                   \
  }

EIGEN_CLANG_PACKET_BITWISE_INT(PacketXi)
EIGEN_CLANG_PACKET_BITWISE_INT(PacketXl)
#undef EIGEN_CLANG_PACKET_BITWISE_INT

// Bitwise ops for floating point packets
#define EIGEN_CLANG_PACKET_BITWISE_FLOAT(PACKET_TYPE, CAST_TO_INT, CAST_FROM_INT)                    \
  template <>                                                                                        \
  constexpr EIGEN_STRONG_INLINE PACKET_TYPE pzero<PACKET_TYPE>(const PACKET_TYPE& /*unused*/) {      \
    using Scalar = detail::scalar_type_of_vector_t<PACKET_TYPE>;                                     \
    return PACKET_TYPE(Scalar(0));                                                                   \
  }                                                                                                  \
  template <>                                                                                        \
  EIGEN_STRONG_INLINE PACKET_TYPE ptrue<PACKET_TYPE>(const PACKET_TYPE& /* unused */) {              \
    using Scalar = detail::scalar_type_of_vector_t<PACKET_TYPE>;                                     \
    PACKET_TYPE r = numext::bit_cast<PACKET_TYPE>(PACKET_TYPE(Scalar(0)) == PACKET_TYPE(Scalar(0))); \
    EIGEN_FAST_MATH_CONSTANT_BARRIER(r);                                                             \
    return r;                                                                                        \
  }                                                                                                  \
  template <>                                                                                        \
  EIGEN_STRONG_INLINE PACKET_TYPE pand<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b) {    \
    return CAST_FROM_INT(CAST_TO_INT(a) & CAST_TO_INT(b));                                           \
  }                                                                                                  \
  template <>                                                                                        \
  EIGEN_STRONG_INLINE PACKET_TYPE por<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b) {     \
    return CAST_FROM_INT(CAST_TO_INT(a) | CAST_TO_INT(b));                                           \
  }                                                                                                  \
  template <>                                                                                        \
  EIGEN_STRONG_INLINE PACKET_TYPE pxor<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b) {    \
    return CAST_FROM_INT(CAST_TO_INT(a) ^ CAST_TO_INT(b));                                           \
  }                                                                                                  \
  template <>                                                                                        \
  EIGEN_STRONG_INLINE PACKET_TYPE pandnot<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b) { \
    return CAST_FROM_INT(CAST_TO_INT(a) & ~CAST_TO_INT(b));                                          \
  }

EIGEN_CLANG_PACKET_BITWISE_FLOAT(PacketXf, detail::preinterpret_float_to_int, detail::preinterpret_int_to_float)
EIGEN_CLANG_PACKET_BITWISE_FLOAT(PacketXd, detail::preinterpret_double_to_long, detail::preinterpret_long_to_double)
#undef EIGEN_CLANG_PACKET_BITWISE_FLOAT

// --- Comparison operations ---
// Clang vector extensions perform comparisons in the original type (float/double),
// returning an int vector with all-ones (-1) for true and all-zeros for false.
// The bit_cast reinterprets those int bitmasks as float packets, which is the
// format expected by pselect and other Eigen packet operations.
#define EIGEN_CLANG_PACKET_CMP(PACKET_TYPE, INT_PACKET_TYPE)                                                \
  template <>                                                                                               \
  EIGEN_STRONG_INLINE PACKET_TYPE pcmp_eq<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b) {        \
    return numext::bit_cast<PACKET_TYPE>(INT_PACKET_TYPE(a == b));                                          \
  }                                                                                                         \
  template <>                                                                                               \
  EIGEN_STRONG_INLINE PACKET_TYPE pcmp_lt<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b) {        \
    return numext::bit_cast<PACKET_TYPE>(INT_PACKET_TYPE(a < b));                                           \
  }                                                                                                         \
  template <>                                                                                               \
  EIGEN_STRONG_INLINE PACKET_TYPE pcmp_le<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b) {        \
    return numext::bit_cast<PACKET_TYPE>(INT_PACKET_TYPE(a <= b));                                          \
  }                                                                                                         \
  template <>                                                                                               \
  EIGEN_STRONG_INLINE PACKET_TYPE pcmp_lt_or_nan<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b) { \
    return numext::bit_cast<PACKET_TYPE>(INT_PACKET_TYPE(!(a >= b)));                                       \
  }

EIGEN_CLANG_PACKET_CMP(PacketXf, PacketXi)
EIGEN_CLANG_PACKET_CMP(PacketXd, PacketXl)
#undef EIGEN_CLANG_PACKET_CMP

// --- Min/Max/select operations ---
namespace detail {
// Functors usable at any vector width; the min/max reduction trees in
// Reductions.h reuse them on progressively narrower vectors. The
// compare-select forms compile to a single min/max instruction on targets
// whose min/max returns the second operand when the inputs are unordered
// (e.g. x86), and they spell out the NaN propagation of std::min/std::max:
// the first argument is returned if either input is NaN.
struct pmin_op {
  template <typename VectorT>
  EIGEN_STRONG_INLINE VectorT operator()(const VectorT& a, const VectorT& b) const {
    return b < a ? b : a;
  }
};
struct pmax_op {
  template <typename VectorT>
  EIGEN_STRONG_INLINE VectorT operator()(const VectorT& a, const VectorT& b) const {
    return b > a ? b : a;
  }
};
// IEEE 754-2008 minNum/maxNum semantics: return the other operand if one input
// is NaN. Floating-point support in __builtin_elementwise_{min,max} is
// deprecated because the name does not say which of the several IEEE min/max
// flavors is meant; __builtin_elementwise_{minnum,maxnum} spell out the same
// semantics the deprecated builtins provided for floats, and additionally pin
// down +0.0 > -0.0. The elementwise_{min,max} fallback (always available
// under this backend's clang >= 16 gate) has the same NaN semantics but
// leaves the zero-sign tie unspecified.
struct pmin_num_op {
  template <typename VectorT>
  EIGEN_STRONG_INLINE VectorT operator()(const VectorT& a, const VectorT& b) const {
#if EIGEN_HAS_BUILTIN(__builtin_elementwise_minnum)
    return __builtin_elementwise_minnum(a, b);
#else
    return __builtin_elementwise_min(a, b);
#endif
  }
};
struct pmax_num_op {
  template <typename VectorT>
  EIGEN_STRONG_INLINE VectorT operator()(const VectorT& a, const VectorT& b) const {
#if EIGEN_HAS_BUILTIN(__builtin_elementwise_maxnum)
    return __builtin_elementwise_maxnum(a, b);
#else
    return __builtin_elementwise_max(a, b);
#endif
  }
};
// Return NaN if either input is NaN, otherwise the min/max. When a is NaN the
// plain compare-select form already returns a, so only b needs an explicit
// test.
struct pmin_nan_op {
  template <typename VectorT>
  EIGEN_STRONG_INLINE VectorT operator()(const VectorT& a, const VectorT& b) const {
    return b != b ? b : pmin_op()(a, b);
  }
};
struct pmax_nan_op {
  template <typename VectorT>
  EIGEN_STRONG_INLINE VectorT operator()(const VectorT& a, const VectorT& b) const {
    return b != b ? b : pmax_op()(a, b);
  }
};
}  // namespace detail

// pmin/pmax/pselect are pure compare-select code and apply to all packet types.
#define EIGEN_CLANG_PACKET_MINMAX_SELECT(PACKET_TYPE)                                                 \
  template <>                                                                                         \
  EIGEN_STRONG_INLINE PACKET_TYPE pmin<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b) {     \
    return detail::pmin_op()(a, b);                                                                   \
  }                                                                                                   \
  template <>                                                                                         \
  EIGEN_STRONG_INLINE PACKET_TYPE pmax<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b) {     \
    return detail::pmax_op()(a, b);                                                                   \
  }                                                                                                   \
  template <>                                                                                         \
  EIGEN_STRONG_INLINE PACKET_TYPE pselect<PACKET_TYPE>(const PACKET_TYPE& mask, const PACKET_TYPE& a, \
                                                       const PACKET_TYPE& b) {                        \
    /* The mask is all-ones or all-zeros per lane, so testing the sign of the */                      \
    /* signed integer view suffices and maps to a single blend instruction.   */                      \
    /* Unlike a floating-point `mask != 0` test it also survives -ffast-math, */                      \
    /* which may assume the all-ones NaN bit pattern cannot occur in a float. */                      \
    return reinterpret_cast<detail::signed_vector_t<PACKET_TYPE>>(mask) < 0 ? a : b;                  \
  }

EIGEN_CLANG_PACKET_MINMAX_SELECT(PacketXf)
EIGEN_CLANG_PACKET_MINMAX_SELECT(PacketXd)
EIGEN_CLANG_PACKET_MINMAX_SELECT(PacketXi)
EIGEN_CLANG_PACKET_MINMAX_SELECT(PacketXl)
#undef EIGEN_CLANG_PACKET_MINMAX_SELECT

// NaN-propagation variants for the floating-point packets.
#define EIGEN_CLANG_PACKET_MINMAX_FLOAT(PACKET_TYPE)                                                                \
  template <>                                                                                                       \
  EIGEN_STRONG_INLINE PACKET_TYPE pmin<PropagateNumbers, PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b) { \
    return detail::pmin_num_op()(a, b);                                                                             \
  }                                                                                                                 \
  template <>                                                                                                       \
  EIGEN_STRONG_INLINE PACKET_TYPE pmax<PropagateNumbers, PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b) { \
    return detail::pmax_num_op()(a, b);                                                                             \
  }                                                                                                                 \
  template <>                                                                                                       \
  EIGEN_STRONG_INLINE PACKET_TYPE pmin<PropagateNaN, PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b) {     \
    return detail::pmin_nan_op()(a, b);                                                                             \
  }                                                                                                                 \
  template <>                                                                                                       \
  EIGEN_STRONG_INLINE PACKET_TYPE pmax<PropagateNaN, PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b) {     \
    return detail::pmax_nan_op()(a, b);                                                                             \
  }

EIGEN_CLANG_PACKET_MINMAX_FLOAT(PacketXf)
EIGEN_CLANG_PACKET_MINMAX_FLOAT(PacketXd)
#undef EIGEN_CLANG_PACKET_MINMAX_FLOAT

#if EIGEN_HAS_BUILTIN(__builtin_elementwise_abs)
#define EIGEN_CLANG_PACKET_ABS(PACKET_TYPE)                                 \
  template <>                                                               \
  EIGEN_STRONG_INLINE PACKET_TYPE pabs<PACKET_TYPE>(const PACKET_TYPE& a) { \
    return __builtin_elementwise_abs(a);                                    \
  }

EIGEN_CLANG_PACKET_ABS(PacketXf)
EIGEN_CLANG_PACKET_ABS(PacketXd)
EIGEN_CLANG_PACKET_ABS(PacketXi)
EIGEN_CLANG_PACKET_ABS(PacketXl)
#undef EIGEN_CLANG_PACKET_ABS
#endif

// psignbit: a signed compare of the integer view is a single instruction,
// unlike the generic floating-point fallback.
template <>
EIGEN_STRONG_INLINE PacketXf psignbit(const PacketXf& a) {
  return reinterpret_cast<PacketXf>(reinterpret_cast<PacketXi>(a) < 0);
}
template <>
EIGEN_STRONG_INLINE PacketXd psignbit(const PacketXd& a) {
  return reinterpret_cast<PacketXd>(reinterpret_cast<PacketXl>(a) < 0);
}

// --- Math functions (float/double only) ---

#if EIGEN_HAS_BUILTIN(__builtin_elementwise_floor) && EIGEN_HAS_BUILTIN(__builtin_elementwise_ceil) &&      \
    EIGEN_HAS_BUILTIN(__builtin_elementwise_round) && EIGEN_HAS_BUILTIN(__builtin_elementwise_roundeven) && \
    EIGEN_HAS_BUILTIN(__builtin_elementwise_trunc) && EIGEN_HAS_BUILTIN(__builtin_elementwise_sqrt)
#define EIGEN_CLANG_PACKET_MATH_FLOAT(PACKET_TYPE)                            \
  template <>                                                                 \
  EIGEN_STRONG_INLINE PACKET_TYPE pfloor<PACKET_TYPE>(const PACKET_TYPE& a) { \
    return __builtin_elementwise_floor(a);                                    \
  }                                                                           \
  template <>                                                                 \
  EIGEN_STRONG_INLINE PACKET_TYPE pceil<PACKET_TYPE>(const PACKET_TYPE& a) {  \
    return __builtin_elementwise_ceil(a);                                     \
  }                                                                           \
  template <>                                                                 \
  EIGEN_STRONG_INLINE PACKET_TYPE pround<PACKET_TYPE>(const PACKET_TYPE& a) { \
    return __builtin_elementwise_round(a);                                    \
  }                                                                           \
  template <>                                                                 \
  EIGEN_STRONG_INLINE PACKET_TYPE print<PACKET_TYPE>(const PACKET_TYPE& a) {  \
    return __builtin_elementwise_roundeven(a);                                \
  }                                                                           \
  template <>                                                                 \
  EIGEN_STRONG_INLINE PACKET_TYPE ptrunc<PACKET_TYPE>(const PACKET_TYPE& a) { \
    return __builtin_elementwise_trunc(a);                                    \
  }                                                                           \
  template <>                                                                 \
  EIGEN_STRONG_INLINE PACKET_TYPE psqrt<PACKET_TYPE>(const PACKET_TYPE& a) {  \
    return __builtin_elementwise_sqrt(a);                                     \
  }

EIGEN_CLANG_PACKET_MATH_FLOAT(PacketXf)
EIGEN_CLANG_PACKET_MATH_FLOAT(PacketXd)
#undef EIGEN_CLANG_PACKET_MATH_FLOAT
#endif

// --- Fused Multiply-Add (MADD) ---
#if defined(__FMA__) && EIGEN_HAS_BUILTIN(__builtin_elementwise_fma)
#define EIGEN_CLANG_PACKET_MADD(PACKET_TYPE)                                                      \
  template <>                                                                                     \
  EIGEN_STRONG_INLINE PACKET_TYPE pmadd<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b,  \
                                                     const PACKET_TYPE& c) {                      \
    return __builtin_elementwise_fma(a, b, c);                                                    \
  }                                                                                               \
  template <>                                                                                     \
  EIGEN_STRONG_INLINE PACKET_TYPE pmsub<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b,  \
                                                     const PACKET_TYPE& c) {                      \
    return __builtin_elementwise_fma(a, b, -c);                                                   \
  }                                                                                               \
  template <>                                                                                     \
  EIGEN_STRONG_INLINE PACKET_TYPE pnmadd<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b, \
                                                      const PACKET_TYPE& c) {                     \
    return __builtin_elementwise_fma(-a, b, c);                                                   \
  }                                                                                               \
  template <>                                                                                     \
  EIGEN_STRONG_INLINE PACKET_TYPE pnmsub<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b, \
                                                      const PACKET_TYPE& c) {                     \
    return -(__builtin_elementwise_fma(a, b, c));                                                 \
  }
#else
// Fallback if FMA builtin is not available
#define EIGEN_CLANG_PACKET_MADD(PACKET_TYPE)                                                      \
  template <>                                                                                     \
  EIGEN_STRONG_INLINE PACKET_TYPE pmadd<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b,  \
                                                     const PACKET_TYPE& c) {                      \
    return (a * b) + c;                                                                           \
  }                                                                                               \
  template <>                                                                                     \
  EIGEN_STRONG_INLINE PACKET_TYPE pmsub<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b,  \
                                                     const PACKET_TYPE& c) {                      \
    return (a * b) - c;                                                                           \
  }                                                                                               \
  template <>                                                                                     \
  EIGEN_STRONG_INLINE PACKET_TYPE pnmadd<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b, \
                                                      const PACKET_TYPE& c) {                     \
    return c - (a * b);                                                                           \
  }                                                                                               \
  template <>                                                                                     \
  EIGEN_STRONG_INLINE PACKET_TYPE pnmsub<PACKET_TYPE>(const PACKET_TYPE& a, const PACKET_TYPE& b, \
                                                      const PACKET_TYPE& c) {                     \
    return -((a * b) + c);                                                                        \
  }
#endif

EIGEN_CLANG_PACKET_MADD(PacketXf)
EIGEN_CLANG_PACKET_MADD(PacketXd)
#undef EIGEN_CLANG_PACKET_MADD

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
      result[i] = from[i * stride];                                                                                  \
    }                                                                                                                \
    return result;                                                                                                   \
  }

EIGEN_CLANG_PACKET_SCATTER_GATHER(PacketXf)
EIGEN_CLANG_PACKET_SCATTER_GATHER(PacketXd)
EIGEN_CLANG_PACKET_SCATTER_GATHER(PacketXi)
EIGEN_CLANG_PACKET_SCATTER_GATHER(PacketXl)

#undef EIGEN_CLANG_PACKET_SCATTER_GATHER

// ---- Various operations that depend on __builtin_shufflevector.
#if EIGEN_HAS_BUILTIN(__builtin_shufflevector)
namespace detail {

// --- Half / whole vector helpers ---
template <typename VectorT, std::size_t... Is>
EIGEN_STRONG_INLINE half_vector_t<VectorT> lower_half_impl(const VectorT& a, std::index_sequence<Is...>) {
  return __builtin_shufflevector(a, a, Is...);
}

template <typename VectorT, std::size_t... Is>
EIGEN_STRONG_INLINE half_vector_t<VectorT> upper_half_impl(const VectorT& a, std::index_sequence<Is...>) {
  return __builtin_shufflevector(a, a, (sizeof...(Is) + Is)...);
}

template <typename VectorT, typename HalfT, std::size_t... Is>
EIGEN_STRONG_INLINE VectorT concat_halves_impl(const HalfT& lo, const HalfT& hi, std::index_sequence<Is...>) {
  return __builtin_shufflevector(lo, hi, Is...);
}

template <typename VectorT>
EIGEN_STRONG_INLINE half_vector_t<VectorT> lower_half(const VectorT& a) {
  return lower_half_impl(a, vector_indices<half_vector_t<VectorT>>{});
}

template <typename VectorT>
EIGEN_STRONG_INLINE half_vector_t<VectorT> upper_half(const VectorT& a) {
  return upper_half_impl(a, vector_indices<half_vector_t<VectorT>>{});
}

template <typename VectorT, typename HalfT>
EIGEN_STRONG_INLINE VectorT concat_halves(const HalfT& lo, const HalfT& hi) {
  return concat_halves_impl<VectorT>(lo, hi, vector_indices<VectorT>{});
}

// --- Width-generic bodies for the packet operations below ---
template <typename Packet, std::size_t... Is>
EIGEN_STRONG_INLINE Packet preverse_impl(const Packet& a, std::index_sequence<Is...>) {
  return __builtin_shufflevector(a, a, (sizeof...(Is) - 1 - Is)...);
}

// Loads half a packet worth of scalars and repeats each of them twice.
template <typename Packet, std::size_t... Is>
EIGEN_STRONG_INLINE Packet ploaddup_impl(const typename unpacket_traits<Packet>::type* from,
                                         std::index_sequence<Is...>) {
  static_assert((unpacket_traits<Packet>::size) % 2 == 0, "Packet size must be a multiple of 2");
  using HalfT = half_vector_t<Packet>;
  const HalfT a = load_vector_unaligned<HalfT>(from);
  return __builtin_shufflevector(a, a, (Is / 2)...);
}

// Loads a quarter of a packet worth of scalars and repeats each of them four times.
template <typename Packet, std::size_t... Is>
EIGEN_STRONG_INLINE Packet ploadquad_impl(const typename unpacket_traits<Packet>::type* from,
                                          std::index_sequence<Is...>) {
  static_assert((unpacket_traits<Packet>::size) % 4 == 0, "Packet size must be a multiple of 4");
  using QuarterT = quarter_vector_t<Packet>;
  const QuarterT a = load_vector_unaligned<QuarterT>(from);
  return __builtin_shufflevector(a, a, (Is / 4)...);
}

template <typename Packet, std::size_t... Is>
EIGEN_STRONG_INLINE Packet plset_impl(const typename unpacket_traits<Packet>::type& a, std::index_sequence<Is...>) {
  using Scalar = typename unpacket_traits<Packet>::type;
  return Packet{(a + Scalar(Is))...};
}

// All ones in the even lanes, all zeros in the odd ones. Return the integer representation so finite fast-math cannot
// make the all-ones lanes poison before the caller applies EIGEN_FAST_MATH_CONSTANT_BARRIER.
template <typename Packet, std::size_t... Is>
EIGEN_STRONG_INLINE typename unpacket_traits<Packet>::integer_packet peven_mask_impl(std::index_sequence<Is...>) {
  using IntegerPacket = typename unpacket_traits<Packet>::integer_packet;
  using Bits = scalar_type_of_vector_t<IntegerPacket>;
  return IntegerPacket{(Is % 2 == 0 ? Bits(-1) : Bits(0))...};
}

}  // namespace detail

#define EIGEN_CLANG_PACKET_PREVERSE(PACKET_TYPE)                                \
  template <>                                                                   \
  EIGEN_STRONG_INLINE PACKET_TYPE preverse<PACKET_TYPE>(const PACKET_TYPE& a) { \
    return detail::preverse_impl(a, detail::vector_indices<PACKET_TYPE>{});     \
  }

EIGEN_CLANG_PACKET_PREVERSE(PacketXf)
EIGEN_CLANG_PACKET_PREVERSE(PacketXd)
EIGEN_CLANG_PACKET_PREVERSE(PacketXi)
EIGEN_CLANG_PACKET_PREVERSE(PacketXl)
#undef EIGEN_CLANG_PACKET_PREVERSE

#define EIGEN_CLANG_PACKET_LOADDUP(PACKET_TYPE)                                                           \
  template <>                                                                                             \
  EIGEN_STRONG_INLINE PACKET_TYPE ploaddup<PACKET_TYPE>(const unpacket_traits<PACKET_TYPE>::type* from) { \
    return detail::ploaddup_impl<PACKET_TYPE>(from, detail::vector_indices<PACKET_TYPE>{});               \
  }

EIGEN_CLANG_PACKET_LOADDUP(PacketXf)
EIGEN_CLANG_PACKET_LOADDUP(PacketXd)
EIGEN_CLANG_PACKET_LOADDUP(PacketXi)
EIGEN_CLANG_PACKET_LOADDUP(PacketXl)
#undef EIGEN_CLANG_PACKET_LOADDUP

#define EIGEN_CLANG_PACKET_LOADQUAD(PACKET_TYPE)                                                           \
  template <>                                                                                              \
  EIGEN_STRONG_INLINE PACKET_TYPE ploadquad<PACKET_TYPE>(const unpacket_traits<PACKET_TYPE>::type* from) { \
    return detail::ploadquad_impl<PACKET_TYPE>(from, detail::vector_indices<PACKET_TYPE>{});               \
  }

EIGEN_CLANG_PACKET_LOADQUAD(PacketXf)
EIGEN_CLANG_PACKET_LOADQUAD(PacketXi)
#if EIGEN_GENERIC_VECTOR_SIZE_BYTES >= 32
// PacketXd and PacketXl hold only two elements at 16 bytes, so they have no quarter packet to load from.
EIGEN_CLANG_PACKET_LOADQUAD(PacketXd)
EIGEN_CLANG_PACKET_LOADQUAD(PacketXl)
#endif
#undef EIGEN_CLANG_PACKET_LOADQUAD

#define EIGEN_CLANG_PACKET_PLSET(PACKET_TYPE)                                                       \
  template <>                                                                                       \
  EIGEN_STRONG_INLINE PACKET_TYPE plset<PACKET_TYPE>(const unpacket_traits<PACKET_TYPE>::type& a) { \
    return detail::plset_impl<PACKET_TYPE>(a, detail::vector_indices<PACKET_TYPE>{});               \
  }

EIGEN_CLANG_PACKET_PLSET(PacketXf)
EIGEN_CLANG_PACKET_PLSET(PacketXd)
EIGEN_CLANG_PACKET_PLSET(PacketXi)
EIGEN_CLANG_PACKET_PLSET(PacketXl)
#undef EIGEN_CLANG_PACKET_PLSET

// --- peven_mask ---
template <>
EIGEN_STRONG_INLINE PacketXf peven_mask(const PacketXf& /* unused */) {
  PacketXf r = numext::bit_cast<PacketXf>(detail::peven_mask_impl<PacketXf>(detail::vector_indices<PacketXf>{}));
  EIGEN_FAST_MATH_CONSTANT_BARRIER(r);
  return r;
}
template <>
EIGEN_STRONG_INLINE PacketXd peven_mask(const PacketXd& /* unused */) {
  PacketXd r = numext::bit_cast<PacketXd>(detail::peven_mask_impl<PacketXd>(detail::vector_indices<PacketXd>{}));
  EIGEN_FAST_MATH_CONSTANT_BARRIER(r);
  return r;
}

// Helpers for ptranspose.
namespace detail {

// Shuffle index of output element `i` when interleaving two vectors of `Size`
// elements each. `Group` adjacent elements move together: 1 for scalar packets,
// 2 for the complex packets in Complex.h, whose real and imaginary parts must
// stay adjacent. Output groups alternate between the two inputs, taking group
// `first_group` of each first.
template <std::size_t Group, std::size_t Size>
constexpr std::size_t zip_index(std::size_t i, std::size_t first_group) {
  return Size * ((i / Group) % 2) + Group * (first_group + i / Group / 2) + i % Group;
}

// Interleaves p1 and p2 in place, leaving the low half of the result in p1 and
// the high half in p2.
template <std::size_t Group, typename VectorT, std::size_t... Is>
EIGEN_ALWAYS_INLINE void zip_in_place_impl(VectorT& p1, VectorT& p2, std::index_sequence<Is...>) {
  constexpr std::size_t kSize = sizeof...(Is);
  // With a single lane group per vector both output shuffles would pick group
  // 0 and silently duplicate p1; such packets must not reach this code.
  static_assert(kSize >= 2 * Group, "zip_in_place needs at least two lane groups per vector");
  const VectorT tmp = __builtin_shufflevector(p1, p2, zip_index<Group, kSize>(Is, 0)...);
  p2 = __builtin_shufflevector(p1, p2, zip_index<Group, kSize>(Is, kSize / (2 * Group))...);
  p1 = tmp;
}

// Complex.h specializes this for its packet types, which zip whole complex
// values rather than individual reals.
template <typename Packet>
EIGEN_ALWAYS_INLINE void zip_in_place(Packet& p1, Packet& p2) {
  zip_in_place_impl<1>(p1, p2, vector_indices<Packet>{});
}

template <typename Packet>
EIGEN_ALWAYS_INLINE void ptranspose_impl(PacketBlock<Packet, 2>& kernel) {
  zip_in_place(kernel.packet[0], kernel.packet[1]);
}

template <typename Packet>
EIGEN_ALWAYS_INLINE void ptranspose_impl(PacketBlock<Packet, 4>& kernel) {
  zip_in_place(kernel.packet[0], kernel.packet[2]);
  zip_in_place(kernel.packet[1], kernel.packet[3]);
  zip_in_place(kernel.packet[0], kernel.packet[1]);
  zip_in_place(kernel.packet[2], kernel.packet[3]);
}

template <typename Packet>
EIGEN_ALWAYS_INLINE void ptranspose_impl(PacketBlock<Packet, 8>& kernel) {
  zip_in_place(kernel.packet[0], kernel.packet[4]);
  zip_in_place(kernel.packet[1], kernel.packet[5]);
  zip_in_place(kernel.packet[2], kernel.packet[6]);
  zip_in_place(kernel.packet[3], kernel.packet[7]);

  zip_in_place(kernel.packet[0], kernel.packet[2]);
  zip_in_place(kernel.packet[1], kernel.packet[3]);
  zip_in_place(kernel.packet[4], kernel.packet[6]);
  zip_in_place(kernel.packet[5], kernel.packet[7]);

  zip_in_place(kernel.packet[0], kernel.packet[1]);
  zip_in_place(kernel.packet[2], kernel.packet[3]);
  zip_in_place(kernel.packet[4], kernel.packet[5]);
  zip_in_place(kernel.packet[6], kernel.packet[7]);
}

template <typename Packet>
EIGEN_ALWAYS_INLINE void ptranspose_impl(PacketBlock<Packet, 16>& kernel) {
  EIGEN_UNROLL_LOOP
  for (int i = 0; i < 4; ++i) {
    const int m = (1 << i);
    EIGEN_UNROLL_LOOP
    for (int j = 0; j < m; ++j) {
      const int n = (1 << (3 - i));
      EIGEN_UNROLL_LOOP
      for (int k = 0; k < n; ++k) {
        const int idx = 2 * j * n + k;
        zip_in_place(kernel.packet[idx], kernel.packet[idx + n]);
      }
    }
  }
}

}  // namespace detail

// ptranspose overloads: only emit valid block sizes per vector size.
// At 16 bytes: float has 4 elems, double has 2 elems.
// At 32 bytes: float has 8 elems, double has 4 elems.
// At 64 bytes: float has 16 elems, double has 8 elems.

// All sizes support PacketBlock<PacketXf, 2> and PacketBlock<PacketXf, 4>.
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void ptranspose(PacketBlock<PacketXf, 4>& kernel) {
  detail::ptranspose_impl(kernel);
}
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void ptranspose(PacketBlock<PacketXf, 2>& kernel) {
  detail::ptranspose_impl(kernel);
}

// All sizes support PacketBlock<PacketXd, 2>.
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void ptranspose(PacketBlock<PacketXd, 2>& kernel) {
  detail::ptranspose_impl(kernel);
}

// All sizes support PacketBlock<PacketXi, 2> and PacketBlock<PacketXi, 4>.
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void ptranspose(PacketBlock<PacketXi, 4>& kernel) {
  detail::ptranspose_impl(kernel);
}
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void ptranspose(PacketBlock<PacketXi, 2>& kernel) {
  detail::ptranspose_impl(kernel);
}

// All sizes support PacketBlock<PacketXl, 2>.
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void ptranspose(PacketBlock<PacketXl, 2>& kernel) {
  detail::ptranspose_impl(kernel);
}

#if EIGEN_GENERIC_VECTOR_SIZE_BYTES >= 32
// 32+ bytes: float has 8+ elems, double has 4+ elems.
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void ptranspose(PacketBlock<PacketXf, 8>& kernel) {
  detail::ptranspose_impl(kernel);
}
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void ptranspose(PacketBlock<PacketXd, 4>& kernel) {
  detail::ptranspose_impl(kernel);
}
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void ptranspose(PacketBlock<PacketXi, 8>& kernel) {
  detail::ptranspose_impl(kernel);
}
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void ptranspose(PacketBlock<PacketXl, 4>& kernel) {
  detail::ptranspose_impl(kernel);
}
#endif

#if EIGEN_GENERIC_VECTOR_SIZE_BYTES >= 64
// 64 bytes: float has 16 elems, double has 8 elems.
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void ptranspose(PacketBlock<PacketXf, 16>& kernel) {
  detail::ptranspose_impl(kernel);
}
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void ptranspose(PacketBlock<PacketXd, 8>& kernel) {
  detail::ptranspose_impl(kernel);
}
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void ptranspose(PacketBlock<PacketXi, 16>& kernel) {
  detail::ptranspose_impl(kernel);
}
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void ptranspose(PacketBlock<PacketXl, 8>& kernel) {
  detail::ptranspose_impl(kernel);
}
#endif
#endif

}  // end namespace internal
}  // end namespace Eigen

#endif  // EIGEN_PACKET_MATH_CLANG_H
