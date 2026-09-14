// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2025 Charlie Schlosser <cs.schlosser@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_REDUCTIONS_SSE_H
#define EIGEN_REDUCTIONS_SSE_H

// IWYU pragma: private
#include "../../InternalHeaderCheck.h"

namespace Eigen {

namespace internal {

// Lane 0 of the result is lane 1 of a; the other lanes are unspecified. That is all
// the 4->2->1 reductions below need from it, so take whichever single instruction the
// target offers.
EIGEN_STRONG_INLINE Packet4f sse_lane1(const Packet4f& a) {
#ifdef EIGEN_VECTORIZE_SSE3
  return _mm_movehdup_ps(a);
#else
  return _mm_shuffle_ps(a, a, 1);
#endif
}

/* -- -- -- -- -- -- -- -- -- -- -- -- Packet16b -- -- -- -- -- -- -- -- -- -- -- -- */

// Packet16b stores one bool per byte. Reduce the byte-wise zero mask rather than extracting and short-circuiting two
// scalar halves. This also treats every nonzero byte as true, matching the scalar reduction for non-canonical inputs.
template <>
EIGEN_STRONG_INLINE Index predux_count(const Packet16b& a) {
  const __m128i normalized = _mm_min_epu8(a, _mm_set1_epi8(1));
  const __m128i sums = _mm_sad_epu8(normalized, _mm_setzero_si128());
  return static_cast<Index>(_mm_cvtsi128_si32(sums)) +
         static_cast<Index>(_mm_cvtsi128_si32(_mm_unpackhi_epi64(sums, sums)));
}

template <>
EIGEN_STRONG_INLINE bool predux(const Packet16b& a) {
  return _mm_movemask_epi8(_mm_cmpeq_epi8(a, _mm_setzero_si128())) != 0xffff;
}

template <>
EIGEN_STRONG_INLINE bool predux_mul(const Packet16b& a) {
  return _mm_movemask_epi8(_mm_cmpeq_epi8(a, _mm_setzero_si128())) == 0;
}

template <>
EIGEN_STRONG_INLINE bool predux_min(const Packet16b& a) {
  return predux_mul(a);
}

template <>
EIGEN_STRONG_INLINE bool predux_max(const Packet16b& a) {
  return predux(a);
}

template <>
EIGEN_STRONG_INLINE bool predux_any(const Packet16b& a) {
  return predux(a);
}

/* -- -- -- -- -- -- -- -- -- -- -- -- Packet4i -- -- -- -- -- -- -- -- -- -- -- -- */

template <>
EIGEN_STRONG_INLINE int predux(const Packet4i& a) {
  Packet4i tmp = _mm_add_epi32(a, _mm_shuffle_epi32(a, _MM_SHUFFLE(0, 1, 2, 3)));
  tmp = _mm_add_epi32(tmp, _mm_unpackhi_epi32(tmp, tmp));
  return _mm_cvtsi128_si32(tmp);
}

template <>
EIGEN_STRONG_INLINE int predux_mul(const Packet4i& a) {
  Packet4i tmp = pmul<Packet4i>(a, _mm_shuffle_epi32(a, _MM_SHUFFLE(0, 1, 2, 3)));
  tmp = pmul<Packet4i>(tmp, _mm_unpackhi_epi32(tmp, tmp));
  return _mm_cvtsi128_si32(tmp);
}

#ifdef EIGEN_VECTORIZE_SSE4_1
template <>
EIGEN_STRONG_INLINE int predux_min(const Packet4i& a) {
  Packet4i tmp = pmin<Packet4i>(a, _mm_shuffle_epi32(a, _MM_SHUFFLE(0, 1, 2, 3)));
  tmp = pmin<Packet4i>(tmp, _mm_unpackhi_epi32(tmp, tmp));
  return _mm_cvtsi128_si32(tmp);
}

template <>
EIGEN_STRONG_INLINE int predux_max(const Packet4i& a) {
  Packet4i tmp = pmax<Packet4i>(a, _mm_shuffle_epi32(a, _MM_SHUFFLE(0, 1, 2, 3)));
  tmp = pmax<Packet4i>(tmp, _mm_unpackhi_epi32(tmp, tmp));
  return _mm_cvtsi128_si32(tmp);
}
#endif

template <>
EIGEN_STRONG_INLINE bool predux_any(const Packet4i& a) {
  return _mm_movemask_ps(_mm_castsi128_ps(a)) != 0x0;
}

/* -- -- -- -- -- -- -- -- -- -- -- -- Packet4ui -- -- -- -- -- -- -- -- -- -- -- -- */

template <>
EIGEN_STRONG_INLINE uint32_t predux(const Packet4ui& a) {
  Packet4ui tmp = _mm_add_epi32(a, _mm_shuffle_epi32(a, _MM_SHUFFLE(0, 1, 2, 3)));
  tmp = _mm_add_epi32(tmp, _mm_unpackhi_epi32(tmp, tmp));
  return static_cast<uint32_t>(_mm_cvtsi128_si32(tmp));
}

template <>
EIGEN_STRONG_INLINE uint32_t predux_mul(const Packet4ui& a) {
  Packet4ui tmp = pmul<Packet4ui>(a, _mm_shuffle_epi32(a, _MM_SHUFFLE(0, 1, 2, 3)));
  tmp = pmul<Packet4ui>(tmp, _mm_unpackhi_epi32(tmp, tmp));
  return static_cast<uint32_t>(_mm_cvtsi128_si32(tmp));
}

#ifdef EIGEN_VECTORIZE_SSE4_1
template <>
EIGEN_STRONG_INLINE uint32_t predux_min(const Packet4ui& a) {
  Packet4ui tmp = pmin<Packet4ui>(a, _mm_shuffle_epi32(a, _MM_SHUFFLE(0, 1, 2, 3)));
  tmp = pmin<Packet4ui>(tmp, _mm_unpackhi_epi32(tmp, tmp));
  return static_cast<uint32_t>(_mm_cvtsi128_si32(tmp));
}

template <>
EIGEN_STRONG_INLINE uint32_t predux_max(const Packet4ui& a) {
  Packet4ui tmp = pmax<Packet4ui>(a, _mm_shuffle_epi32(a, _MM_SHUFFLE(0, 1, 2, 3)));
  tmp = pmax<Packet4ui>(tmp, _mm_unpackhi_epi32(tmp, tmp));
  return static_cast<uint32_t>(_mm_cvtsi128_si32(tmp));
}
#endif

template <>
EIGEN_STRONG_INLINE bool predux_any(const Packet4ui& a) {
  return _mm_movemask_ps(_mm_castsi128_ps(a)) != 0x0;
}

/* -- -- -- -- -- -- -- -- -- -- -- -- Packet2l -- -- -- -- -- -- -- -- -- -- -- -- */

template <>
EIGEN_STRONG_INLINE int64_t predux(const Packet2l& a) {
  Packet2l tmp = _mm_add_epi64(a, _mm_unpackhi_epi64(a, a));
  return pfirst(tmp);
}

template <>
EIGEN_STRONG_INLINE bool predux_any(const Packet2l& a) {
  return _mm_movemask_pd(_mm_castsi128_pd(a)) != 0x0;
}

/* -- -- -- -- -- -- -- -- -- -- -- -- Packet4f -- -- -- -- -- -- -- -- -- -- -- -- */

template <>
EIGEN_STRONG_INLINE float predux(const Packet4f& a) {
  // The 2->1 step is the low-lane form, as for Packet2d. It reads an already-reduced
  // temporary rather than the live packet, so unlike Packet2d it needs no encoding split.
  Packet4f tmp = _mm_add_ps(a, _mm_movehl_ps(a, a));
  tmp = _mm_add_ss(tmp, sse_lane1(tmp));
  return _mm_cvtss_f32(tmp);
}

template <>
EIGEN_STRONG_INLINE float predux_mul(const Packet4f& a) {
  Packet4f tmp = _mm_mul_ps(a, _mm_movehl_ps(a, a));
  tmp = _mm_mul_ss(tmp, sse_lane1(tmp));
  return _mm_cvtss_f32(tmp);
}

template <>
EIGEN_STRONG_INLINE float predux_min(const Packet4f& a) {
  Packet4f tmp = pmin<Packet4f>(a, _mm_movehl_ps(a, a));
  tmp = pmin<Packet4f>(tmp, sse_lane1(tmp));
  return _mm_cvtss_f32(tmp);
}

template <>
EIGEN_STRONG_INLINE float predux_min<PropagateNumbers>(const Packet4f& a) {
  Packet4f tmp = pmin<PropagateNumbers, Packet4f>(a, _mm_movehl_ps(a, a));
  tmp = pmin<PropagateNumbers, Packet4f>(tmp, sse_lane1(tmp));
  return _mm_cvtss_f32(tmp);
}

template <>
EIGEN_STRONG_INLINE float predux_min<PropagateNaN>(const Packet4f& a) {
  Packet4f tmp = pmin<PropagateNaN, Packet4f>(a, _mm_movehl_ps(a, a));
  tmp = pmin<PropagateNaN, Packet4f>(tmp, sse_lane1(tmp));
  return _mm_cvtss_f32(tmp);
}

template <>
EIGEN_STRONG_INLINE float predux_max(const Packet4f& a) {
  Packet4f tmp = pmax<Packet4f>(a, _mm_movehl_ps(a, a));
  tmp = pmax<Packet4f>(tmp, sse_lane1(tmp));
  return _mm_cvtss_f32(tmp);
}

template <>
EIGEN_STRONG_INLINE float predux_max<PropagateNumbers>(const Packet4f& a) {
  Packet4f tmp = pmax<PropagateNumbers, Packet4f>(a, _mm_movehl_ps(a, a));
  tmp = pmax<PropagateNumbers, Packet4f>(tmp, sse_lane1(tmp));
  return _mm_cvtss_f32(tmp);
}

template <>
EIGEN_STRONG_INLINE float predux_max<PropagateNaN>(const Packet4f& a) {
  Packet4f tmp = pmax<PropagateNaN, Packet4f>(a, _mm_movehl_ps(a, a));
  tmp = pmax<PropagateNaN, Packet4f>(tmp, sse_lane1(tmp));
  return _mm_cvtss_f32(tmp);
}

template <>
EIGEN_STRONG_INLINE bool predux_any(const Packet4f& a) {
  return _mm_movemask_ps(a) != 0x0;
}

#ifdef EIGEN_VECTORIZE_SSE4_2
template <>
EIGEN_STRONG_INLINE Index predux_count(const Packet4f& a) {
  const unsigned int mask = static_cast<unsigned int>(_mm_movemask_ps(_mm_cmpneq_ps(a, _mm_setzero_ps())));
  return Index(popcount(mask));
}
#endif

/* -- -- -- -- -- -- -- -- -- -- -- -- Packet2d -- -- -- -- -- -- -- -- -- -- -- -- */

// The 2->1 step is not packed: a packed step pins the result in a vector register, so
// neighbouring reductions -- one per coefficient of a small coeff-based product -- are
// not re-packed into one store (~16% on clang/AVX2). Take the high lane first: pfirst(a)
// is a's register, so reading it first keeps a live across the shuffle and gcc copies it
// out. Without VEX the low-lane form wins when a is an accumulator rather than the data.
template <>
EIGEN_STRONG_INLINE double predux(const Packet2d& a) {
#ifdef EIGEN_VECTORIZE_AVX
  const double hi = pfirst(preverse(a));
  return pfirst(a) + hi;
#else
  return _mm_cvtsd_f64(_mm_add_sd(a, preverse(a)));
#endif
}

template <>
EIGEN_STRONG_INLINE double predux_mul(const Packet2d& a) {
#ifdef EIGEN_VECTORIZE_AVX
  const double hi = pfirst(preverse(a));
  return pfirst(a) * hi;
#else
  return _mm_cvtsd_f64(_mm_mul_sd(a, preverse(a)));
#endif
}

template <>
EIGEN_STRONG_INLINE double predux_min(const Packet2d& a) {
#ifdef EIGEN_VECTORIZE_AVX
  const double hi = pfirst(preverse(a));
  return pmin<double>(pfirst(a), hi);
#else
  // _mm_unpackhi_pd, not preverse: clang folds this whole reduction into a scalar
  // load from lane 1, and only recognises that spelling.
  return _mm_cvtsd_f64(pmin<Packet2d>(a, _mm_unpackhi_pd(a, a)));
#endif
}

template <>
EIGEN_STRONG_INLINE double predux_min<PropagateNumbers>(const Packet2d& a) {
  Packet2d tmp = pmin<PropagateNumbers, Packet2d>(a, _mm_unpackhi_pd(a, a));
  return _mm_cvtsd_f64(tmp);
}

template <>
EIGEN_STRONG_INLINE double predux_min<PropagateNaN>(const Packet2d& a) {
  Packet2d tmp = pmin<PropagateNaN, Packet2d>(a, _mm_unpackhi_pd(a, a));
  return _mm_cvtsd_f64(tmp);
}

template <>
EIGEN_STRONG_INLINE double predux_max(const Packet2d& a) {
#ifdef EIGEN_VECTORIZE_AVX
  const double hi = pfirst(preverse(a));
  return pmax<double>(pfirst(a), hi);
#else
  // _mm_unpackhi_pd, not preverse: clang folds this whole reduction into a scalar
  // load from lane 1, and only recognises that spelling.
  return _mm_cvtsd_f64(pmax<Packet2d>(a, _mm_unpackhi_pd(a, a)));
#endif
}

template <>
EIGEN_STRONG_INLINE double predux_max<PropagateNumbers>(const Packet2d& a) {
  Packet2d tmp = pmax<PropagateNumbers, Packet2d>(a, _mm_unpackhi_pd(a, a));
  return _mm_cvtsd_f64(tmp);
}

template <>
EIGEN_STRONG_INLINE double predux_max<PropagateNaN>(const Packet2d& a) {
  Packet2d tmp = pmax<PropagateNaN, Packet2d>(a, _mm_unpackhi_pd(a, a));
  return _mm_cvtsd_f64(tmp);
}

template <>
EIGEN_STRONG_INLINE bool predux_any(const Packet2d& a) {
  return _mm_movemask_pd(a) != 0x0;
}

#ifdef EIGEN_VECTORIZE_SSE4_2
template <>
EIGEN_STRONG_INLINE Index predux_count(const Packet2d& a) {
  const unsigned int mask = static_cast<unsigned int>(_mm_movemask_pd(_mm_cmpneq_pd(a, _mm_setzero_pd())));
  return Index(popcount(mask));
}
#endif

}  // end namespace internal

}  // end namespace Eigen

#endif  // EIGEN_REDUCTIONS_SSE_H
