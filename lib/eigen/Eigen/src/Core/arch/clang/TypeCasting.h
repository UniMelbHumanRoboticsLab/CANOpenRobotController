// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2025 Rasmus Munk Larsen
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_TYPE_CASTING_CLANG_H
#define EIGEN_TYPE_CASTING_CLANG_H

// IWYU pragma: private
#include "../../InternalHeaderCheck.h"

namespace Eigen {
namespace internal {

//==============================================================================
// preinterpret
//==============================================================================
template <>
EIGEN_STRONG_INLINE PacketXf preinterpret<PacketXf, PacketXi>(const PacketXi& a) {
  return reinterpret_cast<PacketXf>(a);
}
template <>
EIGEN_STRONG_INLINE PacketXi preinterpret<PacketXi, PacketXf>(const PacketXf& a) {
  return reinterpret_cast<PacketXi>(a);
}

template <>
EIGEN_STRONG_INLINE PacketXd preinterpret<PacketXd, PacketXl>(const PacketXl& a) {
  return reinterpret_cast<PacketXd>(a);
}
template <>
EIGEN_STRONG_INLINE PacketXl preinterpret<PacketXl, PacketXd>(const PacketXd& a) {
  return reinterpret_cast<PacketXl>(a);
}

//==============================================================================
// pcast
//==============================================================================
#if EIGEN_HAS_BUILTIN(__builtin_convertvector)
// Float-to-int conversions: __builtin_convertvector has UB for NaN/inf/
// out-of-range inputs. Replace NaN with 0 before converting so that
// pldexp_fast (which may pass NaN exponents) doesn't trigger UB.
template <>
EIGEN_STRONG_INLINE PacketXi pcast<PacketXf, PacketXi>(const PacketXf& a) {
  const PacketXf safe = a == a ? a : PacketXf(0);
  return __builtin_convertvector(safe, PacketXi);
}
template <>
EIGEN_STRONG_INLINE PacketXf pcast<PacketXi, PacketXf>(const PacketXi& a) {
  return __builtin_convertvector(a, PacketXf);
}

template <>
EIGEN_STRONG_INLINE PacketXl pcast<PacketXd, PacketXl>(const PacketXd& a) {
  const PacketXd safe = a == a ? a : PacketXd(0);
  return __builtin_convertvector(safe, PacketXl);
}
template <>
EIGEN_STRONG_INLINE PacketXd pcast<PacketXl, PacketXd>(const PacketXl& a) {
  return __builtin_convertvector(a, PacketXd);
}

// Widening and narrowing conversions run on half-width vectors: a widening cast
// consumes the lower half of its input, and a narrowing cast joins the halves
// produced from its two inputs.
template <>
EIGEN_STRONG_INLINE PacketXd pcast<PacketXf, PacketXd>(const PacketXf& a) {
  return __builtin_convertvector(detail::lower_half(a), PacketXd);
}

template <>
EIGEN_STRONG_INLINE PacketXf pcast<PacketXd, PacketXf>(const PacketXd& a, const PacketXd& b) {
  using HalfFloat = detail::half_vector_t<PacketXf>;
  return detail::concat_halves<PacketXf>(__builtin_convertvector(a, HalfFloat), __builtin_convertvector(b, HalfFloat));
}

template <>
EIGEN_STRONG_INLINE PacketXl pcast<PacketXi, PacketXl>(const PacketXi& a) {
  return __builtin_convertvector(detail::lower_half(a), PacketXl);
}

template <>
EIGEN_STRONG_INLINE PacketXi pcast<PacketXl, PacketXi>(const PacketXl& a, const PacketXl& b) {
  using HalfInt = detail::half_vector_t<PacketXi>;
  return detail::concat_halves<PacketXi>(__builtin_convertvector(a, HalfInt), __builtin_convertvector(b, HalfInt));
}
#endif

}  // end namespace internal
}  // end namespace Eigen

#endif  // EIGEN_TYPE_CASTING_CLANG_H
