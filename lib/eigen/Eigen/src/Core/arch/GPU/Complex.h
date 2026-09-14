// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2014 Benoit Steiner <benoit.steiner.goog@gmail.com>
// Copyright (C) 2021 C. Antonio Sanchez <cantonios@google.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_COMPLEX_GPU_H
#define EIGEN_COMPLEX_GPU_H

// Many std::complex methods such as operator+, operator-, operator* and
// operator/ are not constexpr. Due to this, GCC and older versions of clang do
// not treat them as device functions and thus Eigen functors making use of
// these operators fail to compile. Here, we manually specialize these
// operators for complex types when building for CUDA to enable their use
// on-device.
//
// Eigen/Core includes this header ahead of the Core headers (Meta.h's
// equal_strict, MathFunctions.h, GenericPacketMath.h, the functors), which
// apply these operators to a dependent Scalar: two-phase
// lookup finds non-ADL candidates in the definition context only, and ADL for
// std::complex<T> reaches namespace std alone, so an overload declared later
// leaves those templates bound to the host-only std:: operators. nvcc before
// CUDA 13.3 resolved them at the instantiation point, which masked the ordering.
//
// NOTES:
//  - Compound assignment operators +=,-=,*=,/=(Scalar) will not work on device,
//    since they are already specialized in the standard. Using them will result
//    in silent kernel failures.
//  - Compiling with MSVC and using +=,-=,*=,/=(std::complex<Scalar>) will lead
//    to duplicate definition errors, since these are already specialized in
//    Visual Studio's <complex> header (contrary to the standard).  This is
//    preferable to removing such definitions, which will lead to silent kernel
//    failures.
//  - Compiling with ICC requires defining _USE_COMPLEX_SPECIALIZATION_ prior
//    to the first inclusion of <complex>.
//  - Device code outside namespace Eigen that applies these operators to a
//    dependent Scalar reaches them only through `using namespace Eigen;` or
//    using-declarations in scope (see test/gpu_basic.cu); ADL alone finds the
//    host-only std:: operators.

#if defined(EIGEN_GPUCC) && defined(EIGEN_GPU_COMPILE_PHASE)

// ICC already specializes std::complex<float> and std::complex<double>
// operators, preventing us from making them device functions here.
// This will lead to silent runtime errors if the operators are used on device.
//
// To allow std::complex operator use on device, define _OVERRIDE_COMPLEX_SPECIALIZATION_
// prior to first inclusion of <complex>.  This prevents ICC from adding
// its own specializations, so our custom ones below can be used instead.
#if !(EIGEN_COMP_ICC && defined(_USE_COMPLEX_SPECIALIZATION_))

// Import Eigen's internal operator specializations.
#define EIGEN_USING_STD_COMPLEX_OPERATORS           \
  using Eigen::complex_operator_detail::operator+;  \
  using Eigen::complex_operator_detail::operator-;  \
  using Eigen::complex_operator_detail::operator*;  \
  using Eigen::complex_operator_detail::operator/;  \
  using Eigen::complex_operator_detail::operator+=; \
  using Eigen::complex_operator_detail::operator-=; \
  using Eigen::complex_operator_detail::operator*=; \
  using Eigen::complex_operator_detail::operator/=; \
  using Eigen::complex_operator_detail::operator==; \
  using Eigen::complex_operator_detail::operator!=;

// IWYU pragma: private
#include "../../InternalHeaderCheck.h"

namespace Eigen {

namespace internal {
// Defined in MathFunctions.h, which follows this header.
template <typename T>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T> complex_multiply(const std::complex<T>& a,
                                                                       const std::complex<T>& b);
template <typename T>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T> complex_divide(const std::complex<T>& a,
                                                                     const std::complex<T>& b);
}  // namespace internal

// Specialized std::complex overloads.
namespace complex_operator_detail {

// NOTE: We cannot specialize compound assignment operators with Scalar T,
//         (i.e.  operator@=(const T&), for @=+,-,*,/)
//       since they are already specialized for float/double/long double within
//       the standard <complex> header. We also do not specialize the stream
//       operators.
//       numext is not declared yet; the std::complex members used below are
//       constexpr, hence device-callable under EIGEN_CONSTEXPR_ARE_DEVICE_FUNC
//       (nvcc: --expt-relaxed-constexpr), as real_impl<std::complex<T>>
//       already assumes.
#define EIGEN_CREATE_STD_COMPLEX_OPERATOR_SPECIALIZATIONS(T)                                                        \
                                                                                                                    \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T> operator+(const std::complex<T>& a) { return a; }           \
                                                                                                                    \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T> operator-(const std::complex<T>& a) {                       \
    return std::complex<T>(-a.real(), -a.imag());                                                                   \
  }                                                                                                                 \
                                                                                                                    \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T> operator+(const std::complex<T>& a,                         \
                                                                  const std::complex<T>& b) {                       \
    return std::complex<T>(a.real() + b.real(), a.imag() + b.imag());                                               \
  }                                                                                                                 \
                                                                                                                    \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T> operator+(const std::complex<T>& a, const T& b) {           \
    return std::complex<T>(a.real() + b, a.imag());                                                                 \
  }                                                                                                                 \
                                                                                                                    \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T> operator+(const T& a, const std::complex<T>& b) {           \
    return std::complex<T>(a + b.real(), b.imag());                                                                 \
  }                                                                                                                 \
                                                                                                                    \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T> operator-(const std::complex<T>& a,                         \
                                                                  const std::complex<T>& b) {                       \
    return std::complex<T>(a.real() - b.real(), a.imag() - b.imag());                                               \
  }                                                                                                                 \
                                                                                                                    \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T> operator-(const std::complex<T>& a, const T& b) {           \
    return std::complex<T>(a.real() - b, a.imag());                                                                 \
  }                                                                                                                 \
                                                                                                                    \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T> operator-(const T& a, const std::complex<T>& b) {           \
    return std::complex<T>(a - b.real(), -b.imag());                                                                \
  }                                                                                                                 \
                                                                                                                    \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T> operator*(const std::complex<T>& a,                         \
                                                                  const std::complex<T>& b) {                       \
    return internal::complex_multiply(a, b);                                                                        \
  }                                                                                                                 \
                                                                                                                    \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T> operator*(const std::complex<T>& a, const T& b) {           \
    return std::complex<T>(a.real() * b, a.imag() * b);                                                             \
  }                                                                                                                 \
                                                                                                                    \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T> operator*(const T& a, const std::complex<T>& b) {           \
    return std::complex<T>(a * b.real(), a * b.imag());                                                             \
  }                                                                                                                 \
                                                                                                                    \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T> operator/(const std::complex<T>& a,                         \
                                                                  const std::complex<T>& b) {                       \
    return internal::complex_divide(a, b);                                                                          \
  }                                                                                                                 \
                                                                                                                    \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T> operator/(const std::complex<T>& a, const T& b) {           \
    return std::complex<T>(a.real() / b, a.imag() / b);                                                             \
  }                                                                                                                 \
                                                                                                                    \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T> operator/(const T& a, const std::complex<T>& b) {           \
    return internal::complex_divide(std::complex<T>(a, 0), b);                                                      \
  }                                                                                                                 \
                                                                                                                    \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T>& operator+=(std::complex<T>& a, const std::complex<T>& b) { \
    a = std::complex<T>(a.real() + b.real(), a.imag() + b.imag());                                                  \
    return a;                                                                                                       \
  }                                                                                                                 \
                                                                                                                    \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T>& operator-=(std::complex<T>& a, const std::complex<T>& b) { \
    a = std::complex<T>(a.real() - b.real(), a.imag() - b.imag());                                                  \
    return a;                                                                                                       \
  }                                                                                                                 \
                                                                                                                    \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T>& operator*=(std::complex<T>& a, const std::complex<T>& b) { \
    a = internal::complex_multiply(a, b);                                                                           \
    return a;                                                                                                       \
  }                                                                                                                 \
                                                                                                                    \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE std::complex<T>& operator/=(std::complex<T>& a, const std::complex<T>& b) { \
    a = internal::complex_divide(a, b);                                                                             \
    return a;                                                                                                       \
  }                                                                                                                 \
                                                                                                                    \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE bool operator==(const std::complex<T>& a, const std::complex<T>& b) {       \
    return a.real() == b.real() && a.imag() == b.imag();                                                            \
  }                                                                                                                 \
                                                                                                                    \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE bool operator==(const std::complex<T>& a, const T& b) {                     \
    return a.real() == b && a.imag() == 0;                                                                          \
  }                                                                                                                 \
                                                                                                                    \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE bool operator==(const T& a, const std::complex<T>& b) {                     \
    return a == b.real() && 0 == b.imag();                                                                          \
  }                                                                                                                 \
                                                                                                                    \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE bool operator!=(const std::complex<T>& a, const std::complex<T>& b) {       \
    return !(a == b);                                                                                               \
  }                                                                                                                 \
                                                                                                                    \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE bool operator!=(const std::complex<T>& a, const T& b) { return !(a == b); } \
                                                                                                                    \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE bool operator!=(const T& a, const std::complex<T>& b) { return !(a == b); }

// Do not specialize for long double, since that reduces to double on device.
EIGEN_CREATE_STD_COMPLEX_OPERATOR_SPECIALIZATIONS(float)
EIGEN_CREATE_STD_COMPLEX_OPERATOR_SPECIALIZATIONS(double)

#undef EIGEN_CREATE_STD_COMPLEX_OPERATOR_SPECIALIZATIONS

}  // namespace complex_operator_detail

EIGEN_USING_STD_COMPLEX_OPERATORS

namespace numext {
EIGEN_USING_STD_COMPLEX_OPERATORS
}  // namespace numext

namespace internal {
EIGEN_USING_STD_COMPLEX_OPERATORS

}  // namespace internal
}  // namespace Eigen

#undef EIGEN_USING_STD_COMPLEX_OPERATORS

#endif  // !(EIGEN_COMP_ICC && _USE_COMPLEX_SPECIALIZATION_)

#endif  // EIGEN_GPUCC && EIGEN_GPU_COMPILE_PHASE

#endif  // EIGEN_COMPLEX_GPU_H
