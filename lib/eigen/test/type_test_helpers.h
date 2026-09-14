// SPDX-FileCopyrightText: 2006-2008 Benoit Jacob <jacob.benoit.1@gmail.com>
// SPDX-FileCopyrightText: 2008 Gael Guennebaud <gael.guennebaud@inria.fr>
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_TEST_TYPE_TEST_HELPERS_H
#define EIGEN_TEST_TYPE_TEST_HELPERS_H

#include <complex>
#include <cstdint>
#include <string>
#include <type_traits>
#include <typeinfo>

#include <Eigen/Core>

#if EIGEN_COMP_ICC
template <typename T>
inline std::string type_name();
#endif

namespace Eigen {

template <typename T1, typename T2>
std::enable_if_t<std::is_same<T1, T2>::value, bool> is_same_type(const T1&, const T2&) {
  return true;
}

}  // namespace Eigen

template <typename T>
struct GetDifferentType;

template <>
struct GetDifferentType<float> {
  typedef double type;
};
template <>
struct GetDifferentType<double> {
  typedef float type;
};
template <typename T>
struct GetDifferentType<std::complex<T>> {
  typedef std::complex<typename GetDifferentType<T>::type> type;
};

template <typename T>
inline std::string type_name(T) {
  return typeid(T).name();
}
template <>
inline std::string type_name<float>(float) {
  return "float";
}
template <>
inline std::string type_name<double>(double) {
  return "double";
}
template <>
inline std::string type_name<long double>(long double) {
  return "long double";
}
template <>
inline std::string type_name<Eigen::half>(Eigen::half) {
  return "half";
}
template <>
inline std::string type_name<Eigen::bfloat16>(Eigen::bfloat16) {
  return "bfloat16";
}
template <>
inline std::string type_name<int8_t>(int8_t) {
  return "int8_t";
}
template <>
inline std::string type_name<int16_t>(int16_t) {
  return "int16_t";
}
template <>
inline std::string type_name<int32_t>(int32_t) {
  return "int32_t";
}
template <>
inline std::string type_name<int64_t>(int64_t) {
  return "int64_t";
}
template <>
inline std::string type_name<uint8_t>(uint8_t) {
  return "uint8_t";
}
template <>
inline std::string type_name<uint16_t>(uint16_t) {
  return "uint16_t";
}
template <>
inline std::string type_name<uint32_t>(uint32_t) {
  return "uint32_t";
}
template <>
inline std::string type_name<uint64_t>(uint64_t) {
  return "uint64_t";
}
template <>
inline std::string type_name<std::complex<float>>(std::complex<float>) {
  return "complex<float>";
}
template <>
inline std::string type_name<std::complex<double>>(std::complex<double>) {
  return "complex<double>";
}
template <>
inline std::string type_name<std::complex<long double>>(std::complex<long double>) {
  return "complex<long double>";
}
template <>
inline std::string type_name<std::complex<int>>(std::complex<int>) {
  return "complex<int>";
}
template <typename T>
inline std::string type_name() {
  return type_name(T());
}

#endif  // EIGEN_TEST_TYPE_TEST_HELPERS_H
