// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2015 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2006-2008 Benoit Jacob <jacob.benoit.1@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_MOREMETA_H
#define EIGEN_MOREMETA_H

// IWYU pragma: private
#include "../InternalHeaderCheck.h"

namespace Eigen {

namespace internal {

template <typename... tt>
struct type_list {
  constexpr static int count = sizeof...(tt);
};

template <typename t, typename... tt>
struct type_list<t, tt...> {
  constexpr static int count = sizeof...(tt) + 1;
  using first_type = t;
};

/* list manipulation: concatenate */

template <class a, class b>
struct concat;

template <typename... as, typename... bs>
struct concat<type_list<as...>, type_list<bs...>> {
  using type = type_list<as..., bs...>;
};

template <typename... p>
struct mconcat;
template <typename a>
struct mconcat<a> {
  using type = a;
};
template <typename a, typename b>
struct mconcat<a, b> : concat<a, b> {};
template <typename a, typename b, typename... cs>
struct mconcat<a, b, cs...> : concat<a, typename mconcat<b, cs...>::type> {};

/* list manipulation: extract slices */

template <int n, typename x>
struct take;

template <int n, typename a, typename... as>
struct take<n, type_list<a, as...>> : concat<type_list<a>, typename take<n - 1, type_list<as...>>::type> {};

template <int n>
struct take<n, type_list<>> {
  using type = type_list<>;
};

template <typename a, typename... as>
struct take<0, type_list<a, as...>> {
  using type = type_list<>;
};

template <>
struct take<0, type_list<>> {
  using type = type_list<>;
};

template <int n, typename a>
struct skip;

template <int n, typename a, typename... as>
struct skip<n, type_list<a, as...>> : skip<n - 1, type_list<as...>> {};

template <typename a, typename... as>
struct skip<0, type_list<a, as...>> {
  using type = type_list<a, as...>;
};

template <int n>
struct skip<n, type_list<>> {
  using type = type_list<>;
};

template <>
struct skip<0, type_list<>> {
  using type = type_list<>;
};

template <int start, int count, typename a>
struct slice : take<count, typename skip<start, a>::type> {};

/* list manipulation: retrieve single element from list */

template <int n, typename x>
struct get;

template <int n, typename a, typename... as>
struct get<n, type_list<a, as...>> : get<n - 1, type_list<as...>> {};
template <typename a, typename... as>
struct get<0, type_list<a, as...>> {
  using type = a;
};

template <typename T, int n, T a, T... as>
struct get<n, std::integer_sequence<T, a, as...>> : get<n - 1, std::integer_sequence<T, as...>> {};
template <typename T, T a, T... as>
struct get<0, std::integer_sequence<T, a, as...>> {
  constexpr static T value = a;
};

/* equality checking, flagged version */

template <typename a, typename b>
struct is_same_gf : std::is_same<a, b> {
  constexpr static int global_flags = 0;
};

/* apply_op to list */

template <template <typename, typename> class op, typename additional_param, typename a>
struct apply_op_from_left;

template <template <typename, typename> class op, typename additional_param, typename... values>
struct apply_op_from_left<op, additional_param, type_list<values...>> {
  using type = type_list<typename op<additional_param, values>::type...>;
};

template <template <typename, typename> class op, typename additional_param, typename a>
struct apply_op_from_right;

template <template <typename, typename> class op, typename additional_param, typename... values>
struct apply_op_from_right<op, additional_param, type_list<values...>> {
  using type = type_list<typename op<values, additional_param>::type...>;
};

/* see if an element is in a list */

template <template <typename, typename> class test, typename check_against, typename h_list,
          bool last_check_positive = false>
struct contained_in_list;

template <template <typename, typename> class test, typename check_against, typename h_list>
struct contained_in_list<test, check_against, h_list, true> {
  constexpr static bool value = true;
};

template <template <typename, typename> class test, typename check_against, typename a, typename... as>
struct contained_in_list<test, check_against, type_list<a, as...>, false>
    : contained_in_list<test, check_against, type_list<as...>, test<check_against, a>::value> {};

template <template <typename, typename> class test, typename check_against, typename... empty>
struct contained_in_list<test, check_against, type_list<empty...>, false> {
  constexpr static bool value = false;
};

/* see if an element is in a list and check for global flags */

template <template <typename, typename> class test, typename check_against, typename h_list, int default_flags = 0,
          bool last_check_positive = false, int last_check_flags = default_flags>
struct contained_in_list_gf;

template <template <typename, typename> class test, typename check_against, typename h_list, int default_flags,
          int last_check_flags>
struct contained_in_list_gf<test, check_against, h_list, default_flags, true, last_check_flags> {
  constexpr static bool value = true;
  constexpr static int global_flags = last_check_flags;
};

template <template <typename, typename> class test, typename check_against, typename a, typename... as,
          int default_flags, int last_check_flags>
struct contained_in_list_gf<test, check_against, type_list<a, as...>, default_flags, false, last_check_flags>
    : contained_in_list_gf<test, check_against, type_list<as...>, default_flags, test<check_against, a>::value,
                           test<check_against, a>::global_flags> {};

template <template <typename, typename> class test, typename check_against, typename... empty, int default_flags,
          int last_check_flags>
struct contained_in_list_gf<test, check_against, type_list<empty...>, default_flags, false, last_check_flags> {
  constexpr static bool value = false;
  constexpr static int global_flags = default_flags;
};

/* generic reductions */

template <typename Reducer, typename... Ts>
struct reduce;

template <typename Reducer>
struct reduce<Reducer> {
  EIGEN_DEVICE_FUNC constexpr static int run() { return Reducer::Identity; }
};

template <typename Reducer, typename A>
struct reduce<Reducer, A> {
  EIGEN_DEVICE_FUNC constexpr static A run(A a) { return a; }
};

template <typename Reducer, typename A, typename... Ts>
struct reduce<Reducer, A, Ts...> {
  EIGEN_DEVICE_FUNC constexpr static auto run(A a, Ts... ts) {
    return Reducer::run(a, reduce<Reducer, Ts...>::run(ts...));
  }
};

/* generic binary operations */

struct sum_op {
  template <typename A, typename B>
  EIGEN_DEVICE_FUNC constexpr static auto run(A a, B b) {
    return a + b;
  }
  static constexpr int Identity = 0;
};
struct product_op {
  template <typename A, typename B>
  EIGEN_DEVICE_FUNC constexpr static auto run(A a, B b) {
    return a * b;
  }
  static constexpr int Identity = 1;
};

/* reductions for lists */

template <typename... Ts>
EIGEN_DEVICE_FUNC constexpr auto arg_prod(Ts... ts) {
  return reduce<product_op, Ts...>::run(ts...);
}

template <typename... Ts>
constexpr auto arg_sum(Ts... ts) {
  return reduce<sum_op, Ts...>::run(ts...);
}

/* generic array reductions */

template <typename Reducer, typename T, std::size_t N>
struct array_reducer {
  EIGEN_DEVICE_FUNC constexpr static auto run(const array<T, N>& arr, T) {
    auto result = Reducer::run(arr[0], arr[1]);
    for (std::size_t i = 2; i < N; ++i) {
      result = Reducer::run(result, arr[i]);
    }
    return result;
  }
};

template <typename Reducer, typename T>
struct array_reducer<Reducer, T, 1> {
  EIGEN_DEVICE_FUNC constexpr static T run(const array<T, 1>& arr, T) { return arr[0]; }
};

template <typename Reducer, typename T>
struct array_reducer<Reducer, T, 0> {
  EIGEN_DEVICE_FUNC constexpr static T run(const array<T, 0>&, T identity) { return identity; }
};

template <typename Reducer, typename T, std::size_t N>
EIGEN_DEVICE_FUNC constexpr auto array_reduce(const array<T, N>& arr, T identity) {
  return array_reducer<Reducer, T, N>::run(arr, identity);
}

/* standard array reductions */

template <typename T, std::size_t N>
EIGEN_DEVICE_FUNC constexpr auto array_sum(const array<T, N>& arr) {
  return array_reduce<sum_op, T, N>(arr, static_cast<T>(0));
}

template <typename T, std::size_t N>
EIGEN_DEVICE_FUNC constexpr auto array_prod(const array<T, N>& arr) {
  return array_reduce<product_op, T, N>(arr, static_cast<T>(1));
}

template <typename t>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE t array_prod(const std::vector<t>& a) {
  eigen_assert(a.size() > 0);
  t prod = 1;
  for (size_t i = 0; i < a.size(); ++i) {
    prod *= a[i];
  }
  return prod;
}

}  // end namespace internal

}  // end namespace Eigen

#endif  // EIGEN_MOREMETA_H
