// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "main.h"

#include <array>
#include <Eigen/src/Core/util/Meta.h>

struct FooReturnType {
  typedef int ReturnType;
};

using Eigen::internal::apply_op_from_left;
using Eigen::internal::apply_op_from_right;
using Eigen::internal::arg_prod;
using Eigen::internal::arg_sum;
using Eigen::internal::array_prod;
using Eigen::internal::array_reduce;
using Eigen::internal::array_sum;
using Eigen::internal::concat;
using Eigen::internal::contained_in_list;
using Eigen::internal::contained_in_list_gf;
// Eigen::internal::get is intentionally left fully-qualified below: test/main.h
// does `using namespace Eigen;`, and Eigen/src/Core/StructuredBindings.h defines
// free functions `Eigen::get` that would otherwise clash with this metafunction.
using Eigen::internal::is_same_gf;
using Eigen::internal::mconcat;
using Eigen::internal::skip;
using Eigen::internal::slice;
using Eigen::internal::take;
using Eigen::internal::type_list;

struct dummy_a {};
struct dummy_b {};
struct dummy_c {};
struct dummy_d {};
struct dummy_e {};

struct widening_sum_op {
  template <typename A, typename B>
  EIGEN_DEVICE_FUNC constexpr static long long run(A a, B b) {
    return static_cast<long long>(a) + static_cast<long long>(b);
  }
};

// dummy operation for testing apply
template <typename A, typename B>
struct dummy_op;
template <>
struct dummy_op<dummy_a, dummy_b> {
  typedef dummy_c type;
};
template <>
struct dummy_op<dummy_b, dummy_a> {
  typedef dummy_d type;
};
template <>
struct dummy_op<dummy_b, dummy_c> {
  typedef dummy_a type;
};
template <>
struct dummy_op<dummy_c, dummy_b> {
  typedef dummy_d type;
};
template <>
struct dummy_op<dummy_c, dummy_a> {
  typedef dummy_b type;
};
template <>
struct dummy_op<dummy_a, dummy_c> {
  typedef dummy_d type;
};
template <>
struct dummy_op<dummy_a, dummy_a> {
  typedef dummy_e type;
};
template <>
struct dummy_op<dummy_b, dummy_b> {
  typedef dummy_e type;
};
template <>
struct dummy_op<dummy_c, dummy_c> {
  typedef dummy_e type;
};

template <typename A, typename B>
struct dummy_test {
  constexpr static bool value = false;
  constexpr static int global_flags = 0;
};
template <>
struct dummy_test<dummy_a, dummy_a> {
  constexpr static bool value = true;
  constexpr static int global_flags = 1;
};
template <>
struct dummy_test<dummy_b, dummy_b> {
  constexpr static bool value = true;
  constexpr static int global_flags = 2;
};
template <>
struct dummy_test<dummy_c, dummy_c> {
  constexpr static bool value = true;
  constexpr static int global_flags = 4;
};

static void test_concat() {
  VERIFY((std::is_same<typename concat<type_list<dummy_a, dummy_a>, type_list<>>::type,
                       type_list<dummy_a, dummy_a>>::value));
  VERIFY((std::is_same<typename concat<type_list<>, type_list<dummy_a, dummy_a>>::type,
                       type_list<dummy_a, dummy_a>>::value));
  VERIFY((std::is_same<typename concat<type_list<dummy_a, dummy_a>, type_list<dummy_a, dummy_a>>::type,
                       type_list<dummy_a, dummy_a, dummy_a, dummy_a>>::value));
  VERIFY((std::is_same<typename concat<type_list<dummy_a, dummy_a>, type_list<dummy_b, dummy_c>>::type,
                       type_list<dummy_a, dummy_a, dummy_b, dummy_c>>::value));
  VERIFY((std::is_same<typename concat<type_list<dummy_a>, type_list<dummy_b, dummy_c>>::type,
                       type_list<dummy_a, dummy_b, dummy_c>>::value));

  VERIFY((std::is_same<typename mconcat<type_list<dummy_a>>::type, type_list<dummy_a>>::value));
  VERIFY((std::is_same<typename mconcat<type_list<dummy_a>, type_list<dummy_b>>::type,
                       type_list<dummy_a, dummy_b>>::value));
  VERIFY((std::is_same<typename mconcat<type_list<dummy_a>, type_list<dummy_b>, type_list<dummy_c>>::type,
                       type_list<dummy_a, dummy_b, dummy_c>>::value));
  VERIFY((std::is_same<typename mconcat<type_list<dummy_a>, type_list<dummy_b, dummy_c>>::type,
                       type_list<dummy_a, dummy_b, dummy_c>>::value));
  VERIFY((std::is_same<typename mconcat<type_list<dummy_a, dummy_b>, type_list<dummy_c>>::type,
                       type_list<dummy_a, dummy_b, dummy_c>>::value));
}

static void test_slice() {
  typedef type_list<dummy_a, dummy_a, dummy_b, dummy_b, dummy_c, dummy_c> tl;

  VERIFY((std::is_same<typename take<0, tl>::type, type_list<>>::value));
  VERIFY((std::is_same<typename take<1, tl>::type, type_list<dummy_a>>::value));
  VERIFY((std::is_same<typename take<2, tl>::type, type_list<dummy_a, dummy_a>>::value));
  VERIFY((std::is_same<typename take<3, tl>::type, type_list<dummy_a, dummy_a, dummy_b>>::value));
  VERIFY((std::is_same<typename take<4, tl>::type, type_list<dummy_a, dummy_a, dummy_b, dummy_b>>::value));
  VERIFY((std::is_same<typename take<5, tl>::type, type_list<dummy_a, dummy_a, dummy_b, dummy_b, dummy_c>>::value));
  VERIFY((std::is_same<typename take<6, tl>::type,
                       type_list<dummy_a, dummy_a, dummy_b, dummy_b, dummy_c, dummy_c>>::value));

  VERIFY((std::is_same<typename skip<0, tl>::type,
                       type_list<dummy_a, dummy_a, dummy_b, dummy_b, dummy_c, dummy_c>>::value));
  VERIFY((std::is_same<typename skip<1, tl>::type, type_list<dummy_a, dummy_b, dummy_b, dummy_c, dummy_c>>::value));
  VERIFY((std::is_same<typename skip<2, tl>::type, type_list<dummy_b, dummy_b, dummy_c, dummy_c>>::value));
  VERIFY((std::is_same<typename skip<3, tl>::type, type_list<dummy_b, dummy_c, dummy_c>>::value));
  VERIFY((std::is_same<typename skip<4, tl>::type, type_list<dummy_c, dummy_c>>::value));
  VERIFY((std::is_same<typename skip<5, tl>::type, type_list<dummy_c>>::value));
  VERIFY((std::is_same<typename skip<6, tl>::type, type_list<>>::value));
  VERIFY((std::is_same<typename skip<7, tl>::type, type_list<>>::value));

  VERIFY((std::is_same<typename slice<0, 3, tl>::type, typename take<3, tl>::type>::value));
  VERIFY((std::is_same<typename slice<1, 3, tl>::type, type_list<dummy_a, dummy_b, dummy_b>>::value));
}

static void test_get() {
  typedef type_list<dummy_a, dummy_a, dummy_b, dummy_b, dummy_c, dummy_c> tl;
  typedef std::integer_sequence<int, 4, 8, 15, 16, 23, 42> il;

  VERIFY((std::is_same<typename Eigen::internal::get<0, tl>::type, dummy_a>::value));
  VERIFY((std::is_same<typename Eigen::internal::get<1, tl>::type, dummy_a>::value));
  VERIFY((std::is_same<typename Eigen::internal::get<2, tl>::type, dummy_b>::value));
  VERIFY((std::is_same<typename Eigen::internal::get<3, tl>::type, dummy_b>::value));
  VERIFY((std::is_same<typename Eigen::internal::get<4, tl>::type, dummy_c>::value));
  VERIFY((std::is_same<typename Eigen::internal::get<5, tl>::type, dummy_c>::value));

  VERIFY_IS_EQUAL(((int)Eigen::internal::get<0, il>::value), 4);
  VERIFY_IS_EQUAL(((int)Eigen::internal::get<1, il>::value), 8);
  VERIFY_IS_EQUAL(((int)Eigen::internal::get<2, il>::value), 15);
  VERIFY_IS_EQUAL(((int)Eigen::internal::get<3, il>::value), 16);
  VERIFY_IS_EQUAL(((int)Eigen::internal::get<4, il>::value), 23);
  VERIFY_IS_EQUAL(((int)Eigen::internal::get<5, il>::value), 42);
}

static void test_is_same_gf() {
  VERIFY((!is_same_gf<dummy_a, dummy_b>::value));
  VERIFY((!!is_same_gf<dummy_a, dummy_a>::value));
  VERIFY_IS_EQUAL((!!is_same_gf<dummy_a, dummy_b>::global_flags), false);
  VERIFY_IS_EQUAL((!!is_same_gf<dummy_a, dummy_a>::global_flags), false);
}

static void test_apply_op() {
  typedef type_list<dummy_a, dummy_b, dummy_c> tl;
  VERIFY((!!std::is_same<typename apply_op_from_left<dummy_op, dummy_a, tl>::type,
                         type_list<dummy_e, dummy_c, dummy_d>>::value));
  VERIFY((!!std::is_same<typename apply_op_from_right<dummy_op, dummy_a, tl>::type,
                         type_list<dummy_e, dummy_d, dummy_b>>::value));
  VERIFY((!!std::is_same<typename apply_op_from_left<dummy_op, dummy_a, type_list<>>::type, type_list<>>::value));
  VERIFY((!!std::is_same<typename apply_op_from_right<dummy_op, dummy_a, type_list<>>::type, type_list<>>::value));
}

static void test_contained_in_list() {
  typedef type_list<dummy_a, dummy_b, dummy_c> tl;

  VERIFY((!!contained_in_list<std::is_same, dummy_a, tl>::value));
  VERIFY((!!contained_in_list<std::is_same, dummy_b, tl>::value));
  VERIFY((!!contained_in_list<std::is_same, dummy_c, tl>::value));
  VERIFY((!contained_in_list<std::is_same, dummy_d, tl>::value));
  VERIFY((!contained_in_list<std::is_same, dummy_e, tl>::value));

  VERIFY((!!contained_in_list_gf<dummy_test, dummy_a, tl>::value));
  VERIFY((!!contained_in_list_gf<dummy_test, dummy_b, tl>::value));
  VERIFY((!!contained_in_list_gf<dummy_test, dummy_c, tl>::value));
  VERIFY((!contained_in_list_gf<dummy_test, dummy_d, tl>::value));
  VERIFY((!contained_in_list_gf<dummy_test, dummy_e, tl>::value));

  VERIFY_IS_EQUAL(((int)contained_in_list_gf<dummy_test, dummy_a, tl>::global_flags), 1);
  VERIFY_IS_EQUAL(((int)contained_in_list_gf<dummy_test, dummy_b, tl>::global_flags), 2);
  VERIFY_IS_EQUAL(((int)contained_in_list_gf<dummy_test, dummy_c, tl>::global_flags), 4);
  VERIFY_IS_EQUAL(((int)contained_in_list_gf<dummy_test, dummy_d, tl>::global_flags), 0);
  VERIFY_IS_EQUAL(((int)contained_in_list_gf<dummy_test, dummy_e, tl>::global_flags), 0);
}

static void test_arg_reductions() {
  VERIFY_IS_EQUAL(arg_sum(1, 2, 3, 4), 10);
  VERIFY_IS_EQUAL(arg_prod(1, 2, 3, 4), 24);
  VERIFY_IS_APPROX(arg_sum(0.5, 2, 5), 7.5);
  VERIFY_IS_APPROX(arg_prod(0.5, 2, 5), 5.0);
}

static void test_array_reductions() {
  array<int, 6> a{4, 8, 15, 16, 23, 42};
  array<int, 6> b{42, 23, 16, 15, 8, 4};
  array<unsigned char, 0> empty{};
  array<unsigned char, 1> singleton{200};
  array<unsigned char, 2> narrow{200, 100};
  array<unsigned char, 3> custom{200, 100, 50};

  VERIFY_IS_EQUAL((array_sum(a)), 108);
  VERIFY_IS_EQUAL((array_sum(b)), 108);
  VERIFY_IS_EQUAL((array_prod(a)), 7418880);
  VERIFY_IS_EQUAL((array_prod(b)), 7418880);
  VERIFY((std::is_same<decltype(array_sum(empty)), unsigned char>::value));
  VERIFY((std::is_same<decltype(array_sum(singleton)), unsigned char>::value));
  VERIFY((std::is_same<decltype(array_sum(narrow)), int>::value));
  VERIFY((std::is_same<decltype(array_prod(narrow)), int>::value));
  VERIFY_IS_EQUAL((array_sum(empty)), 0);
  VERIFY_IS_EQUAL((array_prod(empty)), 1);
  VERIFY_IS_EQUAL((array_sum(singleton)), 200);
  VERIFY_IS_EQUAL((array_prod(singleton)), 200);
  VERIFY_IS_EQUAL((array_sum(narrow)), 300);
  VERIFY_IS_EQUAL((array_prod(narrow)), 20000);
  VERIFY(
      (std::is_same<decltype(array_reduce<widening_sum_op>(custom, static_cast<unsigned char>(0))), long long>::value));
  VERIFY_IS_EQUAL((array_reduce<widening_sum_op>(custom, static_cast<unsigned char>(0))), 350);
}

EIGEN_DECLARE_TEST(meta) {
  VERIFY((std::is_same<float, float>::value));
  VERIFY((!std::is_same<float, double>::value));
  VERIFY((!std::is_same<float, float&>::value));
  VERIFY((!std::is_same<float, const float&>::value));

  VERIFY((std::is_same<float, internal::remove_all_t<const float&>>::value));
  VERIFY((std::is_same<float, internal::remove_all_t<const float*>>::value));
  VERIFY((std::is_same<float, internal::remove_all_t<const float*&>>::value));
  VERIFY((std::is_same<float, internal::remove_all_t<float**>>::value));
  VERIFY((std::is_same<float, internal::remove_all_t<float**&>>::value));
  VERIFY((std::is_same<float, internal::remove_all_t<float* const*&>>::value));
  VERIFY((std::is_same<float, internal::remove_all_t<float* const>>::value));

  // test add_const_on_value_type
  VERIFY((std::is_same<internal::add_const_on_value_type_t<float&>, float const&>::value));
  VERIFY((std::is_same<internal::add_const_on_value_type_t<float*>, float const*>::value));

  VERIFY((std::is_same<internal::add_const_on_value_type_t<float>, const float>::value));
  VERIFY((std::is_same<internal::add_const_on_value_type_t<const float>, const float>::value));

  VERIFY((std::is_same<internal::add_const_on_value_type_t<const float* const>, const float* const>::value));
  VERIFY((std::is_same<internal::add_const_on_value_type_t<float* const>, const float* const>::value));

  // is_convertible
  STATIC_CHECK((std::is_convertible<float, double>::value));
  STATIC_CHECK((std::is_convertible<int, double>::value));
  STATIC_CHECK((std::is_convertible<int, short>::value));
  STATIC_CHECK((std::is_convertible<short, int>::value));
  STATIC_CHECK((std::is_convertible<double, int>::value));
  STATIC_CHECK((std::is_convertible<double, std::complex<double>>::value));
  STATIC_CHECK((!std::is_convertible<std::complex<double>, double>::value));
  STATIC_CHECK((std::is_convertible<Array33f, Matrix3f>::value));
  STATIC_CHECK((std::is_convertible<Matrix3f&, Matrix3f>::value));
  STATIC_CHECK((std::is_convertible<Matrix3f&, Matrix3f&>::value));
  STATIC_CHECK((std::is_convertible<Matrix3f&, const Matrix3f&>::value));
  STATIC_CHECK((std::is_convertible<const Matrix3f&, Matrix3f>::value));
  STATIC_CHECK((std::is_convertible<const Matrix3f&, const Matrix3f&>::value));
  STATIC_CHECK((!std::is_convertible<const Matrix3f&, Matrix3f&>::value));
  STATIC_CHECK((!std::is_convertible<const Matrix3f, Matrix3f&>::value));
  STATIC_CHECK(!(std::is_convertible<Matrix3f, Matrix3f&>::value));

  STATIC_CHECK(!(std::is_convertible<int, int&>::value));
  STATIC_CHECK((std::is_convertible<const int, const int&>::value));

  // STATIC_CHECK((!std::is_convertible<Matrix3f,Matrix3d>::value )); //does not even compile because the
  // conversion is prevented by a static assertion
  STATIC_CHECK((!std::is_convertible<Array33f, int>::value));
  STATIC_CHECK((!std::is_convertible<MatrixXf, float>::value));
  {
    MatrixXf A, B;
    VectorXf a, b;
    VERIFY((std::is_convertible<decltype(a.dot(b)), float>::value));
    VERIFY((std::is_convertible<decltype(a.transpose() * b), float>::value));
    VERIFY((!std::is_convertible<decltype(A * B), float>::value));
    VERIFY((std::is_convertible<decltype(A * B), MatrixXf>::value));
  }

  {
    VERIFY((std::is_convertible<decltype(fix<3>()), int>::value));
    VERIFY((!std::is_convertible<int, decltype(fix<DynamicIndex>())>::value));
  }

  VERIFY((internal::has_ReturnType<FooReturnType>::value));
  VERIFY((internal::has_ReturnType<ScalarBinaryOpTraits<int, int>>::value));
  VERIFY((!internal::has_ReturnType<MatrixXf>::value));
  VERIFY((!internal::has_ReturnType<int>::value));

  // size_at_compile_time falls back to Dynamic rather than overflowing int
  // (46341^2 is the first square past INT_MAX).
  STATIC_CHECK((internal::size_at_compile_time(0, Dynamic) == 0));
  STATIC_CHECK((internal::size_at_compile_time(3, Dynamic) == Dynamic));
  STATIC_CHECK((internal::size_at_compile_time(46340, 46340) == 46340 * 46340));
  STATIC_CHECK((internal::size_at_compile_time(46341, 46341) == Dynamic));
  STATIC_CHECK((internal::size_at_compile_time(1 << 16, 1 << 16) == Dynamic));

  CALL_SUBTEST(test_concat());
  CALL_SUBTEST(test_slice());
  CALL_SUBTEST(test_get());
  CALL_SUBTEST(test_is_same_gf());
  CALL_SUBTEST(test_apply_op());
  CALL_SUBTEST(test_contained_in_list());
  CALL_SUBTEST(test_arg_reductions());
  CALL_SUBTEST(test_array_reductions());
}
