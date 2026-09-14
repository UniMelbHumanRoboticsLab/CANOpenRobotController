// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Pavel Guzenfeld
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "main.h"

#if EIGEN_MAX_CPP_VER >= 17 && EIGEN_COMP_CXXVER >= 17

template <typename Scalar>
void check_vector_bindings() {
  // Vector2
  {
    Matrix<Scalar, 2, 1> v;
    v << Scalar(1), Scalar(2);
    auto [x, y] = v;
    VERIFY_IS_EQUAL(x, Scalar(1));
    VERIFY_IS_EQUAL(y, Scalar(2));
  }

  // Vector3
  {
    Matrix<Scalar, 3, 1> v;
    v << Scalar(3), Scalar(4), Scalar(5);
    auto [x, y, z] = v;
    VERIFY_IS_EQUAL(x, Scalar(3));
    VERIFY_IS_EQUAL(y, Scalar(4));
    VERIFY_IS_EQUAL(z, Scalar(5));
  }

  // Vector4
  {
    Matrix<Scalar, 4, 1> v;
    v << Scalar(6), Scalar(7), Scalar(8), Scalar(9);
    auto [a, b, c, d] = v;
    VERIFY_IS_EQUAL(a, Scalar(6));
    VERIFY_IS_EQUAL(b, Scalar(7));
    VERIFY_IS_EQUAL(c, Scalar(8));
    VERIFY_IS_EQUAL(d, Scalar(9));
  }

  // 1x1 matrix (scalar-like)
  {
    Matrix<Scalar, 1, 1> s;
    s << Scalar(42);
    auto [val] = s;
    VERIFY_IS_EQUAL(val, Scalar(42));
  }

  // RowVector
  {
    Matrix<Scalar, 1, 3> rv;
    rv << Scalar(10), Scalar(20), Scalar(30);
    auto [a, b, c] = rv;
    VERIFY_IS_EQUAL(a, Scalar(10));
    VERIFY_IS_EQUAL(b, Scalar(20));
    VERIFY_IS_EQUAL(c, Scalar(30));
  }
}

template <typename Scalar>
void check_array_bindings() {
  // Array3
  {
    Array<Scalar, 3, 1> a;
    a << Scalar(100), Scalar(200), Scalar(300);
    auto [x, y, z] = a;
    VERIFY_IS_EQUAL(x, Scalar(100));
    VERIFY_IS_EQUAL(y, Scalar(200));
    VERIFY_IS_EQUAL(z, Scalar(300));
  }

  // Array2
  {
    Array<Scalar, 2, 1> a;
    a << Scalar(10), Scalar(20);
    auto [x, y] = a;
    VERIFY_IS_EQUAL(x, Scalar(10));
    VERIFY_IS_EQUAL(y, Scalar(20));
  }
}

template <typename Scalar>
void check_reference_bindings() {
  // Mutable reference binding
  {
    Matrix<Scalar, 3, 1> v;
    v << Scalar(1), Scalar(2), Scalar(3);
    auto& [x, y, z] = v;
    x = Scalar(10);
    y = Scalar(20);
    z = Scalar(30);
    VERIFY_IS_EQUAL(v(0), Scalar(10));
    VERIFY_IS_EQUAL(v(1), Scalar(20));
    VERIFY_IS_EQUAL(v(2), Scalar(30));
  }

  // Const reference binding
  {
    const Matrix<Scalar, 3, 1> v(Scalar(4), Scalar(5), Scalar(6));
    const auto& [x, y, z] = v;
    VERIFY_IS_EQUAL(x, Scalar(4));
    VERIFY_IS_EQUAL(y, Scalar(5));
    VERIFY_IS_EQUAL(z, Scalar(6));
  }

  // Array mutable reference binding
  {
    Array<Scalar, 2, 1> a;
    a << Scalar(7), Scalar(8);
    auto& [x, y] = a;
    x = Scalar(70);
    VERIFY_IS_EQUAL(a(0), Scalar(70));
    VERIFY_IS_EQUAL(a(1), Scalar(8));
  }
}

template <typename Scalar>
void check_matrix_bindings() {
  // 2x2 matrix (column-major order)
  {
    Matrix<Scalar, 2, 2> m;
    m << Scalar(1), Scalar(2), Scalar(3), Scalar(4);
    auto [m00, m10, m01, m11] = m;
    // Column-major: (0,0), (1,0), (0,1), (1,1)
    VERIFY_IS_EQUAL(m00, Scalar(1));
    VERIFY_IS_EQUAL(m10, Scalar(3));
    VERIFY_IS_EQUAL(m01, Scalar(2));
    VERIFY_IS_EQUAL(m11, Scalar(4));
  }
}

template <typename Scalar>
void check_storage_order_semantics() {
  // Row vectors are forced to RowMajor by Eigen (a 1xN can't meaningfully be
  // column-major). Ensure decomposition still matches the single-row layout.
  {
    Matrix<Scalar, 1, 3> rv;
    rv << Scalar(1), Scalar(2), Scalar(3);
    auto [a, b, c] = rv;
    VERIFY_IS_EQUAL(a, Scalar(1));
    VERIFY_IS_EQUAL(b, Scalar(2));
    VERIFY_IS_EQUAL(c, Scalar(3));
  }
  // Nx1 column vectors are ColMajor regardless and decompose top-to-bottom.
  {
    Matrix<Scalar, 3, 1> cv;
    cv << Scalar(4), Scalar(5), Scalar(6);
    auto [a, b, c] = cv;
    VERIFY_IS_EQUAL(a, Scalar(4));
    VERIFY_IS_EQUAL(b, Scalar(5));
    VERIFY_IS_EQUAL(c, Scalar(6));
  }
  // 2D ColMajor matrix decomposes in column-major order: (0,0),(1,0),(0,1),(1,1).
  // 2D RowMajor is rejected via static_assert — see failtest/structured_bindings_rowmajor.cpp.
  {
    Matrix<Scalar, 2, 2, ColMajor> m;
    m << Scalar(1), Scalar(2), Scalar(3), Scalar(4);
    auto [m00, m10, m01, m11] = m;
    VERIFY_IS_EQUAL(m00, Scalar(1));
    VERIFY_IS_EQUAL(m10, Scalar(3));
    VERIFY_IS_EQUAL(m01, Scalar(2));
    VERIFY_IS_EQUAL(m11, Scalar(4));
  }
}

void check_tuple_size() {
  STATIC_CHECK((std::tuple_size<Vector2d>::value == 2));
  STATIC_CHECK((std::tuple_size<Vector3f>::value == 3));
  STATIC_CHECK((std::tuple_size<Vector4i>::value == 4));
  STATIC_CHECK((std::tuple_size<Matrix2d>::value == 4));
  STATIC_CHECK((std::tuple_size<Matrix3f>::value == 9));
  STATIC_CHECK((std::tuple_size<Array3i>::value == 3));
  STATIC_CHECK((std::tuple_size<Array<double, 2, 1>>::value == 2));
  STATIC_CHECK((std::tuple_size<Matrix<float, 1, 1>>::value == 1));
  STATIC_CHECK((std::tuple_size<RowVector3d>::value == 3));
}

void check_tuple_element() {
  STATIC_CHECK((std::is_same<std::tuple_element_t<0, Vector3d>, double>::value));
  STATIC_CHECK((std::is_same<std::tuple_element_t<1, Vector3f>, float>::value));
  STATIC_CHECK((std::is_same<std::tuple_element_t<2, Vector4i>, int>::value));
  STATIC_CHECK((std::is_same<std::tuple_element_t<0, Array3i>, int>::value));
}

// Emulates generic tuple-like detection as done by fmt's range formatter
// (issue #3103): probing tuple_size<T>::value in a SFINAE context must be a
// substitution failure for dynamic-size types, not a hard error.
template <typename T, typename = void>
struct is_tuple_like : std::false_type {};

template <typename T>
struct is_tuple_like<T, internal::void_t<decltype(std::tuple_size<T>::value)>> : std::true_type {};

template <typename T, typename = void>
struct has_tuple_element0 : std::false_type {};

template <typename T>
struct has_tuple_element0<T, internal::void_t<typename std::tuple_element<0, T>::type>> : std::true_type {};

void check_sfinae_friendly_detection() {
  STATIC_CHECK((is_tuple_like<Vector3d>::value));
  STATIC_CHECK((is_tuple_like<Matrix2f>::value));
  STATIC_CHECK((is_tuple_like<Array3i>::value));
  STATIC_CHECK((!is_tuple_like<VectorXd>::value));
  STATIC_CHECK((!is_tuple_like<MatrixXf>::value));
  STATIC_CHECK((!is_tuple_like<ArrayXd>::value));
  STATIC_CHECK((!is_tuple_like<Matrix<double, 3, Dynamic>>::value));
  STATIC_CHECK((!is_tuple_like<Matrix<double, Dynamic, 3>>::value));
  STATIC_CHECK((has_tuple_element0<Vector3d>::value));
  STATIC_CHECK((has_tuple_element0<Array3i>::value));
  STATIC_CHECK((!has_tuple_element0<VectorXd>::value));
  STATIC_CHECK((!has_tuple_element0<ArrayXXf>::value));
}

EIGEN_DECLARE_TEST(structured_bindings) {
  CALL_SUBTEST_1(check_vector_bindings<double>());
  CALL_SUBTEST_1(check_vector_bindings<float>());
  CALL_SUBTEST_1(check_vector_bindings<int>());
  CALL_SUBTEST_2(check_array_bindings<double>());
  CALL_SUBTEST_2(check_array_bindings<int>());
  CALL_SUBTEST_3(check_reference_bindings<double>());
  CALL_SUBTEST_3(check_reference_bindings<float>());
  CALL_SUBTEST_4(check_matrix_bindings<double>());
  CALL_SUBTEST_4(check_matrix_bindings<int>());
  CALL_SUBTEST_5(check_tuple_size());
  CALL_SUBTEST_5(check_tuple_element());
  CALL_SUBTEST_5(check_sfinae_friendly_detection());
  CALL_SUBTEST_6(check_storage_order_semantics<double>());
  CALL_SUBTEST_6(check_storage_order_semantics<int>());
}

#else

EIGEN_DECLARE_TEST(structured_bindings) {
  // Structured bindings require C++17.
  VERIFY(true);
}

#endif
