// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2014 Benoit Steiner <benoit.steiner.goog@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "main.h"

#include <Eigen/Tensor>

using Eigen::Tensor;

template <int DataLayout>
static void test_simple_striding() {
  Tensor<float, 4, DataLayout> tensor(2, 3, 5, 7);
  tensor.setRandom();
  array<ptrdiff_t, 4> strides;
  strides[0] = 1;
  strides[1] = 1;
  strides[2] = 1;
  strides[3] = 1;

  Tensor<float, 4, DataLayout> no_stride;
  no_stride = tensor.stride(strides);

  VERIFY_IS_EQUAL(no_stride.dimension(0), 2);
  VERIFY_IS_EQUAL(no_stride.dimension(1), 3);
  VERIFY_IS_EQUAL(no_stride.dimension(2), 5);
  VERIFY_IS_EQUAL(no_stride.dimension(3), 7);

  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 5; ++k) {
        for (int l = 0; l < 7; ++l) {
          VERIFY_IS_EQUAL(tensor(i, j, k, l), no_stride(i, j, k, l));
        }
      }
    }
  }

  strides[0] = 2;
  strides[1] = 4;
  strides[2] = 2;
  strides[3] = 3;
  Tensor<float, 4, DataLayout> stride;
  stride = tensor.stride(strides);

  VERIFY_IS_EQUAL(stride.dimension(0), 1);
  VERIFY_IS_EQUAL(stride.dimension(1), 1);
  VERIFY_IS_EQUAL(stride.dimension(2), 3);
  VERIFY_IS_EQUAL(stride.dimension(3), 3);

  for (int i = 0; i < 1; ++i) {
    for (int j = 0; j < 1; ++j) {
      for (int k = 0; k < 3; ++k) {
        for (int l = 0; l < 3; ++l) {
          VERIFY_IS_EQUAL(tensor(2 * i, 4 * j, 2 * k, 3 * l), stride(i, j, k, l));
        }
      }
    }
  }
}

template <int DataLayout>
static void test_striding_as_lvalue() {
  Tensor<float, 4, DataLayout> tensor(2, 3, 5, 7);
  tensor.setRandom();
  array<ptrdiff_t, 4> strides;
  strides[0] = 2;
  strides[1] = 4;
  strides[2] = 2;
  strides[3] = 3;

  Tensor<float, 4, DataLayout> result(3, 12, 10, 21);
  result.stride(strides) = tensor;

  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 5; ++k) {
        for (int l = 0; l < 7; ++l) {
          VERIFY_IS_EQUAL(tensor(i, j, k, l), result(2 * i, 4 * j, 2 * k, 3 * l));
        }
      }
    }
  }

  array<ptrdiff_t, 4> no_strides;
  no_strides[0] = 1;
  no_strides[1] = 1;
  no_strides[2] = 1;
  no_strides[3] = 1;
  Tensor<float, 4, DataLayout> result2(3, 12, 10, 21);
  result2.stride(strides) = tensor.stride(no_strides);

  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 5; ++k) {
        for (int l = 0; l < 7; ++l) {
          VERIFY_IS_EQUAL(tensor(i, j, k, l), result2(2 * i, 4 * j, 2 * k, 3 * l));
        }
      }
    }
  }
}

// setRandom() draws integers from the full range of T, so the values scaled
// below would overflow. Bounded draws keep them representable for every T.
template <typename T, int NumDims, int DataLayout>
static void set_bounded_random(Tensor<T, NumDims, DataLayout>& tensor) {
  for (Index i = 0; i < tensor.size(); ++i) {
    tensor.coeffRef(i) = internal::random<T>(T(-10), T(10));
  }
}

template <typename T, int DataLayout>
static void test_striding_packet_paths() {
  // Sizes with partial-packet tails; expression-sourced reads so the packet
  // path is exercised, sweeping strides that keep the inner dimension intact
  // (contiguous inner runs), that stride it (gathers), and the identity.
  Tensor<T, 3, DataLayout> tensor(17, 5, 7);
  set_bounded_random(tensor);

  const ptrdiff_t stride_sets[][3] = {{1, 1, 1}, {2, 1, 1}, {1, 2, 1}, {1, 1, 2}, {2, 2, 2}, {3, 1, 2}};
  for (const auto& s : stride_sets) {
    array<ptrdiff_t, 3> strides{{s[0], s[1], s[2]}};

    // Expression-sourced: the argument has no raw buffer, so the striding
    // serves packets assembled from the nested expression.
    Tensor<T, 3, DataLayout> result = (tensor * tensor.constant(T(2))).stride(strides);
    for (Index i = 0; i < result.dimension(0); ++i) {
      for (Index j = 0; j < result.dimension(1); ++j) {
        for (Index k = 0; k < result.dimension(2); ++k) {
          VERIFY_IS_EQUAL(result(i, j, k), T(2) * tensor(i * s[0], j * s[1], k * s[2]));
        }
      }
    }

    // Lvalue: writePacket through the strided destination.
    Tensor<T, 3, DataLayout> dst(17, 5, 7);
    dst.setZero();
    Tensor<T, 3, DataLayout> src((17 + s[0] - 1) / s[0], (5 + s[1] - 1) / s[1], (7 + s[2] - 1) / s[2]);
    set_bounded_random(src);
    dst.stride(strides) = src * src.constant(T(3));
    for (Index i = 0; i < src.dimension(0); ++i) {
      for (Index j = 0; j < src.dimension(1); ++j) {
        for (Index k = 0; k < src.dimension(2); ++k) {
          VERIFY_IS_EQUAL(dst(i * s[0], j * s[1], k * s[2]), T(3) * src(i, j, k));
        }
      }
    }
  }
}

template <typename T, int DataLayout>
static void test_striding_without_impl_packet_access() {
  // A TensorRef argument exposes only coefficient access, but the striding
  // still serves packets by assembling them coefficient by coefficient.
  Tensor<T, 3, DataLayout> tensor(17, 5, 7);
  tensor.setRandom();
  const Eigen::TensorRef<const Tensor<T, 3, DataLayout>> ref(tensor);

  array<ptrdiff_t, 3> identity{{1, 1, 1}};

  // Pin the capabilities this test relies on: the nested TensorRef evaluator
  // must not have packet access (otherwise this test exercises nothing), while
  // the striding evaluator on top of it must still advertise it.
  typedef Eigen::TensorRef<const Tensor<T, 3, DataLayout>> RefType;
  typedef decltype(std::declval<const RefType&>().stride(identity)) StrideExprType;
  typedef Eigen::TensorEvaluator<const RefType, Eigen::DefaultDevice> RefEvaluator;
  typedef Eigen::TensorEvaluator<const StrideExprType, Eigen::DefaultDevice> StrideEvaluator;
  EIGEN_STATIC_ASSERT(!RefEvaluator::PacketAccess, YOU_MADE_A_PROGRAMMING_MISTAKE)
  EIGEN_STATIC_ASSERT(StrideEvaluator::PacketAccess, YOU_MADE_A_PROGRAMMING_MISTAKE)

  Tensor<T, 3, DataLayout> result = ref.stride(identity);
  for (Index i = 0; i < result.size(); ++i) {
    VERIFY_IS_EQUAL(result.coeff(i), tensor.coeff(i));
  }

  array<ptrdiff_t, 3> strides{{2, 1, 3}};
  Tensor<T, 3, DataLayout> strided = ref.stride(strides);
  for (Index i = 0; i < strided.dimension(0); ++i) {
    for (Index j = 0; j < strided.dimension(1); ++j) {
      for (Index k = 0; k < strided.dimension(2); ++k) {
        VERIFY_IS_EQUAL(strided(i, j, k), tensor(i * 2, j, k * 3));
      }
    }
  }
}

EIGEN_DECLARE_TEST(tensor_striding) {
  CALL_SUBTEST(test_simple_striding<ColMajor>());
  CALL_SUBTEST(test_simple_striding<RowMajor>());
  CALL_SUBTEST(test_striding_as_lvalue<ColMajor>());
  CALL_SUBTEST(test_striding_as_lvalue<RowMajor>());
  CALL_SUBTEST((test_striding_packet_paths<float, ColMajor>()));
  CALL_SUBTEST((test_striding_packet_paths<float, RowMajor>()));
  CALL_SUBTEST((test_striding_packet_paths<int, ColMajor>()));
  CALL_SUBTEST((test_striding_packet_paths<int, RowMajor>()));
  CALL_SUBTEST((test_striding_without_impl_packet_access<float, ColMajor>()));
  CALL_SUBTEST((test_striding_without_impl_packet_access<float, RowMajor>()));
}
