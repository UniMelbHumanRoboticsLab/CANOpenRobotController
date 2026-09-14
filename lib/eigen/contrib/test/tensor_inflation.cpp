// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2015 Ke Yang <yangke@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#define EIGEN_USE_THREADS

#include "main.h"

#include <Eigen/Tensor>
#include <Eigen/ThreadPool>

using Eigen::Tensor;

template <int DataLayout>
static void test_simple_inflation() {
  Tensor<float, 4, DataLayout> tensor(2, 3, 5, 7);
  tensor.setRandom();
  array<ptrdiff_t, 4> strides;

  strides[0] = 1;
  strides[1] = 1;
  strides[2] = 1;
  strides[3] = 1;

  Tensor<float, 4, DataLayout> no_stride;
  no_stride = tensor.inflate(strides);

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
  Tensor<float, 4, DataLayout> inflated;
  inflated = tensor.inflate(strides);

  VERIFY_IS_EQUAL(inflated.dimension(0), 3);
  VERIFY_IS_EQUAL(inflated.dimension(1), 9);
  VERIFY_IS_EQUAL(inflated.dimension(2), 9);
  VERIFY_IS_EQUAL(inflated.dimension(3), 19);

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 9; ++j) {
      for (int k = 0; k < 9; ++k) {
        for (int l = 0; l < 19; ++l) {
          if (i % 2 == 0 && j % 4 == 0 && k % 2 == 0 && l % 3 == 0) {
            VERIFY_IS_EQUAL(inflated(i, j, k, l), tensor(i / 2, j / 4, k / 2, l / 3));
          } else {
            VERIFY_IS_EQUAL(0, inflated(i, j, k, l));
          }
        }
      }
    }
  }
}

template <int DataLayout>
static void test_inflation_of_expression() {
  // An expression argument has no raw buffer; the block path reads it through
  // the argument evaluator's coeff(). Sizes with partial-packet tails.
  Tensor<float, 3, DataLayout> tensor(17, 5, 7);
  tensor.setRandom();

  array<ptrdiff_t, 3> strides;
  strides[0] = 2;
  strides[1] = 3;
  strides[2] = 1;

  Tensor<float, 3, DataLayout> inflated = (tensor + tensor.constant(1.0f)).inflate(strides);

  VERIFY_IS_EQUAL(inflated.dimension(0), 33);
  VERIFY_IS_EQUAL(inflated.dimension(1), 13);
  VERIFY_IS_EQUAL(inflated.dimension(2), 7);

  for (Index i = 0; i < inflated.dimension(0); ++i) {
    for (Index j = 0; j < inflated.dimension(1); ++j) {
      for (Index k = 0; k < inflated.dimension(2); ++k) {
        if (i % 2 == 0 && j % 3 == 0) {
          VERIFY_IS_EQUAL(inflated(i, j, k), tensor(i / 2, j / 3, k) + 1.0f);
        } else {
          VERIFY_IS_EQUAL(inflated(i, j, k), 0.0f);
        }
      }
    }
  }
}

template <int DataLayout>
static void test_inflation_block_access_gating() {
  // Inflation reads its argument through coeff() in block(), so it must only
  // advertise BlockAccess when the argument itself is safe for block-style
  // (concurrent, repeated) evaluation. A random-generator nullary is repeatable
  // -- every draw is a pure function of (seed, index) -- so it keeps the tiled
  // path, as a plain tensor and a cwise expression do. A TensorRef exposes no
  // block access and must disable it.
  typedef Tensor<float, 3, DataLayout> TensorType;
  typedef Eigen::array<ptrdiff_t, 3> Strides;

  typedef decltype(std::declval<const TensorType&>().inflate(std::declval<Strides>())) PlainInflate;
  typedef decltype((std::declval<const TensorType&>() * std::declval<const TensorType&>().constant(1.0f))
                       .inflate(std::declval<Strides>())) CwiseInflate;
  typedef decltype(std::declval<const TensorType&>().random().inflate(std::declval<Strides>())) RandomInflate;
  typedef Eigen::TensorRef<const TensorType> RefType;
  typedef decltype(std::declval<const RefType&>().inflate(std::declval<Strides>())) RefInflate;

  EIGEN_STATIC_ASSERT((Eigen::TensorEvaluator<const PlainInflate, Eigen::DefaultDevice>::BlockAccess),
                      YOU_MADE_A_PROGRAMMING_MISTAKE)
  EIGEN_STATIC_ASSERT((Eigen::TensorEvaluator<const CwiseInflate, Eigen::DefaultDevice>::BlockAccess),
                      YOU_MADE_A_PROGRAMMING_MISTAKE)
  EIGEN_STATIC_ASSERT((Eigen::TensorEvaluator<const RandomInflate, Eigen::DefaultDevice>::BlockAccess),
                      YOU_MADE_A_PROGRAMMING_MISTAKE)
  EIGEN_STATIC_ASSERT((!Eigen::TensorEvaluator<const RefInflate, Eigen::DefaultDevice>::BlockAccess),
                      YOU_MADE_A_PROGRAMMING_MISTAKE)
}

template <int DataLayout>
static void test_inflation_thread_pool() {
  // Tiled ThreadPool evaluation shares the evaluator across concurrent block
  // tasks; exercise it with a raw tensor and an expression argument. The output
  // has to be large enough that the executor's block mapper emits more than one
  // block, otherwise it runs single-block on the calling thread and the
  // concurrency this test exists for never happens.
  Tensor<float, 3, DataLayout> tensor(101, 51, 33);
  tensor.setRandom();

  array<ptrdiff_t, 3> strides;
  strides[0] = 2;
  strides[1] = 3;
  strides[2] = 1;

  Eigen::array<Index, 3> inflated_dims;
  for (int i = 0; i < 3; ++i) inflated_dims[i] = (tensor.dimension(i) - 1) * strides[i] + 1;

  Eigen::ThreadPool tp(4);
  Eigen::ThreadPoolDevice device(&tp, 4);

  Tensor<float, 3, DataLayout> inflated(inflated_dims);
  inflated.device(device) = tensor.inflate(strides);

  Tensor<float, 3, DataLayout> inflated_expr(inflated_dims);
  inflated_expr.device(device) = (tensor + tensor.constant(1.0f)).inflate(strides);

  for (Index i = 0; i < inflated.dimension(0); ++i) {
    for (Index j = 0; j < inflated.dimension(1); ++j) {
      for (Index k = 0; k < inflated.dimension(2); ++k) {
        const bool on_lattice = (i % strides[0] == 0) && (j % strides[1] == 0) && (k % strides[2] == 0);
        const float expected = on_lattice ? tensor(i / strides[0], j / strides[1], k / strides[2]) : 0.0f;
        VERIFY_IS_EQUAL(inflated(i, j, k), expected);
        VERIFY_IS_EQUAL(inflated_expr(i, j, k), on_lattice ? expected + 1.0f : 0.0f);
      }
    }
  }
}

EIGEN_DECLARE_TEST(tensor_inflation) {
  CALL_SUBTEST(test_simple_inflation<ColMajor>());
  CALL_SUBTEST(test_simple_inflation<RowMajor>());
  CALL_SUBTEST(test_inflation_of_expression<ColMajor>());
  CALL_SUBTEST(test_inflation_of_expression<RowMajor>());
  CALL_SUBTEST(test_inflation_block_access_gating<ColMajor>());
  CALL_SUBTEST(test_inflation_block_access_gating<RowMajor>());
  CALL_SUBTEST(test_inflation_thread_pool<ColMajor>());
  CALL_SUBTEST(test_inflation_thread_pool<RowMajor>());
}
