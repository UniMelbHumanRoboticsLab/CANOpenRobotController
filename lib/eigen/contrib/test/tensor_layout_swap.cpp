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

static void test_simple_swap() {
  Tensor<float, 3, ColMajor> tensor(2, 3, 7);
  tensor.setRandom();

  Tensor<float, 3, RowMajor> tensor2 = tensor.swap_layout();
  VERIFY_IS_EQUAL(tensor.dimension(0), tensor2.dimension(2));
  VERIFY_IS_EQUAL(tensor.dimension(1), tensor2.dimension(1));
  VERIFY_IS_EQUAL(tensor.dimension(2), tensor2.dimension(0));

  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 7; ++k) {
        VERIFY_IS_EQUAL(tensor(i, j, k), tensor2(k, j, i));
      }
    }
  }
}

static void test_swap_as_lvalue() {
  Tensor<float, 3, ColMajor> tensor(2, 3, 7);
  tensor.setRandom();

  Tensor<float, 3, RowMajor> tensor2(7, 3, 2);
  tensor2.swap_layout() = tensor;
  VERIFY_IS_EQUAL(tensor.dimension(0), tensor2.dimension(2));
  VERIFY_IS_EQUAL(tensor.dimension(1), tensor2.dimension(1));
  VERIFY_IS_EQUAL(tensor.dimension(2), tensor2.dimension(0));

  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 7; ++k) {
        VERIFY_IS_EQUAL(tensor(i, j, k), tensor2(k, j, i));
      }
    }
  }
}

template <int DataLayout>
static void test_block_swap_read() {
  constexpr int SwappedLayout = (DataLayout == ColMajor) ? RowMajor : ColMajor;
  Tensor<float, 3, DataLayout> tensor(17, 5, 7);
  tensor.setRandom();

  array<ptrdiff_t, 3> shuffle{{2, 0, 1}};
  Tensor<float, 3, DataLayout> shuffled = tensor.shuffle(shuffle);

  // The shuffle prefers block evaluation and serves materialized blocks; the
  // layout swap forwards the block request and re-wraps the result. This runs
  // through the tiled executor end to end.
  Tensor<float, 3, SwappedLayout> result = tensor.shuffle(shuffle).swap_layout();
  VERIFY_IS_EQUAL(result.dimension(0), shuffled.dimension(2));
  VERIFY_IS_EQUAL(result.dimension(1), shuffled.dimension(1));
  VERIFY_IS_EQUAL(result.dimension(2), shuffled.dimension(0));
  for (ptrdiff_t i = 0; i < shuffled.dimension(0); ++i) {
    for (ptrdiff_t j = 0; j < shuffled.dimension(1); ++j) {
      for (ptrdiff_t k = 0; k < shuffled.dimension(2); ++k) {
        VERIFY_IS_EQUAL(result(k, j, i), shuffled(i, j, k));
      }
    }
  }

  // A cwise expression on top of the shuffle serves lazy blocks that the
  // layout swap has to materialize itself.
  Tensor<float, 3, DataLayout> bias(shuffled.dimensions());
  bias.setRandom();
  Tensor<float, 3, SwappedLayout> result2 = (tensor.shuffle(shuffle) + bias).swap_layout();
  for (ptrdiff_t i = 0; i < shuffled.dimension(0); ++i) {
    for (ptrdiff_t j = 0; j < shuffled.dimension(1); ++j) {
      for (ptrdiff_t k = 0; k < shuffled.dimension(2); ++k) {
        VERIFY_IS_EQUAL(result2(k, j, i), shuffled(i, j, k) + bias(i, j, k));
      }
    }
  }
}

template <int DataLayout>
static void test_block_swap_write() {
  constexpr int SwappedLayout = (DataLayout == ColMajor) ? RowMajor : ColMajor;
  Tensor<float, 3, SwappedLayout> src(17, 5, 7);
  src.setRandom();

  array<ptrdiff_t, 3> shuffle{{2, 0, 1}};
  Tensor<float, 3, SwappedLayout> shuffled = src.shuffle(shuffle);
  Tensor<float, 3, SwappedLayout> bias(shuffled.dimensions());
  bias.setRandom();

  // The destination is a raw buffer viewed through a layout swap. The shuffle
  // on the right prefers block evaluation, so the tiled executor either
  // materializes right-hand-side blocks straight into the destination or
  // hands lazy blocks to the layout swap's writeBlock.
  Tensor<float, 3, DataLayout> dst(shuffled.dimension(2), shuffled.dimension(1), shuffled.dimension(0));
  dst.swap_layout() = src.shuffle(shuffle) + bias;

  for (ptrdiff_t i = 0; i < shuffled.dimension(0); ++i) {
    for (ptrdiff_t j = 0; j < shuffled.dimension(1); ++j) {
      for (ptrdiff_t k = 0; k < shuffled.dimension(2); ++k) {
        VERIFY_IS_EQUAL(dst(k, j, i), shuffled(i, j, k) + bias(i, j, k));
      }
    }
  }
}

EIGEN_DECLARE_TEST(tensor_layout_swap) {
  CALL_SUBTEST(test_simple_swap());
  CALL_SUBTEST(test_swap_as_lvalue());
  CALL_SUBTEST(test_block_swap_read<ColMajor>());
  CALL_SUBTEST(test_block_swap_read<RowMajor>());
  CALL_SUBTEST(test_block_swap_write<ColMajor>());
  CALL_SUBTEST(test_block_swap_write<RowMajor>());
}
