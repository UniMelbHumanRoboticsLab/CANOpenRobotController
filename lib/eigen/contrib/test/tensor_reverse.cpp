// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2014 Navdeep Jaitly <ndjaitly@google.com and
//                    Benoit Steiner <benoit.steiner.goog@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "main.h"

#include <Eigen/Tensor>

using Eigen::array;
using Eigen::Tensor;

template <int DataLayout>
static void test_simple_reverse() {
  Tensor<float, 4, DataLayout> tensor(2, 3, 5, 7);
  tensor.setRandom();

  array<bool, 4> dim_rev;
  dim_rev[0] = false;
  dim_rev[1] = true;
  dim_rev[2] = true;
  dim_rev[3] = false;

  Tensor<float, 4, DataLayout> reversed_tensor;
  reversed_tensor = tensor.reverse(dim_rev);

  VERIFY_IS_EQUAL(reversed_tensor.dimension(0), 2);
  VERIFY_IS_EQUAL(reversed_tensor.dimension(1), 3);
  VERIFY_IS_EQUAL(reversed_tensor.dimension(2), 5);
  VERIFY_IS_EQUAL(reversed_tensor.dimension(3), 7);

  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 5; ++k) {
        for (int l = 0; l < 7; ++l) {
          VERIFY_IS_EQUAL(tensor(i, j, k, l), reversed_tensor(i, 2 - j, 4 - k, l));
        }
      }
    }
  }

  dim_rev[0] = true;
  dim_rev[1] = false;
  dim_rev[2] = false;
  dim_rev[3] = false;

  reversed_tensor = tensor.reverse(dim_rev);

  VERIFY_IS_EQUAL(reversed_tensor.dimension(0), 2);
  VERIFY_IS_EQUAL(reversed_tensor.dimension(1), 3);
  VERIFY_IS_EQUAL(reversed_tensor.dimension(2), 5);
  VERIFY_IS_EQUAL(reversed_tensor.dimension(3), 7);

  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 5; ++k) {
        for (int l = 0; l < 7; ++l) {
          VERIFY_IS_EQUAL(tensor(i, j, k, l), reversed_tensor(1 - i, j, k, l));
        }
      }
    }
  }

  dim_rev[0] = true;
  dim_rev[1] = false;
  dim_rev[2] = false;
  dim_rev[3] = true;

  reversed_tensor = tensor.reverse(dim_rev);

  VERIFY_IS_EQUAL(reversed_tensor.dimension(0), 2);
  VERIFY_IS_EQUAL(reversed_tensor.dimension(1), 3);
  VERIFY_IS_EQUAL(reversed_tensor.dimension(2), 5);
  VERIFY_IS_EQUAL(reversed_tensor.dimension(3), 7);

  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 5; ++k) {
        for (int l = 0; l < 7; ++l) {
          VERIFY_IS_EQUAL(tensor(i, j, k, l), reversed_tensor(1 - i, j, k, 6 - l));
        }
      }
    }
  }
}

template <int DataLayout>
static void test_expr_reverse(bool LValue) {
  Tensor<float, 4, DataLayout> tensor(2, 3, 5, 7);
  tensor.setRandom();

  array<bool, 4> dim_rev;
  dim_rev[0] = false;
  dim_rev[1] = true;
  dim_rev[2] = false;
  dim_rev[3] = true;

  Tensor<float, 4, DataLayout> expected(2, 3, 5, 7);
  if (LValue) {
    expected.reverse(dim_rev) = tensor;
  } else {
    expected = tensor.reverse(dim_rev);
  }

  Tensor<float, 4, DataLayout> result(2, 3, 5, 7);

  array<ptrdiff_t, 4> src_slice_dim;
  src_slice_dim[0] = 2;
  src_slice_dim[1] = 3;
  src_slice_dim[2] = 1;
  src_slice_dim[3] = 7;
  array<ptrdiff_t, 4> src_slice_start;
  src_slice_start[0] = 0;
  src_slice_start[1] = 0;
  src_slice_start[2] = 0;
  src_slice_start[3] = 0;
  array<ptrdiff_t, 4> dst_slice_dim = src_slice_dim;
  array<ptrdiff_t, 4> dst_slice_start = src_slice_start;

  for (int i = 0; i < 5; ++i) {
    if (LValue) {
      result.slice(dst_slice_start, dst_slice_dim).reverse(dim_rev) = tensor.slice(src_slice_start, src_slice_dim);
    } else {
      result.slice(dst_slice_start, dst_slice_dim) = tensor.slice(src_slice_start, src_slice_dim).reverse(dim_rev);
    }
    src_slice_start[2] += 1;
    dst_slice_start[2] += 1;
  }

  VERIFY_IS_EQUAL(result.dimension(0), 2);
  VERIFY_IS_EQUAL(result.dimension(1), 3);
  VERIFY_IS_EQUAL(result.dimension(2), 5);
  VERIFY_IS_EQUAL(result.dimension(3), 7);

  for (int i = 0; i < expected.dimension(0); ++i) {
    for (int j = 0; j < expected.dimension(1); ++j) {
      for (int k = 0; k < expected.dimension(2); ++k) {
        for (int l = 0; l < expected.dimension(3); ++l) {
          VERIFY_IS_EQUAL(result(i, j, k, l), expected(i, j, k, l));
        }
      }
    }
  }

  dst_slice_start[2] = 0;
  result.setRandom();
  for (int i = 0; i < 5; ++i) {
    if (LValue) {
      result.slice(dst_slice_start, dst_slice_dim).reverse(dim_rev) = tensor.slice(dst_slice_start, dst_slice_dim);
    } else {
      result.slice(dst_slice_start, dst_slice_dim) = tensor.reverse(dim_rev).slice(dst_slice_start, dst_slice_dim);
    }
    dst_slice_start[2] += 1;
  }

  for (int i = 0; i < expected.dimension(0); ++i) {
    for (int j = 0; j < expected.dimension(1); ++j) {
      for (int k = 0; k < expected.dimension(2); ++k) {
        for (int l = 0; l < expected.dimension(3); ++l) {
          VERIFY_IS_EQUAL(result(i, j, k, l), expected(i, j, k, l));
        }
      }
    }
  }
}

// Verify that the rvalue evaluator's packet() returns the same lanes as
// coeff() at every aligned and unaligned packet offset. This guards against
// regressions in the packet implementation that the executor-level tests
// (which only compare the assembled result) would not surface.
template <int DataLayout>
static void test_packet_reverse() {
  using namespace Eigen::internal;

  Tensor<float, 3, DataLayout> tensor(8, 5, 7);
  tensor.setRandom();

  array<bool, 3> dim_rev_inner =
      (DataLayout == ColMajor) ? array<bool, 3>{{true, false, false}} : array<bool, 3>{{false, false, true}};
  array<bool, 3> dim_rev_outer =
      (DataLayout == ColMajor) ? array<bool, 3>{{false, false, true}} : array<bool, 3>{{true, false, false}};
  array<bool, 3> dim_rev_all{{true, true, true}};

  for (const auto& dim_rev : {dim_rev_inner, dim_rev_outer, dim_rev_all}) {
    auto expr = tensor.reverse(dim_rev);
    using Eval = TensorEvaluator<const decltype(expr), DefaultDevice>;
    using Packet = typename Eval::PacketReturnType;
    constexpr int PacketSize = Eval::PacketSize;

    DefaultDevice device;
    Eval eval(expr, device);
    eval.evalSubExprsIfNeeded(nullptr);

    const Index total = tensor.size();
    EIGEN_ALIGN_MAX float lanes[PacketSize];
    for (Index offset = 0; offset + PacketSize <= total; ++offset) {
      Packet p = eval.template packet<Unaligned>(offset);
      pstoreu(lanes, p);
      for (int i = 0; i < PacketSize; ++i) {
        VERIFY_IS_EQUAL(lanes[i], eval.coeff(offset + i));
      }
    }
    eval.cleanup();
  }
}

template <typename T, int DataLayout>
static void test_reverse_write_packet_paths() {
  // Writes through a reversed destination hit the lvalue writePacket fast
  // path (contiguous store, preverse when the inner dimension is reversed).
  // Sizes with partial-packet tails; sweep all 8 reverse-flag combinations.
  Tensor<T, 3, DataLayout> src(17, 5, 7);
  src.setRandom();

  for (int mask = 0; mask < 8; ++mask) {
    array<bool, 3> rev{{(mask & 1) != 0, (mask & 2) != 0, (mask & 4) != 0}};

    Tensor<T, 3, DataLayout> dst(17, 5, 7);
    dst.setZero();
    dst.reverse(rev) = src;

    for (Index i = 0; i < 17; ++i) {
      for (Index j = 0; j < 5; ++j) {
        for (Index k = 0; k < 7; ++k) {
          const Index di = rev[0] ? 16 - i : i;
          const Index dj = rev[1] ? 4 - j : j;
          const Index dk = rev[2] ? 6 - k : k;
          VERIFY_IS_EQUAL(dst(di, dj, dk), src(i, j, k));
        }
      }
    }

    // Also through an expression source, so the RHS runs its packet path.
    Tensor<T, 3, DataLayout> dst2(17, 5, 7);
    dst2.setZero();
    dst2.reverse(rev) = src * src.constant(T(2));
    for (Index i = 0; i < 17; ++i) {
      for (Index j = 0; j < 5; ++j) {
        for (Index k = 0; k < 7; ++k) {
          const Index di = rev[0] ? 16 - i : i;
          const Index dj = rev[1] ? 4 - j : j;
          const Index dk = rev[2] ? 6 - k : k;
          VERIFY_IS_EQUAL(dst2(di, dj, dk), T(2) * src(i, j, k));
        }
      }
    }
  }
}

template <typename T, int DataLayout>
static void test_reverse_write_blocks() {
  // A right-hand side that prefers block evaluation (a shuffle) routes the
  // assignment through the tiled executor and the reverse destination's
  // writeBlock. Sweep all 8 reverse-flag combinations; sizes have
  // partial-packet tails.
  Tensor<T, 3, DataLayout> src(17, 5, 7);
  EIGEN_IF_CONSTEXPR (NumTraits<T>::IsInteger) {
    setRandomDataInRange(src, T(-1000), T(1000));
  } else {
    src.setRandom();
  }

  array<ptrdiff_t, 3> shuffle{{2, 0, 1}};
  const Tensor<T, 3, DataLayout> shuffled = src.shuffle(shuffle);

  for (int mask = 0; mask < 8; ++mask) {
    array<bool, 3> rev{{(mask & 1) != 0, (mask & 2) != 0, (mask & 4) != 0}};

    // Materialized right-hand-side blocks.
    Tensor<T, 3, DataLayout> dst(shuffled.dimensions());
    dst.setZero();
    dst.reverse(rev) = src.shuffle(shuffle);

    // Lazy right-hand-side blocks (cwise on top of the shuffle).
    Tensor<T, 3, DataLayout> dst2(shuffled.dimensions());
    dst2.setZero();
    dst2.reverse(rev) = src.shuffle(shuffle) * src.shuffle(shuffle).constant(T(2));

    for (Index i = 0; i < shuffled.dimension(0); ++i) {
      for (Index j = 0; j < shuffled.dimension(1); ++j) {
        for (Index k = 0; k < shuffled.dimension(2); ++k) {
          const Index di = rev[0] ? shuffled.dimension(0) - 1 - i : i;
          const Index dj = rev[1] ? shuffled.dimension(1) - 1 - j : j;
          const Index dk = rev[2] ? shuffled.dimension(2) - 1 - k : k;
          VERIFY_IS_EQUAL(dst(di, dj, dk), shuffled(i, j, k));
          VERIFY_IS_EQUAL(dst2(di, dj, dk), T(2) * shuffled(i, j, k));
        }
      }
    }
  }
}

EIGEN_DECLARE_TEST(tensor_reverse) {
  CALL_SUBTEST(test_simple_reverse<ColMajor>());
  CALL_SUBTEST(test_simple_reverse<RowMajor>());
  CALL_SUBTEST(test_expr_reverse<ColMajor>(true));
  CALL_SUBTEST(test_expr_reverse<RowMajor>(true));
  CALL_SUBTEST(test_expr_reverse<ColMajor>(false));
  CALL_SUBTEST(test_expr_reverse<RowMajor>(false));
  CALL_SUBTEST(test_packet_reverse<ColMajor>());
  CALL_SUBTEST(test_packet_reverse<RowMajor>());
  CALL_SUBTEST((test_reverse_write_packet_paths<float, ColMajor>()));
  CALL_SUBTEST((test_reverse_write_packet_paths<float, RowMajor>()));
  CALL_SUBTEST((test_reverse_write_packet_paths<double, ColMajor>()));
  CALL_SUBTEST((test_reverse_write_packet_paths<double, RowMajor>()));
  CALL_SUBTEST((test_reverse_write_blocks<float, ColMajor>()));
  CALL_SUBTEST((test_reverse_write_blocks<float, RowMajor>()));
  CALL_SUBTEST((test_reverse_write_blocks<int, ColMajor>()));
  CALL_SUBTEST((test_reverse_write_blocks<int, RowMajor>()));
}
