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

static void test_dynamic_size() {
  Eigen::DSizes<int, 3> dimensions(2, 3, 7);

  VERIFY_IS_EQUAL((int)Eigen::internal::array_get<0>(dimensions), 2);
  VERIFY_IS_EQUAL((int)Eigen::internal::array_get<1>(dimensions), 3);
  VERIFY_IS_EQUAL((int)Eigen::internal::array_get<2>(dimensions), 7);
  VERIFY_IS_EQUAL((int)dimensions.TotalSize(), 2 * 3 * 7);
  VERIFY_IS_EQUAL((int)dimensions[0], 2);
  VERIFY_IS_EQUAL((int)dimensions[1], 3);
  VERIFY_IS_EQUAL((int)dimensions[2], 7);
}

static void test_fixed_size() {
  Eigen::Sizes<2, 3, 7> dimensions;

  VERIFY_IS_EQUAL((int)Eigen::internal::array_get<0>(dimensions), 2);
  VERIFY_IS_EQUAL((int)Eigen::internal::array_get<1>(dimensions), 3);
  VERIFY_IS_EQUAL((int)Eigen::internal::array_get<2>(dimensions), 7);
  VERIFY_IS_EQUAL((int)dimensions.TotalSize(), 2 * 3 * 7);
}

static void test_match() {
  Eigen::DSizes<unsigned int, 3> dyn((unsigned int)2, (unsigned int)3, (unsigned int)7);
  Eigen::Sizes<2, 3, 7> stat;
  VERIFY_IS_EQUAL(Eigen::dimensions_match(dyn, stat), true);

  Eigen::DSizes<int, 3> dyn1(2, 3, 7);
  Eigen::DSizes<int, 2> dyn2(2, 3);
  VERIFY_IS_EQUAL(Eigen::dimensions_match(dyn1, dyn2), false);
}

static void test_rank_zero() {
  Eigen::Sizes<> scalar;
  VERIFY_IS_EQUAL((int)scalar.TotalSize(), 1);
  VERIFY_IS_EQUAL((int)scalar.rank(), 0);
  VERIFY_IS_EQUAL((int)internal::array_prod(scalar), 1);

  Eigen::DSizes<ptrdiff_t, 0> dscalar;
  VERIFY_IS_EQUAL((int)dscalar.TotalSize(), 1);
  VERIFY_IS_EQUAL((int)dscalar.rank(), 0);
}

static void test_index_type_promotion() {
  Eigen::DSizes<int, 3> src0(1, 2, 3);
  Eigen::array<int, 3> src1;
  src1[0] = 4;
  src1[1] = 5;
  src1[2] = 6;

  Eigen::DSizes<long, 3> dst0(src0);
  Eigen::DSizes<long, 3> dst1(src1);

  VERIFY_IS_EQUAL(dst0[0], 1L);
  VERIFY_IS_EQUAL(dst0[1], 2L);
  VERIFY_IS_EQUAL(dst0[2], 3L);
  VERIFY_IS_EQUAL(dst1[0], 4L);
  VERIFY_IS_EQUAL(dst1[1], 5L);
  VERIFY_IS_EQUAL(dst1[2], 6L);
}

// Regression test for issue #1616: unsigned dimension types must not trigger
// narrowing errors in the variadic constructor.
static void test_unsigned_dimensions() {
  Eigen::DSizes<int, 3> dimensions(2u, 3ul, std::size_t(7));

  VERIFY_IS_EQUAL((int)dimensions[0], 2);
  VERIFY_IS_EQUAL((int)dimensions[1], 3);
  VERIFY_IS_EQUAL((int)dimensions[2], 7);
}

// A dimension the index type cannot represent must assert rather than silently truncate.
static void test_narrowing_dimensions() {
  // Fixed width: on ILP32 targets shifting std::size_t by 40 is undefined, and GCC folds it to an in-range 0.
  VERIFY_RAISES_ASSERT((Eigen::DSizes<int, 3>(2, 3, std::uint64_t(1) << 40)));
  VERIFY_RAISES_ASSERT((Eigen::DSizes<std::ptrdiff_t, 3>(2, 3, std::size_t(-1))));
}

EIGEN_DECLARE_TEST(tensor_dimension) {
  CALL_SUBTEST(test_dynamic_size());
  CALL_SUBTEST(test_fixed_size());
  CALL_SUBTEST(test_match());
  CALL_SUBTEST(test_rank_zero());
  CALL_SUBTEST(test_index_type_promotion());
  CALL_SUBTEST(test_unsigned_dimensions());
  CALL_SUBTEST(test_narrowing_dimensions());
}
