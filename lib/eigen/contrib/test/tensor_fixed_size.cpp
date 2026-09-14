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

using Eigen::RowMajor;
using Eigen::Tensor;

static void test_0d() {
  TensorFixedSize<float, Sizes<> > scalar1;
  TensorFixedSize<float, Sizes<>, RowMajor> scalar2;
  VERIFY_IS_EQUAL(scalar1.rank(), 0);
  VERIFY_IS_EQUAL(scalar1.size(), 1);
  VERIFY_IS_EQUAL(internal::array_prod(scalar1.dimensions()), 1);

  scalar1() = 7.0;
  scalar2() = 13.0;

  // Test against shallow copy.
  TensorFixedSize<float, Sizes<> > copy = scalar1;
  VERIFY_IS_NOT_EQUAL(scalar1.data(), copy.data());
  VERIFY_IS_APPROX(scalar1(), copy());
  copy = scalar1;
  VERIFY_IS_NOT_EQUAL(scalar1.data(), copy.data());
  VERIFY_IS_APPROX(scalar1(), copy());

  TensorFixedSize<float, Sizes<> > scalar3 = scalar1.sqrt();
  TensorFixedSize<float, Sizes<>, RowMajor> scalar4 = scalar2.sqrt();
  VERIFY_IS_EQUAL(scalar3.rank(), 0);
  VERIFY_IS_APPROX(scalar3(), sqrtf(7.0));
  VERIFY_IS_APPROX(scalar4(), sqrtf(13.0));

  scalar3 = scalar1 + scalar2;
  VERIFY_IS_APPROX(scalar3(), 7.0f + 13.0f);
}

static void test_1d() {
  TensorFixedSize<float, Sizes<6> > vec1;
  TensorFixedSize<float, Sizes<6>, RowMajor> vec2;

  VERIFY_IS_EQUAL((vec1.size()), 6);
  //  VERIFY_IS_EQUAL((vec1.dimensions()[0]), 6);
  //  VERIFY_IS_EQUAL((vec1.dimension(0)), 6);

  vec1(0) = 4.0;
  vec2(0) = 0.0;
  vec1(1) = 8.0;
  vec2(1) = 1.0;
  vec1(2) = 15.0;
  vec2(2) = 2.0;
  vec1(3) = 16.0;
  vec2(3) = 3.0;
  vec1(4) = 23.0;
  vec2(4) = 4.0;
  vec1(5) = 42.0;
  vec2(5) = 5.0;

  // Test against shallow copy.
  TensorFixedSize<float, Sizes<6> > copy = vec1;
  VERIFY_IS_NOT_EQUAL(vec1.data(), copy.data());
  for (int i = 0; i < 6; ++i) {
    VERIFY_IS_APPROX(vec1(i), copy(i));
  }
  copy = vec1;
  VERIFY_IS_NOT_EQUAL(vec1.data(), copy.data());
  for (int i = 0; i < 6; ++i) {
    VERIFY_IS_APPROX(vec1(i), copy(i));
  }

  TensorFixedSize<float, Sizes<6> > vec3 = vec1.sqrt();
  TensorFixedSize<float, Sizes<6>, RowMajor> vec4 = vec2.sqrt();

  VERIFY_IS_EQUAL((vec3.size()), 6);
  VERIFY_IS_EQUAL(vec3.rank(), 1);
  //  VERIFY_IS_EQUAL((vec3.dimensions()[0]), 6);
  //  VERIFY_IS_EQUAL((vec3.dimension(0)), 6);

  VERIFY_IS_APPROX(vec3(0), sqrtf(4.0));
  VERIFY_IS_APPROX(vec3(1), sqrtf(8.0));
  VERIFY_IS_APPROX(vec3(2), sqrtf(15.0));
  VERIFY_IS_APPROX(vec3(3), sqrtf(16.0));
  VERIFY_IS_APPROX(vec3(4), sqrtf(23.0));
  VERIFY_IS_APPROX(vec3(5), sqrtf(42.0));

  VERIFY_IS_APPROX(vec4(0), sqrtf(0.0));
  VERIFY_IS_APPROX(vec4(1), sqrtf(1.0));
  VERIFY_IS_APPROX(vec4(2), sqrtf(2.0));
  VERIFY_IS_APPROX(vec4(3), sqrtf(3.0));
  VERIFY_IS_APPROX(vec4(4), sqrtf(4.0));
  VERIFY_IS_APPROX(vec4(5), sqrtf(5.0));

  vec3 = vec1 + vec2;
  VERIFY_IS_APPROX(vec3(0), 4.0f + 0.0f);
  VERIFY_IS_APPROX(vec3(1), 8.0f + 1.0f);
  VERIFY_IS_APPROX(vec3(2), 15.0f + 2.0f);
  VERIFY_IS_APPROX(vec3(3), 16.0f + 3.0f);
  VERIFY_IS_APPROX(vec3(4), 23.0f + 4.0f);
  VERIFY_IS_APPROX(vec3(5), 42.0f + 5.0f);
}

static void test_tensor_map() {
  TensorFixedSize<float, Sizes<6> > vec1;
  TensorFixedSize<float, Sizes<6>, RowMajor> vec2;

  vec1(0) = 4.0;
  vec2(0) = 0.0;
  vec1(1) = 8.0;
  vec2(1) = 1.0;
  vec1(2) = 15.0;
  vec2(2) = 2.0;
  vec1(3) = 16.0;
  vec2(3) = 3.0;
  vec1(4) = 23.0;
  vec2(4) = 4.0;
  vec1(5) = 42.0;
  vec2(5) = 5.0;

  float data3[6];
  TensorMap<TensorFixedSize<float, Sizes<6> > > vec3(data3, 6);
  vec3 = vec1.sqrt() + vec2;

  VERIFY_IS_APPROX(vec3(0), sqrtf(4.0));
  VERIFY_IS_APPROX(vec3(1), sqrtf(8.0) + 1.0f);
  VERIFY_IS_APPROX(vec3(2), sqrtf(15.0) + 2.0f);
  VERIFY_IS_APPROX(vec3(3), sqrtf(16.0) + 3.0f);
  VERIFY_IS_APPROX(vec3(4), sqrtf(23.0) + 4.0f);
  VERIFY_IS_APPROX(vec3(5), sqrtf(42.0) + 5.0f);
}

static void test_2d() {
  float data1[6];
  TensorMap<TensorFixedSize<float, Sizes<2, 3> > > mat1(data1, 2, 3);
  float data2[6];
  TensorMap<TensorFixedSize<float, Sizes<2, 3>, RowMajor> > mat2(data2, 2, 3);

  VERIFY_IS_EQUAL((mat1.size()), 2 * 3);
  VERIFY_IS_EQUAL(mat1.rank(), 2);
  //  VERIFY_IS_EQUAL((mat1.dimension(0)), 2);
  //  VERIFY_IS_EQUAL((mat1.dimension(1)), 3);

  mat1(0, 0) = 0.0;
  mat1(0, 1) = 1.0;
  mat1(0, 2) = 2.0;
  mat1(1, 0) = 3.0;
  mat1(1, 1) = 4.0;
  mat1(1, 2) = 5.0;

  mat2(0, 0) = -0.0;
  mat2(0, 1) = -1.0;
  mat2(0, 2) = -2.0;
  mat2(1, 0) = -3.0;
  mat2(1, 1) = -4.0;
  mat2(1, 2) = -5.0;

  TensorFixedSize<float, Sizes<2, 3> > mat3;
  TensorFixedSize<float, Sizes<2, 3>, RowMajor> mat4;
  mat3 = mat1.abs();
  mat4 = mat2.abs();

  VERIFY_IS_EQUAL((mat3.size()), 2 * 3);
  //  VERIFY_IS_EQUAL((mat3.dimension(0)), 2);
  //  VERIFY_IS_EQUAL((mat3.dimension(1)), 3);

  VERIFY_IS_APPROX(mat3(0, 0), 0.0f);
  VERIFY_IS_APPROX(mat3(0, 1), 1.0f);
  VERIFY_IS_APPROX(mat3(0, 2), 2.0f);
  VERIFY_IS_APPROX(mat3(1, 0), 3.0f);
  VERIFY_IS_APPROX(mat3(1, 1), 4.0f);
  VERIFY_IS_APPROX(mat3(1, 2), 5.0f);

  VERIFY_IS_APPROX(mat4(0, 0), 0.0f);
  VERIFY_IS_APPROX(mat4(0, 1), 1.0f);
  VERIFY_IS_APPROX(mat4(0, 2), 2.0f);
  VERIFY_IS_APPROX(mat4(1, 0), 3.0f);
  VERIFY_IS_APPROX(mat4(1, 1), 4.0f);
  VERIFY_IS_APPROX(mat4(1, 2), 5.0f);
}

static void test_3d() {
  TensorFixedSize<float, Sizes<2, 3, 7> > mat1;
  TensorFixedSize<float, Sizes<2, 3, 7>, RowMajor> mat2;

  VERIFY_IS_EQUAL((mat1.size()), 2 * 3 * 7);
  VERIFY_IS_EQUAL(mat1.rank(), 3);
  //  VERIFY_IS_EQUAL((mat1.dimension(0)), 2);
  //  VERIFY_IS_EQUAL((mat1.dimension(1)), 3);
  //  VERIFY_IS_EQUAL((mat1.dimension(2)), 7);

  float val = 0.0f;
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 7; ++k) {
        mat1(i, j, k) = val;
        mat2(i, j, k) = val;
        val += 1.0f;
      }
    }
  }

  TensorFixedSize<float, Sizes<2, 3, 7> > mat3;
  mat3 = mat1.sqrt();
  TensorFixedSize<float, Sizes<2, 3, 7>, RowMajor> mat4;
  mat4 = mat2.sqrt();

  VERIFY_IS_EQUAL((mat3.size()), 2 * 3 * 7);
  //  VERIFY_IS_EQUAL((mat3.dimension(0)), 2);
  //  VERIFY_IS_EQUAL((mat3.dimension(1)), 3);
  //  VERIFY_IS_EQUAL((mat3.dimension(2)), 7);

  val = 0.0f;
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 7; ++k) {
        VERIFY_IS_APPROX(mat3(i, j, k), sqrtf(val));
        VERIFY_IS_APPROX(mat4(i, j, k), sqrtf(val));
        val += 1.0f;
      }
    }
  }
}

static void test_array() {
  TensorFixedSize<float, Sizes<2, 3, 7> > mat1;
  float val = 0.0f;
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 7; ++k) {
        mat1(i, j, k) = val;
        val += 1.0f;
      }
    }
  }

  TensorFixedSize<float, Sizes<2, 3, 7> > mat3;
  mat3 = mat1.pow(3.5f);

  val = 0.0f;
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 7; ++k) {
        VERIFY_IS_APPROX(mat3(i, j, k), powf(val, 3.5f));
        val += 1.0f;
      }
    }
  }
}

// Sanity check: `TensorFixedSize` whose element count isn't a multiple of the packet width
// (5 floats) must still be safely usable when embedded in a larger aligned struct at
// a non-16-byte-aligned offset.  Since `IsAligned` (and thus the `Aligned`/`Unaligned`
// `LoadMode` `TensorEvaluator` picks for packet ops) is a flat, `Size`-independent trait,
// it must not assume a stronger alignment than the storage actually provides for odd
// sizes like this one.
struct TensorFixedSizeEmbeddingHolder {
  float pad;
  TensorFixedSize<float, Sizes<5> > tensor;
};

static void test_embedded_odd_size() {
  EIGEN_ALIGN_MAX TensorFixedSizeEmbeddingHolder lhs;
  EIGEN_ALIGN_MAX TensorFixedSizeEmbeddingHolder rhs;

  for (int i = 0; i < 5; ++i) {
    rhs.tensor(i) = static_cast<float>(i + 1);
  }

  lhs.tensor = rhs.tensor + rhs.tensor;

  for (int i = 0; i < 5; ++i) {
    VERIFY_IS_APPROX(lhs.tensor(i), 2.0f * static_cast<float>(i + 1));
  }
}

// Regression test: `IsAligned` is consumed by evaluators as the legacy `Aligned` load mode,
// which promises `Aligned16`-byte alignment regardless of what `EIGEN_MAX_STATIC_ALIGN_BYTES`
// is actually set to.  A build configured with `0 < EIGEN_MAX_STATIC_ALIGN_BYTES < Aligned`
// (e.g. 8) still only 8-byte-aligns `TensorStorage`'s fixed array via `EIGEN_ALIGN_MAX`, so
// `IsAligned` must not claim aligned access is safe unless the configured static alignment
// actually reaches `Aligned` (16) bytes.
static_assert(TensorFixedSize<float, Sizes<5> >::IsAligned == (EIGEN_MAX_STATIC_ALIGN_BYTES >= 16),
              "IsAligned must match whether EIGEN_MAX_STATIC_ALIGN_BYTES actually provides Aligned16 storage");

// Regression test for issue #1616: unsigned index types must not trigger
// narrowing errors in the variadic accessors.
static void test_unsigned_indices() {
  TensorFixedSize<float, Sizes<2, 3> > tensor;
  tensor.setZero();

  tensor(1u, std::size_t(2)) = 5.0f;
  VERIFY_IS_EQUAL(tensor(1ul, 2u), 5.0f);
  tensor.coeffRef(0u, 1ul) = 7.0f;

  const TensorFixedSize<float, Sizes<2, 3> >& const_tensor = tensor;
  VERIFY_IS_EQUAL(const_tensor.coeff(0ul, 1u), 7.0f);
  VERIFY_IS_EQUAL(const_tensor(0u, std::size_t(1)), 7.0f);
}

// An index the tensor's index type cannot represent must assert rather than silently truncate.
static void test_narrowing_indices() {
  // Fixed width: on ILP32 targets shifting std::size_t by 40 is undefined, and GCC folds it to an in-range 0.
  const std::uint64_t too_large = std::uint64_t(1) << 40;

  TensorFixedSize<float, Sizes<2, 3>, ColMajor, int> tensor;
  tensor.setZero();
  VERIFY_RAISES_ASSERT(tensor(0, too_large) = 1.0f);
  VERIFY_RAISES_ASSERT(tensor.coeffRef(0, too_large) = 1.0f);

  const TensorFixedSize<float, Sizes<2, 3>, ColMajor, int>& const_tensor = tensor;
  VERIFY_RAISES_ASSERT(const_tensor(0, too_large));
  VERIFY_RAISES_ASSERT(const_tensor.coeff(0, too_large));
}

EIGEN_DECLARE_TEST(tensor_fixed_size) {
  CALL_SUBTEST(test_0d());
  CALL_SUBTEST(test_1d());
  CALL_SUBTEST(test_tensor_map());
  CALL_SUBTEST(test_2d());
  CALL_SUBTEST(test_3d());
  CALL_SUBTEST(test_array());
  CALL_SUBTEST(test_embedded_odd_size());
  CALL_SUBTEST(test_unsigned_indices());
  CALL_SUBTEST(test_narrowing_indices());
}
