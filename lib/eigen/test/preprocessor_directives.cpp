// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

// Checks that the configuration macros documented in
// doc/TopicPreprocessorDirectives.dox are still honored by the declarations
// that spell them.
//
// A mechanical rewrite of such a declaration prints the *resolved* type, so
// `typedef EIGEN_DEFAULT_DENSE_INDEX_TYPE Index;` silently becomes
// `using Index = std::ptrdiff_t;`. Every default-configured build stays green;
// only a translation unit that overrides the macro notices. Each part below
// therefore compiles with a different configuration.

#if defined(EIGEN_TEST_PART_2)
#define EIGEN_DEFAULT_DENSE_INDEX_TYPE int
#elif defined(EIGEN_TEST_PART_3)
#define EIGEN_64BIT_BLAS
#elif defined(EIGEN_TEST_PART_4)
#define EIGEN_ALIGN_TO_AVOID_FALSE_SHARING EIGEN_ALIGN_TO_BOUNDARY(256)
#endif

#include "main.h"

// The index type must follow EIGEN_DEFAULT_DENSE_INDEX_TYPE in every configuration.
static_assert(std::is_same<Eigen::Index, EIGEN_DEFAULT_DENSE_INDEX_TYPE>::value,
              "Eigen::Index no longer follows EIGEN_DEFAULT_DENSE_INDEX_TYPE");
static_assert(std::is_same<Eigen::DenseIndex, EIGEN_DEFAULT_DENSE_INDEX_TYPE>::value,
              "Eigen::DenseIndex no longer follows EIGEN_DEFAULT_DENSE_INDEX_TYPE");

// Outside of MKL, the BLAS integer type must follow EIGEN_BLAS_INT, which the
// external BLAS prototypes in Eigen/src/misc/blas.h are declared with.
#if !defined(EIGEN_USE_MKL)
static_assert(std::is_same<Eigen::BlasIndex, EIGEN_BLAS_INT>::value,
              "Eigen::BlasIndex no longer follows EIGEN_BLAS_INT");
#endif

// Buffers that Eigen aligns itself must be mapped with EIGEN_DEFAULT_ALIGN_BYTES,
// not with whatever that happened to expand to on the machine of the last edit.
static_assert(int(Eigen::internal::traits<
                  Eigen::internal::local_nested_eval_wrapper<Eigen::MatrixXd, 2, true>::ObjectType>::Alignment) ==
                  EIGEN_DEFAULT_ALIGN_BYTES,
              "local_nested_eval_wrapper no longer follows EIGEN_DEFAULT_ALIGN_BYTES");

// EIGEN_ALIGN_TO_AVOID_FALSE_SHARING must honor pre-existing definitions and stay pinned to 128 by default.
struct AvoidFalseSharingAligned {
  EIGEN_ALIGN_TO_AVOID_FALSE_SHARING char c;
};

#if defined(EIGEN_TEST_PART_4)
static_assert(alignof(AvoidFalseSharingAligned) == 256,
              "Pre-existing EIGEN_ALIGN_TO_AVOID_FALSE_SHARING definition was not preserved");
#else
static_assert(alignof(AvoidFalseSharingAligned) == 128,
              "EIGEN_ALIGN_TO_AVOID_FALSE_SHARING should be pinned to 128 bytes by default");
#endif

// Per-part checks that the override actually took effect.
#if defined(EIGEN_TEST_PART_2)
static_assert(std::is_same<Eigen::Index, int>::value, "EIGEN_DEFAULT_DENSE_INDEX_TYPE was ignored");
#endif
#if defined(EIGEN_TEST_PART_3) && !defined(EIGEN_USE_MKL)
static_assert(sizeof(Eigen::BlasIndex) == 8, "EIGEN_64BIT_BLAS was ignored");
#endif

void check_index_type() {
  VERIFY_IS_EQUAL(int(sizeof(Eigen::Index)), int(sizeof(EIGEN_DEFAULT_DENSE_INDEX_TYPE)));
  // The configured index type is the one the containers actually index with.
  MatrixXd m(3, 4);
  VERIFY_IS_EQUAL(m.rows(), Eigen::Index(3));
  VERIFY_IS_EQUAL(m.cols(), Eigen::Index(4));
}

void check_blas_index() {
#if !defined(EIGEN_USE_MKL)
  VERIFY_IS_EQUAL(int(sizeof(Eigen::BlasIndex)), int(sizeof(EIGEN_BLAS_INT)));
#endif
}

void check_false_sharing_alignment() {
#if defined(EIGEN_TEST_PART_4)
  VERIFY_IS_EQUAL(std::size_t(alignof(AvoidFalseSharingAligned)), std::size_t(256));
#else
  VERIFY_IS_EQUAL(std::size_t(alignof(AvoidFalseSharingAligned)), std::size_t(128));
#endif
}

EIGEN_DECLARE_TEST(preprocessor_directives) {
  CALL_SUBTEST_1(check_index_type());
  CALL_SUBTEST_1(check_false_sharing_alignment());
  CALL_SUBTEST_2(check_index_type());
  CALL_SUBTEST_3(check_blas_index());
  CALL_SUBTEST_4(check_false_sharing_alignment());
}
