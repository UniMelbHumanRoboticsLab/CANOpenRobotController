// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

// A backend whose blocked kernels need a larger stack panel may raise Eigen's own default limit --
// see the SVE and RVV blocks in ConfigureVectorization.h -- but an explicit EIGEN_STACK_ALLOCATION_LIMIT
// is a caller policy on stack safety and must reach DenseStorage.h and Memory.h unchanged. Rewriting
// one upward turns OBJECT_ALLOCATED_ON_STACK_IS_TOO_BIG off for objects the caller wanted rejected and
// switches internal buffers from the heap back to alloca.
//
// 1 is below anything a backend could derive, so this never passes vacuously, and nothing here
// allocates a fixed-size Eigen object that the limit would then reject.
#define EIGEN_STACK_ALLOCATION_LIMIT 1
#include "main.h"

static_assert(EIGEN_STACK_ALLOCATION_LIMIT == 1, "an explicit EIGEN_STACK_ALLOCATION_LIMIT must survive");

// Memory.h reads the macro at each use rather than through a saved constant, so check the users too.
void check_stack_allocation_is_disabled() {
  VERIFY_IS_EQUAL(int(sizeof(double) * 4 <= EIGEN_STACK_ALLOCATION_LIMIT), 0);
  ei_declare_aligned_stack_constructed_variable(double, buffer, 4, 0);
  VERIFY(buffer != nullptr);
  for (Index i = 0; i < 4; ++i) buffer[i] = double(i);
  for (Index i = 0; i < 4; ++i) VERIFY_IS_EQUAL(buffer[i], double(i));
}

EIGEN_DECLARE_TEST(stack_allocation_limit) { CALL_SUBTEST(check_stack_allocation_is_disabled()); }
