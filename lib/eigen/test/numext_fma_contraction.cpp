// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include "main.h"
#include "twoprod_helpers.h"

#if defined(EIGEN_VECTORIZE_FMA) || !defined(FP_FAST_FMA) || !defined(FP_FAST_FMAF)
#error This test requires fast scalar FMA without EIGEN_VECTORIZE_FMA.
#endif

EIGEN_DECLARE_TEST(numext_fma_contraction) {
  CALL_SUBTEST(check_twoprod_contraction<float>());
  CALL_SUBTEST(check_twoprod_contraction<double>());
}
