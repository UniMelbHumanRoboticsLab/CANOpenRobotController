// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

// Compile- and host-time checks for the cuBLAS 64-bit dimension adapters.

#define EIGEN_USE_GPU
#include "main.h"
#include <contrib/Eigen/GPU>

using namespace Eigen;

using CublasXgemmFloat = cublasStatus_t (*)(cublasHandle_t, cublasOperation_t, cublasOperation_t, int64_t, int64_t,
                                            int64_t, const float*, const float*, int64_t, const float*, int64_t,
                                            const float*, float*, int64_t);

static_assert(
    std::is_same<decltype(static_cast<CublasXgemmFloat>(&gpu::internal::cublasXgemm)), CublasXgemmFloat>::value,
    "cublasXgemm dimensions must remain 64-bit");

#if defined(CUBLAS_VERSION) && CUBLAS_VERSION >= 120000
static_assert(std::is_same<decltype(gpu::internal::to_blas_dim(int64_t{})), int64_t>::value,
              "cuBLAS 12 and newer must preserve 64-bit dimensions");
#else
static_assert(std::is_same<decltype(gpu::internal::to_blas_dim(int64_t{})), int>::value,
              "pre-12 cuBLAS must narrow dimensions to int");
#endif

void test_cublas_64bit_dimension_contract() {
  constexpr int64_t kLargeDim = int64_t{1} << 32;
  VERIFY_IS_EQUAL(gpu::internal::blas1_size(Index{1} << 16, Index{1} << 16), kLargeDim);

#if defined(CUBLAS_VERSION) && CUBLAS_VERSION >= 120000
  VERIFY_IS_EQUAL(gpu::internal::to_blas_dim(kLargeDim), kLargeDim);
#endif

  CublasXgemmFloat volatile gemm = static_cast<CublasXgemmFloat>(&gpu::internal::cublasXgemm);
  VERIFY(gemm != nullptr);
}

EIGEN_DECLARE_TEST(gpu_cublas_64bit) { CALL_SUBTEST(test_cublas_64bit_dimension_contract()); }
