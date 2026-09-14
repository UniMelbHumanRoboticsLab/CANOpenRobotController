// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// Device-resident scalar and element-wise operations built on the NPP signal
// primitives (CUDA::npps), which keeps the module header-only: no custom CUDA
// kernels to compile.

#ifndef EIGEN_GPU_DEVICE_SCALAR_OPS_H
#define EIGEN_GPU_DEVICE_SCALAR_OPS_H

#include <cuda_runtime.h>
#include <npps_arithmetic_and_logical_operations.h>

#include "./GpuSupport.h"

namespace Eigen {
namespace gpu {
namespace internal {

// NPP requires nCudaDeviceId and the device attributes to match the device that
// owns `stream`. Re-querying per call is cheap next to the NPP launch itself and
// keeps multi-device and borrowed-stream callers correct.
inline NppStreamContext make_npp_stream_ctx(cudaStream_t stream) {
  NppStreamContext ctx = {};
  ctx.hStream = stream;
#if CUDART_VERSION >= 12080
  // cudaStreamGetDevice reports the stream's owning device irrespective of the
  // calling thread's current device.
  EIGEN_CUDA_RUNTIME_CHECK(cudaStreamGetDevice(stream, &ctx.nCudaDeviceId));
#else
  // Without cudaStreamGetDevice (pre-CUDA 12.8), a caller borrowing a stream from
  // another device must cudaSetDevice() first.
  EIGEN_CUDA_RUNTIME_CHECK(cudaGetDevice(&ctx.nCudaDeviceId));
#endif
  EIGEN_CUDA_RUNTIME_CHECK(cudaDeviceGetAttribute(&ctx.nCudaDevAttrComputeCapabilityMajor,
                                                  cudaDevAttrComputeCapabilityMajor, ctx.nCudaDeviceId));
  EIGEN_CUDA_RUNTIME_CHECK(cudaDeviceGetAttribute(&ctx.nCudaDevAttrComputeCapabilityMinor,
                                                  cudaDevAttrComputeCapabilityMinor, ctx.nCudaDeviceId));
  EIGEN_CUDA_RUNTIME_CHECK(
      cudaDeviceGetAttribute(&ctx.nMultiProcessorCount, cudaDevAttrMultiProcessorCount, ctx.nCudaDeviceId));
  EIGEN_CUDA_RUNTIME_CHECK(cudaDeviceGetAttribute(&ctx.nMaxThreadsPerMultiProcessor,
                                                  cudaDevAttrMaxThreadsPerMultiProcessor, ctx.nCudaDeviceId));
  EIGEN_CUDA_RUNTIME_CHECK(
      cudaDeviceGetAttribute(&ctx.nMaxThreadsPerBlock, cudaDevAttrMaxThreadsPerBlock, ctx.nCudaDeviceId));
  int shared_mem_per_block = 0;
  EIGEN_CUDA_RUNTIME_CHECK(
      cudaDeviceGetAttribute(&shared_mem_per_block, cudaDevAttrMaxSharedMemoryPerBlock, ctx.nCudaDeviceId));
  ctx.nSharedMemPerBlock = static_cast<size_t>(shared_mem_per_block);
  EIGEN_CUDA_RUNTIME_CHECK(cudaStreamGetFlags(stream, &ctx.nStreamFlags));
  return ctx;
}

// c = a / b. The operands are swapped because NPP computes
// pDst[i] = pSrc2[i] / pSrc1[i].
inline void device_scalar_div(const float* a, const float* b, float* c, cudaStream_t stream) {
  NppStreamContext npp_ctx = make_npp_stream_ctx(stream);
  nppsDiv_32f_Ctx(b, a, c, 1, npp_ctx);
}

inline void device_scalar_div(const double* a, const double* b, double* c, cudaStream_t stream) {
  NppStreamContext npp_ctx = make_npp_stream_ctx(stream);
  nppsDiv_64f_Ctx(b, a, c, 1, npp_ctx);
}

// c = -a.
inline void device_scalar_neg(const float* a, float* c, cudaStream_t stream) {
  NppStreamContext npp_ctx = make_npp_stream_ctx(stream);
  nppsMulC_32f_Ctx(a, -1.0f, c, 1, npp_ctx);
}

inline void device_scalar_neg(const double* a, double* c, cudaStream_t stream) {
  NppStreamContext npp_ctx = make_npp_stream_ctx(stream);
  nppsMulC_64f_Ctx(a, -1.0, c, 1, npp_ctx);
}

// c[i] = a[i] * b[i].
inline void device_cwiseProduct(const float* a, const float* b, float* c, int n, cudaStream_t stream) {
  NppStreamContext npp_ctx = make_npp_stream_ctx(stream);
  nppsMul_32f_Ctx(a, b, c, static_cast<size_t>(n), npp_ctx);
}

inline void device_cwiseProduct(const double* a, const double* b, double* c, int n, cudaStream_t stream) {
  NppStreamContext npp_ctx = make_npp_stream_ctx(stream);
  nppsMul_64f_Ctx(a, b, c, static_cast<size_t>(n), npp_ctx);
}

// c[i] = a[i] / b[i], with operands swapped as in device_scalar_div.
inline void device_cwiseQuotient(const float* a, const float* b, float* c, int n, cudaStream_t stream) {
  NppStreamContext npp_ctx = make_npp_stream_ctx(stream);
  nppsDiv_32f_Ctx(b, a, c, static_cast<size_t>(n), npp_ctx);
}

inline void device_cwiseQuotient(const double* a, const double* b, double* c, int n, cudaStream_t stream) {
  NppStreamContext npp_ctx = make_npp_stream_ctx(stream);
  nppsDiv_64f_Ctx(b, a, c, static_cast<size_t>(n), npp_ctx);
}

}  // namespace internal
}  // namespace gpu
}  // namespace Eigen

#endif  // EIGEN_GPU_DEVICE_SCALAR_OPS_H
