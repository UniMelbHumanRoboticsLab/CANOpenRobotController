// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

// Host-side helpers shared by the CUDA and HIP paths of the Tensor GPU device: reporting of failed runtime calls
// and checked kernel launches. Written against the gpu* aliases of GpuHipCudaDefines.inc, so it is only meaningful
// inside that alias window (TensorDeviceGpu.h includes it there) and neither includes nor undefines the .inc files
// itself; outside the window it is inert, so a standalone parse sees nothing. The macros below may expand after the
// window closes, which is why each of them only calls a function defined here.
#if defined(EIGEN_CORE_GPU_HIP_CUDA_DEFINES_H) && !defined(EIGEN_CORE_UTIL_GPU_RUNTIME_H)
#define EIGEN_CORE_UTIL_GPU_RUNTIME_H

#include <cstdio>
#include <cstdlib>
#include <utility>

namespace Eigen {
namespace internal {

// Prints `file:line: call: error name: description` to stderr and stops: std::abort() where assertions are
// compiled out, a failed eigen_assert otherwise, so a debugger or the test harness sees the usual assertion.
inline void gpu_runtime_check_failed(gpuError_t error, const char* expression, const char* file, int line) {
  std::fprintf(stderr, "%s:%d: %s: %s: %s\n", file, line, expression, gpuGetErrorName(error), gpuGetErrorString(error));
#if defined(EIGEN_NO_DEBUG)
  std::abort();
#else
  eigen_assert(false && "GPU runtime call failed");
#endif
}

inline void gpu_runtime_check(gpuError_t error, const char* expression, const char* file, int line) {
  if (error != gpuSuccess) gpu_runtime_check_failed(error, expression, file, line);
}

}  // namespace internal
}  // namespace Eigen

// EIGEN_GPU_RUNTIME_CHECK(call) evaluates a runtime-API call once and reports a failure through
// gpu_runtime_check_failed(). Define the macro before including the module to route failures elsewhere; define
// EIGEN_GPU_NO_RUNTIME_CHECKS to evaluate the call and ignore its result, which is what release builds did before.
#if !defined(EIGEN_GPU_RUNTIME_CHECK)
#if defined(EIGEN_GPU_NO_RUNTIME_CHECKS)
#define EIGEN_GPU_RUNTIME_CHECK(expr) \
  do {                                \
    (void)(expr);                     \
  } while (0)
#else
#define EIGEN_GPU_RUNTIME_CHECK(expr)                                        \
  do {                                                                       \
    ::Eigen::internal::gpu_runtime_check((expr), #expr, __FILE__, __LINE__); \
  } while (0)
#endif
#endif

#if defined(EIGEN_GPUCC)
namespace Eigen {
namespace internal {

// Enqueues `kernel` on `stream` and reports a launch failure through EIGEN_GPU_RUNTIME_CHECK. hip-clang accepts the
// <<<>>> syntax, so CUDA and HIP share one definition; the kernel is passed as the function pointer the occupancy
// API takes as well. With EIGEN_GPU_SYNC_LAUNCHES defined every launch is followed by a checked synchronization of
// its stream, so an execution failure is reported at the launch that caused it rather than at a later call.
template <typename... KernelArgs, typename... Args>
void gpu_launch(void (*kernel)(KernelArgs...), dim3 grid, dim3 block, size_t shared_mem, gpuStream_t stream,
                Args&&... args) {
  // clang-format off
  kernel<<<grid, block, shared_mem, stream>>>(std::forward<Args>(args)...);
  // clang-format on
  EIGEN_GPU_RUNTIME_CHECK(gpuGetLastError());
#if defined(EIGEN_GPU_SYNC_LAUNCHES)
  EIGEN_GPU_RUNTIME_CHECK(gpuStreamSynchronize(stream));
#endif
}

}  // namespace internal
}  // namespace Eigen
#endif  // EIGEN_GPUCC

#endif  // EIGEN_CORE_UTIL_GPU_RUNTIME_H
