// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

// A failed runtime call or kernel launch inside GpuDevice has to reach EIGEN_GPU_RUNTIME_CHECK, in a release build
// as much as in a debug one (tensor_gpu_runtime_check_ndebug.cu compiles this file with EIGEN_NO_DEBUG). The macro
// is overridden here to record the failure instead of stopping the process.

#define EIGEN_TEST_NO_LONGDOUBLE
#define EIGEN_TEST_NO_COMPLEX

#define EIGEN_USE_GPU

#include <string>

struct RecordedGpuError {
  int code = 0;
  std::string expression;
  std::string file;
  int line = 0;
};
static RecordedGpuError g_last_gpu_error;
static int g_num_gpu_errors = 0;

// cudaSuccess and hipSuccess are both 0.
template <typename Error>
__host__ void record_gpu_runtime_error(Error error, const char* expression, const char* file, int line) {
  if (static_cast<int>(error) == 0) return;
  ++g_num_gpu_errors;
  g_last_gpu_error.code = static_cast<int>(error);
  g_last_gpu_error.expression = expression;
  g_last_gpu_error.file = file;
  g_last_gpu_error.line = line;
}
#define EIGEN_GPU_RUNTIME_CHECK(expr) ::record_gpu_runtime_error((expr), #expr, __FILE__, __LINE__)

#include "main.h"
#include <contrib/Eigen/Tensor>

#if defined(EIGEN_USE_HIP)
static const int kInvalidConfiguration = static_cast<int>(hipErrorInvalidConfiguration);
static const int kInvalidValue = static_cast<int>(hipErrorInvalidValue);
#else
static const int kInvalidConfiguration = static_cast<int>(cudaErrorInvalidConfiguration);
static const int kInvalidValue = static_cast<int>(cudaErrorInvalidValue);
#endif

__global__ void write_one_kernel(int* out) {
  if (blockIdx.x == 0 && threadIdx.x == 0) *out = 1;
}

static bool ends_with(const std::string& s, const std::string& suffix) {
  return s.size() >= suffix.size() && s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// A valid launch through LAUNCH_GPU_KERNEL runs and records nothing; one with more threads per block than any
// device allows is refused at enqueue time, and the refusal is reported from gpu_launch's own check.
static void test_launch_check() {
  Eigen::GpuStreamDevice stream;
  Eigen::GpuDevice device(&stream);
  int* d_out = static_cast<int*>(device.allocate(sizeof(int)));
  device.memset(d_out, 0, sizeof(int));
  g_num_gpu_errors = 0;

  LAUNCH_GPU_KERNEL(write_one_kernel, 1, 32, 0, device, d_out);
  int out = 0;
  device.memcpyDeviceToHost(&out, d_out, sizeof(int));
  device.synchronize();
  VERIFY_IS_EQUAL(out, 1);
  VERIFY_IS_EQUAL(g_num_gpu_errors, 0);

  const int too_many_threads = 2 * device.maxGpuThreadsPerBlock();
  LAUNCH_GPU_KERNEL(write_one_kernel, 1, too_many_threads, 0, device, d_out);
  VERIFY_IS_EQUAL(g_num_gpu_errors, 1);
  // Which of the two codes an oversize block gets depends on the toolkit: CUDA 13.3 reports invalid value.
  VERIFY(g_last_gpu_error.code == kInvalidConfiguration || g_last_gpu_error.code == kInvalidValue);
  VERIFY_IS_EQUAL(g_last_gpu_error.expression, std::string("gpuGetLastError()"));
  VERIFY(ends_with(g_last_gpu_error.file, "GpuRuntime.h"));

  device.deallocate(d_out);
  VERIFY_IS_EQUAL(g_num_gpu_errors, 1);
}

// A runtime call that fails inside GpuDevice is reported with the call's own text and location.
static void test_runtime_call_check() {
  Eigen::GpuStreamDevice stream;
  Eigen::GpuDevice device(&stream);
  g_num_gpu_errors = 0;

  int host_dst = 0;
  // HIP can accept a host source here; a null source with a nonzero size is invalid on both runtimes.
  device.memcpyDeviceToHost(&host_dst, nullptr, sizeof(int));
  VERIFY_IS_EQUAL(g_num_gpu_errors, 1);
  VERIFY_IS_EQUAL(g_last_gpu_error.code, kInvalidValue);
  VERIFY(g_last_gpu_error.expression.find("gpuMemcpyAsync") == 0);
  VERIFY(ends_with(g_last_gpu_error.file, "TensorDeviceGpu.h"));
  VERIFY(g_last_gpu_error.line > 0);
}

EIGEN_DECLARE_TEST(tensor_gpu_runtime_check) {
  CALL_SUBTEST(test_launch_check());
  CALL_SUBTEST(test_runtime_call_check());
}
