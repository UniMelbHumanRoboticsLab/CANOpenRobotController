// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// Helpers shared across GPU library tests:
//   * runtime probes that exit(77) (CI skip) when a library is unavailable
//   * a small make_test_value() that constructs a Scalar with an imaginary
//     component for complex types so the complex code paths are genuinely
//     exercised — without this, Scalar(real_value) silently zeros the imag.

#ifndef EIGEN_UNSUPPORTED_TEST_GPU_TEST_HELPERS_H
#define EIGEN_UNSUPPORTED_TEST_GPU_TEST_HELPERS_H

#include <Eigen/Core>
#include <cstdlib>
#include <iostream>
#include <type_traits>

namespace gpu_test {

template <typename Scalar, typename RealScalar>
inline std::enable_if_t<Eigen::NumTraits<Scalar>::IsComplex, Scalar> make_test_value(RealScalar re, RealScalar im) {
  return Scalar(re, im);
}
template <typename Scalar, typename RealScalar>
inline std::enable_if_t<!Eigen::NumTraits<Scalar>::IsComplex, Scalar> make_test_value(RealScalar re,
                                                                                      RealScalar /*im*/) {
  return Scalar(re);
}

#ifdef CUDART_VERSION
// The CUDA runtime loads without a driver, so a GPU test binary starts happily on
// a machine with no device and only fails at its first allocation -- as an abort
// out of EIGEN_CUDA_RUNTIME_CHECK, not a skip. Probe the device count first so
// those runs report as skipped, the way the per-library probes below do.
inline void require_cuda_device() {
  int count = 0;
  const cudaError_t status = cudaGetDeviceCount(&count);
  if (status != cudaSuccess || count == 0) {
    std::cout << "SKIP: GPU tests require a CUDA device. cudaGetDeviceCount reported " << cudaGetErrorString(status)
              << " with " << count << " device(s)." << std::endl;
    std::exit(77);
  }
}
#endif

#ifdef CUDSS_VERSION
inline void require_cudss_context() {
  cudssHandle_t handle = nullptr;
  const cudssStatus_t status = cudssCreate(&handle);
  if (status != CUDSS_STATUS_SUCCESS) {
    std::cout << "SKIP: cuDSS tests require an initialized cuDSS context. cudssCreate failed with status "
              << static_cast<int>(status) << std::endl;
    std::exit(77);
  }
  EIGEN_CUDSS_CHECK(cudssDestroy(handle));
}
#endif

#ifdef CUSPARSE_VERSION
inline void require_cusparse_context() {
  cusparseHandle_t handle = nullptr;
  const cusparseStatus_t status = cusparseCreate(&handle);
  if (status != CUSPARSE_STATUS_SUCCESS) {
    std::cout << "SKIP: cuSPARSE tests require an initialized cuSPARSE context. cusparseCreate failed with status "
              << static_cast<int>(status) << std::endl;
    std::exit(77);
  }
  EIGEN_CUSPARSE_CHECK(cusparseDestroy(handle));
}
#endif

#ifdef CUFFT_VERSION
inline void require_cufft_context() {
  cufftHandle plan = 0;
  // cufftCreate allocates a plan handle without configuring it; succeeds only
  // when the cuFFT runtime is loadable.
  const cufftResult status = cufftCreate(&plan);
  if (status != CUFFT_SUCCESS) {
    std::cout << "SKIP: cuFFT tests require a working cuFFT runtime. cufftCreate failed with status "
              << static_cast<int>(status) << std::endl;
    std::exit(77);
  }
  EIGEN_CUFFT_CHECK(cufftDestroy(plan));
}
#endif

}  // namespace gpu_test

#endif  // EIGEN_UNSUPPORTED_TEST_GPU_TEST_HELPERS_H
