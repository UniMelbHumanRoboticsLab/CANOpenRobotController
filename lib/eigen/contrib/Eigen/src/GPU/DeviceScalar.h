// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// Device-resident scalar for deferred host synchronization.
//
// Reductions (dot, nrm2) write their result straight to device memory under
// CUBLAS_POINTER_MODE_DEVICE, so no host sync happens until the value is read.
// Conversion to Scalar is that read. Because the first conversion flushes the
// stream, later conversions in the same expression only download: a CG iteration
// costs one sync rather than three.

#ifndef EIGEN_GPU_DEVICE_SCALAR_H
#define EIGEN_GPU_DEVICE_SCALAR_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

#include "./GpuSupport.h"
#include "./DeviceScalarOps.h"

namespace Eigen {
namespace gpu {

/** \brief RAII wrapper for a scalar in GPU device memory. */
template <typename Scalar_>
class DeviceScalar {
 public:
  using Scalar = Scalar_;

  /** Allocate an uninitialized device scalar. Contents are undefined until
   * written, e.g. by cuBLAS dot/nrm2 under POINTER_MODE_DEVICE. */
  explicit DeviceScalar(cudaStream_t stream = nullptr) : d_val_(sizeof(Scalar)), stream_(stream) {}

  DeviceScalar(Scalar host_val, cudaStream_t stream) : d_val_(sizeof(Scalar)), stream_(stream) {
    EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpyAsync(d_val_.get(), &host_val, sizeof(Scalar), cudaMemcpyHostToDevice, stream_));
  }

  DeviceScalar(DeviceScalar&& o) noexcept : d_val_(std::move(o.d_val_)), stream_(o.stream_) { o.stream_ = nullptr; }

  DeviceScalar& operator=(DeviceScalar&& o) noexcept {
    if (this != &o) {
      d_val_ = std::move(o.d_val_);
      stream_ = o.stream_;
      o.stream_ = nullptr;
    }
    return *this;
  }

  DeviceScalar(const DeviceScalar&) = delete;
  DeviceScalar& operator=(const DeviceScalar&) = delete;

  /** Download from device, synchronizing the stream. */
  Scalar get() const {
    Scalar result;
    EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpyAsync(&result, d_val_.get(), sizeof(Scalar), cudaMemcpyDeviceToHost, stream_));
    EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(stream_));
    return result;
  }

  /** Implicit conversion, enabling `Scalar alpha = deviceScalar` and
   * `if (deviceScalar < threshold)`. Triggers a sync. */
  operator Scalar() const { return get(); }

  Scalar* devicePtr() { return static_cast<Scalar*>(d_val_.get()); }
  const Scalar* devicePtr() const { return static_cast<const Scalar*>(d_val_.get()); }
  cudaStream_t stream() const { return stream_; }

  // The arithmetic below keeps results on device via the NPP helpers in
  // DeviceScalarOps.h, and covers real types only; complex division falls back to
  // the implicit conversion and its host sync. Unlike DeviceMatrix, DeviceScalar
  // tracks no cross-stream readiness, so all operands must share one stream.

  friend DeviceScalar operator/(const DeviceScalar& a, const DeviceScalar& b) {
    eigen_assert(a.stream_ == b.stream_ && "DeviceScalar operator/: operands must share the same stream");
    DeviceScalar result(a.stream_);
    gpu::internal::device_scalar_div(a.devicePtr(), b.devicePtr(), result.devicePtr(), a.stream_);
    return result;
  }

  friend DeviceScalar operator/(Scalar a, const DeviceScalar& b) {
    DeviceScalar d_a(a, b.stream_);
    return d_a / b;
  }

  friend DeviceScalar operator/(const DeviceScalar& a, Scalar b) {
    DeviceScalar d_b(b, a.stream_);
    return a / d_b;
  }

  DeviceScalar operator-() const {
    DeviceScalar result(stream_);
    gpu::internal::device_scalar_neg(devicePtr(), result.devicePtr(), stream_);
    return result;
  }

 private:
  internal::DeviceBuffer d_val_;
  cudaStream_t stream_ = nullptr;
};

}  // namespace gpu
}  // namespace Eigen

#endif  // EIGEN_GPU_DEVICE_SCALAR_H
