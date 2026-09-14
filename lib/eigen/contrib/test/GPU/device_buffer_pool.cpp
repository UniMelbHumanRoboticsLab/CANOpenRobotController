// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Eigen Authors
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// Tests for internal::DeviceBufferPool: a released block is recycled only once
// the device has retired the work enqueued before the release. The pool is
// driven directly, since DeviceBuffer only routes through it on devices without
// memory pools.

#define EIGEN_USE_GPU
#include "main.h"
#include <contrib/Eigen/GPU>
#include <atomic>
#include <chrono>
#include <thread>

#include "./gpu_test_helpers.h"

using namespace Eigen;

namespace {

using Pool = gpu::internal::DeviceBufferPool<>;
constexpr size_t kBytes = Pool::kSmallBufferThreshold / 4;

void spin_until(const std::atomic<bool>& flag) {
  while (!flag.load(std::memory_order_acquire)) std::this_thread::yield();
}

// A host function parked on a blocking stream keeps everything ordered after
// it, including the pool's release events on the legacy default stream,
// pending until the gate opens.
struct StreamGate {
  std::atomic<bool> entered{false};
  std::atomic<bool> release{false};
};

void CUDART_CB wait_for_gate(void* data) {
  StreamGate* gate = static_cast<StreamGate*>(data);
  gate->entered.store(true, std::memory_order_release);
  spin_until(gate->release);
}

// An idle device recycles a released block on the next allocation that fits.
void test_idle_reuse() {
  Pool& pool = Pool::threadLocal();
  void* const p = pool.allocate(kBytes);
  VERIFY(p != nullptr);
  pool.deallocate(p, kBytes);
  EIGEN_CUDA_RUNTIME_CHECK(cudaDeviceSynchronize());
  void* const q = pool.allocate(kBytes);
  VERIFY_IS_EQUAL(q, p);
  pool.deallocate(q, kBytes);
}

// A block released while a blocking stream is still busy stays out of
// circulation until that stream drains; afterwards it is recycled again.
void test_reuse_waits_for_in_flight_work() {
  Pool& pool = Pool::threadLocal();
  EIGEN_CUDA_RUNTIME_CHECK(cudaDeviceSynchronize());
  cudaStream_t stream = nullptr;
  EIGEN_CUDA_RUNTIME_CHECK(cudaStreamCreate(&stream));

  void* const p = pool.allocate(kBytes);

  StreamGate gate;
  EIGEN_CUDA_RUNTIME_CHECK(cudaLaunchHostFunc(stream, wait_for_gate, &gate));
  std::thread release_thread([&gate]() {
    spin_until(gate.entered);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    gate.release.store(true, std::memory_order_release);
  });

  // Released while `stream` is parked: the release event cannot have completed.
  pool.deallocate(p, kBytes);
  void* const q = pool.allocate(kBytes);
  VERIFY(q != p);

  release_thread.join();
  EIGEN_CUDA_RUNTIME_CHECK(cudaDeviceSynchronize());

  // Both blocks are free and retired; the older release is recycled first.
  pool.deallocate(q, kBytes);
  void* const r = pool.allocate(kBytes);
  VERIFY_IS_EQUAL(r, p);
  pool.deallocate(r, kBytes);

  EIGEN_CUDA_RUNTIME_CHECK(cudaStreamDestroy(stream));
}

}  // namespace

EIGEN_DECLARE_TEST(gpu_device_buffer_pool) {
  gpu_test::require_cuda_device();
  CALL_SUBTEST_1(test_idle_reuse());
  CALL_SUBTEST_1(test_reuse_waits_for_in_flight_work());
}
