// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// Generic CUDA runtime support shared by all GPU library integrations.
// Depends only on <cuda_runtime.h>; no NVIDIA library headers.

#ifndef EIGEN_GPU_SUPPORT_H
#define EIGEN_GPU_SUPPORT_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

#include <cuda_runtime.h>
#include <vector>

#include <limits>
#include <memory>
#include <type_traits>

namespace Eigen {
namespace gpu {
// Transpose/adjoint flag for BLAS-, solver-, and sparse-style calls. Each
// library's support header maps it to its own enum (cublasOperation_t,
// cusparseOperation_t, ...) via a to_<lib>_op() helper.
enum class GpuOp { NoTrans, Trans, ConjTrans };

namespace internal {
// Aborts via eigen_assert on failure, and eigen_assert throws where it is so
// configured, so this must not be used in a destructor or any other noexcept
// function: there the throw would call std::terminate.
#define EIGEN_CUDA_RUNTIME_CHECK(expr)                             \
  do {                                                             \
    cudaError_t _e = (expr);                                       \
    eigen_assert(_e == cudaSuccess && "CUDA runtime call failed"); \
  } while (0)

// cuBLAS and the legacy cuSOLVER APIs take dimensions and leading dimensions as
// 32-bit `int`, while Eigen's Index is 64-bit by default and GPU allocations can
// exceed INT_MAX in one dimension. Narrow through this helper at every such call
// site so an out-of-range value asserts instead of silently overflowing.
inline int to_blas_int(int64_t v) {
  eigen_assert(v >= 0 && v <= static_cast<int64_t>((std::numeric_limits<int>::max)()) &&
               "dimension exceeds the int range supported by cuBLAS / cuSOLVER");
  return static_cast<int>(v);
}

// cudaMallocAsync / cudaFreeAsync (CUDA 11.2+) allocate from a stream-ordered
// memory pool: both are cheap enqueues instead of the device-wide
// synchronization performed by cudaMalloc / cudaFree. All module allocations
// go through device_malloc / device_free on the *legacy default stream*:
// legacy-stream ordering guarantees that work enqueued later on any blocking
// stream observes the allocation, and that a free waits for all previously
// enqueued work on blocking streams — the same lifetime guarantees callers
// got from cudaMalloc / cudaFree, minus the host stalls.
//
// Caveat: streams created with cudaStreamNonBlocking do not synchronize with
// the legacy stream. When borrowing such a stream (gpu::Context(stream)),
// define EIGEN_GPU_NO_STREAM_ORDERED_ALLOC to fall back to cudaMalloc/cudaFree.
//
// Support is detected once per process, from the device current at first use.

inline bool device_supports_memory_pools() {
#ifdef EIGEN_GPU_NO_STREAM_ORDERED_ALLOC
  return false;
#else
  static const bool supported = [] {
    int device = 0;
    if (cudaGetDevice(&device) != cudaSuccess) return false;
    int v = 0;
    if (cudaDeviceGetAttribute(&v, cudaDevAttrMemoryPoolsSupported, device) != cudaSuccess) return false;
    if (v == 0) return false;
    // Keep freed memory in the pool instead of trimming at every stream
    // synchronize — repeated alloc/free cycles (temporaries in loops) then
    // recycle at user-space speed.
    cudaMemPool_t pool = nullptr;
    if (cudaDeviceGetDefaultMemPool(&pool, device) == cudaSuccess) {
      // The attribute value type is cuuint64_t; use a same-size stand-in to
      // avoid requiring the driver-API header.
      unsigned long long threshold = ~0ULL;
      (void)cudaMemPoolSetAttribute(pool, cudaMemPoolAttrReleaseThreshold, &threshold);
    }
    return true;
  }();
  return supported;
#endif
}

inline void* device_malloc(size_t bytes) {
  void* p = nullptr;
  if (device_supports_memory_pools()) {
    EIGEN_CUDA_RUNTIME_CHECK(cudaMallocAsync(&p, bytes, /*legacy default stream*/ nullptr));
  } else {
    EIGEN_CUDA_RUNTIME_CHECK(cudaMalloc(&p, bytes));
  }
  return p;
}

inline void device_free(void* p) noexcept {
  if (!p) return;
  if (device_supports_memory_pools()) {
    (void)cudaFreeAsync(p, /*legacy default stream*/ nullptr);
  } else {
    (void)cudaFree(p);
  }
}

struct CudaFreeDeleter {
  // When `borrow == true`, the unique_ptr does not free the pointer. Used by
  // DeviceMatrix::view() to wrap a non-owning device pointer with the same
  // smart-pointer machinery as owning storage, without changing the type.
  bool borrow = false;
  void operator()(void* p) const noexcept {
    if (p && !borrow) device_free(p);
  }
};

struct CudaFreeHostDeleter {
  void operator()(void* p) const noexcept {
    if (p) (void)cudaFreeHost(p);
  }
};

// RAII CUDA stream; the ownership flag supports borrowed, caller-owned streams.
struct CudaStreamDeleter {
  bool owns = true;
  void operator()(cudaStream_t s) const noexcept {
    if (owns && s) (void)cudaStreamDestroy(s);
  }
};
using UniqueStream = std::unique_ptr<std::remove_pointer_t<cudaStream_t>, CudaStreamDeleter>;

// Recycles allocations up to kSmallBufferThreshold bytes (e.g. DeviceScalar) to
// avoid cudaMalloc/cudaFree overhead on devices without memory pools; where
// cudaMallocAsync exists it is both stream-ordered and cheaper than this pool's
// release event, so DeviceBuffer bypasses the pool there (bench_overhead:
// CudaMallocAsyncFree vs PoolAllocFree). Larger allocations always bypass it.
// Invariant: a block is recycled only after the device has retired every
// operation enqueued before its release on any blocking stream: deallocate()
// records an event on the legacy default stream (the ordering device_free
// relies on), the free list stays in release order, and events on one stream
// retire in order, so allocate() scans until the first pending entry.
template <size_t SmallBufferThreshold = 256, size_t MaxPoolSize = 64>
struct DeviceBufferPool {
  static constexpr size_t kSmallBufferThreshold = SmallBufferThreshold;
  static constexpr size_t kMaxPoolSize = MaxPoolSize;

  struct Entry {
    void* ptr;
    size_t bytes;
    cudaEvent_t release_event;
  };

  // Lifetime marker for the thread-local pool. thread_local destruction runs
  // in reverse construction order, so a long-lived object holding pooled
  // buffers (e.g. the thread-local gpu::Context, or a static) can be
  // destroyed *after* the pool. The marker is trivially destructible — it
  // stays readable during TLS teardown — letting the deleter fall back to a
  // direct device_free once the pool is gone instead of touching a destroyed
  // vector.
  enum class State : signed char { kNotConstructed = 0, kAlive = 1, kDestroyed = 2 };

  static State& threadState() {
    thread_local State state = State::kNotConstructed;
    return state;
  }

  DeviceBufferPool() { threadState() = State::kAlive; }

  ~DeviceBufferPool() {
    for (const Entry& entry : free_list_) freeBlock(entry.ptr, entry.release_event);
    for (cudaEvent_t event : spare_events_) (void)cudaEventDestroy(event);
    threadState() = State::kDestroyed;
  }

  // First fit among the retired blocks, oldest release first; the first pending
  // entry ends the scan because every later one is pending too.
  void* allocate(size_t bytes) {
    for (auto it = free_list_.begin(); it != free_list_.end(); ++it) {
      if (cudaEventQuery(it->release_event) != cudaSuccess) break;
      if (it->bytes >= bytes) {
        void* p = it->ptr;
        spare_events_.push_back(it->release_event);
        free_list_.erase(it);
        return p;
      }
    }
    return device_malloc(bytes);
  }

  // Called from a noexcept deleter: every failure falls back to device_free.
  void deallocate(void* p, size_t bytes) noexcept {
    if (free_list_.size() >= kMaxPoolSize) {
      device_free(p);
      return;
    }
    cudaEvent_t release_event = acquireEvent();
    if (release_event == nullptr) {
      device_free(p);
      return;
    }
    if (cudaEventRecord(release_event, /*legacy default stream*/ nullptr) != cudaSuccess) {
      freeBlock(p, release_event);
      return;
    }
    free_list_.push_back({p, bytes, release_event});
  }

  static DeviceBufferPool& threadLocal() {
    thread_local DeviceBufferPool pool;
    return pool;
  }

 private:
  // Returns a spare event, or a newly created one; nullptr if creation fails.
  cudaEvent_t acquireEvent() noexcept {
    if (!spare_events_.empty()) {
      cudaEvent_t event = spare_events_.back();
      spare_events_.pop_back();
      return event;
    }
    cudaEvent_t event = nullptr;
    if (cudaEventCreateWithFlags(&event, cudaEventDisableTiming) != cudaSuccess) return nullptr;
    return event;
  }

  // Gives up a block and the event tracking its release. Destroying a pending
  // event is non-blocking; the runtime defers it until the event completes.
  static void freeBlock(void* p, cudaEvent_t release_event) noexcept {
    (void)cudaEventDestroy(release_event);
    device_free(p);
  }

  std::vector<Entry> free_list_;
  // Events of recycled entries; free_list_.size() + spare_events_.size() <= kMaxPoolSize.
  std::vector<cudaEvent_t> spare_events_;
};

// Stateful deleter that returns pooled buffers to the thread-local pool and
// device_free's the rest. size==0 means "always device_free" (adopted pointers
// and allocations that went straight to device_malloc).
struct PooledCudaFreeDeleter {
  size_t size = 0;

  void operator()(void* p) const noexcept {
    if (!p) return;
    if (size > 0 && size <= DeviceBufferPool<>::kSmallBufferThreshold &&
        DeviceBufferPool<>::threadState() == DeviceBufferPool<>::State::kAlive) {
      DeviceBufferPool<>::threadLocal().deallocate(p, size);
    } else {
      device_free(p);
    }
  }
};

/** \brief Internal RAII owner for an untyped GPU device allocation. */
class DeviceBuffer {
 public:
  DeviceBuffer() = default;

  explicit DeviceBuffer(size_t bytes) : bytes_(bytes) {
    if (bytes > 0) {
      // The pool serves small blocks only on the cudaMalloc fallback path, and
      // not once its thread_local has been destroyed (allocation from a
      // static/TLS destructor). A deleter size of 0 keeps the other
      // allocations on the direct device_free path.
      const bool pooled = bytes <= DeviceBufferPool<>::kSmallBufferThreshold && !device_supports_memory_pools() &&
                          DeviceBufferPool<>::threadState() != DeviceBufferPool<>::State::kDestroyed;
      void* p = pooled ? DeviceBufferPool<>::threadLocal().allocate(bytes) : device_malloc(bytes);
      ptr_ = std::unique_ptr<void, PooledCudaFreeDeleter>(p, PooledCudaFreeDeleter{pooled ? bytes : 0});
    }
  }

  // Explicit moves so a moved-from buffer reports size() == 0 (callers use
  // size() for grow-only reuse decisions; a stale size on a null buffer would
  // suppress the reallocation).
  DeviceBuffer(DeviceBuffer&& o) noexcept : ptr_(std::move(o.ptr_)), bytes_(o.bytes_) { o.bytes_ = 0; }
  DeviceBuffer& operator=(DeviceBuffer&& o) noexcept {
    if (this != &o) {
      ptr_ = std::move(o.ptr_);
      bytes_ = o.bytes_;
      o.bytes_ = 0;
    }
    return *this;
  }

  void* get() const noexcept { return ptr_.get(); }
  void* release() noexcept {
    bytes_ = 0;
    return ptr_.release();
  }
  explicit operator bool() const noexcept { return static_cast<bool>(ptr_); }

  /** Logical allocation size in bytes, tracked for adopted pointers as well. */
  size_t size() const noexcept { return bytes_; }

  // Adopt an existing device pointer of `bytes` usable bytes. Caller
  // relinquishes ownership. Adopted buffers bypass the pool on destruction
  // (deleter size == 0).
  static DeviceBuffer adopt(void* p, size_t bytes) noexcept {
    DeviceBuffer b;
    b.ptr_ = std::unique_ptr<void, PooledCudaFreeDeleter>(p, PooledCudaFreeDeleter{});
    b.bytes_ = p ? bytes : 0;
    return b;
  }

 private:
  std::unique_ptr<void, PooledCudaFreeDeleter> ptr_;
  size_t bytes_ = 0;
};

// cudaMemcpyAsync only overlaps with compute when the host side is pinned, so
// async D2H staging goes through this buffer.
class PinnedHostBuffer {
 public:
  PinnedHostBuffer() = default;

  explicit PinnedHostBuffer(size_t bytes) {
    if (bytes > 0) {
      void* p = nullptr;
      EIGEN_CUDA_RUNTIME_CHECK(cudaMallocHost(&p, bytes));
      ptr_.reset(p);
    }
  }

  void* get() const noexcept { return ptr_.get(); }
  explicit operator bool() const noexcept { return static_cast<bool>(ptr_); }

 private:
  std::unique_ptr<void, CudaFreeHostDeleter> ptr_;
};

// Upload a column-major host matrix whose strides are in elements. Ref<const
// PlainMatrix> can bind any outer stride in place. Use a 2D DMA for ordinary
// padded layouts; copy legal negative or overlapping Eigen strides one
// contiguous column at a time because CUDA cannot express them as a pitch.
template <typename Scalar>
void upload_host_matrix(Scalar* dst, Index dst_outer_stride, const Scalar* src, Index src_outer_stride, Index rows,
                        Index cols, cudaStream_t stream) {
  if (rows <= 0 || cols <= 0) return;
  eigen_assert(dst_outer_stride >= rows);
  const size_t column_bytes = static_cast<size_t>(rows) * sizeof(Scalar);
  if (src_outer_stride >= rows) {
    EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpy2DAsync(dst, static_cast<size_t>(dst_outer_stride) * sizeof(Scalar), src,
                                               static_cast<size_t>(src_outer_stride) * sizeof(Scalar), column_bytes,
                                               static_cast<size_t>(cols), cudaMemcpyHostToDevice, stream));
  } else {
    for (Index col = 0; col < cols; ++col) {
      EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpyAsync(dst + col * dst_outer_stride, src + col * src_outer_stride, column_bytes,
                                               cudaMemcpyHostToDevice, stream));
    }
  }
}

// cudaDataType_t lives in library_types.h, pulled in transitively by
// cuda_runtime.h, so this trait needs no NVIDIA library header of its own.
template <typename Scalar>
struct cuda_data_type;

template <>
struct cuda_data_type<float> {
  static constexpr cudaDataType_t value = CUDA_R_32F;
};
template <>
struct cuda_data_type<double> {
  static constexpr cudaDataType_t value = CUDA_R_64F;
};
template <>
struct cuda_data_type<std::complex<float>> {
  static constexpr cudaDataType_t value = CUDA_C_32F;
};
template <>
struct cuda_data_type<std::complex<double>> {
  static constexpr cudaDataType_t value = CUDA_C_64F;
};
}  // namespace internal
}  // namespace gpu
}  // namespace Eigen

#endif  // EIGEN_GPU_SUPPORT_H
