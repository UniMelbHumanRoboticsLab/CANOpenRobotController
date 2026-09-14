// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// Compares the host-to-device storage strategies available to DeviceMatrix on
// an integrated GPU, where the "copy" moves bytes from one region of DRAM to
// another region of the same DRAM.
//
// DeviceMatrix currently always allocates with cudaMalloc and reaches host data
// through cudaMemcpy. On a discrete GPU that is the only option. On a device
// reporting cudaDeviceProp::integrated the copy is avoidable, but only if the
// alternative storage is also fast to compute on -- mapped host memory is not
// cached in the GPU's L2 on Tegra, so a cheaper upload can cost more than it
// saves once a kernel reads the data. Every strategy is therefore measured
// twice: what it costs to get the data in place, and what a GEMM reading it
// then achieves.
//
//   Upload_*     cost to make an existing host matrix readable by cuBLAS
//   Gemm_*       GEMM throughput with both operands in that storage
//   Download_*   cost to make a device-written result readable by the host
//   RoundTrip_*  upload + GEMM + download, the figure that decides the design
//
// Strategies:
//   DeviceMalloc   cudaMalloc + cudaMemcpy from pageable host memory (today)
//   Pinned         cudaMalloc + cudaMemcpy staged through cudaMallocHost
//   Managed        cudaMallocManaged + host memcpy
//   ManagedInPlace cudaMallocManaged, matrix built in place -- no copy at all
//   Mapped         cudaHostAlloc(cudaHostAllocMapped) + memcpy, zero-copy
//   Registered     cudaHostRegister over the host matrix's own storage
//
// Build (standalone project, see CMakeLists.txt in this directory):
//   cmake -G Ninja -B build-bench-gpu -S contrib/benchmarks/GPU \
//         -DCMAKE_CUDA_ARCHITECTURES=87
//   cmake --build build-bench-gpu --target bench_unified_memory
//   ./build-bench-gpu/bench_unified_memory

#ifndef EIGEN_USE_GPU
#define EIGEN_USE_GPU
#endif
#include <Eigen/Core>
#include <contrib/Eigen/GPU>

#include <benchmark/benchmark.h>
#include <cublas_v2.h>
#include <cuda_runtime.h>

#include <cstring>

#ifndef SCALAR
#define SCALAR double
#endif
using Scalar = SCALAR;

using Eigen::Index;
namespace gpu = Eigen::gpu;
using HostMatrix = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
using HostMap = Eigen::Map<HostMatrix>;

namespace {

// cuBLAS handle and stream shared by every case, created once.
struct Blas {
  Blas() {
    EIGEN_CUDA_RUNTIME_CHECK(cudaStreamCreate(&stream));
    cublasCreate(&handle);
    cublasSetStream(handle, stream);
  }
  ~Blas() {
    cublasDestroy(handle);
    (void)cudaStreamDestroy(stream);
  }
  cublasHandle_t handle;
  cudaStream_t stream;
};

Blas& blas() {
  static Blas b;
  return b;
}

void gemm(Index n, const Scalar* a, const Scalar* b, Scalar* c) {
  const Scalar alpha = 1, beta = 0;
  const int m = gpu::internal::to_blas_int(n);
  if (std::is_same<Scalar, double>::value) {
    cublasDgemm(blas().handle, CUBLAS_OP_N, CUBLAS_OP_N, m, m, m, reinterpret_cast<const double*>(&alpha),
                reinterpret_cast<const double*>(a), m, reinterpret_cast<const double*>(b), m,
                reinterpret_cast<const double*>(&beta), reinterpret_cast<double*>(c), m);
  } else {
    cublasSgemm(blas().handle, CUBLAS_OP_N, CUBLAS_OP_N, m, m, m, reinterpret_cast<const float*>(&alpha),
                reinterpret_cast<const float*>(a), m, reinterpret_cast<const float*>(b), m,
                reinterpret_cast<const float*>(&beta), reinterpret_cast<float*>(c), m);
  }
}

// The six storage strategies, behind one interface. host() is null when the
// strategy has no host-addressable view of the allocation.
enum class Storage { DeviceMalloc, Pinned, Managed, Mapped, Registered };

class Buffer {
 public:
  Buffer(Storage s, size_t bytes) : storage_(s), bytes_(bytes) {
    switch (storage_) {
      case Storage::DeviceMalloc:
        EIGEN_CUDA_RUNTIME_CHECK(cudaMalloc(&device_, bytes));
        break;
      case Storage::Pinned:
        EIGEN_CUDA_RUNTIME_CHECK(cudaMalloc(&device_, bytes));
        EIGEN_CUDA_RUNTIME_CHECK(cudaMallocHost(&host_, bytes));
        break;
      case Storage::Managed:
        EIGEN_CUDA_RUNTIME_CHECK(cudaMallocManaged(&device_, bytes));
        host_ = device_;
        break;
      case Storage::Mapped:
        EIGEN_CUDA_RUNTIME_CHECK(cudaHostAlloc(&host_, bytes, cudaHostAllocMapped));
        EIGEN_CUDA_RUNTIME_CHECK(cudaHostGetDevicePointer(&device_, host_, 0));
        break;
      case Storage::Registered:
        // Registration targets caller-owned storage; adopt() supplies it.
        break;
    }
  }

  // Register an existing host allocation in place. Timed separately, because
  // the page-table work scales with the allocation and dominates for one-shot
  // use while amortizing to nothing for a matrix uploaded repeatedly.
  void adopt(void* host_storage) {
    EIGEN_CUDA_RUNTIME_CHECK(cudaHostRegister(host_storage, bytes_, cudaHostRegisterMapped));
    host_ = host_storage;
    registered_ = true;
    EIGEN_CUDA_RUNTIME_CHECK(cudaHostGetDevicePointer(&device_, host_, 0));
  }

  ~Buffer() {
    switch (storage_) {
      case Storage::DeviceMalloc:
        (void)cudaFree(device_);
        break;
      case Storage::Pinned:
        (void)cudaFree(device_);
        (void)cudaFreeHost(host_);
        break;
      case Storage::Managed:
        (void)cudaFree(device_);
        break;
      case Storage::Mapped:
        (void)cudaFreeHost(host_);
        break;
      case Storage::Registered:
        if (registered_) (void)cudaHostUnregister(host_);
        break;
    }
  }

  Buffer(const Buffer&) = delete;
  Buffer& operator=(const Buffer&) = delete;

  Scalar* device() const { return static_cast<Scalar*>(device_); }
  Scalar* host() const { return static_cast<Scalar*>(host_); }

  // Make `src` readable by a kernel through device(). This is the operation
  // the upload benchmarks time.
  void upload(const Scalar* src) {
    switch (storage_) {
      case Storage::DeviceMalloc:
        EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpyAsync(device_, src, bytes_, cudaMemcpyHostToDevice, blas().stream));
        EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(blas().stream));
        break;
      case Storage::Pinned:
        std::memcpy(host_, src, bytes_);
        EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpyAsync(device_, host_, bytes_, cudaMemcpyHostToDevice, blas().stream));
        EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(blas().stream));
        break;
      case Storage::Managed:
      case Storage::Mapped:
        std::memcpy(host_, src, bytes_);
        break;
      case Storage::Registered:
        // The data is already there; registration made it visible.
        break;
    }
  }

  // Make a device-written result readable through host(), into `dst`.
  void download(Scalar* dst) {
    switch (storage_) {
      case Storage::DeviceMalloc:
        EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpyAsync(dst, device_, bytes_, cudaMemcpyDeviceToHost, blas().stream));
        EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(blas().stream));
        break;
      case Storage::Pinned:
        EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpyAsync(host_, device_, bytes_, cudaMemcpyDeviceToHost, blas().stream));
        EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(blas().stream));
        std::memcpy(dst, host_, bytes_);
        break;
      case Storage::Managed:
        // concurrentManagedAccess is 0 on Tegra: the host may not read a
        // managed allocation while device work is outstanding.
        EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(blas().stream));
        std::memcpy(dst, host_, bytes_);
        break;
      case Storage::Mapped:
      case Storage::Registered:
        EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(blas().stream));
        std::memcpy(dst, host_, bytes_);
        break;
    }
  }

 private:
  Storage storage_;
  size_t bytes_ = 0;
  void* device_ = nullptr;
  void* host_ = nullptr;
  bool registered_ = false;
};

size_t byteCount(Index n) { return static_cast<size_t>(n) * static_cast<size_t>(n) * sizeof(Scalar); }

void setBytesCounter(benchmark::State& state, Index n, int64_t buffers) {
  state.SetBytesProcessed(state.iterations() * buffers * static_cast<int64_t>(byteCount(n)));
}

// Google Benchmark finishes rate counters against CPU time unless the benchmark
// opts into UseRealTime(). Every case here blocks in cudaStreamSynchronize, so
// CPU time is a small and size-dependent fraction of wall time (0.2% at n=4096
// on Orin) and a counter computed from it overstates throughput by that factor.
// All registrations below call UseRealTime(); see the note in CMakeLists.txt.
void setFlopsCounter(benchmark::State& state, Index n) {
  const double flops = 2.0 * double(n) * double(n) * double(n);
  state.counters["GFLOPS"] = benchmark::Counter(flops * double(state.iterations()), benchmark::Counter::kIsRate,
                                                benchmark::Counter::OneK::kIs1000);
}

}  // namespace

// ---------------------------------------------------------------------------
// Upload: cost of making an existing host matrix readable by cuBLAS.
// ---------------------------------------------------------------------------

void uploadBench(benchmark::State& state, Storage storage) {
  const Index n = state.range(0);
  const HostMatrix src = HostMatrix::Random(n, n);
  Buffer buf(storage, byteCount(n));
  if (storage == Storage::Registered) buf.adopt(const_cast<Scalar*>(src.data()));
  for (auto _ : state) {
    buf.upload(src.data());
    benchmark::DoNotOptimize(buf.device());
  }
  setBytesCounter(state, n, 1);
}

#define UPLOAD_CASE(Name, Enum)                                                                \
  static void BM_Upload_##Name(benchmark::State& state) { uploadBench(state, Storage::Enum); } \
  BENCHMARK(BM_Upload_##Name)                                                                  \
      ->Arg(256)                                                                               \
      ->Arg(1024)                                                                              \
      ->Arg(4096)                                                                              \
      ->Unit(benchmark::kMicrosecond)                                                          \
      ->UseRealTime()                                                                          \
      ->MinWarmUpTime(0.5)

UPLOAD_CASE(DeviceMalloc, DeviceMalloc);
UPLOAD_CASE(Pinned, Pinned);
UPLOAD_CASE(Managed, Managed);
UPLOAD_CASE(Mapped, Mapped);
UPLOAD_CASE(Registered, Registered);

// The page-table cost that Upload_Registered amortizes away. Registering a
// 4096x4096 double matrix touches 32K pages; whether that is worth paying
// depends entirely on how often the same buffer is reused.
static void BM_HostRegisterCost(benchmark::State& state) {
  const Index n = state.range(0);
  HostMatrix src = HostMatrix::Random(n, n);
  for (auto _ : state) {
    EIGEN_CUDA_RUNTIME_CHECK(cudaHostRegister(src.data(), byteCount(n), cudaHostRegisterMapped));
    EIGEN_CUDA_RUNTIME_CHECK(cudaHostUnregister(src.data()));
  }
  setBytesCounter(state, n, 1);
}
BENCHMARK(BM_HostRegisterCost)
    ->Arg(256)
    ->Arg(1024)
    ->Arg(4096)
    ->Unit(benchmark::kMicrosecond)
    ->UseRealTime()
    ->MinWarmUpTime(0.5);

// ---------------------------------------------------------------------------
// GEMM throughput with both operands in each storage class. The destination is
// always ordinary device memory, isolating the cost of *reading* the operands.
// ---------------------------------------------------------------------------

void gemmBench(benchmark::State& state, Storage storage) {
  const Index n = state.range(0);
  const HostMatrix ha = HostMatrix::Random(n, n);
  const HostMatrix hb = HostMatrix::Random(n, n);
  Buffer a(storage, byteCount(n)), b(storage, byteCount(n));
  Buffer c(Storage::DeviceMalloc, byteCount(n));
  if (storage == Storage::Registered) {
    a.adopt(const_cast<Scalar*>(ha.data()));
    b.adopt(const_cast<Scalar*>(hb.data()));
  } else {
    a.upload(ha.data());
    b.upload(hb.data());
  }
  for (auto _ : state) {
    gemm(n, a.device(), b.device(), c.device());
    EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(blas().stream));
  }
  setFlopsCounter(state, n);
}

#define GEMM_CASE(Name, Enum)                                                              \
  static void BM_Gemm_##Name(benchmark::State& state) { gemmBench(state, Storage::Enum); } \
  BENCHMARK(BM_Gemm_##Name)                                                                \
      ->Arg(256)                                                                           \
      ->Arg(1024)                                                                          \
      ->Arg(4096)                                                                          \
      ->Unit(benchmark::kMicrosecond)                                                      \
      ->UseRealTime()                                                                      \
      ->MinWarmUpTime(0.5)

GEMM_CASE(DeviceMalloc, DeviceMalloc);
GEMM_CASE(Managed, Managed);
GEMM_CASE(Mapped, Mapped);
GEMM_CASE(Registered, Registered);

// ---------------------------------------------------------------------------
// Download: cost of making a device-written result readable by the host.
// ---------------------------------------------------------------------------

void downloadBench(benchmark::State& state, Storage storage) {
  const Index n = state.range(0);
  HostMatrix dst(n, n);
  Buffer buf(storage, byteCount(n));
  if (storage == Storage::Registered) buf.adopt(dst.data());
  EIGEN_CUDA_RUNTIME_CHECK(cudaMemsetAsync(buf.device(), 0, byteCount(n), blas().stream));
  EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(blas().stream));
  for (auto _ : state) {
    buf.download(dst.data());
    benchmark::DoNotOptimize(dst.data());
  }
  setBytesCounter(state, n, 1);
}

#define DOWNLOAD_CASE(Name, Enum)                                                                  \
  static void BM_Download_##Name(benchmark::State& state) { downloadBench(state, Storage::Enum); } \
  BENCHMARK(BM_Download_##Name)                                                                    \
      ->Arg(256)                                                                                   \
      ->Arg(1024)                                                                                  \
      ->Arg(4096)                                                                                  \
      ->Unit(benchmark::kMicrosecond)                                                              \
      ->UseRealTime()                                                                              \
      ->MinWarmUpTime(0.5)

DOWNLOAD_CASE(DeviceMalloc, DeviceMalloc);
DOWNLOAD_CASE(Pinned, Pinned);
DOWNLOAD_CASE(Managed, Managed);
DOWNLOAD_CASE(Mapped, Mapped);

// ---------------------------------------------------------------------------
// End-to-end: upload two operands, GEMM, download the result. This is the
// number that decides which storage DeviceMatrix should use on an integrated
// GPU -- a strategy can win the upload and still lose here.
// ---------------------------------------------------------------------------

void roundTripBench(benchmark::State& state, Storage storage) {
  const Index n = state.range(0);
  const HostMatrix ha = HostMatrix::Random(n, n);
  const HostMatrix hb = HostMatrix::Random(n, n);
  HostMatrix hc(n, n);
  Buffer a(storage, byteCount(n)), b(storage, byteCount(n)), c(storage, byteCount(n));
  if (storage == Storage::Registered) {
    a.adopt(const_cast<Scalar*>(ha.data()));
    b.adopt(const_cast<Scalar*>(hb.data()));
    c.adopt(hc.data());
  }
  for (auto _ : state) {
    a.upload(ha.data());
    b.upload(hb.data());
    gemm(n, a.device(), b.device(), c.device());
    c.download(hc.data());
  }
  setFlopsCounter(state, n);
}

#define ROUNDTRIP_CASE(Name, Enum)                                                                   \
  static void BM_RoundTrip_##Name(benchmark::State& state) { roundTripBench(state, Storage::Enum); } \
  BENCHMARK(BM_RoundTrip_##Name)                                                                     \
      ->Arg(256)                                                                                     \
      ->Arg(1024)                                                                                    \
      ->Arg(4096)                                                                                    \
      ->Unit(benchmark::kMicrosecond)                                                                \
      ->UseRealTime()                                                                                \
      ->MinWarmUpTime(0.5)

ROUNDTRIP_CASE(DeviceMalloc, DeviceMalloc);
ROUNDTRIP_CASE(Pinned, Pinned);
ROUNDTRIP_CASE(Managed, Managed);
ROUNDTRIP_CASE(Mapped, Mapped);
ROUNDTRIP_CASE(Registered, Registered);

// ---------------------------------------------------------------------------
// The bound the others are chasing: operands built directly in managed memory
// through an Eigen::Map, so no strategy has anything to copy on the way in.
// ---------------------------------------------------------------------------

static void BM_RoundTrip_ManagedInPlace(benchmark::State& state) {
  const Index n = state.range(0);
  Buffer a(Storage::Managed, byteCount(n)), b(Storage::Managed, byteCount(n)), c(Storage::Managed, byteCount(n));
  HostMap(a.host(), n, n) = HostMatrix::Random(n, n);
  HostMap(b.host(), n, n) = HostMatrix::Random(n, n);
  for (auto _ : state) {
    gemm(n, a.device(), b.device(), c.device());
    EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(blas().stream));
    benchmark::DoNotOptimize(HostMap(c.host(), n, n).coeff(0, 0));
  }
  setFlopsCounter(state, n);
}
BENCHMARK(BM_RoundTrip_ManagedInPlace)
    ->Arg(256)
    ->Arg(1024)
    ->Arg(4096)
    ->Unit(benchmark::kMicrosecond)
    ->UseRealTime()
    ->MinWarmUpTime(0.5);

// ---------------------------------------------------------------------------
// Reported once so the results carry the device's own answer to "is this an
// integrated part, and what is it allowed to do with managed memory".
// ---------------------------------------------------------------------------

static void BM_ReportDeviceProperties(benchmark::State& state) {
  int dev = 0;
  EIGEN_CUDA_RUNTIME_CHECK(cudaGetDevice(&dev));
  cudaDeviceProp prop{};
  EIGEN_CUDA_RUNTIME_CHECK(cudaGetDeviceProperties(&prop, dev));
  for (auto _ : state) benchmark::DoNotOptimize(dev);
  state.counters["integrated"] = prop.integrated;
  state.counters["canMapHostMemory"] = prop.canMapHostMemory;
  state.counters["managedMemory"] = prop.managedMemory;
  state.counters["concurrentManagedAccess"] = prop.concurrentManagedAccess;
  state.counters["pageableMemoryAccess"] = prop.pageableMemoryAccess;
  state.counters["multiProcessorCount"] = prop.multiProcessorCount;
}
BENCHMARK(BM_ReportDeviceProperties)->Iterations(1);
