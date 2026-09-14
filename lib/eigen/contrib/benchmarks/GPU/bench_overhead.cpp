// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// Micro-benchmarks for per-call overhead in the GPU module's core layer:
// allocation traffic, hidden host synchronization, and host-side copies.
// Each benchmark pairs the current API path against a hand-rolled baseline
// that shows what the same operation costs without the overhead, so the gap
// is directly attributable.
//
//   FromHost / FromHostAsync    — extra host-side PlainMatrix copy in fromHost()
//   GemmFreshDst / PreallocDst  — cudaMalloc/cudaFree per GEMM temporary
//   DotDeviceScalar / DotRaw    — DeviceScalar wrapper cost per reduction
//   OneShotLltExpr / CachedLlt / RawPotrs — expression-solve sync + allocs
//   CudaMalloc / CudaMallocAsync — stream-ordered allocation as a remedy
//   SmallBuffer* / PoolAllocFree — small DeviceBuffer round trip (idle, behind in-flight work) and the pool itself
//
// Build (standalone project, see CMakeLists.txt in this directory):
//   cmake -G Ninja -B build-bench-gpu -S contrib/benchmarks/GPU \
//         -DCMAKE_CUDA_ARCHITECTURES=89
//   cmake --build build-bench-gpu --target bench_overhead
//   ./build-bench-gpu/bench_overhead

#ifndef EIGEN_USE_GPU
#define EIGEN_USE_GPU
#endif
#include <Eigen/Cholesky>
#include <Eigen/SparseCore>
#include <contrib/Eigen/GPU>

#include <benchmark/benchmark.h>
#include <cuda_runtime.h>
#include <cusolverDn.h>

#ifndef SCALAR
#define SCALAR double
#endif
using Scalar = SCALAR;

using Eigen::Index;
namespace gpu = Eigen::gpu;
using DeviceMatrix = gpu::DeviceMatrix<Scalar>;
using HostMatrix = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

namespace {

HostMatrix spdMatrix(Index n) {
  HostMatrix A = HostMatrix::Random(n, n);
  HostMatrix S = A * A.transpose() + HostMatrix::Identity(n, n) * Scalar(n);
  return S;
}

void syncStream(cudaStream_t s) { EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(s)); }

}  // namespace

// ---------------------------------------------------------------------------
// 1. Host->device upload: fromHost(MatrixBase) evaluates the input into a
//    fresh PlainMatrix even when it is already a plain contiguous matrix,
//    adding a full host-side copy before the H2D transfer. Baseline uploads
//    straight from the existing host buffer.
// ---------------------------------------------------------------------------

static void BM_FromHost(benchmark::State& state) {
  const Index n = state.range(0);
  HostMatrix A = HostMatrix::Random(n, n);
  gpu::Context& ctx = gpu::Context::threadLocal();
  for (auto _ : state) {
    DeviceMatrix d_A = DeviceMatrix::fromHost(A, ctx.stream());
    benchmark::DoNotOptimize(d_A.data());
  }
  state.SetBytesProcessed(state.iterations() * n * n * sizeof(Scalar));
}
BENCHMARK(BM_FromHost)->Arg(256)->Arg(1024)->Arg(4096)->UseRealTime()->MinWarmUpTime(0.5);

static void BM_FromHostRawPointer(benchmark::State& state) {
  const Index n = state.range(0);
  HostMatrix A = HostMatrix::Random(n, n);
  gpu::Context& ctx = gpu::Context::threadLocal();
  for (auto _ : state) {
    DeviceMatrix d_A = DeviceMatrix::fromHostAsync(A.data(), n, n, ctx.stream());
    syncStream(ctx.stream());
    benchmark::DoNotOptimize(d_A.data());
  }
  state.SetBytesProcessed(state.iterations() * n * n * sizeof(Scalar));
}
BENCHMARK(BM_FromHostRawPointer)->Arg(256)->Arg(1024)->Arg(4096)->UseRealTime()->MinWarmUpTime(0.5);

// ---------------------------------------------------------------------------
// 2. GEMM destination allocation: assigning into a fresh (or wrong-shaped)
//    DeviceMatrix triggers cudaMalloc in resize() and cudaFree in the
//    destructor on every iteration; a pre-allocated same-shape destination
//    skips both.
// ---------------------------------------------------------------------------

static void BM_GemmFreshDst(benchmark::State& state) {
  const Index n = state.range(0);
  gpu::Context& ctx = gpu::Context::threadLocal();
  DeviceMatrix d_A = DeviceMatrix::fromHost(HostMatrix::Random(n, n), ctx.stream());
  DeviceMatrix d_B = DeviceMatrix::fromHost(HostMatrix::Random(n, n), ctx.stream());
  for (auto _ : state) {
    DeviceMatrix d_C;  // fresh: resize() -> cudaMalloc; dtor -> cudaFree
    d_C.device(ctx) = d_A * d_B;
    syncStream(ctx.stream());
  }
}
BENCHMARK(BM_GemmFreshDst)->Arg(64)->Arg(256)->Arg(1024)->UseRealTime()->MinWarmUpTime(0.5);

static void BM_GemmPreallocDst(benchmark::State& state) {
  const Index n = state.range(0);
  gpu::Context& ctx = gpu::Context::threadLocal();
  DeviceMatrix d_A = DeviceMatrix::fromHost(HostMatrix::Random(n, n), ctx.stream());
  DeviceMatrix d_B = DeviceMatrix::fromHost(HostMatrix::Random(n, n), ctx.stream());
  DeviceMatrix d_C(n, n);
  for (auto _ : state) {
    d_C.device(ctx) = d_A * d_B;
    syncStream(ctx.stream());
  }
}
BENCHMARK(BM_GemmPreallocDst)->Arg(64)->Arg(256)->Arg(1024)->UseRealTime()->MinWarmUpTime(0.5);

// ---------------------------------------------------------------------------
// 3. BLAS-1 reduction wrapper: dot() constructs a DeviceScalar initialized to
//    zero (a small H2D upload that the following cublasXdot immediately
//    overwrites) and toggles the cuBLAS pointer mode with three API calls.
//    Baseline: cublasDdot into a pre-allocated device slot with pointer mode
//    set once. Both enqueue only (no readback); sync once per iteration.
// ---------------------------------------------------------------------------

static void BM_DotDeviceScalar(benchmark::State& state) {
  const Index n = state.range(0);
  gpu::Context& ctx = gpu::Context::threadLocal();
  DeviceMatrix d_x = DeviceMatrix::fromHost(HostMatrix::Random(n, 1), ctx.stream());
  DeviceMatrix d_y = DeviceMatrix::fromHost(HostMatrix::Random(n, 1), ctx.stream());
  for (auto _ : state) {
    gpu::DeviceScalar<Scalar> r = d_x.dot(ctx, d_y);
    benchmark::DoNotOptimize(r.devicePtr());
    syncStream(ctx.stream());
  }
}
BENCHMARK(BM_DotDeviceScalar)->Arg(1 << 12)->Arg(1 << 20)->UseRealTime()->MinWarmUpTime(0.5);

static void BM_DotRawCublas(benchmark::State& state) {
  const Index n = state.range(0);
  gpu::Context& ctx = gpu::Context::threadLocal();
  DeviceMatrix d_x = DeviceMatrix::fromHost(HostMatrix::Random(n, 1), ctx.stream());
  DeviceMatrix d_y = DeviceMatrix::fromHost(HostMatrix::Random(n, 1), ctx.stream());
  Scalar* d_result = nullptr;
  EIGEN_CUDA_RUNTIME_CHECK(cudaMalloc(&d_result, sizeof(Scalar)));
  EIGEN_CUBLAS_CHECK(cublasSetPointerMode(ctx.cublasHandle(), CUBLAS_POINTER_MODE_DEVICE));
  for (auto _ : state) {
    EIGEN_CUBLAS_CHECK(
        gpu::internal::cublasXdot(ctx.cublasHandle(), static_cast<int>(n), d_x.data(), 1, d_y.data(), 1, d_result));
    syncStream(ctx.stream());
  }
  EIGEN_CUBLAS_CHECK(cublasSetPointerMode(ctx.cublasHandle(), CUBLAS_POINTER_MODE_HOST));
  EIGEN_CUDA_RUNTIME_CHECK(cudaFree(d_result));
}
BENCHMARK(BM_DotRawCublas)->Arg(1 << 12)->Arg(1 << 20)->UseRealTime()->MinWarmUpTime(0.5);

// ---------------------------------------------------------------------------
// 4. Cholesky solve paths.
//    OneShotLltExpr: d_X = d_A.llt().solve(d_B) — re-factorizes, allocates
//      d_factor/workspace/pinned-info per call, and ends in a mandatory
//      cudaStreamSynchronize inside the dispatch.
//    CachedLltSolve: gpu::LLT computed once; solve() still allocates a fresh
//      RHS DeviceBuffer per call and syncs info on the first call.
//    RawPotrs: baseline — potrs into pre-allocated buffers, nothing else.
//    The gap between CachedLltSolve and RawPotrs is the per-solve overhead
//    attributable to the wrapper (allocation + event bookkeeping).
// ---------------------------------------------------------------------------

static void BM_OneShotLltExpr(benchmark::State& state) {
  const Index n = state.range(0);
  const Index nrhs = 8;
  gpu::Context& ctx = gpu::Context::threadLocal();
  DeviceMatrix d_A = DeviceMatrix::fromHost(spdMatrix(n), ctx.stream());
  DeviceMatrix d_B = DeviceMatrix::fromHost(HostMatrix::Random(n, nrhs), ctx.stream());
  DeviceMatrix d_X(n, nrhs);
  for (auto _ : state) {
    d_X.device(ctx) = d_A.llt().solve(d_B);
    syncStream(ctx.stream());
  }
}
BENCHMARK(BM_OneShotLltExpr)->Arg(256)->Arg(1024)->UseRealTime()->MinWarmUpTime(0.5);

static void BM_CachedLltSolve(benchmark::State& state) {
  const Index n = state.range(0);
  const Index nrhs = 8;
  gpu::Context& ctx = gpu::Context::threadLocal();
  DeviceMatrix d_A = DeviceMatrix::fromHost(spdMatrix(n), ctx.stream());
  DeviceMatrix d_B = DeviceMatrix::fromHost(HostMatrix::Random(n, nrhs), ctx.stream());
  gpu::LLT<Scalar> llt;
  llt.compute(d_A);
  benchmark::DoNotOptimize(llt.info());  // factor + first-solve sync out of the loop
  for (auto _ : state) {
    DeviceMatrix d_X = llt.solve(d_B);
    syncStream(llt.stream());
    benchmark::DoNotOptimize(d_X.data());
  }
}
BENCHMARK(BM_CachedLltSolve)->Arg(256)->Arg(1024)->UseRealTime()->MinWarmUpTime(0.5);

static void BM_RawPotrs(benchmark::State& state) {
  const Index n = state.range(0);
  const Index nrhs = 8;
  gpu::Context& ctx = gpu::Context::threadLocal();
  cusolverDnHandle_t solver = ctx.cusolverHandle();

  // Factor once with the module (correctness is not under test here), then
  // extract the factor into a raw buffer for repeated potrs calls.
  HostMatrix A = spdMatrix(n);
  Eigen::LLT<HostMatrix> hostLlt(A);
  HostMatrix L = HostMatrix(hostLlt.matrixL());
  DeviceMatrix d_L = DeviceMatrix::fromHost(L, ctx.stream());
  DeviceMatrix d_B = DeviceMatrix::fromHost(HostMatrix::Random(n, nrhs), ctx.stream());
  DeviceMatrix d_X(n, nrhs);

  gpu::internal::CusolverParams params;
  int* d_info = nullptr;
  EIGEN_CUDA_RUNTIME_CHECK(cudaMalloc(&d_info, sizeof(int)));
  constexpr cudaDataType_t dtype = gpu::internal::cuda_data_type<Scalar>::value;

  for (auto _ : state) {
    EIGEN_CUDA_RUNTIME_CHECK(
        cudaMemcpyAsync(d_X.data(), d_B.data(), sizeof(Scalar) * n * nrhs, cudaMemcpyDeviceToDevice, ctx.stream()));
    EIGEN_CUSOLVER_CHECK(cusolverDnXpotrs(solver, params.p, CUBLAS_FILL_MODE_LOWER, n, nrhs, dtype, d_L.data(), n,
                                          dtype, d_X.data(), n, d_info));
    syncStream(ctx.stream());
  }
  EIGEN_CUDA_RUNTIME_CHECK(cudaFree(d_info));
}
BENCHMARK(BM_RawPotrs)->Arg(256)->Arg(1024)->UseRealTime()->MinWarmUpTime(0.5);

// ---------------------------------------------------------------------------
// 4b. Device-resident SpMV in a loop (the CG hot path): measures per-call
//     cuSPARSE descriptor/bufferSize overhead on the cached deviceView path.
// ---------------------------------------------------------------------------

static void BM_DeviceSpMV(benchmark::State& state) {
  const Index n = state.range(0);
  using SpMat = Eigen::SparseMatrix<Scalar, Eigen::ColMajor, int>;
  SpMat A(n, n);
  // 1D Laplacian stencil.
  A.reserve(Eigen::VectorXi::Constant(n, 3));
  for (Index i = 0; i < n; ++i) {
    A.insert(i, i) = Scalar(2);
    if (i > 0) A.insert(i - 1, i) = Scalar(-1);
    if (i + 1 < n) A.insert(i + 1, i) = Scalar(-1);
  }
  A.makeCompressed();

  gpu::Context& gctx = gpu::Context::threadLocal();
  gpu::SparseContext<Scalar> ctx(gctx);
  auto view = ctx.deviceView(A);
  DeviceMatrix d_x = DeviceMatrix::fromHost(HostMatrix::Random(n, 1), gctx.stream());
  DeviceMatrix d_y(n, 1);
  for (auto _ : state) {
    d_y = view * d_x;
    syncStream(gctx.stream());
  }
}
BENCHMARK(BM_DeviceSpMV)->Arg(1 << 12)->Arg(1 << 20)->UseRealTime()->MinWarmUpTime(0.5);

// ---------------------------------------------------------------------------
// 5. Allocator microbenchmark: cudaMalloc/cudaFree (what DeviceMatrix uses
//    today) vs cudaMallocAsync/cudaFreeAsync (stream-ordered pool). Supports
//    the proposal to switch DeviceMatrix storage to stream-ordered allocation.
// ---------------------------------------------------------------------------

static void BM_CudaMallocFree(benchmark::State& state) {
  const size_t bytes = static_cast<size_t>(state.range(0));
  for (auto _ : state) {
    void* p = nullptr;
    EIGEN_CUDA_RUNTIME_CHECK(cudaMalloc(&p, bytes));
    EIGEN_CUDA_RUNTIME_CHECK(cudaFree(p));
  }
}
BENCHMARK(BM_CudaMallocFree)->Arg(1 << 10)->Arg(1 << 20)->Arg(1 << 26)->UseRealTime()->MinWarmUpTime(0.5);

static void BM_CudaMallocAsyncFree(benchmark::State& state) {
  const size_t bytes = static_cast<size_t>(state.range(0));
  gpu::Context& ctx = gpu::Context::threadLocal();
  int pools_supported = 0;
  int device = 0;
  EIGEN_CUDA_RUNTIME_CHECK(cudaGetDevice(&device));
  EIGEN_CUDA_RUNTIME_CHECK(cudaDeviceGetAttribute(&pools_supported, cudaDevAttrMemoryPoolsSupported, device));
  if (!pools_supported) {
    state.SkipWithError("cudaMallocAsync not supported on this device");
    return;
  }
  for (auto _ : state) {
    void* p = nullptr;
    EIGEN_CUDA_RUNTIME_CHECK(cudaMallocAsync(&p, bytes, ctx.stream()));
    EIGEN_CUDA_RUNTIME_CHECK(cudaFreeAsync(p, ctx.stream()));
  }
  syncStream(ctx.stream());
}
BENCHMARK(BM_CudaMallocAsyncFree)->Arg(1 << 10)->Arg(1 << 20)->Arg(1 << 26)->UseRealTime()->MinWarmUpTime(0.5);

// ---------------------------------------------------------------------------
// 6. Small buffers: a DeviceBuffer below DeviceBufferPool's threshold takes
//    the stream-ordered allocator where memory pools exist and the thread-local
//    pool otherwise; PoolAllocFree drives the pool directly, paying one event
//    record per release and one query per reuse. Behind an in-flight kernel a
//    release trails that kernel, so a reuse finds it retired or falls through
//    to the allocator (see CudaMallocAsyncFree).
// ---------------------------------------------------------------------------

static void BM_SmallBufferAllocFree(benchmark::State& state) {
  const size_t bytes = static_cast<size_t>(state.range(0));
  for (auto _ : state) {
    gpu::internal::DeviceBuffer b(bytes);
    benchmark::DoNotOptimize(b.get());
  }
}
BENCHMARK(BM_SmallBufferAllocFree)->Arg(8)->Arg(64)->Arg(256)->UseRealTime()->MinWarmUpTime(0.5);

static void BM_PoolAllocFree(benchmark::State& state) {
  const size_t bytes = static_cast<size_t>(state.range(0));
  auto& pool = gpu::internal::DeviceBufferPool<>::threadLocal();
  for (auto _ : state) {
    void* p = pool.allocate(bytes);
    benchmark::DoNotOptimize(p);
    pool.deallocate(p, bytes);
  }
}
BENCHMARK(BM_PoolAllocFree)->Arg(8)->Arg(256)->UseRealTime()->MinWarmUpTime(0.5);

static void BM_SmallBufferAllocFreeBusyStream(benchmark::State& state) {
  const size_t bytes = static_cast<size_t>(state.range(0));
  gpu::Context& ctx = gpu::Context::threadLocal();
  const Index n = Index(1) << 22;
  DeviceMatrix d_x = DeviceMatrix::fromHost(HostMatrix::Random(n, 1));
  DeviceMatrix d_y = DeviceMatrix::fromHost(HostMatrix::Random(n, 1));
  for (auto _ : state) {
    // ~1 ms of work ahead of each release, as in an iterative solver loop.
    gpu::DeviceScalar<Scalar> r = d_x.dot(ctx, d_y);
    gpu::internal::DeviceBuffer b(bytes);
    benchmark::DoNotOptimize(b.get());
    benchmark::DoNotOptimize(r.devicePtr());
  }
  syncStream(ctx.stream());
}
BENCHMARK(BM_SmallBufferAllocFreeBusyStream)->Arg(8)->Arg(64)->UseRealTime()->MinWarmUpTime(0.5);
