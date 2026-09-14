// Benchmarks for Eigen Tensor morphing operations: reshape, slice, chip, pad, strided slice.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#define EIGEN_USE_THREADS

#include <benchmark/benchmark.h>
#include <contrib/Eigen/Tensor>
#include <Eigen/ThreadPool>

using namespace Eigen;

typedef float Scalar;

// --- Reshape (zero-cost if no evaluation needed; force eval via assignment) ---
static void BM_Reshape(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);

  Tensor<Scalar, 2> A(M, N);
  A.setRandom();

  Eigen::array<Index, 1> new_shape = {M * N};

  for (auto _ : state) {
    Tensor<Scalar, 1> B = A.reshape(new_shape);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * M * N * sizeof(Scalar));
}

// --- Slice ---
static void BM_Slice(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);

  Tensor<Scalar, 2> A(M, N);
  A.setRandom();

  int sliceM = M / 2;
  int sliceN = N / 2;
  Eigen::array<Index, 2> offsets = {0, 0};
  Eigen::array<Index, 2> extents = {sliceM, sliceN};

  for (auto _ : state) {
    Tensor<Scalar, 2> B = A.slice(offsets, extents);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * sliceM * sliceN * sizeof(Scalar));
}

// --- Chip (extract a sub-tensor along one dimension) ---
static void BM_Chip(benchmark::State& state) {
  const int D0 = state.range(0);
  const int D1 = state.range(1);
  const int D2 = state.range(2);

  Tensor<Scalar, 3> A(D0, D1, D2);
  A.setRandom();

  for (auto _ : state) {
    Tensor<Scalar, 2> B = A.chip(0, 0);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * D1 * D2 * sizeof(Scalar));
}

// --- Pad ---
static void BM_Pad(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);
  const int padSize = state.range(2);

  Tensor<Scalar, 2> A(M, N);
  A.setRandom();

  Eigen::array<std::pair<int, int>, 2> paddings;
  paddings[0] = {padSize, padSize};
  paddings[1] = {padSize, padSize};

  for (auto _ : state) {
    Tensor<Scalar, 2> B = A.pad(paddings);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  int outM = M + 2 * padSize;
  int outN = N + 2 * padSize;
  state.SetBytesProcessed(state.iterations() * outM * outN * sizeof(Scalar));
}

// --- Strided slice ---
// Covers the packet fast paths of the strided-slice evaluator: contiguous
// inner loads (inner stride 1), reversed inner loads (inner stride -1), and
// the scalar gather fallback (non-unit inner stride).
static void BM_StridedSliceRead(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);
  const Index s0 = state.range(2);
  const Index s1 = state.range(3);

  Tensor<Scalar, 2> A(M, N);
  A.setRandom();

  const Eigen::array<Index, 2> start = {s0 > 0 ? 1 : M - 2, s1 > 0 ? 1 : N - 2};
  const Eigen::array<Index, 2> stop = {s0 > 0 ? M - 1 : 0, s1 > 0 ? N - 1 : 0};
  const Eigen::array<Index, 2> strides = {s0, s1};

  Tensor<Scalar, 2> B;
  for (auto _ : state) {
    B = A.stridedSlice(start, stop, strides);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }

  // Validate outside the timed loop.
  for (Index i = 0; i < B.dimension(0); ++i) {
    for (Index j = 0; j < B.dimension(1); ++j) {
      if (B(i, j) != A(start[0] + i * s0, start[1] + j * s1)) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  state.SetBytesProcessed(state.iterations() * B.size() * sizeof(Scalar));
}

// Strided slice feeding a packet-math kernel the compiler cannot rescue by
// auto-vectorizing the scalar loop.
static void BM_StridedSliceExp(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);
  const Index s0 = state.range(2);
  const Index s1 = state.range(3);

  Tensor<Scalar, 2> A(M, N);
  A.setRandom();

  const Eigen::array<Index, 2> start = {s0 > 0 ? 1 : M - 2, s1 > 0 ? 1 : N - 2};
  const Eigen::array<Index, 2> stop = {s0 > 0 ? M - 1 : 0, s1 > 0 ? N - 1 : 0};
  const Eigen::array<Index, 2> strides = {s0, s1};

  Tensor<Scalar, 2> B;
  for (auto _ : state) {
    B = A.stridedSlice(start, stop, strides).exp();
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }

  for (Index i = 0; i < B.dimension(0); ++i) {
    for (Index j = 0; j < B.dimension(1); ++j) {
      const Scalar expected = std::exp(A(start[0] + i * s0, start[1] + j * s1));
      if (std::abs(B(i, j) - expected) > 1e-4f * std::abs(expected)) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  state.SetBytesProcessed(state.iterations() * B.size() * sizeof(Scalar));
}

static void BM_StridedSliceWrite(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);
  const Index s0 = state.range(2);
  const Index s1 = state.range(3);

  const Eigen::array<Index, 2> start = {s0 > 0 ? 1 : M - 2, s1 > 0 ? 1 : N - 2};
  const Eigen::array<Index, 2> stop = {s0 > 0 ? M - 1 : 0, s1 > 0 ? N - 1 : 0};
  const Eigen::array<Index, 2> strides = {s0, s1};

  Tensor<Scalar, 2> B(M, N);
  B.setZero();
  // Size the source from the slice itself, so it tracks the stride sweep.
  Tensor<Scalar, 2> A = B.stridedSlice(start, stop, strides);
  A.setRandom();

  for (auto _ : state) {
    B.stridedSlice(start, stop, strides) = A;
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }

  for (Index i = 0; i < A.dimension(0); ++i) {
    for (Index j = 0; j < A.dimension(1); ++j) {
      if (B(start[0] + i * s0, start[1] + j * s1) != A(i, j)) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  state.SetBytesProcessed(state.iterations() * A.size() * sizeof(Scalar));
}

// Expression assigned through a strided-slice destination. The rhs block has
// no raw buffer, so this exercises the expression side of writeBlock: direct
// assignment into the destination for a unit inner stride, and the
// materialize-into-temp path otherwise.
static void BM_StridedSliceWriteExp(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);
  const Index s0 = state.range(2);
  const Index s1 = state.range(3);

  const Eigen::array<Index, 2> start = {s0 > 0 ? 1 : M - 2, s1 > 0 ? 1 : N - 2};
  const Eigen::array<Index, 2> stop = {s0 > 0 ? M - 1 : 0, s1 > 0 ? N - 1 : 0};
  const Eigen::array<Index, 2> strides = {s0, s1};

  Tensor<Scalar, 2> B(M, N);
  B.setZero();
  // Size the source from the slice itself, so it tracks the stride sweep.
  Tensor<Scalar, 2> A = B.stridedSlice(start, stop, strides);
  A.setRandom();

  for (auto _ : state) {
    B.stridedSlice(start, stop, strides) = A.exp();
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }

  for (Index i = 0; i < A.dimension(0); ++i) {
    for (Index j = 0; j < A.dimension(1); ++j) {
      const Scalar expected = std::exp(A(i, j));
      if (std::abs(B(start[0] + i * s0, start[1] + j * s1) - expected) > 1e-4f * std::abs(expected)) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  state.SetBytesProcessed(state.iterations() * A.size() * sizeof(Scalar));
}

// --- ThreadPool variants ---

static void BM_Slice_ThreadPool(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);
  const int threads = state.range(2);

  Tensor<Scalar, 2> A(M, N);
  A.setRandom();

  int sliceM = M / 2;
  int sliceN = N / 2;
  Eigen::array<Index, 2> offsets = {0, 0};
  Eigen::array<Index, 2> extents = {sliceM, sliceN};

  ThreadPool tp(threads);
  ThreadPoolDevice dev(&tp, threads);

  Tensor<Scalar, 2> B(sliceM, sliceN);

  for (auto _ : state) {
    B.device(dev) = A.slice(offsets, extents);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * sliceM * sliceN * sizeof(Scalar));
  state.counters["threads"] = threads;
}

static void BM_Pad_ThreadPool(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);
  const int threads = state.range(2);

  Tensor<Scalar, 2> A(M, N);
  A.setRandom();

  Eigen::array<std::pair<int, int>, 2> paddings;
  paddings[0] = {4, 4};
  paddings[1] = {4, 4};

  int outM = M + 8;
  int outN = N + 8;

  ThreadPool tp(threads);
  ThreadPoolDevice dev(&tp, threads);

  Tensor<Scalar, 2> B(outM, outN);

  for (auto _ : state) {
    B.device(dev) = A.pad(paddings);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * outM * outN * sizeof(Scalar));
  state.counters["threads"] = threads;
}

// clang-format off
#define MORPH_SIZES \
  ->Args({256, 256})->Args({1024, 1024})

#define CHIP_SIZES \
  ->Args({32, 256, 256})->Args({64, 128, 128})->Args({8, 512, 512})

#define PAD_SIZES \
  ->Args({256, 256, 1})->Args({256, 256, 4})->Args({256, 256, 16}) \
  ->Args({1024, 1024, 1})->Args({1024, 1024, 4})->Args({1024, 1024, 16})

#define STRIDED_SLICE_SIZES \
  ->Args({256, 256, 1, 1})->Args({1024, 1024, 1, 1})   /* contiguous inner */ \
  ->Args({256, 256, -1, -1})->Args({1024, 1024, -1, -1}) /* reversed inner */ \
  ->Args({256, 256, 2, 2})->Args({1024, 1024, 2, 2})   /* gather fallback */

#define MORPH_THREADPOOL_SIZES \
  ->Args({256, 256, 1})->Args({256, 256, 2})->Args({256, 256, 4}) \
  ->Args({256, 256, 8})->Args({256, 256, 12})->Args({256, 256, 16}) \
  ->Args({1024, 1024, 1})->Args({1024, 1024, 2})->Args({1024, 1024, 4}) \
  ->Args({1024, 1024, 8})->Args({1024, 1024, 12})->Args({1024, 1024, 16})
// clang-format on

BENCHMARK(BM_Reshape) MORPH_SIZES;
BENCHMARK(BM_Slice) MORPH_SIZES;
BENCHMARK(BM_Chip) CHIP_SIZES;
BENCHMARK(BM_Pad) PAD_SIZES;
BENCHMARK(BM_StridedSliceRead) STRIDED_SLICE_SIZES;
BENCHMARK(BM_StridedSliceExp) STRIDED_SLICE_SIZES;
BENCHMARK(BM_StridedSliceWrite) STRIDED_SLICE_SIZES;
BENCHMARK(BM_StridedSliceWriteExp) STRIDED_SLICE_SIZES;
BENCHMARK(BM_Slice_ThreadPool) MORPH_THREADPOOL_SIZES->UseRealTime();
BENCHMARK(BM_Pad_ThreadPool) MORPH_THREADPOOL_SIZES->UseRealTime();
