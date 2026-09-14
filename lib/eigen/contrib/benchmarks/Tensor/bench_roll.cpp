// Benchmarks for Eigen TensorRoll.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <contrib/Eigen/Tensor>

using namespace Eigen;

typedef float Scalar;

// --- Roll only the inner-most (contiguous) dimension. ---
static void BM_Roll_Inner(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);
  const int shift = state.range(2);

  Tensor<Scalar, 2> A(M, N);
  A.setRandom();

  array<Index, 2> rolls = {shift, 0};

  for (auto _ : state) {
    Tensor<Scalar, 2> B = A.roll(rolls);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * static_cast<int64_t>(M) * N * sizeof(Scalar));
}

// --- Roll only an outer dimension. Inner dim stays contiguous. ---
static void BM_Roll_Outer(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);
  const int shift = state.range(2);

  Tensor<Scalar, 2> A(M, N);
  A.setRandom();

  array<Index, 2> rolls = {0, shift};

  for (auto _ : state) {
    Tensor<Scalar, 2> B = A.roll(rolls);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * static_cast<int64_t>(M) * N * sizeof(Scalar));
}

// --- Roll every dimension. ---
static void BM_Roll_All(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);
  const int shift = state.range(2);

  Tensor<Scalar, 2> A(M, N);
  A.setRandom();

  array<Index, 2> rolls = {shift, shift};

  for (auto _ : state) {
    Tensor<Scalar, 2> B = A.roll(rolls);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * static_cast<int64_t>(M) * N * sizeof(Scalar));
}

// --- 3D roll with the inner dim shifted. ---
static void BM_Roll_3D_Inner(benchmark::State& state) {
  const int D0 = state.range(0);
  const int D1 = state.range(1);
  const int D2 = state.range(2);

  Tensor<Scalar, 3> A(D0, D1, D2);
  A.setRandom();

  array<Index, 3> rolls = {D0 / 4, 0, 0};

  for (auto _ : state) {
    Tensor<Scalar, 3> B = A.roll(rolls);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * static_cast<int64_t>(D0) * D1 * D2 * sizeof(Scalar));
}

// --- Writes through a rolled destination (lvalue writeBlock path, which
// scatters each block into its wrap-around pieces) ---
static void BM_RollWrite_Inner(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);
  const int shift = state.range(2);

  Tensor<Scalar, 2> src(M, N);
  src.setRandom();
  Tensor<Scalar, 2> dst(M, N);
  dst.setZero();

  array<Index, 2> rolls = {shift, 0};

  for (auto _ : state) {
    dst.roll(rolls) = src;
    benchmark::DoNotOptimize(dst.data());
    benchmark::ClobberMemory();
  }
  for (int i = 0; i < M; ++i) {
    for (int j = 0; j < N; ++j) {
      if (dst((i + shift) % M, j) != src(i, j)) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  state.SetBytesProcessed(state.iterations() * static_cast<int64_t>(M) * N * sizeof(Scalar));
}

static void BM_RollWrite_Outer(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);
  const int shift = state.range(2);

  Tensor<Scalar, 2> src(M, N);
  src.setRandom();
  Tensor<Scalar, 2> dst(M, N);
  dst.setZero();

  array<Index, 2> rolls = {0, shift};

  for (auto _ : state) {
    dst.roll(rolls) = src;
    benchmark::DoNotOptimize(dst.data());
    benchmark::ClobberMemory();
  }
  for (int i = 0; i < M; ++i) {
    for (int j = 0; j < N; ++j) {
      if (dst(i, (j + shift) % N) != src(i, j)) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  state.SetBytesProcessed(state.iterations() * static_cast<int64_t>(M) * N * sizeof(Scalar));
}

// Lazy right-hand-side blocks: writeBlock materializes the cwise expression
// into a temporary before scattering the pieces.
static void BM_RollWrite_Expr(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);
  const int shift = state.range(2);

  Tensor<Scalar, 2> src(M, N);
  src.setRandom();
  Tensor<Scalar, 2> dst(M, N);
  dst.setZero();

  array<Index, 2> rolls = {shift, shift};

  for (auto _ : state) {
    dst.roll(rolls) = src + src.constant(1.0f);
    benchmark::DoNotOptimize(dst.data());
    benchmark::ClobberMemory();
  }
  for (int i = 0; i < M; ++i) {
    for (int j = 0; j < N; ++j) {
      if (dst((i + shift) % M, (j + shift) % N) != src(i, j) + 1.0f) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  state.SetBytesProcessed(state.iterations() * static_cast<int64_t>(M) * N * sizeof(Scalar));
}

// clang-format off
#define ROLL_SIZES \
  ->Args({64, 64, 1})->Args({64, 64, 13}) \
  ->Args({256, 256, 1})->Args({256, 256, 13}) \
  ->Args({1024, 1024, 1})->Args({1024, 1024, 13})

// Transition sizes around the tiled-evaluation crossover (~8x8) for the
// write benchmarks: below it the fixed tiled-executor setup cost dominates.
#define ROLL_WRITE_SIZES \
  ->Args({2, 2, 1})->Args({4, 4, 1})->Args({8, 8, 1})->Args({16, 16, 3})->Args({32, 32, 3}) \
  ->Args({64, 64, 1})->Args({64, 64, 13}) \
  ->Args({256, 256, 1})->Args({256, 256, 13}) \
  ->Args({1024, 1024, 1})->Args({1024, 1024, 13})

#define ROLL_3D_SIZES \
  ->Args({32, 32, 32})->Args({64, 64, 64})->Args({128, 128, 128})
// clang-format on

BENCHMARK(BM_Roll_Inner) ROLL_SIZES;
BENCHMARK(BM_Roll_Outer) ROLL_SIZES;
BENCHMARK(BM_Roll_All) ROLL_SIZES;
BENCHMARK(BM_Roll_3D_Inner) ROLL_3D_SIZES;
BENCHMARK(BM_RollWrite_Inner) ROLL_WRITE_SIZES;
BENCHMARK(BM_RollWrite_Outer) ROLL_WRITE_SIZES;
BENCHMARK(BM_RollWrite_Expr) ROLL_WRITE_SIZES;
