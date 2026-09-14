// Benchmarks for Eigen Tensor shuffling (transpose / permutation).
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#define EIGEN_USE_THREADS

#include <benchmark/benchmark.h>
#include <contrib/Eigen/Tensor>
#include <Eigen/ThreadPool>

using namespace Eigen;

typedef float Scalar;

// --- Rank-2 transpose ---
static void BM_Shuffle2D(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);

  Tensor<Scalar, 2> A(M, N);
  Tensor<Scalar, 2> B(N, M);
  A.setRandom();

  Eigen::array<int, 2> perm = {1, 0};

  for (auto _ : state) {
    B = A.shuffle(perm);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * M * N * sizeof(Scalar) * 2);
}

// --- Identity shuffle (no permutation, measures overhead) ---
static void BM_ShuffleIdentity(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);

  Tensor<Scalar, 2> A(M, N);
  Tensor<Scalar, 2> B(M, N);
  A.setRandom();

  Eigen::array<int, 2> perm = {0, 1};

  for (auto _ : state) {
    B = A.shuffle(perm);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * M * N * sizeof(Scalar) * 2);
}

// --- Rank-3 permutation ---
static void BM_Shuffle3D(benchmark::State& state) {
  const int D0 = state.range(0);
  const int D1 = state.range(1);
  const int D2 = state.range(2);

  Tensor<Scalar, 3> A(D0, D1, D2);
  A.setRandom();

  // Permutation (2, 0, 1)
  Eigen::array<int, 3> perm = {2, 0, 1};

  for (auto _ : state) {
    Tensor<Scalar, 3> B = A.shuffle(perm);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * D0 * D1 * D2 * sizeof(Scalar) * 2);
}

// --- Rank-4 permutation (NCHW -> NHWC layout conversion) ---
static void BM_Shuffle4D_NCHW_to_NHWC(benchmark::State& state) {
  const int N = state.range(0);
  const int C = state.range(1);
  const int H = state.range(2);

  Tensor<Scalar, 4> A(N, C, H, H);
  A.setRandom();

  // NCHW -> NHWC: permute (0, 2, 3, 1)
  Eigen::array<int, 4> perm = {0, 2, 3, 1};

  for (auto _ : state) {
    Tensor<Scalar, 4> B = A.shuffle(perm);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * N * C * H * H * sizeof(Scalar) * 2);
}

// --- Expression-sourced shuffles (packet path) ---
// An expression argument has no raw buffer, so the shuffle cannot use its
// block path and serves packets instead. Permutations that preserve the
// inner dimension load whole packets from one inner run; inner-shuffled
// permutations gather, but still compute the index mapping once per packet.
static void BM_ShuffleExprInnerPreserved(benchmark::State& state) {
  const int D0 = state.range(0);
  const int D1 = state.range(1);
  const int D2 = state.range(2);

  Tensor<Scalar, 3> A(D0, D1, D2);
  A.setRandom();

  // Permutation (0, 2, 1): the inner (first, ColMajor) dimension stays.
  Eigen::array<int, 3> perm = {0, 2, 1};

  Tensor<Scalar, 3> B;
  for (auto _ : state) {
    B = (A + A.constant(Scalar(1))).shuffle(perm);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }

  // Validate outside the timed loop.
  for (int i = 0; i < D0; ++i) {
    for (int j = 0; j < D2; ++j) {
      for (int k = 0; k < D1; ++k) {
        if (B(i, j, k) != A(i, k, j) + Scalar(1)) {
          state.SkipWithError("validation failed");
          return;
        }
      }
    }
  }
  state.SetBytesProcessed(state.iterations() * D0 * D1 * D2 * sizeof(Scalar) * 2);
}

static void BM_ShuffleExprInnerShuffled(benchmark::State& state) {
  const int D0 = state.range(0);
  const int D1 = state.range(1);
  const int D2 = state.range(2);

  Tensor<Scalar, 3> A(D0, D1, D2);
  A.setRandom();

  // Permutation (2, 0, 1): the inner dimension changes.
  Eigen::array<int, 3> perm = {2, 0, 1};

  Tensor<Scalar, 3> B;
  for (auto _ : state) {
    B = (A + A.constant(Scalar(1))).shuffle(perm);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }

  for (int i = 0; i < D2; ++i) {
    for (int j = 0; j < D0; ++j) {
      for (int k = 0; k < D1; ++k) {
        if (B(i, j, k) != A(j, k, i) + Scalar(1)) {
          state.SkipWithError("validation failed");
          return;
        }
      }
    }
  }
  state.SetBytesProcessed(state.iterations() * D0 * D1 * D2 * sizeof(Scalar) * 2);
}

// Lvalue: packet writes through a shuffled destination.
static void BM_ShuffleWrite(benchmark::State& state) {
  const int D0 = state.range(0);
  const int D1 = state.range(1);
  const int D2 = state.range(2);

  // Destination shuffled with inner-preserving permutation (0, 2, 1).
  Eigen::array<int, 3> perm = {0, 2, 1};

  Tensor<Scalar, 3> src(D0, D2, D1);
  src.setRandom();
  Tensor<Scalar, 3> dst(D0, D1, D2);
  dst.setZero();
  Eigen::array<Index, 3> no_strides = {1, 1, 1};

  for (auto _ : state) {
    // Unit striding disables block evaluation while preserving packet access,
    // forcing the shuffled lvalue's writePacket path.
    dst.shuffle(perm) = src.stride(no_strides);
    benchmark::DoNotOptimize(dst.data());
    benchmark::ClobberMemory();
  }

  for (int i = 0; i < D0; ++i) {
    for (int j = 0; j < D2; ++j) {
      for (int k = 0; k < D1; ++k) {
        if (dst(i, k, j) != src(i, j, k)) {
          state.SkipWithError("validation failed");
          return;
        }
      }
    }
  }
  state.SetBytesProcessed(state.iterations() * D0 * D1 * D2 * sizeof(Scalar) * 2);
}

// --- ThreadPool variants ---

static void BM_Shuffle2D_ThreadPool(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);
  const int threads = state.range(2);

  Tensor<Scalar, 2> A(M, N);
  Tensor<Scalar, 2> B(N, M);
  A.setRandom();

  ThreadPool tp(threads);
  ThreadPoolDevice dev(&tp, threads);

  Eigen::array<int, 2> perm = {1, 0};

  for (auto _ : state) {
    B.device(dev) = A.shuffle(perm);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * M * N * sizeof(Scalar) * 2);
  state.counters["threads"] = threads;
}

static void BM_Shuffle4D_NCHW_to_NHWC_ThreadPool(benchmark::State& state) {
  const int N = state.range(0);
  const int C = state.range(1);
  const int H = state.range(2);
  const int threads = state.range(3);

  Tensor<Scalar, 4> A(N, C, H, H);
  Tensor<Scalar, 4> B(N, H, H, C);
  A.setRandom();

  ThreadPool tp(threads);
  ThreadPoolDevice dev(&tp, threads);

  // NCHW -> NHWC: permute (0, 2, 3, 1)
  Eigen::array<int, 4> perm = {0, 2, 3, 1};

  for (auto _ : state) {
    B.device(dev) = A.shuffle(perm);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * N * C * H * H * sizeof(Scalar) * 2);
  state.counters["threads"] = threads;
}

// clang-format off
#define SHUFFLE_2D_SIZES \
  ->Args({256, 256})->Args({1024, 1024}) \
  ->Args({64, 4096})->Args({4096, 64})

#define SHUFFLE_3D_SIZES \
  ->Args({64, 64, 64})->Args({128, 128, 64})->Args({32, 256, 256})

// {batch, channels, h}: pure Cartesian product.
#define SHUFFLE_4D_SIZES ->ArgsProduct({{1, 8}, {3, 64}, {32, 64}})

#define SHUFFLE_2D_THREADPOOL_SIZES \
  ->Args({256, 256, 1})->Args({256, 256, 2})->Args({256, 256, 4}) \
  ->Args({256, 256, 8})->Args({256, 256, 12})->Args({256, 256, 16}) \
  ->Args({1024, 1024, 1})->Args({1024, 1024, 2})->Args({1024, 1024, 4}) \
  ->Args({1024, 1024, 8})->Args({1024, 1024, 12})->Args({1024, 1024, 16})

// {batch, channels, h, threads}: pure Cartesian product.
#define SHUFFLE_4D_THREADPOOL_SIZES ->ArgsProduct({{1, 8}, {64}, {32, 64}, {1, 2, 4, 8, 12, 16}})
// clang-format on

BENCHMARK(BM_Shuffle2D) SHUFFLE_2D_SIZES;
BENCHMARK(BM_ShuffleIdentity) SHUFFLE_2D_SIZES;
BENCHMARK(BM_Shuffle3D) SHUFFLE_3D_SIZES;
BENCHMARK(BM_ShuffleExprInnerPreserved) SHUFFLE_3D_SIZES;
BENCHMARK(BM_ShuffleExprInnerShuffled) SHUFFLE_3D_SIZES;
BENCHMARK(BM_ShuffleWrite) SHUFFLE_3D_SIZES;
BENCHMARK(BM_Shuffle4D_NCHW_to_NHWC) SHUFFLE_4D_SIZES;
BENCHMARK(BM_Shuffle2D_ThreadPool) SHUFFLE_2D_THREADPOOL_SIZES->UseRealTime();
BENCHMARK(BM_Shuffle4D_NCHW_to_NHWC_ThreadPool) SHUFFLE_4D_THREADPOOL_SIZES->UseRealTime();
