// Benchmarks for Eigen Tensor broadcasting.
// Tests broadcasting along various dimensions and ranks.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#define EIGEN_USE_THREADS

#include <benchmark/benchmark.h>
#include <contrib/Eigen/Tensor>
#include <Eigen/ThreadPool>

using namespace Eigen;

typedef float Scalar;

// --- Broadcast row vector {1,N} -> {M,N} ---
static void BM_BroadcastRow(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);

  Tensor<Scalar, 2> row(1, N);
  Tensor<Scalar, 2> result(M, N);
  row.setRandom();

  Eigen::array<int, 2> bcast = {M, 1};

  for (auto _ : state) {
    result = row.broadcast(bcast);
    benchmark::DoNotOptimize(result.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * M * N * sizeof(Scalar));
}

// --- Broadcast col vector {M,1} -> {M,N} ---
static void BM_BroadcastCol(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);

  Tensor<Scalar, 2> col(M, 1);
  Tensor<Scalar, 2> result(M, N);
  col.setRandom();

  Eigen::array<int, 2> bcast = {1, N};

  for (auto _ : state) {
    result = col.broadcast(bcast);
    benchmark::DoNotOptimize(result.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * M * N * sizeof(Scalar));
}

// --- Broadcast + element-wise add (bias addition pattern) ---
static void BM_BroadcastAdd(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);

  Tensor<Scalar, 2> mat(M, N);
  Tensor<Scalar, 2> bias(1, N);
  Tensor<Scalar, 2> result(M, N);
  mat.setRandom();
  bias.setRandom();

  Eigen::array<int, 2> bcast = {M, 1};

  for (auto _ : state) {
    result = mat + bias.broadcast(bcast);
    benchmark::DoNotOptimize(result.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * M * N * sizeof(Scalar) * 2);
}

// --- Broadcast + scalar clamp (bias addition + clamp pattern) ---
// cwiseMax(Scalar) / cwiseMin(Scalar) build constant() leaves (unlike
// clip(), which is a single unary functor). Constants used to disable tiled
// evaluation for the whole expression.
static void BM_BroadcastClamp(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);

  Tensor<Scalar, 2> mat(M, N);
  Tensor<Scalar, 2> bias(1, N);
  Tensor<Scalar, 2> result(M, N);
  mat.setRandom();
  bias.setRandom();

  Eigen::array<int, 2> bcast = {M, 1};

  for (auto _ : state) {
    result = (mat + bias.broadcast(bcast)).cwiseMax(Scalar(-0.5)).cwiseMin(Scalar(0.5));
    benchmark::DoNotOptimize(result.data());
    benchmark::ClobberMemory();
  }

  // Validate outside the timed loop.
  for (int i = 0; i < M; ++i) {
    for (int j = 0; j < N; ++j) {
      const Scalar expected = std::min(Scalar(0.5), std::max(Scalar(-0.5), mat(i, j) + bias(0, j)));
      if (result(i, j) != expected) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  state.SetBytesProcessed(state.iterations() * M * N * sizeof(Scalar) * 2);
}

// --- Outer-dimension broadcast + clip ---
// Broadcasting a large plane along the outer-most dimension has a reuse
// distance of the whole plane: the untiled path streams the plane from
// memory B times, while tiled evaluation reuses cache-resident pieces
// across the broadcast factor. The cwiseMax/cwiseMin constants decide
// which of the two paths runs, so before nullary block support they forced
// the memory-bound path.
static void BM_OuterBroadcastClamp(benchmark::State& state) {
  const int D0 = state.range(0);
  const int D1 = state.range(1);
  const int B = state.range(2);

  Tensor<Scalar, 3> plane(D0, D1, 1);
  Tensor<Scalar, 3> result(D0, D1, B);
  plane.setRandom();

  Eigen::array<int, 3> bcast = {1, 1, B};

  for (auto _ : state) {
    result = plane.broadcast(bcast).cwiseMax(Scalar(-0.5)).cwiseMin(Scalar(0.5));
    benchmark::DoNotOptimize(result.data());
    benchmark::ClobberMemory();
  }

  for (int i = 0; i < D0; ++i) {
    for (int j = 0; j < D1; ++j) {
      const Scalar expected = std::min(Scalar(0.5), std::max(Scalar(-0.5), plane(i, j, 0)));
      for (int k = 0; k < B; ++k) {
        if (result(i, j, k) != expected) {
          state.SkipWithError("validation failed");
          return;
        }
      }
    }
  }
  state.SetBytesProcessed(state.iterations() * D0 * D1 * B * sizeof(Scalar));
}

// --- Transpose (shuffle) + scalar clamp ---
// A shuffled inner dimension makes the linear path a per-element gather with
// a full index-remapping walk, while the block path runs a cache-blocked
// transpose through TensorBlockIO. The cwiseMax/cwiseMin constants decide
// which path runs, so before nullary block support they forced the gather.
static void BM_ShuffleClamp(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);

  Tensor<Scalar, 2> mat(M, N);
  Tensor<Scalar, 2> result(N, M);
  mat.setRandom();

  Eigen::array<int, 2> transpose = {1, 0};

  for (auto _ : state) {
    result = mat.shuffle(transpose).cwiseMax(Scalar(-0.5)).cwiseMin(Scalar(0.5));
    benchmark::DoNotOptimize(result.data());
    benchmark::ClobberMemory();
  }

  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < M; ++j) {
      const Scalar expected = std::min(Scalar(0.5), std::max(Scalar(-0.5), mat(j, i)));
      if (result(i, j) != expected) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  state.SetBytesProcessed(state.iterations() * M * N * sizeof(Scalar));
}

// --- Pad + scalar clamp (convolution-style padding + clamp) ---
static void BM_PadClamp(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);
  const int P = state.range(2);

  Tensor<Scalar, 2> mat(M, N);
  Tensor<Scalar, 2> result(M + 2 * P, N + 2 * P);
  mat.setRandom();

  Eigen::array<std::pair<int, int>, 2> paddings;
  paddings[0] = {P, P};
  paddings[1] = {P, P};

  for (auto _ : state) {
    result = mat.pad(paddings).cwiseMax(Scalar(-0.5)).cwiseMin(Scalar(0.5));
    benchmark::DoNotOptimize(result.data());
    benchmark::ClobberMemory();
  }

  for (int i = 0; i < M + 2 * P; ++i) {
    for (int j = 0; j < N + 2 * P; ++j) {
      const bool interior = i >= P && i < M + P && j >= P && j < N + P;
      const Scalar padded = interior ? mat(i - P, j - P) : Scalar(0);
      const Scalar expected = std::min(Scalar(0.5), std::max(Scalar(-0.5), padded));
      if (result(i, j) != expected) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  state.SetBytesProcessed(state.iterations() * (M + 2 * P) * (N + 2 * P) * sizeof(Scalar));
}

// --- Constant multiply without any block-preferring node (control) ---
// No node prefers block access here, so this expression stays on the linear
// vectorized path; it pins that nullary block support does not change it.
static void BM_ConstantMul(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);

  Tensor<Scalar, 2> mat(M, N);
  Tensor<Scalar, 2> result(M, N);
  mat.setRandom();

  for (auto _ : state) {
    result = mat * mat.constant(Scalar(3));
    benchmark::DoNotOptimize(result.data());
    benchmark::ClobberMemory();
  }

  for (int i = 0; i < M; ++i) {
    for (int j = 0; j < N; ++j) {
      if (result(i, j) != Scalar(3) * mat(i, j)) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  state.SetBytesProcessed(state.iterations() * M * N * sizeof(Scalar) * 2);
}

// --- Rank-4 broadcast (batch x channels x 1 x 1) -> (batch x channels x H x W) ---
static void BM_BroadcastRank4(benchmark::State& state) {
  const int batch = state.range(0);
  const int C = state.range(1);
  const int H = state.range(2);

  Tensor<Scalar, 4> bias(batch, C, 1, 1);
  Tensor<Scalar, 4> result(batch, C, H, H);
  bias.setRandom();

  Eigen::array<int, 4> bcast = {1, 1, H, H};

  for (auto _ : state) {
    result = bias.broadcast(bcast);
    benchmark::DoNotOptimize(result.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * batch * C * H * H * sizeof(Scalar));
}

// --- ThreadPool variants ---

static void BM_BroadcastRow_ThreadPool(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);
  const int threads = state.range(2);

  Tensor<Scalar, 2> row(1, N);
  Tensor<Scalar, 2> result(M, N);
  row.setRandom();

  ThreadPool tp(threads);
  ThreadPoolDevice dev(&tp, threads);

  Eigen::array<int, 2> bcast = {M, 1};

  for (auto _ : state) {
    result.device(dev) = row.broadcast(bcast);
    benchmark::DoNotOptimize(result.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * M * N * sizeof(Scalar));
  state.counters["threads"] = threads;
}

static void BM_BroadcastAdd_ThreadPool(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);
  const int threads = state.range(2);

  Tensor<Scalar, 2> mat(M, N);
  Tensor<Scalar, 2> bias(1, N);
  Tensor<Scalar, 2> result(M, N);
  mat.setRandom();
  bias.setRandom();

  ThreadPool tp(threads);
  ThreadPoolDevice dev(&tp, threads);

  Eigen::array<int, 2> bcast = {M, 1};

  for (auto _ : state) {
    result.device(dev) = mat + bias.broadcast(bcast);
    benchmark::DoNotOptimize(result.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * M * N * sizeof(Scalar) * 2);
  state.counters["threads"] = threads;
}

// {m, n} and {batch, c, h}: pure Cartesian products.
#define BROADCAST_SIZES ->ArgsProduct({{64, 256, 1024}, {64, 256, 1024}})
#define BROADCAST_RANK4_SIZES ->ArgsProduct({{1, 8}, {64, 256}, {16, 32}})

// {size, size, threads}: explicit because size is repeated.
// clang-format off
#define BROADCAST_THREADPOOL_SIZES \
  ->Args({256, 256, 1})->Args({256, 256, 2})->Args({256, 256, 4}) \
  ->Args({256, 256, 8})->Args({256, 256, 12})->Args({256, 256, 16}) \
  ->Args({1024, 1024, 1})->Args({1024, 1024, 2})->Args({1024, 1024, 4}) \
  ->Args({1024, 1024, 8})->Args({1024, 1024, 12})->Args({1024, 1024, 16})
// clang-format on

BENCHMARK(BM_BroadcastRow) BROADCAST_SIZES;
BENCHMARK(BM_BroadcastCol) BROADCAST_SIZES;
BENCHMARK(BM_BroadcastAdd) BROADCAST_SIZES;
BENCHMARK(BM_BroadcastClamp) BROADCAST_SIZES;
BENCHMARK(BM_OuterBroadcastClamp)->ArgsProduct({{512, 1024}, {512, 1024}, {8}});
BENCHMARK(BM_ShuffleClamp)->ArgsProduct({{256, 1024, 4096}, {256, 1024, 4096}});
BENCHMARK(BM_PadClamp)->ArgsProduct({{256, 1024}, {256, 1024}, {1, 4}});
BENCHMARK(BM_ConstantMul) BROADCAST_SIZES;
BENCHMARK(BM_BroadcastRank4) BROADCAST_RANK4_SIZES;
BENCHMARK(BM_BroadcastRow_ThreadPool) BROADCAST_THREADPOOL_SIZES->UseRealTime();
BENCHMARK(BM_BroadcastAdd_ThreadPool) BROADCAST_THREADPOOL_SIZES->UseRealTime();
