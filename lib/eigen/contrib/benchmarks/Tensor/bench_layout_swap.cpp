// Benchmarks for Eigen TensorLayoutSwap.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <contrib/Eigen/Tensor>

using namespace Eigen;

typedef float Scalar;

static void BM_LayoutSwap_2D(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);

  Tensor<Scalar, 2, ColMajor> A(M, N);
  A.setRandom();

  for (auto _ : state) {
    Tensor<Scalar, 2, RowMajor> B = A.swap_layout();
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  // 1 read (A) + 1 write (B).
  state.SetBytesProcessed(state.iterations() * 2ll * static_cast<int64_t>(M) * N * sizeof(Scalar));
}

static void BM_LayoutSwap_3D(benchmark::State& state) {
  const int D0 = state.range(0);
  const int D1 = state.range(1);
  const int D2 = state.range(2);

  Tensor<Scalar, 3, ColMajor> A(D0, D1, D2);
  A.setRandom();

  for (auto _ : state) {
    Tensor<Scalar, 3, RowMajor> B = A.swap_layout();
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  // 1 read (A) + 1 write (B).
  state.SetBytesProcessed(state.iterations() * 2ll * static_cast<int64_t>(D0) * D1 * D2 * sizeof(Scalar));
}

// Composing swap_layout with a coefficient-wise op forces evaluation through
// the executor and exercises any subsequent block consumers.
static void BM_LayoutSwap_Composed(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);

  Tensor<Scalar, 2, ColMajor> A(M, N);
  Tensor<Scalar, 2, ColMajor> B(M, N);
  A.setRandom();
  B.setRandom();

  for (auto _ : state) {
    Tensor<Scalar, 2, RowMajor> C = (A + B).swap_layout();
    benchmark::DoNotOptimize(C.data());
    benchmark::ClobberMemory();
  }
  // 2 reads (A, B) + 1 write (C).
  state.SetBytesProcessed(state.iterations() * 3ll * static_cast<int64_t>(M) * N * sizeof(Scalar));
}

// A transpose shuffle composed with swap_layout converts a matrix to the
// opposite storage order while keeping its logical indexing: B(i,j) == A(i,j).
// The shuffle serves materialized blocks that the swap re-wraps.
static void BM_LayoutSwap_Shuffle2D(benchmark::State& state) {
  const Index M = state.range(0);
  const Index N = state.range(1);

  Tensor<Scalar, 2, ColMajor> A(M, N);
  A.setRandom();
  Eigen::array<Index, 2> transpose = {1, 0};

  Tensor<Scalar, 2, RowMajor> B;
  for (auto _ : state) {
    B = A.shuffle(transpose).swap_layout();
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  for (Index i = 0; i < M; ++i) {
    for (Index j = 0; j < N; ++j) {
      if (B(i, j) != A(i, j)) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  // 1 read (A) + 1 write (B).
  state.SetBytesProcessed(state.iterations() * 2ll * static_cast<int64_t>(M) * N * sizeof(Scalar));
}

static void BM_LayoutSwap_Shuffle3D(benchmark::State& state) {
  const Index D0 = state.range(0);
  const Index D1 = state.range(1);
  const Index D2 = state.range(2);

  Tensor<Scalar, 3, ColMajor> A(D0, D1, D2);
  A.setRandom();
  Eigen::array<Index, 3> perm = {2, 0, 1};
  const Tensor<Scalar, 3, ColMajor> ref = A.shuffle(perm);

  Tensor<Scalar, 3, RowMajor> B;
  for (auto _ : state) {
    B = A.shuffle(perm).swap_layout();
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  for (Index i = 0; i < ref.dimension(0); ++i) {
    for (Index j = 0; j < ref.dimension(1); ++j) {
      for (Index k = 0; k < ref.dimension(2); ++k) {
        if (B(k, j, i) != ref(i, j, k)) {
          state.SkipWithError("validation failed");
          return;
        }
      }
    }
  }
  // 1 read (A) + 1 write (B).
  state.SetBytesProcessed(state.iterations() * 2ll * static_cast<int64_t>(D0) * D1 * D2 * sizeof(Scalar));
}

// A cwise op on top of the shuffle serves lazy blocks that the layout swap
// materializes itself.
static void BM_LayoutSwap_ShuffleCwise2D(benchmark::State& state) {
  const Index M = state.range(0);
  const Index N = state.range(1);

  Tensor<Scalar, 2, ColMajor> A(M, N);
  Tensor<Scalar, 2, ColMajor> bias(N, M);
  A.setRandom();
  bias.setRandom();
  Eigen::array<Index, 2> transpose = {1, 0};

  Tensor<Scalar, 2, RowMajor> B;
  for (auto _ : state) {
    B = (A.shuffle(transpose) + bias).swap_layout();
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  for (Index i = 0; i < M; ++i) {
    for (Index j = 0; j < N; ++j) {
      if (B(i, j) != A(i, j) + bias(j, i)) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  // 2 reads (A, bias) + 1 write (B).
  state.SetBytesProcessed(state.iterations() * 3ll * static_cast<int64_t>(M) * N * sizeof(Scalar));
}

// Write through a layout-swapped destination from a block-preferring source.
static void BM_LayoutSwap_Write2D(benchmark::State& state) {
  const Index M = state.range(0);
  const Index N = state.range(1);

  Tensor<Scalar, 2, ColMajor> A(M, N);
  Tensor<Scalar, 2, ColMajor> bias(N, M);
  A.setRandom();
  bias.setRandom();
  Eigen::array<Index, 2> transpose = {1, 0};

  Tensor<Scalar, 2, RowMajor> C(M, N);
  C.setZero();
  for (auto _ : state) {
    C.swap_layout() = A.shuffle(transpose) + bias;
    benchmark::DoNotOptimize(C.data());
    benchmark::ClobberMemory();
  }
  for (Index i = 0; i < M; ++i) {
    for (Index j = 0; j < N; ++j) {
      if (C(i, j) != A(i, j) + bias(j, i)) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  // 2 reads (A, bias) + 1 write (C).
  state.SetBytesProcessed(state.iterations() * 3ll * static_cast<int64_t>(M) * N * sizeof(Scalar));
}

// {n, n} and {n, n, n}: explicit because dims are repeated.
#define LAYOUT_SWAP_SIZES ->Args({64, 64})->Args({256, 256})->Args({1024, 1024})
#define LAYOUT_SWAP_3D_SIZES ->Args({32, 32, 32})->Args({64, 64, 64})->Args({128, 128, 128})

BENCHMARK(BM_LayoutSwap_2D) LAYOUT_SWAP_SIZES;
BENCHMARK(BM_LayoutSwap_3D) LAYOUT_SWAP_3D_SIZES;
BENCHMARK(BM_LayoutSwap_Composed) LAYOUT_SWAP_SIZES;
BENCHMARK(BM_LayoutSwap_Shuffle2D) LAYOUT_SWAP_SIZES;
BENCHMARK(BM_LayoutSwap_Shuffle3D) LAYOUT_SWAP_3D_SIZES;
BENCHMARK(BM_LayoutSwap_ShuffleCwise2D) LAYOUT_SWAP_SIZES;
BENCHMARK(BM_LayoutSwap_Write2D) LAYOUT_SWAP_SIZES;
