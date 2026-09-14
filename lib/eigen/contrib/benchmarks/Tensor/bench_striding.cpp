// Benchmarks for Eigen Tensor striding (subsampling views).
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <contrib/Eigen/Tensor>

using namespace Eigen;

typedef float Scalar;

// --- Rank-2 read, outer dimension strided (inner runs stay contiguous) ---
static void BM_StrideReadOuter2D(benchmark::State& state) {
  const Index M = state.range(0);
  const Index N = state.range(1);
  const Index S = state.range(2);

  Tensor<Scalar, 2> A(M, N);
  A.setRandom();
  Eigen::array<Index, 2> strides = {1, S};  // ColMajor: dimension 0 is inner.

  Tensor<Scalar, 2> B;
  for (auto _ : state) {
    B = A.stride(strides);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  for (Index i = 0; i < B.dimension(0); ++i) {
    for (Index j = 0; j < B.dimension(1); ++j) {
      if (B(i, j) != A(i, j * S)) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  state.SetItemsProcessed(state.iterations() * B.size());
}

// --- Rank-2 read, inner dimension strided (gather within inner runs) ---
static void BM_StrideReadInner2D(benchmark::State& state) {
  const Index M = state.range(0);
  const Index N = state.range(1);
  const Index S = state.range(2);

  Tensor<Scalar, 2> A(M, N);
  A.setRandom();
  Eigen::array<Index, 2> strides = {S, 1};

  Tensor<Scalar, 2> B;
  for (auto _ : state) {
    B = A.stride(strides);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  for (Index i = 0; i < B.dimension(0); ++i) {
    for (Index j = 0; j < B.dimension(1); ++j) {
      if (B(i, j) != A(i * S, j)) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  state.SetItemsProcessed(state.iterations() * B.size());
}

// --- Identity striding (all strides 1, measures overhead) ---
static void BM_StrideIdentity2D(benchmark::State& state) {
  const Index M = state.range(0);
  const Index N = state.range(1);

  Tensor<Scalar, 2> A(M, N);
  A.setRandom();
  Eigen::array<Index, 2> strides = {1, 1};

  Tensor<Scalar, 2> B;
  for (auto _ : state) {
    B = A.stride(strides);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  for (Index i = 0; i < B.dimension(0); ++i) {
    for (Index j = 0; j < B.dimension(1); ++j) {
      if (B(i, j) != A(i, j)) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  state.SetItemsProcessed(state.iterations() * B.size());
}

// --- Rank-3 read, all dimensions strided ---
static void BM_StrideRead3D(benchmark::State& state) {
  const Index D0 = state.range(0);
  const Index D1 = state.range(1);
  const Index D2 = state.range(2);

  Tensor<Scalar, 3> A(D0, D1, D2);
  A.setRandom();
  Eigen::array<Index, 3> strides = {2, 2, 2};

  Tensor<Scalar, 3> B;
  for (auto _ : state) {
    B = A.stride(strides);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  for (Index i = 0; i < B.dimension(0); ++i) {
    for (Index j = 0; j < B.dimension(1); ++j) {
      for (Index k = 0; k < B.dimension(2); ++k) {
        if (B(i, j, k) != A(i * 2, j * 2, k * 2)) {
          state.SkipWithError("validation failed");
          return;
        }
      }
    }
  }
  state.SetItemsProcessed(state.iterations() * B.size());
}

// --- Expression-sourced read (no raw buffer behind the striding) ---
static void BM_StrideExprInner2D(benchmark::State& state) {
  const Index M = state.range(0);
  const Index N = state.range(1);
  const Index S = state.range(2);

  Tensor<Scalar, 2> A(M, N);
  A.setRandom();
  Eigen::array<Index, 2> strides = {S, 1};

  Tensor<Scalar, 2> B;
  for (auto _ : state) {
    B = (A + A.constant(Scalar(1))).stride(strides);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  for (Index i = 0; i < B.dimension(0); ++i) {
    for (Index j = 0; j < B.dimension(1); ++j) {
      if (B(i, j) != A(i * S, j) + Scalar(1)) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  state.SetItemsProcessed(state.iterations() * B.size());
}

// --- Rank-2 write through a strided destination ---
static void BM_StrideWriteOuter2D(benchmark::State& state) {
  const Index M = state.range(0);
  const Index N = state.range(1);
  const Index S = state.range(2);

  Tensor<Scalar, 2> A(M, (N + S - 1) / S);
  A.setRandom();
  Tensor<Scalar, 2> C(M, N);
  C.setZero();
  Eigen::array<Index, 2> strides = {1, S};

  for (auto _ : state) {
    C.stride(strides) = A;
    benchmark::DoNotOptimize(C.data());
    benchmark::ClobberMemory();
  }
  for (Index i = 0; i < A.dimension(0); ++i) {
    for (Index j = 0; j < A.dimension(1); ++j) {
      if (C(i, j * S) != A(i, j)) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  state.SetItemsProcessed(state.iterations() * A.size());
}

static void BM_StrideWriteInner2D(benchmark::State& state) {
  const Index M = state.range(0);
  const Index N = state.range(1);
  const Index S = state.range(2);

  Tensor<Scalar, 2> A((M + S - 1) / S, N);
  A.setRandom();
  Tensor<Scalar, 2> C(M, N);
  C.setZero();
  Eigen::array<Index, 2> strides = {S, 1};

  for (auto _ : state) {
    C.stride(strides) = A;
    benchmark::DoNotOptimize(C.data());
    benchmark::ClobberMemory();
  }
  for (Index i = 0; i < A.dimension(0); ++i) {
    for (Index j = 0; j < A.dimension(1); ++j) {
      if (C(i * S, j) != A(i, j)) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  state.SetItemsProcessed(state.iterations() * A.size());
}

// clang-format off
#define STRIDE_2D_SIZES ->ArgsProduct({{256, 1024}, {1024}, {2, 4}})
#define STRIDE_2D_PLAIN_SIZES ->Args({256, 1024})->Args({1024, 1024})
#define STRIDE_3D_SIZES ->Args({64, 64, 64})->Args({128, 128, 64})
// clang-format on

BENCHMARK(BM_StrideReadOuter2D) STRIDE_2D_SIZES;
BENCHMARK(BM_StrideReadInner2D) STRIDE_2D_SIZES;
BENCHMARK(BM_StrideIdentity2D) STRIDE_2D_PLAIN_SIZES;
BENCHMARK(BM_StrideRead3D) STRIDE_3D_SIZES;
BENCHMARK(BM_StrideExprInner2D) STRIDE_2D_SIZES;
BENCHMARK(BM_StrideWriteOuter2D) STRIDE_2D_SIZES;
BENCHMARK(BM_StrideWriteInner2D) STRIDE_2D_SIZES;
