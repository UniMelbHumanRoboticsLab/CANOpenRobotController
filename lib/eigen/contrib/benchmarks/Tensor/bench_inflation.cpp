// Benchmarks for Eigen TensorInflation (zero-interleaved upsampling views).
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <contrib/Eigen/Tensor>

using namespace Eigen;

typedef float Scalar;

// --- Inflate only the inner-most (contiguous) dimension. ---
static void BM_Inflate2D_Inner(benchmark::State& state) {
  const Index M = state.range(0);
  const Index N = state.range(1);
  const Index S = state.range(2);

  Tensor<Scalar, 2> A(M, N);
  A.setRandom();
  Eigen::array<Index, 2> strides = {S, 1};

  Tensor<Scalar, 2> B;
  for (auto _ : state) {
    B = A.inflate(strides);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  for (Index i = 0; i < B.dimension(0); ++i) {
    for (Index j = 0; j < B.dimension(1); ++j) {
      const Scalar expected = (i % S == 0) ? A(i / S, j) : Scalar(0);
      if (B(i, j) != expected) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  state.SetItemsProcessed(state.iterations() * B.size());
}

// --- Inflate only an outer dimension. Inner dim stays contiguous. ---
static void BM_Inflate2D_Outer(benchmark::State& state) {
  const Index M = state.range(0);
  const Index N = state.range(1);
  const Index S = state.range(2);

  Tensor<Scalar, 2> A(M, N);
  A.setRandom();
  Eigen::array<Index, 2> strides = {1, S};

  Tensor<Scalar, 2> B;
  for (auto _ : state) {
    B = A.inflate(strides);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  for (Index i = 0; i < B.dimension(0); ++i) {
    for (Index j = 0; j < B.dimension(1); ++j) {
      const Scalar expected = (j % S == 0) ? A(i, j / S) : Scalar(0);
      if (B(i, j) != expected) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  state.SetItemsProcessed(state.iterations() * B.size());
}

// --- Inflate every dimension. ---
static void BM_Inflate2D_All(benchmark::State& state) {
  const Index M = state.range(0);
  const Index N = state.range(1);
  const Index S = state.range(2);

  Tensor<Scalar, 2> A(M, N);
  A.setRandom();
  Eigen::array<Index, 2> strides = {S, S};

  Tensor<Scalar, 2> B;
  for (auto _ : state) {
    B = A.inflate(strides);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  for (Index i = 0; i < B.dimension(0); ++i) {
    for (Index j = 0; j < B.dimension(1); ++j) {
      const Scalar expected = (i % S == 0 && j % S == 0) ? A(i / S, j / S) : Scalar(0);
      if (B(i, j) != expected) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  state.SetItemsProcessed(state.iterations() * B.size());
}

// --- Expression-sourced inflation (no raw buffer behind the argument) ---
static void BM_Inflate2D_Expr(benchmark::State& state) {
  const Index M = state.range(0);
  const Index N = state.range(1);
  const Index S = state.range(2);

  Tensor<Scalar, 2> A(M, N);
  A.setRandom();
  Eigen::array<Index, 2> strides = {S, S};

  Tensor<Scalar, 2> B;
  for (auto _ : state) {
    B = (A + A.constant(Scalar(1))).inflate(strides);
    benchmark::DoNotOptimize(B.data());
    benchmark::ClobberMemory();
  }
  for (Index i = 0; i < B.dimension(0); ++i) {
    for (Index j = 0; j < B.dimension(1); ++j) {
      const Scalar expected = (i % S == 0 && j % S == 0) ? A(i / S, j / S) + Scalar(1) : Scalar(0);
      if (B(i, j) != expected) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  state.SetItemsProcessed(state.iterations() * B.size());
}

// clang-format off
#define INFLATE_SIZES ->ArgsProduct({{256, 1024}, {1024}, {2, 4}})
// clang-format on

BENCHMARK(BM_Inflate2D_Inner) INFLATE_SIZES;
BENCHMARK(BM_Inflate2D_Outer) INFLATE_SIZES;
BENCHMARK(BM_Inflate2D_All) INFLATE_SIZES;
BENCHMARK(BM_Inflate2D_Expr) INFLATE_SIZES;
