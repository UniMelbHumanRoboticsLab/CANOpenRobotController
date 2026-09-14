// Benchmarks for the DiagonalPlusLowRank operator: the implicit O(nk) product
// and the O(nk^2 + k^3) Woodbury solve against their dense equivalents (the
// O(n^2) materialized product and the O(n^3) LU factor-and-solve). The solve
// benchmarks time factorization plus one right-hand side on both sides, the
// use case the Woodbury identity accelerates.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <contrib/Eigen/StructuredMatrices>

using namespace Eigen;

typedef Matrix<double, Dynamic, 1> Vec;
typedef Matrix<double, Dynamic, Dynamic> Mat;

// Well-conditioned operator: diagonal bounded away from zero and a contractive
// correction, so the capacitance matrix is safely invertible.
static DiagonalPlusLowRank<double> makeOperator(Index n, Index k) {
  Vec d = Vec::Random(n).array() + 3.0;
  Mat U = 0.5 * Mat::Random(n, k);
  Mat V = 0.5 * Mat::Random(n, k);
  return DiagonalPlusLowRank<double>(d, U, V);
}

// --- Matrix-vector product: implicit O(nk) vs. materialized dense O(n^2) ---
static void BM_DplrProduct(benchmark::State& state) {
  const Index n = state.range(0), k = state.range(1);
  DiagonalPlusLowRank<double> A = makeOperator(n, k);
  Vec x = Vec::Random(n), y(n);
  for (auto _ : state) {
    y.noalias() = A * x;
    benchmark::DoNotOptimize(y.data());
  }
}
BENCHMARK(BM_DplrProduct)->ArgsProduct({{256, 1024, 4096}, {4, 32}});

static void BM_DenseProduct(benchmark::State& state) {
  const Index n = state.range(0), k = state.range(1);
  Mat dense = makeOperator(n, k);
  Vec x = Vec::Random(n), y(n);
  for (auto _ : state) {
    y.noalias() = dense * x;
    benchmark::DoNotOptimize(y.data());
  }
}
BENCHMARK(BM_DenseProduct)->ArgsProduct({{256, 1024, 4096}, {4, 32}});

// --- Solve: Woodbury (k x k capacitance LU) vs. dense n x n LU ---
static void BM_DplrSolve(benchmark::State& state) {
  const Index n = state.range(0), k = state.range(1);
  DiagonalPlusLowRank<double> A = makeOperator(n, k);
  Vec b = Vec::Random(n), x(n);
  for (auto _ : state) {
    x = A.solve(b);  // factors the k x k capacitance matrix each iteration
    benchmark::DoNotOptimize(x.data());
  }
}
BENCHMARK(BM_DplrSolve)->ArgsProduct({{256, 1024, 4096}, {4, 32}});

static void BM_DenseLuSolve(benchmark::State& state) {
  const Index n = state.range(0), k = state.range(1);
  Mat dense = makeOperator(n, k);
  Vec b = Vec::Random(n), x(n);
  for (auto _ : state) {
    x = dense.partialPivLu().solve(b);  // factors the n x n matrix each iteration
    benchmark::DoNotOptimize(x.data());
  }
}
BENCHMARK(BM_DenseLuSolve)->ArgsProduct({{256, 1024, 4096}, {4, 32}});
