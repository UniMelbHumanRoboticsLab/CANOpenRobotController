// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Sparse>
#include <cstdint>
#include <random>

using namespace Eigen;

typedef double Scalar;
typedef Matrix<Scalar, Dynamic, Dynamic> DenseMat;
typedef SparseMatrix<Scalar> SpMat;

static DenseMat makeDense(int rows, int cols, double density, std::uint64_t seed) {
  DenseMat dense = DenseMat::Zero(rows, cols);
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<Scalar> coin(0.0, 1.0);
  std::uniform_real_distribution<Scalar> value(-1.0, 1.0);
  for (int j = 0; j < cols; ++j) {
    for (int i = 0; i < rows; ++i) {
      if (coin(rng) < density) dense(i, j) = value(rng);
    }
  }
  return dense;
}

static void BM_SparseViewAssign(benchmark::State& state) {
  const int rows = static_cast<int>(state.range(0));
  const int cols = static_cast<int>(state.range(1));
  const double density = static_cast<double>(state.range(2)) / 10000.0;
  const DenseMat dense = makeDense(rows, cols, density, 0xC0FFEEu);
  SpMat result;

  for (auto _ : state) {
    result = dense.sparseView();
    benchmark::DoNotOptimize(result.valuePtr());
    benchmark::ClobberMemory();
  }

  state.counters["density%"] = density * 100.0;
  state.counters["nnz"] = result.nonZeros();
}

// Args: {rows, cols, density*10000}: 0%, 1%, 10%, 50%, 100%.
BENCHMARK(BM_SparseViewAssign)->ArgsProduct({{500}, {1, 8, 500}, {0, 100, 1000, 5000, 10000}});
