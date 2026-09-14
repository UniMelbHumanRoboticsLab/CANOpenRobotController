// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>

using namespace Eigen;

static void BM_MatrixReverse(benchmark::State& state) {
  int n = state.range(0);
  typedef Matrix<double, Dynamic, Dynamic> MatrixType;
  MatrixType a = MatrixType::Random(n, n);
  MatrixType b(n, n);
  for (auto _ : state) {
    b = a.reverse();
    benchmark::DoNotOptimize(b.data());
  }
  state.SetBytesProcessed(state.iterations() * n * n * sizeof(double));
}
BENCHMARK(BM_MatrixReverse)->RangeMultiplier(2)->Range(4, 512);

static void BM_VectorReverse(benchmark::State& state) {
  int n = state.range(0);
  typedef Matrix<double, Dynamic, 1> VectorType;
  VectorType a = VectorType::Random(n);
  VectorType b(n);
  for (auto _ : state) {
    b = a.reverse();
    benchmark::DoNotOptimize(b.data());
  }
  state.SetBytesProcessed(state.iterations() * n * sizeof(double));
}
BENCHMARK(BM_VectorReverse)->RangeMultiplier(4)->Range(16, 1 << 18);
