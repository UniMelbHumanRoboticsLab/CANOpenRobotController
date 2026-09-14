// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>

using namespace Eigen;

static void BM_VecAdd(benchmark::State& state) {
  int size = state.range(0);
  VectorXf a = VectorXf::Random(size);
  VectorXf b = VectorXf::Random(size);
  for (auto _ : state) {
    a = a + b;
    benchmark::DoNotOptimize(a.data());
  }
  state.SetBytesProcessed(state.iterations() * size * sizeof(float) * 3);
}
BENCHMARK(BM_VecAdd)->RangeMultiplier(4)->Range(64, 1 << 20);

static void BM_MatAdd(benchmark::State& state) {
  int n = state.range(0);
  MatrixXf a = MatrixXf::Random(n, n);
  MatrixXf b = MatrixXf::Random(n, n);
  for (auto _ : state) {
    a = a + b;
    benchmark::DoNotOptimize(a.data());
  }
  state.SetBytesProcessed(state.iterations() * n * n * sizeof(float) * 3);
}
BENCHMARK(BM_MatAdd)->RangeMultiplier(2)->Range(8, 512);
