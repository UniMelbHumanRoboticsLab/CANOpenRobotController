// Benchmarks for the fast Hankel matrix-vector product: the structured operator
// (O(n log n) via a cached DFT symbol, with direct O(n) dispatch for single-row
// and single-column shapes) against the equivalent dense GEMV. The *Dense
// variants evaluate the same product through a materialized dense matrix.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>
#include <contrib/Eigen/StructuredMatrices>

using namespace Eigen;

typedef Matrix<double, Dynamic, 1> Vec;
typedef Matrix<double, Dynamic, Dynamic> Mat;

// --- Square n x n product: FFT-based operator vs dense GEMV ---
static void BM_HankelProduct(benchmark::State& state) {
  const Index n = state.range(0);
  Vec h = Vec::Random(2 * n - 1);
  Hankel<double> H(h.head(n), h.tail(n));  // the FFT symbol is paid once, here
  Vec x = Vec::Random(n), y(n);
  for (auto _ : state) {
    y.noalias() = H * x;
    benchmark::DoNotOptimize(y.data());
  }
}
BENCHMARK(BM_HankelProduct)->Arg(256)->Arg(1024)->Arg(4096)->Arg(16384);

static void BM_HankelProductDense(benchmark::State& state) {
  const Index n = state.range(0);
  Vec h = Vec::Random(2 * n - 1);
  Mat dense = Hankel<double>(h.head(n), h.tail(n));
  Vec x = Vec::Random(n), y(n);
  for (auto _ : state) {
    y.noalias() = dense * x;
    benchmark::DoNotOptimize(y.data());
  }
}
BENCHMARK(BM_HankelProductDense)->Arg(256)->Arg(1024)->Arg(4096)->Arg(16384);

// --- Skinny 1 x n: dispatched to a direct O(n) dot-product kernel, not the FFT
// (whose O(n log n) cost plus setup is asymptotically worse than direct here) ---
static void BM_HankelProductSkinnyRow(benchmark::State& state) {
  const Index n = state.range(0);
  Vec h = Vec::Random(n);  // generating sequence of the 1 x n operator
  Hankel<double> H(h.head(1), h.tail(n));
  Vec x = Vec::Random(n);
  Matrix<double, 1, 1> y;
  for (auto _ : state) {
    y.noalias() = H * x;
    benchmark::DoNotOptimize(y.data());
  }
}
BENCHMARK(BM_HankelProductSkinnyRow)->Arg(4096)->Arg(65536)->Arg(1048576);

static void BM_HankelProductSkinnyRowDense(benchmark::State& state) {
  const Index n = state.range(0);
  Vec h = Vec::Random(n);
  Mat dense = Hankel<double>(h.head(1), h.tail(n));
  Vec x = Vec::Random(n);
  Matrix<double, 1, 1> y;
  for (auto _ : state) {
    y.noalias() = dense * x;
    benchmark::DoNotOptimize(y.data());
  }
}
BENCHMARK(BM_HankelProductSkinnyRowDense)->Arg(4096)->Arg(65536)->Arg(1048576);

// --- Skinny n x 1: a single axpy of the generating sequence ---
static void BM_HankelProductSkinnyCol(benchmark::State& state) {
  const Index n = state.range(0);
  Vec h = Vec::Random(n);  // generating sequence of the n x 1 operator
  Hankel<double> H(h.head(n), h.tail(1));
  Matrix<double, 1, 1> x = Matrix<double, 1, 1>::Random();
  Vec y(n);
  for (auto _ : state) {
    y.noalias() = H * x;
    benchmark::DoNotOptimize(y.data());
  }
}
BENCHMARK(BM_HankelProductSkinnyCol)->Arg(4096)->Arg(65536)->Arg(1048576);

static void BM_HankelProductSkinnyColDense(benchmark::State& state) {
  const Index n = state.range(0);
  Vec h = Vec::Random(n);
  Mat dense = Hankel<double>(h.head(n), h.tail(1));
  Matrix<double, 1, 1> x = Matrix<double, 1, 1>::Random();
  Vec y(n);
  for (auto _ : state) {
    y.noalias() = dense * x;
    benchmark::DoNotOptimize(y.data());
  }
}
BENCHMARK(BM_HankelProductSkinnyColDense)->Arg(4096)->Arg(65536)->Arg(1048576);
