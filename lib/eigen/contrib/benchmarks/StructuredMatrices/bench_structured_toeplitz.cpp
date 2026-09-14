// Benchmarks for the Toeplitz operator's products across its dispatch tiers and
// shapes (square, tall, wide; 5-smooth and prime embedding sizes) against the
// dense product, plus the one-time circulant-embedding construction.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Dense>
#include <contrib/Eigen/StructuredMatrices>

using namespace Eigen;

typedef Matrix<double, Dynamic, 1> Vec;
typedef Matrix<double, Dynamic, Dynamic> Mat;
typedef Matrix<std::complex<double>, Dynamic, 1> CVec;

// --- Construction: the one-time embedding symbol FFT ---
// 97 exercises the 5-smooth padding of the embedding (2*97-1 = 193 is prime).
static void BM_ToeplitzConstruct(benchmark::State& state) {
  const Index n = state.range(0);
  Vec c = Vec::Random(n), r = Vec::Random(n);
  for (auto _ : state) {
    Toeplitz<double> T(c, r);
    benchmark::DoNotOptimize(T);
  }
}
BENCHMARK(BM_ToeplitzConstruct)->Arg(96)->Arg(97)->Arg(1024)->Arg(4096);

// --- Square product y = T * x across the dispatch tiers ---
// 8: scalar loop; 32: segment tier boundary; larger: FFT via the embedding.
static void BM_ToeplitzProduct(benchmark::State& state) {
  const Index n = state.range(0);
  Vec c = Vec::Random(n), r = Vec::Random(n), x = Vec::Random(n), y(n);
  Toeplitz<double> T(c, r);
  for (auto _ : state) {
    y.noalias() = T * x;
    benchmark::DoNotOptimize(y.data());
  }
}
BENCHMARK(BM_ToeplitzProduct)->Arg(8)->Arg(32)->Arg(96)->Arg(1024)->Arg(4096);

static void BM_ToeplitzProductDense(benchmark::State& state) {
  const Index n = state.range(0);
  Vec c = Vec::Random(n), r = Vec::Random(n), x = Vec::Random(n), y(n);
  Mat dense = Toeplitz<double>(c, r);
  for (auto _ : state) {
    y.noalias() = dense * x;
    benchmark::DoNotOptimize(y.data());
  }
}
BENCHMARK(BM_ToeplitzProductDense)->Arg(96)->Arg(1024)->Arg(4096);

// --- Rectangular products: tall (2n x n) and wide (n x 2n) ---
static void BM_ToeplitzProductTall(benchmark::State& state) {
  const Index n = state.range(0);
  Vec c = Vec::Random(2 * n), r = Vec::Random(n), x = Vec::Random(n), y(2 * n);
  Toeplitz<double> T(c, r);
  for (auto _ : state) {
    y.noalias() = T * x;
    benchmark::DoNotOptimize(y.data());
  }
}
BENCHMARK(BM_ToeplitzProductTall)->Arg(96)->Arg(1024)->Arg(4096);

static void BM_ToeplitzProductWide(benchmark::State& state) {
  const Index n = state.range(0);
  Vec c = Vec::Random(n), r = Vec::Random(2 * n), x = Vec::Random(2 * n), y(n);
  Toeplitz<double> T(c, r);
  for (auto _ : state) {
    y.noalias() = T * x;
    benchmark::DoNotOptimize(y.data());
  }
}
BENCHMARK(BM_ToeplitzProductWide)->Arg(96)->Arg(1024)->Arg(4096);

static void BM_ToeplitzProductComplex(benchmark::State& state) {
  const Index n = state.range(0);
  CVec c = CVec::Random(n), r = CVec::Random(n), x = CVec::Random(n), y(n);
  Toeplitz<std::complex<double>> T(c, r);
  for (auto _ : state) {
    y.noalias() = T * x;
    benchmark::DoNotOptimize(y.data());
  }
}
BENCHMARK(BM_ToeplitzProductComplex)->Arg(96)->Arg(4096);

// Multi-column right-hand sides: one FFT round trip per column.
static void BM_ToeplitzProductMultiRhs(benchmark::State& state) {
  const Index n = state.range(0), k = state.range(1);
  Vec c = Vec::Random(n), r = Vec::Random(n);
  Mat X = Mat::Random(n, k), Y(n, k);
  Toeplitz<double> T(c, r);
  for (auto _ : state) {
    Y.noalias() = T * X;
    benchmark::DoNotOptimize(Y.data());
  }
}
BENCHMARK(BM_ToeplitzProductMultiRhs)->ArgsProduct({{96, 4096}, {8}});
