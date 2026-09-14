// Benchmarks for dot product (BLAS-1 critical path).
//
// Flop count: 2n for real, 8n for complex.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>

using namespace Eigen;

template <typename Scalar>
double dotFlops(Index n) {
  return (NumTraits<Scalar>::IsComplex ? 8.0 : 2.0) * n;
}

template <typename Scalar>
static void BM_Dot(benchmark::State& state) {
  const Index n = state.range(0);
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Vec a = Vec::Random(n);
  Vec b = Vec::Random(n);
  for (auto _ : state) {
    Scalar d = a.dot(b);
    benchmark::DoNotOptimize(d);
  }
  state.counters["GFLOPS"] = benchmark::Counter(dotFlops<Scalar>(n), benchmark::Counter::kIsIterationInvariantRate,
                                                benchmark::Counter::kIs1000);
}

// clang-format off
#define DOT_SIZES ->Arg(64)->Arg(256)->Arg(1024)->Arg(4096)->Arg(16384)->Arg(65536)->Arg(262144)->Arg(1048576)
BENCHMARK(BM_Dot<float>) DOT_SIZES ->Name("Dot_float");
BENCHMARK(BM_Dot<double>) DOT_SIZES ->Name("Dot_double");
BENCHMARK(BM_Dot<std::complex<float>>) DOT_SIZES ->Name("Dot_cfloat");
BENCHMARK(BM_Dot<std::complex<double>>) DOT_SIZES ->Name("Dot_cdouble");
#undef DOT_SIZES
// clang-format on
