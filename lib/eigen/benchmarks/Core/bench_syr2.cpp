// Benchmarks for symmetric rank-2 update (SYR2).
//
// Tests C.selfadjointView<Lower>().rankUpdate(u, v, alpha) which computes
// C += alpha * u * v^T + conj(alpha) * v * u^T.
// Exercises SelfadjointRank2Update.h.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>

using namespace Eigen;

template <typename Scalar>
double syr2Flops(Index n) {
  // SYR2: 2 * n*(n+1)/2 multiply-adds ~ 2*n^2
  return (NumTraits<Scalar>::IsComplex ? 8.0 : 2.0) * 2 * n * (n + 1) / 2;
}

template <typename Scalar>
static void BM_SYR2_Lower(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Vec u = Vec::Random(n);
  Vec v = Vec::Random(n);
  Mat C = Mat::Zero(n, n);
  Scalar alpha(1);
  for (auto _ : state) {
    C.template selfadjointView<Lower>().rankUpdate(u, v, alpha);
    benchmark::DoNotOptimize(C.data());
    benchmark::ClobberMemory();
  }
  state.counters["GFLOPS"] = benchmark::Counter(syr2Flops<Scalar>(n), benchmark::Counter::kIsIterationInvariantRate,
                                                benchmark::Counter::kIs1000);
}

template <typename Scalar>
static void BM_SYR2_Upper(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Vec u = Vec::Random(n);
  Vec v = Vec::Random(n);
  Mat C = Mat::Zero(n, n);
  Scalar alpha(1);
  for (auto _ : state) {
    C.template selfadjointView<Upper>().rankUpdate(u, v, alpha);
    benchmark::DoNotOptimize(C.data());
    benchmark::ClobberMemory();
  }
  state.counters["GFLOPS"] = benchmark::Counter(syr2Flops<Scalar>(n), benchmark::Counter::kIsIterationInvariantRate,
                                                benchmark::Counter::kIs1000);
}

// clang-format off
#define SYR2_SIZES ->Arg(8)->Arg(16)->Arg(32)->Arg(64)->Arg(128)->Arg(256)->Arg(512)->Arg(1024)->Arg(2048)
BENCHMARK(BM_SYR2_Lower<float>) SYR2_SIZES ->Name("SYR2_Lower_float");
BENCHMARK(BM_SYR2_Lower<double>) SYR2_SIZES ->Name("SYR2_Lower_double");
BENCHMARK(BM_SYR2_Upper<float>) SYR2_SIZES ->Name("SYR2_Upper_float");
BENCHMARK(BM_SYR2_Upper<double>) SYR2_SIZES ->Name("SYR2_Upper_double");
#undef SYR2_SIZES
// clang-format on
