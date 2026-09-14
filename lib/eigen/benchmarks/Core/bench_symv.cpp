// Benchmarks for selfadjoint matrix-vector product (SYMV/HEMV).
//
// Tests y += selfadjointView(A) * x for various sizes and scalar types.
// Exercises SelfadjointMatrixVector.h kernel.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>

using namespace Eigen;

template <typename Scalar>
double symvFlops(Index n) {
  // SYMV uses n^2 multiply-adds (exploiting symmetry)
  return (NumTraits<Scalar>::IsComplex ? 8.0 : 2.0) * n * n;
}

// y += selfadjointView<Lower>(A) * x
template <typename Scalar>
static void BM_SYMV_Lower(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Mat A = Mat::Random(n, n);
  A = (A + A.transpose().eval()) / Scalar(2);
  Vec x = Vec::Random(n);
  Vec y = Vec::Random(n);
  for (auto _ : state) {
    y.noalias() += A.template selfadjointView<Lower>() * x;
    benchmark::DoNotOptimize(y.data());
    benchmark::ClobberMemory();
  }
  state.counters["GFLOPS"] = benchmark::Counter(symvFlops<Scalar>(n), benchmark::Counter::kIsIterationInvariantRate,
                                                benchmark::Counter::kIs1000);
}

// y += selfadjointView<Upper>(A) * x
template <typename Scalar>
static void BM_SYMV_Upper(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Mat A = Mat::Random(n, n);
  A = (A + A.transpose().eval()) / Scalar(2);
  Vec x = Vec::Random(n);
  Vec y = Vec::Random(n);
  for (auto _ : state) {
    y.noalias() += A.template selfadjointView<Upper>() * x;
    benchmark::DoNotOptimize(y.data());
    benchmark::ClobberMemory();
  }
  state.counters["GFLOPS"] = benchmark::Counter(symvFlops<Scalar>(n), benchmark::Counter::kIsIterationInvariantRate,
                                                benchmark::Counter::kIs1000);
}

// clang-format off
#define SYMV_SIZES ->Arg(8)->Arg(16)->Arg(32)->Arg(64)->Arg(128)->Arg(256)->Arg(512)->Arg(1024)->Arg(2048)
BENCHMARK(BM_SYMV_Lower<float>) SYMV_SIZES ->Name("SYMV_Lower_float");
BENCHMARK(BM_SYMV_Lower<double>) SYMV_SIZES ->Name("SYMV_Lower_double");
BENCHMARK(BM_SYMV_Upper<float>) SYMV_SIZES ->Name("SYMV_Upper_float");
BENCHMARK(BM_SYMV_Upper<double>) SYMV_SIZES ->Name("SYMV_Upper_double");
#undef SYMV_SIZES
// clang-format on
