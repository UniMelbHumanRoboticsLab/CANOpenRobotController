// Benchmarks for triangular-dense matrix products (TRMM).
//
// Tests C = triangular(A) * B for various modes (Lower/Upper) and sides (Left/Right).
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>

using namespace Eigen;

// C = triangularView<Mode>(A) * B
template <typename Scalar, unsigned int Mode>
static void BM_TRMM_Left(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(n, n);
  Mat B = Mat::Random(n, n);
  Mat C(n, n);
  for (auto _ : state) {
    C.noalias() = A.template triangularView<Mode>() * B;
    benchmark::DoNotOptimize(C.data());
  }
  state.counters["GFLOPS"] =
      benchmark::Counter(1.0 * n * n * n, benchmark::Counter::kIsIterationInvariantRate, benchmark::Counter::kIs1000);
}

// C = B * triangularView<Mode>(A)
template <typename Scalar, unsigned int Mode>
static void BM_TRMM_Right(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(n, n);
  Mat B = Mat::Random(n, n);
  Mat C(n, n);
  for (auto _ : state) {
    C.noalias() = B * A.template triangularView<Mode>();
    benchmark::DoNotOptimize(C.data());
  }
  state.counters["GFLOPS"] =
      benchmark::Counter(1.0 * n * n * n, benchmark::Counter::kIsIterationInvariantRate, benchmark::Counter::kIs1000);
}

// clang-format off
#define TRMM_SIZES ->Arg(64)->Arg(128)->Arg(256)->Arg(512)->Arg(1024)
// Left product
BENCHMARK(BM_TRMM_Left<float, Lower>) TRMM_SIZES ->Name("TRMM_Left_float_Lower");
BENCHMARK(BM_TRMM_Left<float, Upper>) TRMM_SIZES ->Name("TRMM_Left_float_Upper");
BENCHMARK(BM_TRMM_Left<double, Lower>) TRMM_SIZES ->Name("TRMM_Left_double_Lower");
BENCHMARK(BM_TRMM_Left<double, Upper>) TRMM_SIZES ->Name("TRMM_Left_double_Upper");
// Right product
BENCHMARK(BM_TRMM_Right<float, Lower>) TRMM_SIZES ->Name("TRMM_Right_float_Lower");
BENCHMARK(BM_TRMM_Right<float, Upper>) TRMM_SIZES ->Name("TRMM_Right_float_Upper");
BENCHMARK(BM_TRMM_Right<double, Lower>) TRMM_SIZES ->Name("TRMM_Right_double_Lower");
BENCHMARK(BM_TRMM_Right<double, Upper>) TRMM_SIZES ->Name("TRMM_Right_double_Upper");
#undef TRMM_SIZES
// clang-format on
