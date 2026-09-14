// Benchmarks for triangular matrix-vector product (TRMV).
//
// Tests y += triangularView(A) * x for various modes and sizes.
// Exercises TriangularMatrixVector.h kernel.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>

using namespace Eigen;

template <typename Scalar>
double trmvFlops(Index n) {
  // TRMV: ~n^2 multiply-adds
  return (NumTraits<Scalar>::IsComplex ? 8.0 : 2.0) * n * n;
}

// y = triangularView<Mode>(A) * x
template <typename Scalar, unsigned int Mode>
static void BM_TRMV(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Mat A = Mat::Random(n, n);
  Vec x = Vec::Random(n);
  Vec y(n);
  for (auto _ : state) {
    y.noalias() = A.template triangularView<Mode>() * x;
    benchmark::DoNotOptimize(y.data());
    benchmark::ClobberMemory();
  }
  state.counters["GFLOPS"] = benchmark::Counter(trmvFlops<Scalar>(n), benchmark::Counter::kIsIterationInvariantRate,
                                                benchmark::Counter::kIs1000);
}

// clang-format off
#define TRMV_SIZES ->Arg(8)->Arg(16)->Arg(32)->Arg(64)->Arg(128)->Arg(256)->Arg(512)->Arg(1024)->Arg(2048)
BENCHMARK(BM_TRMV<float, Lower>) TRMV_SIZES ->Name("TRMV_float_Lower");
BENCHMARK(BM_TRMV<float, Upper>) TRMV_SIZES ->Name("TRMV_float_Upper");
BENCHMARK(BM_TRMV<float, UnitLower>) TRMV_SIZES ->Name("TRMV_float_UnitLower");
BENCHMARK(BM_TRMV<float, UnitUpper>) TRMV_SIZES ->Name("TRMV_float_UnitUpper");
BENCHMARK(BM_TRMV<double, Lower>) TRMV_SIZES ->Name("TRMV_double_Lower");
BENCHMARK(BM_TRMV<double, Upper>) TRMV_SIZES ->Name("TRMV_double_Upper");
BENCHMARK(BM_TRMV<double, UnitLower>) TRMV_SIZES ->Name("TRMV_double_UnitLower");
BENCHMARK(BM_TRMV<double, UnitUpper>) TRMV_SIZES ->Name("TRMV_double_UnitUpper");
#undef TRMV_SIZES
// clang-format on
