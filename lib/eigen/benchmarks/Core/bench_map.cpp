// Benchmarks for Map and Ref with various strides.
//
// Compares contiguous Map vs strided Map vs owned matrix for basic
// operations (GEMV and vector sum).
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>

using namespace Eigen;

// Sum a contiguous Map<VectorX>.
template <typename Scalar>
static void BM_MapContiguousSum(benchmark::State& state) {
  const Index n = state.range(0);
  std::vector<Scalar> buf(n);
  Map<Matrix<Scalar, Dynamic, 1>> v(buf.data(), n);
  v.setRandom();
  for (auto _ : state) {
    Scalar s = v.sum();
    benchmark::DoNotOptimize(s);
  }
  state.SetBytesProcessed(state.iterations() * n * sizeof(Scalar));
}

// Sum a strided Map (InnerStride).
template <typename Scalar>
static void BM_MapStridedSum(benchmark::State& state) {
  const Index n = state.range(0);
  const Index stride = 3;
  std::vector<Scalar> buf(n * stride);
  Map<Matrix<Scalar, Dynamic, 1>, 0, InnerStride<>> v(buf.data(), n, InnerStride<>(stride));
  v.setRandom();
  for (auto _ : state) {
    Scalar s = v.sum();
    benchmark::DoNotOptimize(s);
  }
  state.SetBytesProcessed(state.iterations() * n * sizeof(Scalar));
}

// Sum an owned VectorX (baseline).
template <typename Scalar>
static void BM_OwnedSum(benchmark::State& state) {
  const Index n = state.range(0);
  Matrix<Scalar, Dynamic, 1> v = Matrix<Scalar, Dynamic, 1>::Random(n);
  for (auto _ : state) {
    Scalar s = v.sum();
    benchmark::DoNotOptimize(s);
  }
  state.SetBytesProcessed(state.iterations() * n * sizeof(Scalar));
}

// GEMV through contiguous Map<MatrixX>.
template <typename Scalar>
static void BM_MapGemv(benchmark::State& state) {
  const Index n = state.range(0);
  std::vector<Scalar> buf(n * n);
  Map<Matrix<Scalar, Dynamic, Dynamic>> A(buf.data(), n, n);
  A.setRandom();
  Matrix<Scalar, Dynamic, 1> x = Matrix<Scalar, Dynamic, 1>::Random(n);
  Matrix<Scalar, Dynamic, 1> y = Matrix<Scalar, Dynamic, 1>::Random(n);
  for (auto _ : state) {
    y.noalias() += A * x;
    benchmark::DoNotOptimize(y.data());
    benchmark::ClobberMemory();
  }
  state.counters["GFLOPS"] =
      benchmark::Counter(2.0 * n * n, benchmark::Counter::kIsIterationInvariantRate, benchmark::Counter::kIs1000);
}

// GEMV with owned matrix (baseline).
template <typename Scalar>
static void BM_OwnedGemv(benchmark::State& state) {
  const Index n = state.range(0);
  Matrix<Scalar, Dynamic, Dynamic> A = Matrix<Scalar, Dynamic, Dynamic>::Random(n, n);
  Matrix<Scalar, Dynamic, 1> x = Matrix<Scalar, Dynamic, 1>::Random(n);
  Matrix<Scalar, Dynamic, 1> y = Matrix<Scalar, Dynamic, 1>::Random(n);
  for (auto _ : state) {
    y.noalias() += A * x;
    benchmark::DoNotOptimize(y.data());
    benchmark::ClobberMemory();
  }
  state.counters["GFLOPS"] =
      benchmark::Counter(2.0 * n * n, benchmark::Counter::kIsIterationInvariantRate, benchmark::Counter::kIs1000);
}

// clang-format off
#define SUM_SIZES ->Arg(256)->Arg(1024)->Arg(4096)->Arg(16384)->Arg(65536)->Arg(262144)->Arg(1048576)
BENCHMARK(BM_MapContiguousSum<float>) SUM_SIZES ->Name("MapContiguousSum_float");
BENCHMARK(BM_MapStridedSum<float>) SUM_SIZES ->Name("MapStridedSum_float");
BENCHMARK(BM_OwnedSum<float>) SUM_SIZES ->Name("OwnedSum_float");
BENCHMARK(BM_MapContiguousSum<double>) SUM_SIZES ->Name("MapContiguousSum_double");
BENCHMARK(BM_MapStridedSum<double>) SUM_SIZES ->Name("MapStridedSum_double");
BENCHMARK(BM_OwnedSum<double>) SUM_SIZES ->Name("OwnedSum_double");
#undef SUM_SIZES
BENCHMARK(BM_MapGemv<float>)->Arg(32)->Arg(128)->Arg(512)->Arg(1024)->Name("MapGemv_float");
BENCHMARK(BM_OwnedGemv<float>)->Arg(32)->Arg(128)->Arg(512)->Arg(1024)->Name("OwnedGemv_float");
BENCHMARK(BM_MapGemv<double>)->Arg(32)->Arg(128)->Arg(512)->Arg(1024)->Name("MapGemv_double");
BENCHMARK(BM_OwnedGemv<double>)->Arg(32)->Arg(128)->Arg(512)->Arg(1024)->Name("OwnedGemv_double");
// clang-format on
