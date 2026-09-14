// Benchmarks for diagonal operations.
//
// Tests diagonal extraction, diagonal-matrix product, and matrix-diagonal product.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>

using namespace Eigen;

// Extract diagonal from a square matrix and sum it.
template <typename Scalar>
static void BM_DiagonalExtract(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(n, n);
  for (auto _ : state) {
    Scalar s = A.diagonal().sum();
    benchmark::DoNotOptimize(s);
  }
  state.SetBytesProcessed(state.iterations() * n * sizeof(Scalar));
}

// y = diag(d) * x  (diagonal matrix times vector).
template <typename Scalar>
static void BM_DiagonalTimesVector(benchmark::State& state) {
  const Index n = state.range(0);
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Vec d = Vec::Random(n);
  Vec x = Vec::Random(n);
  Vec y(n);
  for (auto _ : state) {
    y = d.asDiagonal() * x;
    benchmark::DoNotOptimize(y.data());
  }
  state.SetBytesProcessed(state.iterations() * 3 * n * sizeof(Scalar));
}

// C = diag(d) * A  (diagonal matrix times dense matrix).
template <typename Scalar>
static void BM_DiagonalTimesMatrix(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Vec d = Vec::Random(n);
  Mat A = Mat::Random(n, n);
  Mat C(n, n);
  for (auto _ : state) {
    C.noalias() = d.asDiagonal() * A;
    benchmark::DoNotOptimize(C.data());
  }
  state.SetBytesProcessed(state.iterations() * 2 * n * n * sizeof(Scalar));
}

// C = A * diag(d)  (dense matrix times diagonal matrix).
template <typename Scalar>
static void BM_MatrixTimesDiagonal(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Vec d = Vec::Random(n);
  Mat A = Mat::Random(n, n);
  Mat C(n, n);
  for (auto _ : state) {
    C.noalias() = A * d.asDiagonal();
    benchmark::DoNotOptimize(C.data());
  }
  state.SetBytesProcessed(state.iterations() * 2 * n * n * sizeof(Scalar));
}

// clang-format off
#define DIAG_SIZES ->Arg(32)->Arg(64)->Arg(128)->Arg(256)->Arg(512)->Arg(1024)
BENCHMARK(BM_DiagonalExtract<float>) DIAG_SIZES ->Name("DiagonalExtract_float");
BENCHMARK(BM_DiagonalExtract<double>) DIAG_SIZES ->Name("DiagonalExtract_double");
BENCHMARK(BM_DiagonalTimesVector<float>) DIAG_SIZES ->Name("DiagonalTimesVector_float");
BENCHMARK(BM_DiagonalTimesVector<double>) DIAG_SIZES ->Name("DiagonalTimesVector_double");
BENCHMARK(BM_DiagonalTimesMatrix<float>) DIAG_SIZES ->Name("DiagonalTimesMatrix_float");
BENCHMARK(BM_DiagonalTimesMatrix<double>) DIAG_SIZES ->Name("DiagonalTimesMatrix_double");
BENCHMARK(BM_MatrixTimesDiagonal<float>) DIAG_SIZES ->Name("MatrixTimesDiagonal_float");
BENCHMARK(BM_MatrixTimesDiagonal<double>) DIAG_SIZES ->Name("MatrixTimesDiagonal_double");
#undef DIAG_SIZES
// clang-format on
