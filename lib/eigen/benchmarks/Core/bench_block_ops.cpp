// Benchmarks for block extraction and assignment operations.
//
// Tests sub-matrix views: block(), topRows(), leftCols(), middleCols().
// Measures expression template overhead for read and write patterns.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>

using namespace Eigen;

// Read a block and assign to a separate matrix (forces evaluation).
template <typename Scalar>
static void BM_BlockRead(benchmark::State& state) {
  const Index n = state.range(0);
  const Index block_size = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat src = Mat::Random(n, n);
  Mat dst(block_size, block_size);
  const Index off = (n - block_size) / 2;
  for (auto _ : state) {
    dst = src.block(off, off, block_size, block_size);
    benchmark::DoNotOptimize(dst.data());
  }
  state.SetBytesProcessed(state.iterations() * block_size * block_size * sizeof(Scalar));
}

// Write into a block of a larger matrix.
template <typename Scalar>
static void BM_BlockWrite(benchmark::State& state) {
  const Index n = state.range(0);
  const Index block_size = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat dst = Mat::Random(n, n);
  Mat src = Mat::Random(block_size, block_size);
  const Index off = (n - block_size) / 2;
  for (auto _ : state) {
    dst.block(off, off, block_size, block_size) = src;
    benchmark::DoNotOptimize(dst.data());
  }
  state.SetBytesProcessed(state.iterations() * block_size * block_size * sizeof(Scalar));
}

// topRows extraction.
template <typename Scalar>
static void BM_TopRows(benchmark::State& state) {
  const Index n = state.range(0);
  const Index k = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat src = Mat::Random(n, n);
  Mat dst(k, n);
  for (auto _ : state) {
    dst = src.topRows(k);
    benchmark::DoNotOptimize(dst.data());
  }
  state.SetBytesProcessed(state.iterations() * k * n * sizeof(Scalar));
}

// leftCols extraction.
template <typename Scalar>
static void BM_LeftCols(benchmark::State& state) {
  const Index n = state.range(0);
  const Index k = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat src = Mat::Random(n, n);
  Mat dst(n, k);
  for (auto _ : state) {
    dst = src.leftCols(k);
    benchmark::DoNotOptimize(dst.data());
  }
  state.SetBytesProcessed(state.iterations() * n * k * sizeof(Scalar));
}

// (matrix_size, block_size)
// clang-format off
#define BLOCK_SIZES ->ArgsProduct({{256, 512, 1024}, {16, 64, 128}})
BENCHMARK(BM_BlockRead<float>) BLOCK_SIZES ->Name("BlockRead_float");
BENCHMARK(BM_BlockRead<double>) BLOCK_SIZES ->Name("BlockRead_double");
BENCHMARK(BM_BlockWrite<float>) BLOCK_SIZES ->Name("BlockWrite_float");
BENCHMARK(BM_BlockWrite<double>) BLOCK_SIZES ->Name("BlockWrite_double");
BENCHMARK(BM_TopRows<float>) BLOCK_SIZES ->Name("TopRows_float");
BENCHMARK(BM_TopRows<double>) BLOCK_SIZES ->Name("TopRows_double");
BENCHMARK(BM_LeftCols<float>) BLOCK_SIZES ->Name("LeftCols_float");
BENCHMARK(BM_LeftCols<double>) BLOCK_SIZES ->Name("LeftCols_double");
#undef BLOCK_SIZES
// clang-format on
