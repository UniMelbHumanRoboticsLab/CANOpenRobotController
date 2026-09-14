// Benchmarks for matrix exponential.
// Critical for Sophus Lie group operations (SLAM, visual odometry).
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>
#include <contrib/Eigen/MatrixFunctions>

using namespace Eigen;

#ifndef SCALAR
#define SCALAR double
#endif

typedef SCALAR Scalar;

static void BM_MatrixExp(benchmark::State& state) {
  int n = state.range(0);
  Scalar scale = Scalar(state.range(1));
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixType;

  // The larger scale exercises the scale-and-square path, including the
  // coefficient-wise exact power-of-two scale-down.
  MatrixType A = scale * MatrixType::Random(n, n) / Scalar(n);
  MatrixType result(n, n);

  for (auto _ : state) {
    result = A.exp();
    benchmark::DoNotOptimize(result.data());
    benchmark::ClobberMemory();
  }
}

// Fixed-size specializations for Lie group sizes.
template <int N>
static void BM_MatrixExp_Fixed(benchmark::State& state) {
  typedef Matrix<Scalar, N, N> MatrixType;

  MatrixType A = MatrixType::Random() / Scalar(N);
  MatrixType result;

  for (auto _ : state) {
    result = A.exp();
    benchmark::DoNotOptimize(result.data());
    benchmark::ClobberMemory();
  }
}

// Dynamic sizes: Lie groups (2,3,4) plus larger, both well-scaled and through
// the scale-and-square path.
BENCHMARK(BM_MatrixExp)->ArgsProduct({{2, 3, 4, 8, 16, 32, 64, 128}, {1, 64}});

// Fixed-size Lie group dimensions.
BENCHMARK(BM_MatrixExp_Fixed<2>);
BENCHMARK(BM_MatrixExp_Fixed<3>);
BENCHMARK(BM_MatrixExp_Fixed<4>);
