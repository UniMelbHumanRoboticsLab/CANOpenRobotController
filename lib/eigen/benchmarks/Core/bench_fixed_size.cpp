// Benchmarks for batches of fixed-size products (4x4 transform of 4xN points, arrays of 3x3 products).
// Critical for PCL, ROS, Sophus, Drake which use small matrices extensively.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>

using namespace Eigen;

#ifndef SCALAR
#define SCALAR float
#endif

typedef SCALAR Scalar;

// --- Batch transform: Matrix4 * Matrix<4,N> ---
static void BM_BatchTransform4xN(benchmark::State& state) {
  int N = state.range(0);
  typedef Matrix<Scalar, 4, 4> Mat4;
  typedef Matrix<Scalar, 4, Dynamic> MatXN;

  Mat4 transform = Mat4::Random();
  MatXN points = MatXN::Random(4, N);
  MatXN result(4, N);

  for (auto _ : state) {
    result.noalias() = transform * points;
    benchmark::DoNotOptimize(result.data());
    benchmark::ClobberMemory();
  }
  state.counters["GFLOPS"] =
      benchmark::Counter(2.0 * 4 * 4 * N, benchmark::Counter::kIsIterationInvariantRate, benchmark::Counter::kIs1000);
}

// --- Fixed 3x3 batch operations (common in point cloud processing) ---
static void BM_Batch3x3Gemm(benchmark::State& state) {
  int count = state.range(0);
  typedef Matrix<Scalar, 3, 3> Mat3;

  std::vector<Mat3> a(count), b(count), c(count);
  for (int i = 0; i < count; ++i) {
    a[i] = Mat3::Random();
    b[i] = Mat3::Random();
  }

  for (auto _ : state) {
    for (int i = 0; i < count; ++i) {
      c[i].noalias() = a[i] * b[i];
    }
    benchmark::DoNotOptimize(c.data());
    benchmark::ClobberMemory();
  }
  state.counters["GFLOPS"] =
      benchmark::Counter(2.0 * 27 * count, benchmark::Counter::kIsIterationInvariantRate, benchmark::Counter::kIs1000);
}

// Batch 4xN transform
BENCHMARK(BM_BatchTransform4xN)->Arg(1)->Arg(4)->Arg(8)->Arg(16)->Arg(64);

// Batch 3x3 GEMM
BENCHMARK(BM_Batch3x3Gemm)->Arg(100)->Arg(1000)->Arg(10000);
