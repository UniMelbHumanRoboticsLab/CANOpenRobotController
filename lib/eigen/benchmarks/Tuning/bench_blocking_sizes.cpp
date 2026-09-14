// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>

#include <cstdint>

bool eigen_use_specific_block_size;
int eigen_block_size_k, eigen_block_size_m, eigen_block_size_n;
#define EIGEN_TEST_SPECIFIC_BLOCKING_SIZES eigen_use_specific_block_size
#define EIGEN_TEST_SPECIFIC_BLOCKING_SIZE_K eigen_block_size_k
#define EIGEN_TEST_SPECIFIC_BLOCKING_SIZE_M eigen_block_size_m
#define EIGEN_TEST_SPECIFIC_BLOCKING_SIZE_N eigen_block_size_n
#include <Eigen/Core>

using namespace Eigen;

typedef MatrixXf MatrixType;
typedef MatrixType::Scalar Scalar;

static void BM_GemmDefaultBlocking(benchmark::State& state) {
  int k = state.range(0);
  int m = state.range(1);
  int n = state.range(2);
  eigen_use_specific_block_size = false;
  MatrixType lhs = MatrixType::Random(m, k);
  MatrixType rhs = MatrixType::Random(k, n);
  MatrixType dst = MatrixType::Zero(m, n);
  for (auto _ : state) {
    dst.noalias() = lhs * rhs;
    benchmark::DoNotOptimize(dst.data());
  }
  state.counters["GFLOPS"] =
      benchmark::Counter(2.0 * k * m * n, benchmark::Counter::kIsIterationInvariantRate, benchmark::Counter::kIs1000);
}

static void BM_GemmCustomBlocking(benchmark::State& state) {
  int k = state.range(0);
  int m = state.range(1);
  int n = state.range(2);
  int bk = state.range(3);
  int bm = state.range(4);
  int bn = state.range(5);
  eigen_use_specific_block_size = true;
  eigen_block_size_k = bk;
  eigen_block_size_m = bm;
  eigen_block_size_n = bn;
  MatrixType lhs = MatrixType::Random(m, k);
  MatrixType rhs = MatrixType::Random(k, n);
  MatrixType dst = MatrixType::Zero(m, n);
  for (auto _ : state) {
    dst.noalias() = lhs * rhs;
    benchmark::DoNotOptimize(dst.data());
  }
  state.counters["GFLOPS"] =
      benchmark::Counter(2.0 * k * m * n, benchmark::Counter::kIsIterationInvariantRate, benchmark::Counter::kIs1000);
}

// clang-format off
BENCHMARK(BM_GemmDefaultBlocking)
    ->Args({64, 64, 64})->Args({128, 128, 128})->Args({256, 256, 256})
    ->Args({512, 512, 512})->Args({1024, 1024, 1024})->Args({2048, 2048, 2048});

// {k, m, n, bk, bm, bn} — all bk/bm values ≤ 256 ≤ min(s), so no filtering needed.
BENCHMARK(BM_GemmCustomBlocking)
    // s = 256
    ->Args({256,256,256,16,16,256})->Args({256,256,256,16,32,256})->Args({256,256,256,16,64,256})->Args({256,256,256,16,128,256})->Args({256,256,256,16,256,256})
    ->Args({256,256,256,32,16,256})->Args({256,256,256,32,32,256})->Args({256,256,256,32,64,256})->Args({256,256,256,32,128,256})->Args({256,256,256,32,256,256})
    ->Args({256,256,256,64,16,256})->Args({256,256,256,64,32,256})->Args({256,256,256,64,64,256})->Args({256,256,256,64,128,256})->Args({256,256,256,64,256,256})
    ->Args({256,256,256,128,16,256})->Args({256,256,256,128,32,256})->Args({256,256,256,128,64,256})->Args({256,256,256,128,128,256})->Args({256,256,256,128,256,256})
    ->Args({256,256,256,256,16,256})->Args({256,256,256,256,32,256})->Args({256,256,256,256,64,256})->Args({256,256,256,256,128,256})->Args({256,256,256,256,256,256})
    // s = 512
    ->Args({512,512,512,16,16,512})->Args({512,512,512,16,32,512})->Args({512,512,512,16,64,512})->Args({512,512,512,16,128,512})->Args({512,512,512,16,256,512})
    ->Args({512,512,512,32,16,512})->Args({512,512,512,32,32,512})->Args({512,512,512,32,64,512})->Args({512,512,512,32,128,512})->Args({512,512,512,32,256,512})
    ->Args({512,512,512,64,16,512})->Args({512,512,512,64,32,512})->Args({512,512,512,64,64,512})->Args({512,512,512,64,128,512})->Args({512,512,512,64,256,512})
    ->Args({512,512,512,128,16,512})->Args({512,512,512,128,32,512})->Args({512,512,512,128,64,512})->Args({512,512,512,128,128,512})->Args({512,512,512,128,256,512})
    ->Args({512,512,512,256,16,512})->Args({512,512,512,256,32,512})->Args({512,512,512,256,64,512})->Args({512,512,512,256,128,512})->Args({512,512,512,256,256,512})
    // s = 1024
    ->Args({1024,1024,1024,16,16,1024})->Args({1024,1024,1024,16,32,1024})->Args({1024,1024,1024,16,64,1024})->Args({1024,1024,1024,16,128,1024})->Args({1024,1024,1024,16,256,1024})
    ->Args({1024,1024,1024,32,16,1024})->Args({1024,1024,1024,32,32,1024})->Args({1024,1024,1024,32,64,1024})->Args({1024,1024,1024,32,128,1024})->Args({1024,1024,1024,32,256,1024})
    ->Args({1024,1024,1024,64,16,1024})->Args({1024,1024,1024,64,32,1024})->Args({1024,1024,1024,64,64,1024})->Args({1024,1024,1024,64,128,1024})->Args({1024,1024,1024,64,256,1024})
    ->Args({1024,1024,1024,128,16,1024})->Args({1024,1024,1024,128,32,1024})->Args({1024,1024,1024,128,64,1024})->Args({1024,1024,1024,128,128,1024})->Args({1024,1024,1024,128,256,1024})
    ->Args({1024,1024,1024,256,16,1024})->Args({1024,1024,1024,256,32,1024})->Args({1024,1024,1024,256,64,1024})->Args({1024,1024,1024,256,128,1024})->Args({1024,1024,1024,256,256,1024});
// clang-format on
