// Benchmarks for the Bccb operator: the O(N log N) 2-D-FFT-based product and
// direct solve against their dense counterparts (GEMV, and a solve with a
// precomputed PartialPivLU factorization -- the analogue of Bccb's symbol,
// which is precomputed at construction). The generating array is n x n, so the
// operator is N x N with N = n^2; the dense variants stop at n = 64 (N = 4096)
// to keep the dense matrix and its factorization affordable.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <contrib/Eigen/StructuredMatrices>

using namespace Eigen;

typedef Matrix<double, Dynamic, 1> Vec;
typedef Matrix<double, Dynamic, Dynamic> Mat;

// Diagonally dominant generating array: keeps the symbol away from zero, so the
// solves are well conditioned.
static Mat generatingArray(Index n) {
  Mat G = Mat::Random(n, n);
  G(0, 0) += double(2 * n * n);
  return G;
}

// The dense representation, built entry-wise: entry (i,j) with i = b1*n2 + i2,
// j = c1*n2 + j2 is G((i2-j2) mod n2, (b1-c1) mod n1).
static Mat denseBccb(const Mat& G) {
  const Index n2 = G.rows(), n1 = G.cols(), N = n1 * n2;
  Mat dense(N, N);
  for (Index j = 0; j < N; ++j)
    for (Index i = 0; i < N; ++i) {
      Index k2 = i % n2 - j % n2;
      if (k2 < 0) k2 += n2;
      Index k1 = i / n2 - j / n2;
      if (k1 < 0) k1 += n1;
      dense(i, j) = G(k2, k1);
    }
  return dense;
}

// --- product: y = C * x ---
static void BM_BccbProduct(benchmark::State& state) {
  const Index n = state.range(0), N = n * n;
  Mat G = generatingArray(n);
  Bccb<double> C(G);
  Vec x = Vec::Random(N), y(N);
  for (auto _ : state) {
    y.noalias() = C * x;
    benchmark::DoNotOptimize(y.data());
  }
}
// 97 is prime in both block dimensions: the product pins the padded embedding
// (the exact-size transform would run kissfft's quadratic generic butterfly).
BENCHMARK(BM_BccbProduct)->Arg(8)->Arg(16)->Arg(32)->Arg(64)->Arg(97)->Arg(128);

static void BM_DenseProduct(benchmark::State& state) {
  const Index n = state.range(0), N = n * n;
  Mat dense = denseBccb(generatingArray(n));
  Vec x = Vec::Random(N), y(N);
  for (auto _ : state) {
    y.noalias() = dense * x;
    benchmark::DoNotOptimize(y.data());
  }
}
BENCHMARK(BM_DenseProduct)->Arg(8)->Arg(16)->Arg(32)->Arg(64);

// --- solve: x = C^{-1} * b, both sides reusing their precomputed factorization
// (the 2-D DFT symbol for Bccb, the LU factors for the dense matrix) ---
static void BM_BccbSolve(benchmark::State& state) {
  const Index n = state.range(0), N = n * n;
  Mat G = generatingArray(n);
  Bccb<double> C(G);
  Vec b = Vec::Random(N), x(N);
  for (auto _ : state) {
    x = C.solve(b);
    benchmark::DoNotOptimize(x.data());
  }
}
// All 5-smooth: solve transforms at the exact size, so a prime dimension would
// only time kissfft's generic butterfly instead of the operator.
BENCHMARK(BM_BccbSolve)->Arg(8)->Arg(16)->Arg(32)->Arg(64)->Arg(96)->Arg(128);

static void BM_DenseSolve(benchmark::State& state) {
  const Index n = state.range(0), N = n * n;
  Mat dense = denseBccb(generatingArray(n));
  PartialPivLU<Mat> lu(dense);
  Vec b = Vec::Random(N), x(N);
  for (auto _ : state) {
    x = lu.solve(b);
    benchmark::DoNotOptimize(x.data());
  }
}
BENCHMARK(BM_DenseSolve)->Arg(8)->Arg(16)->Arg(32)->Arg(64);
