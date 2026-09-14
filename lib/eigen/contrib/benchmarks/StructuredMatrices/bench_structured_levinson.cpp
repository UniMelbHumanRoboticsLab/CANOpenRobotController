// Benchmarks for the LookAheadLevinson direct Toeplitz solver: factorization and
// cached solves on well-conditioned systems, the look-ahead path on a shifted
// KMS family whose leading submatrices are singular (forcing block steps), and
// the dense LU alternative.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Dense>
#include <contrib/Eigen/StructuredMatrices>

using namespace Eigen;

typedef Matrix<double, Dynamic, 1> Vec;
typedef Matrix<double, Dynamic, Dynamic> Mat;

// Diagonally dominant generators: the classical Levinson recursion runs with no
// look-ahead blocks.
static void wellConditionedToeplitz(Index n, Vec& c, Vec& r) {
  c = Vec::Random(n);
  r = Vec::Random(n);
  r[0] = c[0] += 2.0 * double(n);
}

// Shifted Kac-Murdock-Szego family: leading submatrices T_k with k = 1, 4, 7,
// ... are singular, so every third step is a look-ahead block (n must be a
// multiple of 3 for T_n itself to be non-singular).
static void shiftedKmsToeplitz(Index n, Vec& c, Vec& r) {
  c.resize(n);
  r.resize(n);
  c[0] = r[0] = 1e-14;
  for (Index i = 1; i < n; ++i) c[i] = r[i] = std::pow(0.5, double(i - 1));
}

// --- Factorization ---
static void BM_LevinsonCompute(benchmark::State& state) {
  const Index n = state.range(0);
  Vec c, r;
  wellConditionedToeplitz(n, c, r);
  Toeplitz<double> T(c, r);
  for (auto _ : state) {
    LookAheadLevinson<double> lev(T);
    benchmark::DoNotOptimize(lev);
  }
}
BENCHMARK(BM_LevinsonCompute)->Arg(32)->Arg(256)->Arg(1024);

// --- Solve from a cached factorization ---
static void BM_LevinsonSolve(benchmark::State& state) {
  const Index n = state.range(0);
  Vec c, r;
  wellConditionedToeplitz(n, c, r);
  Toeplitz<double> T(c, r);
  LookAheadLevinson<double> lev(T);
  Vec b = Vec::Random(n);
  for (auto _ : state) {
    Vec x = lev.solve(b);
    benchmark::DoNotOptimize(x.data());
  }
}
BENCHMARK(BM_LevinsonSolve)->Arg(32)->Arg(256)->Arg(1024);

// --- Look-ahead path: factorization dominated by block steps ---
static void BM_LevinsonComputeLookAhead(benchmark::State& state) {
  const Index n = state.range(0);
  Vec c, r;
  shiftedKmsToeplitz(n, c, r);
  Toeplitz<double> T(c, r);
  for (auto _ : state) {
    LookAheadLevinson<double> lev;
    lev.setMaxBlockSize(3).compute(T);
    benchmark::DoNotOptimize(lev);
  }
}
BENCHMARK(BM_LevinsonComputeLookAhead)->Arg(30)->Arg(240)->Arg(1023);

// --- Dense alternative: LU factorization + solve on every call ---
static void BM_LevinsonSolveDense(benchmark::State& state) {
  const Index n = state.range(0);
  Vec c, r;
  wellConditionedToeplitz(n, c, r);
  Mat dense = Toeplitz<double>(c, r);
  Vec b = Vec::Random(n);
  for (auto _ : state) {
    Vec x = PartialPivLU<Mat>(dense).solve(b);
    benchmark::DoNotOptimize(x.data());
  }
}
BENCHMARK(BM_LevinsonSolveDense)->Arg(256)->Arg(1024);
