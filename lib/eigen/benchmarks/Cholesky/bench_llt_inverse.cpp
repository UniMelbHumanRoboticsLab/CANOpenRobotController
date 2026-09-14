// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0
//
// Micro-benchmark for inverting a symmetric positive definite matrix from its Cholesky factor.
// LLT::inverse() runs the LAPACK POTRI sequence (TRTRI then LAUUM) for 2n^3/3 flops, against the
// 2n^3 of the solve-with-an-explicit-identity form that was the only route before it existed. The
// GFLOPS counters report each routine's own operation count, so they measure utilization rather
// than the speedup; the speedup is the ratio of the wall times.

#include <benchmark/benchmark.h>
#include <Eigen/Cholesky>
#include <Eigen/Core>

using namespace Eigen;

#ifndef SCALAR
#define SCALAR double
#endif

using Scalar = SCALAR;
using MatrixType = Matrix<Scalar, Dynamic, Dynamic>;
using RealScalar = NumTraits<Scalar>::Real;

// Flops counted as in bench_cholesky.cpp: multiply and add counted separately, complex arithmetic
// left at its real-arithmetic count so that the real and complex builds stay comparable.
static double potri_cost(int n) { return 2.0 / 3.0 * double(n) * n * n; }
static double solve_identity_cost(int n) { return 2.0 * double(n) * n * n; }
// TRTRI and LAUUM each cost n^3/3, the two halves of the POTRI sequence above.
static double potri_kernel_cost(int n) { return 1.0 / 3.0 * double(n) * n * n; }
static double trsm_identity_cost(int n) { return double(n) * n * n; }

static MatrixType make_spd(int n) {
  MatrixType a = MatrixType::Random(n, n);
  return a * a.adjoint() + RealScalar(n) * MatrixType::Identity(n, n);
}

static void set_rate(benchmark::State& state, double flops) {
  state.counters["GFLOPS"] =
      benchmark::Counter(flops, benchmark::Counter::kIsIterationInvariantRate, benchmark::Counter::kIs1000);
}

static void BM_LLT_Inverse(benchmark::State& state) {
  const int n = state.range(0);
  const LLT<MatrixType> llt(make_spd(n));
  MatrixType inv(n, n);
  for (auto _ : state) {
    inv = llt.inverse();
    benchmark::DoNotOptimize(inv.data());
    benchmark::ClobberMemory();
  }
  set_rate(state, potri_cost(n));
}
BENCHMARK(BM_LLT_Inverse)->RangeMultiplier(2)->Range(8, 2048);

static void BM_LLT_SolveIdentity(benchmark::State& state) {
  const int n = state.range(0);
  const LLT<MatrixType> llt(make_spd(n));
  MatrixType inv(n, n);
  for (auto _ : state) {
    inv = llt.solve(MatrixType::Identity(n, n));
    benchmark::DoNotOptimize(inv.data());
    benchmark::ClobberMemory();
  }
  set_rate(state, solve_identity_cost(n));
}
BENCHMARK(BM_LLT_SolveIdentity)->RangeMultiplier(2)->Range(8, 2048);

// The two kernels LLT::inverse() is built from, and the pre-existing operation each replaces. Both
// pairs carry the same O(n^2) per-iteration setup, so the ratio within a pair is meaningful.
static void BM_TriangularInverseInPlace(benchmark::State& state) {
  const int n = state.range(0);
  const MatrixType factor = LLT<MatrixType>(make_spd(n)).matrixLLT();
  MatrixType x(n, n);
  for (auto _ : state) {
    x = factor;
    x.triangularView<Lower>().inverseInPlace();
    benchmark::DoNotOptimize(x.data());
    benchmark::ClobberMemory();
  }
  set_rate(state, potri_kernel_cost(n));
}
BENCHMARK(BM_TriangularInverseInPlace)->RangeMultiplier(2)->Range(8, 2048);

static void BM_TriangularSolveIdentity(benchmark::State& state) {
  const int n = state.range(0);
  const MatrixType factor = LLT<MatrixType>(make_spd(n)).matrixLLT();
  MatrixType x(n, n);
  for (auto _ : state) {
    x = MatrixType::Identity(n, n);
    factor.triangularView<Lower>().solveInPlace(x);
    benchmark::DoNotOptimize(x.data());
    benchmark::ClobberMemory();
  }
  set_rate(state, trsm_identity_cost(n));
}
BENCHMARK(BM_TriangularSolveIdentity)->RangeMultiplier(2)->Range(8, 2048);

static void BM_TriangularAdjointSquare(benchmark::State& state) {
  const int n = state.range(0);
  const MatrixType factor = LLT<MatrixType>(make_spd(n)).matrixLLT();
  MatrixType x(n, n);
  for (auto _ : state) {
    x = factor;
    internal::triangular_adjoint_square_in_place<Lower>(x);
    benchmark::DoNotOptimize(x.data());
    benchmark::ClobberMemory();
  }
  set_rate(state, potri_kernel_cost(n));
}
BENCHMARK(BM_TriangularAdjointSquare)->RangeMultiplier(2)->Range(8, 2048);
