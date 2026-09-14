// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

// Benchmarks the eigenvalues of a random symmetric tridiagonal matrix computed three ways:
//   - BM_qr_*         : the implicit-QR algorithm (SelfAdjointEigenSolver::computeFromTridiagonal),
//                       all eigenvalues;
//   - BM_bisect_*     : Sturm-sequence spectral bisection (TridiagonalEigenSolver), all eigenvalues;
//   - BM_bisect_sub_* : bisection of only the 10% smallest eigenvalues (an index-range subset,
//                       a query QR cannot answer without computing the whole spectrum).
// Suffix _f / _d selects float / double.
//
// When built with OpenMP, the bisection path parallelizes its independent per-eigenvalue bisections
// across Eigen::nbThreads() threads (the QR path is serial). To measure thread scaling, run the
// binary once per thread count via the environment, e.g.
//     for t in 1 2 4 8; do OMP_NUM_THREADS=$t ./bench_tridiagonal_bisection \
//         --benchmark_filter='BM_bisect_d/' --benchmark_repetitions=5; done
// Do not call Eigen::setNbThreads() to sweep the thread count inside one process: it mutates a global,
// so consecutive/interleaved runs at different thread counts overwrite each other's setting and the
// measured scaling is meaningless. Keep thread selection in the environment (OMP_NUM_THREADS), one
// process per data point.

#include <benchmark/benchmark.h>
#include <Eigen/Eigenvalues>

using namespace Eigen;

namespace {

enum Mode { kBisectAll, kQrAll, kBisectSubset };

template <typename Scalar>
void run(benchmark::State& state, Mode mode) {
  const Index n = state.range(0);
  Matrix<Scalar, Dynamic, 1> diag = Matrix<Scalar, Dynamic, 1>::Random(n);
  Matrix<Scalar, Dynamic, 1> subdiag = Matrix<Scalar, Dynamic, 1>::Random(n > 1 ? n - 1 : 0);

  if (mode == kQrAll) {
    SelfAdjointEigenSolver<Matrix<Scalar, Dynamic, Dynamic>> solver(n);
    for (auto _ : state) {
      solver.computeFromTridiagonal(diag, subdiag, EigenvaluesOnly);
      benchmark::DoNotOptimize(solver.eigenvalues().data());
      benchmark::ClobberMemory();
    }
  } else {
    TridiagonalEigenSolver<Scalar> solver(n);
    const EigenvalueRange range = mode == kBisectSubset ? EigenvalueRange::indices(0, n / 10) : EigenvalueRange::all();
    for (auto _ : state) {
      solver.computeEigenvalues(diag, subdiag, range);
      benchmark::DoNotOptimize(solver.eigenvalues().data());
      benchmark::ClobberMemory();
    }
  }
  state.SetItemsProcessed(state.iterations() * n);
}

void BM_qr_f(benchmark::State& s) { run<float>(s, kQrAll); }
void BM_bisect_f(benchmark::State& s) { run<float>(s, kBisectAll); }
void BM_bisect_sub_f(benchmark::State& s) { run<float>(s, kBisectSubset); }
void BM_qr_d(benchmark::State& s) { run<double>(s, kQrAll); }
void BM_bisect_d(benchmark::State& s) { run<double>(s, kBisectAll); }
void BM_bisect_sub_d(benchmark::State& s) { run<double>(s, kBisectSubset); }

// Bisection parallelizes per-eigenvalue, so it keeps scaling to large n; include sizes up to 8192
// (the QR path dominates wall time there but is the right serial baseline).
#define EIGEN_BENCH_SIZES ArgsProduct({{16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192}})

BENCHMARK(BM_qr_f)->EIGEN_BENCH_SIZES->UseRealTime();
BENCHMARK(BM_bisect_f)->EIGEN_BENCH_SIZES->UseRealTime();
BENCHMARK(BM_bisect_sub_f)->EIGEN_BENCH_SIZES->UseRealTime();
BENCHMARK(BM_qr_d)->EIGEN_BENCH_SIZES->UseRealTime();
BENCHMARK(BM_bisect_d)->EIGEN_BENCH_SIZES->UseRealTime();
BENCHMARK(BM_bisect_sub_d)->EIGEN_BENCH_SIZES->UseRealTime();

}  // namespace
