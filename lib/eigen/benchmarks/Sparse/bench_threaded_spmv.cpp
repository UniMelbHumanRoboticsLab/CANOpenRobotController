// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#define EIGEN_USE_THREADS 1

#include <benchmark/benchmark.h>
#include <Eigen/Sparse>
#include <Eigen/SparseCore>

#include <random>
#include <set>

using namespace Eigen;

using Scalar = double;
using SpMatColMajor = SparseMatrix<Scalar, ColMajor>;
using SpMatRowMajor = SparseMatrix<Scalar, RowMajor>;
using DenseVec = Matrix<Scalar, Dynamic, 1>;

// Three matrix shapes, chosen to expose different parallel-SpMV regimes.
enum MatrixKind { kBanded = 0, kRandom = 1, kHub = 2 };

template <typename SpMat>
static SpMat make_banded(int n, int bandwidth) {
  std::mt19937 gen(12345);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);
  std::vector<Triplet<Scalar>> triplets;
  triplets.reserve(static_cast<std::size_t>(n) * (2 * bandwidth + 1));
  for (int i = 0; i < n; ++i) {
    for (int j = std::max(0, i - bandwidth); j < std::min(n, i + bandwidth + 1); ++j) {
      triplets.emplace_back(i, j, dist(gen));
    }
  }
  SpMat m(n, n);
  m.setFromTriplets(triplets.begin(), triplets.end());
  m.makeCompressed();
  return m;
}

template <typename SpMat>
static SpMat make_random(int n, int nnz_per_row) {
  std::mt19937 gen(67890);
  std::uniform_int_distribution<int> col_dist(0, n - 1);
  std::uniform_real_distribution<double> val_dist(-1.0, 1.0);
  std::vector<Triplet<Scalar>> triplets;
  triplets.reserve(static_cast<std::size_t>(n) * nnz_per_row);
  for (int i = 0; i < n; ++i) {
    std::set<int> cols;
    while (static_cast<int>(cols.size()) < nnz_per_row) cols.insert(col_dist(gen));
    for (int j : cols) triplets.emplace_back(i, j, val_dist(gen));
  }
  SpMat m(n, n);
  m.setFromTriplets(triplets.begin(), triplets.end());
  m.makeCompressed();
  return m;
}

// Hub matrix: a few extremely dense rows (the "hubs") plus a sparse tail.
// Forces partition imbalance to expose nnz-balance behaviour.
template <typename SpMat>
static SpMat make_hub(int n, int hub_rows, int hub_density_pct, int tail_nnz_per_row) {
  std::mt19937 gen(13579);
  std::uniform_int_distribution<int> col_dist(0, n - 1);
  std::uniform_real_distribution<double> val_dist(-1.0, 1.0);
  std::vector<Triplet<Scalar>> triplets;
  for (int i = 0; i < n; ++i) {
    if (i < hub_rows) {
      // Hub row: hub_density_pct% of all columns
      int target = (n * hub_density_pct) / 100;
      std::set<int> cols;
      while (static_cast<int>(cols.size()) < target) cols.insert(col_dist(gen));
      for (int j : cols) triplets.emplace_back(i, j, val_dist(gen));
    } else {
      std::set<int> cols;
      while (static_cast<int>(cols.size()) < tail_nnz_per_row) cols.insert(col_dist(gen));
      for (int j : cols) triplets.emplace_back(i, j, val_dist(gen));
    }
  }
  SpMat m(n, n);
  m.setFromTriplets(triplets.begin(), triplets.end());
  m.makeCompressed();
  return m;
}

template <typename SpMat>
static SpMat make_matrix(MatrixKind kind, int n) {
  switch (kind) {
    case kBanded:
      return make_banded<SpMat>(n, 32);  // ~65 nnz/row, regular
    case kRandom:
      return make_random<SpMat>(n, 40);  // 40 nnz/row, uniform
    case kHub:
      return make_hub<SpMat>(n, 4, 50, 10);  // 4 dense rows + sparse tail
  }
  return SpMat{};
}

// Baseline: existing single-threaded (or OpenMP, if compiled with it) SpMV.
template <typename SpMat>
static void BM_Baseline_Forward(benchmark::State& state) {
  const MatrixKind kind = static_cast<MatrixKind>(state.range(0));
  const int n = static_cast<int>(state.range(1));
  SpMat A = make_matrix<SpMat>(kind, n);
  DenseVec x = DenseVec::Random(n);
  DenseVec y(n);
  for (auto _ : state) {
    y.noalias() = A * x;
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["nnz"] = static_cast<double>(A.nonZeros());
  state.counters["n"] = n;
}

template <typename SpMat>
static void BM_Baseline_Adjoint(benchmark::State& state) {
  const MatrixKind kind = static_cast<MatrixKind>(state.range(0));
  const int n = static_cast<int>(state.range(1));
  SpMat A = make_matrix<SpMat>(kind, n);
  DenseVec x = DenseVec::Random(n);
  DenseVec y(n);
  for (auto _ : state) {
    y.noalias() = A.adjoint() * x;
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["nnz"] = static_cast<double>(A.nonZeros());
  state.counters["n"] = n;
}

// New: ThreadedSparseProduct, pool sized via state.range(2).
template <typename SpMat>
static void BM_Threaded_Forward(benchmark::State& state) {
  const MatrixKind kind = static_cast<MatrixKind>(state.range(0));
  const int n = static_cast<int>(state.range(1));
  const int threads = static_cast<int>(state.range(2));
  // Make OpenMP team size match the requested thread count: under
  // EIGEN_HAS_OPENMP the operator launches `omp parallel for num_threads(T)`
  // where T = Eigen::nbThreads(), not the pool's size. Otherwise the
  // reported "T" counter would not match the actual parallelism.
  Eigen::setNbThreads(threads);
  SpMat A = make_matrix<SpMat>(kind, n);
  ThreadPool pool(threads);
  ThreadedSparseProduct<SpMat> op(A, &pool);
  DenseVec x = DenseVec::Random(n);
  DenseVec y(n);
  // Warm the mirror (only material for ColMajor forward) and prime the pool.
  op.apply(x, y);
  for (auto _ : state) {
    op.apply(x, y);
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["nnz"] = static_cast<double>(A.nonZeros());
  state.counters["n"] = n;
  state.counters["T"] = threads;
}

template <typename SpMat>
static void BM_Threaded_Adjoint(benchmark::State& state) {
  const MatrixKind kind = static_cast<MatrixKind>(state.range(0));
  const int n = static_cast<int>(state.range(1));
  const int threads = static_cast<int>(state.range(2));
  // Make OpenMP team size match the requested thread count: under
  // EIGEN_HAS_OPENMP the operator launches `omp parallel for num_threads(T)`
  // where T = Eigen::nbThreads(), not the pool's size. Otherwise the
  // reported "T" counter would not match the actual parallelism.
  Eigen::setNbThreads(threads);
  SpMat A = make_matrix<SpMat>(kind, n);
  ThreadPool pool(threads);
  ThreadedSparseProduct<SpMat> op(A, &pool);
  DenseVec x = DenseVec::Random(n);
  DenseVec y(n);
  // Warm the mirror outside the timed loop.
  op.applyAdjoint(x, y);
  for (auto _ : state) {
    op.applyAdjoint(x, y);
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["nnz"] = static_cast<double>(A.nonZeros());
  state.counters["n"] = n;
  state.counters["T"] = threads;
}

// Matrix sizes: spans the threading threshold (20000 nnz) and goes up to a
// few million nnz where threading should clearly win.
//   n=200   -> ~13K  nnz (random)  -> below threshold, expects serial
//   n=2000  -> ~80K  nnz           -> threaded
//   n=20000 -> ~800K nnz           -> threaded, parallelism matters
//   n=100000-> ~4M   nnz           -> threaded, memory-bandwidth bound

// Common arg grids, expressed declaratively with ArgsProduct: the Cartesian
// product of {kind} x {n} for the baselines, and {kind} x {n} x {threads} for
// the threaded variants. ArgsProduct keeps the grid on the registration itself
// rather than routing through an Apply() callback, which would have to name the
// implementation-detail benchmark::internal::Benchmark* type.
BENCHMARK_TEMPLATE(BM_Baseline_Forward, SpMatRowMajor)
    ->ArgsProduct({{kBanded, kRandom, kHub}, {200, 2000, 20000, 100000}});
BENCHMARK_TEMPLATE(BM_Baseline_Forward, SpMatColMajor)
    ->ArgsProduct({{kBanded, kRandom, kHub}, {200, 2000, 20000, 100000}});
BENCHMARK_TEMPLATE(BM_Baseline_Adjoint, SpMatRowMajor)
    ->ArgsProduct({{kBanded, kRandom, kHub}, {200, 2000, 20000, 100000}});
BENCHMARK_TEMPLATE(BM_Baseline_Adjoint, SpMatColMajor)
    ->ArgsProduct({{kBanded, kRandom, kHub}, {200, 2000, 20000, 100000}});
// Threaded benchmarks measure wall-clock latency: Google Benchmark's default
// CPU-time mode sums across worker threads and would hide actual speedup.
BENCHMARK_TEMPLATE(BM_Threaded_Forward, SpMatRowMajor)
    ->ArgsProduct({{kBanded, kRandom, kHub}, {200, 2000, 20000, 100000}, {1, 2, 4, 8}})
    ->UseRealTime();
BENCHMARK_TEMPLATE(BM_Threaded_Forward, SpMatColMajor)
    ->ArgsProduct({{kBanded, kRandom, kHub}, {200, 2000, 20000, 100000}, {1, 2, 4, 8}})
    ->UseRealTime();
BENCHMARK_TEMPLATE(BM_Threaded_Adjoint, SpMatRowMajor)
    ->ArgsProduct({{kBanded, kRandom, kHub}, {200, 2000, 20000, 100000}, {1, 2, 4, 8}})
    ->UseRealTime();
BENCHMARK_TEMPLATE(BM_Threaded_Adjoint, SpMatColMajor)
    ->ArgsProduct({{kBanded, kRandom, kHub}, {200, 2000, 20000, 100000}, {1, 2, 4, 8}})
    ->UseRealTime();
