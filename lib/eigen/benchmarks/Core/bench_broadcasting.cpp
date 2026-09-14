// Benchmarks for colwise/rowwise reductions and broadcasting operations.
//
// Tests vectorwise reductions (sum, mean, norm, minCoeff, maxCoeff) and
// broadcasting arithmetic (rowwise += vec, colwise -= vec, rowwise *= vec).
// The BroadcastSubExp pair isolates Eigen's packet path through the Replicate
// evaluator: the scalar fallback calls libm exp per element, so unlike the
// plain add/mul broadcasts it cannot be rescued by compiler auto-vectorization.
// The PerCol variant performs the same operation with an explicit column loop
// and serves as an upper-bound reference.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>

using namespace Eigen;

// --- Colwise reductions (reduce each column to a scalar) ---

template <typename Scalar>
static void BM_ColwiseSum(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat m = Mat::Random(rows, cols);
  Matrix<Scalar, 1, Dynamic> result(cols);
  for (auto _ : state) {
    result = m.colwise().sum();
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(state.iterations() * rows * cols * sizeof(Scalar));
}

template <typename Scalar>
static void BM_ColwiseMean(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat m = Mat::Random(rows, cols);
  Matrix<Scalar, 1, Dynamic> result(cols);
  for (auto _ : state) {
    result = m.colwise().mean();
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(state.iterations() * rows * cols * sizeof(Scalar));
}

template <typename Scalar>
static void BM_ColwiseNorm(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat m = Mat::Random(rows, cols);
  Matrix<Scalar, 1, Dynamic> result(cols);
  for (auto _ : state) {
    result = m.colwise().norm();
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(state.iterations() * rows * cols * sizeof(Scalar));
}

template <typename Scalar>
static void BM_ColwiseMinCoeff(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat m = Mat::Random(rows, cols);
  Matrix<Scalar, 1, Dynamic> result(cols);
  for (auto _ : state) {
    result = m.colwise().minCoeff();
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(state.iterations() * rows * cols * sizeof(Scalar));
}

template <typename Scalar>
static void BM_ColwiseMaxCoeff(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat m = Mat::Random(rows, cols);
  Matrix<Scalar, 1, Dynamic> result(cols);
  for (auto _ : state) {
    result = m.colwise().maxCoeff();
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(state.iterations() * rows * cols * sizeof(Scalar));
}

// --- Rowwise reductions (reduce each row to a scalar) ---

template <typename Scalar>
static void BM_RowwiseSum(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat m = Mat::Random(rows, cols);
  Matrix<Scalar, Dynamic, 1> result(rows);
  for (auto _ : state) {
    result = m.rowwise().sum();
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(state.iterations() * rows * cols * sizeof(Scalar));
}

template <typename Scalar>
static void BM_RowwiseNorm(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat m = Mat::Random(rows, cols);
  Matrix<Scalar, Dynamic, 1> result(rows);
  for (auto _ : state) {
    result = m.rowwise().norm();
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(state.iterations() * rows * cols * sizeof(Scalar));
}

// --- Broadcasting operations ---

template <typename Scalar>
static void BM_RowwiseBroadcastAdd(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, 1, Dynamic>;
  Mat m = Mat::Random(rows, cols);
  Vec v = Vec::Random(cols);
  for (auto _ : state) {
    m.noalias() = m.rowwise() + v;
    benchmark::DoNotOptimize(m.data());
  }
  state.SetBytesProcessed(state.iterations() * rows * cols * sizeof(Scalar) * 2);
}

template <typename Scalar>
static void BM_ColwiseBroadcastAdd(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Mat m = Mat::Random(rows, cols);
  Vec v = Vec::Random(rows);
  for (auto _ : state) {
    m.noalias() = m.colwise() + v;
    benchmark::DoNotOptimize(m.data());
  }
  state.SetBytesProcessed(state.iterations() * rows * cols * sizeof(Scalar) * 2);
}

template <typename Scalar>
static void BM_RowwiseBroadcastMul(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat m = Mat::Random(rows, cols);
  Array<Scalar, 1, Dynamic> v = Array<Scalar, 1, Dynamic>::Random(cols);
  for (auto _ : state) {
    m.array().rowwise() *= v;
    benchmark::DoNotOptimize(m.data());
  }
  state.SetBytesProcessed(state.iterations() * rows * cols * sizeof(Scalar) * 2);
}

// Broadcast feeding a transcendental (softmax-style normalization):
// out = exp(m - v replicated across columns).
template <typename Scalar>
static void BM_ColwiseBroadcastSubExp(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Mat m = Mat::Random(rows, cols);
  Vec v = Vec::Random(rows);
  Mat out(rows, cols);
  for (auto _ : state) {
    out = (m.colwise() - v).array().exp();
    benchmark::DoNotOptimize(out.data());
  }
  state.SetBytesProcessed(state.iterations() * rows * cols * sizeof(Scalar) * 2);
}

// Same operation with an explicit per-column loop: upper-bound reference for
// BM_ColwiseBroadcastSubExp.
template <typename Scalar>
static void BM_PerColBroadcastSubExp(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Mat m = Mat::Random(rows, cols);
  Vec v = Vec::Random(rows);
  Mat out(rows, cols);
  for (auto _ : state) {
    for (Index j = 0; j < cols; ++j) out.col(j) = (m.col(j) - v).array().exp();
    benchmark::DoNotOptimize(out.data());
  }
  state.SetBytesProcessed(state.iterations() * rows * cols * sizeof(Scalar) * 2);
}

// --- Size configurations ---
// clang-format off
// Square matrices; tall-thin (many rows, few cols); short-wide (few rows, many cols).
#define BROADCAST_SIZES \
    ->Args({64, 64})->Args({128, 128})->Args({256, 256})->Args({512, 512})->Args({1024, 1024}) \
    ->Args({10000, 32})->Args({32, 10000})

// --- Register: float ---
BENCHMARK(BM_ColwiseSum<float>) BROADCAST_SIZES ->Name("ColwiseSum_float");
BENCHMARK(BM_ColwiseMean<float>) BROADCAST_SIZES ->Name("ColwiseMean_float");
BENCHMARK(BM_ColwiseNorm<float>) BROADCAST_SIZES ->Name("ColwiseNorm_float");
BENCHMARK(BM_ColwiseMinCoeff<float>) BROADCAST_SIZES ->Name("ColwiseMinCoeff_float");
BENCHMARK(BM_ColwiseMaxCoeff<float>) BROADCAST_SIZES ->Name("ColwiseMaxCoeff_float");
BENCHMARK(BM_RowwiseSum<float>) BROADCAST_SIZES ->Name("RowwiseSum_float");
BENCHMARK(BM_RowwiseNorm<float>) BROADCAST_SIZES ->Name("RowwiseNorm_float");
BENCHMARK(BM_RowwiseBroadcastAdd<float>) BROADCAST_SIZES ->Name("RowwiseBroadcastAdd_float");
BENCHMARK(BM_ColwiseBroadcastAdd<float>) BROADCAST_SIZES ->Name("ColwiseBroadcastAdd_float");
BENCHMARK(BM_RowwiseBroadcastMul<float>) BROADCAST_SIZES ->Name("RowwiseBroadcastMul_float");
BENCHMARK(BM_ColwiseBroadcastSubExp<float>) BROADCAST_SIZES ->Name("ColwiseBroadcastSubExp_float");
BENCHMARK(BM_PerColBroadcastSubExp<float>) BROADCAST_SIZES ->Name("PerColBroadcastSubExp_float");

// --- Register: double ---
BENCHMARK(BM_ColwiseSum<double>) BROADCAST_SIZES ->Name("ColwiseSum_double");
BENCHMARK(BM_ColwiseMean<double>) BROADCAST_SIZES ->Name("ColwiseMean_double");
BENCHMARK(BM_ColwiseNorm<double>) BROADCAST_SIZES ->Name("ColwiseNorm_double");
BENCHMARK(BM_ColwiseMinCoeff<double>) BROADCAST_SIZES ->Name("ColwiseMinCoeff_double");
BENCHMARK(BM_ColwiseMaxCoeff<double>) BROADCAST_SIZES ->Name("ColwiseMaxCoeff_double");
BENCHMARK(BM_RowwiseSum<double>) BROADCAST_SIZES ->Name("RowwiseSum_double");
BENCHMARK(BM_RowwiseNorm<double>) BROADCAST_SIZES ->Name("RowwiseNorm_double");
BENCHMARK(BM_RowwiseBroadcastAdd<double>) BROADCAST_SIZES ->Name("RowwiseBroadcastAdd_double");
BENCHMARK(BM_ColwiseBroadcastAdd<double>) BROADCAST_SIZES ->Name("ColwiseBroadcastAdd_double");
BENCHMARK(BM_RowwiseBroadcastMul<double>) BROADCAST_SIZES ->Name("RowwiseBroadcastMul_double");
BENCHMARK(BM_ColwiseBroadcastSubExp<double>) BROADCAST_SIZES ->Name("ColwiseBroadcastSubExp_double");
BENCHMARK(BM_PerColBroadcastSubExp<double>) BROADCAST_SIZES ->Name("PerColBroadcastSubExp_double");

#undef BROADCAST_SIZES
// clang-format on
