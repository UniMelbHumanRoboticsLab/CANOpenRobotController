// Benchmarks for the Cauchy operator: the O(mn)-flop, O(m+n)-storage direct
// product against its dense equivalents (with and without materializing the
// matrix first), and the O(n^2) GKO pivoted solver (CauchyLU) against the
// O(n^3) dense PartialPivLU of the materialized matrix.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <contrib/Eigen/StructuredMatrices>

using namespace Eigen;

typedef Matrix<double, Dynamic, 1> Vec;
typedef Matrix<double, Dynamic, Dynamic> Mat;

// After reversing its rows, this Cauchy matrix has diagonal 4n and off-diagonal
// row sums below 2 H_n. It is well conditioned while still exercising row
// pivoting in CauchyLU.
static void wellConditionedNodes(Index n, Vec& x, Vec& y) {
  x.resize(n);
  y.resize(n);
  const double delta = 1.0 / (4.0 * static_cast<double>(n));
  for (Index i = 0; i < n; ++i) {
    x[i] = static_cast<double>(n - 1 - i);
    y[i] = static_cast<double>(i) - delta;
  }
}

static Mat denseCauchy(const Vec& x, const Vec& y) {
  Mat dense(x.size(), y.size());
  for (Index j = 0; j < y.size(); ++j)
    for (Index i = 0; i < x.size(); ++i) dense(i, j) = 1.0 / (x[i] - y[j]);
  return dense;
}

static Vec benchmarkVector(Index n) { return Vec::LinSpaced(n, -1.0, 1.0); }

static bool productIsAccurate(const Vec& product, const Vec& x, const Vec& y, const Vec& vector) {
  if (!product.allFinite()) return false;
  Vec reference = Vec::Zero(x.size());
  for (Index i = 0; i < x.size(); ++i)
    for (Index j = 0; j < y.size(); ++j) reference[i] += vector[j] / (x[i] - y[j]);
  const double tolerance = 64.0 * static_cast<double>(y.size()) * NumTraits<double>::epsilon();
  return (product - reference).norm() <= tolerance * (reference.norm() + 1.0);
}

static bool solveIsAccurate(const Mat& matrix, const Vec& solution, const Vec& rhs) {
  if (!solution.allFinite()) return false;
  const Vec residual = matrix * solution - rhs;
  if (!residual.allFinite()) return false;
  const double scale = matrix.norm() * solution.norm() + rhs.norm();
  const double tolerance = 512.0 * static_cast<double>(matrix.rows()) * NumTraits<double>::epsilon();
  return residual.norm() <= tolerance * scale;
}

// --- Matrix-vector product w = C * v ---
static void BM_CauchyProduct(benchmark::State& state) {
  const Index n = state.range(0);
  Vec x, y;
  wellConditionedNodes(n, x, y);
  Cauchy<double> C(x, y);  // O(n) storage; entries generated on the fly
  Vec v = benchmarkVector(n), w(n);
  {
    const Vec product = C * v;
    if (!productIsAccurate(product, x, y, v)) {
      state.SkipWithError("Cauchy product validation failed");
      return;
    }
  }
  for (auto _ : state) {
    w.noalias() = C * v;
    benchmark::DoNotOptimize(w.data());
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CauchyProduct)->Arg(64)->Arg(256)->Arg(1024);

static void BM_DenseCauchyProduct(benchmark::State& state) {
  // The dense product once the matrix has been materialized (O(n^2) storage).
  const Index n = state.range(0);
  Vec x, y;
  wellConditionedNodes(n, x, y);
  Mat dense = denseCauchy(x, y);
  Vec v = benchmarkVector(n), w(n);
  {
    const Vec product = dense * v;
    if (!productIsAccurate(product, x, y, v)) {
      state.SkipWithError("dense product validation failed");
      return;
    }
  }
  for (auto _ : state) {
    w.noalias() = dense * v;
    benchmark::DoNotOptimize(w.data());
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_DenseCauchyProduct)->Arg(64)->Arg(256)->Arg(1024);

static void BM_DenseCauchyMaterializeProduct(benchmark::State& state) {
  // The dense equivalent when the matrix is not kept around: materialize from
  // the node vectors, then multiply.
  const Index n = state.range(0);
  Vec x, y;
  wellConditionedNodes(n, x, y);
  Cauchy<double> C(x, y);
  Vec v = benchmarkVector(n), w(n);
  {
    const Mat dense = C;
    const Vec product = dense * v;
    if (!productIsAccurate(product, x, y, v)) {
      state.SkipWithError("materialized product validation failed");
      return;
    }
  }
  for (auto _ : state) {
    Mat dense = C;
    w.noalias() = dense * v;
    benchmark::DoNotOptimize(w.data());
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_DenseCauchyMaterializeProduct)->Arg(64)->Arg(256)->Arg(1024);

// --- Square solve C * u = b from the node vectors ---
static void BM_CauchyLUSolve(benchmark::State& state) {
  const Index n = state.range(0);
  Vec x, y;
  wellConditionedNodes(n, x, y);
  Cauchy<double> C(x, y);
  Vec b(n);
  {
    const Vec expected = benchmarkVector(n);
    const Mat dense = C;
    b = dense * expected;
    CauchyLU<double> check(C);
    if (check.info() != Success) {
      state.SkipWithError("CauchyLU factorization failed");
      return;
    }
    const Vec solution = check.solve(b);
    if (!solveIsAccurate(dense, solution, b)) {
      state.SkipWithError("CauchyLU validation failed");
      return;
    }
  }
  Vec u(n);
  for (auto _ : state) {
    CauchyLU<double> lu(C);  // O(n^2) pivoted factorization (GKO)
    u = lu.solve(b);
    benchmark::DoNotOptimize(u.data());
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CauchyLUSolve)->Arg(64)->Arg(256)->Arg(1024);

static void BM_DensePartialPivLUSolve(benchmark::State& state) {
  const Index n = state.range(0);
  Vec x, y;
  wellConditionedNodes(n, x, y);
  Mat dense = denseCauchy(x, y);
  Vec b(n);
  {
    const Vec expected = benchmarkVector(n);
    b = dense * expected;
    PartialPivLU<Mat> check(dense);
    const Vec solution = check.solve(b);
    if (!solveIsAccurate(dense, solution, b)) {
      state.SkipWithError("PartialPivLU validation failed");
      return;
    }
  }
  Vec u(n);
  for (auto _ : state) {
    PartialPivLU<Mat> lu(dense);  // O(n^3) pivoted factorization
    u = lu.solve(b);
    benchmark::DoNotOptimize(u.data());
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_DensePartialPivLUSolve)->Arg(64)->Arg(256)->Arg(1024);
