// Benchmarks for DPR1EigenSolver, the direct O(n^2) secular-equation
// eigensolver for real symmetric diagonal-plus-rank-one matrices
// D + rho*z*z^T, against the dense O(n^3) SelfAdjointEigenSolver applied to
// the materialized matrix. The dense matrix is formed outside the timed loop,
// so the dense side is not even charged for materialization.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Dense>
#include <contrib/Eigen/StructuredMatrices>

using namespace Eigen;

typedef Matrix<double, Dynamic, 1> Vec;
typedef Matrix<double, Dynamic, Dynamic> Mat;

struct DPR1Problem {
  explicit DPR1Problem(Index n) : d(n), z(n), rho(2.0) {
    for (Index i = 0; i < n; ++i) {
      d[i] = 2.0 * double((37 * i) % n) / double(n - 1) - 1.0;
      z[i] = 2.0 * (double((53 * i + 1) % n) + 0.5) / double(n) - 1.0;
    }
    dense = Mat(d.asDiagonal()) + rho * z * z.transpose();
  }

  Vec d;
  Vec z;
  double rho;
  Mat dense;
};

static bool validEigenvectors(const DPR1Problem& problem, const Vec& eigenvalues, const Mat& eigenvectors, double tol,
                              double scale) {
  const Index n = problem.d.size();
  const double residual = (problem.dense * eigenvectors - eigenvectors * eigenvalues.asDiagonal()).norm();
  const double orthogonality = (eigenvectors.transpose() * eigenvectors - Mat::Identity(n, n)).norm();
  return residual <= tol * scale * numext::sqrt(double(n)) && orthogonality <= tol;
}

static bool preflight(const DPR1Problem& problem, int options) {
  const Index n = problem.d.size();
  const double scale = problem.d.cwiseAbs().maxCoeff() + numext::abs(problem.rho) * problem.z.squaredNorm();
  const double tol = 100.0 * double(n) * NumTraits<double>::epsilon();
  const bool computeVectors = (options & ComputeEigenvectors) == ComputeEigenvectors;

  DPR1EigenSolver<double> dpr1(problem.d, problem.rho, problem.z, options);
  SelfAdjointEigenSolver<Mat> dense(problem.dense, options);
  if (dpr1.info() != Success || dense.info() != Success || !dpr1.eigenvalues().allFinite() ||
      !dense.eigenvalues().allFinite()) {
    return false;
  }
  if ((dpr1.eigenvalues() - dense.eigenvalues()).cwiseAbs().maxCoeff() > tol * scale) return false;
  if (!computeVectors) return true;
  return validEigenvectors(problem, dpr1.eigenvalues(), dpr1.eigenvectors(), tol, scale) &&
         validEigenvectors(problem, dense.eigenvalues(), dense.eigenvectors(), tol, scale);
}

// --- full decomposition: eigenvalues + orthogonal eigenvectors ---
static void BM_DPR1EigenSolver(benchmark::State& state) {
  const Index n = state.range(0);
  const DPR1Problem problem(n);
  if (!preflight(problem, ComputeEigenvectors)) {
    state.SkipWithError("DPR1 benchmark preflight failed");
    return;
  }
  for (auto _ : state) {
    DPR1EigenSolver<double> es(problem.d, problem.rho, problem.z);
    benchmark::DoNotOptimize(es.eigenvalues().data());
    benchmark::DoNotOptimize(es.eigenvectors().data());
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_DPR1EigenSolver)->Arg(32)->Arg(128)->Arg(512)->Arg(1024);

static void BM_DenseSelfAdjointEigenSolver(benchmark::State& state) {
  const Index n = state.range(0);
  const DPR1Problem problem(n);
  if (!preflight(problem, ComputeEigenvectors)) {
    state.SkipWithError("dense benchmark preflight failed");
    return;
  }
  for (auto _ : state) {
    SelfAdjointEigenSolver<Mat> es(problem.dense);
    benchmark::DoNotOptimize(es.eigenvalues().data());
    benchmark::DoNotOptimize(es.eigenvectors().data());
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_DenseSelfAdjointEigenSolver)->Arg(32)->Arg(128)->Arg(512)->Arg(1024);

// --- eigenvalues only ---
static void BM_DPR1EigenSolverValuesOnly(benchmark::State& state) {
  const Index n = state.range(0);
  const DPR1Problem problem(n);
  if (!preflight(problem, EigenvaluesOnly)) {
    state.SkipWithError("DPR1 values-only benchmark preflight failed");
    return;
  }
  for (auto _ : state) {
    DPR1EigenSolver<double> es(problem.d, problem.rho, problem.z, EigenvaluesOnly);
    benchmark::DoNotOptimize(es.eigenvalues().data());
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_DPR1EigenSolverValuesOnly)->Arg(32)->Arg(128)->Arg(512)->Arg(1024);

static void BM_DenseSelfAdjointEigenSolverValuesOnly(benchmark::State& state) {
  const Index n = state.range(0);
  const DPR1Problem problem(n);
  if (!preflight(problem, EigenvaluesOnly)) {
    state.SkipWithError("dense values-only benchmark preflight failed");
    return;
  }
  for (auto _ : state) {
    SelfAdjointEigenSolver<Mat> es(problem.dense, EigenvaluesOnly);
    benchmark::DoNotOptimize(es.eigenvalues().data());
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_DenseSelfAdjointEigenSolverValuesOnly)->Arg(32)->Arg(128)->Arg(512)->Arg(1024);
