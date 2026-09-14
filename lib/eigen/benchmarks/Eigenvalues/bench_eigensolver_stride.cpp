// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Eigenvalues>
#include <cstdlib>

using namespace Eigen;

template <typename Scalar>
static void BM_EigenSolverStride(benchmark::State& state) {
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  const Index n = state.range(0);
  const bool computeVectors = state.range(1) != 0;
  std::srand(1);
  const Mat a = Mat::Random(n, n);
  EigenSolver<Mat> solver(a, true);
  if (solver.info() != Success) {
    state.SkipWithError("EigenSolver did not converge");
    return;
  }
  const Mat vectors = solver.pseudoEigenvectors();
  const Mat values = solver.pseudoEigenvalueMatrix();
  const Scalar residual = (a * vectors - vectors * values).norm();
  const Scalar bound = Scalar(64 * n) * NumTraits<Scalar>::epsilon() * a.norm() * vectors.norm();
  if (!(residual <= bound)) {
    state.SkipWithError("Eigenpair residual failed");
    return;
  }
  for (auto _ : state) {
    solver.compute(a, computeVectors);
    benchmark::DoNotOptimize(solver.eigenvalues().data());
    benchmark::ClobberMemory();
  }
}

BENCHMARK_TEMPLATE(BM_EigenSolverStride, float)->ArgsProduct({{32, 128, 500, 512, 768, 1000, 1001, 1024}, {0, 1}});
BENCHMARK_TEMPLATE(BM_EigenSolverStride, double)->ArgsProduct({{32, 128, 500, 512, 768, 1000, 1001, 1024}, {0, 1}});
