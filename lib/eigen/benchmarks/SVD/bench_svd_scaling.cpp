// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#ifndef SCALAR
#define SCALAR double
#endif

using Scalar = SCALAR;
using Matrix = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
#ifdef EIGEN_BENCH_BDCSVD
using Solver = Eigen::BDCSVD<Matrix, Eigen::ComputeThinU | Eigen::ComputeThinV>;
#else
using Solver = Eigen::JacobiSVD<Matrix, Eigen::ComputeThinU | Eigen::ComputeThinV>;
#endif

static void BM_SVDScaling(benchmark::State& state) {
  const Eigen::Index rows = state.range(0);
  const Eigen::Index cols = state.range(1);
  const int scaleKind = int(state.range(2));
  const int exponent = scaleKind == 1   ? std::numeric_limits<Scalar>::max_exponent - 8
                       : scaleKind == 2 ? std::numeric_limits<Scalar>::min_exponent + 4
                                        : 0;
  const Scalar scale = std::ldexp(Scalar(1), exponent);
  constexpr int kBatchSize = 8;
  std::mt19937 random(12345);
  std::vector<Matrix> inputs;
  inputs.reserve(kBatchSize);
  Solver solver(rows, cols);
  const Scalar tolerance = Scalar(32 * (std::max)(rows, cols)) * Eigen::NumTraits<Scalar>::epsilon();
  for (int sample = 0; sample < kBatchSize; ++sample) {
    Matrix reference(rows, cols);
    for (Eigen::Index i = 0; i < reference.size(); ++i) {
      reference.data()[i] = Scalar(int(random() % 2001) - 1000) / Scalar(1024);
    }
    inputs.emplace_back(reference * scale);
    benchmark::DoNotOptimize(inputs.back().data());
    benchmark::ClobberMemory();
    solver.compute(inputs.back());
    if (solver.info() != Eigen::Success) {
      state.SkipWithError("SVD did not converge");
      return;
    }
    // Restore in normalized coordinates so validation itself cannot overflow or underflow.
    const Matrix reconstruction =
        solver.matrixU() * (solver.singularValues() / scale).asDiagonal() * solver.matrixV().adjoint();
    const Scalar residual = (reconstruction - reference).norm() / reference.norm();
    const Eigen::Index rank = (std::min)(rows, cols);
    const Matrix identity = Matrix::Identity(rank, rank);
    const Scalar uError = (solver.matrixU().adjoint() * solver.matrixU() - identity).norm();
    const Scalar vError = (solver.matrixV().adjoint() * solver.matrixV() - identity).norm();
    if (!(residual <= tolerance && uError <= tolerance && vError <= tolerance)) {
      state.SkipWithError("SVD reconstruction or orthogonality check failed");
      return;
    }
  }
  state.SetLabel(scaleKind == 0 ? "ordinary" : scaleKind == 1 ? "large" : "tiny");
  // Every timed batch visits the same matrix pool, independent of iteration-count calibration.
  while (state.KeepRunningBatch(kBatchSize)) {
    for (const Matrix& input : inputs) {
      benchmark::DoNotOptimize(input.data());
      benchmark::ClobberMemory();
      solver.compute(input);
      benchmark::DoNotOptimize(solver.singularValues().data());
      benchmark::DoNotOptimize(solver.matrixU().data());
      benchmark::DoNotOptimize(solver.matrixV().data());
      benchmark::ClobberMemory();
    }
  }
  state.SetItemsProcessed(state.iterations());
}

#ifdef EIGEN_BENCH_BDCSVD
BENCHMARK(BM_SVDScaling)
    ->Args({32, 32, 0})
    ->Args({64, 64, 0})
    ->Args({128, 128, 0})
    ->Args({256, 32, 0})
    ->Args({32, 256, 0})
    ->Args({64, 64, 1})
    ->Args({64, 64, 2})
    ->ArgNames({"rows", "cols", "scale"});
#else
BENCHMARK(BM_SVDScaling)
    ->Args({4, 4, 0})
    ->Args({16, 16, 0})
    ->Args({32, 32, 0})
    ->Args({128, 16, 0})
    ->Args({16, 128, 0})
    ->Args({32, 32, 1})
    ->Args({32, 32, 2})
    ->ArgNames({"rows", "cols", "scale"});
#endif
