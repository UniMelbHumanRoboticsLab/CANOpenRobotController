// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>
#include <contrib/Eigen/StructuredMatrices>

using Eigen::Circulant;
using Eigen::Dynamic;
using Eigen::Index;
using Eigen::Matrix;

template <typename Scalar>
static void BM_CirculantSolveReciprocals(benchmark::State& state) {
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using RealScalar = typename Eigen::NumTraits<Scalar>::Real;
  const Index n = state.range(0), nrhs = state.range(1), kind = state.range(2);
  Matrix<Scalar, Dynamic, 1> generator = Matrix<Scalar, Dynamic, 1>::Zero(n);
  if (kind == 0) {
    for (Index k = 0; k < n; ++k) generator(k) = Scalar(RealScalar(k % 17 - 8) / RealScalar(16));
    generator(0, 0) += Scalar(2 * n);
  } else if (kind == 1) {
    generator(0, 0) = Scalar(1);  // Real symbol: exercises the scalar fallback.
  } else {
    generator.setConstant(Scalar(1));  // Only the DC mode survives the threshold.
  }
  const Circulant<Scalar> op(generator);
  Mat expected(n, nrhs);
  for (Index k = 0; k < expected.size(); ++k)
    expected(k) = kind == 2 ? Scalar(1) : Scalar(RealScalar(k % 13 - 6) / RealScalar(8));
  const Mat rhs = op * expected;
  Mat solution = op.solve(rhs);
  // The round trip through these well-conditioned operators costs O(log N) eps.
  if (!solution.allFinite() ||
      (solution - expected).norm() > RealScalar(128) * Eigen::NumTraits<RealScalar>::epsilon() * expected.norm()) {
    state.SkipWithError("Circulant solve round-trip check failed");
    return;
  }
  for (auto _ : state) {
    solution = op.solve(rhs);
    benchmark::DoNotOptimize(solution.data());
    benchmark::ClobberMemory();
  }
}

BENCHMARK_TEMPLATE(BM_CirculantSolveReciprocals, float)->ArgsProduct({{15, 64, 1024, 16384}, {1, 8}, {0, 1, 2}});
BENCHMARK_TEMPLATE(BM_CirculantSolveReciprocals, double)->ArgsProduct({{15, 64, 1024, 16384}, {1, 8}, {0, 1, 2}});
BENCHMARK_TEMPLATE(BM_CirculantSolveReciprocals, std::complex<float>)
    ->ArgsProduct({{15, 64, 1024, 16384}, {1, 8}, {0, 1, 2}});
BENCHMARK_TEMPLATE(BM_CirculantSolveReciprocals, std::complex<double>)
    ->ArgsProduct({{15, 64, 1024, 16384}, {1, 8}, {0, 1, 2}});
