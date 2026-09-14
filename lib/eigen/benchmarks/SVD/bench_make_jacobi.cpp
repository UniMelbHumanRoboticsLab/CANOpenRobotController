// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Jacobi>

#include <array>
#include <complex>
#include <cstdlib>

template <typename Scalar>
static void BM_MakeJacobi(benchmark::State& state) {
  using RealScalar = typename Eigen::NumTraits<Scalar>::Real;
  struct Input {
    RealScalar x;
    Scalar y;
    RealScalar z;
  };
  std::array<Input, 256> inputs;
  std::srand(5489);
  const RealScalar couplingScale =
      state.range(0) == 0 ? RealScalar(1) : Eigen::numext::sqrt(Eigen::NumTraits<RealScalar>::epsilon());
  for (auto& input : inputs) {
    input.x = Eigen::internal::random<RealScalar>();
    input.z = Eigen::internal::random<RealScalar>();
    input.y = Eigen::internal::random<Scalar>() * couplingScale;
  }
  // Hide the generated range, not just the outputs: range propagation can eliminate branches under test.
  benchmark::DoNotOptimize(inputs.data());
  benchmark::ClobberMemory();

  const RealScalar tolerance = RealScalar(64) * Eigen::NumTraits<RealScalar>::epsilon();
  for (const auto& input : inputs) {
    Eigen::JacobiRotation<Scalar> rotation;
    rotation.makeJacobi(input.x, input.y, input.z);
    const Scalar c = rotation.c();
    const Scalar s = rotation.s();
    const Scalar conjugateS = Eigen::numext::conj(s);
    const Scalar offdiag =
        c * conjugateS * (input.x - input.z) + c * c * input.y - conjugateS * conjugateS * Eigen::numext::conj(input.y);
    if (!(Eigen::numext::abs(Eigen::numext::abs2(c) + Eigen::numext::abs2(s) - RealScalar(1)) <= tolerance) ||
        !(Eigen::numext::abs(offdiag) <= tolerance * Eigen::numext::abs(input.y))) {
      state.SkipWithError("Jacobi rotation failed unitarity or annihilation check");
      return;
    }
  }

  for (auto _ : state) {
    benchmark::ClobberMemory();
    for (const auto& input : inputs) {
      Eigen::JacobiRotation<Scalar> rotation;
      rotation.makeJacobi(input.x, input.y, input.z);
      Scalar c = rotation.c();
      Scalar s = rotation.s();
      benchmark::DoNotOptimize(c);
      benchmark::DoNotOptimize(s);
    }
  }
  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(inputs.size()));
}

// 0: mixed ordinary couplings; 1: small couplings representative of later Jacobi sweeps.
BENCHMARK_TEMPLATE(BM_MakeJacobi, float)->DenseRange(0, 1);
BENCHMARK_TEMPLATE(BM_MakeJacobi, double)->DenseRange(0, 1);
BENCHMARK_TEMPLATE(BM_MakeJacobi, std::complex<float>)->DenseRange(0, 1);
BENCHMARK_TEMPLATE(BM_MakeJacobi, std::complex<double>)->DenseRange(0, 1);
