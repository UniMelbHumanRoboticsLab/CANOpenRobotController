// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>
#include <contrib/Eigen/StructuredMatrices>

using Eigen::Dynamic;
using Eigen::Index;
using Eigen::Matrix;

template <typename Scalar>
static void BM_CirculantInverseReciprocals(benchmark::State& state) {
  using Vec = Matrix<Scalar, Dynamic, 1>;
  using RealScalar = typename Eigen::NumTraits<Scalar>::Real;
  using Operator = Eigen::Circulant<Scalar>;
  using Generator = typename Operator::GeneratorType;
  const Index size = state.range(0), kind = state.range(1);
  Generator generator = Generator::Zero(size);
  if (kind == 0) {
    for (Index k = 0; k < size; ++k) generator(k) = Scalar(RealScalar(k % 17 - 8) / RealScalar(16));
    generator(0) += Scalar(2 * size);
  } else {
    generator(0) = Scalar(1);  // Real symbol: exercises the scalar fallback.
  }
  const Operator op(generator);
  Vec expected(size);
  for (Index k = 0; k < size; ++k) expected(k) = Scalar(RealScalar(k % 13 - 6) / RealScalar(8));
  const Vec rhs = op * expected;
  Operator inverse = op.inverse();
  const Vec actual = inverse * rhs;
  if (!actual.allFinite() ||
      (actual - expected).norm() > RealScalar(128) * Eigen::NumTraits<RealScalar>::epsilon() * expected.norm()) {
    state.SkipWithError("Circulant inverse round-trip check failed");
    return;
  }
  for (auto _ : state) {
    inverse = op.inverse();
    benchmark::DoNotOptimize(inverse);
    benchmark::ClobberMemory();
  }
}

BENCHMARK_TEMPLATE(BM_CirculantInverseReciprocals, float)->ArgsProduct({{25, 64, 1024, 16384}, {0, 1}});
BENCHMARK_TEMPLATE(BM_CirculantInverseReciprocals, double)->ArgsProduct({{25, 64, 1024, 16384}, {0, 1}});
BENCHMARK_TEMPLATE(BM_CirculantInverseReciprocals, std::complex<float>)->ArgsProduct({{25, 64, 1024, 16384}, {0, 1}});
BENCHMARK_TEMPLATE(BM_CirculantInverseReciprocals, std::complex<double>)->ArgsProduct({{25, 64, 1024, 16384}, {0, 1}});
