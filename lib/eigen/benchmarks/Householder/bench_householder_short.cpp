// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Householder>
#include <cstdlib>

using namespace Eigen;

template <typename Scalar, int Size, int Side>
static void BM_ShortHouseholder(benchmark::State& state) {
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  const Index n = state.range(0);
  const Index stride = n + state.range(1);
  std::srand(1);
  Mat storage = Mat::Random(stride, n);
  Matrix<Scalar, Dynamic, 1> workspace(n);
  Matrix<Scalar, Size, 1> vector = Matrix<Scalar, Size, 1>::Random();
  Matrix<Scalar, Size - 1, 1> essential;
  Scalar tau, beta;
  vector.makeHouseholder(essential, tau, beta);
  vector << Scalar(1), essential;
  const Matrix<Scalar, Size, Size> h = Matrix<Scalar, Size, Size>::Identity() - tau * vector * vector.adjoint();
  auto block = storage.block(1, 1, Side == OnTheLeft ? Size : n - 1, Side == OnTheLeft ? n - 1 : Size);
  const Mat original = block;
  Mat expected;
  if (Side == OnTheLeft) {
    expected = h * original;
    block.applyHouseholderOnTheLeft(essential, tau, workspace.data());
  } else {
    expected = original * h;
    block.applyHouseholderOnTheRight(essential, tau, workspace.data());
  }
  if (!((block - expected).norm() <= Scalar(16 * Size) * NumTraits<Scalar>::epsilon() * original.norm())) {
    state.SkipWithError("Householder application failed");
    return;
  }
  for (auto _ : state) {
    if (Side == OnTheLeft)
      block.applyHouseholderOnTheLeft(essential, tau, workspace.data());
    else
      block.applyHouseholderOnTheRight(essential, tau, workspace.data());
    benchmark::DoNotOptimize(block.data());
    benchmark::ClobberMemory();
  }
}

BENCHMARK_TEMPLATE(BM_ShortHouseholder, float, 2, OnTheLeft)->ArgsProduct({{32, 500, 512, 768, 1000, 1024}, {0, 8}});
BENCHMARK_TEMPLATE(BM_ShortHouseholder, float, 3, OnTheLeft)->ArgsProduct({{32, 500, 512, 768, 1000, 1024}, {0, 8}});
BENCHMARK_TEMPLATE(BM_ShortHouseholder, double, 2, OnTheLeft)->ArgsProduct({{32, 500, 512, 768, 1000, 1024}, {0, 8}});
BENCHMARK_TEMPLATE(BM_ShortHouseholder, double, 3, OnTheLeft)->ArgsProduct({{32, 500, 512, 768, 1000, 1024}, {0, 8}});
BENCHMARK_TEMPLATE(BM_ShortHouseholder, float, 3, OnTheRight)->ArgsProduct({{32, 500, 512, 768, 1000, 1024}, {0, 8}});
BENCHMARK_TEMPLATE(BM_ShortHouseholder, double, 3, OnTheRight)->ArgsProduct({{32, 500, 512, 768, 1000, 1024}, {0, 8}});
