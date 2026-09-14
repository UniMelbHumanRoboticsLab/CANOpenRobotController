// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/LU>
#include <cstdlib>

using namespace Eigen;

template <typename Scalar, int StorageOrder>
static void BM_PartialPivLUStride(benchmark::State& state) {
  using Mat = Matrix<Scalar, Dynamic, Dynamic, StorageOrder>;
  const Index n = state.range(0);
  const Index stride = n + state.range(1);
  std::srand(1);
  const Mat a = Mat::Random(n, n);
  Matrix<Scalar, Dynamic, 1> storage(stride * n);
  Map<Mat, 0, OuterStride<>> work(storage.data(), n, n, OuterStride<>(stride));
  work = a;
  PartialPivLU<Ref<Mat>> solver(work);
  const Mat lower = work.template triangularView<UnitLower>();
  const Mat upper = work.template triangularView<Upper>();
  const Scalar residual = (solver.permutationP() * a - lower * upper).norm();
  const Scalar bound = Scalar(64 * n) * NumTraits<Scalar>::epsilon() * a.norm();
  if (!(residual <= bound)) {
    state.SkipWithError("LU reconstruction failed");
    return;
  }
  for (auto _ : state) {
    state.PauseTiming();
    work = a;
    state.ResumeTiming();
    solver.compute(work);
    benchmark::DoNotOptimize(work.data());
    benchmark::ClobberMemory();
  }
}

BENCHMARK_TEMPLATE(BM_PartialPivLUStride, float, ColMajor)
    ->ArgsProduct({{32, 128, 500, 512, 768, 1000, 1001, 1024}, {0, 8}});
BENCHMARK_TEMPLATE(BM_PartialPivLUStride, double, ColMajor)
    ->ArgsProduct({{32, 128, 500, 512, 768, 1000, 1001, 1024}, {0, 8}});
BENCHMARK_TEMPLATE(BM_PartialPivLUStride, float, RowMajor)->ArgsProduct({{32, 128, 500, 512, 1000, 1024}, {0, 8}});
BENCHMARK_TEMPLATE(BM_PartialPivLUStride, double, RowMajor)->ArgsProduct({{32, 128, 500, 512, 1000, 1024}, {0, 8}});
