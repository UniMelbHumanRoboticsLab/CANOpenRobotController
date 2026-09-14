// Benchmarks for self-adjoint (symmetric/hermitian) matrix operations.
//
// Tests SYMM (selfadjointView * dense) and rank-k updates.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>

using namespace Eigen;

// C = selfadjointView<Lower>(A) * B  (SYMM)
template <typename Scalar>
static void BM_SYMM_Left(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(n, n);
  A = (A + A.transpose()).eval() / Scalar(2);
  Mat B = Mat::Random(n, n);
  Mat C(n, n);
  for (auto _ : state) {
    C.noalias() = A.template selfadjointView<Lower>() * B;
    benchmark::DoNotOptimize(C.data());
  }
  state.counters["GFLOPS"] =
      benchmark::Counter(2.0 * n * n * n, benchmark::Counter::kIsIterationInvariantRate, benchmark::Counter::kIs1000);
}

// C = B * selfadjointView<Lower>(A)
template <typename Scalar>
static void BM_SYMM_Right(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(n, n);
  A = (A + A.transpose()).eval() / Scalar(2);
  Mat B = Mat::Random(n, n);
  Mat C(n, n);
  for (auto _ : state) {
    C.noalias() = B * A.template selfadjointView<Lower>();
    benchmark::DoNotOptimize(C.data());
  }
  state.counters["GFLOPS"] =
      benchmark::Counter(2.0 * n * n * n, benchmark::Counter::kIsIterationInvariantRate, benchmark::Counter::kIs1000);
}

// C = selfadjointView<Lower>(A) * B with few columns: O(n^2) SYMM packing of
// A against O(n^2 m) FLOPs, so this measures the packers, not the kernel.
template <typename Scalar>
static void BM_SYMM_Left_Thin(benchmark::State& state) {
  const Index n = state.range(0);
  const Index m = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(n, n);
  A = (A + A.transpose()).eval() / Scalar(2);
  Mat B = Mat::Random(n, m);
  Mat C(n, m);
  for (auto _ : state) {
    C.noalias() = A.template selfadjointView<Lower>() * B;
    benchmark::DoNotOptimize(C.data());
  }
  state.counters["GFLOPS"] =
      benchmark::Counter(2.0 * n * n * m, benchmark::Counter::kIsIterationInvariantRate, benchmark::Counter::kIs1000);
}

// C = B * selfadjointView<Lower>(A) with few rows.
template <typename Scalar>
static void BM_SYMM_Right_Thin(benchmark::State& state) {
  const Index n = state.range(0);
  const Index m = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(n, n);
  A = (A + A.transpose()).eval() / Scalar(2);
  Mat B = Mat::Random(m, n);
  Mat C(m, n);
  for (auto _ : state) {
    C.noalias() = B * A.template selfadjointView<Lower>();
    benchmark::DoNotOptimize(C.data());
  }
  state.counters["GFLOPS"] =
      benchmark::Counter(2.0 * n * n * m, benchmark::Counter::kIsIterationInvariantRate, benchmark::Counter::kIs1000);
}

// Rank-k update: C.selfadjointView<Lower>().rankUpdate(A)
// Computes C += A * A^T
template <typename Scalar>
static void BM_RankUpdate(benchmark::State& state) {
  const Index n = state.range(0);
  const Index k = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(n, k);
  Mat C = Mat::Zero(n, n);
  for (auto _ : state) {
    C.template selfadjointView<Lower>().rankUpdate(A);
    benchmark::DoNotOptimize(C.data());
  }
  state.counters["GFLOPS"] =
      benchmark::Counter(1.0 * n * n * k, benchmark::Counter::kIsIterationInvariantRate, benchmark::Counter::kIs1000);
}

// clang-format off
BENCHMARK(BM_SYMM_Left<float>)->Arg(64)->Arg(128)->Arg(256)->Arg(512)->Arg(1024)->Name("SYMM_Left_float");
BENCHMARK(BM_SYMM_Left<double>)->Arg(64)->Arg(128)->Arg(256)->Arg(512)->Arg(1024)->Name("SYMM_Left_double");
BENCHMARK(BM_SYMM_Right<float>)->Arg(64)->Arg(128)->Arg(256)->Arg(512)->Arg(1024)->Name("SYMM_Right_float");
BENCHMARK(BM_SYMM_Right<double>)->Arg(64)->Arg(128)->Arg(256)->Arg(512)->Arg(1024)->Name("SYMM_Right_double");
BENCHMARK(BM_SYMM_Left_Thin<float>)->ArgsProduct({{256, 512, 1024, 2048}, {4, 8, 32}})->Name("SYMM_Left_Thin_float");
BENCHMARK(BM_SYMM_Left_Thin<double>)->ArgsProduct({{256, 512, 1024, 2048}, {4, 8, 32}})->Name("SYMM_Left_Thin_double");
BENCHMARK(BM_SYMM_Right_Thin<float>)->ArgsProduct({{256, 512, 1024, 2048}, {4, 8, 32}})->Name("SYMM_Right_Thin_float");
BENCHMARK(BM_SYMM_Right_Thin<double>)->ArgsProduct({{256, 512, 1024, 2048}, {4, 8, 32}})->Name("SYMM_Right_Thin_double");
BENCHMARK(BM_RankUpdate<float>)->ArgsProduct({{64, 128, 256, 512}, {16, 64, 256}})->Name("RankUpdate_float");
BENCHMARK(BM_RankUpdate<double>)->ArgsProduct({{64, 128, 256, 512}, {16, 64, 256}})->Name("RankUpdate_double");
// clang-format on
