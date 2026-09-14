// Benchmarks for LU decompositions.
//
// Tests PartialPivLU and FullPivLU: compute, solve, inverse, determinant.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/LU>

using namespace Eigen;

typedef Matrix<float, Dynamic, Dynamic> Matf;
typedef Matrix<double, Dynamic, Dynamic> Matd;

// --- PartialPivLU ---

template <typename Scalar>
EIGEN_DONT_INLINE void do_compute(PartialPivLU<Matrix<Scalar, Dynamic, Dynamic>>& lu,
                                  const Matrix<Scalar, Dynamic, Dynamic>& A) {
  lu.compute(A);
}

template <typename Scalar>
static void BM_PartialPivLU_Compute(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(n, n);
  PartialPivLU<Mat> lu(n);
  for (auto _ : state) {
    do_compute(lu, A);
    benchmark::DoNotOptimize(lu.matrixLU().data());
  }
  state.SetItemsProcessed(state.iterations());
}

template <typename Scalar>
static void BM_PartialPivLU_Solve(benchmark::State& state) {
  const Index n = state.range(0);
  const Index nrhs = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(n, n);
  Mat B = Mat::Random(n, nrhs);
  PartialPivLU<Mat> lu(A);
  Mat X(n, nrhs);
  for (auto _ : state) {
    X = lu.solve(B);
    benchmark::DoNotOptimize(X.data());
  }
  state.SetItemsProcessed(state.iterations());
}

template <typename Scalar>
static void BM_PartialPivLU_Inverse(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(n, n);
  PartialPivLU<Mat> lu(A);
  Mat inv(n, n);
  for (auto _ : state) {
    inv = lu.inverse();
    benchmark::DoNotOptimize(inv.data());
  }
  state.SetItemsProcessed(state.iterations());
}

template <typename Scalar>
static void BM_PartialPivLU_Determinant(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(n, n);
  PartialPivLU<Mat> lu(A);
  for (auto _ : state) {
    Scalar d = lu.determinant();
    benchmark::DoNotOptimize(d);
  }
  state.SetItemsProcessed(state.iterations());
}

// --- FullPivLU ---

template <typename Scalar>
EIGEN_DONT_INLINE void do_compute(FullPivLU<Matrix<Scalar, Dynamic, Dynamic>>& lu,
                                  const Matrix<Scalar, Dynamic, Dynamic>& A) {
  lu.compute(A);
}

template <typename Scalar>
static void BM_FullPivLU_Compute(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(n, n);
  FullPivLU<Mat> lu(n, n);
  for (auto _ : state) {
    do_compute(lu, A);
    benchmark::DoNotOptimize(lu.matrixLU().data());
  }
  state.SetItemsProcessed(state.iterations());
}

template <typename Scalar>
static void BM_FullPivLU_Solve(benchmark::State& state) {
  const Index n = state.range(0);
  const Index nrhs = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(n, n);
  Mat B = Mat::Random(n, nrhs);
  FullPivLU<Mat> lu(A);
  Mat X(n, nrhs);
  for (auto _ : state) {
    X = lu.solve(B);
    benchmark::DoNotOptimize(X.data());
  }
  state.SetItemsProcessed(state.iterations());
}

// --- Size configurations ---

// clang-format off
BENCHMARK(BM_PartialPivLU_Compute<float>)->Arg(8)->Arg(32)->Arg(64)->Arg(128)->Arg(256)->Arg(512)->Arg(1024)->Name("PartialPivLU_Compute_float");
BENCHMARK(BM_PartialPivLU_Compute<double>)->Arg(8)->Arg(32)->Arg(64)->Arg(128)->Arg(256)->Arg(512)->Arg(1024)->Name("PartialPivLU_Compute_double");
BENCHMARK(BM_PartialPivLU_Solve<float>)->ArgsProduct({{32, 128, 512, 1024}, {1, 16, 64}})->Name("PartialPivLU_Solve_float");
BENCHMARK(BM_PartialPivLU_Solve<double>)->ArgsProduct({{32, 128, 512, 1024}, {1, 16, 64}})->Name("PartialPivLU_Solve_double");
BENCHMARK(BM_PartialPivLU_Inverse<float>)->Arg(8)->Arg(32)->Arg(64)->Arg(128)->Arg(256)->Arg(512)->Arg(1024)->Name("PartialPivLU_Inverse_float");
BENCHMARK(BM_PartialPivLU_Inverse<double>)->Arg(8)->Arg(32)->Arg(64)->Arg(128)->Arg(256)->Arg(512)->Arg(1024)->Name("PartialPivLU_Inverse_double");
BENCHMARK(BM_PartialPivLU_Determinant<float>)->Arg(8)->Arg(32)->Arg(64)->Arg(128)->Arg(256)->Arg(512)->Arg(1024)->Name("PartialPivLU_Determinant_float");
BENCHMARK(BM_PartialPivLU_Determinant<double>)->Arg(8)->Arg(32)->Arg(64)->Arg(128)->Arg(256)->Arg(512)->Arg(1024)->Name("PartialPivLU_Determinant_double");
BENCHMARK(BM_FullPivLU_Compute<float>)->Arg(8)->Arg(32)->Arg(64)->Arg(128)->Arg(256)->Arg(512)->Arg(1024)->Name("FullPivLU_Compute_float");
BENCHMARK(BM_FullPivLU_Compute<double>)->Arg(8)->Arg(32)->Arg(64)->Arg(128)->Arg(256)->Arg(512)->Arg(1024)->Name("FullPivLU_Compute_double");
BENCHMARK(BM_FullPivLU_Solve<float>)->ArgsProduct({{32, 128, 512, 1024}, {1, 16, 64}})->Name("FullPivLU_Solve_float");
BENCHMARK(BM_FullPivLU_Solve<double>)->ArgsProduct({{32, 128, 512, 1024}, {1, 16, 64}})->Name("FullPivLU_Solve_double");
// clang-format on
