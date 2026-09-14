// Benchmarks for QR decompositions.
//
// Tests HouseholderQR, ColPivHouseholderQR, FullPivHouseholderQR, and COD.
// Both square and tall-thin matrix shapes are tested.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/QR>

using namespace Eigen;

template <typename QR>
EIGEN_DONT_INLINE void do_compute(QR& qr, const typename QR::MatrixType& A) {
  qr.compute(A);
}

// --- HouseholderQR ---

template <typename Scalar>
static void BM_HouseholderQR(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(rows, cols);
  HouseholderQR<Mat> qr(rows, cols);
  for (auto _ : state) {
    do_compute(qr, A);
    benchmark::DoNotOptimize(qr.matrixQR().data());
  }
  state.SetItemsProcessed(state.iterations());
}

// --- ColPivHouseholderQR ---

template <typename Scalar>
static void BM_ColPivHouseholderQR(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(rows, cols);
  ColPivHouseholderQR<Mat> qr(rows, cols);
  for (auto _ : state) {
    do_compute(qr, A);
    benchmark::DoNotOptimize(qr.matrixQR().data());
  }
  state.SetItemsProcessed(state.iterations());
}

// --- RandColPivHouseholderQR ---

template <typename Scalar>
static void BM_RandColPivHouseholderQR(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(rows, cols);
  RandColPivHouseholderQR<Mat> qr(rows, cols);
  qr.setSeed(0xbe9c4);
  for (auto _ : state) {
    do_compute(qr, A);
    benchmark::DoNotOptimize(qr.matrixQR().data());
  }
  state.SetItemsProcessed(state.iterations());
}

// --- FullPivHouseholderQR ---

template <typename Scalar>
static void BM_FullPivHouseholderQR(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(rows, cols);
  FullPivHouseholderQR<Mat> qr(rows, cols);
  for (auto _ : state) {
    do_compute(qr, A);
    benchmark::DoNotOptimize(qr.matrixQR().data());
  }
  state.SetItemsProcessed(state.iterations());
}

// --- CompleteOrthogonalDecomposition (COD) ---

template <typename Scalar>
static void BM_COD(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(rows, cols);
  CompleteOrthogonalDecomposition<Mat> cod(rows, cols);
  for (auto _ : state) {
    do_compute(cod, A);
    benchmark::DoNotOptimize(cod.matrixQTZ().data());
  }
  state.SetItemsProcessed(state.iterations());
}

// --- RandCompleteOrthogonalDecomposition (RandCOD) ---

template <typename Scalar>
static void BM_RandCOD(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(rows, cols);
  RandCompleteOrthogonalDecomposition<Mat> cod(rows, cols);
  cod.setSeed(0xbe9c4);
  for (auto _ : state) {
    do_compute(cod, A);
    benchmark::DoNotOptimize(cod.matrixQTZ().data());
  }
  state.SetItemsProcessed(state.iterations());
}

// --- QR solve ---

template <typename Scalar>
static void BM_HouseholderQR_Solve(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Mat A = Mat::Random(rows, cols);
  Vec b = Vec::Random(rows);
  HouseholderQR<Mat> qr(A);
  Vec x(cols);
  for (auto _ : state) {
    x = qr.solve(b);
    benchmark::DoNotOptimize(x.data());
  }
  state.SetItemsProcessed(state.iterations());
}

// --- Size configurations ---

// clang-format off
// Square sizes + tall-thin sizes.
#define QR_SIZES \
    ->Args({32, 32})->Args({64, 64})->Args({128, 128})->Args({256, 256})->Args({512, 512})->Args({1024, 1024}) \
    ->Args({1000, 32})->Args({1000, 100})->Args({10000, 32})->Args({10000, 100})

// Register: float
BENCHMARK(BM_HouseholderQR<float>) QR_SIZES ->Name("HouseholderQR_float");
BENCHMARK(BM_ColPivHouseholderQR<float>) QR_SIZES ->Name("ColPivHouseholderQR_float");
BENCHMARK(BM_RandColPivHouseholderQR<float>) QR_SIZES ->Name("RandColPivHouseholderQR_float");
BENCHMARK(BM_FullPivHouseholderQR<float>) QR_SIZES ->Name("FullPivHouseholderQR_float");
BENCHMARK(BM_COD<float>) QR_SIZES ->Name("COD_float");
BENCHMARK(BM_RandCOD<float>) QR_SIZES ->Name("RandCOD_float");
BENCHMARK(BM_HouseholderQR_Solve<float>) QR_SIZES ->Name("HouseholderQR_Solve_float");

// Register: double
BENCHMARK(BM_HouseholderQR<double>) QR_SIZES ->Name("HouseholderQR_double");
BENCHMARK(BM_ColPivHouseholderQR<double>) QR_SIZES ->Name("ColPivHouseholderQR_double");
BENCHMARK(BM_RandColPivHouseholderQR<double>) QR_SIZES ->Name("RandColPivHouseholderQR_double");
BENCHMARK(BM_FullPivHouseholderQR<double>) QR_SIZES ->Name("FullPivHouseholderQR_double");
BENCHMARK(BM_COD<double>) QR_SIZES ->Name("COD_double");
BENCHMARK(BM_RandCOD<double>) QR_SIZES ->Name("RandCOD_double");
BENCHMARK(BM_HouseholderQR_Solve<double>) QR_SIZES ->Name("HouseholderQR_Solve_double");

#undef QR_SIZES
// clang-format on
