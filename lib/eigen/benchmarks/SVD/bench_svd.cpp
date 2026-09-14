// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Dense>

using namespace Eigen;

// Benchmark JacobiSVD and BDCSVD for various scalar types, matrix shapes,
// and computation options.

// ---------- helpers ----------

template <typename Scalar>
using Mat = Matrix<Scalar, Dynamic, Dynamic>;

template <typename SVD>
EIGEN_DONT_INLINE void do_compute(SVD& svd, const typename SVD::MatrixType& A) {
  svd.compute(A);
}

// ---------- JacobiSVD ----------

template <typename Scalar, int Options>
static void BM_JacobiSVD(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  Mat<Scalar> A = Mat<Scalar>::Random(rows, cols);
  JacobiSVD<Mat<Scalar>, Options> svd(rows, cols);
  for (auto _ : state) {
    do_compute(svd, A);
    benchmark::DoNotOptimize(svd.singularValues().data());
  }
  state.SetItemsProcessed(state.iterations());
}

// ---------- BDCSVD ----------

template <typename Scalar, int Options>
static void BM_BDCSVD(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  Mat<Scalar> A = Mat<Scalar>::Random(rows, cols);
  BDCSVD<Mat<Scalar>, Options> svd(rows, cols);
  for (auto _ : state) {
    do_compute(svd, A);
    benchmark::DoNotOptimize(svd.singularValues().data());
  }
  state.SetItemsProcessed(state.iterations());
}

template <typename Scalar, int Options>
static void BM_BDCSVDBidiagonal(benchmark::State& state) {
  const Index size = state.range(0);
  Matrix<Scalar, Dynamic, 1> diagonal = Matrix<Scalar, Dynamic, 1>::Random(size);
  Matrix<Scalar, Dynamic, 1> superdiagonal = Matrix<Scalar, Dynamic, 1>::Random(size - 1);
  BDCSVD<Mat<Scalar>, Options> svd(size, size);
  for (auto _ : state) {
    svd.compute(diagonal, superdiagonal);
    benchmark::DoNotOptimize(svd.singularValues().data());
  }
  state.SetItemsProcessed(state.iterations());
}

// ---------- Size configurations ----------

// ---------- Register benchmarks ----------

// clang-format off
// JacobiSVD sizes: square + tall-skinny (expensive for large n).
#define JACOBI_SIZES \
    ->Args({4, 4})->Args({8, 8})->Args({16, 16})->Args({32, 32})->Args({64, 64}) \
    ->Args({128, 128})->Args({256, 256})->Args({512, 512}) \
    ->Args({100, 4})->Args({1000, 4})->Args({1000, 10})

// BDCSVD sizes: square + tall-skinny (triggers R-bidiagonalization when aspect ratio > 4).
#define BDC_SIZES \
    ->Args({4, 4})->Args({8, 8})->Args({16, 16})->Args({32, 32})->Args({64, 64}) \
    ->Args({128, 128})->Args({256, 256})->Args({512, 512})->Args({1024, 1024}) \
    ->Args({100, 4})->Args({1000, 4})->Args({1000, 10})->Args({1000, 100}) \
    ->Args({10000, 10})->Args({10000, 100})

// Direct bidiagonal input isolates the divide-and-conquer phase.
#define BDC_BIDIAG_SIZES ->Arg(16)->Arg(32)->Arg(64)->Arg(128)->Arg(256)->Arg(512)->Arg(1024)

// Complex JacobiSVD above the shapes bench_jacobisvd_rotations covers.
#define JACOBI_COMPLEX_SIZES ->Args({128, 128})->Args({256, 256})->Args({512, 512})

// JacobiSVD — float
BENCHMARK(BM_JacobiSVD<float, ComputeThinU | ComputeThinV>) JACOBI_SIZES ->Name("JacobiSVD_float_ThinUV");
BENCHMARK(BM_JacobiSVD<float, 0>) JACOBI_SIZES ->Name("JacobiSVD_float_ValuesOnly");

// JacobiSVD — double
BENCHMARK(BM_JacobiSVD<double, ComputeThinU | ComputeThinV>) JACOBI_SIZES ->Name("JacobiSVD_double_ThinUV");
BENCHMARK(BM_JacobiSVD<double, 0>) JACOBI_SIZES ->Name("JacobiSVD_double_ValuesOnly");

// JacobiSVD — complex
BENCHMARK(BM_JacobiSVD<std::complex<float>, ComputeThinU | ComputeThinV>) JACOBI_COMPLEX_SIZES ->Name("JacobiSVD_cfloat_ThinUV");
BENCHMARK(BM_JacobiSVD<std::complex<float>, 0>) JACOBI_COMPLEX_SIZES ->Name("JacobiSVD_cfloat_ValuesOnly");
BENCHMARK(BM_JacobiSVD<std::complex<double>, ComputeThinU | ComputeThinV>) JACOBI_COMPLEX_SIZES ->Name("JacobiSVD_cdouble_ThinUV");
BENCHMARK(BM_JacobiSVD<std::complex<double>, 0>) JACOBI_COMPLEX_SIZES ->Name("JacobiSVD_cdouble_ValuesOnly");

// BDCSVD — float
BENCHMARK(BM_BDCSVD<float, ComputeThinU | ComputeThinV>) BDC_SIZES ->Name("BDCSVD_float_ThinUV");
BENCHMARK(BM_BDCSVD<float, 0>) BDC_SIZES ->Name("BDCSVD_float_ValuesOnly");

// BDCSVD — double
BENCHMARK(BM_BDCSVD<double, ComputeThinU | ComputeThinV>) BDC_SIZES ->Name("BDCSVD_double_ThinUV");
BENCHMARK(BM_BDCSVD<double, 0>) BDC_SIZES ->Name("BDCSVD_double_ValuesOnly");

BENCHMARK(BM_BDCSVDBidiagonal<double, ComputeThinU | ComputeThinV>)
    BDC_BIDIAG_SIZES ->Name("BDCSVD_Bidiagonal_double_ThinUV");
BENCHMARK(BM_BDCSVDBidiagonal<double, 0>) BDC_BIDIAG_SIZES ->Name("BDCSVD_Bidiagonal_double_ValuesOnly");

#undef JACOBI_SIZES
#undef BDC_SIZES
#undef BDC_BIDIAG_SIZES
#undef JACOBI_COMPLEX_SIZES
// clang-format on

// JacobiSVD — QR preconditioner comparison (double, 64x64, ThinUV)
BENCHMARK(BM_JacobiSVD<double, ComputeThinU | ComputeThinV | ColPivHouseholderQRPreconditioner>)
    ->Args({64, 64})
    ->Args({1000, 10})
    ->Name("JacobiSVD_double_ColPivQR");
BENCHMARK(BM_JacobiSVD<double, ComputeThinU | ComputeThinV | HouseholderQRPreconditioner>)
    ->Args({64, 64})
    ->Args({1000, 10})
    ->Name("JacobiSVD_double_HouseholderQR");
BENCHMARK(BM_JacobiSVD<double, ComputeFullU | ComputeFullV | FullPivHouseholderQRPreconditioner>)
    ->Args({64, 64})
    ->Name("JacobiSVD_double_FullPivQR");
