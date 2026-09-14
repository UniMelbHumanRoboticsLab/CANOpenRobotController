// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/SVD>

using namespace Eigen;

// Benchmark the UpperBidiagonalization reduction in isolation,
// covering both the unblocked and blocked code paths, and
// sweeping the block size parameter for tuning.

template <typename Scalar>
using Mat = Matrix<Scalar, Dynamic, Dynamic>;

// ---------- Blocked (default) path ----------

template <typename Scalar>
static void BM_UpperBidiag_Blocked(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  Mat<Scalar> A = Mat<Scalar>::Random(rows, cols);
  internal::UpperBidiagonalization<Mat<Scalar>> ubd(rows, cols);
  for (auto _ : state) {
    ubd.compute(A);
    benchmark::DoNotOptimize(ubd.bidiagonal().toDenseMatrix().data());
  }
  state.SetItemsProcessed(state.iterations());
}

// ---------- Unblocked path ----------

template <typename Scalar>
static void BM_UpperBidiag_Unblocked(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  Mat<Scalar> A = Mat<Scalar>::Random(rows, cols);
  internal::UpperBidiagonalization<Mat<Scalar>> ubd(rows, cols);
  for (auto _ : state) {
    ubd.computeUnblocked(A);
    benchmark::DoNotOptimize(ubd.bidiagonal().toDenseMatrix().data());
  }
  state.SetItemsProcessed(state.iterations());
}

// ---------- Blocked with explicit block size ----------

template <typename Scalar>
static void BM_UpperBidiag_BlockSize(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  const Index blockSize = state.range(2);
  Mat<Scalar> A = Mat<Scalar>::Random(rows, cols);
  using RealScalar = typename NumTraits<Scalar>::Real;
  using BidiagType = internal::BandMatrix<RealScalar, Dynamic, Dynamic, 1, 0, RowMajor>;
  Mat<Scalar> householder(rows, cols);
  BidiagType bidiagonal(cols, cols);
  for (auto _ : state) {
    householder = A;
    internal::upperbidiagonalization_inplace_blocked(householder, bidiagonal, blockSize);
    benchmark::DoNotOptimize(bidiagonal.toDenseMatrix().data());
  }
  state.SetItemsProcessed(state.iterations());
  state.SetLabel("bs=" + std::to_string(blockSize));
}

// ---------- Size configurations ----------

// clang-format off

// Small sizes: always hit the unblocked path.
#define SMALL_SIZES \
    ->Args({4, 4})->Args({8, 8})->Args({16, 16})->Args({32, 32}) \
    ->Args({16, 4})->Args({32, 8})->Args({32, 16})

// Medium sizes: exercise the blocked path.
#define MEDIUM_SIZES \
    ->Args({64, 64})->Args({128, 128})->Args({256, 256})->Args({512, 512}) \
    ->Args({256, 64})->Args({512, 64})->Args({512, 128})

// Large sizes: dominated by the blocked GEMM updates.
#define LARGE_SIZES \
    ->Args({1024, 1024})->Args({2048, 2048})->Args({4096, 4096}) \
    ->Args({2048, 256})->Args({4096, 256})->Args({4096, 1024})

// Block size sweep: {rows, cols, blockSize}
// Square matrices from 128 to 4096, block sizes 16..96.
#define BLOCKSIZE_SWEEP \
    ->Args({128, 128, 16})->Args({128, 128, 32})->Args({128, 128, 48})->Args({128, 128, 64})->Args({128, 128, 96}) \
    ->Args({256, 256, 16})->Args({256, 256, 32})->Args({256, 256, 48})->Args({256, 256, 64})->Args({256, 256, 96}) \
    ->Args({512, 512, 16})->Args({512, 512, 32})->Args({512, 512, 48})->Args({512, 512, 64})->Args({512, 512, 96}) \
    ->Args({1024, 1024, 16})->Args({1024, 1024, 32})->Args({1024, 1024, 48})->Args({1024, 1024, 64})->Args({1024, 1024, 96}) \
    ->Args({2048, 2048, 16})->Args({2048, 2048, 32})->Args({2048, 2048, 48})->Args({2048, 2048, 64})->Args({2048, 2048, 96}) \
    ->Args({4096, 4096, 16})->Args({4096, 4096, 32})->Args({4096, 4096, 48})->Args({4096, 4096, 64})->Args({4096, 4096, 96})

// Tall-skinny block size sweep.
#define BLOCKSIZE_SWEEP_TALL \
    ->Args({1024, 128, 16})->Args({1024, 128, 32})->Args({1024, 128, 48})->Args({1024, 128, 64}) \
    ->Args({4096, 256, 16})->Args({4096, 256, 32})->Args({4096, 256, 48})->Args({4096, 256, 64}) \
    ->Args({4096, 1024, 16})->Args({4096, 1024, 32})->Args({4096, 1024, 48})->Args({4096, 1024, 64})

// ---------- Register benchmarks ----------

// Blocked — float
BENCHMARK(BM_UpperBidiag_Blocked<float>) SMALL_SIZES  ->Name("Bidiag_Blocked_float_Small");
BENCHMARK(BM_UpperBidiag_Blocked<float>) MEDIUM_SIZES ->Name("Bidiag_Blocked_float_Medium");
BENCHMARK(BM_UpperBidiag_Blocked<float>) LARGE_SIZES  ->Name("Bidiag_Blocked_float_Large");

// Blocked — double
BENCHMARK(BM_UpperBidiag_Blocked<double>) SMALL_SIZES  ->Name("Bidiag_Blocked_double_Small");
BENCHMARK(BM_UpperBidiag_Blocked<double>) MEDIUM_SIZES ->Name("Bidiag_Blocked_double_Medium");
BENCHMARK(BM_UpperBidiag_Blocked<double>) LARGE_SIZES  ->Name("Bidiag_Blocked_double_Large");

// Unblocked — float
BENCHMARK(BM_UpperBidiag_Unblocked<float>) SMALL_SIZES  ->Name("Bidiag_Unblocked_float_Small");
BENCHMARK(BM_UpperBidiag_Unblocked<float>) MEDIUM_SIZES ->Name("Bidiag_Unblocked_float_Medium");

// Unblocked — double
BENCHMARK(BM_UpperBidiag_Unblocked<double>) SMALL_SIZES  ->Name("Bidiag_Unblocked_double_Small");
BENCHMARK(BM_UpperBidiag_Unblocked<double>) MEDIUM_SIZES ->Name("Bidiag_Unblocked_double_Medium");

// Block size sweep — double
BENCHMARK(BM_UpperBidiag_BlockSize<double>) BLOCKSIZE_SWEEP ->Name("Bidiag_BS_double_Square");
BENCHMARK(BM_UpperBidiag_BlockSize<double>) BLOCKSIZE_SWEEP_TALL ->Name("Bidiag_BS_double_Tall");

// Block size sweep — float
BENCHMARK(BM_UpperBidiag_BlockSize<float>) BLOCKSIZE_SWEEP ->Name("Bidiag_BS_float_Square");
BENCHMARK(BM_UpperBidiag_BlockSize<float>) BLOCKSIZE_SWEEP_TALL ->Name("Bidiag_BS_float_Tall");

#undef SMALL_SIZES
#undef MEDIUM_SIZES
#undef LARGE_SIZES
#undef BLOCKSIZE_SWEEP
#undef BLOCKSIZE_SWEEP_TALL
// clang-format on
