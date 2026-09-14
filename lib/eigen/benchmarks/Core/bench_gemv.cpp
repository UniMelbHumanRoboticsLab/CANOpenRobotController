// Benchmark for dense general matrix-vector multiplication (GEMV).
//
// Tests performance of y += op(A) * x for various matrix sizes, aspect ratios,
// scalar types, and operation variants (transpose, conjugate, adjoint).
//
// The Eigen GEMV kernel (Eigen/src/Core/products/GeneralMatrixVector.h) has
// two main specializations:
//   - ColMajor kernel: used for y += A * x with column-major A.
//     Processes vertical panels, vectorizes along rows.
//   - RowMajor kernel: used for y += A^T * x with column-major A.
//     Processes groups of rows, vectorizes the dot product along columns.
//
// For complex scalars, conjugation flags (ConjugateLhs, ConjugateRhs) select
// additional code paths within each kernel via conj_helper.
//
// Operation mapping (for column-major stored A):
//   Gemv       y += A * x           -> ColMajor kernel, no conjugation
//   GemvTrans  y += A^T * x         -> RowMajor kernel, no conjugation
//   GemvConj   y += conj(A) * x     -> ColMajor kernel, ConjugateLhs=true
//   GemvAdj    y += A^H * x         -> RowMajor kernel, ConjugateLhs=true
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>

using namespace Eigen;

static void BM_GemvBfloat16Strided(benchmark::State& state) {
  using Mat = Matrix<bfloat16, Dynamic, Dynamic, RowMajor>;
  using Vec = Matrix<bfloat16, Dynamic, 1>;
  const Index rows = state.range(0), cols = state.range(1), stride = state.range(2);
  const Mat A = Mat::Ones(rows, cols);
  const Vec x = Vec::Ones(cols);
  Vec storage((rows - 1) * stride + 1);
  Map<Vec, 0, InnerStride<Dynamic>> y(storage.data(), rows, InnerStride<Dynamic>(stride));
  for (auto _ : state) {
    y.noalias() = A * x;
    benchmark::DoNotOptimize(y.data());
    benchmark::ClobberMemory();
  }
  if (!(y.array() == bfloat16(cols)).all()) state.SkipWithError("Incorrect strided GEMV result");
}
BENCHMARK(BM_GemvBfloat16Strided)
    ->ArgNames({"rows", "cols", "stride"})
    ->ArgsProduct({{4, 5, 31, 32, 33, 128}, {65, 128}, {1, 2, 17}});

// ---------- Benchmark helpers ----------

// GEMV flop count: 2*m*n for real, 8*m*n for complex.
template <typename Scalar>
double gemvFlops(Index m, Index n) {
  return (NumTraits<Scalar>::IsComplex ? 8.0 : 2.0) * m * n;
}

// ---------- y += A * x  (ColMajor GEMV kernel, no conjugation) ----------

template <typename Scalar>
static void BM_Gemv(benchmark::State& state) {
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  const Index m = state.range(0);
  const Index n = state.range(1);
  Mat A = Mat::Random(m, n);
  Vec x = Vec::Random(n);
  Vec y = Vec::Random(m);
  for (auto _ : state) {
    y.noalias() += A * x;
    benchmark::DoNotOptimize(y.data());
    benchmark::ClobberMemory();
  }
  state.counters["GFLOPS"] = benchmark::Counter(gemvFlops<Scalar>(m, n), benchmark::Counter::kIsIterationInvariantRate,
                                                benchmark::Counter::kIs1000);
}

// ---------- y += A^T * x  (RowMajor GEMV kernel, no conjugation) ----------

template <typename Scalar>
static void BM_GemvTrans(benchmark::State& state) {
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  const Index m = state.range(0);
  const Index n = state.range(1);
  Mat A = Mat::Random(m, n);
  Vec x = Vec::Random(m);
  Vec y = Vec::Random(n);
  for (auto _ : state) {
    y.noalias() += A.transpose() * x;
    benchmark::DoNotOptimize(y.data());
    benchmark::ClobberMemory();
  }
  state.counters["GFLOPS"] = benchmark::Counter(gemvFlops<Scalar>(m, n), benchmark::Counter::kIsIterationInvariantRate,
                                                benchmark::Counter::kIs1000);
}

// ---------- y += conj(A) * x  (ColMajor kernel, ConjugateLhs=true) ----------

template <typename Scalar>
static void BM_GemvConj(benchmark::State& state) {
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  const Index m = state.range(0);
  const Index n = state.range(1);
  Mat A = Mat::Random(m, n);
  Vec x = Vec::Random(n);
  Vec y = Vec::Random(m);
  for (auto _ : state) {
    y.noalias() += A.conjugate() * x;
    benchmark::DoNotOptimize(y.data());
    benchmark::ClobberMemory();
  }
  state.counters["GFLOPS"] = benchmark::Counter(gemvFlops<Scalar>(m, n), benchmark::Counter::kIsIterationInvariantRate,
                                                benchmark::Counter::kIs1000);
}

// ---------- y += A^H * x  (RowMajor kernel, ConjugateLhs=true) ----------

template <typename Scalar>
static void BM_GemvAdj(benchmark::State& state) {
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  const Index m = state.range(0);
  const Index n = state.range(1);
  Mat A = Mat::Random(m, n);
  Vec x = Vec::Random(m);
  Vec y = Vec::Random(n);
  for (auto _ : state) {
    y.noalias() += A.adjoint() * x;
    benchmark::DoNotOptimize(y.data());
    benchmark::ClobberMemory();
  }
  state.counters["GFLOPS"] = benchmark::Counter(gemvFlops<Scalar>(m, n), benchmark::Counter::kIsIterationInvariantRate,
                                                benchmark::Counter::kIs1000);
}

// ---------- Size configurations ----------
// All sizes refer to the stored matrix A (m rows, n cols).

// ---------- Register benchmarks ----------

// clang-format off
// Square matrices; tall-thin (m >> n); short-wide (m << n).
// The 4096..32768-row x 1..3-col cases straddle the run_small_cols
// "stride*sizeof > L1" threshold where the 8-row inner unroll flips off.
#define GEMV_SIZES \
    ->Args({8, 8})->Args({32, 32})->Args({128, 128})->Args({512, 512})->Args({1024, 1024}) \
    ->Args({256, 1})->Args({1024, 1})->Args({256, 16})->Args({1024, 16}) \
    ->Args({1, 256})->Args({1, 1024})->Args({16, 256})->Args({16, 1024}) \
    ->Args({4096, 1})->Args({8192, 1})->Args({16384, 1})->Args({32768, 1}) \
    ->Args({4096, 2})->Args({8192, 2})->Args({16384, 2})

// Real types: Gemv and GemvTrans exercise the two kernel specializations.
// Conjugation is a no-op for real scalars.
BENCHMARK(BM_Gemv<float>) GEMV_SIZES ->Name("Gemv_float");
BENCHMARK(BM_Gemv<double>) GEMV_SIZES ->Name("Gemv_double");
BENCHMARK(BM_GemvTrans<float>) GEMV_SIZES ->Name("GemvTrans_float");
BENCHMARK(BM_GemvTrans<double>) GEMV_SIZES ->Name("GemvTrans_double");

// Complex types: all four variants exercise distinct kernel code paths.
// Only cfloat is benchmarked since cdouble exercises the same paths but slower.
BENCHMARK(BM_Gemv<std::complex<float>>) GEMV_SIZES ->Name("Gemv_cfloat");
BENCHMARK(BM_GemvTrans<std::complex<float>>) GEMV_SIZES ->Name("GemvTrans_cfloat");
BENCHMARK(BM_GemvConj<std::complex<float>>) GEMV_SIZES ->Name("GemvConj_cfloat");
BENCHMARK(BM_GemvAdj<std::complex<float>>) GEMV_SIZES ->Name("GemvAdj_cfloat");

#undef GEMV_SIZES
// clang-format on
