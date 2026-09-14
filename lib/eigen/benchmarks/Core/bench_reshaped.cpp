// Benchmarks for reshaped() views of dense matrices.
//
// Covers the direct-access paths where the Reshaped evaluator serves packets:
// flatten reads, accumulation from a flattened view, reshaped destinations,
// and the AutoOrder flatten of a row-major matrix; plus the expression-sourced
// (non-direct) reshapes that forward the nested evaluator's linear accesses.
// The Map variants perform the same memory operation through Map and serve as
// an upper-bound reference. Each Reshaped benchmark checks its result against
// the Map reference after the timed loop and reports an error on mismatch.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>

using namespace Eigen;

// Flatten a matrix into a vector: dst = src.reshaped().
template <typename Scalar>
static void BM_ReshapedFlattenRead(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Mat src = Mat::Random(n, n);
  Vec dst(n * n);
  for (auto _ : state) {
    dst = src.reshaped();
    benchmark::DoNotOptimize(dst.data());
  }
  if (dst != Map<const Vec>(src.data(), src.size())) state.SkipWithError("reshaped flatten differs from Map");
  state.SetBytesProcessed(state.iterations() * n * n * sizeof(Scalar));
}

// Same flatten through Map: upper-bound reference for BM_ReshapedFlattenRead.
template <typename Scalar>
static void BM_MapFlattenRead(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Mat src = Mat::Random(n, n);
  Vec dst(n * n);
  for (auto _ : state) {
    dst = Map<const Vec>(src.data(), src.size());
    benchmark::DoNotOptimize(dst.data());
  }
  state.SetBytesProcessed(state.iterations() * n * n * sizeof(Scalar));
}

// Accumulate a flattened view: dst += alpha * src.reshaped().
template <typename Scalar>
static void BM_ReshapedFlattenAccumulate(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Mat src = Mat::Random(n, n);
  Vec dst = Vec::Random(n * n);
  const Scalar alpha(2);
  for (auto _ : state) {
    dst += alpha * src.reshaped();
    benchmark::DoNotOptimize(dst.data());
  }
  // The timed dst depends on the iteration count, so check one application from a zero start
  // instead; alpha is a power of two, so both paths are exact.
  Vec chk = Vec::Zero(n * n);
  Vec ref = Vec::Zero(n * n);
  chk += alpha * src.reshaped();
  ref += alpha * Map<const Vec>(src.data(), src.size());
  if (chk != ref) state.SkipWithError("reshaped accumulate differs from Map");
  state.SetBytesProcessed(state.iterations() * n * n * sizeof(Scalar));
}

// Same accumulation through Map: upper-bound reference for BM_ReshapedFlattenAccumulate.
template <typename Scalar>
static void BM_MapFlattenAccumulate(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Mat src = Mat::Random(n, n);
  Vec dst = Vec::Random(n * n);
  const Scalar alpha(2);
  for (auto _ : state) {
    dst += alpha * Map<const Vec>(src.data(), src.size());
    benchmark::DoNotOptimize(dst.data());
  }
  state.SetBytesProcessed(state.iterations() * n * n * sizeof(Scalar));
}

// Write through a reshaped destination: dst.reshaped() = src.
template <typename Scalar>
static void BM_ReshapedWrite(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Mat dst(n, n);
  Vec src = Vec::Random(n * n);
  for (auto _ : state) {
    dst.reshaped() = src;
    benchmark::DoNotOptimize(dst.data());
  }
  if (Map<const Vec>(dst.data(), dst.size()) != src) state.SkipWithError("reshaped write differs from Map");
  state.SetBytesProcessed(state.iterations() * n * n * sizeof(Scalar));
}

// AutoOrder flatten of a row-major matrix (vector-shaped reshape with mismatched
// canonical storage order).
template <typename Scalar>
static void BM_ReshapedRowMajorFlatten(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic, RowMajor>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Mat src = Mat::Random(n, n);
  Vec dst(n * n);
  for (auto _ : state) {
    dst = src.template reshaped<AutoOrder>();
    benchmark::DoNotOptimize(dst.data());
  }
  // AutoOrder flattens in storage order, so dst is the row-major buffer verbatim.
  if (dst != Map<const Vec>(src.data(), src.size())) state.SkipWithError("reshaped flatten differs from Map");
  state.SetBytesProcessed(state.iterations() * n * n * sizeof(Scalar));
}

// Sum-reduce a flattened view. FP reductions are not reassociable by the compiler without
// -ffast-math, so unlike the plain copies above this cannot be rescued by auto-vectorization
// and isolates Eigen's packet path.
template <typename Scalar>
static void BM_ReshapedSum(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Mat src = Mat::Random(n, n);
  Scalar acc(0);
  for (auto _ : state) {
    acc += src.reshaped().sum();
    benchmark::DoNotOptimize(acc);
  }
  // The two paths may sum in different orders (packet vs. scalar redux), so allow rounding
  // noise on the order of n*n additions of values in [-1, 1]. This catches garbage results,
  // not last-ulp deviations.
  const Scalar sum = src.reshaped().sum();
  const Scalar ref = Map<const Vec>(src.data(), src.size()).sum();
  if (numext::abs(sum - ref) > Scalar(16 * n * n) * NumTraits<Scalar>::epsilon()) {
    state.SkipWithError("reshaped sum differs from Map");
  }
  state.SetBytesProcessed(state.iterations() * n * n * sizeof(Scalar));
}

// Same reduction through Map: upper-bound reference for BM_ReshapedSum.
template <typename Scalar>
static void BM_MapSum(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Mat src = Mat::Random(n, n);
  Scalar acc(0);
  for (auto _ : state) {
    acc += Map<const Vec>(src.data(), src.size()).sum();
    benchmark::DoNotOptimize(acc);
  }
  state.SetBytesProcessed(state.iterations() * n * n * sizeof(Scalar));
}

// Elementwise exp of a flattened view. Scalar fallback calls libm per element; the packet
// path uses Eigen's vectorized exp, so this also cannot be rescued by auto-vectorization.
template <typename Scalar>
static void BM_ReshapedCwiseExp(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Mat src = Mat::Random(n, n);
  Vec dst(n * n);
  for (auto _ : state) {
    dst = src.reshaped().array().exp();
    benchmark::DoNotOptimize(dst.data());
  }
  // Scalar libm exp and packet exp may differ by a few ulps, so compare approximately.
  const Vec ref = Map<const Vec>(src.data(), src.size()).array().exp();
  if (!dst.isApprox(ref)) state.SkipWithError("reshaped exp differs from Map");
  state.SetBytesProcessed(state.iterations() * n * n * sizeof(Scalar));
}

// Same elementwise exp through Map: upper-bound reference for BM_ReshapedCwiseExp.
template <typename Scalar>
static void BM_MapCwiseExp(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Mat src = Mat::Random(n, n);
  Vec dst(n * n);
  for (auto _ : state) {
    dst = Map<const Vec>(src.data(), src.size()).array().exp();
    benchmark::DoNotOptimize(dst.data());
  }
  state.SetBytesProcessed(state.iterations() * n * n * sizeof(Scalar));
}

// Flatten an expression source: dst = (a + b).reshaped(). The reshape has no direct access
// and forwards the nested evaluator's linear packets.
template <typename Scalar>
static void BM_ReshapedExprFlattenRead(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Mat a = Mat::Random(n, n), b = Mat::Random(n, n);
  Vec dst(n * n);
  for (auto _ : state) {
    dst = (a + b).reshaped();
    benchmark::DoNotOptimize(dst.data());
  }
  // Elementwise additions match exactly between the packet and scalar paths.
  if (dst != Map<const Vec>(a.data(), a.size()) + Map<const Vec>(b.data(), b.size()))
    state.SkipWithError("reshaped expression flatten differs from Map");
  state.SetBytesProcessed(state.iterations() * n * n * sizeof(Scalar));
}

// Same operation through flat Maps: upper-bound reference for BM_ReshapedExprFlattenRead.
template <typename Scalar>
static void BM_MapExprFlattenRead(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Mat a = Mat::Random(n, n), b = Mat::Random(n, n);
  Vec dst(n * n);
  for (auto _ : state) {
    dst = Map<const Vec>(a.data(), a.size()) + Map<const Vec>(b.data(), b.size());
    benchmark::DoNotOptimize(dst.data());
  }
  state.SetBytesProcessed(state.iterations() * n * n * sizeof(Scalar));
}

// Sum-reduce a flattened expression: like BM_ReshapedSum this isolates Eigen's packet path,
// but through the non-direct evaluator, which previously also paid a div/mod index remap per
// coefficient.
template <typename Scalar>
static void BM_ReshapedExprSum(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat a = Mat::Random(n, n), b = Mat::Random(n, n);
  Scalar acc(0);
  for (auto _ : state) {
    acc += (a + b).reshaped().sum();
    benchmark::DoNotOptimize(acc);
  }
  // Like BM_ReshapedSum, but the summands a+b lie in [-2, 2], so double the tolerance.
  const Scalar sum = (a + b).reshaped().sum();
  const Scalar ref = (a + b).sum();
  if (numext::abs(sum - ref) > Scalar(32 * n * n) * NumTraits<Scalar>::epsilon()) {
    state.SkipWithError("reshaped expression sum differs from reference");
  }
  state.SetBytesProcessed(state.iterations() * n * n * sizeof(Scalar));
}

// Same reduction without the reshape: upper-bound reference for BM_ReshapedExprSum.
template <typename Scalar>
static void BM_ExprSum(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat a = Mat::Random(n, n), b = Mat::Random(n, n);
  Scalar acc(0);
  for (auto _ : state) {
    acc += (a + b).sum();
    benchmark::DoNotOptimize(acc);
  }
  state.SetBytesProcessed(state.iterations() * n * n * sizeof(Scalar));
}

// (matrix dimension n; the reshaped view has n*n elements)
// clang-format off
#define RESHAPED_SIZES ->RangeMultiplier(4)->Range(16, 1024)
BENCHMARK(BM_ReshapedFlattenRead<float>) RESHAPED_SIZES ->Name("ReshapedFlattenRead_float");
BENCHMARK(BM_ReshapedFlattenRead<double>) RESHAPED_SIZES ->Name("ReshapedFlattenRead_double");
BENCHMARK(BM_MapFlattenRead<float>) RESHAPED_SIZES ->Name("MapFlattenRead_float");
BENCHMARK(BM_MapFlattenRead<double>) RESHAPED_SIZES ->Name("MapFlattenRead_double");
BENCHMARK(BM_ReshapedFlattenAccumulate<float>) RESHAPED_SIZES ->Name("ReshapedFlattenAccumulate_float");
BENCHMARK(BM_ReshapedFlattenAccumulate<double>) RESHAPED_SIZES ->Name("ReshapedFlattenAccumulate_double");
BENCHMARK(BM_MapFlattenAccumulate<float>) RESHAPED_SIZES ->Name("MapFlattenAccumulate_float");
BENCHMARK(BM_MapFlattenAccumulate<double>) RESHAPED_SIZES ->Name("MapFlattenAccumulate_double");
BENCHMARK(BM_ReshapedWrite<float>) RESHAPED_SIZES ->Name("ReshapedWrite_float");
BENCHMARK(BM_ReshapedWrite<double>) RESHAPED_SIZES ->Name("ReshapedWrite_double");
BENCHMARK(BM_ReshapedRowMajorFlatten<float>) RESHAPED_SIZES ->Name("ReshapedRowMajorFlatten_float");
BENCHMARK(BM_ReshapedRowMajorFlatten<double>) RESHAPED_SIZES ->Name("ReshapedRowMajorFlatten_double");
BENCHMARK(BM_ReshapedSum<float>) RESHAPED_SIZES ->Name("ReshapedSum_float");
BENCHMARK(BM_ReshapedSum<double>) RESHAPED_SIZES ->Name("ReshapedSum_double");
BENCHMARK(BM_MapSum<float>) RESHAPED_SIZES ->Name("MapSum_float");
BENCHMARK(BM_MapSum<double>) RESHAPED_SIZES ->Name("MapSum_double");
BENCHMARK(BM_ReshapedExprFlattenRead<float>) RESHAPED_SIZES ->Name("ReshapedExprFlattenRead_float");
BENCHMARK(BM_ReshapedExprFlattenRead<double>) RESHAPED_SIZES ->Name("ReshapedExprFlattenRead_double");
BENCHMARK(BM_MapExprFlattenRead<float>) RESHAPED_SIZES ->Name("MapExprFlattenRead_float");
BENCHMARK(BM_MapExprFlattenRead<double>) RESHAPED_SIZES ->Name("MapExprFlattenRead_double");
BENCHMARK(BM_ReshapedExprSum<float>) RESHAPED_SIZES ->Name("ReshapedExprSum_float");
BENCHMARK(BM_ReshapedExprSum<double>) RESHAPED_SIZES ->Name("ReshapedExprSum_double");
BENCHMARK(BM_ExprSum<float>) RESHAPED_SIZES ->Name("ExprSum_float");
BENCHMARK(BM_ExprSum<double>) RESHAPED_SIZES ->Name("ExprSum_double");
BENCHMARK(BM_ReshapedCwiseExp<float>) RESHAPED_SIZES ->Name("ReshapedCwiseExp_float");
BENCHMARK(BM_ReshapedCwiseExp<double>) RESHAPED_SIZES ->Name("ReshapedCwiseExp_double");
BENCHMARK(BM_MapCwiseExp<float>) RESHAPED_SIZES ->Name("MapCwiseExp_float");
BENCHMARK(BM_MapCwiseExp<double>) RESHAPED_SIZES ->Name("MapCwiseExp_double");
#undef RESHAPED_SIZES
// clang-format on
