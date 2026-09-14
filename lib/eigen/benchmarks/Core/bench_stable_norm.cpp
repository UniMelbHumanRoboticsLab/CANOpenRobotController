// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

// The large and small inputs have finite, nonzero mathematical norms, but their squared norms
// overflow and underflow, respectively, when accumulated without scaling.

#include <benchmark/benchmark.h>
#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>
#include <vector>

using namespace Eigen;

namespace {

enum InputScale { kOrdinary = 0, kLarge = 1, kSmall = 2 };

const char* inputScaleName(InputScale scale) {
  switch (scale) {
    case kOrdinary:
      return "ordinary";
    case kLarge:
      return "large_finite_norm";
    case kSmall:
      return "small_nonzero_norm";
  }
  return "unknown";
}

template <typename Scalar>
struct coefficient_maker {
  typedef typename NumTraits<Scalar>::Real RealScalar;

  static Scalar run(RealScalar magnitude, Index index) {
    return index % 2 == 0 ? Scalar(magnitude) : Scalar(-magnitude);
  }
};

template <typename RealScalar>
struct coefficient_maker<std::complex<RealScalar>> {
  static std::complex<RealScalar> run(RealScalar magnitude, Index index) {
    const RealScalar real = index % 2 == 0 ? magnitude : -magnitude;
    const RealScalar imag = index % 4 < 2 ? magnitude : -magnitude;
    return std::complex<RealScalar>(real, imag);
  }
};

template <typename Scalar>
Matrix<Scalar, Dynamic, 1> makeInput(Index size, InputScale scale) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, 1> Vector;

  RealScalar magnitude = RealScalar(0.5);
  if (scale == kLarge) {
    // The factor 16 also leaves enough headroom for two-component complex coefficients.
    const double highest = static_cast<double>((std::numeric_limits<RealScalar>::max)());
    magnitude = RealScalar(highest / (16.0 * std::sqrt(static_cast<double>(size))));
  } else if (scale == kSmall) {
    magnitude = (std::numeric_limits<RealScalar>::min)();
  }

  Vector input(size);
  for (Index i = 0; i < size; ++i) input(i) = coefficient_maker<Scalar>::run(magnitude, i);
  return input;
}

template <typename Scalar>
void setNormCounters(benchmark::State& state, Index size) {
  state.SetBytesProcessed(state.iterations() * size * static_cast<int64_t>(sizeof(Scalar)));
}

template <typename Scalar>
Matrix<Scalar, Dynamic, 1> makeStableInput(Index size, InputScale scale) {
  using RealScalar = typename NumTraits<Scalar>::Real;
  Matrix<Scalar, Dynamic, 1> input = makeInput<Scalar>(size, scale);
  // Vary magnitudes so factor selection does not always receive an exact power of two.
  for (Index i = 0; i < size; ++i) input(i) *= RealScalar(0.5) + RealScalar((i * 37) % 101) / RealScalar(202);
  benchmark::DoNotOptimize(input.data());
  benchmark::ClobberMemory();
  return input;
}

template <typename Scalar>
long double referenceNorm(const Matrix<Scalar, Dynamic, 1>& input) {
  long double result = 0;
  for (Index i = 0; i < input.size(); ++i) {
    result = std::hypot(result, static_cast<long double>(numext::real(input(i))));
    result = std::hypot(result, static_cast<long double>(numext::imag(input(i))));
  }
  return result;
}

template <typename Scalar>
bool validNormalization(const Matrix<Scalar, Dynamic, 1>& input, const Matrix<Scalar, Dynamic, 1>& output) {
  using RealScalar = typename NumTraits<Scalar>::Real;
  const long double norm = referenceNorm(input);
  long double error = 0;
  for (Index i = 0; i < input.size(); ++i) {
    error = std::hypot(error, static_cast<long double>(numext::real(output(i))) -
                                  static_cast<long double>(numext::real(input(i))) / norm);
    error = std::hypot(error, static_cast<long double>(numext::imag(output(i))) -
                                  static_cast<long double>(numext::imag(input(i))) / norm);
  }
  return error <= 64 * static_cast<long double>(NumTraits<RealScalar>::epsilon());
}

template <typename Scalar>
void setNormalizeCounters(benchmark::State& state, Index size) {
  // Count logical traffic rather than implementation-specific passes.
  state.SetBytesProcessed(state.iterations() * size * static_cast<int64_t>(2 * sizeof(Scalar)));
}

Index normalizationBatchSize(Index size) {
  const Index kMaxBatchSize = 128;
  const Index kCoefficientBudget = 65536;
  return (std::max)(Index(1), (std::min)(kMaxBatchSize, kCoefficientBudget / size));
}

template <typename Scalar>
static void BM_Norm(benchmark::State& state) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  const Index size = state.range(0);
  const InputScale scale = static_cast<InputScale>(state.range(1));
  const Matrix<Scalar, Dynamic, 1> input = makeInput<Scalar>(size, scale);
  state.SetLabel(inputScaleName(scale));

  for (auto _ : state) {
    RealScalar result = input.norm();
    benchmark::DoNotOptimize(result);
  }
  setNormCounters<Scalar>(state, size);
}

template <typename Scalar>
static void BM_StableNorm(benchmark::State& state) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  const Index size = state.range(0);
  const InputScale scale = static_cast<InputScale>(state.range(1));
  const Matrix<Scalar, Dynamic, 1> input = makeStableInput<Scalar>(size, scale);
  state.SetLabel(inputScaleName(scale));

  const long double reference = referenceNorm(input);
  const long double error = std::abs(static_cast<long double>(input.stableNorm()) / reference - 1);
  if (!(error <= 64 * static_cast<long double>(NumTraits<RealScalar>::epsilon()))) {
    state.SkipWithError("stableNorm failed reference validation");
    return;
  }

  for (auto _ : state) {
    benchmark::DoNotOptimize(input.data());
    benchmark::ClobberMemory();
    RealScalar result = input.stableNorm();
    benchmark::DoNotOptimize(result);
  }
  setNormCounters<Scalar>(state, size);
}

template <typename Plain>
Plain makeFixedInput(Index seed) {
  typedef typename Plain::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;
  const RealScalar magnitude = RealScalar(0.25) + RealScalar(seed % 17) / RealScalar(64);
  Plain input;
  for (Index i = 0; i < input.size(); ++i) input.data()[i] = coefficient_maker<Scalar>::run(magnitude, i + seed);
  return input;
}

template <typename Plain>
static void BM_FixedMapStableNorm(benchmark::State& state) {
  typedef typename Plain::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Stride<Dynamic, Dynamic> DynamicStride;
  typedef Map<const Plain, Unaligned, DynamicStride> MapType;

  const Index input_count = 256;
  std::vector<Plain, aligned_allocator<Plain>> inputs(static_cast<std::size_t>(input_count));
  for (Index i = 0; i < input_count; ++i) inputs[static_cast<std::size_t>(i)] = makeFixedInput<Plain>(i);
  benchmark::DoNotOptimize(inputs.data());
  benchmark::ClobberMemory();

  Index index = 0;
  for (auto _ : state) {
    const Plain& storage = inputs[static_cast<std::size_t>(index)];
    const DynamicStride stride(storage.outerStride(), storage.innerStride());
    const MapType input(storage.data(), storage.rows(), storage.cols(), stride);
    RealScalar result = input.stableNorm();
    benchmark::DoNotOptimize(result);
    index = (index + 1) & (input_count - 1);
  }
  setNormCounters<Scalar>(state, Plain::SizeAtCompileTime);
}

template <typename Scalar>
static void BM_BlueNorm(benchmark::State& state) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  const Index size = state.range(0);
  const InputScale scale = static_cast<InputScale>(state.range(1));
  const Matrix<Scalar, Dynamic, 1> input = makeInput<Scalar>(size, scale);
  state.SetLabel(inputScaleName(scale));

  for (auto _ : state) {
    RealScalar result = input.blueNorm();
    benchmark::DoNotOptimize(result);
  }
  setNormCounters<Scalar>(state, size);
}

template <typename Scalar>
static void BM_HypotNorm(benchmark::State& state) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  const Index size = state.range(0);
  const InputScale scale = static_cast<InputScale>(state.range(1));
  const Matrix<Scalar, Dynamic, 1> input = makeInput<Scalar>(size, scale);
  state.SetLabel(inputScaleName(scale));

  for (auto _ : state) {
    RealScalar result = input.hypotNorm();
    benchmark::DoNotOptimize(result);
  }
  setNormCounters<Scalar>(state, size);
}

template <typename Scalar>
static void BM_Normalize(benchmark::State& state) {
  const Index size = state.range(0);
  const InputScale scale = static_cast<InputScale>(state.range(1));
  const Matrix<Scalar, Dynamic, 1> input = makeInput<Scalar>(size, scale);
  const Index batch_size = normalizationBatchSize(size);
  std::vector<Matrix<Scalar, Dynamic, 1>> results(static_cast<std::size_t>(batch_size), input);
  state.SetLabel(inputScaleName(scale));

  while (state.KeepRunningBatch(batch_size)) {
    state.PauseTiming();
    for (Index i = 0; i < batch_size; ++i) results[static_cast<std::size_t>(i)] = input;
    state.ResumeTiming();
    for (Index i = 0; i < batch_size; ++i) {
      results[static_cast<std::size_t>(i)].normalize();
      benchmark::DoNotOptimize(results[static_cast<std::size_t>(i)].data());
    }
    benchmark::ClobberMemory();
  }
  setNormalizeCounters<Scalar>(state, size);
}

template <typename Scalar>
static void BM_StableNormalize(benchmark::State& state) {
  const Index size = state.range(0);
  const InputScale scale = static_cast<InputScale>(state.range(1));
  const Matrix<Scalar, Dynamic, 1> input = makeStableInput<Scalar>(size, scale);
  const Index batch_size = normalizationBatchSize(size);
  std::vector<Matrix<Scalar, Dynamic, 1>> results(static_cast<std::size_t>(batch_size), input);
  state.SetLabel(inputScaleName(scale));

  results[0].stableNormalize();
  if (!validNormalization(input, results[0])) {
    state.SkipWithError("stableNormalize failed reference validation");
    return;
  }

  while (state.KeepRunningBatch(batch_size)) {
    state.PauseTiming();
    for (Index i = 0; i < batch_size; ++i) results[static_cast<std::size_t>(i)] = input;
    state.ResumeTiming();
    for (Index i = 0; i < batch_size; ++i) {
      results[static_cast<std::size_t>(i)].stableNormalize();
      benchmark::DoNotOptimize(results[static_cast<std::size_t>(i)].data());
    }
    benchmark::ClobberMemory();
  }
  setNormalizeCounters<Scalar>(state, size);
}

template <typename Scalar>
static void BM_StableNormalized(benchmark::State& state) {
  const Index size = state.range(0);
  const InputScale scale = static_cast<InputScale>(state.range(1));
  const Matrix<Scalar, Dynamic, 1> input = makeStableInput<Scalar>(size, scale);
  const Matrix<Scalar, Dynamic, 1> checked = input.stableNormalized();
  if (!validNormalization(input, checked)) {
    state.SkipWithError("stableNormalized failed reference validation");
    return;
  }
  state.SetLabel(inputScaleName(scale));
  for (auto _ : state) {
    benchmark::DoNotOptimize(input.data());
    benchmark::ClobberMemory();
    const Matrix<Scalar, Dynamic, 1> result = input.stableNormalized();
    benchmark::DoNotOptimize(result.data());
    benchmark::ClobberMemory();
  }
  setNormalizeCounters<Scalar>(state, size);
}

typedef Matrix<float, 1, 16> FixedRow16f;
typedef Matrix<float, 16, 1> FixedCol16f;
typedef Matrix<float, 4, 4> FixedMatrix4f;
typedef Matrix<double, 1, 8> FixedRow8d;
typedef Matrix<double, 8, 1> FixedCol8d;
typedef Matrix<double, 3, 3> FixedMatrix3d;
typedef Matrix<std::complex<float>, 1, 8> FixedRow8cf;
typedef Matrix<std::complex<float>, 8, 1> FixedCol8cf;
typedef Matrix<std::complex<float>, 2, 4> FixedMatrix4cf;
typedef Matrix<std::complex<double>, 1, 4> FixedRow4cd;
typedef Matrix<std::complex<double>, 4, 1> FixedCol4cd;
typedef Matrix<std::complex<double>, 2, 2> FixedMatrix4cd;

// clang-format off
#define NORM_ARGS ->ArgsProduct({{64, 4096, 65536}, {kOrdinary, kLarge, kSmall}})->ArgNames({"size", "scale"})
#define STABLE_ARGS ->ArgsProduct({{3, 16, 64, 256, 1024, 4096}, {kOrdinary, kLarge, kSmall}})->ArgNames({"size", "scale"})
#define FIXED_MAP_STABLE_NORM(Type, Label) BENCHMARK(BM_FixedMapStableNorm<Type>)->Name("FixedMapStableNorm_" #Label)
BENCHMARK(BM_Norm<float>) NORM_ARGS ->Name("Norm_float");
BENCHMARK(BM_Norm<double>) NORM_ARGS ->Name("Norm_double");
BENCHMARK(BM_Norm<half>) NORM_ARGS ->Name("Norm_half");
BENCHMARK(BM_Norm<bfloat16>) NORM_ARGS ->Name("Norm_bfloat16");
BENCHMARK(BM_Norm<std::complex<float>>) NORM_ARGS ->Name("Norm_cfloat");
BENCHMARK(BM_Norm<std::complex<double>>) NORM_ARGS ->Name("Norm_cdouble");
BENCHMARK(BM_StableNorm<float>) STABLE_ARGS ->Name("StableNorm_float");
BENCHMARK(BM_StableNorm<double>) STABLE_ARGS ->Name("StableNorm_double");
BENCHMARK(BM_StableNorm<half>) STABLE_ARGS ->Name("StableNorm_half");
BENCHMARK(BM_StableNorm<bfloat16>) STABLE_ARGS ->Name("StableNorm_bfloat16");
BENCHMARK(BM_StableNorm<std::complex<float>>) STABLE_ARGS ->Name("StableNorm_cfloat");
BENCHMARK(BM_StableNorm<std::complex<double>>) STABLE_ARGS ->Name("StableNorm_cdouble");
FIXED_MAP_STABLE_NORM(FixedRow16f, row16f);
FIXED_MAP_STABLE_NORM(FixedCol16f, col16f);
FIXED_MAP_STABLE_NORM(FixedMatrix4f, matrix4f);
FIXED_MAP_STABLE_NORM(FixedRow8d, row8d);
FIXED_MAP_STABLE_NORM(FixedCol8d, col8d);
FIXED_MAP_STABLE_NORM(FixedMatrix3d, matrix3d);
FIXED_MAP_STABLE_NORM(FixedRow8cf, row8cf);
FIXED_MAP_STABLE_NORM(FixedCol8cf, col8cf);
FIXED_MAP_STABLE_NORM(FixedMatrix4cf, matrix4cf);
FIXED_MAP_STABLE_NORM(FixedRow4cd, row4cd);
FIXED_MAP_STABLE_NORM(FixedCol4cd, col4cd);
FIXED_MAP_STABLE_NORM(FixedMatrix4cd, matrix4cd);
BENCHMARK(BM_BlueNorm<float>) NORM_ARGS ->Name("BlueNorm_float");
BENCHMARK(BM_BlueNorm<double>) NORM_ARGS ->Name("BlueNorm_double");
BENCHMARK(BM_BlueNorm<half>) NORM_ARGS ->Name("BlueNorm_half");
BENCHMARK(BM_BlueNorm<bfloat16>) NORM_ARGS ->Name("BlueNorm_bfloat16");
BENCHMARK(BM_BlueNorm<std::complex<float>>) NORM_ARGS ->Name("BlueNorm_cfloat");
BENCHMARK(BM_BlueNorm<std::complex<double>>) NORM_ARGS ->Name("BlueNorm_cdouble");
BENCHMARK(BM_HypotNorm<float>) NORM_ARGS ->Name("HypotNorm_float");
BENCHMARK(BM_HypotNorm<double>) NORM_ARGS ->Name("HypotNorm_double");
BENCHMARK(BM_HypotNorm<half>) NORM_ARGS ->Name("HypotNorm_half");
BENCHMARK(BM_HypotNorm<bfloat16>) NORM_ARGS ->Name("HypotNorm_bfloat16");
BENCHMARK(BM_HypotNorm<std::complex<float>>) NORM_ARGS ->Name("HypotNorm_cfloat");
BENCHMARK(BM_HypotNorm<std::complex<double>>) NORM_ARGS ->Name("HypotNorm_cdouble");
BENCHMARK(BM_Normalize<float>) NORM_ARGS ->Name("Normalize_float");
BENCHMARK(BM_Normalize<double>) NORM_ARGS ->Name("Normalize_double");
BENCHMARK(BM_Normalize<half>) NORM_ARGS ->Name("Normalize_half");
BENCHMARK(BM_Normalize<bfloat16>) NORM_ARGS ->Name("Normalize_bfloat16");
BENCHMARK(BM_Normalize<std::complex<float>>) NORM_ARGS ->Name("Normalize_cfloat");
BENCHMARK(BM_Normalize<std::complex<double>>) NORM_ARGS ->Name("Normalize_cdouble");
BENCHMARK(BM_StableNormalize<float>) STABLE_ARGS ->Name("StableNormalize_float");
BENCHMARK(BM_StableNormalize<double>) STABLE_ARGS ->Name("StableNormalize_double");
BENCHMARK(BM_StableNormalize<half>) STABLE_ARGS ->Name("StableNormalize_half");
BENCHMARK(BM_StableNormalize<bfloat16>) STABLE_ARGS ->Name("StableNormalize_bfloat16");
BENCHMARK(BM_StableNormalize<std::complex<float>>) STABLE_ARGS ->Name("StableNormalize_cfloat");
BENCHMARK(BM_StableNormalize<std::complex<double>>) STABLE_ARGS ->Name("StableNormalize_cdouble");
#undef NORM_ARGS
BENCHMARK(BM_StableNormalized<float>) STABLE_ARGS ->Name("StableNormalized_float");
BENCHMARK(BM_StableNormalized<double>) STABLE_ARGS ->Name("StableNormalized_double");
BENCHMARK(BM_StableNormalized<std::complex<float>>) STABLE_ARGS ->Name("StableNormalized_cfloat");
BENCHMARK(BM_StableNormalized<std::complex<double>>) STABLE_ARGS ->Name("StableNormalized_cdouble");
#undef STABLE_ARGS
#undef FIXED_MAP_STABLE_NORM
// clang-format on

}  // namespace
