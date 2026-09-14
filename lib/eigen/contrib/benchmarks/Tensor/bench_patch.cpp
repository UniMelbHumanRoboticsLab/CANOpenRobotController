// Benchmarks for Eigen TensorPatch and TensorVolumePatch.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <contrib/Eigen/Tensor>

using namespace Eigen;

typedef float Scalar;

// Validates `result` against the expression's scalar coeff() path, which does
// not depend on block evaluation.
template <typename ResultTensor, typename Expr>
static bool ValidateAgainstCoeff(benchmark::State& state, const ResultTensor& result, const Expr& expr) {
  TensorEvaluator<const Expr, DefaultDevice> eval(expr, DefaultDevice());
  eval.evalSubExprsIfNeeded(nullptr);
  bool ok = true;
  for (Index i = 0; i < result.size(); ++i) {
    if (result.coeff(i) != eval.coeff(i)) {
      state.SkipWithError("validation failed");
      ok = false;
      break;
    }
  }
  eval.cleanup();
  return ok;
}

// --- Generic sliding-window patches on a 2D tensor ---
static void BM_Patch2D(benchmark::State& state) {
  const Index N = state.range(0);
  const Index K = state.range(1);

  Tensor<Scalar, 2> input(N, N);
  input.setRandom();
  Eigen::array<Index, 2> patch_dims = {K, K};
  const auto patches = input.extract_patches(patch_dims);

  Tensor<Scalar, 3> result;
  for (auto _ : state) {
    result = patches;
    benchmark::DoNotOptimize(result.data());
    benchmark::ClobberMemory();
  }
  if (!ValidateAgainstCoeff(state, result, patches)) return;
  state.SetBytesProcessed(state.iterations() * static_cast<int64_t>(result.size()) * sizeof(Scalar));
}

// --- Generic sliding-window patches on a 3D tensor ---
static void BM_Patch3D(benchmark::State& state) {
  const Index N = state.range(0);
  const Index K = state.range(1);

  Tensor<Scalar, 3> input(N, N, N);
  input.setRandom();
  Eigen::array<Index, 3> patch_dims = {K, K, K};
  const auto patches = input.extract_patches(patch_dims);

  Tensor<Scalar, 4> result;
  for (auto _ : state) {
    result = patches;
    benchmark::DoNotOptimize(result.data());
    benchmark::ClobberMemory();
  }
  if (!ValidateAgainstCoeff(state, result, patches)) return;
  state.SetBytesProcessed(state.iterations() * static_cast<int64_t>(result.size()) * sizeof(Scalar));
}

// --- Volume patches, no padding ---
static void BM_VolumePatch_Valid(benchmark::State& state) {
  const Index C = state.range(0);
  const Index N = state.range(1);
  const Index K = state.range(2);

  Tensor<Scalar, 5> input(C, N, N, N, 1);
  input.setRandom();
  const auto patches = input.extract_volume_patches(K, K, K, 1, 1, 1, PADDING_VALID);

  Tensor<Scalar, 6> result;
  for (auto _ : state) {
    result = patches;
    benchmark::DoNotOptimize(result.data());
    benchmark::ClobberMemory();
  }
  if (!ValidateAgainstCoeff(state, result, patches)) return;
  state.SetBytesProcessed(state.iterations() * static_cast<int64_t>(result.size()) * sizeof(Scalar));
}

// --- Volume patches, strided with same padding (blocks contain padding) ---
static void BM_VolumePatch_Same(benchmark::State& state) {
  const Index C = state.range(0);
  const Index N = state.range(1);
  const Index K = state.range(2);

  Tensor<Scalar, 5> input(C, N, N, N, 1);
  input.setRandom();
  const auto patches = input.extract_volume_patches(K, K, K, 2, 2, 2, PADDING_SAME);

  Tensor<Scalar, 6> result;
  for (auto _ : state) {
    result = patches;
    benchmark::DoNotOptimize(result.data());
    benchmark::ClobberMemory();
  }
  if (!ValidateAgainstCoeff(state, result, patches)) return;
  state.SetBytesProcessed(state.iterations() * static_cast<int64_t>(result.size()) * sizeof(Scalar));
}

// clang-format off
#define PATCH2D_SIZES ->ArgsProduct({{128, 256}, {4, 8}})
#define PATCH3D_SIZES ->ArgsProduct({{24, 32}, {2, 4}})
#define VOLUME_SIZES ->ArgsProduct({{4, 16}, {32}, {3}})
// clang-format on

BENCHMARK(BM_Patch2D) PATCH2D_SIZES;
BENCHMARK(BM_Patch3D) PATCH3D_SIZES;
BENCHMARK(BM_VolumePatch_Valid) VOLUME_SIZES;
BENCHMARK(BM_VolumePatch_Same) VOLUME_SIZES;
