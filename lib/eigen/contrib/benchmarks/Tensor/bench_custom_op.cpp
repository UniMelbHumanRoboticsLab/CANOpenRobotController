// Benchmarks for compositions over Eigen Tensor custom ops.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <contrib/Eigen/Tensor>

using namespace Eigen;

typedef float Scalar;

// A trivial custom op (scale by 2) with the same output dimensions as its
// input. The interesting part is not the op itself but how its materialized
// result composes with downstream expressions.
struct ScaleByTwo {
  DSizes<DenseIndex, 2> dimensions(const Tensor<Scalar, 2>& input) const { return input.dimensions(); }

  template <typename Output, typename Device>
  void eval(const Tensor<Scalar, 2>& input, Output& output, const Device& device) const {
    output.device(device) = input * Scalar(2);
  }
};

// Custom op consumed by a transpose. With block access on the custom-op
// evaluator the shuffle runs as a cache-blocked TensorBlockIO transpose over
// the custom op's raw buffer; without it, as a per-element gather.
static void BM_CustomOpShuffle(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);

  Tensor<Scalar, 2> mat(M, N);
  Tensor<Scalar, 2> result(N, M);
  mat.setRandom();

  Eigen::array<int, 2> transpose = {1, 0};

  for (auto _ : state) {
    result = mat.customOp(ScaleByTwo()).shuffle(transpose);
    benchmark::DoNotOptimize(result.data());
    benchmark::ClobberMemory();
  }

  // Validate outside the timed loop.
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < M; ++j) {
      if (result(i, j) != Scalar(2) * mat(j, i)) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  state.SetBytesProcessed(state.iterations() * M * N * sizeof(Scalar));
}

// Custom op consumed by a contiguous slice. With raw access on the custom-op
// evaluator the slice hits its memcpy fast path.
static void BM_CustomOpSlice(benchmark::State& state) {
  const int M = state.range(0);
  const int N = state.range(1);

  Tensor<Scalar, 2> mat(M, N);
  mat.setRandom();

  const Eigen::array<Index, 2> offsets = {0, N / 4};
  const Eigen::array<Index, 2> extents = {M, N / 2};

  Tensor<Scalar, 2> result;
  for (auto _ : state) {
    result = mat.customOp(ScaleByTwo()).slice(offsets, extents);
    benchmark::DoNotOptimize(result.data());
    benchmark::ClobberMemory();
  }

  for (Index i = 0; i < extents[0]; ++i) {
    for (Index j = 0; j < extents[1]; ++j) {
      if (result(i, j) != Scalar(2) * mat(i, N / 4 + j)) {
        state.SkipWithError("validation failed");
        return;
      }
    }
  }
  state.SetBytesProcessed(state.iterations() * extents[0] * extents[1] * sizeof(Scalar));
}

#define CUSTOM_OP_SIZES ->ArgsProduct({{256, 1024, 4096}, {256, 1024}})

BENCHMARK(BM_CustomOpShuffle) CUSTOM_OP_SIZES;
BENCHMARK(BM_CustomOpSlice) CUSTOM_OP_SIZES;
