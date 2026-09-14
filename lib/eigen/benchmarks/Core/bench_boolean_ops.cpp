// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>

#include "benchmark_boolean_helpers.h"

namespace Eigen {
namespace {

using namespace benchmark_detail;

struct Sum {
  static bool run(const BoolArray& input) { return input.sum(); }
  static bool reference(bool accum, bool value) { return accum || value; }
  static bool identity() { return false; }
};

struct Product {
  static bool run(const BoolArray& input) { return input.prod(); }
  static bool reference(bool accum, bool value) { return accum && value; }
  static bool identity() { return true; }
};

template <typename Operation, InputPattern Pattern>
void BM_BooleanReduction(benchmark::State& state) {
  const Index size = state.range(0);
  BoolArray input(size);
  fill_input<Pattern>(input, 0x12345678u);

  bool result = Operation::identity();
  for (auto _ : state) {
    result = Operation::run(input);
    benchmark::DoNotOptimize(result);
  }

  bool reference = Operation::identity();
  for (Index i = 0; i < size; ++i) reference = Operation::reference(reference, input[i]);
  if (result != reference) state.SkipWithError("reduction result does not match the boolean reference");
  state.SetItemsProcessed(state.iterations() * size);
}

struct LogicalAnd {
  static void run(BoolArray& result, const BoolArray& lhs, const BoolArray& rhs) { result = lhs && rhs; }
  static bool reference(bool lhs, bool rhs) { return lhs && rhs; }
};

struct LogicalOr {
  static void run(BoolArray& result, const BoolArray& lhs, const BoolArray& rhs) { result = lhs || rhs; }
  static bool reference(bool lhs, bool rhs) { return lhs || rhs; }
};

template <typename Operation, InputPattern Pattern>
void BM_BooleanBinary(benchmark::State& state) {
  const Index size = state.range(0);
  BoolArray lhs(size);
  BoolArray rhs(size);
  BoolArray result(size);
  fill_input<Pattern>(lhs, 0x12345678u);
  fill_input<Pattern>(rhs, 0x87654321u);

  for (auto _ : state) {
    Operation::run(result, lhs, rhs);
    benchmark::DoNotOptimize(result.data());
    benchmark::ClobberMemory();
  }

  for (Index i = 0; i < size; ++i) {
    if (result[i] != Operation::reference(lhs[i], rhs[i])) {
      state.SkipWithError("binary result does not match the boolean reference");
      break;
    }
  }
  state.SetItemsProcessed(state.iterations() * size);
}

#define EIGEN_BENCH_BOOLEAN_PATTERNS(BENCHMARK_FUNCTION, OPERATION)         \
  BENCHMARK_TEMPLATE(BENCHMARK_FUNCTION, OPERATION, Random)->Arg(262151);   \
  BENCHMARK_TEMPLATE(BENCHMARK_FUNCTION, OPERATION, AllFalse)->Arg(262151); \
  BENCHMARK_TEMPLATE(BENCHMARK_FUNCTION, OPERATION, AllTrue)->Arg(262151);  \
  BENCHMARK_TEMPLATE(BENCHMARK_FUNCTION, OPERATION, benchmark_detail::Sparse)->Arg(262151)

EIGEN_BENCH_BOOLEAN_PATTERNS(BM_BooleanReduction, Sum);
EIGEN_BENCH_BOOLEAN_PATTERNS(BM_BooleanReduction, Product);
EIGEN_BENCH_BOOLEAN_PATTERNS(BM_BooleanBinary, LogicalAnd);
EIGEN_BENCH_BOOLEAN_PATTERNS(BM_BooleanBinary, LogicalOr);

#undef EIGEN_BENCH_BOOLEAN_PATTERNS

}  // namespace
}  // namespace Eigen
