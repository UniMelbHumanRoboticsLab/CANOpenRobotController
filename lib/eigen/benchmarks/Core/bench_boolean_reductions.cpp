// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>

#include <Eigen/Core>

#include "benchmark_boolean_helpers.h"

namespace Eigen {
namespace {

using namespace benchmark_detail;

struct AllVisitor {
  static bool run(const BoolArray& input) { return input.all(); }
  static bool reference(const BoolArray& input) {
    for (Index i = 0; i < input.size(); ++i)
      if (!input[i]) return false;
    return true;
  }
};

struct AnyVisitor {
  static bool run(const BoolArray& input) { return input.any(); }
  static bool reference(const BoolArray& input) {
    for (Index i = 0; i < input.size(); ++i)
      if (input[i]) return true;
    return false;
  }
};

template <typename Operation, InputPattern Pattern>
void BM_BooleanVisitor(benchmark::State& state) {
  const Index size = state.range(0);
  BoolArray input(size);
  fill_input<Pattern>(input, 0x12345678u);

  bool result = false;
  for (auto _ : state) {
    benchmark::ClobberMemory();
    result = Operation::run(input);
    benchmark::DoNotOptimize(result);
  }

  if (result != Operation::reference(input)) state.SkipWithError("visitor result does not match the scalar reference");
}

#define EIGEN_BENCH_BOOLEAN_VISITOR(OPERATION)                               \
  BENCHMARK_TEMPLATE(BM_BooleanVisitor, OPERATION, Random)->Arg(262151);     \
  BENCHMARK_TEMPLATE(BM_BooleanVisitor, OPERATION, AllFalse)->Arg(262151);   \
  BENCHMARK_TEMPLATE(BM_BooleanVisitor, OPERATION, AllTrue)->Arg(262151);    \
  BENCHMARK_TEMPLATE(BM_BooleanVisitor, OPERATION, FirstTrue)->Arg(262151);  \
  BENCHMARK_TEMPLATE(BM_BooleanVisitor, OPERATION, LastTrue)->Arg(262151);   \
  BENCHMARK_TEMPLATE(BM_BooleanVisitor, OPERATION, FirstFalse)->Arg(262151); \
  BENCHMARK_TEMPLATE(BM_BooleanVisitor, OPERATION, LastFalse)->Arg(262151)

EIGEN_BENCH_BOOLEAN_VISITOR(AllVisitor);
EIGEN_BENCH_BOOLEAN_VISITOR(AnyVisitor);

#undef EIGEN_BENCH_BOOLEAN_VISITOR

template <typename Packet>
struct PacketAny {
  static bool run(const Packet& packet) { return internal::predux_any(packet); }
  static bool reference(bool any, bool) { return any; }
};

template <typename Packet>
struct PacketOr {
  static bool run(const Packet& packet) { return internal::predux(packet); }
  static bool reference(bool any, bool) { return any; }
};

template <typename Packet>
struct PacketAnd {
  static bool run(const Packet& packet) { return internal::predux_mul(packet); }
  static bool reference(bool, bool all) { return all; }
};

template <typename Packet, template <typename> class Operation, InputPattern Pattern>
void BM_PacketReduction(benchmark::State& state) {
  using Scalar = typename internal::unpacket_traits<Packet>::type;
  constexpr int PacketSize = internal::unpacket_traits<Packet>::size;
  alignas(internal::unpacket_traits<Packet>::alignment) Scalar input[PacketSize];
  std::uint32_t randomState = 0x12345678u;
  for (int i = 0; i < PacketSize; ++i) {
    randomState = randomState * 1664525u + 1013904223u;
    const bool value = Pattern == AllTrue || Pattern == FirstFalse || Pattern == LastFalse ||
                       (Pattern == Random && (randomState >> 31) != 0);
    input[i] = Scalar(value);
  }
  if (Pattern == FirstTrue) input[0] = Scalar(1);
  if (Pattern == LastTrue) input[PacketSize - 1] = Scalar(1);
  if (Pattern == FirstFalse) input[0] = Scalar(0);
  if (Pattern == LastFalse) input[PacketSize - 1] = Scalar(0);

  Packet packet = internal::pload<Packet>(input);
  bool result = false;
  for (auto _ : state) {
    benchmark::DoNotOptimize(packet);
    result = Operation<Packet>::run(packet);
    benchmark::DoNotOptimize(result);
  }

  bool any = false;
  bool all = true;
  for (int i = 0; i < PacketSize; ++i) {
    any = any || input[i] != Scalar(0);
    all = all && input[i] != Scalar(0);
  }
  const bool reference = Operation<Packet>::reference(any, all);
  if (result != reference) state.SkipWithError("packet result does not match the scalar reference");
  state.SetItemsProcessed(state.iterations() * PacketSize);
}

#ifdef EIGEN_VECTORIZE_SSE
#define EIGEN_BENCH_PACKET16B(OPERATION)                                              \
  BENCHMARK_TEMPLATE(BM_PacketReduction, internal::Packet16b, OPERATION, Random);     \
  BENCHMARK_TEMPLATE(BM_PacketReduction, internal::Packet16b, OPERATION, AllFalse);   \
  BENCHMARK_TEMPLATE(BM_PacketReduction, internal::Packet16b, OPERATION, AllTrue);    \
  BENCHMARK_TEMPLATE(BM_PacketReduction, internal::Packet16b, OPERATION, FirstTrue);  \
  BENCHMARK_TEMPLATE(BM_PacketReduction, internal::Packet16b, OPERATION, LastTrue);   \
  BENCHMARK_TEMPLATE(BM_PacketReduction, internal::Packet16b, OPERATION, FirstFalse); \
  BENCHMARK_TEMPLATE(BM_PacketReduction, internal::Packet16b, OPERATION, LastFalse)

EIGEN_BENCH_PACKET16B(PacketOr);
EIGEN_BENCH_PACKET16B(PacketAnd);

#undef EIGEN_BENCH_PACKET16B
#endif

#ifdef EIGEN_VECTORIZE_AVX512
#define EIGEN_BENCH_AVX512_PACKET(PACKET)                               \
  BENCHMARK_TEMPLATE(BM_PacketReduction, PACKET, PacketAny, Random);    \
  BENCHMARK_TEMPLATE(BM_PacketReduction, PACKET, PacketAny, AllFalse);  \
  BENCHMARK_TEMPLATE(BM_PacketReduction, PACKET, PacketAny, AllTrue);   \
  BENCHMARK_TEMPLATE(BM_PacketReduction, PACKET, PacketAny, FirstTrue); \
  BENCHMARK_TEMPLATE(BM_PacketReduction, PACKET, PacketAny, LastTrue)

EIGEN_BENCH_AVX512_PACKET(internal::Packet16i);
EIGEN_BENCH_AVX512_PACKET(internal::Packet8l);
EIGEN_BENCH_AVX512_PACKET(internal::Packet16f);
EIGEN_BENCH_AVX512_PACKET(internal::Packet8d);

#ifdef EIGEN_VECTORIZE_AVX512FP16
EIGEN_BENCH_AVX512_PACKET(internal::Packet32h);
EIGEN_BENCH_AVX512_PACKET(internal::Packet16h);
EIGEN_BENCH_AVX512_PACKET(internal::Packet8h);
#endif

#undef EIGEN_BENCH_AVX512_PACKET
#endif

}  // namespace
}  // namespace Eigen
