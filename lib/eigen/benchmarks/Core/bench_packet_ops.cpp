// Benchmarks for the PacketMath implementations of `plset`, `ploaddup`,
// `ploadquad`, `predux_mul`, and `ptranspose`, at the packet-op level and
// shared across whichever architecture backend the build targets. To
// compare against a prior implementation, build and run this same file
// against the Eigen checkout in question -- it only calls the public
// `Eigen::internal` packet API, so it is source-compatible with whatever
// PacketMath.h happens to provide.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>

#include <cmath>
#include <cstdint>
#include <type_traits>

namespace Eigen {
namespace {

using internal::packet_traits;
using internal::PacketBlock;
using internal::pfirst;
using internal::ploadu;
using internal::pstoreu;

template <typename Packet, int N>
EIGEN_DONT_INLINE void call_ptranspose(PacketBlock<Packet, N>& kernel) {
  internal::ptranspose(kernel);
}

// ---- plset ----

template <typename Scalar>
void BM_Plset(benchmark::State& state) {
  using Packet = typename packet_traits<Scalar>::type;
  constexpr int N = packet_traits<Scalar>::size;
  Scalar a = Scalar(7);
  Scalar out[N];

  pstoreu(out, internal::plset<Packet>(a));
  for (int i = 0; i < N; ++i) {
    if (out[i] != static_cast<Scalar>(a + i)) {
      state.SkipWithError("Plset: materialized result does not match scalar reference");
      return;
    }
  }

  benchmark::DoNotOptimize(a);
  for (auto _ : state) {
    pstoreu(out, internal::plset<Packet>(a));
    benchmark::DoNotOptimize(out);
  }
}
BENCHMARK(BM_Plset<numext::int32_t>)->Name("Plset_int32");
BENCHMARK(BM_Plset<float>)->Name("Plset_float");
BENCHMARK(BM_Plset<double>)->Name("Plset_double");

// ---- ploaddup ----

template <typename Scalar>
void BM_Ploaddup(benchmark::State& state) {
  using Packet = typename packet_traits<Scalar>::type;
  constexpr int N = packet_traits<Scalar>::size;
  Scalar in[N];
  for (int i = 0; i < N; ++i) in[i] = static_cast<Scalar>(i);
  Scalar out[N];

  pstoreu(out, internal::ploaddup<Packet>(in));
  for (int i = 0; i < N; ++i) {
    if (out[i] != in[i / 2]) {
      state.SkipWithError("Ploaddup: materialized result does not match scalar reference");
      return;
    }
  }

  benchmark::DoNotOptimize(in);
  for (auto _ : state) {
    pstoreu(out, internal::ploaddup<Packet>(in));
    benchmark::DoNotOptimize(out);
  }
}
BENCHMARK(BM_Ploaddup<numext::int32_t>)->Name("Ploaddup_int32");
BENCHMARK(BM_Ploaddup<float>)->Name("Ploaddup_float");
BENCHMARK(BM_Ploaddup<double>)->Name("Ploaddup_double");

// ---- ploadquad ----

template <typename Scalar>
void BM_Ploadquad(benchmark::State& state) {
  using Packet = typename packet_traits<Scalar>::type;
  constexpr int N = packet_traits<Scalar>::size;
  Scalar in[N];
  for (int i = 0; i < N; ++i) in[i] = static_cast<Scalar>(i);
  Scalar out[N];

  pstoreu(out, internal::ploadquad<Packet>(in));
  for (int i = 0; i < N; ++i) {
    if (out[i] != in[i / 4]) {
      state.SkipWithError("Ploadquad: materialized result does not match scalar reference");
      return;
    }
  }

  benchmark::DoNotOptimize(in);
  for (auto _ : state) {
    pstoreu(out, internal::ploadquad<Packet>(in));
    benchmark::DoNotOptimize(out);
  }
}
BENCHMARK(BM_Ploadquad<numext::int32_t>)->Name("Ploadquad_int32");
BENCHMARK(BM_Ploadquad<float>)->Name("Ploadquad_float");
BENCHMARK(BM_Ploadquad<double>)->Name("Ploadquad_double");

// ---- predux_mul ----
// Inputs are chosen so the true product is exactly representable regardless
// of the order the reduction folds lanes together, at every vector length:
// powers of two multiply without rounding, and centering the exponents around
// zero keeps both the inputs and the product in range for float/double even
// at N == 64; multiplying by 1 is exact and overflow-free for int32.

template <typename Scalar>
void fill_redux_mul_input(Scalar (&in)[packet_traits<Scalar>::size], Scalar& expected) {
  constexpr int N = packet_traits<Scalar>::size;
  if constexpr (std::is_integral<Scalar>::value) {
    for (int i = 0; i < N; ++i) in[i] = Scalar(1);
    in[0] = Scalar(-3);
    expected = Scalar(-3);
    if (N > 1) {
      in[N - 1] = Scalar(2);
      expected = Scalar(-6);
    }
  } else {
    for (int i = 0; i < N; ++i) in[i] = std::ldexp(Scalar(1), i - N / 2);
    expected = std::ldexp(Scalar(1), -N / 2);
  }
}

template <typename Scalar>
void BM_ReduxMul(benchmark::State& state) {
  using Packet = typename packet_traits<Scalar>::type;
  constexpr int N = packet_traits<Scalar>::size;
  Scalar in[N];
  Scalar expected;
  fill_redux_mul_input<Scalar>(in, expected);
  Packet a = ploadu<Packet>(in);

  if (internal::predux_mul<Packet>(a) != expected) {
    state.SkipWithError("ReduxMul: materialized result does not match scalar reference");
    return;
  }

  benchmark::DoNotOptimize(a);
  for (auto _ : state) benchmark::DoNotOptimize(internal::predux_mul<Packet>(a));
}
BENCHMARK(BM_ReduxMul<numext::int32_t>)->Name("ReduxMul_int32");
BENCHMARK(BM_ReduxMul<float>)->Name("ReduxMul_float");
BENCHMARK(BM_ReduxMul<double>)->Name("ReduxMul_double");

// ---- ptranspose ----
// Benchmarked at N == the type's packet width, i.e. a full square transpose,
// so the expected result is simply new_packet[i][k] == old_packet[k][i].
// Correctness is checked once on a scratch kernel -- ptranspose applied
// repeatedly toggles between the original and transposed state, so checking
// after the timed loop would depend on the (unpredictable) iteration count.
//
// call_ptranspose is deliberately EIGEN_DONT_INLINE: real callers inline
// ptranspose on register-resident packets, but this wrapper forces the block
// through memory, so this benchmark understates the win; transposeInPlace is
// the macro benchmark for the inlined case.

template <typename Scalar>
void BM_Ptranspose(benchmark::State& state) {
  using Packet = typename packet_traits<Scalar>::type;
  constexpr int N = packet_traits<Scalar>::size;
  Scalar in[N * N];
  for (int i = 0; i < N * N; ++i) in[i] = static_cast<Scalar>(i);

  PacketBlock<Packet, N> check;
  for (int i = 0; i < N; ++i) check.packet[i] = ploadu<Packet>(in + i * N);
  call_ptranspose<Packet, N>(check);
  for (int i = 0; i < N; ++i) {
    Scalar row[N];
    pstoreu(row, check.packet[i]);
    for (int k = 0; k < N; ++k) {
      if (row[k] != in[k * N + i]) {
        state.SkipWithError("Ptranspose: materialized result does not match scalar reference");
        return;
      }
    }
  }

  PacketBlock<Packet, N> kernel;
  for (int i = 0; i < N; ++i) kernel.packet[i] = ploadu<Packet>(in + i * N);
  for (auto _ : state) {
    call_ptranspose<Packet, N>(kernel);
    benchmark::DoNotOptimize(pfirst<Packet>(kernel.packet[0]));
  }
}
BENCHMARK(BM_Ptranspose<numext::int32_t>)->Name("Ptranspose_int32");
BENCHMARK(BM_Ptranspose<float>)->Name("Ptranspose_float");
BENCHMARK(BM_Ptranspose<double>)->Name("Ptranspose_double");

}  // namespace
}  // namespace Eigen
