// Benchmarks for the structured operators' transposition family: transpose(),
// conjugate() and adjoint() reuse the operator's cached DFT symbol (an O(n)
// permutation / conjugation) instead of recomputing an FFT. The *Recomputed
// variants build the same operator through the public constructor, which pays
// the FFT, to quantify the reuse.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>
#include <contrib/Eigen/StructuredMatrices>

using namespace Eigen;

typedef Matrix<double, Dynamic, 1> Vec;

// The generating column of C^T: the index-reversed column of C (what transpose()
// computes internally before attaching the reused symbol).
static Vec reversedGenerator(const Vec& c) {
  const Index n = c.size();
  Vec rev(n);
  rev[0] = c[0];
  if (n > 1) rev.tail(n - 1) = c.tail(n - 1).reverse();
  return rev;
}

// --- Circulant transpose: construction only ---
static void BM_CirculantTransposeCached(benchmark::State& state) {
  const Index n = state.range(0);
  Vec c = Vec::Random(n);
  Circulant<double> C(c);
  for (auto _ : state) {
    Circulant<double> Ct = C.transpose();  // symbol reuse: no FFT
    benchmark::DoNotOptimize(Ct);
  }
}
BENCHMARK(BM_CirculantTransposeCached)->Arg(256)->Arg(1024)->Arg(4096)->Arg(16384);

static void BM_CirculantTransposeRecomputed(benchmark::State& state) {
  const Index n = state.range(0);
  Vec c = Vec::Random(n);
  Vec rev = reversedGenerator(c);
  for (auto _ : state) {
    Circulant<double> Ct(rev);  // public constructor: recomputes the symbol by FFT
    benchmark::DoNotOptimize(Ct);
  }
}
BENCHMARK(BM_CirculantTransposeRecomputed)->Arg(256)->Arg(1024)->Arg(4096)->Arg(16384);

// --- Circulant transpose: end-to-end transposed product y = C^T * x ---
static void BM_CirculantTransposeProductCached(benchmark::State& state) {
  const Index n = state.range(0);
  Vec c = Vec::Random(n), x = Vec::Random(n), y(n);
  Circulant<double> C(c);
  for (auto _ : state) {
    y.noalias() = C.transpose() * x;
    benchmark::DoNotOptimize(y.data());
  }
}
BENCHMARK(BM_CirculantTransposeProductCached)->Arg(256)->Arg(1024)->Arg(4096)->Arg(16384);

static void BM_CirculantTransposeProductRecomputed(benchmark::State& state) {
  const Index n = state.range(0);
  Vec c = Vec::Random(n), x = Vec::Random(n), y(n);
  Vec rev = reversedGenerator(c);
  for (auto _ : state) {
    Circulant<double> Ct(rev);
    y.noalias() = Ct * x;
    benchmark::DoNotOptimize(y.data());
  }
}
BENCHMARK(BM_CirculantTransposeProductRecomputed)->Arg(256)->Arg(1024)->Arg(4096)->Arg(16384);

// --- Toeplitz adjoint: construction only ---
static void BM_ToeplitzAdjointCached(benchmark::State& state) {
  const Index n = state.range(0);
  Vec c = Vec::Random(n), r = Vec::Random(n);
  Toeplitz<double> T(c, r);
  for (auto _ : state) {
    Toeplitz<double> Ta = T.adjoint();  // symbol reuse: no FFT
    benchmark::DoNotOptimize(Ta);
  }
}
BENCHMARK(BM_ToeplitzAdjointCached)->Arg(256)->Arg(1024)->Arg(4096)->Arg(16384);

static void BM_ToeplitzAdjointRecomputed(benchmark::State& state) {
  const Index n = state.range(0);
  Vec c = Vec::Random(n), r = Vec::Random(n);
  // The adjoint's generators (real scalars, so conjugation is the identity).
  Vec ca(n);
  ca[0] = c[0];
  if (n > 1) ca.tail(n - 1) = r.tail(n - 1);
  for (auto _ : state) {
    Toeplitz<double> Ta(ca, c);  // public constructor: recomputes the embedding symbol
    benchmark::DoNotOptimize(Ta);
  }
}
BENCHMARK(BM_ToeplitzAdjointRecomputed)->Arg(256)->Arg(1024)->Arg(4096)->Arg(16384);
