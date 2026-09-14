// Benchmarks for the Vandermonde operator: the O(mn)-flops / O(m)-storage
// Horner product against the equivalent dense GEMV on the materialized matrix
// (comparable O(mn) arithmetic, O(mn) storage), and the O(n^2)
// Björck-Pereyra square solve against a dense PartialPivLU factor-and-solve
// (O(n^3)). These cases cover nominal real-input performance. Both solve
// variants time the full pipeline from the stored representation to the
// solution; the dense one is additionally handed the materialized matrix for
// free (built outside the loop).
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <contrib/Eigen/StructuredMatrices>

using namespace Eigen;

typedef Matrix<double, Dynamic, 1> Vec;
typedef Matrix<double, Dynamic, Dynamic> Mat;

// Chebyshev nodes in (-1, 1): deterministic, bounded, and distinct (no
// Björck-Pereyra division by zero or exact-zero LU pivot).
static Vec chebyshevNodes(Index n) {
  Vec x(n);
  for (Index i = 0; i < n; ++i) x[i] = std::cos(EIGEN_PI * double(2 * i + 1) / double(2 * n));
  return x;
}

static Vec polynomialCoefficients(Index n) {
  Vec a(n);
  for (Index i = 0; i < n; ++i) a[i] = std::cos(0.25 * double(i + 1)) / double(i + 1);
  return a;
}

// Long-double Horner reference used only before timing. The dimension-scaled
// bound accounts for at most n multiply-adds in each polynomial evaluation.
static bool validateProduct(benchmark::State& state, const Vec& x, const Vec& a, const Vec& actual) {
  Vec expected(x.size());
  for (Index i = 0; i < x.size(); ++i) {
    long double acc = static_cast<long double>(a[a.size() - 1]);
    for (Index j = a.size() - 2; j >= 0; --j)
      acc = acc * static_cast<long double>(x[i]) + static_cast<long double>(a[j]);
    expected[i] = static_cast<double>(acc);
  }
  const double tol = 64.0 * double(a.size()) * NumTraits<double>::epsilon();
  if (!actual.allFinite() || (actual - expected).norm() > tol * numext::maxi(1.0, expected.norm())) {
    state.SkipWithError("Vandermonde product failed validation");
    return false;
  }
  return true;
}

// The large Chebyshev-node systems are extremely ill-conditioned, so a random
// right-hand side has no meaningful double-precision monomial solution. The
// exact constant polynomial f_i=1 gives both algorithms the finite,
// independently known solution a=e_0 and keeps this a reproducible nominal-path
// benchmark. Validate the solution once before entering either timed loop.
static bool validateConstantSolve(benchmark::State& state, const Vec& actual) {
  Vec expected = Vec::Zero(actual.size());
  expected[0] = 1.0;
  const double tol = 64.0 * double(actual.size()) * NumTraits<double>::epsilon();
  if (!actual.allFinite() || (actual - expected).norm() > tol) {
    state.SkipWithError("Vandermonde solve failed validation");
    return false;
  }
  return true;
}

// --- Product y = V * a: Horner rule on the nodes ---
static void BM_VandermondeProduct(benchmark::State& state) {
  const Index m = state.range(0), n = state.range(1);
  Vec x = chebyshevNodes(m), a = polynomialCoefficients(n), y(m);
  Vandermonde<double> V(x, n);
  Vec check = V * a;
  if (!validateProduct(state, x, a, check)) return;
  for (auto _ : state) {
    y.noalias() = V * a;
    benchmark::DoNotOptimize(y.data());
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_VandermondeProduct)
    ->Args({64, 64})
    ->Args({256, 256})
    ->Args({1024, 1024})
    ->Args({4096, 4096})
    ->Args({4096, 256})
    ->Args({256, 4096})
    ->ArgNames({"m", "n"});

// --- Product y = dense * a: GEMV on the pre-materialized matrix ---
static void BM_VandermondeProductDense(benchmark::State& state) {
  const Index m = state.range(0), n = state.range(1);
  Vec x = chebyshevNodes(m), a = polynomialCoefficients(n), y(m);
  Vandermonde<double> V(x, n);
  Mat dense = V;  // materialized once, outside the timed loop
  Vec check = dense * a;
  if (!validateProduct(state, x, a, check)) return;
  for (auto _ : state) {
    y.noalias() = dense * a;
    benchmark::DoNotOptimize(y.data());
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_VandermondeProductDense)
    ->Args({64, 64})
    ->Args({256, 256})
    ->Args({1024, 1024})
    ->Args({4096, 4096})
    ->Args({4096, 256})
    ->Args({256, 4096})
    ->ArgNames({"m", "n"});

// --- Square solve V * a = f: Björck-Pereyra divided-difference recurrences ---
static void BM_BjorckPereyraSolve(benchmark::State& state) {
  const Index n = state.range(0);
  Vec x = chebyshevNodes(n), f = Vec::Ones(n), a(n);
  Vandermonde<double> V(x);
  BjorckPereyra<double> bpCheck(V);
  Vec check = bpCheck.solve(f);
  if (!validateConstantSolve(state, check)) return;
  for (auto _ : state) {
    BjorckPereyra<double> bp(V);  // compute() includes the O(n^2) duplicate scan
    a = bp.solve(f);
    benchmark::DoNotOptimize(a.data());
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_BjorckPereyraSolve)->Arg(16)->Arg(64)->Arg(256)->Arg(1024);

// --- Square solve dense * a = f: PartialPivLU on the pre-materialized matrix ---
static void BM_VandermondeSolveDense(benchmark::State& state) {
  const Index n = state.range(0);
  Vec x = chebyshevNodes(n), f = Vec::Ones(n), a(n);
  Vandermonde<double> V(x);
  Mat dense = V;  // materialized once, outside the timed loop
  PartialPivLU<Mat> luCheck(dense);
  Vec check = luCheck.solve(f);
  if (!validateConstantSolve(state, check)) return;
  for (auto _ : state) {
    PartialPivLU<Mat> lu(dense);  // the O(n^3) elimination
    a = lu.solve(f);
    benchmark::DoNotOptimize(a.data());
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_VandermondeSolveDense)->Arg(16)->Arg(64)->Arg(256)->Arg(1024);
