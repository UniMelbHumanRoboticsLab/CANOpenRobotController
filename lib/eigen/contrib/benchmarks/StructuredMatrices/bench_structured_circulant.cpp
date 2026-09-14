// Benchmarks for the Circulant operator's public operations: the fast product
// across its three dispatch tiers (scalar, segment, FFT; 5-smooth and prime
// transform sizes), the direct solve, and the closed-form factorizations
// (determinant, rank, inverse, eigendecomposition, SVD), each against the dense
// alternative where one exists. The near-overflow product pins the power-of-two
// scaled FFT path, which is tuned to cost nothing for moderate magnitudes.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Dense>
#include <contrib/Eigen/StructuredMatrices>

using namespace Eigen;

typedef Matrix<double, Dynamic, 1> Vec;
typedef Matrix<double, Dynamic, Dynamic> Mat;
typedef Matrix<std::complex<double>, Dynamic, 1> CVec;

// Diagonally dominant generator: keeps the operator safely invertible for the
// solve/inverse/determinant benchmarks.
static Vec wellConditionedGenerator(Index n) {
  Vec c = Vec::Random(n);
  c[0] += 2.0 * double(n);
  return c;
}

// --- Construction: the one-time symbol FFT (5-smooth vs prime sizes) ---
static void BM_CirculantConstruct(benchmark::State& state) {
  const Index n = state.range(0);
  Vec c = Vec::Random(n);
  for (auto _ : state) {
    Circulant<double> C(c);
    benchmark::DoNotOptimize(C);
  }
}
BENCHMARK(BM_CirculantConstruct)->Arg(96)->Arg(97)->Arg(1024)->Arg(4096);

// --- Product y = C * x across the dispatch tiers ---
// 8: scalar loop; 32: segment tier boundary; 96/1024/4096: FFT (5-smooth);
// 97: FFT at a prime size (no 5-smooth padding for a circulant).
static void BM_CirculantProduct(benchmark::State& state) {
  const Index n = state.range(0);
  Vec c = Vec::Random(n), x = Vec::Random(n), y(n);
  Circulant<double> C(c);
  for (auto _ : state) {
    y.noalias() = C * x;
    benchmark::DoNotOptimize(y.data());
  }
}
BENCHMARK(BM_CirculantProduct)->Arg(8)->Arg(32)->Arg(96)->Arg(97)->Arg(1024)->Arg(4096);

static void BM_CirculantProductDense(benchmark::State& state) {
  const Index n = state.range(0);
  Vec c = Vec::Random(n), x = Vec::Random(n), y(n);
  Mat dense = Circulant<double>(c);
  for (auto _ : state) {
    y.noalias() = dense * x;
    benchmark::DoNotOptimize(y.data());
  }
}
BENCHMARK(BM_CirculantProductDense)->Arg(96)->Arg(1024)->Arg(4096);

static void BM_CirculantProductComplex(benchmark::State& state) {
  const Index n = state.range(0);
  CVec c = CVec::Random(n), x = CVec::Random(n), y(n);
  Circulant<std::complex<double>> C(c);
  for (auto _ : state) {
    y.noalias() = C * x;
    benchmark::DoNotOptimize(y.data());
  }
}
BENCHMARK(BM_CirculantProductComplex)->Arg(96)->Arg(4096);

// Multi-column right-hand sides: one FFT round trip per column.
static void BM_CirculantProductMultiRhs(benchmark::State& state) {
  const Index n = state.range(0), k = state.range(1);
  Vec c = Vec::Random(n);
  Mat X = Mat::Random(n, k), Y(n, k);
  Circulant<double> C(c);
  for (auto _ : state) {
    Y.noalias() = C * X;
    benchmark::DoNotOptimize(Y.data());
  }
}
BENCHMARK(BM_CirculantProductMultiRhs)->ArgsProduct({{96, 4096}, {8}});

// Near-overflow magnitudes engage the exact power-of-two scaling inside the FFT
// path; regression guard for that machinery staying cheap.
static void BM_CirculantProductNearOverflow(benchmark::State& state) {
  const Index n = state.range(0);
  const double huge = (std::numeric_limits<double>::max)() / 16.0;
  Vec c = Vec::Random(n), x = Vec::Constant(n, huge), y(n);
  Circulant<double> C(Vec(c / double(n)));  // keep the products representable
  for (auto _ : state) {
    y.noalias() = C * x;
    benchmark::DoNotOptimize(y.data());
  }
}
BENCHMARK(BM_CirculantProductNearOverflow)->Arg(4096);

// --- Solve (direct, in the Fourier domain) vs a dense LU solve ---
static void BM_CirculantSolve(benchmark::State& state) {
  const Index n = state.range(0);
  Vec c = wellConditionedGenerator(n), b = Vec::Random(n);
  Circulant<double> C(c);
  for (auto _ : state) {
    Vec x = C.solve(b);
    benchmark::DoNotOptimize(x.data());
  }
}
BENCHMARK(BM_CirculantSolve)->Arg(96)->Arg(1024)->Arg(4096);

// The dense path pays the O(n^3) factorization on every call, which is the
// honest end-to-end comparison starting from a formed operator.
static void BM_CirculantSolveDense(benchmark::State& state) {
  const Index n = state.range(0);
  Vec c = wellConditionedGenerator(n), b = Vec::Random(n);
  Mat dense = Circulant<double>(c);
  for (auto _ : state) {
    Vec x = PartialPivLU<Mat>(dense).solve(b);
    benchmark::DoNotOptimize(x.data());
  }
}
BENCHMARK(BM_CirculantSolveDense)->Arg(96)->Arg(1024);

// --- Determinant (balanced product of the symbol entries) vs dense LU ---
static void BM_CirculantDeterminant(benchmark::State& state) {
  const Index n = state.range(0);
  Circulant<double> C(wellConditionedGenerator(n));
  for (auto _ : state) {
    double d = C.determinant();
    benchmark::DoNotOptimize(d);
  }
}
BENCHMARK(BM_CirculantDeterminant)->Arg(1024)->Arg(4096);

static void BM_CirculantDeterminantDense(benchmark::State& state) {
  const Index n = state.range(0);
  Mat dense = Circulant<double>(wellConditionedGenerator(n));
  for (auto _ : state) {
    double d = PartialPivLU<Mat>(dense).determinant();
    benchmark::DoNotOptimize(d);
  }
}
BENCHMARK(BM_CirculantDeterminantDense)->Arg(256);

// --- Rank (rescaled symbol moduli against the SVD-style threshold) ---
static void BM_CirculantRank(benchmark::State& state) {
  const Index n = state.range(0);
  Circulant<double> C(wellConditionedGenerator(n));
  for (auto _ : state) {
    Index r = C.rank();
    benchmark::DoNotOptimize(r);
  }
}
BENCHMARK(BM_CirculantRank)->Arg(1024)->Arg(4096);

// --- Inverse operator (reciprocal symbol + one inverse FFT) ---
static void BM_CirculantInverse(benchmark::State& state) {
  const Index n = state.range(0);
  Circulant<double> C(wellConditionedGenerator(n));
  for (auto _ : state) {
    Circulant<double> Ci = C.inverse();
    benchmark::DoNotOptimize(Ci);
  }
}
BENCHMARK(BM_CirculantInverse)->Arg(256)->Arg(1024)->Arg(4096);

// --- Eigendecomposition: closed form vs the dense (non-symmetric) solver ---
static void BM_CirculantEigenvalues(benchmark::State& state) {
  const Index n = state.range(0);
  Circulant<double> C(Vec(Vec::Random(n)));
  for (auto _ : state) {
    CVec ev = C.eigenvalues();
    benchmark::DoNotOptimize(ev.data());
  }
}
BENCHMARK(BM_CirculantEigenvalues)->Arg(1024)->Arg(4096);

// Materializes the dense n x n Fourier eigenvector matrix.
static void BM_CirculantEigenvectors(benchmark::State& state) {
  const Index n = state.range(0);
  Circulant<double> C(Vec(Vec::Random(n)));
  for (auto _ : state) {
    auto V = C.eigenvectors();
    benchmark::DoNotOptimize(V.data());
  }
}
BENCHMARK(BM_CirculantEigenvectors)->Arg(64)->Arg(256);

static void BM_CirculantEigenDense(benchmark::State& state) {
  const Index n = state.range(0);
  Mat dense = Circulant<double>(Vec(Vec::Random(n)));
  for (auto _ : state) {
    EigenSolver<Mat> es(dense);
    benchmark::DoNotOptimize(es.eigenvalues().data());
  }
}
BENCHMARK(BM_CirculantEigenDense)->Arg(256);

// --- SVD: closed form (moduli/phases of the symbol) vs dense BDCSVD ---
static void BM_CirculantSingularValues(benchmark::State& state) {
  const Index n = state.range(0);
  Circulant<double> C(Vec(Vec::Random(n)));
  for (auto _ : state) {
    Vec sv = C.singularValues();
    benchmark::DoNotOptimize(sv.data());
  }
}
BENCHMARK(BM_CirculantSingularValues)->Arg(1024)->Arg(4096);

// Materializes the dense n x n left singular vector matrix.
static void BM_CirculantMatrixU(benchmark::State& state) {
  const Index n = state.range(0);
  Circulant<double> C(Vec(Vec::Random(n)));
  for (auto _ : state) {
    auto U = C.matrixU();
    benchmark::DoNotOptimize(U.data());
  }
}
BENCHMARK(BM_CirculantMatrixU)->Arg(64)->Arg(256);

static void BM_CirculantSvdDense(benchmark::State& state) {
  const Index n = state.range(0);
  Mat dense = Circulant<double>(Vec(Vec::Random(n)));
  for (auto _ : state) {
    BDCSVD<Mat, ComputeThinU | ComputeThinV> svd(dense);
    benchmark::DoNotOptimize(svd.singularValues().data());
  }
}
BENCHMARK(BM_CirculantSvdDense)->Arg(256);
