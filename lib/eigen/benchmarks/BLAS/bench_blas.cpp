// Benchmark for Eigen's BLAS implementation.
//
// Calls the Eigen BLAS C interface directly (the extern "C" functions defined
// in blas/{single,double,complex_single,complex_double}.cpp).
//
// Covers Level 1, 2, and 3 routines — with emphasis on the routines that
// were recently rewritten from f2c to C++: rotm, rotmg, spmv, sbmv, hbmv,
// hpmv, tbmv, lsame, and complex dot products.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>

#include <Eigen/Core>
#include <complex>
#include <vector>

#include "blas/blas.h"

using Eigen::Index;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Flop-rate counter (units = individual flops per call).
static benchmark::Counter GflopsCounter(double flops) {
  return benchmark::Counter(flops, benchmark::Counter::kIsIterationInvariantRate, benchmark::Counter::kIs1000);
}

// Fill a vector with random values in [-1, 1].
template <typename T>
static void fillRand(T* data, Index n) {
  Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>>(data, n).setRandom();
}

// Fill a symmetric band matrix A in BLAS band storage (column-major).
// Upper triangle: A[i,j] stored at a[(k+i-j) + j*lda], 0 <= j-i <= k.
template <typename T>
static void fillSymBandUpper(T* a, int n, int k, int lda) {
  std::fill(a, a + lda * n, T(0));
  for (int j = 0; j < n; ++j)
    for (int i = std::max(0, j - k); i <= j; ++i) a[(k + i - j) + j * lda] = T(std::rand()) / T(RAND_MAX) - T(0.5);
}

// Fill a packed symmetric matrix (upper triangle, column-major).
template <typename T>
static void fillSymPacked(T* ap, int n) {
  int sz = n * (n + 1) / 2;
  for (int i = 0; i < sz; ++i) ap[i] = T(std::rand()) / T(RAND_MAX) - T(0.5);
}

// Fill a triangular band matrix in BLAS band storage (upper, column-major).
template <typename T>
static void fillTriBandUpper(T* a, int n, int k, int lda) {
  std::fill(a, a + lda * n, T(0));
  for (int j = 0; j < n; ++j)
    for (int i = std::max(0, j - k); i <= j; ++i) {
      T val = T(std::rand()) / T(RAND_MAX) - T(0.5);
      if (i == j) val += T(n);  // diagonal dominance
      a[(k + i - j) + j * lda] = val;
    }
}

// ---------------------------------------------------------------------------
// Type-dispatched BLAS wrappers
// ---------------------------------------------------------------------------

inline float blas_dot(int* n, float* x, int* incx, float* y, int* incy) { return sdot_(n, x, incx, y, incy); }
inline double blas_dot(int* n, double* x, int* incx, double* y, int* incy) { return ddot_(n, x, incx, y, incy); }

inline void blas_axpy(int* n, float* a, float* x, int* incx, float* y, int* incy) { saxpy_(n, a, x, incx, y, incy); }
inline void blas_axpy(int* n, double* a, double* x, int* incx, double* y, int* incy) { daxpy_(n, a, x, incx, y, incy); }

inline float blas_nrm2(int* n, float* x, int* incx) { return snrm2_(n, x, incx); }
inline double blas_nrm2(int* n, double* x, int* incx) { return dnrm2_(n, x, incx); }

inline void blas_rotm(int* n, float* x, int* incx, float* y, int* incy, float* p) { srotm_(n, x, incx, y, incy, p); }
inline void blas_rotm(int* n, double* x, int* incx, double* y, int* incy, double* p) { drotm_(n, x, incx, y, incy, p); }

inline void blas_rotmg(float* d1, float* d2, float* x1, float* y1, float* p) { srotmg_(d1, d2, x1, y1, p); }
inline void blas_rotmg(double* d1, double* d2, double* x1, double* y1, double* p) { drotmg_(d1, d2, x1, y1, p); }

inline void blas_dotcw(int* n, float* cx, int* incx, float* cy, int* incy, float* res) {
  cdotcw_(n, cx, incx, cy, incy, res);
}
inline void blas_dotcw(int* n, double* cx, int* incx, double* cy, int* incy, double* res) {
  zdotcw_(n, cx, incx, cy, incy, res);
}

inline void blas_gemv(char* t, int* m, int* n, float* a, float* A, int* lda, float* x, int* incx, float* b, float* y,
                      int* incy) {
  sgemv_(t, m, n, a, A, lda, x, incx, b, y, incy);
}
inline void blas_gemv(char* t, int* m, int* n, double* a, double* A, int* lda, double* x, int* incx, double* b,
                      double* y, int* incy) {
  dgemv_(t, m, n, a, A, lda, x, incx, b, y, incy);
}

inline void blas_spmv(char* uplo, int* n, float* alpha, float* ap, float* x, int* incx, float* beta, float* y,
                      int* incy) {
  sspmv_(uplo, n, alpha, ap, x, incx, beta, y, incy);
}
inline void blas_spmv(char* uplo, int* n, double* alpha, double* ap, double* x, int* incx, double* beta, double* y,
                      int* incy) {
  dspmv_(uplo, n, alpha, ap, x, incx, beta, y, incy);
}

inline void blas_sbmv(char* uplo, int* n, int* k, float* alpha, float* a, int* lda, float* x, int* incx, float* beta,
                      float* y, int* incy) {
  ssbmv_(uplo, n, k, alpha, a, lda, x, incx, beta, y, incy);
}
inline void blas_sbmv(char* uplo, int* n, int* k, double* alpha, double* a, int* lda, double* x, int* incx,
                      double* beta, double* y, int* incy) {
  dsbmv_(uplo, n, k, alpha, a, lda, x, incx, beta, y, incy);
}

inline void blas_tbmv(char* uplo, char* trans, char* diag, int* n, int* k, float* a, int* lda, float* x, int* incx) {
  stbmv_(uplo, trans, diag, n, k, a, lda, x, incx);
}
inline void blas_tbmv(char* uplo, char* trans, char* diag, int* n, int* k, double* a, int* lda, double* x, int* incx) {
  dtbmv_(uplo, trans, diag, n, k, a, lda, x, incx);
}

inline void blas_hbmv(char* uplo, int* n, int* k, float* alpha, float* a, int* lda, float* x, int* incx, float* beta,
                      float* y, int* incy) {
  chbmv_(uplo, n, k, alpha, a, lda, x, incx, beta, y, incy);
}
inline void blas_hbmv(char* uplo, int* n, int* k, double* alpha, double* a, int* lda, double* x, int* incx,
                      double* beta, double* y, int* incy) {
  zhbmv_(uplo, n, k, alpha, a, lda, x, incx, beta, y, incy);
}

inline void blas_hpmv(char* uplo, int* n, float* alpha, float* ap, float* x, int* incx, float* beta, float* y,
                      int* incy) {
  chpmv_(uplo, n, alpha, ap, x, incx, beta, y, incy);
}
inline void blas_hpmv(char* uplo, int* n, double* alpha, double* ap, double* x, int* incx, double* beta, double* y,
                      int* incy) {
  zhpmv_(uplo, n, alpha, ap, x, incx, beta, y, incy);
}

inline void blas_gemm(char* ta, char* tb, int* m, int* n, int* k, float* alpha, float* a, int* lda, float* b, int* ldb,
                      float* beta, float* c, int* ldc) {
  sgemm_(ta, tb, m, n, k, alpha, a, lda, b, ldb, beta, c, ldc);
}
inline void blas_gemm(char* ta, char* tb, int* m, int* n, int* k, double* alpha, double* a, int* lda, double* b,
                      int* ldb, double* beta, double* c, int* ldc) {
  dgemm_(ta, tb, m, n, k, alpha, a, lda, b, ldb, beta, c, ldc);
}

// =========================================================================
//  Level 1 — Real
// =========================================================================

// ----- SDOT / DDOT -----
template <typename T>
static void BM_dot(benchmark::State& state) {
  int n = static_cast<int>(state.range(0));
  int one = 1;
  std::vector<T> x(n), y(n);
  fillRand(x.data(), n);
  fillRand(y.data(), n);
  for (auto _ : state) {
    T r = blas_dot(&n, x.data(), &one, y.data(), &one);
    benchmark::DoNotOptimize(r);
  }
  state.counters["GFLOPS"] = GflopsCounter(2.0 * n);
}

// ----- SAXPY / DAXPY -----
template <typename T>
static void BM_axpy(benchmark::State& state) {
  int n = static_cast<int>(state.range(0));
  int one = 1;
  T alpha = T(2.5);
  std::vector<T> x(n), y(n);
  fillRand(x.data(), n);
  fillRand(y.data(), n);
  for (auto _ : state) {
    blas_axpy(&n, &alpha, x.data(), &one, y.data(), &one);
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["GFLOPS"] = GflopsCounter(2.0 * n);
}

// ----- SNRM2 / DNRM2 -----
template <typename T>
static void BM_nrm2(benchmark::State& state) {
  int n = static_cast<int>(state.range(0));
  int one = 1;
  std::vector<T> x(n);
  fillRand(x.data(), n);
  for (auto _ : state) {
    T r = blas_nrm2(&n, x.data(), &one);
    benchmark::DoNotOptimize(r);
  }
  // Nominal flops; Eigen's stableNorm() does more work internally.
  state.counters["GFLOPS"] = GflopsCounter(2.0 * n - 1);
}

// ----- SROTM / DROTM -----
template <typename T>
static void BM_rotm(benchmark::State& state) {
  int n = static_cast<int>(state.range(0));
  int one = 1;
  std::vector<T> x(n), y(n);
  T param[5] = {T(-1), T(0.6), T(-0.8), T(0.8), T(0.6)};  // full rotation
  fillRand(x.data(), n);
  fillRand(y.data(), n);
  for (auto _ : state) {
    blas_rotm(&n, x.data(), &one, y.data(), &one, param);
    benchmark::DoNotOptimize(x.data());
    benchmark::DoNotOptimize(y.data());
  }
  // 4 muls + 2 adds per element pair.
  state.counters["GFLOPS"] = GflopsCounter(6.0 * n);
}

// ----- SROTMG / DROTMG -----
template <typename T>
static void BM_rotmg(benchmark::State& state) {
  T d1 = T(2), d2 = T(3), x1 = T(1), y1 = T(0.5);
  T param[5];
  for (auto _ : state) {
    T td1 = d1, td2 = d2, tx1 = x1;
    blas_rotmg(&td1, &td2, &tx1, &y1, param);
    benchmark::DoNotOptimize(param);
  }
}

// =========================================================================
//  Level 1 — Complex
// =========================================================================

// Complex conjugate dot product via the worker functions (cdotcw_ / zdotcw_)
// which use an output pointer, avoiding the ABI ambiguity of the struct-returning
// cdotc_ / zdotc_ wrappers.
template <typename T>
static void BM_dotc(benchmark::State& state) {
  int n = static_cast<int>(state.range(0));
  int one = 1;
  std::vector<T> x(2 * n), y(2 * n);  // interleaved real/imag
  fillRand(x.data(), 2 * n);
  fillRand(y.data(), 2 * n);
  T res[2];
  for (auto _ : state) {
    blas_dotcw(&n, x.data(), &one, y.data(), &one, res);
    benchmark::DoNotOptimize(res);
  }
  // Conjugate dot: 6 mul + 2 add per element = 8n flops.
  state.counters["GFLOPS"] = GflopsCounter(8.0 * n);
}

// =========================================================================
//  Level 2 — General Matrix-Vector (SGEMV / DGEMV)
// =========================================================================

template <typename T>
static void BM_gemv(benchmark::State& state) {
  int m = static_cast<int>(state.range(0));
  int n = static_cast<int>(state.range(1));
  int one = 1;
  T alpha = T(1), beta = T(0);
  char trans = 'N';
  std::vector<T> a(m * n), x(n), y(m);
  fillRand(a.data(), m * n);
  fillRand(x.data(), n);
  fillRand(y.data(), m);
  for (auto _ : state) {
    blas_gemv(&trans, &m, &n, &alpha, a.data(), &m, x.data(), &one, &beta, y.data(), &one);
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["GFLOPS"] = GflopsCounter(2.0 * m * n);
}

// =========================================================================
//  Level 2 — Symmetric Packed (SSPMV / DSPMV)
// =========================================================================

template <typename T>
static void BM_spmv(benchmark::State& state) {
  int n = static_cast<int>(state.range(0));
  int one = 1;
  T alpha = T(1), beta = T(0);
  char uplo = 'U';
  int sz = n * (n + 1) / 2;
  std::vector<T> ap(sz), x(n), y(n);
  fillSymPacked(ap.data(), n);
  fillRand(x.data(), n);
  fillRand(y.data(), n);
  for (auto _ : state) {
    blas_spmv(&uplo, &n, &alpha, ap.data(), x.data(), &one, &beta, y.data(), &one);
    benchmark::DoNotOptimize(y.data());
  }
  // Symmetric: each off-diag element contributes to two y entries.
  state.counters["GFLOPS"] = GflopsCounter(2.0 * n * n);
}

// =========================================================================
//  Level 2 — Symmetric Band (SSBMV / DSBMV)
// =========================================================================

template <typename T>
static void BM_sbmv(benchmark::State& state) {
  int n = static_cast<int>(state.range(0));
  int k = static_cast<int>(state.range(1));
  int lda = k + 1;
  int one = 1;
  T alpha = T(1), beta = T(0);
  char uplo = 'U';
  std::vector<T> a(lda * n), x(n), y(n);
  fillSymBandUpper(a.data(), n, k, lda);
  fillRand(x.data(), n);
  fillRand(y.data(), n);
  for (auto _ : state) {
    blas_sbmv(&uplo, &n, &k, &alpha, a.data(), &lda, x.data(), &one, &beta, y.data(), &one);
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["GFLOPS"] = GflopsCounter(2.0 * n * (2 * k + 1));
}

// =========================================================================
//  Level 2 — Triangular Band (STBMV / DTBMV)
// =========================================================================

template <typename T>
static void BM_tbmv(benchmark::State& state) {
  int n = static_cast<int>(state.range(0));
  int k = static_cast<int>(state.range(1));
  int lda = k + 1;
  int one = 1;
  char uplo = 'U', trans = 'N', diag = 'N';
  std::vector<T> a(lda * n), x(n), x_orig(n);
  fillTriBandUpper(a.data(), n, k, lda);
  fillRand(x_orig.data(), n);
  for (auto _ : state) {
    state.PauseTiming();
    std::copy(x_orig.begin(), x_orig.end(), x.begin());
    state.ResumeTiming();
    blas_tbmv(&uplo, &trans, &diag, &n, &k, a.data(), &lda, x.data(), &one);
    benchmark::DoNotOptimize(x.data());
  }
  state.counters["GFLOPS"] = GflopsCounter(1.0 * n * (k + 1));
}

// =========================================================================
//  Level 2 — Hermitian Band (CHBMV / ZHBMV)
// =========================================================================

template <typename T>
static void BM_hbmv(benchmark::State& state) {
  int n = static_cast<int>(state.range(0));
  int k = static_cast<int>(state.range(1));
  int lda = k + 1;
  int one = 1;
  char uplo = 'U';
  // Complex: each element is 2 reals.
  std::vector<T> a(2 * lda * n), x(2 * n), y(2 * n);
  T alpha[2] = {T(1), T(0)};
  T beta[2] = {T(0), T(0)};
  fillRand(a.data(), 2 * lda * n);
  // Make diagonal real (imag part = 0).
  for (int j = 0; j < n; ++j) a[2 * (k + j * lda) + 1] = T(0);
  fillRand(x.data(), 2 * n);
  fillRand(y.data(), 2 * n);
  for (auto _ : state) {
    blas_hbmv(&uplo, &n, &k, alpha, a.data(), &lda, x.data(), &one, beta, y.data(), &one);
    benchmark::DoNotOptimize(y.data());
  }
  // Complex hermitian band: 8*n*(2k+1) flops approximately.
  state.counters["GFLOPS"] = GflopsCounter(8.0 * n * (2 * k + 1));
}

// =========================================================================
//  Level 2 — Hermitian Packed (CHPMV / ZHPMV)
// =========================================================================

template <typename T>
static void BM_hpmv(benchmark::State& state) {
  int n = static_cast<int>(state.range(0));
  int one = 1;
  char uplo = 'U';
  int sz = n * (n + 1) / 2;
  std::vector<T> ap(2 * sz), x(2 * n), y(2 * n);
  T alpha[2] = {T(1), T(0)};
  T beta[2] = {T(0), T(0)};
  fillRand(ap.data(), 2 * sz);
  // Make diagonal real.
  int kk = 0;
  for (int j = 0; j < n; ++j) {
    ap[2 * (kk + j) + 1] = T(0);
    kk += j + 1;
  }
  fillRand(x.data(), 2 * n);
  fillRand(y.data(), 2 * n);
  for (auto _ : state) {
    blas_hpmv(&uplo, &n, alpha, ap.data(), x.data(), &one, beta, y.data(), &one);
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["GFLOPS"] = GflopsCounter(8.0 * n * n);
}

// =========================================================================
//  Level 3 — General Matrix Multiply (SGEMM / DGEMM)
// =========================================================================

template <typename T>
static void BM_gemm(benchmark::State& state) {
  int n = static_cast<int>(state.range(0));
  T alpha = T(1), beta = T(0);
  char trans = 'N';
  std::vector<T> a(n * n), b(n * n), c(n * n);
  fillRand(a.data(), n * n);
  fillRand(b.data(), n * n);
  fillRand(c.data(), n * n);
  for (auto _ : state) {
    blas_gemm(&trans, &trans, &n, &n, &n, &alpha, a.data(), &n, b.data(), &n, &beta, c.data(), &n);
    benchmark::DoNotOptimize(c.data());
  }
  state.counters["GFLOPS"] = GflopsCounter(2.0 * n * n * n);
}

// =========================================================================
//  Register benchmarks
// =========================================================================

// clang-format off

// --- Vector sizes for Level 1 ---
#define L1_SIZES ->Arg(64)->Arg(256)->Arg(1024)->Arg(4096)->Arg(16384)->Arg(65536)

BENCHMARK(BM_dot<float>)    L1_SIZES ->Name("sdot");
BENCHMARK(BM_dot<double>)   L1_SIZES ->Name("ddot");
BENCHMARK(BM_axpy<float>)   L1_SIZES ->Name("saxpy");
BENCHMARK(BM_axpy<double>)  L1_SIZES ->Name("daxpy");
BENCHMARK(BM_nrm2<float>)   L1_SIZES ->Name("snrm2");
BENCHMARK(BM_nrm2<double>)  L1_SIZES ->Name("dnrm2");
BENCHMARK(BM_rotm<float>)   L1_SIZES ->Name("srotm");
BENCHMARK(BM_rotm<double>)  L1_SIZES ->Name("drotm");
BENCHMARK(BM_rotmg<float>)  ->Name("srotmg");
BENCHMARK(BM_rotmg<double>) ->Name("drotmg");
BENCHMARK(BM_dotc<float>)   L1_SIZES ->Name("cdotc");
BENCHMARK(BM_dotc<double>)  L1_SIZES ->Name("zdotc");

#undef L1_SIZES

// --- Matrix sizes for Level 2 ---
// GEMV: {m, n}
#define GEMV_SIZES \
    ->Args({64, 64})->Args({256, 256})->Args({1024, 1024})->Args({4096, 4096}) \
    ->Args({4096, 64})->Args({64, 4096})

BENCHMARK(BM_gemv<float>)   GEMV_SIZES ->Name("sgemv");
BENCHMARK(BM_gemv<double>)  GEMV_SIZES ->Name("dgemv");
#undef GEMV_SIZES

// Symmetric packed: {n}
#define SPM_SIZES ->Arg(64)->Arg(256)->Arg(1024)->Arg(4096)

BENCHMARK(BM_spmv<float>)  SPM_SIZES ->Name("sspmv");
BENCHMARK(BM_spmv<double>) SPM_SIZES ->Name("dspmv");
BENCHMARK(BM_hpmv<float>)  SPM_SIZES ->Name("chpmv");
BENCHMARK(BM_hpmv<double>) SPM_SIZES ->Name("zhpmv");

#undef SPM_SIZES

// Band: {n, k}
#define BAND_SIZES \
    ->Args({256, 4})->Args({256, 32})->Args({1024, 4})->Args({1024, 32}) \
    ->Args({4096, 4})->Args({4096, 32})->Args({4096, 128})

BENCHMARK(BM_sbmv<float>)  BAND_SIZES ->Name("ssbmv");
BENCHMARK(BM_sbmv<double>) BAND_SIZES ->Name("dsbmv");
BENCHMARK(BM_tbmv<float>)  BAND_SIZES ->Name("stbmv");
BENCHMARK(BM_tbmv<double>) BAND_SIZES ->Name("dtbmv");
BENCHMARK(BM_hbmv<float>)  BAND_SIZES ->Name("chbmv");
BENCHMARK(BM_hbmv<double>) BAND_SIZES ->Name("zhbmv");

#undef BAND_SIZES

// --- Square sizes for Level 3 ---
#define GEMM_SIZES ->Arg(32)->Arg(64)->Arg(128)->Arg(256)->Arg(512)->Arg(1024)

BENCHMARK(BM_gemm<float>)  GEMM_SIZES ->Name("sgemm");
BENCHMARK(BM_gemm<double>) GEMM_SIZES ->Name("dgemm");

#undef GEMM_SIZES

// clang-format on
