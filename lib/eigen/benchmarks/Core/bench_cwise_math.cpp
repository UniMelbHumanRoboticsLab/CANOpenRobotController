// Benchmarks for vectorized coefficient-wise math functions.
//
// Each function is benchmarked on ArrayXf/ArrayXd with inputs chosen to
// stay in the valid domain and avoid NaN/Inf.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>
#include <contrib/Eigen/SpecialFunctions>

using namespace Eigen;

// Macro to define a benchmark for a unary array operation.
// NAME:     benchmark function suffix (e.g. Exp)
// EXPR:     expression applied to the array (e.g. a.exp())
// LO, HI:  input range [LO, HI] mapped from the default Random() range [-1,1]
#define BENCH_CWISE_UNARY(NAME, EXPR, LO, HI)                                                    \
  template <typename Scalar>                                                                     \
  static void BM_##NAME(benchmark::State& state) {                                               \
    const Index n = state.range(0);                                                              \
    using Arr = Array<Scalar, Dynamic, 1>;                                                       \
    /* Map Random [-1,1] to [LO, HI] */                                                          \
    Arr a = (Arr::Random(n) + Scalar(1)) * Scalar((double(HI) - double(LO)) / 2.0) + Scalar(LO); \
    Arr b(n);                                                                                    \
    for (auto _ : state) {                                                                       \
      b = EXPR;                                                                                  \
      benchmark::DoNotOptimize(b.data());                                                        \
    }                                                                                            \
    state.SetBytesProcessed(state.iterations() * n * sizeof(Scalar) * 2);                        \
  }

// Transcendental functions
BENCH_CWISE_UNARY(Exp, a.exp(), -10, 10)
BENCH_CWISE_UNARY(Log, a.log(), 0.01, 100)
BENCH_CWISE_UNARY(Log1p, a.log1p(), -0.5, 100)
BENCH_CWISE_UNARY(Sqrt, a.sqrt(), 0, 100)
BENCH_CWISE_UNARY(Rsqrt, a.rsqrt(), 0.01, 100)

BENCH_CWISE_UNARY(Log2, a.log2(), 0.01, 100)
BENCH_CWISE_UNARY(Exp2, a.exp2(), -10, 10)
BENCH_CWISE_UNARY(Expm1, a.expm1(), -2, 2)
BENCH_CWISE_UNARY(Cbrt, a.cbrt(), -100, 100)

// Trigonometric functions
BENCH_CWISE_UNARY(Sin, a.sin(), -3.14, 3.14)
BENCH_CWISE_UNARY(Cos, a.cos(), -3.14, 3.14)
BENCH_CWISE_UNARY(Tan, a.tan(), -1.5, 1.5)
BENCH_CWISE_UNARY(Asin, a.asin(), -0.99, 0.99)
BENCH_CWISE_UNARY(Acos, a.acos(), -0.99, 0.99)
BENCH_CWISE_UNARY(Atan, a.atan(), -10, 10)

// Hyperbolic / special
BENCH_CWISE_UNARY(Sinh, a.sinh(), -5, 5)
BENCH_CWISE_UNARY(Cosh, a.cosh(), -5, 5)
BENCH_CWISE_UNARY(Tanh, a.tanh(), -5, 5)
BENCH_CWISE_UNARY(Asinh, a.asinh(), -5, 5)
BENCH_CWISE_UNARY(Acosh, a.acosh(), 1.01, 10)
BENCH_CWISE_UNARY(Atanh, a.atanh(), -0.99, 0.99)
BENCH_CWISE_UNARY(Log10, a.log10(), 0.01, 100)

// mode: 0 = ordinary inputs, 1 = subnormal inputs, 2 = one subnormal per 64 coefficients.
template <typename Scalar>
static void BM_Erf(benchmark::State& state) {
  using Bits = typename numext::get_integer_by_size<sizeof(Scalar)>::unsigned_type;
  constexpr Bits sign = Bits(1) << (8 * sizeof(Scalar) - 1);
  constexpr Bits minNormal = Bits(1) << (std::numeric_limits<Scalar>::digits - 1);
  const Index n = state.range(0);
  const int mode = int(state.range(1));
  Array<Scalar, Dynamic, 1> a(n), b(n);
  for (Index i = 0; i < n; ++i) {
    a(i) = Scalar(double(i % 257 - 128) / 32.0);
    if (mode == 1 || (mode == 2 && i % 64 == 0)) {
      const Bits magnitude = Bits(1 + (Bits(i) * 37) % (minNormal - 1));
      a(i) = numext::bit_cast<Scalar>(Bits(magnitude | (i % 2 ? sign : Bits(0))));
    }
  }
  for (auto _ : state) {
    benchmark::DoNotOptimize(a.data());
    b = Eigen::erf(a);
    benchmark::DoNotOptimize(b.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * n * sizeof(Scalar) * 2);
}

// Simple operations (should be very fast / memory-bound)
BENCH_CWISE_UNARY(Abs, a.abs(), -100, 100)
BENCH_CWISE_UNARY(Square, a.square(), -100, 100)
BENCH_CWISE_UNARY(Cube, a.cube(), -10, 10)
BENCH_CWISE_UNARY(Ceil, a.ceil(), -100, 100)
BENCH_CWISE_UNARY(Floor, a.floor(), -100, 100)
BENCH_CWISE_UNARY(Round, a.round(), -100, 100)
BENCH_CWISE_UNARY(Rint, a.rint(), -100, 100)
BENCH_CWISE_UNARY(Trunc, a.trunc(), -100, 100)

// Sigmoid: 1 / (1 + exp(-x)), common in ML.
BENCH_CWISE_UNARY(Sigmoid, Scalar(1) / (Scalar(1) + (-a).exp()), -10, 10)

// Macro for real binary benchmarks (e.g. coefficient-wise min/max).
#define BENCH_CWISE_BINARY(NAME, EXPR, LO, HI)                                                   \
  template <typename Scalar>                                                                     \
  static void BM_##NAME(benchmark::State& state) {                                               \
    const Index n = state.range(0);                                                              \
    using Arr = Array<Scalar, Dynamic, 1>;                                                       \
    Arr a = (Arr::Random(n) + Scalar(1)) * Scalar((double(HI) - double(LO)) / 2.0) + Scalar(LO); \
    Arr b = (Arr::Random(n) + Scalar(1)) * Scalar((double(HI) - double(LO)) / 2.0) + Scalar(LO); \
    Arr c(n);                                                                                    \
    for (auto _ : state) {                                                                       \
      c = EXPR;                                                                                  \
      benchmark::DoNotOptimize(c.data());                                                        \
    }                                                                                            \
    state.SetBytesProcessed(state.iterations() * n * sizeof(Scalar) * 3);                        \
  }

BENCH_CWISE_BINARY(Min, a.min(b), -100, 100)
BENCH_CWISE_BINARY(Max, a.max(b), -100, 100)

// Power: array^scalar
template <typename Scalar>
static void BM_Pow(benchmark::State& state) {
  const Index n = state.range(0);
  using Arr = Array<Scalar, Dynamic, 1>;
  Arr a = (Arr::Random(n) + Scalar(1)) * Scalar(50);  // [0, 100]
  Arr b(n);
  for (auto _ : state) {
    b = a.pow(Scalar(2.5));
    benchmark::DoNotOptimize(b.data());
  }
  state.SetBytesProcessed(state.iterations() * n * sizeof(Scalar) * 2);
}

// Macro for complex unary benchmarks. Random() already produces complex
// values with real & imag in [-1,1]; scale both parts to [LO, HI].
#define BENCH_CWISE_UNARY_COMPLEX(NAME, EXPR, LO, HI)                                \
  template <typename RealScalar>                                                     \
  static void BM_##NAME##_complex(benchmark::State& state) {                         \
    using Scalar = std::complex<RealScalar>;                                         \
    const Index n = state.range(0);                                                  \
    using Arr = Array<Scalar, Dynamic, 1>;                                           \
    Arr a = (Arr::Random(n) + Scalar(RealScalar(1), RealScalar(1))) *                \
                Scalar(RealScalar((double(HI) - double(LO)) / 2.0), RealScalar(0)) + \
            Scalar(RealScalar(LO), RealScalar(LO));                                  \
    Arr b(n);                                                                        \
    for (auto _ : state) {                                                           \
      b = EXPR;                                                                      \
      benchmark::DoNotOptimize(b.data());                                            \
    }                                                                                \
    state.SetBytesProcessed(state.iterations() * n * Index(sizeof(Scalar)) * 2);     \
  }

// Macro for complex binary benchmarks (e.g. multiply, divide).
#define BENCH_CWISE_BINARY_COMPLEX(NAME, EXPR, LO, HI)                               \
  template <typename RealScalar>                                                     \
  static void BM_##NAME##_complex(benchmark::State& state) {                         \
    using Scalar = std::complex<RealScalar>;                                         \
    const Index n = state.range(0);                                                  \
    using Arr = Array<Scalar, Dynamic, 1>;                                           \
    Arr a = (Arr::Random(n) + Scalar(RealScalar(1), RealScalar(1))) *                \
                Scalar(RealScalar((double(HI) - double(LO)) / 2.0), RealScalar(0)) + \
            Scalar(RealScalar(LO), RealScalar(LO));                                  \
    Arr b = (Arr::Random(n) + Scalar(RealScalar(1), RealScalar(1))) *                \
                Scalar(RealScalar((double(HI) - double(LO)) / 2.0), RealScalar(0)) + \
            Scalar(RealScalar(LO), RealScalar(LO));                                  \
    Arr c(n);                                                                        \
    for (auto _ : state) {                                                           \
      c = EXPR;                                                                      \
      benchmark::DoNotOptimize(c.data());                                            \
    }                                                                                \
    state.SetBytesProcessed(state.iterations() * n * Index(sizeof(Scalar)) * 3);     \
  }

// Complex unary (SIMD implementations in GenericPacketMathFunctions.h)
BENCH_CWISE_UNARY_COMPLEX(Exp, a.exp(), -5, 5)
BENCH_CWISE_UNARY_COMPLEX(Log, a.log(), 0.01, 100)
BENCH_CWISE_UNARY_COMPLEX(Sqrt, a.sqrt(), -100, 100)
BENCH_CWISE_UNARY_COMPLEX(Square, a.square(), -10, 10)

// Complex binary (pdiv_complex, pmul_complex)
BENCH_CWISE_BINARY_COMPLEX(Mul, a* b, -10, 10)
BENCH_CWISE_BINARY_COMPLEX(Div, a / b, -10, 10)

// clang-format off
#define CWISE_SIZES ->Arg(1024)->Arg(4096)->Arg(16384)->Arg(65536)->Arg(262144)->Arg(1048576)

// --- Register float ---
BENCHMARK(BM_Exp<float>) CWISE_SIZES ->Name("Exp_float");
BENCHMARK(BM_Log<float>) CWISE_SIZES ->Name("Log_float");
BENCHMARK(BM_Log1p<float>) CWISE_SIZES ->Name("Log1p_float");
BENCHMARK(BM_Log2<float>) CWISE_SIZES ->Name("Log2_float");
BENCHMARK(BM_Sqrt<float>) CWISE_SIZES ->Name("Sqrt_float");
BENCHMARK(BM_Rsqrt<float>) CWISE_SIZES ->Name("Rsqrt_float");
BENCHMARK(BM_Exp2<float>) CWISE_SIZES ->Name("Exp2_float");
BENCHMARK(BM_Expm1<float>) CWISE_SIZES ->Name("Expm1_float");
BENCHMARK(BM_Cbrt<float>) CWISE_SIZES ->Name("Cbrt_float");
BENCHMARK(BM_Sin<float>) CWISE_SIZES ->Name("Sin_float");
BENCHMARK(BM_Cos<float>) CWISE_SIZES ->Name("Cos_float");
BENCHMARK(BM_Tan<float>) CWISE_SIZES ->Name("Tan_float");
BENCHMARK(BM_Asin<float>) CWISE_SIZES ->Name("Asin_float");
BENCHMARK(BM_Acos<float>) CWISE_SIZES ->Name("Acos_float");
BENCHMARK(BM_Atan<float>) CWISE_SIZES ->Name("Atan_float");
BENCHMARK(BM_Sinh<float>) CWISE_SIZES ->Name("Sinh_float");
BENCHMARK(BM_Cosh<float>) CWISE_SIZES ->Name("Cosh_float");
BENCHMARK(BM_Tanh<float>) CWISE_SIZES ->Name("Tanh_float");
BENCHMARK(BM_Asinh<float>) CWISE_SIZES ->Name("Asinh_float");
BENCHMARK(BM_Acosh<float>) CWISE_SIZES ->Name("Acosh_float");
BENCHMARK(BM_Atanh<float>) CWISE_SIZES ->Name("Atanh_float");
BENCHMARK(BM_Log10<float>) CWISE_SIZES ->Name("Log10_float");
BENCHMARK(BM_Erf<float>)->ArgsProduct({{1024, 4096, 16384, 65536, 262144, 1048576}, {0, 1, 2}})
    ->ArgNames({"size", "mode"})->Name("Erf_float");
BENCHMARK(BM_Erf<bfloat16>)->ArgsProduct({{1024, 4096, 16384, 65536, 262144, 1048576}, {0, 1, 2}})
    ->ArgNames({"size", "mode"})->Name("Erf_bfloat16");
BENCHMARK(BM_Abs<float>) CWISE_SIZES ->Name("Abs_float");
BENCHMARK(BM_Square<float>) CWISE_SIZES ->Name("Square_float");
BENCHMARK(BM_Cube<float>) CWISE_SIZES ->Name("Cube_float");
BENCHMARK(BM_Ceil<float>) CWISE_SIZES ->Name("Ceil_float");
BENCHMARK(BM_Floor<float>) CWISE_SIZES ->Name("Floor_float");
BENCHMARK(BM_Round<float>) CWISE_SIZES ->Name("Round_float");
BENCHMARK(BM_Rint<float>) CWISE_SIZES ->Name("Rint_float");
BENCHMARK(BM_Trunc<float>) CWISE_SIZES ->Name("Trunc_float");
BENCHMARK(BM_Sigmoid<float>) CWISE_SIZES ->Name("Sigmoid_float");
BENCHMARK(BM_Pow<float>) CWISE_SIZES ->Name("Pow_float");
BENCHMARK(BM_Min<float>) CWISE_SIZES ->Name("Min_float");
BENCHMARK(BM_Max<float>) CWISE_SIZES ->Name("Max_float");

// --- Register double ---
BENCHMARK(BM_Exp<double>) CWISE_SIZES ->Name("Exp_double");
BENCHMARK(BM_Log<double>) CWISE_SIZES ->Name("Log_double");
BENCHMARK(BM_Log1p<double>) CWISE_SIZES ->Name("Log1p_double");
BENCHMARK(BM_Log2<double>) CWISE_SIZES ->Name("Log2_double");
BENCHMARK(BM_Sqrt<double>) CWISE_SIZES ->Name("Sqrt_double");
BENCHMARK(BM_Rsqrt<double>) CWISE_SIZES ->Name("Rsqrt_double");
BENCHMARK(BM_Exp2<double>) CWISE_SIZES ->Name("Exp2_double");
BENCHMARK(BM_Expm1<double>) CWISE_SIZES ->Name("Expm1_double");
BENCHMARK(BM_Cbrt<double>) CWISE_SIZES ->Name("Cbrt_double");
BENCHMARK(BM_Sin<double>) CWISE_SIZES ->Name("Sin_double");
BENCHMARK(BM_Cos<double>) CWISE_SIZES ->Name("Cos_double");
BENCHMARK(BM_Tan<double>) CWISE_SIZES ->Name("Tan_double");
BENCHMARK(BM_Asin<double>) CWISE_SIZES ->Name("Asin_double");
BENCHMARK(BM_Acos<double>) CWISE_SIZES ->Name("Acos_double");
BENCHMARK(BM_Atan<double>) CWISE_SIZES ->Name("Atan_double");
BENCHMARK(BM_Sinh<double>) CWISE_SIZES ->Name("Sinh_double");
BENCHMARK(BM_Cosh<double>) CWISE_SIZES ->Name("Cosh_double");
BENCHMARK(BM_Tanh<double>) CWISE_SIZES ->Name("Tanh_double");
BENCHMARK(BM_Asinh<double>) CWISE_SIZES ->Name("Asinh_double");
BENCHMARK(BM_Acosh<double>) CWISE_SIZES ->Name("Acosh_double");
BENCHMARK(BM_Atanh<double>) CWISE_SIZES ->Name("Atanh_double");
BENCHMARK(BM_Log10<double>) CWISE_SIZES ->Name("Log10_double");
BENCHMARK(BM_Erf<double>)->ArgsProduct({{1024, 4096, 16384, 65536, 262144, 1048576}, {0, 1, 2}})
    ->ArgNames({"size", "mode"})->Name("Erf_double");
BENCHMARK(BM_Abs<double>) CWISE_SIZES ->Name("Abs_double");
BENCHMARK(BM_Square<double>) CWISE_SIZES ->Name("Square_double");
BENCHMARK(BM_Cube<double>) CWISE_SIZES ->Name("Cube_double");
BENCHMARK(BM_Ceil<double>) CWISE_SIZES ->Name("Ceil_double");
BENCHMARK(BM_Floor<double>) CWISE_SIZES ->Name("Floor_double");
BENCHMARK(BM_Round<double>) CWISE_SIZES ->Name("Round_double");
BENCHMARK(BM_Rint<double>) CWISE_SIZES ->Name("Rint_double");
BENCHMARK(BM_Trunc<double>) CWISE_SIZES ->Name("Trunc_double");
BENCHMARK(BM_Sigmoid<double>) CWISE_SIZES ->Name("Sigmoid_double");
BENCHMARK(BM_Pow<double>) CWISE_SIZES ->Name("Pow_double");
BENCHMARK(BM_Min<double>) CWISE_SIZES ->Name("Min_double");
BENCHMARK(BM_Max<double>) CWISE_SIZES ->Name("Max_double");

// --- Register complex<float> ---
BENCHMARK(BM_Exp_complex<float>) CWISE_SIZES ->Name("Exp_complexf");
BENCHMARK(BM_Log_complex<float>) CWISE_SIZES ->Name("Log_complexf");
BENCHMARK(BM_Sqrt_complex<float>) CWISE_SIZES ->Name("Sqrt_complexf");
BENCHMARK(BM_Square_complex<float>) CWISE_SIZES ->Name("Square_complexf");
BENCHMARK(BM_Mul_complex<float>) CWISE_SIZES ->Name("Mul_complexf");
BENCHMARK(BM_Div_complex<float>) CWISE_SIZES ->Name("Div_complexf");

// --- Register complex<double> ---
BENCHMARK(BM_Exp_complex<double>) CWISE_SIZES ->Name("Exp_complexd");
BENCHMARK(BM_Log_complex<double>) CWISE_SIZES ->Name("Log_complexd");
BENCHMARK(BM_Sqrt_complex<double>) CWISE_SIZES ->Name("Sqrt_complexd");
BENCHMARK(BM_Square_complex<double>) CWISE_SIZES ->Name("Square_complexd");
BENCHMARK(BM_Mul_complex<double>) CWISE_SIZES ->Name("Mul_complexd");
BENCHMARK(BM_Div_complex<double>) CWISE_SIZES ->Name("Div_complexd");

#undef CWISE_SIZES
// clang-format on
