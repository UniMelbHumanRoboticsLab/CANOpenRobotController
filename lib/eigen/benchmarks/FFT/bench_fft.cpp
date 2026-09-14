// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>
#include <contrib/Eigen/FFT>
#include <complex>
#include <vector>

using namespace Eigen;

template <typename T, bool Forward, bool Unscaled = false, bool HalfSpec = false>
static void BM_FFT(benchmark::State& state) {
  typedef typename NumTraits<T>::Real ScalarType;
  typedef std::complex<ScalarType> Complex;
  int nfft = state.range(0);
  std::vector<T> inbuf(nfft);
  std::vector<Complex> outbuf(nfft);
  FFT<ScalarType> fft;
  if (Unscaled) fft.SetFlag(fft.Unscaled);
  if (HalfSpec) fft.SetFlag(fft.HalfSpectrum);
  std::fill(inbuf.begin(), inbuf.end(), T(0));
  fft.fwd(outbuf, inbuf);

  for (auto _ : state) {
    if (Forward)
      fft.fwd(outbuf, inbuf);
    else
      fft.inv(inbuf, outbuf);
    benchmark::DoNotOptimize(outbuf.data());
    benchmark::DoNotOptimize(inbuf.data());
  }
  double mflops_per_iter = 5.0 * nfft * std::log2(static_cast<double>(nfft));
  if (!NumTraits<T>::IsComplex) mflops_per_iter /= 2;
  state.counters["MFLOPS"] =
      benchmark::Counter(mflops_per_iter, benchmark::Counter::kIsIterationInvariantRate, benchmark::Counter::kIs1000);
}

// clang-format off
#define FFT_SIZES \
    ->Arg(64)->Arg(128)->Arg(256)->Arg(512)->Arg(1024)->Arg(2048) \
    ->Arg(4096)->Arg(8192)->Arg(16384)->Arg(65536) \
    ->Arg(1000)->Arg(5000)
BENCHMARK(BM_FFT<std::complex<float>, true>) FFT_SIZES;
BENCHMARK(BM_FFT<std::complex<float>, false>) FFT_SIZES;
BENCHMARK(BM_FFT<float, true>) FFT_SIZES;
BENCHMARK(BM_FFT<float, false>) FFT_SIZES;
BENCHMARK(BM_FFT<std::complex<double>, true>) FFT_SIZES;
BENCHMARK(BM_FFT<std::complex<double>, false>) FFT_SIZES;
BENCHMARK(BM_FFT<double, true>) FFT_SIZES;
BENCHMARK(BM_FFT<double, false>) FFT_SIZES;
#undef FFT_SIZES
// clang-format on
