// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2014 Benoit Steiner <benoit.steiner.goog@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#define EIGEN_TEST_NO_LONGDOUBLE
#define EIGEN_TEST_NO_COMPLEX

#define EIGEN_DEFAULT_DENSE_INDEX_TYPE int
#define EIGEN_USE_GPU

#include "main.h"
#include <Eigen/Tensor>

void test_gpu_random_uniform() {
  Tensor<float, 2> out(72, 97);
  out.setZero();

  std::size_t out_bytes = out.size() * sizeof(float);

  float* d_out;
  gpuMalloc((void**)(&d_out), out_bytes);

  Eigen::GpuStreamDevice stream;
  Eigen::GpuDevice gpu_device(&stream);

  Eigen::TensorMap<Eigen::Tensor<float, 2> > gpu_out(d_out, 72, 97);

  gpu_out.device(gpu_device) = gpu_out.random();

  assert(gpuMemcpyAsync(out.data(), d_out, out_bytes, gpuMemcpyDeviceToHost, gpu_device.stream()) == gpuSuccess);
  assert(gpuStreamSynchronize(gpu_device.stream()) == gpuSuccess);

  // For now we just check this code doesn't crash.
  // TODO: come up with a valid test of randomness
}

void test_gpu_random_normal() {
  Tensor<float, 2> out(72, 97);
  out.setZero();

  std::size_t out_bytes = out.size() * sizeof(float);

  float* d_out;
  gpuMalloc((void**)(&d_out), out_bytes);

  Eigen::GpuStreamDevice stream;
  Eigen::GpuDevice gpu_device(&stream);

  Eigen::TensorMap<Eigen::Tensor<float, 2> > gpu_out(d_out, 72, 97);

  Eigen::internal::NormalRandomGenerator<float> gen(true);
  gpu_out.device(gpu_device) = gpu_out.random(gen);

  assert(gpuMemcpyAsync(out.data(), d_out, out_bytes, gpuMemcpyDeviceToHost, gpu_device.stream()) == gpuSuccess);
  assert(gpuStreamSynchronize(gpu_device.stream()) == gpuSuccess);
}

template <typename Scalar>
void test_gpu_random_uniform_range(int rows, int cols) {
  Tensor<Scalar, 2> out(rows, cols);
  out.setZero();

  std::size_t out_bytes = out.size() * sizeof(Scalar);

  Scalar* d_out;
  gpuMalloc((void**)(&d_out), out_bytes);

  Eigen::GpuStreamDevice stream;
  Eigen::GpuDevice gpu_device(&stream);

  Eigen::TensorMap<Eigen::Tensor<Scalar, 2> > gpu_out(d_out, rows, cols);

  gpu_out.device(gpu_device) = gpu_out.random();

  assert(gpuMemcpyAsync(out.data(), d_out, out_bytes, gpuMemcpyDeviceToHost, gpu_device.stream()) == gpuSuccess);
  assert(gpuStreamSynchronize(gpu_device.stream()) == gpuSuccess);

  // All uniform draws must lie in [0, 1).
  int num_out_of_range = 0;
  for (int i = 0; i < out.size(); ++i) {
    if (!(out.data()[i] >= Scalar(0.0f) && out.data()[i] < Scalar(1.0f))) ++num_out_of_range;
  }
  VERIFY_IS_EQUAL(num_out_of_range, 0);
}

template <typename Scalar>
void test_gpu_random_normal_all_finite(int rows, int cols) {
  Tensor<Scalar, 2> out(rows, cols);
  out.setZero();

  std::size_t out_bytes = out.size() * sizeof(Scalar);

  Scalar* d_out;
  gpuMalloc((void**)(&d_out), out_bytes);

  Eigen::GpuStreamDevice stream;
  Eigen::GpuDevice gpu_device(&stream);

  Eigen::TensorMap<Eigen::Tensor<Scalar, 2> > gpu_out(d_out, rows, cols);

  Eigen::internal::NormalRandomGenerator<Scalar> gen(true);
  gpu_out.device(gpu_device) = gpu_out.random(gen);

  assert(gpuMemcpyAsync(out.data(), d_out, out_bytes, gpuMemcpyDeviceToHost, gpu_device.stream()) == gpuSuccess);
  assert(gpuStreamSynchronize(gpu_device.stream()) == gpuSuccess);

  // Regression test for 16-bit types: the deviate must be computed in float,
  // otherwise log(0) in the rejection algorithm emits NaN/Inf.
  int num_not_finite = 0;
  for (int i = 0; i < out.size(); ++i) {
    if (!(numext::isfinite)(out.data()[i])) ++num_not_finite;
  }
  VERIFY_IS_EQUAL(num_not_finite, 0);
}

// Fills n elements on the device through `gen` and returns them. MapOptions selects the executor path: Aligned
// takes the packet path, Unaligned the scalar one.
template <typename Scalar, int MapOptions, typename Generator>
static Tensor<Scalar, 1> device_random_fill(int n, const Generator& gen) {
  const std::size_t bytes = n * sizeof(Scalar);
  Eigen::GpuStreamDevice stream;
  Eigen::GpuDevice gpu_device(&stream);
  Scalar* d_out = static_cast<Scalar*>(gpu_device.allocate(bytes));
  Eigen::TensorMap<Eigen::Tensor<Scalar, 1>, MapOptions> gpu_out(d_out, n);
  gpu_out.device(gpu_device) = gpu_out.random(gen);
  Tensor<Scalar, 1> out(n);
  gpu_device.memcpyDeviceToHost(out.data(), d_out, bytes);
  gpu_device.synchronize();
  gpu_device.deallocate(d_out);
  return out;
}

// The generator travels into the kernel by value, so every thread starts from the same state, yet each element has
// to come from its own stream. An unaligned map takes the scalar path and an aligned one the packet path; both have
// to produce a 64-bucket histogram within 5% of flat (6.4 standard deviations at n = 2^20) with almost every
// element different from element 0 (a correct 23-bit uniform repeats a given value about n / 2^23 times).
template <int MapOptions>
static void test_gpu_random_uniform_distribution() {
  const int n = 1 << 20;
  const int num_buckets = 64;
  const int expected_per_bucket = n / num_buckets;
  const int bucket_tolerance = expected_per_bucket / 20;
  const int max_equal_to_first = n / 100;

  const Tensor<float, 1> out =
      device_random_fill<float, MapOptions>(n, Eigen::internal::UniformRandomGenerator<float>());

  Eigen::Array<int, num_buckets, 1> histogram = Eigen::Array<int, num_buckets, 1>::Zero();
  int num_out_of_range = 0;
  int num_equal_to_first = 0;
  for (int i = 0; i < n; ++i) {
    const float x = out(i);
    if (!(x >= 0.0f && x < 1.0f)) {
      ++num_out_of_range;
      continue;
    }
    ++histogram(static_cast<int>(x * num_buckets));
    if (x == out(0)) ++num_equal_to_first;
  }
  VERIFY_IS_EQUAL(num_out_of_range, 0);
  VERIFY(num_equal_to_first < max_equal_to_first);
  VERIFY((histogram - expected_per_bucket).abs().maxCoeff() <= bucket_tolerance);
}

// Same two paths for the normal generator: sample mean, variance and the mass within one standard deviation,
// each allowed about ten standard errors at n = 2^20, plus the distinctness check.
template <int MapOptions>
static void test_gpu_random_normal_distribution() {
  const int n = 1 << 20;
  const double mean_tolerance = 0.01;
  const double variance_tolerance = 0.02;
  const double one_sigma_mass = 0.682689;
  const double one_sigma_tolerance = 0.005;
  const int max_equal_to_first = n / 100;

  const Tensor<float, 1> out =
      device_random_fill<float, MapOptions>(n, Eigen::internal::NormalRandomGenerator<float>());

  double sum = 0.0;
  double sum_of_squares = 0.0;
  int num_within_one_sigma = 0;
  int num_equal_to_first = 0;
  for (int i = 0; i < n; ++i) {
    const double x = out(i);
    sum += x;
    sum_of_squares += x * x;
    if (std::abs(x) < 1.0) ++num_within_one_sigma;
    if (out(i) == out(0)) ++num_equal_to_first;
  }
  const double mean = sum / n;
  const double variance = sum_of_squares / n - mean * mean;
  VERIFY(std::abs(mean) < mean_tolerance);
  VERIFY(std::abs(variance - 1.0) < variance_tolerance);
  VERIFY(std::abs(double(num_within_one_sigma) / n - one_sigma_mass) < one_sigma_tolerance);
  VERIFY(num_equal_to_first < max_equal_to_first);
}

// A seeded uniform generator is a pure function of (seed, index): the device fill reproduces the host fill bit for
// bit whatever path either side takes (scalar or packet, at any packet width). The odd length leaves a scalar tail
// behind the packet loop.
template <typename Scalar, int MapOptions>
static void test_gpu_random_matches_host() {
  const int n = (1 << 16) + 3;
  const uint64_t seed = 0x9e3779b97f4a7c15ULL;

  Eigen::internal::UniformRandomGenerator<Scalar> gen(seed);
  Tensor<Scalar, 1> host(n);
  host = host.random(gen);

  const Tensor<Scalar, 1> out = device_random_fill<Scalar, MapOptions>(n, gen);

  for (int i = 0; i < n; ++i) {
    VERIFY_IS_EQUAL(out(i), host(i));
  }
}

static void test_complex() {
  Tensor<std::complex<float>, 1> vec(6);
  vec.setRandom();

  // Fixme: we should check that the generated numbers follow a uniform
  // distribution instead.
  for (int i = 1; i < 6; ++i) {
    VERIFY_IS_NOT_EQUAL(vec(i), vec(i - 1));
  }
}

EIGEN_DECLARE_TEST(tensor_random_gpu) {
  CALL_SUBTEST(test_gpu_random_uniform());
  CALL_SUBTEST(test_gpu_random_normal());
  CALL_SUBTEST(test_gpu_random_uniform_range<Eigen::half>(256, 256));
  CALL_SUBTEST(test_gpu_random_uniform_range<Eigen::bfloat16>(256, 256));
  CALL_SUBTEST(test_gpu_random_normal_all_finite<Eigen::half>(1024, 2048));
  CALL_SUBTEST(test_gpu_random_normal_all_finite<Eigen::bfloat16>(512, 512));
  CALL_SUBTEST(test_gpu_random_uniform_distribution<Eigen::Unaligned>());
  CALL_SUBTEST(test_gpu_random_uniform_distribution<Eigen::Aligned>());
  CALL_SUBTEST(test_gpu_random_normal_distribution<Eigen::Unaligned>());
  CALL_SUBTEST(test_gpu_random_normal_distribution<Eigen::Aligned>());
  CALL_SUBTEST((test_gpu_random_matches_host<float, Eigen::Unaligned>()));
  CALL_SUBTEST((test_gpu_random_matches_host<float, Eigen::Aligned>()));
  CALL_SUBTEST((test_gpu_random_matches_host<double, Eigen::Unaligned>()));
  CALL_SUBTEST((test_gpu_random_matches_host<double, Eigen::Aligned>()));
  CALL_SUBTEST(test_complex());
}
