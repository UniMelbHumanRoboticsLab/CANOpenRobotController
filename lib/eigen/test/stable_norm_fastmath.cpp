// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include "main.h"
#include "fp_control.h"

template <typename RealScalar>
EIGEN_DONT_INLINE RealScalar underflow_probe() {
  volatile RealScalar normal_min = (std::numeric_limits<RealScalar>::min)();
  volatile RealScalar one_half = RealScalar(0.5);
  return normal_min * one_half;
}

#if defined(EIGEN_HAS_ARM64_FP16_SCALAR_ARITHMETIC) && EIGEN_HAS_ARM64_FP16_SCALAR_ARITHMETIC
EIGEN_DONT_INLINE half fp16_underflow_probe(half normal_min, half one_half) { return normal_min * one_half; }

// Volatile dispatch prevents constant-propagation clones from bypassing the changed FPCR.
typedef half (*Fp16UnderflowProbe)(half, half);
static Fp16UnderflowProbe volatile fp16_underflow_probe_runtime = &fp16_underflow_probe;

void scoped_flush_to_zero_fp16() {
  const half normal_min = (std::numeric_limits<half>::min)();
  const half one_half(0.5f);
  const half underflow_before = fp16_underflow_probe_runtime(normal_min, one_half);
  bool flush_to_zero_supported = false;
  {
    Eigen::ScopedFlushToZero flush_to_zero;
    flush_to_zero_supported = flush_to_zero.isSupported();
    VERIFY(flush_to_zero_supported);
    VERIFY_IS_EQUAL(static_cast<float>(fp16_underflow_probe_runtime(normal_min, one_half)), 0.0f);
  }
  VERIFY_IS_EQUAL(static_cast<float>(fp16_underflow_probe_runtime(normal_min, one_half)),
                  static_cast<float>(underflow_before));
}
#endif

template <typename RealScalar>
void stable_normalize_fastmath() {
  typedef Matrix<RealScalar, 2, 1> Vector2;
  typedef std::complex<RealScalar> Complex;
  typedef Matrix<Complex, 1, 1> ComplexVector1;
  using std::sqrt;

  const RealScalar highest = (std::numeric_limits<RealScalar>::max)();
  const RealScalar smallest = (std::numeric_limits<RealScalar>::min)();
  const RealScalar inv_sqrt_two = RealScalar(1) / sqrt(RealScalar(2));
  const Vector2 expected = Vector2::Constant(inv_sqrt_two);

  Vector2 large = Vector2::Constant(highest);
  VERIFY_IS_APPROX(large.stableNormalized(), expected);
  large.stableNormalize();
  VERIFY_IS_APPROX(large, expected);

  Vector2 small = Vector2::Constant(smallest);
  VERIFY_IS_APPROX(small.stableNormalized(), expected);
  small.stableNormalize();
  VERIFY_IS_APPROX(small, expected);

  ComplexVector1 complex_large;
  complex_large(0) = Complex(highest, highest);
  const Complex expected_complex(inv_sqrt_two, inv_sqrt_two);
  VERIFY_IS_APPROX(complex_large.stableNormalized()(0), expected_complex);
  complex_large.stableNormalize();
  VERIFY_IS_APPROX(complex_large(0), expected_complex);
}

template <typename RealScalar>
void stable_norm_ftz_boundaries() {
  typedef Matrix<RealScalar, 2, 1> Vector2;
  typedef Matrix<RealScalar, Dynamic, 1> VectorX;
  using std::sqrt;

  const RealScalar normal_min = (std::numeric_limits<RealScalar>::min)();
  const RealScalar inv_sqrt_two = RealScalar(1) / sqrt(RealScalar(2));
  const Vector2 expected_normalized = Vector2::Constant(inv_sqrt_two);
  Vector2 reciprocal_boundary = Vector2::Constant(RealScalar(1) / normal_min);

  const Index block_size = 4096;
  const RealScalar large = RealScalar(128) * sqrt(normal_min);
  const RealScalar small = RealScalar(0.5) * sqrt(normal_min);
  VectorX mixed = VectorX::Constant(block_size, small);
  mixed(0) = large;
  const RealScalar expected_norm = large * sqrt(RealScalar(1) + RealScalar(block_size - 1) / RealScalar(65536));

  const RealScalar underflow_before = underflow_probe<RealScalar>();
  bool flush_to_zero_supported = false;
  {
    Eigen::ScopedFlushToZero flush_to_zero;
    flush_to_zero_supported = flush_to_zero.isSupported();
    if (flush_to_zero_supported) VERIFY_IS_EQUAL(underflow_probe<RealScalar>(), RealScalar(0));
    VERIFY_IS_APPROX(reciprocal_boundary.stableNormalized(), expected_normalized);
    reciprocal_boundary.stableNormalize();
    VERIFY_IS_APPROX(reciprocal_boundary, expected_normalized);
    VERIFY_IS_APPROX(mixed.stableNorm(), expected_norm);
  }
  if (flush_to_zero_supported) VERIFY_IS_EQUAL(underflow_probe<RealScalar>(), underflow_before);
}

template <typename RealScalar>
void stable_norm_multiblock_fastmath() {
  typedef Matrix<RealScalar, Dynamic, 1> VectorX;
  using std::sqrt;

  const Index block_size = 4096;
  const RealScalar scale = sqrt((std::numeric_limits<RealScalar>::max)());
  VectorX input = VectorX::Zero(2 * block_size);
  input(0) = scale;
  input.tail(block_size).setConstant(scale / RealScalar(128));
  const RealScalar expected = scale * sqrt(RealScalar(1.25));
  VERIFY_IS_APPROX(input.stableNorm(), expected);
}

EIGEN_DECLARE_TEST(stable_norm_fastmath) {
  CALL_SUBTEST_1(stable_normalize_fastmath<float>());
  CALL_SUBTEST_2(stable_normalize_fastmath<double>());
  CALL_SUBTEST_1(stable_norm_multiblock_fastmath<float>());
  CALL_SUBTEST_2(stable_norm_multiblock_fastmath<double>());
  CALL_SUBTEST_1(stable_norm_ftz_boundaries<float>());
  CALL_SUBTEST_2(stable_norm_ftz_boundaries<double>());
#if defined(EIGEN_HAS_ARM64_FP16_SCALAR_ARITHMETIC) && EIGEN_HAS_ARM64_FP16_SCALAR_ARITHMETIC
  CALL_SUBTEST_1(scoped_flush_to_zero_fp16());
#endif
}
