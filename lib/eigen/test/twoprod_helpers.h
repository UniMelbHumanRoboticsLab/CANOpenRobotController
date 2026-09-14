// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_TEST_TWOPROD_HELPERS_H
#define EIGEN_TEST_TWOPROD_HELPERS_H

// Where a hardware fma exists, GCC's C++ default -ffp-contract=fast may fuse fl(x*y) into the sums that consume it, and
// Dekker's splitting product is then no longer error-free. The double-word product exposes it:
// (1 + eps)(1 - eps) = 1 - eps^2, whose fl(x*y) = 1 feeds the renormalizing sums.
template <typename T>
void check_twoprod_contraction() {
  const T epsilon = NumTraits<T>::epsilon();
  // Runtime operands: constant folding would round every step before contraction could apply.
  volatile T one = T(1);
  for (int sign : {-1, 1}) {
    const T x = T(sign) * (one + epsilon), y = one - epsilon;
    T hi, lo;
    internal::twoprod(x, T(0), y, hi, lo);
    VERIFY_IS_EQUAL(hi, T(sign));
    VERIFY_IS_EQUAL(lo, -T(sign) * epsilon * epsilon);
  }
}

#endif  // EIGEN_TEST_TWOPROD_HELPERS_H
