// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#ifdef EIGEN_DEFAULT_DENSE_INDEX_TYPE
#undef EIGEN_DEFAULT_DENSE_INDEX_TYPE
#endif
#define EIGEN_DEFAULT_DENSE_INDEX_TYPE int
#include "main.h"

#include <contrib/Eigen/StructuredMatrices>

using namespace Eigen;

// Roughly 2.2 million factors near the top of the double range exceed a
// 32-bit Index exponent even though the determinant has the unambiguous result
// +Inf. The internal exponent type must not wrap.
void test_vandermonde_determinant_int_index() {
  typedef Matrix<double, Dynamic, 1> Vec;
  const Index n = 2100;
  const double low = -9e307, high = 9e307;
  Vec x(n);
  for (Index i = 0; i < n; ++i) {
    const double t = double(i) / double(n - 1);
    x[i] = (1.0 - t) * low + t * high;
  }

  const double det = Vandermonde<double>(x).determinant();
  VERIFY((numext::isinf)(det));
  VERIFY(det > 0.0);
}

// The scaled-Horner frame can likewise exceed a 32-bit Index exponent. A
// degree-2,099,999 monomial at DBL_MAX must overflow to +Inf rather than wrap
// its accumulated exponent and underflow to zero.
void test_vandermonde_scaled_horner_int_index() {
  typedef Matrix<double, Dynamic, 1> Vec;
  const Index n = 2100000;
  Vec x(1), a = Vec::Zero(n);
  x[0] = (std::numeric_limits<double>::max)();
  a[n - 1] = 1.0;

  Vec y = Vandermonde<double>(x, n) * a;
  VERIFY((numext::isinf)(y[0]));
  VERIFY(y[0] > 0.0);
}

EIGEN_DECLARE_TEST(structured_vandermonde_int_index) {
  STATIC_CHECK(sizeof(Index) == sizeof(int));
  CALL_SUBTEST_1(test_vandermonde_determinant_int_index());
  CALL_SUBTEST_2(test_vandermonde_scaled_horner_int_index());
}
