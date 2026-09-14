// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008 Benoit Jacob <jacob.benoit.1@gmail.com>
// Copyright (C) 2008 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "main.h"
#include <Eigen/LU>
#include <algorithm>
#include <limits>
#include <vector>

template <typename Scalar>
struct determinant_reference_scalar {
  typedef long double type;
};

template <typename RealScalar>
struct determinant_reference_scalar<std::complex<RealScalar> > {
  typedef std::complex<long double> type;
};

template <typename MatrixType>
typename determinant_reference_scalar<typename MatrixType::Scalar>::type brute_force_determinant(const MatrixType& m) {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename determinant_reference_scalar<Scalar>::type ReferenceScalar;
  const Index size = m.rows();
  std::vector<Index> permutation(size);
  for (Index i = 0; i < size; ++i) permutation[i] = i;

  ReferenceScalar result(0);
  do {
    int inversions = 0;
    for (Index i = 0; i < size; ++i)
      for (Index j = i + 1; j < size; ++j)
        if (permutation[i] > permutation[j]) ++inversions;

    ReferenceScalar term(1);
    for (Index i = 0; i < size; ++i) term *= ReferenceScalar(m(i, permutation[i]));
    result += inversions % 2 ? -term : term;
  } while (std::next_permutation(permutation.begin(), permutation.end()));

  return result;
}

template <typename MatrixType>
void verify_determinant_against_reference(const MatrixType& m) {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;

  Scalar expected = Scalar(brute_force_determinant(m));
  Scalar actual = m.determinant();
  const RealScalar max_abs = (std::max)(RealScalar(1), m.cwiseAbs().maxCoeff());
  RealScalar scale = max_abs;
  for (Index i = 1; i < m.rows(); ++i) scale *= max_abs;
  RealScalar tolerance = RealScalar(100000) * NumTraits<RealScalar>::epsilon() * scale;
  RealScalar error = numext::abs(actual - expected);
  if (error > tolerance) std::cerr << "determinant error " << error << " exceeds tolerance " << tolerance << std::endl;
  VERIFY(error <= tolerance);
}

template <typename MatrixType>
void determinant_lu_fallback_reference(const MatrixType& m) {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;
  const Index size = m.rows();

  MatrixType random(size, size);
  random.setRandom();
  verify_determinant_against_reference(random);

  MatrixType ill_conditioned = random;
  ill_conditioned.col(0) = ill_conditioned.col(1) + RealScalar(1e-8) * ill_conditioned.col(0);
  verify_determinant_against_reference(ill_conditioned);

  MatrixType scaled = RealScalar(1e6) * random;
  verify_determinant_against_reference(scaled);

  MatrixType tiny = MatrixType::Identity(size, size);
  tiny(0, 0) = Scalar((std::numeric_limits<RealScalar>::min)());
  verify_determinant_against_reference(tiny);

#if !EIGEN_ARCH_ARM
  MatrixType subnormal = MatrixType::Identity(size, size);
  subnormal(0, 0) = Scalar(std::numeric_limits<RealScalar>::denorm_min() * RealScalar(16));
  verify_determinant_against_reference(subnormal);
#endif
}

template <typename MatrixType>
void verify_non_finite_lu_determinant(MatrixType& m) {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;

  m.setIdentity();
  m(0, 0) = Scalar(std::numeric_limits<RealScalar>::quiet_NaN());
  VERIFY((numext::isnan)(m.determinant()));

  m.setIdentity();
  m(0, 0) = Scalar(std::numeric_limits<RealScalar>::infinity());
  VERIFY((numext::isinf)(m.determinant()));
}

void determinant_non_finite_lu_fallback() {
  Matrix<double, 5, 5> fixed;
  verify_non_finite_lu_determinant(fixed);

  MatrixXd dynamic(5, 5);
  verify_non_finite_lu_determinant(dynamic);

  std::vector<double> storage(25);
  Map<Matrix<double, Dynamic, Dynamic> > mapped(storage.data(), 5, 5);
  verify_non_finite_lu_determinant(mapped);
}

template <typename MatrixType>
void determinant(const MatrixType& m) {
  /* this test covers the following files:
     Determinant.h
  */
  Index size = m.rows();

  MatrixType m1(size, size), m2(size, size);
  m1.setRandom();
  m2.setRandom();
  typedef typename MatrixType::Scalar Scalar;
  Scalar x = internal::random<Scalar>();
  VERIFY_IS_APPROX(MatrixType::Identity(size, size).determinant(), Scalar(1));
  VERIFY_IS_APPROX((m1 * m2).eval().determinant(), m1.determinant() * m2.determinant());
  if (size == 1) return;
  Index i = internal::random<Index>(0, size - 1);
  Index j;
  do {
    j = internal::random<Index>(0, size - 1);
  } while (j == i);
  m2 = m1;
  m2.row(i).swap(m2.row(j));
  VERIFY_IS_APPROX(m2.determinant(), -m1.determinant());
  m2 = m1;
  m2.col(i).swap(m2.col(j));
  VERIFY_IS_APPROX(m2.determinant(), -m1.determinant());
  VERIFY_IS_APPROX(m2.determinant(), m2.transpose().determinant());
  VERIFY_IS_APPROX(numext::conj(m2.determinant()), m2.adjoint().determinant());
  m2 = m1;
  m2.row(i) += x * m2.row(j);
  VERIFY_IS_APPROX(m2.determinant(), m1.determinant());
  m2 = m1;
  m2.row(i) *= x;
  VERIFY_IS_APPROX(m2.determinant(), m1.determinant() * x);

  // check empty matrix
  VERIFY_IS_APPROX(m2.block(0, 0, 0, 0).determinant(), Scalar(1));
}

EIGEN_DECLARE_TEST(determinant) {
  for (int i = 0; i < g_repeat; i++) {
    int s = 0;
    CALL_SUBTEST_1(determinant(Matrix<float, 1, 1>()));
    CALL_SUBTEST_2(determinant(Matrix<double, 2, 2>()));
    CALL_SUBTEST_3(determinant(Matrix<double, 3, 3>()));
    CALL_SUBTEST_4(determinant(Matrix<double, 4, 4>()));
    CALL_SUBTEST_5(determinant(Matrix<double, 5, 5>()));
    CALL_SUBTEST_6(determinant(Matrix<std::complex<double>, 10, 10>()));
    CALL_SUBTEST_7(determinant_lu_fallback_reference(Matrix<double, 5, 5>()));
    CALL_SUBTEST_8(determinant_lu_fallback_reference(MatrixXcd(5, 5)));
    CALL_SUBTEST_9(determinant_non_finite_lu_fallback());
    s = internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 4);
    CALL_SUBTEST_10(determinant(MatrixXd(s, s)));
    TEST_SET_BUT_UNUSED_VARIABLE(s);
  }
}
