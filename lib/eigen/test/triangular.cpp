// This file is triangularView of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2009 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#if defined(EIGEN_TEST_PART_100) || defined(EIGEN_TEST_PART_ALL)
#define EIGEN_NO_DEPRECATED_WARNING
#endif

#include "main.h"

template <typename ViewType, typename = void>
struct has_left_scalar_multiply : std::false_type {};

template <typename ViewType>
struct has_left_scalar_multiply<
    ViewType, internal::void_t<decltype(std::declval<typename ViewType::Scalar>() * std::declval<const ViewType&>())>>
    : std::true_type {};

template <typename ViewType, typename = void>
struct has_right_scalar_multiply : std::false_type {};

template <typename ViewType>
struct has_right_scalar_multiply<
    ViewType, internal::void_t<decltype(std::declval<const ViewType&>() * std::declval<typename ViewType::Scalar>())>>
    : std::true_type {};

template <typename ViewType, typename = void>
struct has_structured_sum : std::false_type {};

template <typename ViewType>
struct has_structured_sum<ViewType,
                          internal::void_t<decltype(std::declval<const ViewType&>() + std::declval<const ViewType&>())>>
    : std::true_type {};

template <unsigned int Mode, typename MatrixType>
void triangular_scalar_multiply(const MatrixType& m) {
  typedef typename MatrixType::Scalar Scalar;

  const Index rows = m.rows();
  const Index cols = m.cols();

  const Scalar s = internal::random<Scalar>();
  const MatrixType triangular = MatrixType::Random(rows, cols);

  VERIFY_IS_APPROX((s * triangular.template triangularView<Mode>()).toDenseMatrix(),
                   (s * triangular).template triangularView<Mode>().toDenseMatrix());
  VERIFY_IS_APPROX((triangular.template triangularView<Mode>() * s).toDenseMatrix(),
                   (triangular * s).template triangularView<Mode>().toDenseMatrix());
}

template <unsigned int Mode, typename MatrixType, bool IsSelfAdjointMode = Mode == Upper || Mode == Lower>
struct selfadjoint_structured_sum_impl {
  typedef typename MatrixType::Scalar Scalar;

  static void run(const Scalar&, const MatrixType&, const MatrixType&, MatrixType&, MatrixType&) {}
};

template <unsigned int Mode, typename MatrixType>
struct selfadjoint_structured_sum_impl<Mode, MatrixType, true> {
  typedef typename MatrixType::Scalar Scalar;

  static void run(const Scalar& s, const MatrixType& a, const MatrixType& b, MatrixType& result,
                  MatrixType& reference) {
    if (a.rows() != a.cols()) return;

    result.setRandom();
    result.template selfadjointView<Mode>() =
        s * a.template selfadjointView<Mode>() + b.template selfadjointView<Mode>();
    reference = (s * a + b).template selfadjointView<Mode>().toDenseMatrix();
    VERIFY_IS_APPROX(result, reference);

    result.setRandom();
    result.template selfadjointView<Mode>() =
        a.template selfadjointView<Mode>() - s * b.template selfadjointView<Mode>();
    reference = (a - s * b).template selfadjointView<Mode>().toDenseMatrix();
    VERIFY_IS_APPROX(result, reference);
  }
};

template <unsigned int Mode, typename MatrixType>
void triangular_structured_sum(const MatrixType& m) {
  typedef typename MatrixType::Scalar Scalar;

  const Index rows = m.rows();
  const Index cols = m.cols();

  const Scalar s = internal::random<Scalar>();
  const MatrixType a = MatrixType::Random(rows, cols);
  const MatrixType b = MatrixType::Random(rows, cols);

  MatrixType result = MatrixType::Random(rows, cols);
  MatrixType reference = result;

  result.template triangularView<Mode>() = s * a.template triangularView<Mode>() + b.template triangularView<Mode>();
  reference.template triangularView<Mode>() = s * a + b;
  VERIFY_IS_APPROX(result, reference);

  result.setRandom();
  reference = result;
  result.template triangularView<Mode>() = a.template triangularView<Mode>() - s * b.template triangularView<Mode>();
  reference.template triangularView<Mode>() = a - s * b;
  VERIFY_IS_APPROX(result, reference);

  selfadjoint_structured_sum_impl<Mode, MatrixType>::run(s, a, b, result, reference);
}

template <typename MatrixType>
void triangular_setters(const MatrixType& m) {
  const Index rows = m.rows();
  const Index cols = m.cols();

  MatrixType result = MatrixType::Random(rows, cols);
  MatrixType reference = result;

  result.template triangularView<Upper>().setIdentity();
  reference.template triangularView<Upper>() = MatrixType::Identity(rows, cols);
  VERIFY_IS_APPROX(result, reference);

  result.setRandom();
  reference = result;
  result.template triangularView<Lower>().setIdentity();
  reference.template triangularView<Lower>() = MatrixType::Identity(rows, cols);
  VERIFY_IS_APPROX(result, reference);

  result.setRandom();
  reference = result;
  result.template triangularView<Upper>().setRandom();
  VERIFY_IS_APPROX(result.template triangularView<StrictlyLower>().toDenseMatrix(),
                   reference.template triangularView<StrictlyLower>().toDenseMatrix());

  result.setRandom();
  reference = result;
  result.template triangularView<StrictlyLower>().setRandom();
  VERIFY_IS_APPROX(result.template triangularView<Upper>().toDenseMatrix(),
                   reference.template triangularView<Upper>().toDenseMatrix());
}

template <typename MatrixType>
void triangular_scalar_multiply_sfinae() {
  typedef decltype(std::declval<MatrixType&>().template triangularView<Upper>()) UpperView;
  typedef decltype(std::declval<MatrixType&>().template triangularView<Lower>()) LowerView;
  typedef decltype(std::declval<MatrixType&>().template triangularView<StrictlyLower>()) StrictlyLowerView;
  typedef decltype(std::declval<MatrixType&>().template triangularView<StrictlyUpper>()) StrictlyUpperView;
  typedef decltype(std::declval<MatrixType&>().template triangularView<UnitLower>()) UnitLowerView;

  STATIC_CHECK((has_left_scalar_multiply<UpperView>::value));
  STATIC_CHECK((has_right_scalar_multiply<UpperView>::value));
  STATIC_CHECK((has_left_scalar_multiply<LowerView>::value));
  STATIC_CHECK((has_right_scalar_multiply<LowerView>::value));
  STATIC_CHECK((has_left_scalar_multiply<StrictlyLowerView>::value));
  STATIC_CHECK((has_right_scalar_multiply<StrictlyLowerView>::value));
  STATIC_CHECK((has_left_scalar_multiply<StrictlyUpperView>::value));
  STATIC_CHECK((has_right_scalar_multiply<StrictlyUpperView>::value));
  STATIC_CHECK((!has_left_scalar_multiply<UnitLowerView>::value));
  STATIC_CHECK((!has_right_scalar_multiply<UnitLowerView>::value));

  STATIC_CHECK((has_structured_sum<UpperView>::value));
  STATIC_CHECK((has_structured_sum<LowerView>::value));
  STATIC_CHECK((has_structured_sum<StrictlyLowerView>::value));
  STATIC_CHECK((has_structured_sum<StrictlyUpperView>::value));
  STATIC_CHECK((!has_structured_sum<UnitLowerView>::value));
}

template <typename MatrixType>
void triangular_deprecated(const MatrixType& m) {
  Index rows = m.rows();
  Index cols = m.cols();
  MatrixType m1, m2, m3, m4;
  m1.setRandom(rows, cols);
  m2.setRandom(rows, cols);
  m3 = m1;
  m4 = m2;
  // deprecated method:
  m1.template triangularView<Eigen::Upper>().swap(m2);
  // use this method instead:
  m3.template triangularView<Eigen::Upper>().swap(m4.template triangularView<Eigen::Upper>());
  VERIFY_IS_APPROX(m1, m3);
  VERIFY_IS_APPROX(m2, m4);
  // deprecated method:
  m1.template triangularView<Eigen::Lower>().swap(m4);
  // use this method instead:
  m3.template triangularView<Eigen::Lower>().swap(m2.template triangularView<Eigen::Lower>());
  VERIFY_IS_APPROX(m1, m3);
  VERIFY_IS_APPROX(m2, m4);
}

template <typename MatrixType>
void triangular_square(const MatrixType& m) {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, MatrixType::RowsAtCompileTime, 1> VectorType;

  triangular_scalar_multiply_sfinae<MatrixType>();
  triangular_structured_sum<Upper>(m);
  triangular_structured_sum<Lower>(m);
  triangular_structured_sum<StrictlyUpper>(m);
  triangular_structured_sum<StrictlyLower>(m);
  triangular_setters(m);

  RealScalar largerEps = 10 * test_precision<RealScalar>();

  Index rows = m.rows();
  Index cols = m.cols();

  MatrixType m1 = MatrixType::Random(rows, cols), m2 = MatrixType::Random(rows, cols), m3(rows, cols), m4(rows, cols),
             r1(rows, cols), r2(rows, cols);
  VectorType v2 = VectorType::Random(rows);
  VectorType v3 = VectorType::Zero(rows);

  MatrixType m1up = m1.template triangularView<Upper>();
  MatrixType m2up = m2.template triangularView<Upper>();

  if (rows * cols > 1) {
    VERIFY(m1up.isUpperTriangular());
    VERIFY(m2up.transpose().isLowerTriangular());
    VERIFY(!m2.isLowerTriangular());
  }

  //   VERIFY_IS_APPROX(m1up.transpose() * m2, m1.upper().transpose().lower() * m2);

  // test overloaded operator+=
  r1.setZero();
  r2.setZero();
  r1.template triangularView<Upper>() += m1;
  r2 += m1up;
  VERIFY_IS_APPROX(r1, r2);

  // test overloaded operator=
  m1.setZero();
  m1.template triangularView<Upper>() = m2.transpose() + m2;
  m3 = m2.transpose() + m2;
  VERIFY_IS_APPROX(m3.template triangularView<Lower>().transpose().toDenseMatrix(), m1);

  // test overloaded operator=
  m1.setZero();
  m1.template triangularView<Lower>() = m2.transpose() + m2;
  VERIFY_IS_APPROX(m3.template triangularView<Lower>().toDenseMatrix(), m1);

  VERIFY_IS_APPROX(m3.template triangularView<Lower>().conjugate().toDenseMatrix(),
                   m3.conjugate().template triangularView<Lower>().toDenseMatrix());

  m1 = MatrixType::Random(rows, cols);
  for (int i = 0; i < rows; ++i)
    if (numext::abs2(m1(i, i)) < RealScalar(1e-1)) m1(i, i) = Scalar(1);

  Transpose<MatrixType> trm4(m4);
  // test back and forward substitution with a vector as the rhs
  m3 = m1.template triangularView<Upper>();
  v3 = m3.adjoint() * (m1.adjoint().template triangularView<Lower>().solve(v2));
  VERIFY(v2.isApprox(v3, largerEps));
  m3 = m1.template triangularView<Lower>();
  v3 = m3.transpose() * (m1.transpose().template triangularView<Upper>().solve(v2));
  VERIFY(v2.isApprox(v3, largerEps));
  m3 = m1.template triangularView<Upper>();
  v3 = m3 * (m1.template triangularView<Upper>().solve(v2));
  VERIFY(v2.isApprox(v3, largerEps));
  m3 = m1.template triangularView<Lower>();
  v3 = m3.conjugate() * (m1.conjugate().template triangularView<Lower>().solve(v2));
  VERIFY(v2.isApprox(v3, largerEps));

  // test back and forward substitution with a matrix as the rhs
  m3 = m1.template triangularView<Upper>();
  m4 = m3.adjoint() * (m1.adjoint().template triangularView<Lower>().solve(m2));
  VERIFY(m2.isApprox(m4, largerEps));
  m3 = m1.template triangularView<Lower>();
  m4 = m3.transpose() * (m1.transpose().template triangularView<Upper>().solve(m2));
  VERIFY(m2.isApprox(m4, largerEps));
  m3 = m1.template triangularView<Upper>();
  m4 = m3 * (m1.template triangularView<Upper>().solve(m2));
  VERIFY(m2.isApprox(m4, largerEps));
  m3 = m1.template triangularView<Lower>();
  m4 = m3.conjugate() * (m1.conjugate().template triangularView<Lower>().solve(m2));
  VERIFY(m2.isApprox(m4, largerEps));

  // check M * inv(L) using in place API
  m4 = m3;
  m1.transpose().template triangularView<Eigen::Upper>().solveInPlace(trm4);
  VERIFY_IS_APPROX(m4 * m1.template triangularView<Eigen::Lower>(), m3);

  // check M * inv(U) using in place API
  m3 = m1.template triangularView<Upper>();
  m4 = m3;
  m3.transpose().template triangularView<Eigen::Lower>().solveInPlace(trm4);
  VERIFY_IS_APPROX(m4 * m1.template triangularView<Eigen::Upper>(), m3);

  // check solve with unit diagonal
  m3 = m1.template triangularView<UnitUpper>();
  VERIFY(m2.isApprox(m3 * (m1.template triangularView<UnitUpper>().solve(m2)), largerEps));

  //   VERIFY((  m1.template triangularView<Upper>()
  //           * m2.template triangularView<Upper>()).isUpperTriangular());

  // test swap
  m1.setOnes();
  m2.setZero();
  m2.template triangularView<Upper>().swap(m1.template triangularView<Eigen::Upper>());
  m3.setZero();
  m3.template triangularView<Upper>().setOnes();
  VERIFY_IS_APPROX(m2, m3);

  m1.setRandom();
  m3 = m1.template triangularView<Upper>();
  Matrix<Scalar, MatrixType::ColsAtCompileTime, Dynamic> m5(cols, internal::random<int>(1, 20));
  m5.setRandom();
  Matrix<Scalar, Dynamic, MatrixType::RowsAtCompileTime> m6(internal::random<int>(1, 20), rows);
  m6.setRandom();
  VERIFY_IS_APPROX(m1.template triangularView<Upper>() * m5, m3 * m5);
  VERIFY_IS_APPROX(m6 * m1.template triangularView<Upper>(), m6 * m3);
  triangular_scalar_multiply<Upper>(m1);
  triangular_scalar_multiply<Lower>(m1);
  triangular_scalar_multiply<StrictlyUpper>(m1);
  triangular_scalar_multiply<StrictlyLower>(m1);

  m1up = m1.template triangularView<Upper>();
  VERIFY_IS_APPROX(m1.template selfadjointView<Upper>().template triangularView<Upper>().toDenseMatrix(), m1up);
  VERIFY_IS_APPROX(m1up.template selfadjointView<Upper>().template triangularView<Upper>().toDenseMatrix(), m1up);
  VERIFY_IS_APPROX(m1.template selfadjointView<Upper>().template triangularView<Lower>().toDenseMatrix(),
                   m1up.adjoint());
  VERIFY_IS_APPROX(m1up.template selfadjointView<Upper>().template triangularView<Lower>().toDenseMatrix(),
                   m1up.adjoint());

  VERIFY_IS_APPROX(m1.template selfadjointView<Upper>().diagonal(), m1.diagonal());

  m3.setRandom();
  const MatrixType& m3c(m3);
  VERIFY(is_same_type(m3c.template triangularView<Lower>(),
                      m3.template triangularView<Lower>().template conjugateIf<false>()));
  VERIFY(is_same_type(m3c.template triangularView<Lower>().conjugate(),
                      m3.template triangularView<Lower>().template conjugateIf<true>()));
  VERIFY_IS_APPROX(m3.template triangularView<Lower>().template conjugateIf<true>().toDenseMatrix(),
                   m3.conjugate().template triangularView<Lower>().toDenseMatrix());
  VERIFY_IS_APPROX(m3.template triangularView<Lower>().template conjugateIf<false>().toDenseMatrix(),
                   m3.template triangularView<Lower>().toDenseMatrix());

  VERIFY(is_same_type(m3c.template selfadjointView<Lower>(),
                      m3.template selfadjointView<Lower>().template conjugateIf<false>()));
  VERIFY(is_same_type(m3c.template selfadjointView<Lower>().conjugate(),
                      m3.template selfadjointView<Lower>().template conjugateIf<true>()));
  VERIFY_IS_APPROX(m3.template selfadjointView<Lower>().template conjugateIf<true>().toDenseMatrix(),
                   m3.conjugate().template selfadjointView<Lower>().toDenseMatrix());
  VERIFY_IS_APPROX(m3.template selfadjointView<Lower>().template conjugateIf<false>().toDenseMatrix(),
                   m3.template selfadjointView<Lower>().toDenseMatrix());
}

template <typename MatrixType>
void triangular_rect(const MatrixType& m) {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;
  enum { Rows = MatrixType::RowsAtCompileTime, Cols = MatrixType::ColsAtCompileTime };

  Index rows = m.rows();
  Index cols = m.cols();

  MatrixType m1 = MatrixType::Random(rows, cols), m2 = MatrixType::Random(rows, cols), m3(rows, cols), m4(rows, cols),
             r1(rows, cols), r2(rows, cols);

  MatrixType m1up = m1.template triangularView<Upper>();
  MatrixType m2up = m2.template triangularView<Upper>();

  if (rows > 1 && cols > 1) {
    VERIFY(m1up.isUpperTriangular());
    VERIFY(m2up.transpose().isLowerTriangular());
    VERIFY(!m2.isLowerTriangular());
  }

  // test overloaded operator+=
  r1.setZero();
  r2.setZero();
  r1.template triangularView<Upper>() += m1;
  r2 += m1up;
  VERIFY_IS_APPROX(r1, r2);

  // test overloaded operator=
  m1.setZero();
  m1.template triangularView<Upper>() = 3 * m2;
  m3 = 3 * m2;
  VERIFY_IS_APPROX(m3.template triangularView<Upper>().toDenseMatrix(), m1);

  m1.setZero();
  m1.template triangularView<Lower>() = 3 * m2;
  VERIFY_IS_APPROX(m3.template triangularView<Lower>().toDenseMatrix(), m1);

  m1.setZero();
  m1.template triangularView<StrictlyUpper>() = 3 * m2;
  VERIFY_IS_APPROX(m3.template triangularView<StrictlyUpper>().toDenseMatrix(), m1);

  m1.setZero();
  m1.template triangularView<StrictlyLower>() = 3 * m2;
  VERIFY_IS_APPROX(m3.template triangularView<StrictlyLower>().toDenseMatrix(), m1);
  triangular_scalar_multiply<Upper>(m1);
  triangular_scalar_multiply<Lower>(m1);
  triangular_scalar_multiply<StrictlyUpper>(m1);
  triangular_scalar_multiply<StrictlyLower>(m1);
  triangular_structured_sum<Upper>(m1);
  triangular_structured_sum<Lower>(m1);
  triangular_structured_sum<StrictlyUpper>(m1);
  triangular_structured_sum<StrictlyLower>(m1);
  triangular_setters(m1);
  m1.setRandom();
  m2 = m1.template triangularView<Upper>();
  VERIFY(m2.isUpperTriangular());
  VERIFY(!m2.isLowerTriangular());
  m2 = m1.template triangularView<StrictlyUpper>();
  VERIFY(m2.isUpperTriangular());
  VERIFY(m2.diagonal().isMuchSmallerThan(RealScalar(1)));
  m2 = m1.template triangularView<UnitUpper>();
  VERIFY(m2.isUpperTriangular());
  m2.diagonal().array() -= Scalar(1);
  VERIFY(m2.diagonal().isMuchSmallerThan(RealScalar(1)));
  m2 = m1.template triangularView<Lower>();
  VERIFY(m2.isLowerTriangular());
  VERIFY(!m2.isUpperTriangular());
  m2 = m1.template triangularView<StrictlyLower>();
  VERIFY(m2.isLowerTriangular());
  VERIFY(m2.diagonal().isMuchSmallerThan(RealScalar(1)));
  m2 = m1.template triangularView<UnitLower>();
  VERIFY(m2.isLowerTriangular());
  m2.diagonal().array() -= Scalar(1);
  VERIFY(m2.diagonal().isMuchSmallerThan(RealScalar(1)));
  // test swap
  m1.setOnes();
  m2.setZero();
  m2.template triangularView<Upper>().swap(m1.template triangularView<Eigen::Upper>());
  m3.setZero();
  m3.template triangularView<Upper>().setOnes();
  VERIFY_IS_APPROX(m2, m3);
}

// isLowerTriangular() and isUpperTriangular() must inspect every off-diagonal coefficient. In a wide matrix the
// columns past the diagonal block are strictly upper down to their last row, which an off-by-one row bound in
// isLowerTriangular() used to skip.
template <typename MatrixType>
void triangular_predicates(const MatrixType& m) {
  typedef typename MatrixType::Scalar Scalar;

  const Index rows = m.rows();
  const Index cols = m.cols();

  const MatrixType m1 = MatrixType::Random(rows, cols);
  const MatrixType lower = m1.template triangularView<Lower>();
  const MatrixType upper = m1.template triangularView<Upper>();
  VERIFY(lower.isLowerTriangular());
  VERIFY(upper.isUpperTriangular());

  MatrixType diag = MatrixType::Zero(rows, cols);
  diag.diagonal().setConstant(Scalar(1));
  VERIFY(diag.isLowerTriangular());
  VERIFY(diag.isUpperTriangular());

  // A single large coefficient off the diagonal must be caught by exactly one of the two predicates, wherever it sits.
  const Scalar offending = Scalar(100);
  for (Index j = 0; j < cols; ++j)
    for (Index i = 0; i < rows; ++i) {
      if (i == j) continue;
      MatrixType m2 = diag;
      m2(i, j) = offending;
      if (i < j) {
        VERIFY(!m2.isLowerTriangular());
        VERIFY(m2.isUpperTriangular());
      } else {
        VERIFY(!m2.isUpperTriangular());
        VERIFY(m2.isLowerTriangular());
      }
    }
}

// Test triangular solve and product at sizes that exercise GEBP blocking.
// The standard test caps at maxsize=20, which never triggers the blocked code paths
// in TriangularSolverMatrix.h (requires size >= 48 with EIGEN_DEBUG_SMALL_PRODUCT_BLOCKS).
template <int>
void triangular_at_blocking_boundaries() {
  typedef double Scalar;
  typedef Matrix<Scalar, Dynamic, Dynamic> Mat;
  typedef Matrix<Scalar, Dynamic, 1> Vec;

  const int sizes[] = {47, 48, 49, 64, 96, 128};
  for (int si = 0; si < 6; ++si) {
    int n = sizes[si];
    Mat m1 = Mat::Random(n, n);
    // Make well-conditioned: dominant diagonal
    for (int i = 0; i < n; ++i) m1(i, i) += Scalar(n);

    Vec v = Vec::Random(n);
    Mat rhs = Mat::Random(n, 5);

    // Upper triangular solve with vector
    Mat U = m1.triangularView<Upper>();
    Vec x = m1.triangularView<Upper>().solve(v);
    VERIFY_IS_APPROX(U * x, v);

    // Lower triangular solve with vector
    Mat L = m1.triangularView<Lower>();
    x = m1.triangularView<Lower>().solve(v);
    VERIFY_IS_APPROX(L * x, v);

    // Upper triangular solve with matrix rhs
    Mat X = m1.triangularView<Upper>().solve(rhs);
    VERIFY_IS_APPROX(U * X, rhs);

    // Lower triangular solve with matrix rhs
    X = m1.triangularView<Lower>().solve(rhs);
    VERIFY_IS_APPROX(L * X, rhs);

    // Triangular product
    Mat prod = m1.triangularView<Upper>() * rhs;
    VERIFY_IS_APPROX(prod, U * rhs);
    prod = rhs.transpose() * m1.triangularView<Upper>();
    VERIFY_IS_APPROX(prod, rhs.transpose() * U);
  }

  // Also test with float and RowMajor
  {
    typedef Matrix<float, Dynamic, Dynamic, RowMajor> RMat;
    typedef Matrix<float, Dynamic, 1> FVec;
    for (int si = 0; si < 6; ++si) {
      int n = sizes[si];
      RMat m1 = RMat::Random(n, n);
      for (int i = 0; i < n; ++i) m1(i, i) += float(n);

      FVec v = FVec::Random(n);
      RMat U = m1.triangularView<Upper>();
      FVec x = m1.triangularView<Upper>().solve(v);
      VERIFY_IS_APPROX(U * x, v);

      RMat L = m1.triangularView<Lower>();
      x = m1.triangularView<Lower>().solve(v);
      VERIFY_IS_APPROX(L * x, v);
    }
  }
}

void bug_159() {
  Matrix3d m = Matrix3d::Random().triangularView<Lower>();
  EIGEN_UNUSED_VARIABLE(m);
}

EIGEN_DECLARE_TEST(triangular) {
  int maxsize = (std::min)(EIGEN_TEST_MAX_SIZE, 20);
  for (int i = 0; i < g_repeat; i++) {
    int r = internal::random<int>(2, maxsize);
    TEST_SET_BUT_UNUSED_VARIABLE(r);
    int c = internal::random<int>(2, maxsize);
    TEST_SET_BUT_UNUSED_VARIABLE(c);

    CALL_SUBTEST_1(triangular_square(Matrix<float, 1, 1>()));
    CALL_SUBTEST_2(triangular_square(Matrix<float, 2, 2>()));
    CALL_SUBTEST_3(triangular_square(Matrix3d()));
    CALL_SUBTEST_4(triangular_square(Matrix<std::complex<float>, 8, 8>()));
    CALL_SUBTEST_5(triangular_square(MatrixXcd(r, r)));
    CALL_SUBTEST_6(triangular_square(Matrix<float, Dynamic, Dynamic, RowMajor>(r, r)));

    CALL_SUBTEST_7(triangular_rect(Matrix<float, 4, 5>()));
    CALL_SUBTEST_8(triangular_rect(Matrix<double, 6, 2>()));
    CALL_SUBTEST_9(triangular_rect(MatrixXcf(r, c)));
    CALL_SUBTEST_5(triangular_rect(MatrixXcd(r, c)));
    CALL_SUBTEST_6(triangular_rect(Matrix<float, Dynamic, Dynamic, RowMajor>(r, c)));

    CALL_SUBTEST_100(triangular_deprecated(Matrix<float, 5, 7>()));
    CALL_SUBTEST_100(triangular_deprecated(MatrixXd(r, c)));

    // isLowerTriangular()/isUpperTriangular() on wide, tall and square shapes, including the 1xN and Nx1 edges.
    CALL_SUBTEST_12(triangular_predicates(MatrixXd(2, 3)));
    CALL_SUBTEST_12(triangular_predicates(MatrixXd(3, 2)));
    CALL_SUBTEST_12(triangular_predicates(MatrixXd(3, 3)));
    CALL_SUBTEST_12(triangular_predicates(MatrixXd(1, 4)));
    CALL_SUBTEST_12(triangular_predicates(MatrixXd(4, 1)));
    CALL_SUBTEST_12(triangular_predicates(MatrixXd(1, 1)));
    CALL_SUBTEST_12(triangular_predicates(Matrix<float, 2, 5>()));
    CALL_SUBTEST_12(triangular_predicates(Matrix<double, 5, 2>()));
    CALL_SUBTEST_12(triangular_predicates(Matrix<std::complex<float>, 3, 7>()));
    CALL_SUBTEST_12(triangular_predicates(MatrixXcd(r, c)));
    CALL_SUBTEST_12(triangular_predicates(MatrixXcd(c, r)));
    CALL_SUBTEST_12(triangular_predicates(Matrix<float, Dynamic, Dynamic, RowMajor>(2, 5)));
    CALL_SUBTEST_12(triangular_predicates(Matrix<float, Dynamic, Dynamic, RowMajor>(r, c)));
  }

  CALL_SUBTEST_1(bug_159());

  // Triangular solve/product at blocking boundaries (deterministic, outside g_repeat).
  CALL_SUBTEST_11(triangular_at_blocking_boundaries<0>());
}
