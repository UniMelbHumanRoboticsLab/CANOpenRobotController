// This file is triangularView of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2010 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#define TEST_CHECK_STATIC_ASSERTIONS
#include "main.h"

// This file tests the basic selfadjointView API,
// the related products and decompositions are tested in specific files.

template <typename MatrixType>
void selfadjoint(const MatrixType& m) {
  typedef typename MatrixType::Scalar Scalar;

  Index rows = m.rows();
  Index cols = m.cols();

  MatrixType m1 = MatrixType::Random(rows, cols), m2 = MatrixType::Random(rows, cols), m3(rows, cols), m4(rows, cols);

  m1.diagonal() = m1.diagonal().real().template cast<Scalar>();

  // check selfadjoint to dense
  m3 = m1.template selfadjointView<Upper>();
  VERIFY_IS_APPROX(MatrixType(m3.template triangularView<Upper>()), MatrixType(m1.template triangularView<Upper>()));
  VERIFY_IS_APPROX(m3, m3.adjoint());

  m3 = m1.template selfadjointView<Lower>();
  VERIFY_IS_APPROX(MatrixType(m3.template triangularView<Lower>()), MatrixType(m1.template triangularView<Lower>()));
  VERIFY_IS_APPROX(m3, m3.adjoint());

  m3 = m1.template selfadjointView<Upper>();
  m4 = m2;
  m4 += m1.template selfadjointView<Upper>();
  VERIFY_IS_APPROX(m4, m2 + m3);

  m3 = m1.template selfadjointView<Lower>();
  m4 = m2;
  m4 -= m1.template selfadjointView<Lower>();
  VERIFY_IS_APPROX(m4, m2 - m3);

  Scalar s = internal::random<Scalar>();

  m4 = s * m1.template selfadjointView<Upper>();
  VERIFY_IS_APPROX(m4, MatrixType((s * m1).template selfadjointView<Upper>()));
  m4 = m1.template selfadjointView<Upper>() * s;
  VERIFY_IS_APPROX(m4, MatrixType((m1 * s).template selfadjointView<Upper>()));

  m4 = s * m1.template selfadjointView<Lower>();
  VERIFY_IS_APPROX(m4, MatrixType((s * m1).template selfadjointView<Lower>()));
  m4 = m1.template selfadjointView<Lower>() * s;
  VERIFY_IS_APPROX(m4, MatrixType((m1 * s).template selfadjointView<Lower>()));

  // l1Norm: reads only the stored triangle but must agree with the L1 norm of
  // the materialized full matrix. Upper and Lower views of the same (stored-
  // full) self-adjoint matrix must return the same value; complex scalars
  // behave identically since |conj(x)| = |x|.
  typedef typename NumTraits<Scalar>::Real RealScalar;
  m3 = m1.template selfadjointView<Upper>();  // m3 is now fully self-adjoint
  RealScalar ref_l1 = m3.cwiseAbs().colwise().sum().maxCoeff();
  VERIFY_IS_APPROX(m3.template selfadjointView<Upper>().l1Norm(), ref_l1);
  VERIFY_IS_APPROX(m3.template selfadjointView<Lower>().l1Norm(), ref_l1);
  // Either triangle alone still gives the correct L1 norm even if the other
  // half is zero (the view conjure it back via symmetry).
  MatrixType upperOnly = MatrixType::Zero(rows, cols);
  upperOnly.template triangularView<Upper>() = m3;
  VERIFY_IS_APPROX(upperOnly.template selfadjointView<Upper>().l1Norm(), ref_l1);
  MatrixType lowerOnly = MatrixType::Zero(rows, cols);
  lowerOnly.template triangularView<Lower>() = m3;
  VERIFY_IS_APPROX(lowerOnly.template selfadjointView<Lower>().l1Norm(), ref_l1);
}

// l1Norm switches from a per-column to a streaming form at a small size and its column pass has
// packet tails, so sweep sizes around the switch and across packet boundaries: a mis-sized
// segment would otherwise hide between the random sizes above.
template <typename Scalar>
void selfadjoint_l1norm_sizes() {
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixType;
  typedef typename NumTraits<Scalar>::Real RealScalar;
  for (Index n : {Index(0), Index(1), Index(2), Index(15), Index(16), Index(17), Index(63), Index(64), Index(65),
                  Index(127), Index(128), Index(129)}) {
    MatrixType m = MatrixType::Random(n, n);
    MatrixType full = m.template selfadjointView<Lower>();
    RealScalar ref = n == 0 ? RealScalar(0) : full.cwiseAbs().colwise().sum().maxCoeff();
    VERIFY_IS_APPROX(m.template selfadjointView<Lower>().l1Norm(), ref);
    VERIFY_IS_APPROX(full.template selfadjointView<Upper>().l1Norm(), ref);
  }
}

// The vectorized complex path squares the parts, so entries beyond the square-root range of the
// scalar must come back through the scalar fallback: squares that overflow, and squares that
// land in the denormals and lose precision.
template <typename Scalar>
void selfadjoint_l1norm_range() {
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixType;
  typedef typename NumTraits<Scalar>::Real RealScalar;
  Index n = 70;
  MatrixType m = MatrixType::Random(n, n);
  RealScalar big = numext::sqrt(NumTraits<RealScalar>::highest()) * RealScalar(1e3);
  RealScalar small = numext::sqrt((std::numeric_limits<RealScalar>::min)()) * RealScalar(1e-3);
  for (RealScalar scale : {big, small}) {
    MatrixType ms = m * scale;
    MatrixType full = ms.template selfadjointView<Lower>();
    RealScalar ref = full.cwiseAbs().colwise().sum().maxCoeff();
    VERIFY_IS_APPROX(ms.template selfadjointView<Lower>().l1Norm(), ref);
    VERIFY_IS_APPROX(full.template selfadjointView<Upper>().l1Norm(), ref);
  }
}

// half and bfloat16 accumulate the norm in float, so only the final rounding to the scalar separates
// the result from a float reference, not the size of the matrix.
template <typename Scalar>
void selfadjoint_l1norm_lowprec() {
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixType;
  for (Index n : {Index(8), Index(64), Index(300)}) {
    MatrixType m = MatrixType::Random(n, n).template selfadjointView<Lower>();
    float ref = m.template cast<float>().cwiseAbs().colwise().sum().maxCoeff();
    float tol = 2 * float(NumTraits<Scalar>::epsilon()) * ref;
    VERIFY(numext::abs(float(m.template selfadjointView<Lower>().l1Norm()) - ref) <= tol);
    VERIFY(numext::abs(float(m.template selfadjointView<Upper>().l1Norm()) - ref) <= tol);
  }
}

// Narrow integers promote when added: the column totals must be materialized in the scalar type
// before they are compared (n=2 takes the per-column form, n=8 the column pass).
template <typename Scalar>
void selfadjoint_l1norm_integer() {
  for (Index n : {Index(2), Index(8)}) {
    typedef Matrix<Scalar, Dynamic, Dynamic> MatrixType;
    // Random() spans the whole range; keep the column sums representable.
    MatrixType m = (MatrixType::Random(n, n) / Scalar(NumTraits<Scalar>::highest() / 16)).eval();
    m = m.template selfadjointView<Lower>();
    Scalar ref = m.template cast<int>().cwiseAbs().colwise().sum().maxCoeff();
    VERIFY_IS_EQUAL(m.template selfadjointView<Lower>().l1Norm(), ref);
    VERIFY_IS_EQUAL(m.template selfadjointView<Upper>().l1Norm(), ref);
  }
}

void bug_159() {
  Matrix3d m = Matrix3d::Random().selfadjointView<Lower>();
  EIGEN_UNUSED_VARIABLE(m);
}

EIGEN_DECLARE_TEST(selfadjoint) {
  for (int i = 0; i < g_repeat; i++) {
    int s = internal::random<int>(1, EIGEN_TEST_MAX_SIZE);

    CALL_SUBTEST_1(selfadjoint(Matrix<float, 1, 1>()));
    CALL_SUBTEST_2(selfadjoint(Matrix<float, 2, 2>()));
    CALL_SUBTEST_3(selfadjoint(Matrix3cf()));
    CALL_SUBTEST_4(selfadjoint(MatrixXcd(s, s)));
    CALL_SUBTEST_5(selfadjoint(Matrix<float, Dynamic, Dynamic, RowMajor>(s, s)));
    CALL_SUBTEST_6(selfadjoint(Matrix<std::complex<float>, Dynamic, Dynamic, RowMajor>(s, s)));

    TEST_SET_BUT_UNUSED_VARIABLE(s);
  }

  CALL_SUBTEST_1(bug_159());
  CALL_SUBTEST_4(selfadjoint_l1norm_sizes<double>());
  CALL_SUBTEST_4(selfadjoint_l1norm_sizes<std::complex<double> >());
  CALL_SUBTEST_6(selfadjoint_l1norm_sizes<std::complex<float> >());
  CALL_SUBTEST_4(selfadjoint_l1norm_range<std::complex<double> >());
  CALL_SUBTEST_6(selfadjoint_l1norm_range<std::complex<float> >());
  CALL_SUBTEST_1(selfadjoint_l1norm_integer<short>());
  CALL_SUBTEST_1(selfadjoint_l1norm_integer<int>());
  CALL_SUBTEST_7(selfadjoint_l1norm_lowprec<half>());
  CALL_SUBTEST_7(selfadjoint_l1norm_lowprec<bfloat16>());
}
