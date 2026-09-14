// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2011 Benoit Jacob <jacob.benoit.1@gmail.com>
// Copyright (C) 2015 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#define TEST_ENABLE_TEMPORARY_TRACKING

#include "main.h"
#include <Eigen/Geometry>

template <typename Scalar>
struct broadcast_affine_op {
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar operator()(const Scalar& lhs, const Scalar& rhs) const {
    return lhs + Scalar(2) * rhs;
  }
};

template <typename ArrayType>
void vectorwiseop_array(const ArrayType& m) {
  typedef typename ArrayType::Scalar Scalar;
  typedef Array<Scalar, ArrayType::RowsAtCompileTime, 1> ColVectorType;
  typedef Array<Scalar, 1, ArrayType::ColsAtCompileTime> RowVectorType;

  Index rows = m.rows();
  Index cols = m.cols();
  Index r = internal::random<Index>(0, rows - 1), c = internal::random<Index>(0, cols - 1);

  ArrayType m1 = ArrayType::Random(rows, cols), m2(rows, cols), m3(rows, cols);

  ColVectorType colvec = ColVectorType::Random(rows);
  RowVectorType rowvec = RowVectorType::Random(cols);

  // test addition
  m2 = m1;
  m2.colwise() += colvec;
  VERIFY_IS_APPROX(m2, m1.colwise() + colvec);
  VERIFY_IS_APPROX(m2, colvec + m1.colwise());
  VERIFY_IS_APPROX(m2.col(c), m1.col(c) + colvec);

  m2 = m1;
  m2.rowwise() += rowvec;
  VERIFY_IS_APPROX(m2, m1.rowwise() + rowvec);
  VERIFY_IS_APPROX(m2, rowvec + m1.rowwise());
  VERIFY_IS_APPROX(m2.row(r), m1.row(r) + rowvec);

  // test subtraction
  m2 = m1;
  m2.colwise() -= colvec;
  VERIFY_IS_APPROX(m2, m1.colwise() - colvec);
  VERIFY_IS_APPROX(m2.col(c), m1.col(c) - colvec);

  m2 = m1;
  m2.rowwise() -= rowvec;
  VERIFY_IS_APPROX(m2, m1.rowwise() - rowvec);
  VERIFY_IS_APPROX(m2.row(r), m1.row(r) - rowvec);

  // test multiplication
  m2 = m1;
  m2.colwise() *= colvec;
  VERIFY_IS_APPROX(m2, m1.colwise() * colvec);
  VERIFY_IS_APPROX(m2, colvec * m1.colwise());
  VERIFY_IS_APPROX(m2.col(c), m1.col(c) * colvec);

  m2 = m1;
  m2.rowwise() *= rowvec;
  VERIFY_IS_APPROX(m2, m1.rowwise() * rowvec);
  VERIFY_IS_APPROX(m2, rowvec * m1.rowwise());
  VERIFY_IS_APPROX(m2.row(r), m1.row(r) * rowvec);

  // test quotient
  m2 = m1;
  m2.colwise() /= colvec;
  VERIFY_IS_APPROX(m2, m1.colwise() / colvec);
  VERIFY_IS_APPROX(m2.col(c), m1.col(c) / colvec);

  m2 = m1;
  m2.rowwise() /= rowvec;
  VERIFY_IS_APPROX(m2, m1.rowwise() / rowvec);
  VERIFY_IS_APPROX(m2.row(r), m1.row(r) / rowvec);

  // test assignment
  m2 = m1;
  m2.rowwise() = rowvec;
  for (Index k = 0; k < rows; ++k) {
    VERIFY_IS_APPROX(m2.row(k), rowvec);
  }
  m2 = m1;
  m2.colwise() = colvec;
  for (Index k = 0; k < cols; ++k) {
    VERIFY_IS_APPROX(m2.col(k), colvec);
  }

  // Block rhs: nested_eval must bind by reference, not materialize.
  {
    ArrayType other = ArrayType::Random(rows, cols);
    m2 = m1;
    m2.rowwise() += other.row(0);
    VERIFY_IS_APPROX(m2, m1.rowwise() + RowVectorType(other.row(0)));
    m2 = m1;
    m2.rowwise() -= other.row(0);
    VERIFY_IS_APPROX(m2, m1.rowwise() - RowVectorType(other.row(0)));
    m2 = m1;
    m2.colwise() *= other.col(0);
    VERIFY_IS_APPROX(m2, m1.colwise() * ColVectorType(other.col(0)));
    m2 = m1;
    m2.colwise() = other.col(0);
    for (Index k = 0; k < cols; ++k) {
      VERIFY_IS_APPROX(m2.col(k), ColVectorType(other.col(0)));
    }
  }

  // Issue #1731: aliased rhs across all five compound operators (used to
  // fail on `/=` when RowsAtCompileTime == 2).
  m2 = m1;
  m2.rowwise() /= m2.colwise().sum();
  VERIFY_IS_APPROX(m2, m1.rowwise() / m1.colwise().sum());

  m2 = m1;
  m2.colwise() /= m2.rowwise().sum();
  VERIFY_IS_APPROX(m2, m1.colwise() / m1.rowwise().sum());

  m2 = m1;
  m2.rowwise() += m2.colwise().sum();
  VERIFY_IS_APPROX(m2, m1.rowwise() + m1.colwise().sum());

  m2 = m1;
  m2.rowwise() -= m2.colwise().sum();
  VERIFY_IS_APPROX(m2, m1.rowwise() - m1.colwise().sum());

  m2 = m1;
  m2.rowwise() *= m2.colwise().sum();
  VERIFY_IS_APPROX(m2, m1.rowwise() * m1.colwise().sum());

  m2 = m1;
  m2.rowwise() = m2.colwise().sum();
  for (Index k = 0; k < rows; ++k) {
    VERIFY_IS_APPROX(m2.row(k), m1.colwise().sum());
  }

  // all/any
  Array<bool, Dynamic, Dynamic> mb(rows, cols);
  mb = (m1.real() <= 0.7).colwise().all();
  VERIFY((mb.col(c) == (m1.real().col(c) <= 0.7).all()).all());
  mb = (m1.real() <= 0.7).rowwise().all();
  VERIFY((mb.row(r) == (m1.real().row(r) <= 0.7).all()).all());

  mb = (m1.real() >= 0.7).colwise().any();
  VERIFY((mb.col(c) == (m1.real().col(c) >= 0.7).any()).all());
  mb = (m1.real() >= 0.7).rowwise().any();
  VERIFY((mb.row(r) == (m1.real().row(r) >= 0.7).any()).all());

  // test count()
  {
    Array<Index, 1, ArrayType::ColsAtCompileTime> colcounts(cols);
    Array<Index, ArrayType::RowsAtCompileTime, 1> rowcounts(rows);
    colcounts = (m1.real() >= 0).colwise().count();
    for (Index k = 0; k < cols; ++k) VERIFY_IS_EQUAL(colcounts(k), (m1.real().col(k) >= 0).count());
    rowcounts = (m1.real() >= 0).rowwise().count();
    for (Index k = 0; k < rows; ++k) VERIFY_IS_EQUAL(rowcounts(k), (m1.real().row(k) >= 0).count());
  }
}

template <typename MatrixType>
void vectorwiseop_matrix(const MatrixType& m) {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, MatrixType::RowsAtCompileTime, 1> ColVectorType;
  typedef Matrix<Scalar, 1, MatrixType::ColsAtCompileTime> RowVectorType;
  typedef Matrix<RealScalar, MatrixType::RowsAtCompileTime, 1> RealColVectorType;
  typedef Matrix<RealScalar, 1, MatrixType::ColsAtCompileTime> RealRowVectorType;
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixX;

  Index rows = m.rows();
  Index cols = m.cols();
  Index r = internal::random<Index>(0, rows - 1), c = internal::random<Index>(0, cols - 1);

  MatrixType m1 = MatrixType::Random(rows, cols), m2(rows, cols), m3(rows, cols);

  ColVectorType colvec = ColVectorType::Random(rows);
  RowVectorType rowvec = RowVectorType::Random(cols);
  RealColVectorType rcres;
  RealRowVectorType rrres;

  Scalar small_scalar = (std::numeric_limits<RealScalar>::min)();

  // test broadcast assignment
  m2 = m1;
  m2.colwise() = colvec;
  for (Index j = 0; j < cols; ++j) VERIFY_IS_APPROX(m2.col(j), colvec);
  m2.rowwise() = rowvec;
  for (Index i = 0; i < rows; ++i) VERIFY_IS_APPROX(m2.row(i), rowvec);

  // test addition
  m2 = m1;
  m2.colwise() += colvec;
  VERIFY_IS_APPROX(m2, m1.colwise() + colvec);
  VERIFY_IS_APPROX(m2, colvec + m1.colwise());
  VERIFY_IS_APPROX(m2.col(c), m1.col(c) + colvec);

  m2 = m1;
  m2.rowwise() += rowvec;
  VERIFY_IS_APPROX(m2, m1.rowwise() + rowvec);
  VERIFY_IS_APPROX(m2, rowvec + m1.rowwise());
  VERIFY_IS_APPROX(m2.row(r), m1.row(r) + rowvec);

  // test subtraction
  m2 = m1;
  m2.colwise() -= colvec;
  VERIFY_IS_APPROX(m2, m1.colwise() - colvec);
  VERIFY_IS_APPROX(m2.col(c), m1.col(c) - colvec);

  m2 = m1;
  m2.rowwise() -= rowvec;
  VERIFY_IS_APPROX(m2, m1.rowwise() - rowvec);
  VERIFY_IS_APPROX(m2.row(r), m1.row(r) - rowvec);

  // Block rhs: nested_eval must bind by reference, not materialize.
  {
    MatrixType other = MatrixType::Random(rows, cols);
    m2 = m1;
    m2.rowwise() += other.row(0);
    VERIFY_IS_APPROX(m2, m1.rowwise() + RowVectorType(other.row(0)));
    m2 = m1;
    m2.colwise() -= other.col(0);
    VERIFY_IS_APPROX(m2, m1.colwise() - ColVectorType(other.col(0)));
    m2 = m1;
    m2.colwise() = other.col(0);
    for (Index k = 0; k < cols; ++k) {
      VERIFY_IS_APPROX(m2.col(k), ColVectorType(other.col(0)));
    }
  }

  // Issue #1731: aliased rhs across `=`/`+=`/`-=` on matrices (Array's
  // `*=`/`/=` variants are exercised in vectorwiseop_array).
  m2 = m1;
  m2.rowwise() += m2.colwise().sum();
  VERIFY_IS_APPROX(m2, m1.rowwise() + m1.colwise().sum());

  m2 = m1;
  m2.colwise() -= m2.rowwise().sum();
  VERIFY_IS_APPROX(m2, m1.colwise() - m1.rowwise().sum());

  m2 = m1;
  m2.rowwise() = m2.colwise().sum();
  for (Index k = 0; k < rows; ++k) {
    VERIFY_IS_APPROX(m2.row(k), m1.colwise().sum());
  }

  // ------ partial reductions ------

#define TEST_PARTIAL_REDUX_BASIC(FUNC, ROW, COL, PREPROCESS)                              \
  {                                                                                       \
    ROW = m1 PREPROCESS.colwise().FUNC;                                                   \
    for (Index k = 0; k < cols; ++k) VERIFY_IS_APPROX(ROW(k), m1.col(k) PREPROCESS.FUNC); \
    COL = m1 PREPROCESS.rowwise().FUNC;                                                   \
    for (Index k = 0; k < rows; ++k) VERIFY_IS_APPROX(COL(k), m1.row(k) PREPROCESS.FUNC); \
  }

  TEST_PARTIAL_REDUX_BASIC(sum(), rowvec, colvec, EIGEN_EMPTY);
  TEST_PARTIAL_REDUX_BASIC(prod(), rowvec, colvec, EIGEN_EMPTY);
  TEST_PARTIAL_REDUX_BASIC(mean(), rowvec, colvec, EIGEN_EMPTY);
  TEST_PARTIAL_REDUX_BASIC(minCoeff(), rrres, rcres, .real());
  TEST_PARTIAL_REDUX_BASIC(maxCoeff(), rrres, rcres, .real());
  TEST_PARTIAL_REDUX_BASIC(norm(), rrres, rcres, EIGEN_EMPTY);
  TEST_PARTIAL_REDUX_BASIC(squaredNorm(), rrres, rcres, EIGEN_EMPTY);
  TEST_PARTIAL_REDUX_BASIC(redux(internal::scalar_sum_op<Scalar, Scalar>()), rowvec, colvec, EIGEN_EMPTY);

  VERIFY_IS_APPROX(m1.cwiseAbs().colwise().sum(), m1.colwise().template lpNorm<1>());
  VERIFY_IS_APPROX(m1.cwiseAbs().rowwise().sum(), m1.rowwise().template lpNorm<1>());
  VERIFY_IS_APPROX(m1.cwiseAbs().colwise().maxCoeff(), m1.colwise().template lpNorm<Infinity>());
  VERIFY_IS_APPROX(m1.cwiseAbs().rowwise().maxCoeff(), m1.rowwise().template lpNorm<Infinity>());

  // regression for bug 1158
  VERIFY_IS_APPROX(m1.cwiseAbs().colwise().sum().x(), m1.col(0).cwiseAbs().sum());

  // test normalized
  m2 = m1;
  m2.col(c).fill(small_scalar);
  m3 = m2.colwise().normalized();
  for (Index k = 0; k < cols; ++k) VERIFY_IS_APPROX(m3.col(k), m2.col(k).normalized());
  m2 = m1;
  m2.row(r).setZero();
  m3 = m2.rowwise().normalized();
  for (Index k = 0; k < rows; ++k) VERIFY_IS_APPROX(m3.row(k), m2.row(k).normalized());

  // test normalize
  m2 = m1;
  m2.col(c).setZero();
  m3 = m2;
  m3.colwise().normalize();
  for (Index k = 0; k < cols; ++k) VERIFY_IS_APPROX(m3.col(k), m2.col(k).normalized());
  m2 = m1;
  m2.row(r).fill(small_scalar);
  m3 = m2;
  m3.rowwise().normalize();
  for (Index k = 0; k < rows; ++k) VERIFY_IS_APPROX(m3.row(k), m2.row(k).normalized());

  // test with partial reduction of products
  Matrix<Scalar, MatrixType::RowsAtCompileTime, MatrixType::RowsAtCompileTime> m1m1 = m1 * m1.transpose();
  VERIFY_IS_APPROX((m1 * m1.transpose()).colwise().sum(), m1m1.colwise().sum());
  Matrix<Scalar, 1, MatrixType::RowsAtCompileTime> tmp(rows);
  VERIFY_EVALUATION_COUNT(tmp = (m1 * m1.transpose()).colwise().sum(), 1);

  m2 = m1.rowwise() - (m1.colwise().sum() / RealScalar(m1.rows())).eval();
  m1 = m1.rowwise() - (m1.colwise().sum() / RealScalar(m1.rows()));
  VERIFY_IS_APPROX(m1, m2);
  VERIFY_EVALUATION_COUNT(m2 = (m1.rowwise() - m1.colwise().sum() / RealScalar(m1.rows())),
                          (MatrixType::RowsAtCompileTime != 1 ? 1 : 0));

  // test colwise/rowwise reverse
  {
    MatrixType m_rev(rows, cols);
    m_rev = m1.colwise().reverse();
    for (Index k = 0; k < cols; ++k) VERIFY_IS_APPROX(m_rev.col(k), m1.col(k).reverse());
    m_rev = m1.rowwise().reverse();
    for (Index k = 0; k < rows; ++k) VERIFY_IS_APPROX(m_rev.row(k), m1.row(k).reverse());
  }

  // test empty expressions
  VERIFY_IS_APPROX(m1.matrix().middleCols(0, 0).rowwise().sum().eval(), MatrixX::Zero(rows, 1));
  VERIFY_IS_APPROX(m1.matrix().middleRows(0, 0).colwise().sum().eval(), MatrixX::Zero(1, cols));
  VERIFY_IS_APPROX(m1.matrix().middleCols(0, fix<0>).rowwise().sum().eval(), MatrixX::Zero(rows, 1));
  VERIFY_IS_APPROX(m1.matrix().middleRows(0, fix<0>).colwise().sum().eval(), MatrixX::Zero(1, cols));

  VERIFY_IS_APPROX(m1.matrix().middleCols(0, 0).rowwise().prod().eval(), MatrixX::Ones(rows, 1));
  VERIFY_IS_APPROX(m1.matrix().middleRows(0, 0).colwise().prod().eval(), MatrixX::Ones(1, cols));
  VERIFY_IS_APPROX(m1.matrix().middleCols(0, fix<0>).rowwise().prod().eval(), MatrixX::Ones(rows, 1));
  VERIFY_IS_APPROX(m1.matrix().middleRows(0, fix<0>).colwise().prod().eval(), MatrixX::Ones(1, cols));
  VERIFY_IS_APPROX(m1.matrix().middleCols(0, 0).rowwise().squaredNorm().eval(), MatrixX::Zero(rows, 1));

  VERIFY_IS_EQUAL(m1.real().middleRows(0, 0).rowwise().maxCoeff().eval().rows(), 0);
  VERIFY_IS_EQUAL(m1.real().middleCols(0, 0).colwise().maxCoeff().eval().cols(), 0);
  VERIFY_IS_EQUAL(m1.real().middleRows(0, fix<0>).rowwise().maxCoeff().eval().rows(), 0);
  VERIFY_IS_EQUAL(m1.real().middleCols(0, fix<0>).colwise().maxCoeff().eval().cols(), 0);
}

// Integer-safe subset of vectorwiseop_array: tests +, -, all/any, count only.
// Skips *, / which cause integer overflow or division-by-zero with full-range random ints.
template <typename ArrayType>
void vectorwiseop_array_integer(const ArrayType& m) {
  typedef typename ArrayType::Scalar Scalar;
  typedef Array<Scalar, ArrayType::RowsAtCompileTime, 1> ColVectorType;
  typedef Array<Scalar, 1, ArrayType::ColsAtCompileTime> RowVectorType;

  Index rows = m.rows();
  Index cols = m.cols();
  Index r = internal::random<Index>(0, rows - 1), c = internal::random<Index>(0, cols - 1);

  ArrayType m1 = ArrayType::Random(rows, cols), m2(rows, cols);
  // Clamp to avoid overflow even in addition/subtraction.
  for (Index j = 0; j < cols; ++j)
    for (Index i = 0; i < rows; ++i) m1(i, j) = m1(i, j) % Scalar(10000);

  ColVectorType colvec = ColVectorType::Random(rows);
  for (Index i = 0; i < rows; ++i) colvec(i) = colvec(i) % Scalar(10000);
  RowVectorType rowvec = RowVectorType::Random(cols);
  for (Index j = 0; j < cols; ++j) rowvec(j) = rowvec(j) % Scalar(10000);

  // test addition
  m2 = m1;
  m2.colwise() += colvec;
  VERIFY_IS_APPROX(m2, m1.colwise() + colvec);
  VERIFY_IS_APPROX(m2.col(c), m1.col(c) + colvec);

  m2 = m1;
  m2.rowwise() += rowvec;
  VERIFY_IS_APPROX(m2, m1.rowwise() + rowvec);
  VERIFY_IS_APPROX(m2.row(r), m1.row(r) + rowvec);

  // test subtraction
  m2 = m1;
  m2.colwise() -= colvec;
  VERIFY_IS_APPROX(m2, m1.colwise() - colvec);
  VERIFY_IS_APPROX(m2.col(c), m1.col(c) - colvec);

  m2 = m1;
  m2.rowwise() -= rowvec;
  VERIFY_IS_APPROX(m2, m1.rowwise() - rowvec);
  VERIFY_IS_APPROX(m2.row(r), m1.row(r) - rowvec);

  // all/any
  Array<bool, Dynamic, Dynamic> mb(rows, cols);
  mb = (m1 <= Scalar(0)).colwise().all();
  VERIFY((mb.col(c) == (m1.col(c) <= Scalar(0)).all()).all());
  mb = (m1 <= Scalar(0)).rowwise().all();
  VERIFY((mb.row(r) == (m1.row(r) <= Scalar(0)).all()).all());

  mb = (m1 >= Scalar(0)).colwise().any();
  VERIFY((mb.col(c) == (m1.col(c) >= Scalar(0)).any()).all());
  mb = (m1 >= Scalar(0)).rowwise().any();
  VERIFY((mb.row(r) == (m1.row(r) >= Scalar(0)).any()).all());

  // test count()
  {
    Array<Index, 1, ArrayType::ColsAtCompileTime> colcounts(cols);
    Array<Index, ArrayType::RowsAtCompileTime, 1> rowcounts(rows);
    colcounts = (m1 >= Scalar(0)).colwise().count();
    for (Index k = 0; k < cols; ++k) VERIFY_IS_EQUAL(colcounts(k), (m1.col(k) >= Scalar(0)).count());
    rowcounts = (m1 >= Scalar(0)).rowwise().count();
    for (Index k = 0; k < rows; ++k) VERIFY_IS_EQUAL(rowcounts(k), (m1.row(k) >= Scalar(0)).count());
  }
}

void vectorwiseop_mixedscalar() {
  Matrix4cd a = Matrix4cd::Random();
  Vector4cd b = Vector4cd::Random();
  b.imag().setZero();
  Vector4d b_real = b.real();

  Matrix4cd c = a.array().rowwise() * b.array().transpose();
  Matrix4cd d = a.array().rowwise() * b_real.array().transpose();
  Matrix4cd e = b.array().transpose() * a.array().rowwise();
  Matrix4cd f = b_real.array().transpose() * a.array().rowwise();
  VERIFY_IS_CWISE_EQUAL(c, d);
  VERIFY_IS_CWISE_EQUAL(c, e);
  VERIFY_IS_CWISE_EQUAL(d, f);
}

template <typename ArrayType>
void vectorwiseop_array_extensions(const ArrayType& m) {
  typedef typename ArrayType::Scalar Scalar;
  typedef Array<Scalar, ArrayType::RowsAtCompileTime, 1> ColVectorType;
  typedef Array<Scalar, 1, ArrayType::ColsAtCompileTime> RowVectorType;

  Index rows = m.rows();
  Index cols = m.cols();

  ArrayType m1 = ArrayType::Random(rows, cols) + ArrayType::Constant(rows, cols, Scalar(2));
  ArrayType expected(rows, cols);
  ColVectorType colvec = ColVectorType::Random(rows);
  RowVectorType rowvec = RowVectorType::Random(cols);
  broadcast_affine_op<Scalar> op;

  for (Index j = 0; j < cols; ++j) expected.col(j) = colvec - m1.col(j);
  VERIFY_IS_APPROX(expected, colvec - m1.colwise());

  for (Index i = 0; i < rows; ++i) expected.row(i) = rowvec - m1.row(i);
  VERIFY_IS_APPROX(expected, rowvec - m1.rowwise());

  for (Index j = 0; j < cols; ++j) expected.col(j) = colvec / m1.col(j);
  VERIFY_IS_APPROX(expected, colvec / m1.colwise());

  for (Index i = 0; i < rows; ++i) expected.row(i) = rowvec / m1.row(i);
  VERIFY_IS_APPROX(expected, rowvec / m1.rowwise());

  for (Index j = 0; j < cols; ++j) expected.col(j) = m1.col(j).binaryExpr(colvec, op);
  VERIFY_IS_APPROX(expected, m1.colwise().binaryExpr(colvec, op));

  for (Index i = 0; i < rows; ++i) expected.row(i) = m1.row(i).binaryExpr(rowvec, op);
  VERIFY_IS_APPROX(expected, m1.rowwise().binaryExpr(rowvec, op));

  m1 = ArrayType::Random(rows, cols);
  m1(0, 0) = Scalar(-2);
  m1(rows - 1, cols - 1) = Scalar(2);
  for (Index j = 0; j < cols; ++j) expected.col(j) = m1.col(j).cwiseMin(colvec);
  VERIFY_IS_APPROX(expected, m1.colwise().cwiseMin(colvec));

  for (Index i = 0; i < rows; ++i) expected.row(i) = m1.row(i).cwiseMin(rowvec);
  VERIFY_IS_APPROX(expected, m1.rowwise().cwiseMin(rowvec));

  expected = m1.cwiseMin(Scalar(0.25));
  VERIFY_IS_APPROX(expected, m1.colwise().cwiseMin(Scalar(0.25)));
  VERIFY_IS_APPROX(expected, m1.rowwise().cwiseMin(Scalar(0.25)));

  for (Index j = 0; j < cols; ++j) expected.col(j) = m1.col(j).cwiseMax(colvec);
  VERIFY_IS_APPROX(expected, m1.colwise().cwiseMax(colvec));

  for (Index i = 0; i < rows; ++i) expected.row(i) = m1.row(i).cwiseMax(rowvec);
  VERIFY_IS_APPROX(expected, m1.rowwise().cwiseMax(rowvec));

  expected = m1.cwiseMax(Scalar(-0.25));
  VERIFY_IS_APPROX(expected, m1.colwise().cwiseMax(Scalar(-0.25)));
  VERIFY_IS_APPROX(expected, m1.rowwise().cwiseMax(Scalar(-0.25)));
}

template <typename MatrixType>
void vectorwiseop_matrix_extensions(const MatrixType& m) {
  typedef typename MatrixType::Scalar Scalar;
  typedef Matrix<Scalar, MatrixType::RowsAtCompileTime, 1> ColVectorType;
  typedef Matrix<Scalar, 1, MatrixType::ColsAtCompileTime> RowVectorType;

  Index rows = m.rows();
  Index cols = m.cols();

  MatrixType m1 = MatrixType::Random(rows, cols), expected(rows, cols);
  ColVectorType colvec = ColVectorType::Random(rows);
  RowVectorType rowvec = RowVectorType::Random(cols);
  broadcast_affine_op<Scalar> op;

  for (Index j = 0; j < cols; ++j) expected.col(j) = colvec - m1.col(j);
  VERIFY_IS_APPROX(expected, colvec - m1.colwise());

  for (Index i = 0; i < rows; ++i) expected.row(i) = rowvec - m1.row(i);
  VERIFY_IS_APPROX(expected, rowvec - m1.rowwise());

  for (Index j = 0; j < cols; ++j) expected.col(j) = m1.col(j).binaryExpr(colvec, op);
  VERIFY_IS_APPROX(expected, m1.colwise().binaryExpr(colvec, op));

  for (Index i = 0; i < rows; ++i) expected.row(i) = m1.row(i).binaryExpr(rowvec, op);
  VERIFY_IS_APPROX(expected, m1.rowwise().binaryExpr(rowvec, op));

  for (Index j = 0; j < cols; ++j) expected.col(j) = m1.col(j).cwiseMin(colvec);
  VERIFY_IS_APPROX(expected, m1.colwise().cwiseMin(colvec));

  for (Index i = 0; i < rows; ++i) expected.row(i) = m1.row(i).cwiseMin(rowvec);
  VERIFY_IS_APPROX(expected, m1.rowwise().cwiseMin(rowvec));

  expected = m1.cwiseMin(Scalar(0.25));
  VERIFY_IS_APPROX(expected, m1.colwise().cwiseMin(Scalar(0.25)));
  VERIFY_IS_APPROX(expected, m1.rowwise().cwiseMin(Scalar(0.25)));

  for (Index j = 0; j < cols; ++j) expected.col(j) = m1.col(j).cwiseMax(colvec);
  VERIFY_IS_APPROX(expected, m1.colwise().cwiseMax(colvec));

  for (Index i = 0; i < rows; ++i) expected.row(i) = m1.row(i).cwiseMax(rowvec);
  VERIFY_IS_APPROX(expected, m1.rowwise().cwiseMax(rowvec));

  expected = m1.cwiseMax(Scalar(-0.25));
  VERIFY_IS_APPROX(expected, m1.colwise().cwiseMax(Scalar(-0.25)));
  VERIFY_IS_APPROX(expected, m1.rowwise().cwiseMax(Scalar(-0.25)));
}

void vectorwiseop_operand_order() {
  Array<std::string, 2, 3> strings;
  strings.setConstant("R");
  Array<std::string, 2, 1> column;
  column.setConstant("L");
  Array<std::string, 1, 3> row;
  row.setConstant("L");
  const Array<std::string, 2, 3> column_sum = column + strings.colwise();
  const Array<std::string, 2, 3> row_sum = row + strings.rowwise();
  for (Index i = 0; i < strings.size(); ++i) {
    VERIFY_IS_EQUAL(column_sum(i), std::string("LR"));
    VERIFY_IS_EQUAL(row_sum(i), std::string("LR"));
  }

  Array<Quaterniond, 2, 3> quaternions;
  quaternions.setConstant(Quaterniond(0, 0, 1, 0));
  Array<Quaterniond, 2, 1> column_quaternions;
  column_quaternions.setConstant(Quaterniond(0, 1, 0, 0));
  Array<Quaterniond, 1, 3> row_quaternions;
  row_quaternions.setConstant(Quaterniond(0, 1, 0, 0));
  const Array<Quaterniond, 2, 3> column_product = column_quaternions * quaternions.colwise();
  const Array<Quaterniond, 2, 3> row_product = row_quaternions * quaternions.rowwise();
  const Quaterniond expected(0, 0, 0, 1);  // i*j = k, whereas j*i = -k.
  for (Index i = 0; i < quaternions.size(); ++i) {
    VERIFY_IS_CWISE_EQUAL(column_product(i).coeffs(), expected.coeffs());
    VERIFY_IS_CWISE_EQUAL(row_product(i).coeffs(), expected.coeffs());
  }
}

template <int NaNPropagation>
void vectorwiseop_minmax_nan() {
  const double nan = NumTraits<double>::quiet_NaN();
  Array22d a;
  a << nan, 2, -3, nan;
  Array2d v;
  v << 1, nan;
  const Array22d min_col = a.colwise().cwiseMin<NaNPropagation>(v);
  const Array22d max_col = a.colwise().cwiseMax<NaNPropagation>(v);
  const Array22d min_row = a.transpose().rowwise().cwiseMin<NaNPropagation>(v.transpose());
  const Array22d max_row = a.transpose().rowwise().cwiseMax<NaNPropagation>(v.transpose());
  for (Index i = 0; i < 2; ++i) {
    for (Index j = 0; j < 2; ++j) {
      const bool a_nan = (numext::isnan)(a(i, j));
      const bool v_nan = (numext::isnan)(v(i));
      const bool expect_nan = NaNPropagation == PropagateNaN ? a_nan || v_nan : a_nan && v_nan;
      VERIFY_IS_EQUAL((numext::isnan)(min_col(i, j)), expect_nan);
      VERIFY_IS_EQUAL((numext::isnan)(max_col(i, j)), expect_nan);
      VERIFY_IS_EQUAL((numext::isnan)(min_row(j, i)), expect_nan);
      VERIFY_IS_EQUAL((numext::isnan)(max_row(j, i)), expect_nan);
      if (!expect_nan) {
        const double lo = a_nan ? v(i) : v_nan ? a(i, j) : (std::min)(a(i, j), v(i));
        const double hi = a_nan ? v(i) : v_nan ? a(i, j) : (std::max)(a(i, j), v(i));
        VERIFY_IS_EQUAL(min_col(i, j), lo);
        VERIFY_IS_EQUAL(max_col(i, j), hi);
        VERIFY_IS_EQUAL(min_row(j, i), lo);
        VERIFY_IS_EQUAL(max_row(j, i), hi);
      }
    }
  }
}

// Test partial reductions on RowMajor matrices.
// The existing tests only use ColMajor matrices.
template <typename Scalar>
void vectorwiseop_rowmajor() {
  typedef Matrix<Scalar, Dynamic, Dynamic, RowMajor> RowMajorMatrix;
  typedef Matrix<Scalar, Dynamic, Dynamic, ColMajor> ColMajorMatrix;
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<RealScalar, 1, Dynamic> RealRowVectorType;
  typedef Matrix<RealScalar, Dynamic, 1> RealColVectorType;

  const Index rows = 7;
  const Index cols = 11;
  ColMajorMatrix mc = ColMajorMatrix::Random(rows, cols);
  RowMajorMatrix mr = mc;  // same data, different storage

  // Partial reductions should give the same result regardless of storage order.
  VERIFY_IS_APPROX(mc.colwise().sum(), mr.colwise().sum());
  VERIFY_IS_APPROX(mc.rowwise().sum(), mr.rowwise().sum());
  VERIFY_IS_APPROX(mc.colwise().prod(), mr.colwise().prod());
  VERIFY_IS_APPROX(mc.rowwise().prod(), mr.rowwise().prod());
  VERIFY_IS_APPROX(mc.colwise().squaredNorm(), mr.colwise().squaredNorm());
  VERIFY_IS_APPROX(mc.rowwise().squaredNorm(), mr.rowwise().squaredNorm());
  VERIFY_IS_APPROX(mc.colwise().norm(), mr.colwise().norm());
  VERIFY_IS_APPROX(mc.rowwise().norm(), mr.rowwise().norm());

  RealRowVectorType rr_c, rr_r;
  RealColVectorType rc_c, rc_r;
  rr_c = mc.real().colwise().minCoeff();
  rr_r = mr.real().colwise().minCoeff();
  VERIFY_IS_APPROX(rr_c, rr_r);
  rr_c = mc.real().colwise().maxCoeff();
  rr_r = mr.real().colwise().maxCoeff();
  VERIFY_IS_APPROX(rr_c, rr_r);
  rc_c = mc.real().rowwise().minCoeff();
  rc_r = mr.real().rowwise().minCoeff();
  VERIFY_IS_APPROX(rc_c, rc_r);
  rc_c = mc.real().rowwise().maxCoeff();
  rc_r = mr.real().rowwise().maxCoeff();
  VERIFY_IS_APPROX(rc_c, rc_r);

  // Broadcast operations
  typedef Matrix<Scalar, Dynamic, 1> ColVectorType;
  typedef Matrix<Scalar, 1, Dynamic> RowVectorType;
  ColVectorType cv = ColVectorType::Random(rows);
  RowVectorType rv = RowVectorType::Random(cols);

  VERIFY_IS_APPROX(ColMajorMatrix(mc.colwise() + cv), ColMajorMatrix(mr.colwise() + cv));
  VERIFY_IS_APPROX(ColMajorMatrix(mc.rowwise() + rv), ColMajorMatrix(mr.rowwise() + rv));
  VERIFY_IS_APPROX(ColMajorMatrix(cv + mc.colwise()), ColMajorMatrix(cv + mr.colwise()));
  VERIFY_IS_APPROX(ColMajorMatrix(rv + mc.rowwise()), ColMajorMatrix(rv + mr.rowwise()));
  VERIFY_IS_APPROX(ColMajorMatrix(mc.colwise() - cv), ColMajorMatrix(mr.colwise() - cv));
  VERIFY_IS_APPROX(ColMajorMatrix(mc.rowwise() - rv), ColMajorMatrix(mr.rowwise() - rv));
}

EIGEN_DECLARE_TEST(vectorwiseop) {
  CALL_SUBTEST_11(vectorwiseop_operand_order());
  CALL_SUBTEST_11(vectorwiseop_minmax_nan<PropagateNaN>());
  CALL_SUBTEST_11(vectorwiseop_minmax_nan<PropagateNumbers>());
  CALL_SUBTEST_1(vectorwiseop_array(Array22cd()));
  CALL_SUBTEST_2(vectorwiseop_array(Array<double, 3, 2>()));
  CALL_SUBTEST_3(vectorwiseop_array(ArrayXXf(3, 4)));
  // Issue #1731: compile-time 2-row Array used to alias on rowwise() /= colwise().sum().
  CALL_SUBTEST_3(vectorwiseop_array(Array<double, 2, Dynamic>(2, internal::random<int>(2, EIGEN_TEST_MAX_SIZE))));
  CALL_SUBTEST_4(vectorwiseop_matrix(Matrix4cf()));
  CALL_SUBTEST_5(vectorwiseop_matrix(Matrix4f()));
  CALL_SUBTEST_5(vectorwiseop_matrix(Vector4f()));
  CALL_SUBTEST_5(vectorwiseop_matrix(Matrix<float, 4, 5>()));
  CALL_SUBTEST_6(vectorwiseop_matrix(
      MatrixXd(internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
  CALL_SUBTEST_7(vectorwiseop_matrix(VectorXd(internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
  CALL_SUBTEST_7(vectorwiseop_matrix(RowVectorXd(internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
  CALL_SUBTEST_8(vectorwiseop_mixedscalar());
  CALL_SUBTEST_9(vectorwiseop_array_integer(
      ArrayXXi(internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));

  // RowMajor partial reductions (deterministic, outside g_repeat).
  CALL_SUBTEST_10(vectorwiseop_rowmajor<float>());
  CALL_SUBTEST_10(vectorwiseop_rowmajor<double>());
  CALL_SUBTEST_10(vectorwiseop_rowmajor<std::complex<float>>());
  CALL_SUBTEST_11(vectorwiseop_array_extensions(Array<float, 3, 4>()));
  CALL_SUBTEST_11(vectorwiseop_array_extensions(ArrayXXd(3, 4)));
  CALL_SUBTEST_12(vectorwiseop_matrix_extensions(Matrix<float, 4, 5>()));
  CALL_SUBTEST_12(vectorwiseop_matrix_extensions(MatrixXd(3, 4)));
}
