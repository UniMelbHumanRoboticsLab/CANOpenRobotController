// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2006-2010 Benoit Jacob <jacob.benoit.1@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "main.h"
#include "random_for_arithmetic.h"

template <typename MatrixType, typename Index, typename Scalar>
std::enable_if_t<!NumTraits<typename MatrixType::Scalar>::IsComplex, typename MatrixType::Scalar> block_real_only(
    const MatrixType& m1, Index r1, Index r2, Index c1, Index c2, const Scalar& s1) {
  // check cwise-Functions:
  VERIFY_IS_APPROX(m1.row(r1).cwiseMax(s1), m1.cwiseMax(s1).row(r1));
  VERIFY_IS_APPROX(m1.col(c1).cwiseMin(s1), m1.cwiseMin(s1).col(c1));

  VERIFY_IS_APPROX(m1.block(r1, c1, r2 - r1 + 1, c2 - c1 + 1).cwiseMin(s1),
                   m1.cwiseMin(s1).block(r1, c1, r2 - r1 + 1, c2 - c1 + 1));
  VERIFY_IS_APPROX(m1.block(r1, c1, r2 - r1 + 1, c2 - c1 + 1).cwiseMax(s1),
                   m1.cwiseMax(s1).block(r1, c1, r2 - r1 + 1, c2 - c1 + 1));

  return Scalar(0);
}

template <typename MatrixType, typename Index, typename Scalar>
std::enable_if_t<NumTraits<typename MatrixType::Scalar>::IsComplex, typename MatrixType::Scalar> block_real_only(
    const MatrixType&, Index, Index, Index, Index, const Scalar&) {
  return Scalar(0);
}

// Check at compile-time that T1==T2, and at runtime-time that a==b
template <typename T1, typename T2>
std::enable_if_t<std::is_same<T1, T2>::value, bool> is_same_block(const T1& a, const T2& b) {
  return a.isApprox(b);
}

template <typename MatrixType>
std::enable_if_t<((MatrixType::Flags & RowMajorBit) == 0), void> check_left_top(const MatrixType& m, Index r, Index c,
                                                                                Index rows, Index /*unused*/) {
  if (c > 0) VERIFY_IS_EQUAL(m.leftCols(c).coeff(r + c * rows), m(r, c));
}

template <typename MatrixType>
std::enable_if_t<((MatrixType::Flags & RowMajorBit) != 0), void> check_left_top(const MatrixType& m, Index r, Index c,
                                                                                Index /*unused*/, Index cols) {
  if (r > 0) VERIFY_IS_EQUAL(m.topRows(r).coeff(c + r * cols), m(r, c));
}

template <typename MatrixType>
std::enable_if_t<((MatrixType::Flags & RowMajorBit) == 0), void> checkInnerVector(const MatrixType& m, Index /*r*/,
                                                                                  Index c, Index /*rows*/, Index cols) {
  VERIFY_IS_CWISE_EQUAL(m.innerVector(c), m.col(c));
  VERIFY_IS_CWISE_EQUAL(m.innerVectors(c, cols), m.middleCols(c, cols));
}

template <typename MatrixType>
std::enable_if_t<((MatrixType::Flags & RowMajorBit) != 0), void> checkInnerVector(const MatrixType& m, Index r,
                                                                                  Index /*c*/, Index rows,
                                                                                  Index /*cols*/) {
  VERIFY_IS_CWISE_EQUAL(m.innerVector(r), m.row(r));
  VERIFY_IS_CWISE_EQUAL(m.innerVectors(r, rows), m.middleRows(r, rows));
}

template <typename MatrixType>
void block(const MatrixType& m) {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename MatrixType::RealScalar RealScalar;
  typedef Matrix<Scalar, MatrixType::RowsAtCompileTime, 1> VectorType;
  typedef Matrix<Scalar, 1, MatrixType::ColsAtCompileTime> RowVectorType;
  typedef Matrix<Scalar, Dynamic, Dynamic, MatrixType::IsRowMajor ? RowMajor : ColMajor> DynamicMatrixType;
  typedef Matrix<Scalar, Dynamic, 1> DynamicVectorType;

  Index rows = m.rows();
  Index cols = m.cols();

  MatrixType m1 = random_for_arithmetic<MatrixType>(rows, cols), m1_copy = m1,
             m2 = random_for_arithmetic<MatrixType>(rows, cols), m3(rows, cols), ones = MatrixType::Ones(rows, cols);
  VectorType v1 = random_for_arithmetic<VectorType>(rows);

  Scalar s1 = random_scalar_for_arithmetic<Scalar>();

  Index r1 = internal::random<Index>(0, rows - 1);
  Index r2 = internal::random<Index>(r1, rows - 1);
  Index c1 = internal::random<Index>(0, cols - 1);
  Index c2 = internal::random<Index>(c1, cols - 1);

  block_real_only(m1, r1, r2, c1, c1, s1);
  checkInnerVector(m1, r1, c1, r2 - r1, c2 - c1);

  // test fill logic with innerpanel and non-innerpanel blocks
  m1.row(r1).setConstant(s1);
  VERIFY_IS_CWISE_EQUAL(m1.row(r1), DynamicVectorType::Constant(cols, s1).transpose());
  m1 = m1_copy;
  m1.col(c1).setConstant(s1);
  VERIFY_IS_CWISE_EQUAL(m1.col(c1), DynamicVectorType::Constant(rows, s1));
  m1 = m1_copy;
  // test setZero logic with innerpanel and non-innerpanel blocks
  m1.row(r1).setZero();
  VERIFY_IS_CWISE_EQUAL(m1.row(r1), DynamicVectorType::Zero(cols).transpose());
  m1 = m1_copy;
  m1.col(c1).setZero();
  VERIFY_IS_CWISE_EQUAL(m1.col(c1), DynamicVectorType::Zero(rows));
  m1 = m1_copy;
  // test setZero/setConstant/setOnes on non-contiguous multi-row/multi-col blocks
  // This exercises the non-fill_n path in Fill.h and verifies no data corruption
  if (r2 > r1 && c2 > c1) {
    Index br = r2 - r1, bc = c2 - c1;
    m1 = m1_copy;
    m1.block(r1, c1, br, bc).setZero();
    VERIFY_IS_CWISE_EQUAL(m1.block(r1, c1, br, bc), DynamicMatrixType::Zero(br, bc));
    for (Index j = 0; j < cols; ++j)
      for (Index i = 0; i < rows; ++i)
        if (i < r1 || i >= r2 || j < c1 || j >= c2) VERIFY_IS_EQUAL(m1(i, j), m1_copy(i, j));

    m1 = m1_copy;
    m1.block(r1, c1, br, bc).setConstant(s1);
    VERIFY_IS_CWISE_EQUAL(m1.block(r1, c1, br, bc), DynamicMatrixType::Constant(br, bc, s1));
    for (Index j = 0; j < cols; ++j)
      for (Index i = 0; i < rows; ++i)
        if (i < r1 || i >= r2 || j < c1 || j >= c2) VERIFY_IS_EQUAL(m1(i, j), m1_copy(i, j));

    m1 = m1_copy;
    m1.block(r1, c1, br, bc).setOnes();
    VERIFY_IS_CWISE_EQUAL(m1.block(r1, c1, br, bc), DynamicMatrixType::Ones(br, bc));
    for (Index j = 0; j < cols; ++j)
      for (Index i = 0; i < rows; ++i)
        if (i < r1 || i >= r2 || j < c1 || j >= c2) VERIFY_IS_EQUAL(m1(i, j), m1_copy(i, j));
  }
  m1 = m1_copy;

  // check row() and col()
  VERIFY_IS_EQUAL(m1.col(c1).transpose(), m1.transpose().row(c1));
  // check operator(), both constant and non-constant, on row() and col()
  m1 = m1_copy;
  m1.row(r1) += s1 * m1_copy.row(r2);
  VERIFY_IS_APPROX(m1.row(r1), m1_copy.row(r1) + s1 * m1_copy.row(r2));
  // check nested block xpr on lhs
  m1.row(r1).row(0) += s1 * m1_copy.row(r2);
  VERIFY_IS_APPROX(m1.row(r1), m1_copy.row(r1) + Scalar(2) * s1 * m1_copy.row(r2));
  m1 = m1_copy;
  m1.col(c1) += s1 * m1_copy.col(c2);
  VERIFY_IS_APPROX(m1.col(c1), m1_copy.col(c1) + s1 * m1_copy.col(c2));
  m1.col(c1).col(0) += s1 * m1_copy.col(c2);
  VERIFY_IS_APPROX(m1.col(c1), m1_copy.col(c1) + Scalar(2) * s1 * m1_copy.col(c2));

  check_left_top(m1, r1, c1, rows, cols);

  // check block()
  Matrix<Scalar, Dynamic, Dynamic> b1(1, 1);
  b1(0, 0) = m1(r1, c1);

  RowVectorType br1(m1.block(r1, 0, 1, cols));
  VectorType bc1(m1.block(0, c1, rows, 1));
  VERIFY_IS_EQUAL(b1, m1.block(r1, c1, 1, 1));
  VERIFY_IS_EQUAL(m1.row(r1), br1);
  VERIFY_IS_EQUAL(m1.col(c1), bc1);
  // check operator(), both constant and non-constant, on block()
  m1.block(r1, c1, r2 - r1 + 1, c2 - c1 + 1) = s1 * m2.block(0, 0, r2 - r1 + 1, c2 - c1 + 1);
  m1.block(r1, c1, r2 - r1 + 1, c2 - c1 + 1)(r2 - r1, c2 - c1) = m2.block(0, 0, r2 - r1 + 1, c2 - c1 + 1)(0, 0);

  const Index BlockRows = 2;
  const Index BlockCols = 5;

  if (rows >= 5 && cols >= 8) {
    // test fixed block() as lvalue
    m1.template block<BlockRows, BlockCols>(1, 1) *= s1;
    // test operator() on fixed block() both as constant and non-constant
    m1.template block<BlockRows, BlockCols>(1, 1)(0, 3) = m1.template block<2, 5>(1, 1)(1, 2);
    // check that fixed block() and block() agree
    Matrix<Scalar, Dynamic, Dynamic> b = m1.template block<BlockRows, BlockCols>(3, 3);
    VERIFY_IS_EQUAL(b, m1.block(3, 3, BlockRows, BlockCols));

    // same tests with mixed fixed/dynamic size
    m1.template block<BlockRows, Dynamic>(1, 1, BlockRows, BlockCols) *= s1;
    m1.template block<BlockRows, Dynamic>(1, 1, BlockRows, BlockCols)(0, 3) = m1.template block<2, 5>(1, 1)(1, 2);
    Matrix<Scalar, Dynamic, Dynamic> b2 = m1.template block<Dynamic, BlockCols>(3, 3, 2, 5);
    VERIFY_IS_EQUAL(b2, m1.block(3, 3, BlockRows, BlockCols));

    VERIFY(is_same_block(m1.block(3, 3, BlockRows, BlockCols),
                         m1.block(3, 3, fix<Dynamic>(BlockRows), fix<Dynamic>(BlockCols))));
    VERIFY(is_same_block(m1.template block<BlockRows, Dynamic>(1, 1, BlockRows, BlockCols),
                         m1.block(1, 1, fix<BlockRows>, BlockCols)));
    VERIFY(is_same_block(m1.template block<BlockRows, BlockCols>(1, 1, BlockRows, BlockCols),
                         m1.block(1, 1, fix<BlockRows>(), fix<BlockCols>)));
    VERIFY(is_same_block(m1.template block<BlockRows, BlockCols>(1, 1, BlockRows, BlockCols),
                         m1.block(1, 1, fix<BlockRows>, fix<BlockCols>(BlockCols))));
  }

  if (rows > 2) {
    // test sub vectors
    VERIFY_IS_EQUAL(v1.template head<2>(), v1.block(0, 0, 2, 1));
    VERIFY_IS_EQUAL(v1.template head<2>(), v1.head(2));
    VERIFY_IS_EQUAL(v1.template head<2>(), v1.segment(0, 2));
    VERIFY_IS_EQUAL(v1.template head<2>(), v1.template segment<2>(0));
    Index i = rows - 2;
    VERIFY_IS_EQUAL(v1.template tail<2>(), v1.block(i, 0, 2, 1));
    VERIFY_IS_EQUAL(v1.template tail<2>(), v1.tail(2));
    VERIFY_IS_EQUAL(v1.template tail<2>(), v1.segment(i, 2));
    VERIFY_IS_EQUAL(v1.template tail<2>(), v1.template segment<2>(i));
    i = internal::random<Index>(0, rows - 2);
    VERIFY_IS_EQUAL(v1.segment(i, 2), v1.template segment<2>(i));
  }

  // stress some basic stuffs with block matrices
  VERIFY_IS_EQUAL(numext::real(ones.col(c1).sum()), RealScalar(rows));
  VERIFY_IS_EQUAL(numext::real(ones.row(r1).sum()), RealScalar(cols));

  VERIFY_IS_EQUAL(numext::real(ones.col(c1).dot(ones.col(c2))), RealScalar(rows));
  VERIFY_IS_EQUAL(numext::real(ones.row(r1).dot(ones.row(r2))), RealScalar(cols));

  // check that linear accessors works on blocks
  m1 = m1_copy;

  // now test some block-inside-of-block.

  // expressions with direct access
  VERIFY_IS_EQUAL((m1.block(r1, c1, rows - r1, cols - c1).block(r2 - r1, c2 - c1, rows - r2, cols - c2)),
                  (m1.block(r2, c2, rows - r2, cols - c2)));
  VERIFY_IS_EQUAL((m1.block(r1, c1, r2 - r1 + 1, c2 - c1 + 1).row(0)), (m1.row(r1).segment(c1, c2 - c1 + 1)));
  VERIFY_IS_EQUAL((m1.block(r1, c1, r2 - r1 + 1, c2 - c1 + 1).col(0)), (m1.col(c1).segment(r1, r2 - r1 + 1)));
  VERIFY_IS_EQUAL((m1.block(r1, c1, r2 - r1 + 1, c2 - c1 + 1).transpose().col(0)),
                  (m1.row(r1).segment(c1, c2 - c1 + 1)).transpose());
  VERIFY_IS_EQUAL((m1.transpose().block(c1, r1, c2 - c1 + 1, r2 - r1 + 1).col(0)),
                  (m1.row(r1).segment(c1, c2 - c1 + 1)).transpose());

  // expressions without direct access
  VERIFY_IS_APPROX(((m1 + m2).block(r1, c1, rows - r1, cols - c1).block(r2 - r1, c2 - c1, rows - r2, cols - c2)),
                   ((m1 + m2).block(r2, c2, rows - r2, cols - c2)));
  VERIFY_IS_APPROX(((m1 + m2).block(r1, c1, r2 - r1 + 1, c2 - c1 + 1).row(0)),
                   ((m1 + m2).row(r1).segment(c1, c2 - c1 + 1)));
  VERIFY_IS_APPROX(((m1 + m2).block(r1, c1, r2 - r1 + 1, c2 - c1 + 1).row(0)),
                   ((m1 + m2).eval().row(r1).segment(c1, c2 - c1 + 1)));
  VERIFY_IS_APPROX(((m1 + m2).block(r1, c1, r2 - r1 + 1, c2 - c1 + 1).col(0)),
                   ((m1 + m2).col(c1).segment(r1, r2 - r1 + 1)));
  VERIFY_IS_APPROX(((m1 + m2).block(r1, c1, r2 - r1 + 1, c2 - c1 + 1).transpose().col(0)),
                   ((m1 + m2).row(r1).segment(c1, c2 - c1 + 1)).transpose());
  VERIFY_IS_APPROX(((m1 + m2).transpose().block(c1, r1, c2 - c1 + 1, r2 - r1 + 1).col(0)),
                   ((m1 + m2).row(r1).segment(c1, c2 - c1 + 1)).transpose());
  VERIFY_IS_APPROX(((m1 + m2).template block<Dynamic, 1>(r1, c1, r2 - r1 + 1, 1)),
                   ((m1 + m2).eval().col(c1).eval().segment(r1, r2 - r1 + 1)));
  VERIFY_IS_APPROX(((m1 + m2).template block<1, Dynamic>(r1, c1, 1, c2 - c1 + 1)),
                   ((m1 + m2).eval().row(r1).eval().segment(c1, c2 - c1 + 1)));
  VERIFY_IS_APPROX(((m1 + m2).transpose().template block<1, Dynamic>(c1, r1, 1, r2 - r1 + 1)),
                   ((m1 + m2).eval().col(c1).eval().segment(r1, r2 - r1 + 1)).transpose());
  VERIFY_IS_APPROX((m1 + m2).row(r1).eval(), (m1 + m2).eval().row(r1));
  VERIFY_IS_APPROX((m1 + m2).adjoint().col(r1).eval(), (m1 + m2).adjoint().eval().col(r1));
  VERIFY_IS_APPROX((m1 + m2).adjoint().row(c1).eval(), (m1 + m2).adjoint().eval().row(c1));
  VERIFY_IS_APPROX((m1 * 1).row(r1).segment(c1, c2 - c1 + 1).eval(), m1.row(r1).eval().segment(c1, c2 - c1 + 1).eval());
  VERIFY_IS_APPROX(m1.col(c1).reverse().segment(r1, r2 - r1 + 1).eval(),
                   m1.col(c1).reverse().eval().segment(r1, r2 - r1 + 1).eval());

  VERIFY_IS_APPROX((m1 * 1).topRows(r1), m1.topRows(r1));
  VERIFY_IS_APPROX((m1 * 1).leftCols(c1), m1.leftCols(c1));
  VERIFY_IS_APPROX((m1 * 1).transpose().topRows(c1), m1.transpose().topRows(c1));
  VERIFY_IS_APPROX((m1 * 1).transpose().leftCols(r1), m1.transpose().leftCols(r1));
  VERIFY_IS_APPROX((m1 * 1).transpose().middleRows(c1, c2 - c1 + 1), m1.transpose().middleRows(c1, c2 - c1 + 1));
  VERIFY_IS_APPROX((m1 * 1).transpose().middleCols(r1, r2 - r1 + 1), m1.transpose().middleCols(r1, r2 - r1 + 1));

  // evaluation into plain matrices from expressions with direct access (stress MapBase)
  DynamicMatrixType dm;
  DynamicVectorType dv;
  dm.setZero();
  dm = m1.block(r1, c1, rows - r1, cols - c1).block(r2 - r1, c2 - c1, rows - r2, cols - c2);
  VERIFY_IS_EQUAL(dm, (m1.block(r2, c2, rows - r2, cols - c2)));
  dm.setZero();
  dv.setZero();
  dm = m1.block(r1, c1, r2 - r1 + 1, c2 - c1 + 1).row(0).transpose();
  dv = m1.row(r1).segment(c1, c2 - c1 + 1);
  VERIFY_IS_EQUAL(dv, dm);
  dm.setZero();
  dv.setZero();
  dm = m1.col(c1).segment(r1, r2 - r1 + 1);
  dv = m1.block(r1, c1, r2 - r1 + 1, c2 - c1 + 1).col(0);
  VERIFY_IS_EQUAL(dv, dm);
  dm.setZero();
  dv.setZero();
  dm = m1.block(r1, c1, r2 - r1 + 1, c2 - c1 + 1).transpose().col(0);
  dv = m1.row(r1).segment(c1, c2 - c1 + 1);
  VERIFY_IS_EQUAL(dv, dm);
  dm.setZero();
  dv.setZero();
  dm = m1.row(r1).segment(c1, c2 - c1 + 1).transpose();
  dv = m1.transpose().block(c1, r1, c2 - c1 + 1, r2 - r1 + 1).col(0);
  VERIFY_IS_EQUAL(dv, dm);

  VERIFY_IS_EQUAL((m1.template block<Dynamic, 1>(1, 0, 0, 1)), m1.block(1, 0, 0, 1));
  VERIFY_IS_EQUAL((m1.template block<1, Dynamic>(0, 1, 1, 0)), m1.block(0, 1, 1, 0));
  VERIFY_IS_EQUAL(((m1 * 1).template block<Dynamic, 1>(1, 0, 0, 1)), m1.block(1, 0, 0, 1));
  VERIFY_IS_EQUAL(((m1 * 1).template block<1, Dynamic>(0, 1, 1, 0)), m1.block(0, 1, 1, 0));

  VERIFY_IS_EQUAL(m1.template subVector<Horizontal>(r1), m1.row(r1));
  VERIFY_IS_APPROX((m1 + m1).template subVector<Horizontal>(r1), (m1 + m1).row(r1));
  VERIFY_IS_EQUAL(m1.template subVector<Vertical>(c1), m1.col(c1));
  VERIFY_IS_APPROX((m1 + m1).template subVector<Vertical>(c1), (m1 + m1).col(c1));
  VERIFY_IS_EQUAL(m1.template subVectors<Horizontal>(), m1.rows());
  VERIFY_IS_EQUAL(m1.template subVectors<Vertical>(), m1.cols());

  if (rows >= 2 || cols >= 2) {
    VERIFY_IS_EQUAL(int(m1.middleCols(0, 0).IsRowMajor), int(m1.IsRowMajor));
    VERIFY_IS_EQUAL(m1.middleCols(0, 0).outerSize(), m1.IsRowMajor ? rows : 0);
    VERIFY_IS_EQUAL(m1.middleCols(0, 0).innerSize(), m1.IsRowMajor ? 0 : rows);

    VERIFY_IS_EQUAL(int(m1.middleRows(0, 0).IsRowMajor), int(m1.IsRowMajor));
    VERIFY_IS_EQUAL(m1.middleRows(0, 0).outerSize(), m1.IsRowMajor ? 0 : cols);
    VERIFY_IS_EQUAL(m1.middleRows(0, 0).innerSize(), m1.IsRowMajor ? cols : 0);
  }
}

template <typename MatrixType>
std::enable_if_t<MatrixType::IsVectorAtCompileTime, void> compare_using_data_and_stride(const MatrixType& m) {
  Index rows = m.rows();
  Index cols = m.cols();
  Index size = m.size();
  Index innerStride = m.innerStride();
  Index rowStride = m.rowStride();
  Index colStride = m.colStride();
  const typename MatrixType::Scalar* data = m.data();

  for (int j = 0; j < cols; ++j)
    for (int i = 0; i < rows; ++i) VERIFY(m.coeff(i, j) == data[i * rowStride + j * colStride]);

  VERIFY(innerStride == int((&m.coeff(1)) - (&m.coeff(0))));
  for (int i = 0; i < size; ++i) VERIFY(m.coeff(i) == data[i * innerStride]);
}

template <typename MatrixType>
std::enable_if_t<!MatrixType::IsVectorAtCompileTime, void> compare_using_data_and_stride(const MatrixType& m) {
  Index rows = m.rows();
  Index cols = m.cols();
  Index innerStride = m.innerStride();
  Index outerStride = m.outerStride();
  Index rowStride = m.rowStride();
  Index colStride = m.colStride();
  const typename MatrixType::Scalar* data = m.data();

  for (int j = 0; j < cols; ++j)
    for (int i = 0; i < rows; ++i) VERIFY(m.coeff(i, j) == data[i * rowStride + j * colStride]);

  for (int j = 0; j < cols; ++j)
    for (int i = 0; i < rows; ++i)
      VERIFY(m.coeff(i, j) == data[(MatrixType::Flags & RowMajorBit) ? i * outerStride + j * innerStride
                                                                     : j * outerStride + i * innerStride]);
}

template <typename MatrixType>
void data_and_stride(const MatrixType& m) {
  Index rows = m.rows();
  Index cols = m.cols();

  Index r1 = internal::random<Index>(0, rows - 1);
  Index r2 = internal::random<Index>(r1, rows - 1);
  Index c1 = internal::random<Index>(0, cols - 1);
  Index c2 = internal::random<Index>(c1, cols - 1);

  MatrixType m1 = MatrixType::Random(rows, cols);
  compare_using_data_and_stride(m1.block(r1, c1, r2 - r1 + 1, c2 - c1 + 1));
  compare_using_data_and_stride(m1.transpose().block(c1, r1, c2 - c1 + 1, r2 - r1 + 1));
  compare_using_data_and_stride(m1.row(r1));
  compare_using_data_and_stride(m1.col(c1));
  compare_using_data_and_stride(m1.row(r1).transpose());
  compare_using_data_and_stride(m1.col(c1).transpose());
}

template <typename BaseXpr, typename Xpr = BaseXpr, int Depth = 0>
struct unwind_test_impl {
  static void run(Xpr& xpr) {
    Index startRow = internal::random<Index>(0, xpr.rows() / 5);
    Index startCol = internal::random<Index>(0, xpr.cols() / 6);
    Index rows = xpr.rows() / 3;
    Index cols = xpr.cols() / 2;
    // test equivalence of const expressions
    const Block<const Xpr> constNestedBlock(xpr, startRow, startCol, rows, cols);
    const Block<const BaseXpr> constUnwoundBlock = constNestedBlock.unwind();
    VERIFY_IS_CWISE_EQUAL(constNestedBlock, constUnwoundBlock);
    // modify a random element in each representation and test equivalence of non-const expressions
    Block<Xpr> nestedBlock(xpr, startRow, startCol, rows, cols);
    Block<BaseXpr> unwoundBlock = nestedBlock.unwind();
    Index r1 = internal::random<Index>(0, rows - 1);
    Index c1 = internal::random<Index>(0, cols - 1);
    Index r2 = internal::random<Index>(0, rows - 1);
    Index c2 = internal::random<Index>(0, cols - 1);
    nestedBlock.coeffRef(r1, c1) = internal::random<typename DenseBase<Xpr>::Scalar>();
    unwoundBlock.coeffRef(r2, c2) = internal::random<typename DenseBase<Xpr>::Scalar>();
    VERIFY_IS_CWISE_EQUAL(nestedBlock, unwoundBlock);
    unwind_test_impl<BaseXpr, Block<Xpr>, Depth + 1>::run(nestedBlock);
  }
};

template <typename BaseXpr, typename Xpr>
struct unwind_test_impl<BaseXpr, Xpr, 4> {
  static void run(const Xpr&) {}
};

template <typename BaseXpr>
void unwind_test(const BaseXpr&) {
  BaseXpr xpr = BaseXpr::Random(100, 100);
  unwind_test_impl<BaseXpr>::run(xpr);
}

// A block that is an inner panel of a general sub-block is not an inner panel of
// the sub-block's base expression: it spans the full inner dimension of its
// immediate parent, but not that of the base. Unwinding must not promote it to
// one, since an inner panel advertises linear access over the base's storage.
template <typename BaseXpr>
void unwind_inner_panel_test(const BaseXpr&) {
  const Index rows = 20, cols = 20;
  BaseXpr xpr = BaseXpr::Random(rows, cols);

  // The parent spans neither dimension fully, so neither panel taken from it is
  // an inner panel of `xpr`. Copying out through a plain object exercises this:
  // an unwound block that wrongly claims to be an inner panel is traversed
  // linearly and reads the wrong coefficients.
  auto sub = xpr.block(1, 1, rows - 3, cols - 3);
  auto sub_cols = sub.middleCols(2, 3);
  auto sub_rows = sub.middleRows(2, 3);
  VERIFY_IS_EQUAL(int(internal::traits<decltype(sub_cols.unwind())>::InnerPanel), 0);
  VERIFY_IS_EQUAL(int(internal::traits<decltype(sub_rows.unwind())>::InnerPanel), 0);
  BaseXpr sub_cols_ref = sub_cols, sub_cols_unwound = sub_cols.unwind();
  VERIFY_IS_CWISE_EQUAL(sub_cols_ref, sub_cols_unwound);
  BaseXpr sub_rows_ref = sub_rows, sub_rows_unwound = sub_rows.unwind();
  VERIFY_IS_CWISE_EQUAL(sub_rows_ref, sub_rows_unwound);

  // An inner panel of an inner panel *is* an inner panel of the base, and must
  // keep saying so after unwinding.
  auto inner_cols = xpr.middleCols(1, cols - 3).middleCols(2, 3);
  VERIFY_IS_EQUAL(int(internal::traits<decltype(inner_cols.unwind())>::InnerPanel), int(BaseXpr::IsRowMajor ? 0 : 1));
  BaseXpr inner_cols_ref = inner_cols, inner_cols_unwound = inner_cols.unwind();
  VERIFY_IS_CWISE_EQUAL(inner_cols_ref, inner_cols_unwound);

  auto inner_rows = xpr.middleRows(1, rows - 3).middleRows(2, 3);
  VERIFY_IS_EQUAL(int(internal::traits<decltype(inner_rows.unwind())>::InnerPanel), int(BaseXpr::IsRowMajor ? 1 : 0));
  BaseXpr inner_rows_ref = inner_rows, inner_rows_unwound = inner_rows.unwind();
  VERIFY_IS_CWISE_EQUAL(inner_rows_ref, inner_rows_unwound);
}

EIGEN_DECLARE_TEST(block) {
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(block(Matrix<float, 1, 1>()));
    CALL_SUBTEST_10(block(Matrix<float, 1, Dynamic>(internal::random(2, 50))));
    CALL_SUBTEST_11(block(Matrix<float, Dynamic, 1>(internal::random(2, 50))));
    CALL_SUBTEST_2(block(Matrix4d()));
    CALL_SUBTEST_3(block(MatrixXcf(internal::random(2, 50), internal::random(2, 50))));
    CALL_SUBTEST_4(block(MatrixXi(internal::random(2, 50), internal::random(2, 50))));
    CALL_SUBTEST_5(block(MatrixXcd(internal::random(2, 50), internal::random(2, 50))));
    CALL_SUBTEST_6(block(MatrixXf(internal::random(2, 50), internal::random(2, 50))));
    CALL_SUBTEST_7(block(Matrix<int, Dynamic, Dynamic, RowMajor>(internal::random(2, 50), internal::random(2, 50))));

    CALL_SUBTEST_8(block(Matrix<float, Dynamic, 4>(3, 4)));
    CALL_SUBTEST_9(unwind_test(MatrixXf()));
    CALL_SUBTEST_9(unwind_inner_panel_test(MatrixXf()));
    CALL_SUBTEST_9(unwind_inner_panel_test(Matrix<float, Dynamic, Dynamic, RowMajor>()));

#ifndef EIGEN_DEFAULT_TO_ROW_MAJOR
    CALL_SUBTEST_6(data_and_stride(MatrixXf(internal::random(5, 50), internal::random(5, 50))));
    CALL_SUBTEST_7(
        data_and_stride(Matrix<int, Dynamic, Dynamic, RowMajor>(internal::random(5, 50), internal::random(5, 50))));
#endif
  }
}
