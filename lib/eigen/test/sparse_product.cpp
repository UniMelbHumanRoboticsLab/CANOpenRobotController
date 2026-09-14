// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2011 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

static long int nb_temporaries;

inline void on_temporary_creation() {
  // here's a great place to set a breakpoint when debugging failures in this test!
  nb_temporaries++;
}

#define EIGEN_SPARSE_CREATE_TEMPORARY_PLUGIN \
  { on_temporary_creation(); }

#include "sparse.h"

#define EIGEN_TEST_ANNOYING_SCALAR_DONT_THROW
#include "AnnoyingScalar.h"

#define VERIFY_EVALUATION_COUNT(XPR, N)                                                   \
  {                                                                                       \
    nb_temporaries = 0;                                                                   \
    CALL_SUBTEST(XPR);                                                                    \
    if (nb_temporaries != N) std::cerr << "nb_temporaries == " << nb_temporaries << "\n"; \
    VERIFY((#XPR) && nb_temporaries == N);                                                \
  }

template <typename Lhs, typename Rhs, typename = void>
struct has_product : std::false_type {};

template <typename Lhs, typename Rhs>
struct has_product<Lhs, Rhs, internal::void_t<decltype(std::declval<const Lhs&>() * std::declval<const Rhs&>())>>
    : std::true_type {};

template <typename SparseMatrixType>
void sparse_structured_view_product_sfinae() {
  typedef typename SparseMatrixType::Scalar Scalar;
  typedef Matrix<Scalar, Dynamic, Dynamic> DenseMatrixType;
  typedef Matrix<Scalar, Dynamic, 1> DenseVectorType;
  typedef decltype(std::declval<SparseMatrixType&>().template triangularView<Lower>()) TriangularViewType;
  typedef decltype(std::declval<SparseMatrixType&>().template selfadjointView<Lower>()) SelfAdjointViewType;
  typedef decltype(std::declval<const DenseVectorType&>().asDiagonal()) DiagonalType;

  STATIC_CHECK((has_product<TriangularViewType, SparseMatrixType>::value));
  STATIC_CHECK((has_product<SparseMatrixType, TriangularViewType>::value));
  STATIC_CHECK((has_product<TriangularViewType, DenseMatrixType>::value));
  STATIC_CHECK((has_product<DenseMatrixType, TriangularViewType>::value));
  STATIC_CHECK((has_product<TriangularViewType, DiagonalType>::value));
  STATIC_CHECK((has_product<DiagonalType, TriangularViewType>::value));

  STATIC_CHECK((has_product<SelfAdjointViewType, SparseMatrixType>::value));
  STATIC_CHECK((has_product<SparseMatrixType, SelfAdjointViewType>::value));
  STATIC_CHECK((has_product<SelfAdjointViewType, DenseMatrixType>::value));
  STATIC_CHECK((has_product<DenseMatrixType, SelfAdjointViewType>::value));
  STATIC_CHECK((has_product<SelfAdjointViewType, DiagonalType>::value));
  STATIC_CHECK((has_product<DiagonalType, SelfAdjointViewType>::value));
}

template <typename SparseMatrixType>
void sparse_product() {
  typedef typename SparseMatrixType::StorageIndex StorageIndex;
  Index n = 100;
  const Index rows = internal::random<Index>(1, n);
  const Index cols = internal::random<Index>(1, n);
  const Index depth = internal::random<Index>(1, n);
  typedef typename SparseMatrixType::Scalar Scalar;
  enum { Flags = SparseMatrixType::Flags };

  double density = (std::max)(8. / (rows * cols), 0.2);
  typedef Matrix<Scalar, Dynamic, Dynamic> DenseMatrix;
  typedef Matrix<Scalar, Dynamic, 1> DenseVector;
  typedef Matrix<Scalar, 1, Dynamic> RowDenseVector;
  typedef SparseVector<Scalar, 0, StorageIndex> ColSpVector;
  typedef SparseVector<Scalar, RowMajor, StorageIndex> RowSpVector;

  Scalar s1 = internal::random<Scalar>();
  Scalar s2 = internal::random<Scalar>();

  // test matrix-matrix product
  {
    DenseMatrix refMat2 = DenseMatrix::Zero(rows, depth);
    DenseMatrix refMat2t = DenseMatrix::Zero(depth, rows);
    DenseMatrix refMat3 = DenseMatrix::Zero(depth, cols);
    DenseMatrix refMat3t = DenseMatrix::Zero(cols, depth);
    DenseMatrix refMat4 = DenseMatrix::Zero(rows, cols);
    DenseMatrix refMat4t = DenseMatrix::Zero(cols, rows);
    DenseMatrix refMat5 = DenseMatrix::Random(depth, cols);
    DenseMatrix refMat6 = DenseMatrix::Random(rows, rows);
    DenseMatrix dm4 = DenseMatrix::Zero(rows, rows);
    //     DenseVector dv1 = DenseVector::Random(rows);
    SparseMatrixType m2(rows, depth);
    SparseMatrixType m2t(depth, rows);
    SparseMatrixType m3(depth, cols);
    SparseMatrixType m3t(cols, depth);
    SparseMatrixType m4(rows, cols);
    SparseMatrixType m4t(cols, rows);
    SparseMatrixType m6(rows, rows);
    initSparse(density, refMat2, m2);
    initSparse(density, refMat2t, m2t);
    initSparse(density, refMat3, m3);
    initSparse(density, refMat3t, m3t);
    initSparse(density, refMat4, m4);
    initSparse(density, refMat4t, m4t);
    initSparse(density, refMat6, m6);

    //     int c = internal::random<int>(0,depth-1);

    // sparse * sparse
    VERIFY_IS_APPROX(m4 = m2 * m3, refMat4 = refMat2 * refMat3);
    VERIFY_IS_APPROX(m4 = m2t.transpose() * m3, refMat4 = refMat2t.transpose() * refMat3);
    VERIFY_IS_APPROX(m4 = m2t.transpose() * m3t.transpose(), refMat4 = refMat2t.transpose() * refMat3t.transpose());
    VERIFY_IS_APPROX(m4 = m2 * m3t.transpose(), refMat4 = refMat2 * refMat3t.transpose());

    VERIFY_IS_APPROX(m4 = m2 * m3 / s1, refMat4 = refMat2 * refMat3 / s1);
    VERIFY_IS_APPROX(m4 = m2 * m3 * s1, refMat4 = refMat2 * refMat3 * s1);
    VERIFY_IS_APPROX(m4 = s2 * m2 * m3 * s1, refMat4 = s2 * refMat2 * refMat3 * s1);
    VERIFY_IS_APPROX(m4 = (m2 + m2) * m3, refMat4 = (refMat2 + refMat2) * refMat3);
    VERIFY_IS_APPROX(m4 = m2 * m3.leftCols(cols / 2), refMat4 = refMat2 * refMat3.leftCols(cols / 2));
    VERIFY_IS_APPROX(m4 = m2 * (m3 + m3).leftCols(cols / 2),
                     refMat4 = refMat2 * (refMat3 + refMat3).leftCols(cols / 2));

    VERIFY_IS_APPROX(m4 = (m2 * m3).pruned(0), refMat4 = refMat2 * refMat3);
    VERIFY_IS_APPROX(m4 = (m2t.transpose() * m3).pruned(0), refMat4 = refMat2t.transpose() * refMat3);
    VERIFY_IS_APPROX(m4 = (m2t.transpose() * m3t.transpose()).pruned(0),
                     refMat4 = refMat2t.transpose() * refMat3t.transpose());
    VERIFY_IS_APPROX(m4 = (m2 * m3t.transpose()).pruned(0), refMat4 = refMat2 * refMat3t.transpose());

#ifndef EIGEN_SPARSE_PRODUCT_IGNORE_TEMPORARY_COUNT
    // make sure the right product implementation is called:
    if ((!SparseMatrixType::IsRowMajor) && m2.rows() <= m3.cols()) {
      VERIFY_EVALUATION_COUNT(m4 = m2 * m3, 2);  // 2 for transposing and get a sorted result.
      VERIFY_EVALUATION_COUNT(m4 = (m2 * m3).pruned(0), 1);
      VERIFY_EVALUATION_COUNT(m4 = (m2 * m3).eval().pruned(0), 4);
    }
#endif

    // and that pruning is effective:
    {
      DenseMatrix Ad(2, 2);
      Ad << -1, 1, 1, 1;
      SparseMatrixType As(Ad.sparseView()), B(2, 2);
      VERIFY_IS_EQUAL((As * As.transpose()).eval().nonZeros(), 4);
      VERIFY_IS_EQUAL((Ad * Ad.transpose()).eval().sparseView().eval().nonZeros(), 2);
      VERIFY_IS_EQUAL((As * As.transpose()).pruned(1e-6).eval().nonZeros(), 2);
    }

    // dense ?= sparse * sparse
    VERIFY_IS_APPROX(dm4 = m2 * m3, refMat4 = refMat2 * refMat3);
    VERIFY_IS_APPROX(dm4 += m2 * m3, refMat4 += refMat2 * refMat3);
    VERIFY_IS_APPROX(dm4 -= m2 * m3, refMat4 -= refMat2 * refMat3);
    VERIFY_IS_APPROX(dm4 = m2t.transpose() * m3, refMat4 = refMat2t.transpose() * refMat3);
    VERIFY_IS_APPROX(dm4 += m2t.transpose() * m3, refMat4 += refMat2t.transpose() * refMat3);
    VERIFY_IS_APPROX(dm4 -= m2t.transpose() * m3, refMat4 -= refMat2t.transpose() * refMat3);
    VERIFY_IS_APPROX(dm4 = m2t.transpose() * m3t.transpose(), refMat4 = refMat2t.transpose() * refMat3t.transpose());
    VERIFY_IS_APPROX(dm4 += m2t.transpose() * m3t.transpose(), refMat4 += refMat2t.transpose() * refMat3t.transpose());
    VERIFY_IS_APPROX(dm4 -= m2t.transpose() * m3t.transpose(), refMat4 -= refMat2t.transpose() * refMat3t.transpose());
    VERIFY_IS_APPROX(dm4 = m2 * m3t.transpose(), refMat4 = refMat2 * refMat3t.transpose());
    VERIFY_IS_APPROX(dm4 += m2 * m3t.transpose(), refMat4 += refMat2 * refMat3t.transpose());
    VERIFY_IS_APPROX(dm4 -= m2 * m3t.transpose(), refMat4 -= refMat2 * refMat3t.transpose());
    VERIFY_IS_APPROX(dm4 = m2 * m3 * s1, refMat4 = refMat2 * refMat3 * s1);

    // test aliasing
    m4 = m2;
    refMat4 = refMat2;
    VERIFY_IS_APPROX(m4 = m4 * m3, refMat4 = refMat4 * refMat3);

    // sparse * dense matrix
    VERIFY_IS_APPROX(dm4 = m2 * refMat3, refMat4 = refMat2 * refMat3);
    VERIFY_IS_APPROX(dm4 = m2 * refMat3t.transpose(), refMat4 = refMat2 * refMat3t.transpose());
    VERIFY_IS_APPROX(dm4 = m2t.transpose() * refMat3, refMat4 = refMat2t.transpose() * refMat3);
    VERIFY_IS_APPROX(dm4 = m2t.transpose() * refMat3t.transpose(),
                     refMat4 = refMat2t.transpose() * refMat3t.transpose());

    VERIFY_IS_APPROX(dm4 = m2 * refMat3, refMat4 = refMat2 * refMat3);
    VERIFY_IS_APPROX(dm4 = dm4 + m2 * refMat3, refMat4 = refMat4 + refMat2 * refMat3);
    VERIFY_IS_APPROX(dm4 += m2 * refMat3, refMat4 += refMat2 * refMat3);
    VERIFY_IS_APPROX(dm4 -= m2 * refMat3, refMat4 -= refMat2 * refMat3);
    VERIFY_IS_APPROX(dm4.noalias() += m2 * refMat3, refMat4 += refMat2 * refMat3);
    VERIFY_IS_APPROX(dm4.noalias() -= m2 * refMat3, refMat4 -= refMat2 * refMat3);
    VERIFY_IS_APPROX(dm4 = m2 * (refMat3 + refMat3), refMat4 = refMat2 * (refMat3 + refMat3));
    VERIFY_IS_APPROX(dm4 = m2t.transpose() * (refMat3 + refMat5) * 0.5,
                     refMat4 = refMat2t.transpose() * (refMat3 + refMat5) * 0.5);

    // sparse * dense expression without DirectAccessBit (e.g. CwiseNullaryOp)
    VERIFY_IS_APPROX(dm4 = m2 * DenseMatrix::Constant(depth, cols, s1),
                     refMat4 = refMat2 * DenseMatrix::Constant(depth, cols, s1));
    VERIFY_IS_APPROX(dm4 = m2 * DenseMatrix::Zero(depth, cols), refMat4 = refMat2 * DenseMatrix::Zero(depth, cols));

    // sparse * dense vector
    VERIFY_IS_APPROX(dm4.col(0) = m2 * refMat3.col(0), refMat4.col(0) = refMat2 * refMat3.col(0));
    VERIFY_IS_APPROX(dm4.col(0) = m2 * refMat3t.transpose().col(0),
                     refMat4.col(0) = refMat2 * refMat3t.transpose().col(0));
    VERIFY_IS_APPROX(dm4.col(0) = m2t.transpose() * refMat3.col(0),
                     refMat4.col(0) = refMat2t.transpose() * refMat3.col(0));
    VERIFY_IS_APPROX(dm4.col(0) = m2t.transpose() * refMat3t.transpose().col(0),
                     refMat4.col(0) = refMat2t.transpose() * refMat3t.transpose().col(0));

    // dense * sparse
    VERIFY_IS_APPROX(dm4 = refMat2 * m3, refMat4 = refMat2 * refMat3);
    VERIFY_IS_APPROX(dm4 = dm4 + refMat2 * m3, refMat4 = refMat4 + refMat2 * refMat3);
    VERIFY_IS_APPROX(dm4 += refMat2 * m3, refMat4 += refMat2 * refMat3);
    VERIFY_IS_APPROX(dm4 -= refMat2 * m3, refMat4 -= refMat2 * refMat3);
    VERIFY_IS_APPROX(dm4.noalias() += refMat2 * m3, refMat4 += refMat2 * refMat3);
    VERIFY_IS_APPROX(dm4.noalias() -= refMat2 * m3, refMat4 -= refMat2 * refMat3);
    VERIFY_IS_APPROX(dm4 = refMat2 * m3t.transpose(), refMat4 = refMat2 * refMat3t.transpose());
    VERIFY_IS_APPROX(dm4 = refMat2t.transpose() * m3, refMat4 = refMat2t.transpose() * refMat3);
    VERIFY_IS_APPROX(dm4 = refMat2t.transpose() * m3t.transpose(),
                     refMat4 = refMat2t.transpose() * refMat3t.transpose());

    // sparse * dense and dense * sparse outer product
    {
      Index c = internal::random<Index>(0, depth - 1);
      Index r = internal::random<Index>(0, rows - 1);
      Index c1 = internal::random<Index>(0, cols - 1);
      Index r1 = internal::random<Index>(0, depth - 1);
      DenseMatrix dm5 = DenseMatrix::Random(depth, cols);

      VERIFY_IS_APPROX(m4 = m2.col(c) * dm5.col(c1).transpose(), refMat4 = refMat2.col(c) * dm5.col(c1).transpose());
      VERIFY_IS_EQUAL(m4.nonZeros(), (refMat4.array() != 0).count());
      VERIFY_IS_APPROX(m4 = m2.middleCols(c, 1) * dm5.col(c1).transpose(),
                       refMat4 = refMat2.col(c) * dm5.col(c1).transpose());
      VERIFY_IS_EQUAL(m4.nonZeros(), (refMat4.array() != 0).count());
      VERIFY_IS_APPROX(dm4 = m2.col(c) * dm5.col(c1).transpose(), refMat4 = refMat2.col(c) * dm5.col(c1).transpose());

      VERIFY_IS_APPROX(m4 = dm5.col(c1) * m2.col(c).transpose(), refMat4 = dm5.col(c1) * refMat2.col(c).transpose());
      VERIFY_IS_EQUAL(m4.nonZeros(), (refMat4.array() != 0).count());
      VERIFY_IS_APPROX(m4 = dm5.col(c1) * m2.middleCols(c, 1).transpose(),
                       refMat4 = dm5.col(c1) * refMat2.col(c).transpose());
      VERIFY_IS_EQUAL(m4.nonZeros(), (refMat4.array() != 0).count());
      VERIFY_IS_APPROX(dm4 = dm5.col(c1) * m2.col(c).transpose(), refMat4 = dm5.col(c1) * refMat2.col(c).transpose());

      VERIFY_IS_APPROX(m4 = dm5.row(r1).transpose() * m2.col(c).transpose(),
                       refMat4 = dm5.row(r1).transpose() * refMat2.col(c).transpose());
      VERIFY_IS_EQUAL(m4.nonZeros(), (refMat4.array() != 0).count());
      VERIFY_IS_APPROX(dm4 = dm5.row(r1).transpose() * m2.col(c).transpose(),
                       refMat4 = dm5.row(r1).transpose() * refMat2.col(c).transpose());

      VERIFY_IS_APPROX(m4 = m2.row(r).transpose() * dm5.col(c1).transpose(),
                       refMat4 = refMat2.row(r).transpose() * dm5.col(c1).transpose());
      VERIFY_IS_EQUAL(m4.nonZeros(), (refMat4.array() != 0).count());
      VERIFY_IS_APPROX(m4 = m2.middleRows(r, 1).transpose() * dm5.col(c1).transpose(),
                       refMat4 = refMat2.row(r).transpose() * dm5.col(c1).transpose());
      VERIFY_IS_EQUAL(m4.nonZeros(), (refMat4.array() != 0).count());
      VERIFY_IS_APPROX(dm4 = m2.row(r).transpose() * dm5.col(c1).transpose(),
                       refMat4 = refMat2.row(r).transpose() * dm5.col(c1).transpose());

      VERIFY_IS_APPROX(m4 = dm5.col(c1) * m2.row(r), refMat4 = dm5.col(c1) * refMat2.row(r));
      VERIFY_IS_EQUAL(m4.nonZeros(), (refMat4.array() != 0).count());
      VERIFY_IS_APPROX(m4 = dm5.col(c1) * m2.middleRows(r, 1), refMat4 = dm5.col(c1) * refMat2.row(r));
      VERIFY_IS_EQUAL(m4.nonZeros(), (refMat4.array() != 0).count());
      VERIFY_IS_APPROX(dm4 = dm5.col(c1) * m2.row(r), refMat4 = dm5.col(c1) * refMat2.row(r));

      VERIFY_IS_APPROX(m4 = dm5.row(r1).transpose() * m2.row(r), refMat4 = dm5.row(r1).transpose() * refMat2.row(r));
      VERIFY_IS_EQUAL(m4.nonZeros(), (refMat4.array() != 0).count());
      VERIFY_IS_APPROX(dm4 = dm5.row(r1).transpose() * m2.row(r), refMat4 = dm5.row(r1).transpose() * refMat2.row(r));
    }

    VERIFY_IS_APPROX(m6 = m6 * m6, refMat6 = refMat6 * refMat6);

    // sparse matrix * sparse vector
    ColSpVector cv0(cols), cv1;
    DenseVector dcv0(cols), dcv1;
    initSparse(2 * density, dcv0, cv0);

    RowSpVector rv0(depth), rv1;
    RowDenseVector drv0(depth), drv1(rv1);
    initSparse(2 * density, drv0, rv0);

    VERIFY_IS_APPROX(cv1 = m3 * cv0, dcv1 = refMat3 * dcv0);
    VERIFY_IS_APPROX(rv1 = rv0 * m3, drv1 = drv0 * refMat3);
    VERIFY_IS_APPROX(cv1 = m3t.adjoint() * cv0, dcv1 = refMat3t.adjoint() * dcv0);
    VERIFY_IS_APPROX(cv1 = rv0 * m3, dcv1 = drv0 * refMat3);
    VERIFY_IS_APPROX(rv1 = m3 * cv0, drv1 = refMat3 * dcv0);
  }

  // test matrix - diagonal product
  {
    DenseMatrix refM2 = DenseMatrix::Zero(rows, cols);
    DenseMatrix refM3 = DenseMatrix::Zero(rows, cols);
    DenseMatrix d3 = DenseMatrix::Zero(rows, cols);
    DiagonalMatrix<Scalar, Dynamic> d1(DenseVector::Random(cols));
    DiagonalMatrix<Scalar, Dynamic> d2(DenseVector::Random(rows));
    SparseMatrixType m2(rows, cols);
    SparseMatrixType m3(rows, cols);
    initSparse<Scalar>(density, refM2, m2);
    initSparse<Scalar>(density, refM3, m3);
    VERIFY_IS_APPROX(m3 = m2 * d1, refM3 = refM2 * d1);
    VERIFY_IS_APPROX(m3 = m2.transpose() * d2, refM3 = refM2.transpose() * d2);
    VERIFY_IS_APPROX(m3 = d2 * m2, refM3 = d2 * refM2);
    VERIFY_IS_APPROX(m3 = d1 * m2.transpose(), refM3 = d1 * refM2.transpose());

    // also check with a SparseWrapper:
    DenseVector v1 = DenseVector::Random(cols);
    DenseVector v2 = DenseVector::Random(rows);
    DenseVector v3 = DenseVector::Random(rows);
    VERIFY_IS_APPROX(m3 = m2 * v1.asDiagonal(), refM3 = refM2 * v1.asDiagonal());
    VERIFY_IS_APPROX(m3 = m2.transpose() * v2.asDiagonal(), refM3 = refM2.transpose() * v2.asDiagonal());
    VERIFY_IS_APPROX(m3 = v2.asDiagonal() * m2, refM3 = v2.asDiagonal() * refM2);
    VERIFY_IS_APPROX(m3 = v1.asDiagonal() * m2.transpose(), refM3 = v1.asDiagonal() * refM2.transpose());

    VERIFY_IS_APPROX(m3 = v2.asDiagonal() * m2 * v1.asDiagonal(), refM3 = v2.asDiagonal() * refM2 * v1.asDiagonal());

    VERIFY_IS_APPROX(v2 = m2 * v1.asDiagonal() * v1, refM2 * v1.asDiagonal() * v1);
    VERIFY_IS_APPROX(v3 = v2.asDiagonal() * m2 * v1, v2.asDiagonal() * refM2 * v1);

    // evaluate to a dense matrix to check the .row() and .col() iterator functions
    VERIFY_IS_APPROX(d3 = m2 * d1, refM3 = refM2 * d1);
    VERIFY_IS_APPROX(d3 = m2.transpose() * d2, refM3 = refM2.transpose() * d2);
    VERIFY_IS_APPROX(d3 = d2 * m2, refM3 = d2 * refM2);
    VERIFY_IS_APPROX(d3 = d1 * m2.transpose(), refM3 = d1 * refM2.transpose());
  }

  // test self-adjoint and triangular-view products
  {
    DenseMatrix b = DenseMatrix::Random(rows, rows);
    DenseMatrix x = DenseMatrix::Random(rows, rows);
    DenseMatrix refX = DenseMatrix::Random(rows, rows);
    DenseMatrix refUp = DenseMatrix::Zero(rows, rows);
    DenseMatrix refLo = DenseMatrix::Zero(rows, rows);
    DenseMatrix refS = DenseMatrix::Zero(rows, rows);
    DenseMatrix refA = DenseMatrix::Zero(rows, rows);
    SparseMatrixType mUp(rows, rows);
    SparseMatrixType mLo(rows, rows);
    SparseMatrixType mS(rows, rows);
    SparseMatrixType mA(rows, rows);
    initSparse<Scalar>(density, refA, mA);
    initSparse<Scalar>(density, refUp, mUp, ForceRealDiag | /*ForceNonZeroDiag|*/ MakeUpperTriangular);
    if (refUp.isZero()) {
      refUp(0, 0) = Scalar(1);
      mUp.coeffRef(0, 0) = Scalar(1);
    }
    refLo = refUp.adjoint();
    mLo = mUp.adjoint();
    refS = refUp + refLo;
    refS.diagonal() *= 0.5;
    mS = mUp + mLo;
    // TODO be able to address the diagonal....
    for (int k = 0; k < mS.outerSize(); ++k)
      for (typename SparseMatrixType::InnerIterator it(mS, k); it; ++it)
        if (it.index() == k) it.valueRef() *= Scalar(0.5);

    VERIFY_IS_APPROX(refS.adjoint(), refS);
    VERIFY_IS_APPROX(mS.adjoint(), mS);
    VERIFY_IS_APPROX(mS, refS);
    VERIFY_IS_APPROX(x = mS * b, refX = refS * b);

    // sparse selfadjointView with dense matrices
    VERIFY_IS_APPROX(x = mUp.template selfadjointView<Upper>() * b, refX = refS * b);
    VERIFY_IS_APPROX(x = mLo.template selfadjointView<Lower>() * b, refX = refS * b);
    VERIFY_IS_APPROX(x = mS.template selfadjointView<Upper | Lower>() * b, refX = refS * b);

    VERIFY_IS_APPROX(x = b * mUp.template selfadjointView<Upper>(), refX = b * refS);
    VERIFY_IS_APPROX(x = b * mLo.template selfadjointView<Lower>(), refX = b * refS);
    VERIFY_IS_APPROX(x = b * mS.template selfadjointView<Upper | Lower>(), refX = b * refS);

    VERIFY_IS_APPROX(x.noalias() += mUp.template selfadjointView<Upper>() * b, refX += refS * b);
    VERIFY_IS_APPROX(x.noalias() -= mLo.template selfadjointView<Lower>() * b, refX -= refS * b);
    VERIFY_IS_APPROX(x.noalias() += mS.template selfadjointView<Upper | Lower>() * b, refX += refS * b);

    DenseVector scale = DenseVector::Random(rows);
    VERIFY_IS_APPROX(x = mLo.template selfadjointView<Lower>() * scale.asDiagonal(), refX = refS * scale.asDiagonal());
    VERIFY_IS_APPROX(x = scale.asDiagonal() * mLo.template selfadjointView<Lower>(), refX = scale.asDiagonal() * refS);
    VERIFY_IS_APPROX(x = mUp.template selfadjointView<Upper>() * scale.asDiagonal(), refX = refS * scale.asDiagonal());
    VERIFY_IS_APPROX(x = scale.asDiagonal() * mUp.template selfadjointView<Upper>(), refX = scale.asDiagonal() * refS);
    VERIFY_IS_APPROX(x = mS.template selfadjointView<Upper | Lower>() * scale.asDiagonal(),
                     refX = refS * scale.asDiagonal());
    VERIFY_IS_APPROX(x = scale.asDiagonal() * mS.template selfadjointView<Upper | Lower>(),
                     refX = scale.asDiagonal() * refS);

    // sparse selfadjointView with sparse matrices
    SparseMatrixType mSres(rows, rows);
    VERIFY_IS_APPROX(mSres = mLo.template selfadjointView<Lower>() * mS,
                     refX = refLo.template selfadjointView<Lower>() * refS);
    VERIFY_IS_APPROX(mSres = mS * mLo.template selfadjointView<Lower>(),
                     refX = refS * refLo.template selfadjointView<Lower>());
    VERIFY_IS_APPROX(mSres = mLo.template selfadjointView<Lower>() * scale.asDiagonal(),
                     refX = refS * scale.asDiagonal());
    VERIFY_IS_APPROX(mSres = scale.asDiagonal() * mLo.template selfadjointView<Lower>(),
                     refX = scale.asDiagonal() * refS);
    VERIFY_IS_APPROX(mSres = mUp.template selfadjointView<Upper>() * scale.asDiagonal(),
                     refX = refS * scale.asDiagonal());
    VERIFY_IS_APPROX(mSres = scale.asDiagonal() * mUp.template selfadjointView<Upper>(),
                     refX = scale.asDiagonal() * refS);
    VERIFY_IS_APPROX(mSres = mS.template selfadjointView<Upper | Lower>() * scale.asDiagonal(),
                     refX = refS * scale.asDiagonal());
    VERIFY_IS_APPROX(mSres = scale.asDiagonal() * mS.template selfadjointView<Upper | Lower>(),
                     refX = scale.asDiagonal() * refS);

    // sparse triangularView with dense matrices
    VERIFY_IS_APPROX(x = mA.template triangularView<Upper>() * b, refX = refA.template triangularView<Upper>() * b);
    VERIFY_IS_APPROX(x = mA.template triangularView<Lower>() * b, refX = refA.template triangularView<Lower>() * b);
    VERIFY_IS_APPROX(x = b * mA.template triangularView<Upper>(), refX = b * refA.template triangularView<Upper>());
    VERIFY_IS_APPROX(x = b * mA.template triangularView<Lower>(), refX = b * refA.template triangularView<Lower>());

    // sparse triangularView with sparse matrices
    VERIFY_IS_APPROX(mSres = mA.template triangularView<Lower>() * mS,
                     refX = refA.template triangularView<Lower>() * refS);
    VERIFY_IS_APPROX(mSres = mS * mA.template triangularView<Lower>(),
                     refX = refS * refA.template triangularView<Lower>());
    VERIFY_IS_APPROX(mSres = mA.template triangularView<Upper>() * mS,
                     refX = refA.template triangularView<Upper>() * refS);
    VERIFY_IS_APPROX(mSres = mS * mA.template triangularView<Upper>(),
                     refX = refS * refA.template triangularView<Upper>());

    VERIFY_IS_APPROX(mSres = mA.template triangularView<UnitLower>() * scale.asDiagonal(),
                     refX = DenseMatrix(refA.template triangularView<UnitLower>()) * scale.asDiagonal());
    VERIFY_IS_APPROX(mSres = scale.asDiagonal() * mA.template triangularView<UnitLower>(),
                     refX = scale.asDiagonal() * DenseMatrix(refA.template triangularView<UnitLower>()));
    VERIFY_IS_APPROX(mSres = mA.template triangularView<UnitUpper>() * scale.asDiagonal(),
                     refX = DenseMatrix(refA.template triangularView<UnitUpper>()) * scale.asDiagonal());
    VERIFY_IS_APPROX(mSres = scale.asDiagonal() * mA.template triangularView<UnitUpper>(),
                     refX = scale.asDiagonal() * DenseMatrix(refA.template triangularView<UnitUpper>()));
  }
}

// New test for Bug in SparseTimeDenseProduct
template <typename SparseMatrixType, typename DenseMatrixType>
void sparse_product_regression_test() {
  // This code does not compile with afflicted versions of the bug
  SparseMatrixType sm1(3, 2);
  DenseMatrixType m2(2, 2);
  sm1.setZero();
  m2.setZero();

  DenseMatrixType m3 = sm1 * m2;

  // This code produces a segfault with afflicted versions of another SparseTimeDenseProduct
  // bug

  SparseMatrixType sm2(20000, 2);
  sm2.setZero();
  DenseMatrixType m4(sm2 * m2);

  VERIFY_IS_APPROX(m4(0, 0), 0.0);
}

template <typename Scalar>
void bug_942() {
  typedef Matrix<Scalar, Dynamic, 1> Vector;
  typedef SparseMatrix<Scalar, ColMajor> ColSpMat;
  typedef SparseMatrix<Scalar, RowMajor> RowSpMat;
  ColSpMat cmA(1, 1);
  cmA.insert(0, 0) = 1;

  RowSpMat rmA(1, 1);
  rmA.insert(0, 0) = 1;

  Vector d(1);
  d[0] = 2;

  double res = 2;

  VERIFY_IS_APPROX((cmA * d.asDiagonal()).eval().coeff(0, 0), res);
  VERIFY_IS_APPROX((d.asDiagonal() * rmA).eval().coeff(0, 0), res);
  VERIFY_IS_APPROX((rmA * d.asDiagonal()).eval().coeff(0, 0), res);
  VERIFY_IS_APPROX((d.asDiagonal() * cmA).eval().coeff(0, 0), res);
}

template <typename Real>
void test_mixing_types() {
  typedef std::complex<Real> Cplx;
  typedef SparseMatrix<Real> SpMatReal;
  typedef SparseMatrix<Cplx> SpMatCplx;
  typedef SparseMatrix<Cplx, RowMajor> SpRowMatCplx;
  typedef Matrix<Real, Dynamic, Dynamic> DenseMatReal;
  typedef Matrix<Cplx, Dynamic, Dynamic> DenseMatCplx;

  Index n = internal::random<Index>(1, 100);
  double density = (std::max)(8. / static_cast<double>(n * n), 0.2);

  SpMatReal sR1(n, n);
  SpMatCplx sC1(n, n), sC2(n, n), sC3(n, n);
  SpRowMatCplx sCR(n, n);
  DenseMatReal dR1(n, n);
  DenseMatCplx dC1(n, n), dC2(n, n), dC3(n, n);

  initSparse<Real>(density, dR1, sR1);
  initSparse<Cplx>(density, dC1, sC1);
  initSparse<Cplx>(density, dC2, sC2);

  VERIFY_IS_APPROX(sC2 = (sR1 * sC1), dC3 = dR1.template cast<Cplx>() * dC1);
  VERIFY_IS_APPROX(sC2 = (sC1 * sR1), dC3 = dC1 * dR1.template cast<Cplx>());
  VERIFY_IS_APPROX(sC2 = (sR1.transpose() * sC1), dC3 = dR1.template cast<Cplx>().transpose() * dC1);
  VERIFY_IS_APPROX(sC2 = (sC1.transpose() * sR1), dC3 = dC1.transpose() * dR1.template cast<Cplx>());
  VERIFY_IS_APPROX(sC2 = (sR1 * sC1.transpose()), dC3 = dR1.template cast<Cplx>() * dC1.transpose());
  VERIFY_IS_APPROX(sC2 = (sC1 * sR1.transpose()), dC3 = dC1 * dR1.template cast<Cplx>().transpose());
  VERIFY_IS_APPROX(sC2 = (sR1.transpose() * sC1.transpose()),
                   dC3 = dR1.template cast<Cplx>().transpose() * dC1.transpose());
  VERIFY_IS_APPROX(sC2 = (sC1.transpose() * sR1.transpose()),
                   dC3 = dC1.transpose() * dR1.template cast<Cplx>().transpose());

  VERIFY_IS_APPROX(sCR = (sR1 * sC1), dC3 = dR1.template cast<Cplx>() * dC1);
  VERIFY_IS_APPROX(sCR = (sC1 * sR1), dC3 = dC1 * dR1.template cast<Cplx>());
  VERIFY_IS_APPROX(sCR = (sR1.transpose() * sC1), dC3 = dR1.template cast<Cplx>().transpose() * dC1);
  VERIFY_IS_APPROX(sCR = (sC1.transpose() * sR1), dC3 = dC1.transpose() * dR1.template cast<Cplx>());
  VERIFY_IS_APPROX(sCR = (sR1 * sC1.transpose()), dC3 = dR1.template cast<Cplx>() * dC1.transpose());
  VERIFY_IS_APPROX(sCR = (sC1 * sR1.transpose()), dC3 = dC1 * dR1.template cast<Cplx>().transpose());
  VERIFY_IS_APPROX(sCR = (sR1.transpose() * sC1.transpose()),
                   dC3 = dR1.template cast<Cplx>().transpose() * dC1.transpose());
  VERIFY_IS_APPROX(sCR = (sC1.transpose() * sR1.transpose()),
                   dC3 = dC1.transpose() * dR1.template cast<Cplx>().transpose());

  VERIFY_IS_APPROX(sC2 = (sR1 * sC1).pruned(), dC3 = dR1.template cast<Cplx>() * dC1);
  VERIFY_IS_APPROX(sC2 = (sC1 * sR1).pruned(), dC3 = dC1 * dR1.template cast<Cplx>());
  VERIFY_IS_APPROX(sC2 = (sR1.transpose() * sC1).pruned(), dC3 = dR1.template cast<Cplx>().transpose() * dC1);
  VERIFY_IS_APPROX(sC2 = (sC1.transpose() * sR1).pruned(), dC3 = dC1.transpose() * dR1.template cast<Cplx>());
  VERIFY_IS_APPROX(sC2 = (sR1 * sC1.transpose()).pruned(), dC3 = dR1.template cast<Cplx>() * dC1.transpose());
  VERIFY_IS_APPROX(sC2 = (sC1 * sR1.transpose()).pruned(), dC3 = dC1 * dR1.template cast<Cplx>().transpose());
  VERIFY_IS_APPROX(sC2 = (sR1.transpose() * sC1.transpose()).pruned(),
                   dC3 = dR1.template cast<Cplx>().transpose() * dC1.transpose());
  VERIFY_IS_APPROX(sC2 = (sC1.transpose() * sR1.transpose()).pruned(),
                   dC3 = dC1.transpose() * dR1.template cast<Cplx>().transpose());

  VERIFY_IS_APPROX(sCR = (sR1 * sC1).pruned(), dC3 = dR1.template cast<Cplx>() * dC1);
  VERIFY_IS_APPROX(sCR = (sC1 * sR1).pruned(), dC3 = dC1 * dR1.template cast<Cplx>());
  VERIFY_IS_APPROX(sCR = (sR1.transpose() * sC1).pruned(), dC3 = dR1.template cast<Cplx>().transpose() * dC1);
  VERIFY_IS_APPROX(sCR = (sC1.transpose() * sR1).pruned(), dC3 = dC1.transpose() * dR1.template cast<Cplx>());
  VERIFY_IS_APPROX(sCR = (sR1 * sC1.transpose()).pruned(), dC3 = dR1.template cast<Cplx>() * dC1.transpose());
  VERIFY_IS_APPROX(sCR = (sC1 * sR1.transpose()).pruned(), dC3 = dC1 * dR1.template cast<Cplx>().transpose());
  VERIFY_IS_APPROX(sCR = (sR1.transpose() * sC1.transpose()).pruned(),
                   dC3 = dR1.template cast<Cplx>().transpose() * dC1.transpose());
  VERIFY_IS_APPROX(sCR = (sC1.transpose() * sR1.transpose()).pruned(),
                   dC3 = dC1.transpose() * dR1.template cast<Cplx>().transpose());

  VERIFY_IS_APPROX(dC2 = (sR1 * sC1), dC3 = dR1.template cast<Cplx>() * dC1);
  VERIFY_IS_APPROX(dC2 = (sC1 * sR1), dC3 = dC1 * dR1.template cast<Cplx>());
  VERIFY_IS_APPROX(dC2 = (sR1.transpose() * sC1), dC3 = dR1.template cast<Cplx>().transpose() * dC1);
  VERIFY_IS_APPROX(dC2 = (sC1.transpose() * sR1), dC3 = dC1.transpose() * dR1.template cast<Cplx>());
  VERIFY_IS_APPROX(dC2 = (sR1 * sC1.transpose()), dC3 = dR1.template cast<Cplx>() * dC1.transpose());
  VERIFY_IS_APPROX(dC2 = (sC1 * sR1.transpose()), dC3 = dC1 * dR1.template cast<Cplx>().transpose());
  VERIFY_IS_APPROX(dC2 = (sR1.transpose() * sC1.transpose()),
                   dC3 = dR1.template cast<Cplx>().transpose() * dC1.transpose());
  VERIFY_IS_APPROX(dC2 = (sC1.transpose() * sR1.transpose()),
                   dC3 = dC1.transpose() * dR1.template cast<Cplx>().transpose());

  VERIFY_IS_APPROX(dC2 = dR1 * sC1, dC3 = dR1.template cast<Cplx>() * sC1);
  VERIFY_IS_APPROX(dC2 = sR1 * dC1, dC3 = sR1.template cast<Cplx>() * dC1);
  VERIFY_IS_APPROX(dC2 = dC1 * sR1, dC3 = dC1 * sR1.template cast<Cplx>());
  VERIFY_IS_APPROX(dC2 = sC1 * dR1, dC3 = sC1 * dR1.template cast<Cplx>());

  VERIFY_IS_APPROX(dC2 = dR1.row(0) * sC1, dC3 = dR1.template cast<Cplx>().row(0) * sC1);
  VERIFY_IS_APPROX(dC2 = sR1 * dC1.col(0), dC3 = sR1.template cast<Cplx>() * dC1.col(0));
  VERIFY_IS_APPROX(dC2 = dC1.row(0) * sR1, dC3 = dC1.row(0) * sR1.template cast<Cplx>());
  VERIFY_IS_APPROX(dC2 = sC1 * dR1.col(0), dC3 = sC1 * dR1.template cast<Cplx>().col(0));
}

// Test mixed storage types
template <int OrderA, int OrderB, int OrderC>
void test_mixed_storage_imp() {
  typedef float Real;
  typedef Matrix<Real, Dynamic, Dynamic> DenseMat;

  // Case: Large inputs but small result
  {
    SparseMatrix<Real, OrderA> A(8, 512);
    SparseMatrix<Real, OrderB> B(512, 8);
    DenseMat refA(8, 512);
    DenseMat refB(512, 8);

    initSparse<Real>(0.1, refA, A);
    initSparse<Real>(0.1, refB, B);

    SparseMatrix<Real, OrderC, std::int8_t> result;
    SparseMatrix<Real, OrderC> result_large;
    DenseMat refResult;

    VERIFY_IS_APPROX(result = (A * B), refResult = refA * refB);
  }

  // Case: Small input but large result
  {
    SparseMatrix<Real, OrderA, std::int8_t> A(127, 8);
    SparseMatrix<Real, OrderB, std::int8_t> B(8, 127);
    DenseMat refA(127, 8);
    DenseMat refB(8, 127);

    initSparse<Real>(0.01, refA, A);
    initSparse<Real>(0.01, refB, B);

    SparseMatrix<Real, OrderC> result;
    SparseMatrix<Real, OrderC> result_large;
    DenseMat refResult;

    VERIFY_IS_APPROX(result = (A * B), refResult = refA * refB);
  }
}

void test_mixed_storage() {
  test_mixed_storage_imp<RowMajor, RowMajor, RowMajor>();
  test_mixed_storage_imp<RowMajor, RowMajor, ColMajor>();
  test_mixed_storage_imp<RowMajor, ColMajor, RowMajor>();
  test_mixed_storage_imp<RowMajor, ColMajor, ColMajor>();
  test_mixed_storage_imp<ColMajor, RowMajor, RowMajor>();
  test_mixed_storage_imp<ColMajor, RowMajor, ColMajor>();
  test_mixed_storage_imp<ColMajor, ColMajor, RowMajor>();
  test_mixed_storage_imp<ColMajor, ColMajor, ColMajor>();
}

// The pruned sparse*sparse product accumulates each column into an AmbiVector,
// which stores its coefficients either in a dense buffer or in a linked list of
// (index, value) nodes depending on the estimated density. Both storage modes
// must construct, move, and destroy the coefficients properly for
// non-trivially-copyable scalars (issue #2873).
void test_pruned_product_custom_scalar() {
  typedef SparseMatrix<AnnoyingScalar> SpMat;
  typedef Matrix<AnnoyingScalar, Dynamic, Dynamic> DenseMat;
  int instances_before = AnnoyingScalar::instances;
  {
    // Low density runs the AmbiVector in linked-list mode, high density in
    // dense-buffer mode.
    for (double density : {0.02, 0.3}) {
      const Index n = 50;
      SpMat A(n, n), B(n, n);
      for (Index k = 0; k < Index(density * n * n); ++k) {
        A.coeffRef(internal::random<Index>(0, n - 1), internal::random<Index>(0, n - 1)) =
            AnnoyingScalar(internal::random<float>());
        B.coeffRef(internal::random<Index>(0, n - 1), internal::random<Index>(0, n - 1)) =
            AnnoyingScalar(internal::random<float>());
      }
      SpMat C = (A * B).pruned();
      DenseMat refC = DenseMat(A).lazyProduct(DenseMat(B));
      VERIFY_IS_APPROX(DenseMat(C), refC);
    }
    // A single dense column in a large matrix overflows the initial node
    // capacity and exercises the linked-list reallocation path.
    {
      const Index n = 1200;
      SpMat A(n, n), B(n, n);
      for (Index i = 0; i < 900; ++i) A.coeffRef(i, 0) = AnnoyingScalar(float(i % 7) + 1.0f);
      B.coeffRef(0, 0) = AnnoyingScalar(2.0f);
      SpMat C = (A * B).pruned();
      VERIFY_IS_EQUAL(C.nonZeros(), 900);
      for (Index i = 0; i < 900; i += 123) {
        VERIFY_IS_APPROX(C.coeff(i, 0), AnnoyingScalar(2.0f * (float(i % 7) + 1.0f)));
      }
    }
  }
  // Every constructed AnnoyingScalar must have been destroyed again.
  VERIFY_IS_EQUAL(AnnoyingScalar::instances, instances_before);
}

// setZero() drops the AmbiVector's whole linked list, and the pruned product
// reaches it through init(), which drops the list too. Neither may abandon the
// coefficients the dropped nodes own, and neither may leave a node behind for
// the next coeffRef() to construct over.
void test_ambivector_discard_custom_scalar() {
  typedef internal::AmbiVector<AnnoyingScalar, int> AmbiVec;
  const Index n = 32;
  int instances_before = AnnoyingScalar::instances;
  {
    AmbiVec v(n);
    for (int discard = 0; discard < 2; ++discard) {
      v.init(IsSparse);
      v.restart();
      for (Index i = 0; i < n; i += 3) v.coeffRef(i) = AnnoyingScalar(float(i) + 1.0f);
      VERIFY_IS_EQUAL(v.nonZeros(), (n + 2) / 3);

      if (discard == 0)
        v.setZero();
      else
        v.init(IsSparse);
      VERIFY_IS_EQUAL(v.nonZeros(), 0);

      // Reinserting must find storage no live node occupies.
      v.restart();
      v.coeffRef(1) = AnnoyingScalar(5.0f);
      VERIFY_IS_APPROX(v.coeff(1), AnnoyingScalar(5.0f));
      VERIFY_IS_APPROX(v.coeff(0), AnnoyingScalar(0.0f));
    }
  }
  VERIFY_IS_EQUAL(AnnoyingScalar::instances, instances_before);
}

// resize() reuses its allocation whenever the new size still fits, and the
// sub-vector bounds an iterator walks describe the size the vector had before.
void test_ambivector_resize_bounds() {
  typedef internal::AmbiVector<double, int> AmbiVec;
  {
    AmbiVec v(20);
    v.init(IsDense);
    v.setZero();
    v.coeffRef(15) = 15.0;

    // Shrinking below the coefficient just written: iteration must end at the
    // new size rather than reaching a coefficient that is no longer part of the
    // vector.
    v.resize(10);
    VERIFY_IS_EQUAL(v.size(), 10);
    v.init(IsDense);
    Index count = 0;
    for (AmbiVec::Iterator it(v); it; ++it) {
      VERIFY(it.index() < v.size());
      ++count;
    }
    VERIFY_IS_EQUAL(count, 0);
  }
  {
    // Growing within the spare capacity: iteration must reach the coefficients
    // the new size added.
    AmbiVec v(10);
    v.init(IsDense);
    v.setZero();
    v.resize(20);
    VERIFY_IS_EQUAL(v.size(), 20);
    v.init(IsDense);
    v.setZero();
    v.coeffRef(19) = 19.0;
    Index count = 0;
    for (AmbiVec::Iterator it(v); it; ++it) {
      VERIFY_IS_EQUAL(it.index(), 19);
      VERIFY_IS_APPROX(it.value(), 19.0);
      ++count;
    }
    VERIFY_IS_EQUAL(count, 1);
  }
}

#if defined(EIGEN_EXCEPTIONS)
namespace ambivector_throwing {
struct scalar_exception {};

// A scalar that counts its live instances and can be made to throw from the
// constructor AmbiVector builds a node's coefficient with, or from relocating an
// already stored coefficient when the list outgrows its capacity. Relocation is
// keyed on a nonzero source value, which a freshly built node never has, so that
// each flag selects exactly one of the two paths whether or not the compiler
// elides the initialization of a node's coefficient from Scalar(0).
struct ThrowingScalar {
  static int live;
  static bool throw_on_construction;
  static bool throw_on_relocation;

  float value;

  ThrowingScalar() : value(0) { ++live; }
  ThrowingScalar(int v) : value(float(v)) {
    if (throw_on_construction) throw scalar_exception();
    ++live;
  }
  ThrowingScalar(const ThrowingScalar& other) : value(other.value) {
    if (throw_on_relocation && other.value != 0) throw scalar_exception();
    ++live;
  }
  ThrowingScalar(ThrowingScalar&& other) {
    if (throw_on_relocation && other.value != 0) throw scalar_exception();
    value = other.value;
    ++live;
  }
  ThrowingScalar& operator=(const ThrowingScalar& other) {
    value = other.value;
    return *this;
  }
  ThrowingScalar& operator=(ThrowingScalar&& other) {
    value = other.value;
    return *this;
  }
  ~ThrowingScalar() { --live; }
};

int ThrowingScalar::live = 0;
bool ThrowingScalar::throw_on_construction = false;
bool ThrowingScalar::throw_on_relocation = false;
}  // namespace ambivector_throwing

// A coefficient that fails to construct leaves no node: the count of live nodes
// the destructor walks may not include it, and the vector must remain usable.
void test_ambivector_failed_insertion() {
  using ambivector_throwing::ThrowingScalar;
  typedef internal::AmbiVector<ThrowingScalar, int> AmbiVec;
  const int live_before = ThrowingScalar::live;
  {
    AmbiVec v(8);
    v.init(IsSparse);
    v.restart();

    ThrowingScalar::throw_on_construction = true;
    bool threw = false;
    try {
      v.coeffRef(0);
    } catch (const ambivector_throwing::scalar_exception&) {
      threw = true;
    }
    ThrowingScalar::throw_on_construction = false;
    VERIFY(threw);
    VERIFY_IS_EQUAL(v.nonZeros(), 0);

    v.restart();
    v.coeffRef(0) = ThrowingScalar(3);
    v.coeffRef(4) = ThrowingScalar(4);
    VERIFY_IS_EQUAL(v.nonZeros(), 2);

    // The same for the two remaining insertion branches, which reach a node through the list
    // rather than starting one: a new lowest index, and an index past the last node.
    v.setZero();
    v.restart();
    v.coeffRef(5) = ThrowingScalar(5);

    ThrowingScalar::throw_on_construction = true;
    threw = false;
    try {
      v.coeffRef(0);
    } catch (const ambivector_throwing::scalar_exception&) {
      threw = true;
    }
    ThrowingScalar::throw_on_construction = false;
    VERIFY(threw);
    VERIFY_IS_EQUAL(v.nonZeros(), 1);

    ThrowingScalar::throw_on_construction = true;
    threw = false;
    try {
      v.coeffRef(7);
    } catch (const ambivector_throwing::scalar_exception&) {
      threw = true;
    }
    ThrowingScalar::throw_on_construction = false;
    VERIFY(threw);
    VERIFY_IS_EQUAL(v.nonZeros(), 1);

    v.restart();
    v.coeffRef(0) = ThrowingScalar(1);
    v.coeffRef(7) = ThrowingScalar(7);
    VERIFY_IS_EQUAL(v.nonZeros(), 3);
  }
  VERIFY_IS_EQUAL(ThrowingScalar::live, live_before);
}

// A throwing relocation must release the buffer it was relocating into and
// leave the capacity describing the buffer the vector still owns, so that the
// next insertion relocates again instead of writing past its end. The leak
// itself shows up under a leak checker; the retry below is what fails when the
// capacity has already been advanced.
void test_ambivector_failed_reallocation() {
  using ambivector_throwing::ThrowingScalar;
  typedef internal::AmbiVector<ThrowingScalar, int> AmbiVec;
  const int live_before = ThrowingScalar::live;
  {
    // Sized so that the initial node capacity is a fraction of the vector's own
    // size, which is what makes the reallocation path reachable at all.
    const Index n = 1200;
    AmbiVec v(n);
    v.init(IsSparse);
    v.restart();

    // Insert until the list has to grow, which is the first insertion that
    // relocates an already stored coefficient and therefore the first that
    // throws.
    ThrowingScalar::throw_on_relocation = true;
    Index inserted = 0;
    bool threw = false;
    while (!threw) {
      VERIFY(inserted < n);
      try {
        // In C++14 mode, NVHPC 22.9 through at least 26.5 do not destroy the right-hand temporary if a combined
        // assignment's coeffRef() throws.
        ThrowingScalar& value = v.coeffRef(inserted);
        value = ThrowingScalar(1);
        ++inserted;
      } catch (const ambivector_throwing::scalar_exception&) {
        threw = true;
      }
    }
    ThrowingScalar::throw_on_relocation = false;
    VERIFY(inserted > 0);
    VERIFY_IS_EQUAL(v.nonZeros(), inserted);

    // Same insertion again, now relocating for real.
    v.restart();
    v.coeffRef(inserted) = ThrowingScalar(2);
    VERIFY_IS_EQUAL(v.nonZeros(), inserted + 1);
  }
  VERIFY_IS_EQUAL(ThrowingScalar::live, live_before);
}
#endif  // EIGEN_EXCEPTIONS

void test_sparse_vector_dense_product() {
  SparseVector<double> sv(3);
  sv.insert(0) = 1.0;
  sv.insert(2) = 2.0;

  MatrixXd dm = MatrixXd::Random(3, 2);
  MatrixXd res = sv.transpose() * dm;
  MatrixXd ref = MatrixXd(sv).transpose() * dm;
  VERIFY_IS_APPROX(res, ref);
}

EIGEN_DECLARE_TEST(sparse_product) {
  sparse_structured_view_product_sfinae<SparseMatrix<double, ColMajor>>();
  sparse_structured_view_product_sfinae<SparseMatrix<double, RowMajor>>();

  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1((test_sparse_vector_dense_product()));
    CALL_SUBTEST_1((sparse_product<SparseMatrix<double, ColMajor>>()));
    CALL_SUBTEST_1((sparse_product<SparseMatrix<double, RowMajor>>()));
    CALL_SUBTEST_1((bug_942<double>()));
    CALL_SUBTEST_2((sparse_product<SparseMatrix<std::complex<double>, ColMajor>>()));
    CALL_SUBTEST_2((sparse_product<SparseMatrix<std::complex<double>, RowMajor>>()));
    CALL_SUBTEST_3((sparse_product<SparseMatrix<float, ColMajor, long int>>()));
    CALL_SUBTEST_4(
        (sparse_product_regression_test<SparseMatrix<double, RowMajor>, Matrix<double, Dynamic, Dynamic, RowMajor>>()));

    CALL_SUBTEST_5((test_mixing_types<float>()));
    CALL_SUBTEST_5((test_mixed_storage()));

    CALL_SUBTEST_6((test_pruned_product_custom_scalar()));
    CALL_SUBTEST_6((test_ambivector_discard_custom_scalar()));
    CALL_SUBTEST_6((test_ambivector_resize_bounds()));
#if defined(EIGEN_EXCEPTIONS)
    CALL_SUBTEST_6((test_ambivector_failed_insertion()));
    CALL_SUBTEST_6((test_ambivector_failed_reallocation()));
#endif
  }
}
