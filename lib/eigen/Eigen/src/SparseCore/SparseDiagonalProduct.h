// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009-2015 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_SPARSE_DIAGONAL_PRODUCT_H
#define EIGEN_SPARSE_DIAGONAL_PRODUCT_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

// The product of a diagonal matrix with a sparse matrix can be easily
// implemented using expression template.
// We have to consider two very different cases:
// 1 - diag * row-major sparse
//     => each inner vector <=> scalar * sparse vector product
//     => so we can reuse CwiseUnaryOp::InnerIterator
// 2 - diag * col-major sparse
//     => each inner vector <=> densevector * sparse vector cwise product
//     => again, we can reuse specialization of CwiseBinaryOp::InnerIterator
//        for that particular case
// The two other cases are symmetric.

namespace internal {

enum { SDP_AsScalarProduct, SDP_AsCwiseProduct };

template <typename SparseXprType, typename DiagonalCoeffType, int SDP_Tag>
struct sparse_diagonal_product_evaluator;

template <typename Lhs, typename Rhs, int ProductTag>
struct product_evaluator<Product<Lhs, Rhs, DefaultProduct>, ProductTag, DiagonalShape, SparseShape>
    : public sparse_diagonal_product_evaluator<Rhs, typename Lhs::DiagonalVectorType,
                                               Rhs::Flags & RowMajorBit ? SDP_AsScalarProduct : SDP_AsCwiseProduct> {
  using XprType = Product<Lhs, Rhs, DefaultProduct>;
  enum {
    CoeffReadCost = HugeCost,
    Flags = Rhs::Flags & RowMajorBit,
    Alignment = 0
  };  // FIXME: compute proper CoeffReadCost and propagate Flags.

  using Base = sparse_diagonal_product_evaluator<Rhs, typename Lhs::DiagonalVectorType,
                                                 Rhs::Flags & RowMajorBit ? SDP_AsScalarProduct : SDP_AsCwiseProduct>;
  explicit product_evaluator(const XprType& xpr) : Base(xpr.rhs(), xpr.lhs().diagonal()) {}
};

template <typename Lhs, typename Rhs, int ProductTag>
struct product_evaluator<Product<Lhs, Rhs, DefaultProduct>, ProductTag, SparseShape, DiagonalShape>
    : public sparse_diagonal_product_evaluator<Lhs, Transpose<const typename Rhs::DiagonalVectorType>,
                                               Lhs::Flags & RowMajorBit ? SDP_AsCwiseProduct : SDP_AsScalarProduct> {
  using XprType = Product<Lhs, Rhs, DefaultProduct>;
  enum {
    CoeffReadCost = HugeCost,
    Flags = Lhs::Flags & RowMajorBit,
    Alignment = 0
  };  // FIXME: compute proper CoeffReadCost and propagate Flags.

  using Base = sparse_diagonal_product_evaluator<Lhs, Transpose<const typename Rhs::DiagonalVectorType>,
                                                 Lhs::Flags & RowMajorBit ? SDP_AsCwiseProduct : SDP_AsScalarProduct>;
  explicit product_evaluator(const XprType& xpr) : Base(xpr.lhs(), xpr.rhs().diagonal().transpose()) {}
};

// SparseSelfAdjointView synthesizes mirrored entries; build the diagonal-scaled
// full result directly instead of first materializing an unscaled PlainObject.
template <int Mode, int ProductOrder, typename SelfAdjointViewType, typename DiagonalType, typename Dest>
struct sparse_selfadjoint_diagonal_product_impl {
  using MatrixType = typename SelfAdjointViewType::MatrixTypeNested_;
  using MatrixEvaluator = evaluator<MatrixType>;
  using MatrixIterator = typename MatrixEvaluator::InnerIterator;
  using StorageIndex = typename Dest::StorageIndex;
  using VectorI = Matrix<StorageIndex, Dynamic, 1>;
  enum { IsFullMode = Mode == int(Upper | Lower), IsLowerMode = (Mode & int(Lower)) == int(Lower) };

  static void run(Dest& dest, const SelfAdjointViewType& selfadjoint, const DiagonalType& diagonal) {
    MatrixEvaluator matrixEval(selfadjoint.matrix());
    const Index size = selfadjoint.rows();
    VectorI count(size);
    count.setZero();
    dest.resize(size, size);

    for (Index outer = 0; outer < selfadjoint.matrix().outerSize(); ++outer) {
      for (MatrixIterator it(matrixEval, outer); it; ++it) countEntry(count, it.row(), it.col());
    }

    Index nnz = count.sum();
    dest.resizeNonZeros(nnz);
    dest.outerIndexPtr()[0] = 0;
    for (Index outer = 0; outer < size; ++outer)
      dest.outerIndexPtr()[outer + 1] = dest.outerIndexPtr()[outer] + count[outer];
    for (Index outer = 0; outer < size; ++outer) count[outer] = dest.outerIndexPtr()[outer];

    for (Index outer = 0; outer < selfadjoint.matrix().outerSize(); ++outer) {
      for (MatrixIterator it(matrixEval, outer); it; ++it) {
        const Index row = it.row();
        const Index col = it.col();
        if (isStored(row, col)) {
          insertEntry(dest, count, diagonal, row, col, it.value());
        }
        if (mirrorsStoredEntry(row, col)) {
          insertEntry(dest, count, diagonal, col, row, numext::conj(it.value()));
        }
      }
    }
  }

 private:
  static EIGEN_STRONG_INLINE bool isStored(Index row, Index col) {
    return IsFullMode || row == col || (IsLowerMode ? row > col : row < col);
  }

  static EIGEN_STRONG_INLINE bool mirrorsStoredEntry(Index row, Index col) {
    return !IsFullMode && row != col && (IsLowerMode ? row > col : row < col);
  }

  static void countEntry(VectorI& count, Index row, Index col) {
    if (isStored(row, col)) {
      ++count[outerIndex(row, col)];
    }
    if (mirrorsStoredEntry(row, col)) {
      ++count[outerIndex(col, row)];
    }
  }

  static EIGEN_STRONG_INLINE StorageIndex outerIndex(Index row, Index col) {
    return internal::convert_index<StorageIndex>(Dest::IsRowMajor ? row : col);
  }

  static EIGEN_STRONG_INLINE StorageIndex innerIndex(Index row, Index col) {
    return internal::convert_index<StorageIndex>(Dest::IsRowMajor ? col : row);
  }

  template <typename Coeff>
  static void insertEntry(Dest& dest, VectorI& count, const DiagonalType& diagonal, Index row, Index col,
                          const Coeff& coeff) {
    const StorageIndex outer = outerIndex(row, col);
    const Index k = count[outer]++;
    dest.innerIndexPtr()[k] = innerIndex(row, col);
    EIGEN_IF_CONSTEXPR (ProductOrder == OnTheLeft) {
      dest.valuePtr()[k] = diagonal.coeff(row) * coeff;
    } else {
      dest.valuePtr()[k] = coeff * diagonal.coeff(col);
    }
  }
};

template <typename Lhs, typename Rhs>
struct materialized_left_sparse_product_evaluator_base
    : public evaluator<typename Product<Lhs, typename Rhs::PlainObject, DefaultProduct>::PlainObject> {
  using XprType = Product<Lhs, Rhs, DefaultProduct>;
  using PlainObject = typename XprType::PlainObject;
  using Base = evaluator<PlainObject>;

  explicit materialized_left_sparse_product_evaluator_base(const XprType& xpr) : m_result(xpr.rows(), xpr.cols()) {
    internal::construct_at<Base>(this, m_result);
    sparse_selfadjoint_diagonal_product_impl<Rhs::Mode, OnTheLeft, Rhs, typename Lhs::DiagonalVectorType,
                                             PlainObject>::run(m_result, xpr.rhs(), xpr.lhs().diagonal());
  }

 protected:
  PlainObject m_result;
};

template <typename Lhs, typename Rhs>
struct materialized_right_sparse_product_evaluator_base
    : public evaluator<typename Product<typename Lhs::PlainObject, Rhs, DefaultProduct>::PlainObject> {
  using XprType = Product<Lhs, Rhs, DefaultProduct>;
  using PlainObject = typename XprType::PlainObject;
  using Base = evaluator<PlainObject>;

  explicit materialized_right_sparse_product_evaluator_base(const XprType& xpr) : m_result(xpr.rows(), xpr.cols()) {
    internal::construct_at<Base>(this, m_result);
    sparse_selfadjoint_diagonal_product_impl<Lhs::Mode, OnTheRight, Lhs, typename Rhs::DiagonalVectorType,
                                             PlainObject>::run(m_result, xpr.lhs(), xpr.rhs().diagonal());
  }

 protected:
  PlainObject m_result;
};

template <typename Lhs, typename Rhs, int ProductTag>
struct product_evaluator<Product<Lhs, Rhs, DefaultProduct>, ProductTag, DiagonalShape, SparseTriangularShape>
    : product_evaluator<Product<Lhs, Rhs, DefaultProduct>, ProductTag, DiagonalShape, SparseShape> {
  using Base = product_evaluator<Product<Lhs, Rhs, DefaultProduct>, ProductTag, DiagonalShape, SparseShape>;
  using Base::Base;
};

template <typename Lhs, typename Rhs, int ProductTag>
struct product_evaluator<Product<Lhs, Rhs, DefaultProduct>, ProductTag, DiagonalShape, SparseSelfAdjointShape>
    : materialized_left_sparse_product_evaluator_base<Lhs, Rhs> {
  using materialized_left_sparse_product_evaluator_base<Lhs, Rhs>::materialized_left_sparse_product_evaluator_base;
};

template <typename Lhs, typename Rhs, int ProductTag>
struct product_evaluator<Product<Lhs, Rhs, DefaultProduct>, ProductTag, SparseTriangularShape, DiagonalShape>
    : product_evaluator<Product<Lhs, Rhs, DefaultProduct>, ProductTag, SparseShape, DiagonalShape> {
  using Base = product_evaluator<Product<Lhs, Rhs, DefaultProduct>, ProductTag, SparseShape, DiagonalShape>;
  using Base::Base;
};

template <typename Lhs, typename Rhs, int ProductTag>
struct product_evaluator<Product<Lhs, Rhs, DefaultProduct>, ProductTag, SparseSelfAdjointShape, DiagonalShape>
    : materialized_right_sparse_product_evaluator_base<Lhs, Rhs> {
  using materialized_right_sparse_product_evaluator_base<Lhs, Rhs>::materialized_right_sparse_product_evaluator_base;
};

template <typename SparseXprType, typename DiagonalCoeffType>
struct sparse_diagonal_product_evaluator<SparseXprType, DiagonalCoeffType, SDP_AsScalarProduct> {
 protected:
  using SparseXprInnerIterator = typename evaluator<SparseXprType>::InnerIterator;
  using Scalar = typename SparseXprType::Scalar;

 public:
  class InnerIterator : public SparseXprInnerIterator {
   public:
    InnerIterator(const sparse_diagonal_product_evaluator& xprEval, Index outer)
        : SparseXprInnerIterator(xprEval.m_sparseXprImpl, outer), m_coeff(xprEval.m_diagCoeffImpl.coeff(outer)) {}

    EIGEN_STRONG_INLINE Scalar value() const { return m_coeff * SparseXprInnerIterator::value(); }

   protected:
    typename DiagonalCoeffType::Scalar m_coeff;
  };

  sparse_diagonal_product_evaluator(const SparseXprType& sparseXpr, const DiagonalCoeffType& diagCoeff)
      : m_sparseXprImpl(sparseXpr), m_diagCoeffImpl(diagCoeff) {}

  Index nonZerosEstimate() const { return m_sparseXprImpl.nonZerosEstimate(); }

 protected:
  evaluator<SparseXprType> m_sparseXprImpl;
  evaluator<DiagonalCoeffType> m_diagCoeffImpl;
};

template <typename SparseXprType, typename DiagCoeffType>
struct sparse_diagonal_product_evaluator<SparseXprType, DiagCoeffType, SDP_AsCwiseProduct> {
  using Scalar = typename SparseXprType::Scalar;
  using StorageIndex = typename SparseXprType::StorageIndex;

  using DiagCoeffNested =
      typename nested_eval<DiagCoeffType, SparseXprType::IsRowMajor ? SparseXprType::RowsAtCompileTime
                                                                    : SparseXprType::ColsAtCompileTime>::type;

  class InnerIterator {
    using SparseXprIter = typename evaluator<SparseXprType>::InnerIterator;

   public:
    InnerIterator(const sparse_diagonal_product_evaluator& xprEval, Index outer)
        : m_sparseIter(xprEval.m_sparseXprEval, outer), m_diagCoeffNested(xprEval.m_diagCoeffNested) {}

    inline Scalar value() const { return m_sparseIter.value() * m_diagCoeffNested.coeff(index()); }
    inline StorageIndex index() const { return m_sparseIter.index(); }
    inline Index outer() const { return m_sparseIter.outer(); }
    inline Index col() const { return SparseXprType::IsRowMajor ? m_sparseIter.index() : m_sparseIter.outer(); }
    inline Index row() const { return SparseXprType::IsRowMajor ? m_sparseIter.outer() : m_sparseIter.index(); }

    EIGEN_STRONG_INLINE InnerIterator& operator++() {
      ++m_sparseIter;
      return *this;
    }
    inline operator bool() const { return m_sparseIter; }

   protected:
    SparseXprIter m_sparseIter;
    DiagCoeffNested m_diagCoeffNested;
  };

  sparse_diagonal_product_evaluator(const SparseXprType& sparseXpr, const DiagCoeffType& diagCoeff)
      : m_sparseXprEval(sparseXpr), m_diagCoeffNested(diagCoeff) {}

  Index nonZerosEstimate() const { return m_sparseXprEval.nonZerosEstimate(); }

 protected:
  evaluator<SparseXprType> m_sparseXprEval;
  DiagCoeffNested m_diagCoeffNested;
};

}  // end namespace internal

}  // end namespace Eigen

#endif  // EIGEN_SPARSE_DIAGONAL_PRODUCT_H
