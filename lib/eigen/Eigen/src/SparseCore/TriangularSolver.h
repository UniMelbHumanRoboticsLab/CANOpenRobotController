// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_SPARSETRIANGULARSOLVER_H
#define EIGEN_SPARSETRIANGULARSOLVER_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

namespace internal {

template <typename Lhs, typename Rhs, int Mode,
          int UpLo = (Mode & Lower)   ? Lower
                     : (Mode & Upper) ? Upper
                                      : -1,
          int StorageOrder = int(traits<Lhs>::Flags) & RowMajorBit>
struct sparse_solve_triangular_selector;

// forward substitution, row-major
template <typename Lhs, typename Rhs, int Mode>
struct sparse_solve_triangular_selector<Lhs, Rhs, Mode, Lower, RowMajor> {
  using Scalar = typename Rhs::Scalar;
  using LhsEval = evaluator<Lhs>;
  using LhsIterator = typename evaluator<Lhs>::InnerIterator;
  static void run(const Lhs& lhs, Rhs& other) {
    LhsEval lhsEval(lhs);
    for (Index col = 0; col < other.cols(); ++col) {
      for (Index i = 0; i < lhs.rows(); ++i) {
        Scalar tmp = other.coeff(i, col);
        Scalar lastVal(0);
        Index lastIndex = 0;
        for (LhsIterator it(lhsEval, i); it; ++it) {
          lastVal = it.value();
          lastIndex = it.index();
          if (lastIndex == i) break;
          tmp = numext::madd<Scalar>(-lastVal, other.coeff(lastIndex, col), tmp);
        }
        EIGEN_IF_CONSTEXPR (Mode & UnitDiag)
          other.coeffRef(i, col) = tmp;
        else {
          eigen_assert(lastIndex == i);
          other.coeffRef(i, col) = tmp / lastVal;
        }
      }
    }
  }
};

// backward substitution, row-major
template <typename Lhs, typename Rhs, int Mode>
struct sparse_solve_triangular_selector<Lhs, Rhs, Mode, Upper, RowMajor> {
  using Scalar = typename Rhs::Scalar;
  using LhsEval = evaluator<Lhs>;
  using LhsIterator = typename evaluator<Lhs>::InnerIterator;
  static void run(const Lhs& lhs, Rhs& other) {
    LhsEval lhsEval(lhs);
    for (Index col = 0; col < other.cols(); ++col) {
      for (Index i = lhs.rows() - 1; i >= 0; --i) {
        Scalar tmp = other.coeff(i, col);
        Scalar l_ii(0);
        LhsIterator it(lhsEval, i);
        while (it && it.index() < i) ++it;
        EIGEN_IF_CONSTEXPR (!(Mode & UnitDiag)) {
          eigen_assert(it && it.index() == i);
          l_ii = it.value();
          ++it;
        } else if (it && it.index() == i)
          ++it;
        for (; it; ++it) {
          tmp = numext::madd<Scalar>(-it.value(), other.coeff(it.index(), col), tmp);
        }

        EIGEN_IF_CONSTEXPR (Mode & UnitDiag)
          other.coeffRef(i, col) = tmp;
        else
          other.coeffRef(i, col) = tmp / l_ii;
      }
    }
  }
};

// forward substitution, col-major
template <typename Lhs, typename Rhs, int Mode>
struct sparse_solve_triangular_selector<Lhs, Rhs, Mode, Lower, ColMajor> {
  using Scalar = typename Rhs::Scalar;
  using LhsEval = evaluator<Lhs>;
  using LhsIterator = typename evaluator<Lhs>::InnerIterator;
  static void run(const Lhs& lhs, Rhs& other) {
    LhsEval lhsEval(lhs);
    for (Index col = 0; col < other.cols(); ++col) {
      for (Index i = 0; i < lhs.cols(); ++i) {
        Scalar& tmp = other.coeffRef(i, col);
        if (!numext::is_exactly_zero(tmp))  // optimization when other is actually sparse
        {
          LhsIterator it(lhsEval, i);
          while (it && it.index() < i) ++it;
          EIGEN_IF_CONSTEXPR (!(Mode & UnitDiag)) {
            eigen_assert(it && it.index() == i);
            tmp /= it.value();
          }
          if (it && it.index() == i) ++it;
          for (; it; ++it) {
            other.coeffRef(it.index(), col) = numext::madd<Scalar>(-tmp, it.value(), other.coeffRef(it.index(), col));
          }
        }
      }
    }
  }
};

// backward substitution, col-major
template <typename Lhs, typename Rhs, int Mode>
struct sparse_solve_triangular_selector<Lhs, Rhs, Mode, Upper, ColMajor> {
  using Scalar = typename Rhs::Scalar;
  using LhsEval = evaluator<Lhs>;
  using LhsIterator = typename evaluator<Lhs>::InnerIterator;
  static void run(const Lhs& lhs, Rhs& other) {
    LhsEval lhsEval(lhs);
    for (Index col = 0; col < other.cols(); ++col) {
      for (Index i = lhs.cols() - 1; i >= 0; --i) {
        Scalar& tmp = other.coeffRef(i, col);
        if (!numext::is_exactly_zero(tmp))  // optimization when other is actually sparse
        {
          EIGEN_IF_CONSTEXPR (!(Mode & UnitDiag)) {
            // TODO: replace this with a binary search. make sure the binary search is safe for partially sorted
            // elements
            LhsIterator it(lhsEval, i);
            while (it && it.index() != i) ++it;
            eigen_assert(it && it.index() == i);
            other.coeffRef(i, col) /= it.value();
          }
          LhsIterator it(lhsEval, i);
          for (; it && it.index() < i; ++it) {
            other.coeffRef(it.index(), col) = numext::madd<Scalar>(-tmp, it.value(), other.coeffRef(it.index(), col));
          }
        }
      }
    }
  }
};

}  // end namespace internal

#ifndef EIGEN_PARSED_BY_DOXYGEN

template <typename ExpressionType, unsigned int Mode>
template <typename OtherDerived>
void TriangularViewImpl<ExpressionType, Mode, Sparse>::solveInPlace(MatrixBase<OtherDerived>& other) const {
  eigen_assert(derived().cols() == derived().rows() && derived().cols() == other.rows());
  eigen_assert((!(Mode & ZeroDiag)) && bool(Mode & (Upper | Lower)));

  enum { copy = internal::traits<OtherDerived>::Flags & RowMajorBit };

  using OtherCopy =
      std::conditional_t<copy, typename internal::plain_matrix_type_column_major<OtherDerived>::type, OtherDerived&>;
  OtherCopy otherCopy(other.derived());

  internal::sparse_solve_triangular_selector<ExpressionType, std::remove_reference_t<OtherCopy>, Mode>::run(
      derived().nestedExpression(), otherCopy);

  if (copy) other = otherCopy;
}
#endif

// pure sparse path

namespace internal {

template <typename Lhs, typename Rhs, int Mode,
          int UpLo = (Mode & Lower)   ? Lower
                     : (Mode & Upper) ? Upper
                                      : -1,
          int StorageOrder = int(Lhs::Flags) & RowMajorBit>
struct sparse_solve_triangular_sparse_selector;

// True when the rhs exposes raw CSC storage with a StorageIndex matching the lhs, so a
// column's stored index slice can serve as the reach roots directly (no bIdx copy). A
// SparseVector qualifies too -- it is a single compressed column, handled below via the
// null-outerIndexPtr guard (its outerIndexPtr() is null since it has no outer array).
template <typename Lhs, typename Rhs>
using rhs_matching_slice = std::integral_constant<
    bool, has_compressed_access<Rhs>::value &&
              std::is_same<typename traits<Rhs>::StorageIndex, typename traits<Lhs>::StorageIndex>::value>;

// The reach for a column arrives in one of three compile-time-known orders, but the
// output column must store ascending inner index. An unordered reach (Ordered == false,
// the pointer/DFS path -- Upper irrelevant) is sorted ascending here; the iterator reach
// already arrives in solve order, ascending for lower and descending for upper, so it
// needs no pass of its own -- the descending one is read back to front at insertion.
template <bool Ordered, bool Upper, typename StorageIndex>
struct reach_reorder {  // Ordered == false: unordered pointer/DFS reach
  static void run(StorageIndex* first, StorageIndex* last) { std::sort(first, last); }
};
template <bool Upper, typename StorageIndex>
struct reach_reorder<true, Upper, StorageIndex> {  // iterator reach: already in solve order
  static void run(StorageIndex* /*first*/, StorageIndex* /*last*/) {}
};

// Common per-column finish: read the reach xi[top..n) in ascending inner index order,
// insert reading values from xwork, and clear xwork and mark for the next column.
// The reach is a structural bound, not a numeric one: a reached coefficient can be
// exactly zero (a zero rhs entry, or numerical cancellation), so skip exact zeros at
// insertion. This matches the AmbiVector path, which pruned zeros, and keeps a zero rhs
// from materializing O(|reach|) stored zeros. xwork and mark are cleared regardless.
template <bool Ordered, bool Upper, typename Res, typename StorageIndex, typename Scalar>
void reach_insert_column(Res& res, Index col, StorageIndex* xi, Index top, Index n, Scalar* xwork, uint8_t* mark) {
  reach_reorder<Ordered, Upper, StorageIndex>::run(xi + top, xi + n);
  // Only the iterator upper reach is descending; it is read back to front, so k walks
  // up while the load walks down. Every other order is ascending by this point.
  constexpr bool Descending = Ordered && Upper;
  for (Index k = top; k < n; ++k) {
    StorageIndex j = xi[Descending ? top + n - 1 - k : k];
    if (!numext::is_exactly_zero(xwork[j])) res.insert(j, col) = xwork[j];
    xwork[j] = Scalar(0);
    mark[j] = 0;
  }
}

// Column loop, fast path: the rhs matches (see rhs_matching_slice), so each column's
// stored index slice is the reach root list and the value slice is scattered directly
// -- no bIdx copy, so iwork is just 2n (xi | pstack).
template <bool Upper, bool UnitDiag, typename Lhs, typename Rhs, typename Res, typename Scalar,
          std::enable_if_t<rhs_matching_slice<Lhs, Rhs>::value, int> = 0>
void reach_solve_columns(const Lhs& lhs, const Rhs& other, Res& res, uint8_t* mark, Scalar* xwork, Index n) {
  using StorageIndex = typename traits<Lhs>::StorageIndex;
  Matrix<StorageIndex, Dynamic, 1> iwork(2 * n);  // xi | pstack
  StorageIndex* xi = iwork.data();
  for (Index col = 0; col < other.cols(); ++col) {
    const StorageIndex* outer = other.outerIndexPtr();  // null for a SparseVector (single column)
    const StorageIndex* nnz = other.innerNonZeroPtr();  // null when compressed
    Index p = outer ? outer[col] : 0;
    Index bCount = outer ? (nnz ? Index(nnz[col]) : Index(outer[col + 1]) - p) : other.nonZeros();
    const StorageIndex* roots = other.innerIndexPtr() + p;  // the column's stored indices
    const Scalar* vals = other.valuePtr() + p;
    // Roots are the rhs slice directly (no bIdx copy, so iwork stays 2n). An exact-zero
    // stored rhs entry is seeded harmlessly -- it propagates zeros and is dropped at
    // insertion; filtering it here would cost a compacted root buffer (the 3n path).
    for (Index r = 0; r < bCount; ++r) xwork[roots[r]] = vals[r];
    Index top = reach_solve_dense<Upper, UnitDiag>(lhs, roots, bCount, xi, mark, xwork);
    reach_insert_column<!has_compressed_access<Lhs>::value, Upper>(res, col, xi, top, n, xwork, mark);
  }
}

// Column loop, fallback: read each column through the InnerIterator, copying indices
// into the bIdx third of a 3n iwork. For a rhs without raw storage or with a
// mismatched index type.
template <bool Upper, bool UnitDiag, typename Lhs, typename Rhs, typename Res, typename Scalar,
          std::enable_if_t<!rhs_matching_slice<Lhs, Rhs>::value, int> = 0>
void reach_solve_columns(const Lhs& lhs, const Rhs& other, Res& res, uint8_t* mark, Scalar* xwork, Index n) {
  using StorageIndex = typename traits<Lhs>::StorageIndex;
  Matrix<StorageIndex, Dynamic, 1> iwork(3 * n);  // xi | pstack | bIdx
  StorageIndex* xi = iwork.data();
  StorageIndex* bIdx = iwork.data() + 2 * n;
  for (Index col = 0; col < other.cols(); ++col) {
    Index bCount = 0;
    for (typename Rhs::InnerIterator it(other, col); it; ++it) {
      if (numext::is_exactly_zero(it.value())) continue;  // a zero root seeds nothing; xwork stays clear there
      bIdx[bCount] = StorageIndex(it.index());
      xwork[it.index()] = it.value();
      ++bCount;
    }
    Index top = reach_solve_dense<Upper, UnitDiag>(lhs, bIdx, bCount, xi, mark, xwork);
    reach_insert_column<!has_compressed_access<Lhs>::value, Upper>(res, col, xi, top, n, xwork, mark);
  }
}

// Reach-based (Gilbert-Peierls) sparse triangular solve, col-major, for lower OR
// upper. Only the columns reachable from each rhs column's pattern are touched, so
// the cost is O(|reach| + flops) per column instead of a dense O(n)-per-column sweep
// (which also pays a coeff(i,i) binary search per row in the upper case). It is
// the sole col-major sparse-sparse selector, dispatching lower/upper via the UpLo
// template argument. reach_solve_dense leaves the solution values in
// xwork and the reached indices in iwork[top..n); reach_solve_columns (slice or
// fallback, selected on the rhs storage) scatters each column and solves, and
// reach_insert_column reads the values out and restores mark/xwork. Only mark and
// xwork need zeroing -- iwork is entirely written before read.
template <bool Upper, typename Lhs, typename Rhs, int Mode>
void run_sparse_reach_triangular_solve(const Lhs& lhs, Rhs& other) {
  using Scalar = typename Rhs::Scalar;
  Index n = lhs.rows();
  Matrix<uint8_t, Dynamic, 1> mark = Matrix<uint8_t, Dynamic, 1>::Zero(n);
  Matrix<Scalar, Dynamic, 1> xwork = Matrix<Scalar, Dynamic, 1>::Zero(n);
  Rhs res(other.rows(), other.cols());
  res.reserve(other.nonZeros());
  reach_solve_columns<Upper, bool(Mode & UnitDiag)>(lhs, other, res, mark.data(), xwork.data(), n);
  res.finalize();
  other = res.markAsRValue();
}

// forward and backward substitution, col-major
template <typename Lhs, typename Rhs, int Mode, int UpLo>
struct sparse_solve_triangular_sparse_selector<Lhs, Rhs, Mode, UpLo, ColMajor> {
  static void run(const Lhs& lhs, Rhs& other) {
    run_sparse_reach_triangular_solve<UpLo == Upper, Lhs, Rhs, Mode>(lhs, other);
  }
};

}  // end namespace internal

#ifndef EIGEN_PARSED_BY_DOXYGEN
template <typename ExpressionType, unsigned int Mode>
template <typename OtherDerived>
void TriangularViewImpl<ExpressionType, Mode, Sparse>::solveInPlace(SparseMatrixBase<OtherDerived>& other) const {
  eigen_assert(derived().cols() == derived().rows() && derived().cols() == other.rows());
  eigen_assert((!(Mode & ZeroDiag)) && bool(Mode & (Upper | Lower)));

  internal::sparse_solve_triangular_sparse_selector<ExpressionType, OtherDerived, Mode>::run(
      derived().nestedExpression(), other.derived());
}
#endif

}  // end namespace Eigen

#endif  // EIGEN_SPARSETRIANGULARSOLVER_H
