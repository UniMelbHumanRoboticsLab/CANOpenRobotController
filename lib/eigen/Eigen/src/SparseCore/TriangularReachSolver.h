// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_TRIANGULAR_REACH_SOLVER_H
#define EIGEN_TRIANGULAR_REACH_SOLVER_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

namespace internal {

// Gilbert-Peierls sparse-rhs triangular solve.
//
// Solves T x = b for a column-major sparse triangular T (lower OR upper) with sorted
// columns, and a sparse right-hand side b. The cost is O(|reach| + flops): only the
// columns reachable from b's pattern are touched, independent of the dimension.
//
// T is the triangular VIEW of a possibly-general stored matrix, so a column may hold
// entries on BOTH sides of the diagonal; only the entries in the active triangle
// participate. Everything is therefore direction-aware via a compile-time bool `Upper`:
//   - the reach follows only in-triangle successors (strictly below the diagonal for
//     lower, strictly above for upper -- one comparison per stored entry);
//   - the numeric sweep locates the diagonal within the sorted column (first entry with
//     index >= j for lower, last with index <= j for upper) and updates only the
//     in-triangle off-diagonal entries.
// For a genuinely triangular stored matrix (Eigen's SparseMatrix) the in-triangle
// entries are the whole column and the diagonal sits at the extreme, so the boundary
// scans are single-comparison no-ops.

// ===========================================================================
// Reach: any triangular view, raw CSC storage.
// ===========================================================================

// Computes reach_{G(T)}(pattern(b)) via a non-recursive depth-first search, emitting
// the reached columns into xi[top..n) in topological (solve) order and returning top.
// `mark` (length-n byte array; a 0/1 visited flag needs no more) must be all-zero on
// entry; every reached node is flagged, and since the reached set is exactly the
// output, the caller clears those flags again while gathering (no reset needed). `xi`
// and `pstack` are size-n scratch; the DFS stack occupies xi[0..head] while the output
// grows down from xi[n), and head < top holds so they never overlap. Only in-triangle
// stored entries (index > j for lower, index < j for upper) are reach successors; the
// diagonal and any out-of-triangle entries of the stored matrix are skipped.
// `innerNonZeroPtr` is the per-column nonzero count: pass it for an uncompressed matrix
// so column j ends at outerIndexPtr[j]+innerNonZeroPtr[j]; pass nullptr (compressed) to
// end at outerIndexPtr[j+1].
template <bool Upper, typename StorageIndex>
Index triangular_reach(const StorageIndex* outerIndexPtr, const StorageIndex* innerIndexPtr,
                       const StorageIndex* innerNonZeroPtr, const StorageIndex* bIdx, Index bCount, StorageIndex* xi,
                       StorageIndex* pstack, uint8_t* mark, Index n) {
  Index top = n;
  for (Index r = 0; r < bCount; ++r) {
    StorageIndex root = bIdx[r];
    if (mark[root]) continue;

    Index head = 0;
    xi[0] = root;
    while (head >= 0) {
      StorageIndex j = xi[head];
      Index colBeg = outerIndexPtr[j];
      Index colEnd = innerNonZeroPtr ? outerIndexPtr[j] + innerNonZeroPtr[j] : outerIndexPtr[j + 1];
      if (!mark[j]) {
        mark[j] = 1;
        pstack[head] = StorageIndex(colBeg);
      }
      bool done = true;
      for (Index p = pstack[head]; p < colEnd; ++p) {
        StorageIndex i = innerIndexPtr[p];
        if (Upper ? (i >= j) : (i <= j)) continue;  // out of triangle, or the diagonal
        if (mark[i]) continue;                      // already visited
        pstack[head] = StorageIndex(p + 1);
        xi[++head] = i;  // descend
        done = false;
        break;
      }
      if (done) {  // no unvisited successor: postorder j
        xi[--top] = j;
        --head;
      }
    }
  }
  return top;
}

// ===========================================================================
// Numeric sweep over a precomputed reach.
// ===========================================================================

// Solves T x = b in place on the dense accumulator x (zero except where b was
// scattered), touching only the reached columns in the order produced by the reach.
// Columns are sorted, so the active triangle is a contiguous run: for lower it is the
// suffix from the first entry with index >= j (the diagonal, then the sub-diagonal
// off-diagonals); for upper it is the prefix up to the last entry with index <= j (the
// off-diagonals, then the diagonal). Locating the boundary is O(1) for a genuinely
// triangular column (the extreme stored entry is already the diagonal); only when a
// stored column carries out-of-triangle entries (a general matrix seen through a
// TriangularView) do we binary-search past them, in O(log nnz/col) rather than a
// linear scan of the wrong-side run.
// The stored lhs scalar (LhsScalar, read from valuePtr) and the accumulator/rhs scalar
// (RhsScalar, held in x) are separate: a real factor applied to a complex rhs must
// accumulate in the complex type. The arithmetic runs in RhsScalar; LhsScalar values
// promote to it (e.g. double -> complex<double>).
template <bool Upper, bool UnitDiag, typename StorageIndex, typename LhsScalar, typename RhsScalar>
void triangular_solve_over_reach(const StorageIndex* outerIndexPtr, const StorageIndex* innerIndexPtr,
                                 const LhsScalar* valuePtr, const StorageIndex* innerNonZeroPtr, const StorageIndex* xi,
                                 Index top, Index n, RhsScalar* x) {
  for (Index k = top; k < n; ++k) {
    StorageIndex j = xi[k];
    Index colBeg = outerIndexPtr[j];
    Index colEnd = innerNonZeroPtr ? outerIndexPtr[j] + innerNonZeroPtr[j] : outerIndexPtr[j + 1];
    RhsScalar xj;
    Index offBeg, offEnd;
    EIGEN_IF_CONSTEXPR (Upper) {
      // e = one past the last in-triangle entry (index <= j)
      Index e = colEnd;
      if (e > colBeg && innerIndexPtr[e - 1] > j)  // wrong-side (below-diagonal) tail: skip it
        e = std::upper_bound(innerIndexPtr + colBeg, innerIndexPtr + colEnd, j) - innerIndexPtr;
      bool hasDiag = e > colBeg && innerIndexPtr[e - 1] == j;
      offBeg = colBeg;
      offEnd = hasDiag ? e - 1 : e;  // drop the diagonal slot from the update iff it is stored
      EIGEN_IF_CONSTEXPR (!UnitDiag) {
        eigen_assert(hasDiag && "sparse triangular solve: missing diagonal");
        // Missing diagonal is out of contract; match the old AmbiVector path -- divide by 0
        // for a deterministic inf/NaN, rather than reading valuePtr[e-1] out of bounds.
        x[j] /= hasDiag ? valuePtr[e - 1] : LhsScalar(0);
      }
    } else {
      // s = first in-triangle entry (index >= j)
      Index s = colBeg;
      if (s < colEnd && innerIndexPtr[s] < j)  // wrong-side (above-diagonal) head: skip it
        s = std::lower_bound(innerIndexPtr + colBeg, innerIndexPtr + colEnd, j) - innerIndexPtr;
      bool hasDiag = s < colEnd && innerIndexPtr[s] == j;
      offBeg = hasDiag ? s + 1 : s;  // drop the diagonal slot from the update iff it is stored
      offEnd = colEnd;
      EIGEN_IF_CONSTEXPR (!UnitDiag) {
        eigen_assert(hasDiag && "sparse triangular solve: missing diagonal");
        x[j] /= hasDiag ? valuePtr[s] : LhsScalar(0);  // missing diagonal -> inf/NaN, not an OOB read
      }
    }
    xj = x[j];
    for (Index p = offBeg; p < offEnd; ++p) {
      StorageIndex i = innerIndexPtr[p];
      x[i] = numext::madd<RhsScalar>(-xj, RhsScalar(valuePtr[p]), x[i]);
    }
  }
}

// ===========================================================================
// Borrow-a-buffer solve core.
//
// The caller owns the scratch and passes it in, so a reused buffer makes repeated
// solves allocation-free. All buffers are restored on exit, so one setup suffices for
// many solves.
// ===========================================================================

// Core: compute reach(pattern(b)) and run the numeric sweep, returning top -- WITHOUT
// any cleanup. The rhs must ALREADY be scattered into xwork (xwork[bIdx[r]] = value) by
// the caller; bIdx is the rhs pattern (the reach roots). Pulling the scatter out lets a
// caller reading the rhs through an iterator scatter as it reads, dropping the separate
// value array. On return, xi = iwork[top..n) holds the reached columns in topological
// order, xwork holds their solution values, and mark is set on the reached set; the
// caller consumes xwork/xi and clears them.
// Solving T x = b for a column-major, sorted triangular T (lower or upper):
//   - iwork: >= 2n StorageIndex, carved into xi | pstack (each length n).
//   - mark:  >= n bytes, all-zero.
//   - xwork: >= n Scalar, the dense accumulator, zero except b scattered on bIdx.
// `innerNonZeroPtr` is nullptr for a compressed T, or the per-column nonzero count for
// an uncompressed T (columns then end at outerIndexPtr[j]+innerNonZeroPtr[j]).
template <bool Upper, bool UnitDiag, typename StorageIndex, typename LhsScalar, typename RhsScalar>
Index reach_solve_dense(const StorageIndex* outerIndexPtr, const StorageIndex* innerIndexPtr, const LhsScalar* valuePtr,
                        const StorageIndex* innerNonZeroPtr, Index n, const StorageIndex* bIdx, Index bCount,
                        StorageIndex* iwork, uint8_t* mark, RhsScalar* xwork) {
  Index top =
      triangular_reach<Upper>(outerIndexPtr, innerIndexPtr, innerNonZeroPtr, bIdx, bCount, iwork, iwork + n, mark, n);
  triangular_solve_over_reach<Upper, UnitDiag>(outerIndexPtr, innerIndexPtr, valuePtr, innerNonZeroPtr, iwork, top, n,
                                               xwork);
  return top;
}

// ---------------------------------------------------------------------------
// Iterator-driven path: for a triangular, column-major sparse expression that does NOT
// expose raw CSC storage (has_compressed_access is false). Columns are read through the
// expression's evaluator InnerIterator. Because an InnerIterator can't cheaply hold DFS
// resume state, the reach uses a mark-on-push worklist plus a final sort into
// topological order (ascending index for lower, descending for upper -- both valid
// topological orders for the respective solve); the log factor is empirically ~free.
// ---------------------------------------------------------------------------

// Reach via a mark-on-push worklist that opens one column at a time through
// InnerIterator. Only in-triangle successors (index > j for lower, index < j for upper)
// are pushed. Scratch: xi (n, shared stack+output) and mark (n bytes, all-zero in,
// reach flagged out). Returns top; xi[top..n) is sorted into the topological order for
// the solve direction.
template <bool Upper, typename Eval, typename StorageIndex>
Index triangular_reach_iter(const Eval& mat, const StorageIndex* bIdx, Index bCount, StorageIndex* xi, uint8_t* mark,
                            Index n) {
  Index top = n;
  Index sp = 0;
  for (Index r = 0; r < bCount; ++r) {
    StorageIndex root = bIdx[r];
    if (!mark[root]) {
      mark[root] = 1;
      xi[sp++] = root;
    }
  }
  while (sp > 0) {
    StorageIndex j = xi[--sp];
    xi[--top] = j;  // collect
    for (typename Eval::InnerIterator it(mat, j); it; ++it) {
      StorageIndex i = StorageIndex(it.index());
      if (Upper ? (i >= j) : (i <= j)) continue;  // out of triangle, or the diagonal
      if (!mark[i]) {
        mark[i] = 1;
        xi[sp++] = i;
      }
    }
  }
  // descending for upper, ascending for lower (comparator type picked at compile time)
  using Comp = std::conditional_t<Upper, std::greater<StorageIndex>, std::less<StorageIndex>>;
  std::sort(xi + top, xi + n, Comp{});
  return top;
}

// Numeric sweep over the reach, reading columns through InnerIterator. Direction-aware
// like the pointer sweep: for lower the leading out-of-triangle entries (index < j) are
// skipped so the diagonal is the first remaining entry; for upper the diagonal is found
// by scan and only the entries with index < j are updated.
template <bool Upper, bool UnitDiag, typename Eval, typename StorageIndex, typename Scalar>
void triangular_solve_over_reach_iter(const Eval& mat, const StorageIndex* xi, Index top, Index n, Scalar* x) {
  for (Index k = top; k < n; ++k) {
    StorageIndex j = xi[k];
    EIGEN_IF_CONSTEXPR (Upper) {
      EIGEN_IF_CONSTEXPR (!UnitDiag) {
        Scalar d(0);  // stays 0 if the diagonal is missing, so singularity surfaces as inf/NaN
        bool hasDiag = false;
        for (typename Eval::InnerIterator dt(mat, j); dt; ++dt)
          if (StorageIndex(dt.index()) == j) {
            d = dt.value();
            hasDiag = true;
          }
        eigen_assert(hasDiag && "sparse triangular solve: missing diagonal");
        x[j] /= d;  // d == 0 when the diagonal is missing -> inf/NaN, consistent with the pointer path
      }
      Scalar xj = x[j];
      for (typename Eval::InnerIterator it(mat, j); it && StorageIndex(it.index()) < j; ++it)
        x[it.index()] = numext::madd<Scalar>(-xj, it.value(), x[it.index()]);
    } else {
      typename Eval::InnerIterator it(mat, j);
      while (it && StorageIndex(it.index()) < j) ++it;     // skip out-of-triangle (index < j)
      bool hasDiag = it && StorageIndex(it.index()) == j;  // diagonal is the first in-triangle entry
      EIGEN_IF_CONSTEXPR (!UnitDiag) {
        eigen_assert(hasDiag && "sparse triangular solve: missing diagonal");
        // Missing diagonal -> inf/NaN, not it.value() on an ended/wrong iterator (see pointer path).
        x[j] /= hasDiag ? it.value() : Scalar(0);
      }
      if (hasDiag) ++it;  // step past the stored diagonal (the divisor above, or a unit entry)
      Scalar xj = x[j];
      for (; it; ++it) x[it.index()] = numext::madd<Scalar>(-xj, it.value(), x[it.index()]);
    }
  }
}

// Iterator core: reach + numeric, returning top (no cleanup), the iterator counterpart
// of reach_solve_dense -- xwork must already hold the scattered rhs. Uses the same 2n /
// n(bytes) / n workspace layout (the pstack half of iwork is left unused), so the two
// general paths share one workspace contract.
template <bool Upper, bool UnitDiag, typename LhsType, typename StorageIndex, typename Scalar>
Index reach_solve_dense_iter(const LhsType& lhs, Index n, const StorageIndex* bIdx, Index bCount, StorageIndex* iwork,
                             uint8_t* mark, Scalar* xwork) {
  evaluator<LhsType> mat(lhs);
  Index top = triangular_reach_iter<Upper>(mat, bIdx, bCount, iwork, mark, n);
  triangular_solve_over_reach_iter<Upper, UnitDiag>(mat, iwork, top, n, xwork);
  return top;
}

// Policy dispatch for the core (returns top): an expression that exposes raw storage
// takes the pointer + DFS fast path; anything else takes the evaluator + worklist path.
// Tag dispatch (not if-constexpr) keeps the untaken branch from being instantiated, so
// outerIndexPtr() is never named on a type that lacks it.
//
// CompressedAccessBit is a compile-time capability, not a guarantee the instance is
// compressed: an uncompressed SparseMatrix keeps per-column gaps addressed via
// innerNonZeroPtr(), so its columns do NOT run to outerIndexPtr()[j+1]. Passing
// innerNonZeroPtr() through keeps the raw-pointer path valid either way -- it is nullptr
// exactly when compressed (columns end at outerIndexPtr[j+1]) and the per-column count
// otherwise (columns end at outerIndexPtr[j]+innerNonZeroPtr[j]).
template <bool Upper, bool UnitDiag, typename LhsType, typename StorageIndex, typename Scalar>
Index reach_solve_dense_dispatch(std::true_type /*compressed*/, const LhsType& lhs, Index n, const StorageIndex* bIdx,
                                 Index bCount, StorageIndex* iwork, uint8_t* mark, Scalar* xwork) {
  return reach_solve_dense<Upper, UnitDiag>(lhs.outerIndexPtr(), lhs.innerIndexPtr(), lhs.valuePtr(),
                                            lhs.innerNonZeroPtr(), n, bIdx, bCount, iwork, mark, xwork);
}
template <bool Upper, bool UnitDiag, typename LhsType, typename StorageIndex, typename Scalar>
Index reach_solve_dense_dispatch(std::false_type /*iterator*/, const LhsType& lhs, Index n, const StorageIndex* bIdx,
                                 Index bCount, StorageIndex* iwork, uint8_t* mark, Scalar* xwork) {
  return reach_solve_dense_iter<Upper, UnitDiag>(lhs, n, bIdx, bCount, iwork, mark, xwork);
}

// Expression core: solve T x = b for a sparse-expression triangular T with the rhs
// PRE-SCATTERED into xwork (bIdx is its pattern), selecting the pointer or iterator path
// at compile time; RETURNS top with the solution left in xwork and the reach in
// iwork[top..n) (see reach_solve_dense). This is what the sparse selector uses -- it
// scatters the rhs as it reads it and consumes xwork directly, so no bVal/outIdx/outVal.
template <bool Upper, bool UnitDiag, typename LhsDerived, typename StorageIndex, typename Scalar>
Index reach_solve_dense(const SparseMatrixBase<LhsDerived>& lhs, const StorageIndex* bIdx, Index bCount,
                        StorageIndex* iwork, uint8_t* mark, Scalar* xwork) {
  return reach_solve_dense_dispatch<Upper, UnitDiag>(bool_constant<has_compressed_access<LhsDerived>::value>{},
                                                     lhs.derived(), lhs.rows(), bIdx, bCount, iwork, mark, xwork);
}

}  // namespace internal

}  // namespace Eigen

#endif  // EIGEN_TRIANGULAR_REACH_SOLVER_H
