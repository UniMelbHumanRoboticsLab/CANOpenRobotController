// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#define EIGEN_USE_THREADS 1
#include "sparse.h"

// Pulls in the ThreadedSparseProduct header via the SparseCore module.
// (Eigen/Sparse -> Eigen/SparseCore conditional include under EIGEN_USE_THREADS.)
#include <Eigen/SparseCore>

template <typename SparseMatrixType, typename DenseVectorType>
void verify_threaded_spmv(Index rows, Index cols, double density) {
  typedef typename SparseMatrixType::Scalar Scalar;
  typedef Matrix<Scalar, Dynamic, Dynamic> DenseMatrix;

  DenseMatrix refMat(rows, cols);
  SparseMatrixType A(rows, cols);
  initSparse<Scalar>(density, refMat, A);
  A.makeCompressed();

  DenseVectorType x_fwd = DenseVectorType::Random(cols);
  DenseVectorType x_adj = DenseVectorType::Random(rows);

  // Reference results via the existing dense path.
  DenseVectorType y_fwd_ref = refMat * x_fwd;
  DenseVectorType y_adj_ref = refMat.adjoint() * x_adj;

  ThreadedSparseProduct<SparseMatrixType> op(A);
  VERIFY(op.rows() == A.rows());
  VERIFY(op.cols() == A.cols());

  // Overwriting forms.
  {
    DenseVectorType y(rows);
    y.setRandom();
    op.apply(x_fwd, y);
    VERIFY_IS_APPROX(y, y_fwd_ref);
  }
  {
    DenseVectorType y(cols);
    y.setRandom();
    op.applyAdjoint(x_adj, y);
    VERIFY_IS_APPROX(y, y_adj_ref);
  }

  // Accumulating forms: y += alpha * A * x, with both alpha = 1 and a
  // non-trivial alpha.
  for (Scalar alpha : {Scalar(1), Scalar(internal::random<Scalar>())}) {
    {
      DenseVectorType y0 = DenseVectorType::Random(rows);
      DenseVectorType y_ref = y0 + alpha * y_fwd_ref;
      DenseVectorType y = y0;
      op.applyAddTo(x_fwd, y, alpha);
      VERIFY_IS_APPROX(y, y_ref);
    }
    {
      DenseVectorType y0 = DenseVectorType::Random(cols);
      DenseVectorType y_ref = y0 + alpha * y_adj_ref;
      DenseVectorType y = y0;
      op.applyAdjointAddTo(x_adj, y, alpha);
      VERIFY_IS_APPROX(y, y_ref);
    }
  }

  // Calling adjoint must have materialized the mirror; calling it again must
  // not rebuild (no easy assertion for "not rebuilt", but verify the path is
  // consistent across repeated calls).
  VERIFY(op.hasMirror());
  {
    DenseVectorType y(cols);
    op.applyAdjoint(x_adj, y);
    VERIFY_IS_APPROX(y, y_adj_ref);
  }

  // Re-running analyzePattern must invalidate the mirror.
  op.analyzePattern(A);
  VERIFY(!op.hasMirror());
  {
    DenseVectorType y(cols);
    op.applyAdjoint(x_adj, y);
    VERIFY_IS_APPROX(y, y_adj_ref);
  }

  // Coefficient-only invalidation: mutate the bound matrix's stored values
  // (same sparsity pattern), call refreshValues(), and verify the next
  // mirror-using direction picks up the new coefficients. The mirror is
  // materialized lazily by the direction that needs the transposed view --
  // forward for ColMajor A, adjoint for RowMajor A -- so call both to warm
  // it regardless of storage order.
  {
    DenseVectorType y_fwd(rows);
    DenseVectorType y_adj(cols);
    op.apply(x_fwd, y_fwd);
    op.applyAdjoint(x_adj, y_adj);
    VERIFY(op.hasMirror());
  }
  for (Index k = 0; k < A.nonZeros(); ++k) A.valuePtr()[k] *= Scalar(2);
  refMat *= Scalar(2);
  DenseVectorType y_fwd_ref_updated = refMat * x_fwd;
  DenseVectorType y_adj_ref_updated = refMat.adjoint() * x_adj;
  op.refreshValues();
  VERIFY(!op.hasMirror());
  {
    DenseVectorType y(rows);
    op.apply(x_fwd, y);
    VERIFY_IS_APPROX(y, y_fwd_ref_updated);
  }
  {
    DenseVectorType y(cols);
    op.applyAdjoint(x_adj, y);
    VERIFY_IS_APPROX(y, y_adj_ref_updated);
  }
}

// IEEE-754 propagation through the ColMajor SpMV OMP scatter/reduce path
// (Eigen/src/SparseCore/SparseDenseProduct.h). With OpenMP enabled, the
// kernel sums per-thread scratch buffers into y; NaN/Inf in any thread's
// contribution must propagate to the reduced sum. Without OpenMP this still
// exercises the serial scatter, where the same propagation must hold.
template <typename Scalar>
void verify_threaded_scatter_ieee754() {
  typedef SparseMatrix<Scalar, ColMajor> SpMat;
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  typedef typename NumTraits<Scalar>::Real Real;

  // Pin the OMP team size so the scatter/reduce gate (nnz >= threads * m)
  // is deterministic across runs.
  const int prev_threads = Eigen::nbThreads();
  Eigen::setNbThreads(4);

  // 300x300 with diagonal + (i+j)%4==0 entries -> ~22600 nnz, above the
  // 20000-nnz threshold and well above threads*m = 4*300 for the gate.
  const Index n = 300;
  std::vector<Triplet<Scalar> > triplets;
  triplets.reserve(static_cast<std::size_t>(n) * (n / 4 + 1));
  // Simple deterministic LCG so we don't depend on <random> being included
  // transitively. Modulo-bounded so we don't rely on a specific integer width.
  unsigned rng = 0xC0FFEEu;
  for (int j = 0; j < int(n); ++j) {
    for (int i = 0; i < int(n); ++i) {
      if (i == j || (i + j) % 4 == 0) {
        rng = (rng * 1664525u + 1013904223u) & 0xFFFFu;
        const Real v = Real(0.1) + Real(0.9) * Real(rng) / Real(0xFFFF);
        triplets.push_back(Triplet<Scalar>(i, j, Scalar(v)));
      }
    }
  }
  SpMat A(n, n);
  A.setFromTriplets(triplets.begin(), triplets.end());
  A.makeCompressed();
  VERIFY(A.nonZeros() > 20000);

  // Inject NaN/Inf into three rows of column 0 by overwriting valuePtr().
  // Pick rows that are guaranteed in column 0's pattern: row 0 (diagonal),
  // row 4 ((4+0)%4==0), row 8 ((8+0)%4==0).
  const Real nan_r = std::numeric_limits<Real>::quiet_NaN();
  const Real inf_r = std::numeric_limits<Real>::infinity();
  const Index i_nan = 0, i_pinf = 4, i_ninf = 8;
  bool set_nan = false, set_pinf = false, set_ninf = false;
  for (Index k = A.outerIndexPtr()[0]; k < A.outerIndexPtr()[1]; ++k) {
    const Index i = A.innerIndexPtr()[k];
    if (i == i_nan) {
      A.valuePtr()[k] = Scalar(nan_r);
      set_nan = true;
    } else if (i == i_pinf) {
      A.valuePtr()[k] = Scalar(inf_r);
      set_pinf = true;
    } else if (i == i_ninf) {
      A.valuePtr()[k] = Scalar(-inf_r);
      set_ninf = true;
    }
  }
  VERIFY(set_nan && set_pinf && set_ninf);

  Vec x = Vec::Random(n);
  // Ensure x[0] is finite and strictly positive (real part), so the injected
  // ±Inf contributions don't cancel against a zero-multiplier or flip sign.
  x[0] = Scalar(1);

  Vec y = A * x;

  VERIFY((numext::isnan)(numext::real(y[i_nan])));
  VERIFY((numext::isinf)(numext::real(y[i_pinf])) && numext::real(y[i_pinf]) > Real(0));
  VERIFY((numext::isinf)(numext::real(y[i_ninf])) && numext::real(y[i_ninf]) < Real(0));

  Eigen::setNbThreads(prev_threads);
}

template <typename Scalar>
void run_grid() {
  typedef Matrix<Scalar, Dynamic, 1> Vec;
  // Cover both storage orders, square and rectangular, small (serial path,
  // nnz < threshold) and larger (threaded path).
  for (int order = 0; order < 2; ++order) {
    if (order == 0) {
      verify_threaded_spmv<SparseMatrix<Scalar, ColMajor>, Vec>(8, 8, 0.5);
      verify_threaded_spmv<SparseMatrix<Scalar, ColMajor>, Vec>(13, 21, 0.3);
      verify_threaded_spmv<SparseMatrix<Scalar, ColMajor>, Vec>(500, 500, 0.15);
      verify_threaded_spmv<SparseMatrix<Scalar, ColMajor>, Vec>(800, 400, 0.10);
      // Large enough that nnz > kThreadingThreshold (20000) to exercise the
      // multi-threaded dispatch path.
      verify_threaded_spmv<SparseMatrix<Scalar, ColMajor>, Vec>(1500, 1500, 0.02);
    } else {
      verify_threaded_spmv<SparseMatrix<Scalar, RowMajor>, Vec>(8, 8, 0.5);
      verify_threaded_spmv<SparseMatrix<Scalar, RowMajor>, Vec>(21, 13, 0.3);
      verify_threaded_spmv<SparseMatrix<Scalar, RowMajor>, Vec>(500, 500, 0.15);
      verify_threaded_spmv<SparseMatrix<Scalar, RowMajor>, Vec>(400, 800, 0.10);
      verify_threaded_spmv<SparseMatrix<Scalar, RowMajor>, Vec>(1500, 1500, 0.02);
    }
  }
}

EIGEN_DECLARE_TEST(sparse_threaded_product) {
  for (int i = 0; i < g_repeat; ++i) {
    CALL_SUBTEST_1(run_grid<float>());
    CALL_SUBTEST_2(run_grid<double>());
    CALL_SUBTEST_3(run_grid<std::complex<double> >());
    CALL_SUBTEST_4(verify_threaded_scatter_ieee754<double>());
    CALL_SUBTEST_4(verify_threaded_scatter_ieee754<float>());
  }
}
