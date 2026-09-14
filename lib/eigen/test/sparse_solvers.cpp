// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2010 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "sparse.h"

template <typename Scalar>
void initSPD(double density, Matrix<Scalar, Dynamic, Dynamic>& refMat, SparseMatrix<Scalar>& sparseMat) {
  Matrix<Scalar, Dynamic, Dynamic> aux(refMat.rows(), refMat.cols());
  initSparse(density, refMat, sparseMat);
  refMat = refMat * refMat.adjoint();
  for (int k = 0; k < 2; ++k) {
    initSparse(density, aux, sparseMat, ForceNonZeroDiag);
    refMat += aux * aux.adjoint();
  }
  sparseMat.setZero();
  for (int j = 0; j < sparseMat.cols(); ++j)
    for (int i = j; i < sparseMat.rows(); ++i)
      if (refMat(i, j) != Scalar(0)) sparseMat.insert(i, j) = refMat(i, j);
  sparseMat.finalize();
}

template <typename Scalar>
void sparse_solvers(int rows, int cols) {
  double density = (std::max)(8. / (rows * cols), 0.01);
  typedef Matrix<Scalar, Dynamic, Dynamic> DenseMatrix;
  typedef Matrix<Scalar, Dynamic, 1> DenseVector;
  // Scalar eps = 1e-6;

  DenseVector vec1 = DenseVector::Random(rows);

  std::vector<Vector2i> zeroCoords;
  std::vector<Vector2i> nonzeroCoords;

  // test triangular solver
  {
    DenseVector vec2 = vec1, vec3 = vec1;
    SparseMatrix<Scalar> m2(rows, cols);
    DenseMatrix refMat2 = DenseMatrix::Zero(rows, cols);

    // lower - dense
    initSparse<Scalar>(density, refMat2, m2, ForceNonZeroDiag | MakeLowerTriangular, &zeroCoords, &nonzeroCoords);
    VERIFY_IS_APPROX(refMat2.template triangularView<Lower>().solve(vec2),
                     m2.template triangularView<Lower>().solve(vec3));

    // upper - dense
    initSparse<Scalar>(density, refMat2, m2, ForceNonZeroDiag | MakeUpperTriangular, &zeroCoords, &nonzeroCoords);
    VERIFY_IS_APPROX(refMat2.template triangularView<Upper>().solve(vec2),
                     m2.template triangularView<Upper>().solve(vec3));
    VERIFY_IS_APPROX(refMat2.conjugate().template triangularView<Upper>().solve(vec2),
                     m2.conjugate().template triangularView<Upper>().solve(vec3));
    {
      SparseMatrix<Scalar> cm2(m2);
      // Index rows, Index cols, Index nnz, Index* outerIndexPtr, Index* innerIndexPtr, Scalar* valuePtr
      Map<SparseMatrix<Scalar> > mm2(rows, cols, cm2.nonZeros(), cm2.outerIndexPtr(), cm2.innerIndexPtr(),
                                     cm2.valuePtr());
      VERIFY_IS_APPROX(refMat2.conjugate().template triangularView<Upper>().solve(vec2),
                       mm2.conjugate().template triangularView<Upper>().solve(vec3));
    }

    // lower - transpose
    initSparse<Scalar>(density, refMat2, m2, ForceNonZeroDiag | MakeLowerTriangular, &zeroCoords, &nonzeroCoords);
    VERIFY_IS_APPROX(refMat2.transpose().template triangularView<Upper>().solve(vec2),
                     m2.transpose().template triangularView<Upper>().solve(vec3));

    // upper - transpose
    initSparse<Scalar>(density, refMat2, m2, ForceNonZeroDiag | MakeUpperTriangular, &zeroCoords, &nonzeroCoords);
    VERIFY_IS_APPROX(refMat2.transpose().template triangularView<Lower>().solve(vec2),
                     m2.transpose().template triangularView<Lower>().solve(vec3));

    SparseMatrix<Scalar> matB(rows, rows);
    DenseMatrix refMatB = DenseMatrix::Zero(rows, rows);

    // lower - sparse
    initSparse<Scalar>(density, refMat2, m2, ForceNonZeroDiag | MakeLowerTriangular);
    initSparse<Scalar>(density, refMatB, matB);
    refMat2.template triangularView<Lower>().solveInPlace(refMatB);
    m2.template triangularView<Lower>().solveInPlace(matB);
    VERIFY_IS_APPROX(matB.toDense(), refMatB);

    // upper - sparse
    initSparse<Scalar>(density, refMat2, m2, ForceNonZeroDiag | MakeUpperTriangular);
    initSparse<Scalar>(density, refMatB, matB);
    refMat2.template triangularView<Upper>().solveInPlace(refMatB);
    m2.template triangularView<Upper>().solveInPlace(matB);
    VERIFY_IS_APPROX(matB, refMatB);

    // A triangularView is a view of the triangular PART of a possibly-general matrix,
    // so the stored matrix need not be strictly triangular. Exercise a general lhs, a
    // SparseVector rhs, a mismatched-StorageIndex rhs, an uncompressed lhs, an
    // expression lhs, and a unit diagonal -- none of which the checks above cover.
    {
      SparseMatrix<Scalar> mg(rows, rows);
      DenseMatrix refMatG = DenseMatrix::Zero(rows, rows);
      initSparse<Scalar>(density, refMatG, mg, ForceNonZeroDiag);  // GENERAL (both triangles stored)
      initSparse<Scalar>(density, refMatB, matB);

      // general matrix through a lower / upper / unit-upper view, sparse rhs
      for (int mode = 0; mode < 3; ++mode) {
        DenseMatrix rb = refMatB;
        SparseMatrix<Scalar> mb = matB;
        if (mode == 0) {
          refMatG.template triangularView<Lower>().solveInPlace(rb);
          mg.template triangularView<Lower>().solveInPlace(mb);
        } else if (mode == 1) {
          refMatG.template triangularView<Upper>().solveInPlace(rb);
          mg.template triangularView<Upper>().solveInPlace(mb);
        } else {
          refMatG.template triangularView<UnitUpper>().solveInPlace(rb);
          mg.template triangularView<UnitUpper>().solveInPlace(mb);
        }
        VERIFY_IS_APPROX(mb.toDense(), rb);
      }

      // expression lhs (no raw storage -> iterator path)
      {
        DenseMatrix rb = refMatB;
        SparseMatrix<Scalar> mb = matB;
        refMatG.template triangularView<Upper>().solveInPlace(rb);
        (Scalar(1) * mg).template triangularView<Upper>().solveInPlace(mb);
        VERIFY_IS_APPROX(mb.toDense(), rb);
      }

      // uncompressed lhs (innerNonZeroPtr != null)
      {
        DenseMatrix rb = refMatB;
        SparseMatrix<Scalar> mb = matB, mu = mg;
        mu.reserve(Matrix<int, Dynamic, 1>::Constant(mu.cols(), rows));  // -> uncompressed
        refMatG.template triangularView<Lower>().solveInPlace(rb);
        mu.template triangularView<Lower>().solveInPlace(mb);
        VERIFY_IS_APPROX(mb.toDense(), rb);
      }

      // mismatched-StorageIndex rhs (-> InnerIterator fallback)
      {
        DenseMatrix rb = refMatB;
        SparseMatrix<Scalar, ColMajor, long> mbl = matB;
        refMatG.template triangularView<Lower>().solveInPlace(rb);
        mg.template triangularView<Lower>().solveInPlace(mbl);
        VERIFY_IS_APPROX(DenseMatrix(mbl), rb);
      }

      // SparseVector rhs (sets CompressedAccessBit but its outerIndexPtr() is null)
      {
        DenseVector rv = DenseVector::Zero(rows);
        SparseVector<Scalar> vb(rows);
        for (Index i = 0; i < rows; ++i)
          if (internal::random<int>(0, 2) == 0) {
            Scalar s = internal::random<Scalar>();
            vb.coeffRef(i) = s;
            rv(i) = s;
          }
        DenseVector rref = refMatG.template triangularView<Lower>().solve(rv);
        SparseVector<Scalar> vx = vb;
        mg.template triangularView<Lower>().solveInPlace(vx);
        VERIFY_IS_APPROX(DenseVector(vx), rref);
      }

      // explicitly-stored zero rhs entries must not expand into stored zeros: the reach
      // is a structural bound, so a zero rhs coefficient (or one that cancels to zero)
      // is pruned at insertion rather than materialized across the whole reach.
      {
        SparseMatrix<Scalar> mb(rows, matB.cols());
        DenseMatrix rb = DenseMatrix::Zero(rows, matB.cols());
        for (Index c = 0; c < mb.cols(); ++c)
          for (Index i = 0; i < rows; ++i)
            if (internal::random<int>(0, 3) == 0) {
              Scalar s = internal::random<int>(0, 2) == 0 ? Scalar(0) : internal::random<Scalar>();  // some explicit 0
              mb.insert(i, c) = s;
              rb(i, c) = s;
            }
        mb.makeCompressed();
        refMatG.template triangularView<Lower>().solveInPlace(rb);
        mg.template triangularView<Lower>().solveInPlace(mb);
        VERIFY_IS_APPROX(mb.toDense(), rb);
        for (Index c = 0; c < mb.cols(); ++c)
          for (typename SparseMatrix<Scalar>::InnerIterator it(mb, c); it; ++it)
            VERIFY(!numext::is_exactly_zero(it.value()));  // no stored zeros
      }

      // A reached column with no stored diagonal (non-unit) is out of contract: it must
      // assert in debug on every path (pointer/iterator x lower/upper), rather than the
      // failure being silently keyed to has_compressed_access. In release these divide by
      // zero -> inf/NaN with no out-of-bounds read (covered by the sanitizer drivers).
      {
        SparseMatrix<Scalar> us(3, 3);
        us.insert(0, 1) = Scalar(1);
        us.insert(1, 1) = Scalar(2);
        us.insert(2, 2) = Scalar(3);
        us.makeCompressed();  // column 0 empty -> reached from rhs(0) but no diagonal
        SparseMatrix<Scalar> ub(3, 1);
        ub.insert(0, 0) = Scalar(1);
        ub.makeCompressed();
        SparseMatrix<Scalar> up = ub, ui = ub;
        VERIFY_RAISES_ASSERT(us.template triangularView<Upper>().solveInPlace(up));                // pointer upper
        VERIFY_RAISES_ASSERT((Scalar(1) * us).template triangularView<Upper>().solveInPlace(ui));  // iterator upper

        SparseMatrix<Scalar> ls(3, 3);
        ls.insert(0, 0) = Scalar(2);
        ls.insert(1, 1) = Scalar(3);
        ls.makeCompressed();  // column 2 empty (last) -> reached from rhs(2) but no diagonal
        SparseMatrix<Scalar> lb(3, 1);
        lb.insert(2, 0) = Scalar(1);
        lb.makeCompressed();
        SparseMatrix<Scalar> lp = lb, li = lb;
        VERIFY_RAISES_ASSERT(ls.template triangularView<Lower>().solveInPlace(lp));                // pointer lower
        VERIFY_RAISES_ASSERT((Scalar(1) * ls).template triangularView<Lower>().solveInPlace(li));  // iterator lower
      }

      // mixed-scalar rhs: a real lhs applied to a rhs must accumulate in the rhs scalar.
      // For real Scalar this is the ordinary path; for complex Scalar it is the
      // real-factor / complex-data case that must both compile and be correct.
      {
        typedef typename NumTraits<Scalar>::Real Real;
        SparseMatrix<Real> mr(rows, rows);
        Matrix<Real, Dynamic, Dynamic> refMatR = Matrix<Real, Dynamic, Dynamic>::Zero(rows, rows);
        initSparse<Real>(density, refMatR, mr, ForceNonZeroDiag);
        DenseMatrix rb = refMatB;
        SparseMatrix<Scalar> mb = matB;
        refMatR.template cast<Scalar>().template triangularView<Lower>().solveInPlace(rb);
        mr.template triangularView<Lower>().solveInPlace(mb);  // SparseMatrix<Real> lhs, <Scalar> rhs
        VERIFY_IS_APPROX(mb.toDense(), rb);
      }
    }

    // test deprecated API
    initSparse<Scalar>(density, refMat2, m2, ForceNonZeroDiag | MakeLowerTriangular, &zeroCoords, &nonzeroCoords);
    VERIFY_IS_APPROX(refMat2.template triangularView<Lower>().solve(vec2),
                     m2.template triangularView<Lower>().solve(vec3));

    // test empty triangular matrix
    {
      m2.resize(0, 0);
      refMatB.resize(0, refMatB.cols());
      DenseMatrix res = m2.template triangularView<Lower>().solve(refMatB);
      VERIFY_IS_EQUAL(res.rows(), 0);
      VERIFY_IS_EQUAL(res.cols(), refMatB.cols());
      res = refMatB;
      m2.template triangularView<Lower>().solveInPlace(res);
      VERIFY_IS_EQUAL(res.rows(), 0);
      VERIFY_IS_EQUAL(res.cols(), refMatB.cols());
    }
  }
}

EIGEN_DECLARE_TEST(sparse_solvers) {
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(sparse_solvers<double>(8, 8));
    int s = internal::random<int>(1, 300);
    CALL_SUBTEST_2(sparse_solvers<std::complex<double> >(s, s));
    CALL_SUBTEST_1(sparse_solvers<double>(s, s));
  }
}
