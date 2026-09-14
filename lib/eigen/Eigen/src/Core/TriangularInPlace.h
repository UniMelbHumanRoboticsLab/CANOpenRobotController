// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_TRIANGULARINPLACE_H
#define EIGEN_TRIANGULARINPLACE_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

namespace internal {

// In-place counterparts of LAPACK xTRTRI and xLAUUM. Each overwrites its operand; the only scratch is
// one block-sized panel, at most 128x128 elements whatever the matrix size, through which the in-place
// TRMM steps run Eigen's blocked kernel, the same way optimized BLAS implement xTRMM.

// Only the lower forms exist. An upper U runs the lower kernel on Transpose<MatrixType>: with
// V = U^T, V^-1 = (U^-1)^T and V^* V = conj(U U^*) = (U U^*)^T since U U^* is self-adjoint, so the
// lower triangle of the transposed view is the upper triangle of the original in both cases.

// Both kernels have the panel-times-trailing-block shape of the Cholesky factorization, so the block
// size tracks llt_inplace::blocked(), but each has its own floor and its own size below which the
// unblocked form still wins: TRTRI's blocked step is a TRMM plus a TRSM against a bs-wide panel, and
// its unblocked form is already a sequence of TRMVs, while LAUUM's is a TRMM plus a HERK and its
// unblocked form is a sequence of GEMVs against a strided row. Measured on Zen 4 at n = 32..512.
constexpr Index kTriangularInverseMinBlocked = 64;
constexpr Index kTriangularInverseMinBlockSize = 24;
constexpr Index kAdjointSquareMinBlocked = 128;
constexpr Index kAdjointSquareMinBlockSize = 48;

EIGEN_DEVICE_FUNC inline Index triangular_in_place_block_size(Index size, Index min_block_size) {
  const Index block_size = ((size / 8) / 8) * 8;
  return numext::mini(numext::maxi(block_size, min_block_size), Index(128));
}

template <typename MatrixType>
struct triangular_in_place_workspace {
  using type = Matrix<typename MatrixType::Scalar, Dynamic, Dynamic,
                      (int(traits<MatrixType>::Flags) & RowMajorBit) ? RowMajor : ColMajor>;
};

// Unblocked lower triangular inverse, columns bottom-up so that the trailing block already holds
// X22 = L22^-1 when column j is reached: then X(j+1:, j) = -X22 L(j+1:, j) / L(j, j), one TRMV per
// column rather than the chain of AXPYs a forward substitution per column would run. \a tmp is
// scratch of at least n - 1 entries, since the TRMV's destination is its own operand.
template <unsigned int Mode, typename MatrixType, typename VectorType>
EIGEN_DEVICE_FUNC void triangular_inverse_unblocked(MatrixType& mat, VectorType& tmp) {
  using Scalar = typename MatrixType::Scalar;
  constexpr bool kUnitDiag = (Mode & UnitDiag) != 0;
  const Index n = mat.rows();
  eigen_internal_assert(n < 2 || tmp.size() >= n - 1);
  for (Index j = n - 1; j >= 0; --j) {
    Scalar xj = Scalar(1);
    EIGEN_IF_CONSTEXPR (!kUnitDiag) {
      xj = Scalar(1) / mat.coeff(j, j);
      mat.coeffRef(j, j) = xj;
    }
    const Index rs = n - j - 1;
    if (rs == 0) continue;
    tmp.head(rs).noalias() = mat.block(j + 1, j + 1, rs, rs).template triangularView<Mode>() * mat.col(j).tail(rs);
    mat.col(j).tail(rs) = -xj * tmp.head(rs);
  }
}

// Blocked lower triangular inverse, block columns bottom-up as in LAPACK xTRTRI. With
// L = [[L11, 0], [L21, L22]] the inverse is [[L11^-1, 0], [-L22^-1 L21 L11^-1, L22^-1]]; walking upwards
// makes X22 = L22^-1 available first, so the large operation is the TRMM X22 L21 rather than a solve
// against L22, and Eigen's TRMM runs closer to GEMM rate than its TRSM does on a 128-column operand.
template <unsigned int Mode, typename MatrixType>
EIGEN_DEVICE_FUNC void triangular_inverse_lower(MatrixType& mat) {
  eigen_assert(mat.rows() == mat.cols());
  const Index n = mat.rows();
  if (n < kTriangularInverseMinBlocked) {
    using Scalar = typename MatrixType::Scalar;
    ei_declare_aligned_stack_constructed_variable(Scalar, tmp_data, numext::maxi(n, Index(1)), 0);
    Map<Matrix<Scalar, Dynamic, 1> > tmp(tmp_data, n);
    triangular_inverse_unblocked<Mode>(mat, tmp);
    return;
  }
  const Index block_size = triangular_in_place_block_size(n, kTriangularInverseMinBlockSize);
  // One block-sized panel, whose first column doubles as the TRMV scratch of the unblocked kernel:
  // that kernel runs on L11 only after the panel's other uses in the same iteration are finished.
  typename triangular_in_place_workspace<MatrixType>::type work(block_size, block_size);
  for (Index k = ((n - 1) / block_size) * block_size; k >= 0; k -= block_size) {
    const Index bs = numext::mini(block_size, n - k);
    const Index rs = n - k - bs;
    Block<MatrixType, Dynamic, Dynamic> L11(mat, k, k, bs, bs);
    if (rs > 0) {
      Block<MatrixType, Dynamic, Dynamic> L21(mat, k + bs, k, rs, bs);
      Block<MatrixType, Dynamic, Dynamic> X22(mat, k + bs, k + bs, rs, rs);
      // L21 <- X22 L21. Row panel r of the product reads only rows <= r of L21, so going upwards
      // leaves every row a later panel still needs untouched.
      for (Index r_end = rs; r_end > 0; r_end -= block_size) {
        const Index r = numext::maxi(Index(0), r_end - block_size);
        const Index h = r_end - r;
        work.topLeftCorner(h, bs) = L21.middleRows(r, h);
        L21.middleRows(r, h).noalias() =
            X22.block(r, r, h, h).template triangularView<Mode>() * work.topLeftCorner(h, bs);
        if (r > 0) L21.middleRows(r, h).noalias() += X22.block(r, 0, h, r) * L21.topRows(r);
      }
      L11.template triangularView<Mode>().template solveInPlace<OnTheRight>(L21);
      L21 = -L21;
    }
    auto tmp = work.col(0);
    triangular_inverse_unblocked<Mode>(L11, tmp);
  }
}

template <unsigned int Mode, bool IsLower = (int(Mode) & int(Lower)) != 0>
struct triangular_inverse_selector {
  template <typename MatrixType>
  EIGEN_DEVICE_FUNC static void run(MatrixType& mat) {
    triangular_inverse_lower<Mode>(mat);
  }
};

template <unsigned int Mode>
struct triangular_inverse_selector<Mode, false> {
  template <typename MatrixType>
  EIGEN_DEVICE_FUNC static void run(MatrixType& mat) {
    Transpose<MatrixType> matt(mat);
    triangular_inverse_lower<(int(Mode) & int(UnitDiag)) | int(Lower)>(matt);
  }
};

// Unblocked lower self-adjoint square: mat <- lower(L^* L). Row i of the result reads column i of L
// below the diagonal and rows > i of the columns left of it, none of which row i's own update writes.
template <typename MatrixType>
void triangular_adjoint_square_unblocked(MatrixType& mat) {
  using Scalar = typename MatrixType::Scalar;
  const Index n = mat.rows();
  for (Index i = 0; i < n; ++i) {
    const Scalar lii = mat.coeff(i, i);
    const Index rs = n - i - 1;
    mat.coeffRef(i, i) = Scalar(mat.col(i).tail(n - i).squaredNorm());
    if (i > 0) {
      mat.row(i).head(i) *= numext::conj(lii);
      if (rs > 0) mat.row(i).head(i).noalias() += mat.col(i).tail(rs).adjoint() * mat.bottomLeftCorner(rs, i);
    }
  }
}

// Blocked lower self-adjoint square, left-looking as in LAPACK xLAUUM: block row k of the result is
// completed against the columns to its left before the trailing block rows consume the factor.
template <typename MatrixType>
void triangular_adjoint_square_lower(MatrixType& mat) {
  eigen_assert(mat.rows() == mat.cols());
  const Index n = mat.rows();
  if (n < kAdjointSquareMinBlocked) {
    triangular_adjoint_square_unblocked(mat);
    return;
  }
  const Index block_size = triangular_in_place_block_size(n, kAdjointSquareMinBlockSize);
  typename triangular_in_place_workspace<MatrixType>::type work(block_size, block_size);
  for (Index k = 0; k < n; k += block_size) {
    const Index bs = numext::mini(block_size, n - k);
    const Index rs = n - k - bs;
    Block<MatrixType, Dynamic, Dynamic> L11(mat, k, k, bs, bs);
    if (k > 0) {
      // A(k:k+bs, 0:k) = sum over rows m >= k of L(m, k:k+bs)^* L(m, 0:k), split at m = k+bs. The
      // first part is the TRMM L11^* B in place, done by column panels through the workspace.
      Block<MatrixType, Dynamic, Dynamic> B(mat, k, 0, bs, k);
      for (Index c = 0; c < k; c += block_size) {
        const Index w = numext::mini(block_size, k - c);
        work.topLeftCorner(bs, w) = B.middleCols(c, w);
        B.middleCols(c, w).noalias() = L11.adjoint().template triangularView<Upper>() * work.topLeftCorner(bs, w);
      }
      if (rs > 0) B.noalias() += mat.block(k + bs, k, rs, bs).adjoint() * mat.block(k + bs, 0, rs, k);
    }
    triangular_adjoint_square_unblocked(L11);
    if (rs > 0) {
      L11.template selfadjointView<Lower>().rankUpdate(mat.block(k + bs, k, rs, bs).adjoint());
      // Fused complex products can leave a rounding residual in the imaginary diagonal.
      EIGEN_IF_CONSTEXPR (NumTraits<typename MatrixType::Scalar>::IsComplex) {
        L11.diagonal() = L11.diagonal().real().template cast<typename MatrixType::Scalar>();
      }
    }
  }
}

template <int UpLo, bool IsLower = (int(UpLo) & int(Lower)) != 0>
struct triangular_adjoint_square_selector {
  template <typename MatrixType>
  static void run(MatrixType& mat) {
    triangular_adjoint_square_lower(mat);
  }
};

template <int UpLo>
struct triangular_adjoint_square_selector<UpLo, false> {
  template <typename MatrixType>
  static void run(MatrixType& mat) {
    Transpose<MatrixType> matt(mat);
    triangular_adjoint_square_lower(matt);
  }
};

/** \internal Replaces the \a UpLo triangle of \a mat, holding a triangular factor T, by the same
 * triangle of the self-adjoint product T^* T (\c Lower) or T T^* (\c Upper). The opposite triangle is
 * neither read nor written. This is LAPACK's xLAUUM. */
template <int UpLo, typename MatrixType>
void triangular_adjoint_square_in_place(MatrixType& mat) {
  triangular_adjoint_square_selector<UpLo>::run(mat);
}

}  // end namespace internal

#ifndef EIGEN_PARSED_BY_DOXYGEN
template <typename MatrixType, unsigned int Mode>
EIGEN_DEVICE_FUNC void TriangularViewImpl<MatrixType, Mode, Dense>::inverseInPlace() {
  EIGEN_STATIC_ASSERT_LVALUE(MatrixType)
  EIGEN_STATIC_ASSERT((int(Mode) & int(Upper | Lower)) != 0 && (int(Mode) & int(ZeroDiag)) == 0, PROGRAMMING_ERROR)
  eigen_assert(derived().rows() == derived().cols());
  internal::triangular_inverse_selector<Mode>::run(derived().nestedExpression());
}
#endif

}  // end namespace Eigen

#endif  // EIGEN_TRIANGULARINPLACE_H
