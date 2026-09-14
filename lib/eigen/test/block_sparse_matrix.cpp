// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include "sparse.h"

using namespace Eigen;

// ---------------------------------------------------------------------------
// Helper: build a BlockSparseMatrix from a dense matrix
// ---------------------------------------------------------------------------

// Treats each BlockRows x BlockCols tile of `dense` as one block whenever the
// tile is non-zero.
template <int BlockRows, int BlockCols, typename Scalar, int Options, typename StorageIndex>
BlockSparseMatrix<Scalar, Options, BlockRows, BlockCols, StorageIndex> denseToBlock(
    const Matrix<Scalar, Dynamic, Dynamic>& dense) {
  using BSM = BlockSparseMatrix<Scalar, Options, BlockRows, BlockCols, StorageIndex>;
  using Triplet = typename BSM::TripletType;

  Index bRows = dense.rows() / BlockRows;
  Index bCols = dense.cols() / BlockCols;

  std::vector<Triplet> triplets;
  for (Index bi = 0; bi < bRows; ++bi) {
    for (Index bj = 0; bj < bCols; ++bj) {
      Matrix<Scalar, BlockRows, BlockCols> tile = dense.block(bi * BlockRows, bj * BlockCols, BlockRows, BlockCols);
      if (tile.squaredNorm() > 0) triplets.emplace_back(StorageIndex(bi), StorageIndex(bj), tile);
    }
  }

  BSM bsm(bRows, bCols);
  bsm.setFromTriplets(triplets.begin(), triplets.end());
  return bsm;
}

template <bool HasSquareBlocks>
struct BlockSparseIdentityTester {
  template <int BlockRows, int BlockCols, int Options, typename Scalar, typename StorageIndex>
  static void run(int, int, const BlockSparseMatrix<Scalar, Options, BlockRows, BlockCols, StorageIndex>&,
                  const Matrix<Scalar, Dynamic, Dynamic>&) {}
};

template <>
struct BlockSparseIdentityTester<true> {
  template <int BlockRows, int BlockCols, int Options, typename Scalar, typename StorageIndex>
  static void run(int bRows, int bCols, const BlockSparseMatrix<Scalar, Options, BlockRows, BlockCols, StorageIndex>& A,
                  const Matrix<Scalar, Dynamic, Dynamic>& dA) {
    using BSM = BlockSparseMatrix<Scalar, Options, BlockRows, BlockCols, StorageIndex>;
    using DenseMat = Matrix<Scalar, Dynamic, Dynamic>;

    const int rows = bRows * BlockRows;
    const int cols = bCols * BlockCols;
    BSM Id(bRows, bCols);
    Id.setIdentity();
    VERIFY_IS_APPROX(DenseMat(Id.toSparse()), DenseMat::Identity(rows, cols));
    if (bRows == bCols) {
      VERIFY_IS_APPROX(DenseMat((Id * A).toSparse()), dA);
      VERIFY_IS_APPROX(DenseMat((A * Id).toSparse()), dA);
    }
  }
};

// ---------------------------------------------------------------------------
// Core test driver templated on block size, storage order, and scalar type
// ---------------------------------------------------------------------------

template <int BlockRows, int BlockCols, int Options, typename Scalar = double>
void test_block_sparse(int bRows, int bCols) {
  using StorageIndex = int;
  using BSM = BlockSparseMatrix<Scalar, Options, BlockRows, BlockCols, StorageIndex>;
  using SpMat = SparseMatrix<Scalar, Options, StorageIndex>;
  using DenseMat = Matrix<Scalar, Dynamic, Dynamic>;

  int rows = bRows * BlockRows;
  int cols = bCols * BlockCols;

  // Build two random dense matrices with a coarse block structure.
  DenseMat dA = DenseMat::Zero(rows, cols);
  DenseMat dB = DenseMat::Zero(rows, cols);
  for (int bi = 0; bi < bRows; ++bi) {
    for (int bj = 0; bj < bCols; ++bj) {
      if (internal::random<double>(0.0, 1.0) < 0.4) {
        dA.block(bi * BlockRows, bj * BlockCols, BlockRows, BlockCols) = DenseMat::Random(BlockRows, BlockCols);
      }
      if (internal::random<double>(0.0, 1.0) < 0.4) {
        dB.block(bi * BlockRows, bj * BlockCols, BlockRows, BlockCols) = DenseMat::Random(BlockRows, BlockCols);
      }
    }
  }

  BSM A = denseToBlock<BlockRows, BlockCols, Scalar, Options, StorageIndex>(dA);
  BSM B = denseToBlock<BlockRows, BlockCols, Scalar, Options, StorageIndex>(dB);

  // ---- toSparse / fromSparse round-trip -----------------------------------
  {
    SpMat spA = A.toSparse();
    DenseMat dense_spA(spA);
    VERIFY_IS_APPROX(dense_spA, dA);

    BSM A2 = BSM::fromSparse(spA);
    VERIFY_IS_APPROX(A2.toSparse(), spA);

    // Implicit conversion operator
    SpMat spA3 = A;
    VERIFY_IS_APPROX(DenseMat(spA3), dA);
  }

  // ---- Addition -----------------------------------------------------------
  {
    BSM C = A + B;
    DenseMat dC(C.toSparse());
    VERIFY_IS_APPROX(dC, dA + dB);

    BSM D = A;
    D += B;
    VERIFY_IS_APPROX(DenseMat(D.toSparse()), dA + dB);
  }

  // ---- Subtraction --------------------------------------------------------
  {
    BSM C = A - B;
    VERIFY_IS_APPROX(DenseMat(C.toSparse()), dA - dB);

    BSM D = A;
    D -= B;
    VERIFY_IS_APPROX(DenseMat(D.toSparse()), dA - dB);
  }

  // ---- Unary minus --------------------------------------------------------
  {
    BSM C = -A;
    VERIFY_IS_APPROX(DenseMat(C.toSparse()), -dA);
  }

  // ---- cwiseProduct (conjunction / intersection) --------------------------
  {
    BSM C = A.cwiseProduct(B);
    // Result must only have blocks where both A and B had blocks.
    DenseMat dC = dA.cwiseProduct(dB);
    VERIFY_IS_APPROX(DenseMat(C.toSparse()), dC);
  }

  // ---- unaryExpr ----------------------------------------------------------
  {
    BSM C = A.unaryExpr([](const Scalar& x) { return x * x; });
    VERIFY_IS_APPROX(DenseMat(C.toSparse()), dA.array().square().matrix());
  }

  // ---- disjunctionExpr (union sparsity) -----------------------------------
  {
    // sum via disjunctionExpr should match operator+
    struct AddExpr {
      Scalar operator()(const Scalar& a, const Scalar& b) const { return a + b; }
      Scalar lhs(const Scalar& a) const { return a; }
      Scalar rhs(const Scalar& b) const { return b; }
    };
    BSM C = A.disjunctionExpr(B, AddExpr{});
    VERIFY_IS_APPROX(DenseMat(C.toSparse()), dA + dB);
  }

  // ---- conjunctionExpr (intersection sparsity) ----------------------------
  {
    BSM C = A.conjunctionExpr(B, [](const Scalar& a, const Scalar& b) { return a * b; });
    VERIFY_IS_APPROX(DenseMat(C.toSparse()), dA.cwiseProduct(dB));
  }

  // ---- Scalar multiplication ----------------------------------------------
  {
    Scalar s = Scalar(3.14);
    BSM C = A * s;
    VERIFY_IS_APPROX(DenseMat(C.toSparse()), dA * s);

    BSM D = s * A;
    VERIFY_IS_APPROX(DenseMat(D.toSparse()), s * dA);

    BSM E = A;
    E *= s;
    VERIFY_IS_APPROX(DenseMat(E.toSparse()), dA * s);
  }

  // ---- Element access (coeff) ---------------------------------------------
  {
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        VERIFY_IS_APPROX(A.coeff(i, j), dA(i, j));
      }
    }
  }

  // ---- setIdentity ----------------------------------------------------------
  BlockSparseIdentityTester<BlockRows == BlockCols>::template run<BlockRows, BlockCols, Options, Scalar, StorageIndex>(
      bRows, bCols, A, dA);

  // ---- setFromTriplets with duplicate blocks (accumulation) ---------------
  {
    using Trip = typename BSM::TripletType;
    using BlockMat = Matrix<Scalar, BlockRows, BlockCols>;
    BlockMat half = BlockMat::Ones() * Scalar(0.5);

    std::vector<Trip> trips;
    trips.emplace_back(StorageIndex(0), StorageIndex(0), half);
    trips.emplace_back(StorageIndex(0), StorageIndex(0), half);  // duplicate -> sum

    BSM M(bRows, bCols);
    M.setFromTriplets(trips.begin(), trips.end());

    VERIFY(M.nonZeroBlocks() == 1);
    VERIFY_IS_APPROX(DenseMat(M.blockRef(0)), DenseMat(BlockMat::Ones()));
  }
}

// ---------------------------------------------------------------------------
// Square-block product test
// ---------------------------------------------------------------------------

template <int B, int Options, typename Scalar = double>
void test_block_sparse_product(int bM, int bK, int bN) {
  using StorageIndex = int;
  using BSMA = BlockSparseMatrix<Scalar, Options, B, B, StorageIndex>;
  using DenseMat = Matrix<Scalar, Dynamic, Dynamic>;

  int rowsA = bM * B, colsA = bK * B;
  int colsB = bN * B;

  DenseMat dA = DenseMat::Zero(rowsA, colsA);
  DenseMat dB = DenseMat::Zero(colsA, colsB);

  for (int bi = 0; bi < bM; ++bi)
    for (int bk = 0; bk < bK; ++bk)
      if (internal::random<double>(0.0, 1.0) < 0.4) dA.block(bi * B, bk * B, B, B) = DenseMat::Random(B, B);

  for (int bk = 0; bk < bK; ++bk)
    for (int bj = 0; bj < bN; ++bj)
      if (internal::random<double>(0.0, 1.0) < 0.4) dB.block(bk * B, bj * B, B, B) = DenseMat::Random(B, B);

  BSMA A = denseToBlock<B, B, Scalar, Options, StorageIndex>(dA);
  BSMA Bmat = denseToBlock<B, B, Scalar, Options, StorageIndex>(dB);

  BSMA C = A * Bmat;
  DenseMat dC(C.toSparse());
  VERIFY_IS_APPROX(dC, dA * dB);
}

// ---------------------------------------------------------------------------
// Block-sparse * dense and dense * block-sparse products
// ---------------------------------------------------------------------------

template <int BlockRows, int BlockCols, int Options, typename Scalar = double>
void test_block_sparse_dense_product(int bRows, int bCols) {
  using StorageIndex = int;
  using BSM = BlockSparseMatrix<Scalar, Options, BlockRows, BlockCols, StorageIndex>;
  using DenseMat = Matrix<Scalar, Dynamic, Dynamic>;
  using DenseVec = Matrix<Scalar, Dynamic, 1>;
  using RowVec = Matrix<Scalar, 1, Dynamic>;

  int rows = bRows * BlockRows;
  int cols = bCols * BlockCols;

  DenseMat dA = DenseMat::Zero(rows, cols);
  for (int bi = 0; bi < bRows; ++bi)
    for (int bj = 0; bj < bCols; ++bj)
      if (internal::random<double>(0.0, 1.0) < 0.5)
        dA.block(bi * BlockRows, bj * BlockCols, BlockRows, BlockCols) = DenseMat::Random(BlockRows, BlockCols);

  BSM A = denseToBlock<BlockRows, BlockCols, Scalar, Options, StorageIndex>(dA);

  // BSM * dense matrix
  {
    DenseMat rhs = DenseMat::Random(cols, 5);
    VERIFY_IS_APPROX(A * rhs, dA * rhs);
  }

  // BSM * column vector
  {
    DenseVec v = DenseVec::Random(cols);
    VERIFY_IS_APPROX(A * v, dA * v);
  }

  // dense matrix * BSM
  {
    DenseMat lhs = DenseMat::Random(5, rows);
    VERIFY_IS_APPROX(lhs * A, lhs * dA);
  }

  // row vector * BSM
  {
    RowVec v = RowVec::Random(rows);
    VERIFY_IS_APPROX(v * A, v * dA);
  }
}

// ---------------------------------------------------------------------------
// Non-square block product: A(BR x BC) * B(BC x BC2) -> C(BR x BC2)
// ---------------------------------------------------------------------------

void test_nonsquare_block_product() {
  using Scalar = double;
  using StorageIndex = int;
  constexpr int BR = 2, BC = 3, BC2 = 4;
  using BSMA = BlockSparseMatrix<Scalar, ColMajor, BR, BC, StorageIndex>;
  using BSMB = BlockSparseMatrix<Scalar, ColMajor, BC, BC2, StorageIndex>;
  using DenseMat = Matrix<Scalar, Dynamic, Dynamic>;

  int bM = 4, bK = 3, bN = 5;
  DenseMat dA = DenseMat::Zero(bM * BR, bK * BC);
  DenseMat dB = DenseMat::Zero(bK * BC, bN * BC2);

  for (int bi = 0; bi < bM; ++bi)
    for (int bk = 0; bk < bK; ++bk)
      if (internal::random<double>() > 0.0) dA.block(bi * BR, bk * BC, BR, BC) = DenseMat::Random(BR, BC);

  for (int bk = 0; bk < bK; ++bk)
    for (int bj = 0; bj < bN; ++bj)
      if (internal::random<double>() > 0.0) dB.block(bk * BC, bj * BC2, BC, BC2) = DenseMat::Random(BC, BC2);

  BSMA A = denseToBlock<BR, BC, Scalar, ColMajor, StorageIndex>(dA);
  BSMB Bmat = denseToBlock<BC, BC2, Scalar, ColMajor, StorageIndex>(dB);

  BlockSparseMatrix<Scalar, ColMajor, BR, BC2, StorageIndex> C = A * Bmat;
  DenseMat dC(C.toSparse());
  VERIFY_IS_APPROX(dC, dA * dB);
}

// ---------------------------------------------------------------------------
// Transpose and adjoint
// ---------------------------------------------------------------------------

template <int BlockRows, int BlockCols, int Options, typename Scalar = double>
void test_block_sparse_transpose(int bRows, int bCols) {
  using StorageIndex = int;
  using BSM = BlockSparseMatrix<Scalar, Options, BlockRows, BlockCols, StorageIndex>;
  using BSMT = BlockSparseMatrix<Scalar, Options, BlockCols, BlockRows, StorageIndex>;
  using DenseMat = Matrix<Scalar, Dynamic, Dynamic>;

  int rows = bRows * BlockRows, cols = bCols * BlockCols;
  DenseMat dA = DenseMat::Zero(rows, cols);
  for (int bi = 0; bi < bRows; ++bi)
    for (int bj = 0; bj < bCols; ++bj)
      if (internal::random<double>(0.0, 1.0) < 0.5)
        dA.block(bi * BlockRows, bj * BlockCols, BlockRows, BlockCols) = DenseMat::Random(BlockRows, BlockCols);

  BSM A = denseToBlock<BlockRows, BlockCols, Scalar, Options, StorageIndex>(dA);

  // transpose
  BSMT At = A.transpose();
  VERIFY_IS_APPROX(DenseMat(At.toSparse()), dA.transpose());

  // adjoint (conjugate transpose for complex, same as transpose for real)
  BSMT Ah = A.adjoint();
  VERIFY_IS_APPROX(DenseMat(Ah.toSparse()), dA.adjoint());

  // (A^T)^T == A
  BSM AtT = At.transpose();
  VERIFY_IS_APPROX(DenseMat(AtT.toSparse()), dA);
}

// ---------------------------------------------------------------------------
// Triangular view: eval, +/-, dense products, DiagIsTriangular path
// ---------------------------------------------------------------------------

template <int B, int Options, typename Scalar = double>
void test_block_sparse_triangular(int bN) {
  using StorageIndex = int;
  using BSM = BlockSparseMatrix<Scalar, Options, B, B, StorageIndex>;
  using DenseMat = Matrix<Scalar, Dynamic, Dynamic>;

  int N = bN * B;
  DenseMat dA = DenseMat::Zero(N, N);
  for (int bi = 0; bi < bN; ++bi)
    for (int bj = 0; bj < bN; ++bj)
      if (internal::random<double>(0.0, 1.0) < 0.5) dA.block(bi * B, bj * B, B, B) = DenseMat::Random(B, B);

  BSM A = denseToBlock<B, B, Scalar, Options, StorageIndex>(dA);

  // Upper eval
  {
    BSM Au = A.template triangularView<Upper>().eval();
    DenseMat dAu = dA;
    for (int bi = 0; bi < bN; ++bi) {
      for (int bj = 0; bj < bi; ++bj) dAu.block(bi * B, bj * B, B, B).setZero();
      dAu.block(bi * B, bi * B, B, B).template triangularView<StrictlyLower>().setZero();
    }
    VERIFY_IS_APPROX(DenseMat(Au.toSparse()), dAu);
  }

  // Lower eval
  {
    BSM Al = A.template triangularView<Lower>().eval();
    DenseMat dAl = dA;
    for (int bi = 0; bi < bN; ++bi) {
      for (int bj = bi + 1; bj < bN; ++bj) dAl.block(bi * B, bj * B, B, B).setZero();
      dAl.block(bi * B, bi * B, B, B).template triangularView<StrictlyUpper>().setZero();
    }
    VERIFY_IS_APPROX(DenseMat(Al.toSparse()), dAl);
  }

  // Tri * dense and dense * Tri
  {
    DenseMat dAu = dA;
    for (int bi = 0; bi < bN; ++bi) {
      for (int bj = 0; bj < bi; ++bj) dAu.block(bi * B, bj * B, B, B).setZero();
      dAu.block(bi * B, bi * B, B, B).template triangularView<StrictlyLower>().setZero();
    }

    DenseMat rhs = DenseMat::Random(N, 4);
    DenseMat lhs = DenseMat::Random(3, N);

    VERIFY_IS_APPROX(A.template triangularView<Upper>() * rhs, dAu * rhs);
    VERIFY_IS_APPROX(lhs * A.template triangularView<Upper>(), lhs * dAu);
  }

  // Tri + Tri
  {
    DenseMat dB = DenseMat::Zero(N, N);
    for (int bi = 0; bi < bN; ++bi)
      for (int bj = 0; bj < bN; ++bj)
        if (internal::random<double>(0.0, 1.0) < 0.5) dB.block(bi * B, bj * B, B, B) = DenseMat::Random(B, B);
    BSM Bmat = denseToBlock<B, B, Scalar, Options, StorageIndex>(dB);

    DenseMat dAu = dA, dBu = dB;
    for (int bi = 0; bi < bN; ++bi) {
      for (int bj = 0; bj < bi; ++bj) {
        dAu.block(bi * B, bj * B, B, B).setZero();
        dBu.block(bi * B, bj * B, B, B).setZero();
      }
      dAu.block(bi * B, bi * B, B, B).template triangularView<StrictlyLower>().setZero();
      dBu.block(bi * B, bi * B, B, B).template triangularView<StrictlyLower>().setZero();
    }
    BSM C = A.template triangularView<Upper>() + Bmat.template triangularView<Upper>();
    VERIFY_IS_APPROX(DenseMat(C.toSparse()), dAu + dBu);
  }

  // DiagIsTriangular=true product: build a BSM whose diagonal blocks are already
  // upper-triangular in storage, and verify the product matches DiagIsTriangular=false.
  {
    DenseMat dAu = DenseMat::Zero(N, N);
    for (int bi = 0; bi < bN; ++bi) {
      DenseMat blk = DenseMat::Random(B, B);
      blk.template triangularView<StrictlyLower>().setZero();
      dAu.block(bi * B, bi * B, B, B) = blk;
      for (int bj = bi + 1; bj < bN; ++bj)
        if (internal::random<double>(0.0, 1.0) < 0.5) dAu.block(bi * B, bj * B, B, B) = DenseMat::Random(B, B);
    }
    BSM Au = denseToBlock<B, B, Scalar, Options, StorageIndex>(dAu);
    DenseMat rhs = DenseMat::Random(N, 3);
    DenseMat r1 = Au.template triangularView<Upper, false>() * rhs;
    DenseMat r2 = Au.template triangularView<Upper, true>() * rhs;
    VERIFY_IS_APPROX(r1, r2);
  }
}

// ---------------------------------------------------------------------------
// Triangular solve: forward/backward x direct/transposed/adjoint x both layouts
// ---------------------------------------------------------------------------

template <int B, int Options, typename Scalar>
void test_block_sparse_triangular_solve(int bN) {
  using StorageIndex = int;
  using BSM = BlockSparseMatrix<Scalar, Options, B, B, StorageIndex>;
  using DenseMat = Matrix<Scalar, Dynamic, Dynamic>;
  using RealScalar = typename NumTraits<Scalar>::Real;

  int N = bN * B;

  // Build a dense lower-block-triangular matrix.
  // Diagonal blocks are lower triangular with non-zero diagonal.
  // Off-diagonal (lower) blocks are random dense.
  auto makeDenseLower = [&]() {
    DenseMat dL = DenseMat::Zero(N, N);
    for (int bi = 0; bi < bN; ++bi) {
      DenseMat blk = DenseMat::Random(B, B);
      blk.template triangularView<StrictlyUpper>().setZero();
      for (int k = 0; k < B; ++k) blk(k, k) = Scalar(RealScalar(B + k + 1));
      dL.block(bi * B, bi * B, B, B) = blk;
      for (int bj = 0; bj < bi; ++bj)
        if (internal::random<double>(0.0, 1.0) < 0.6) dL.block(bi * B, bj * B, B, B) = DenseMat::Random(B, B);
    }
    return dL;
  };

  auto makeDenseUpper = [&]() {
    DenseMat dU = DenseMat::Zero(N, N);
    for (int bi = 0; bi < bN; ++bi) {
      DenseMat blk = DenseMat::Random(B, B);
      blk.template triangularView<StrictlyLower>().setZero();
      for (int k = 0; k < B; ++k) blk(k, k) = Scalar(RealScalar(B + k + 1));
      dU.block(bi * B, bi * B, B, B) = blk;
      for (int bj = bi + 1; bj < bN; ++bj)
        if (internal::random<double>(0.0, 1.0) < 0.6) dU.block(bi * B, bj * B, B, B) = DenseMat::Random(B, B);
    }
    return dU;
  };

  // Lower triangular: direct, transposed, adjoint
  {
    DenseMat dL = makeDenseLower();
    BSM L = denseToBlock<B, B, Scalar, Options, StorageIndex>(dL);

    // L x = b
    {
      DenseMat b = DenseMat::Random(N, 3);
      DenseMat x = b;
      L.template triangularView<Lower>().solveInPlace(x);
      VERIFY_IS_APPROX(dL * x, b);
    }
    // L^T x = b
    {
      DenseMat b = DenseMat::Random(N, 3);
      DenseMat x = b;
      L.template triangularView<Lower>().transpose().solveInPlace(x);
      VERIFY_IS_APPROX(dL.transpose() * x, b);
    }
    // L^H x = b
    {
      DenseMat b = DenseMat::Random(N, 3);
      DenseMat x = b;
      L.template triangularView<Lower>().adjoint().solveInPlace(x);
      VERIFY_IS_APPROX(dL.adjoint() * x, b);
    }
  }

  // Upper triangular: direct, transposed, adjoint
  {
    DenseMat dU = makeDenseUpper();
    BSM U = denseToBlock<B, B, Scalar, Options, StorageIndex>(dU);

    // U x = b
    {
      DenseMat b = DenseMat::Random(N, 3);
      DenseMat x = b;
      U.template triangularView<Upper>().solveInPlace(x);
      VERIFY_IS_APPROX(dU * x, b);
    }
    // U^T x = b
    {
      DenseMat b = DenseMat::Random(N, 3);
      DenseMat x = b;
      U.template triangularView<Upper>().transpose().solveInPlace(x);
      VERIFY_IS_APPROX(dU.transpose() * x, b);
    }
    // U^H x = b
    {
      DenseMat b = DenseMat::Random(N, 3);
      DenseMat x = b;
      U.template triangularView<Upper>().adjoint().solveInPlace(x);
      VERIFY_IS_APPROX(dU.adjoint() * x, b);
    }
  }

  // DiagIsTriangular=true: diagonal blocks are already properly triangular in storage;
  // result must match DiagIsTriangular=false (which zeroes the unused triangle first).
  {
    DenseMat dL = makeDenseLower();
    BSM L = denseToBlock<B, B, Scalar, Options, StorageIndex>(dL);
    DenseMat b = DenseMat::Random(N, 2);
    DenseMat x1 = b, x2 = b;
    {
      auto tri_false = L.template triangularView<Lower, false>();
      tri_false.solveInPlace(x1);
    }
    {
      auto tri_true = L.template triangularView<Lower, true>();
      tri_true.solveInPlace(x2);
    }
    VERIFY_IS_APPROX(x1, x2);
  }
}

// ---------------------------------------------------------------------------
// Self-adjoint view: eval, +/-, dense products
// ---------------------------------------------------------------------------

template <int B, int Options, typename Scalar = double>
void test_block_sparse_selfadjoint(int bN) {
  using StorageIndex = int;
  using BSM = BlockSparseMatrix<Scalar, Options, B, B, StorageIndex>;
  using DenseMat = Matrix<Scalar, Dynamic, Dynamic>;

  int N = bN * B;

  // Build a Hermitian dense matrix (stored upper triangle only).
  DenseMat dFull = DenseMat::Zero(N, N);
  for (int bi = 0; bi < bN; ++bi) {
    // Diagonal block: Hermitian (symmetric for real, conjugate-symmetric for complex).
    DenseMat blk = DenseMat::Random(B, B);
    blk = (blk + blk.adjoint()).eval();
    dFull.block(bi * B, bi * B, B, B) = blk;
    for (int bj = bi + 1; bj < bN; ++bj)
      if (internal::random<double>(0.0, 1.0) < 0.5) {
        DenseMat offblk = DenseMat::Random(B, B);
        dFull.block(bi * B, bj * B, B, B) = offblk;
        dFull.block(bj * B, bi * B, B, B) = offblk.adjoint();  // conjugate transpose mirror
      }
  }

  // Build BSM from the upper triangle only.
  DenseMat dUpper = DenseMat::Zero(N, N);
  for (int bi = 0; bi < bN; ++bi)
    for (int bj = bi; bj < bN; ++bj) dUpper.block(bi * B, bj * B, B, B) = dFull.block(bi * B, bj * B, B, B);

  BSM A = denseToBlock<B, B, Scalar, Options, StorageIndex>(dUpper);

  // eval() must reproduce the full Hermitian matrix
  {
    BSM Asym = A.template selfadjointView<Upper>().eval();
    VERIFY_IS_APPROX(DenseMat(Asym.toSparse()), dFull);
  }

  // selfadjointView * dense
  {
    DenseMat rhs = DenseMat::Random(N, 5);
    VERIFY_IS_APPROX(A.template selfadjointView<Upper>() * rhs, dFull * rhs);
  }

  // dense * selfadjointView
  {
    DenseMat lhs = DenseMat::Random(4, N);
    VERIFY_IS_APPROX(lhs * A.template selfadjointView<Upper>(), lhs * dFull);
  }

  // selfadjointView + selfadjointView
  {
    BSM Bmat = denseToBlock<B, B, Scalar, Options, StorageIndex>(dUpper * Scalar(2));
    BSM C = A.template selfadjointView<Upper>() + Bmat.template selfadjointView<Upper>();
    VERIFY_IS_APPROX(DenseMat(C.toSparse()), dFull * Scalar(3));
  }

  // DiagIsSelfAdjoint path: diagonal blocks ARE Hermitian, product should match
  {
    DenseMat rhs = DenseMat::Random(N, 3);
    DenseMat result = A.template selfadjointView<Upper, true>() * rhs;
    VERIFY_IS_APPROX(result, dFull * rhs);
  }
}

// ---------------------------------------------------------------------------
// BlockTriplet type-trait checks
// ---------------------------------------------------------------------------

void test_block_triplet_traits() {
  // Flat scalar array means BlockTriplet should be trivially copyable and
  // standard-layout for any trivially-copyable Scalar and StorageIndex.
  EIGEN_STATIC_ASSERT((std::is_trivially_copyable<BlockTriplet<float, 2, 2>>::value),
                      BLOCKTRIPLET_MUST_BE_TRIVIALLY_COPYABLE)
  EIGEN_STATIC_ASSERT((std::is_trivially_copyable<BlockTriplet<double, 3, 3>>::value),
                      BLOCKTRIPLET_MUST_BE_TRIVIALLY_COPYABLE)
  EIGEN_STATIC_ASSERT((std::is_trivially_copyable<BlockTriplet<float, 2, 3>>::value),
                      BLOCKTRIPLET_MUST_BE_TRIVIALLY_COPYABLE)
  EIGEN_STATIC_ASSERT((std::is_standard_layout<BlockTriplet<float, 2, 2>>::value), BLOCKTRIPLET_MUST_BE_STANDARD_LAYOUT)
  EIGEN_STATIC_ASSERT((std::is_standard_layout<BlockTriplet<double, 4, 4>>::value),
                      BLOCKTRIPLET_MUST_BE_STANDARD_LAYOUT)

  // No alignment padding: size must equal 2*sizeof(StorageIndex) + BlockSize*sizeof(Scalar).
  EIGEN_STATIC_ASSERT((sizeof(BlockTriplet<float, 2, 2>) == 2 * sizeof(int) + 4 * sizeof(float)),
                      BLOCKTRIPLET_MUST_HAVE_NO_ALIGNMENT_PADDING)
  EIGEN_STATIC_ASSERT((sizeof(BlockTriplet<double, 2, 2>) == 2 * sizeof(int) + 4 * sizeof(double)),
                      BLOCKTRIPLET_MUST_HAVE_NO_ALIGNMENT_PADDING)
  EIGEN_STATIC_ASSERT((sizeof(BlockTriplet<float, 4, 4>) == 2 * sizeof(int) + 16 * sizeof(float)),
                      BLOCKTRIPLET_MUST_HAVE_NO_ALIGNMENT_PADDING)
  EIGEN_STATIC_ASSERT((sizeof(BlockTriplet<double, 4, 4>) == 2 * sizeof(int) + 16 * sizeof(double)),
                      BLOCKTRIPLET_MUST_HAVE_NO_ALIGNMENT_PADDING)
}

// ---------------------------------------------------------------------------
// Main entry point
// ---------------------------------------------------------------------------

EIGEN_DECLARE_TEST(block_sparse_matrix) {
  // ColMajor, real double, various block sizes and matrix sizes
  CALL_SUBTEST_1((test_block_sparse<1, 1, ColMajor>(6, 8)));
  CALL_SUBTEST_2((test_block_sparse<2, 2, ColMajor>(4, 6)));
  CALL_SUBTEST_3((test_block_sparse<3, 3, ColMajor>(4, 5)));
  CALL_SUBTEST_4((test_block_sparse<4, 4, ColMajor>(3, 3)));
  CALL_SUBTEST_5((test_block_sparse<2, 3, ColMajor>(5, 4)));

  // RowMajor, real double
  CALL_SUBTEST_6((test_block_sparse<2, 2, RowMajor>(4, 6)));
  CALL_SUBTEST_7((test_block_sparse<3, 3, RowMajor>(4, 5)));
  CALL_SUBTEST_8((test_block_sparse<2, 3, RowMajor>(5, 4)));

  // Products (ColMajor)
  CALL_SUBTEST_9((test_block_sparse_product<2, ColMajor>(4, 5, 3)));
  CALL_SUBTEST_9((test_block_sparse_product<3, ColMajor>(3, 4, 5)));

  // Products (RowMajor)
  CALL_SUBTEST_10((test_block_sparse_product<2, RowMajor>(4, 5, 3)));
  CALL_SUBTEST_10((test_block_sparse_product<3, RowMajor>(3, 4, 5)));

  // Non-square block product
  CALL_SUBTEST_11(test_nonsquare_block_product());

  // Block-sparse * dense and dense * block-sparse (ColMajor)
  CALL_SUBTEST_12((test_block_sparse_dense_product<2, 2, ColMajor>(4, 5)));
  CALL_SUBTEST_12((test_block_sparse_dense_product<3, 3, ColMajor>(3, 4)));
  CALL_SUBTEST_12((test_block_sparse_dense_product<2, 3, ColMajor>(4, 3)));

  // Block-sparse * dense and dense * block-sparse (RowMajor)
  CALL_SUBTEST_13((test_block_sparse_dense_product<2, 2, RowMajor>(4, 5)));
  CALL_SUBTEST_13((test_block_sparse_dense_product<3, 3, RowMajor>(3, 4)));
  CALL_SUBTEST_13((test_block_sparse_dense_product<2, 3, RowMajor>(4, 3)));

  // Transpose / adjoint (ColMajor and RowMajor, square and non-square blocks)
  CALL_SUBTEST_14((test_block_sparse_transpose<2, 2, ColMajor>(4, 5)));
  CALL_SUBTEST_14((test_block_sparse_transpose<2, 3, ColMajor>(5, 4)));
  CALL_SUBTEST_14((test_block_sparse_transpose<2, 2, RowMajor>(4, 5)));
  CALL_SUBTEST_14((test_block_sparse_transpose<3, 2, RowMajor>(4, 5)));

  // Triangular view (ColMajor and RowMajor)
  CALL_SUBTEST_15((test_block_sparse_triangular<2, ColMajor>(5)));
  CALL_SUBTEST_15((test_block_sparse_triangular<3, ColMajor>(4)));
  CALL_SUBTEST_15((test_block_sparse_triangular<2, RowMajor>(5)));

  // Self-adjoint view (ColMajor and RowMajor)
  CALL_SUBTEST_16((test_block_sparse_selfadjoint<2, ColMajor>(5)));
  CALL_SUBTEST_16((test_block_sparse_selfadjoint<3, ColMajor>(4)));
  CALL_SUBTEST_16((test_block_sparse_selfadjoint<2, RowMajor>(5)));

  // BlockTriplet type traits
  CALL_SUBTEST_17(test_block_triplet_traits());

  // Complex scalar coverage: conjugation paths in adjoint, selfadjoint, triangular products
  CALL_SUBTEST_18((test_block_sparse<2, 2, ColMajor, std::complex<double>>(4, 6)));
  CALL_SUBTEST_18((test_block_sparse<3, 3, ColMajor, std::complex<double>>(4, 5)));
  CALL_SUBTEST_18((test_block_sparse<2, 2, RowMajor, std::complex<double>>(4, 6)));
  CALL_SUBTEST_18((test_block_sparse_dense_product<2, 2, ColMajor, std::complex<double>>(4, 5)));
  CALL_SUBTEST_18((test_block_sparse_dense_product<2, 2, RowMajor, std::complex<double>>(4, 5)));
  CALL_SUBTEST_18((test_block_sparse_transpose<2, 2, ColMajor, std::complex<double>>(4, 5)));
  CALL_SUBTEST_18((test_block_sparse_transpose<2, 3, ColMajor, std::complex<double>>(5, 4)));
  CALL_SUBTEST_18((test_block_sparse_selfadjoint<2, ColMajor, std::complex<double>>(5)));
  CALL_SUBTEST_18((test_block_sparse_selfadjoint<3, ColMajor, std::complex<double>>(4)));
  CALL_SUBTEST_18((test_block_sparse_selfadjoint<2, RowMajor, std::complex<double>>(5)));
  CALL_SUBTEST_18((test_block_sparse_triangular<2, ColMajor, std::complex<double>>(5)));
  CALL_SUBTEST_18((test_block_sparse_triangular<2, RowMajor, std::complex<double>>(5)));

  // Triangular solve: forward/backward x direct/transposed/adjoint x ColMajor+RowMajor
  CALL_SUBTEST_19((test_block_sparse_triangular_solve<2, ColMajor, double>(5)));
  CALL_SUBTEST_19((test_block_sparse_triangular_solve<3, ColMajor, double>(4)));
  CALL_SUBTEST_19((test_block_sparse_triangular_solve<2, RowMajor, double>(5)));
  CALL_SUBTEST_19((test_block_sparse_triangular_solve<3, RowMajor, double>(4)));
  CALL_SUBTEST_19((test_block_sparse_triangular_solve<2, ColMajor, std::complex<double>>(5)));
  CALL_SUBTEST_19((test_block_sparse_triangular_solve<3, ColMajor, std::complex<double>>(4)));
  CALL_SUBTEST_19((test_block_sparse_triangular_solve<2, RowMajor, std::complex<double>>(5)));
  CALL_SUBTEST_19((test_block_sparse_triangular_solve<3, RowMajor, std::complex<double>>(4)));
}
