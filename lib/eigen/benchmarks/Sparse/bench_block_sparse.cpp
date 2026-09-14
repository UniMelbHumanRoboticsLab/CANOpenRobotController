// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

// Benchmark: SparseMatrix vs BlockSparseMatrix, real and complex scalars.
//
// Naming convention:
//   BM_Sm_*         — SparseMatrix (standard Eigen sparse)
//   BM_BSM_*        — BlockSparseMatrix
//   *_SpMV          — sparse × dense vector (GEMV)
//   *_TriMV         — triangular-view × dense vector
//   *_SymmMV        — selfadjoint-view × dense vector
//   *_TriSolve      — triangular solve in-place
//   BM_Sm_Sm_*      — SparseMatrix × SparseMatrix
//   BM_BSM_BSM_*    — BlockSparseMatrix × BlockSparseMatrix
//   DiagT           — DiagIsTriangular=true (diagonal blocks are actually triangular)
//   DiagNSA         — DiagIsSelfAdjoint=false with Hermitian diagonal blocks
//   DiagSA          — DiagIsSelfAdjoint=true with Hermitian diagonal blocks

#include <benchmark/benchmark.h>
#include <Eigen/Sparse>

#include <complex>
#include <random>
#include <set>

using namespace Eigen;
using cd = std::complex<double>;
using cf = std::complex<float>;

// ---------------------------------------------------------------------------
// Decode benchmark args: range(0)=nB, range(1)=sparsity% → nnzPerCol.
// ---------------------------------------------------------------------------
static void parseArgs(const benchmark::State& state, int& nB, int& nnz) {
  nB = state.range(0);
  nnz = std::max(1, (int)(state.range(1) * nB / 100));
}

// ---------------------------------------------------------------------------
// Random-value helper — works for float, double, complex<float>, complex<double>.
// ---------------------------------------------------------------------------
template <typename Scalar>
static typename std::enable_if<std::is_floating_point<Scalar>::value, Scalar>::type randVal(
    std::mt19937& rng, std::normal_distribution<double>& d) {
  return Scalar(d(rng));
}

template <typename Scalar>
static typename std::enable_if<!std::is_floating_point<Scalar>::value, Scalar>::type randVal(
    std::mt19937& rng, std::normal_distribution<double>& d) {
  using R = typename Scalar::value_type;
  return Scalar{R(d(rng)), R(d(rng))};
}

// ---------------------------------------------------------------------------
// Build a general (full) block-sparse pair.
// ---------------------------------------------------------------------------
template <typename Scalar, int B>
static void buildPair(int nB, int nnzPerCol, unsigned seed, BlockSparseMatrix<Scalar, ColMajor, B, B>& bsm,
                      SparseMatrix<Scalar>& sm) {
  using BSM = BlockSparseMatrix<Scalar, ColMajor, B, B>;
  using BT = typename BSM::BlockType;
  using Triplet = typename BSM::TripletType;

  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> rowDist(0, nB - 1);
  std::normal_distribution<double> vd;

  std::vector<Triplet> triplets;
  triplets.reserve(nB * nnzPerCol);
  for (int j = 0; j < nB; ++j) {
    std::set<int> rows;
    rows.insert(j % nB);
    while ((int)rows.size() < std::min(nnzPerCol, nB)) rows.insert(rowDist(rng));
    for (int bi : rows) {
      BT blk;
      for (int r = 0; r < B; ++r)
        for (int c = 0; c < B; ++c) blk(r, c) = randVal<Scalar>(rng, vd);
      triplets.emplace_back(bi, j, blk);
    }
  }
  bsm = BSM(nB, nB);
  bsm.setFromTriplets(triplets.begin(), triplets.end());
  sm = bsm.toSparse();
}

// ---------------------------------------------------------------------------
// Build an upper-triangular block-sparse pair (diagonal blocks are general).
// forSolve=true → diagonal blocks diagonally dominant (well-conditioned).
// ---------------------------------------------------------------------------
template <typename Scalar, int B>
static void buildUpperTriPair(int nB, int nnzPerCol, unsigned seed, BlockSparseMatrix<Scalar, ColMajor, B, B>& bsm,
                              SparseMatrix<Scalar>& sm, bool forSolve = false) {
  using BSM = BlockSparseMatrix<Scalar, ColMajor, B, B>;
  using BT = typename BSM::BlockType;
  using Triplet = typename BSM::TripletType;

  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> rowDist(0, nB - 1);
  std::normal_distribution<double> vd;

  std::vector<Triplet> triplets;
  triplets.reserve(nB * nnzPerCol);
  for (int j = 0; j < nB; ++j) {
    BT diag;
    for (int r = 0; r < B; ++r)
      for (int c = 0; c < B; ++c) diag(r, c) = randVal<Scalar>(rng, vd);
    if (forSolve) {
      diag *= Scalar(0.1);
      for (int d = 0; d < B; ++d) diag(d, d) += Scalar(double(B));
    }
    triplets.emplace_back(j, j, diag);

    std::set<int> rows;
    while ((int)rows.size() < std::min(nnzPerCol - 1, j)) rows.insert(rowDist(rng) % j);
    for (int bi : rows) {
      BT blk;
      for (int r = 0; r < B; ++r)
        for (int c = 0; c < B; ++c) blk(r, c) = randVal<Scalar>(rng, vd);
      triplets.emplace_back(bi, j, blk);
    }
  }
  bsm = BSM(nB, nB);
  bsm.setFromTriplets(triplets.begin(), triplets.end());
  sm = bsm.toSparse();
}

// ---------------------------------------------------------------------------
// Build an upper-triangular pair with actually-triangular diagonal blocks
// (strict lower triangle zeroed). Valid for DiagIsTriangular=true.
// ---------------------------------------------------------------------------
template <typename Scalar, int B>
static void buildActuallyTriPair(int nB, int nnzPerCol, unsigned seed, BlockSparseMatrix<Scalar, ColMajor, B, B>& bsm,
                                 SparseMatrix<Scalar>& sm) {
  using BSM = BlockSparseMatrix<Scalar, ColMajor, B, B>;
  using BT = typename BSM::BlockType;
  using Triplet = typename BSM::TripletType;

  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> rowDist(0, nB - 1);
  std::normal_distribution<double> vd;

  std::vector<Triplet> triplets;
  triplets.reserve(nB * nnzPerCol);
  for (int j = 0; j < nB; ++j) {
    BT diag = BT::Zero();
    for (int r = 0; r < B; ++r)
      for (int c = r; c < B; ++c)  // upper triangle only
        diag(r, c) = randVal<Scalar>(rng, vd);
    triplets.emplace_back(j, j, diag);

    std::set<int> rows;
    while ((int)rows.size() < std::min(nnzPerCol - 1, j)) rows.insert(rowDist(rng) % j);
    for (int bi : rows) {
      BT blk;
      for (int r = 0; r < B; ++r)
        for (int c = 0; c < B; ++c) blk(r, c) = randVal<Scalar>(rng, vd);
      triplets.emplace_back(bi, j, blk);
    }
  }
  bsm = BSM(nB, nB);
  bsm.setFromTriplets(triplets.begin(), triplets.end());
  sm = bsm.toSparse();
}

// ---------------------------------------------------------------------------
// Build an upper-triangular pair with Hermitian diagonal blocks.
// Valid for DiagIsSelfAdjoint=true.
// ---------------------------------------------------------------------------
template <typename Scalar, int B>
static void buildHermDiagUpperTriPair(int nB, int nnzPerCol, unsigned seed,
                                      BlockSparseMatrix<Scalar, ColMajor, B, B>& bsm, SparseMatrix<Scalar>& sm) {
  using BSM = BlockSparseMatrix<Scalar, ColMajor, B, B>;
  using BT = typename BSM::BlockType;
  using Triplet = typename BSM::TripletType;

  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> rowDist(0, nB - 1);
  std::normal_distribution<double> vd;

  std::vector<Triplet> triplets;
  triplets.reserve(nB * nnzPerCol);
  for (int j = 0; j < nB; ++j) {
    BT raw;
    for (int r = 0; r < B; ++r)
      for (int c = 0; c < B; ++c) raw(r, c) = randVal<Scalar>(rng, vd);
    BT diag = (raw + raw.adjoint()) / Scalar(2);
    triplets.emplace_back(j, j, diag);

    std::set<int> rows;
    while ((int)rows.size() < std::min(nnzPerCol - 1, j)) rows.insert(rowDist(rng) % j);
    for (int bi : rows) {
      BT blk;
      for (int r = 0; r < B; ++r)
        for (int c = 0; c < B; ++c) blk(r, c) = randVal<Scalar>(rng, vd);
      triplets.emplace_back(bi, j, blk);
    }
  }
  bsm = BSM(nB, nB);
  bsm.setFromTriplets(triplets.begin(), triplets.end());
  sm = bsm.toSparse();
}

// ---------------------------------------------------------------------------
// Sparse×Sparse: Addition
// ---------------------------------------------------------------------------
template <typename Scalar, int B>
static void BM_Sm_Sm_Add(benchmark::State& state) {
  int nB, nnz;
  parseArgs(state, nB, nnz);
  SparseMatrix<Scalar> smA, smB, smC;
  BlockSparseMatrix<Scalar, ColMajor, B, B> tmp;
  buildPair<Scalar, B>(nB, nnz, 1, tmp, smA);
  buildPair<Scalar, B>(nB, nnz, 2, tmp, smB);
  for (auto _ : state) {
    smC = smA + smB;
    benchmark::DoNotOptimize(smC.valuePtr());
  }
  state.counters["n"] = smA.rows();
}

template <typename Scalar, int B>
static void BM_BSM_BSM_Add(benchmark::State& state) {
  int nB, nnz;
  parseArgs(state, nB, nnz);
  using BSM = BlockSparseMatrix<Scalar, ColMajor, B, B>;
  BSM bsmA, bsmB, bsmC;
  SparseMatrix<Scalar> smTmp;
  buildPair<Scalar, B>(nB, nnz, 1, bsmA, smTmp);
  buildPair<Scalar, B>(nB, nnz, 2, bsmB, smTmp);
  for (auto _ : state) {
    bsmC = bsmA + bsmB;
    benchmark::DoNotOptimize(bsmC.valuePtr());
  }
  state.counters["n"] = bsmA.rows();
}

// ---------------------------------------------------------------------------
// Sparse×Dense: GEMV
// ---------------------------------------------------------------------------
template <typename Scalar, int B>
static void BM_Sm_SpMV(benchmark::State& state) {
  int nB, nnz;
  parseArgs(state, nB, nnz);
  SparseMatrix<Scalar> sm;
  BlockSparseMatrix<Scalar, ColMajor, B, B> tmp;
  buildPair<Scalar, B>(nB, nnz, 1, tmp, sm);
  Matrix<Scalar, Dynamic, 1> x = Matrix<Scalar, Dynamic, 1>::Random(sm.cols());
  Matrix<Scalar, Dynamic, 1> y(sm.rows());
  for (auto _ : state) {
    y.noalias() = sm * x;
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["n"] = sm.rows();
}

template <typename Scalar, int B>
static void BM_BSM_SpMV(benchmark::State& state) {
  int nB, nnz;
  parseArgs(state, nB, nnz);
  using BSM = BlockSparseMatrix<Scalar, ColMajor, B, B>;
  BSM bsm;
  SparseMatrix<Scalar> smTmp;
  buildPair<Scalar, B>(nB, nnz, 1, bsm, smTmp);
  Matrix<Scalar, Dynamic, 1> x = Matrix<Scalar, Dynamic, 1>::Random(bsm.cols());
  Matrix<Scalar, Dynamic, 1> y(bsm.rows());
  for (auto _ : state) {
    y.noalias() = bsm * x;
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["n"] = bsm.rows();
}

// Accumulating GEMV: y += A * x.
template <typename Scalar, int B>
static void BM_Sm_SpMV_Acc(benchmark::State& state) {
  int nB, nnz;
  parseArgs(state, nB, nnz);
  SparseMatrix<Scalar> sm;
  BlockSparseMatrix<Scalar, ColMajor, B, B> tmp;
  buildPair<Scalar, B>(nB, nnz, 1, tmp, sm);
  Matrix<Scalar, Dynamic, 1> x = Matrix<Scalar, Dynamic, 1>::Random(sm.cols());
  Matrix<Scalar, Dynamic, 1> y = Matrix<Scalar, Dynamic, 1>::Random(sm.rows());
  for (auto _ : state) {
    y.noalias() += sm * x;
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["n"] = sm.rows();
}

template <typename Scalar, int B>
static void BM_BSM_SpMV_Acc(benchmark::State& state) {
  int nB, nnz;
  parseArgs(state, nB, nnz);
  using BSM = BlockSparseMatrix<Scalar, ColMajor, B, B>;
  BSM bsm;
  SparseMatrix<Scalar> smTmp;
  buildPair<Scalar, B>(nB, nnz, 1, bsm, smTmp);
  Matrix<Scalar, Dynamic, 1> x = Matrix<Scalar, Dynamic, 1>::Random(bsm.cols());
  Matrix<Scalar, Dynamic, 1> y = Matrix<Scalar, Dynamic, 1>::Random(bsm.rows());
  for (auto _ : state) {
    y.noalias() += bsm * x;
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["n"] = bsm.rows();
}

// ---------------------------------------------------------------------------
// Sparse×Dense: Triangular MV
// ---------------------------------------------------------------------------
template <typename Scalar, int B>
static void BM_Sm_TriMV(benchmark::State& state) {
  int nB, nnz;
  parseArgs(state, nB, nnz);
  SparseMatrix<Scalar> sm;
  BlockSparseMatrix<Scalar, ColMajor, B, B> tmp;
  buildUpperTriPair<Scalar, B>(nB, nnz, 1, tmp, sm);
  Matrix<Scalar, Dynamic, 1> x = Matrix<Scalar, Dynamic, 1>::Random(sm.cols());
  Matrix<Scalar, Dynamic, 1> y(sm.rows());
  for (auto _ : state) {
    y.noalias() = sm.template triangularView<Upper>() * x;
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["n"] = sm.rows();
}

// BSM triangular view, DiagIsTriangular=false: diagonal blocks treated as triangular via triangularView<>.
template <typename Scalar, int B>
static void BM_BSM_TriMV(benchmark::State& state) {
  int nB, nnz;
  parseArgs(state, nB, nnz);
  using BSM = BlockSparseMatrix<Scalar, ColMajor, B, B>;
  BSM bsm;
  SparseMatrix<Scalar> smTmp;
  buildUpperTriPair<Scalar, B>(nB, nnz, 1, bsm, smTmp);
  Matrix<Scalar, Dynamic, 1> x = Matrix<Scalar, Dynamic, 1>::Random(bsm.cols());
  Matrix<Scalar, Dynamic, 1> y(bsm.rows());
  for (auto _ : state) {
    y.noalias() = bsm.template triangularView<Upper, false>() * x;
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["n"] = bsm.rows();
}

// BSM triangular view, DiagIsTriangular=true: diagonal blocks are actually triangular, uses full GEMV.
template <typename Scalar, int B>
static void BM_BSM_TriMV_DiagT(benchmark::State& state) {
  int nB, nnz;
  parseArgs(state, nB, nnz);
  using BSM = BlockSparseMatrix<Scalar, ColMajor, B, B>;
  BSM bsm;
  SparseMatrix<Scalar> smTmp;
  buildActuallyTriPair<Scalar, B>(nB, nnz, 1, bsm, smTmp);
  Matrix<Scalar, Dynamic, 1> x = Matrix<Scalar, Dynamic, 1>::Random(bsm.cols());
  Matrix<Scalar, Dynamic, 1> y(bsm.rows());
  for (auto _ : state) {
    y.noalias() = bsm.template triangularView<Upper, true>() * x;
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["n"] = bsm.rows();
}

// Accumulating triangular MV: y += A * x.
template <typename Scalar, int B>
static void BM_Sm_TriMV_Acc(benchmark::State& state) {
  int nB, nnz;
  parseArgs(state, nB, nnz);
  SparseMatrix<Scalar> sm;
  BlockSparseMatrix<Scalar, ColMajor, B, B> tmp;
  buildUpperTriPair<Scalar, B>(nB, nnz, 1, tmp, sm);
  Matrix<Scalar, Dynamic, 1> x = Matrix<Scalar, Dynamic, 1>::Random(sm.cols());
  Matrix<Scalar, Dynamic, 1> y = Matrix<Scalar, Dynamic, 1>::Random(sm.rows());
  for (auto _ : state) {
    y.noalias() += sm.template triangularView<Upper>() * x;
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["n"] = sm.rows();
}

template <typename Scalar, int B>
static void BM_BSM_TriMV_Acc(benchmark::State& state) {
  int nB, nnz;
  parseArgs(state, nB, nnz);
  using BSM = BlockSparseMatrix<Scalar, ColMajor, B, B>;
  BSM bsm;
  SparseMatrix<Scalar> smTmp;
  buildUpperTriPair<Scalar, B>(nB, nnz, 1, bsm, smTmp);
  Matrix<Scalar, Dynamic, 1> x = Matrix<Scalar, Dynamic, 1>::Random(bsm.cols());
  Matrix<Scalar, Dynamic, 1> y = Matrix<Scalar, Dynamic, 1>::Random(bsm.rows());
  for (auto _ : state) {
    y.noalias() += bsm.template triangularView<Upper, false>() * x;
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["n"] = bsm.rows();
}

template <typename Scalar, int B>
static void BM_BSM_TriMV_DiagT_Acc(benchmark::State& state) {
  int nB, nnz;
  parseArgs(state, nB, nnz);
  using BSM = BlockSparseMatrix<Scalar, ColMajor, B, B>;
  BSM bsm;
  SparseMatrix<Scalar> smTmp;
  buildActuallyTriPair<Scalar, B>(nB, nnz, 1, bsm, smTmp);
  Matrix<Scalar, Dynamic, 1> x = Matrix<Scalar, Dynamic, 1>::Random(bsm.cols());
  Matrix<Scalar, Dynamic, 1> y = Matrix<Scalar, Dynamic, 1>::Random(bsm.rows());
  for (auto _ : state) {
    y.noalias() += bsm.template triangularView<Upper, true>() * x;
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["n"] = bsm.rows();
}

// ---------------------------------------------------------------------------
// Sparse×Dense: Selfadjoint MV
// ---------------------------------------------------------------------------
template <typename Scalar, int B>
static void BM_Sm_SymmMV(benchmark::State& state) {
  int nB, nnz;
  parseArgs(state, nB, nnz);
  SparseMatrix<Scalar> sm;
  BlockSparseMatrix<Scalar, ColMajor, B, B> tmp;
  buildUpperTriPair<Scalar, B>(nB, nnz, 1, tmp, sm);
  Matrix<Scalar, Dynamic, 1> x = Matrix<Scalar, Dynamic, 1>::Random(sm.cols());
  Matrix<Scalar, Dynamic, 1> y(sm.rows());
  for (auto _ : state) {
    y.noalias() = sm.template selfadjointView<Upper>() * x;
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["n"] = sm.rows();
}

// BSM selfadjoint view, general (non-Hermitian) diagonal blocks, DiagIsSA=false.
template <typename Scalar, int B>
static void BM_BSM_SymmMV(benchmark::State& state) {
  int nB, nnz;
  parseArgs(state, nB, nnz);
  using BSM = BlockSparseMatrix<Scalar, ColMajor, B, B>;
  BSM bsm;
  SparseMatrix<Scalar> smTmp;
  buildUpperTriPair<Scalar, B>(nB, nnz, 1, bsm, smTmp);
  Matrix<Scalar, Dynamic, 1> x = Matrix<Scalar, Dynamic, 1>::Random(bsm.cols());
  Matrix<Scalar, Dynamic, 1> y(bsm.rows());
  for (auto _ : state) {
    y.noalias() = bsm.template selfadjointView<Upper>() * x;
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["n"] = bsm.rows();
}

// BSM selfadjoint view, Hermitian diagonal blocks, DiagIsSA=false: fills diagonal via selfadjointView<>.
template <typename Scalar, int B>
static void BM_BSM_SymmMV_DiagNSA(benchmark::State& state) {
  int nB, nnz;
  parseArgs(state, nB, nnz);
  using BSM = BlockSparseMatrix<Scalar, ColMajor, B, B>;
  BSM bsm;
  SparseMatrix<Scalar> smTmp;
  buildHermDiagUpperTriPair<Scalar, B>(nB, nnz, 1, bsm, smTmp);
  Matrix<Scalar, Dynamic, 1> x = Matrix<Scalar, Dynamic, 1>::Random(bsm.cols());
  Matrix<Scalar, Dynamic, 1> y(bsm.rows());
  for (auto _ : state) {
    y.noalias() = bsm.template selfadjointView<Upper, false>() * x;
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["n"] = bsm.rows();
}

// BSM selfadjoint view, Hermitian diagonal blocks, DiagIsSA=true: full GEMV for diagonal blocks.
template <typename Scalar, int B>
static void BM_BSM_SymmMV_DiagSA(benchmark::State& state) {
  int nB, nnz;
  parseArgs(state, nB, nnz);
  using BSM = BlockSparseMatrix<Scalar, ColMajor, B, B>;
  BSM bsm;
  SparseMatrix<Scalar> smTmp;
  buildHermDiagUpperTriPair<Scalar, B>(nB, nnz, 1, bsm, smTmp);
  Matrix<Scalar, Dynamic, 1> x = Matrix<Scalar, Dynamic, 1>::Random(bsm.cols());
  Matrix<Scalar, Dynamic, 1> y(bsm.rows());
  for (auto _ : state) {
    y.noalias() = bsm.template selfadjointView<Upper, true>() * x;
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["n"] = bsm.rows();
}

// Accumulating selfadjoint MV: y += A * x.
template <typename Scalar, int B>
static void BM_Sm_SymmMV_Acc(benchmark::State& state) {
  int nB, nnz;
  parseArgs(state, nB, nnz);
  SparseMatrix<Scalar> sm;
  BlockSparseMatrix<Scalar, ColMajor, B, B> tmp;
  buildUpperTriPair<Scalar, B>(nB, nnz, 1, tmp, sm);
  Matrix<Scalar, Dynamic, 1> x = Matrix<Scalar, Dynamic, 1>::Random(sm.cols());
  Matrix<Scalar, Dynamic, 1> y = Matrix<Scalar, Dynamic, 1>::Random(sm.rows());
  for (auto _ : state) {
    y.noalias() += sm.template selfadjointView<Upper>() * x;
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["n"] = sm.rows();
}

template <typename Scalar, int B>
static void BM_BSM_SymmMV_Acc(benchmark::State& state) {
  int nB, nnz;
  parseArgs(state, nB, nnz);
  using BSM = BlockSparseMatrix<Scalar, ColMajor, B, B>;
  BSM bsm;
  SparseMatrix<Scalar> smTmp;
  buildUpperTriPair<Scalar, B>(nB, nnz, 1, bsm, smTmp);
  Matrix<Scalar, Dynamic, 1> x = Matrix<Scalar, Dynamic, 1>::Random(bsm.cols());
  Matrix<Scalar, Dynamic, 1> y = Matrix<Scalar, Dynamic, 1>::Random(bsm.rows());
  for (auto _ : state) {
    y.noalias() += bsm.template selfadjointView<Upper>() * x;
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["n"] = bsm.rows();
}

template <typename Scalar, int B>
static void BM_BSM_SymmMV_DiagNSA_Acc(benchmark::State& state) {
  int nB, nnz;
  parseArgs(state, nB, nnz);
  using BSM = BlockSparseMatrix<Scalar, ColMajor, B, B>;
  BSM bsm;
  SparseMatrix<Scalar> smTmp;
  buildHermDiagUpperTriPair<Scalar, B>(nB, nnz, 1, bsm, smTmp);
  Matrix<Scalar, Dynamic, 1> x = Matrix<Scalar, Dynamic, 1>::Random(bsm.cols());
  Matrix<Scalar, Dynamic, 1> y = Matrix<Scalar, Dynamic, 1>::Random(bsm.rows());
  for (auto _ : state) {
    y.noalias() += bsm.template selfadjointView<Upper, false>() * x;
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["n"] = bsm.rows();
}

template <typename Scalar, int B>
static void BM_BSM_SymmMV_DiagSA_Acc(benchmark::State& state) {
  int nB, nnz;
  parseArgs(state, nB, nnz);
  using BSM = BlockSparseMatrix<Scalar, ColMajor, B, B>;
  BSM bsm;
  SparseMatrix<Scalar> smTmp;
  buildHermDiagUpperTriPair<Scalar, B>(nB, nnz, 1, bsm, smTmp);
  Matrix<Scalar, Dynamic, 1> x = Matrix<Scalar, Dynamic, 1>::Random(bsm.cols());
  Matrix<Scalar, Dynamic, 1> y = Matrix<Scalar, Dynamic, 1>::Random(bsm.rows());
  for (auto _ : state) {
    y.noalias() += bsm.template selfadjointView<Upper, true>() * x;
    benchmark::DoNotOptimize(y.data());
  }
  state.counters["n"] = bsm.rows();
}

// ---------------------------------------------------------------------------
// Sparse×Dense: Triangular solve
// ---------------------------------------------------------------------------
template <typename Scalar, int B>
static void BM_Sm_TriSolve(benchmark::State& state) {
  int nB, nnz;
  parseArgs(state, nB, nnz);
  SparseMatrix<Scalar> sm;
  BlockSparseMatrix<Scalar, ColMajor, B, B> tmp;
  buildUpperTriPair<Scalar, B>(nB, nnz, 1, tmp, sm, true);
  Matrix<Scalar, Dynamic, 1> rhs = Matrix<Scalar, Dynamic, 1>::Random(sm.cols());
  Matrix<Scalar, Dynamic, 1> x(sm.cols());
  for (auto _ : state) {
    x = rhs;
    sm.template triangularView<Upper>().solveInPlace(x);
    benchmark::DoNotOptimize(x.data());
  }
  state.counters["n"] = sm.rows();
}

template <typename Scalar, int B>
static void BM_BSM_TriSolve(benchmark::State& state) {
  int nB, nnz;
  parseArgs(state, nB, nnz);
  using BSM = BlockSparseMatrix<Scalar, ColMajor, B, B>;
  BSM bsm;
  SparseMatrix<Scalar> smTmp;
  buildUpperTriPair<Scalar, B>(nB, nnz, 1, bsm, smTmp, true);
  Matrix<Scalar, Dynamic, 1> rhs = Matrix<Scalar, Dynamic, 1>::Random(bsm.cols());
  Matrix<Scalar, Dynamic, 1> x(bsm.cols());
  for (auto _ : state) {
    x = rhs;
    bsm.template triangularView<Upper>().solveInPlace(x);
    benchmark::DoNotOptimize(x.data());
  }
  state.counters["n"] = bsm.rows();
}

// ---------------------------------------------------------------------------
// Sparse×Sparse: Matrix multiply
// ---------------------------------------------------------------------------
template <typename Scalar, int B>
static void BM_Sm_Sm_Mul(benchmark::State& state) {
  int nB, nnz;
  parseArgs(state, nB, nnz);
  SparseMatrix<Scalar> smA, smB, smC;
  BlockSparseMatrix<Scalar, ColMajor, B, B> tmp;
  buildPair<Scalar, B>(nB, nnz, 1, tmp, smA);
  buildPair<Scalar, B>(nB, nnz, 2, tmp, smB);
  for (auto _ : state) {
    smC = smA * smB;
    benchmark::DoNotOptimize(smC.valuePtr());
  }
  state.counters["n"] = smA.rows();
}

template <typename Scalar, int B>
static void BM_BSM_BSM_Mul(benchmark::State& state) {
  int nB, nnz;
  parseArgs(state, nB, nnz);
  using BSM = BlockSparseMatrix<Scalar, ColMajor, B, B>;
  BSM bsmA, bsmB, bsmC;
  SparseMatrix<Scalar> smTmp;
  buildPair<Scalar, B>(nB, nnz, 1, bsmA, smTmp);
  buildPair<Scalar, B>(nB, nnz, 2, bsmB, smTmp);
  for (auto _ : state) {
    bsmC = bsmA * bsmB;
    benchmark::DoNotOptimize(bsmC.valuePtr());
  }
  state.counters["n"] = bsmA.rows();
}

// ---------------------------------------------------------------------------
// Registration
// ---------------------------------------------------------------------------

#define NS benchmark::kNanosecond
#define US benchmark::kMicrosecond
// Args: {nB, sparsity%} — nnzPerCol = max(1, pct*nB/100)
#define REG(fn, S, B) BENCHMARK(fn<S, B>)->Args({200, 1})->Args({200, 5})->Args({200, 10})

#define BENCH_TYPE(S, B)                          \
  REG(BM_Sm_Sm_Add, S, B)->Unit(US);              \
  REG(BM_BSM_BSM_Add, S, B)->Unit(US);            \
  REG(BM_Sm_SpMV, S, B)->Unit(NS);                \
  REG(BM_BSM_SpMV, S, B)->Unit(NS);               \
  REG(BM_Sm_SpMV_Acc, S, B)->Unit(NS);            \
  REG(BM_BSM_SpMV_Acc, S, B)->Unit(NS);           \
  REG(BM_Sm_TriMV, S, B)->Unit(NS);               \
  REG(BM_BSM_TriMV, S, B)->Unit(NS);              \
  REG(BM_BSM_TriMV_DiagT, S, B)->Unit(NS);        \
  REG(BM_Sm_TriMV_Acc, S, B)->Unit(NS);           \
  REG(BM_BSM_TriMV_Acc, S, B)->Unit(NS);          \
  REG(BM_BSM_TriMV_DiagT_Acc, S, B)->Unit(NS);    \
  REG(BM_Sm_SymmMV, S, B)->Unit(NS);              \
  REG(BM_BSM_SymmMV, S, B)->Unit(NS);             \
  REG(BM_BSM_SymmMV_DiagNSA, S, B)->Unit(NS);     \
  REG(BM_BSM_SymmMV_DiagSA, S, B)->Unit(NS);      \
  REG(BM_Sm_SymmMV_Acc, S, B)->Unit(NS);          \
  REG(BM_BSM_SymmMV_Acc, S, B)->Unit(NS);         \
  REG(BM_BSM_SymmMV_DiagNSA_Acc, S, B)->Unit(NS); \
  REG(BM_BSM_SymmMV_DiagSA_Acc, S, B)->Unit(NS);  \
  REG(BM_Sm_TriSolve, S, B)->Unit(NS);            \
  REG(BM_BSM_TriSolve, S, B)->Unit(NS);           \
  REG(BM_Sm_Sm_Mul, S, B)->Unit(US);              \
  REG(BM_BSM_BSM_Mul, S, B)->Unit(US);

BENCH_TYPE(float, 2)
BENCH_TYPE(cf, 2)
BENCH_TYPE(double, 2)
BENCH_TYPE(cd, 2)
BENCH_TYPE(float, 3)
BENCH_TYPE(cf, 3)
BENCH_TYPE(double, 3)
BENCH_TYPE(cd, 3)
BENCH_TYPE(float, 4)
BENCH_TYPE(cf, 4)
BENCH_TYPE(double, 4)
BENCH_TYPE(cd, 4)
