// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

// SME GEMM kernel tests.
// Requires a compiler targeting SME2, e.g. -march=armv9.2-a+sme2, which selects
// the backend on its own; -DEIGEN_ARM64_USE_SME then makes a toolchain that
// cannot provide it fail the build rather than fall back to NEON.
// double and complex<double> additionally need FEAT_SME_F64F64 (+sme-f64f64, or
// a -mcpu that implies it); without it EIGEN_VECTORIZE_SME_F64F64 is undefined
// and they keep the generic kernel, so their subtests pack their cases through
// that path instead.

#include "product.h"

// Without the right -march flags, __ARM_FEATURE_SME is undefined and
// EIGEN_VECTORIZE_SME never fires - the test would silently compile
// against the NEON GEBP kernel and pass, making this a useless no-op.
// Fail the build instead.
#if !defined(EIGEN_VECTORIZE_SME)
#error \
    "product_sme requires the SME backend.  Build with -march=armv9.2-a+sme2 " \
    "(see -DEIGEN_TEST_SME=ON in test/CMakeLists.txt for the typical CMake " \
    "invocation)."
#endif

// The backend is chosen from the compiler's target features; EIGEN_ARM64_USE_SME,
// which this test's CMake option passes, only turns a toolchain that cannot
// provide it into a build error. Assert that here, so a regression making
// selection depend on the opt-in again is caught rather than masked by it.
#if !defined(EIGEN_ARM64_SME_SELECTED)
#error "the SME backend was forced by EIGEN_ARM64_USE_SME rather than selected from the compiler's target features."
#endif

template <typename Scalar>
using SmeColMajorMat = Matrix<Scalar, Dynamic, Dynamic, ColMajor>;
template <typename Scalar>
using SmeRowMajorMat = Matrix<Scalar, Dynamic, Dynamic, RowMajor>;
template <typename Scalar>
using SmeVector = Matrix<Scalar, Dynamic, 1>;
template <typename Scalar>
using SmeColMajorStridedMat = Map<SmeColMajorMat<Scalar>, 0, Stride<Dynamic, Dynamic>>;
template <typename Scalar>
using SmeRowMajorStridedMat = Map<SmeRowMajorMat<Scalar>, 0, Stride<Dynamic, Dynamic>>;

// The logical micro-kernel block for Scalar (kSmeMr/kSmeNr for float and so
// on).  Sizes below are expressed in terms of it so each scalar sweeps its own
// block boundaries; sme_tile() is the ZA tile side at the SVL=512 design point,
// which is where the intra-block splits fall.  Complex scalars pair two tiles
// per accumulator, so their block is not square.
template <typename Scalar>
static constexpr int sme_mr() {
  return internal::sme_block<Scalar>::mr;
}
template <typename Scalar>
static constexpr int sme_nr() {
  return internal::sme_block<Scalar>::nr;
}
template <typename Scalar>
static constexpr int sme_tile() {
  return internal::sme_block<Scalar>::mr / internal::sme_block<Scalar>::kGridRows;
}

// svwhilelt_b* is overloaded only on the four fixed-width types, so forwarding
// the caller's own type would leave Index ambiguous wherever it is `long` while
// int64_t is `long long`, and deducing one type from both arguments would reject
// the literal-plus-Index bounds the packers pass.  Name every spelling Index
// takes, mix the two argument types, and check the predicate so the traits'
// widening cannot change which lanes are active.
template <typename Scalar, typename Begin, typename End = Begin>
__arm_locally_streaming static bool sme_whilelt_covers_first_lane_only() {
  using Traits = internal::sme_traits<Scalar>;
  const svbool_t pg = Traits::whilelt(Begin(0), End(1));
  return svptest_first(Traits::ptrue(), pg) && svcntp_b8(Traits::ptrue(), pg) == 1;
}

template <typename Scalar>
static void test_whilelt_operand_types() {
  VERIFY((sme_whilelt_covers_first_lane_only<Scalar, int>()));
  VERIFY((sme_whilelt_covers_first_lane_only<Scalar, long>()));
  VERIFY((sme_whilelt_covers_first_lane_only<Scalar, long long>()));
  VERIFY((sme_whilelt_covers_first_lane_only<Scalar, Index>()));
  VERIFY((sme_whilelt_covers_first_lane_only<Scalar, int, Index>()));
}

// Write one element into a packed panel of width w, in the layout the SME
// kernel reads: a real scalar lands at dst[k*w + r], a complex one splits into
// the depth step's real and imaginary halves.
template <typename Scalar>
static void set_packed(Scalar* panel, Index w, Index k, Index r, const Scalar& v) {
  panel[k * w + r] = v;
}
template <typename RealScalar>
static void set_packed(std::complex<RealScalar>* panel, Index w, Index k, Index r, const std::complex<RealScalar>& v) {
  RealScalar* p = reinterpret_cast<RealScalar*>(panel + k * w);
  p[r] = numext::real(v);
  p[w + r] = numext::imag(v);
}

template <typename InputMat, typename ResultMat, typename ResultMap>
static void verify_strided_result(int n, ResultMat& storage, const Stride<Dynamic, Dynamic>& stride) {
  InputMat A = InputMat::Random(n, n);
  InputMat B = InputMat::Random(n, n);
  ResultMap C(storage.data(), n, n, stride);
  C = ResultMat::Random(n, n);
  ResultMat c_before = C.eval();

  C.noalias() += A * B;

  ResultMat ref = c_before + (A.lazyProduct(B)).eval();
  ResultMat got = C;
  VERIFY_IS_APPROX(got, ref);
}

template <typename Scalar, typename InputMat>
static void test_general_strided_result(int n) {
  // General-stride C path: InputMat selects the source packers, while both C
  // strides are non-unit so sme_store_za_tile uses scalar scatter.
  SmeColMajorMat<Scalar> storage = SmeColMajorMat<Scalar>::Zero(2 * n, n);
  verify_strided_result<InputMat, SmeColMajorMat<Scalar>, SmeColMajorStridedMat<Scalar>>(
      n, storage, Stride<Dynamic, Dynamic>(/*outer=*/2 * n, /*inner=*/2));

  // Padding rows skipped by the strided Map should not be touched.
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      VERIFY(storage(2 * i + 1, j) == Scalar(0));
    }
  }
}

template <typename Scalar>
static void test_rowmajor_strided_result(int n) {
  // RowMajor C path: inner stride is one, with padded columns after the Map.
  SmeRowMajorMat<Scalar> storage = SmeRowMajorMat<Scalar>::Zero(n, 2 * n);
  verify_strided_result<SmeRowMajorMat<Scalar>, SmeRowMajorMat<Scalar>, SmeRowMajorStridedMat<Scalar>>(
      n, storage, Stride<Dynamic, Dynamic>(/*outer=*/2 * n, /*inner=*/1));

  // Padding columns skipped by the strided Map should not be touched.
  for (int i = 0; i < n; ++i) {
    for (int j = n; j < 2 * n; ++j) {
      VERIFY(storage(i, j) == Scalar(0));
    }
  }
}

// A non-trivial alpha: complex where the scalar is, so the store's rotation of
// the accumulator is exercised rather than a plain rescale.
template <typename Scalar>
struct nontrivial_alpha_impl {
  static Scalar run() { return Scalar(1.375); }
};
template <typename RealScalar>
struct nontrivial_alpha_impl<std::complex<RealScalar>> {
  static std::complex<RealScalar> run() { return std::complex<RealScalar>(RealScalar(1.375), RealScalar(-0.625)); }
};

// Conjugated operands and a non-real alpha.
//
// blas_traits folds .adjoint()/.conjugate() and a scalar factor into the
// gebp_kernel's ConjugateLhs/ConjugateRhs and alpha rather than materializing a
// temporary, so these expressions are the only thing that reaches the complex
// kernel's FMOPA/FMOPS sign choices and its alpha-scaled store. The references
// materialize the conjugation first and multiply coefficient-wise, so they do
// not share a code path with what they check.
template <typename Scalar, typename LhsMat, typename RhsMat, typename ResMat>
static void verify_conjugated_products(int n) {
  const LhsMat A = LhsMat::Random(n, n);
  const RhsMat B = RhsMat::Random(n, n);
  const SmeColMajorMat<Scalar> Ap = A, Bp = B;
  const SmeColMajorMat<Scalar> Aa = Ap.adjoint().eval(), Ac = Ap.conjugate().eval();
  const SmeColMajorMat<Scalar> Ba = Bp.adjoint().eval(), Bc = Bp.conjugate().eval();
  const Scalar alpha = nontrivial_alpha_impl<Scalar>::run();

  ResMat C(n, n);
  C.setZero();
  C.noalias() += A.adjoint() * B;
  VERIFY_IS_APPROX(SmeColMajorMat<Scalar>(C), SmeColMajorMat<Scalar>(Aa.lazyProduct(Bp)));
  C.setZero();
  C.noalias() += A * B.adjoint();
  VERIFY_IS_APPROX(SmeColMajorMat<Scalar>(C), SmeColMajorMat<Scalar>(Ap.lazyProduct(Ba)));
  C.setZero();
  C.noalias() += A.adjoint() * B.adjoint();
  VERIFY_IS_APPROX(SmeColMajorMat<Scalar>(C), SmeColMajorMat<Scalar>(Aa.lazyProduct(Ba)));
  C.setZero();
  C.noalias() += A.conjugate() * B;
  VERIFY_IS_APPROX(SmeColMajorMat<Scalar>(C), SmeColMajorMat<Scalar>(Ac.lazyProduct(Bp)));
  C.setZero();
  C.noalias() += A * B.conjugate();
  VERIFY_IS_APPROX(SmeColMajorMat<Scalar>(C), SmeColMajorMat<Scalar>(Ap.lazyProduct(Bc)));
  C.setZero();
  C.noalias() += A.conjugate() * B.conjugate();
  VERIFY_IS_APPROX(SmeColMajorMat<Scalar>(C), SmeColMajorMat<Scalar>(Ac.lazyProduct(Bc)));
  C.setZero();
  C.noalias() += alpha * (A.adjoint() * B);
  VERIFY_IS_APPROX(SmeColMajorMat<Scalar>(C), SmeColMajorMat<Scalar>(alpha * Aa.lazyProduct(Bp)));
}

// Exercise the kc split path just above the SME blocking heuristic's depth cap
// (sme_max_kc in GeneralBlockPanelKernel.h, scaled by the scalar width).
template <typename Scalar>
static void test_deep_k_split() {
  constexpr int rows = 64;
  const int depth = int(2 * (Index(EIGEN_SME_MAX_KC) * Index(sizeof(float)) / Index(sizeof(Scalar)))) + 2;
  constexpr int cols = 64;
  SmeColMajorMat<Scalar> A = SmeColMajorMat<Scalar>::Random(rows, depth);
  SmeColMajorMat<Scalar> B = SmeColMajorMat<Scalar>::Random(depth, cols);
  SmeColMajorMat<Scalar> C = SmeColMajorMat<Scalar>::Random(rows, cols);
  SmeColMajorMat<Scalar> c_before = C;

  C.noalias() += A * B;

  VERIFY_IS_APPROX(C, c_before + (A.lazyProduct(B)).eval());
}

// ---------------------------------------------------------------------------
// Raw packed-buffer tests.
//
// The product tests above validate the packers only transitively through a full
// product, where the gebp_kernel can mask a mispack, or where a bug only shows
// at a specific SVL/region boundary.  The tests below call the SME packers
// directly and compare the packed buffer exactly against a scalar reference
// (equality is exact).  This pins every region -- in particular a dropped
// row-group in the two-pass trailing transpose, and a SYMM packer that copies
// the unused triangle instead of mirroring -- at whatever SVL the run uses.
// ---------------------------------------------------------------------------

// A distinctive marker for buffer cells the packer must leave untouched, and
// for the unused triangle of a lower-triangular operand. Random values live in
// [-1, 1], so it never collides with a real packed value. The complex marker's
// imaginary part differs from its real one, so a packer that fills only the
// real half of a split depth step is caught.
template <typename Scalar>
struct pack_sentinel_impl {
  static Scalar run() { return Scalar(98765); }
};
template <typename RealScalar>
struct pack_sentinel_impl<std::complex<RealScalar>> {
  static std::complex<RealScalar> run() { return std::complex<RealScalar>(RealScalar(98765), RealScalar(-54321)); }
};
template <typename Scalar>
static Scalar pack_sentinel() {
  return pack_sentinel_impl<Scalar>::run();
}

// Lower-triangular n x n operand plus the dense selfadjoint reference the packer
// must emit. The unused triangle is filled with the sentinel so a packer that
// copies the dense matrix and never mirrors fails VERIFY_IS_EQUAL.
// product_selfadjoint_matrix stores the valid triangle where row >= col
// (after the Upper/RowMajor xor), so the packer must read stored(row,col)
// below the diagonal and conj(stored(col,row)) above it. Averaging with the
// adjoint cancels the diagonal's imaginary part exactly, which is what a
// selfadjoint view defines it to be.
template <typename Scalar, int StorageOrder>
static void make_lower_stored_selfadjoint(Index n, Matrix<Scalar, Dynamic, Dynamic, StorageOrder>& stored,
                                          Matrix<Scalar, Dynamic, Dynamic, StorageOrder>& full) {
  full = Matrix<Scalar, Dynamic, Dynamic, StorageOrder>::Random(n, n);
  full = ((full + full.adjoint()) * Scalar(0.5)).eval();
  stored = Matrix<Scalar, Dynamic, Dynamic, StorageOrder>::Constant(n, n, pack_sentinel<Scalar>());
  for (Index i = 0; i < n; ++i)
    for (Index j = 0; j <= i; ++j) stored(i, j) = full(i, j);
}

// LHS SYMM packer: a square selfadjoint diagonal block of size n, packed into
// uniform mr-wide depth-major panels.  Reference: full(i+r, k).
template <typename Scalar, int StorageOrder>
static void verify_symm_pack_lhs(Index n) {
  const Index MR = sme_mr<Scalar>();
  Matrix<Scalar, Dynamic, Dynamic, StorageOrder> stored, full;
  make_lower_stored_selfadjoint<Scalar, StorageOrder>(n, stored, full);

  SmeVector<Scalar> packed = SmeVector<Scalar>::Constant(n * n, pack_sentinel<Scalar>());
  SmeVector<Scalar> ref = SmeVector<Scalar>::Constant(n * n, pack_sentinel<Scalar>());
  for (Index i = 0; i < n; i += MR) {
    const Index w = numext::mini(MR, n - i);
    for (Index k = 0; k < n; ++k)
      for (Index r = 0; r < w; ++r) set_packed(ref.data() + i * n, w, k, r, Scalar(full(i + r, k)));
  }

  internal::symm_pack_lhs<Scalar, Index, sme_mr<Scalar>(), 1, StorageOrder> pack;
  pack(packed.data(), stored.data(), stored.outerStride(), /*cols(depth)=*/n, /*rows=*/n);
  VERIFY_IS_EQUAL(packed, ref);
}

// RHS SYMM packer: a depth block [k2, k2 + depth) x cols columns of an N x N
// selfadjoint matrix, packed into nr-wide depth-major panels.  Reference:
// full(k2 + k, j + c).  A k2 > 0 offset makes the transposed region non-empty,
// so partial-width panels reach the two-pass transpose.
template <typename Scalar, int StorageOrder>
static void verify_symm_pack_rhs(Index N, Index depth, Index cols, Index k2) {
  eigen_assert(k2 + depth <= N && cols <= N);
  const Index NR = sme_nr<Scalar>();
  Matrix<Scalar, Dynamic, Dynamic, StorageOrder> stored, full;
  make_lower_stored_selfadjoint<Scalar, StorageOrder>(N, stored, full);

  SmeVector<Scalar> packed = SmeVector<Scalar>::Constant(cols * depth, pack_sentinel<Scalar>());
  SmeVector<Scalar> ref = SmeVector<Scalar>::Constant(cols * depth, pack_sentinel<Scalar>());
  for (Index j = 0; j < cols; j += NR) {
    const Index w = numext::mini(NR, cols - j);
    for (Index k = 0; k < depth; ++k)
      for (Index c = 0; c < w; ++c) set_packed(ref.data() + j * depth, w, k, c, Scalar(full(k2 + k, j + c)));
  }

  internal::symm_pack_rhs<Scalar, Index, sme_nr<Scalar>(), StorageOrder> pack;
  pack(packed.data(), stored.data(), stored.outerStride(), /*rows(depth)=*/depth, /*cols=*/cols, k2);
  VERIFY_IS_EQUAL(packed, ref);
}

template <typename Scalar>
static void test_symm_pack() {
  // The last panel width sweeps a range of partial widths; at each SVL the
  // two-pass trailing transpose (the if->loop fix) fires when a partial width
  // leaves a trailing row-group remainder in (svl, 2*svl).  The spread below
  // hits that for svl in {2, 4, 8, 16, 32, 64} -- fp32 SVL 128..2048 and the
  // fp64 lane counts, which are half of those.
  const int sizes[] = {1, 5, 7, 17, 31, 32, 33, 37, 39, 45, 48, 49, 55, 57, 63, 64, 65, 79, 96, 97};
  for (int n : sizes) {
    verify_symm_pack_lhs<Scalar, ColMajor>(n);
    verify_symm_pack_lhs<Scalar, RowMajor>(n);
    // RHS, single depth block anchored at the diagonal (k2 == 0).
    verify_symm_pack_rhs<Scalar, ColMajor>(n, n, n, 0);
    verify_symm_pack_rhs<Scalar, RowMajor>(n, n, n, 0);
  }

  // RHS depth blocks offset from the diagonal (k2 > 0): the transposed region is
  // non-empty, so the RowMajor operand drives partial-width panels through the
  // two-pass transpose and the ColMajor operand through the partial copy.
  struct RhsCase {
    int N, depth, cols, k2;
  };
  const RhsCase rhs_cases[] = {
      {100, 32, 39, 16}, {100, 24, 39, 32}, {100, 40, 64, 8}, {100, 39, 39, 33}, {128, 57, 57, 40}, {128, 33, 45, 60},
  };
  for (const RhsCase& c : rhs_cases) {
    verify_symm_pack_rhs<Scalar, ColMajor>(c.N, c.depth, c.cols, c.k2);
    verify_symm_pack_rhs<Scalar, RowMajor>(c.N, c.depth, c.cols, c.k2);
  }
}

// ---------------------------------------------------------------------------
// Mapper-based packing fallback (sme_pack_{lhs,rhs}_fallback).
//
// Taken by by-value tensor sub-mappers and inner-strided blas mappers, which
// the raw pointer + stride packers cannot walk.  The product suite reaches these
// only through tensor contractions (nightly, SVL=512) and TriangularSolver
// (random sizes), so here we drive the fallback directly with both mapper
// families and compare the packed buffer against a scalar reference.
// ---------------------------------------------------------------------------

// Minimal stand-ins for by-value sub-mappers. ColMajor packets advance the
// first index, while RowMajor packets follow the normal storage-inner second
// index. operator() returns by value so both take the no-direct-access dispatch.
template <typename Scalar>
struct ByValueColMajorLhsMapper {
  const Scalar* data;
  Index stride;  // element(i, k) = data[i + k * stride], contiguous in i
  Scalar operator()(Index i, Index k) const { return data[i + k * stride]; }
  template <typename Packet>
  EIGEN_ALWAYS_INLINE Packet loadPacket(Index i, Index k) const {
    return internal::ploadu<Packet>(data + i + k * stride);
  }
};

template <typename Scalar>
struct ByValueRowMajorLhsMapper {
  const Scalar* data;
  Index stride;  // element(i, k) = data[i * stride + k], contiguous in k
  Scalar operator()(Index i, Index k) const { return data[i * stride + k]; }
  template <typename Packet>
  EIGEN_ALWAYS_INLINE Packet loadPacket(Index i, Index k) const {
    return internal::ploadu<Packet>(data + i * stride + k);
  }
};

template <typename Scalar>
struct ByValueColMajorRhsMapper {
  struct LinearMapper {
    const Scalar* p;  // &element(0, col); contiguous in depth
    Scalar operator()(Index k) const { return p[k]; }
    template <typename Packet>
    EIGEN_ALWAYS_INLINE Packet loadPacket(Index k) const {
      return internal::ploadu<Packet>(p + k);
    }
  };
  const Scalar* data;
  Index stride;  // element(k, col) = data[k + col * stride], contiguous in k
  Scalar operator()(Index k, Index col) const { return data[k + col * stride]; }
  LinearMapper getLinearMapper(Index k, Index col) const { return LinearMapper{data + k + col * stride}; }
};

template <typename Scalar>
struct ByValueRowMajorRhsMapper {
  struct LinearMapper {
    const Scalar* p;  // &element(row, col); packet offsets advance columns
    Scalar operator()(Index offset) const { return p[offset]; }
    template <typename Packet>
    EIGEN_ALWAYS_INLINE Packet loadPacket(Index offset) const {
      return internal::ploadu<Packet>(p + offset);
    }
  };
  const Scalar* data;
  Index stride;  // element(k, col) = data[k * stride + col], contiguous in col
  Scalar operator()(Index k, Index col) const { return data[k * stride + col]; }
  LinearMapper getLinearMapper(Index k, Index col) const { return LinearMapper{data + k * stride + col}; }
};

// Length of the packed LHS buffer for the given panel-mode layout, matching the
// dst_panel formula in sme_pack_lhs_fallback.
template <bool PanelMode>
static Index packed_len(Index outer, Index depth, Index unit, Index dst_stride, Index dst_offset) {
  // `outer` is rows (LHS) or cols (RHS); `unit` is the panel width mr or nr.
  if (!PanelMode) return outer * depth;
  Index end = 0;
  for (Index i = 0; i < outer; i += unit) {
    const Index w = numext::mini(unit, outer - i);
    end = numext::maxi(end, i * dst_stride + dst_offset * w + depth * w);
  }
  return end;
}

template <typename Scalar, bool PanelMode, bool Conjugate, typename MatrixType>
static void fill_lhs_ref(SmeVector<Scalar>& ref, const MatrixType& V, Index rows, Index depth, Index dst_stride,
                         Index dst_offset) {
  const Index MR = sme_mr<Scalar>();
  ref.setConstant(pack_sentinel<Scalar>());
  for (Index i = 0; i < rows; i += MR) {
    const Index w = numext::mini(MR, rows - i);
    const Index base = PanelMode ? i * dst_stride + dst_offset * w : i * depth;
    for (Index k = 0; k < depth; ++k)
      for (Index r = 0; r < w; ++r) {
        const Scalar v = V(i + r, k);
        set_packed(ref.data() + base, w, k, r, Conjugate ? numext::conj(v) : v);
      }
  }
}

template <typename Scalar, bool PanelMode, bool Conjugate, typename MatrixType>
static void fill_rhs_ref(SmeVector<Scalar>& ref, const MatrixType& V, Index cols, Index depth, Index dst_stride,
                         Index dst_offset) {
  const Index NR = sme_nr<Scalar>();
  ref.setConstant(pack_sentinel<Scalar>());
  for (Index j = 0; j < cols; j += NR) {
    const Index w = numext::mini(NR, cols - j);
    const Index base = PanelMode ? j * dst_stride + dst_offset * w : j * depth;
    for (Index k = 0; k < depth; ++k)
      for (Index c = 0; c < w; ++c) {
        const Scalar v = V(k, j + c);
        set_packed(ref.data() + base, w, k, c, Conjugate ? numext::conj(v) : v);
      }
  }
}

// Every packer check below is the same nine lines -- fill a sentinel buffer,
// build the scalar reference, run the packer, compare exactly -- over a
// different mapper. The mapper is what each check is really about, so it is
// built by the caller and the rest lives here once.
template <typename Scalar, int StorageOrder, bool PanelMode, bool Conjugate, typename Mapper, typename MatrixType>
static void check_packed_lhs(const Mapper& mapper, const MatrixType& V, Index rows, Index depth) {
  const Index dst_stride = PanelMode ? depth + 5 : 0;
  const Index dst_offset = PanelMode ? 3 : 0;
  const Index len = packed_len<PanelMode>(rows, depth, sme_mr<Scalar>(), dst_stride, dst_offset);
  SmeVector<Scalar> packed = SmeVector<Scalar>::Constant(len, pack_sentinel<Scalar>());
  SmeVector<Scalar> ref(len);
  fill_lhs_ref<Scalar, PanelMode, Conjugate>(ref, V, rows, depth, dst_stride, dst_offset);

  internal::gemm_pack_lhs<Scalar, Index, Mapper, sme_mr<Scalar>(), 1, typename internal::packet_traits<Scalar>::type,
                          StorageOrder, Conjugate, PanelMode>
      pack;
  pack(packed.data(), mapper, depth, rows, dst_stride, dst_offset);
  VERIFY_IS_EQUAL(packed, ref);
}

template <typename Scalar, int StorageOrder, bool PanelMode, bool Conjugate, typename Mapper, typename MatrixType>
static void check_packed_rhs(const Mapper& mapper, const MatrixType& V, Index depth, Index cols) {
  const Index dst_stride = PanelMode ? depth + 5 : 0;
  const Index dst_offset = PanelMode ? 3 : 0;
  const Index len = packed_len<PanelMode>(cols, depth, sme_nr<Scalar>(), dst_stride, dst_offset);
  SmeVector<Scalar> packed = SmeVector<Scalar>::Constant(len, pack_sentinel<Scalar>());
  SmeVector<Scalar> ref(len);
  fill_rhs_ref<Scalar, PanelMode, Conjugate>(ref, V, cols, depth, dst_stride, dst_offset);

  internal::gemm_pack_rhs<Scalar, Index, Mapper, sme_nr<Scalar>(), StorageOrder, Conjugate, PanelMode> pack;
  pack(packed.data(), mapper, depth, cols, dst_stride, dst_offset);
  VERIFY_IS_EQUAL(packed, ref);
}

// The raw pointer + stride packers, taken whenever the mapper grants direct
// unit-inner-stride access.
template <typename Scalar, int StorageOrder, bool PanelMode, bool Conjugate>
static void verify_pack_lhs_direct(Index rows, Index depth) {
  using MatrixType = Matrix<Scalar, Dynamic, Dynamic, StorageOrder>;
  using Mapper = internal::const_blas_data_mapper<Scalar, Index, StorageOrder>;
  MatrixType V = MatrixType::Random(rows, depth);
  check_packed_lhs<Scalar, StorageOrder, PanelMode, Conjugate>(Mapper(V.data(), V.outerStride()), V, rows, depth);
}

template <typename Scalar, int StorageOrder, bool PanelMode, bool Conjugate>
static void verify_pack_rhs_direct(Index depth, Index cols) {
  using MatrixType = Matrix<Scalar, Dynamic, Dynamic, StorageOrder>;
  using Mapper = internal::const_blas_data_mapper<Scalar, Index, StorageOrder>;
  MatrixType V = MatrixType::Random(depth, cols);
  check_packed_rhs<Scalar, StorageOrder, PanelMode, Conjugate>(Mapper(V.data(), V.outerStride()), V, depth, cols);
}

// Inner-strided blas mapper LHS: element(i, k) laid out with inner stride
// `incr`.  ColMajor takes the vectorised gather path; RowMajor takes the scalar
// path (its packets would run along depth, not rows).
template <typename Scalar, int StorageOrder, bool PanelMode, bool Conjugate>
static void verify_fallback_lhs_strided(Index rows, Index depth, Index incr) {
  using Mapper = internal::blas_data_mapper<Scalar, Index, StorageOrder, Unaligned, Dynamic>;
  Matrix<Scalar, Dynamic, Dynamic> V = Matrix<Scalar, Dynamic, Dynamic>::Random(rows, depth);
  const Index mstride = (StorageOrder == ColMajor ? rows : depth) * incr;
  SmeVector<Scalar> buf = SmeVector<Scalar>::Zero((StorageOrder == ColMajor ? depth : rows) * mstride + incr);
  for (Index k = 0; k < depth; ++k)
    for (Index i = 0; i < rows; ++i)
      buf[StorageOrder == ColMajor ? i * incr + k * mstride : k * incr + i * mstride] = V(i, k);
  check_packed_lhs<Scalar, StorageOrder, PanelMode, Conjugate>(Mapper(buf.data(), mstride, incr), V, rows, depth);
}

// By-value LHS mappers exercise both packet directions. RowMajor must stay
// scalar because its packets advance depth rather than rows.
template <typename Scalar, int StorageOrder, bool PanelMode, bool Conjugate>
static void verify_fallback_lhs_byvalue(Index rows, Index depth) {
  using MatrixType = Matrix<Scalar, Dynamic, Dynamic, StorageOrder>;
  using Mapper = typename std::conditional<StorageOrder == ColMajor, ByValueColMajorLhsMapper<Scalar>,
                                           ByValueRowMajorLhsMapper<Scalar>>::type;
  MatrixType V = MatrixType::Random(rows, depth);
  check_packed_lhs<Scalar, StorageOrder, PanelMode, Conjugate>(Mapper{V.data(), V.outerStride()}, V, rows, depth);
}

// Inner-strided blas mapper RHS: element(k, col) with inner stride `incr`.
// ColMajor takes the vectorised transpose path; RowMajor takes the scalar path.
template <typename Scalar, int StorageOrder, bool PanelMode, bool Conjugate>
static void verify_fallback_rhs_strided(Index depth, Index cols, Index incr) {
  using Mapper = internal::blas_data_mapper<Scalar, Index, StorageOrder, Unaligned, Dynamic>;
  Matrix<Scalar, Dynamic, Dynamic> V = Matrix<Scalar, Dynamic, Dynamic>::Random(depth, cols);
  const Index mstride = (StorageOrder == ColMajor ? depth : cols) * incr;
  SmeVector<Scalar> buf = SmeVector<Scalar>::Zero((StorageOrder == ColMajor ? cols : depth) * mstride + incr);
  for (Index col = 0; col < cols; ++col)
    for (Index k = 0; k < depth; ++k)
      buf[StorageOrder == ColMajor ? k * incr + col * mstride : col * incr + k * mstride] = V(k, col);
  check_packed_rhs<Scalar, StorageOrder, PanelMode, Conjugate>(Mapper(buf.data(), mstride, incr), V, depth, cols);
}

// By-value RHS mappers likewise cover both packet directions. RowMajor packets
// advance columns, so the depth-oriented transpose fallback must stay scalar.
template <typename Scalar, int StorageOrder, bool PanelMode, bool Conjugate>
static void verify_fallback_rhs_byvalue(Index depth, Index cols) {
  using MatrixType = Matrix<Scalar, Dynamic, Dynamic, StorageOrder>;
  using Mapper = typename std::conditional<StorageOrder == ColMajor, ByValueColMajorRhsMapper<Scalar>,
                                           ByValueRowMajorRhsMapper<Scalar>>::type;
  MatrixType V = MatrixType::Random(depth, cols);
  check_packed_rhs<Scalar, StorageOrder, PanelMode, Conjugate>(Mapper{V.data(), V.outerStride()}, V, depth, cols);
}

// Each packer is swept over all four (PanelMode, Conjugate) combinations.
// Conjugate=true is the identity on real scalars; for complex ones it negates
// the packed imaginary half, and reaches the direct packers from the SYMM
// above-diagonal transposed pack and the fallbacks from an inner-strided
// selfadjoint operand.
template <typename Scalar, int SO>
static void sweep_pack_direct(Index n, Index depth) {
  verify_pack_lhs_direct<Scalar, SO, false, false>(n, depth);
  verify_pack_lhs_direct<Scalar, SO, false, true>(n, depth);
  verify_pack_lhs_direct<Scalar, SO, true, false>(n, depth);
  verify_pack_lhs_direct<Scalar, SO, true, true>(n, depth);
  verify_pack_rhs_direct<Scalar, SO, false, false>(depth, n);
  verify_pack_rhs_direct<Scalar, SO, false, true>(depth, n);
  verify_pack_rhs_direct<Scalar, SO, true, false>(depth, n);
  verify_pack_rhs_direct<Scalar, SO, true, true>(depth, n);
}

template <typename Scalar, int SO>
static void sweep_fallback_strided(Index n, Index depth, Index incr) {
  verify_fallback_lhs_strided<Scalar, SO, false, false>(n, depth, incr);
  verify_fallback_lhs_strided<Scalar, SO, false, true>(n, depth, incr);
  verify_fallback_lhs_strided<Scalar, SO, true, false>(n, depth, incr);
  verify_fallback_lhs_strided<Scalar, SO, true, true>(n, depth, incr);
  verify_fallback_rhs_strided<Scalar, SO, false, false>(depth, n, incr);
  verify_fallback_rhs_strided<Scalar, SO, false, true>(depth, n, incr);
  verify_fallback_rhs_strided<Scalar, SO, true, false>(depth, n, incr);
  verify_fallback_rhs_strided<Scalar, SO, true, true>(depth, n, incr);
}

template <typename Scalar, int SO>
static void sweep_fallback_byvalue(Index n, Index depth) {
  verify_fallback_lhs_byvalue<Scalar, SO, false, false>(n, depth);
  verify_fallback_lhs_byvalue<Scalar, SO, false, true>(n, depth);
  verify_fallback_lhs_byvalue<Scalar, SO, true, false>(n, depth);
  verify_fallback_lhs_byvalue<Scalar, SO, true, true>(n, depth);
  verify_fallback_rhs_byvalue<Scalar, SO, false, false>(depth, n);
  verify_fallback_rhs_byvalue<Scalar, SO, false, true>(depth, n);
  verify_fallback_rhs_byvalue<Scalar, SO, true, false>(depth, n);
  verify_fallback_rhs_byvalue<Scalar, SO, true, true>(depth, n);
}

template <typename Scalar>
static void test_pack_direct() {
  const int TILE = sme_tile<Scalar>();
  const int MR = sme_mr<Scalar>();
  const int NR = sme_nr<Scalar>();
  // Widths around the tile side and both panel widths, which differ for
  // complex scalars.
  const int widths[] = {1, TILE - 1, TILE, TILE + 1, MR, MR + 1, NR, NR + 1, 2 * NR + 1};
  const int depths[] = {1, 3, 8, 35};
  for (int d : depths) {
    for (int n : widths) {
      sweep_pack_direct<Scalar, ColMajor>(n, d);
      sweep_pack_direct<Scalar, RowMajor>(n, d);
    }
  }
}

template <typename Scalar>
static void test_mapper_fallback() {
  const int MR = sme_mr<Scalar>();
  const int widths[] = {4, 5, MR, MR + 1, 2 * MR + 1};  // rows/cols around 4 and the panel width
  const int depths[] = {1, 3, 8, 35};                   // depth remainders 1..3 and larger
  for (int n : widths) {
    for (int d : depths) {
      // RowMajor mappers take the scalar path: their packets advance the index
      // the fallback does not transpose.
      for (int incr : {2, 3}) {
        sweep_fallback_strided<Scalar, ColMajor>(n, d, incr);
        sweep_fallback_strided<Scalar, RowMajor>(n, d, incr);
      }
      sweep_fallback_byvalue<Scalar, ColMajor>(n, d);
      sweep_fallback_byvalue<Scalar, RowMajor>(n, d);
    }
  }
}

// ---------------------------------------------------------------------------
// Product-level coverage, swept relative to the scalar's own block width.
// ---------------------------------------------------------------------------

// Sizes that land just on and off the block tails and the intra-block ZA-tile
// splits, for both panel widths (they differ for complex scalars) and the tile
// side at the SVL=512 design point.
template <typename Scalar>
static std::vector<int> sme_edge_sizes() {
  const int T = sme_tile<Scalar>();
  const int MR = sme_mr<Scalar>();
  const int NR = sme_nr<Scalar>();
  std::vector<int> sizes = {1,  T - 1,  T,          T + 1,  MR - 1,     MR,         MR + 1, NR - 1,
                            NR, NR + 1, 2 * MR - 1, 2 * MR, 2 * MR + 1, 2 * NR - 1, 2 * NR, 2 * NR + 1};
  std::sort(sizes.begin(), sizes.end());
  sizes.erase(std::unique(sizes.begin(), sizes.end()), sizes.end());
  return sizes;
}

// The dimension-sum and output-area bounds in GeneralProduct.h route a small
// square product to the coeff-based evaluator, so the n x n sweeps above stop
// reaching the SME kernel once n gets small -- which is exactly where the
// intra-block ZA tile-split edges (T-1, T, T+1) live for double and
// complex<double>.  Repeat the same widths over a depth that clears both
// bounds, so every m and n edge still reaches the kernel, and check the
// conjugated forms too: at those sizes test_conjugated_products compares a
// lazyProduct against a lazyProduct and cannot fail.
template <typename Scalar>
static void test_edge_sizes_deep_k() {
  const int depth = 4 * sme_mr<Scalar>();
  for (int n : sme_edge_sizes<Scalar>()) {
    if (n < 2) continue;
    const SmeColMajorMat<Scalar> A = SmeColMajorMat<Scalar>::Random(n, depth);
    const SmeColMajorMat<Scalar> B = SmeColMajorMat<Scalar>::Random(depth, n);
    const SmeColMajorMat<Scalar> Aa = A.adjoint().eval();
    const SmeColMajorMat<Scalar> Ba = B.adjoint().eval();
    const SmeColMajorMat<Scalar> c_before = SmeColMajorMat<Scalar>::Random(n, n);
    const SmeColMajorMat<Scalar> expected = c_before + A.lazyProduct(B);

    SmeColMajorMat<Scalar> C = c_before;
    C.noalias() += A * B;
    VERIFY_IS_APPROX(C, expected);

    C = c_before;
    C.noalias() += Aa.adjoint() * B;
    VERIFY_IS_APPROX(C, expected);

    C = c_before;
    C.noalias() += A * Ba.adjoint();
    VERIFY_IS_APPROX(C, expected);
  }
}

template <typename Scalar>
static void test_conjugated_products() {
  for (int n : sme_edge_sizes<Scalar>()) {
    verify_conjugated_products<Scalar, SmeColMajorMat<Scalar>, SmeColMajorMat<Scalar>, SmeColMajorMat<Scalar>>(n);
    verify_conjugated_products<Scalar, SmeRowMajorMat<Scalar>, SmeColMajorMat<Scalar>, SmeColMajorMat<Scalar>>(n);
    verify_conjugated_products<Scalar, SmeColMajorMat<Scalar>, SmeRowMajorMat<Scalar>, SmeRowMajorMat<Scalar>>(n);
    verify_conjugated_products<Scalar, SmeRowMajorMat<Scalar>, SmeRowMajorMat<Scalar>, SmeColMajorMat<Scalar>>(n);
  }
}

template <typename Scalar>
static void test_products() {
  const int MR = sme_mr<Scalar>();

  // Square edge cases around the block and tile boundaries.
  for (int n : sme_edge_sizes<Scalar>()) product(SmeColMajorMat<Scalar>(n, n));

  // Thin / wide rectangular cases (M x 1, 1 x N) and non-square cases that
  // exercise tail paths for both M and N.
  product(SmeColMajorMat<Scalar>(MR, 1));
  product(SmeColMajorMat<Scalar>(1, MR));
  product(SmeColMajorMat<Scalar>(1, 2 * MR));
  product(SmeColMajorMat<Scalar>(2 * MR, 1));
  product(SmeColMajorMat<Scalar>(MR + 1, 2 * MR + 1));
  product(SmeColMajorMat<Scalar>(2 * MR + 1, MR + 1));
  product(SmeColMajorMat<Scalar>(MR - 1, 2 * MR - 1));
  product(SmeColMajorMat<Scalar>(MR + 1, 7));
  product(SmeColMajorMat<Scalar>(7, MR + 1));
  product(SmeColMajorMat<Scalar>(4 * MR, 3));
  product(SmeColMajorMat<Scalar>(3, 4 * MR));

  test_deep_k_split<Scalar>();
  test_edge_sizes_deep_k<Scalar>();

  // Random sizes
  for (int i = 0; i < g_repeat; i++) {
    product(SmeColMajorMat<Scalar>(internal::random<int>(1, EIGEN_TEST_MAX_SIZE),
                                   internal::random<int>(1, EIGEN_TEST_MAX_SIZE)));
  }

  // Exercise the RowMajor packers and RowMajor result path.  When the input
  // MatrixType is RowMajor, product() instantiates m1/m2/m3/res in RowMajor,
  // so every matrix-matrix product in the suite flows through:
  //   - the RowMajor LHS packer (gemm_pack_lhs<..., RowMajor>)
  //   - the RowMajor RHS packer (gemm_pack_rhs<..., RowMajor>)
  //   - the RowMajor-C dispatch in GeneralMatrixMatrix.h (which transposes
  //     the computation: C^T = B^T * A^T).
  for (int n : sme_edge_sizes<Scalar>()) {
    if (n > 1) product(SmeRowMajorMat<Scalar>(n, n));
  }
  product(SmeRowMajorMat<Scalar>(MR + 1, 2 * MR + 1));
  product(SmeRowMajorMat<Scalar>(2 * MR + 1, MR + 1));
  for (int i = 0; i < g_repeat; i++) {
    product(SmeRowMajorMat<Scalar>(internal::random<int>(1, EIGEN_TEST_MAX_SIZE),
                                   internal::random<int>(1, EIGEN_TEST_MAX_SIZE)));
  }

  // Exercise the general-stride branch of sme_store_za_tile: fires when both
  // C_stride_row != 1 and C_stride_col != 1, e.g. a Map<Matrix> with an
  // explicit non-unit inner stride.  product.h never builds such a result, so
  // without this subtest the scalar-scatter path is effectively untested.
  for (int n : sme_edge_sizes<Scalar>()) {
    if (n < 2) continue;
    test_general_strided_result<Scalar, SmeColMajorMat<Scalar>>(n);
    test_general_strided_result<Scalar, SmeRowMajorMat<Scalar>>(n);
    test_rowmajor_strided_result<Scalar>(n);
  }

  // Row-LHS x Row-RHS -> Col-C: the one LHS/RHS/C storage combination that
  // product.h's transpose-style expressions never build directly (it always
  // flips one side of the multiplication).  The code paths are the same as
  // other combinations via Eigen's dispatch, but exercise them explicitly.
  for (int n : sme_edge_sizes<Scalar>()) {
    if (n < 2) continue;
    SmeRowMajorMat<Scalar> A = SmeRowMajorMat<Scalar>::Random(n, n);
    SmeRowMajorMat<Scalar> B = SmeRowMajorMat<Scalar>::Random(n, n);
    SmeColMajorMat<Scalar> C = SmeColMajorMat<Scalar>::Zero(n, n);
    C.noalias() += A * B;
    VERIFY_IS_APPROX(C, (A.lazyProduct(B)).eval());
  }
}

EIGEN_DECLARE_TEST(product_sme) {
  CALL_SUBTEST_1(test_whilelt_operand_types<float>());
  CALL_SUBTEST_1(test_products<float>());
  CALL_SUBTEST_1(test_conjugated_products<float>());
  CALL_SUBTEST_1(test_symm_pack<float>());
  CALL_SUBTEST_1(test_pack_direct<float>());
  CALL_SUBTEST_1(test_mapper_fallback<float>());

  // double reaches the SME kernel and packers only with FEAT_SME_F64F64; the
  // product sweep is meaningful either way, but the packed-layout tests name
  // specializations that only exist when it is available.
  CALL_SUBTEST_2(test_products<double>());
  CALL_SUBTEST_2(test_conjugated_products<double>());
#ifdef EIGEN_VECTORIZE_SME_F64F64
  CALL_SUBTEST_2(test_whilelt_operand_types<double>());
  CALL_SUBTEST_2(test_symm_pack<double>());
  CALL_SUBTEST_2(test_pack_direct<double>());
  CALL_SUBTEST_2(test_mapper_fallback<double>());
#endif

  CALL_SUBTEST_3(test_products<std::complex<float>>());
  CALL_SUBTEST_3(test_conjugated_products<std::complex<float>>());
  CALL_SUBTEST_3(test_symm_pack<std::complex<float>>());
  CALL_SUBTEST_3(test_pack_direct<std::complex<float>>());
  CALL_SUBTEST_3(test_mapper_fallback<std::complex<float>>());

  // complex<double> accumulates into ZA.D tiles, so it needs FEAT_SME_F64F64
  // exactly as double does.
  CALL_SUBTEST_4(test_products<std::complex<double>>());
  CALL_SUBTEST_4(test_conjugated_products<std::complex<double>>());
#ifdef EIGEN_VECTORIZE_SME_F64F64
  CALL_SUBTEST_4(test_symm_pack<std::complex<double>>());
  CALL_SUBTEST_4(test_pack_direct<std::complex<double>>());
  CALL_SUBTEST_4(test_mapper_fallback<std::complex<double>>());
#endif

  // A scalar type SME does not specialize, proving it still routes through the
  // generic product path inside an SME build -- where packet traits, alignment
  // and cache blocking all differ from a plain NEON build.
  CALL_SUBTEST_5(product(Matrix<long double, Dynamic, Dynamic>(33, 17)));
}
