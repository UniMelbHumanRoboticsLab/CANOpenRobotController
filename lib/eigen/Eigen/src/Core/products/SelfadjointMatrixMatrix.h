// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_SELFADJOINT_MATRIX_MATRIX_H
#define EIGEN_SELFADJOINT_MATRIX_MATRIX_H

// IWYU pragma: private
#include "../InternalHeaderCheck.h"

namespace Eigen {

namespace internal {

// pack a selfadjoint block diagonal for use with the gebp_kernel
//
// The panel-width schedule and depth-major layout must match gemm_pack_lhs:
// gebp_kernel cannot tell symm-packed and gemm-packed operands apart. A panel
// of width bw at row offset i splits at the diagonal: depth [0, i) reads the
// stored triangle directly, depth [i + bw, cols) reads its conjugated mirror
// through a transposed-order mapper, and both rectangles delegate to the
// (vectorized) gemm_pack_lhs; only the bw x bw diagonal band is packed per row.
template <typename Scalar, typename Index, int Pack1, int Pack2, int StorageOrder>
struct symm_pack_lhs {
  static constexpr int TransposedStorageOrder = StorageOrder == ColMajor ? RowMajor : ColMajor;
  static constexpr int PacketSize = packet_traits<Scalar>::size;
  using PacketType = typename packet_traits<Scalar>::type;
  using Mapper = const_blas_data_mapper<Scalar, Index, StorageOrder>;
  using TransposedMapper = const_blas_data_mapper<Scalar, Index, TransposedStorageOrder>;
  // Pack2 is Traits::LhsProgress, the same value the driver hands its
  // gemm_pack_lhs for the off-diagonal panels: the diagonal block has to be
  // packed the way those are, since gebp_kernel consumes them alike.
  using DirectPacker = gemm_pack_lhs<Scalar, Index, Mapper, Pack1, Pack2, PacketType, StorageOrder, false, false>;
  using MirroredPacker =
      gemm_pack_lhs<Scalar, Index, TransposedMapper, Pack1, Pack2, PacketType, TransposedStorageOrder, true, false>;

  void pack_panel(Scalar* blockA, const Mapper& lhs, const TransposedMapper& lhs_t, Index cols, Index i, Index bw,
                  Index& count) const {
    Scalar* panel = blockA + count;
    const Index band_end = i + bw;
    if (i > 0) DirectPacker()(panel, lhs.getSubMapper(i, 0), i, bw);
    // Diagonal band: row k mirrors its first k - i entries, drops the
    // imaginary part of the diagonal, and reads the rest directly.
    for (Index k = i; k < band_end; k++) {
      Scalar* row = panel + k * bw;
      const Index h = k - i;
      for (Index w = 0; w < h; w++) row[w] = numext::conj(lhs(k, i + w));
      row[h] = numext::real(lhs(k, k));
      for (Index w = h + 1; w < bw; w++) row[w] = lhs(i + w, k);
    }
    if (cols > band_end) MirroredPacker()(panel + band_end * bw, lhs_t.getSubMapper(i, band_end), cols - band_end, bw);
    count += bw * numext::maxi(cols, band_end);
  }

  void operator()(Scalar* blockA, const Scalar* lhs_, Index lhsStride, Index cols, Index rows) const {
    using HalfPacket = typename unpacket_traits<PacketType>::half;
    using QuarterPacket = typename unpacket_traits<HalfPacket>::half;
    constexpr int HalfPacketSize = unpacket_traits<HalfPacket>::size;
    constexpr int QuarterPacketSize = unpacket_traits<QuarterPacket>::size;
    constexpr bool HasHalf = HalfPacketSize < PacketSize;
    constexpr bool HasQuarter = QuarterPacketSize < HalfPacketSize;

    const Mapper lhs(lhs_, lhsStride);
    const TransposedMapper lhs_t(lhs_, lhsStride);
    Index count = 0;

    const Index peeled_mc3 = Pack1 >= 3 * PacketSize ? (rows / (3 * PacketSize)) * (3 * PacketSize) : 0;
    const Index peeled_mc2 =
        Pack1 >= 2 * PacketSize ? peeled_mc3 + ((rows - peeled_mc3) / (2 * PacketSize)) * (2 * PacketSize) : 0;
    const Index peeled_mc1 =
        Pack1 >= 1 * PacketSize ? peeled_mc2 + ((rows - peeled_mc2) / (1 * PacketSize)) * (1 * PacketSize) : 0;
    const Index peeled_mc_half =
        Pack1 >= HalfPacketSize ? peeled_mc1 + ((rows - peeled_mc1) / (HalfPacketSize)) * (HalfPacketSize) : 0;
    const Index peeled_mc_quarter =
        Pack1 >= QuarterPacketSize
            ? peeled_mc_half + ((rows - peeled_mc_half) / (QuarterPacketSize)) * (QuarterPacketSize)
            : 0;

    auto pack_rows = [&](Index begin, Index end, Index bw) {
      for (Index i = begin; i < end; i += bw) pack_panel(blockA, lhs, lhs_t, cols, i, bw, count);
    };

    EIGEN_IF_CONSTEXPR (Pack1 >= 3 * PacketSize) pack_rows(0, peeled_mc3, 3 * PacketSize);
    EIGEN_IF_CONSTEXPR (Pack1 >= 2 * PacketSize) pack_rows(peeled_mc3, peeled_mc2, 2 * PacketSize);
    EIGEN_IF_CONSTEXPR (Pack1 >= 1 * PacketSize) pack_rows(peeled_mc2, peeled_mc1, 1 * PacketSize);
    EIGEN_IF_CONSTEXPR (HasHalf && Pack1 >= HalfPacketSize) pack_rows(peeled_mc1, peeled_mc_half, HalfPacketSize);
    EIGEN_IF_CONSTEXPR (HasQuarter && Pack1 >= QuarterPacketSize)
      pack_rows(peeled_mc_half, peeled_mc_quarter, QuarterPacketSize);

    // do the same with mr==1
    pack_rows(peeled_mc_quarter, rows, 1);
  }
};

template <typename Scalar, typename Index, int nr, int StorageOrder>
struct symm_pack_rhs {
  static constexpr int TransposedStorageOrder = StorageOrder == ColMajor ? RowMajor : ColMajor;
  using Mapper = const_blas_data_mapper<Scalar, Index, StorageOrder>;
  using TransposedMapper = const_blas_data_mapper<Scalar, Index, TransposedStorageOrder>;
  using DirectPacker = gemm_pack_rhs<Scalar, Index, Mapper, nr, StorageOrder, false, false>;
  using MirroredPacker = gemm_pack_rhs<Scalar, Index, TransposedMapper, nr, TransposedStorageOrder, true, false>;

  // One nr-wide panel the diagonal crosses: a conjugated-mirror head above the
  // diagonal, the Width x Width band packed per row, and a direct tail below.
  template <int Width>
  void pack_diagonal_panel(Scalar* blockB, const Mapper& rhs, const TransposedMapper& rhs_t, Index k2, Index end_k,
                           Index j2, Index& count) const {
    const Index band_end = j2 + Width;
    if (j2 > k2) {
      MirroredPacker()(blockB + count, rhs_t.getSubMapper(k2, j2), j2 - k2, Width);
      count += Width * (j2 - k2);
    }
    for (Index k = j2; k < band_end; k++) {
      Scalar* row = blockB + count;
      const Index h = k - j2;
      for (Index w = 0; w < h; w++) row[w] = rhs(k, j2 + w);
      row[h] = numext::real(rhs(k, k));
      for (Index w = h + 1; w < Width; w++) row[w] = numext::conj(rhs(j2 + w, k));
      count += Width;
    }
    if (end_k > band_end) {
      DirectPacker()(blockB + count, rhs.getSubMapper(band_end, j2), end_k - band_end, Width);
      count += Width * (end_k - band_end);
    }
  }

  void operator()(Scalar* blockB, const Scalar* rhs_, Index rhsStride, Index rows, Index cols, Index k2) const {
    const Index end_k = k2 + rows;
    const Mapper rhs(rhs_, rhsStride);
    const TransposedMapper rhs_t(rhs_, rhsStride);
    Index count = 0;
    const Index packet_cols8 = nr >= 8 ? (cols / 8) * 8 : 0;
    const Index packet_cols4 = nr >= 4 ? (cols / 4) * 4 : 0;

    // first part: whole panels left of the diagonal block, read directly.
    // k2 is a multiple of the panel width (blocking rounds kc to a multiple
    // of 8), so this region and the delegate lie on the same panel grid.
    // The delegates re-derive that grid from the extent they are handed, so a
    // k2 or end_k off the grid would silently pack a correctly sized buffer
    // with the wrong panel shape.
    eigen_internal_assert(k2 % nr == 0);
    eigen_internal_assert(end_k % nr == 0 || end_k == cols);
    if (k2 > 0) {
      DirectPacker()(blockB, rhs.getSubMapper(k2, 0), rows, k2);
      count += k2 * rows;
    }

    // second part: the panels the diagonal block crosses
    const Index end8 = nr >= 8 ? numext::mini(end_k, packet_cols8) : k2;
    const Index end4 = numext::mini(end_k, packet_cols4);
    EIGEN_IF_CONSTEXPR (nr >= 8) {
      for (Index j2 = k2; j2 < end8; j2 += 8) pack_diagonal_panel<8>(blockB, rhs, rhs_t, k2, end_k, j2, count);
    }
    EIGEN_IF_CONSTEXPR (nr >= 4) {
      for (Index j2 = end8; j2 < end4; j2 += 4) pack_diagonal_panel<4>(blockB, rhs, rhs_t, k2, end_k, j2, count);
    }

    // third part: whole panels right of the diagonal block, read mirrored
    EIGEN_IF_CONSTEXPR (nr >= 8) {
      if (packet_cols8 > end_k) {
        MirroredPacker()(blockB + count, rhs_t.getSubMapper(k2, end_k), rows, packet_cols8 - end_k);
        count += rows * (packet_cols8 - end_k);
      }
    }
    EIGEN_IF_CONSTEXPR (nr >= 4) {
      const Index j3 = numext::maxi(packet_cols8, end_k);
      if (packet_cols4 > j3) {
        MirroredPacker()(blockB + count, rhs_t.getSubMapper(k2, j3), rows, packet_cols4 - j3);
        count += rows * (packet_cols4 - j3);
      }
    }

    // copy the remaining columns one at a time (=> the same with nr==1)
    for (Index j2 = packet_cols4; j2 < cols; ++j2) {
      // transpose
      Index half = numext::mini(end_k, j2);
      for (Index k = k2; k < half; k++) {
        blockB[count] = numext::conj(rhs(j2, k));
        count += 1;
      }

      if (half == j2 && half < k2 + rows) {
        blockB[count] = numext::real(rhs(j2, j2));
        count += 1;
      } else
        half--;

      // normal
      for (Index k = half + 1; k < k2 + rows; k++) {
        blockB[count] = rhs(k, j2);
        count += 1;
      }
    }
  }
};

/* Optimized selfadjoint matrix * matrix (_SYMM) product built on top of
 * the general matrix matrix product.
 */
template <typename Scalar, typename Index, int LhsStorageOrder, bool LhsSelfAdjoint, bool ConjugateLhs,
          int RhsStorageOrder, bool RhsSelfAdjoint, bool ConjugateRhs, int ResStorageOrder, int ResInnerStride>
struct product_selfadjoint_matrix;

template <typename Scalar, typename Index, int LhsStorageOrder, bool LhsSelfAdjoint, bool ConjugateLhs,
          int RhsStorageOrder, bool RhsSelfAdjoint, bool ConjugateRhs, int ResInnerStride>
struct product_selfadjoint_matrix<Scalar, Index, LhsStorageOrder, LhsSelfAdjoint, ConjugateLhs, RhsStorageOrder,
                                  RhsSelfAdjoint, ConjugateRhs, RowMajor, ResInnerStride> {
  static EIGEN_STRONG_INLINE void run(Index rows, Index cols, const Scalar* lhs, Index lhsStride, const Scalar* rhs,
                                      Index rhsStride, Scalar* res, Index resIncr, Index resStride, const Scalar& alpha,
                                      level3_blocking<Scalar, Scalar>& blocking) {
    product_selfadjoint_matrix<
        Scalar, Index, logical_xor(RhsSelfAdjoint, RhsStorageOrder == RowMajor) ? ColMajor : RowMajor, RhsSelfAdjoint,
        NumTraits<Scalar>::IsComplex && logical_xor(RhsSelfAdjoint, ConjugateRhs),
        logical_xor(LhsSelfAdjoint, LhsStorageOrder == RowMajor) ? ColMajor : RowMajor, LhsSelfAdjoint,
        NumTraits<Scalar>::IsComplex && logical_xor(LhsSelfAdjoint, ConjugateLhs), ColMajor,
        ResInnerStride>::run(cols, rows, rhs, rhsStride, lhs, lhsStride, res, resIncr, resStride, alpha, blocking);
  }
};

template <typename Scalar, typename Index, int LhsStorageOrder, bool ConjugateLhs, int RhsStorageOrder,
          bool ConjugateRhs, int ResInnerStride>
struct product_selfadjoint_matrix<Scalar, Index, LhsStorageOrder, true, ConjugateLhs, RhsStorageOrder, false,
                                  ConjugateRhs, ColMajor, ResInnerStride> {
  static EIGEN_DONT_INLINE void run(Index rows, Index cols, const Scalar* lhs_, Index lhsStride, const Scalar* rhs_,
                                    Index rhsStride, Scalar* res, Index resIncr, Index resStride, const Scalar& alpha,
                                    level3_blocking<Scalar, Scalar>& blocking);
};

template <typename Scalar, typename Index, int LhsStorageOrder, bool ConjugateLhs, int RhsStorageOrder,
          bool ConjugateRhs, int ResInnerStride>
EIGEN_DONT_INLINE void
product_selfadjoint_matrix<Scalar, Index, LhsStorageOrder, true, ConjugateLhs, RhsStorageOrder, false, ConjugateRhs,
                           ColMajor, ResInnerStride>::run(Index rows, Index cols, const Scalar* lhs_, Index lhsStride,
                                                          const Scalar* rhs_, Index rhsStride, Scalar* res_,
                                                          Index resIncr, Index resStride, const Scalar& alpha,
                                                          level3_blocking<Scalar, Scalar>& blocking) {
  Index size = rows;

  using Traits = gebp_traits<Scalar, Scalar>;

  using LhsMapper = const_blas_data_mapper<Scalar, Index, LhsStorageOrder>;
  using LhsTransposeMapper = const_blas_data_mapper<Scalar, Index, (LhsStorageOrder == RowMajor) ? ColMajor : RowMajor>;
  using RhsMapper = const_blas_data_mapper<Scalar, Index, RhsStorageOrder>;
  using ResMapper = blas_data_mapper<typename Traits::ResScalar, Index, ColMajor, Unaligned, ResInnerStride>;
  LhsMapper lhs(lhs_, lhsStride);
  LhsTransposeMapper lhs_transpose(lhs_, lhsStride);
  RhsMapper rhs(rhs_, rhsStride);
  ResMapper res(res_, resStride, resIncr);

  Index kc = blocking.kc();                    // cache block size along the K direction
  Index mc = (std::min)(rows, blocking.mc());  // cache block size along the M direction
  // kc must be smaller than mc
  kc = (std::min)(kc, mc);
  std::size_t sizeA = kc * mc;
  std::size_t sizeB = kc * cols;
  ei_declare_aligned_stack_constructed_variable(Scalar, blockA, sizeA, blocking.blockA());
  ei_declare_aligned_stack_constructed_variable(Scalar, blockB, sizeB, blocking.blockB());

  gebp_kernel<Scalar, Scalar, Index, ResMapper, Traits::mr, Traits::nr, ConjugateLhs, ConjugateRhs> gebp_kernel;
  symm_pack_lhs<Scalar, Index, Traits::mr, Traits::LhsProgress, LhsStorageOrder> pack_lhs;
  gemm_pack_rhs<Scalar, Index, RhsMapper, Traits::nr, RhsStorageOrder> pack_rhs;
  gemm_pack_lhs<Scalar, Index, LhsTransposeMapper, Traits::mr, Traits::LhsProgress, typename Traits::LhsPacket4Packing,
                LhsStorageOrder == RowMajor ? ColMajor : RowMajor, true>
      pack_lhs_transposed;

  for (Index k2 = 0; k2 < size; k2 += kc) {
    const Index actual_kc = (std::min)(k2 + kc, size) - k2;

    // we have selected one row panel of rhs and one column panel of lhs
    // pack rhs's panel into a sequential chunk of memory
    // and expand each coeff to a constant packet for further reuse
    pack_rhs(blockB, rhs.getSubMapper(k2, 0), actual_kc, cols);

    // the select lhs's panel has to be split in three different parts:
    //  1 - the transposed panel above the diagonal block => transposed packed copy
    //  2 - the diagonal block => special packed copy
    //  3 - the panel below the diagonal block => generic packed copy
    for (Index i2 = 0; i2 < k2; i2 += mc) {
      const Index actual_mc = (std::min)(i2 + mc, k2) - i2;
      // transposed packed copy
      pack_lhs_transposed(blockA, lhs_transpose.getSubMapper(i2, k2), actual_kc, actual_mc);

      gebp_kernel(res.getSubMapper(i2, 0), blockA, blockB, actual_mc, actual_kc, cols, alpha);
    }
    // the block diagonal
    {
      const Index actual_mc = (std::min)(k2 + kc, size) - k2;
      // symmetric packed copy
      pack_lhs(blockA, &lhs(k2, k2), lhsStride, actual_kc, actual_mc);

      gebp_kernel(res.getSubMapper(k2, 0), blockA, blockB, actual_mc, actual_kc, cols, alpha);
    }

    for (Index i2 = k2 + kc; i2 < size; i2 += mc) {
      const Index actual_mc = (std::min)(i2 + mc, size) - i2;
      gemm_pack_lhs<Scalar, Index, LhsMapper, Traits::mr, Traits::LhsProgress, typename Traits::LhsPacket4Packing,
                    LhsStorageOrder, false>()(blockA, lhs.getSubMapper(i2, k2), actual_kc, actual_mc);

      gebp_kernel(res.getSubMapper(i2, 0), blockA, blockB, actual_mc, actual_kc, cols, alpha);
    }
  }
}

// matrix * selfadjoint product
template <typename Scalar, typename Index, int LhsStorageOrder, bool ConjugateLhs, int RhsStorageOrder,
          bool ConjugateRhs, int ResInnerStride>
struct product_selfadjoint_matrix<Scalar, Index, LhsStorageOrder, false, ConjugateLhs, RhsStorageOrder, true,
                                  ConjugateRhs, ColMajor, ResInnerStride> {
  static EIGEN_DONT_INLINE void run(Index rows, Index cols, const Scalar* lhs_, Index lhsStride, const Scalar* rhs_,
                                    Index rhsStride, Scalar* res, Index resIncr, Index resStride, const Scalar& alpha,
                                    level3_blocking<Scalar, Scalar>& blocking);
};

template <typename Scalar, typename Index, int LhsStorageOrder, bool ConjugateLhs, int RhsStorageOrder,
          bool ConjugateRhs, int ResInnerStride>
EIGEN_DONT_INLINE void
product_selfadjoint_matrix<Scalar, Index, LhsStorageOrder, false, ConjugateLhs, RhsStorageOrder, true, ConjugateRhs,
                           ColMajor, ResInnerStride>::run(Index rows, Index cols, const Scalar* lhs_, Index lhsStride,
                                                          const Scalar* rhs_, Index rhsStride, Scalar* res_,
                                                          Index resIncr, Index resStride, const Scalar& alpha,
                                                          level3_blocking<Scalar, Scalar>& blocking) {
  Index size = cols;

  using Traits = gebp_traits<Scalar, Scalar>;

  using LhsMapper = const_blas_data_mapper<Scalar, Index, LhsStorageOrder>;
  using ResMapper = blas_data_mapper<typename Traits::ResScalar, Index, ColMajor, Unaligned, ResInnerStride>;
  LhsMapper lhs(lhs_, lhsStride);
  ResMapper res(res_, resStride, resIncr);

  Index kc = blocking.kc();                    // cache block size along the K direction
  Index mc = (std::min)(rows, blocking.mc());  // cache block size along the M direction
  std::size_t sizeA = kc * mc;
  std::size_t sizeB = kc * cols;
  ei_declare_aligned_stack_constructed_variable(Scalar, blockA, sizeA, blocking.blockA());
  ei_declare_aligned_stack_constructed_variable(Scalar, blockB, sizeB, blocking.blockB());

  gebp_kernel<Scalar, Scalar, Index, ResMapper, Traits::mr, Traits::nr, ConjugateLhs, ConjugateRhs> gebp_kernel;
  gemm_pack_lhs<Scalar, Index, LhsMapper, Traits::mr, Traits::LhsProgress, typename Traits::LhsPacket4Packing,
                LhsStorageOrder>
      pack_lhs;
  symm_pack_rhs<Scalar, Index, Traits::nr, RhsStorageOrder> pack_rhs;

  for (Index k2 = 0; k2 < size; k2 += kc) {
    const Index actual_kc = (std::min)(k2 + kc, size) - k2;

    pack_rhs(blockB, rhs_, rhsStride, actual_kc, cols, k2);

    // => GEPP
    for (Index i2 = 0; i2 < rows; i2 += mc) {
      const Index actual_mc = (std::min)(i2 + mc, rows) - i2;
      pack_lhs(blockA, lhs.getSubMapper(i2, k2), actual_kc, actual_mc);

      gebp_kernel(res.getSubMapper(i2, 0), blockA, blockB, actual_mc, actual_kc, cols, alpha);
    }
  }
}

}  // end namespace internal

/***************************************************************************
 * Wrapper to product_selfadjoint_matrix
 ***************************************************************************/

namespace internal {

template <typename Lhs, int LhsMode, typename Rhs, int RhsMode>
struct selfadjoint_product_impl<Lhs, LhsMode, false, Rhs, RhsMode, false> {
  using Scalar = typename Product<Lhs, Rhs>::Scalar;

  using LhsBlasTraits = internal::blas_traits<Lhs>;
  using ActualLhsType = typename LhsBlasTraits::DirectLinearAccessType;
  using RhsBlasTraits = internal::blas_traits<Rhs>;
  using ActualRhsType = typename RhsBlasTraits::DirectLinearAccessType;

  enum {
    LhsIsUpper = (LhsMode & (Upper | Lower)) == Upper,
    LhsIsSelfAdjoint = (LhsMode & SelfAdjoint) == SelfAdjoint,
    RhsIsUpper = (RhsMode & (Upper | Lower)) == Upper,
    RhsIsSelfAdjoint = (RhsMode & SelfAdjoint) == SelfAdjoint
  };

  template <typename Dest>
  static void run(Dest& dst, const Lhs& a_lhs, const Rhs& a_rhs, const Scalar& alpha) {
    eigen_assert(dst.rows() == a_lhs.rows() && dst.cols() == a_rhs.cols());

    add_const_on_value_type_t<ActualLhsType> lhs = LhsBlasTraits::extract(a_lhs);
    add_const_on_value_type_t<ActualRhsType> rhs = RhsBlasTraits::extract(a_rhs);

    // Empty product, return early.  Otherwise, we get `nullptr` use errors below when we try to access
    // coeffRef(0,0).
    if (lhs.size() == 0 || rhs.size() == 0) return;

    Scalar actualAlpha = alpha * LhsBlasTraits::extractScalarFactor(a_lhs) * RhsBlasTraits::extractScalarFactor(a_rhs);

    using BlockingType = internal::gemm_blocking_space<(Dest::Flags & RowMajorBit) ? RowMajor : ColMajor, Scalar,
                                                       Scalar, Lhs::MaxRowsAtCompileTime, Rhs::MaxColsAtCompileTime,
                                                       Lhs::MaxColsAtCompileTime, 1>;

    BlockingType blocking(lhs.rows(), rhs.cols(), lhs.cols(), 1, false);

    internal::product_selfadjoint_matrix<
        Scalar, Index,
        internal::logical_xor(LhsIsUpper, internal::traits<Lhs>::Flags & RowMajorBit) ? RowMajor : ColMajor,
        LhsIsSelfAdjoint,
        NumTraits<Scalar>::IsComplex && internal::logical_xor(LhsIsUpper, bool(LhsBlasTraits::NeedToConjugate)),
        internal::logical_xor(RhsIsUpper, internal::traits<Rhs>::Flags & RowMajorBit) ? RowMajor : ColMajor,
        RhsIsSelfAdjoint,
        NumTraits<Scalar>::IsComplex && internal::logical_xor(RhsIsUpper, bool(RhsBlasTraits::NeedToConjugate)),
        internal::traits<Dest>::Flags & RowMajorBit ? RowMajor : ColMajor,
        Dest::InnerStrideAtCompileTime>::run(lhs.rows(), rhs.cols(),                                     // sizes
                                             &lhs.coeffRef(0, 0), lhs.outerStride(),                     // lhs info
                                             &rhs.coeffRef(0, 0), rhs.outerStride(),                     // rhs info
                                             &dst.coeffRef(0, 0), dst.innerStride(), dst.outerStride(),  // result info
                                             actualAlpha, blocking                                       // alpha
    );
  }
};

}  // end namespace internal

}  // end namespace Eigen

#endif  // EIGEN_SELFADJOINT_MATRIX_MATRIX_H
