// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2014 Benoit Steiner <benoit.steiner.goog@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_TENSOR_TENSOR_PATCH_H
#define EIGEN_TENSOR_TENSOR_PATCH_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

namespace internal {
template <typename PatchDim, typename XprType>
struct traits<TensorPatchOp<PatchDim, XprType> > : traits<XprType> {
  typedef typename XprType::Scalar Scalar;
  typedef traits<XprType> XprTraits;
  typedef typename XprTraits::StorageKind StorageKind;
  typedef typename XprTraits::Index Index;
  static constexpr int NumDimensions = XprTraits::NumDimensions + 1;
  static constexpr int Layout = XprTraits::Layout;
  typedef typename XprTraits::PointerType PointerType;
};

template <typename PatchDim, typename XprType>
struct eval<TensorPatchOp<PatchDim, XprType>, Eigen::Dense> {
  typedef const TensorPatchOp<PatchDim, XprType>& type;
};

}  // end namespace internal

/**
 * \ingroup Tensor_Module
 *
 * \brief Tensor patch class.
 */
template <typename PatchDim, typename XprType>
class TensorPatchOp : public TensorBase<TensorPatchOp<PatchDim, XprType>, ReadOnlyAccessors> {
 public:
  typedef typename Eigen::internal::traits<TensorPatchOp>::Scalar Scalar;
  typedef typename Eigen::NumTraits<Scalar>::Real RealScalar;
  typedef typename XprType::CoeffReturnType CoeffReturnType;
  typedef typename Eigen::internal::ref_selector<TensorPatchOp>::type Nested;
  typedef typename Eigen::internal::traits<TensorPatchOp>::StorageKind StorageKind;
  typedef typename Eigen::internal::traits<TensorPatchOp>::Index Index;

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorPatchOp(const XprType& expr, const PatchDim& patch_dims)
      : m_xpr(expr), m_patch_dims(patch_dims) {}

  EIGEN_DEVICE_FUNC const PatchDim& patch_dims() const { return m_patch_dims; }

  EIGEN_DEVICE_FUNC const internal::remove_all_t<typename XprType::Nested>& expression() const { return m_xpr; }

 protected:
  typename XprType::Nested m_xpr;
  const PatchDim m_patch_dims;
};

// Eval as rvalue
template <typename PatchDim, typename ArgType, typename Device>
struct TensorEvaluator<const TensorPatchOp<PatchDim, ArgType>, Device> {
  typedef TensorPatchOp<PatchDim, ArgType> XprType;
  typedef typename XprType::Index Index;
  static constexpr int NumDims = internal::array_size<typename TensorEvaluator<ArgType, Device>::Dimensions>::value + 1;
  typedef DSizes<Index, NumDims> Dimensions;
  typedef typename XprType::Scalar Scalar;
  typedef typename XprType::CoeffReturnType CoeffReturnType;
  typedef typename PacketType<CoeffReturnType, Device>::type PacketReturnType;
  static constexpr int PacketSize = PacketType<CoeffReturnType, Device>::size;
  typedef StorageMemory<CoeffReturnType, Device> Storage;
  typedef typename Storage::Type EvaluatorPointerType;

  static constexpr int Layout = TensorEvaluator<ArgType, Device>::Layout;
  enum {
    IsAligned = false,
    PacketAccess = TensorEvaluator<ArgType, Device>::PacketAccess,
    // block() reads the argument one coefficient at a time through coeff() --
    // the contract the scalar executors already rely on for every evaluator --
    // so it requires no capability bit from the argument (same as
    // TensorReverse).
    BlockAccess = NumDims > 1,
    // The coeff/packet path pays a div/mod cascade per element; the block
    // path copies whole in-bounds boxes patch by patch.
    PreferBlockAccess = true,
    CoordAccess = false,
    RawAccess = false
  };

  //===- Tensor block evaluation strategy (see TensorBlock.h) -------------===//
  typedef internal::TensorBlockDescriptor<NumDims, Index> TensorBlockDesc;
  typedef internal::TensorBlockScratchAllocator<Device> TensorBlockScratch;
  typedef typename internal::TensorMaterializedBlock<CoeffReturnType, NumDims, Layout, Index> TensorBlock;
  //===--------------------------------------------------------------------===//

  EIGEN_STRONG_INLINE TensorEvaluator(const XprType& op, const Device& device)
      : m_impl(op.expression(), device), m_device(device) {
    Index num_patches = 1;
    const typename TensorEvaluator<ArgType, Device>::Dimensions& input_dims = m_impl.dimensions();
    const PatchDim& patch_dims = op.patch_dims();
    EIGEN_IF_CONSTEXPR (static_cast<int>(Layout) == static_cast<int>(ColMajor)) {
      for (int i = 0; i < NumDims - 1; ++i) {
        m_dimensions[i] = patch_dims[i];
        num_patches *= (input_dims[i] - patch_dims[i] + 1);
      }
      m_dimensions[NumDims - 1] = num_patches;

      m_inputStrides[0] = 1;
      m_patchStrides[0] = 1;
      for (int i = 1; i < NumDims - 1; ++i) {
        m_inputStrides[i] = m_inputStrides[i - 1] * input_dims[i - 1];
        m_patchStrides[i] = m_patchStrides[i - 1] * (input_dims[i - 1] - patch_dims[i - 1] + 1);
      }
      m_outputStrides[0] = 1;
      for (int i = 1; i < NumDims; ++i) {
        m_outputStrides[i] = m_outputStrides[i - 1] * m_dimensions[i - 1];
      }
    } else {
      for (int i = 0; i < NumDims - 1; ++i) {
        m_dimensions[i + 1] = patch_dims[i];
        num_patches *= (input_dims[i] - patch_dims[i] + 1);
      }
      m_dimensions[0] = num_patches;

      m_inputStrides[NumDims - 2] = 1;
      m_patchStrides[NumDims - 2] = 1;
      for (int i = NumDims - 3; i >= 0; --i) {
        m_inputStrides[i] = m_inputStrides[i + 1] * input_dims[i + 1];
        m_patchStrides[i] = m_patchStrides[i + 1] * (input_dims[i + 1] - patch_dims[i + 1] + 1);
      }
      m_outputStrides[NumDims - 1] = 1;
      for (int i = NumDims - 2; i >= 0; --i) {
        m_outputStrides[i] = m_outputStrides[i + 1] * m_dimensions[i + 1];
      }
    }
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const Dimensions& dimensions() const { return m_dimensions; }

  EIGEN_STRONG_INLINE bool evalSubExprsIfNeeded(EvaluatorPointerType /*data*/) {
    m_impl.evalSubExprsIfNeeded(nullptr);
    return true;
  }

  EIGEN_STRONG_INLINE void cleanup() { m_impl.cleanup(); }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE CoeffReturnType coeff(Index index) const {
    Index output_stride_index = (static_cast<int>(Layout) == static_cast<int>(ColMajor)) ? NumDims - 1 : 0;
    // Find the location of the first element of the patch.
    Index patchIndex = index / m_outputStrides[output_stride_index];
    // Find the offset of the element wrt the location of the first element.
    Index patchOffset = index - patchIndex * m_outputStrides[output_stride_index];
    Index inputIndex = 0;
    EIGEN_IF_CONSTEXPR (static_cast<int>(Layout) == static_cast<int>(ColMajor)) {
      EIGEN_UNROLL_LOOP
      for (int i = NumDims - 2; i > 0; --i) {
        const Index patchIdx = patchIndex / m_patchStrides[i];
        patchIndex -= patchIdx * m_patchStrides[i];
        const Index offsetIdx = patchOffset / m_outputStrides[i];
        patchOffset -= offsetIdx * m_outputStrides[i];
        inputIndex += (patchIdx + offsetIdx) * m_inputStrides[i];
      }
    } else {
      EIGEN_UNROLL_LOOP
      for (int i = 0; i < NumDims - 2; ++i) {
        const Index patchIdx = patchIndex / m_patchStrides[i];
        patchIndex -= patchIdx * m_patchStrides[i];
        const Index offsetIdx = patchOffset / m_outputStrides[i + 1];
        patchOffset -= offsetIdx * m_outputStrides[i + 1];
        inputIndex += (patchIdx + offsetIdx) * m_inputStrides[i];
      }
    }
    inputIndex += (patchIndex + patchOffset);
    return m_impl.coeff(inputIndex);
  }

  template <int LoadMode>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE PacketReturnType packet(Index index) const {
    eigen_assert(index + PacketSize - 1 < dimensions().TotalSize());

    Index output_stride_index = (static_cast<int>(Layout) == static_cast<int>(ColMajor)) ? NumDims - 1 : 0;
    Index indices[2] = {index, index + PacketSize - 1};
    Index patchIndices[2] = {indices[0] / m_outputStrides[output_stride_index],
                             indices[1] / m_outputStrides[output_stride_index]};
    Index patchOffsets[2] = {indices[0] - patchIndices[0] * m_outputStrides[output_stride_index],
                             indices[1] - patchIndices[1] * m_outputStrides[output_stride_index]};

    Index inputIndices[2] = {0, 0};
    EIGEN_IF_CONSTEXPR (static_cast<int>(Layout) == static_cast<int>(ColMajor)) {
      EIGEN_UNROLL_LOOP
      for (int i = NumDims - 2; i > 0; --i) {
        const Index patchIdx[2] = {patchIndices[0] / m_patchStrides[i], patchIndices[1] / m_patchStrides[i]};
        patchIndices[0] -= patchIdx[0] * m_patchStrides[i];
        patchIndices[1] -= patchIdx[1] * m_patchStrides[i];

        const Index offsetIdx[2] = {patchOffsets[0] / m_outputStrides[i], patchOffsets[1] / m_outputStrides[i]};
        patchOffsets[0] -= offsetIdx[0] * m_outputStrides[i];
        patchOffsets[1] -= offsetIdx[1] * m_outputStrides[i];

        inputIndices[0] += (patchIdx[0] + offsetIdx[0]) * m_inputStrides[i];
        inputIndices[1] += (patchIdx[1] + offsetIdx[1]) * m_inputStrides[i];
      }
    } else {
      EIGEN_UNROLL_LOOP
      for (int i = 0; i < NumDims - 2; ++i) {
        const Index patchIdx[2] = {patchIndices[0] / m_patchStrides[i], patchIndices[1] / m_patchStrides[i]};
        patchIndices[0] -= patchIdx[0] * m_patchStrides[i];
        patchIndices[1] -= patchIdx[1] * m_patchStrides[i];

        const Index offsetIdx[2] = {patchOffsets[0] / m_outputStrides[i + 1], patchOffsets[1] / m_outputStrides[i + 1]};
        patchOffsets[0] -= offsetIdx[0] * m_outputStrides[i + 1];
        patchOffsets[1] -= offsetIdx[1] * m_outputStrides[i + 1];

        inputIndices[0] += (patchIdx[0] + offsetIdx[0]) * m_inputStrides[i];
        inputIndices[1] += (patchIdx[1] + offsetIdx[1]) * m_inputStrides[i];
      }
    }
    inputIndices[0] += (patchIndices[0] + patchOffsets[0]);
    inputIndices[1] += (patchIndices[1] + patchOffsets[1]);

    if (inputIndices[1] - inputIndices[0] == PacketSize - 1) {
      PacketReturnType rslt = m_impl.template packet<Unaligned>(inputIndices[0]);
      return rslt;
    } else {
      EIGEN_ALIGN_TO_BOUNDARY(internal::unpacket_traits<PacketReturnType>::alignment)
      CoeffReturnType values[PacketSize];
      values[0] = m_impl.coeff(inputIndices[0]);
      values[PacketSize - 1] = m_impl.coeff(inputIndices[1]);
      EIGEN_UNROLL_LOOP
      for (int i = 1; i < PacketSize - 1; ++i) {
        values[i] = coeff(index + i);
      }
      PacketReturnType rslt = internal::pload<PacketReturnType>(values);
      return rslt;
    }
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE internal::TensorBlockResourceRequirements getResourceRequirements() const {
    const size_t target_size = m_device.firstLevelCacheSize();
    // Every output coefficient is read once from the argument and stored
    // once. Pass the full cost explicitly rather than adding to skewed()'s
    // default load+store seed, which would double-count the baseline byte
    // traffic and halve the tile size.
    const TensorOpCost cost_per_coeff =
        m_impl.costPerCoeff(/*vectorized=*/false) + TensorOpCost(0, sizeof(CoeffReturnType), 0);
    return internal::TensorBlockResourceRequirements::withShapeAndSize<Scalar>(
        internal::TensorBlockShapeType::kSkewedInnerDims, target_size, cost_per_coeff);
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorBlock block(TensorBlockDesc& desc, TensorBlockScratch& scratch,
                                                          bool /*root_of_expr_ast*/ = false) const {
    constexpr bool is_col_major = static_cast<int>(Layout) == static_cast<int>(ColMajor);

    if (desc.size() == 0) {
      return TensorBlock(internal::TensorBlockKind::kView, nullptr, desc.dimensions());
    }

    typename TensorBlock::Storage block_storage = TensorBlock::prepareStorage(desc, scratch);
    CoeffReturnType* block_buffer = block_storage.data();

    // Output coordinates of the block's corner.
    array<Index, NumDims> coords;
    Index remaining = desc.offset();
    EIGEN_IF_CONSTEXPR (is_col_major) {
      for (int i = NumDims - 1; i > 0; --i) {
        coords[i] = remaining / m_outputStrides[i];
        remaining -= coords[i] * m_outputStrides[i];
      }
      coords[0] = remaining;
    } else {
      for (int i = 0; i < NumDims - 1; ++i) {
        coords[i] = remaining / m_outputStrides[i];
        remaining -= coords[i] * m_outputStrides[i];
      }
      coords[NumDims - 1] = remaining;
    }

    const int patch_dim = is_col_major ? NumDims - 1 : 0;
    const int inner_dim = is_col_major ? 0 : NumDims - 1;

    const Index num_patches_in_block = desc.dimension(patch_dim);
    // The input's inner-most stride is 1 by construction, so the inner run is
    // contiguous on both sides.
    const Index inner_size = desc.dimension(inner_dim);

    // The within-patch dimensions between the inner-most one and the patch
    // index, ordered inner-most to outer-most, plus the input offset the
    // block's corner contributes on every within-patch dimension.
    array<Index, NumDims> mid_sizes;
    array<Index, NumDims> mid_src_stride;
    array<Index, NumDims> mid_count;
    int num_mid = 0;
    Index src_corner = 0;
    for (int k = 0; k < NumDims - 1; ++k) {
      const int d = is_col_major ? k : NumDims - 1 - k;  // output dimension
      const int in_d = is_col_major ? d : d - 1;         // input-strides index
      src_corner += coords[d] * m_inputStrides[in_d];
      if (k > 0) {
        mid_sizes[num_mid] = desc.dimension(d);
        mid_src_stride[num_mid] = m_inputStrides[in_d];
        ++num_mid;
      }
    }

    // The loop nest below visits the block in exactly its memory order (the
    // storage returned by prepareStorage() is dense with the block's own
    // layout-order strides), so the destination is one running cursor.
    Index dst = 0;
    for (Index p = 0; p < num_patches_in_block; ++p) {
      // Input offset of this patch's first element.
      Index patch_index = coords[patch_dim] + p;
      Index src_patch = 0;
      EIGEN_IF_CONSTEXPR (is_col_major) {
        for (int i = NumDims - 2; i > 0; --i) {
          const Index idx = patch_index / m_patchStrides[i];
          patch_index -= idx * m_patchStrides[i];
          src_patch += idx * m_inputStrides[i];
        }
      } else {
        for (int i = 0; i < NumDims - 2; ++i) {
          const Index idx = patch_index / m_patchStrides[i];
          patch_index -= idx * m_patchStrides[i];
          src_patch += idx * m_inputStrides[i];
        }
      }
      src_patch += patch_index;

      Index src = src_patch + src_corner;
      for (int k = 0; k < num_mid; ++k) mid_count[k] = 0;
      for (;;) {
        for (Index j = 0; j < inner_size; ++j) {
          block_buffer[dst + j] = m_impl.coeff(src + j);
        }
        dst += inner_size;
        int k = 0;
        for (; k < num_mid; ++k) {
          if (++mid_count[k] < mid_sizes[k]) {
            src += mid_src_stride[k];
            break;
          }
          mid_count[k] = 0;
          src -= mid_src_stride[k] * (mid_sizes[k] - 1);
        }
        if (k == num_mid) break;
      }
    }
    eigen_assert(dst == desc.size());

    return block_storage.AsTensorMaterializedBlock();
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorOpCost costPerCoeff(bool vectorized) const {
    const double compute_cost = NumDims * (TensorOpCost::DivCost<Index>() + TensorOpCost::MulCost<Index>() +
                                           2 * TensorOpCost::AddCost<Index>());
    return m_impl.costPerCoeff(vectorized) + TensorOpCost(0, 0, compute_cost, vectorized, PacketSize);
  }

  EIGEN_DEVICE_FUNC EvaluatorPointerType data() const { return nullptr; }

 protected:
  Dimensions m_dimensions;
  array<Index, NumDims> m_outputStrides;
  array<Index, NumDims - 1> m_inputStrides;
  array<Index, NumDims - 1> m_patchStrides;

  TensorEvaluator<ArgType, Device> m_impl;
  const Device EIGEN_DEVICE_REF m_device;
};

}  // end namespace Eigen

#endif  // EIGEN_TENSOR_TENSOR_PATCH_H
