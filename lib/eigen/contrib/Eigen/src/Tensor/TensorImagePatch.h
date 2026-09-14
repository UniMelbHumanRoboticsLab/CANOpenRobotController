// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2014 Benoit Steiner <benoit.steiner.goog@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_TENSOR_TENSOR_IMAGE_PATCH_H
#define EIGEN_TENSOR_TENSOR_IMAGE_PATCH_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

namespace internal {

template <DenseIndex Rows, DenseIndex Cols, typename XprType>
struct traits<TensorImagePatchOp<Rows, Cols, XprType>> : traits<XprType> {
  typedef std::remove_const_t<typename XprType::Scalar> Scalar;
  typedef traits<XprType> XprTraits;
  typedef typename XprTraits::StorageKind StorageKind;
  typedef typename XprTraits::Index Index;
  static constexpr int NumDimensions = XprTraits::NumDimensions + 1;
  static constexpr int Layout = XprTraits::Layout;
  typedef typename XprTraits::PointerType PointerType;
};

template <DenseIndex Rows, DenseIndex Cols, typename XprType>
struct eval<TensorImagePatchOp<Rows, Cols, XprType>, Eigen::Dense> {
  typedef const TensorImagePatchOp<Rows, Cols, XprType>& type;
};

}  // end namespace internal

/**
 * \ingroup Tensor_Module
 *
 * \brief Patch extraction specialized for image processing.
 * This assumes that the input has at least 3 dimensions ordered as follows:
 *  1st dimension: channels (of size d)
 *  2nd dimension: rows (of size r)
 *  3rd dimension: columns (of size c)
 *  There can be additional dimensions such as time (for video) or batch (for
 * bulk processing) after the first 3.
 * Calling the image patch code with patch_rows and patch_cols is equivalent
 * to calling the regular patch extraction code with parameters d, patch_rows,
 * patch_cols, and 1 for all the additional dimensions.
 */
template <DenseIndex Rows, DenseIndex Cols, typename XprType>
class TensorImagePatchOp : public TensorBase<TensorImagePatchOp<Rows, Cols, XprType>, ReadOnlyAccessors> {
 public:
  typedef typename Eigen::internal::traits<TensorImagePatchOp>::Scalar Scalar;
  typedef typename Eigen::NumTraits<Scalar>::Real RealScalar;
  typedef typename XprType::CoeffReturnType CoeffReturnType;
  typedef typename Eigen::internal::ref_selector<TensorImagePatchOp>::type Nested;
  typedef typename Eigen::internal::traits<TensorImagePatchOp>::StorageKind StorageKind;
  typedef typename Eigen::internal::traits<TensorImagePatchOp>::Index Index;

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorImagePatchOp(const XprType& expr, DenseIndex patch_rows,
                                                           DenseIndex patch_cols, DenseIndex row_strides,
                                                           DenseIndex col_strides, DenseIndex in_row_strides,
                                                           DenseIndex in_col_strides, DenseIndex row_inflate_strides,
                                                           DenseIndex col_inflate_strides, PaddingType padding_type,
                                                           Scalar padding_value)
      : m_xpr(expr),
        m_patch_rows(patch_rows),
        m_patch_cols(patch_cols),
        m_row_strides(row_strides),
        m_col_strides(col_strides),
        m_in_row_strides(in_row_strides),
        m_in_col_strides(in_col_strides),
        m_row_inflate_strides(row_inflate_strides),
        m_col_inflate_strides(col_inflate_strides),
        m_padding_explicit(false),
        m_padding_top(0),
        m_padding_bottom(0),
        m_padding_left(0),
        m_padding_right(0),
        m_padding_type(padding_type),
        m_padding_value(padding_value) {}

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorImagePatchOp(const XprType& expr, DenseIndex patch_rows,
                                                           DenseIndex patch_cols, DenseIndex row_strides,
                                                           DenseIndex col_strides, DenseIndex in_row_strides,
                                                           DenseIndex in_col_strides, DenseIndex row_inflate_strides,
                                                           DenseIndex col_inflate_strides, DenseIndex padding_top,
                                                           DenseIndex padding_bottom, DenseIndex padding_left,
                                                           DenseIndex padding_right, Scalar padding_value)
      : m_xpr(expr),
        m_patch_rows(patch_rows),
        m_patch_cols(patch_cols),
        m_row_strides(row_strides),
        m_col_strides(col_strides),
        m_in_row_strides(in_row_strides),
        m_in_col_strides(in_col_strides),
        m_row_inflate_strides(row_inflate_strides),
        m_col_inflate_strides(col_inflate_strides),
        m_padding_explicit(true),
        m_padding_top(padding_top),
        m_padding_bottom(padding_bottom),
        m_padding_left(padding_left),
        m_padding_right(padding_right),
        m_padding_type(PADDING_VALID),
        m_padding_value(padding_value) {}

  EIGEN_DEVICE_FUNC DenseIndex patch_rows() const { return m_patch_rows; }
  EIGEN_DEVICE_FUNC DenseIndex patch_cols() const { return m_patch_cols; }
  EIGEN_DEVICE_FUNC DenseIndex row_strides() const { return m_row_strides; }
  EIGEN_DEVICE_FUNC DenseIndex col_strides() const { return m_col_strides; }
  EIGEN_DEVICE_FUNC DenseIndex in_row_strides() const { return m_in_row_strides; }
  EIGEN_DEVICE_FUNC DenseIndex in_col_strides() const { return m_in_col_strides; }
  EIGEN_DEVICE_FUNC DenseIndex row_inflate_strides() const { return m_row_inflate_strides; }
  EIGEN_DEVICE_FUNC DenseIndex col_inflate_strides() const { return m_col_inflate_strides; }
  EIGEN_DEVICE_FUNC bool padding_explicit() const { return m_padding_explicit; }
  EIGEN_DEVICE_FUNC DenseIndex padding_top() const { return m_padding_top; }
  EIGEN_DEVICE_FUNC DenseIndex padding_bottom() const { return m_padding_bottom; }
  EIGEN_DEVICE_FUNC DenseIndex padding_left() const { return m_padding_left; }
  EIGEN_DEVICE_FUNC DenseIndex padding_right() const { return m_padding_right; }
  EIGEN_DEVICE_FUNC PaddingType padding_type() const { return m_padding_type; }
  EIGEN_DEVICE_FUNC Scalar padding_value() const { return m_padding_value; }

  EIGEN_DEVICE_FUNC const internal::remove_all_t<typename XprType::Nested>& expression() const { return m_xpr; }

 protected:
  typename XprType::Nested m_xpr;
  const DenseIndex m_patch_rows;
  const DenseIndex m_patch_cols;
  const DenseIndex m_row_strides;
  const DenseIndex m_col_strides;
  const DenseIndex m_in_row_strides;
  const DenseIndex m_in_col_strides;
  const DenseIndex m_row_inflate_strides;
  const DenseIndex m_col_inflate_strides;
  const bool m_padding_explicit;
  const DenseIndex m_padding_top;
  const DenseIndex m_padding_bottom;
  const DenseIndex m_padding_left;
  const DenseIndex m_padding_right;
  const PaddingType m_padding_type;
  const Scalar m_padding_value;
};

// Eval as rvalue
template <DenseIndex Rows, DenseIndex Cols, typename ArgType, typename Device>
struct TensorEvaluator<const TensorImagePatchOp<Rows, Cols, ArgType>, Device> {
  typedef TensorImagePatchOp<Rows, Cols, ArgType> XprType;
  typedef typename XprType::Index Index;
  static constexpr int NumInputDims =
      internal::array_size<typename TensorEvaluator<ArgType, Device>::Dimensions>::value;
  static constexpr int NumDims = NumInputDims + 1;
  typedef DSizes<Index, NumDims> Dimensions;
  typedef std::remove_const_t<typename XprType::Scalar> Scalar;
  typedef TensorEvaluator<const TensorImagePatchOp<Rows, Cols, ArgType>, Device> Self;
  typedef TensorEvaluator<ArgType, Device> Impl;
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
    BlockAccess = true,
    PreferBlockAccess = true,
    CoordAccess = false,
    RawAccess = false
  };

  //===- Tensor block evaluation strategy (see TensorBlock.h) -------------===//
  typedef internal::TensorBlockDescriptor<NumDims, Index> TensorBlockDesc;
  typedef internal::TensorBlockScratchAllocator<Device> TensorBlockScratch;
  typedef typename internal::TensorMaterializedBlock<Scalar, NumDims, Layout, Index> TensorBlock;
  //===--------------------------------------------------------------------===//

  EIGEN_STRONG_INLINE TensorEvaluator(const XprType& op, const Device& device)
      : m_device(device), m_impl(op.expression(), device) {
    EIGEN_STATIC_ASSERT((NumDims >= 4), YOU_MADE_A_PROGRAMMING_MISTAKE);

    m_paddingValue = op.padding_value();

    const typename TensorEvaluator<ArgType, Device>::Dimensions& input_dims = m_impl.dimensions();

    // Caches a few variables.
    EIGEN_IF_CONSTEXPR (static_cast<int>(Layout) == static_cast<int>(ColMajor)) {
      m_inputDepth = input_dims[0];
      m_inputRows = input_dims[1];
      m_inputCols = input_dims[2];
    } else {
      m_inputDepth = input_dims[NumInputDims - 1];
      m_inputRows = input_dims[NumInputDims - 2];
      m_inputCols = input_dims[NumInputDims - 3];
    }

    m_row_strides = op.row_strides();
    m_col_strides = op.col_strides();

    // Input strides and effective input/patch size
    m_in_row_strides = op.in_row_strides();
    m_in_col_strides = op.in_col_strides();
    m_row_inflate_strides = op.row_inflate_strides();
    m_col_inflate_strides = op.col_inflate_strides();
    // The "effective" input rows and input cols are the input rows and cols
    // after inflating them with zeros.
    // For example, a 2x3 matrix with row_inflate_strides and
    // col_inflate_strides of 2 comes from:
    //   A B C
    //   D E F
    //
    // to a matrix is 3 x 5:
    //
    //   A . B . C
    //   . . . . .
    //   D . E . F

    m_input_rows_eff = (m_inputRows - 1) * m_row_inflate_strides + 1;
    m_input_cols_eff = (m_inputCols - 1) * m_col_inflate_strides + 1;
    m_patch_rows_eff = op.patch_rows() + (op.patch_rows() - 1) * (m_in_row_strides - 1);
    m_patch_cols_eff = op.patch_cols() + (op.patch_cols() - 1) * (m_in_col_strides - 1);

    if (op.padding_explicit()) {
      m_outputRows = numext::ceil((m_input_rows_eff + op.padding_top() + op.padding_bottom() - m_patch_rows_eff + 1.f) /
                                  static_cast<float>(m_row_strides));
      m_outputCols = numext::ceil((m_input_cols_eff + op.padding_left() + op.padding_right() - m_patch_cols_eff + 1.f) /
                                  static_cast<float>(m_col_strides));
      m_rowPaddingTop = op.padding_top();
      m_colPaddingLeft = op.padding_left();
    } else {
      // Computing padding from the type
      switch (op.padding_type()) {
        case PADDING_VALID:
          m_outputRows = numext::ceil((m_input_rows_eff - m_patch_rows_eff + 1.f) / static_cast<float>(m_row_strides));
          m_outputCols = numext::ceil((m_input_cols_eff - m_patch_cols_eff + 1.f) / static_cast<float>(m_col_strides));
          // Calculate the padding
          m_rowPaddingTop =
              numext::maxi<Index>(0, ((m_outputRows - 1) * m_row_strides + m_patch_rows_eff - m_input_rows_eff) / 2);
          m_colPaddingLeft =
              numext::maxi<Index>(0, ((m_outputCols - 1) * m_col_strides + m_patch_cols_eff - m_input_cols_eff) / 2);
          break;
        case PADDING_SAME:
          m_outputRows = numext::ceil(m_input_rows_eff / static_cast<float>(m_row_strides));
          m_outputCols = numext::ceil(m_input_cols_eff / static_cast<float>(m_col_strides));
          // Calculate the padding
          m_rowPaddingTop = ((m_outputRows - 1) * m_row_strides + m_patch_rows_eff - m_input_rows_eff) / 2;
          m_colPaddingLeft = ((m_outputCols - 1) * m_col_strides + m_patch_cols_eff - m_input_cols_eff) / 2;
          // The padding size calculation for PADDING_SAME has been updated to
          // be consistent with how TensorFlow extracts its paddings.
          m_rowPaddingTop = numext::maxi<Index>(0, m_rowPaddingTop);
          m_colPaddingLeft = numext::maxi<Index>(0, m_colPaddingLeft);
          break;
        default:
          eigen_assert(false && "unexpected padding");
          m_outputCols = 0;  // Silence the uninitialized warning.
          m_outputRows = 0;  // Silence the uninitialized warning.
      }
    }
    eigen_assert(m_outputRows > 0);
    eigen_assert(m_outputCols > 0);

    // Dimensions for result of extraction.
    EIGEN_IF_CONSTEXPR (static_cast<int>(Layout) == static_cast<int>(ColMajor)) {
      // ColMajor
      // 0: depth
      // 1: patch_rows
      // 2: patch_cols
      // 3: number of patches
      // 4 and beyond: anything else (such as batch).
      m_dimensions[0] = input_dims[0];
      m_dimensions[1] = op.patch_rows();
      m_dimensions[2] = op.patch_cols();
      m_dimensions[3] = m_outputRows * m_outputCols;
      for (int i = 4; i < NumDims; ++i) {
        m_dimensions[i] = input_dims[i - 1];
      }
    } else {
      // RowMajor
      // NumDims-1: depth
      // NumDims-2: patch_rows
      // NumDims-3: patch_cols
      // NumDims-4: number of patches
      // NumDims-5 and beyond: anything else (such as batch).
      m_dimensions[NumDims - 1] = input_dims[NumInputDims - 1];
      m_dimensions[NumDims - 2] = op.patch_rows();
      m_dimensions[NumDims - 3] = op.patch_cols();
      m_dimensions[NumDims - 4] = m_outputRows * m_outputCols;
      for (int i = NumDims - 5; i >= 0; --i) {
        m_dimensions[i] = input_dims[i];
      }
    }

    // Strides for moving the patch in various dimensions.
    EIGEN_IF_CONSTEXPR (static_cast<int>(Layout) == static_cast<int>(ColMajor)) {
      m_colStride = m_dimensions[1];
      m_patchStride = m_colStride * m_dimensions[2] * m_dimensions[0];
      m_otherStride = m_patchStride * m_dimensions[3];
    } else {
      m_colStride = m_dimensions[NumDims - 2];
      m_patchStride = m_colStride * m_dimensions[NumDims - 3] * m_dimensions[NumDims - 1];
      m_otherStride = m_patchStride * m_dimensions[NumDims - 4];
    }

    // Strides for navigating through the input tensor.
    m_rowInputStride = m_inputDepth;
    m_colInputStride = m_inputDepth * m_inputRows;
    m_patchInputStride = m_inputDepth * m_inputRows * m_inputCols;

    // Fast representations of different variables.
    m_fastOtherStride = internal::TensorIntDivisor<Index>(m_otherStride);
    m_fastPatchStride = internal::TensorIntDivisor<Index>(m_patchStride);
    m_fastColStride = internal::TensorIntDivisor<Index>(m_colStride);
    m_fastInflateRowStride = internal::TensorIntDivisor<Index>(m_row_inflate_strides);
    m_fastInflateColStride = internal::TensorIntDivisor<Index>(m_col_inflate_strides);
    m_fastInputColsEff = internal::TensorIntDivisor<Index>(m_input_cols_eff);

    // Number of patches in the width dimension.
    m_fastOutputRows = internal::TensorIntDivisor<Index>(m_outputRows);
    EIGEN_IF_CONSTEXPR (static_cast<int>(Layout) == static_cast<int>(ColMajor)) {
      m_fastOutputDepth = internal::TensorIntDivisor<Index>(m_dimensions[0]);
    } else {
      m_fastOutputDepth = internal::TensorIntDivisor<Index>(m_dimensions[NumDims - 1]);
    }
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const Dimensions& dimensions() const { return m_dimensions; }

  EIGEN_STRONG_INLINE bool evalSubExprsIfNeeded(EvaluatorPointerType /*data*/) {
    m_impl.evalSubExprsIfNeeded(nullptr);
    return true;
  }

#ifdef EIGEN_USE_THREADS
  template <typename EvalSubExprsCallback>
  EIGEN_STRONG_INLINE void evalSubExprsIfNeededAsync(EvaluatorPointerType, EvalSubExprsCallback done) {
    m_impl.evalSubExprsIfNeededAsync(nullptr, [done](bool) { done(true); });
  }
#endif  // EIGEN_USE_THREADS

  EIGEN_STRONG_INLINE void cleanup() { m_impl.cleanup(); }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE CoeffReturnType coeff(Index index) const {
    // Other ways to index this element.
    Index otherIndex, patch2DIndex;
    EIGEN_IF_CONSTEXPR (NumDims == 4) {
      otherIndex = 0;
      patch2DIndex = index / m_fastPatchStride;
    } else {
      otherIndex = index / m_fastOtherStride;
      patch2DIndex = (index - otherIndex * m_otherStride) / m_fastPatchStride;
    }

    // Compute the remainder within the patch once, then derive both
    // patchOffset and depth from it without an extra division.
    constexpr int depth_index = static_cast<int>(Layout) == static_cast<int>(ColMajor) ? 0 : NumDims - 1;
    const Index patchRemainder = index - otherIndex * m_otherStride - patch2DIndex * m_patchStride;
    const Index patchOffset = patchRemainder / m_fastOutputDepth;
    const Index depth = patchRemainder - patchOffset * m_dimensions[depth_index];

    // Calculate col index in the input original tensor.
    const Index colIndex = patch2DIndex / m_fastOutputRows;
    const Index colOffset = patchOffset / m_fastColStride;
    const Index inputCol = colIndex * m_col_strides + colOffset * m_in_col_strides - m_colPaddingLeft;
    const Index origInputCol =
        (m_col_inflate_strides == 1) ? inputCol : ((inputCol >= 0) ? (inputCol / m_fastInflateColStride) : 0);
    if (inputCol < 0 || inputCol >= m_input_cols_eff ||
        ((m_col_inflate_strides != 1) && (inputCol != origInputCol * m_col_inflate_strides))) {
      return Scalar(m_paddingValue);
    }

    // Calculate row index in the original input tensor.
    const Index rowIndex = patch2DIndex - colIndex * m_outputRows;
    const Index rowOffset = patchOffset - colOffset * m_colStride;
    const Index inputRow = rowIndex * m_row_strides + rowOffset * m_in_row_strides - m_rowPaddingTop;
    const Index origInputRow =
        (m_row_inflate_strides == 1) ? inputRow : ((inputRow >= 0) ? (inputRow / m_fastInflateRowStride) : 0);
    if (inputRow < 0 || inputRow >= m_input_rows_eff ||
        ((m_row_inflate_strides != 1) && (inputRow != origInputRow * m_row_inflate_strides))) {
      return Scalar(m_paddingValue);
    }

    const Index inputIndex =
        depth + origInputRow * m_rowInputStride + origInputCol * m_colInputStride + otherIndex * m_patchInputStride;
    return m_impl.coeff(inputIndex);
  }

  template <int LoadMode>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE PacketReturnType packet(Index index) const {
    eigen_assert(index + PacketSize - 1 < dimensions().TotalSize());

    constexpr int depth_index = static_cast<int>(Layout) == static_cast<int>(ColMajor) ? 0 : NumDims - 1;
    const Index lastIdx = index + PacketSize - 1;

    // Decompose index into (otherIndex, patch2DIndex, patchRemainder).
    // Use multiply+compare instead of a second TensorIntDivisor division
    // to check whether the last packet element is in the same region.
    Index otherIndex, patch2DIndex, patchRemainder0, patchRemainder1;
    EIGEN_IF_CONSTEXPR (NumDims == 4) {
      otherIndex = 0;
      patch2DIndex = index / m_fastPatchStride;
      const Index patchBase = patch2DIndex * m_patchStride;
      if (lastIdx >= patchBase + m_patchStride) {
        return packetWithPossibleZero(index);
      }
      patchRemainder0 = index - patchBase;
      patchRemainder1 = lastIdx - patchBase;
    } else {
      otherIndex = index / m_fastOtherStride;
      const Index otherBase = otherIndex * m_otherStride;
      if (lastIdx >= otherBase + m_otherStride) {
        return packetWithPossibleZero(index);
      }
      const Index patchBase0 = index - otherBase;
      patch2DIndex = patchBase0 / m_fastPatchStride;
      const Index patchStart = patch2DIndex * m_patchStride;
      if (lastIdx - otherBase >= patchStart + m_patchStride) {
        return packetWithPossibleZero(index);
      }
      patchRemainder0 = patchBase0 - patchStart;
      patchRemainder1 = lastIdx - otherBase - patchStart;
    }

    // Compute patchOffset for the first element. Defer the second
    // division until we know we need it.
    const Index patchOffset0 = patchRemainder0 / m_fastOutputDepth;
    const Index colIndex = patch2DIndex / m_fastOutputRows;

    // If all packet elements share the same (row, col) within the patch,
    // the input data is contiguous regardless of dilation/inflation strides.
    // Check using multiply+compare instead of dividing patchRemainder1.
    const Index outputDepth = m_dimensions[depth_index];
    if (patchRemainder1 < (patchOffset0 + 1) * outputDepth) {
      const Index colOffset = patchOffset0 / m_fastColStride;
      const Index rowIndex = patch2DIndex - colIndex * m_outputRows;
      const Index rowOffset = patchOffset0 - colOffset * m_colStride;

      const Index inputCol = colIndex * m_col_strides + colOffset * m_in_col_strides - m_colPaddingLeft;
      const Index inputRow = rowIndex * m_row_strides + rowOffset * m_in_row_strides - m_rowPaddingTop;

      // Check col bounds and inflate alignment.
      if (inputCol < 0 || inputCol >= m_input_cols_eff) {
        return internal::pset1<PacketReturnType>(Scalar(m_paddingValue));
      }
      if (m_col_inflate_strides != 1) {
        const Index origCol = inputCol / m_fastInflateColStride;
        if (inputCol != origCol * m_col_inflate_strides) {
          return internal::pset1<PacketReturnType>(Scalar(m_paddingValue));
        }
      }

      // Check row bounds and inflate alignment.
      if (inputRow < 0 || inputRow >= m_input_rows_eff) {
        return internal::pset1<PacketReturnType>(Scalar(m_paddingValue));
      }
      if (m_row_inflate_strides != 1) {
        const Index origRow = inputRow / m_fastInflateRowStride;
        if (inputRow != origRow * m_row_inflate_strides) {
          return internal::pset1<PacketReturnType>(Scalar(m_paddingValue));
        }
      }

      // Compute original input coordinates.
      const Index origInputCol = (m_col_inflate_strides == 1) ? inputCol : inputCol / m_fastInflateColStride;
      const Index origInputRow = (m_row_inflate_strides == 1) ? inputRow : inputRow / m_fastInflateRowStride;

      const Index depth = patchRemainder0 - patchOffset0 * outputDepth;
      const Index inputIndex =
          depth + origInputRow * m_rowInputStride + origInputCol * m_colInputStride + otherIndex * m_patchInputStride;
      return m_impl.template packet<Unaligned>(inputIndex);
    }

    // For non-unit strides spanning multiple rows, fall back to scalar.
    if (m_in_row_strides != 1 || m_in_col_strides != 1 || m_row_inflate_strides != 1 || m_col_inflate_strides != 1) {
      return packetWithPossibleZero(index);
    }

    // Unit strides: try to serve the packet from contiguous input.
    // Now we need the second patchOffset.
    const Index patchOffset1 = patchRemainder1 / m_fastOutputDepth;
    const Index colOffset0 = patchOffset0 / m_fastColStride;

    // Check if both ends of the packet are in the same column using
    // multiply+compare instead of dividing patchOffset1.
    const Index colBound = (colOffset0 + 1) * m_colStride;
    const bool sameCol = (patchOffset1 < colBound);

    // Calculate col indices in the original input tensor.
    const Index inputCol0 = colIndex * m_col_strides + colOffset0 - m_colPaddingLeft;
    if (sameCol) {
      if (inputCol0 < 0 || inputCol0 >= m_inputCols) {
        return internal::pset1<PacketReturnType>(Scalar(m_paddingValue));
      }

      const Index rowIndex = patch2DIndex - colIndex * m_outputRows;
      const Index rowOffset0 = patchOffset0 - colOffset0 * m_colStride;
      const Index rowOffset1 = patchOffset1 - colOffset0 * m_colStride;
      eigen_assert(rowOffset0 <= rowOffset1);
      // Calculate row indices in the original input tensor.
      const Index inputRow0 = rowIndex * m_row_strides + rowOffset0 - m_rowPaddingTop;
      const Index inputRow1 = rowIndex * m_row_strides + rowOffset1 - m_rowPaddingTop;

      if (inputRow1 < 0 || inputRow0 >= m_inputRows) {
        return internal::pset1<PacketReturnType>(Scalar(m_paddingValue));
      }

      if (inputRow0 >= 0 && inputRow1 < m_inputRows) {
        // no padding
        const Index depth = patchRemainder0 - patchOffset0 * outputDepth;
        const Index inputIndex =
            depth + inputRow0 * m_rowInputStride + inputCol0 * m_colInputStride + otherIndex * m_patchInputStride;
        return m_impl.template packet<Unaligned>(inputIndex);
      }
    } else {
      // Packet spans two columns. Check if both columns are entirely
      // outside the valid range (all-padding).
      const Index colOffset1 = patchOffset1 / m_fastColStride;
      const Index inputCol1 = colIndex * m_col_strides + colOffset1 - m_colPaddingLeft;
      if (inputCol1 < 0 || inputCol0 >= m_inputCols) {
        return internal::pset1<PacketReturnType>(Scalar(m_paddingValue));
      }
    }

    return packetWithPossibleZero(index);
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE internal::TensorBlockResourceRequirements getResourceRequirements() const {
    const size_t target_size = m_device.firstLevelCacheSize();
    // In-bounds output coefficients read the argument once and every output
    // coefficient is stored once (padding runs make this a slight
    // over-estimate). Pass the full cost explicitly rather than adding to
    // skewed()'s default load+store seed, which would double-count the
    // baseline byte traffic and halve the tile size.
    const TensorOpCost cost_per_coeff = m_impl.costPerCoeff(/*vectorized=*/false) + TensorOpCost(0, sizeof(Scalar), 0);
    return internal::TensorBlockResourceRequirements::withShapeAndSize<Scalar>(
        internal::TensorBlockShapeType::kSkewedInnerDims, target_size, cost_per_coeff);
  }

  // Materializes the block by iterating patch/col/row coordinates and either
  // copying the (always input-contiguous) depth run or filling it with the
  // padding value. All per-coordinate index math and bounds checks are
  // amortized over a whole depth run instead of paid per coefficient.
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorBlock block(TensorBlockDesc& desc, TensorBlockScratch& scratch,
                                                          bool /*root_of_expr_ast*/ = false) const {
    constexpr bool is_col_major = static_cast<int>(Layout) == static_cast<int>(ColMajor);

    if (desc.size() == 0) {
      return TensorBlock(internal::TensorBlockKind::kView, nullptr, desc.dimensions());
    }

    typename TensorBlock::Storage block_storage = TensorBlock::prepareStorage(desc, scratch);
    Scalar* block_buffer = block_storage.data();

    // Output coordinates of the block's corner.
    const DSizes<Index, NumDims> output_strides = internal::strides<Layout>(m_dimensions);
    array<Index, NumDims> coords;
    Index remaining = desc.offset();
    EIGEN_IF_CONSTEXPR (is_col_major) {
      for (int i = NumDims - 1; i > 0; --i) {
        coords[i] = remaining / output_strides[i];
        remaining -= coords[i] * output_strides[i];
      }
      coords[0] = remaining;
    } else {
      for (int i = 0; i < NumDims - 1; ++i) {
        coords[i] = remaining / output_strides[i];
        remaining -= coords[i] * output_strides[i];
      }
      coords[NumDims - 1] = remaining;
    }

    // Output dimensions: depth, patch row/col offset, 2d patch index, rest.
    const int dd = is_col_major ? 0 : NumDims - 1;
    const int rd = is_col_major ? 1 : NumDims - 2;
    const int cd = is_col_major ? 2 : NumDims - 3;
    const int pd = is_col_major ? 3 : NumDims - 4;

    const Index depth_start = coords[dd];
    const Index depth_size = desc.dimension(dd);
    const Index row_start = coords[rd];
    const Index row_size = desc.dimension(rd);
    const Index col_start = coords[cd];
    const Index col_size = desc.dimension(cd);
    const Index patch_start = coords[pd];
    const Index patch_size = desc.dimension(pd);

    // Odometer over the remaining (batch etc.) dimensions, tracking the input
    // offset they contribute.
    array<Index, NumDims> other_sizes;
    array<Index, NumDims> other_src_stride;
    array<Index, NumDims> other_count;
    int num_other = 0;
    Index src_other = 0;
    {
      Index in_stride = m_patchInputStride;
      for (int k = 4; k < NumDims; ++k) {
        const int d = is_col_major ? k : NumDims - 1 - k;
        other_sizes[num_other] = desc.dimension(d);
        other_src_stride[num_other] = in_stride;
        other_count[num_other] = 0;
        src_other += coords[d] * in_stride;
        in_stride *= m_dimensions[d];
        ++num_other;
      }
    }

    typedef internal::StridedLinearBufferCopy<Scalar, Index> LinCopy;

    // The loop nest below visits the block in exactly its memory order (the
    // storage returned by prepareStorage() is dense with the block's own
    // layout-order strides), so the destination is one running cursor.
    Index dst = 0;
    for (;;) {
      for (Index p = 0; p < patch_size; ++p) {
        const Index patch2DIndex = patch_start + p;
        const Index colIndex = patch2DIndex / m_fastOutputRows;
        const Index rowIndex = patch2DIndex - colIndex * m_outputRows;

        for (Index c = 0; c < col_size; ++c) {
          const Index colOffset = col_start + c;
          const Index inputCol = colIndex * m_col_strides + colOffset * m_in_col_strides - m_colPaddingLeft;
          Index origInputCol = inputCol;
          bool col_valid = inputCol >= 0 && inputCol < m_input_cols_eff;
          if (col_valid && m_col_inflate_strides != 1) {
            origInputCol = inputCol / m_fastInflateColStride;
            col_valid = (inputCol == origInputCol * m_col_inflate_strides);
          }

          for (Index r = 0; r < row_size; ++r) {
            const Index rowOffset = row_start + r;
            bool valid = col_valid;
            Index origInputRow = 0;
            if (valid) {
              const Index inputRow = rowIndex * m_row_strides + rowOffset * m_in_row_strides - m_rowPaddingTop;
              valid = inputRow >= 0 && inputRow < m_input_rows_eff;
              if (valid) {
                origInputRow = inputRow;
                if (m_row_inflate_strides != 1) {
                  origInputRow = inputRow / m_fastInflateRowStride;
                  valid = (inputRow == origInputRow * m_row_inflate_strides);
                }
              }
            }
            if (valid) {
              const Index src =
                  depth_start + origInputRow * m_rowInputStride + origInputCol * m_colInputStride + src_other;
              for (Index d = 0; d < depth_size; ++d) {
                block_buffer[dst + d] = m_impl.coeff(src + d);
              }
            } else {
              LinCopy::template Run<LinCopy::Kind::FillLinear>(typename LinCopy::Dst(dst, 1, block_buffer),
                                                               typename LinCopy::Src(0, 0, &m_paddingValue),
                                                               depth_size);
            }
            dst += depth_size;
          }
        }
      }

      int k = 0;
      for (; k < num_other; ++k) {
        if (++other_count[k] < other_sizes[k]) {
          src_other += other_src_stride[k];
          break;
        }
        other_count[k] = 0;
        src_other -= other_src_stride[k] * (other_sizes[k] - 1);
      }
      if (k == num_other) break;
    }
    eigen_assert(dst == desc.size());

    return block_storage.AsTensorMaterializedBlock();
  }

  EIGEN_DEVICE_FUNC EvaluatorPointerType data() const { return nullptr; }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const TensorEvaluator<ArgType, Device>& impl() const { return m_impl; }
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Index rowPaddingTop() const { return m_rowPaddingTop; }
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Index colPaddingLeft() const { return m_colPaddingLeft; }
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Index outputRows() const { return m_outputRows; }
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Index outputCols() const { return m_outputCols; }
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Index userRowStride() const { return m_row_strides; }
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Index userColStride() const { return m_col_strides; }
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Index userInRowStride() const { return m_in_row_strides; }
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Index userInColStride() const { return m_in_col_strides; }
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Index rowInflateStride() const { return m_row_inflate_strides; }
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Index colInflateStride() const { return m_col_inflate_strides; }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorOpCost costPerCoeff(bool vectorized) const {
    // Estimate for the non-padding code path with CoordAccess=false:
    // 5 TensorIntDivisor divs (otherStride, patchStride, outputDepth, outputRows, colStride),
    // 12 muls (index arithmetic), 8 adds/subs (offsets, padding checks).
    const double compute_cost =
        5 * TensorOpCost::DivCost<Index>() + 12 * TensorOpCost::MulCost<Index>() + 8 * TensorOpCost::AddCost<Index>();
    return m_impl.costPerCoeff(vectorized) + TensorOpCost(0, 0, compute_cost, vectorized, PacketSize);
  }

 protected:
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE PacketReturnType packetWithPossibleZero(Index index) const {
    EIGEN_ALIGN_TO_BOUNDARY(internal::unpacket_traits<PacketReturnType>::alignment)
    std::remove_const_t<CoeffReturnType> values[PacketSize];
    EIGEN_UNROLL_LOOP
    for (int i = 0; i < PacketSize; ++i) {
      values[i] = coeff(index + i);
    }
    PacketReturnType rslt = internal::pload<PacketReturnType>(values);
    return rslt;
  }

  Dimensions m_dimensions;

  Index m_otherStride;
  Index m_patchStride;
  Index m_colStride;
  Index m_row_strides;
  Index m_col_strides;

  Index m_in_row_strides;
  Index m_in_col_strides;
  Index m_row_inflate_strides;
  Index m_col_inflate_strides;

  Index m_input_rows_eff;
  Index m_input_cols_eff;
  Index m_patch_rows_eff;
  Index m_patch_cols_eff;

  internal::TensorIntDivisor<Index> m_fastOtherStride;
  internal::TensorIntDivisor<Index> m_fastPatchStride;
  internal::TensorIntDivisor<Index> m_fastColStride;
  internal::TensorIntDivisor<Index> m_fastInflateRowStride;
  internal::TensorIntDivisor<Index> m_fastInflateColStride;
  internal::TensorIntDivisor<Index> m_fastInputColsEff;

  Index m_rowInputStride;
  Index m_colInputStride;
  Index m_patchInputStride;

  Index m_inputDepth;
  Index m_inputRows;
  Index m_inputCols;

  Index m_outputRows;
  Index m_outputCols;

  Index m_rowPaddingTop;
  Index m_colPaddingLeft;

  internal::TensorIntDivisor<Index> m_fastOutputRows;
  internal::TensorIntDivisor<Index> m_fastOutputDepth;

  Scalar m_paddingValue;

  const Device EIGEN_DEVICE_REF m_device;
  TensorEvaluator<ArgType, Device> m_impl;
};

}  // end namespace Eigen

#endif  // EIGEN_TENSOR_TENSOR_IMAGE_PATCH_H
