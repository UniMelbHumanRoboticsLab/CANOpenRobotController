// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2015 Ke Yang <yangke@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_TENSOR_TENSOR_INFLATION_H
#define EIGEN_TENSOR_TENSOR_INFLATION_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

namespace internal {
template <typename Strides, typename XprType>
struct traits<TensorInflationOp<Strides, XprType>> : public traits<XprType> {
  typedef typename XprType::Scalar Scalar;
  typedef traits<XprType> XprTraits;
  typedef typename XprTraits::StorageKind StorageKind;
  typedef typename XprTraits::Index Index;
  static constexpr int NumDimensions = XprTraits::NumDimensions;
  static constexpr int Layout = XprTraits::Layout;
  typedef typename XprTraits::PointerType PointerType;
};

template <typename Strides, typename XprType>
struct eval<TensorInflationOp<Strides, XprType>, Eigen::Dense> {
  typedef const TensorInflationOp<Strides, XprType>& type;
};

}  // end namespace internal

/**
 * \ingroup Tensor_Module
 *
 * \brief Tensor inflation class.
 */
template <typename Strides, typename XprType>
class TensorInflationOp : public TensorBase<TensorInflationOp<Strides, XprType>, ReadOnlyAccessors> {
 public:
  typedef typename Eigen::internal::traits<TensorInflationOp>::Scalar Scalar;
  typedef typename Eigen::NumTraits<Scalar>::Real RealScalar;
  typedef typename XprType::CoeffReturnType CoeffReturnType;
  typedef typename Eigen::internal::ref_selector<TensorInflationOp>::type Nested;
  typedef typename Eigen::internal::traits<TensorInflationOp>::StorageKind StorageKind;
  typedef typename Eigen::internal::traits<TensorInflationOp>::Index Index;

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorInflationOp(const XprType& expr, const Strides& strides)
      : m_xpr(expr), m_strides(strides) {}

  EIGEN_DEVICE_FUNC const Strides& strides() const { return m_strides; }

  EIGEN_DEVICE_FUNC const internal::remove_all_t<typename XprType::Nested>& expression() const { return m_xpr; }

 protected:
  typename XprType::Nested m_xpr;
  const Strides m_strides;
};

// Eval as rvalue
template <typename Strides, typename ArgType, typename Device>
struct TensorEvaluator<const TensorInflationOp<Strides, ArgType>, Device> {
  typedef TensorInflationOp<Strides, ArgType> XprType;
  typedef typename XprType::Index Index;
  static constexpr int NumDims = internal::array_size<typename TensorEvaluator<ArgType, Device>::Dimensions>::value;
  typedef DSizes<Index, NumDims> Dimensions;
  typedef typename XprType::Scalar Scalar;
  typedef typename XprType::CoeffReturnType CoeffReturnType;
  typedef typename PacketType<CoeffReturnType, Device>::type PacketReturnType;
  static constexpr int PacketSize = PacketType<CoeffReturnType, Device>::size;
  typedef StorageMemory<CoeffReturnType, Device> Storage;
  typedef typename Storage::Type EvaluatorPointerType;

  static constexpr int Layout = TensorEvaluator<ArgType, Device>::Layout;
  enum {
    IsAligned = /*TensorEvaluator<ArgType, Device>::IsAligned*/ false,
    PacketAccess = TensorEvaluator<ArgType, Device>::PacketAccess,
    // block() reads the argument through coeff(), and under a ThreadPool the
    // tiled executor shares this evaluator across concurrent block tasks, so
    // the argument must be safe to read repeatedly and concurrently. Either bit
    // establishes that: BlockAccess is what a nullary functor that carries
    // state between calls clears (the built-in random generators are pure in
    // seed and index and keep it), and RawAccess means coeff() is a plain
    // buffer read. Requiring BlockAccess alone would needlessly exclude raw
    // arguments whose scalar is not arithmetic, such as complex tensors.
    BlockAccess =
        (TensorEvaluator<ArgType, Device>::BlockAccess || TensorEvaluator<ArgType, Device>::RawAccess) && NumDims > 0,
    // The coeff/packet path pays a div/mod walk plus a hole check per output
    // scalar; the block path is a zero-fill plus a sparse copy of the stride
    // lattice.
    PreferBlockAccess = true,
    CoordAccess = false,  // to be implemented
    RawAccess = false
  };

  //===- Tensor block evaluation strategy (see TensorBlock.h) -------------===//
  typedef internal::TensorBlockDescriptor<NumDims, Index> TensorBlockDesc;
  typedef internal::TensorBlockScratchAllocator<Device> TensorBlockScratch;
  typedef internal::TensorMaterializedBlock<CoeffReturnType, NumDims, Layout, Index> TensorBlock;
  //===--------------------------------------------------------------------===//

  EIGEN_STRONG_INLINE TensorEvaluator(const XprType& op, const Device& device)
      : m_impl(op.expression(), device), m_strides(op.strides()), m_device(device) {
    m_dimensions = m_impl.dimensions();
    // Expand each dimension to the inflated dimension.
    for (int i = 0; i < NumDims; ++i) {
      m_dimensions[i] = (m_dimensions[i] - 1) * op.strides()[i] + 1;
    }

    // Remember the strides for fast division.
    for (int i = 0; i < NumDims; ++i) {
      m_fastStrides[i] = internal::TensorIntDivisor<Index>(m_strides[i]);
    }

    const typename TensorEvaluator<ArgType, Device>::Dimensions& input_dims = m_impl.dimensions();
    EIGEN_IF_CONSTEXPR (static_cast<int>(Layout) == static_cast<int>(ColMajor)) {
      m_outputStrides[0] = 1;
      m_inputStrides[0] = 1;
      for (int i = 1; i < NumDims; ++i) {
        m_outputStrides[i] = m_outputStrides[i - 1] * m_dimensions[i - 1];
        m_inputStrides[i] = m_inputStrides[i - 1] * input_dims[i - 1];
      }
    } else {  // RowMajor
      m_outputStrides[NumDims - 1] = 1;
      m_inputStrides[NumDims - 1] = 1;
      for (int i = NumDims - 2; i >= 0; --i) {
        m_outputStrides[i] = m_outputStrides[i + 1] * m_dimensions[i + 1];
        m_inputStrides[i] = m_inputStrides[i + 1] * input_dims[i + 1];
      }
    }
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const Dimensions& dimensions() const { return m_dimensions; }

  EIGEN_STRONG_INLINE bool evalSubExprsIfNeeded(EvaluatorPointerType /*data*/) {
    m_impl.evalSubExprsIfNeeded(nullptr);
    return true;
  }
  EIGEN_STRONG_INLINE void cleanup() { m_impl.cleanup(); }

  // Computes the input index given the output index. Returns true if the output
  // index doesn't fall into a hole.
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE bool getInputIndex(Index index, Index* inputIndex) const {
    eigen_assert(index < dimensions().TotalSize());
    *inputIndex = 0;
    EIGEN_IF_CONSTEXPR (static_cast<int>(Layout) == static_cast<int>(ColMajor)) {
      EIGEN_UNROLL_LOOP
      for (int i = NumDims - 1; i > 0; --i) {
        const Index idx = index / m_outputStrides[i];
        if (idx != idx / m_fastStrides[i] * m_strides[i]) {
          return false;
        }
        *inputIndex += idx / m_strides[i] * m_inputStrides[i];
        index -= idx * m_outputStrides[i];
      }
      if (index != index / m_fastStrides[0] * m_strides[0]) {
        return false;
      }
      *inputIndex += index / m_strides[0];
      return true;
    } else {
      EIGEN_UNROLL_LOOP
      for (int i = 0; i < NumDims - 1; ++i) {
        const Index idx = index / m_outputStrides[i];
        if (idx != idx / m_fastStrides[i] * m_strides[i]) {
          return false;
        }
        *inputIndex += idx / m_strides[i] * m_inputStrides[i];
        index -= idx * m_outputStrides[i];
      }
      if (index != index / m_fastStrides[NumDims - 1] * m_strides[NumDims - 1]) {
        return false;
      }
      *inputIndex += index / m_strides[NumDims - 1];
    }
    return true;
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE CoeffReturnType coeff(Index index) const {
    Index inputIndex = 0;
    if (getInputIndex(index, &inputIndex)) {
      return m_impl.coeff(inputIndex);
    } else {
      return Scalar(0);
    }
  }

  // TODO(yangke): optimize this function so that we can detect and produce
  // all-zero packets
  template <int LoadMode>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE PacketReturnType packet(Index index) const {
    EIGEN_STATIC_ASSERT((PacketSize > 1), YOU_MADE_A_PROGRAMMING_MISTAKE)
    eigen_assert(index + PacketSize - 1 < dimensions().TotalSize());

    EIGEN_ALIGN_TO_BOUNDARY(internal::unpacket_traits<PacketReturnType>::alignment)
    std::remove_const_t<CoeffReturnType> values[PacketSize];
    EIGEN_UNROLL_LOOP
    for (int i = 0; i < PacketSize; ++i) {
      values[i] = coeff(index + i);
    }
    PacketReturnType rslt = internal::pload<PacketReturnType>(values);
    return rslt;
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE internal::TensorBlockResourceRequirements getResourceRequirements() const {
    const size_t target_size = m_device.lastLevelCacheSize();
    // One store per output coefficient for the zero fill, plus a
    // lattice-density-weighted argument read and overwrite store, so that
    // ThreadPool scheduling sees the true cost of expensive arguments.
    const double density = latticeDensity();
    const TensorOpCost cost_per_coeff =
        density * m_impl.costPerCoeff(/*vectorized=*/false) +
        TensorOpCost(/*bytes_loaded=*/0, /*bytes_stored=*/(1.0 + density) * sizeof(CoeffReturnType),
                     /*compute_cycles=*/0);
    // withShapeAndSize rather than skewed(), because skewed() seeds a load and
    // a store per coefficient that this model already accounts for.
    return internal::TensorBlockResourceRequirements::withShapeAndSize<Scalar>(
        internal::TensorBlockShapeType::kSkewedInnerDims, target_size, cost_per_coeff);
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorBlock block(TensorBlockDesc& desc, TensorBlockScratch& scratch,
                                                          bool /*root_of_expr_ast*/ = false) const {
    constexpr bool is_col_major = static_cast<int>(Layout) == static_cast<int>(ColMajor);

    // If one of the dimensions is zero, return empty block view.
    if (desc.size() == 0) {
      return TensorBlock(internal::TensorBlockKind::kView, nullptr, desc.dimensions());
    }

    // Everything outside the stride lattice is a hole, so zero-fill first and
    // then copy the covered input values onto the lattice.
    typename TensorBlock::Storage block_storage = TensorBlock::prepareStorage(desc, scratch);
    CoeffReturnType* block_buffer = block_storage.data();

    // Output coordinates of the block's corner.
    array<Index, NumDims> coords;
    extract_coordinates(desc.offset(), coords);

    // First lattice point inside the block and the lattice extent, per dim.
    const DSizes<Index, NumDims>& block_strides = block_storage.strides();
    array<Index, NumDims> lattice_count;
    Index dst_offset = 0;
    Index src_offset = 0;
    for (int i = 0; i < NumDims; ++i) {
      const Index stride = m_strides[i];
      const Index first_input = numext::div_ceil(coords[i], stride);
      const Index first = first_input * stride;         // output coordinate
      const Index end = coords[i] + desc.dimension(i);  // exclusive
      if (first >= end) {
        // No lattice point along this dimension: the block is all holes.
        Map<ArrayX<CoeffReturnType>>(block_buffer, desc.size()).setZero();
        return block_storage.AsTensorMaterializedBlock();
      }
      lattice_count[i] = numext::div_ceil(end - first, stride);
      dst_offset += (first - coords[i]) * block_strides[i];
      src_offset += first_input * m_inputStrides[i];
    }

    // The fill is pure waste when the lattice covers every coefficient, which
    // is the whole block for identity striding.
    bool dense = true;
    for (int i = 0; i < NumDims; ++i) dense = dense && (lattice_count[i] == desc.dimension(i));
    if (!dense) Map<ArrayX<CoeffReturnType>>(block_buffer, desc.size()).setZero();

    // Iterate the lattice (dimensions ordered inner-most to outer-most).
    array<BlockIteratorState, NumDims> it;
    for (int i = 0; i < NumDims; ++i) {
      const int dim = is_col_major ? i : NumDims - 1 - i;
      const Index size = lattice_count[dim];
      const Index dst_stride = block_strides[dim] * m_strides[dim];
      const Index src_stride = m_inputStrides[dim];
      it[i] = {/*size=*/size,
               /*count=*/0,
               /*dst_stride=*/dst_stride,
               /*dst_span=*/dst_stride * (size - 1),
               /*src_stride=*/src_stride,
               /*src_span=*/src_stride * (size - 1)};
    }

    const Index inner_size = it[0].size;
    const Index inner_dst_stride = it[0].dst_stride;
    // m_inputStrides is 1 on the inner dimension in both layouts, so the
    // argument is read as a contiguous run.
    eigen_assert(it[0].src_stride == 1);
    Index dst = dst_offset;
    Index src = src_offset;
    while (it[NumDims - 1].count < it[NumDims - 1].size) {
      for (Index j = 0; j < inner_size; ++j) {
        block_buffer[dst + j * inner_dst_stride] = m_impl.coeff(src + j);
      }

      EIGEN_IF_CONSTEXPR (NumDims == 1) break;

      for (int i = 1; i < NumDims; ++i) {
        if (++it[i].count < it[i].size) {
          dst += it[i].dst_stride;
          src += it[i].src_stride;
          break;
        }
        if (i != NumDims - 1) it[i].count = 0;
        dst -= it[i].dst_span;
        src -= it[i].src_span;
      }
    }

    return block_storage.AsTensorMaterializedBlock();
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorOpCost costPerCoeff(bool vectorized) const {
    const double compute_cost = NumDims * (3 * TensorOpCost::DivCost<Index>() + 3 * TensorOpCost::MulCost<Index>() +
                                           2 * TensorOpCost::AddCost<Index>());
    if (m_dimensions.TotalSize() == 0) return TensorOpCost();
    return m_impl.costPerCoeff(vectorized) +
           TensorOpCost(sizeof(CoeffReturnType) * latticeDensity(), 0, compute_cost, vectorized, PacketSize);
  }

  EIGEN_DEVICE_FUNC EvaluatorPointerType data() const { return nullptr; }

 protected:
  // Fraction of output coefficients that fall on the stride lattice.
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE double latticeDensity() const {
    const double output_size = static_cast<double>(m_dimensions.TotalSize());
    if (output_size == 0) return 0.0;
    return static_cast<double>(m_impl.dimensions().TotalSize()) / output_size;
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void extract_coordinates(Index index, array<Index, NumDims>& coords) const {
    EIGEN_IF_CONSTEXPR (static_cast<int>(Layout) == static_cast<int>(ColMajor)) {
      for (int i = NumDims - 1; i > 0; --i) {
        const Index idx = index / m_outputStrides[i];
        index -= idx * m_outputStrides[i];
        coords[i] = idx;
      }
      coords[0] = index;
    } else {
      for (int i = 0; i < NumDims - 1; ++i) {
        const Index idx = index / m_outputStrides[i];
        index -= idx * m_outputStrides[i];
        coords[i] = idx;
      }
      coords[NumDims - 1] = index;
    }
  }

  Dimensions m_dimensions;
  array<Index, NumDims> m_outputStrides;
  array<Index, NumDims> m_inputStrides;
  TensorEvaluator<ArgType, Device> m_impl;
  const Strides m_strides;
  array<internal::TensorIntDivisor<Index>, NumDims> m_fastStrides;
  const Device EIGEN_DEVICE_REF m_device;

 private:
  struct BlockIteratorState {
    Index size;
    Index count;
    Index dst_stride;
    Index dst_span;
    Index src_stride;
    Index src_span;
  };
};

}  // end namespace Eigen

#endif  // EIGEN_TENSOR_TENSOR_INFLATION_H
