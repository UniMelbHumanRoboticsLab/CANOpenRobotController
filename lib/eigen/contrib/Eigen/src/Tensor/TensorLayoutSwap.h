// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2014 Benoit Steiner <benoit.steiner.goog@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_TENSOR_TENSOR_LAYOUT_SWAP_H
#define EIGEN_TENSOR_TENSOR_LAYOUT_SWAP_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

namespace internal {
template <typename XprType>
struct traits<TensorLayoutSwapOp<XprType> > : public traits<XprType> {
  typedef typename XprType::Scalar Scalar;
  typedef traits<XprType> XprTraits;
  typedef typename XprTraits::StorageKind StorageKind;
  typedef typename XprTraits::Index Index;
  static constexpr int NumDimensions = traits<XprType>::NumDimensions;
  static constexpr int Layout = (traits<XprType>::Layout == ColMajor) ? RowMajor : ColMajor;
  typedef typename XprTraits::PointerType PointerType;
};

template <typename XprType>
struct eval<TensorLayoutSwapOp<XprType>, Eigen::Dense> {
  typedef const TensorLayoutSwapOp<XprType>& type;
};

}  // end namespace internal

/**
 * \ingroup Tensor_Module
 *
 * \brief Swap the layout from col-major to row-major, or row-major
 * to col-major, and invert the order of the dimensions.
 *
 * Beware: the dimensions are reversed by this operation. If you want to
 * preserve the ordering of the dimensions, you need to combine this
 * operation with a shuffle.
 *
 * \example:
 * Tensor<float, 2, ColMajor> input(2, 4);
 * Tensor<float, 2, RowMajor> output = input.swap_layout();
 * eigen_assert(output.dimension(0) == 4);
 * eigen_assert(output.dimension(1) == 2);
 *
 * array<int, 2> shuffle(1, 0);
 * output = input.swap_layout().shuffle(shuffle);
 * eigen_assert(output.dimension(0) == 2);
 * eigen_assert(output.dimension(1) == 4);
 *
 */
template <typename XprType>
class TensorLayoutSwapOp : public TensorBase<TensorLayoutSwapOp<XprType>, WriteAccessors> {
 public:
  typedef TensorBase<TensorLayoutSwapOp<XprType>, WriteAccessors> Base;
  typedef typename Eigen::internal::traits<TensorLayoutSwapOp>::Scalar Scalar;
  typedef typename Eigen::NumTraits<Scalar>::Real RealScalar;
  typedef std::remove_const_t<typename XprType::CoeffReturnType> CoeffReturnType;
  typedef typename Eigen::internal::ref_selector<TensorLayoutSwapOp>::type Nested;
  typedef typename Eigen::internal::traits<TensorLayoutSwapOp>::StorageKind StorageKind;
  typedef typename Eigen::internal::traits<TensorLayoutSwapOp>::Index Index;

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorLayoutSwapOp(const XprType& expr) : m_xpr(expr) {}

  EIGEN_DEVICE_FUNC const internal::remove_all_t<typename XprType::Nested>& expression() const { return m_xpr; }

  EIGEN_INHERIT_ASSIGNMENT_OPERATORS(TensorLayoutSwapOp)
 protected:
  typename XprType::Nested m_xpr;
};

// Eval as rvalue
template <typename ArgType, typename Device>
struct TensorEvaluator<const TensorLayoutSwapOp<ArgType>, Device> {
  typedef TensorLayoutSwapOp<ArgType> XprType;
  typedef typename XprType::Index Index;
  static constexpr int NumDims = internal::array_size<typename TensorEvaluator<ArgType, Device>::Dimensions>::value;
  typedef DSizes<Index, NumDims> Dimensions;

  static constexpr int Layout =
      (TensorEvaluator<ArgType, Device>::Layout == static_cast<int>(ColMajor)) ? RowMajor : ColMajor;
  enum {
    IsAligned = TensorEvaluator<ArgType, Device>::IsAligned,
    PacketAccess = TensorEvaluator<ArgType, Device>::PacketAccess,
    // Layout swap is a no-op at the flat-memory level: serve blocks from the
    // argument's raw data pointer when it has one, and otherwise forward the
    // block request to the argument with reversed dimensions.
    BlockAccess =
        (TensorEvaluator<ArgType, Device>::RawAccess || TensorEvaluator<ArgType, Device>::BlockAccess) && NumDims > 0,
    PreferBlockAccess = TensorEvaluator<ArgType, Device>::PreferBlockAccess,
    CoordAccess = false,  // to be implemented
    RawAccess = TensorEvaluator<ArgType, Device>::RawAccess
  };

  // Blocks are forwarded to the argument only when it cannot hand out a flat
  // buffer directly (the raw fast path below is cheaper).
  static constexpr bool ForwardBlocksToArg =
      TensorEvaluator<ArgType, Device>::BlockAccess && !TensorEvaluator<ArgType, Device>::RawAccess;
  static constexpr int ArgLayout = TensorEvaluator<ArgType, Device>::Layout;

  typedef typename XprType::Scalar Scalar;
  typedef typename XprType::CoeffReturnType CoeffReturnType;
  typedef typename PacketType<CoeffReturnType, Device>::type PacketReturnType;
  typedef StorageMemory<CoeffReturnType, Device> Storage;
  typedef typename Storage::Type EvaluatorPointerType;

  typedef std::remove_const_t<Scalar> ScalarNoConst;

  //===- Tensor block evaluation strategy (see TensorBlock.h) -------------===//
  typedef internal::TensorBlockDescriptor<NumDims, Index> TensorBlockDesc;
  typedef internal::TensorBlockScratchAllocator<Device> TensorBlockScratch;
  typedef typename internal::TensorMaterializedBlock<ScalarNoConst, NumDims, Layout, Index> TensorBlock;
  typedef typename TensorEvaluator<ArgType, Device>::TensorBlock ArgTensorBlock;
  //===--------------------------------------------------------------------===//

  EIGEN_STRONG_INLINE TensorEvaluator(const XprType& op, const Device& device) : m_impl(op.expression(), device) {
    for (int i = 0; i < NumDims; ++i) {
      m_dimensions[i] = m_impl.dimensions()[NumDims - 1 - i];
    }
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const Dimensions& dimensions() const { return m_dimensions; }

  EIGEN_STRONG_INLINE bool evalSubExprsIfNeeded(EvaluatorPointerType data) { return m_impl.evalSubExprsIfNeeded(data); }
  EIGEN_STRONG_INLINE void cleanup() { m_impl.cleanup(); }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE CoeffReturnType coeff(Index index) const { return m_impl.coeff(index); }

  template <int LoadMode>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE PacketReturnType packet(Index index) const {
    return m_impl.template packet<LoadMode>(index);
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorOpCost costPerCoeff(bool vectorized) const {
    return m_impl.costPerCoeff(vectorized);
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE internal::TensorBlockResourceRequirements getResourceRequirements() const {
    return getResourceRequirementsImpl(std::integral_constant<bool, ForwardBlocksToArg>());
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorBlock block(TensorBlockDesc& desc, TensorBlockScratch& scratch,
                                                          bool root_of_expr_ast = false) const {
    return blockImpl(desc, scratch, root_of_expr_ast, std::integral_constant<bool, ForwardBlocksToArg>());
  }

  EIGEN_DEVICE_FUNC typename Storage::Type data() const { return constCast(m_impl.data()); }

  const TensorEvaluator<ArgType, Device>& impl() const { return m_impl; }

 protected:
  // Sizes or strides of this expression in the argument's index order.
  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE DSizes<Index, NumDims> reversed(const DSizes<Index, NumDims>& sizes) {
    DSizes<Index, NumDims> result;
    for (int i = 0; i < NumDims; ++i) result[i] = sizes[NumDims - 1 - i];
    return result;
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE internal::TensorBlockResourceRequirements getResourceRequirementsImpl(
      std::true_type /*forward_to_arg*/) const {
    return m_impl.getResourceRequirements();
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE internal::TensorBlockResourceRequirements getResourceRequirementsImpl(
      std::false_type /*forward_to_arg*/) const {
    return internal::TensorBlockResourceRequirements::any();
  }

  // The argument owns a flat buffer this expression is a plain view of.
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorBlock blockImpl(TensorBlockDesc& desc, TensorBlockScratch& scratch,
                                                              bool /*root_of_expr_ast*/,
                                                              std::false_type /*forward_to_arg*/) const {
    eigen_assert(m_impl.data() != nullptr);
    return TensorBlock::materialize(m_impl.data(), m_dimensions, desc, scratch);
  }

  // Forward the block request to the argument: reversing the descriptor's
  // dimensions maps this block exactly onto an argument block at the same
  // flat offset, and the swapped layout makes the two flat buffers identical.
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorBlock blockImpl(TensorBlockDesc& desc, TensorBlockScratch& scratch,
                                                              bool root_of_expr_ast,
                                                              std::true_type /*forward_to_arg*/) const {
    const DSizes<Index, NumDims> arg_dims = reversed(desc.dimensions());
    TensorBlockDesc arg_desc(desc.offset(), arg_dims);

    // A destination buffer describes flat memory, which the layout swap leaves
    // alone: reversing its strides alongside the dimensions hands the argument
    // the very same bytes. A strided destination carries no valid dense
    // expression, so it is only passed on at the root of the expression tree,
    // where the block is written once and never read back through expr().
    typedef typename TensorBlockDesc::DestinationBuffer DestinationBuffer;
    const bool strided_destination = desc.destination().kind() == DestinationBuffer::kStrided;
    if (desc.destination().kind() == DestinationBuffer::kContiguous || (strided_destination && root_of_expr_ast)) {
      arg_desc.template AddDestinationBuffer<ArgLayout>(desc.destination().template data<ScalarNoConst>(),
                                                        reversed(desc.destination().strides()));
    }

    ArgTensorBlock arg_block = m_impl.block(arg_desc, scratch, root_of_expr_ast);

    if (arg_block.data() != NULL) {
      // A materialized argument block already stores this block's values in
      // this block's flat order; re-wrap the buffer with reversed dimensions.
      const bool materialized_in_output = arg_block.kind() == internal::TensorBlockKind::kMaterializedInOutput;
      if (materialized_in_output) desc.DropDestinationBuffer();
      return TensorBlock(arg_block.kind(), arg_block.data(), desc.dimensions(),
                         /*valid_expr=*/!(materialized_in_output && strided_destination));
    }

    // A lazy argument block has no buffer to share: materialize it into this
    // block's storage, evaluating in the argument's (flat-identical) layout.
    // The storage strides carry whichever destination prepareStorage accepted.
    typedef internal::TensorBlockAssignment<ScalarNoConst, NumDims, typename ArgTensorBlock::XprType, Index>
        ArgBlockAssign;
    typename TensorBlock::Storage storage =
        TensorBlock::prepareStorage(desc, scratch, /*allow_strided_storage=*/root_of_expr_ast);
    ArgBlockAssign::Run(ArgBlockAssign::target(arg_dims, reversed(storage.strides()), storage.data()),
                        arg_block.expr());
    arg_block.cleanup();
    return storage.AsTensorMaterializedBlock();
  }

  TensorEvaluator<ArgType, Device> m_impl;
  Dimensions m_dimensions;
};

// Eval as lvalue
template <typename ArgType, typename Device>
struct TensorEvaluator<TensorLayoutSwapOp<ArgType>, Device>
    : public TensorEvaluator<const TensorLayoutSwapOp<ArgType>, Device> {
  typedef TensorEvaluator<const TensorLayoutSwapOp<ArgType>, Device> Base;
  typedef TensorLayoutSwapOp<ArgType> XprType;

  static constexpr int NumDims = Base::NumDims;
  static constexpr int Layout = Base::Layout;
  enum {
    IsAligned = TensorEvaluator<ArgType, Device>::IsAligned,
    PacketAccess = TensorEvaluator<ArgType, Device>::PacketAccess,
    // Writing a block only needs the argument's flat buffer: layout swap does
    // not touch flat memory, so the block is assigned straight into it. The
    // argument cannot be forwarded to as it is on the read side, because
    // TensorBlockAssignment takes the inner dimension from the block
    // expression's layout but the strides from the target.
    BlockAccess = TensorEvaluator<ArgType, Device>::RawAccess && NumDims > 0,
    PreferBlockAccess = TensorEvaluator<ArgType, Device>::PreferBlockAccess,
    CoordAccess = false  // to be implemented
  };

  typedef typename XprType::Index Index;
  typedef typename XprType::Scalar Scalar;
  typedef typename XprType::CoeffReturnType CoeffReturnType;
  typedef typename PacketType<CoeffReturnType, Device>::type PacketReturnType;

  //===- Tensor block evaluation strategy (see TensorBlock.h) -------------===//
  typedef typename Base::TensorBlockDesc TensorBlockDesc;
  //===--------------------------------------------------------------------===//

  EIGEN_STRONG_INLINE TensorEvaluator(const XprType& op, const Device& device) : Base(op, device) {}

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE CoeffReturnType& coeffRef(Index index) const {
    return this->m_impl.coeffRef(index);
  }
  template <int StoreMode>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void writePacket(Index index, const PacketReturnType& x) const {
    this->m_impl.template writePacket<StoreMode>(index, x);
  }

  template <typename TensorBlock>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void writeBlock(const TensorBlockDesc& desc, const TensorBlock& block) {
    eigen_assert(this->m_impl.data() != NULL);

    // Dense strides of the swapped dimensions in this layout are exactly the
    // argument's flat strides, so the block expression can be assigned
    // directly into the argument's buffer at the block's flat offset.
    typedef typename TensorBlock::XprType TensorBlockExpr;
    typedef internal::TensorBlockAssignment<Scalar, NumDims, TensorBlockExpr, Index> TensorBlockAssign;

    TensorBlockAssign::Run(TensorBlockAssign::target(desc.dimensions(), internal::strides<Layout>(this->dimensions()),
                                                     this->m_impl.data(), desc.offset()),
                           block.expr());
  }
};

}  // end namespace Eigen

#endif  // EIGEN_TENSOR_TENSOR_LAYOUT_SWAP_H
