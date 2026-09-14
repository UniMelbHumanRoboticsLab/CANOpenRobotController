// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2014 Benoit Steiner <benoit.steiner.goog@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_TENSOR_TENSOR_EVALUATOR_H
#define EIGEN_TENSOR_TENSOR_EVALUATOR_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

namespace internal {

// Whether functor_traits declare the functor repeatable: pure calls that may
// be re-evaluated freely and invoked from concurrent threads. The primary
// functor_traits template defaults IsRepeatable to false, but specializations
// for custom functors routinely omit the member, so absence must also map to
// the conservative default.
template <typename Functor, typename = void>
struct tensor_functor_is_repeatable : std::false_type {};

template <typename Functor>
struct tensor_functor_is_repeatable<Functor, void_t<decltype(functor_traits<Functor>::IsRepeatable)>>
    : bool_constant<functor_traits<Functor>::IsRepeatable> {};

}  // namespace internal

// Generic evaluator
/**
 * \ingroup Tensor_Module
 *
 * \brief The tensor evaluator class.
 *
 * These classes are responsible for the evaluation of the tensor expression.
 *
 * TODO: add support for more types of expressions, in particular expressions
 * leading to lvalues (slicing, reshaping, etc...)
 */
template <typename Derived, typename Device>
struct TensorEvaluator {
  typedef typename Derived::Index Index;
  typedef typename Derived::Scalar Scalar;
  typedef typename Derived::Scalar CoeffReturnType;
  typedef typename PacketType<CoeffReturnType, Device>::type PacketReturnType;
  typedef typename Derived::Dimensions Dimensions;
  typedef Derived XprType;
  static constexpr int PacketSize = PacketType<CoeffReturnType, Device>::size;
  typedef typename internal::traits<Derived>::template MakePointer<Scalar>::Type TensorPointerType;
  typedef StorageMemory<Scalar, Device> Storage;
  typedef typename Storage::Type EvaluatorPointerType;

  // NumDimensions is -1 for variable dim tensors
  static constexpr int NumCoords =
      internal::traits<Derived>::NumDimensions > 0 ? internal::traits<Derived>::NumDimensions : 0;
  static constexpr int Layout = Derived::Layout;

  enum {
    IsAligned = Derived::IsAligned,
    PacketAccess = (PacketType<CoeffReturnType, Device>::size > 1),
    BlockAccess = internal::is_arithmetic<std::remove_const_t<Scalar>>::value,
    PreferBlockAccess = false,
    CoordAccess = NumCoords > 0,
    RawAccess = true
  };

  typedef std::remove_const_t<Scalar> ScalarNoConst;

  //===- Tensor block evaluation strategy (see TensorBlock.h) -------------===//
  typedef internal::TensorBlockDescriptor<NumCoords, Index> TensorBlockDesc;
  typedef internal::TensorBlockScratchAllocator<Device> TensorBlockScratch;

  typedef typename internal::TensorMaterializedBlock<ScalarNoConst, NumCoords, Layout, Index> TensorBlock;
  //===--------------------------------------------------------------------===//

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorEvaluator(const Derived& m, const Device& device)
      : m_data(device.get(const_cast<TensorPointerType>(m.data()))), m_dims(m.dimensions()), m_device(device) {}

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const Dimensions& dimensions() const { return m_dims; }

  EIGEN_STRONG_INLINE bool evalSubExprsIfNeeded(EvaluatorPointerType dest) {
    EIGEN_IF_CONSTEXPR (!NumTraits<std::remove_const_t<Scalar>>::RequireInitialization) {
      if (dest) {
        m_device.memcpy((void*)(m_device.get(dest)), m_device.get(m_data), m_dims.TotalSize() * sizeof(Scalar));
        return false;
      }
    }
    return true;
  }

#ifdef EIGEN_USE_THREADS
  template <typename EvalSubExprsCallback>
  EIGEN_STRONG_INLINE void evalSubExprsIfNeededAsync(EvaluatorPointerType dest, EvalSubExprsCallback done) {
    // TODO(ezhulenev): ThreadPoolDevice memcpy is a blocking operation.
    done(evalSubExprsIfNeeded(dest));
  }
#endif  // EIGEN_USE_THREADS

  EIGEN_STRONG_INLINE void cleanup() {}

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE CoeffReturnType coeff(Index index) const {
    eigen_assert(m_data != nullptr);
    return m_data[index];
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE CoeffReturnType& coeffRef(Index index) const {
    eigen_assert(m_data != nullptr);
    return m_data[index];
  }

  template <int LoadMode>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE PacketReturnType packet(Index index) const {
    return internal::ploadt<PacketReturnType, LoadMode>(m_data + index);
  }

  // Return a packet starting at `index` where `umask` specifies which elements
  // have to be loaded. Type/size of mask depends on PacketReturnType, e.g. for
  // Packet16f, `umask` is of type uint16_t and if a bit is 1, corresponding
  // float element will be loaded, otherwise 0 will be loaded.
  // Function has been templatized to enable Sfinae.
  template <typename PacketReturnTypeT>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
      std::enable_if_t<internal::unpacket_traits<PacketReturnTypeT>::masked_load_available, PacketReturnTypeT>
      partialPacket(Index index, typename internal::unpacket_traits<PacketReturnTypeT>::mask_t umask) const {
    return internal::ploadu<PacketReturnTypeT>(m_data + index, umask);
  }

  template <int StoreMode>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void writePacket(Index index, const PacketReturnType& x) const {
    return internal::pstoret<Scalar, PacketReturnType, StoreMode>(m_data + index, x);
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE CoeffReturnType coeff(const array<DenseIndex, NumCoords>& coords) const {
    eigen_assert(m_data != nullptr);
    EIGEN_IF_CONSTEXPR (static_cast<int>(Layout) == static_cast<int>(ColMajor)) {
      return m_data[m_dims.IndexOfColMajor(coords)];
    } else {
      return m_data[m_dims.IndexOfRowMajor(coords)];
    }
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE CoeffReturnType& coeffRef(const array<DenseIndex, NumCoords>& coords) const {
    eigen_assert(m_data != nullptr);
    EIGEN_IF_CONSTEXPR (static_cast<int>(Layout) == static_cast<int>(ColMajor)) {
      return m_data[m_dims.IndexOfColMajor(coords)];
    } else {
      return m_data[m_dims.IndexOfRowMajor(coords)];
    }
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorOpCost costPerCoeff(bool vectorized) const {
    return TensorOpCost(sizeof(CoeffReturnType), 0, 0, vectorized, PacketType<CoeffReturnType, Device>::size);
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE internal::TensorBlockResourceRequirements getResourceRequirements() const {
    return internal::TensorBlockResourceRequirements::any();
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorBlock block(TensorBlockDesc& desc, TensorBlockScratch& scratch,
                                                          bool /*root_of_expr_ast*/ = false) const {
    eigen_assert(m_data != nullptr);
    return TensorBlock::materialize(m_data, m_dims, desc, scratch);
  }

  template <typename TensorBlock>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void writeBlock(const TensorBlockDesc& desc, const TensorBlock& block) {
    eigen_assert(m_data != nullptr);

    typedef typename TensorBlock::XprType TensorBlockExpr;
    typedef internal::TensorBlockAssignment<Scalar, NumCoords, TensorBlockExpr, Index> TensorBlockAssign;

    TensorBlockAssign::Run(
        TensorBlockAssign::target(desc.dimensions(), internal::strides<Layout>(m_dims), m_data, desc.offset()),
        block.expr());
  }

  EIGEN_DEVICE_FUNC EvaluatorPointerType data() const { return m_data; }

 protected:
  EvaluatorPointerType m_data;
  Dimensions m_dims;
  const Device EIGEN_DEVICE_REF m_device;
};

namespace internal {
template <typename T>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE T loadConstant(const T* address) {
  return *address;
}
// Use the texture cache on CUDA devices whenever possible
#if defined(EIGEN_CUDA_ARCH)
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE float loadConstant(const float* address) {
  return __ldg(address);
}
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE double loadConstant(const double* address) {
  return __ldg(address);
}
template <>
EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE Eigen::half loadConstant(const Eigen::half* address) {
  return Eigen::half(half_impl::raw_uint16_to_half(__ldg(&address->x)));
}
#endif

}  // namespace internal

// Default evaluator for rvalues
template <typename Derived, typename Device>
struct TensorEvaluator<const Derived, Device> {
  typedef typename Derived::Index Index;
  typedef typename Derived::Scalar Scalar;
  typedef typename Derived::Scalar CoeffReturnType;
  typedef typename PacketType<CoeffReturnType, Device>::type PacketReturnType;
  typedef typename Derived::Dimensions Dimensions;
  typedef const Derived XprType;
  typedef typename internal::traits<Derived>::template MakePointer<const Scalar>::Type TensorPointerType;
  typedef StorageMemory<const Scalar, Device> Storage;
  typedef typename Storage::Type EvaluatorPointerType;

  typedef std::remove_const_t<Scalar> ScalarNoConst;

  // NumDimensions is -1 for variable dim tensors
  static constexpr int NumCoords =
      internal::traits<Derived>::NumDimensions > 0 ? internal::traits<Derived>::NumDimensions : 0;
  static constexpr int PacketSize = PacketType<CoeffReturnType, Device>::size;
  static constexpr int Layout = Derived::Layout;

  enum {
    IsAligned = Derived::IsAligned,
    PacketAccess = (PacketType<CoeffReturnType, Device>::size > 1),
    BlockAccess = internal::is_arithmetic<ScalarNoConst>::value,
    PreferBlockAccess = false,
    CoordAccess = NumCoords > 0,
    RawAccess = true
  };

  //===- Tensor block evaluation strategy (see TensorBlock.h) -------------===//
  typedef internal::TensorBlockDescriptor<NumCoords, Index> TensorBlockDesc;
  typedef internal::TensorBlockScratchAllocator<Device> TensorBlockScratch;

  typedef typename internal::TensorMaterializedBlock<ScalarNoConst, NumCoords, Layout, Index> TensorBlock;
  //===--------------------------------------------------------------------===//

  EIGEN_STRONG_INLINE EIGEN_DEVICE_FUNC TensorEvaluator(const Derived& m, const Device& device)
      : m_data(device.get(m.data())), m_dims(m.dimensions()), m_device(device) {}

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const Dimensions& dimensions() const { return m_dims; }

  EIGEN_STRONG_INLINE bool evalSubExprsIfNeeded(EvaluatorPointerType data) {
    EIGEN_IF_CONSTEXPR (!NumTraits<std::remove_const_t<Scalar>>::RequireInitialization) {
      if (data) {
        m_device.memcpy((void*)(m_device.get(data)), m_device.get(m_data), m_dims.TotalSize() * sizeof(Scalar));
        return false;
      }
    }
    return true;
  }

#ifdef EIGEN_USE_THREADS
  template <typename EvalSubExprsCallback>
  EIGEN_STRONG_INLINE void evalSubExprsIfNeededAsync(EvaluatorPointerType dest, EvalSubExprsCallback done) {
    // TODO(ezhulenev): ThreadPoolDevice memcpy is a blocking operation.
    done(evalSubExprsIfNeeded(dest));
  }
#endif  // EIGEN_USE_THREADS

  EIGEN_STRONG_INLINE void cleanup() {}

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE CoeffReturnType coeff(Index index) const {
    eigen_assert(m_data != nullptr);
    return internal::loadConstant(m_data + index);
  }

  template <int LoadMode>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE PacketReturnType packet(Index index) const {
    return internal::ploadt_ro<PacketReturnType, LoadMode>(m_data + index);
  }

  // Return a packet starting at `index` where `umask` specifies which elements
  // have to be loaded. Type/size of mask depends on PacketReturnType, e.g. for
  // Packet16f, `umask` is of type uint16_t and if a bit is 1, corresponding
  // float element will be loaded, otherwise 0 will be loaded.
  // Function has been templatized to enable Sfinae.
  template <typename PacketReturnTypeT>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
      std::enable_if_t<internal::unpacket_traits<PacketReturnTypeT>::masked_load_available, PacketReturnTypeT>
      partialPacket(Index index, typename internal::unpacket_traits<PacketReturnTypeT>::mask_t umask) const {
    return internal::ploadu<PacketReturnTypeT>(m_data + index, umask);
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE CoeffReturnType coeff(const array<DenseIndex, NumCoords>& coords) const {
    eigen_assert(m_data != nullptr);
    const Index index = (static_cast<int>(Layout) == static_cast<int>(ColMajor)) ? m_dims.IndexOfColMajor(coords)
                                                                                 : m_dims.IndexOfRowMajor(coords);
    return internal::loadConstant(m_data + index);
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorOpCost costPerCoeff(bool vectorized) const {
    return TensorOpCost(sizeof(CoeffReturnType), 0, 0, vectorized, PacketType<CoeffReturnType, Device>::size);
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE internal::TensorBlockResourceRequirements getResourceRequirements() const {
    return internal::TensorBlockResourceRequirements::any();
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorBlock block(TensorBlockDesc& desc, TensorBlockScratch& scratch,
                                                          bool /*root_of_expr_ast*/ = false) const {
    eigen_assert(m_data != nullptr);
    return TensorBlock::materialize(m_data, m_dims, desc, scratch);
  }

  EIGEN_DEVICE_FUNC EvaluatorPointerType data() const { return m_data; }

 protected:
  EvaluatorPointerType m_data;
  Dimensions m_dims;
  const Device EIGEN_DEVICE_REF m_device;
};

// -------------------- CwiseNullaryOp --------------------

template <typename NullaryOp, typename ArgType, typename Device>
struct TensorEvaluator<const TensorCwiseNullaryOp<NullaryOp, ArgType>, Device> {
  typedef TensorCwiseNullaryOp<NullaryOp, ArgType> XprType;

  EIGEN_DEVICE_FUNC TensorEvaluator(const XprType& op, const Device& device)
      : m_functor(op.functor()), m_argImpl(op.nestedExpression(), device), m_wrapper() {}

  typedef typename XprType::Index Index;
  typedef typename XprType::Scalar Scalar;
  typedef typename internal::traits<XprType>::Scalar CoeffReturnType;
  typedef typename PacketType<CoeffReturnType, Device>::type PacketReturnType;
  static constexpr int PacketSize = PacketType<CoeffReturnType, Device>::size;
  typedef typename TensorEvaluator<ArgType, Device>::Dimensions Dimensions;
  typedef StorageMemory<CoeffReturnType, Device> Storage;
  typedef typename Storage::Type EvaluatorPointerType;
  static constexpr int NumDims = internal::array_size<Dimensions>::value;
  typedef std::remove_const_t<CoeffReturnType> ScalarNoConst;

  // Only functors whose functor_traits declare IsRepeatable (e.g. the
  // scalar_constant_op behind constant(), cwiseMax(Scalar) and clip(), and the
  // index-seeded random generators) are known to be pure; unannotated custom
  // functors conservatively default to non-repeatable, since a functor that
  // carries state between calls cannot be re-evaluated or visited out of order.
  static constexpr bool RepeatableFunctor = internal::tensor_functor_is_repeatable<NullaryOp>::value;

  // nullary_wrapper dispatches to an indexed operator() whenever one exists,
  // even if a zero-argument overload is also present.
  static constexpr bool IndexDependentFunctor =
      internal::has_unary_operator<NullaryOp, Index>::value || internal::has_binary_operator<NullaryOp, Index>::value;

  // A lazy block rebuilds the nullary expression over the block's local
  // extent with a copy of the functor, so it is only correct for repeatable
  // functors evaluated through the zero-argument overload: an index-dependent
  // functor would see indices restart at the block origin, and a
  // non-repeatable one would restart its state per block. Repeatable indexed
  // functors are instead materialized with their true tensor-linear indices.
  static constexpr bool IndexIndependentFunctor =
      RepeatableFunctor && !IndexDependentFunctor && internal::has_nullary_operator<NullaryOp, Index>::value;

  static constexpr int Layout = TensorEvaluator<ArgType, Device>::Layout;
  enum {
    IsAligned = true,
    PacketAccess = internal::functor_traits<NullaryOp>::PacketAccess
#ifdef EIGEN_USE_SYCL
                   && (PacketType<CoeffReturnType, Device>::size > 1)
#endif
        ,
    // A nullary leaf can serve any block; without this, a single constant()
    // in an expression disables tiled evaluation for the whole tree. Never
    // *prefer* block access on its own account, though. Blocks are declined
    // when they could change behavior relative to coefficient evaluation. In
    // particular, non-repeatable functors never serve blocks: even on a
    // single-threaded device, block traversal permutes their call sequence
    // relative to linear coefficient order.
    BlockAccess = NumDims > 0 && internal::is_arithmetic<ScalarNoConst>::value &&
                  (IndexIndependentFunctor || (IndexDependentFunctor && RepeatableFunctor)),
    PreferBlockAccess = false,
    CoordAccess = false,  // to be implemented
    RawAccess = false
  };

  //===- Tensor block evaluation strategy (see TensorBlock.h) -------------===//
  typedef internal::TensorBlockDescriptor<NumDims, Index> TensorBlockDesc;
  typedef internal::TensorBlockScratchAllocator<Device> TensorBlockScratch;

  // Lazy block for index-independent functors: rebuilds the nullary
  // expression over the block extent. The nested map exists only to give the
  // expression its dimensions; the nullary evaluator never touches its
  // argument's data.
  class TensorNullaryBlock {
   public:
    typedef TensorMap<const Tensor<ScalarNoConst, NumDims, Layout, Index>> ArgXprType;
    typedef TensorCwiseNullaryOp<NullaryOp, const ArgXprType> XprType;
    typedef typename internal::XprScalar<XprType>::type Scalar;

    TensorNullaryBlock(const NullaryOp& functor, const DSizes<Index, NumDims>& dimensions)
        : m_functor(functor), m_dimensions(dimensions) {}

    constexpr internal::TensorBlockKind kind() const { return internal::TensorBlockKind::kExpr; }
    XprType expr() const {
      return XprType(ArgXprType(static_cast<const ScalarNoConst*>(nullptr), m_dimensions), m_functor);
    }
    const Scalar* data() const { return nullptr; }
    void cleanup() {}

   private:
    NullaryOp m_functor;
    DSizes<Index, NumDims> m_dimensions;
  };

  typedef internal::TensorMaterializedBlock<ScalarNoConst, NumDims, Layout, Index> MaterializedTensorBlock;
  typedef std::conditional_t<IndexIndependentFunctor, TensorNullaryBlock, MaterializedTensorBlock> TensorBlock;
  //===--------------------------------------------------------------------===//

  EIGEN_DEVICE_FUNC const Dimensions& dimensions() const { return m_argImpl.dimensions(); }

  EIGEN_STRONG_INLINE bool evalSubExprsIfNeeded(EvaluatorPointerType) { return true; }

#ifdef EIGEN_USE_THREADS
  template <typename EvalSubExprsCallback>
  EIGEN_STRONG_INLINE void evalSubExprsIfNeededAsync(EvaluatorPointerType, EvalSubExprsCallback done) {
    done(true);
  }
#endif  // EIGEN_USE_THREADS

  EIGEN_STRONG_INLINE void cleanup() {}

  EIGEN_DEVICE_FUNC CoeffReturnType coeff(Index index) const { return m_wrapper(m_functor, index); }

  template <int LoadMode>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE PacketReturnType packet(Index index) const {
    return m_wrapper.template packetOp<PacketReturnType, Index>(m_functor, index);
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorOpCost costPerCoeff(bool vectorized) const {
    // NullaryOps (constants, zero, identity, random) generate values from
    // registers or minimal state — they do not load from memory.  Report
    // zero bytes_loaded so the cost model correctly classifies expressions
    // containing many constants (e.g. Horner polynomials) as compute-bound
    // rather than memory-bound.
    return TensorOpCost(0, 0, 0, vectorized, PacketType<CoeffReturnType, Device>::size);
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE internal::TensorBlockResourceRequirements getResourceRequirements() const {
    return internal::TensorBlockResourceRequirements::any();
  }

  // Fills `count` elements of an inner run, whose tensor-linear indices are
  // tensor_index .. tensor_index + count - 1. Specialized on the functor's
  // packet support so that packetOp is only instantiated when it exists.
  template <typename Self, bool Vectorizable>
  struct NullaryBlockFill {
    static EIGEN_STRONG_INLINE void Run(const Self& self, ScalarNoConst* buffer, Index tensor_index, Index count) {
      for (Index i = 0; i < count; ++i) {
        buffer[i] = self.m_wrapper(self.m_functor, tensor_index + i);
      }
    }
  };

  template <typename Self>
  struct NullaryBlockFill<Self, true> {
    static EIGEN_STRONG_INLINE void Run(const Self& self, ScalarNoConst* buffer, Index tensor_index, Index count) {
      Index i = 0;
      for (; i + PacketSize <= count; i += PacketSize) {
        internal::pstoreu(buffer + i,
                          self.m_wrapper.template packetOp<PacketReturnType, Index>(self.m_functor, tensor_index + i));
      }
      for (; i < count; ++i) {
        buffer[i] = self.m_wrapper(self.m_functor, tensor_index + i);
      }
    }
  };

  template <typename Self, bool IndexIndependent>
  struct NullaryBlockImpl {
    // Index-independent functors: return a lazy block; the consumer folds the
    // functor into its own (vectorized) evaluation loop.
    static EIGEN_STRONG_INLINE TensorBlock Run(const Self& self, TensorBlockDesc& desc, TensorBlockScratch&) {
      return TensorBlock(self.m_functor, desc.dimensions());
    }
  };

  template <typename Self>
  struct NullaryBlockImpl<Self, false> {
    // Index-dependent functors: materialize the block by calling the functor
    // with the block elements' true tensor-linear indices, so the values
    // match linear evaluation semantics. Along the inner dimension a block
    // row is contiguous in linear-index space, so each run is one
    // packet-sized sweep.
    static TensorBlock Run(const Self& self, TensorBlockDesc& desc, TensorBlockScratch& scratch) {
      constexpr bool is_col_major = static_cast<int>(Layout) == static_cast<int>(ColMajor);
      typedef NullaryBlockFill<Self, bool(PacketAccess)> Fill;

      if (desc.size() == 0) {
        return TensorBlock(internal::TensorBlockKind::kView, nullptr, desc.dimensions());
      }

      // Strides of the full tensor in linear-index space.
      const Dimensions& dims = self.m_argImpl.dimensions();
      array<Index, NumDims> tensor_strides;
      EIGEN_IF_CONSTEXPR (is_col_major) {
        tensor_strides[0] = 1;
        for (int i = 1; i < NumDims; ++i) tensor_strides[i] = tensor_strides[i - 1] * dims[i - 1];
      } else {
        tensor_strides[NumDims - 1] = 1;
        for (int i = NumDims - 2; i >= 0; --i) tensor_strides[i] = tensor_strides[i + 1] * dims[i + 1];
      }

      // Block iteration state, inner-most dimension first.
      struct BlockIteratorState {
        Index size;
        Index count;
        Index tensor_stride;
        Index tensor_span;
      };
      array<BlockIteratorState, NumDims> it;
      for (int i = 0; i < NumDims; ++i) {
        const int dim = is_col_major ? i : NumDims - 1 - i;
        const Index size = desc.dimension(dim);
        const Index stride = tensor_strides[dim];
        it[i] = {/*size=*/size, /*count=*/0, /*tensor_stride=*/stride, /*tensor_span=*/stride * (size - 1)};
      }
      eigen_assert(it[0].tensor_stride == 1);

      const typename TensorBlock::Storage block_storage = TensorBlock::prepareStorage(desc, scratch);
      ScalarNoConst* block_buffer = block_storage.data();

      const Index inner_size = it[0].size;
      Index tensor_index = desc.offset();
      Index offset = 0;
      for (;;) {
        Fill::Run(self, block_buffer + offset, tensor_index, inner_size);
        offset += inner_size;

        // Advance the odometer over the outer dimensions.
        int i = 1;
        for (; i < NumDims; ++i) {
          if (++it[i].count < it[i].size) {
            tensor_index += it[i].tensor_stride;
            break;
          }
          it[i].count = 0;
          tensor_index -= it[i].tensor_span;
        }
        if (i == NumDims) break;
      }

      return block_storage.AsTensorMaterializedBlock();
    }
  };

  EIGEN_STRONG_INLINE TensorBlock block(TensorBlockDesc& desc, TensorBlockScratch& scratch,
                                        bool /*root_of_expr_ast*/ = false) const {
    typedef TensorEvaluator<const TensorCwiseNullaryOp<NullaryOp, ArgType>, Device> Self;
    return NullaryBlockImpl<Self, IndexIndependentFunctor>::Run(*this, desc, scratch);
  }

  EIGEN_DEVICE_FUNC EvaluatorPointerType data() const { return nullptr; }

 private:
  const NullaryOp m_functor;
  TensorEvaluator<ArgType, Device> m_argImpl;
  const internal::nullary_wrapper<CoeffReturnType, NullaryOp> m_wrapper;
};

// -------------------- CwiseUnaryOp --------------------

template <typename UnaryOp, typename ArgType, typename Device>
struct TensorEvaluator<const TensorCwiseUnaryOp<UnaryOp, ArgType>, Device> {
  typedef TensorCwiseUnaryOp<UnaryOp, ArgType> XprType;

  static constexpr int Layout = TensorEvaluator<ArgType, Device>::Layout;
  enum {
    IsAligned = TensorEvaluator<ArgType, Device>::IsAligned,
    PacketAccess =
        int(TensorEvaluator<ArgType, Device>::PacketAccess) & int(internal::functor_traits<UnaryOp>::PacketAccess),
    BlockAccess = TensorEvaluator<ArgType, Device>::BlockAccess,
    PreferBlockAccess = TensorEvaluator<ArgType, Device>::PreferBlockAccess,
    CoordAccess = false,  // to be implemented
    RawAccess = false
  };

  EIGEN_DEVICE_FUNC TensorEvaluator(const XprType& op, const Device& device)
      : m_device(device), m_functor(op.functor()), m_argImpl(op.nestedExpression(), device) {}

  typedef typename XprType::Index Index;
  typedef typename XprType::Scalar Scalar;
  typedef std::remove_const_t<Scalar> ScalarNoConst;
  typedef typename internal::traits<XprType>::Scalar CoeffReturnType;
  typedef typename PacketType<CoeffReturnType, Device>::type PacketReturnType;
  static constexpr int PacketSize = PacketType<CoeffReturnType, Device>::size;
  typedef typename TensorEvaluator<ArgType, Device>::Dimensions Dimensions;
  typedef StorageMemory<CoeffReturnType, Device> Storage;
  typedef typename Storage::Type EvaluatorPointerType;
  static constexpr int NumDims = internal::array_size<Dimensions>::value;

  //===- Tensor block evaluation strategy (see TensorBlock.h) -------------===//
  typedef internal::TensorBlockDescriptor<NumDims, Index> TensorBlockDesc;
  typedef internal::TensorBlockScratchAllocator<Device> TensorBlockScratch;

  typedef typename TensorEvaluator<const ArgType, Device>::TensorBlock ArgTensorBlock;

  typedef internal::TensorCwiseUnaryBlock<UnaryOp, ArgTensorBlock> TensorBlock;
  //===--------------------------------------------------------------------===//

  EIGEN_DEVICE_FUNC const Dimensions& dimensions() const { return m_argImpl.dimensions(); }

  EIGEN_STRONG_INLINE bool evalSubExprsIfNeeded(EvaluatorPointerType) {
    m_argImpl.evalSubExprsIfNeeded(nullptr);
    return true;
  }

#ifdef EIGEN_USE_THREADS
  template <typename EvalSubExprsCallback>
  EIGEN_STRONG_INLINE void evalSubExprsIfNeededAsync(EvaluatorPointerType, EvalSubExprsCallback done) {
    m_argImpl.evalSubExprsIfNeededAsync(nullptr, [done](bool) { done(true); });
  }
#endif  // EIGEN_USE_THREADS

  EIGEN_STRONG_INLINE void cleanup() { m_argImpl.cleanup(); }

  EIGEN_DEVICE_FUNC CoeffReturnType coeff(Index index) const { return m_functor(m_argImpl.coeff(index)); }

  template <int LoadMode>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE PacketReturnType packet(Index index) const {
    return m_functor.packetOp(m_argImpl.template packet<LoadMode>(index));
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorOpCost costPerCoeff(bool vectorized) const {
    const double functor_cost = internal::functor_traits<UnaryOp>::Cost;
    return m_argImpl.costPerCoeff(vectorized) + TensorOpCost(0, 0, functor_cost, vectorized, PacketSize);
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE internal::TensorBlockResourceRequirements getResourceRequirements() const {
    static constexpr double functor_cost = internal::functor_traits<UnaryOp>::Cost;
    return m_argImpl.getResourceRequirements().addCostPerCoeff({0, 0, functor_cost / PacketSize});
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorBlock block(TensorBlockDesc& desc, TensorBlockScratch& scratch,
                                                          bool /*root_of_expr_ast*/ = false) const {
    // The forwarded destination buffer is sized for *our* output Scalar (the
    // assign LHS), but the child block evaluator below us writes ArgScalar.
    // For functors that change the scalar type (abs(complex)->real,
    // isnan/isfinite->bool, ...) the buffer would be misinterpreted by any
    // block-materializing child's prepareStorage (assert in debug, corruption
    // in release). Drop the buffer in that case; the child falls back to
    // scratch and writeBlock still lands the converted values in the LHS.
    constexpr bool ScalarTypesMatch =
        std::is_same<std::remove_const_t<Scalar>,
                     std::remove_const_t<typename TensorEvaluator<ArgType, Device>::Scalar>>::value;
    EIGEN_IF_CONSTEXPR (!ScalarTypesMatch) {
      desc.DropDestinationBuffer();
    }
    return TensorBlock(m_argImpl.block(desc, scratch), m_functor);
  }

  EIGEN_DEVICE_FUNC EvaluatorPointerType data() const { return nullptr; }

 private:
  const Device EIGEN_DEVICE_REF m_device;
  const UnaryOp m_functor;
  TensorEvaluator<ArgType, Device> m_argImpl;
};

// -------------------- CwiseBinaryOp --------------------

template <typename BinaryOp, typename LeftArgType, typename RightArgType, typename Device>
struct TensorEvaluator<const TensorCwiseBinaryOp<BinaryOp, LeftArgType, RightArgType>, Device> {
  typedef TensorCwiseBinaryOp<BinaryOp, LeftArgType, RightArgType> XprType;

  static constexpr int Layout = TensorEvaluator<LeftArgType, Device>::Layout;
  enum {
    IsAligned =
        int(TensorEvaluator<LeftArgType, Device>::IsAligned) & int(TensorEvaluator<RightArgType, Device>::IsAligned),
    PacketAccess = int(TensorEvaluator<LeftArgType, Device>::PacketAccess) &
                   int(TensorEvaluator<RightArgType, Device>::PacketAccess) &
                   int(internal::functor_traits<BinaryOp>::PacketAccess),
    BlockAccess = int(TensorEvaluator<LeftArgType, Device>::BlockAccess) &
                  int(TensorEvaluator<RightArgType, Device>::BlockAccess),
    PreferBlockAccess = int(TensorEvaluator<LeftArgType, Device>::PreferBlockAccess) |
                        int(TensorEvaluator<RightArgType, Device>::PreferBlockAccess),
    CoordAccess = false,  // to be implemented
    RawAccess = false
  };

  EIGEN_DEVICE_FUNC TensorEvaluator(const XprType& op, const Device& device)
      : m_device(device),
        m_functor(op.functor()),
        m_leftImpl(op.lhsExpression(), device),
        m_rightImpl(op.rhsExpression(), device) {
    EIGEN_STATIC_ASSERT((static_cast<int>(TensorEvaluator<LeftArgType, Device>::Layout) ==
                             static_cast<int>(TensorEvaluator<RightArgType, Device>::Layout) ||
                         internal::traits<XprType>::NumDimensions <= 1),
                        YOU_MADE_A_PROGRAMMING_MISTAKE);
    eigen_assert(dimensions_match(m_leftImpl.dimensions(), m_rightImpl.dimensions()));
  }

  typedef typename XprType::Index Index;
  typedef typename XprType::Scalar Scalar;
  typedef typename internal::traits<XprType>::Scalar CoeffReturnType;
  typedef typename PacketType<CoeffReturnType, Device>::type PacketReturnType;
  static constexpr int PacketSize = PacketType<CoeffReturnType, Device>::size;
  typedef typename TensorEvaluator<LeftArgType, Device>::Dimensions Dimensions;
  typedef StorageMemory<CoeffReturnType, Device> Storage;
  typedef typename Storage::Type EvaluatorPointerType;

  static constexpr int NumDims = internal::array_size<typename TensorEvaluator<LeftArgType, Device>::Dimensions>::value;

  //===- Tensor block evaluation strategy (see TensorBlock.h) -------------===//
  typedef internal::TensorBlockDescriptor<NumDims, Index> TensorBlockDesc;
  typedef internal::TensorBlockScratchAllocator<Device> TensorBlockScratch;

  typedef typename TensorEvaluator<const LeftArgType, Device>::TensorBlock LeftTensorBlock;
  typedef typename TensorEvaluator<const RightArgType, Device>::TensorBlock RightTensorBlock;

  typedef internal::TensorCwiseBinaryBlock<BinaryOp, LeftTensorBlock, RightTensorBlock> TensorBlock;
  //===--------------------------------------------------------------------===//

  EIGEN_DEVICE_FUNC const Dimensions& dimensions() const {
    // TODO: use right impl instead if right impl dimensions are known at compile time.
    return m_leftImpl.dimensions();
  }

  EIGEN_STRONG_INLINE bool evalSubExprsIfNeeded(EvaluatorPointerType) {
    m_leftImpl.evalSubExprsIfNeeded(nullptr);
    m_rightImpl.evalSubExprsIfNeeded(nullptr);
    return true;
  }

#ifdef EIGEN_USE_THREADS
  template <typename EvalSubExprsCallback>
  EIGEN_STRONG_INLINE void evalSubExprsIfNeededAsync(EvaluatorPointerType, EvalSubExprsCallback done) {
    // TODO(ezhulenev): Evaluate two expression in parallel?
    m_leftImpl.evalSubExprsIfNeededAsync(
        nullptr, [this, done](bool) { m_rightImpl.evalSubExprsIfNeededAsync(nullptr, [done](bool) { done(true); }); });
  }
#endif  // EIGEN_USE_THREADS

  EIGEN_STRONG_INLINE void cleanup() {
    m_leftImpl.cleanup();
    m_rightImpl.cleanup();
  }

  EIGEN_DEVICE_FUNC CoeffReturnType coeff(Index index) const {
    return m_functor(m_leftImpl.coeff(index), m_rightImpl.coeff(index));
  }
  template <int LoadMode>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE PacketReturnType packet(Index index) const {
    return m_functor.packetOp(m_leftImpl.template packet<LoadMode>(index),
                              m_rightImpl.template packet<LoadMode>(index));
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorOpCost costPerCoeff(bool vectorized) const {
    const double functor_cost = internal::functor_traits<BinaryOp>::Cost;
    return m_leftImpl.costPerCoeff(vectorized) + m_rightImpl.costPerCoeff(vectorized) +
           TensorOpCost(0, 0, functor_cost, vectorized, PacketSize);
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE internal::TensorBlockResourceRequirements getResourceRequirements() const {
    static constexpr double functor_cost = internal::functor_traits<BinaryOp>::Cost;
    return internal::TensorBlockResourceRequirements::merge(m_leftImpl.getResourceRequirements(),
                                                            m_rightImpl.getResourceRequirements())
        .addCostPerCoeff({0, 0, functor_cost / PacketSize});
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorBlock block(TensorBlockDesc& desc, TensorBlockScratch& scratch,
                                                          bool /*root_of_expr_ast*/ = false) const {
    desc.DropDestinationBuffer();
    return TensorBlock(m_leftImpl.block(desc, scratch), m_rightImpl.block(desc, scratch), m_functor);
  }

  EIGEN_DEVICE_FUNC EvaluatorPointerType data() const { return nullptr; }

 private:
  const Device EIGEN_DEVICE_REF m_device;
  const BinaryOp m_functor;
  TensorEvaluator<LeftArgType, Device> m_leftImpl;
  TensorEvaluator<RightArgType, Device> m_rightImpl;
};

// -------------------- CwiseTernaryOp --------------------

template <typename TernaryOp, typename Arg1Type, typename Arg2Type, typename Arg3Type, typename Device>
struct TensorEvaluator<const TensorCwiseTernaryOp<TernaryOp, Arg1Type, Arg2Type, Arg3Type>, Device> {
  typedef TensorCwiseTernaryOp<TernaryOp, Arg1Type, Arg2Type, Arg3Type> XprType;

  static constexpr int Layout = TensorEvaluator<Arg1Type, Device>::Layout;
  enum {
    IsAligned = TensorEvaluator<Arg1Type, Device>::IsAligned & TensorEvaluator<Arg2Type, Device>::IsAligned &
                TensorEvaluator<Arg3Type, Device>::IsAligned,
    PacketAccess = TensorEvaluator<Arg1Type, Device>::PacketAccess && TensorEvaluator<Arg2Type, Device>::PacketAccess &&
                   TensorEvaluator<Arg3Type, Device>::PacketAccess && internal::functor_traits<TernaryOp>::PacketAccess,
    BlockAccess = TensorEvaluator<Arg1Type, Device>::BlockAccess && TensorEvaluator<Arg2Type, Device>::BlockAccess &&
                  TensorEvaluator<Arg3Type, Device>::BlockAccess,
    PreferBlockAccess = TensorEvaluator<Arg1Type, Device>::PreferBlockAccess ||
                        TensorEvaluator<Arg2Type, Device>::PreferBlockAccess ||
                        TensorEvaluator<Arg3Type, Device>::PreferBlockAccess,
    CoordAccess = false,  // to be implemented
    RawAccess = false
  };

  EIGEN_DEVICE_FUNC TensorEvaluator(const XprType& op, const Device& device)
      : m_functor(op.functor()),
        m_arg1Impl(op.arg1Expression(), device),
        m_arg2Impl(op.arg2Expression(), device),
        m_arg3Impl(op.arg3Expression(), device) {
    EIGEN_STATIC_ASSERT((static_cast<int>(TensorEvaluator<Arg1Type, Device>::Layout) ==
                             static_cast<int>(TensorEvaluator<Arg3Type, Device>::Layout) ||
                         internal::traits<XprType>::NumDimensions <= 1),
                        YOU_MADE_A_PROGRAMMING_MISTAKE);

    EIGEN_STATIC_ASSERT((std::is_same<typename internal::traits<Arg1Type>::StorageKind,
                                      typename internal::traits<Arg2Type>::StorageKind>::value),
                        STORAGE_KIND_MUST_MATCH)
    EIGEN_STATIC_ASSERT((std::is_same<typename internal::traits<Arg1Type>::StorageKind,
                                      typename internal::traits<Arg3Type>::StorageKind>::value),
                        STORAGE_KIND_MUST_MATCH)
    EIGEN_STATIC_ASSERT(
        (std::is_same<typename internal::traits<Arg1Type>::Index, typename internal::traits<Arg2Type>::Index>::value),
        STORAGE_INDEX_MUST_MATCH)
    EIGEN_STATIC_ASSERT(
        (std::is_same<typename internal::traits<Arg1Type>::Index, typename internal::traits<Arg3Type>::Index>::value),
        STORAGE_INDEX_MUST_MATCH)

    eigen_assert(dimensions_match(m_arg1Impl.dimensions(), m_arg2Impl.dimensions()) &&
                 dimensions_match(m_arg1Impl.dimensions(), m_arg3Impl.dimensions()));
  }

  typedef typename XprType::Index Index;
  typedef typename XprType::Scalar Scalar;
  typedef typename internal::traits<XprType>::Scalar CoeffReturnType;
  typedef typename PacketType<CoeffReturnType, Device>::type PacketReturnType;
  static constexpr int PacketSize = PacketType<CoeffReturnType, Device>::size;
  typedef typename TensorEvaluator<Arg1Type, Device>::Dimensions Dimensions;
  typedef StorageMemory<CoeffReturnType, Device> Storage;
  typedef typename Storage::Type EvaluatorPointerType;
  static constexpr int NumDims = internal::array_size<Dimensions>::value;

  //===- Tensor block evaluation strategy (see TensorBlock.h) -------------===//
  typedef internal::TensorBlockDescriptor<NumDims, Index> TensorBlockDesc;
  typedef internal::TensorBlockScratchAllocator<Device> TensorBlockScratch;

  typedef typename TensorEvaluator<const Arg1Type, Device>::TensorBlock Arg1TensorBlock;
  typedef typename TensorEvaluator<const Arg2Type, Device>::TensorBlock Arg2TensorBlock;
  typedef typename TensorEvaluator<const Arg3Type, Device>::TensorBlock Arg3TensorBlock;

  // Rebuilds the ternary expression over the arguments' block expressions,
  // carrying the functor; mirrors TensorSelectOpBlockFactory.
  struct TensorCwiseTernaryOpBlockFactory {
    TernaryOp func;

    template <typename Arg1XprType, typename Arg2XprType, typename Arg3XprType>
    struct XprType {
      typedef TensorCwiseTernaryOp<TernaryOp, const Arg1XprType, const Arg2XprType, const Arg3XprType> type;
    };

    template <typename Arg1XprType, typename Arg2XprType, typename Arg3XprType>
    typename XprType<Arg1XprType, Arg2XprType, Arg3XprType>::type expr(const Arg1XprType& arg1, const Arg2XprType& arg2,
                                                                       const Arg3XprType& arg3) const {
      return typename XprType<Arg1XprType, Arg2XprType, Arg3XprType>::type(arg1, arg2, arg3, func);
    }
  };

  typedef internal::TensorTernaryExprBlock<TensorCwiseTernaryOpBlockFactory, Arg1TensorBlock, Arg2TensorBlock,
                                           Arg3TensorBlock>
      TensorBlock;
  //===--------------------------------------------------------------------===//

  EIGEN_DEVICE_FUNC const Dimensions& dimensions() const {
    // TODO: use arg2 or arg3 dimensions if they are known at compile time.
    return m_arg1Impl.dimensions();
  }

  EIGEN_STRONG_INLINE bool evalSubExprsIfNeeded(EvaluatorPointerType) {
    m_arg1Impl.evalSubExprsIfNeeded(nullptr);
    m_arg2Impl.evalSubExprsIfNeeded(nullptr);
    m_arg3Impl.evalSubExprsIfNeeded(nullptr);
    return true;
  }
  EIGEN_STRONG_INLINE void cleanup() {
    m_arg1Impl.cleanup();
    m_arg2Impl.cleanup();
    m_arg3Impl.cleanup();
  }

  EIGEN_DEVICE_FUNC CoeffReturnType coeff(Index index) const {
    return m_functor(m_arg1Impl.coeff(index), m_arg2Impl.coeff(index), m_arg3Impl.coeff(index));
  }
  template <int LoadMode>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE PacketReturnType packet(Index index) const {
    return m_functor.packetOp(m_arg1Impl.template packet<LoadMode>(index), m_arg2Impl.template packet<LoadMode>(index),
                              m_arg3Impl.template packet<LoadMode>(index));
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorOpCost costPerCoeff(bool vectorized) const {
    const double functor_cost = internal::functor_traits<TernaryOp>::Cost;
    return m_arg1Impl.costPerCoeff(vectorized) + m_arg2Impl.costPerCoeff(vectorized) +
           m_arg3Impl.costPerCoeff(vectorized) + TensorOpCost(0, 0, functor_cost, vectorized, PacketSize);
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE internal::TensorBlockResourceRequirements getResourceRequirements() const {
    static constexpr double functor_cost = internal::functor_traits<TernaryOp>::Cost;
    return internal::TensorBlockResourceRequirements::merge(
               m_arg1Impl.getResourceRequirements(),
               internal::TensorBlockResourceRequirements::merge(m_arg2Impl.getResourceRequirements(),
                                                                m_arg3Impl.getResourceRequirements()))
        .addCostPerCoeff({0, 0, functor_cost / PacketSize});
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorBlock block(TensorBlockDesc& desc, TensorBlockScratch& scratch,
                                                          bool /*root_of_expr_ast*/ = false) const {
    // The forwarded destination buffer might alias one of the inputs; drop it
    // like the binary evaluator does.
    desc.DropDestinationBuffer();
    return TensorBlock(m_arg1Impl.block(desc, scratch), m_arg2Impl.block(desc, scratch),
                       m_arg3Impl.block(desc, scratch), TensorCwiseTernaryOpBlockFactory{m_functor});
  }

  EIGEN_DEVICE_FUNC EvaluatorPointerType data() const { return nullptr; }

 private:
  const TernaryOp m_functor;
  TensorEvaluator<Arg1Type, Device> m_arg1Impl;
  TensorEvaluator<Arg2Type, Device> m_arg2Impl;
  TensorEvaluator<Arg3Type, Device> m_arg3Impl;
};

// -------------------- SelectOp --------------------

template <typename IfArgType, typename ThenArgType, typename ElseArgType, typename Device>
struct TensorEvaluator<const TensorSelectOp<IfArgType, ThenArgType, ElseArgType>, Device> {
  typedef TensorSelectOp<IfArgType, ThenArgType, ElseArgType> XprType;
  typedef typename XprType::Scalar Scalar;

  using TernarySelectOp = internal::scalar_boolean_select_op<typename internal::traits<ThenArgType>::Scalar,
                                                             typename internal::traits<ElseArgType>::Scalar,
                                                             typename internal::traits<IfArgType>::Scalar>;
  static constexpr bool TernaryPacketAccess =
      TensorEvaluator<ThenArgType, Device>::PacketAccess && TensorEvaluator<ElseArgType, Device>::PacketAccess &&
      TensorEvaluator<IfArgType, Device>::PacketAccess && internal::functor_traits<TernarySelectOp>::PacketAccess;

  static constexpr int Layout = TensorEvaluator<IfArgType, Device>::Layout;
  enum {
    IsAligned = TensorEvaluator<ThenArgType, Device>::IsAligned & TensorEvaluator<ElseArgType, Device>::IsAligned,
    PacketAccess =
        (TensorEvaluator<ThenArgType, Device>::PacketAccess && TensorEvaluator<ElseArgType, Device>::PacketAccess) ||
        TernaryPacketAccess,
    BlockAccess = TensorEvaluator<IfArgType, Device>::BlockAccess &&
                  TensorEvaluator<ThenArgType, Device>::BlockAccess &&
                  TensorEvaluator<ElseArgType, Device>::BlockAccess,
    PreferBlockAccess = TensorEvaluator<IfArgType, Device>::PreferBlockAccess ||
                        TensorEvaluator<ThenArgType, Device>::PreferBlockAccess ||
                        TensorEvaluator<ElseArgType, Device>::PreferBlockAccess,
    CoordAccess = false,  // to be implemented
    RawAccess = false
  };

  EIGEN_DEVICE_FUNC TensorEvaluator(const XprType& op, const Device& device)
      : m_condImpl(op.ifExpression(), device),
        m_thenImpl(op.thenExpression(), device),
        m_elseImpl(op.elseExpression(), device) {
    EIGEN_STATIC_ASSERT((static_cast<int>(TensorEvaluator<IfArgType, Device>::Layout) ==
                         static_cast<int>(TensorEvaluator<ThenArgType, Device>::Layout)),
                        YOU_MADE_A_PROGRAMMING_MISTAKE);
    EIGEN_STATIC_ASSERT((static_cast<int>(TensorEvaluator<IfArgType, Device>::Layout) ==
                         static_cast<int>(TensorEvaluator<ElseArgType, Device>::Layout)),
                        YOU_MADE_A_PROGRAMMING_MISTAKE);
    eigen_assert(dimensions_match(m_condImpl.dimensions(), m_thenImpl.dimensions()));
    eigen_assert(dimensions_match(m_thenImpl.dimensions(), m_elseImpl.dimensions()));
  }

  typedef typename XprType::Index Index;
  typedef typename internal::traits<XprType>::Scalar CoeffReturnType;
  typedef typename PacketType<CoeffReturnType, Device>::type PacketReturnType;
  static constexpr int PacketSize = PacketType<CoeffReturnType, Device>::size;
  typedef typename TensorEvaluator<IfArgType, Device>::Dimensions Dimensions;
  typedef StorageMemory<CoeffReturnType, Device> Storage;
  typedef typename Storage::Type EvaluatorPointerType;

  static constexpr int NumDims = internal::array_size<Dimensions>::value;

  //===- Tensor block evaluation strategy (see TensorBlock.h) -------------===//
  typedef internal::TensorBlockDescriptor<NumDims, Index> TensorBlockDesc;
  typedef internal::TensorBlockScratchAllocator<Device> TensorBlockScratch;

  typedef typename TensorEvaluator<const IfArgType, Device>::TensorBlock IfArgTensorBlock;
  typedef typename TensorEvaluator<const ThenArgType, Device>::TensorBlock ThenArgTensorBlock;
  typedef typename TensorEvaluator<const ElseArgType, Device>::TensorBlock ElseArgTensorBlock;

  struct TensorSelectOpBlockFactory {
    template <typename IfArgXprType, typename ThenArgXprType, typename ElseArgXprType>
    struct XprType {
      typedef TensorSelectOp<const IfArgXprType, const ThenArgXprType, const ElseArgXprType> type;
    };

    template <typename IfArgXprType, typename ThenArgXprType, typename ElseArgXprType>
    typename XprType<IfArgXprType, ThenArgXprType, ElseArgXprType>::type expr(const IfArgXprType& if_expr,
                                                                              const ThenArgXprType& then_expr,
                                                                              const ElseArgXprType& else_expr) const {
      return typename XprType<IfArgXprType, ThenArgXprType, ElseArgXprType>::type(if_expr, then_expr, else_expr);
    }
  };

  typedef internal::TensorTernaryExprBlock<TensorSelectOpBlockFactory, IfArgTensorBlock, ThenArgTensorBlock,
                                           ElseArgTensorBlock>
      TensorBlock;
  //===--------------------------------------------------------------------===//

  EIGEN_DEVICE_FUNC const Dimensions& dimensions() const {
    // TODO: use then or else impl instead if they happen to be known at compile time.
    return m_condImpl.dimensions();
  }

  EIGEN_STRONG_INLINE bool evalSubExprsIfNeeded(EvaluatorPointerType) {
    m_condImpl.evalSubExprsIfNeeded(nullptr);
    m_thenImpl.evalSubExprsIfNeeded(nullptr);
    m_elseImpl.evalSubExprsIfNeeded(nullptr);
    return true;
  }

#ifdef EIGEN_USE_THREADS
  template <typename EvalSubExprsCallback>
  EIGEN_STRONG_INLINE void evalSubExprsIfNeededAsync(EvaluatorPointerType, EvalSubExprsCallback done) {
    m_condImpl.evalSubExprsIfNeeded(nullptr, [this, done](bool) {
      m_thenImpl.evalSubExprsIfNeeded(
          nullptr, [this, done](bool) { m_elseImpl.evalSubExprsIfNeeded(nullptr, [done](bool) { done(true); }); });
    });
  }
#endif  // EIGEN_USE_THREADS

  EIGEN_STRONG_INLINE void cleanup() {
    m_condImpl.cleanup();
    m_thenImpl.cleanup();
    m_elseImpl.cleanup();
  }

  EIGEN_DEVICE_FUNC CoeffReturnType coeff(Index index) const {
    return m_condImpl.coeff(index) ? m_thenImpl.coeff(index) : m_elseImpl.coeff(index);
  }

  template <int LoadMode, bool UseTernary = TernaryPacketAccess, std::enable_if_t<!UseTernary, bool> = true>
  EIGEN_DEVICE_FUNC PacketReturnType packet(Index index) const {
    EIGEN_ALIGN_TO_BOUNDARY(internal::unpacket_traits<PacketReturnType>::alignment)
    std::remove_const_t<Scalar> arr[PacketSize];
    EIGEN_UNROLL_LOOP
    for (Index i = 0; i < PacketSize; ++i) {
      arr[i] = m_condImpl.coeff(index + i) ? Scalar(-1) : Scalar(0);
    }
    return TernarySelectOp().template packetOp<PacketReturnType>(m_thenImpl.template packet<LoadMode>(index),
                                                                 m_elseImpl.template packet<LoadMode>(index),
                                                                 internal::pload<PacketReturnType>(arr));
  }

  template <int LoadMode, bool UseTernary = TernaryPacketAccess, std::enable_if_t<UseTernary, bool> = true>
  EIGEN_DEVICE_FUNC PacketReturnType packet(Index index) const {
    return TernarySelectOp().template packetOp<PacketReturnType>(m_thenImpl.template packet<LoadMode>(index),
                                                                 m_elseImpl.template packet<LoadMode>(index),
                                                                 m_condImpl.template packet<LoadMode>(index));
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorOpCost costPerCoeff(bool vectorized) const {
    return m_condImpl.costPerCoeff(vectorized) +
           m_thenImpl.costPerCoeff(vectorized).cwiseMax(m_elseImpl.costPerCoeff(vectorized));
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE internal::TensorBlockResourceRequirements getResourceRequirements() const {
    auto then_req = m_thenImpl.getResourceRequirements();
    auto else_req = m_elseImpl.getResourceRequirements();

    auto merged_req = internal::TensorBlockResourceRequirements::merge(then_req, else_req);
    merged_req.cost_per_coeff = then_req.cost_per_coeff.cwiseMax(else_req.cost_per_coeff);

    return internal::TensorBlockResourceRequirements::merge(m_condImpl.getResourceRequirements(), merged_req);
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorBlock block(TensorBlockDesc& desc, TensorBlockScratch& scratch,
                                                          bool /*root_of_expr_ast*/ = false) const {
    // It's unsafe to pass destination buffer to underlying expressions, because
    // output might be aliased with one of the inputs.
    desc.DropDestinationBuffer();

    return TensorBlock(m_condImpl.block(desc, scratch), m_thenImpl.block(desc, scratch),
                       m_elseImpl.block(desc, scratch), TensorSelectOpBlockFactory());
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE EvaluatorPointerType data() const { return nullptr; }

#ifdef EIGEN_USE_SYCL
  // binding placeholder accessors to a command group handler for SYCL
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void bind(cl::sycl::handler& cgh) const {
    m_condImpl.bind(cgh);
    m_thenImpl.bind(cgh);
    m_elseImpl.bind(cgh);
  }
#endif
 private:
  TensorEvaluator<IfArgType, Device> m_condImpl;
  TensorEvaluator<ThenArgType, Device> m_thenImpl;
  TensorEvaluator<ElseArgType, Device> m_elseImpl;
};

}  // end namespace Eigen

#if defined(EIGEN_USE_SYCL) && defined(SYCL_COMPILER_IS_DPCPP)
template <typename Derived, typename Device>
struct cl::sycl::is_device_copyable<
    Eigen::TensorEvaluator<Derived, Device>,
    std::enable_if_t<!std::is_trivially_copyable<Eigen::TensorEvaluator<Derived, Device>>::value>> : std::true_type {};
#endif

#endif  // EIGEN_TENSOR_TENSOR_EVALUATOR_H
