// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2014 Benoit Steiner <benoit.steiner.goog@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_TENSOR_TENSOR_STRIDING_H
#define EIGEN_TENSOR_TENSOR_STRIDING_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

namespace internal {
template <typename Strides, typename XprType>
struct traits<TensorStridingOp<Strides, XprType> > : public traits<XprType> {
  typedef typename XprType::Scalar Scalar;
  typedef traits<XprType> XprTraits;
  typedef typename XprTraits::StorageKind StorageKind;
  typedef typename XprTraits::Index Index;
  static constexpr int NumDimensions = XprTraits::NumDimensions;
  static constexpr int Layout = XprTraits::Layout;
  typedef typename XprTraits::PointerType PointerType;
};

template <typename Strides, typename XprType>
struct eval<TensorStridingOp<Strides, XprType>, Eigen::Dense> {
  typedef const TensorStridingOp<Strides, XprType> EIGEN_DEVICE_REF type;
};

}  // end namespace internal

/**
 * \ingroup Tensor_Module
 *
 * \brief Tensor striding class.
 */
template <typename Strides, typename XprType>
class TensorStridingOp : public TensorBase<TensorStridingOp<Strides, XprType> > {
 public:
  typedef TensorBase<TensorStridingOp<Strides, XprType> > Base;
  typedef typename Eigen::internal::traits<TensorStridingOp>::Scalar Scalar;
  typedef typename Eigen::NumTraits<Scalar>::Real RealScalar;
  typedef typename XprType::CoeffReturnType CoeffReturnType;
  typedef typename Eigen::internal::ref_selector<TensorStridingOp>::type Nested;
  typedef typename Eigen::internal::traits<TensorStridingOp>::StorageKind StorageKind;
  typedef typename Eigen::internal::traits<TensorStridingOp>::Index Index;

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorStridingOp(const XprType& expr, const Strides& dims)
      : m_xpr(expr), m_dims(dims) {}

  EIGEN_DEVICE_FUNC const Strides& strides() const { return m_dims; }

  EIGEN_DEVICE_FUNC const internal::remove_all_t<typename XprType::Nested>& expression() const { return m_xpr; }

  EIGEN_INHERIT_ASSIGNMENT_OPERATORS(TensorStridingOp)

 protected:
  typename XprType::Nested m_xpr;
  const Strides m_dims;
};

// Eval as rvalue
template <typename Strides, typename ArgType, typename Device>
struct TensorEvaluator<const TensorStridingOp<Strides, ArgType>, Device> {
  typedef TensorStridingOp<Strides, ArgType> XprType;
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
  // Inner-most dimension in storage order: the one a packet runs along.
  static constexpr int kInnerDim = (static_cast<int>(Layout) == static_cast<int>(ColMajor)) ? 0 : NumDims - 1;
  // Tag selecting whether the argument can be asked for whole packets.
  typedef std::integral_constant<bool, TensorEvaluator<ArgType, Device>::PacketAccess> ImplHasPacket;
  enum {
    IsAligned = false,
    // Packets are assembled from inner runs even when the nested evaluator
    // only exposes coefficient access.
    PacketAccess = (PacketSize > 1),
    BlockAccess = false,
    PreferBlockAccess = TensorEvaluator<ArgType, Device>::PreferBlockAccess,
    CoordAccess = false,  // to be implemented
    RawAccess = false
  };

  //===- Tensor block evaluation strategy (see TensorBlock.h) -------------===//
  typedef internal::TensorBlockNotImplemented TensorBlock;
  //===--------------------------------------------------------------------===//

  EIGEN_STRONG_INLINE TensorEvaluator(const XprType& op, const Device& device) : m_impl(op.expression(), device) {
    m_dimensions = m_impl.dimensions();
    m_is_identity = true;
    for (int i = 0; i < NumDims; ++i) {
      m_dimensions[i] = Eigen::numext::ceil(static_cast<float>(m_dimensions[i]) / op.strides()[i]);
      if (op.strides()[i] != 1) m_is_identity = false;
    }

    const typename TensorEvaluator<ArgType, Device>::Dimensions& input_dims = m_impl.dimensions();
    EIGEN_IF_CONSTEXPR (static_cast<int>(Layout) == static_cast<int>(ColMajor)) {
      m_outputStrides[0] = 1;
      m_inputStrides[0] = 1;
      for (int i = 1; i < NumDims; ++i) {
        m_outputStrides[i] = m_outputStrides[i - 1] * m_dimensions[i - 1];
        m_inputStrides[i] = m_inputStrides[i - 1] * input_dims[i - 1];
        m_inputStrides[i - 1] *= op.strides()[i - 1];
      }
      m_inputStrides[NumDims - 1] *= op.strides()[NumDims - 1];
    } else {  // RowMajor
      m_outputStrides[NumDims - 1] = 1;
      m_inputStrides[NumDims - 1] = 1;
      for (int i = NumDims - 2; i >= 0; --i) {
        m_outputStrides[i] = m_outputStrides[i + 1] * m_dimensions[i + 1];
        m_inputStrides[i] = m_inputStrides[i + 1] * input_dims[i + 1];
        m_inputStrides[i + 1] *= op.strides()[i + 1];
      }
      m_inputStrides[0] *= op.strides()[0];
    }
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const Dimensions& dimensions() const { return m_dimensions; }

  EIGEN_STRONG_INLINE bool evalSubExprsIfNeeded(EvaluatorPointerType /*data*/) {
    m_impl.evalSubExprsIfNeeded(nullptr);
    return true;
  }
  EIGEN_STRONG_INLINE void cleanup() { m_impl.cleanup(); }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE CoeffReturnType coeff(Index index) const {
    if (m_is_identity) {
      return m_impl.coeff(index);
    }
    return m_impl.coeff(srcCoeff(index));
  }

  // Reads PacketSize coefficients of the argument starting at `base` and
  // stepping by `inner_stride`. Callers guarantee those lanes lie in a single
  // inner-most run, which is what makes the input indices an arithmetic
  // progression and lets the index mapping be computed once per packet
  // instead of once per coefficient. The argument's packet() must not be
  // instantiated when it has no packet access, hence the tag-dispatched pair
  // rather than a plain branch (C++14 has no if constexpr).
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE PacketReturnType loadInnerRun(Index base, Index inner_stride,
                                                                      std::false_type) const {
    EIGEN_ALIGN_TO_BOUNDARY(internal::unpacket_traits<PacketReturnType>::alignment)
    std::remove_const_t<CoeffReturnType> values[PacketSize];
    EIGEN_UNROLL_LOOP
    for (int i = 0; i < PacketSize; ++i) {
      values[i] = m_impl.coeff(base + i * inner_stride);
    }
    return internal::pload<PacketReturnType>(values);
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE PacketReturnType loadInnerRun(Index base, Index inner_stride,
                                                                      std::true_type) const {
    if (inner_stride == 1) return m_impl.template packet<Unaligned>(base);
    return loadInnerRun(base, inner_stride, std::false_type());
  }

  template <int LoadMode>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE PacketReturnType packet(Index index) const {
    eigen_assert(index + PacketSize - 1 < dimensions().TotalSize());
    // Identity striding is just the inner-run case with step one: srcCoeff is
    // then the identity and the inner input stride is 1.
    if (m_is_identity) return loadInnerRun(index, 1, ImplHasPacket());
    Index base;
    if (packetStaysInInnerRun(index, base)) return loadInnerRun(base, m_inputStrides[kInnerDim], ImplHasPacket());

    // The packet crosses an inner-run boundary, so every lane needs its own
    // index mapping.
    EIGEN_ALIGN_TO_BOUNDARY(internal::unpacket_traits<PacketReturnType>::alignment)
    std::remove_const_t<CoeffReturnType> values[PacketSize];
    EIGEN_UNROLL_LOOP
    for (int i = 0; i < PacketSize; ++i) {
      values[i] = coeff(index + i);
    }
    return internal::pload<PacketReturnType>(values);
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE TensorOpCost costPerCoeff(bool vectorized) const {
    const double compute_cost = m_is_identity
                                    ? TensorOpCost::AddCost<Index>()
                                    : (NumDims - 1) * (TensorOpCost::AddCost<Index>() + TensorOpCost::MulCost<Index>() +
                                                       TensorOpCost::DivCost<Index>()) +
                                          TensorOpCost::MulCost<Index>();
    // The nested evaluator is served whole packets only when it has packet
    // access at all, and the inner runs are packet-aligned with an unstrided
    // inner dimension; everywhere else it is driven coefficient by coefficient
    // and must be charged at scalar rates. Likewise the once-per-packet index
    // mapping amortizes only while packets stay inside one inner run; the
    // cross-run fallback recomputes it per lane. Identity striding satisfies
    // both conditions, so it needs no special case here.
    const bool packets_stay_in_inner =
        m_is_identity || (m_dimensions[kInnerDim] > 0 && m_dimensions[kInnerDim] % PacketSize == 0);
    const bool packetizes_arg =
        TensorEvaluator<ArgType, Device>::PacketAccess && packets_stay_in_inner && m_inputStrides[kInnerDim] == 1;
    return m_impl.costPerCoeff(vectorized && packetizes_arg) +
           TensorOpCost(0, 0, compute_cost, vectorized && packets_stay_in_inner, PacketSize);
  }

  EIGEN_DEVICE_FUNC typename Storage::Type data() const { return nullptr; }

 protected:
  // Computes the input index of output index `index` and, as a by-product of
  // the same walk, the output's inner-dimension coordinate. The packet paths
  // use the latter to test whether a whole packet stays inside one inner-most
  // run without spending an extra division on it.
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Index srcCoeffInner(Index index, Index& inner_pos) const {
    Index inputIndex = 0;
    EIGEN_IF_CONSTEXPR (static_cast<int>(Layout) == static_cast<int>(ColMajor)) {
      EIGEN_UNROLL_LOOP
      for (int i = NumDims - 1; i > 0; --i) {
        const Index idx = index / m_outputStrides[i];
        inputIndex += idx * m_inputStrides[i];
        index -= idx * m_outputStrides[i];
      }
      inner_pos = index;
      inputIndex += index * m_inputStrides[0];
    } else {  // RowMajor
      EIGEN_UNROLL_LOOP
      for (int i = 0; i < NumDims - 1; ++i) {
        const Index idx = index / m_outputStrides[i];
        inputIndex += idx * m_inputStrides[i];
        index -= idx * m_outputStrides[i];
      }
      inner_pos = index;
      inputIndex += index * m_inputStrides[NumDims - 1];
    }
    return inputIndex;
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Index srcCoeff(Index index) const {
    Index inner_pos;
    return srcCoeffInner(index, inner_pos);
  }

  // True when the whole packet at output `index` stays inside one inner-most
  // run; sets `base` to the input index of its first lane either way.
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE bool packetStaysInInnerRun(Index index, Index& base) const {
    Index inner_pos;
    base = srcCoeffInner(index, inner_pos);
    return inner_pos + PacketSize <= m_dimensions[kInnerDim];
  }

  Dimensions m_dimensions;
  bool m_is_identity;
  array<Index, NumDims> m_outputStrides;
  array<Index, NumDims> m_inputStrides;
  TensorEvaluator<ArgType, Device> m_impl;
};

// Eval as lvalue
template <typename Strides, typename ArgType, typename Device>
struct TensorEvaluator<TensorStridingOp<Strides, ArgType>, Device>
    : public TensorEvaluator<const TensorStridingOp<Strides, ArgType>, Device> {
  typedef TensorStridingOp<Strides, ArgType> XprType;
  typedef TensorEvaluator<const XprType, Device> Base;
  static constexpr int NumDims = internal::array_size<typename TensorEvaluator<ArgType, Device>::Dimensions>::value;

  typedef typename XprType::Index Index;
  typedef typename XprType::Scalar Scalar;
  typedef typename XprType::CoeffReturnType CoeffReturnType;
  typedef typename PacketType<CoeffReturnType, Device>::type PacketReturnType;
  static constexpr int PacketSize = PacketType<CoeffReturnType, Device>::size;
  static constexpr int Layout = TensorEvaluator<ArgType, Device>::Layout;
  enum {
    IsAligned = false,
    // Packets are scattered into inner runs even when the nested evaluator
    // only exposes coefficient access.
    PacketAccess = Base::PacketAccess,
    PreferBlockAccess = false,
    CoordAccess = false,  // to be implemented
    RawAccess = false
  };

  EIGEN_STRONG_INLINE TensorEvaluator(const XprType& op, const Device& device) : Base(op, device) {}

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar& coeffRef(Index index) const {
    if (this->m_is_identity) return this->m_impl.coeffRef(index);
    return this->m_impl.coeffRef(this->srcCoeff(index));
  }

  // Mirror of the rvalue loadInnerRun: scatters a packet across PacketSize
  // coefficients starting at `base` and stepping by `inner_stride`.
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void storeInnerRun(Index base, Index inner_stride, const PacketReturnType& x,
                                                           std::false_type) const {
    EIGEN_ALIGN_TO_BOUNDARY(internal::unpacket_traits<PacketReturnType>::alignment) Scalar values[PacketSize];
    internal::pstore<Scalar, PacketReturnType>(values, x);
    EIGEN_UNROLL_LOOP
    for (int i = 0; i < PacketSize; ++i) {
      this->m_impl.coeffRef(base + i * inner_stride) = values[i];
    }
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void storeInnerRun(Index base, Index inner_stride, const PacketReturnType& x,
                                                           std::true_type) const {
    if (inner_stride == 1) {
      this->m_impl.template writePacket<Unaligned>(base, x);
      return;
    }
    storeInnerRun(base, inner_stride, x, std::false_type());
  }

  template <int StoreMode>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void writePacket(Index index, const PacketReturnType& x) const {
    eigen_assert(index + PacketSize - 1 < this->dimensions().TotalSize());
    typedef typename Base::ImplHasPacket ImplHasPacket;
    if (this->m_is_identity) {
      storeInnerRun(index, 1, x, ImplHasPacket());
      return;
    }
    Index base;
    if (this->packetStaysInInnerRun(index, base)) {
      storeInnerRun(base, this->m_inputStrides[Base::kInnerDim], x, ImplHasPacket());
      return;
    }

    // The packet crosses an inner-run boundary, so every lane needs its own
    // index mapping.
    EIGEN_ALIGN_TO_BOUNDARY(internal::unpacket_traits<PacketReturnType>::alignment) Scalar values[PacketSize];
    internal::pstore<Scalar, PacketReturnType>(values, x);
    EIGEN_UNROLL_LOOP
    for (int i = 0; i < PacketSize; ++i) {
      this->coeffRef(index + i) = values[i];
    }
  }
};

}  // end namespace Eigen

#endif  // EIGEN_TENSOR_TENSOR_STRIDING_H
