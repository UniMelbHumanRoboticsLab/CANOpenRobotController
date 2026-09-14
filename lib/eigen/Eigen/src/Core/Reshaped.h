// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2017 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2014 yoco <peter.xiau@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_RESHAPED_H
#define EIGEN_RESHAPED_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

/** \class Reshaped
 * \ingroup Core_Module
 *
 * \brief Expression of a fixed-size or dynamic-size reshape
 *
 * \tparam XprType the type of the expression in which we are taking a reshape
 * \tparam Rows the number of rows of the reshape we are taking at compile time (optional)
 * \tparam Cols the number of columns of the reshape we are taking at compile time (optional)
 * \tparam Order can be ColMajor or RowMajor, default is ColMajor.
 *
 * This class represents an expression of either a fixed-size or dynamic-size reshape.
 * It is the return type of DenseBase::reshaped(NRowsType,NColsType) and
 * most of the time this is the only way it is used.
 *
 * If you want to directly manipulate reshaped expressions,
 * for instance if you want to write a function returning such an expression,
 * it is advised to use the \em auto keyword for such use cases.
 *
 * Here is an example illustrating the dynamic case:
 * \include class_Reshaped.cpp
 * Output: \verbinclude class_Reshaped.out
 *
 * Here is an example illustrating the fixed-size case:
 * \include class_FixedReshaped.cpp
 * Output: \verbinclude class_FixedReshaped.out
 *
 * \sa DenseBase::reshaped(NRowsType,NColsType)
 */

namespace internal {

template <typename XprType, int Rows, int Cols, int Order>
struct traits<Reshaped<XprType, Rows, Cols, Order> > : traits<XprType> {
  using Scalar = typename traits<XprType>::Scalar;
  using StorageKind = typename traits<XprType>::StorageKind;
  using XprKind = typename traits<XprType>::XprKind;
  enum {
    MatrixRows = traits<XprType>::RowsAtCompileTime,
    MatrixCols = traits<XprType>::ColsAtCompileTime,
    RowsAtCompileTime = Rows,
    ColsAtCompileTime = Cols,
    MaxRowsAtCompileTime = Rows,
    MaxColsAtCompileTime = Cols,
    XprStorageOrder = ((int(traits<XprType>::Flags) & RowMajorBit) == RowMajorBit) ? RowMajor : ColMajor,
    ReshapedStorageOrder = (RowsAtCompileTime == 1 && ColsAtCompileTime != 1)   ? RowMajor
                           : (ColsAtCompileTime == 1 && RowsAtCompileTime != 1) ? ColMajor
                                                                                : XprStorageOrder,
    HasSameStorageOrderAsXprType = (ReshapedStorageOrder == XprStorageOrder),
    InnerSize = (ReshapedStorageOrder == int(RowMajor)) ? int(ColsAtCompileTime) : int(RowsAtCompileTime),
    // A mismatched ReshapedStorageOrder only happens for vector shapes, where the storage order is
    // immaterial: the runtime innerStride() is the nested expression's in all cases (see below).
    InnerStrideAtCompileTime = (HasSameStorageOrderAsXprType || RowsAtCompileTime == 1 || ColsAtCompileTime == 1)
                                   ? int(inner_stride_at_compile_time<XprType>::value)
                                   : Dynamic,
    OuterStrideAtCompileTime = Dynamic,

    HasDirectAccess = internal::has_direct_access<XprType>::value && (Order == int(XprStorageOrder)) &&
                      ((evaluator<XprType>::Flags & LinearAccessBit) == LinearAccessBit),

    MaskPacketAccessBit =
        (InnerSize == Dynamic || (InnerSize % packet_traits<Scalar>::size) == 0) && (InnerStrideAtCompileTime == 1)
            ? PacketAccessBit
            : 0,
    // MaskAlignedBit = ((OuterStrideAtCompileTime!=Dynamic) && (((OuterStrideAtCompileTime * int(sizeof(Scalar))) % 16)
    // == 0)) ? AlignedBit : 0,
    FlagsLinearAccessBit = (RowsAtCompileTime == 1 || ColsAtCompileTime == 1) ? LinearAccessBit : 0,
    FlagsLvalueBit = is_lvalue<XprType>::value ? LvalueBit : 0,
    FlagsRowMajorBit = (ReshapedStorageOrder == int(RowMajor)) ? RowMajorBit : 0,
    FlagsDirectAccessBit = HasDirectAccess ? DirectAccessBit : 0,
    Flags0 = traits<XprType>::Flags & ((HereditaryBits & ~RowMajorBit) | MaskPacketAccessBit),

    Flags = (Flags0 | FlagsLinearAccessBit | FlagsLvalueBit | FlagsRowMajorBit | FlagsDirectAccessBit)
  };
};

template <typename XprType, int Rows, int Cols, int Order, bool HasDirectAccess>
class ReshapedImpl_dense;

}  // end namespace internal

template <typename XprType, int Rows, int Cols, int Order, typename StorageKind>
class ReshapedImpl;

template <typename XprType, int Rows, int Cols, int Order>
class Reshaped : public ReshapedImpl<XprType, Rows, Cols, Order, typename internal::traits<XprType>::StorageKind> {
  using Impl = ReshapedImpl<XprType, Rows, Cols, Order, typename internal::traits<XprType>::StorageKind>;

 public:
  // typedef typename Impl::Base Base;
  using Base = Impl;
  EIGEN_GENERIC_PUBLIC_INTERFACE(Reshaped)
  EIGEN_INHERIT_ASSIGNMENT_OPERATORS(Reshaped)

  /** Fixed-size constructor
   */
  EIGEN_DEVICE_FUNC constexpr inline Reshaped(XprType& xpr) : Impl(xpr) {
    EIGEN_STATIC_ASSERT(RowsAtCompileTime != Dynamic && ColsAtCompileTime != Dynamic,
                        THIS_METHOD_IS_ONLY_FOR_FIXED_SIZE)
    eigen_assert(Rows * Cols == xpr.rows() * xpr.cols());
  }

  /** Dynamic-size constructor
   */
  EIGEN_DEVICE_FUNC constexpr inline Reshaped(XprType& xpr, Index reshapeRows, Index reshapeCols)
      : Impl(xpr, reshapeRows, reshapeCols) {
    eigen_assert((RowsAtCompileTime == Dynamic || RowsAtCompileTime == reshapeRows) &&
                 (ColsAtCompileTime == Dynamic || ColsAtCompileTime == reshapeCols));
    eigen_assert(reshapeRows * reshapeCols == xpr.rows() * xpr.cols());
  }
};

// The generic default implementation for dense reshape simply forward to the internal::ReshapedImpl_dense
// that must be specialized for direct and non-direct access...
template <typename XprType, int Rows, int Cols, int Order>
class ReshapedImpl<XprType, Rows, Cols, Order, Dense>
    : public internal::ReshapedImpl_dense<XprType, Rows, Cols, Order,
                                          internal::traits<Reshaped<XprType, Rows, Cols, Order> >::HasDirectAccess> {
  using Impl = internal::ReshapedImpl_dense<XprType, Rows, Cols, Order,
                                            internal::traits<Reshaped<XprType, Rows, Cols, Order>>::HasDirectAccess>;

 public:
  using Base = Impl;
  EIGEN_INHERIT_ASSIGNMENT_OPERATORS(ReshapedImpl)
  EIGEN_DEVICE_FUNC constexpr inline ReshapedImpl(XprType& xpr) : Impl(xpr) {}
  EIGEN_DEVICE_FUNC constexpr inline ReshapedImpl(XprType& xpr, Index reshapeRows, Index reshapeCols)
      : Impl(xpr, reshapeRows, reshapeCols) {}
};

namespace internal {

/** \internal Internal implementation of dense Reshaped in the general case. */
template <typename XprType, int Rows, int Cols, int Order>
class ReshapedImpl_dense<XprType, Rows, Cols, Order, false>
    : public internal::dense_xpr_base<Reshaped<XprType, Rows, Cols, Order> >::type {
  using ReshapedType = Reshaped<XprType, Rows, Cols, Order>;

 public:
  using Base = typename internal::dense_xpr_base<ReshapedType>::type;
  EIGEN_DENSE_PUBLIC_INTERFACE(ReshapedType)
  EIGEN_INHERIT_ASSIGNMENT_OPERATORS(ReshapedImpl_dense)

  using MatrixTypeNested = typename internal::ref_selector<XprType>::non_const_type;
  using NestedExpression = internal::remove_all_t<XprType>;

  class InnerIterator;

  /** Fixed-size constructor
   */
  EIGEN_DEVICE_FUNC constexpr inline ReshapedImpl_dense(XprType& xpr) : m_xpr(xpr), m_rows(Rows), m_cols(Cols) {}

  /** Dynamic-size constructor
   */
  EIGEN_DEVICE_FUNC constexpr inline ReshapedImpl_dense(XprType& xpr, Index nRows, Index nCols)
      : m_xpr(xpr), m_rows(nRows), m_cols(nCols) {}

  EIGEN_DEVICE_FUNC constexpr Index rows() const { return m_rows; }
  EIGEN_DEVICE_FUNC constexpr Index cols() const { return m_cols; }

#ifdef EIGEN_PARSED_BY_DOXYGEN
  /** \sa MapBase::data() */
  EIGEN_DEVICE_FUNC constexpr const Scalar* data() const;
  EIGEN_DEVICE_FUNC inline Index innerStride() const;
  EIGEN_DEVICE_FUNC inline Index outerStride() const;
#endif

  /** \returns the nested expression */
  EIGEN_DEVICE_FUNC constexpr const internal::remove_all_t<XprType>& nestedExpression() const { return m_xpr; }

  /** \returns the nested expression */
  EIGEN_DEVICE_FUNC constexpr std::remove_reference_t<XprType>& nestedExpression() { return m_xpr; }

 protected:
  MatrixTypeNested m_xpr;
  const internal::variable_if_dynamic<Index, Rows> m_rows;
  const internal::variable_if_dynamic<Index, Cols> m_cols;
};

/** \internal Internal implementation of dense Reshaped in the direct access case. */
template <typename XprType, int Rows, int Cols, int Order>
class ReshapedImpl_dense<XprType, Rows, Cols, Order, true> : public MapBase<Reshaped<XprType, Rows, Cols, Order> > {
  using ReshapedType = Reshaped<XprType, Rows, Cols, Order>;
  using XprTypeNested = typename internal::ref_selector<XprType>::non_const_type;

 public:
  using Base = MapBase<ReshapedType>;
  EIGEN_DENSE_PUBLIC_INTERFACE(ReshapedType)
  EIGEN_INHERIT_ASSIGNMENT_OPERATORS(ReshapedImpl_dense)

  /** Fixed-size constructor
   */
  EIGEN_DEVICE_FUNC constexpr inline ReshapedImpl_dense(XprType& xpr) : Base(xpr.data()), m_xpr(xpr) {}

  /** Dynamic-size constructor
   */
  EIGEN_DEVICE_FUNC constexpr inline ReshapedImpl_dense(XprType& xpr, Index nRows, Index nCols)
      : Base(xpr.data(), nRows, nCols), m_xpr(xpr) {}

  EIGEN_DEVICE_FUNC constexpr const internal::remove_all_t<XprTypeNested>& nestedExpression() const { return m_xpr; }

  EIGEN_DEVICE_FUNC constexpr XprType& nestedExpression() { return m_xpr; }

  /** \sa MapBase::innerStride() */
  EIGEN_DEVICE_FUNC constexpr Index innerStride() const { return m_xpr.innerStride(); }

  /** \sa MapBase::outerStride() */
  EIGEN_DEVICE_FUNC constexpr Index outerStride() const {
    return (((Flags & RowMajorBit) == RowMajorBit) ? this->cols() : this->rows()) * m_xpr.innerStride();
  }

 protected:
  XprTypeNested m_xpr;
};

// Evaluators
template <typename ArgType, int Rows, int Cols, int Order, bool HasDirectAccess>
struct reshaped_evaluator;

template <typename ArgType, int Rows, int Cols, int Order>
struct evaluator<Reshaped<ArgType, Rows, Cols, Order> >
    : reshaped_evaluator<ArgType, Rows, Cols, Order, traits<Reshaped<ArgType, Rows, Cols, Order> >::HasDirectAccess> {
  using XprType = Reshaped<ArgType, Rows, Cols, Order>;
  using Scalar = typename XprType::Scalar;
  // TODO: should check for smaller packet types
  using PacketScalar = typename packet_traits<Scalar>::type;

  enum {
    CoeffReadCost = evaluator<ArgType>::CoeffReadCost,
    HasDirectAccess = traits<XprType>::HasDirectAccess,

    //     RowsAtCompileTime = traits<XprType>::RowsAtCompileTime,
    //     ColsAtCompileTime = traits<XprType>::ColsAtCompileTime,
    //     MaxRowsAtCompileTime = traits<XprType>::MaxRowsAtCompileTime,
    //     MaxColsAtCompileTime = traits<XprType>::MaxColsAtCompileTime,
    //
    //     InnerStrideAtCompileTime = traits<XprType>::HasSameStorageOrderAsXprType
    //                              ? int(inner_stride_at_compile_time<ArgType>::value)
    //                              : Dynamic,
    //     OuterStrideAtCompileTime = Dynamic,

    // Whether the coeff-based specialization below serves all accesses by forwarding the nested
    // evaluator's own linear accesses; always false for the direct-access specialization.
    ForwardLinearAccess = reshaped_evaluator<ArgType, Rows, Cols, Order, bool(HasDirectAccess)>::ForwardLinearAccess,

    FlagsLinearAccessBit = (traits<XprType>::RowsAtCompileTime == 1 || traits<XprType>::ColsAtCompileTime == 1 ||
                            HasDirectAccess || ForwardLinearAccess)
                               ? LinearAccessBit
                               : 0,
    FlagsRowMajorBit = (traits<XprType>::ReshapedStorageOrder == int(RowMajor)) ? RowMajorBit : 0,
    FlagsDirectAccessBit = HasDirectAccess ? DirectAccessBit : 0,
    // A direct-access reshape with unit inner stride is the nested expression's buffer, contiguous
    // from data(), so the mapbase_evaluator packet paths apply whenever the nested evaluator's do.
    // A forwarding coeff-based reshape serves the nested evaluator's own packets directly.
    MaskPacketAccessBit = (HasDirectAccess && (traits<XprType>::InnerStrideAtCompileTime == 1)) || ForwardLinearAccess
                              ? PacketAccessBit
                              : 0,
    Flags0 = evaluator<ArgType>::Flags & ((HereditaryBits & ~RowMajorBit) | MaskPacketAccessBit),
    Flags = Flags0 | FlagsLinearAccessBit | FlagsRowMajorBit | FlagsDirectAccessBit,

    PacketAlignment = unpacket_traits<PacketScalar>::alignment,
    // The view starts at the nested data() with no offset (direct access) or forwards the nested
    // evaluator's accesses element-for-element, so its alignment carries over.
    Alignment = evaluator<ArgType>::Alignment
  };
  using reshaped_evaluator_type = reshaped_evaluator<ArgType, Rows, Cols, Order, HasDirectAccess>;
  EIGEN_DEVICE_FUNC constexpr explicit evaluator(const XprType& xpr) : reshaped_evaluator_type(xpr) {
    EIGEN_INTERNAL_CHECK_COST_VALUE(CoeffReadCost);
  }
};

template <typename ArgType, int Rows, int Cols, int Order>
struct reshaped_evaluator<ArgType, Rows, Cols, Order, /* HasDirectAccess */ false>
    : evaluator_base<Reshaped<ArgType, Rows, Cols, Order> > {
  using XprType = Reshaped<ArgType, Rows, Cols, Order>;

  enum {
    CoeffReadCost = evaluator<ArgType>::CoeffReadCost /* TODO + cost of index computations */,

    // The reshape enumerates the nested expression's elements in `Order`. When the nested
    // evaluator's linear enumeration follows the same order -- its storage order matches, or it is
    // vector-shaped so the order is immaterial -- the n-th reshaped element is the n-th nested
    // element and every access forwards linearly, with no division/modulo index remapping.
    NestedRowMajor = (int(evaluator<ArgType>::Flags) & RowMajorBit) != 0,
    OrderMatchesNested = (Order == int(ColMajor)) != NestedRowMajor,
    ForwardLinearAccess = (OrderMatchesNested || ArgType::RowsAtCompileTime == 1 || ArgType::ColsAtCompileTime == 1) &&
                          ((int(evaluator<ArgType>::Flags) & LinearAccessBit) != 0)

    // Flags and Alignment are defined by evaluator<Reshaped>, which derives from this evaluator.
  };

  EIGEN_DEVICE_FUNC constexpr explicit reshaped_evaluator(const XprType& xpr)
      : m_argImpl(xpr.nestedExpression()), m_xpr(xpr) {
    EIGEN_INTERNAL_CHECK_COST_VALUE(CoeffReadCost);
  }

  using Scalar = typename XprType::Scalar;
  using CoeffReturnType = typename XprType::CoeffReturnType;

  using RowCol = std::pair<Index, Index>;

  // The n-th element of the reshape in `Order` enumeration; under ForwardLinearAccess this is also
  // the nested evaluator's linear index of that element.
  EIGEN_DEVICE_FUNC constexpr EIGEN_STRONG_INLINE Index linear_index(Index rowId, Index colId) const {
    EIGEN_IF_CONSTEXPR (Order == ColMajor) {
      return colId * m_xpr.rows() + rowId;
    } else {
      return colId + rowId * m_xpr.cols();
    }
  }

  EIGEN_DEVICE_FUNC constexpr inline RowCol index_remap(Index rowId, Index colId) const {
    const Index nth_elem_idx = linear_index(rowId, colId);
    EIGEN_IF_CONSTEXPR (Order == ColMajor) {
      return RowCol(nth_elem_idx % m_xpr.nestedExpression().rows(), nth_elem_idx / m_xpr.nestedExpression().rows());
    } else {
      return RowCol(nth_elem_idx / m_xpr.nestedExpression().cols(), nth_elem_idx % m_xpr.nestedExpression().cols());
    }
  }

  EIGEN_DEVICE_FUNC constexpr inline Scalar& coeffRef(Index rowId, Index colId) {
    EIGEN_STATIC_ASSERT_LVALUE(XprType)
    return coeffRef_impl(rowId, colId, bool_constant<ForwardLinearAccess>());
  }

  EIGEN_DEVICE_FUNC constexpr inline const Scalar& coeffRef(Index rowId, Index colId) const {
    return coeffRef_impl(rowId, colId, bool_constant<ForwardLinearAccess>());
  }

  EIGEN_DEVICE_FUNC constexpr EIGEN_STRONG_INLINE const CoeffReturnType coeff(Index rowId, Index colId) const {
    return coeff_impl(rowId, colId, bool_constant<ForwardLinearAccess>());
  }

  EIGEN_DEVICE_FUNC constexpr inline Scalar& coeffRef(Index index) {
    EIGEN_STATIC_ASSERT_LVALUE(XprType)
    return coeffRef_impl(index, bool_constant<ForwardLinearAccess>());
  }

  EIGEN_DEVICE_FUNC constexpr inline const Scalar& coeffRef(Index index) const {
    return coeffRef_impl(index, bool_constant<ForwardLinearAccess>());
  }

  EIGEN_DEVICE_FUNC constexpr inline const CoeffReturnType coeff(Index index) const {
    return coeff_impl(index, bool_constant<ForwardLinearAccess>());
  }

  // The packet paths are advertised only under ForwardLinearAccess (see evaluator<Reshaped>), so
  // they forward the nested evaluator's linear packets without index remapping.
  template <int LoadMode, typename PacketType>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE PacketType packet(Index rowId, Index colId) const {
    return m_argImpl.template packet<LoadMode, PacketType>(linear_index(rowId, colId));
  }

  template <int LoadMode, typename PacketType>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE PacketType packet(Index index) const {
    return m_argImpl.template packet<LoadMode, PacketType>(index);
  }

  template <int StoreMode, typename PacketType>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void writePacket(Index rowId, Index colId, const PacketType& x) {
    m_argImpl.template writePacket<StoreMode, PacketType>(linear_index(rowId, colId), x);
  }

  template <int StoreMode, typename PacketType>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void writePacket(Index index, const PacketType& x) {
    m_argImpl.template writePacket<StoreMode, PacketType>(index, x);
  }

  template <int LoadMode, typename PacketType>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE PacketType packetSegment(Index rowId, Index colId, Index begin,
                                                                 Index count) const {
    return m_argImpl.template packetSegment<LoadMode, PacketType>(linear_index(rowId, colId), begin, count);
  }

  template <int LoadMode, typename PacketType>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE PacketType packetSegment(Index index, Index begin, Index count) const {
    return m_argImpl.template packetSegment<LoadMode, PacketType>(index, begin, count);
  }

  template <int StoreMode, typename PacketType>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void writePacketSegment(Index rowId, Index colId, const PacketType& x,
                                                                Index begin, Index count) {
    m_argImpl.template writePacketSegment<StoreMode, PacketType>(linear_index(rowId, colId), x, begin, count);
  }

  template <int StoreMode, typename PacketType>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void writePacketSegment(Index index, const PacketType& x, Index begin,
                                                                Index count) {
    m_argImpl.template writePacketSegment<StoreMode, PacketType>(index, x, begin, count);
  }

 protected:
  // Linear-access members map an index onto the vector shape's single row or column.
  EIGEN_DEVICE_FUNC static constexpr Index vector_row(Index index) { return Rows == 1 ? 0 : index; }
  EIGEN_DEVICE_FUNC static constexpr Index vector_col(Index index) { return Rows == 1 ? index : 0; }

  EIGEN_DEVICE_FUNC constexpr EIGEN_STRONG_INLINE const CoeffReturnType
  coeff_impl(Index index, std::true_type /* ForwardLinearAccess */) const {
    // The one-dimensional index already follows the nested evaluator's linear enumeration.
    return m_argImpl.coeff(index);
  }

  EIGEN_DEVICE_FUNC constexpr EIGEN_STRONG_INLINE const CoeffReturnType
  coeff_impl(Index index, std::false_type /* not ForwardLinearAccess */) const {
    return coeff_impl(vector_row(index), vector_col(index), std::false_type());
  }

  EIGEN_DEVICE_FUNC constexpr EIGEN_STRONG_INLINE const CoeffReturnType
  coeff_impl(Index rowId, Index colId, std::true_type /* ForwardLinearAccess */) const {
    return m_argImpl.coeff(linear_index(rowId, colId));
  }

  EIGEN_DEVICE_FUNC constexpr EIGEN_STRONG_INLINE const CoeffReturnType
  coeff_impl(Index rowId, Index colId, std::false_type /* not ForwardLinearAccess */) const {
    const RowCol row_col = index_remap(rowId, colId);
    return m_argImpl.coeff(row_col.first, row_col.second);
  }

  EIGEN_DEVICE_FUNC constexpr EIGEN_STRONG_INLINE Scalar& coeffRef_impl(Index index,
                                                                        std::true_type /* ForwardLinearAccess */) {
    return m_argImpl.coeffRef(index);
  }

  EIGEN_DEVICE_FUNC constexpr EIGEN_STRONG_INLINE Scalar& coeffRef_impl(Index index,
                                                                        std::false_type /* not ForwardLinearAccess */) {
    return coeffRef_impl(vector_row(index), vector_col(index), std::false_type());
  }

  EIGEN_DEVICE_FUNC constexpr EIGEN_STRONG_INLINE Scalar& coeffRef_impl(Index rowId, Index colId,
                                                                        std::true_type /* ForwardLinearAccess */) {
    return m_argImpl.coeffRef(linear_index(rowId, colId));
  }

  EIGEN_DEVICE_FUNC constexpr EIGEN_STRONG_INLINE Scalar& coeffRef_impl(Index rowId, Index colId,
                                                                        std::false_type /* not ForwardLinearAccess */) {
    const RowCol row_col = index_remap(rowId, colId);
    return m_argImpl.coeffRef(row_col.first, row_col.second);
  }

  EIGEN_DEVICE_FUNC constexpr EIGEN_STRONG_INLINE const Scalar& coeffRef_impl(
      Index index, std::true_type /* ForwardLinearAccess */) const {
    return m_argImpl.coeffRef(index);
  }

  EIGEN_DEVICE_FUNC constexpr EIGEN_STRONG_INLINE const Scalar& coeffRef_impl(
      Index index, std::false_type /* not ForwardLinearAccess */) const {
    return coeffRef_impl(vector_row(index), vector_col(index), std::false_type());
  }

  EIGEN_DEVICE_FUNC constexpr EIGEN_STRONG_INLINE const Scalar& coeffRef_impl(
      Index rowId, Index colId, std::true_type /* ForwardLinearAccess */) const {
    return m_argImpl.coeffRef(linear_index(rowId, colId));
  }

  EIGEN_DEVICE_FUNC constexpr EIGEN_STRONG_INLINE const Scalar& coeffRef_impl(
      Index rowId, Index colId, std::false_type /* not ForwardLinearAccess */) const {
    const RowCol row_col = index_remap(rowId, colId);
    return m_argImpl.coeffRef(row_col.first, row_col.second);
  }

  evaluator<ArgType> m_argImpl;
  const XprType& m_xpr;
};

template <typename ArgType, int Rows, int Cols, int Order>
struct reshaped_evaluator<ArgType, Rows, Cols, Order, /* HasDirectAccess */ true>
    : mapbase_evaluator<Reshaped<ArgType, Rows, Cols, Order>,
                        typename Reshaped<ArgType, Rows, Cols, Order>::PlainObject> {
  using XprType = Reshaped<ArgType, Rows, Cols, Order>;
  using Scalar = typename XprType::Scalar;

  // Packets come from the mapbase_evaluator machinery, not from linear forwarding.
  enum { ForwardLinearAccess = false };

  EIGEN_DEVICE_FUNC constexpr explicit reshaped_evaluator(const XprType& xpr)
      : mapbase_evaluator<XprType, typename XprType::PlainObject>(xpr) {
    eigen_assert(((std::uintptr_t(xpr.data()) % plain_enum_max(1, evaluator<XprType>::Alignment)) == 0) &&
                 "data is not aligned");
  }
};

}  // end namespace internal

}  // end namespace Eigen

#endif  // EIGEN_RESHAPED_H
