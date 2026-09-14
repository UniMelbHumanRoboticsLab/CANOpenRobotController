// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2006-2008 Benoit Jacob <jacob.benoit.1@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_REDUX_H
#define EIGEN_REDUX_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

namespace internal {

// TODO
//  * implement other kind of vectorization
//  * factorize code

/***************************************************************************
 * Part 1 : the logic deciding a strategy for vectorization and unrolling
 ***************************************************************************/

template <typename Func, typename Evaluator>
struct redux_traits {
 public:
  using PacketType = typename find_best_packet<typename Evaluator::Scalar, Evaluator::SizeAtCompileTime>::type;
  enum {
    PacketSize = unpacket_traits<PacketType>::size,
    InnerMaxSize = int(Evaluator::IsRowMajor) ? Evaluator::MaxColsAtCompileTime : Evaluator::MaxRowsAtCompileTime,
    OuterMaxSize = int(Evaluator::IsRowMajor) ? Evaluator::MaxRowsAtCompileTime : Evaluator::MaxColsAtCompileTime,
    SliceVectorizedWork = int(InnerMaxSize) == Dynamic   ? Dynamic
                          : int(OuterMaxSize) == Dynamic ? (int(InnerMaxSize) >= int(PacketSize) ? Dynamic : 0)
                                                         : (int(InnerMaxSize) / int(PacketSize)) * int(OuterMaxSize)
  };

  enum {
    MayLinearize = (int(Evaluator::Flags) & LinearAccessBit),
    MightVectorize = (int(Evaluator::Flags) & ActualPacketAccessBit) && (functor_traits<Func>::PacketAccess),
    MayLinearVectorize = bool(MightVectorize) && bool(MayLinearize),
    MaySliceVectorize = bool(MightVectorize) && (int(SliceVectorizedWork) == Dynamic || int(SliceVectorizedWork) >= 3)
  };

 public:
  enum {
    Traversal = int(MayLinearVectorize)  ? int(LinearVectorizedTraversal)
                : int(MaySliceVectorize) ? int(SliceVectorizedTraversal)
                : int(MayLinearize)      ? int(LinearTraversal)
                                         : int(DefaultTraversal)
  };

 public:
  enum {
    Cost = Evaluator::SizeAtCompileTime == Dynamic
               ? HugeCost
               : int(Evaluator::SizeAtCompileTime) * int(Evaluator::CoeffReadCost) +
                     (Evaluator::SizeAtCompileTime - 1) * functor_traits<Func>::Cost,
    UnrollingLimit = EIGEN_UNROLLING_LIMIT * (int(Traversal) == int(DefaultTraversal) ? 1 : int(PacketSize))
  };

 public:
  enum { Unrolling = Cost <= UnrollingLimit ? CompleteUnrolling : NoUnrolling };

#ifdef EIGEN_DEBUG_ASSIGN
  static void debug() {
    std::cerr << "Xpr: " << typeid(typename Evaluator::XprType).name() << std::endl;
    std::cerr.setf(std::ios::hex, std::ios::basefield);
    EIGEN_DEBUG_VAR(Evaluator::Flags)
    std::cerr.unsetf(std::ios::hex);
    EIGEN_DEBUG_VAR(InnerMaxSize)
    EIGEN_DEBUG_VAR(OuterMaxSize)
    EIGEN_DEBUG_VAR(SliceVectorizedWork)
    EIGEN_DEBUG_VAR(PacketSize)
    EIGEN_DEBUG_VAR(MightVectorize)
    EIGEN_DEBUG_VAR(MayLinearVectorize)
    EIGEN_DEBUG_VAR(MaySliceVectorize)
    std::cerr << "Traversal"
              << " = " << Traversal << " (" << demangle_traversal(Traversal) << ")" << std::endl;
    EIGEN_DEBUG_VAR(UnrollingLimit)
    std::cerr << "Unrolling"
              << " = " << Unrolling << " (" << demangle_unrolling(Unrolling) << ")" << std::endl;
    std::cerr << std::endl;
  }
#endif
};

/***************************************************************************
 * Part 2 : unrollers
 ***************************************************************************/

/*** no vectorization ***/

template <typename Func, typename Evaluator, Index Start, Index Length>
struct redux_novec_unroller {
  static constexpr Index HalfLength = Length / 2;

  using Scalar = typename Evaluator::Scalar;

  EIGEN_DEVICE_FUNC static constexpr EIGEN_STRONG_INLINE Scalar run(const Evaluator& eval, const Func& func) {
    return func(redux_novec_unroller<Func, Evaluator, Start, HalfLength>::run(eval, func),
                redux_novec_unroller<Func, Evaluator, Start + HalfLength, Length - HalfLength>::run(eval, func));
  }
};

template <typename Func, typename Evaluator, Index Start>
struct redux_novec_unroller<Func, Evaluator, Start, 1> {
  static constexpr Index outer = Start / Evaluator::InnerSizeAtCompileTime;
  static constexpr Index inner = Start % Evaluator::InnerSizeAtCompileTime;

  using Scalar = typename Evaluator::Scalar;

  EIGEN_DEVICE_FUNC static constexpr EIGEN_STRONG_INLINE Scalar run(const Evaluator& eval, const Func&) {
    return eval.coeffByOuterInner(outer, inner);
  }
};

// This is actually dead code and will never be called. It is required
// to prevent false warnings regarding failed inlining though
// for 0 length run() will never be called at all.
template <typename Func, typename Evaluator, Index Start>
struct redux_novec_unroller<Func, Evaluator, Start, 0> {
  using Scalar = typename Evaluator::Scalar;
  EIGEN_DEVICE_FUNC static constexpr EIGEN_STRONG_INLINE Scalar run(const Evaluator&, const Func&) { return Scalar(); }
};

template <typename Func, typename Evaluator, Index Start, Index Length>
struct redux_novec_linear_unroller {
  static constexpr Index HalfLength = Length / 2;

  using Scalar = typename Evaluator::Scalar;

  EIGEN_DEVICE_FUNC static constexpr EIGEN_STRONG_INLINE Scalar run(const Evaluator& eval, const Func& func) {
    return func(redux_novec_linear_unroller<Func, Evaluator, Start, HalfLength>::run(eval, func),
                redux_novec_linear_unroller<Func, Evaluator, Start + HalfLength, Length - HalfLength>::run(eval, func));
  }
};

template <typename Func, typename Evaluator, Index Start>
struct redux_novec_linear_unroller<Func, Evaluator, Start, 1> {
  using Scalar = typename Evaluator::Scalar;

  EIGEN_DEVICE_FUNC static constexpr EIGEN_STRONG_INLINE Scalar run(const Evaluator& eval, const Func&) {
    return eval.coeff(Start);
  }
};

// This is actually dead code and will never be called. It is required
// to prevent false warnings regarding failed inlining though
// for 0 length run() will never be called at all.
template <typename Func, typename Evaluator, Index Start>
struct redux_novec_linear_unroller<Func, Evaluator, Start, 0> {
  using Scalar = typename Evaluator::Scalar;
  EIGEN_DEVICE_FUNC static constexpr EIGEN_STRONG_INLINE Scalar run(const Evaluator&, const Func&) { return Scalar(); }
};

/*** vectorization ***/

template <typename Func, typename Evaluator, Index Start, Index Length>
struct redux_vec_unroller {
  template <typename PacketType>
  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE PacketType run(const Evaluator& eval, const Func& func) {
    constexpr Index HalfLength = Length / 2;

    return func.packetOp(
        redux_vec_unroller<Func, Evaluator, Start, HalfLength>::template run<PacketType>(eval, func),
        redux_vec_unroller<Func, Evaluator, Start + HalfLength, Length - HalfLength>::template run<PacketType>(eval,
                                                                                                               func));
  }
};

template <typename Func, typename Evaluator, Index Start>
struct redux_vec_unroller<Func, Evaluator, Start, 1> {
  template <typename PacketType>
  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE PacketType run(const Evaluator& eval, const Func&) {
    constexpr Index PacketSize = unpacket_traits<PacketType>::size;
    constexpr Index index = Start * PacketSize;
    constexpr Index outer = index / int(Evaluator::InnerSizeAtCompileTime);
    constexpr Index inner = index % int(Evaluator::InnerSizeAtCompileTime);
    constexpr int alignment = Evaluator::Alignment;

    return eval.template packetByOuterInner<alignment, PacketType>(outer, inner);
  }
};

template <typename Func, typename Evaluator, Index Start, Index Length>
struct redux_vec_linear_unroller {
  template <typename PacketType>
  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE PacketType run(const Evaluator& eval, const Func& func) {
    constexpr Index HalfLength = Length / 2;

    return func.packetOp(
        redux_vec_linear_unroller<Func, Evaluator, Start, HalfLength>::template run<PacketType>(eval, func),
        redux_vec_linear_unroller<Func, Evaluator, Start + HalfLength, Length - HalfLength>::template run<PacketType>(
            eval, func));
  }
};

template <typename Func, typename Evaluator, Index Start>
struct redux_vec_linear_unroller<Func, Evaluator, Start, 1> {
  template <typename PacketType>
  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE PacketType run(const Evaluator& eval, const Func&) {
    constexpr Index PacketSize = unpacket_traits<PacketType>::size;
    constexpr Index index = (Start * PacketSize);
    constexpr int alignment = Evaluator::Alignment;
    return eval.template packet<alignment, PacketType>(index);
  }
};

/***************************************************************************
 * Part 3 : implementation of all cases
 ***************************************************************************/

template <typename Func, typename Evaluator, int Traversal = redux_traits<Func, Evaluator>::Traversal,
          int Unrolling = redux_traits<Func, Evaluator>::Unrolling>
struct redux_impl;

// Cutoffs below which the plain serial loop beats the wider unrolled bodies, measured on x86-64
// with GCC 13 and Clang 18. The linear path serves both contiguous data (vectorizes, profits
// from ~24) and strided data (loads dominate, profits only from ~64); 32 is where neither side
// loses measurably.
constexpr Index kReduxCommutativeCutoff = 32;       // independent accumulators, linear traversal
constexpr Index kReduxCommutativeInnerCutoff = 16;  // independent accumulators, outer/inner traversal
// GCC auto-vectorizes the ordered tree through a shuffle network whose setup only amortizes on
// long runs; Clang keeps it scalar, where the shorter dependency chain pays from small sizes.
constexpr Index kReduxOrderedTreeCutoff = EIGEN_COMP_GNUC_STRICT ? 192 : 16;

template <typename Func, typename Evaluator>
struct redux_impl<Func, Evaluator, DefaultTraversal, NoUnrolling> {
  using Scalar = typename Evaluator::Scalar;

  template <typename XprType>
  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE Scalar run(const Evaluator& eval, const Func& func, const XprType& xpr) {
    eigen_assert(xpr.rows() > 0 && xpr.cols() > 0 && "you are using an empty matrix");
    const Index innerSize = xpr.innerSize();
    const Index outerSize = xpr.outerSize();
    EIGEN_IF_CONSTEXPR (functor_is_commutative<Func>::value) {
      if (innerSize >= kReduxCommutativeInnerCutoff) return runCommutative(eval, func, innerSize, outerSize);
    } else {
      if (innerSize >= kReduxOrderedTreeCutoff) return runOrderedTree(eval, func, innerSize, outerSize);
    }
    Scalar res = eval.coeffByOuterInner(0, 0);
    for (Index j = 1; j < innerSize; ++j) res = func(res, eval.coeffByOuterInner(0, j));
    for (Index i = 1; i < outerSize; ++i)
      for (Index j = 0; j < innerSize; ++j) res = func(res, eval.coeffByOuterInner(i, j));
    return res;
  }

  // Commutativity lets coefficients split across eight independent accumulators: the dependency
  // chain drops to size/8 and each stride-8 stream vectorizes without cross-lane shuffles. The
  // accumulators persist across outer slices; only the ragged inner tail of each slice joins a0.
  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE Scalar runCommutative(const Evaluator& eval, const Func& func,
                                                                     Index innerSize, Index outerSize) {
    Scalar a0 = eval.coeffByOuterInner(0, 0), a1 = eval.coeffByOuterInner(0, 1);
    Scalar a2 = eval.coeffByOuterInner(0, 2), a3 = eval.coeffByOuterInner(0, 3);
    Scalar a4 = eval.coeffByOuterInner(0, 4), a5 = eval.coeffByOuterInner(0, 5);
    Scalar a6 = eval.coeffByOuterInner(0, 6), a7 = eval.coeffByOuterInner(0, 7);
    const Index unrolledEnd = innerSize - innerSize % 8;
    for (Index i = 0; i < outerSize; ++i) {
      Index j = (i == 0) ? 8 : 0;
      for (; j < unrolledEnd; j += 8) {
        a0 = func(a0, eval.coeffByOuterInner(i, j + 0));
        a1 = func(a1, eval.coeffByOuterInner(i, j + 1));
        a2 = func(a2, eval.coeffByOuterInner(i, j + 2));
        a3 = func(a3, eval.coeffByOuterInner(i, j + 3));
        a4 = func(a4, eval.coeffByOuterInner(i, j + 4));
        a5 = func(a5, eval.coeffByOuterInner(i, j + 5));
        a6 = func(a6, eval.coeffByOuterInner(i, j + 6));
        a7 = func(a7, eval.coeffByOuterInner(i, j + 7));
      }
      for (; j < innerSize; ++j) a0 = func(a0, eval.coeffByOuterInner(i, j));
    }
    return func(func(func(a0, a1), func(a2, a3)), func(func(a4, a5), func(a6, a7)));
  }

  // Associativity alone: contiguous groups of four combine in traversal order through a pairwise
  // tree, shortening the dependency chain to size/4 without reordering any operands.
  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE Scalar runOrderedTree(const Evaluator& eval, const Func& func,
                                                                     Index innerSize, Index outerSize) {
    const Index unrolledEnd = innerSize - innerSize % 4;
    Scalar res = reduce4(eval, func, 0, 0);
    for (Index i = 0; i < outerSize; ++i) {
      Index j = (i == 0) ? 4 : 0;
      for (; j < unrolledEnd; j += 4) res = func(res, reduce4(eval, func, i, j));
      for (; j < innerSize; ++j) res = func(res, eval.coeffByOuterInner(i, j));
    }
    return res;
  }

  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE Scalar reduce4(const Evaluator& eval, const Func& func, Index outer,
                                                              Index inner) {
    return func(func(eval.coeffByOuterInner(outer, inner + 0), eval.coeffByOuterInner(outer, inner + 1)),
                func(eval.coeffByOuterInner(outer, inner + 2), eval.coeffByOuterInner(outer, inner + 3)));
  }
};

template <typename Func, typename Evaluator>
struct redux_impl<Func, Evaluator, LinearTraversal, NoUnrolling> {
  using Scalar = typename Evaluator::Scalar;

  template <typename XprType>
  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE Scalar run(const Evaluator& eval, const Func& func, const XprType& xpr) {
    const Index size = xpr.size();
    eigen_assert(size > 0 && "you are using an empty matrix");
    EIGEN_IF_CONSTEXPR (functor_is_commutative<Func>::value) {
      if (size >= kReduxCommutativeCutoff) return runCommutative(eval, func, size);
    } else {
      if (size >= kReduxOrderedTreeCutoff) return runOrderedTree(eval, func, size);
    }
    Scalar res = eval.coeff(0);
    for (Index k = 1; k < size; ++k) res = func(res, eval.coeff(k));
    return res;
  }

  // Commutativity lets coefficients split across eight independent accumulators: the dependency
  // chain drops to size/8 and each stride-8 stream vectorizes without cross-lane shuffles.
  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE Scalar runCommutative(const Evaluator& eval, const Func& func,
                                                                     Index size) {
    Scalar a0 = eval.coeff(0), a1 = eval.coeff(1), a2 = eval.coeff(2), a3 = eval.coeff(3);
    Scalar a4 = eval.coeff(4), a5 = eval.coeff(5), a6 = eval.coeff(6), a7 = eval.coeff(7);
    const Index unrolledEnd = size - size % 8;
    Index k = 8;
    for (; k < unrolledEnd; k += 8) {
      a0 = func(a0, eval.coeff(k + 0));
      a1 = func(a1, eval.coeff(k + 1));
      a2 = func(a2, eval.coeff(k + 2));
      a3 = func(a3, eval.coeff(k + 3));
      a4 = func(a4, eval.coeff(k + 4));
      a5 = func(a5, eval.coeff(k + 5));
      a6 = func(a6, eval.coeff(k + 6));
      a7 = func(a7, eval.coeff(k + 7));
    }
    Scalar res = func(func(func(a0, a1), func(a2, a3)), func(func(a4, a5), func(a6, a7)));
    for (; k < size; ++k) res = func(res, eval.coeff(k));
    return res;
  }

  // Associativity alone: contiguous groups of four combine in traversal order through a pairwise
  // tree, shortening the dependency chain to size/4 without reordering any operands.
  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE Scalar runOrderedTree(const Evaluator& eval, const Func& func,
                                                                     Index size) {
    Scalar res = func(func(eval.coeff(0), eval.coeff(1)), func(eval.coeff(2), eval.coeff(3)));
    const Index unrolledEnd = size - size % 4;
    Index k = 4;
    for (; k < unrolledEnd; k += 4) {
      res = func(res, func(func(eval.coeff(k), eval.coeff(k + 1)), func(eval.coeff(k + 2), eval.coeff(k + 3))));
    }
    for (; k < size; ++k) res = func(res, eval.coeff(k));
    return res;
  }
};

template <typename Func, typename Evaluator>
struct redux_impl<Func, Evaluator, DefaultTraversal, CompleteUnrolling>
    : redux_novec_unroller<Func, Evaluator, 0, Evaluator::SizeAtCompileTime> {
  using Base = redux_novec_unroller<Func, Evaluator, 0, Evaluator::SizeAtCompileTime>;
  using Scalar = typename Evaluator::Scalar;
  template <typename XprType>
  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE Scalar run(const Evaluator& eval, const Func& func,
                                                          const XprType& /*xpr*/) {
    return Base::run(eval, func);
  }
};

template <typename Func, typename Evaluator>
struct redux_impl<Func, Evaluator, LinearTraversal, CompleteUnrolling>
    : redux_novec_linear_unroller<Func, Evaluator, 0, Evaluator::SizeAtCompileTime> {
  using Base = redux_novec_linear_unroller<Func, Evaluator, 0, Evaluator::SizeAtCompileTime>;
  using Scalar = typename Evaluator::Scalar;
  template <typename XprType>
  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE Scalar run(const Evaluator& eval, const Func& func,
                                                          const XprType& /*xpr*/) {
    return Base::run(eval, func);
  }
};

template <typename Func, typename Evaluator>
struct redux_impl<Func, Evaluator, LinearVectorizedTraversal, NoUnrolling> {
  using Scalar = typename Evaluator::Scalar;
  using PacketScalar = typename redux_traits<Func, Evaluator>::PacketType;

  template <typename XprType>
  static Scalar run(const Evaluator& eval, const Func& func, const XprType& xpr) {
    const Index size = xpr.size();

    constexpr Index packetSize = redux_traits<Func, Evaluator>::PacketSize;
    constexpr int packetAlignment = unpacket_traits<PacketScalar>::alignment;
    constexpr int alignment0 =
        (bool(Evaluator::Flags & DirectAccessBit) && bool(packet_traits<Scalar>::AlignedOnScalar))
            ? int(packetAlignment)
            : int(Unaligned);
    constexpr int alignment = plain_enum_max(alignment0, Evaluator::Alignment);
    const Index alignedStart = internal::first_default_aligned(xpr);
    const Index alignedSize2 = ((size - alignedStart) / (2 * packetSize)) * (2 * packetSize);
    const Index alignedSize = ((size - alignedStart) / (packetSize)) * (packetSize);
    const Index alignedEnd2 = alignedStart + alignedSize2;
    const Index alignedEnd = alignedStart + alignedSize;
    Scalar res;
    if (alignedSize) {
      PacketScalar packet_res0 = eval.template packet<alignment, PacketScalar>(alignedStart);
      if (alignedSize > packetSize)  // we have at least two packets to partly unroll the loop
      {
        PacketScalar packet_res1 = eval.template packet<alignment, PacketScalar>(alignedStart + packetSize);
        for (Index index = alignedStart + 2 * packetSize; index < alignedEnd2; index += 2 * packetSize) {
          packet_res0 = func.packetOp(packet_res0, eval.template packet<alignment, PacketScalar>(index));
          packet_res1 = func.packetOp(packet_res1, eval.template packet<alignment, PacketScalar>(index + packetSize));
        }

        packet_res0 = func.packetOp(packet_res0, packet_res1);
        if (alignedEnd > alignedEnd2)
          packet_res0 = func.packetOp(packet_res0, eval.template packet<alignment, PacketScalar>(alignedEnd2));
      }
      res = func.predux(packet_res0);

      for (Index index = 0; index < alignedStart; ++index) res = func(res, eval.coeff(index));

      for (Index index = alignedEnd; index < size; ++index) res = func(res, eval.coeff(index));
    } else  // too small to vectorize anything.
            // since this is dynamic-size hence inefficient anyway for such small sizes, don't try to optimize.
    {
      res = eval.coeff(0);
      for (Index index = 1; index < size; ++index) res = func(res, eval.coeff(index));
    }

    return res;
  }
};

// NOTE: for SliceVectorizedTraversal we simply bypass unrolling
template <typename Func, typename Evaluator, int Unrolling>
struct redux_impl<Func, Evaluator, SliceVectorizedTraversal, Unrolling> {
  using Scalar = typename Evaluator::Scalar;
  using PacketType = typename redux_traits<Func, Evaluator>::PacketType;

  template <typename XprType>
  EIGEN_DEVICE_FUNC static Scalar run(const Evaluator& eval, const Func& func, const XprType& xpr) {
    eigen_assert(xpr.rows() > 0 && xpr.cols() > 0 && "you are using an empty matrix");
    constexpr Index packetSize = redux_traits<Func, Evaluator>::PacketSize;
    const Index innerSize = xpr.innerSize();
    const Index outerSize = xpr.outerSize();
    const Index packetedInnerSize = ((innerSize) / packetSize) * packetSize;
    Scalar res;
    if (packetedInnerSize) {
      PacketType packet_res = eval.template packet<Unaligned, PacketType>(0, 0);
      for (Index j = 0; j < outerSize; ++j)
        for (Index i = (j == 0 ? packetSize : 0); i < packetedInnerSize; i += Index(packetSize))
          packet_res = func.packetOp(packet_res, eval.template packetByOuterInner<Unaligned, PacketType>(j, i));

      res = func.predux(packet_res);
      for (Index j = 0; j < outerSize; ++j)
        for (Index i = packetedInnerSize; i < innerSize; ++i) res = func(res, eval.coeffByOuterInner(j, i));
    } else  // too small to vectorize anything.
            // since this is dynamic-size hence inefficient anyway for such small sizes, don't try to optimize.
    {
      res = redux_impl<Func, Evaluator, DefaultTraversal, NoUnrolling>::run(eval, func, xpr);
    }

    return res;
  }
};

template <typename Func, typename Evaluator>
struct redux_impl<Func, Evaluator, LinearVectorizedTraversal, CompleteUnrolling> {
  using Scalar = typename Evaluator::Scalar;

  using PacketType = typename redux_traits<Func, Evaluator>::PacketType;
  static constexpr Index PacketSize = redux_traits<Func, Evaluator>::PacketSize;
  static constexpr Index Size = Evaluator::SizeAtCompileTime;
  static constexpr Index VectorizedSize = (int(Size) / int(PacketSize)) * int(PacketSize);

  template <typename XprType>
  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE Scalar run(const Evaluator& eval, const Func& func, const XprType& xpr) {
    EIGEN_ONLY_USED_FOR_DEBUG(xpr);
    eigen_assert(xpr.rows() > 0 && xpr.cols() > 0 && "you are using an empty matrix");
    if (VectorizedSize > 0) {
      Scalar res = func.predux(
          redux_vec_linear_unroller<Func, Evaluator, 0, Size / PacketSize>::template run<PacketType>(eval, func));
      if (VectorizedSize != Size)
        res = func(
            res, redux_novec_linear_unroller<Func, Evaluator, VectorizedSize, Size - VectorizedSize>::run(eval, func));
      return res;
    } else {
      return redux_novec_linear_unroller<Func, Evaluator, 0, Size>::run(eval, func);
    }
  }
};

// evaluator adaptor
template <typename XprType_>
class redux_evaluator : public internal::evaluator<XprType_> {
  using Base = internal::evaluator<XprType_>;

 public:
  using XprType = XprType_;
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE explicit redux_evaluator(const XprType& xpr) : Base(xpr) {}

  using Scalar = typename XprType::Scalar;
  using CoeffReturnType = typename XprType::CoeffReturnType;
  using PacketScalar = typename XprType::PacketScalar;

  enum {
    MaxRowsAtCompileTime = XprType::MaxRowsAtCompileTime,
    MaxColsAtCompileTime = XprType::MaxColsAtCompileTime,
    // TODO: we should not remove DirectAccessBit and rather find an elegant way to query the alignment offset at
    // runtime from the evaluator
    Flags = Base::Flags & ~DirectAccessBit,
    IsRowMajor = XprType::IsRowMajor,
    SizeAtCompileTime = XprType::SizeAtCompileTime,
    InnerSizeAtCompileTime = XprType::InnerSizeAtCompileTime
  };

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE CoeffReturnType coeffByOuterInner(Index outer, Index inner) const {
    return Base::coeff(IsRowMajor ? outer : inner, IsRowMajor ? inner : outer);
  }

  template <int LoadMode, typename PacketType>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE PacketType packetByOuterInner(Index outer, Index inner) const {
    return Base::template packet<LoadMode, PacketType>(IsRowMajor ? outer : inner, IsRowMajor ? inner : outer);
  }

  template <int LoadMode, typename PacketType>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE PacketType packetSegmentByOuterInner(Index outer, Index inner, Index begin,
                                                                             Index count) const {
    return Base::template packetSegment<LoadMode, PacketType>(IsRowMajor ? outer : inner, IsRowMajor ? inner : outer,
                                                              begin, count);
  }
};

// A reduction over an expression whose inner stride is not statically 1 (e.g. a dynamic-inner-stride
// Map/Ref, or a row of a dynamic matrix) falls back to a scalar traversal, because the evaluator
// drops PacketAccessBit when the inner stride is unknown at compile time. Yet such expressions are
// very often contiguous at runtime. This trait flags the cases where it is worth checking at runtime
// whether the data is contiguous and, if so, reducing it as a contiguous vector to recover full
// vectorization. We only bother when the expression has direct access, the functor and scalar are
// vectorizable, and the inner stride is not already statically 1 (otherwise it is handled directly).
template <typename Func, typename Evaluator>
struct redux_has_runtime_unit_stride_path {
  using XprType = typename Evaluator::XprType;
  using Scalar = typename Evaluator::Scalar;
  static constexpr bool value = bool(traits<XprType>::Flags & DirectAccessBit) &&
                                bool(functor_traits<Func>::PacketAccess) && bool(packet_traits<Scalar>::Vectorizable) &&
                                (int(inner_stride_at_compile_time<XprType>::value) != 1);
};

template <typename Func, typename Evaluator, typename XprType,
          bool = redux_has_runtime_unit_stride_path<Func, Evaluator>::value>
struct redux_dispatch {
  static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE typename Evaluator::Scalar run(const Evaluator& thisEval,
                                                                              const Func& func, const XprType& xpr) {
    return redux_impl<Func, Evaluator>::run(thisEval, func, xpr);
  }
};

// Runtime contiguity fast path: when the inner stride is 1 and the data is fully packed
// (a single inner panel, or no gap between inner panels), reduce the underlying buffer as a
// contiguous vector. The reduction is over all coefficients with an associative functor, so
// reducing in storage order yields the same result (up to the usual floating-point reassociation
// already inherent to vectorized reductions).
template <typename Func, typename Evaluator, typename XprType>
struct redux_dispatch<Func, Evaluator, XprType, true> {
  using Scalar = typename Evaluator::Scalar;
  static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar run(const Evaluator& thisEval, const Func& func,
                                                          const XprType& xpr) {
    if (xpr.innerStride() == 1 && (xpr.outerSize() == 1 || xpr.outerStride() == xpr.innerSize())) {
      using PlainVector = Matrix<Scalar, Dynamic, 1>;
      using MapType = Map<const PlainVector, Evaluator::Alignment>;
      MapType contiguous(xpr.data(), xpr.size());
      redux_evaluator<MapType> mapEval(contiguous);
      return redux_impl<Func, redux_evaluator<MapType>>::run(mapEval, func, contiguous);
    }
    return redux_impl<Func, Evaluator>::run(thisEval, func, xpr);
  }
};

}  // end namespace internal

/***************************************************************************
 * Part 4 : public API
 ***************************************************************************/

/** \returns the result of a full redux operation on the whole matrix or vector using \a func
 *
 * The template parameter \a BinaryOp is the type of the functor \a func which must be
 * an associative operator. Coefficients are combined in traversal order, though possibly
 * re-associated into groups. If \c Eigen::internal::functor_is_commutative<BinaryOp> is
 * specialized to derive from \c std::true_type, the implementation may also reorder operands,
 * which enables a faster reduction; Eigen's own sum, product, min and max functors opt in.
 *
 * \warning the matrix must be not empty, otherwise an assertion is triggered.
 *
 * \sa DenseBase::sum(), DenseBase::minCoeff(), DenseBase::maxCoeff(), MatrixBase::colwise(), MatrixBase::rowwise()
 */
template <typename Derived>
template <typename Func>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE typename internal::traits<Derived>::Scalar DenseBase<Derived>::redux(
    const Func& func) const {
  eigen_assert(this->rows() > 0 && this->cols() > 0 && "you are using an empty matrix");

  using ThisEvaluator = typename internal::redux_evaluator<Derived>;
  ThisEvaluator thisEval(derived());

  // The initial expression is passed to the reducer as an additional argument instead of
  // passing it as a member of redux_evaluator. redux_dispatch additionally takes a runtime
  // contiguity fast path for expressions that lose compile-time vectorization to a dynamic
  // inner stride but are contiguous at runtime (see redux_dispatch).
  return internal::redux_dispatch<Func, ThisEvaluator, Derived>::run(thisEval, func, derived());
}

/** \returns the minimum of all coefficients of \c *this.
 * In case \c *this contains NaN, NaNPropagation determines the behavior:
 *   NaNPropagation == PropagateFast : undefined
 *   NaNPropagation == PropagateNaN : result is NaN
 *   NaNPropagation == PropagateNumbers : result is minimum of elements that are not NaN
 * \warning the matrix must be not empty, otherwise an assertion is triggered.
 */
template <typename Derived>
template <int NaNPropagation>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE typename internal::traits<Derived>::Scalar DenseBase<Derived>::minCoeff() const {
  return derived().redux(Eigen::internal::scalar_min_op<Scalar, Scalar, NaNPropagation>());
}

/** \returns the maximum of all coefficients of \c *this.
 * In case \c *this contains NaN, NaNPropagation determines the behavior:
 *   NaNPropagation == PropagateFast : undefined
 *   NaNPropagation == PropagateNaN : result is NaN
 *   NaNPropagation == PropagateNumbers : result is maximum of elements that are not NaN
 * \warning the matrix must be not empty, otherwise an assertion is triggered.
 */
template <typename Derived>
template <int NaNPropagation>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE typename internal::traits<Derived>::Scalar DenseBase<Derived>::maxCoeff() const {
  return derived().redux(Eigen::internal::scalar_max_op<Scalar, Scalar, NaNPropagation>());
}

/** \returns the sum of all coefficients of \c *this
 *
 * If \c *this is empty, then the value 0 is returned.
 *
 * \sa trace(), prod(), mean()
 */
template <typename Derived>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE typename internal::traits<Derived>::Scalar DenseBase<Derived>::sum() const {
  if (SizeAtCompileTime == 0 || (SizeAtCompileTime == Dynamic && size() == 0)) return Scalar(0);
  return derived().redux(Eigen::internal::scalar_sum_op<Scalar, Scalar>());
}

/** \returns the mean of all coefficients of *this
 *
 * \sa trace(), prod(), sum()
 */
template <typename Derived>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE typename internal::traits<Derived>::Scalar DenseBase<Derived>::mean() const {
#ifdef __INTEL_COMPILER
#pragma warning push
#pragma warning(disable : 2259)
#endif
  return Scalar(derived().redux(Eigen::internal::scalar_sum_op<Scalar, Scalar>())) / Scalar(this->size());
#ifdef __INTEL_COMPILER
#pragma warning pop
#endif
}

/** \returns the product of all coefficients of *this
 *
 * Example: \include MatrixBase_prod.cpp
 * Output: \verbinclude MatrixBase_prod.out
 *
 * \sa sum(), mean(), trace()
 */
template <typename Derived>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE typename internal::traits<Derived>::Scalar DenseBase<Derived>::prod() const {
  if (SizeAtCompileTime == 0 || (SizeAtCompileTime == Dynamic && size() == 0)) return Scalar(1);
  return derived().redux(Eigen::internal::scalar_product_op<Scalar>());
}

/** \returns the trace of \c *this, i.e. the sum of the coefficients on the main diagonal.
 *
 * \c *this can be any matrix, not necessarily square.
 *
 * \sa diagonal(), sum()
 */
template <typename Derived>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE typename internal::traits<Derived>::Scalar MatrixBase<Derived>::trace() const {
  return derived().diagonal().sum();
}

}  // end namespace Eigen

#endif  // EIGEN_REDUX_H
