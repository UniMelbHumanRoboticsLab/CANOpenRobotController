// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2006-2008 Benoit Jacob <jacob.benoit.1@gmail.com>
// Copyright (C) 2008-2011 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_GENERAL_PRODUCT_H
#define EIGEN_GENERAL_PRODUCT_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

enum { Large = 2, Small = 3 };

// Runtime-size threshold for falling back from the generic matrix-matrix product
// implementation (heavy) to the lightweight coeff-based product one. See
// generic_product_impl<Lhs,Rhs,DenseShape,DenseShape,GemmProduct> in
// products/GeneralMatrixMatrix.h for more details.
// The crossover belongs to the kernel the GEMM path would select, not to the
// build: a scalar type that falls back to the generic gebp kernel keeps this
// value even in an SME build. The SME kernels have their own, larger crossovers
// -- sme_gemm_to_coeffbased_threshold and
// EIGEN_SME_FIXED_SIZE_GEMM_TO_COEFFBASED_THRESHOLD below.
#ifndef EIGEN_GEMM_TO_COEFFBASED_THRESHOLD
// This default value has been obtained on a Haswell architecture.
#define EIGEN_GEMM_TO_COEFFBASED_THRESHOLD 20
#endif

// Fixed-size products can reach the GEMM product path even when the
// coeff-based evaluator is still faster. Keep this threshold separate so
// runtime-size dispatch preserves the historical heuristic above. This default
// was tuned on the same Haswell system as the runtime threshold, and deliberately
// tracks EIGEN_GEMM_TO_COEFFBASED_THRESHOLD unless specialized independently.
#ifndef EIGEN_FIXED_SIZE_GEMM_TO_COEFFBASED_THRESHOLD
#define EIGEN_FIXED_SIZE_GEMM_TO_COEFFBASED_THRESHOLD (2 * EIGEN_GEMM_TO_COEFFBASED_THRESHOLD)
#endif

#ifdef EIGEN_VECTORIZE_SME
// Measured separately rather than inherited as twice the runtime threshold: on
// SME the fixed-size crossover sits at 56, where doubling would put it at 80 and
// cost up to 3.2x on fixed-size products in the high teens and twenties. The
// optimum is again flat (48..60 within 1%). It applies only to the scalar pairs
// the SME kernel claims -- int, half, bfloat16, mixed real x complex, and
// double / complex<double> without FEAT_SME_F64F64, all run the generic kernel
// and want the generic value.
#ifndef EIGEN_SME_FIXED_SIZE_GEMM_TO_COEFFBASED_THRESHOLD
#define EIGEN_SME_FIXED_SIZE_GEMM_TO_COEFFBASED_THRESHOLD 56
#endif
#endif

// The dimension-sum bound in sme_gemm_to_coeffbased_threshold below grows with
// the depth, so it cannot reach a small output over a long depth: a 2x2 result
// at k=1024 sums to 1028 and takes the SME path with nearly the whole ZA grid
// predicated off. The crossover for that shape family is an output *area* rather
// than a dimension sum, and it tracks the scalar width.
//
// Measured on Apple M4 at SVL=512, every m x n with m,n in [2,12] against the
// coeff-based path: the largest area at which *no* shape loses is 27, 14, 15 and
// 6 for float, complex<float>, double and complex<double>. 96/sizeof (24, 12,
// 12, 6) sits just inside all four, and holds out to k=32768 -- the worst cell
// there is float 4x6 at 1.03x, i.e. neutral, while 2x2 is 13x and 3x3 2.0x.
// A flat constant does not work: the value that is safe for complex<double>
// admits only 2x2 for float, and float's would cost complex<double> up to 2x.
#ifndef EIGEN_SME_GEMM_TO_COEFFBASED_OUTPUT_AREA_THRESHOLD
#define EIGEN_SME_GEMM_TO_COEFFBASED_OUTPUT_AREA_THRESHOLD(Scalar) (96 / int(sizeof(Scalar)))
#endif

namespace internal {

#ifdef EIGEN_VECTORIZE_SME
// Defined in products/GeneralBlockPanelKernel.h, which Core includes after this
// header; only the declaration is needed to form the dependent type below.
template <typename LhsScalar, typename RhsScalar>
struct sme_has_gebp_kernel;
// Dimension sum below which the coeff-based product beats the SME GEMM kernel.
// mr x nr is sized in ZA tiles, so a product too small to fill the grid wastes
// most of the outer products the kernel issues, and the block holds fewer
// scalars as the scalar widens: the crossover sits well above the Haswell-tuned
// EIGEN_GEMM_TO_COEFFBASED_THRESHOLD, and it moves with the scalar type.
//
// First cube (m=n=k) at which the SME kernel wins, from two sweeps that each
// built the same source twice with only this threshold changed, so that the real
// dispatch selected the path in both:
//
//   scalar            M4   M4 Pro   M4 Pro, result 64-byte aligned
//   complex<double>   14   14       15
//   double            16   18       16
//   complex<float>    20   19       19
//   float             26   22       23
//
// The entries below are 3n for the largest of each row, so neither host loses at
// the crossover it gets. Over the sizes where the two disagree the paths are
// within 1.2x on both, while at n=14 a too-low bound costs 1.7x to 2.9x.
//
// Defining EIGEN_SME_GEMM_TO_COEFFBASED_THRESHOLD replaces every entry with one
// value, which is what such a sweep wants. The primary template answers with the
// generic threshold: sme_has_gebp_kernel gates the dispatch, so a scalar pair
// without an SME kernel never reads the table.
#ifdef EIGEN_SME_GEMM_TO_COEFFBASED_THRESHOLD
template <typename Scalar>
struct sme_gemm_to_coeffbased_threshold : std::integral_constant<int, EIGEN_SME_GEMM_TO_COEFFBASED_THRESHOLD> {};
#else
template <typename Scalar>
struct sme_gemm_to_coeffbased_threshold : std::integral_constant<int, EIGEN_GEMM_TO_COEFFBASED_THRESHOLD> {};
template <>
struct sme_gemm_to_coeffbased_threshold<float> : std::integral_constant<int, 78> {};
template <>
struct sme_gemm_to_coeffbased_threshold<double> : std::integral_constant<int, 54> {};
template <>
struct sme_gemm_to_coeffbased_threshold<std::complex<float> > : std::integral_constant<int, 60> {};
template <>
struct sme_gemm_to_coeffbased_threshold<std::complex<double> > : std::integral_constant<int, 45> {};
#endif
#endif

template <int Rows, int Cols, int Depth>
struct product_type_selector;

template <int Size, int MaxSize>
struct product_size_category {
  enum {
#ifndef EIGEN_GPU_COMPILE_PHASE
    is_large = MaxSize == Dynamic || Size >= EIGEN_CACHEFRIENDLY_PRODUCT_THRESHOLD ||
               (Size == Dynamic && MaxSize >= EIGEN_CACHEFRIENDLY_PRODUCT_THRESHOLD),
#else
    is_large = 0,
#endif
    value = is_large    ? Large
            : Size == 1 ? 1
                        : Small
  };
};

template <typename Lhs, typename Rhs>
struct product_type {
  using Lhs_ = remove_all_t<Lhs>;
  using Rhs_ = remove_all_t<Rhs>;
  enum {
    MaxRows = traits<Lhs_>::MaxRowsAtCompileTime,
    Rows = traits<Lhs_>::RowsAtCompileTime,
    MaxCols = traits<Rhs_>::MaxColsAtCompileTime,
    Cols = traits<Rhs_>::ColsAtCompileTime,
    MaxDepth = min_size_prefer_fixed(traits<Lhs_>::MaxColsAtCompileTime, traits<Rhs_>::MaxRowsAtCompileTime),
    Depth = min_size_prefer_fixed(traits<Lhs_>::ColsAtCompileTime, traits<Rhs_>::RowsAtCompileTime)
  };

  static constexpr int ProductType =
      product_type_selector<product_size_category<Rows, MaxRows>::value, product_size_category<Cols, MaxCols>::value,
                            product_size_category<Depth, MaxDepth>::value>::value;

  // Same rule as the runtime dispatch in products/GeneralMatrixMatrix.h: the
  // threshold follows the kernel the GEMM path would select, so a scalar pair
  // without an SME kernel keeps the generic crossover in an SME build.
  static constexpr int FixedSizeThreshold =
#ifdef EIGEN_VECTORIZE_SME
      sme_has_gebp_kernel<typename traits<Lhs_>::Scalar, typename traits<Rhs_>::Scalar>::value
          ? EIGEN_SME_FIXED_SIZE_GEMM_TO_COEFFBASED_THRESHOLD
          :
#endif
          EIGEN_FIXED_SIZE_GEMM_TO_COEFFBASED_THRESHOLD;

  static constexpr bool FixedSizeCoeffBasedProduct = ProductType == GemmProduct && Rows != Dynamic && Cols != Dynamic &&
                                                     Depth != Dynamic &&
                                                     (int(Rows) + int(Cols) + int(Depth) < FixedSizeThreshold);

  static constexpr int value = FixedSizeCoeffBasedProduct ? CoeffBasedProductMode : ProductType;
#ifdef EIGEN_DEBUG_PRODUCT
  static void debug() {
    const int rows_select = product_size_category<Rows, MaxRows>::value;
    const int cols_select = product_size_category<Cols, MaxCols>::value;
    const int depth_select = product_size_category<Depth, MaxDepth>::value;
    EIGEN_DEBUG_VAR(Rows);
    EIGEN_DEBUG_VAR(Cols);
    EIGEN_DEBUG_VAR(Depth);
    EIGEN_DEBUG_VAR(rows_select);
    EIGEN_DEBUG_VAR(cols_select);
    EIGEN_DEBUG_VAR(depth_select);
    EIGEN_DEBUG_VAR(ProductType);
    EIGEN_DEBUG_VAR(FixedSizeCoeffBasedProduct);
    EIGEN_DEBUG_VAR(value);
  }
#endif
};

/* The following allows to select the kind of product at compile time
 * based on the three dimensions of the product.
 * This is a compile time mapping from {1,Small,Large}^3 -> {product types} */
// FIXME: the current compile-time product-type mapping may not be optimal.
template <int M, int N>
struct product_type_selector<M, N, 1> : std::integral_constant<int, OuterProduct> {};
template <int M>
struct product_type_selector<M, 1, 1> : std::integral_constant<int, LazyCoeffBasedProductMode> {};
template <int N>
struct product_type_selector<1, N, 1> : std::integral_constant<int, LazyCoeffBasedProductMode> {};
template <int Depth>
struct product_type_selector<1, 1, Depth> : std::integral_constant<int, InnerProduct> {};
template <>
struct product_type_selector<1, 1, 1> : std::integral_constant<int, InnerProduct> {};
template <>
struct product_type_selector<Small, 1, Small> : std::integral_constant<int, CoeffBasedProductMode> {};
template <>
struct product_type_selector<1, Small, Small> : std::integral_constant<int, CoeffBasedProductMode> {};
template <>
struct product_type_selector<Small, Small, Small> : std::integral_constant<int, CoeffBasedProductMode> {};
template <>
struct product_type_selector<Small, Small, 1> : std::integral_constant<int, LazyCoeffBasedProductMode> {};
template <>
struct product_type_selector<Small, Large, 1> : std::integral_constant<int, LazyCoeffBasedProductMode> {};
template <>
struct product_type_selector<Large, Small, 1> : std::integral_constant<int, LazyCoeffBasedProductMode> {};
template <>
struct product_type_selector<1, Large, Small> : std::integral_constant<int, CoeffBasedProductMode> {};
template <>
struct product_type_selector<1, Large, Large> : std::integral_constant<int, GemvProduct> {};
template <>
struct product_type_selector<1, Small, Large> : std::integral_constant<int, CoeffBasedProductMode> {};
template <>
struct product_type_selector<Large, 1, Small> : std::integral_constant<int, CoeffBasedProductMode> {};
template <>
struct product_type_selector<Large, 1, Large> : std::integral_constant<int, GemvProduct> {};
template <>
struct product_type_selector<Small, 1, Large> : std::integral_constant<int, CoeffBasedProductMode> {};
template <>
struct product_type_selector<Small, Small, Large> : std::integral_constant<int, GemmProduct> {};
template <>
struct product_type_selector<Large, Small, Large> : std::integral_constant<int, GemmProduct> {};
template <>
struct product_type_selector<Small, Large, Large> : std::integral_constant<int, GemmProduct> {};
template <>
struct product_type_selector<Large, Large, Large> : std::integral_constant<int, GemmProduct> {};
template <>
struct product_type_selector<Large, Small, Small> : std::integral_constant<int, CoeffBasedProductMode> {};
template <>
struct product_type_selector<Small, Large, Small> : std::integral_constant<int, CoeffBasedProductMode> {};
template <>
struct product_type_selector<Large, Large, Small> : std::integral_constant<int, GemmProduct> {};

}  // end namespace internal

/***********************************************************************
 *  Implementation of Inner Vector Vector Product
 ***********************************************************************/

// FIXME: consider returning a Scalar instead of a 1x1 matrix for inner products.
// Pro: more natural for the user.
// Con: in a meta-unrolled algorithm a matrix-matrix product may reduce to a
// row-vector times column-vector product. To handle this, we could specialize
// Block<MatrixType,1,1> with operator=(Scalar x).

/***********************************************************************
 *  Implementation of Outer Vector Vector Product
 ***********************************************************************/

/***********************************************************************
 *  Implementation of General Matrix Vector Product
 ***********************************************************************/

/*  According to the shape/flags of the matrix we have to distinguish 3 different cases:
 *   1 - the matrix is col-major, BLAS compatible and M is large => call fast BLAS-like colmajor routine
 *   2 - the matrix is row-major, BLAS compatible and N is large => call fast BLAS-like rowmajor routine
 *   3 - all other cases are handled using a simple loop along the outer-storage direction.
 *  Therefore we need a lower level meta selector.
 *  Furthermore, if the matrix is the rhs, then the product has to be transposed.
 */
namespace internal {

template <int Side, int StorageOrder, bool BlasCompatible>
struct gemv_dense_selector;

}  // end namespace internal

namespace internal {

template <typename Scalar, int Size, int MaxSize, bool Cond>
struct gemv_static_vector_if;

template <typename Scalar, int Size, int MaxSize>
struct gemv_static_vector_if<Scalar, Size, MaxSize, false> {
  EIGEN_DEVICE_FUNC constexpr Scalar* data() {
    eigen_internal_assert(false && "should never be called");
    return 0;
  }
};

template <typename Scalar, int Size>
struct gemv_static_vector_if<Scalar, Size, Dynamic, true> {
  EIGEN_DEVICE_FUNC constexpr Scalar* data() { return 0; }
};

template <typename Scalar, int Size, int MaxSize>
struct gemv_static_vector_if<Scalar, Size, MaxSize, true> {
#if EIGEN_MAX_STATIC_ALIGN_BYTES != 0
  internal::plain_array<Scalar, internal::min_size_prefer_fixed(Size, MaxSize), 0, AlignedMax> m_data;
  constexpr Scalar* data() { return m_data.array; }
#else
  // Some architectures cannot align on the stack,
  // => let's manually enforce alignment by allocating more data and return the address of the first aligned element.
  internal::plain_array<Scalar, internal::min_size_prefer_fixed(Size, MaxSize) + EIGEN_MAX_ALIGN_BYTES, 0> m_data;
  constexpr Scalar* data() {
    return reinterpret_cast<Scalar*>((std::uintptr_t(m_data.array) & ~(std::size_t(EIGEN_MAX_ALIGN_BYTES - 1))) +
                                     EIGEN_MAX_ALIGN_BYTES);
  }
#endif
};

template <typename ResScalar>
using gemv_mapped_destination =
    Map<Matrix<ResScalar, Dynamic, 1>, plain_enum_min(AlignedMax, internal::packet_traits<ResScalar>::size)>;

template <typename Dest>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE gemv_mapped_destination<typename Dest::Scalar> gemv_construct_mapped_destination(
    Dest& dest, typename Dest::Scalar* actual_dest_ptr) {
#ifdef EIGEN_DENSE_STORAGE_CTOR_PLUGIN
  constexpr int Size = Dest::SizeAtCompileTime;
  Index size = dest.size();
  EIGEN_DENSE_STORAGE_CTOR_PLUGIN
#endif
  return gemv_mapped_destination<typename Dest::Scalar>(actual_dest_ptr, dest.size());
}

// Prepares the temporary destination shared by general, triangular, and selfadjoint GEMV kernels.
template <bool EvalToDest, typename Dest>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void gemv_prepare_destination(Dest& dest,
                                                                    typename Dest::Scalar* actual_dest_ptr) {
  EIGEN_IF_CONSTEXPR (!EvalToDest) gemv_construct_mapped_destination(dest, actual_dest_ptr) = dest;
}

template <bool EvalToDest, typename Dest>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void gemv_prepare_destination(Dest& dest, typename Dest::Scalar* actual_dest_ptr,
                                                                    bool initialize_to_zero) {
  if (initialize_to_zero) {
    gemv_construct_mapped_destination(dest, actual_dest_ptr).setZero();
  } else {
    gemv_prepare_destination<EvalToDest>(dest, actual_dest_ptr);
  }
}

template <bool EvalToDest, typename Dest>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void gemv_copy_destination(Dest& dest, typename Dest::Scalar* actual_dest_ptr) {
  EIGEN_IF_CONSTEXPR (!EvalToDest) {
    dest = gemv_mapped_destination<typename Dest::Scalar>(actual_dest_ptr, dest.size());
  }
}

// Adds complex-by-real scalar adaptation to the shared destination handling when necessary.
template <typename RhsScalar, typename ResScalar, bool EvalToDestAtCompileTime, bool ComplexByReal>
class gemv_destination_policy {
 public:
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE explicit gemv_destination_policy(const ResScalar& alpha) : alpha_(alpha) {}

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE bool eval_to_dest() const {
    return EvalToDestAtCompileTime && alpha_is_compatible();
  }

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE RhsScalar compatible_alpha() const {
    return alpha_is_compatible() ? get_factor<ResScalar, RhsScalar>::run(alpha_) : RhsScalar(1);
  }

  template <typename Dest>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void prepare(Dest& dest, ResScalar* actual_dest_ptr) const {
    gemv_prepare_destination<EvalToDestAtCompileTime>(dest, actual_dest_ptr, !alpha_is_compatible());
  }

  template <typename Dest>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void copy_back(Dest& dest, ResScalar* actual_dest_ptr) const {
    if (!alpha_is_compatible()) {
      dest.matrix() += alpha_ * gemv_mapped_destination<ResScalar>(actual_dest_ptr, dest.size());
    } else {
      gemv_copy_destination<EvalToDestAtCompileTime>(dest, actual_dest_ptr);
    }
  }

 private:
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE bool alpha_is_compatible() const {
    return !ComplexByReal || numext::is_exactly_zero(numext::imag(alpha_));
  }

  ResScalar alpha_;
};

// Copies a non-contiguous GEMV right-hand side into the aligned buffer allocated by the caller.
template <bool DirectlyUseRhs, typename ActualRhsType>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void gemv_prepare_rhs(
    const ActualRhsType& rhs, typename remove_all_t<ActualRhsType>::Scalar* actual_rhs_ptr) {
  using ActualRhsTypeCleaned = remove_all_t<ActualRhsType>;
  EIGEN_IF_CONSTEXPR (!DirectlyUseRhs) {
#ifdef EIGEN_DENSE_STORAGE_CTOR_PLUGIN
    constexpr int Size = ActualRhsTypeCleaned::SizeAtCompileTime;
    Index size = rhs.size();
    EIGEN_DENSE_STORAGE_CTOR_PLUGIN
#endif
    // The caller obtains this temporary from ei_declare_aligned_stack_constructed_variable.
    Map<typename ActualRhsTypeCleaned::PlainObject, AlignedMax>(actual_rhs_ptr, rhs.size()) = rhs;
  }
}

// The vector is on the left => transposition
template <int StorageOrder, bool BlasCompatible>
struct gemv_dense_selector<OnTheLeft, StorageOrder, BlasCompatible> {
  template <typename Lhs, typename Rhs, typename Dest>
  static void run(const Lhs& lhs, const Rhs& rhs, Dest& dest, const typename Dest::Scalar& alpha) {
    Transpose<Dest> destT(dest);
    enum { OtherStorageOrder = StorageOrder == RowMajor ? ColMajor : RowMajor };
    gemv_dense_selector<OnTheRight, OtherStorageOrder, BlasCompatible>::run(rhs.transpose(), lhs.transpose(), destT,
                                                                            alpha);
  }
};

template <>
struct gemv_dense_selector<OnTheRight, ColMajor, true> {
  template <typename Lhs, typename Rhs, typename Dest>
  static inline void run(const Lhs& lhs, const Rhs& rhs, Dest& dest, const typename Dest::Scalar& alpha) {
    using LhsScalar = typename Lhs::Scalar;
    using RhsScalar = typename Rhs::Scalar;
    using ResScalar = typename Dest::Scalar;

    using LhsBlasTraits = internal::blas_traits<Lhs>;
    using ActualLhsType = typename LhsBlasTraits::DirectLinearAccessType;
    using RhsBlasTraits = internal::blas_traits<Rhs>;
    using ActualRhsType = typename RhsBlasTraits::DirectLinearAccessType;

    ActualLhsType actualLhs = LhsBlasTraits::extract(lhs);
    ActualRhsType actualRhs = RhsBlasTraits::extract(rhs);

    ResScalar actualAlpha = combine_scalar_factors(alpha, lhs, rhs);

    // make sure Dest is a compile-time vector type (bug 1166)
    using ActualDest = std::conditional_t<Dest::IsVectorAtCompileTime, Dest, typename Dest::ColXpr>;

    enum {
      // FIXME: find a way to allow an inner stride on the result if packet_traits<Scalar>::size==1
      // on the other hand it is good for the cache to pack the vector anyways...
      EvalToDestAtCompileTime = (ActualDest::InnerStrideAtCompileTime == 1),
      ComplexByReal = (NumTraits<LhsScalar>::IsComplex) && (!NumTraits<RhsScalar>::IsComplex),
      MightCannotUseDest = ((!EvalToDestAtCompileTime) || ComplexByReal) && (ActualDest::MaxSizeAtCompileTime != 0)
    };

    using LhsMapper = const_blas_data_mapper<LhsScalar, Index, ColMajor>;
    using RhsMapper = const_blas_data_mapper<RhsScalar, Index, RowMajor>;
    EIGEN_IF_CONSTEXPR (!MightCannotUseDest) {
      // shortcut if we are sure to be able to use dest directly,
      // this eases the compiler to generate cleaner and more optimized code for most common cases
      general_matrix_vector_product<
          Index, LhsScalar, LhsMapper, ColMajor, LhsBlasTraits::NeedToConjugate, RhsScalar, RhsMapper,
          RhsBlasTraits::NeedToConjugate>::run(actualLhs.rows(), actualLhs.cols(),
                                               LhsMapper(actualLhs.data(), actualLhs.outerStride()),
                                               RhsMapper(actualRhs.data(), actualRhs.innerStride()), dest.data(), 1,
                                               get_factor<ResScalar, RhsScalar>::run(actualAlpha));
    } else {
      gemv_static_vector_if<ResScalar, ActualDest::SizeAtCompileTime, ActualDest::MaxSizeAtCompileTime,
                            MightCannotUseDest>
          static_dest;

      gemv_destination_policy<RhsScalar, ResScalar, EvalToDestAtCompileTime, ComplexByReal> destPolicy(actualAlpha);

      ei_declare_aligned_stack_constructed_variable(ResScalar, actualDestPtr, dest.size(),
                                                    destPolicy.eval_to_dest() ? dest.data() : static_dest.data());

      destPolicy.prepare(dest, actualDestPtr);

      general_matrix_vector_product<Index, LhsScalar, LhsMapper, ColMajor, LhsBlasTraits::NeedToConjugate, RhsScalar,
                                    RhsMapper, RhsBlasTraits::NeedToConjugate>::run(actualLhs.rows(), actualLhs.cols(),
                                                                                    LhsMapper(actualLhs.data(),
                                                                                              actualLhs.outerStride()),
                                                                                    RhsMapper(actualRhs.data(),
                                                                                              actualRhs.innerStride()),
                                                                                    actualDestPtr, 1,
                                                                                    destPolicy.compatible_alpha());

      destPolicy.copy_back(dest, actualDestPtr);
    }
  }
};

template <>
struct gemv_dense_selector<OnTheRight, RowMajor, true> {
  template <typename Lhs, typename Rhs, typename Dest>
  static void run(const Lhs& lhs, const Rhs& rhs, Dest& dest, const typename Dest::Scalar& alpha) {
    using LhsScalar = typename Lhs::Scalar;
    using RhsScalar = typename Rhs::Scalar;
    using ResScalar = typename Dest::Scalar;

    using LhsBlasTraits = internal::blas_traits<Lhs>;
    using ActualLhsType = typename LhsBlasTraits::DirectLinearAccessType;
    using RhsBlasTraits = internal::blas_traits<Rhs>;
    using ActualRhsType = typename RhsBlasTraits::DirectLinearAccessType;
    using ActualRhsTypeCleaned = internal::remove_all_t<ActualRhsType>;

    std::add_const_t<ActualLhsType> actualLhs = LhsBlasTraits::extract(lhs);
    std::add_const_t<ActualRhsType> actualRhs = RhsBlasTraits::extract(rhs);

    ResScalar actualAlpha = combine_scalar_factors(alpha, lhs, rhs);

    enum {
      // FIXME: find a way to allow an inner stride on the result if packet_traits<Scalar>::size==1
      // on the other hand it is good for the cache to pack the vector anyways...
      DirectlyUseRhs =
          ActualRhsTypeCleaned::InnerStrideAtCompileTime == 1 || ActualRhsTypeCleaned::MaxSizeAtCompileTime == 0
    };

    gemv_static_vector_if<RhsScalar, ActualRhsTypeCleaned::SizeAtCompileTime,
                          ActualRhsTypeCleaned::MaxSizeAtCompileTime, !DirectlyUseRhs>
        static_rhs;

    ei_declare_aligned_stack_constructed_variable(
        RhsScalar, actualRhsPtr, actualRhs.size(),
        DirectlyUseRhs ? const_cast<RhsScalar*>(actualRhs.data()) : static_rhs.data());

    gemv_prepare_rhs<DirectlyUseRhs>(actualRhs, actualRhsPtr);

    using LhsMapper = const_blas_data_mapper<LhsScalar, Index, RowMajor>;
    using RhsMapper = const_blas_data_mapper<RhsScalar, Index, ColMajor>;
    general_matrix_vector_product<Index, LhsScalar, LhsMapper, RowMajor, LhsBlasTraits::NeedToConjugate, RhsScalar,
                                  RhsMapper, RhsBlasTraits::NeedToConjugate>::
        run(actualLhs.rows(), actualLhs.cols(), LhsMapper(actualLhs.data(), actualLhs.outerStride()),
            RhsMapper(actualRhsPtr, 1), dest.data(),
            dest.col(0).innerStride(),  // NOTE  if dest is not a vector at compile-time, then dest.innerStride() might
                                        // be wrong. (bug 1166)
            actualAlpha);
  }
};

template <>
struct gemv_dense_selector<OnTheRight, ColMajor, false> {
  template <typename Lhs, typename Rhs, typename Dest>
  static void run(const Lhs& lhs, const Rhs& rhs, Dest& dest, const typename Dest::Scalar& alpha) {
    EIGEN_STATIC_ASSERT((!nested_eval<Lhs, 1>::Evaluate),
                        EIGEN_INTERNAL_COMPILATION_ERROR_OR_YOU_MADE_A_PROGRAMMING_MISTAKE);
    // TODO: if rhs is large enough it might be beneficial to make sure that dest is sequentially stored in memory,
    // otherwise use a temp
    typename nested_eval<Rhs, 1>::type actual_rhs(rhs);
    const Index size = rhs.rows();
    for (Index k = 0; k < size; ++k) dest += (alpha * actual_rhs.coeff(k)) * lhs.col(k);
  }
};

template <>
struct gemv_dense_selector<OnTheRight, RowMajor, false> {
  template <typename Lhs, typename Rhs, typename Dest>
  static void run(const Lhs& lhs, const Rhs& rhs, Dest& dest, const typename Dest::Scalar& alpha) {
    EIGEN_STATIC_ASSERT((!nested_eval<Lhs, 1>::Evaluate),
                        EIGEN_INTERNAL_COMPILATION_ERROR_OR_YOU_MADE_A_PROGRAMMING_MISTAKE);
    typename nested_eval<Rhs, Lhs::RowsAtCompileTime>::type actual_rhs(rhs);
    const Index rows = dest.rows();
    for (Index i = 0; i < rows; ++i)
      dest.coeffRef(i) += alpha * (lhs.row(i).cwiseProduct(actual_rhs.transpose())).sum();
  }
};

}  // end namespace internal

/***************************************************************************
 * Implementation of matrix base methods
 ***************************************************************************/

/** \returns the matrix product of \c *this and \a other.
 *
 * \note If instead of the matrix product you want the coefficient-wise product, see Cwise::operator*().
 *
 * \sa lazyProduct(), operator*=(const MatrixBase&), Cwise::operator*()
 */
template <typename Derived>
template <typename OtherDerived>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const Product<Derived, OtherDerived> MatrixBase<Derived>::operator*(
    const MatrixBase<OtherDerived>& other) const {
  // A note regarding the function declaration: In MSVC, this function will sometimes
  // not be inlined since DenseStorage is an unwindable object for dynamic
  // matrices and product types are holding a member to store the result.
  // Thus it does not help tagging this function with EIGEN_STRONG_INLINE.
  enum {
    ProductIsValid = Derived::ColsAtCompileTime == Dynamic || OtherDerived::RowsAtCompileTime == Dynamic ||
                     int(Derived::ColsAtCompileTime) == int(OtherDerived::RowsAtCompileTime),
    AreVectors = Derived::IsVectorAtCompileTime && OtherDerived::IsVectorAtCompileTime,
    SameSizes = EIGEN_PREDICATE_SAME_MATRIX_SIZE(Derived, OtherDerived)
  };
  // note to the lost user:
  //    * for a dot product use: v1.dot(v2)
  //    * for a coeff-wise product use: v1.cwiseProduct(v2)
  EIGEN_STATIC_ASSERT(
      ProductIsValid || !(AreVectors && SameSizes),
      INVALID_VECTOR_VECTOR_PRODUCT__IF_YOU_WANTED_A_DOT_OR_COEFF_WISE_PRODUCT_YOU_MUST_USE_THE_EXPLICIT_FUNCTIONS)
  EIGEN_STATIC_ASSERT(ProductIsValid || !(SameSizes && !AreVectors),
                      INVALID_MATRIX_PRODUCT__IF_YOU_WANTED_A_COEFF_WISE_PRODUCT_YOU_MUST_USE_THE_EXPLICIT_FUNCTION)
  EIGEN_STATIC_ASSERT(ProductIsValid || SameSizes, INVALID_MATRIX_PRODUCT)
#ifdef EIGEN_DEBUG_PRODUCT
  internal::product_type<Derived, OtherDerived>::debug();
#endif

  return Product<Derived, OtherDerived>(derived(), other.derived());
}

/** \returns an expression of the matrix product of \c *this and \a other without implicit evaluation.
 *
 * The returned product will behave like any other expressions: the coefficients of the product will be
 * computed once at a time as requested. This might be useful in some extremely rare cases when only
 * a small and no coherent fraction of the result's coefficients have to be computed.
 *
 * \warning This version of the matrix product can be much much slower. So use it only if you know
 * what you are doing and that you measured a true speed improvement.
 *
 * \sa operator*(const MatrixBase&)
 */
template <typename Derived>
template <typename OtherDerived>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const Product<Derived, OtherDerived, LazyProduct>
MatrixBase<Derived>::lazyProduct(const MatrixBase<OtherDerived>& other) const {
  enum {
    ProductIsValid = Derived::ColsAtCompileTime == Dynamic || OtherDerived::RowsAtCompileTime == Dynamic ||
                     int(Derived::ColsAtCompileTime) == int(OtherDerived::RowsAtCompileTime),
    AreVectors = Derived::IsVectorAtCompileTime && OtherDerived::IsVectorAtCompileTime,
    SameSizes = EIGEN_PREDICATE_SAME_MATRIX_SIZE(Derived, OtherDerived)
  };
  // note to the lost user:
  //    * for a dot product use: v1.dot(v2)
  //    * for a coeff-wise product use: v1.cwiseProduct(v2)
  EIGEN_STATIC_ASSERT(
      ProductIsValid || !(AreVectors && SameSizes),
      INVALID_VECTOR_VECTOR_PRODUCT__IF_YOU_WANTED_A_DOT_OR_COEFF_WISE_PRODUCT_YOU_MUST_USE_THE_EXPLICIT_FUNCTIONS)
  EIGEN_STATIC_ASSERT(ProductIsValid || !(SameSizes && !AreVectors),
                      INVALID_MATRIX_PRODUCT__IF_YOU_WANTED_A_COEFF_WISE_PRODUCT_YOU_MUST_USE_THE_EXPLICIT_FUNCTION)
  EIGEN_STATIC_ASSERT(ProductIsValid || SameSizes, INVALID_MATRIX_PRODUCT)

  return Product<Derived, OtherDerived, LazyProduct>(derived(), other.derived());
}

}  // end namespace Eigen

#endif  // EIGEN_GENERAL_PRODUCT_H
