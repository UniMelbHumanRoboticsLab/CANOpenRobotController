// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2006-2008, 2010 Benoit Jacob <jacob.benoit.1@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_DOT_H
#define EIGEN_DOT_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

namespace internal {

// Accumulate low-precision norms in float without changing the public result type.
template <typename RealScalar>
struct stable_norm_accumulator {
  using type = RealScalar;
};

template <>
struct stable_norm_accumulator<half> {
  using type = float;
};

template <>
struct stable_norm_accumulator<bfloat16> {
  using type = float;
};

template <typename RealScalar, typename Accumulator>
struct stable_normalization_normal_min {
  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE Accumulator run() {
    return static_cast<Accumulator>((numext::numeric_limits<RealScalar>::min)());
  }
};

// The half and bfloat16 numeric_limits functions are not device functions.
template <typename Accumulator>
struct stable_normalization_normal_min<half, Accumulator> {
  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE Accumulator run() { return Accumulator(1) / Accumulator(16384); }
};

template <typename Accumulator>
struct stable_normalization_normal_min<bfloat16, Accumulator> {
  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE Accumulator run() {
    return static_cast<Accumulator>((numext::numeric_limits<float>::min)());
  }
};

template <typename RealScalar, typename Accumulator>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE bool stable_normalization_inv_scale(const Accumulator& value,
                                                                          Accumulator& invScale) {
  safe_scaling_factors<Accumulator> factors;
  const Accumulator normalMin = stable_normalization_normal_min<RealScalar, Accumulator>::run();
  if (!safe_scaling<Accumulator>::try_compute_ceiling_factors_with_normal_reciprocal(value, normalMin, factors))
    return false;
  invScale = factors.invScale;
  return true;
}

template <typename Accumulator>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE bool stable_normalization_combined_factor(const Accumulator& invScale,
                                                                                const Accumulator& sqrtNorm,
                                                                                Accumulator& factor) {
  // Check before dividing: an overflowing reciprocal can raise FE_OVERFLOW even if the two-step fallback is used.
  if (sqrtNorm < Accumulator(1) && invScale > Accumulator(NumTraits<Accumulator>::highest()) * sqrtNorm) return false;
  factor = invScale / sqrtNorm;
  return factor >= stable_normalization_normal_min<Accumulator, Accumulator>::run();
}

template <typename VectorType, typename Accumulator,
          bool = bool(traits<VectorType>::Flags & DirectAccessBit) &&
                 (int(inner_stride_at_compile_time<VectorType>::value) != 1)>
struct stable_normalization_dispatch {
  using Scalar = typename traits<VectorType>::Scalar;
  using RealScalar = typename NumTraits<Scalar>::Real;
  // Only complex_array_access scalars have a writable component view.
  using HasWritableRealView = bool_constant<!NumTraits<Scalar>::IsComplex || complex_array_access<Scalar>::value>;

  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE Accumulator max_abs(const VectorType& vec) {
    return vec.realView().template cast<Accumulator>().cwiseAbs().template maxCoeff<PropagateNaN>();
  }

  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE Accumulator scaled_squared_norm(const VectorType& vec,
                                                                               const Accumulator& factor) {
    return (vec.realView().template cast<Accumulator>() * factor).squaredNorm();
  }

  template <typename ResultType>
  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE void assign_scaled(ResultType& result, const VectorType& vec,
                                                                  const Accumulator& factor) {
    assign_scaled_impl(result, vec, factor, HasWritableRealView());
  }

  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE void scale_in_place(VectorType& vec, const Accumulator& factor) {
    scale_in_place_impl(vec, factor, HasWritableRealView());
  }

 private:
  template <typename ResultType>
  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE void assign_scaled_impl(ResultType& result, const VectorType& vec,
                                                                       const Accumulator& factor, std::true_type) {
    result.realView() = (vec.realView().template cast<Accumulator>() * factor).template cast<RealScalar>();
  }

  template <typename ResultType>
  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE void assign_scaled_impl(ResultType& result, const VectorType& vec,
                                                                       const Accumulator& factor, std::false_type) {
    result = vec * static_cast<RealScalar>(factor);
  }

  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE void scale_in_place_impl(VectorType& vec, const Accumulator& factor,
                                                                        std::true_type) {
    vec.realView() = (vec.realView().template cast<Accumulator>() * factor).template cast<RealScalar>();
  }

  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE void scale_in_place_impl(VectorType& vec, const Accumulator& factor,
                                                                        std::false_type) {
    vec = vec * static_cast<RealScalar>(factor);
  }
};

// Runtime-contiguous expressions can still use packet traversal.
template <typename VectorType, typename Accumulator>
struct stable_normalization_dispatch<VectorType, Accumulator, true> {
  using Scalar = typename traits<VectorType>::Scalar;
  using RealScalar = typename NumTraits<Scalar>::Real;
  // A dynamic map avoids over-unrolling fixed-size normalization paths.
  using PlainVector = Matrix<Scalar, Dynamic, 1>;
  using ConstContiguousMap = Map<const PlainVector, evaluator<VectorType>::Alignment>;
  using ContiguousMap = Map<PlainVector, evaluator<VectorType>::Alignment>;

  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE bool is_contiguous(const VectorType& vec) {
    return vec.innerStride() == 1 && (vec.outerSize() == 1 || vec.outerStride() == vec.innerSize());
  }

  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE Accumulator max_abs(const VectorType& vec) {
    if (is_contiguous(vec)) {
      const ConstContiguousMap contiguous(vec.data(), vec.size());
      return stable_normalization_dispatch<ConstContiguousMap, Accumulator, false>::max_abs(contiguous);
    }
    return stable_normalization_dispatch<VectorType, Accumulator, false>::max_abs(vec);
  }

  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE Accumulator scaled_squared_norm(const VectorType& vec,
                                                                               const Accumulator& factor) {
    if (is_contiguous(vec)) {
      const ConstContiguousMap contiguous(vec.data(), vec.size());
      return stable_normalization_dispatch<ConstContiguousMap, Accumulator, false>::scaled_squared_norm(contiguous,
                                                                                                        factor);
    }
    return stable_normalization_dispatch<VectorType, Accumulator, false>::scaled_squared_norm(vec, factor);
  }

  template <typename ResultType>
  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE void assign_scaled(ResultType& result, const VectorType& vec,
                                                                  const Accumulator& factor) {
    if (is_contiguous(vec)) {
      const ConstContiguousMap contiguous(vec.data(), vec.size());
      Map<PlainVector> output(result.data(), result.size());
      stable_normalization_dispatch<ConstContiguousMap, Accumulator, false>::assign_scaled(output, contiguous, factor);
      return;
    }
    stable_normalization_dispatch<VectorType, Accumulator, false>::assign_scaled(result, vec, factor);
  }

  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE void scale_in_place(VectorType& vec, const Accumulator& factor) {
    if (is_contiguous(vec)) {
      ContiguousMap contiguous(vec.data(), vec.size());
      stable_normalization_dispatch<ContiguousMap, Accumulator, false>::scale_in_place(contiguous, factor);
      return;
    }
    stable_normalization_dispatch<VectorType, Accumulator, false>::scale_in_place(vec, factor);
  }
};

// Prevent fast-math from merging normal scale factors into a subnormal factor.
template <typename VectorType, typename Accumulator>
EIGEN_DEVICE_FUNC EIGEN_DONT_INLINE void stable_normalization_scale_in_place(VectorType& vec,
                                                                             const Accumulator& factor) {
  stable_normalization_dispatch<VectorType, Accumulator>::scale_in_place(vec, factor);
}

template <typename VectorType, typename Divisor>
EIGEN_DEVICE_FUNC EIGEN_DONT_INLINE void stable_normalization_divide_in_place(VectorType& vec, const Divisor& divisor) {
  vec /= divisor;
}

template <typename VectorType, typename Accumulator>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void stable_normalization_with_division(VectorType& vec,
                                                                              const Accumulator& maxCoeff) {
  using RealScalar = typename NumTraits<typename traits<VectorType>::Scalar>::Real;
  // Two normal divisors avoid an exceptional reciprocal and fast-math
  // reassociation into multiplication by 1 / maxCoeff.
  const Accumulator sqrtMax = numext::sqrt(maxCoeff);
  const RealScalar scale1 = static_cast<RealScalar>(sqrtMax);
  const RealScalar scale2 = static_cast<RealScalar>(maxCoeff / sqrtMax);
  stable_normalization_divide_in_place(vec, scale1);
  stable_normalization_divide_in_place(vec, scale2);
  const Accumulator z = vec.realView().template cast<Accumulator>().squaredNorm();
  if (z > Accumulator(0)) {
    stable_normalization_scale_in_place(vec, Accumulator(1) / numext::sqrt(z));
  }
}

template <typename VectorType, typename Accumulator,
          bool = use_subnormal_preserving_scaling<Accumulator, typename traits<VectorType>::Scalar>::value>
struct stable_normalization_subnormal_recovery {
  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE void run(VectorType&) {}
};

template <typename VectorType, typename Accumulator>
struct stable_normalization_subnormal_recovery<VectorType, Accumulator, true> {
  using RealScalar = typename NumTraits<typename traits<VectorType>::Scalar>::Real;

  EIGEN_DEVICE_FUNC static EIGEN_STRONG_INLINE void run(VectorType& vec) {
    using Binary = binary_floating_point_traits<RealScalar>;
    using Bits = typename Binary::Bits;
    decltype(auto) components = vec.realView();
    Bits maxBits = 0;
    for (Index col = 0; col < components.cols(); ++col) {
      for (Index row = 0; row < components.rows(); ++row) {
        const Bits bits = Binary::magnitude(components.coeff(row, col));
        if (bits > maxBits) maxBits = bits;
      }
    }
    if (maxBits == 0 || maxBits >= Binary::kExponentUnit) return;

    const Accumulator maxAbs = numext::bit_cast<RealScalar>(maxBits);
    const auto factors = safe_scaling<Accumulator>::compute_ceiling_factors(maxAbs);
    safe_scaling<Accumulator>::scale_in_place(vec, maxAbs, factors);
    const Accumulator squaredNorm = components.template cast<Accumulator>().squaredNorm();
    if (squaredNorm > Accumulator(0)) {
      stable_normalization_divide_in_place(vec, numext::sqrt(squaredNorm));
    }
  }
};

// squaredNorm() reduces realView().cwiseAbs2(), a cwise expression with no direct access, so when
// the underlying expression has an inner stride that is not statically 1 (a dynamic-inner-stride
// Map/Ref, a row of a 1xN matrix, ...) the reduction falls back to a scalar traversal even though
// the data is frequently contiguous at runtime. This trait flags the cases where a runtime
// contiguity check is worthwhile; it mirrors the reduction fast path in Redux.h (redux_dispatch).
// bool is excluded: its squared norm is any(), handled by a dedicated specialization below.
template <typename Xpr>
struct squared_norm_runtime_unit_stride {
  using Scalar = typename traits<Xpr>::Scalar;
  static constexpr bool value =
      bool(traits<Xpr>::Flags & DirectAccessBit) && bool(packet_traits<Scalar>::Vectorizable) &&
      !bool(internal::is_same<Scalar, bool>::value) && (int(inner_stride_at_compile_time<Xpr>::value) != 1);
};

template <typename Derived, typename Scalar = typename traits<Derived>::Scalar, typename Enable = void>
struct squared_norm_impl {
  using Real = typename NumTraits<Scalar>::Real;
  static EIGEN_DEVICE_FUNC constexpr EIGEN_STRONG_INLINE Real run(const Derived& a) {
    return a.realView().cwiseAbs2().sum();
  }
};

template <typename Derived>
struct squared_norm_impl<Derived, bool, void> {
  static EIGEN_DEVICE_FUNC constexpr EIGEN_STRONG_INLINE bool run(const Derived& a) { return a.any(); }
};

// Runtime contiguity fast path: when the data is contiguous at runtime (inner stride 1, and a
// single inner panel or no gap between inner panels), reduce the underlying buffer as a contiguous
// vector, recovering vectorization of the abs2 reduction.
template <typename Derived, typename Scalar>
struct squared_norm_impl<Derived, Scalar, std::enable_if_t<squared_norm_runtime_unit_stride<Derived>::value>> {
  using Real = typename NumTraits<Scalar>::Real;
  static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Real run(const Derived& a) {
    if (a.innerStride() == 1 && (a.outerSize() == 1 || a.outerStride() == a.innerSize())) {
      using PlainVector = Matrix<Scalar, Dynamic, 1>;
      Map<const PlainVector, evaluator<Derived>::Alignment> contiguous(a.data(), a.size());
      return contiguous.realView().cwiseAbs2().sum();
    }
    return a.realView().cwiseAbs2().sum();
  }
};

}  // end namespace internal

/** \fn MatrixBase::dot
 * \returns the dot product of *this with other.
 *
 * \only_for_vectors
 *
 * \note If the scalar type is complex numbers, then this function returns the hermitian
 * (sesquilinear) dot product, conjugate-linear in the first variable and linear in the
 * second variable.
 *
 * \sa squaredNorm(), norm()
 */
template <typename Derived>
template <typename OtherDerived>
EIGEN_DEVICE_FUNC constexpr EIGEN_STRONG_INLINE
    typename ScalarBinaryOpTraits<typename internal::traits<Derived>::Scalar,
                                  typename internal::traits<OtherDerived>::Scalar>::ReturnType
    MatrixBase<Derived>::dot(const MatrixBase<OtherDerived>& other) const {
  return internal::dot_impl<Derived, OtherDerived>::run(derived(), other.derived());
}

//---------- implementation of L2 norm and related functions ----------

/** \returns, for vectors, the squared \em l2 norm of \c *this, and for matrices the squared Frobenius norm.
 * In both cases, it consists in the sum of the square of all the matrix entries.
 * For vectors, this is also equal to the dot product of \c *this with itself.
 *
 * \sa dot(), norm(), lpNorm()
 */
template <typename Derived>
EIGEN_DEVICE_FUNC constexpr EIGEN_STRONG_INLINE typename NumTraits<typename internal::traits<Derived>::Scalar>::Real
MatrixBase<Derived>::squaredNorm() const {
  return internal::squared_norm_impl<Derived>::run(derived());
}

/** \returns, for vectors, the \em l2 norm of \c *this, and for matrices the Frobenius norm.
 * In both cases, it consists in the square root of the sum of the square of all the matrix entries.
 * For vectors, this is also equal to the square root of the dot product of \c *this with itself.
 *
 * \sa lpNorm(), dot(), squaredNorm()
 */
template <typename Derived>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE typename NumTraits<typename internal::traits<Derived>::Scalar>::Real
MatrixBase<Derived>::norm() const {
  return numext::sqrt(squaredNorm());
}

/** \returns an expression of the quotient of \c *this by its own norm.
 *
 * \warning If the input vector is too small (i.e., this->norm()==0),
 *          then this function returns a copy of the input.
 *
 * \only_for_vectors
 *
 * \sa norm(), normalize()
 */
template <typename Derived>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const typename MatrixBase<Derived>::PlainObject MatrixBase<Derived>::normalized()
    const {
  using Nested_ = typename internal::nested_eval<Derived, 2>::type;
  Nested_ n(derived());
  RealScalar z = n.squaredNorm();
  // NOTE: after extensive benchmarking, this conditional does not impact performance, at least on recent x86 CPU
  if (z > RealScalar(0))
    return n / numext::sqrt(z);
  else
    return n;
}

/** Normalizes the vector, i.e. divides it by its own norm.
 *
 * \only_for_vectors
 *
 * \warning If the input vector is too small (i.e., this->norm()==0), then \c *this is left unchanged.
 *
 * \sa norm(), normalized()
 */
template <typename Derived>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void MatrixBase<Derived>::normalize() {
  RealScalar z = squaredNorm();
  // NOTE: after extensive benchmarking, this conditional does not impact performance, at least on recent x86 CPU
  if (z > RealScalar(0)) derived() /= numext::sqrt(z);
}

/** \returns an expression of the quotient of \c *this by its own norm while avoiding underflow and overflow.
 *
 * \only_for_vectors
 *
 * This method is analogue to the normalized() method, but it reduces the risk of
 * underflow and overflow when computing the norm.
 *
 * \warning If the input vector is too small (i.e., this->norm()==0),
 *          then this function returns a copy of the input.
 *
 * \sa stableNorm(), stableNormalize(), normalized()
 */
template <typename Derived>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const typename MatrixBase<Derived>::PlainObject
MatrixBase<Derived>::stableNormalized() const {
  using Nested_ = typename internal::nested_eval<Derived, 3>::type;
  using NestedClean = internal::remove_all_t<Nested_>;
  using Accumulator = typename internal::stable_norm_accumulator<RealScalar>::type;
  using Dispatch = internal::stable_normalization_dispatch<NestedClean, Accumulator>;
  Nested_ vec(derived());
  if (EIGEN_PREDICT_FALSE(vec.size() == 0)) return vec;

  // Component-wise scaling stays finite when a finite complex value has an
  // overflowing magnitude, and avoids a hypot per coefficient.
  const Accumulator w = Dispatch::max_abs(vec);
  const Accumulator highest = static_cast<Accumulator>(NumTraits<RealScalar>::highest());
  if (EIGEN_PREDICT_FALSE(!(w > Accumulator(0)))) {
    PlainObject normalized(vec);
    internal::stable_normalization_subnormal_recovery<PlainObject, Accumulator>::run(normalized);
    return normalized;
  }
  if (EIGEN_PREDICT_FALSE(!(w <= highest))) return vec;

  Accumulator invScale;
  if (EIGEN_PREDICT_TRUE((internal::stable_normalization_inv_scale<RealScalar>(w, invScale)))) {
    const Accumulator z = Dispatch::scaled_squared_norm(vec, invScale);
    if (z > Accumulator(0)) {
      const Accumulator sqrt_z = numext::sqrt(z);
      Accumulator factor;
      PlainObject normalized(rows(), cols());
      if (EIGEN_PREDICT_TRUE(internal::stable_normalization_combined_factor(invScale, sqrt_z, factor))) {
        Dispatch::assign_scaled(normalized, vec, factor);
      } else {
        // invScale and sqrt_z are normal even though their quotient is not.
        Dispatch::assign_scaled(normalized, vec, invScale);
        internal::stable_normalization_divide_in_place(normalized, static_cast<RealScalar>(sqrt_z));
      }
      return normalized;
    }
    // Packet arithmetic may flush subnormal inputs even when the maximum reduction preserves them (ARMv7 NEON).
    PlainObject normalized(vec);
    internal::stable_normalization_subnormal_recovery<PlainObject, Accumulator>::run(normalized);
    return normalized;
  }

  PlainObject normalized = vec;
  internal::stable_normalization_with_division(normalized, w);
  return normalized;
}

/** Normalizes the vector while avoid underflow and overflow
 *
 * \only_for_vectors
 *
 * This method is analogue to the normalize() method, but it reduces the risk of
 * underflow and overflow when computing the norm.
 *
 * \warning If the input vector is too small (i.e., this->norm()==0), then \c *this is left unchanged.
 *
 * \sa stableNorm(), stableNormalized(), normalize()
 */
template <typename Derived>
EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE void MatrixBase<Derived>::stableNormalize() {
  using Accumulator = typename internal::stable_norm_accumulator<RealScalar>::type;
  using Dispatch = internal::stable_normalization_dispatch<Derived, Accumulator>;
  if (EIGEN_PREDICT_FALSE(size() == 0)) return;

  const Accumulator w = Dispatch::max_abs(derived());
  const Accumulator highest = static_cast<Accumulator>(NumTraits<RealScalar>::highest());
  if (EIGEN_PREDICT_FALSE(!(w > Accumulator(0)))) {
    internal::stable_normalization_subnormal_recovery<Derived, Accumulator>::run(derived());
    return;
  }
  if (EIGEN_PREDICT_FALSE(!(w <= highest))) return;

  Accumulator invScale;
  if (EIGEN_PREDICT_TRUE((internal::stable_normalization_inv_scale<RealScalar>(w, invScale)))) {
    const Accumulator z = Dispatch::scaled_squared_norm(derived(), invScale);
    if (z > Accumulator(0)) {
      const Accumulator sqrt_z = numext::sqrt(z);
      Accumulator factor;
      if (EIGEN_PREDICT_TRUE(internal::stable_normalization_combined_factor(invScale, sqrt_z, factor))) {
        Dispatch::scale_in_place(derived(), factor);
      } else {
        internal::stable_normalization_scale_in_place(derived(), invScale);
        internal::stable_normalization_divide_in_place(derived(), static_cast<RealScalar>(sqrt_z));
      }
    } else {
      internal::stable_normalization_subnormal_recovery<Derived, Accumulator>::run(derived());
    }
    return;
  }

  internal::stable_normalization_with_division(derived(), w);
}

//---------- implementation of other norms ----------

namespace internal {

template <typename Derived, int p>
struct lpNorm_selector {
  using RealScalar = typename NumTraits<typename traits<Derived>::Scalar>::Real;
  EIGEN_DEVICE_FUNC static inline RealScalar run(const MatrixBase<Derived>& m) {
    EIGEN_USING_STD(pow)
    return pow(m.cwiseAbs().array().pow(p).sum(), RealScalar(1) / p);
  }
};

template <typename Derived>
struct lpNorm_selector<Derived, 1> {
  EIGEN_DEVICE_FUNC static inline typename NumTraits<typename traits<Derived>::Scalar>::Real run(
      const MatrixBase<Derived>& m) {
    return m.cwiseAbs().sum();
  }
};

template <typename Derived>
struct lpNorm_selector<Derived, 2> {
  EIGEN_DEVICE_FUNC static inline typename NumTraits<typename traits<Derived>::Scalar>::Real run(
      const MatrixBase<Derived>& m) {
    return m.norm();
  }
};

template <typename Derived>
struct lpNorm_selector<Derived, Infinity> {
  using RealScalar = typename NumTraits<typename traits<Derived>::Scalar>::Real;
  EIGEN_DEVICE_FUNC static inline RealScalar run(const MatrixBase<Derived>& m) {
    if (Derived::SizeAtCompileTime == 0 || (Derived::SizeAtCompileTime == Dynamic && m.size() == 0))
      return RealScalar(0);
    return m.cwiseAbs().maxCoeff();
  }
};

}  // end namespace internal

/** \returns the \b coefficient-wise \f$ \ell^p \f$ norm of \c *this, that is, returns the p-th root of the sum of the
 * p-th powers of the absolute values of the coefficients of \c *this. If \a p is the special value \a Eigen::Infinity,
 * this function returns the \f$ \ell^\infty \f$ norm, that is the maximum of the absolute values of the coefficients of
 * \c *this.
 *
 * In all cases, if \c *this is empty, then the value 0 is returned.
 *
 * \note For matrices, this function does not compute the <a
 * href="https://en.wikipedia.org/wiki/Operator_norm">operator-norm</a>. That is, if \c *this is a matrix, then its
 * coefficients are interpreted as a 1D vector. Nonetheless, you can easily compute the 1-norm and \f$\infty\f$-norm
 * matrix operator norms using \link TutorialReductionsVisitorsBroadcastingReductionsNorm partial reductions \endlink.
 *
 * \sa norm()
 */
template <typename Derived>
template <int p>
#ifndef EIGEN_PARSED_BY_DOXYGEN
EIGEN_DEVICE_FUNC inline typename NumTraits<typename internal::traits<Derived>::Scalar>::Real
#else
EIGEN_DEVICE_FUNC MatrixBase<Derived>::RealScalar
#endif
MatrixBase<Derived>::lpNorm() const {
  return internal::lpNorm_selector<Derived, p>::run(*this);
}

//---------- implementation of isOrthogonal / isUnitary ----------

/** \returns true if *this is approximately orthogonal to \a other,
 *          within the precision given by \a prec.
 *
 * Example: \include MatrixBase_isOrthogonal.cpp
 * Output: \verbinclude MatrixBase_isOrthogonal.out
 */
template <typename Derived>
template <typename OtherDerived>
bool MatrixBase<Derived>::isOrthogonal(const MatrixBase<OtherDerived>& other, const RealScalar& prec) const {
  typename internal::nested_eval<Derived, 2>::type nested(derived());
  typename internal::nested_eval<OtherDerived, 2>::type otherNested(other.derived());
  return numext::abs2(nested.dot(otherNested)) <= prec * prec * nested.squaredNorm() * otherNested.squaredNorm();
}

/** \returns true if *this is approximately an unitary matrix,
 *          within the precision given by \a prec. In the case where the \a Scalar
 *          type is real numbers, a unitary matrix is an orthogonal matrix, whence the name.
 *
 * \note This can be used to check whether a family of vectors forms an orthonormal basis.
 *       Indeed, \c m.isUnitary() returns true if and only if the columns (equivalently, the rows) of m form an
 *       orthonormal basis.
 *
 * Example: \include MatrixBase_isUnitary.cpp
 * Output: \verbinclude MatrixBase_isUnitary.out
 */
template <typename Derived>
bool MatrixBase<Derived>::isUnitary(const RealScalar& prec) const {
  typename internal::nested_eval<Derived, 1>::type self(derived());
  for (Index i = 0; i < cols(); ++i) {
    if (!internal::isApprox(self.col(i).squaredNorm(), static_cast<RealScalar>(1), prec)) return false;
    for (Index j = 0; j < i; ++j)
      if (!internal::isMuchSmallerThan(self.col(i).dot(self.col(j)), static_cast<Scalar>(1), prec)) return false;
  }
  return true;
}

}  // end namespace Eigen

#endif  // EIGEN_DOT_H
