// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_STABLENORM_H
#define EIGEN_STABLENORM_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

namespace internal {

template <typename Accumulator, bool = std::is_floating_point<Accumulator>::value>
struct stable_norm_unscaled_predicate {
  static inline bool run(const Accumulator&, const Accumulator&) { return false; }
};

template <typename Accumulator>
struct stable_norm_unscaled_predicate<Accumulator, true> {
  static inline bool run(const Accumulator& maxCoeff, const Accumulator& invScale) {
    using std::sqrt;
    // A block has at most 8192 real components. These bounds keep the error
    // from flushed component squares below one epsilon.
    static const Accumulator kSqrtMin =
        sqrt((numext::numeric_limits<Accumulator>::min)() * (Accumulator(16384) / NumTraits<Accumulator>::epsilon()));
    static const Accumulator kSqrtMax = sqrt(NumTraits<Accumulator>::highest() / Accumulator(16384));
    // A normal invScale avoids fast-math flushing invScale^2.
    static const Accumulator kSqrtNormalMin = sqrt((numext::numeric_limits<Accumulator>::min)());
    return maxCoeff >= kSqrtMin && maxCoeff <= kSqrtMax && invScale >= kSqrtNormalMin;
  }
};

template <typename ExpressionType, typename Accumulator>
inline Accumulator stable_norm_squared_norm(const ExpressionType& block, const Accumulator& invScale,
                                            const Accumulator& maxCoeff) {
  if (stable_norm_unscaled_predicate<Accumulator>::run(maxCoeff, invScale)) {
    return block.realView().template cast<Accumulator>().squaredNorm() * numext::abs2(invScale);
  }
  return (block.realView().template cast<Accumulator>() * invScale).squaredNorm();
}

template <typename ExpressionType, typename Accumulator>
inline void stable_norm_kernel(const ExpressionType& block, Accumulator& ssq, Accumulator& scale,
                               Accumulator& invScale) {
  // Component-wise maxima give the required scale without complex hypot calls.
  Accumulator maxCoeff = block.realView().template cast<Accumulator>().cwiseAbs().template maxCoeff<PropagateNaN>();

  if (maxCoeff > scale) {
    if (maxCoeff > NumTraits<Accumulator>::highest())  // we got an INF
    {
      invScale = Accumulator(1);
      scale = maxCoeff;
    } else {
      const auto factors = safe_scaling<Accumulator>::compute_ceiling_factors_with_normal_reciprocal(maxCoeff);
      if (scale > Accumulator(0)) ssq = ssq * numext::abs2(scale * factors.invScale);
      scale = factors.scale;
      invScale = factors.invScale;
    }
  } else if (maxCoeff != maxCoeff)  // we got a NaN
  {
    scale = maxCoeff;
  }

  // TODO: skip sub-vector when maxCoeff << current scale.
  if (scale > Accumulator(0))  // if scale==0, then block is 0
    ssq += stable_norm_squared_norm(block, invScale, maxCoeff);
}

template <typename VectorType, typename Accumulator>
void stable_norm_impl_inner_step(const VectorType& vec, Accumulator& ssq, Accumulator& scale, Accumulator& invScale) {
  const Index blockSize = 4096;

  Index n = vec.size();
  Index blockEnd = numext::round_down(n, blockSize);
  for (Index i = 0; i < blockEnd; i += blockSize) {
    internal::stable_norm_kernel(vec.template segment<blockSize>(i), ssq, scale, invScale);
  }
  if (n > blockEnd) {
    internal::stable_norm_kernel(vec.tail(n - blockEnd), ssq, scale, invScale);
  }
}

template <typename VectorType, typename Accumulator,
          bool = bool(traits<VectorType>::Flags & DirectAccessBit) &&
                 (int(inner_stride_at_compile_time<VectorType>::value) != 1)>
struct stable_norm_vector_dispatch {
  static inline void run(const VectorType& vec, Accumulator& ssq, Accumulator& scale, Accumulator& invScale) {
    stable_norm_impl_inner_step(vec, ssq, scale, invScale);
  }
};

template <typename VectorType, typename Accumulator>
struct stable_norm_vector_dispatch<VectorType, Accumulator, true> {
  static inline void run(const VectorType& vec, Accumulator& ssq, Accumulator& scale, Accumulator& invScale) {
    if (vec.innerStride() == 1) {
      using Scalar = typename traits<VectorType>::Scalar;
      using PlainVector = Matrix<Scalar, VectorType::SizeAtCompileTime, 1, 0, VectorType::MaxSizeAtCompileTime, 1>;
      using ContiguousMap = Map<const PlainVector, evaluator<VectorType>::Alignment>;
      const ContiguousMap contiguous(vec.data(), vec.size());
      stable_norm_impl_inner_step(contiguous, ssq, scale, invScale);
      return;
    }
    stable_norm_impl_inner_step(vec, ssq, scale, invScale);
  }
};

template <typename VectorType, typename Accumulator>
inline void stable_norm_impl_inner_dispatch(const VectorType& vec, Accumulator& ssq, Accumulator& scale,
                                            Accumulator& invScale) {
  stable_norm_vector_dispatch<VectorType, Accumulator>::run(vec, ssq, scale, invScale);
}

template <typename MatrixType, typename Accumulator>
inline void stable_norm_impl_outer_steps(const MatrixType& mat, Accumulator& ssq, Accumulator& scale,
                                         Accumulator& invScale) {
  for (Index j = 0; j < mat.outerSize(); ++j) {
    stable_norm_impl_inner_dispatch(mat.innerVector(j), ssq, scale, invScale);
  }
}

template <typename MatrixType, typename Accumulator, bool = bool(traits<MatrixType>::Flags & DirectAccessBit)>
struct stable_norm_matrix_dispatch {
  static inline void run(const MatrixType& mat, Accumulator& ssq, Accumulator& scale, Accumulator& invScale) {
    stable_norm_impl_outer_steps(mat, ssq, scale, invScale);
  }
};

template <typename MatrixType, typename Accumulator>
struct stable_norm_matrix_dispatch<MatrixType, Accumulator, true> {
  static inline void run(const MatrixType& mat, Accumulator& ssq, Accumulator& scale, Accumulator& invScale) {
    if (mat.innerStride() == 1 && (mat.outerSize() == 1 || mat.outerStride() == mat.innerSize())) {
      using Scalar = typename traits<MatrixType>::Scalar;
      using PlainVector = Matrix<Scalar, MatrixType::SizeAtCompileTime, 1, 0, MatrixType::MaxSizeAtCompileTime, 1>;
      using ContiguousMap = Map<const PlainVector, evaluator<MatrixType>::Alignment>;
      const ContiguousMap contiguous(mat.data(), mat.size());
      stable_norm_impl_inner_step(contiguous, ssq, scale, invScale);
      return;
    }
    stable_norm_impl_outer_steps(mat, ssq, scale, invScale);
  }
};

template <typename VectorType, std::enable_if_t<VectorType::IsVectorAtCompileTime, int> = 0>
typename VectorType::RealScalar stable_norm_impl(const VectorType& vec) {
  using std::sqrt;

  Index n = vec.size();
  if (EIGEN_PREDICT_FALSE(n == 1)) return numext::abs(vec.coeff(0));

  using RealScalar = typename VectorType::RealScalar;
  using Accumulator = typename stable_norm_accumulator<RealScalar>::type;
  Accumulator scale(0);
  Accumulator invScale(1);
  Accumulator ssq(0);  // sum of squares

  stable_norm_vector_dispatch<VectorType, Accumulator>::run(vec, ssq, scale, invScale);

  return RealScalar(scale * sqrt(ssq));
}

template <typename MatrixType, std::enable_if_t<!MatrixType::IsVectorAtCompileTime, int> = 0>
typename MatrixType::RealScalar stable_norm_impl(const MatrixType& mat) {
  using std::sqrt;

  using RealScalar = typename MatrixType::RealScalar;
  using Accumulator = typename stable_norm_accumulator<RealScalar>::type;
  Accumulator scale(0);
  Accumulator invScale(1);
  Accumulator ssq(0);  // sum of squares

  stable_norm_matrix_dispatch<MatrixType, Accumulator>::run(mat, ssq, scale, invScale);
  return RealScalar(scale * sqrt(ssq));
}

inline int stable_norm_floor_div2(int value) { return value / 2 - ((value < 0 && value % 2 != 0) ? 1 : 0); }

inline int stable_norm_ceil_div2(int value) { return value / 2 + ((value > 0 && value % 2 != 0) ? 1 : 0); }

template <typename Accumulator>
inline void blue_norm_accumulate_component(const Accumulator& ax, const Accumulator& tsml, const Accumulator& tbig,
                                           const Accumulator& ssml, const Accumulator& sbig, bool& notBig,
                                           Accumulator& asml, Accumulator& amed, Accumulator& abig) {
  if (ax > tbig) {
    abig += numext::abs2(ax * sbig);
    notBig = false;
  } else if (ax < tsml) {
    if (notBig) asml += numext::abs2(ax * ssml);
  } else {
    amed += numext::abs2(ax);
  }
}

template <typename Scalar, typename Accumulator, bool = NumTraits<Scalar>::IsComplex>
struct blue_norm_accumulate_scalar {
  static inline void run(const Scalar& value, const Accumulator& tsml, const Accumulator& tbig, const Accumulator& ssml,
                         const Accumulator& sbig, bool& notBig, Accumulator& asml, Accumulator& amed,
                         Accumulator& abig) {
    const Accumulator ax = numext::abs(Accumulator(value));
    blue_norm_accumulate_component(ax, tsml, tbig, ssml, sbig, notBig, asml, amed, abig);
  }
};

template <typename Scalar, typename Accumulator>
struct blue_norm_accumulate_scalar<Scalar, Accumulator, true> {
  static inline void run(const Scalar& value, const Accumulator& tsml, const Accumulator& tbig, const Accumulator& ssml,
                         const Accumulator& sbig, bool& notBig, Accumulator& asml, Accumulator& amed,
                         Accumulator& abig) {
    const Accumulator real = numext::abs(Accumulator(numext::real(value)));
    const Accumulator imag = numext::abs(Accumulator(numext::imag(value)));
    blue_norm_accumulate_component(real, tsml, tbig, ssml, sbig, notBig, asml, amed, abig);
    blue_norm_accumulate_component(imag, tsml, tbig, ssml, sbig, notBig, asml, amed, abig);
  }
};

template <typename Derived>
inline typename NumTraits<typename traits<Derived>::Scalar>::Real blueNorm_impl(const EigenBase<Derived>& _vec) {
  using RealScalar = typename Derived::RealScalar;
  using Accumulator = typename stable_norm_accumulator<RealScalar>::type;
  using Scalar = typename traits<Derived>::Scalar;
  using std::pow;
  using std::sqrt;

  const Derived& vec(_vec.derived());
  if (vec.size() == 0) return RealScalar(0);

  // Blue, ACM TOMS 4(1), 1978, https://doi.org/10.1145/355769.355771.
  // The small-value multiplier includes Anderson's denormal correction from
  // Algorithm 978, ACM TOMS 44(1), 2017, https://doi.org/10.1145/3061665.
  // These thresholds and the three-accumulator merge follow Reference BLAS
  // xNRM2 (LAPACK 3.12.1), expressed independently for Eigen scalar types.
  static const int ibeta = std::numeric_limits<Accumulator>::radix;
  static const int it = NumTraits<Accumulator>::digits();
  static const int iemin = NumTraits<Accumulator>::min_exponent();
  static const int iemax = NumTraits<Accumulator>::max_exponent();
  static const Accumulator tsml = Accumulator(pow(Accumulator(ibeta), Accumulator(stable_norm_ceil_div2(iemin - 1))));
  static const Accumulator tbig =
      Accumulator(pow(Accumulator(ibeta), Accumulator(stable_norm_floor_div2(iemax - it + 1))));
  static const Accumulator ssml =
      Accumulator(pow(Accumulator(ibeta), Accumulator(-stable_norm_floor_div2(iemin - it))));
  static const Accumulator sbig =
      Accumulator(pow(Accumulator(ibeta), Accumulator(-stable_norm_ceil_div2(iemax + it - 1))));

  bool notBig = true;
  Accumulator asml(0);
  Accumulator amed(0);
  Accumulator abig(0);

  for (Index j = 0; j < vec.outerSize(); ++j) {
    for (typename Derived::InnerIterator iter(vec, j); iter; ++iter) {
      blue_norm_accumulate_scalar<Scalar, Accumulator>::run(iter.value(), tsml, tbig, ssml, sbig, notBig, asml, amed,
                                                            abig);
    }
  }

  Accumulator scale(1);
  Accumulator sumsq(0);
  if (abig > Accumulator(0)) {
    if (amed > Accumulator(0) || amed > NumTraits<Accumulator>::highest() || amed != amed) abig += (amed * sbig) * sbig;
    scale = Accumulator(1) / sbig;
    sumsq = abig;
  } else if (asml > Accumulator(0)) {
    if (amed > Accumulator(0) || amed > NumTraits<Accumulator>::highest() || amed != amed) {
      amed = sqrt(amed);
      asml = sqrt(asml) / ssml;
      // Spell this as in xNRM2 rather than with min/max: when amed is NaN,
      // it must become ymax so that the final result remains NaN.
      const bool smallIsLarger = asml > amed;
      const Accumulator ymin = smallIsLarger ? amed : asml;
      const Accumulator ymax = smallIsLarger ? asml : amed;
      sumsq = numext::abs2(ymax) * (Accumulator(1) + numext::abs2(ymin / ymax));
    } else {
      scale = Accumulator(1) / ssml;
      sumsq = asml;
    }
  } else {
    sumsq = amed;
  }
  return RealScalar(scale * sqrt(sumsq));
}

}  // end namespace internal

/** \returns the \em l2 norm of \c *this avoiding underflow and overflow.
 * This version uses a blockwise two-pass algorithm:
 *  1 - find the absolute largest coefficient and choose a scale \c s (a nearby normal power of two for supported
 *      binary floating-point accumulators)
 *  2 - compute \f$ s \Vert \frac{*this}{s} \Vert \f$ in a standard way
 *
 * For architecture/scalar types supporting vectorization, this version
 * is faster than blueNorm(). Otherwise the blueNorm() is much faster.
 *
 * \sa norm(), blueNorm(), hypotNorm()
 */
template <typename Derived>
inline typename NumTraits<typename internal::traits<Derived>::Scalar>::Real MatrixBase<Derived>::stableNorm() const {
  using Nested = typename internal::nested_eval<Derived, 2>::type;
  Nested nested(derived());
  return internal::stable_norm_impl(nested);
}

/** \returns the \em l2 norm of \c *this using Blue's algorithm, with
 * Anderson's Algorithm 978 correction for denormalized values.
 * Blue, A Portable Fortran Program to Find the Euclidean Norm of a Vector,
 * ACM TOMS, Vol 4, Issue 1, 1978; Anderson, ACM TOMS, Vol 44, Issue 1, 2017.
 *
 * For architecture/scalar types without vectorization, this version
 * is much faster than stableNorm(). Otherwise the stableNorm() is faster.
 *
 * \sa norm(), stableNorm(), hypotNorm()
 */
template <typename Derived>
inline typename NumTraits<typename internal::traits<Derived>::Scalar>::Real MatrixBase<Derived>::blueNorm() const {
  return internal::blueNorm_impl(*this);
}

/** \returns the \em l2 norm of \c *this avoiding underflow and overflow.
 * This version use a concatenation of hypot() calls, and it is very slow.
 *
 * \sa norm(), stableNorm()
 */
template <typename Derived>
inline typename NumTraits<typename internal::traits<Derived>::Scalar>::Real MatrixBase<Derived>::hypotNorm() const {
  using Accumulator = typename internal::stable_norm_accumulator<RealScalar>::type;
  if (size() == 0) return RealScalar(0);
  // Component reduction avoids rounded complex magnitudes and permits promoted accumulation.
  return RealScalar(
      derived().realView().template cast<Accumulator>().cwiseAbs().redux(internal::scalar_hypot_op<Accumulator>()));
}

}  // end namespace Eigen

#endif  // EIGEN_STABLENORM_H
