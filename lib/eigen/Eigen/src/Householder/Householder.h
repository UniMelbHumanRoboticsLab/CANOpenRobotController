// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2010 Benoit Jacob <jacob.benoit.1@gmail.com>
// Copyright (C) 2009 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_HOUSEHOLDER_H
#define EIGEN_HOUSEHOLDER_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

namespace internal {
template <int N>
struct decrement_size : std::integral_constant<int, N - 1> {};
template <>
struct decrement_size<0> : std::integral_constant<int, 0> {};
template <>
struct decrement_size<Dynamic> : std::integral_constant<int, Dynamic> {};

template <typename RealScalar>
struct householder_norm_accumulator : stable_norm_accumulator<RealScalar> {};

template <>
struct householder_norm_accumulator<float> {
  // Stable norm reductions can avoid underflow by scaling in float. Householder construction must also preserve each
  // subnormal component while forming scale-free tau and essential-vector ratios, so widen only these intermediates.
  using type = double;
};

template <typename Scalar, typename Accumulator, bool IsComplex = NumTraits<Scalar>::IsComplex>
struct householder_rescale;

template <typename Scalar, typename Accumulator>
struct householder_rescale<Scalar, Accumulator, false> {
  EIGEN_DEVICE_FUNC static Scalar run(const Scalar& value, const Accumulator& scale) {
    return Scalar(Accumulator(value) / scale);
  }

  // This overload only keeps the shared C++14 call site well-formed; real inputs return before reaching it.
  EIGEN_DEVICE_FUNC static Scalar zero_tail_tau(const Accumulator& scaledReal, const Accumulator&,
                                                const Scalar& scaledBeta) {
    return Scalar(1) - Scalar(scaledReal) / scaledBeta;
  }

  template <typename EssentialPart, typename TailView>
  EIGEN_DEVICE_FUNC static void run(EssentialPart& essential, const TailView& tail, const Accumulator& scale,
                                    const Scalar& denominator) {
    essential = ((tail.template cast<Accumulator>().array() / scale) / Accumulator(denominator))
                    .matrix()
                    .template cast<Scalar>();
  }
};

template <typename Scalar, typename Accumulator>
struct householder_rescale<Scalar, Accumulator, true> {
  using RealScalar = typename NumTraits<Scalar>::Real;

  EIGEN_DEVICE_FUNC static Scalar run(const Scalar& value, const Accumulator& scale) {
    return Scalar(RealScalar(Accumulator(numext::real(value)) / scale),
                  RealScalar(Accumulator(numext::imag(value)) / scale));
  }

  EIGEN_DEVICE_FUNC static Scalar zero_tail_tau(const Accumulator& scaledReal, const Accumulator& scaledImag,
                                                const RealScalar& scaledBeta) {
    return Scalar(RealScalar(1) - RealScalar(scaledReal) / scaledBeta, RealScalar(scaledImag) / scaledBeta);
  }

  template <typename EssentialPart, typename TailView>
  EIGEN_DEVICE_FUNC static void run(EssentialPart& essential, const TailView& tail, const Accumulator& scale,
                                    const Scalar& denominator) {
    run(essential, tail, scale, denominator, complex_array_access<Scalar>());
  }

 private:
  template <typename EssentialPart, typename TailView>
  EIGEN_DEVICE_FUNC static void run(EssentialPart& essential, const TailView& tail, const Accumulator& scale,
                                    const Scalar& denominator, std::true_type) {
    essential.realView().array() =
        (tail.realView().array().template cast<Accumulator>() / scale).template cast<RealScalar>();
    essential.array() /= denominator;
  }

  template <typename EssentialPart, typename TailView>
  EIGEN_DEVICE_FUNC static void run(EssentialPart& essential, const TailView& tail, const Accumulator& scale,
                                    const Scalar& denominator, std::false_type) {
    for (Index i = 0; i < tail.size(); ++i) essential.coeffRef(i) = run(tail.coeff(i), scale) / denominator;
  }
};
}  // namespace internal

/** Computes the elementary reflector H such that:
 * \f$ H *this = [ beta 0 ... 0]^T \f$
 * where the transformation H is:
 * \f$ H = I - tau v v^*\f$
 * and the vector v is:
 * \f$ v^T = [1 essential^T] \f$
 *
 * The essential part of the vector \c v is stored in *this.
 *
 * On output:
 * \param tau the scaling factor of the Householder transformation
 * \param beta the result of H * \c *this
 *
 * \sa MatrixBase::makeHouseholder(), MatrixBase::applyHouseholderOnTheLeft(),
 *     MatrixBase::applyHouseholderOnTheRight()
 */
template <typename Derived>
EIGEN_DEVICE_FUNC void MatrixBase<Derived>::makeHouseholderInPlace(Scalar& tau, RealScalar& beta) {
  VectorBlock<Derived, internal::decrement_size<Base::SizeAtCompileTime>::value> essentialPart(derived(), 1,
                                                                                               size() - 1);
  makeHouseholder(essentialPart, tau, beta);
}

/** Computes the elementary reflector H such that:
 * \f$ H *this = [ beta 0 ... 0]^T \f$
 * where the transformation H is:
 * \f$ H = I - tau v v^*\f$
 * and the vector v is:
 * \f$ v^T = [1 essential^T] \f$
 *
 * On output:
 * \param essential the essential part of the vector \c v
 * \param tau the scaling factor of the Householder transformation
 * \param beta the result of H * \c *this
 *
 * \sa MatrixBase::makeHouseholderInPlace(), MatrixBase::applyHouseholderOnTheLeft(),
 *     MatrixBase::applyHouseholderOnTheRight()
 */
template <typename Derived>
template <typename EssentialPart>
EIGEN_DEVICE_FUNC void MatrixBase<Derived>::makeHouseholder(EssentialPart& essential, Scalar& tau,
                                                            RealScalar& beta) const {
  using numext::conj;

  EIGEN_STATIC_ASSERT_VECTOR_ONLY(EssentialPart)
  const VectorBlock<const Derived, EssentialPart::SizeAtCompileTime> tail(derived(), 1, size() - 1);

  const RealScalar tailSqNorm = size() == 1 ? RealScalar(0) : tail.unwind().squaredNorm();
  Scalar c0 = coeff(0);
  const RealScalar tol = (std::numeric_limits<RealScalar>::min)();
  RealScalar unscaledNormThreshold = tol;
  // Whether the direct construction's abs2(c0) + tailSqNorm would exceed the range. Integer scalars keep the direct
  // path they have always taken; the scaled path divides by a component maximum, which does not apply to them.
  bool unscaledSqNormOverflows = false;
  EIGEN_IF_CONSTEXPR (!NumTraits<RealScalar>::IsInteger) {
    const RealScalar precision = RealScalar(NumTraits<RealScalar>::epsilon());
    // With flush-to-zero arithmetic, every tail component square below tol can be lost. Account for every component
    // so the discarded contribution is at most epsilon relative to a squared norm above this threshold. The narrow
    // normal range of half makes this scaled path common for moderately small inputs; preserving the bound there is
    // intentional.
    const RealScalar componentCount = RealScalar(size() - 1) * RealScalar(NumTraits<Scalar>::IsComplex ? 2 : 1);
    unscaledNormThreshold = (tol / precision) * componentCount;

    // Both terms overflow well before the reflector stops being representable, so classify the input before the
    // squares are formed: abs2(c0) is at most twice the square of the larger component of c0, and the tail's own
    // reduction has already overflowed if tailSqNorm exceeds the bound. Testing the sum with isinf() instead would
    // not survive -ffinite-math-only, which folds that test away, whereas a comparison against a finite bound is
    // still evaluated.
    const RealScalar sqNormBound = NumTraits<RealScalar>::highest() / RealScalar(2);
    const RealScalar componentBound = numext::sqrt(sqNormBound / RealScalar(2));
    const RealScalar c0Max = numext::maxi(numext::abs(numext::real(c0)), numext::abs(numext::imag(c0)));
    unscaledSqNormOverflows = !(c0Max <= componentBound) || !(tailSqNorm <= sqNormBound);
  }

  // The scaled path forms the reflector from ratios of the largest component and never squares an unscaled
  // coefficient, so it is also the path for inputs the direct construction cannot square.
  if ((tailSqNorm <= unscaledNormThreshold || unscaledSqNormOverflows) && !(numext::isnan)(c0)) {
    using Accumulator = typename internal::householder_norm_accumulator<RealScalar>::type;
    const auto tailView = tail.unwind();
    const auto tailComponents = tailView.realView();
    // Component maxima cannot underflow when a representable tail is nonzero.
    const Accumulator tailMax =
        tailView.size() == 0 ? Accumulator(0) : Accumulator(tailComponents.cwiseAbs().maxCoeff());
    if (numext::is_exactly_zero(tailMax) && numext::is_exactly_zero(numext::imag(c0))) {
      tau = RealScalar(0);
      beta = numext::real(c0);
      essential.setZero();
      return;
    }
    const Accumulator c0RealAbs = numext::abs(Accumulator(numext::real(c0)));
    const Accumulator c0ImagAbs = numext::abs(Accumulator(numext::imag(c0)));
    const Accumulator c0Max = numext::maxi(c0RealAbs, c0ImagAbs);
    const Accumulator scale = numext::maxi(c0Max, tailMax);
    const RealScalar realScale = RealScalar(scale);
    // A target that flushes this scale cannot form meaningful ratios from the entirely subnormal vector.
    if (scale < Accumulator(tol) && numext::is_exactly_zero(realScale + realScale)) {
      tau = RealScalar(0);
      beta = numext::real(c0);
      essential.setZero();
      return;
    }
    if (numext::is_exactly_zero(tailMax)) {
      const Accumulator scaledReal = Accumulator(numext::real(c0)) / scale;
      const Accumulator scaledImag = Accumulator(numext::imag(c0)) / scale;
      RealScalar scaledBeta = RealScalar(numext::hypot(scaledReal, scaledImag));
      if (numext::real(c0) >= RealScalar(0)) scaledBeta = -scaledBeta;
      beta = RealScalar(scale * Accumulator(scaledBeta));
      essential.setZero();
      tau = internal::householder_rescale<Scalar, Accumulator>::zero_tail_tau(scaledReal, scaledImag, scaledBeta);
      return;
    }
    // Form the reflector from scale-free ratios to preserve subnormal inputs and avoid overflowing c0 - beta.
    Accumulator scaledTailSqNorm;
    EIGEN_IF_CONSTEXPR (std::is_same<RealScalar, float>::value) {
      // Double has enough exponent range to square every finite float scale without underflow or overflow.
      scaledTailSqNorm = tailComponents.template cast<Accumulator>().squaredNorm() / (scale * scale);
    } else {
      scaledTailSqNorm = (tailComponents.template cast<Accumulator>().array() / scale).matrix().squaredNorm();
    }
    if (numext::is_exactly_zero(RealScalar(tailMax / scale)) && numext::is_exactly_zero(numext::imag(c0))) {
      tau = RealScalar(0);
      beta = numext::real(c0);
      essential.setZero();
      return;
    }
    const Scalar scaledC0 = internal::householder_rescale<Scalar, Accumulator>::run(c0, scale);
    RealScalar scaledBeta =
        RealScalar(numext::hypot(Accumulator(numext::abs(scaledC0)), numext::sqrt(scaledTailSqNorm)));
    if (numext::real(c0) >= RealScalar(0)) scaledBeta = -scaledBeta;
    beta = RealScalar(scale * Accumulator(scaledBeta));
    internal::householder_rescale<Scalar, Accumulator>::run(essential, tailView, scale, scaledC0 - scaledBeta);
    tau = conj(Scalar(RealScalar(1)) - scaledC0 / scaledBeta);
    return;
  }
  beta = numext::sqrt(numext::abs2(c0) + tailSqNorm);
  if (numext::real(c0) >= RealScalar(0)) beta = -beta;
  essential = tail.unwind() / (c0 - beta);
  tau = conj((beta - c0) / beta);
}

namespace internal {

template <typename Derived, typename EssentialPart,
          bool Fused = !Derived::IsRowMajor && EssentialPart::ColsAtCompileTime == 1 &&
                       (EssentialPart::RowsAtCompileTime == 1 || EssentialPart::RowsAtCompileTime == 2)>
struct householder_apply_left_impl {
  using Scalar = typename Derived::Scalar;
  static EIGEN_DEVICE_FUNC void run(MatrixBase<Derived>& mat, const EssentialPart& essential, const Scalar& tau,
                                    Scalar* workspace) {
    Map<typename plain_row_type<typename Derived::PlainObject>::type> tmp(workspace, mat.cols());
    Block<Derived, EssentialPart::SizeAtCompileTime, Derived::ColsAtCompileTime> bottom(mat.derived(), 1, 0,
                                                                                        mat.rows() - 1, mat.cols());
    tmp.noalias() = essential.adjoint() * bottom.unwind();
    tmp = tau * (tmp + mat.row(0));
    mat.row(0) -= tmp;
    bottom.unwind().noalias() -= essential * tmp;
  }
};

// Two- and three-element reflectors on column-major storage: finishing each column before advancing replaces
// four strided row passes, which thrash the cache at large outer strides (issue #3160).
template <typename Derived, typename EssentialPart>
struct householder_apply_left_impl<Derived, EssentialPart, true> {
  using Scalar = typename Derived::Scalar;
  static EIGEN_DEVICE_FUNC void run(MatrixBase<Derived>& mat, const EssentialPart& essential, const Scalar& tau,
                                    Scalar*) {
    // Evaluated once so that the column loop reads plain coefficients; tau may reference a coefficient of mat.
    const Matrix<Scalar, EssentialPart::RowsAtCompileTime, 1> v = essential;
    const Scalar tauValue = tau;
    Block<Derived, EssentialPart::RowsAtCompileTime, Derived::ColsAtCompileTime> bottom(mat.derived(), 1, 0,
                                                                                        mat.rows() - 1, mat.cols());
    for (Index j = 0; j < mat.cols(); ++j) {
      const Scalar tmp = tauValue * (v.dot(bottom.col(j)) + mat.coeff(0, j));
      mat.coeffRef(0, j) -= tmp;
      bottom.col(j) -= v * tmp;
    }
  }
};

}  // namespace internal

/** Apply the elementary reflector H given by
 * \f$ H = I - tau v v^*\f$
 * with
 * \f$ v^T = [1 essential^T] \f$
 * from the left to a vector or matrix.
 *
 * On input:
 * \param essential the essential part of the vector \c v
 * \param tau the scaling factor of the Householder transformation
 * \param workspace a pointer to working space with at least
 *                  this->cols() entries
 *
 * \sa MatrixBase::makeHouseholder(), MatrixBase::makeHouseholderInPlace(),
 *     MatrixBase::applyHouseholderOnTheRight()
 */
template <typename Derived>
template <typename EssentialPart>
EIGEN_DEVICE_FUNC void MatrixBase<Derived>::applyHouseholderOnTheLeft(const EssentialPart& essential, const Scalar& tau,
                                                                      Scalar* workspace) {
  if (rows() == 1) {
    *this *= Scalar(1) - tau;
  } else if (!numext::is_exactly_zero(tau)) {
    internal::householder_apply_left_impl<Derived, EssentialPart>::run(*this, essential, tau, workspace);
  }
}

/** Apply the elementary reflector H given by
 * \f$ H = I - tau v v^*\f$
 * with
 * \f$ v^T = [1 essential^T] \f$
 * from the right to a vector or matrix.
 *
 * On input:
 * \param essential the essential part of the vector \c v
 * \param tau the scaling factor of the Householder transformation
 * \param workspace a pointer to working space with at least
 *                  this->rows() entries
 *
 * \sa MatrixBase::makeHouseholder(), MatrixBase::makeHouseholderInPlace(),
 *     MatrixBase::applyHouseholderOnTheLeft()
 */
template <typename Derived>
template <typename EssentialPart>
EIGEN_DEVICE_FUNC void MatrixBase<Derived>::applyHouseholderOnTheRight(const EssentialPart& essential,
                                                                       const Scalar& tau, Scalar* workspace) {
  if (cols() == 1) {
    *this *= Scalar(1) - tau;
  } else if (!numext::is_exactly_zero(tau)) {
    Map<typename internal::plain_col_type<PlainObject>::type> tmp(workspace, rows());
    Block<Derived, Derived::RowsAtCompileTime, EssentialPart::SizeAtCompileTime> right(derived(), 0, 1, rows(),
                                                                                       cols() - 1);
    tmp.noalias() = right.unwind() * essential;
    tmp = tau * (tmp + this->col(0));
    this->col(0) = this->col(0) - tmp;
    right.unwind().noalias() -= tmp * essential.adjoint();
  }
}

}  // end namespace Eigen

#endif  // EIGEN_HOUSEHOLDER_H
