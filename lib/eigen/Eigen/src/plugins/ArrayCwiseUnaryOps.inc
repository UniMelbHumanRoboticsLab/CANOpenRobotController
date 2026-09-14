// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

using LdexpReturnType = CwiseUnaryOp<internal::scalar_ldexp_op<Scalar>, const Derived>;
using IsFiniteTypedReturnType = CwiseUnaryOp<internal::scalar_isfinite_op<Scalar, true>, const Derived>;

/** \returns an expression of the coefficient-wise absolute value of \c *this
 *
 * Example: \include Cwise_abs.cpp
 * Output: \verbinclude Cwise_abs.out
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_abs">Math functions</a>, abs2()
 */
EIGEN_MAKE_CWISE_UNARY_OP(abs, internal::scalar_abs_op, AbsReturnType)

/** \returns an expression of the coefficient-wise phase angle of \c *this
 *
 * Example: \include Cwise_arg.cpp
 * Output: \verbinclude Cwise_arg.out
 *
 * \sa abs()
 */
EIGEN_MAKE_CWISE_UNARY_OP(arg, internal::scalar_arg_op, ArgReturnType)

EIGEN_MAKE_CWISE_UNARY_OP(carg, internal::scalar_carg_op, CArgReturnType)

/** \returns an expression of the coefficient-wise squared absolute value of \c *this
 *
 * Example: \include Cwise_abs2.cpp
 * Output: \verbinclude Cwise_abs2.out
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_abs2">Math functions</a>, abs(), square()
 */
EIGEN_MAKE_CWISE_UNARY_OP(abs2, internal::scalar_abs2_op, Abs2ReturnType)

/** \returns an expression of the coefficient-wise exponential of *this.
 *
 * This function computes the coefficient-wise exponential. The function MatrixBase::exp() in the
 * contrib module MatrixFunctions computes the matrix exponential.
 *
 * Example: \include Cwise_exp.cpp
 * Output: \verbinclude Cwise_exp.out
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_exp">Math functions</a>, exp2(), pow(), log(), sin(),
 * cos()
 */
EIGEN_MAKE_CWISE_UNARY_OP(exp, internal::scalar_exp_op, ExpReturnType)

/** \returns an expression of the coefficient-wise exponential of *this.
 *
 * This function computes the coefficient-wise base2 exponential, i.e. 2^x.
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_exp">Math functions</a>, exp(), pow(), log(), sin(),
 * cos()
 */
EIGEN_MAKE_CWISE_UNARY_OP(exp2, internal::scalar_exp2_op, Exp2ReturnType)

/** \returns an expression of the coefficients of \c *this multiplied by \f$ 2^{exponent} \f$.
 *
 * The scaling is exact: it directly adjusts the floating-point exponent, so it produces
 * correct results (including denormals) even when \f$ 2^{exponent} \f$ itself is not
 * representable as a \c Scalar. The result saturates to zero or infinity when the scaled
 * value falls outside the finite range of \c Scalar.
 *
 * Example: \include Cwise_ldexp.cpp
 * Output: \verbinclude Cwise_ldexp.out
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_ldexp">Math functions</a>, exp2(), pow()
 */
EIGEN_DEVICE_FUNC constexpr inline const LdexpReturnType ldexp(int exponent) const {
  return LdexpReturnType(derived(), internal::scalar_ldexp_op<Scalar>(exponent));
}

/** \returns an expression of the coefficient-wise exponential of *this minus 1.
 *
 * In exact arithmetic, \c x.expm1() is equivalent to \c x.exp() - 1,
 * however, with finite precision, this function is much more accurate when \c x is close to zero.
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_expm1">Math functions</a>, exp()
 */
EIGEN_MAKE_CWISE_UNARY_OP(expm1, internal::scalar_expm1_op, Expm1ReturnType)

/** \returns an expression of the coefficient-wise logarithm of *this.
 *
 * This function computes the coefficient-wise logarithm. The function MatrixBase::log() in the
 * contrib module MatrixFunctions computes the matrix logarithm.
 *
 * Example: \include Cwise_log.cpp
 * Output: \verbinclude Cwise_log.out
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_log">Math functions</a>, log()
 */
EIGEN_MAKE_CWISE_UNARY_OP(log, internal::scalar_log_op, LogReturnType)

/** \returns an expression of the coefficient-wise logarithm of 1 plus \c *this.
 *
 * In exact arithmetic, \c x.log1p() is equivalent to \c (x+1).log(),
 * however, with finite precision, this function is much more accurate when \c x is close to zero.
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_log1p">Math functions</a>, log()
 */
EIGEN_MAKE_CWISE_UNARY_OP(log1p, internal::scalar_log1p_op, Log1pReturnType)

/** \returns an expression of the coefficient-wise base-10 logarithm of *this.
 *
 * This function computes the coefficient-wise base-10 logarithm.
 *
 * Example: \include Cwise_log10.cpp
 * Output: \verbinclude Cwise_log10.out
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_log10">Math functions</a>, log()
 */
EIGEN_MAKE_CWISE_UNARY_OP(log10, internal::scalar_log10_op, Log10ReturnType)

/** \returns an expression of the coefficient-wise base-2 logarithm of *this.
 *
 * This function computes the coefficient-wise base-2 logarithm.
 *
 */
EIGEN_MAKE_CWISE_UNARY_OP(log2, internal::scalar_log2_op, Log2ReturnType)

/** \returns an expression of the coefficient-wise square root of *this.
 *
 * This function computes the coefficient-wise square root. The function MatrixBase::sqrt() in the
 * contrib module MatrixFunctions computes the matrix square root.
 *
 * Example: \include Cwise_sqrt.cpp
 * Output: \verbinclude Cwise_sqrt.out
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_sqrt">Math functions</a>, pow(), square(), cbrt()
 */
EIGEN_MAKE_CWISE_UNARY_OP(sqrt, internal::scalar_sqrt_op, SqrtReturnType)

/** \returns an expression of the coefficient-wise cube root of *this.
 *
 * This function computes the coefficient-wise cube root.
 *
 * Example: \include Cwise_cbrt.cpp
 * Output: \verbinclude Cwise_cbrt.out
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_cbrt">Math functions</a>, sqrt(), pow(), square()
 */
EIGEN_MAKE_CWISE_UNARY_OP(cbrt, internal::scalar_cbrt_op, CbrtReturnType)

/** \returns an expression of the coefficient-wise inverse square root of *this.
 *
 * This function computes the coefficient-wise inverse square root.
 *
 * Example: \include Cwise_sqrt.cpp
 * Output: \verbinclude Cwise_sqrt.out
 *
 * \sa pow(), square()
 */
EIGEN_MAKE_CWISE_UNARY_OP(rsqrt, internal::scalar_rsqrt_op, RsqrtReturnType)

/** \returns an expression of the coefficient-wise signum of *this.
 *
 * This function computes the coefficient-wise signum.
 *
 * Example: \include Cwise_sign.cpp
 * Output: \verbinclude Cwise_sign.out
 *
 * \sa pow(), square()
 */
EIGEN_MAKE_CWISE_UNARY_OP(sign, internal::scalar_sign_op, SignReturnType)

/** \returns an expression of the coefficient-wise cosine of *this.
 *
 * This function computes the coefficient-wise cosine. The function MatrixBase::cos() in the
 * contrib module MatrixFunctions computes the matrix cosine.
 *
 * Example: \include Cwise_cos.cpp
 * Output: \verbinclude Cwise_cos.out
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_cos">Math functions</a>, sin(), acos()
 */
EIGEN_MAKE_CWISE_UNARY_OP(cos, internal::scalar_cos_op, CosReturnType)

/** \returns an expression of the coefficient-wise sine of *this.
 *
 * This function computes the coefficient-wise sine. The function MatrixBase::sin() in the
 * contrib module MatrixFunctions computes the matrix sine.
 *
 * Example: \include Cwise_sin.cpp
 * Output: \verbinclude Cwise_sin.out
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_sin">Math functions</a>, cos(), asin()
 */
EIGEN_MAKE_CWISE_UNARY_OP(sin, internal::scalar_sin_op, SinReturnType)

/** \returns an expression of the coefficient-wise tan of *this.
 *
 * Example: \include Cwise_tan.cpp
 * Output: \verbinclude Cwise_tan.out
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_tan">Math functions</a>, cos(), sin()
 */
EIGEN_MAKE_CWISE_UNARY_OP(tan, internal::scalar_tan_op, TanReturnType)

/** \returns an expression of the coefficient-wise arc tan of *this.
 *
 * Example: \include Cwise_atan.cpp
 * Output: \verbinclude Cwise_atan.out
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_atan">Math functions</a>, tan(), asin(), acos()
 */
EIGEN_MAKE_CWISE_UNARY_OP(atan, internal::scalar_atan_op, AtanReturnType)

/** \returns an expression of the coefficient-wise arc cosine of *this.
 *
 * Example: \include Cwise_acos.cpp
 * Output: \verbinclude Cwise_acos.out
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_acos">Math functions</a>, cos(), asin()
 */
EIGEN_MAKE_CWISE_UNARY_OP(acos, internal::scalar_acos_op, AcosReturnType)

/** \returns an expression of the coefficient-wise arc sine of *this.
 *
 * Example: \include Cwise_asin.cpp
 * Output: \verbinclude Cwise_asin.out
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_asin">Math functions</a>, sin(), acos()
 */
EIGEN_MAKE_CWISE_UNARY_OP(asin, internal::scalar_asin_op, AsinReturnType)

/** \returns an expression of the coefficient-wise hyperbolic tan of *this.
 *
 * Example: \include Cwise_tanh.cpp
 * Output: \verbinclude Cwise_tanh.out
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_tanh">Math functions</a>, tan(), sinh(), cosh()
 */
EIGEN_MAKE_CWISE_UNARY_OP(tanh, internal::scalar_tanh_op, TanhReturnType)

/** \returns an expression of the coefficient-wise hyperbolic sin of *this.
 *
 * Example: \include Cwise_sinh.cpp
 * Output: \verbinclude Cwise_sinh.out
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_sinh">Math functions</a>, sin(), tanh(), cosh()
 */
EIGEN_MAKE_CWISE_UNARY_OP(sinh, internal::scalar_sinh_op, SinhReturnType)

/** \returns an expression of the coefficient-wise hyperbolic cos of *this.
 *
 * Example: \include Cwise_cosh.cpp
 * Output: \verbinclude Cwise_cosh.out
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_cosh">Math functions</a>, tanh(), sinh(), cosh()
 */
EIGEN_MAKE_CWISE_UNARY_OP(cosh, internal::scalar_cosh_op, CoshReturnType)

/** \returns an expression of the coefficient-wise inverse hyperbolic tan of *this.
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_atanh">Math functions</a>, atanh(), asinh(), acosh()
 */
EIGEN_MAKE_CWISE_UNARY_OP(atanh, internal::scalar_atanh_op, AtanhReturnType)

/** \returns an expression of the coefficient-wise inverse hyperbolic sin of *this.
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_asinh">Math functions</a>, atanh(), asinh(), acosh()
 */
EIGEN_MAKE_CWISE_UNARY_OP(asinh, internal::scalar_asinh_op, AsinhReturnType)

/** \returns an expression of the coefficient-wise inverse hyperbolic cos of *this.
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_acosh">Math functions</a>, atanh(), asinh(), acosh()
 */
EIGEN_MAKE_CWISE_UNARY_OP(acosh, internal::scalar_acosh_op, AcoshReturnType)

/** \returns an expression of the coefficient-wise logistic of *this.
 */
EIGEN_MAKE_CWISE_UNARY_OP(logistic, internal::scalar_logistic_op, LogisticReturnType)

/** \returns an expression of the coefficient-wise inverse of *this.
 *
 * Example: \include Cwise_inverse.cpp
 * Output: \verbinclude Cwise_inverse.out
 *
 * \sa operator/(), operator*()
 */
EIGEN_MAKE_CWISE_UNARY_OP(inverse, internal::scalar_inverse_op, InverseReturnType)

/** \returns an expression of the coefficient-wise square of *this.
 *
 * Example: \include Cwise_square.cpp
 * Output: \verbinclude Cwise_square.out
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_squareE">Math functions</a>, abs2(), cube(), pow()
 */
EIGEN_MAKE_CWISE_UNARY_OP(square, internal::scalar_square_op, SquareReturnType)

/** \returns an expression of the coefficient-wise cube of *this.
 *
 * Example: \include Cwise_cube.cpp
 * Output: \verbinclude Cwise_cube.out
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_cube">Math functions</a>, square(), pow()
 */
EIGEN_MAKE_CWISE_UNARY_OP(cube, internal::scalar_cube_op, CubeReturnType)

/** \returns an expression of the coefficient-wise rint of *this.
 *
 * Example: \include Cwise_rint.cpp
 * Output: \verbinclude Cwise_rint.out
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_rint">Math functions</a>, ceil(), floor()
 */
EIGEN_MAKE_CWISE_UNARY_OP(rint, internal::scalar_rint_op, RintReturnType)

/** \returns an expression of the coefficient-wise round of *this.
 *
 * Example: \include Cwise_round.cpp
 * Output: \verbinclude Cwise_round.out
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_round">Math functions</a>, ceil(), floor()
 */
EIGEN_MAKE_CWISE_UNARY_OP(round, internal::scalar_round_op, RoundReturnType)

/** \returns an expression of the coefficient-wise floor of *this.
 *
 * Example: \include Cwise_floor.cpp
 * Output: \verbinclude Cwise_floor.out
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_floor">Math functions</a>, ceil(), round()
 */
EIGEN_MAKE_CWISE_UNARY_OP(floor, internal::scalar_floor_op, FloorReturnType)

/** \returns an expression of the coefficient-wise ceil of *this.
 *
 * Example: \include Cwise_ceil.cpp
 * Output: \verbinclude Cwise_ceil.out
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_ceil">Math functions</a>, floor(), round()
 */
EIGEN_MAKE_CWISE_UNARY_OP(ceil, internal::scalar_ceil_op, CeilReturnType)

/** \returns an expression of the coefficient-wise truncation of *this.
 *
 * Example: \include Cwise_trunc.cpp
 * Output: \verbinclude Cwise_trunc.out
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_trunc">Math functions</a>, floor(), round()
 */
EIGEN_MAKE_CWISE_UNARY_OP(trunc, internal::scalar_trunc_op, TruncReturnType)

template <int N>
struct ArithmeticShiftRightXpr {
  using Type = CwiseUnaryOp<internal::scalar_arithmetic_shift_right_op<Scalar, N>, const Derived>;
};

/** \returns an expression of \c *this with the \a Scalar type arithmetically
 * shifted right by \a N bit positions.
 *
 * The template parameter \a N specifies the number of bit positions to shift.
 * The vacated high bits are filled with the sign bit for a signed \a Scalar and
 * with zero for an unsigned one, which has no sign bit to propagate. Use
 * logicalShiftRight() to fill with zero regardless of signedness.
 *
 * \sa logicalShiftRight(), logicalShiftLeft()
 */
template <int N>
EIGEN_DEVICE_FUNC constexpr typename ArithmeticShiftRightXpr<N>::Type arithmeticShiftRight() const {
  return typename ArithmeticShiftRightXpr<N>::Type(derived());
}

template <int N>
struct LogicalShiftRightXpr {
  using Type = CwiseUnaryOp<internal::scalar_logical_shift_right_op<Scalar, N>, const Derived>;
};

/** \returns an expression of \c *this with the \a Scalar type logically
 * shifted right by \a N bit positions.
 *
 * The template parameter \a N specifies the number of bit positions to shift.
 * The vacated high bits are filled with zero even when \a Scalar is signed; use
 * arithmeticShiftRight() to propagate the sign bit instead.
 *
 * \sa arithmeticShiftRight(), logicalShiftLeft()
 */
template <int N>
EIGEN_DEVICE_FUNC constexpr typename LogicalShiftRightXpr<N>::Type logicalShiftRight() const {
  return typename LogicalShiftRightXpr<N>::Type(derived());
}

template <int N>
struct LogicalShiftLeftXpr {
  using Type = CwiseUnaryOp<internal::scalar_logical_shift_left_op<Scalar, N>, const Derived>;
};

/** \returns an expression of \c *this with the \a Scalar type logically
 * shifted left by \a N bit positions.
 *
 * The template parameter \a N specifies the number of bit positions to shift.
 * The vacated low bits are filled with zero. There is no separate arithmetic
 * left shift: for every integral \a Scalar it produces the same bit pattern.
 *
 * \sa arithmeticShiftRight(), logicalShiftRight()
 */
template <int N>
EIGEN_DEVICE_FUNC constexpr typename LogicalShiftLeftXpr<N>::Type logicalShiftLeft() const {
  return typename LogicalShiftLeftXpr<N>::Type(derived());
}

template <int N>
struct ShiftRightXpr {
  using Type = typename ArithmeticShiftRightXpr<N>::Type;
};

/** \deprecated Use arithmeticShiftRight() instead.
 *
 * \returns an expression of \c *this with the \a Scalar type arithmetically
 * shifted right by \a N bit positions.
 *
 * \sa arithmeticShiftRight()
 */
template <int N>
EIGEN_DEPRECATED_WITH_REASON("Use arithmeticShiftRight() instead.")
EIGEN_DEVICE_FUNC constexpr typename ShiftRightXpr<N>::Type shiftRight() const {
  return arithmeticShiftRight<N>();
}

template <int N>
struct ShiftLeftXpr {
  using Type = typename LogicalShiftLeftXpr<N>::Type;
};

/** \deprecated Use logicalShiftLeft() instead.
 *
 * \returns an expression of \c *this with the \a Scalar type logically
 * shifted left by \a N bit positions.
 *
 * \sa logicalShiftLeft()
 */
template <int N>
EIGEN_DEPRECATED_WITH_REASON("Use logicalShiftLeft() instead.")
EIGEN_DEVICE_FUNC constexpr typename ShiftLeftXpr<N>::Type shiftLeft() const {
  return logicalShiftLeft<N>();
}

/** \returns an expression of the coefficient-wise isnan of *this.
 *
 * Example: \include Cwise_isNaN.cpp
 * Output: \verbinclude Cwise_isNaN.out
 *
 * \sa isfinite(), isinf()
 */
EIGEN_MAKE_CWISE_UNARY_OP(isNaN, internal::scalar_isnan_op, IsNaNReturnType)

/** \returns an expression of the coefficient-wise isinf of *this.
 *
 * Example: \include Cwise_isInf.cpp
 * Output: \verbinclude Cwise_isInf.out
 *
 * \sa isnan(), isfinite()
 */
EIGEN_MAKE_CWISE_UNARY_OP(isInf, internal::scalar_isinf_op, IsInfReturnType)

/** \returns an expression of the coefficient-wise isfinite of *this.
 *
 * Example: \include Cwise_isFinite.cpp
 * Output: \verbinclude Cwise_isFinite.out
 *
 * \sa isnan(), isinf()
 */
EIGEN_MAKE_CWISE_UNARY_OP(isFinite, internal::scalar_isfinite_op, IsFiniteReturnType)
EIGEN_DEVICE_FUNC constexpr inline const IsFiniteTypedReturnType isFiniteTyped() const {
  return IsFiniteTypedReturnType(derived());
}

/** \returns an expression of the coefficient-wise ! operator of *this
 *
 * Example: \include Cwise_boolean_not.cpp
 * Output: \verbinclude Cwise_boolean_not.out
 *
 * \sa operator!=()
 */
EIGEN_MAKE_CWISE_UNARY_OP(operator!, internal::scalar_boolean_not_op, BooleanNotReturnType)

/** \returns an expression of the bitwise ~ operator of *this
 */
EIGEN_MAKE_CWISE_UNARY_OP(operator~, internal::scalar_bitwise_not_op, BitwiseNotReturnType)

// --- SpecialFunctions module ---

/** \returns an expression of the coefficient-wise ln(|gamma(*this)|).
 *
 * \specialfunctions_module
 *
 * \note This function supports only float and double scalar types. To support other scalar types,
 * the user has to provide implementations of lgamma(T) for any scalar type T to be supported.
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_lgamma">Math functions</a>, digamma()
 */
EIGEN_MAKE_CWISE_UNARY_OP(lgamma, internal::scalar_lgamma_op, LgammaReturnType)

/** \returns an expression of the coefficient-wise digamma (psi, derivative of lgamma).
 *
 * \specialfunctions_module
 *
 * \note This function supports only float and double scalar types. To support other scalar types,
 * the user has to provide implementations of digamma(T) for any scalar
 * type T to be supported.
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_digamma">Math functions</a>, Eigen::digamma(),
 * Eigen::polygamma(), lgamma()
 */
EIGEN_MAKE_CWISE_UNARY_OP(digamma, internal::scalar_digamma_op, DigammaReturnType)

/** \returns an expression of the coefficient-wise Gauss error
 * function of *this.
 *
 * \specialfunctions_module
 *
 * \note This function supports only float and double scalar types. To support other scalar types,
 * the user has to provide implementations of erf(T) for any scalar type T to be supported.
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_erf">Math functions</a>, erfc()
 */
EIGEN_MAKE_CWISE_UNARY_OP(erf, internal::scalar_erf_op, ErfReturnType)

/** \returns an expression of the coefficient-wise Complementary error
 * function of *this.
 *
 * \specialfunctions_module
 *
 * \note This function supports only float and double scalar types. To support other scalar types,
 * the user has to provide implementations of erfc(T) for any scalar type T to be supported.
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_erfc">Math functions</a>, erf()
 */
EIGEN_MAKE_CWISE_UNARY_OP(erfc, internal::scalar_erfc_op, ErfcReturnType)

/** \returns an expression of the coefficient-wise inverse of the CDF of the Normal distribution
 * function of *this.
 *
 * \specialfunctions_module
 *
 * In other words, considering `x = ndtri(y)`, it returns the argument, x, for which the area under the
 * Gaussian probability density function (integrated from minus infinity to x) is equal to y.
 *
 * \note This function supports only float and double scalar types. To support other scalar types,
 * the user has to provide implementations of ndtri(T) for any scalar type T to be supported.
 *
 * \sa <a href="group__CoeffwiseMathFunctions.html#cwisetable_ndtri">Math functions</a>
 */
EIGEN_MAKE_CWISE_UNARY_OP(ndtri, internal::scalar_ndtri_op, NdtriReturnType)

template <typename ScalarExponent>
using UnaryPowReturnType =
    std::enable_if_t<internal::is_arithmetic<typename NumTraits<ScalarExponent>::Real>::value,
                     CwiseUnaryOp<internal::scalar_unary_pow_op<Scalar, ScalarExponent>, const Derived>>;

/** \returns an expression of the coefficients of \c *this raised to the constant power \a exponent
 *
 * \tparam ScalarExponent is the scalar type of \a exponent. It must be compatible with the scalar type
 *         of the given expression.
 * \param exponent the scalar exponent value.
 *
 * This function computes the coefficient-wise power. The function MatrixBase::pow() in the
 * contrib module MatrixFunctions computes the matrix power.
 *
 * Example: \include Cwise_pow.cpp
 * Output: \verbinclude Cwise_pow.out
 *
 * \sa ArrayBase::pow(ArrayBase), square(), cube(), exp(), log()
 */
template <typename ScalarExponent>
EIGEN_DEVICE_FUNC constexpr EIGEN_STRONG_INLINE const UnaryPowReturnType<ScalarExponent> pow(
    const ScalarExponent& exponent) const {
  return UnaryPowReturnType<ScalarExponent>(derived(), internal::scalar_unary_pow_op<Scalar, ScalarExponent>(exponent));
}
