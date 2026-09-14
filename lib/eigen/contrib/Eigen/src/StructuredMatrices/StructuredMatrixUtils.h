// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

// References:
//  [1] N. J. Higham, "Accuracy and Stability of Numerical Algorithms", 2nd ed.,
//      SIAM, 2002, chapter 27. Avoiding spurious overflow by rescaling with
//      powers of two, the technique behind structured_exponent_bound() and the
//      column scaling in structured_fft_apply() and Bccb::applySymbol().
//  [2] P. H. Sterbenz, "Floating-Point Computation", Prentice-Hall, 1974.
//      Scaling by a power of two is exact, so the scaled transforms introduce
//      no roundoff beyond the transforms themselves.

#ifndef EIGEN_STRUCTURED_MATRIX_UTILS_H
#define EIGEN_STRUCTURED_MATRIX_UTILS_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

namespace internal {

// A distinct shape routes assignment through evalTo/addTo/subTo and lets one
// product specialization cover every product tag without colliding with DenseShape.
struct StructuredShape {};

// Below this dimension the FFT setup costs more than a plain O(n^2) evaluation,
// so the structured operators fall back to a direct segment-based product.
constexpr Index structured_direct_threshold() { return 32; }

// Below this dimension even the segment-based direct product loses to a plain
// scalar loop: the per-segment setup dominates when the average segment holds
// fewer than a couple of packets (measured crossover on AVX2 hardware).
constexpr Index structured_scalar_threshold() { return 16; }

// Numerical scale exponents are independent of Eigen's configurable dimension
// index. A 32-bit Index can overflow while accumulating O(n^2) factor
// exponents for dimensions that are otherwise practical.
using structured_exponent_type = numext::int64_t;

/** \internal Balanced mantissa*2^e arithmetic shared by the structured
 * operators' determinant-style accumulations (the split fraction/exponent
 * convention of LINPACK's xGEDI; see the per-class references).
 * structured_balance() rescales \a z by the power of two that brings
 * \c max(|re|,|im|) (the modulus can overflow where the components do not) --
 * or \c |z| for a real scalar -- into [0.5, 1), accumulating the removed
 * exponent into \a exponent. The rescaling is exact, so no roundoff is
 * introduced; zeros and non-finite values, which must propagate exactly, are
 * returned untouched. */
template <typename Scalar, bool IsComplex = NumTraits<Scalar>::IsComplex>
struct structured_balance_impl {
  using RealScalar = typename NumTraits<Scalar>::Real;
  template <typename Exponent>
  static Scalar run(const Scalar& z, Exponent& exponent) {
    const RealScalar mag = numext::maxi(numext::abs(numext::real(z)), numext::abs(numext::imag(z)));
    if (!(mag > RealScalar(0)) || !(numext::isfinite)(mag)) return z;
    int e;
    EIGEN_USING_STD(frexp);
    frexp(mag, &e);
    exponent += e;
    return Scalar(numext::ldexp(numext::real(z), -e), numext::ldexp(numext::imag(z), -e));
  }
  static Scalar apply_exponent(const Scalar& z, int e) {
    return Scalar(numext::ldexp(numext::real(z), e), numext::ldexp(numext::imag(z), e));
  }
};

template <typename Scalar>
struct structured_balance_impl<Scalar, false> {
  template <typename Exponent>
  static Scalar run(const Scalar& x, Exponent& exponent) {
    const Scalar mag = numext::abs(x);
    if (!(mag > Scalar(0)) || !(numext::isfinite)(mag)) return x;
    int e;
    EIGEN_USING_STD(frexp);
    frexp(mag, &e);
    exponent += e;
    return numext::ldexp(x, -e);
  }
  static Scalar apply_exponent(const Scalar& x, int e) { return numext::ldexp(x, e); }
};

template <typename Scalar, typename Exponent>
Scalar structured_balance(const Scalar& z, Exponent& exponent) {
  return structured_balance_impl<Scalar>::run(z, exponent);
}

/** \internal Applies an accumulated power-of-two \a exponent to \a z,
 * component-wise for complex scalars. ldexp saturates cleanly to zero /
 * infinity (preserving signs) once the exponent leaves the representable
 * range; the clamp only guards the narrowing to int. */
template <typename Scalar, typename Exponent>
Scalar structured_ldexp_clamped(const Scalar& z, Exponent exponent) {
  constexpr Exponent kMaxExponent = Exponent(1) << 24;
  const int e = static_cast<int>(numext::mini(numext::maxi(exponent, -kMaxExponent), kMaxExponent));
  return structured_balance_impl<Scalar>::apply_exponent(z, e);
}

/** \internal \returns 1 / \a z, formed in a balanced frame where the direct
 * reciprocal is unreliable: near the representable boundary both conj(z)/|z|^2
 * and Smith's algorithm (internal::complex_divide_fast and _smith) overflow to
 * zero when |Re z| and |Im z| are each around half the largest finite value.
 * Balancing puts the larger component in [0.5, 1), where the reciprocal is
 * always representable, and both rescalings are exact. Zeros and non-finite
 * values pass through structured_balance() untouched. */
template <typename Scalar>
Scalar structured_scaled_reciprocal(const Scalar& z) {
  using RealScalar = typename NumTraits<Scalar>::Real;
  const RealScalar mag = numext::maxi(numext::abs(numext::real(z)), numext::abs(numext::imag(z)));
  const RealScalar m2 = mag * mag;
  // Where |z|^2 <= 2*mag^2 is normal, no implementation of the reciprocal can
  // overflow or underflow, so only the remaining entries pay for the balancing.
  if (m2 >= (std::numeric_limits<RealScalar>::min)() && (numext::isfinite)(m2 + m2)) return Scalar(1) / z;
  Index e = 0;
  const Scalar zs = structured_balance(z, e);
  return structured_ldexp_clamped(Scalar(1) / zs, -e);
}

template <typename RealScalar>
struct structured_reciprocal_packet_traits {
  // These backends provide interleaved complex packets, ploaddup and mask reductions.
#if defined(EIGEN_VECTORIZE_SSE2)
  static constexpr bool Vectorizable =
      std::is_same<RealScalar, float>::value || std::is_same<RealScalar, double>::value;
#else
  static constexpr bool Vectorizable = false;
#endif
};

template <typename RealScalar, bool ApplyThreshold,
          bool Vectorize = structured_reciprocal_packet_traits<RealScalar>::Vectorizable>
struct structured_symbol_reciprocal_impl {
  using Complex = std::complex<RealScalar>;
  static void run(const Complex* symbol, const RealScalar* moduli, RealScalar tol, Complex* inverse, Index size) {
    for (Index k = 0; k < size; ++k)
      inverse[k] = ApplyThreshold && moduli[k] < tol ? Complex(0) : structured_scaled_reciprocal(symbol[k]);
  }
};

template <typename RealScalar, bool ApplyThreshold>
struct structured_symbol_reciprocal_impl<RealScalar, ApplyThreshold, true> {
  using Complex = std::complex<RealScalar>;
  using Packet = typename packet_traits<Complex>::type;
  using RealPacket = typename unpacket_traits<Packet>::as_real;

  EIGEN_DONT_INLINE static void run(const Complex* symbol, const RealScalar* moduli, RealScalar tol, Complex* inverse,
                                    Index size) {
    constexpr Index kPacketSize = unpacket_traits<Packet>::size;
    const Packet zero = pset1<Packet>(Complex(0));
    const Packet safe_value = pset1<Packet>(Complex(1, 1));
    const RealPacket threshold = pset1<RealPacket>(tol);
    // min_normal <= mag^2 and 2*mag^2 <= 2^(max_exponent-1) < max_finite.
    const RealPacket lower =
        pset1<RealPacket>(numext::ldexp(RealScalar(1), std::numeric_limits<RealScalar>::min_exponent / 2));
    const RealPacket upper =
        pset1<RealPacket>(numext::ldexp(RealScalar(1), std::numeric_limits<RealScalar>::max_exponent / 2 - 1));
    const Index packet_end = size - size % kPacketSize;
    Index k = 0;
    for (; k < packet_end; k += kPacketSize) {
      const RealPacket discard = ApplyThreshold ? pcmp_lt(ploaddup<RealPacket>(moduli + k), threshold) : zero.v;
      // Sanitize before arithmetic: selecting the output alone still divides discarded zeros.
      const Packet z = pselect(Packet(discard), safe_value, ploadu<Packet>(symbol + k));
      const RealPacket a = pabs(z.v);
      const RealPacket mag = pmax(a, pcplxflip(Packet(a)).v);
      // Zero components use the scalar path to preserve std::complex's signed-zero choices.
      const RealPacket safe = pand(pand(pcmp_le(lower, mag), pcmp_le(mag, upper)), pcmp_lt(zero.v, a));
      if (!predux_any(pandnot(ptrue(safe), safe))) {
        const RealPacket square = pmul(z.v, z.v);
        const RealPacket denom = padd(square, pcplxflip(Packet(square)).v);
        const Packet reciprocal(pdiv(pconj(z).v, denom));
        // Underflowed components also retain the scalar signed-zero convention.
        if (!predux_any(pcmp_eq(reciprocal.v, zero.v))) {
          pstoreu(inverse + k, pselect(Packet(discard), zero, reciprocal));
          continue;
        }
      }
      structured_symbol_reciprocal_impl<RealScalar, ApplyThreshold, false>::run(
          symbol + k, ApplyThreshold ? moduli + k : nullptr, tol, inverse + k, kPacketSize);
    }
    if (k < size)
      structured_symbol_reciprocal_impl<RealScalar, ApplyThreshold, false>::run(
          symbol + k, ApplyThreshold ? moduli + k : nullptr, tol, inverse + k, size - k);
  }
};

// The input and output may alias exactly. Threshold equality and NaN moduli are retained.
template <typename RealScalar>
void structured_symbol_reciprocal(const std::complex<RealScalar>* symbol, const RealScalar* moduli, RealScalar tol,
                                  std::complex<RealScalar>* inverse, Index size) {
  structured_symbol_reciprocal_impl<RealScalar, true>::run(symbol, moduli, tol, inverse, size);
}

// True inverses retain every mode and reuse the symbol storage without allocating a modulus array.
template <typename RealScalar>
void structured_symbol_reciprocal(std::complex<RealScalar>* symbol, Index size) {
  structured_symbol_reciprocal_impl<RealScalar, false>::run(symbol, nullptr, RealScalar(0), symbol, size);
}

/** \internal \returns \c a - b guarded against spurious overflow: when the
 * plain difference of two finite values overflows to infinity, it is
 * recomputed from the halved operands with \a e set to 1 so the caller can
 * carry the factor of two in its running exponent (the balanced accumulations
 * fold it into \c exponent at the call site). The recomputation is exact where
 * it matters: a difference of finite values only overflows when both operands
 * are huge, normal values, whose halves and halved difference are exactly
 * representable. For complex scalars the finiteness tests are component-wise;
 * a subnormal component riding along with a huge one loses its last bit to the
 * halving, an error far below one ulp of the factor's magnitude. Non-finite
 * operands (genuine Inf/NaN nodes) propagate untouched, with \a e = 0. */
template <typename Scalar>
Scalar structured_guarded_diff(const Scalar& a, const Scalar& b, int& e) {
  using RealScalar = typename NumTraits<Scalar>::Real;
  e = 0;
  const Scalar t = a - b;
  if ((numext::isfinite)(t) || !(numext::isfinite)(a) || !(numext::isfinite)(b)) return t;
  e = 1;
  return a * RealScalar(0.5) - b * RealScalar(0.5);
}

/** \internal \returns the indices sorted by decreasing precomputed modulus
 * \a mods (each modulus is computed once, not on every comparison); the shared
 * ordering of the operators' singularValues()/matrixU()/matrixV(). The sort is
 * stable so repeated calls agree even in the presence of ties, and NaN moduli
 * order last (comparing through NaN directly would break the strict weak
 * ordering std::stable_sort requires). */
template <typename RealVectorType>
std::vector<Index> structured_svd_permutation(const RealVectorType& mods) {
  using RealScalar = typename RealVectorType::Scalar;
  std::vector<Index> perm;
  perm.reserve(static_cast<std::size_t>(mods.size()));
  for (Index k = 0; k < mods.size(); ++k) perm.push_back(k);
  std::stable_sort(perm.begin(), perm.end(), [&mods](Index a, Index b) {
    const RealScalar ka = mods[a], kb = mods[b];
    // isgreater is quiet for NaNs; the second clause implements NaN-last ordering.
    return std::isgreater(ka, kb) || (!(numext::isnan)(ka) && (numext::isnan)(kb));
  });
  return perm;
}

/** \internal \returns the per-thread FFT engine shared by all structured
 * operators. The kissfft backend caches its twiddle/plan tables per transform
 * size inside the engine, so reusing one engine amortizes the plan setup that a
 * per-call engine would redo on every product, solve and symbol computation
 * (the tables themselves are identical, so results are bit-for-bit unchanged).
 * One engine per thread keeps concurrent products on the same operator free of
 * data races; the cache grows with the number of distinct transform sizes a
 * thread touches, which mirrors the operators it works with.
 *
 * Under EIGEN_AVOID_THREAD_LOCAL -- the library-wide opt-out for targets
 * without usable `thread_local` (see Core's Memory.h and ThreadPool's
 * ThreadLocal.h) -- each call returns a fresh engine by value instead: the
 * plan tables are rebuilt per call, the results are identical. Callers bind
 * the engine with `auto&&`, which works for both signatures. */
#ifndef EIGEN_AVOID_THREAD_LOCAL
template <typename RealScalar>
FFT<RealScalar>& structured_fft_engine() {
  static thread_local FFT<RealScalar> fft;
  return fft;
}
#else
template <typename RealScalar>
FFT<RealScalar> structured_fft_engine() {
  return FFT<RealScalar>();
}
#endif

/** \internal
 * \returns the smallest integer >= \a n whose only prime factors are 2, 3 and 5.
 *
 * Such "5-smooth" sizes keep the FFT fast and sidestep the default kissfft
 * backend's poor handling of sizes with large prime factors. Used to pad the
 * circulant embedding of a Toeplitz matrix. The {2,3,5}-smooth numbers are dense
 * enough that the linear search returns after only a handful of steps.
 */
inline Index fft_next_good_size(Index n) {
  if (n < 1) return 1;
  for (Index m = n;; ++m) {
    Index r = m;
    while (r % 2 == 0) r /= 2;
    while (r % 3 == 0) r /= 3;
    while (r % 5 == 0) r /= 5;
    if (r == 1) return m;
  }
}

/** \internal View a complex-valued expression as \a Scalar: the expression itself
 * when \a Scalar is complex, its real part when \a Scalar is real (the imaginary
 * part then only holds numerically negligible roundoff). \c run_scalar is the
 * single-coefficient analogue. A single dispatch struct keeps the number of
 * instantiated helpers down to one per scalar type. */
template <typename Scalar, bool IsComplex = NumTraits<Scalar>::IsComplex>
struct structured_scalar_part_impl {
  template <typename Xpr>
  static const Xpr& run(const Xpr& xpr) {
    return xpr;
  }
  static const Scalar& run_scalar(const Scalar& x) { return x; }
};

template <typename Scalar>
struct structured_scalar_part_impl<Scalar, false> {
  template <typename Xpr>
  static typename Xpr::RealReturnType run(const Xpr& xpr) {
    return xpr.real();
  }
  static Scalar run_scalar(const std::complex<Scalar>& x) { return numext::real(x); }
};

/** \internal Computes an exponent bound \c e with \c max_k|x[k]| < 2^e (0 when
 * \a x is zero or reduces to a non-finite maximum) and \returns whether \a x is
 * safe for the transforms, from a single plain (fast-max) reduction pass. The
 * bound is derived from the component-wise magnitudes, never from the modulus:
 * a finite complex value near the overflow threshold has a non-representable
 * modulus, which would silently disable the overflow-protection scaling exactly
 * where it is needed. Bounding the modulus by twice the largest component costs
 * at most one extra bit.
 *
 * The routing predicate deliberately uses the fast max reduction, which is not
 * guaranteed to propagate NaN (the NaN-propagating reduction de-vectorizes to a
 * branchy scalar loop on strided component views). That is sufficient: an Inf
 * in NaN-free data always surfaces in the maximum (every comparison is
 * ordered), and a column containing NaN produces the all-NaN output the dense
 * product semantics require through *either* path -- every dot product picks
 * up a coeff*NaN term, and the transforms propagate NaN just the same -- so
 * missing a NaN here cannot change the result. */
template <typename Xpr>
bool structured_exponent_bound_finite(const Xpr& x, int& e) {
  using ScalarTraits = NumTraits<typename Xpr::Scalar>;
  using RealScalar = typename ScalarTraits::Real;
  // maxCoeff() asserts on an empty input, and a degenerate operand -- a rank-0
  // factor, a solve with no right-hand sides -- reaches here legitimately.
  // An empty operand bounds nothing, so its exponent bound is 0.
  e = 0;
  if (x.size() == 0) return true;
  RealScalar m;
  if (ScalarTraits::IsComplex)
    // realView() reduces over both components in one pass, vectorized for
    // direct-access storage; the strided real()/imag() views never vectorize.
    m = x.realView().cwiseAbs().maxCoeff();
  else
    m = x.cwiseAbs().maxCoeff();
  e = 0;
  if (!(numext::isfinite)(m)) return false;
  if (m > RealScalar(0)) {
    EIGEN_USING_STD(frexp);
    frexp(m, &e);
    if (ScalarTraits::IsComplex) ++e;
  }
  return true;
}

/** \internal The exponent bound alone (see structured_exponent_bound_finite()),
 * for callers that handle non-finite data separately: the bound is 0 there. */
template <typename Xpr>
int structured_exponent_bound(const Xpr& x) {
  int e;
  structured_exponent_bound_finite(x, e);
  return e;
}

/** \internal \returns the index reversal of a DFT \a symbol: result[k] =
 * symbol[(p - k) mod p]. This is the symbol of the transposed operator: reversing
 * the generating sequence in index space reverses the frequencies of its DFT, for
 * both a circulant generator and the circulant embedding of a Toeplitz matrix.
 * An empty symbol (small operator, nothing cached) stays empty. */
template <typename ComplexVectorType>
ComplexVectorType structured_reverse_symbol(const ComplexVectorType& symbol) {
  const Index p = symbol.size();
  ComplexVectorType reversed(p);
  if (p > 0) {
    reversed[0] = symbol[0];
    reversed.tail(p - 1) = symbol.tail(p - 1).reverse();
  }
  return reversed;
}

/** \internal Default pointwise step: multiply the transformed column by the symbol. */
struct structured_symbol_multiply {
  template <typename SymbolType>
  int exponent_growth(const SymbolType& symbol) const {
    return structured_exponent_bound(symbol);
  }
  template <typename XfType, typename SymbolType>
  void operator()(XfType& xf, const SymbolType& symbol) const {
    xf.array() *= symbol.array();
  }
};

/** \internal
 * By default computes
 * \c dst.col(k) += alpha * ifft( symbol .* fft(rhs.col(k)) ) for every column of
 * \a rhs, i.e. applies the circulant operator whose eigenvalues are \a symbol.
 * A caller may supply another pointwise operation with its matching exponent
 * growth bound. The leading \a outSize entries of each back-transform form the
 * corresponding output column. All workspace is allocated once outside the
 * per-column loop; right-hand sides shorter than the transform length are
 * zero-padded into the preallocated buffer so the FFT never re-allocates.
 *
 * The transforms accumulate up to \c p addends, so a finite column near the
 * overflow threshold (or one applied through a symbol of huge magnitude) would
 * overflow inside the FFT and turn a representable result into Inf/NaN. Each
 * column is therefore scaled down by a power of two -- an exact shift, no
 * roundoff -- derived from the column's exponent and the pointwise operation's
 * maximum exponent growth, and the exponent is folded back into the output after
 * the back-transform. The scale is one whenever the conservative intermediate
 * bound cannot overflow, so results are bit-identical for inputs of moderate
 * magnitude; zero columns are never scaled. If \f$|x_i|<2^c\f$ and the pointwise
 * operation magnifies by less than \f$2^g\f$, its result and both transforms are
 * bounded by
 * \f[ 2^{c+g+2\lceil\log_2 p\rceil+1}. \f]
 *
 * Genuinely non-finite data must not go through the transforms: they mix every
 * input entry into every output entry, so a single special value would
 * contaminate the whole column with NaN where the dense product only propagates
 * it through the dot products that touch it. A non-finite symbol is the
 * caller's responsibility (the operators route it to their direct kernels up
 * front); a non-finite column is detected here -- in the same single pass that
 * derives its scaling exponent -- and handed to \a directColumn, the caller's
 * per-column direct kernel, so the remaining columns keep the fast path.
 */
template <typename Scalar, typename Dest, typename Rhs, typename DirectColumn,
          typename Pointwise = structured_symbol_multiply>
void structured_fft_apply(Dest& dst, const Matrix<std::complex<typename NumTraits<Scalar>::Real>, Dynamic, 1>& symbol,
                          Index outSize, const Rhs& rhs, const Scalar& alpha, DirectColumn&& directColumn,
                          Pointwise pointwise = Pointwise(), Index outExpAdjust = 0) {
  using RealScalar = typename NumTraits<Scalar>::Real;
  using Complex = std::complex<RealScalar>;
  using ComplexVector = Matrix<Complex, Dynamic, 1>;

  const Index p = symbol.size();
  eigen_assert(rhs.rows() <= p && outSize <= p);

  // Growth of the pointwise step -- max|symbol| for a multiply, the largest
  // retained reciprocal for a divide -- included in the pre-scaling so a
  // representable result cannot overflow before the output rescaling.
  const int pointwiseExp = pointwise.exponent_growth(symbol);

  if (p == 1) {
    // The length-one DFT is the identity and is unsupported by kissfft. The
    // pointwise step still runs, scaled around just as the transforms are on the
    // general path.
    const int budget = NumTraits<RealScalar>::max_exponent() - 2;
    ComplexVector xf(1);
    for (Index k = 0; k < rhs.cols(); ++k) {
      int colExp;
      if (!structured_exponent_bound_finite(rhs.col(k), colExp)) {
        directColumn(k);
        continue;
      }
      const int e = numext::maxi(colExp + pointwiseExp - budget, 0);
      const RealScalar down1 = numext::ldexp(RealScalar(1), -(e / 2)),
                       down2 = numext::ldexp(RealScalar(1), -(e - e / 2));
      const int eo = e - static_cast<int>(outExpAdjust);
      const RealScalar up1 = numext::ldexp(RealScalar(1), eo / 2), up2 = numext::ldexp(RealScalar(1), eo - eo / 2);
      xf.coeffRef(0) = static_cast<Complex>((rhs.coeff(0, k) * down1) * down2);
      pointwise(xf, symbol);
      dst.coeffRef(0, k) += alpha * structured_scalar_part_impl<Scalar>::run_scalar((xf.coeff(0) * up1) * up2);
    }
    return;
  }

  // The bit width of p (>= 1 here), an upper bound for the ceil(log2 p) of the
  // magnitude bound above -- one bit looser at power-of-two transform lengths.
  const int log2p = log2_floor(static_cast<std::make_unsigned_t<Index>>(p)) + 1;
  const int budget = NumTraits<RealScalar>::max_exponent() - 2 * log2p - 2;

  auto&& fft = structured_fft_engine<RealScalar>();
  ComplexVector xt = ComplexVector::Zero(p);
  ComplexVector xf(p), yt(p);
  for (Index k = 0; k < rhs.cols(); ++k) {
    int colExp;  // 0 for an all-zero column: no scaling
    if (!structured_exponent_bound_finite(rhs.col(k), colExp)) {
      directColumn(k);
      continue;
    }
    const int e = numext::maxi(colExp + pointwiseExp - budget, 0);
    // Each power of two is split in halves so that the factors themselves stay
    // inside the exponent range even when e exceeds it (a huge column applied
    // through a huge symbol); scaling by the two exact factors in sequence is
    // still an exact shift wherever the result is representable.
    const RealScalar down1 = numext::ldexp(RealScalar(1), -(e / 2)), down2 = numext::ldexp(RealScalar(1), -(e - e / 2));
    // A caller that pre-balanced the symbol passes its exponent in outExpAdjust,
    // so the pointwise step sees an O(1) symbol whose result never passes through
    // a subnormal; the exponent is folded back into the output here.
    const int eo = e - static_cast<int>(outExpAdjust);
    const RealScalar up1 = numext::ldexp(RealScalar(1), eo / 2), up2 = numext::ldexp(RealScalar(1), eo - eo / 2);
    xt.head(rhs.rows()) = ((rhs.col(k) * down1) * down2).template cast<Complex>();
    fft.fwd(xf, xt, p);
    pointwise(xf, symbol);
    fft.inv(yt, xf, p);
    dst.col(k) += alpha * structured_scalar_part_impl<Scalar>::run((yt.head(outSize) * up1) * up2);
  }
}

/** \internal Overload for callers that guarantee finite right-hand-side data
 * (e.g. solve(), which checks its input up front and takes a dedicated
 * pseudo-inverse fallback otherwise): a non-finite column has no direct kernel
 * here and would be skipped, so it asserts. */
template <typename Scalar, typename Dest, typename Rhs>
void structured_fft_apply(Dest& dst, const Matrix<std::complex<typename NumTraits<Scalar>::Real>, Dynamic, 1>& symbol,
                          Index outSize, const Rhs& rhs, const Scalar& alpha) {
  structured_fft_apply(dst, symbol, outSize, rhs, alpha,
                       [](Index) { eigen_assert(false && "non-finite column requires a direct kernel"); });
}

/** \internal Pointwise step for the pseudo-inverse solve: divides by the symbol
 * instead of multiplying by its reciprocal, since the reciprocal of an entry near
 * the representable boundary is subnormal -- and lost under flush-to-zero -- while
 * the quotient stays normal. Entries below the rank threshold are zeroed, matching
 * the pseudo-inverse convention. */
template <typename ModsType, typename RealScalar>
struct structured_symbol_divide {
  const ModsType* mods;
  RealScalar tol;
  template <typename SymbolType>
  int exponent_growth(const SymbolType&) const {
    int reciprocalExp = 0;
    for (Index k = 0; k < mods->size(); ++k) {
      const RealScalar mod = mods->coeff(k);
      if (!(mod < tol) && mod > RealScalar(0) && (numext::isfinite)(mod)) {
        int modExp;
        EIGEN_USING_STD(frexp);
        frexp(mod, &modExp);
        // mod = fraction * 2^modExp with fraction in [0.5, 1), so 1/mod <=
        // 2^(1-modExp); one more bit keeps the bound strict.
        reciprocalExp = numext::maxi(reciprocalExp, 2 - modExp);
      }
    }
    return reciprocalExp;
  }
  template <typename XfType, typename SymbolType>
  void operator()(XfType& xf, const SymbolType& symbol) const {
    using Complex = std::complex<RealScalar>;
    xf.array() = (mods->array() < tol).select(Complex(0), xf.array() / symbol.array());
  }
};

/** \internal Shared product implementation for the structured operator types.
 * Forwards to the operator's \c addProduct member, which performs the fast
 * matrix-vector product. The same body serves every dense product dispatch tag.
 *
 * The structured products carry the default product tag, so assignment has the
 * ordinary dense-product semantics: \c x = op * expr first materializes the
 * product into a temporary, which resolves every form of aliasing between the
 * destination and the right-hand side (same object, overlapping views,
 * expressions referencing the destination, destinations resized by the
 * assignment), and \c .noalias() skips the temporary under the usual caller
 * promise that no aliasing exists. */
template <typename Op, typename Rhs>
struct structured_product_impl : generic_product_impl_base<Op, Rhs, structured_product_impl<Op, Rhs>> {
  using Scalar = typename Product<Op, Rhs>::Scalar;

  template <typename Dest>
  static void evalTo(Dest& dst, const Op& lhs, const Rhs& rhs) {
    dst.setZero();
    scaleAndAddTo(dst, lhs, rhs, Scalar(1));
  }

  template <typename Dest>
  static void scaleAndAddTo(Dest& dst, const Op& lhs, const Rhs& rhs, const Scalar& alpha) {
    using RhsNested = typename nested_eval<Rhs, Op::RowsAtCompileTime>::type;
    RhsNested actualRhs(rhs);
    lhs.addProduct(dst, actualRhs, alpha);
  }
};

}  // namespace internal

}  // namespace Eigen

#endif  // EIGEN_STRUCTURED_MATRIX_UTILS_H
