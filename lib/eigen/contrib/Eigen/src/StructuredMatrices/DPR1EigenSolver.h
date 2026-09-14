// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_STRUCTURED_DPR1_EIGEN_SOLVER_H
#define EIGEN_STRUCTURED_DPR1_EIGEN_SOLVER_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

/** \ingroup StructuredMatrices_Module
 * \class DPR1EigenSolver
 * \brief Direct O(n^2) eigensolver for real symmetric diagonal-plus-rank-one
 * matrices \f$ A = D + \rho\, z z^T \f$, via the secular equation.
 *
 * This is the standalone version of the kernel at the heart of the
 * divide-and-conquer symmetric eigensolvers (LAPACK's xLAED2/3/4): after
 * \em deflation -- entries with negligible \f$ |z_i| \f$ are eigenpairs of the
 * diagonal already, and (nearly) equal diagonal entries are combined by Givens
 * rotations whose dropped coupling is below a backward-stability threshold --
 * the surviving eigenvalues are the roots of the secular equation
 * \f[ f(\lambda) = 1 + \rho \sum_i \frac{z_i^2}{d_i - \lambda} = 0, \f]
 * one in each interval between consecutive poles. Each root is bracketed and
 * bisected in coordinates \em shifted to its nearest pole, so every distance
 * \f$ \lambda - d_i \f$ is retained as an exact data difference plus a small
 * offset instead of a cancellation-prone subtraction of close numbers.
 * Eigenvectors are then built not from the original \c z but from the
 * Gu-Eisenstat vector \f$ \hat z \f$ -- the one for which the computed roots
 * are \em exact secular eigenvalues -- which is what makes the computed
 * eigenvector matrix numerically orthogonal without any reorthogonalization.
 *
 * The total cost is O(n^2): O(n log(1/eps)) per bisected root in the common
 * case (the iteration cap is sized to the scalar's full exponent range, so
 * even roots subnormally close to their pole resolve), O(n) per Gu-Eisenstat
 * weight, and O(n) per eigenvector (the deflation rotations are replayed
 * instead of accumulated into a dense matrix).
 *
 * Both signs of \f$ \rho \f$ are supported (negative \f$ \rho \f$ is handled by
 * negating the matrix), as are \f$ \rho = 0 \f$, zero \c z, repeated diagonal
 * entries and any ordering of \c d. The problem is rescaled internally by
 * exact powers of two chosen from the component exponents alone, so data
 * anywhere in the representable range is handled without internal overflow --
 * including inputs for which \f$ \rho \|z\|^2 \f$ alone exceeds the largest
 * finite value while every eigenvalue is still representable. \c InvalidInput
 * is reported only for non-finite input or a spectrum that is itself not
 * representable. On well-scaled data the rescaling changes no bits of the
 * result (rounding can occur only at the subnormal boundary, far below the
 * deflation backward-error budget).
 *
 * \code
 *   DPR1EigenSolver<double> es(d, rho, z);
 *   VectorXd  lambda = es.eigenvalues();   // ascending
 *   MatrixXd  V      = es.eigenvectors();  // orthogonal
 * \endcode
 *
 * \tparam RealScalar_ one of \c float, \c double, or \c long \c double.
 *
 * References:
 *  - M. Gu and S. C. Eisenstat, "A stable and efficient algorithm for the
 *    rank-one modification of the symmetric eigenproblem," SIAM J. Matrix Anal.
 *    Appl., 15(4):1266-1276, 1994.
 *  - N. J. Higham, "Accuracy and Stability of Numerical Algorithms", 2nd ed.,
 *    SIAM, 2002, chapter 27. Avoiding spurious overflow by rescaling with
 *    powers of two: the whole-problem scale chosen from component exponents
 *    and the (mantissa, exponent) handling of rho*||z||^2 follow this
 *    technique.
 *  - P. H. Sterbenz, "Floating-Point Computation", Prentice-Hall, 1974.
 *    Scaling by a power of two is exact, the property the frexp/ldexp
 *    problem scaling and the full-range ArrayBase::ldexp unscaling of the
 *    eigenvalues rely on.
 *
 * \sa class SelfAdjointEigenSolver
 */
template <typename RealScalar_>
class DPR1EigenSolver {
 public:
  using RealScalar = RealScalar_;
  using Scalar = RealScalar;
  using Index = Eigen::Index;
  using VectorType = Matrix<RealScalar, Dynamic, 1>;
  using MatrixType = Matrix<RealScalar, Dynamic, Dynamic>;

  static_assert(std::is_same<RealScalar, float>::value || std::is_same<RealScalar, double>::value ||
                    std::is_same<RealScalar, long double>::value,
                "DPR1EigenSolver supports only float, double, and long double scalar types.");

  /** Default constructor; call \ref compute before querying results. */
  DPR1EigenSolver() = default;

  /** Computes the eigendecomposition of \c diag(d) + \c rho*z*z^T.
   * \a options is \c ComputeEigenvectors (the default) or \c EigenvaluesOnly. */
  DPR1EigenSolver(const VectorType& d, RealScalar rho, const VectorType& z, int options = ComputeEigenvectors) {
    compute(d, rho, z, options);
  }

  /** Computes the eigendecomposition of \c diag(d) + \c rho*z*z^T. \sa DPR1EigenSolver() */
  DPR1EigenSolver& compute(const VectorType& d, RealScalar rho, const VectorType& z, int options = ComputeEigenvectors);

  /** \returns the eigenvalues, sorted in increasing order. */
  const VectorType& eigenvalues() const {
    eigen_assert(m_isInitialized && "DPR1EigenSolver is not initialized.");
    return m_eivalues;
  }

  /** \returns the orthogonal matrix of eigenvectors; column \c k matches
   * \c eigenvalues()[k]. \pre \ref compute was called with \c ComputeEigenvectors. */
  const MatrixType& eigenvectors() const {
    eigen_assert(m_isInitialized && "DPR1EigenSolver is not initialized.");
    eigen_assert(m_vectorsComputed && "eigenvectors were not computed");
    return m_eivec;
  }

  /** \returns \c Success if the decomposition succeeded, \c NoConvergence if a
   * secular root or the spectrum's finite-range boundary could not be fully
   * resolved, \c InvalidInput if the input was non-finite or its spectrum is
   * not representable in the scalar type. */
  ComputationInfo info() const {
    eigen_assert(m_isInitialized && "DPR1EigenSolver is not initialized.");
    return m_info;
  }

 private:
  // A deflation-stage Givens rotation acting on working rows (i, j).
  struct Rotation {
    Index i, j;
    RealScalar c, s;
  };

  struct DoubleWord {
    RealScalar hi, lo;
  };

  enum class SpectrumRange { Representable, ExactBoundary, Overflow, Uncertain };

  static DoubleWord addDoubleWords(const DoubleWord& x, const DoubleWord& y) {
    DoubleWord result;
    internal::twosum(x.hi, x.lo, y.hi, y.lo, result.hi, result.lo);
    return result;
  }

  static DoubleWord multiplyDoubleWords(const DoubleWord& x, const DoubleWord& y) {
    DoubleWord result;
    internal::twoprod(x.hi, x.lo, y.hi, y.lo, result.hi, result.lo);
    return result;
  }

  static DoubleWord divideDoubleWords(const DoubleWord& x, const DoubleWord& y) {
    DoubleWord quotient;
    internal::doubleword_div_fp(x.hi, x.lo, y.hi, quotient.hi, quotient.lo);
    const DoubleWord product = multiplyDoubleWords(quotient, y);
    const DoubleWord remainder = addDoubleWords(x, DoubleWord{-product.hi, -product.lo});
    DoubleWord correction;
    internal::doubleword_div_fp(remainder.hi, remainder.lo, y.hi, correction.hi, correction.lo);
    return addDoubleWords(quotient, correction);
  }

  static DoubleWord scaleDoubleWord(const DoubleWord& x, int exponent) {
    EIGEN_USING_STD(ldexp)
    DoubleWord result{ldexp(x.hi, exponent), ldexp(x.lo, exponent)};
    if ((numext::isfinite)(result.hi)) {
      DoubleWord normalized;
      internal::fast_twosum(result.hi, result.lo, normalized.hi, normalized.lo);
      result = normalized;
    }
    return result;
  }

  static SpectrumRange classifySpectrumRange(const VectorType& d, RealScalar rho, const VectorType& z);

  /** \internal Evaluates the shifted secular function
   * g(tau) = 1 + rho * sum_i zeta_i^2 / (delta_i - tau), with delta_i the pole
   * offsets relative to the chosen shift. */
  static RealScalar secular(const VectorType& delta, const VectorType& zeta2, RealScalar rho, RealScalar tau) {
    return RealScalar(1) + rho * (zeta2.array() / (delta.array() - tau)).sum();
  }

  VectorType m_eivalues;
  MatrixType m_eivec;
  bool m_isInitialized = false;
  bool m_vectorsComputed = false;
  ComputationInfo m_info = InvalidInput;
};

template <typename RealScalar_>
typename DPR1EigenSolver<RealScalar_>::SpectrumRange DPR1EigenSolver<RealScalar_>::classifySpectrumRange(
    const VectorType& d, RealScalar rho, const VectorType& z) {
  // For rho >= 0 only the largest eigenvalue can leave the finite range; with M = highest(), the
  // matrix determinant lemma gives lambda_max <= M iff rho * sum_i z_i^2 / (M - d_i) <= 1. Evaluated
  // from the original data, not the normalized secular problem, which has already rounded rho*||z||^2.
  // Double-word arithmetic and its O(u^2) bounds follow Joldes, Muller and Popescu, "Tight and
  // rigorous error bounds for basic building blocks of double-word arithmetic", ACM TOMS 44(2), 2017.
  if (rho == RealScalar(0)) return SpectrumRange::Representable;

  EIGEN_USING_STD(frexp)
  EIGEN_USING_STD(ldexp)
  const RealScalar highest = (std::numeric_limits<RealScalar>::max)();
  const RealScalar highestHalf = highest / RealScalar(2);
  int rhoExponent = 0;
  const RealScalar rhoFraction = frexp(rho, &rhoExponent);
  DoubleWord sum{RealScalar(0), RealScalar(0)};
  bool exactSumValid = true;
  Index active = 0;

  for (Index i = 0; i < d.size(); ++i) {
    if (z[i] == RealScalar(0)) continue;
    ++active;
    if (d[i] == highest) return SpectrumRange::Overflow;

    int zExponent = 0;
    const RealScalar zFraction = frexp(numext::abs(z[i]), &zExponent);
    DoubleWord numerator;
    internal::twoprod(rhoFraction, zFraction, numerator.hi, numerator.lo);
    const bool numeratorExact = numerator.lo == RealScalar(0) || zFraction == RealScalar(0.5);
    if (zFraction == RealScalar(0.5)) {
      numerator.hi *= zFraction;
      numerator.lo *= zFraction;
      DoubleWord normalized;
      internal::fast_twosum(numerator.hi, numerator.lo, normalized.hi, normalized.lo);
      numerator = normalized;
    } else if (numerator.lo == RealScalar(0)) {
      DoubleWord product;
      internal::twoprod(numerator.hi, zFraction, product.hi, product.lo);
      numerator = product;
    } else {
      DoubleWord product;
      internal::twoprod(numerator.hi, numerator.lo, zFraction, product.hi, product.lo);
      numerator = product;
    }

    // Work with (M - d_i)/2, which cannot overflow even when d_i = -M.
    const RealScalar dHalf = d[i] / RealScalar(2);
    DoubleWord denominator;
    internal::twosum(highestHalf, RealScalar(0), -dHalf, RealScalar(0), denominator.hi, denominator.lo);
    const bool denominatorExact = d[i] == RealScalar(0) || dHalf * RealScalar(2) == d[i];
    const DoubleWord unscaledDenominator = denominator;
    int denominatorExponent = 0;
    frexp(unscaledDenominator.hi, &denominatorExponent);
    const RealScalar scaledDenominatorHi = ldexp(unscaledDenominator.hi, -denominatorExponent);
    const RealScalar scaledDenominatorLo = ldexp(unscaledDenominator.lo, -denominatorExponent);
    const bool denominatorScaleExact = ldexp(scaledDenominatorHi, denominatorExponent) == unscaledDenominator.hi &&
                                       ldexp(scaledDenominatorLo, denominatorExponent) == unscaledDenominator.lo;
    denominator = scaleDoubleWord(unscaledDenominator, -denominatorExponent);

    const int termExponent = rhoExponent + 2 * zExponent - denominatorExponent - 1;
    const DoubleWord quotient = divideDoubleWords(numerator, denominator);
    const DoubleWord term = scaleDoubleWord(quotient, termExponent);
    if (!(numext::isfinite)(term.hi) || term.hi > RealScalar(2)) return SpectrumRange::Overflow;

    const RealScalar previousSum = sum.hi;
    sum = addDoubleWords(sum, term);
    if (!(numext::isfinite)(sum.hi) || sum.hi > RealScalar(2)) return SpectrumRange::Overflow;

    // Certify an exact endpoint equality only from terms whose division,
    // exponent scaling, and scalar accumulation are all proven exact. This is
    // deliberately stricter than the O(u^2) sign estimate below: a rounded
    // equality must remain Uncertain rather than become a false Success.
    if (exactSumValid) {
      const RealScalar normalMin = (std::numeric_limits<RealScalar>::min)();
      bool exactTerm = numeratorExact && denominatorExact && denominatorScaleExact && denominator.lo == RealScalar(0) &&
                       quotient.lo == RealScalar(0) && term.lo == RealScalar(0) && term.hi >= normalMin &&
                       ldexp(term.hi, -termExponent) == quotient.hi;
      if (exactTerm) {
        DoubleWord recoveredNumerator;
        internal::twoprod(quotient.hi, denominator.hi, recoveredNumerator.hi, recoveredNumerator.lo);
        exactTerm = recoveredNumerator.hi == numerator.hi && recoveredNumerator.lo == numerator.lo;
      }
      if (exactTerm) {
        const RealScalar larger = numext::maxi(previousSum, term.hi);
        const RealScalar smaller = numext::mini(previousSum, term.hi);
        // The reversible subtraction remains a proof if FTZ erases a
        // subnormal low word from the double-word addition.
        exactSumValid = sum.lo == RealScalar(0) && sum.hi - larger == smaller;
      } else {
        exactSumValid = false;
      }
    }
  }

  if (active == 0) return SpectrumRange::Representable;
  if (exactSumValid && sum.hi == RealScalar(1)) return SpectrumRange::ExactBoundary;

  const DoubleWord difference = addDoubleWords(sum, DoubleWord{RealScalar(-1), RealScalar(0)});
  const RealScalar estimate = difference.hi + difference.lo;
  const RealScalar unitRoundoff = NumTraits<RealScalar>::epsilon() / RealScalar(2);
  // The factor 32 covers two products, corrected division, exponent scaling,
  // and accumulation per active term. Proven exact endpoint equality is handled
  // above rather than widened into the uncertainty interval.
  const RealScalar error = RealScalar(32) * RealScalar(active + 1) * unitRoundoff * unitRoundoff;
  if (estimate < -error) return SpectrumRange::Representable;
  if (estimate > error) return SpectrumRange::Overflow;
  return SpectrumRange::Uncertain;
}

template <typename RealScalar_>
DPR1EigenSolver<RealScalar_>& DPR1EigenSolver<RealScalar_>::compute(const VectorType& d, RealScalar rho,
                                                                    const VectorType& z, int options) {
  const Index n = d.size();
  eigen_assert(z.size() == n && "d and z must have the same size");
  eigen_assert((options & ~EigVecMask) == 0 && (options & EigVecMask) != EigVecMask && "invalid option parameter");
  const bool computeVectors = (options & ComputeEigenvectors) == ComputeEigenvectors;
  m_vectorsComputed = false;
  m_info = Success;

  m_eivalues.resize(n);
  if (computeVectors) m_eivec.setIdentity(n, n);

  // Non-finite input would break the sorting comparator (not a strict weak
  // order under NaN) and silently deflate everything; reject it up front.
  if (!(d.allFinite() && z.allFinite() && (numext::isfinite)(rho))) {
    m_eivalues.setConstant(NumTraits<RealScalar>::quiet_NaN());
    m_info = InvalidInput;
    m_isInitialized = true;
    return *this;
  }
  if (n == 0) {
    m_vectorsComputed = computeVectors;
    m_isInitialized = true;
    return *this;
  }

  const bool negated = rho < RealScalar(0);
  VectorType dW = negated ? VectorType(-d) : d;
  RealScalar rhoW = negated ? -rho : rho;

  // pi maps each sorted working index to its input row.
  std::vector<Index> pi;
  pi.reserve(static_cast<std::size_t>(n));
  for (Index i = 0; i < n; ++i) pi.push_back(i);
  std::stable_sort(pi.begin(), pi.end(), [&dW](Index a, Index b) { return dW[a] < dW[b]; });
  VectorType ds(n), zs(n);
  for (Index i = 0; i < n; ++i) {
    ds[i] = dW[pi[static_cast<std::size_t>(i)]];
    zs[i] = z[pi[static_cast<std::size_t>(i)]];
  }

  // Normalize z and absorb ||z||^2 into rho. Power-of-two scaling is exact away
  // from the subnormal boundary, unlike the general scaling in LAPACK xLAED*.
  EIGEN_USING_STD(frexp)
  EIGEN_USING_STD(ldexp)
  // With max|z_i| < 2^zExp, ||2^-zExp z|| is representable even if ||z|| is not.
  int zExp = 0;
  const RealScalar zmax = zs.cwiseAbs().maxCoeff();
  if (zmax > RealScalar(0)) {
    frexp(zmax, &zExp);
    if (zExp != 0) zs.array() = zs.array().ldexp(-zExp);
  }
  const RealScalar znorm = zs.stableNorm();  // in [0.5, sqrt(n)): safe
  int znormExp = 0;
  const RealScalar znormFrac = frexp(znorm, &znormExp);
  if (znorm > RealScalar(0)) zs /= znorm;
  // Store rho ||z||^2 = rhoMant * 2^rhoTotExp without materializing a possibly
  // overflowing product; each mantissa factor lies in [1/4,1).
  int rhoExp = 0;
  const RealScalar rhoFrac = frexp(rhoW, &rhoExp);
  int rhoAdj = 0;
  const RealScalar rhoMant = frexp((rhoFrac * znormFrac) * znormFrac, &rhoAdj);  // in [0.5, 1), or 0
  // Each frexp exponent is bounded by the scalar's exponent range, but their
  // accumulation is kept in a wide integer type until the (clamped) narrowing
  // to the int that ldexp takes.
  const numext::int64_t rhoTotExp =
      numext::int64_t(rhoExp) + 2 * (numext::int64_t(zExp) + numext::int64_t(znormExp)) + numext::int64_t(rhoAdj);
  // For s = 2^-scaleExp, eig(sD + s rho zz^T) = s eig(D + rho zz^T).
  // Choose s so max(||sD||_inf, s rho ||z||^2) lies in [1/2,1), comparing the
  // two scales as mantissa-exponent pairs to avoid overflow.
  int dExp = 0;
  const RealScalar dFrac = frexp(ds.cwiseAbs().maxCoeff(), &dExp);
  RealScalar scaledNorm;  // max(|d|_inf, rho*||z||^2) * 2^-scaleExp, in [0.5, 1) (or 0 for a zero matrix)
  numext::int64_t scaleExpWide;
  if (rhoMant == RealScalar(0) ||
      (dFrac > RealScalar(0) &&
       (numext::int64_t(dExp) > rhoTotExp || (numext::int64_t(dExp) == rhoTotExp && dFrac >= rhoMant)))) {
    scaledNorm = dFrac;
    scaleExpWide = dExp;
  } else {
    scaledNorm = rhoMant;
    scaleExpWide = rhoTotExp;
  }
  // Clamp before narrowing: 2^(2^30) is far beyond any scalar's exponent
  // range, so the clamp never changes which values are representable.
  const numext::int64_t expCap = numext::int64_t(1) << 30;
  const int scaleExp = static_cast<int>(numext::maxi(-expCap, numext::mini(expCap, scaleExpWide)));
  SpectrumRange spectrumRange = SpectrumRange::Representable;
  if (scaleExpWide >= numext::int64_t(std::numeric_limits<RealScalar>::max_exponent) - 1) {
    spectrumRange = classifySpectrumRange(dW, rhoW, z);
  }
  if (scaleExp != 0) ds.array() = ds.array().ldexp(-scaleExp);
  // Materialize rho * ||z||^2 only in scaled form: its exponent is <= 0 by the
  // choice of scaleExp, so this cannot overflow; it can only underflow when
  // the update is negligible against |d|_inf, in which case it deflates below.
  rhoW = ldexp(rhoMant, static_cast<int>(numext::maxi(-expCap, rhoTotExp - scaleExpWide)));

  // Backward-error budget: dropping a coupling of size <= tol perturbs the
  // matrix by O(tol), like LAPACK's xLAED2. Using max(|d|_inf, rho*||z||^2) as
  // the scale is the unscaled-problem generalization of xLAED2's criterion: the
  // perturbation stays O(eps * (||D|| + rho ||z||^2)), i.e. backward stable in
  // the data.
  const RealScalar tol = RealScalar(8) * NumTraits<RealScalar>::epsilon() * scaledNorm;

  std::vector<Rotation> rotations;
  std::vector<bool> deflated;

  if (rhoW <= tol) {
    deflated.assign(static_cast<std::size_t>(n), true);
  } else {
    // Negligible z_i leaves d_i as an eigenvalue.
    deflated.reserve(static_cast<std::size_t>(n));
    for (Index i = 0; i < n; ++i) deflated.push_back(rhoW * numext::abs(zs[i]) <= tol);
    // The dropped off-diagonal coupling is |c*s*(ds[i]-ds[p])| <= tol.
    Index p = -1;
    for (Index i = 0; i < n; ++i) {
      if (deflated[static_cast<std::size_t>(i)]) continue;
      if (p >= 0) {
        const RealScalar r = numext::hypot(zs[p], zs[i]);
        const RealScalar c = zs[i] / r, s = zs[p] / r;  // zeroes the earlier entry
        if (numext::abs(c * s * (ds[i] - ds[p])) <= tol) {
          // Rotated diagonal: the deflated position keeps one entry, the
          // surviving pole the other; both stay inside [ds[p], ds[i]].
          const RealScalar dp = c * c * ds[p] + s * s * ds[i];
          const RealScalar di = s * s * ds[p] + c * c * ds[i];
          ds[p] = dp;
          ds[i] = di;
          zs[i] = r;
          zs[p] = RealScalar(0);
          deflated[static_cast<std::size_t>(p)] = true;
          if (computeVectors) rotations.push_back(Rotation{p, i, c, s});
        }
      }
      if (!deflated[static_cast<std::size_t>(i)]) p = i;
    }
  }

  std::vector<Index> sub;  // working positions of the surviving poles
  for (Index i = 0; i < n; ++i)
    if (!deflated[static_cast<std::size_t>(i)]) sub.push_back(i);
  const Index m = static_cast<Index>(sub.size());

  VectorType lambdaW = ds;

  MatrixType subVectors;  // m x m secular eigenvectors (in subproblem coordinates)

  if (m > 0) {
    VectorType delta(m), zeta(m), zeta2(m);
    std::vector<Index> shiftIndex(static_cast<std::size_t>(m));
    VectorType tau(m);
    for (Index a = 0; a < m; ++a) {
      delta[a] = ds[sub[static_cast<std::size_t>(a)]];
      zeta[a] = zs[sub[static_cast<std::size_t>(a)]];
    }
    zeta2.array() = zeta.array() * zeta.array();
    const RealScalar zeta2sum = zeta2.sum();

    // Bisection stops when the bracket has collapsed to adjacent floating-point
    // numbers. The backstop covers exponent_range + 2*digits halvings, enough
    // to shrink a unit-width bracket to a subnormal root offset and resolve it.
    const int digits =
        (std::numeric_limits<RealScalar>::digits > 0) ? static_cast<int>(std::numeric_limits<RealScalar>::digits) : 128;
    const int expRange = (std::numeric_limits<RealScalar>::max_exponent > std::numeric_limits<RealScalar>::min_exponent)
                             ? static_cast<int>(std::numeric_limits<RealScalar>::max_exponent) -
                                   static_cast<int>(std::numeric_limits<RealScalar>::min_exponent)
                             : 16 * digits;
    const int maxBisect = expRange + 2 * digits + 32;
    VectorType lam(m), dsh(m);  // dsh is refilled from scratch each root
    for (Index k = 0; k < m; ++k) {
      // Choose the shift pole and the bracket, entirely in shifted coordinates.
      RealScalar lo, hi;
      Index shift;
      if (k + 1 == m) {
        // Last root: it lies in (delta_m, delta_m + rho*|zeta|^2]; never form
        // the unshifted right end (it can round to the pole itself).
        shift = k;
        lo = RealScalar(0);
        hi = rhoW * zeta2sum;
      } else {
        // Interior root: the secular function is increasing between the poles,
        // so its sign at the midpoint picks the nearer pole as the shift.
        const RealScalar left = delta[k], right = delta[k + 1];
        const RealScalar mid = left + (right - left) / RealScalar(2);
        if (secular(delta, zeta2, rhoW, mid) > RealScalar(0)) {
          shift = k;
          lo = RealScalar(0);
          hi = mid - left;
        } else {
          shift = k + 1;
          lo = mid - right;  // negative
          hi = RealScalar(0);
        }
      }
      const RealScalar shiftVal = delta[shift];
      dsh.array() = delta.array() - shiftVal;

      // Bisection: g(lo) < 0 < g(hi) by the pole signs (for shift = k the
      // function tends to -inf as tau -> 0+, for shift = k+1 to +inf as
      // tau -> 0-). The endpoints are never evaluated.
      RealScalar a0 = lo, b0 = hi;
      bool converged = false;
      for (int iter = 0; iter < maxBisect; ++iter) {
        const RealScalar t = a0 + (b0 - a0) / RealScalar(2);
        if (t == a0 || t == b0) {
          converged = true;  // interval fully resolved
          break;
        }
        if (secular(dsh, zeta2, rhoW, t) > RealScalar(0))
          b0 = t;
        else
          a0 = t;
      }
      if (!converged) m_info = NoConvergence;
      const RealScalar t = a0 + (b0 - a0) / RealScalar(2);
      shiftIndex[static_cast<std::size_t>(k)] = shift;
      tau[k] = t;
      lam[k] = shiftVal + t;
    }

    // ---- Gu-Eisenstat weights: the z-vector for which lam are exact roots ----
    // zhat_i^2 = prod_j (lam_j - delta_i) / (rho * prod_{j != i} (delta_j - delta_i)),
    // with every distance lam_j - delta_i formed as (delta_shift(j) - delta_i) + tau_j.
    // Numerator and denominator factors are paired so the running product stays O(1).
    VectorType zhat(m);
    for (Index i = 0; i < m; ++i) {
      RealScalar acc = ((delta[shiftIndex[static_cast<std::size_t>(i)]] - delta[i]) + tau[i]) / rhoW;
      for (Index j = 0; j < m; ++j) {
        if (j == i) continue;
        const RealScalar num = (delta[shiftIndex[static_cast<std::size_t>(j)]] - delta[i]) + tau[j];
        acc *= num / (delta[j] - delta[i]);
      }
      zhat[i] = numext::abs(acc) > RealScalar(0) ? RealScalar(numext::sqrt(numext::abs(acc))) : RealScalar(0);
      if (zeta[i] < RealScalar(0)) zhat[i] = -zhat[i];
    }

    // ---- secular eigenvectors from the Gu-Eisenstat weights ----
    if (computeVectors) {
      subVectors.resize(m, m);
      for (Index j = 0; j < m; ++j) {
        const Index sj = shiftIndex[static_cast<std::size_t>(j)];
        if (tau[j] == RealScalar(0)) {
          // The root coincides with its shift pole to working precision (a
          // fully underflowed bracket): the eigenvector is that pole's axis.
          subVectors.col(j).setZero();
          subVectors(sj, j) = RealScalar(1);
          continue;
        }
        // Divide by the pole distances delta_i - lam_j, each formed as
        // (delta_i - delta_sj) - tau_j.
        subVectors.col(j).array() = zhat.array() / ((delta.array() - delta[sj]) - tau[j]);
        subVectors.col(j).stableNormalize();
      }
    }
    for (Index k = 0; k < m; ++k) lambdaW[sub[static_cast<std::size_t>(k)]] = lam[k];
  }

  std::vector<Index> order;
  order.reserve(static_cast<std::size_t>(n));
  for (Index i = 0; i < n; ++i) order.push_back(i);
  std::stable_sort(order.begin(), order.end(), [&lambdaW](Index a, Index b) { return lambdaW[a] < lambdaW[b]; });

  std::vector<Index> subSlot(static_cast<std::size_t>(n), -1);
  for (Index a = 0; a < m; ++a) subSlot[static_cast<std::size_t>(sub[static_cast<std::size_t>(a)])] = a;

  for (Index t = 0; t < n; ++t) {
    const Index w = order[static_cast<std::size_t>(t)];
    const Index outCol = negated ? n - 1 - t : t;
    m_eivalues[outCol] = negated ? -lambdaW[w] : lambdaW[w];
    if (!computeVectors) continue;

    VectorType wvec = VectorType::Zero(n);
    const Index slot = subSlot[static_cast<std::size_t>(w)];
    if (slot < 0) {
      wvec[w] = RealScalar(1);
    } else {
      for (Index a = 0; a < m; ++a) wvec[sub[static_cast<std::size_t>(a)]] = subVectors(a, slot);
    }
    // w <- G*w with G = [[c, s], [-s, c]]: for exactly equal poles this maps the
    // deflated unit vector e_p to (c, -s) = (z_i, -z_p)/r, the exact eigenvector
    // of the 2x2 block orthogonal to the weight vector.
    for (auto it = rotations.rbegin(); it != rotations.rend(); ++it) {
      const RealScalar wi = wvec[it->i], wj = wvec[it->j];
      wvec[it->i] = it->c * wi + it->s * wj;
      wvec[it->j] = -it->s * wi + it->c * wj;
    }
    for (Index i = 0; i < n; ++i) m_eivec(pi[static_cast<std::size_t>(i)], outCol) = wvec[i];
  }

  // Undo the problem scaling in one full-range pass. ArrayBase::ldexp handles
  // exponents for which 2^scaleExp is not itself representable, preserving
  // representable results exactly and saturating genuine overflow to infinity
  // (validated below).
  if (scaleExp != 0) m_eivalues.array() = m_eivalues.array().ldexp(scaleExp);

  // Range classification uses the original data. The normalized secular
  // problem can round a root across the maximum-finite boundary before this
  // rescaling, so finiteness of the rounded result alone is not evidence that
  // the exact spectrum is (or is not) representable.
  const Index extreme = negated ? 0 : n - 1;
  const RealScalar highest = (std::numeric_limits<RealScalar>::max)();
  if (spectrumRange == SpectrumRange::Overflow) {
    m_eivalues.setConstant(NumTraits<RealScalar>::quiet_NaN());
    m_info = InvalidInput;
  } else {
    // A proven boundary root and a root the rescaling saturated to infinity both belong at the
    // largest finite value: the first is exactly it, the second rounded across it.
    if (spectrumRange == SpectrumRange::ExactBoundary || !(numext::isfinite)(m_eivalues[extreme])) {
      m_eivalues[extreme] = negated ? -highest : highest;
    }
    if (spectrumRange == SpectrumRange::Uncertain && m_info == Success) m_info = NoConvergence;
    if (!m_eivalues.allFinite() && m_info == Success) m_info = NoConvergence;
  }

  m_vectorsComputed = computeVectors && m_info != InvalidInput;
  m_isInitialized = true;
  return *this;
}

}  // namespace Eigen

#endif  // EIGEN_STRUCTURED_DPR1_EIGEN_SOLVER_H
