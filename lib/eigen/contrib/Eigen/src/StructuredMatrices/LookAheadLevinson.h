// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_STRUCTURED_LOOK_AHEAD_LEVINSON_H
#define EIGEN_STRUCTURED_LOOK_AHEAD_LEVINSON_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

template <typename Scalar_>
class LookAheadLevinson;

namespace internal {

template <typename Scalar_>
struct traits<LookAheadLevinson<Scalar_>> : traits<Matrix<Scalar_, Dynamic, Dynamic>> {
  using XprKind = MatrixXpr;
  using StorageKind = SolverStorage;
  using StorageIndex = int;
  using BaseTraits = traits<Matrix<Scalar_, Dynamic, Dynamic>>;
  static constexpr int Flags = BaseTraits::Flags & RowMajorBit;
  static constexpr int CoeffReadCost = Dynamic;
};

// Persymmetry E_n T_n E_n = T_n^T requires sum_i a_i b_i, not the conjugating dot().
template <typename D1, typename D2>
typename D1::Scalar structured_tdot(const MatrixBase<D1>& a, const MatrixBase<D2>& b) {
  return a.cwiseProduct(b).sum();
}

template <typename Scalar>
Matrix<Scalar, Dynamic, 1> structured_upshift(const Matrix<Scalar, Dynamic, 1>& v) {
  const Index k = v.size();
  Matrix<Scalar, Dynamic, 1> w = Matrix<Scalar, Dynamic, 1>::Zero(k);
  if (k > 1) w.head(k - 1) = v.tail(k - 1);
  return w;
}

}  // namespace internal

/** \ingroup StructuredMatrices_Module
 * \class LookAheadLevinson
 * \brief Look-ahead Levinson direct solver for general Toeplitz systems.
 *
 * Solves \c T*x = b for a square \ref Toeplitz matrix \c T in O(n^2) operations.
 * This is an implementation of the look-ahead Levinson algorithm of T. F. Chan and
 * P. C. Hansen, which extends the classical Levinson recursion to remain (weakly)
 * numerically stable for general — including indefinite and ill-conditioned —
 * Toeplitz matrices. When the recursion would otherwise break down at a
 * near-singular leading principal submatrix, the algorithm "looks ahead" and takes
 * a block step (up to \ref maxBlockSize) over it. As a by-product it produces an
 * estimate of the matrix condition number (\ref conditionEstimate).
 *
 * The class derives from \c SolverBase and follows the usual decomposition style;
 * transposed and adjoint systems reuse the factorization of \c T through the
 * persymmetry \f$ T^T = E T E \f$ (with \c E the exchange matrix), so all three
 * solves below cost the same:
 * \code
 *   LookAheadLevinson<double> levinson(T);      // or levinson.compute(T);
 *   VectorXd x = levinson.solve(b);              // solve T   * x = b
 *   VectorXd y = levinson.transpose().solve(b);  // solve T^T * y = b
 *   VectorXd z = levinson.adjoint().solve(b);    // solve T^H * z = b
 * \endcode
 *
 * \tparam Scalar_ the scalar type, real or complex.
 *
 * References:
 *  - T. F. Chan and P. C. Hansen, "A look-ahead Levinson algorithm for general
 *    Toeplitz systems," IEEE Trans. Signal Process., 40(5):1079-1090, 1992.
 *  - T. F. Chan and P. C. Hansen, "A look-ahead Levinson algorithm for indefinite
 *    Toeplitz systems," SIAM J. Matrix Anal. Appl., 13(2):490-506, 1992.
 *
 * \sa class Toeplitz
 */
template <typename Scalar_>
class LookAheadLevinson : public SolverBase<LookAheadLevinson<Scalar_>> {
 public:
  using Base = SolverBase<LookAheadLevinson>;
  friend class SolverBase<LookAheadLevinson>;
  EIGEN_GENERIC_PUBLIC_INTERFACE(LookAheadLevinson)
  using DenseVector = Matrix<Scalar, Dynamic, 1>;
  using DenseMatrix = Matrix<Scalar, Dynamic, Dynamic>;

  /** Default constructor; call \ref compute before \ref solve. */
  LookAheadLevinson() : m_maxBlockSize(4), m_n(0), m_isInitialized(false), m_info(InvalidInput) {}

  /** Constructs and factorizes from the Toeplitz matrix \a T, which may have fixed
   * or dynamic dimensions. */
  template <int Rows_, int Cols_>
  explicit LookAheadLevinson(const Toeplitz<Scalar, Rows_, Cols_>& T)
      : m_maxBlockSize(4), m_n(0), m_isInitialized(false), m_info(InvalidInput) {
    compute(T);
  }

  /** Sets the maximum look-ahead block size \f$ p_{\max} \f$ (default 4). The solver
   * is numerically stable as long as \c T has no more than \c p-1 consecutive
   * ill-conditioned leading principal submatrices. Must be called before \ref compute. */
  LookAheadLevinson& setMaxBlockSize(Index p) {
    eigen_assert(p >= 1);
    m_maxBlockSize = p;
    return *this;
  }

  /** \returns the maximum look-ahead block size \f$ p_{\max} \f$. \sa setMaxBlockSize */
  Index maxBlockSize() const { return m_maxBlockSize; }

  Index rows() const noexcept { return m_n; }
  Index cols() const noexcept { return m_n; }

  /** Factorizes the square Toeplitz matrix \a T. \sa solve */
  template <int Rows_, int Cols_>
  LookAheadLevinson& compute(const Toeplitz<Scalar, Rows_, Cols_>& T);

#ifdef EIGEN_PARSED_BY_DOXYGEN
  /** \returns the solution \c x of \c T*x = \a b, as a lazily evaluated expression.
   * Supports multiple right-hand sides. The transposed and adjoint systems are
   * solved at the same cost, and with the same factorization, through
   * \c transpose().solve(b) and \c adjoint().solve(b).
   * \pre \ref compute has been called. */
  template <typename Rhs>
  inline const Solve<LookAheadLevinson, Rhs> solve(const MatrixBase<Rhs>& b) const;
#endif

#ifndef EIGEN_PARSED_BY_DOXYGEN
  /** \internal Evaluates the solution of \c T*x = rhs into \a dst; called through
   * the \c Solve expression returned by \c SolverBase::solve. The copy must be
   * coefficient-wise (noalias), never a move: when the Solve expression is
   * evaluated inside a larger expression, \c evaluator<Solve> caches pointers
   * into \a dst before calling this, and a move-assignment of the returned
   * matrix (same plain type for a matrix right-hand side) would steal its buffer
   * and leave those pointers dangling. */
  template <typename RhsType, typename DstType>
  void _solve_impl(const RhsType& rhs, DstType& dst) const {
    dst.noalias() = solveForward(rhs);
  }

  /** \internal Persymmetry E T E = T^T (with E the exchange matrix) turns the
   * transposed system T^T x = b into T (E x) = E b, so a transposed solve is a
   * forward solve with row-reversed right-hand side and solution -- conjugated on
   * the way in and out for the adjoint -- reusing the factorization of T as is. */
  template <bool Conjugate, typename RhsType, typename DstType>
  void _solve_impl_transposed(const RhsType& rhs, DstType& dst) const {
    const DenseMatrix reversed = rhs.template conjugateIf<Conjugate>().colwise().reverse();
    dst = solveForward(reversed).colwise().reverse().template conjugateIf<Conjugate>();
  }
#endif

  /** \returns \c Success if the algorithm could skip all ill-conditioned leading
   * submatrices within the look-ahead range, \c NumericalIssue otherwise. */
  ComputationInfo info() const {
    eigen_assert(m_isInitialized && "LookAheadLevinson is not initialized.");
    return m_info;
  }

  /** \returns an estimate of the 2-norm condition number of \c T, available after
   * \ref compute (the "algorithm condition number" of Chan and Hansen). */
  RealScalar conditionEstimate() const {
    eigen_assert(m_isInitialized && "LookAheadLevinson is not initialized.");
    return m_sMin > RealScalar(0) ? m_normEst / m_sMin : NumTraits<RealScalar>::infinity();
  }

 private:
  /** \internal \returns the solution of the forward system \c T*x = b by advancing
   * x through the recorded steps of the factorization. */
  template <typename Rhs>
  DenseMatrix solveForward(const Rhs& b) const {
    const Index nrhs = b.cols();
    DenseMatrix x = m_luInit.solve(b.topRows(m_k0));
    DenseMatrix rhs, a, xn;
    for (const Step& s : m_steps) {
      rhs = b.middleRows(s.k, s.p) - s.WspEk * x;  // (27): b_p - S_p^T E_k x_k
      a = s.luGamma.solve(rhs);
      xn.resize(s.k + s.p, nrhs);
      xn.topRows(s.k) = x + s.EkYp * a;  // [E_k Y_p; I_p] a_p
      xn.bottomRows(s.p) = a;
      x = xn;
    }
    return x;
  }

  // Per accepted block step, the b-independent data needed to advance x in solve().
  struct Step {
    Index k;            // order before the step
    Index p;            // block size
    DenseMatrix EkYp;   // E_k Y_p          (k x p)
    DenseMatrix WspEk;  // S_p^T E_k        (p x k)
    FullPivLU<DenseMatrix> luGamma;
  };

  // Leading p x p principal submatrix built directly from the generators.
  static DenseMatrix leadingBlock(const DenseVector& c, const DenseVector& r, Index p) {
    DenseMatrix B(p, p);
    for (Index j = 0; j < p; ++j)
      for (Index i = 0; i < p; ++i) B(i, j) = (i >= j) ? c[i - j] : r[j - i];
    return B;
  }

  // Builds a (len x cols) block whose column j is the length-len generator segment
  // v.segment(first + j, len); used to assemble the S_p and R_p shift blocks.
  static DenseMatrix shiftBlock(const DenseVector& v, Index first, Index len, Index cols) {
    DenseMatrix B(len, cols);
    for (Index j = 0; j < cols; ++j) B.col(j) = v.segment(first + j, len);
    return B;
  }

  static RealScalar smallestSingularValue(const DenseMatrix& M) {
    if (M.rows() == 1) return numext::abs(M(0, 0));
    JacobiSVD<DenseMatrix> svd(M);
    return svd.singularValues()(svd.singularValues().size() - 1);
  }

  Index m_maxBlockSize;
  Index m_n;
  Index m_k0;
  FullPivLU<DenseMatrix> m_luInit;  // LU of the leading T_{k0}
  std::vector<Step> m_steps;
  bool m_isInitialized;
  ComputationInfo m_info;
  RealScalar m_sMin;
  RealScalar m_normEst;
};

template <typename Scalar_>
template <int Rows_, int Cols_>
LookAheadLevinson<Scalar_>& LookAheadLevinson<Scalar_>::compute(const Toeplitz<Scalar, Rows_, Cols_>& T) {
  using internal::structured_tdot;
  using internal::structured_upshift;

  EIGEN_STATIC_ASSERT(Rows_ == Dynamic || Cols_ == Dynamic || Rows_ == Cols_, YOU_MIXED_MATRICES_OF_DIFFERENT_SIZES)
  eigen_assert(T.rows() == T.cols() && "LookAheadLevinson requires a square Toeplitz matrix");
  const Index n = T.rows();
  const DenseVector c = T.column();  // (rho_0, sigma_1, ..., sigma_{n-1})
  const DenseVector r = T.row();     // (rho_0, rho_1,   ..., rho_{n-1})
  const Scalar rho0 = c[0];

  m_n = n;
  m_steps.clear();
  m_info = Success;
  m_normEst = c.cwiseAbs().sum() + r.cwiseAbs().sum();  // cheap upper bound on ||T||_2

  Index k;
  DenseVector y, z;
  Scalar gamma(0);
  {
    Index kbest = 1;
    RealScalar best = RealScalar(-1);
    const Index kmax = numext::mini<Index>(m_maxBlockSize, n);
    for (Index i = 1; i <= kmax; ++i) {
      RealScalar sv = smallestSingularValue(leadingBlock(c, r, i));
      if (sv > best) {
        best = sv;
        kbest = i;
      }
    }
    k = m_k0 = kbest;
    m_luInit.compute(leadingBlock(c, r, k));
    m_sMin = best;
  }

  bool lastWasBlock = false;
  DenseVector yPrev, zPrev;
  Scalar gammaPrev(0);
  Index kp = 0, pp = 0;
  DenseMatrix Ypp, Zpp;
  FullPivLU<DenseMatrix> luGammaPP;
  DenseVector y_kp_1, z_kp_1;

  if (k < n) {
    y = m_luInit.transpose().solve((-r.segment(1, k)).eval());  // T_k^T y = -r_k
    z = m_luInit.solve((-c.segment(1, k)).eval());              // T_k z   = -s_k
    gamma = rho0 + structured_tdot(c.segment(1, k), y);
    if (k == 1) {
      lastWasBlock = false;
      yPrev = DenseVector::Zero(0);
      zPrev = DenseVector::Zero(0);
      gammaPrev = rho0;  // gamma^{(0)}
    } else {
      lastWasBlock = true;  // treat init as a block step of size k from order 0
      kp = 0;
      pp = k;
      Ypp = DenseMatrix::Zero(0, k);
      Zpp = DenseMatrix::Zero(0, k);
      luGammaPP = m_luInit;
      y_kp_1 = DenseVector::Zero(0);
      z_kp_1 = DenseVector::Zero(0);
    }
  }

  while (k < n) {
    const Index pcap = numext::mini<Index>(m_maxBlockSize, n - k);
    const DenseVector Ek_y = y.reverse(), Ek_z = z.reverse();

    DenseVector g_k, h_k;
    bool gh_ready = false;
    auto ensure_gh = [&]() {
      if (gh_ready) return;
      if (!lastWasBlock) {
        // Theorem 1, eq (33)
        g_k.resize(k);
        h_k.resize(k);
        g_k.head(k - 1) = zPrev.reverse();
        g_k[k - 1] = Scalar(1);
        g_k /= gammaPrev;
        h_k.head(k - 1) = yPrev.reverse();
        h_k[k - 1] = Scalar(1);
        h_k /= gammaPrev;
      } else {
        // Theorem 2: y_{k,1}, z_{k,1} (eqs 47-49), then g_k, h_k (eq 50).
        DenseVector rhs_y = -r.segment(kp + 2, pp);
        DenseVector rhs_z = -c.segment(kp + 2, pp);
        if (kp > 0) {
          const DenseMatrix Rpp = shiftBlock(r, 1, kp, pp);
          const DenseMatrix Spp = shiftBlock(c, 1, kp, pp);
          rhs_y.noalias() -= Rpp.transpose() * y_kp_1.reverse();
          rhs_z.noalias() -= Spp.transpose() * z_kp_1.reverse();
        }
        const DenseVector a_y = luGammaPP.transpose().solve(rhs_y);
        const DenseVector a_z = luGammaPP.solve(rhs_z);
        DenseVector yk1 = DenseVector::Zero(k), zk1 = DenseVector::Zero(k);
        if (kp > 0) {
          yk1.head(kp) = y_kp_1 + (Zpp * a_y).reverse();
          zk1.head(kp) = z_kp_1 + (Ypp * a_z).reverse();
        }
        yk1.tail(pp) = a_y;
        zk1.tail(pp) = a_z;
        const Scalar c1 = -r[k + 1] - structured_tdot(r.segment(1, k), Ek_y);
        const Scalar d1 = -c[k + 1] - structured_tdot(c.segment(1, k), Ek_z);
        g_k = (yk1 - structured_upshift<Scalar>(y) + y[0] * y) / c1;
        h_k = (zk1 - structured_upshift<Scalar>(z) + z[0] * z) / d1;
      }
      gh_ready = true;
    };

    // Build Y_p, Z_p incrementally and pick the block size p (eqs 31-32, 46, 39).
    DenseMatrix Yp(k, pcap), Zp(k, pcap);
    Yp.col(0) = y;
    Zp.col(0) = z;
    Index chosen = -1;
    Index bestP = 1;
    RealScalar bestPsi = RealScalar(-1);
    DenseMatrix bestGamma;

    for (Index p = 1; p <= pcap; ++p) {
      if (p >= 2) {
        ensure_gh();
        const Index i = p - 1;
        const Scalar ci = -r[k + i] - structured_tdot(r.segment(i, k), Ek_y);
        const Scalar di = -c[k + i] - structured_tdot(c.segment(i, k), Ek_z);
        Yp.col(i) = structured_upshift<Scalar>(Yp.col(i - 1)) - Yp.col(i - 1)[0] * y + ci * g_k;
        Zp.col(i) = structured_upshift<Scalar>(Zp.col(i - 1)) - Zp.col(i - 1)[0] * z + di * h_k;
      }
      const DenseMatrix Sp = shiftBlock(c, 1, k, p);
      DenseMatrix Gamma = leadingBlock(c, r, p) + Sp.transpose() * Yp.leftCols(p);  // (20)
      const RealScalar muY = Yp.leftCols(p).cwiseAbs().maxCoeff();
      const RealScalar muZ = Zp.leftCols(p).cwiseAbs().maxCoeff();
      const RealScalar denom = numext::maxi(numext::maxi(RealScalar(1), muY), numext::maxi(muZ, muY * muZ));
      const RealScalar psi = smallestSingularValue(Gamma) / denom;  // (46)
      if (psi > bestPsi) {
        bestPsi = psi;
        bestP = p;
        bestGamma = Gamma;
      }
      if (psi > RealScalar(0.1) * m_sMin) {  // (39)
        chosen = p;
        bestGamma = Gamma;
        break;
      }
    }

    DenseMatrix Gamma = bestGamma;
    Index p;
    if (chosen > 0) {
      p = chosen;
    } else {
      p = bestP;
      m_sMin = bestPsi;  // pmax too small to skip cleanly; record the degraded conditioning
    }

    const Index K = k + p;
    const bool finalStep = (K == n);

    const DenseMatrix Sp = shiftBlock(c, 1, k, p);
    const DenseMatrix Rp = shiftBlock(r, 1, k, p);
    FullPivLU<DenseMatrix> luG(Gamma);

    // Record the b-independent data for solve(): rhs = b_p - (S_p^T E_k) x_k ; x update via E_k Y_p.
    Step step;
    step.k = k;
    step.p = p;
    step.EkYp = Yp.leftCols(p).colwise().reverse();   // E_k Y_p
    step.WspEk = Sp.transpose().rowwise().reverse();  // S_p^T E_k
    step.luGamma = luG;
    m_steps.push_back(step);

    if (!finalStep) {
      const DenseVector rhs_e = -r.segment(k + 1, p) - Rp.transpose() * Ek_y;  // (28)
      const DenseVector rhs_f = -c.segment(k + 1, p) - Sp.transpose() * Ek_z;  // (29)
      const DenseVector e_p = luG.transpose().solve(rhs_e);
      const DenseVector f_p = luG.solve(rhs_f);

      DenseVector y_new(K), z_new(K);
      y_new.head(k) = y + (Zp.leftCols(p) * e_p).reverse();  // (25)
      y_new.tail(p) = e_p;
      z_new.head(k) = z + (Yp.leftCols(p) * f_p).reverse();  // (26)
      z_new.tail(p) = f_p;

      const DenseVector y_save = y, z_save = z;
      const Scalar gamma_save = gamma;
      if (p == 1)
        gamma = (Scalar(1) - e_p[0] * f_p[0]) * gamma;  // (15)
      else
        gamma = rho0 + structured_tdot(c.segment(1, K), y_new);  // (5)
      y = y_new;
      z = z_new;

      if (p == 1) {
        lastWasBlock = false;
        yPrev = y_save;
        zPrev = z_save;
        gammaPrev = gamma_save;
      } else {
        lastWasBlock = true;
        kp = k;
        pp = p;
        Ypp = Yp.leftCols(p);
        Zpp = Zp.leftCols(p);
        luGammaPP = luG;
        y_kp_1 = Yp.col(1);
        z_kp_1 = Zp.col(1);
      }
    }
    k = K;
  }

  const RealScalar kappa = m_sMin > RealScalar(0) ? m_normEst / m_sMin : NumTraits<RealScalar>::infinity();
  if (!(kappa * NumTraits<RealScalar>::epsilon() < RealScalar(1))) m_info = NumericalIssue;
  m_isInitialized = true;
  return *this;
}

}  // namespace Eigen

#endif  // EIGEN_STRUCTURED_LOOK_AHEAD_LEVINSON_H
