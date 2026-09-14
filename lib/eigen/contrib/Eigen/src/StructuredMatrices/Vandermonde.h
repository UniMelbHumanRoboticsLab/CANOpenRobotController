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
//      powers of two, the technique behind determinant()'s balanced
//      accumulation and the scaled Horner recurrence of addProduct().
//  [2] P. H. Sterbenz, "Floating-Point Computation", Prentice-Hall, 1974.
//      Scaling by a power of two is exact, the property the balanced
//      accumulation and the scaled Horner recurrence rely on.
//  [3] J. J. Dongarra, J. R. Bunch, C. B. Moler and G. W. Stewart, "LINPACK
//      Users' Guide", SIAM, 1979. determinant()'s balanced accumulation follows
//      the convention of its xGEDI routines, which return determinants as a
//      (fraction, exponent) pair to avoid spurious overflow/underflow.
//  [4] L. Reichel, "Newton Interpolation at Leja Points", BIT 30 (1990),
//      332--346. Leja ordering controls growth in the Newton representation;
//      BjorckPereyra uses it for genuinely complex node sets.

#ifndef EIGEN_STRUCTURED_VANDERMONDE_H
#define EIGEN_STRUCTURED_VANDERMONDE_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

template <typename Scalar_, int Rows_ = Dynamic, int Cols_ = Dynamic>
class Vandermonde;

template <typename Scalar_>
class BjorckPereyra;

namespace internal {

template <typename Scalar_, int Rows_, int Cols_>
struct traits<Vandermonde<Scalar_, Rows_, Cols_>> {
  using Scalar = Scalar_;
  using StorageKind = Dense;
  using XprKind = MatrixXpr;
  using StorageIndex = int;
  static constexpr int RowsAtCompileTime = Rows_;
  static constexpr int ColsAtCompileTime = Cols_;
  static constexpr int MaxRowsAtCompileTime = Rows_;
  static constexpr int MaxColsAtCompileTime = Cols_;
  // Deliberately no NestByRefBit: the makeVandermonde() factories (and any
  // function returning the operator by value) produce owning temporaries, so
  // Product must nest the operator by value for a delayed-evaluated product
  // expression to keep its left factor alive. The copy is O(m), negligible
  // against the O(mn) product evaluation.
  static constexpr int Flags = Rows_ == 1 && Cols_ != 1 ? RowMajorBit : 0;
};

template <typename Scalar_, int Rows_, int Cols_>
struct evaluator_traits<Vandermonde<Scalar_, Rows_, Cols_>> {
  using Kind = IndexBased;
  using Shape = StructuredShape;
};

// Core rewrites alpha * (lhs * rhs) as (alpha * lhs) * rhs. The scaled
// Vandermonde wrapper needs coefficient and BLAS metadata to participate in
// that general dense-expression machinery.
template <typename Scalar_, int Rows_, int Cols_>
struct evaluator<Vandermonde<Scalar_, Rows_, Cols_>> : evaluator_base<Vandermonde<Scalar_, Rows_, Cols_>> {
  using XprType = Vandermonde<Scalar_, Rows_, Cols_>;
  using Scalar = Scalar_;
  static constexpr int CoeffReadCost = HugeCost;
  static constexpr int Flags = traits<XprType>::Flags;
  static constexpr int Alignment = 0;

  EIGEN_DEVICE_FUNC constexpr EIGEN_STRONG_INLINE explicit evaluator(const XprType& xpr) : m_xpr(xpr) {}

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar coeff(Index row, Index col) const { return m_xpr.coeff(row, col); }

 private:
  const XprType& m_xpr;
};

template <typename Scalar_, int Rows_, int Cols_>
struct blas_traits<Vandermonde<Scalar_, Rows_, Cols_>> {
  using XprType = Vandermonde<Scalar_, Rows_, Cols_>;
  using Scalar = Scalar_;
  using ExtractType = const XprType&;
  using ExtractType_ = XprType;
  using DirectLinearAccessType = XprType;
  static constexpr bool IsComplex = NumTraits<Scalar>::IsComplex;
  static constexpr bool IsTransposed = false;
  static constexpr bool NeedToConjugate = false;
  static constexpr bool HasUsableDirectAccess = false;
  static constexpr bool HasScalarFactor = false;
  static EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE ExtractType extract(const XprType& x) { return x; }
  static EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE Scalar extractScalarFactor(const XprType&) { return Scalar(1); }
};

template <typename Scalar_>
struct traits<BjorckPereyra<Scalar_>> : traits<Matrix<Scalar_, Dynamic, Dynamic>> {
  using XprKind = MatrixXpr;
  using StorageKind = SolverStorage;
  using StorageIndex = int;
  using BaseTraits = traits<Matrix<Scalar_, Dynamic, Dynamic>>;
  static constexpr int Flags = BaseTraits::Flags & RowMajorBit;
  static constexpr int CoeffReadCost = Dynamic;
};

}  // namespace internal

/** \ingroup StructuredMatrices_Module
 * \class Vandermonde
 * \brief An \c m x \c n Vandermonde matrix represented by its node vector.
 *
 * A Vandermonde matrix has entry \c (i,j) equal to \f$ x_i^j \f$, where \c x is
 * the node vector. Thus
 * \f[ (Va)_i = \sum_{j=0}^{n-1} a_j x_i^j, \qquad
 *     p_{n-1}=a_{n-1},\quad p_j=a_j+x_i p_{j+1}. \f]
 * The class stores only the \c m nodes; products evaluate this Horner recurrence
 * rule at O(mn) operations -- the same cost as a dense product, but with O(m)
 * storage and without ever forming the matrix.
 *
 * Square systems are solved in O(n^2) by the Björck-Pereyra algorithm (class
 * \ref BjorckPereyra), whose \c transpose().solve() form covers the dual
 * (moment) system. There is no fast transposed \em product, so the class is not
 * closed under transposition and rectangular least-squares problems are best
 * handled by a dense QR of the materialized matrix.
 *
 * Because \c operator* returns an Eigen product expression, a \c Vandermonde
 * also drops into the matrix-free iterative solvers, and it can be assigned to
 * a dense matrix when an explicit representation is needed. As with any
 * matrix-free operator, the iterative solvers must be instantiated with
 * \c IdentityPreconditioner (e.g.
 * \c BiCGSTAB<Vandermonde<double>,IdentityPreconditioner>): the default
 * preconditioners read individual coefficients through \c col() or
 * \c InnerIterator, which the structured operators do not expose.
 *
 * \warning Vandermonde matrices with real nodes are exponentially
 * ill-conditioned: the condition number grows at least like \f$ 2^n \f$ for any
 * real node configuration (Beckermann, 2000). Solves remain surprisingly
 * accurate for monotone node sets and sign-alternating right-hand sides
 * (Björck-Pereyra's celebrated property, see Higham, ASNA ch. 22), but forward
 * errors necessarily scale with the conditioning in general. Complex nodes on
 * the unit circle are the well-conditioned case: for the n-th roots of unity,
 * \f$ V/\sqrt{n} \f$ is unitary.
 *
 * \tparam Scalar_ a floating-point-like real or complex scalar supporting
 * Eigen's scalar math hooks, including \c isfinite, \c frexp and \c ldexp (and
 * \c log for complex solver ordering). Integer types are rejected.
 * \tparam Rows_ the number of rows (nodes) at compile time, or \c Dynamic.
 * \tparam Cols_ the number of columns (powers) at compile time, or \c Dynamic.
 *
 * \sa class BjorckPereyra, makeVandermonde()
 */
template <typename Scalar_, int Rows_, int Cols_>
class Vandermonde : public EigenBase<Vandermonde<Scalar_, Rows_, Cols_>> {
 public:
  using Derived = Vandermonde;
  using StorageBaseType = Vandermonde;
  using Scalar = Scalar_;
  using RealScalar = typename NumTraits<Scalar>::Real;
  using StorageIndex = int;
  static constexpr int NodeOptions = Rows_ == Dynamic ? AutoAlign : DontAlign;
  using NodeVector = Matrix<Scalar, Rows_, 1, NodeOptions>;
  using Nested = Vandermonde;

  static constexpr int RowsAtCompileTime = Rows_;
  static constexpr int ColsAtCompileTime = Cols_;
  static constexpr int MaxRowsAtCompileTime = Rows_;
  static constexpr int MaxColsAtCompileTime = Cols_;
  static constexpr int SizeAtCompileTime = internal::size_at_compile_time(Rows_, Cols_);
  static constexpr int MaxSizeAtCompileTime = SizeAtCompileTime;
  static constexpr int Flags = internal::traits<Vandermonde>::Flags;
  static constexpr bool IsRowMajor = (Flags & RowMajorBit) != 0;
  // Deliberately no IsVectorAtCompileTime: Ref<const Vandermonde>'s default
  // StrideType argument reads it, so its absence makes internal::is_ref_compatible
  // SFINAE to false and keeps the iterative solvers on their matrix-free path.

  EIGEN_STATIC_ASSERT_NON_INTEGER(RealScalar)
  EIGEN_MAKE_SCALAR_BINARY_OP_ONTHELEFT(operator*, internal::scalar_product_op)

  /** Builds an \c m x \a cols Vandermonde matrix from the \c m nodes \a nodes. */
  template <typename Derived>
  Vandermonde(const MatrixBase<Derived>& nodes, Index cols) : m_x(nodes), m_cols(cols) {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived)
    eigen_assert(m_x.size() > 0 && m_cols > 0 && "Vandermonde must be non-empty");
    eigen_assert((Cols_ == Dynamic || Cols_ == cols) && "cols does not match the compile-time column count");
  }

  /** Builds the square Vandermonde matrix of the nodes \a nodes. */
  template <typename Derived>
  explicit Vandermonde(const MatrixBase<Derived>& nodes) : Vandermonde(nodes, nodes.size()) {
    EIGEN_STATIC_ASSERT(Rows_ == Dynamic || Cols_ == Dynamic || Rows_ == Cols_, YOU_MIXED_MATRICES_OF_DIFFERENT_SIZES)
    EIGEN_STATIC_ASSERT(
        Cols_ == Dynamic || Derived::SizeAtCompileTime == Dynamic || Cols_ == Derived::SizeAtCompileTime,
        YOU_MIXED_MATRICES_OF_DIFFERENT_SIZES)
  }

  EIGEN_DEVICE_FUNC Index rows() const { return m_x.size(); }
  EIGEN_DEVICE_FUNC Index cols() const { return m_cols; }

  /** \returns the node vector. */
  const NodeVector& nodes() const { return m_x; }

  /** \returns the coefficient at row \a row and column \a col, \f$ x_i^j \f$. */
  EIGEN_DEVICE_FUNC Scalar coeff(Index row, Index col) const {
    Scalar p(1);
    const Scalar xi = m_x.coeff(row);
    for (Index t = 0; t < col; ++t) p *= xi;
    return p;
  }

  /** \returns the determinant of a \b square Vandermonde matrix through the
   * closed form \f$ \prod_{i<j} (x_j - x_i) \f$, in O(n^2) operations. The
   * product is accumulated in the balanced form \c m * 2^e (the split
   * fraction/exponent determinant convention of LINPACK's xGEDI [3]) -- every
   * factor and the running product are renormalized to unit magnitude with the
   * power of two tracked separately, an exact rescaling [2] -- so the partial
   * products can neither overflow nor underflow when the determinant itself is
   * representable, whatever the spread of the nodes. Zero factors (repeated
   * nodes, giving an exactly singular matrix) and non-finite factors propagate
   * exactly. */
  Scalar determinant() const {
    eigen_assert(rows() == cols() && "Vandermonde::determinant requires a square matrix");
    const Index n = rows();
    Scalar det(1);
    internal::structured_exponent_type exponent = 0;
    for (Index j = 1; j < n; ++j)
      for (Index i = 0; i < j; ++i) {
        // A node difference can overflow even though the determinant is
        // representable (e.g. nodes near +-max); the guarded difference then
        // enters at half scale with the factor of two carried by the running
        // exponent.
        int shift;
        const Scalar diff = internal::structured_guarded_diff(m_x.coeff(j), m_x.coeff(i), shift);
        exponent += shift;
        det = internal::structured_balance(det * internal::structured_balance(diff, exponent), exponent);
      }
    // ldexp saturates cleanly to zero / infinity once the accumulated exponent
    // leaves the representable range; the clamp only guards the narrowing to int.
    return internal::structured_ldexp_clamped(det, exponent);
  }

  /** \internal Writes the dense representation into \a dst: column \c j is the
   * elementwise product of column \c j-1 with the nodes, so only cumulative
   * columnwise products are involved. Invoked through \c dense = vandermonde; */
  template <typename Dest>
  void evalTo(Dest& dst) const {
    dst.col(0).setOnes();
    for (Index j = 1; j < m_cols; ++j) dst.col(j) = dst.col(j - 1).cwiseProduct(m_x);
  }

  /** \internal Computes \c dst += (*this), see evalTo(). */
  template <typename Dest>
  void addTo(Dest& dst) const {
    NodeVector p = NodeVector::Ones(rows());
    dst.col(0) += p;
    for (Index j = 1; j < m_cols; ++j) {
      p = p.cwiseProduct(m_x);
      dst.col(j) += p;
    }
  }

  /** \internal Computes \c dst -= (*this), see evalTo(). */
  template <typename Dest>
  void subTo(Dest& dst) const {
    NodeVector p = NodeVector::Ones(rows());
    dst.col(0) -= p;
    for (Index j = 1; j < m_cols; ++j) {
      p = p.cwiseProduct(m_x);
      dst.col(j) -= p;
    }
  }

  /** \returns the product expression \c (*this) * \a a: the polynomial with
   * ascending coefficients \c a (per column) evaluated at every node by Horner's
   * rule, at O(mn) operations and O(1) extra storage. The expression carries the
   * default product tag, so assigning it behaves like any dense product: a
   * temporary resolves aliasing between the destination and \a a, and
   * \c .noalias() skips it. */
  template <typename Rhs>
  Product<Vandermonde, Rhs> operator*(const MatrixBase<Rhs>& a) const {
    EIGEN_STATIC_ASSERT(ColsAtCompileTime == Dynamic || Rhs::RowsAtCompileTime == Dynamic ||
                            int(ColsAtCompileTime) == int(Rhs::RowsAtCompileTime),
                        INVALID_MATRIX_PRODUCT)
    eigen_assert(a.rows() == cols() && "invalid product: dimensions do not match");
    return Product<Vandermonde, Rhs>(*this, a.derived());
  }

  /** \internal Computes \c dst += alpha * (*this) * rhs by Horner's rule.
   * \c ProductScalar is the promoted scalar of the product (complex when a real
   * operator is applied to a complex right-hand side); the accumulation runs in
   * the promoted type.
   *
   * Horner intermediates can overflow even when the polynomial value itself is
   * representable (e.g. coefficients near the overflow threshold evaluated at a
   * node of magnitude 1/2). Each (node, column) pair is therefore screened with
   * a conservative exponent bound: when no intermediate can overflow -- every
   * input of moderate magnitude -- the plain Horner loop runs, matching the naive
   * evaluation step for step, though not necessarily bit for bit: the compiler
   * may contract one loop's multiply-add into an FMA and not the other's.
   * Otherwise scaledHorner() keeps the running value in
   * the balanced form m * 2^e of determinant(). Non-finite nodes or coefficients
   * also take the plain loop, which propagates Inf/NaN entrywise like a dense
   * product; and a unit alpha must not multiply -- even the identity complex
   * scalar (1,0) pollutes an (Inf,0) value with NaN through the 0*Inf cross
   * term.
   *
   * The column's finiteness and its exponent bound come from the single
   * fast-max pass of internal::structured_exponent_bound_finite(), which is not
   * guaranteed to propagate NaN. That is sufficient here for the same reason as
   * in the FFT products: an Inf in NaN-free data always surfaces in a fast
   * maximum, and a column containing NaN yields the NaN results dense-product
   * semantics require through the plain loop and the scaled recurrence alike
   * (every Horner step folds the NaN coefficient in, and the balancing helpers
   * pass non-finite values through), so missing a NaN cannot change the
   * result. */
  template <typename Dest, typename Rhs, typename ProductScalar>
  void addProduct(Dest& dst, const Rhs& rhs, const ProductScalar& alpha) const {
    const Index m = rows(), n = m_cols;
    using Exponent = internal::structured_exponent_type;
    eigen_assert(rhs.rows() == n && "invalid product: dimensions do not match");
    const bool unitAlpha = alpha == ProductScalar(1);
    int log2n = 0;  // n < 2^log2n: bounds the number of addends of the Horner sum
    for (Index t = n; t > 0; t /= 2) ++log2n;
    for (Index k = 0; k < rhs.cols(); ++k) {
      int colExp;  // max modulus < 2^colExp; 0 for a zero or non-finite column
      const bool colFinite = internal::structured_exponent_bound_finite(rhs.col(k), colExp);
      for (Index i = 0; i < m; ++i) {
        const Scalar xi = m_x.coeff(i);
        // |p_j| < 2^(colExp+log2n+(n-1) max(xiExp,0)+1).
        const Exponent intermediateBound =
            Exponent(colExp) + Exponent(log2n) + (Exponent(n) - 1) * Exponent(numext::maxi(exponentBound(xi), 0)) + 2;
        const bool plain = !colFinite || !(numext::isfinite)(xi) ||
                           intermediateBound <= Exponent(NumTraits<RealScalar>::max_exponent());
        ProductScalar acc;
        if (plain) {
          acc = rhs.coeff(n - 1, k);
          for (Index j = n - 2; j >= 0; --j) acc = acc * xi + rhs.coeff(j, k);
        } else {
          acc = scaledHorner<ProductScalar>(xi, rhs, k);
        }
        dst.coeffRef(i, k) += unitAlpha ? acc : ProductScalar(alpha * acc);
      }
    }
  }

 private:
  /** \internal \returns an exponent bound \c e with \c |z| < 2^e (the modulus
   * for a complex \a z), or 0 for a zero or non-finite \a z; the
   * single-coefficient analogue of internal::structured_exponent_bound(). */
  template <typename T>
  static int exponentBound(const T& z) {
    return exponentBoundImpl(z, internal::bool_constant<NumTraits<T>::IsComplex>());
  }

  template <typename T>
  static int exponentBoundImpl(const T& z, std::false_type) {
    if (!(numext::abs(z) > T(0)) || !(numext::isfinite)(z)) return 0;
    int e;
    EIGEN_USING_STD(frexp);
    frexp(z, &e);
    return e;
  }

  template <typename T>
  static int exponentBoundImpl(const T& z, std::true_type) {
    using Real = typename NumTraits<T>::Real;
    const Real mag = numext::maxi(numext::abs(numext::real(z)), numext::abs(numext::imag(z)));
    if (!(mag > Real(0)) || !(numext::isfinite)(mag)) return 0;
    int e;
    EIGEN_USING_STD(frexp);
    frexp(mag, &e);
    return e + 1;  // the modulus is at most sqrt(2) times the largest component
  }

  /** \internal \returns \a z * 2^e computed through two exact half-factors, so
   * the factors themselves stay representable for the exponent swings the scaled
   * Horner recurrence produces (|e| up to about twice the scalar's exponent
   * range on the negative side, at most one exponent range on the positive
   * side); a factor past the underflow threshold flushes to zero together with
   * the then-negligible contribution it scales. */
  template <typename T>
  static T twoHalfScale(const T& z, internal::structured_exponent_type e) {
    using Real = typename NumTraits<T>::Real;
    constexpr internal::structured_exponent_type kMaxExponent = internal::structured_exponent_type(1) << 24;
    const int ec = static_cast<int>(numext::mini(numext::maxi(e, -kMaxExponent), kMaxExponent));
    const Real h1 = numext::ldexp(Real(1), ec / 2);
    const Real h2 = numext::ldexp(Real(1), ec - ec / 2);
    return (z * h1) * h2;
  }

  /** \internal Evaluates the polynomial with ascending coefficients
   * \c rhs.col(k) at the node \a xi, keeping the running value in the balanced
   * form \c acc * 2^exponent of determinant() (overflow-avoiding power-of-two
   * rescaling [1], exact by [2]): the node enters through its unit mantissa
   * with its exponent folded into the running one, the mantissa is
   * renormalized after every step, and each coefficient is folded into the
   * running frame scaled by an exact power of two split into two half-factors
   * (when the coefficient dominates the frame, the frame is rebased onto the
   * coefficient's exponent instead). Intermediates can therefore neither
   * overflow nor underflow, and the final ldexp saturates to +-Inf / +-0 exactly
   * where the true value leaves the representable range.
   *
   * An exactly zero mantissa carries no scale, so the frame is reset before
   * every fold: after an exact cancellation (or a zero node annihilating the
   * running value) a stale huge frame would otherwise underflow the next small
   * coefficient to zero. A cancellation that leaves a tiny nonzero mantissa
   * needs no such care -- the frexp renormalization rebases the frame to the
   * surviving magnitude (which is a multiple of the operands' unit roundoff,
   * hence never subnormal for real scalars, and frexp is exact on subnormal
   * component values regardless).
   * \pre the node is finite and the column passed the fast-max routing
   * predicate of addProduct(): it holds no Inf without an accompanying NaN. A
   * NaN-bearing column can reach the recurrence when the fast maximum misses
   * the NaN; every helper passes non-finite values through, so it produces the
   * NaN result dense-product semantics require. */
  template <typename ProductScalar, typename Rhs>
  ProductScalar scaledHorner(const Scalar& xi, const Rhs& rhs, Index k) const {
    using Exponent = internal::structured_exponent_type;
    Exponent xiE = 0;
    const Scalar xiMant = internal::structured_balance(xi, xiE);  // xi = xiMant * 2^xiE, exactly
    ProductScalar acc(0);
    Exponent exponent = 0;  // running value = acc * 2^exponent
    for (Index j = m_cols - 1; j >= 0; --j) {
      if (j < m_cols - 1) {
        exponent += xiE;
        acc = internal::structured_balance(acc * xiMant, exponent);
      }
      // A zero value has no scale: reset the frame so the next coefficient
      // enters at its own magnitude instead of underflowing in a stale one.
      if (acc == ProductScalar(0)) exponent = 0;
      const ProductScalar aj(rhs.coeff(j, k));
      if (aj == ProductScalar(0)) continue;
      const Exponent ajExp = exponentBound(aj);
      if (exponent < ajExp) {
        // The coefficient dominates the running frame: rebase onto the
        // coefficient's exponent. The running value rescales exactly, or
        // underflows harmlessly once it is negligible against the coefficient.
        acc = twoHalfScale(acc, exponent - ajExp);
        exponent = ajExp;
      }
      acc = internal::structured_balance(acc + twoHalfScale(aj, -exponent), exponent);
    }
    return internal::structured_ldexp_clamped(acc, exponent);
  }

  NodeVector m_x;
  Index m_cols;
};

/** \ingroup StructuredMatrices_Module
 * \returns an \c m x \a cols \ref Vandermonde operator with node vector \a nodes;
 * the compile-time row count is deduced from \a nodes. */
template <typename Derived>
Vandermonde<typename Derived::Scalar, Derived::SizeAtCompileTime, Dynamic> makeVandermonde(
    const MatrixBase<Derived>& nodes, Index cols) {
  return Vandermonde<typename Derived::Scalar, Derived::SizeAtCompileTime, Dynamic>(nodes, cols);
}

/** \ingroup StructuredMatrices_Module
 * \returns the square \ref Vandermonde operator of the nodes \a nodes. */
template <typename Derived>
Vandermonde<typename Derived::Scalar, Derived::SizeAtCompileTime, Derived::SizeAtCompileTime> makeVandermonde(
    const MatrixBase<Derived>& nodes) {
  return Vandermonde<typename Derived::Scalar, Derived::SizeAtCompileTime, Derived::SizeAtCompileTime>(nodes);
}

/** \ingroup StructuredMatrices_Module
 * \class BjorckPereyra
 * \brief Björck-Pereyra O(n^2) solver for square Vandermonde systems.
 *
 * Solves \c V*a = f -- polynomial interpolation: find the coefficients of the
 * polynomial taking values \c f at the nodes -- in O(n^2) operations and O(n)
 * storage, via divided differences in the Newton basis followed by the basis
 * change to monomials (Björck & Pereyra, 1970; Golub & Van Loan, Alg. 4.6.2).
 * The transposed (dual, or moment) system \f$ V^T w = b \f$ is solved by the
 * companion dual recurrences through the standard \c SolverBase idiom:
 * \code
 *   BjorckPereyra<double> bp(V);            // or bp.compute(V);
 *   VectorXd a = bp.solve(f);                // solve V   * a = f
 *   VectorXd w = bp.transpose().solve(b);    // solve V^T * w = b
 *   VectorXd u = bp.adjoint().solve(b);      // solve V^H * u = b
 * \endcode
 *
 * There is no factorization: \c compute() stores the nodes (and flags exactly
 * repeated or non-finite nodes through \c info()), and each solve runs the
 * O(n^2) recurrences directly. Genuinely complex node sets are put in a
 * deterministic Leja order to control growth in the Newton representation;
 * real nodes, including complex scalars with zero imaginary parts, retain their
 * input order and its useful monotonicity properties.
 *
 * Despite the exponential conditioning of real-node Vandermonde matrices, the
 * computed solution is often far more accurate than the conditioning suggests:
 * for monotonically ordered nodes and a right-hand side with alternating signs
 * the forward error is governed by a small relative-perturbation bound
 * independent of the condition number (Higham, ASNA ch. 22).
 *
 * \tparam Scalar_ a floating-point-like real or complex scalar supporting
 * Eigen's scalar math hooks, including \c isfinite (and \c abs and \c log for
 * complex node ordering). Integer types are rejected.
 *
 * \sa class Vandermonde
 */
template <typename Scalar_>
class BjorckPereyra : public SolverBase<BjorckPereyra<Scalar_>> {
 public:
  using Base = SolverBase<BjorckPereyra>;
  friend class SolverBase<BjorckPereyra>;
  EIGEN_GENERIC_PUBLIC_INTERFACE(BjorckPereyra)
  EIGEN_STATIC_ASSERT_NON_INTEGER(RealScalar)
  using NodeVector = Matrix<Scalar, Dynamic, 1>;

  /** Default constructor; call \ref compute before \ref solve. */
  BjorckPereyra() : m_isInitialized(false), m_info(InvalidInput) {}

  /** Constructs the solver for the square Vandermonde matrix \a V. */
  template <int Rows_, int Cols_>
  explicit BjorckPereyra(const Vandermonde<Scalar, Rows_, Cols_>& V) : m_isInitialized(false), m_info(InvalidInput) {
    compute(V);
  }

  /** Stores the nodes of the square Vandermonde matrix \a V, checks that they
   * are finite and distinct, and computes a Leja order for genuinely complex
   * nodes. */
  template <int Rows_, int Cols_>
  BjorckPereyra& compute(const Vandermonde<Scalar, Rows_, Cols_>& V) {
    EIGEN_STATIC_ASSERT(Rows_ == Dynamic || Cols_ == Dynamic || Rows_ == Cols_, YOU_MIXED_MATRICES_OF_DIFFERENT_SIZES)
    eigen_assert(V.rows() == V.cols() && "BjorckPereyra requires a square Vandermonde matrix");
    m_x = V.nodes();
    m_order.clear();
    m_info = Success;
    const Index n = m_x.size();
    if (!m_x.allFinite()) m_info = InvalidInput;
    for (Index j = 1; j < n && m_info == Success; ++j)
      for (Index i = 0; i < j; ++i) {
        if (m_x[i] == m_x[j]) {
          m_info = NumericalIssue;
          break;
        }
      }
    if (m_info == Success) initializeNodeOrder(m_x, internal::bool_constant<NumTraits<Scalar>::IsComplex>());
    m_isInitialized = true;
    return *this;
  }

  Index rows() const noexcept { return m_x.size(); }
  Index cols() const noexcept { return m_x.size(); }

  /** \returns \c Success, \c NumericalIssue when the nodes contain an exact
   * duplicate (the matrix is singular), or \c InvalidInput for a non-finite
   * node. */
  ComputationInfo info() const {
    eigen_assert(m_isInitialized && "BjorckPereyra is not initialized.");
    return m_info;
  }

#ifdef EIGEN_PARSED_BY_DOXYGEN
  /** \returns the solution \c a of \c V*a = \a f, as a lazily evaluated
   * expression. Supports multiple right-hand sides. The transposed and adjoint
   * systems are solved through \c transpose().solve(b) and \c adjoint().solve(b).
   * \pre \ref compute has been called. */
  template <typename Rhs>
  inline const Solve<BjorckPereyra, Rhs> solve(const MatrixBase<Rhs>& f) const;
#endif

#ifndef EIGEN_PARSED_BY_DOXYGEN
  /** \internal Primal solve V*a = rhs: divided differences (Newton coefficients),
   * then the Newton-to-monomial basis change. */
  template <typename RhsType, typename DstType>
  void _solve_impl(const RhsType& rhs, DstType& dst) const {
    using RhsScalar = typename RhsType::Scalar;
    using WorkScalar = typename DstType::Scalar;
    using ProductOp = internal::scalar_product_op<Scalar, RhsScalar>;
    EIGEN_CHECK_BINARY_COMPATIBILITY(ProductOp, Scalar, RhsScalar)

    const Index n = m_x.size();
    dst = rhs;
    Matrix<WorkScalar, Dynamic, 1> permuted;
    if (!m_order.empty()) permuted.resize(n);
    for (Index k = 0; k < rhs.cols(); ++k) {
      auto a = dst.col(k);
      if (!m_order.empty()) {
        for (Index i = 0; i < n; ++i) permuted[i] = a[m_order[static_cast<std::size_t>(i)]];
        a = permuted;
      }
      for (Index j = 0; j < n - 1; ++j)
        for (Index i = n - 1; i > j; --i) a[i] = (a[i] - a[i - 1]) / (m_x[i] - m_x[i - j - 1]);
      for (Index j = n - 2; j >= 0; --j)
        for (Index i = j; i < n - 1; ++i) a[i] -= m_x[j] * a[i + 1];
    }
  }

  /** \internal Transposed (dual) solve V^T*w = rhs: the transposes of the primal
   * elementary steps, applied in reverse order; conjugated on the way in and out
   * for the adjoint. */
  template <bool Conjugate, typename RhsType, typename DstType>
  void _solve_impl_transposed(const RhsType& rhs, DstType& dst) const {
    using RhsScalar = typename RhsType::Scalar;
    using WorkScalar = typename DstType::Scalar;
    using ProductOp = internal::scalar_product_op<Scalar, RhsScalar>;
    EIGEN_CHECK_BINARY_COMPATIBILITY(ProductOp, Scalar, RhsScalar)

    const Index n = m_x.size();
    dst = rhs.template conjugateIf<Conjugate>();
    Matrix<WorkScalar, Dynamic, 1> permuted;
    if (!m_order.empty()) permuted.resize(n);
    for (Index k = 0; k < rhs.cols(); ++k) {
      auto w = dst.col(k);
      for (Index j = 0; j < n - 1; ++j)
        for (Index i = n - 1; i > j; --i) w[i] -= m_x[j] * w[i - 1];
      for (Index j = n - 2; j >= 0; --j) {
        w.tail(n - j - 1).array() /= (m_x.tail(n - j - 1) - m_x.head(n - j - 1)).array();
        for (Index i = j; i < n - 1; ++i) w[i] -= w[i + 1];
      }

      if (!m_order.empty()) {
        permuted = w;
        for (Index i = 0; i < n; ++i) w[m_order[static_cast<std::size_t>(i)]] = permuted[i];
      }
    }
    if (Conjugate) dst = dst.conjugate().eval();
  }
#endif

 private:
  static RealScalar lejaLogAbs(const Scalar& z) {
    const RealScalar re = numext::abs(numext::real(z));
    const RealScalar im = numext::abs(numext::imag(z));
    const RealScalar scale = numext::maxi(re, im);
    if (scale == RealScalar(0)) return -NumTraits<RealScalar>::infinity();
    const RealScalar scaledRe = re / scale;
    const RealScalar scaledIm = im / scale;
    return numext::log(scale) + RealScalar(0.5) * numext::log(scaledRe * scaledRe + scaledIm * scaledIm);
  }

  static RealScalar lejaLogDistance(const Scalar& a, const Scalar& b) {
    int exponent;
    const Scalar difference = internal::structured_guarded_diff(a, b, exponent);
    return lejaLogAbs(difference) + RealScalar(exponent) * numext::log(RealScalar(2));
  }

  void initializeNodeOrder(const NodeVector&, std::false_type) {}

  void initializeNodeOrder(const NodeVector& nodes, std::true_type) {
    using Real = typename NumTraits<Scalar>::Real;
    const Index n = nodes.size();
    bool genuinelyComplex = false;
    for (Index i = 0; i < n; ++i) genuinelyComplex = genuinelyComplex || numext::imag(nodes[i]) != Real(0);
    if (!genuinelyComplex || n < 2) return;

    const NodeVector original = nodes;

    m_order.resize(static_cast<std::size_t>(n));
    std::vector<char> selected(static_cast<std::size_t>(n), 0);
    std::vector<RealScalar> scores(static_cast<std::size_t>(n), RealScalar(0));
    Index next;  // the Leja sequence starts at a node of largest modulus
    original.unaryExpr(&lejaLogAbs).maxCoeff(&next);

    for (Index position = 0; position < n; ++position) {
      m_order[static_cast<std::size_t>(position)] = next;
      selected[static_cast<std::size_t>(next)] = 1;
      if (position + 1 == n) break;

      Index candidate = -1;
      RealScalar candidateScore = -NumTraits<RealScalar>::infinity();
      for (Index i = 0; i < n; ++i) {
        if (selected[static_cast<std::size_t>(i)]) continue;
        scores[static_cast<std::size_t>(i)] += lejaLogDistance(original[i], original[next]);
        if (candidate < 0 || scores[static_cast<std::size_t>(i)] > candidateScore) {
          candidate = i;
          candidateScore = scores[static_cast<std::size_t>(i)];
        }
      }
      next = candidate;
    }

    for (Index i = 0; i < n; ++i) m_x[i] = original[m_order[static_cast<std::size_t>(i)]];
  }

  NodeVector m_x;
  std::vector<Index> m_order;
  bool m_isInitialized;
  ComputationInfo m_info;
};

namespace internal {

/** \internal Solve results use the scalar promoted from the solver and RHS.
 * Core's generic solve traits retain the RHS scalar, which would discard the
 * imaginary part when a complex Vandermonde is applied to a real RHS. */
template <typename SolverScalar, typename RhsScalar,
          bool Compatible = has_ReturnType<ScalarBinaryOpTraits<SolverScalar, RhsScalar>>::value>
struct bjorck_pereyra_result_scalar {
  using type = SolverScalar;
};

template <typename SolverScalar, typename RhsScalar>
struct bjorck_pereyra_result_scalar<SolverScalar, RhsScalar, true> {
  using type = typename ScalarBinaryOpTraits<SolverScalar, RhsScalar>::ReturnType;
};

template <typename SolverScalar, typename RhsType>
struct bjorck_pereyra_solve_traits {
  using ResultScalar = typename bjorck_pereyra_result_scalar<SolverScalar, typename RhsType::Scalar>::type;
  using PlainObject =
      typename make_proper_matrix_type<ResultScalar, Dynamic, RhsType::ColsAtCompileTime, RhsType::PlainObject::Options,
                                       Dynamic, RhsType::MaxColsAtCompileTime>::type;
};

template <typename Scalar_, typename RhsType>
struct solve_traits<BjorckPereyra<Scalar_>, RhsType, Dense> : bjorck_pereyra_solve_traits<Scalar_, RhsType> {};

template <typename Scalar_, typename RhsType>
struct solve_traits<Transpose<const BjorckPereyra<Scalar_>>, RhsType, Dense>
    : bjorck_pereyra_solve_traits<Scalar_, RhsType> {};

template <typename Scalar_, typename RhsType>
struct solve_traits<CwiseUnaryOp<scalar_conjugate_op<Scalar_>, const Transpose<const BjorckPereyra<Scalar_>>>, RhsType,
                    Dense> : bjorck_pereyra_solve_traits<Scalar_, RhsType> {};

template <typename Factor, typename Scalar_, int Rows_, int Cols_, typename Plain, typename Rhs>
struct scaled_vandermonde_product_impl
    : generic_product_impl_base<
          CwiseBinaryOp<scalar_product_op<Factor, Scalar_>, const CwiseNullaryOp<scalar_constant_op<Factor>, Plain>,
                        const Vandermonde<Scalar_, Rows_, Cols_>>,
          Rhs, scaled_vandermonde_product_impl<Factor, Scalar_, Rows_, Cols_, Plain, Rhs>> {
  using Op = Vandermonde<Scalar_, Rows_, Cols_>;
  using ScaledOp = CwiseBinaryOp<scalar_product_op<Factor, Scalar_>,
                                 const CwiseNullaryOp<scalar_constant_op<Factor>, Plain>, const Op>;
  using Scalar = typename Product<ScaledOp, Rhs>::Scalar;

  template <typename Dest>
  static void scaleAndAddTo(Dest& dst, const ScaledOp& lhs, const Rhs& rhs, const Scalar& alpha) {
    using RhsNested = typename nested_eval<Rhs, Rows_>::type;
    RhsNested actualRhs(rhs);
    lhs.rhs().addProduct(dst, actualRhs, Scalar(alpha * lhs.lhs().functor().m_other));
  }
};

// Preserve the Horner kernel after Core introduces the scaled wrapper above;
// otherwise the wrapper has DenseShape and falls back to a coefficient product.
#define EIGEN_SCALED_VANDERMONDE_PRODUCT_IMPL(ProductTag)                                                        \
  template <typename Factor, typename Scalar_, int Rows_, int Cols_, typename Plain, typename Rhs>               \
  struct generic_product_impl<                                                                                   \
      CwiseBinaryOp<scalar_product_op<Factor, Scalar_>, const CwiseNullaryOp<scalar_constant_op<Factor>, Plain>, \
                    const Vandermonde<Scalar_, Rows_, Cols_>>,                                                   \
      Rhs, DenseShape, DenseShape, ProductTag>                                                                   \
      : scaled_vandermonde_product_impl<Factor, Scalar_, Rows_, Cols_, Plain, Rhs> {};

EIGEN_SCALED_VANDERMONDE_PRODUCT_IMPL(CoeffBasedProductMode)
EIGEN_SCALED_VANDERMONDE_PRODUCT_IMPL(LazyCoeffBasedProductMode)
EIGEN_SCALED_VANDERMONDE_PRODUCT_IMPL(OuterProduct)
EIGEN_SCALED_VANDERMONDE_PRODUCT_IMPL(InnerProduct)
EIGEN_SCALED_VANDERMONDE_PRODUCT_IMPL(GemvProduct)
EIGEN_SCALED_VANDERMONDE_PRODUCT_IMPL(GemmProduct)

#undef EIGEN_SCALED_VANDERMONDE_PRODUCT_IMPL

template <typename Scalar_, int Rows_, int Cols_, typename Rhs, int ProductTag>
struct generic_product_impl<Vandermonde<Scalar_, Rows_, Cols_>, Rhs, StructuredShape, DenseShape, ProductTag>
    : structured_product_impl<Vandermonde<Scalar_, Rows_, Cols_>, Rhs> {};

}  // namespace internal

}  // namespace Eigen

#endif  // EIGEN_STRUCTURED_VANDERMONDE_H
