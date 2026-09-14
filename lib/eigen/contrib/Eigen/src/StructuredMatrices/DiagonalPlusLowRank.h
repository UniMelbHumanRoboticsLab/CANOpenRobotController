// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

// References:
//  [1] M. A. Woodbury, "Inverting Modified Matrices", Memorandum Report 42,
//      Statistical Research Group, Princeton University, 1950. The Woodbury
//      identity behind solve() and inverse().
//  [2] W. W. Hager, "Updating the Inverse of a Matrix", SIAM Review, 31(2),
//      pp. 221-239, 1989. Review of the Woodbury identity and of the matrix
//      determinant lemma used by determinant().
//  [3] J. J. Dongarra, J. R. Bunch, C. B. Moler and G. W. Stewart, "LINPACK
//      Users' Guide", SIAM, 1979. determinant()'s balanced accumulation follows
//      the convention of its xGEDI routines, which return determinants as a
//      (fraction, exponent) pair to avoid spurious overflow/underflow.
//  [4] P. H. Sterbenz, "Floating-Point Computation", Prentice-Hall, 1974.
//      Scaling by a power of two is exact, the property the balanced
//      accumulation and the normalized Woodbury products rely on.
//  [5] N. J. Higham, "Accuracy and Stability of Numerical Algorithms", 2nd ed.,
//      SIAM, 2002, chapter 27. Overflow-avoiding rescaling of intermediate
//      quantities, the technique behind the normalized capacitance products.
//  [6] E. L. Yip, "A Note on the Stability of Solving a Rank-p Modification of
//      a Linear System by the Sherman-Morrison-Woodbury Formula", SIAM Journal
//      on Scientific and Statistical Computing, 7(3), pp. 507-513, 1986. The
//      accuracy limitation of the Woodbury solve noted on solve().

#ifndef EIGEN_STRUCTURED_DIAGONAL_PLUS_LOW_RANK_H
#define EIGEN_STRUCTURED_DIAGONAL_PLUS_LOW_RANK_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

template <typename Scalar_, int Size_ = Dynamic, int Rank_ = Dynamic>
class DiagonalPlusLowRank;

namespace internal {

template <typename Scalar_, int Size_, int Rank_>
struct traits<DiagonalPlusLowRank<Scalar_, Size_, Rank_>> {
  using Scalar = Scalar_;
  using StorageKind = Dense;
  using XprKind = MatrixXpr;
  using StorageIndex = int;
  static constexpr int RowsAtCompileTime = Size_;
  static constexpr int ColsAtCompileTime = Size_;
  static constexpr int MaxRowsAtCompileTime = Size_;
  static constexpr int MaxColsAtCompileTime = Size_;
  // Deliberately no NestByRefBit: transpose(), conjugate(), adjoint(), inverse()
  // and makeDiagonalPlusLowRank() return owning temporaries, so Product must nest
  // the operator by value for a delayed-evaluated product expression to keep its
  // left factor alive. The copy is O(nk), on par with a single product evaluation.
  static constexpr unsigned int Flags = 0;
};

template <typename Scalar_, int Size_, int Rank_>
struct evaluator_traits<DiagonalPlusLowRank<Scalar_, Size_, Rank_>> {
  using Kind = IndexBased;
  using Shape = StructuredShape;
};

// Entrywise multiplication by the exact power of two 2^e [4], for the factor normalization of the
// Woodbury products [5]. Uses structured_balance_impl::apply_exponent (an ldexp per component) rather
// than a multiplication by 2^e or by two half powers 2^(e/2): ldexp is exact and saturates entrywise
// for every exponent, whereas 2^-eu itself overflows for a subnormal factor bound and a saturated
// half power turns zero entries into NaN through 0 * Inf.
template <typename Scalar>
struct dplr_ldexp_op {
  int e;
  Scalar operator()(const Scalar& x) const { return structured_balance_impl<Scalar>::apply_exponent(x, e); }
};

// The smallest e with n <= 2^e: the inner-dimension term of a product's entry
// magnitude bound (a length-n dot product of entries below 2^a and 2^b stays
// below 2^(a + b + e)).
inline int dplr_index_exponent(Index n) { return log2_ceil(static_cast<std::make_unsigned_t<Index>>(n)); }

// True when a product of operands with entry-magnitude exponent bounds ea and eb over an inner
// dimension of length \a inner provably cannot overflow, with one bit of headroom for a trailing
// addition. Where it fails the Woodbury kernels rescale their factors exactly [4][5]; where it holds
// they keep the plain association.
template <typename RealScalar>
bool dplr_product_fits(int ea, int eb, Index inner) {
  return ea + eb + dplr_index_exponent(inner) + 1 < NumTraits<RealScalar>::max_exponent();
}

// Compile-time dispatch keeps fixed rank zero from instantiating a 0 x 0 LU;
// C++14 cannot express this branch with if constexpr.
template <int Rank_>
struct dplr_capacitance_impl {
  /** Applies the Woodbury correction
   * \f[ x\mathrel{-}=D^{-1}U(I_k+V^H D^{-1}U)^{-1}V^Hx. \f]
   * Exact power-of-two scaling is used only when either structural product can
   * overflow [4][5], preserving the unscaled result bit-for-bit otherwise. */
  template <typename Op, typename Dinv, typename Workspace>
  static void subtractSolveCorrection(const Op& op, const Dinv& dinv, Workspace& x) {
    using Scalar = typename Op::Scalar;
    using RealScalar = typename Op::RealScalar;
    if (op.correctionRank() == 0 || x.size() == 0) return;
    PartialPivLU<typename Op::CapacitanceType> cap(op.capacitance());
    const int eu = structured_exponent_bound(op.factorU());
    const int ev = structured_exponent_bound(op.factorV());
    const int ed = structured_exponent_bound(dinv);
    const int ex = structured_exponent_bound(x);
    if (dplr_product_fits<RealScalar>(ev, ex, op.rows()) && dplr_product_fits<RealScalar>(ed, eu, 1)) {
      x.noalias() -= dinv.asDiagonal() * (op.factorU() * cap.solve(op.factorV().adjoint() * x));
      return;
    }
    Matrix<Scalar, Rank_, Workspace::ColsAtCompileTime> y =
        cap.solve(op.factorV().unaryExpr(dplr_ldexp_op<Scalar>{-ev}).adjoint() * x);
    y = y.unaryExpr(dplr_ldexp_op<Scalar>{eu + ev});
    x.noalias() -= dinv.asDiagonal() * (op.factorU().unaryExpr(dplr_ldexp_op<Scalar>{-eu}) * y);
  }
  /** Forms \f$U'=-D^{-1}U(I_k+V^HD^{-1}U)^{-1}\f$. Scaling prevents a
   * spurious overflow in \f$D^{-1}U\f$ when the capacitance inverse subsequently
   * shrinks it [4][5]. */
  template <typename Op, typename Dinv, typename Factor>
  static void inverseFactor(const Op& op, const Dinv& dinv, Factor& Up) {
    using Scalar = typename Op::Scalar;
    using RealScalar = typename Op::RealScalar;
    if (op.correctionRank() == 0) return;
    PartialPivLU<typename Op::CapacitanceType> cap(op.capacitance());
    const typename Op::CapacitanceType K = cap.inverse();
    const int eu = structured_exponent_bound(op.factorU());
    const int ed = structured_exponent_bound(dinv);
    const int ek = structured_exponent_bound(K);
    if (dplr_product_fits<RealScalar>(ed, eu, 1) && dplr_product_fits<RealScalar>(ed + eu, ek, op.correctionRank())) {
      Up.noalias() = -(dinv.asDiagonal() * op.factorU() * K);
      return;
    }
    Up.noalias() = -(dinv.asDiagonal() * op.factorU().unaryExpr(dplr_ldexp_op<Scalar>{-eu}) * K);
    Up = Up.unaryExpr(dplr_ldexp_op<Scalar>{eu});
  }
  /** Accumulates the determinant-lemma factor
   * \f$\det(I_k+V^HD^{-1}U)\f$ as a mantissa and power of two. Each LU pivot is
   * renormalized before multiplication because \f$\det(D)\f$ and the capacitance
   * determinant can occupy opposite ends of the exponent range. */
  template <typename Op>
  static typename Op::Scalar balancedCapacitanceDeterminant(const Op& op, Index& exponent) {
    using Scalar = typename Op::Scalar;
    if (op.correctionRank() == 0) return Scalar(1);
    const PartialPivLU<typename Op::CapacitanceType> lu(op.capacitance());
    Scalar det(static_cast<typename Op::RealScalar>(lu.permutationP().determinant()));
    for (Index i = 0; i < lu.matrixLU().rows(); ++i)
      det = structured_balance(det * structured_balance(lu.matrixLU().coeff(i, i), exponent), exponent);
    return det;
  }
};

template <>
struct dplr_capacitance_impl<0> {
  template <typename Op, typename Dinv, typename Workspace>
  static void subtractSolveCorrection(const Op&, const Dinv&, Workspace&) {}
  template <typename Op, typename Dinv, typename Factor>
  static void inverseFactor(const Op&, const Dinv&, Factor&) {}
  template <typename Op>
  static typename Op::Scalar balancedCapacitanceDeterminant(const Op&, Index&) {
    return typename Op::Scalar(1);
  }
};

}  // namespace internal

/** \ingroup StructuredMatrices_Module
 * \class DiagonalPlusLowRank
 * \brief An \c n x \c n operator \f$ D + U V^H \f$: a diagonal matrix plus a
 * rank-k correction, stored as its diagonal and the two \c n x \c k factors.
 *
 * Products cost O(nk) instead of O(n^2). Linear systems are solved in O(nk^2)
 * through the Woodbury identity [1]
 * \f$ (D + UV^H)^{-1} = D^{-1} - D^{-1} U (I_k + V^H D^{-1} U)^{-1} V^H D^{-1} \f$,
 * factoring only the \c k x \c k \em capacitance matrix (\ref solve). The
 * determinant follows from the matrix determinant lemma [2]
 * \f$ \det(D)\,\det(I_k + V^H D^{-1} U) \f$ (\ref determinant), and the class is
 * closed under \ref inverse, \ref transpose, \ref conjugate and \ref adjoint.
 *
 * The rank-k correction may have \c k = 0 (a plain diagonal operator). \c k is
 * not required to be small, but every advantage over a dense matrix vanishes as
 * \c k approaches \c n.
 *
 * The operator stores its own copies of the diagonal and the factors and derives
 * from \c EigenBase. Because \c operator* returns an Eigen product expression, a
 * \c DiagonalPlusLowRank also drops into the matrix-free iterative solvers, and
 * it can be assigned to a dense matrix when an explicit representation is
 * needed. As with any matrix-free operator, the iterative solvers must be
 * instantiated with \c IdentityPreconditioner (e.g.
 * \c GMRES<DiagonalPlusLowRank<double>,IdentityPreconditioner>): the default
 * preconditioners read individual coefficients through \c col() or
 * \c InnerIterator, which the structured operators do not expose.
 *
 * \tparam Scalar_ the scalar type, real or complex.
 * \tparam Size_ the dimension at compile time, or \c Dynamic (the default).
 * \tparam Rank_ the correction rank at compile time, or \c Dynamic (the default).
 *
 * \sa makeDiagonalPlusLowRank()
 */
template <typename Scalar_, int Size_, int Rank_>
class DiagonalPlusLowRank : public EigenBase<DiagonalPlusLowRank<Scalar_, Size_, Rank_>> {
 public:
  using Scalar = Scalar_;
  using RealScalar = typename NumTraits<Scalar>::Real;
  using StorageIndex = int;
  using DiagonalVector = Matrix<Scalar, Size_, 1>;
  using FactorType = Matrix<Scalar, Size_, Rank_>;
  using CapacitanceType = Matrix<Scalar, Rank_, Rank_>;

  static constexpr int RowsAtCompileTime = Size_;
  static constexpr int ColsAtCompileTime = Size_;
  static constexpr int MaxRowsAtCompileTime = Size_;
  static constexpr int MaxColsAtCompileTime = Size_;
  static constexpr int SizeAtCompileTime = internal::size_at_compile_time(Size_, Size_);
  static constexpr int MaxSizeAtCompileTime = SizeAtCompileTime;
  static constexpr bool IsRowMajor = false;
  // Deliberately no IsVectorAtCompileTime: Ref<const DiagonalPlusLowRank>'s default
  // StrideType argument reads it, so its absence makes internal::is_ref_compatible
  // SFINAE to false and keeps the iterative solvers on their matrix-free path.

  /** Builds the operator \c diag(d) + \c U*V.adjoint(). The factors must have the
   * same number of columns (the correction rank \c k, possibly zero) and as many
   * rows as \a d has entries. */
  template <typename DDerived, typename UDerived, typename VDerived>
  DiagonalPlusLowRank(const MatrixBase<DDerived>& d, const MatrixBase<UDerived>& U, const MatrixBase<VDerived>& V)
      : m_d(d), m_U(U), m_V(V) {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(DDerived)
    eigen_assert(m_d.size() > 0 && "DiagonalPlusLowRank must be non-empty");
    eigen_assert(m_U.rows() == m_d.size() && m_V.rows() == m_d.size() && m_U.cols() == m_V.cols() &&
                 "factor dimensions do not match");
  }

  EIGEN_DEVICE_FUNC Index rows() const { return m_d.size(); }
  EIGEN_DEVICE_FUNC Index cols() const { return m_d.size(); }
  /** \returns the correction rank \c k. */
  Index correctionRank() const { return m_U.cols(); }

  /** \returns the diagonal vector \c d. */
  const DiagonalVector& diagonal() const { return m_d; }
  /** \returns the left factor \c U. */
  const FactorType& factorU() const { return m_U; }
  /** \returns the right factor \c V. */
  const FactorType& factorV() const { return m_V; }

  /** \returns the coefficient at row \a row and column \a col. */
  Scalar coeff(Index row, Index col) const {
    Scalar r = m_V.row(col).dot(m_U.row(row));  // dot() conjugates its first argument
    if (row == col) r += m_d.coeff(row);
    return r;
  }

  /** \returns the transpose of \c *this, itself a \c DiagonalPlusLowRank
   * operator: \f$ (D + UV^H)^T = D + \bar V \bar U^H \f$. */
  DiagonalPlusLowRank transpose() const { return DiagonalPlusLowRank(m_d, m_V.conjugate(), m_U.conjugate()); }

  /** \returns the complex conjugate of \c *this, itself a \c DiagonalPlusLowRank
   * operator. */
  DiagonalPlusLowRank conjugate() const {
    return DiagonalPlusLowRank(m_d.conjugate(), m_U.conjugate(), m_V.conjugate());
  }

  /** \returns the adjoint of \c *this, itself a \c DiagonalPlusLowRank operator:
   * \f$ (D + UV^H)^H = \bar D + V U^H \f$. */
  DiagonalPlusLowRank adjoint() const { return DiagonalPlusLowRank(m_d.conjugate(), m_V, m_U); }

  /** \returns the capacitance matrix \f$ I_k + V^H D^{-1} U \f$ of the Woodbury
   * identity. Every consumer of the triple product -- \ref solve, \ref inverse
   * and \ref determinant -- forms it through this one method.
   *
   * With extreme factor magnitudes the plain association can overflow even
   * though the capacitance itself is representable: for \c d = 1e-200,
   * \c U = 1e-200, \c V = 1e200 the operator is essentially the identity and
   * the capacitance \c 1 + 1e200 is representable, but \f$ V^H D^{-1} \f$ is
   * \c 1e400. The factors are therefore rescaled by exact powers of two [4]
   * when a conservative exponent bound detects the danger, the product is
   * formed as \f$ \hat V^H (D^{-1} \hat U) \f$ -- whose intermediates are
   * bounded by roughly \c n * max|1/d| -- and the removed exponent is folded
   * back entrywise before the identity is added [5]. When the plain
   * association provably cannot overflow it is kept, so results for moderate
   * data are bit-identical to the unnormalized evaluation.
   * \warning The diagonal must have no zero entries. */
  CapacitanceType capacitance() const {
    const Index k = correctionRank();
    CapacitanceType c = CapacitanceType::Identity(k, k);
    if (k == 0) return c;
    const DiagonalVector dinv = m_d.cwiseInverse();
    const int eu = internal::structured_exponent_bound(m_U);
    const int ev = internal::structured_exponent_bound(m_V);
    const int ed = internal::structured_exponent_bound(dinv);
    if (internal::dplr_product_fits<RealScalar>(ev, ed, 1) &&
        internal::dplr_product_fits<RealScalar>(ev + ed, eu, rows())) {
      c.noalias() += m_V.adjoint() * dinv.asDiagonal() * m_U;
      return c;
    }
    const FactorType W = dinv.asDiagonal() * m_U.unaryExpr(internal::dplr_ldexp_op<Scalar>{-eu});
    CapacitanceType t(k, k);
    t.noalias() = m_V.unaryExpr(internal::dplr_ldexp_op<Scalar>{-ev}).adjoint() * W;
    t = t.unaryExpr(internal::dplr_ldexp_op<Scalar>{eu + ev});
    c += t;
    return c;
  }

  /** \returns the solution of \c (*this) * x = b through the Woodbury identity
   * [1], factoring only the \c k x \c k capacitance matrix: O(nk^2 + k^3) setup
   * and O(nk) per right-hand side. Supports multiple right-hand sides.
   * \warning Requires an invertible diagonal \em and an invertible capacitance
   * matrix (equivalently, an invertible operator); this is not checked beyond
   * the NaN/Inf propagation of the arithmetic itself.
   * \warning The Woodbury splitting routes the solution through \f$ D^{-1} b \f$:
   * when \c max|1/d| greatly exceeds the norm of the operator's inverse, the
   * correction cancels most of those amplified digits and the achievable
   * accuracy degrades accordingly, even for a well-conditioned operator [6].
   * The normalized kernels keep such solves finite; they cannot restore the
   * cancelled digits. */
  template <typename Rhs>
  Matrix<Scalar, Size_, Rhs::ColsAtCompileTime> solve(const MatrixBase<Rhs>& b) const {
    EIGEN_STATIC_ASSERT(RowsAtCompileTime == Dynamic || Rhs::RowsAtCompileTime == Dynamic ||
                            int(RowsAtCompileTime) == int(Rhs::RowsAtCompileTime),
                        YOU_MIXED_MATRICES_OF_DIFFERENT_SIZES)
    eigen_assert(b.rows() == rows() && "right-hand side has the wrong number of rows");
    const DiagonalVector dinv = m_d.cwiseInverse();
    // D^{-1} b can overflow while the solution is representable: at d = 1e-200, b = 1e200 the operator
    // is essentially the identity and x is 1e200, yet this term is 1e400. The solve is linear in b, so
    // b is rescaled by an exact power of two [4] on the same exponent test the other kernels use and
    // the exponent is folded back into the result [5]; the plain form is kept where it provably
    // cannot overflow.
    const int eb = internal::structured_exponent_bound(b.derived());
    const int ed = internal::structured_exponent_bound(dinv);
    const bool rescale = !internal::dplr_product_fits<RealScalar>(ed, eb, 1);

    Matrix<Scalar, Size_, Rhs::ColsAtCompileTime> x =
        rescale ? (dinv.asDiagonal() * b.derived().unaryExpr(internal::dplr_ldexp_op<Scalar>{-eb})).eval()
                : (dinv.asDiagonal() * b).eval();
    internal::dplr_capacitance_impl<Rank_>::subtractSolveCorrection(*this, dinv, x);
    if (rescale) x = x.unaryExpr(internal::dplr_ldexp_op<Scalar>{eb});
    return x;
  }

  /** \returns the inverse of \c *this, itself a \c DiagonalPlusLowRank operator:
   * by the Woodbury identity [1], \f$ (D + UV^H)^{-1} = D^{-1} + U' V'^H \f$ with
   * \f$ U' = -D^{-1} U (I_k + V^H D^{-1} U)^{-1} \f$ and \f$ V' = D^{-H} V \f$.
   * \warning Same invertibility requirements as \ref solve. */
  DiagonalPlusLowRank inverse() const {
    const DiagonalVector dinv = m_d.cwiseInverse();
    FactorType Up(rows(), correctionRank());
    internal::dplr_capacitance_impl<Rank_>::inverseFactor(*this, dinv, Up);
    FactorType Vp = dinv.conjugate().asDiagonal() * m_V;
    return DiagonalPlusLowRank(dinv, Up, Vp);
  }

  /** \returns the determinant through the matrix determinant lemma [2]:
   * \f$ \det(D)\,\det(I_k + V^H D^{-1} U) \f$, in O(nk^2 + k^3) operations.
   * Both factors are accumulated in the balanced form \c m * 2^e (the split
   * fraction/exponent determinant convention of LINPACK's xGEDI [3]) -- the
   * diagonal entries and the LU pivots of the capacitance matrix are
   * renormalized to unit magnitude one at a time, with the powers of two tracked
   * in a shared exponent
   * -- so no partial product, in particular neither ordinary determinant on its
   * own, can overflow or underflow when the combined determinant is
   * representable, whatever the ordering and magnitudes of the entries.
   * Genuinely out-of-range determinants still saturate to (signed) zero or
   * infinity.
   * \warning The diagonal must have no zero entries (use the lemma symmetrically
   * or a dense fallback for that case), and, as with \ref solve and \ref inverse,
   * its reciprocals must be finite: a subnormal diagonal entry overflows
   * \f$ D^{-1} \f$ -- and with it the capacitance entries -- to infinity. */
  Scalar determinant() const {
    Scalar det(1);
    Index exponent = 0;
    for (Index i = 0; i < rows(); ++i)
      det = internal::structured_balance(det * internal::structured_balance(m_d.coeff(i), exponent), exponent);
    const Scalar capDet = internal::dplr_capacitance_impl<Rank_>::balancedCapacitanceDeterminant(*this, exponent);
    det = internal::structured_balance(det * capDet, exponent);
    return internal::structured_ldexp_clamped(det, exponent);
  }

  /** \internal Writes the dense representation into \a dst. Invoked through
   * \c dense = op; */
  template <typename Dest>
  void evalTo(Dest& dst) const {
    dst.noalias() = m_U * m_V.adjoint();
    dst.diagonal() += m_d;
  }

  /** \internal Computes \c dst += (*this), see evalTo(). */
  template <typename Dest>
  void addTo(Dest& dst) const {
    dst.noalias() += m_U * m_V.adjoint();
    dst.diagonal() += m_d;
  }

  /** \internal Computes \c dst -= (*this), see evalTo(). */
  template <typename Dest>
  void subTo(Dest& dst) const {
    dst.noalias() -= m_U * m_V.adjoint();
    dst.diagonal() -= m_d;
  }

  /** \returns the product expression \c (*this) * \a v, evaluated at O(nk)
   * operations without forming the matrix. The expression carries the default
   * product tag, so assigning it behaves like any dense product: a temporary
   * resolves aliasing between the destination and \a v, and \c .noalias() skips
   * it. */
  template <typename Rhs>
  Product<DiagonalPlusLowRank, Rhs> operator*(const MatrixBase<Rhs>& v) const {
    EIGEN_STATIC_ASSERT(ColsAtCompileTime == Dynamic || Rhs::RowsAtCompileTime == Dynamic ||
                            int(ColsAtCompileTime) == int(Rhs::RowsAtCompileTime),
                        INVALID_MATRIX_PRODUCT)
    eigen_assert(v.rows() == cols() && "invalid product: dimensions do not match");
    return Product<DiagonalPlusLowRank, Rhs>(*this, v.derived());
  }

  /** \internal Computes \c dst += alpha * (*this) * rhs. \c ProductScalar is the
   * promoted scalar of the product (complex when a real operator is applied to a
   * complex right-hand side); the workspaces and the accumulation run in the
   * promoted type. */
  template <typename Dest, typename Rhs, typename ProductScalar>
  void addProduct(Dest& dst, const Rhs& rhs, const ProductScalar& alpha) const {
    eigen_assert(rhs.rows() == rows() && "invalid product: dimensions do not match");
    const Matrix<ProductScalar, Size_, Rhs::ColsAtCompileTime> r = rhs;
    dst += alpha * (m_d.asDiagonal() * r);
    if (correctionRank() > 0) {
      const Matrix<ProductScalar, Rank_, Rhs::ColsAtCompileTime> t = m_V.adjoint() * r;
      dst.noalias() += alpha * (m_U * t);
    }
  }

 private:
  DiagonalVector m_d;
  FactorType m_U;
  FactorType m_V;
};

/** \ingroup StructuredMatrices_Module
 * \returns a \ref DiagonalPlusLowRank operator \c diag(d) + \c U*V.adjoint().
 * The compile-time dimension and rank are deduced from the arguments. */
template <typename DDerived, typename UDerived, typename VDerived>
DiagonalPlusLowRank<typename DDerived::Scalar, DDerived::SizeAtCompileTime, UDerived::ColsAtCompileTime>
makeDiagonalPlusLowRank(const MatrixBase<DDerived>& d, const MatrixBase<UDerived>& U, const MatrixBase<VDerived>& V) {
  return DiagonalPlusLowRank<typename DDerived::Scalar, DDerived::SizeAtCompileTime, UDerived::ColsAtCompileTime>(d, U,
                                                                                                                  V);
}

namespace internal {

// Single product specialization covering every product tag; see the note in
// Circulant.h.
template <typename Scalar_, int Size_, int Rank_, typename Rhs, int ProductTag>
struct generic_product_impl<DiagonalPlusLowRank<Scalar_, Size_, Rank_>, Rhs, StructuredShape, DenseShape, ProductTag>
    : structured_product_impl<DiagonalPlusLowRank<Scalar_, Size_, Rank_>, Rhs> {};

}  // namespace internal

}  // namespace Eigen

#endif  // EIGEN_STRUCTURED_DIAGONAL_PLUS_LOW_RANK_H
