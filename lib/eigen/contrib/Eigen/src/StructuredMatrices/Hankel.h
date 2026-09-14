// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

// References:
//  [1] G. H. Golub and C. F. Van Loan, "Matrix Computations", 4th ed., Johns
//      Hopkins University Press, 2013, chapter 4.7 (Toeplitz, Hankel and
//      related systems) and chapter 4.8 (fast products via circulant embedding
//      and the FFT).
//  [2] R. H. Chan and X.-Q. Jin, "An Introduction to Iterative Toeplitz
//      Solvers", SIAM, 2007 (a Hankel matrix is a column-reversed Toeplitz
//      matrix, so its products and solves reduce to the Toeplitz machinery).

#ifndef EIGEN_STRUCTURED_HANKEL_H
#define EIGEN_STRUCTURED_HANKEL_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

template <typename Scalar_, int Rows_ = Dynamic, int Cols_ = Dynamic>
class Hankel;

namespace internal {

template <typename Scalar_, int Rows_, int Cols_>
struct traits<Hankel<Scalar_, Rows_, Cols_>> {
  using Scalar = Scalar_;
  using StorageKind = Dense;
  using XprKind = MatrixXpr;
  using StorageIndex = int;
  static constexpr int RowsAtCompileTime = Rows_;
  static constexpr int ColsAtCompileTime = Cols_;
  static constexpr int MaxRowsAtCompileTime = Rows_;
  static constexpr int MaxColsAtCompileTime = Cols_;
  // Deliberately no NestByRefBit: transpose(), conjugate() and adjoint() return
  // owning temporaries, so Product must nest the operator by value for a
  // delayed-evaluated product expression to keep its left factor alive. The copy
  // is O(n), negligible against the O(n log n) product evaluation.
  static constexpr unsigned int Flags = 0;
};

template <typename Scalar_, int Rows_, int Cols_>
struct evaluator_traits<Hankel<Scalar_, Rows_, Cols_>> {
  using Kind = IndexBased;
  using Shape = StructuredShape;
};

}  // namespace internal

/** \ingroup StructuredMatrices_Module
 * \class Hankel
 * \brief An \c m x \c n Hankel matrix represented by its first column and last row.
 *
 * A Hankel matrix has constant anti-diagonals: entry \c (i,j) equals \c h[i+j],
 * where \c h is the length \c m+n-1 sequence formed by the first column followed
 * by the tail of the last row. The overlap entry \c h[m-1] is taken from the
 * column, so \c row[0] is ignored. Every column of the matrix is a contiguous
 * slice of \c h. A real square Hankel matrix is symmetric.
 *
 * The matrix-vector product (\c operator*) is evaluated in O(n log n): a Hankel
 * matrix is a column-reversed Toeplitz matrix, so the product is a circulant
 * convolution of \c h with the reversed input, evaluated through a DFT symbol
 * computed once at construction. (Small operators, and single-row or
 * single-column ones -- whose products cost O(n) directly -- skip the FFT for a
 * direct evaluation.) As with \ref Toeplitz, \c operator* returns an Eigen
 * product expression, so a \c Hankel plugs into the matrix-free iterative
 * solvers (including the least-squares solvers, through \ref adjoint), and it can
 * be assigned to a dense matrix when an explicit representation is needed. As
 * with any matrix-free operator, the iterative solvers must be instantiated with
 * \c IdentityPreconditioner (e.g.
 * \c LeastSquaresConjugateGradient<Hankel<double>,IdentityPreconditioner>):
 * the default preconditioners read individual coefficients through \c col() or
 * \c InnerIterator, which the structured operators do not expose.
 *
 * The class is closed under \ref transpose, \ref conjugate and \ref adjoint. The
 * transpose is the Hankel matrix of the same sequence with the dimensions
 * swapped, and its symbol is obtained from the cached one by a diagonal phase
 * multiplication instead of a new FFT.
 *
 * Square linear systems are solved directly in O(n^2) through the column-reversed
 * \ref Toeplitz equivalent (\ref toToeplitz) and the \ref LookAheadLevinson
 * solver, see \ref solve.
 *
 * \tparam Scalar_ the scalar type, real or complex.
 * \tparam Rows_ the number of rows at compile time, or \c Dynamic (the default).
 * \tparam Cols_ the number of columns at compile time, or \c Dynamic (the default).
 *
 * \sa class Toeplitz, class Circulant, makeHankel()
 */
template <typename Scalar_, int Rows_, int Cols_>
class Hankel : public EigenBase<Hankel<Scalar_, Rows_, Cols_>> {
 public:
  using Scalar = Scalar_;
  using RealScalar = typename NumTraits<Scalar>::Real;
  using StorageIndex = int;
  using Complex = std::complex<RealScalar>;
  using ColGeneratorType = Matrix<Scalar, Rows_, 1>;
  using RowGeneratorType = Matrix<Scalar, Cols_, 1>;
  using ComplexVector = Matrix<Complex, Dynamic, 1>;

  static constexpr int RowsAtCompileTime = Rows_;
  static constexpr int ColsAtCompileTime = Cols_;
  static constexpr int MaxRowsAtCompileTime = Rows_;
  static constexpr int MaxColsAtCompileTime = Cols_;
  static constexpr int SizeAtCompileTime = internal::size_at_compile_time(Rows_, Cols_);
  static constexpr int MaxSizeAtCompileTime = SizeAtCompileTime;
  static constexpr bool IsRowMajor = false;
  // Deliberately no IsVectorAtCompileTime: Ref<const Hankel>'s default StrideType
  // argument reads it, so its absence makes internal::is_ref_compatible SFINAE to
  // false and keeps the iterative solvers on their matrix-free path.

  /** \internal Compile-time length of the generating sequence h. */
  static constexpr int GenSizeAtCompileTime = (Rows_ == Dynamic || Cols_ == Dynamic) ? Dynamic : (Rows_ + Cols_ - 1);
  using GeneratorType = Matrix<Scalar, GenSizeAtCompileTime, 1>;

  /** Builds a Hankel matrix from its first column \a col and last row \a row.
   * The anti-diagonal overlap entry is taken from \a col, hence \c row[0] is
   * ignored.
   *
   * Unless the matrix is small, or a single row or column (whose products are
   * always evaluated directly, in linear time), the DFT symbol of the underlying
   * circulant convolution is computed here, once, and reused by every subsequent
   * product. */
  template <typename ColDerived, typename RowDerived>
  Hankel(const MatrixBase<ColDerived>& col, const MatrixBase<RowDerived>& row)
      : m_rows(col.size()), m_cols(row.size()) {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(ColDerived)
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(RowDerived)
    const Index m = col.size(), n = row.size();
    eigen_assert(m > 0 && n > 0 && "Hankel generators must be non-empty");
    m_h.resize(m + n - 1);
    m_h.head(m) = col;
    if (n > 1) m_h.tail(n - 1) = row.tail(n - 1);
    if (m > 1 && n > 1 && (m > internal::structured_direct_threshold() || n > internal::structured_direct_threshold()))
      m_symbol = computeSymbol();
    m_fftUsable = computeFftUsable();
  }

  EIGEN_DEVICE_FUNC Index rows() const { return m_rows.value(); }
  EIGEN_DEVICE_FUNC Index cols() const { return m_cols.value(); }

  /** \returns the generating sequence \c h of length \c m+n-1; entry \c (i,j) of
   * the matrix is \c h[i+j]. */
  const GeneratorType& generator() const { return m_h; }

  /** \returns the first column, \c h[0..m-1]. */
  ColGeneratorType column() const { return m_h.head(rows()); }

  /** \returns the last row, \c h[m-1..m+n-2]. */
  RowGeneratorType lastRow() const { return m_h.segment(rows() - 1, cols()); }

  /** \returns the symbol of the underlying circulant convolution: the DFT of the
   * generating sequence, zero-padded to a 5-smooth size p >= m+n-1 and rotated so
   * the product appears in the leading \c m entries of the back-transform. Cached
   * when the operator is large enough for products to take the FFT path, computed
   * on the fly for small operators. */
  ComplexVector symbol() const { return m_symbol.size() > 0 ? m_symbol : computeSymbol(); }

  /** \returns the coefficient at row \a row and column \a col. */
  Scalar coeff(Index row, Index col) const { return m_h.coeff(row + col); }

  /** \returns the transpose of \c *this, itself a Hankel operator: the same
   * generating sequence with the dimensions swapped. The cached symbol, when
   * present, is reused: transposing changes the rotation of the embedding from
   * \c n-1 to \c m-1, which multiplies DFT entry \c f by \c exp(2 pi i f (m-n)/p)
   * -- a diagonal phase multiplication instead of a new FFT. */
  Hankel<Scalar, Cols_, Rows_> transpose() const {
    return Hankel<Scalar, Cols_, Rows_>(m_h, cols(), rows(), transposedSymbol());
  }

  /** \returns the complex conjugate of \c *this, itself a Hankel operator. The
   * cached symbol, when present, is reused: the symbol of the conjugate is the
   * conjugated index reversal of the symbol. */
  Hankel conjugate() const {
    return Hankel(m_h.conjugate(), rows(), cols(), internal::structured_reverse_symbol(m_symbol).conjugate());
  }

  /** \returns the adjoint of \c *this, itself a Hankel operator (the conjugated
   * transpose); the cached symbol is reused through the composition of the
   * \ref conjugate and \ref transpose rules. */
  Hankel<Scalar, Cols_, Rows_> adjoint() const { return conjugate().transpose(); }

  /** \returns the column-reversed \ref Toeplitz equivalent \c T = H*E (with \c E
   * the exchange matrix), i.e. \c T(i,j) = h[i + n-1 - j]. Its symbol is computed
   * afresh. Useful for solving Hankel systems with the Toeplitz machinery:
   * \c H*x = b is \c T*(E*x) = b. */
  Toeplitz<Scalar, Rows_, Cols_> toToeplitz() const {
    ColGeneratorType c = m_h.segment(cols() - 1, rows());
    RowGeneratorType r = m_h.head(cols()).reverse();
    return Toeplitz<Scalar, Rows_, Cols_>(c, r);
  }

  /** \internal Compile-time row count of solve() results: the solution of a
   * square system has cols() == rows() entries, so either fixed dimension
   * determines it at compile time. */
  static constexpr int SolveRowsAtCompileTime = Cols_ != Dynamic ? Cols_ : Rows_;

  /** \returns the solution \c x of \c (*this) * x = b for a \b square Hankel
   * matrix, computed in O(n^2) by solving the column-reversed Toeplitz system
   * \c T*y = b with the \ref LookAheadLevinson solver and reversing the solution.
   * Supports multiple right-hand sides. Each call factorizes anew; to reuse a
   * factorization across calls, or to check for numerical breakdown through
   * \c info(), use \c LookAheadLevinson on \ref toToeplitz directly and reverse
   * the solution rows. */
  template <typename Rhs>
  Matrix<Scalar, SolveRowsAtCompileTime, Rhs::ColsAtCompileTime> solve(const MatrixBase<Rhs>& b) const {
    EIGEN_STATIC_ASSERT(RowsAtCompileTime == Dynamic || Rhs::RowsAtCompileTime == Dynamic ||
                            int(RowsAtCompileTime) == int(Rhs::RowsAtCompileTime),
                        YOU_MIXED_MATRICES_OF_DIFFERENT_SIZES)
    eigen_assert(rows() == cols() && "Hankel::solve requires a square matrix");
    eigen_assert(b.rows() == rows() && "right-hand side has the wrong number of rows");
    LookAheadLevinson<Scalar> levinson(toToeplitz());
    eigen_assert(levinson.info() == Success &&
                 "Hankel::solve: the Levinson recursion broke down; "
                 "use LookAheadLevinson on toToeplitz() for diagnostics");
    return levinson.solve(b).colwise().reverse();
  }

  /** \internal Writes the dense representation into \a dst; column \c j is the
   * contiguous slice \c h[j..j+m-1]. Invoked through \c dense = hankel; */
  template <typename Dest>
  void evalTo(Dest& dst) const {
    for (Index j = 0; j < cols(); ++j) dst.col(j) = m_h.segment(j, rows());
  }

  /** \internal Computes \c dst += (*this), see evalTo(). */
  template <typename Dest>
  void addTo(Dest& dst) const {
    for (Index j = 0; j < cols(); ++j) dst.col(j) += m_h.segment(j, rows());
  }

  /** \internal Computes \c dst -= (*this), see evalTo(). */
  template <typename Dest>
  void subTo(Dest& dst) const {
    for (Index j = 0; j < cols(); ++j) dst.col(j) -= m_h.segment(j, rows());
  }

  /** \returns the product expression \c (*this) * \a x, evaluated through a fast
   * FFT-based matrix-vector product. The expression carries the default product
   * tag, so assigning it behaves like any dense product: a temporary resolves
   * aliasing between the destination and \a x, and \c .noalias() skips it. */
  template <typename Rhs>
  Product<Hankel, Rhs> operator*(const MatrixBase<Rhs>& x) const {
    EIGEN_STATIC_ASSERT(ColsAtCompileTime == Dynamic || Rhs::RowsAtCompileTime == Dynamic ||
                            int(ColsAtCompileTime) == int(Rhs::RowsAtCompileTime),
                        INVALID_MATRIX_PRODUCT)
    eigen_assert(x.rows() == cols() && "invalid product: dimensions do not match");
    return Product<Hankel, Rhs>(*this, x.derived());
  }

  /** \internal Computes \c dst += alpha * (*this) * rhs. \c ProductScalar is the
   * promoted scalar of the product (complex when a real operator is applied to a
   * complex right-hand side); the accumulation runs in the promoted type.
   *
   * The FFT path evaluates the circular convolution
   * \f[ (Hx)_i = \sum_{j=0}^{n-1} h_{i+j} x_j. \f]
   * Reversing \f$x\f$ and rotating the padded symbol by \f$n-1\f$ places these
   * entries at the start of the inverse transform.
   *
   * Non-finite data takes the direct kernels: the transforms would smear a
   * single Inf/NaN into NaNs across the whole output, where the dense product
   * only propagates it through the dot products that touch it. A non-finite
   * generating sequence or cached symbol (which can overflow even for a finite
   * sequence) routes the whole product; a non-finite right-hand-side column is
   * detected inside the FFT loop -- in the same pass that derives its scaling
   * exponent, so finite data pays no extra scan -- and falls back per column. */
  template <typename Dest, typename Rhs, typename ProductScalar>
  void addProduct(Dest& dst, const Rhs& rhs, const ProductScalar& alpha) const {
    const Index m = rows(), n = cols();
    eigen_assert(rhs.rows() == n && "invalid product: dimensions do not match");
    const bool small = m == 1 || n == 1 ||
                       (m <= internal::structured_direct_threshold() && n <= internal::structured_direct_threshold());
    if (small || !m_fftUsable) {
      directProduct(dst, rhs, alpha);
      return;
    }
    // Reversal preserves which right-hand-side columns need the direct fallback.
    internal::structured_fft_apply(dst, m_symbol, m, rhs.colwise().reverse(), alpha,
                                   [&](Index k) { directProductColumn(dst, rhs, k, alpha); });
  }

 private:
  template <typename OtherScalar, int OtherRows, int OtherCols>
  friend class Hankel;

  /** \internal Builds an operator from the generating sequence and an
   * already-known symbol (empty for small operators), skipping the FFT of the
   * public constructor. Used by transpose(), conjugate() and adjoint(), whose
   * symbols are cheap transformations of the existing one. */
  Hankel(const GeneratorType& h, Index rows, Index cols, const ComplexVector& symbol)
      : m_rows(rows), m_cols(cols), m_h(h), m_symbol(symbol), m_fftUsable(computeFftUsable()) {}

  /** \internal Whether products may take the FFT path: the generating sequence
   * and the cached symbol must be finite. The symbol accumulates up to p addends,
   * so it can overflow to Inf even for a finite sequence; such operators fall
   * back to the direct kernels, which stay exact. */
  bool computeFftUsable() const { return m_h.allFinite() && (m_symbol.size() == 0 || m_symbol.allFinite()); }

  /** \internal Direct kernel for column \a k of the right-hand side: computes
   * \c dst.col(k) += alpha * (*this) * rhs.col(k) without transforms. Serves
   * operators below the FFT threshold, single-row and single-column operators
   * (whose products cost O(n) directly, however large the operator), and any
   * column involving non-finite data, whose entrywise IEEE semantics the
   * transforms cannot preserve. */
  template <typename Dest, typename Rhs, typename ProductScalar>
  void directProductColumn(Dest& dst, const Rhs& rhs, Index k, const ProductScalar& alpha) const {
    const Index m = rows(), n = cols();
    // A unit alpha must not multiply: even the identity complex scalar (1,0)
    // pollutes an (Inf,0) value with NaN through the 0*Inf cross term.
    const bool unitAlpha = alpha == ProductScalar(1);

    if (n == 1) {
      // The runtime-sized head keeps other fixed-size shapes well-formed.
      const ProductScalar xj = unitAlpha ? ProductScalar(rhs.coeff(0, k)) : ProductScalar(alpha * rhs.coeff(0, k));
      dst.col(k) += xj * m_h.head(m);
      return;
    }

    if (m == 1) {
      const ProductScalar acc = m_h.head(n).cwiseProduct(rhs.col(k)).sum();
      dst.coeffRef(0, k) += unitAlpha ? acc : ProductScalar(alpha * acc);
      return;
    }

    if (m <= internal::structured_scalar_threshold() && n <= internal::structured_scalar_threshold()) {
      // Tiny sizes: a plain scalar loop beats the segment-based path below, whose
      // per-segment setup dominates when segments hold only a few entries.
      for (Index i = 0; i < m; ++i) {
        ProductScalar acc(0);
        for (Index j = 0; j < n; ++j) acc += coeff(i, j) * rhs.coeff(j, k);
        dst.coeffRef(i, k) += unitAlpha ? acc : ProductScalar(alpha * acc);
      }
      return;
    }

    // Column j is the contiguous, vectorizable slice h[j..j+m-1].
    auto dstCol = dst.col(k);
    for (Index j = 0; j < n; ++j) {
      const ProductScalar xj = unitAlpha ? ProductScalar(rhs.coeff(j, k)) : ProductScalar(alpha * rhs.coeff(j, k));
      dstCol += xj * m_h.segment(j, m);
    }
  }

  /** \internal Direct product kernel over every column, see
   * directProductColumn(). */
  template <typename Dest, typename Rhs, typename ProductScalar>
  void directProduct(Dest& dst, const Rhs& rhs, const ProductScalar& alpha) const {
    for (Index k = 0; k < rhs.cols(); ++k) directProductColumn(dst, rhs, k, alpha);
  }

  /** \internal \returns the DFT of the generating sequence, zero-padded to the
   * 5-smooth size p >= m+n-1 and rotated left by n-1 so that the convolution
   * against a reversed input lands the product in the leading m entries. */
  ComplexVector computeSymbol() const {
    const Index m = rows(), n = cols();
    const Index p = internal::fft_next_good_size(m + n - 1);
    ComplexVector embedding = ComplexVector::Zero(p);
    embedding.head(m) = m_h.tail(m).template cast<Complex>();  // h[n-1 .. m+n-2]
    embedding.tail(n - 1) = m_h.head(n - 1).template cast<Complex>();
    if (p == 1) return embedding;  // the DFT of a single sample is the identity
    ComplexVector symbol(p);
    auto&& fft = internal::structured_fft_engine<RealScalar>();
    fft.fwd(symbol, embedding, p);
    return symbol;
  }

  /** \internal \returns the symbol of the transposed operator: the embedding
   * rotation changes from n-1 to m-1, which by the DFT shift theorem multiplies
   * entry f by exp(2 pi i f (m-n) / p). The frequency-shift product f*(m-n) is
   * accumulated incrementally modulo p, so the angle stays O(2 pi) at full
   * accuracy and no Index overflow can occur. An empty symbol stays empty. */
  ComplexVector transposedSymbol() const {
    ComplexVector sym = m_symbol;
    const Index p = sym.size();
    if (p > 0 && rows() != cols()) {
      Index s = (rows() - cols()) % p;
      if (s < 0) s += p;
      Index fs = 0;  // f * s mod p
      for (Index f = 0; f < p; ++f) {
        sym[f] *= std::polar(RealScalar(1), RealScalar(2 * EIGEN_PI) * RealScalar(fs) / RealScalar(p));
        fs += s;
        if (fs >= p) fs -= p;
      }
    }
    return sym;
  }

  // Empty (compile-time constant) when the corresponding dimension is fixed; the
  // dimensions cannot be recovered from m_h alone, whose length is rows()+cols()-1.
  internal::variable_if_dynamic<Index, RowsAtCompileTime> m_rows;
  internal::variable_if_dynamic<Index, ColsAtCompileTime> m_cols;
  GeneratorType m_h;
  ComplexVector m_symbol;
  bool m_fftUsable;
};

/** \ingroup StructuredMatrices_Module
 * \returns a \ref Hankel operator with first column \a col and last row \a row.
 * The compile-time dimensions of the operator are deduced from the generators. */
template <typename ColDerived, typename RowDerived>
Hankel<typename ColDerived::Scalar, ColDerived::SizeAtCompileTime, RowDerived::SizeAtCompileTime> makeHankel(
    const MatrixBase<ColDerived>& col, const MatrixBase<RowDerived>& row) {
  return Hankel<typename ColDerived::Scalar, ColDerived::SizeAtCompileTime, RowDerived::SizeAtCompileTime>(col, row);
}

namespace internal {

// Single product specialization covering every product tag; see the note in
// Circulant.h.
template <typename Scalar_, int Rows_, int Cols_, typename Rhs, int ProductTag>
struct generic_product_impl<Hankel<Scalar_, Rows_, Cols_>, Rhs, StructuredShape, DenseShape, ProductTag>
    : structured_product_impl<Hankel<Scalar_, Rows_, Cols_>, Rhs> {};

}  // namespace internal

}  // namespace Eigen

#endif  // EIGEN_STRUCTURED_HANKEL_H
