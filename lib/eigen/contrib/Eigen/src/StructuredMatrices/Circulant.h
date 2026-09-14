// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

// References:
//  [1] P. J. Davis, "Circulant Matrices", Wiley, 1979. Diagonalization of a
//      circulant matrix by the DFT and the closed-form eigenstructure used by
//      eigenvalues()/eigenvectors(); the SVD and pseudo-inverse below follow
//      from it by taking moduli/phases of the eigenvalues.
//  [2] R. M. Gray, "Toeplitz and Circulant Matrices: A Review", Foundations and
//      Trends in Communications and Information Theory, 2(3), 2006.
//  [3] G. H. Golub and C. F. Van Loan, "Matrix Computations", 4th ed., Johns
//      Hopkins University Press, 2013, chapter 4.8 (circulant systems and
//      FFT-based products) and chapter 5.4 (numerical rank conventions).
//  [4] J. J. Dongarra, J. R. Bunch, C. B. Moler and G. W. Stewart, "LINPACK
//      Users' Guide", SIAM, 1979. determinant()'s balanced accumulation follows
//      the convention of its xGEDI routines, which return determinants as a
//      (fraction, exponent) pair to avoid spurious overflow/underflow.
//  [5] P. H. Sterbenz, "Floating-Point Computation", Prentice-Hall, 1974.
//      Scaling by a power of two is exact, the property the balanced
//      accumulation, the rescaled rank threshold and the scaled FFT products
//      rely on.

#ifndef EIGEN_STRUCTURED_CIRCULANT_H
#define EIGEN_STRUCTURED_CIRCULANT_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

template <typename Scalar_, int Size_ = Dynamic>
class Circulant;

namespace internal {

template <typename Scalar_, int Size_>
struct traits<Circulant<Scalar_, Size_>> {
  using Scalar = Scalar_;
  using StorageKind = Dense;
  using XprKind = MatrixXpr;
  using StorageIndex = int;
  static constexpr int RowsAtCompileTime = Size_;
  static constexpr int ColsAtCompileTime = Size_;
  static constexpr int MaxRowsAtCompileTime = Size_;
  static constexpr int MaxColsAtCompileTime = Size_;
  // Deliberately no NestByRefBit: transpose(), conjugate() and adjoint() return
  // owning temporaries, so Product must nest the operator by value for a
  // delayed-evaluated product expression to keep its left factor alive. The copy
  // is O(n), negligible against the O(n log n) product evaluation.
  static constexpr int Flags = 0;
};

template <typename Scalar_, int Size_>
struct evaluator_traits<Circulant<Scalar_, Size_>> {
  using Kind = IndexBased;
  using Shape = StructuredShape;
};

}  // namespace internal

/** \ingroup StructuredMatrices_Module
 * \class Circulant
 * \brief An \c n x \c n circulant matrix represented by its first column.
 *
 * For first column \f$c\f$ and unitary DFT matrix \f$F\f$,
 * \f[ C_{ij}=c_{(i-j)\bmod n}, \qquad C=F^*\operatorname{diag}(Fc)F. \f]
 * Thus its eigenvalues -- the operator's \em symbol -- are \f$Fc\f$, computed
 * once at construction and reused by every product. This yields an O(n log n)
 * matrix-vector product (\c operator*), an
 * O(n log n) direct (pseudo-inverse) solve (\ref solve), and closed-form
 * factorizations: the eigendecomposition (\ref eigenvalues, \ref eigenvectors)
 * and the SVD (\ref singularValues, \ref matrixU, \ref matrixV) in the Fourier
 * basis, plus \ref rank, \ref inverse and \ref determinant. The class is closed
 * under \ref transpose, \ref conjugate and \ref adjoint, which reuse the cached
 * symbol instead of recomputing FFTs.
 *
 * The operator stores its own copy of the generating column and derives from
 * \c EigenBase. Because \c operator* returns an Eigen product expression, a
 * \c Circulant also drops into the matrix-free iterative solvers, and it can be
 * assigned to a dense matrix when an explicit representation is needed. As with
 * any matrix-free operator, the iterative solvers must be instantiated with
 * \c IdentityPreconditioner (e.g.
 * \c ConjugateGradient<Circulant<double>,Lower|Upper,IdentityPreconditioner>):
 * the default preconditioners read individual coefficients through \c col() or
 * \c InnerIterator, which the structured operators do not expose.
 *
 * \tparam Scalar_ the scalar type, real or complex.
 * \tparam Size_ the dimension at compile time, or \c Dynamic (the default).
 *
 * \sa class Toeplitz, makeCirculant()
 */
template <typename Scalar_, int Size_>
class Circulant : public EigenBase<Circulant<Scalar_, Size_>> {
 public:
  using Scalar = Scalar_;
  using RealScalar = typename NumTraits<Scalar>::Real;
  using StorageIndex = int;
  using Complex = std::complex<RealScalar>;
  using GeneratorType = Matrix<Scalar, Size_, 1>;
  using ComplexVector = Matrix<Complex, Dynamic, 1>;
  using RealVector = Matrix<RealScalar, Size_, 1>;
  using ComplexMatrix = Matrix<Complex, Size_, Size_>;

  static constexpr int RowsAtCompileTime = Size_;
  static constexpr int ColsAtCompileTime = Size_;
  static constexpr int MaxRowsAtCompileTime = Size_;
  static constexpr int MaxColsAtCompileTime = Size_;
  static constexpr int SizeAtCompileTime = internal::size_at_compile_time(Size_, Size_);
  static constexpr int MaxSizeAtCompileTime = SizeAtCompileTime;
  static constexpr bool IsRowMajor = false;
  // Deliberately no IsVectorAtCompileTime: Ref<const Circulant>'s default StrideType
  // argument reads it, so its absence makes internal::is_ref_compatible SFINAE to
  // false and keeps the iterative solvers on their matrix-free path.

  /** Builds a circulant matrix from its first column \a col.
   *
   * When the matrix is large enough for products to take the FFT path, the DFT of
   * \a col -- the eigenvalues of the matrix -- is computed here, once, and reused
   * by every subsequent product and solve. */
  template <typename Derived>
  explicit Circulant(const MatrixBase<Derived>& col) : m_col(col) {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived)
    eigen_assert(m_col.size() > 0 && "Circulant generator must be non-empty");
    if (m_col.size() > internal::structured_direct_threshold()) {
      m_symbol = computeSymbol();
      m_prodSymbol = computeProdSymbol(m_col);
    }
    m_fftUsable = computeFftUsable();
  }

  EIGEN_DEVICE_FUNC Index rows() const { return m_col.size(); }
  EIGEN_DEVICE_FUNC Index cols() const { return m_col.size(); }

  /** \returns the generating first column. */
  const GeneratorType& column() const { return m_col; }

  /** \returns the symbol of the operator, i.e. the DFT of the generating column.
   * Its entries are the eigenvalues of the matrix. Cached when the operator is
   * large enough for products to take the FFT path, computed on the fly for small
   * operators. */
  ComplexVector symbol() const { return m_symbol.size() > 0 ? m_symbol : computeSymbol(); }

  /** \returns the coefficient at row \a row and column \a col. */
  Scalar coeff(Index row, Index col) const {
    Index k = row - col;
    if (k < 0) k += rows();
    return m_col.coeff(k);
  }

  /** \returns the transpose of \c *this, itself a \c Circulant operator: the one
   * generated by the index-reversed column. The cached symbols, when present, are
   * reused -- the symbols of the transpose are the index reversals of the symbols
   * -- so no FFT is recomputed. */
  Circulant transpose() const {
    const Index n = rows();
    GeneratorType col(n);
    col[0] = m_col[0];
    if (n > 1) col.tail(n - 1) = m_col.tail(n - 1).reverse();
    return Circulant(col, internal::structured_reverse_symbol(m_symbol),
                     internal::structured_reverse_symbol(m_prodSymbol));
  }

  /** \returns the complex conjugate of \c *this, itself a \c Circulant operator.
   * The cached symbols, when present, are reused: the symbols of the conjugate are
   * the conjugated index reversals of the symbols. */
  Circulant conjugate() const {
    return Circulant(m_col.conjugate(), internal::structured_reverse_symbol(m_symbol).conjugate(),
                     internal::structured_reverse_symbol(m_prodSymbol).conjugate());
  }

  /** \returns the adjoint of \c *this, itself a \c Circulant operator. The cached
   * symbols, when present, are reused: the symbols of the adjoint are the
   * elementwise conjugates of the symbols (the eigenvalues conjugate while the
   * Fourier eigenbasis stays fixed). */
  Circulant adjoint() const {
    const Index n = rows();
    GeneratorType col(n);
    col[0] = numext::conj(m_col[0]);
    if (n > 1) col.tail(n - 1) = m_col.tail(n - 1).reverse().conjugate();
    return Circulant(col, m_symbol.conjugate(), m_prodSymbol.conjugate());
  }

  /** \internal Writes the dense representation into \a dst; column \c j is the
   * generator rotated downwards by \c j, so only contiguous segment copies are
   * involved. Invoked through \c dense = circulant; */
  template <typename Dest>
  void evalTo(Dest& dst) const {
    const Index n = rows();
    for (Index j = 0; j < n; ++j) {
      dst.col(j).head(j) = m_col.tail(j);
      dst.col(j).tail(n - j) = m_col.head(n - j);
    }
  }

  /** \internal Computes \c dst += (*this), see evalTo(). */
  template <typename Dest>
  void addTo(Dest& dst) const {
    const Index n = rows();
    for (Index j = 0; j < n; ++j) {
      dst.col(j).head(j) += m_col.tail(j);
      dst.col(j).tail(n - j) += m_col.head(n - j);
    }
  }

  /** \internal Computes \c dst -= (*this), see evalTo(). */
  template <typename Dest>
  void subTo(Dest& dst) const {
    const Index n = rows();
    for (Index j = 0; j < n; ++j) {
      dst.col(j).head(j) -= m_col.tail(j);
      dst.col(j).tail(n - j) -= m_col.head(n - j);
    }
  }

  /** \returns the product expression \c (*this) * \a x, evaluated through a fast
   * FFT-based matrix-vector product. The expression carries the default product
   * tag, so assigning it behaves like any dense product: a temporary resolves
   * aliasing between the destination and \a x, and \c .noalias() skips it. */
  template <typename Rhs>
  Product<Circulant, Rhs> operator*(const MatrixBase<Rhs>& x) const {
    EIGEN_STATIC_ASSERT(ColsAtCompileTime == Dynamic || Rhs::RowsAtCompileTime == Dynamic ||
                            int(ColsAtCompileTime) == int(Rhs::RowsAtCompileTime),
                        INVALID_MATRIX_PRODUCT)
    eigen_assert(x.rows() == cols() && "invalid product: dimensions do not match");
    return Product<Circulant, Rhs>(*this, x.derived());
  }

  /** \returns the minimum-norm least-squares solution of \c (*this) * x = b,
   * computed directly in the Fourier domain. Symbol entries whose modulus reaches
   * the rank threshold (see \ref rank) are inverted; the remaining ones are
   * treated as exact zeros, so the result is the pseudo-inverse applied to \a b.
   * For a non-singular operator this is the exact solution
   * \c x = ifft( fft(b) ./ fft(c) ). Supports multiple right-hand sides. */
  template <typename Rhs>
  Matrix<Scalar, Size_, Rhs::ColsAtCompileTime> solve(const MatrixBase<Rhs>& b) const {
    EIGEN_STATIC_ASSERT(RowsAtCompileTime == Dynamic || Rhs::RowsAtCompileTime == Dynamic ||
                            int(RowsAtCompileTime) == int(Rhs::RowsAtCompileTime),
                        YOU_MIXED_MATRICES_OF_DIFFERENT_SIZES)
    const Index n = rows();
    eigen_assert(b.rows() == n && "right-hand side has the wrong number of rows");
    const ComplexVector s = symbol();
    RealVector mods;
    RealScalar tol;
    int es = 0;
    scaledModuli(s, mods, tol, &es);
    // Strictly-below-threshold entries are zeroed, matching SVDBase::rank(), so a
    // smallest-normal 1x1 operator stays invertible. NaN moduli fail the
    // comparison and land in the inverted set, so a NaN input propagates to the
    // output instead of being silently zeroed.
    ComplexVector sinv(n);
    internal::structured_symbol_reciprocal(s.data(), mods.data(), tol, sinv.data(), n);
    Matrix<Scalar, Size_, Rhs::ColsAtCompileTime> x(n, b.cols());
    x.setZero();
    if (!b.allFinite()) {
      // A non-finite right-hand side cannot go through the transforms (see
      // addProduct): apply the pseudo-inverse -- itself a circulant matrix,
      // generated by the inverse DFT of the thresholded reciprocal symbol --
      // through the direct kernel so Inf/NaN propagate entrywise.
      GeneratorType pcol(n);
      if (n == 1) {
        pcol[0] = internal::structured_scalar_part_impl<Scalar>::run_scalar(sinv.coeff(0));
      } else {
        auto&& fft = internal::structured_fft_engine<RealScalar>();
        ComplexVector pt(n);
        fft.inv(pt, sinv, n);
        pcol = internal::structured_scalar_part_impl<Scalar>::run(pt);
      }
      Circulant(pcol, ComplexVector(), ComplexVector()).directProduct(x, b.derived(), Scalar(1));
      return x;
    }
    // The reciprocal of the largest symbol entry is about 2^-es: subnormal, hence
    // lost under flush-to-zero, only for a symbol at the top of the exponent range.
    // Dividing by the symbol keeps that mode normal but costs a complex division
    // per entry, so it is reserved for that band.
    constexpr Index kSubnormalReciprocal = -(Index(std::numeric_limits<RealScalar>::min_exponent) - 1) - 2;
    if (es <= kSubnormalReciprocal) {
      internal::structured_fft_apply(x, sinv, n, b.derived(), Scalar(1));
      return x;
    }
    // ldexp per component rather than a multiply by 2^-es: for a near-boundary
    // symbol that constant is itself subnormal, and reads as zero under
    // flush-to-zero. The apply folds es back into the output scaling.
    const ComplexVector sb = s.unaryExpr([es](const Complex& z) { return internal::structured_ldexp_clamped(z, -es); });
    // mods and tol are already in a balanced frame, so rebalancing the symbol
    // leaves the threshold comparison unchanged.
    internal::structured_symbol_divide<RealVector, RealScalar> divide{&mods, tol};
    internal::structured_fft_apply(
        x, sb, n, b.derived(), Scalar(1),
        [](Index) { eigen_assert(false && "non-finite column requires a direct kernel"); }, divide, es);
    return x;
  }

  /** \returns the numerical rank: the number of symbol entries whose modulus is
   * no smaller than the threshold \c n * epsilon * max_k|symbol[k]|, both
   * evaluated in an exactly rescaled frame so the moduli cannot overflow (see
   * scaledModuli()). This is the same threshold \ref solve uses to decide which
   * Fourier components to invert, and the comparison is strict like SVDBase's,
   * so an entry sitting exactly on the threshold still counts as non-zero. */
  Index rank() const {
    const ComplexVector s = symbol();
    RealVector mods;
    RealScalar tol;
    scaledModuli(s, mods, tol);
    return (!(mods.array() < tol)).count();  // negated so NaN entries count as non-zero
  }

  /** \returns the inverse of \c *this, itself a \c Circulant operator: the one
   * generated by \c ifft(1 ./ symbol), the first column of the inverse matrix.
   * \warning The operator must be non-singular; use \ref solve for a
   * pseudo-inverse solve of a rank-deficient operator. */
  Circulant inverse() const {
    const Index n = rows();
    ComplexVector sinv = symbol();
    internal::structured_symbol_reciprocal(sinv.data(), n);
    GeneratorType col(n);
    if (n == 1) {
      col = internal::structured_scalar_part_impl<Scalar>::run(sinv);
    } else {
      auto&& fft = internal::structured_fft_engine<RealScalar>();
      ComplexVector ct(n);
      fft.inv(ct, sinv, n);
      col = internal::structured_scalar_part_impl<Scalar>::run(ct);
    }
    return Circulant(col, n > internal::structured_direct_threshold() ? sinv : ComplexVector(),
                     n > internal::structured_direct_threshold() ? computeProdSymbol(col) : ComplexVector());
  }

  /** \returns the determinant, i.e. the product of the eigenvalues (the symbol
   * entries). The product is accumulated in the balanced form \c m * 2^e (the
   * split fraction/exponent determinant convention of LINPACK's xGEDI [4]) --
   * every factor and the running product are renormalized to unit magnitude with
   * the power of two tracked separately -- so the partial products can neither
   * overflow nor underflow when the determinant itself is representable, whatever
   * the ordering of large and small eigenvalues. For a real operator the product
   * is real up to roundoff, and its real part is returned. */
  Scalar determinant() const {
    const ComplexVector s = symbol();
    Complex det(1);
    Index exponent = 0;
    for (Index k = 0; k < s.size(); ++k)
      det = internal::structured_balance(Complex(det * internal::structured_balance(s[k], exponent)), exponent);
    return internal::structured_scalar_part_impl<Scalar>::run_scalar(internal::structured_ldexp_clamped(det, exponent));
  }

  /** \returns the eigenvalues: eigenvalue \c k is \c symbol()[k], and its
   * (unit-norm) eigenvector is the Fourier vector \c f_k with
   * \f$ (f_k)_j = e^{2\pi i j k / n} / \sqrt{n} \f$, see \ref eigenvectors.
   * Every circulant matrix is diagonalized by this same Fourier basis. */
  ComplexVector eigenvalues() const { return symbol(); }

  /** \returns the unitary matrix of eigenvectors: column \c k is the Fourier
   * vector \c f_k matching \c eigenvalues()[k].
   * \note The eigenvector matrix is materialized as a dense \c n x \c n matrix;
   * unlike the other methods of this class this costs O(n^2) storage. */
  ComplexMatrix eigenvectors() const {
    const Index n = rows();
    ComplexMatrix F(n, n);
    for (Index k = 0; k < n; ++k) fourierColumn(F, k, k);
    return F;
  }

  /** \returns the singular values, sorted in decreasing order: the moduli of the
   * symbol entries. The ordering is shared with \ref matrixU and \ref matrixV, so
   * together they form the SVD \c *this = U * singularValues().asDiagonal() * V^H.
   * A real operator has a conjugate-symmetric symbol, so the two modes of a
   * conjugate pair carry exactly equal singular values. */
  RealVector singularValues() const {
    ComplexVector s;
    RealVector mods;
    const std::vector<Index> perm = svdOrdering(s, mods);
    RealVector sv(s.size());
    for (Index t = 0; t < s.size(); ++t) sv[t] = mods[perm[t]];
    return sv;
  }

  /** \returns the matrix of left singular vectors \c U: column \c t is the Fourier
   * vector of the t-th largest symbol entry, scaled by its phase (phase 1 for a
   * zero entry). Dense \c n x \c n, see the note in \ref eigenvectors. */
  ComplexMatrix matrixU() const {
    const Index n = rows();
    ComplexVector s;
    RealVector mods;
    const std::vector<Index> perm = svdOrdering(s, mods);
    ComplexMatrix U(n, n);
    for (Index t = 0; t < n; ++t) {
      fourierColumn(U, perm[t], t);
      const RealScalar a = mods[perm[t]];
      if (a > RealScalar(0)) U.col(t) *= s[perm[t]] / a;
    }
    return U;
  }

  /** \returns the matrix of right singular vectors \c V: column \c t is the
   * Fourier vector of the t-th largest symbol entry. Dense \c n x \c n, see the
   * note in \ref eigenvectors. */
  ComplexMatrix matrixV() const {
    const Index n = rows();
    ComplexVector s;
    RealVector mods;
    const std::vector<Index> perm = svdOrdering(s, mods);
    ComplexMatrix V(n, n);
    for (Index t = 0; t < n; ++t) fourierColumn(V, perm[t], t);
    return V;
  }

  /** \internal Computes \c dst += alpha * (*this) * rhs. \c ProductScalar is the
   * promoted scalar of the product (complex when a real operator is applied to a
   * complex right-hand side); the accumulation runs in the promoted type.
   *
   * Non-finite data takes the direct O(n^2) kernel: the transforms would smear a
   * single Inf/NaN into NaNs across the whole output, where the dense product
   * only propagates it through the dot products that touch it. A non-finite
   * generator or cached symbol (which can overflow even for a finite generator)
   * routes the whole product; a non-finite right-hand-side column is detected
   * inside the FFT loop -- in the same pass that derives its scaling exponent,
   * so finite data pays no extra scan -- and falls back per column. */
  template <typename Dest, typename Rhs, typename ProductScalar>
  void addProduct(Dest& dst, const Rhs& rhs, const ProductScalar& alpha) const {
    const Index n = rows();
    eigen_assert(rhs.rows() == n && "invalid product: dimensions do not match");
    if (n <= internal::structured_direct_threshold() || !m_fftUsable) {
      directProduct(dst, rhs, alpha);
      return;
    }
    // Products use the padded embedding symbol when the operator size is not
    // 5-smooth (see computeProdSymbol()); the exact-size transform of such a
    // size runs through kissfft's quadratic generic butterfly.
    const ComplexVector& s = m_prodSymbol.size() > 0 ? m_prodSymbol : m_symbol;
    internal::structured_fft_apply(dst, s, n, rhs, alpha, [&](Index k) { directProductColumn(dst, rhs, k, alpha); });
  }

 private:
  /** \internal Builds an operator from a generator and already-known symbols
   * (empty for small operators; \a prodSymbol also empty for 5-smooth sizes,
   * which need no padded product embedding), skipping the FFTs of the public
   * constructor. Used by transpose(), conjugate(), adjoint() and inverse(),
   * whose symbols are cheap transformations of the existing ones. */
  Circulant(const GeneratorType& col, const ComplexVector& symbol, const ComplexVector& prodSymbol)
      : m_col(col), m_symbol(symbol), m_prodSymbol(prodSymbol) {
    m_fftUsable = computeFftUsable();
  }

  /** \internal Whether products may take the FFT path: the generator and the
   * cached symbols must be finite. The symbols accumulate up to n (respectively
   * p) addends, so they can overflow to Inf even for a finite generator; such
   * operators fall back to the direct kernel, which stays exact. */
  bool computeFftUsable() const {
    return m_col.allFinite() && (m_symbol.size() == 0 || m_symbol.allFinite()) &&
           (m_prodSymbol.size() == 0 || m_prodSymbol.allFinite());
  }

  /** \internal \returns the symbol products should use for a generator \a col of
   * size n: empty when n is 5-smooth (the exact-size transform is already fast),
   * otherwise the DFT of the circulant embedding of size
   * \c p = fft_next_good_size(2n - 1) -- the length-n cyclic convolution
   * evaluated as a padded linear convolution, exactly the Toeplitz embedding of
   * the circulant. kissfft falls back to a quadratic generic butterfly for prime
   * factors other than 2, 3 and 5, so the exact-size transform of e.g. a prime n
   * costs orders of magnitude more than the padded one. The spectral operations
   * (eigenvalues, solve, rank, determinant, SVD) keep the exact-size symbol,
   * whose entries are the eigenvalues. */
  static ComplexVector computeProdSymbol(const GeneratorType& col) {
    const Index n = col.size();
    if (internal::fft_next_good_size(n) == n) return ComplexVector();
    const Index p = internal::fft_next_good_size(2 * n - 1);
    ComplexVector embedding = ComplexVector::Zero(p);
    embedding.head(n) = col.template cast<Complex>();
    embedding.tail(n - 1) = col.tail(n - 1).template cast<Complex>();
    ComplexVector symbol(p);
    auto&& fft = internal::structured_fft_engine<RealScalar>();
    fft.fwd(symbol, embedding, p);
    return symbol;
  }

  /** \internal Direct O(n^2) kernel for column \a k of the right-hand side:
   * computes \c dst.col(k) += alpha * (*this) * rhs.col(k) without transforms.
   * Serves operators below the FFT threshold and any column involving
   * non-finite data, whose entrywise IEEE semantics the transforms cannot
   * preserve. */
  template <typename Dest, typename Rhs, typename ProductScalar>
  void directProductColumn(Dest& dst, const Rhs& rhs, Index k, const ProductScalar& alpha) const {
    const Index n = rows();
    // A unit alpha must not multiply: even the identity complex scalar (1,0)
    // pollutes an (Inf,0) value with NaN through the 0*Inf cross term.
    const bool unitAlpha = alpha == ProductScalar(1);
    if (n <= internal::structured_scalar_threshold()) {
      // Tiny sizes: a plain scalar loop beats the segment-based path below, whose
      // per-segment setup dominates when segments hold only a few entries.
      for (Index i = 0; i < n; ++i) {
        ProductScalar acc(0);
        for (Index j = 0; j < n; ++j) acc += coeff(i, j) * rhs.coeff(j, k);
        dst.coeffRef(i, k) += unitAlpha ? acc : ProductScalar(alpha * acc);
      }
      return;
    }

    // Column j is c rotated downward by j; split it into two contiguous segments.
    auto dstCol = dst.col(k);
    for (Index j = 0; j < n; ++j) {
      const ProductScalar xj = unitAlpha ? ProductScalar(rhs.coeff(j, k)) : ProductScalar(alpha * rhs.coeff(j, k));
      dstCol.head(j) += xj * m_col.tail(j);
      dstCol.tail(n - j) += xj * m_col.head(n - j);
    }
  }

  /** \internal Direct O(n^2) product kernel over every column, see
   * directProductColumn(). */
  template <typename Dest, typename Rhs, typename ProductScalar>
  void directProduct(Dest& dst, const Rhs& rhs, const ProductScalar& alpha) const {
    for (Index k = 0; k < rhs.cols(); ++k) directProductColumn(dst, rhs, k, alpha);
  }

  /** \internal Computes the moduli of the symbol entries and the matching
   * rank/pseudo-inversion threshold, both evaluated in an exactly rescaled frame:
   * the entries are pre-scaled by a power of two chosen so no modulus can
   * overflow. A finite complex entry near the overflow threshold has a
   * non-representable modulus, which would otherwise turn the threshold into
   * infinity and misclassify every other entry (rank under-reported, solve()
   * zeroing valid Fourier modes). The rescaling is exact, so comparing scaled
   * moduli against the scaled threshold is equivalent to the unscaled
   * comparison. The threshold keeps the n * epsilon * max_k|s[k]| convention of
   * [3], chapter 5.4, and the smallest-normal clamp -- carried into the scaled
   * frame, where its underflowing to zero for a huge frame is correct: no entry
   * of such a symbol can sit below the smallest normal number. Entries at or
   * above the threshold, in particular a smallest-normal entry of a moderate
   * symbol, are inverted (their reciprocals are finite). */
  static void scaledModuli(const ComplexVector& s, RealVector& mods, RealScalar& tol, int* frameExp = nullptr) {
    const int e = numext::maxi(internal::structured_exponent_bound(s), 0);
    if (frameExp) *frameExp = e;
    // Two exact factors, as in structured_fft_apply(): a single 2^-e is itself
    // subnormal once the frame exceeds the exponent range, and reads as zero under
    // flush-to-zero -- collapsing every scaled modulus, and the threshold with
    // them. Neither factor exceeds one, so no intermediate underflows on its own.
    const RealScalar down1 = numext::ldexp(RealScalar(1), -(e / 2)), down2 = numext::ldexp(RealScalar(1), -(e - e / 2));
    mods = ((s * down1) * down2).cwiseAbs();
    tol = numext::maxi(RealScalar(s.size()) * NumTraits<RealScalar>::epsilon() * mods.maxCoeff(),
                       ((std::numeric_limits<RealScalar>::min)() * down1) * down2);
  }

  /** \internal Computes the symbol \a s and the moduli \a mods that order the
   * SVD, and \returns the permutation sorting those moduli in decreasing order --
   * the single ordering singularValues(), matrixU() and matrixV() agree on.
   *
   * For a real generator the symbol is conjugate-symmetric, so modes \c k and
   * \c n-k have equal moduli; taken through the complex transform they agree only
   * to roundoff. Ranking such a pair by the computed moduli leaves its order
   * decided by those last bits, and the three accessors each derive the ordering
   * from their own symbol computation, which nothing requires to agree down to
   * the last bit: one of them ranking a pair the other way pairs a left singular
   * vector with the wrong right one and breaks the reconstruction by an O(1)
   * amount. The moduli of a conjugate pair are therefore tied exactly, which the
   * stable sort resolves by index in every caller. Sharing one value between the
   * pair costs nothing: it cancels out of \c U * S, where matrixU() divides the
   * mode's phase by it and singularValues() returns it.
   */
  std::vector<Index> svdOrdering(ComplexVector& s, RealVector& mods) const {
    const Index n = rows();
    s = symbol();
    mods = s.cwiseAbs();
    if (!NumTraits<Scalar>::IsComplex)
      for (Index k = 1; 2 * k < n; ++k) mods[n - k] = mods[k];
    return internal::structured_svd_permutation(mods);
  }

  /** \internal Writes the unit-norm Fourier eigenvector \c f_k into column
   * \a dstCol of \a F: (f_k)_j = exp(2 pi i j k / n) / sqrt(n). The index product
   * j*k is accumulated incrementally modulo n, so the argument passed to polar()
   * stays O(2 pi) -- keeping full accuracy for any n -- and no Index overflow can
   * occur. */
  void fourierColumn(ComplexMatrix& F, Index k, Index dstCol) const {
    const Index n = rows();
    const RealScalar scale = RealScalar(1) / numext::sqrt(RealScalar(n));
    Index jk = 0;  // j * k mod n
    for (Index j = 0; j < n; ++j) {
      F(j, dstCol) = std::polar(scale, RealScalar(2 * EIGEN_PI) * RealScalar(jk) / RealScalar(n));
      jk += k;
      if (jk >= n) jk -= n;
    }
  }

  /** \internal \returns the DFT of the generating column. */
  ComplexVector computeSymbol() const {
    const Index n = m_col.size();
    const ComplexVector cc = m_col.template cast<Complex>();
    if (n == 1) return cc;  // the DFT of a single sample is the identity
    ComplexVector symbol(n);
    auto&& fft = internal::structured_fft_engine<RealScalar>();
    fft.fwd(symbol, cc, n);
    return symbol;
  }

  GeneratorType m_col;
  ComplexVector m_symbol;
  // The padded embedding symbol products use when the size is not 5-smooth;
  // empty otherwise. See computeProdSymbol().
  ComplexVector m_prodSymbol;
  bool m_fftUsable;
};

/** \ingroup StructuredMatrices_Module
 * \returns a \ref Circulant operator with first column \a col. The compile-time
 * size of the operator is deduced from \a col. */
template <typename Derived>
Circulant<typename Derived::Scalar, Derived::SizeAtCompileTime> makeCirculant(const MatrixBase<Derived>& col) {
  return Circulant<typename Derived::Scalar, Derived::SizeAtCompileTime>(col);
}

namespace internal {

// StructuredShape avoids ambiguity with the stock dense product specializations.
template <typename Scalar_, int Size_, typename Rhs, int ProductTag>
struct generic_product_impl<Circulant<Scalar_, Size_>, Rhs, StructuredShape, DenseShape, ProductTag>
    : structured_product_impl<Circulant<Scalar_, Size_>, Rhs> {};

}  // namespace internal

}  // namespace Eigen

#endif  // EIGEN_STRUCTURED_CIRCULANT_H
