// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

// References:
//  [1] P. J. Davis, "Circulant Matrices", Wiley, 1979. Diagonalization of
//      circulant and block circulant matrices by the DFT; the closed-form
//      eigenstructure used by eigenvalues()/eigenvectors(), and the SVD and
//      pseudo-inverse below, follow from it by taking moduli/phases of the
//      eigenvalues.
//  [2] R. H. Chan and X.-Q. Jin, "An Introduction to Iterative Toeplitz
//      Solvers", SIAM, 2007. BCCB matrices are diagonalized by the 2-D DFT
//      F_{n1} (x) F_{n2}; the FFT-based products and solves below, and the use
//      of BCCB operators as preconditioners for two-level Toeplitz (BTTB)
//      systems, follow this reference.
//  [3] G. H. Golub and C. F. Van Loan, "Matrix Computations", 4th ed., Johns
//      Hopkins University Press, 2013, chapter 5.4 (numerical rank conventions).
//  [4] J. J. Dongarra, J. R. Bunch, C. B. Moler and G. W. Stewart, "LINPACK
//      Users' Guide", SIAM, 1979. determinant()'s balanced accumulation follows
//      the convention of its xGEDI routines, which return determinants as a
//      (fraction, exponent) pair to avoid spurious overflow/underflow.
//  [5] P. H. Sterbenz, "Floating-Point Computation", Prentice-Hall, 1974.
//      Scaling by a power of two is exact, the property the balanced
//      accumulation, the rescaled rank threshold and the scaled 2-D FFT
//      products rely on.

#ifndef EIGEN_STRUCTURED_BCCB_H
#define EIGEN_STRUCTURED_BCCB_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

template <typename Scalar_, int BlockSize_ = Dynamic, int NumBlocks_ = Dynamic>
class Bccb;

namespace internal {

/** \internal Compile-time product of the two circulant levels, Dynamic-aware. */
constexpr int bccb_dim(int a, int b) { return (a == Dynamic || b == Dynamic) ? Dynamic : a * b; }

template <typename Scalar_, int BlockSize_, int NumBlocks_>
struct traits<Bccb<Scalar_, BlockSize_, NumBlocks_>> {
  using Scalar = Scalar_;
  using StorageKind = Dense;
  using XprKind = MatrixXpr;
  using StorageIndex = int;
  static constexpr int RowsAtCompileTime = bccb_dim(BlockSize_, NumBlocks_);
  static constexpr int ColsAtCompileTime = RowsAtCompileTime;
  static constexpr int MaxRowsAtCompileTime = RowsAtCompileTime;
  static constexpr int MaxColsAtCompileTime = RowsAtCompileTime;
  // Deliberately no NestByRefBit: transpose(), conjugate() and adjoint() return
  // owning temporaries, so Product must nest the operator by value for a
  // delayed-evaluated product expression to keep its left factor alive. The copy
  // is O(N), negligible against the O(N log N) product evaluation.
  static constexpr unsigned int Flags = 0;
};

template <typename Scalar_, int BlockSize_, int NumBlocks_>
struct evaluator_traits<Bccb<Scalar_, BlockSize_, NumBlocks_>> {
  using Kind = IndexBased;
  using Shape = StructuredShape;
};

}  // namespace internal

/** \ingroup StructuredMatrices_Module
 * \class Bccb
 * \brief A block circulant matrix with circulant blocks (BCCB), the matrix of a
 * two-dimensional circular convolution, represented by its n2 x n1 generating
 * array.
 *
 * A BCCB matrix is the \c N x \c N matrix, \c N = n1*n2, that is circulant at two
 * levels: it is an n1 x n1 block circulant whose n2 x n2 blocks are themselves
 * circulant. With the generating array \c G (column \c k holds the first column
 * of the k-th block), entry \c (i,j) with \c i = b1*n2 + i2, \c j = c1*n2 + j2
 * equals
 * \f[ C_{i,j}=G_{(i_2-j_2)\bmod n_2,\,(b_1-c_1)\bmod n_1}. \f]
 * On a column-major reshaped vector, \f$C\,\operatorname{vec}(X)
 * =\operatorname{vec}(G\mathbin{\circledast}X)\f$, where \f$\circledast\f$
 * denotes 2-D circular convolution.
 *
 * BCCB matrices are diagonalized by the 2-D discrete Fourier transform
 * \f$ F_{n_1} \otimes F_{n_2} \f$ ([1], [2]): the operator's \em symbol -- the 2-D
 * DFT of \c G -- holds the eigenvalues. Products reuse that symbol or, for an
 * awkward transform size, an equivalent cached padded-embedding symbol. This
 * yields O(N log N) products (\c operator*), an O(N log N) direct
 * (pseudo-inverse) solve (\ref solve), and closed-form factorizations: the
 * eigendecomposition (\ref eigenvalues, \ref eigenvectors) and the SVD
 * (\ref singularValues, \ref matrixU, \ref matrixV) in the 2-D Fourier basis,
 * plus \ref rank, \ref inverse and \ref determinant. The class is closed under
 * \ref transpose, \ref conjugate and \ref adjoint, which reuse the cached symbols.
 * BCCB operators are the workhorse of image deblurring with periodic boundary
 * conditions, and the natural preconditioners for two-level Toeplitz (BTTB)
 * systems [2].
 *
 * The operator stores its own copy of the generating array and derives from
 * \c EigenBase. Because \c operator* returns an Eigen product expression, a
 * \c Bccb also drops into the matrix-free iterative solvers, and it can be
 * assigned to a dense matrix when an explicit representation is needed. As with
 * any matrix-free operator, the iterative solvers must be instantiated with
 * \c IdentityPreconditioner (e.g.
 * \c ConjugateGradient<Bccb<double>,Lower|Upper,IdentityPreconditioner>):
 * the default preconditioners read individual coefficients through \c col() or
 * \c InnerIterator, which the structured operators do not expose.
 *
 * Spectral operations use FFTs of the exact sizes \c n1 and \c n2. Products use
 * those sizes when both are 5-smooth; otherwise an equivalent per-axis
 * circulant embedding pads each awkward dimension to a 5-smooth size, avoiding
 * the default kissfft backend's slow generic butterfly for large prime factors.
 *
 * \tparam Scalar_ the scalar type, real or complex.
 * \tparam BlockSize_ the circulant block dimension \c n2 at compile time, or
 *         \c Dynamic (the default).
 * \tparam NumBlocks_ the number of blocks \c n1 at compile time, or \c Dynamic
 *         (the default).
 *
 * \sa class Circulant, makeBccb()
 */
template <typename Scalar_, int BlockSize_, int NumBlocks_>
class Bccb : public EigenBase<Bccb<Scalar_, BlockSize_, NumBlocks_>> {
  enum DenseAssignment { SetAssignment, AddAssignment, SubAssignment };

 public:
  using Scalar = Scalar_;
  using RealScalar = typename NumTraits<Scalar>::Real;
  using StorageIndex = int;
  using Complex = std::complex<RealScalar>;
  // The two-level structure keys on the *column-major* flattening of the n2 x n1
  // generating array and symbol (entry (k2, k1) is flat index k1*n2 + k2), so
  // every internal 2-D type is pinned to ColMajor explicitly: the semantics must
  // not change under EIGEN_DEFAULT_TO_ROW_MAJOR. The single-row special case
  // only satisfies Eigen's rule that 1 x n matrices be row-major; with one row
  // the two orders coincide.
  using GeneratorType =
      Matrix<Scalar, BlockSize_, NumBlocks_, (BlockSize_ == 1 && NumBlocks_ != 1) ? int(RowMajor) : int(ColMajor)>;
  using ComplexArray = Matrix<Complex, Dynamic, Dynamic, ColMajor>;
  using ComplexVector = Matrix<Complex, Dynamic, 1>;
  using RealVector = Matrix<RealScalar, Dynamic, 1>;
  using RealArray = Matrix<RealScalar, Dynamic, Dynamic, ColMajor>;
  using ComplexMatrix = Matrix<Complex, Dynamic, Dynamic, ColMajor>;

  static constexpr int RowsAtCompileTime = internal::bccb_dim(BlockSize_, NumBlocks_);
  static constexpr int ColsAtCompileTime = RowsAtCompileTime;
  static constexpr int MaxRowsAtCompileTime = RowsAtCompileTime;
  static constexpr int MaxColsAtCompileTime = RowsAtCompileTime;
  static constexpr int SizeAtCompileTime = internal::size_at_compile_time(RowsAtCompileTime, ColsAtCompileTime);
  static constexpr int MaxSizeAtCompileTime = SizeAtCompileTime;
  static constexpr bool IsRowMajor = false;
  // Deliberately no IsVectorAtCompileTime: Ref<const Bccb>'s default StrideType
  // argument reads it, so its absence makes internal::is_ref_compatible SFINAE to
  // false and keeps the iterative solvers on their matrix-free path.

  /** Builds a BCCB matrix from its generating array \a generator: column \c k is
   * the first column of the k-th circulant block.
   *
   * When the matrix is large enough for products to take the FFT path, the 2-D
   * DFT of the array -- the eigenvalues of the matrix -- and any padded product
   * symbol are computed here. Subsequent products and solves reuse the applicable
   * cached transform. */
  template <typename Derived>
  explicit Bccb(const MatrixBase<Derived>& generator) : m_g(generator) {
    eigen_assert(m_g.rows() > 0 && m_g.cols() > 0 && "Bccb generator must be non-empty");
    if (m_g.size() > internal::structured_direct_threshold()) {
      m_symbol = computeSymbol();
      m_prodSymbol = computeProdSymbol(m_g);
    }
    m_fftUsable = computeFftUsable();
  }

  EIGEN_DEVICE_FUNC Index rows() const { return m_g.size(); }
  EIGEN_DEVICE_FUNC Index cols() const { return m_g.size(); }

  /** \returns the circulant block dimension \c n2. */
  Index blockSize() const { return m_g.rows(); }
  /** \returns the number of blocks \c n1 in each block row. */
  Index numBlocks() const { return m_g.cols(); }
  /** \returns the generating array. */
  const GeneratorType& generator() const { return m_g; }

  /** \returns the symbol of the operator: the 2-D DFT of the generating array,
   * an n2 x n1 complex array whose entries are the eigenvalues of the matrix
   * (see \ref eigenvalues for the ordering). Cached when the operator is large
   * enough for products to take the FFT path, computed on the fly for small
   * operators. */
  ComplexArray symbol() const { return m_symbol.size() > 0 ? m_symbol : computeSymbol(); }

  /** \returns the coefficient at row \a row and column \a col. */
  Scalar coeff(Index row, Index col) const {
    const Index n2 = blockSize();
    Index k2 = row % n2 - col % n2;
    if (k2 < 0) k2 += n2;
    Index k1 = row / n2 - col / n2;
    if (k1 < 0) k1 += numBlocks();
    return m_g.coeff(k2, k1);
  }

  /** \returns the transpose of \c *this, itself a \c Bccb operator: the one
   * generated by the array index-reversed in both dimensions. The cached
   * symbols, when present, are reused -- the symbols of the transpose are their
   * two-dimensional index reversals (embedding a generator commutes with
   * index-reversing it, per axis, so the padded product symbol follows the same
   * rule at its own grid) -- so no FFT is recomputed. */
  Bccb transpose() const { return Bccb(reverse2(m_g), reverse2(m_symbol), reverse2(m_prodSymbol)); }

  /** \returns the complex conjugate of \c *this, itself a \c Bccb operator. The
   * cached symbols, when present, are reused: the symbols of the conjugate are
   * the conjugated two-dimensional index reversals of the symbols. */
  Bccb conjugate() const {
    return Bccb(m_g.conjugate(), reverse2(m_symbol).conjugate(), reverse2(m_prodSymbol).conjugate());
  }

  /** \returns the adjoint of \c *this, itself a \c Bccb operator. The cached
   * symbols, when present, are reused: the symbols of the adjoint are the
   * elementwise conjugates of the symbols (the eigenvalues conjugate while the
   * 2-D Fourier eigenbasis stays fixed). */
  Bccb adjoint() const { return Bccb(reverse2(m_g).conjugate(), m_symbol.conjugate(), m_prodSymbol.conjugate()); }

  /** \returns the minimum-norm least-squares solution of \c (*this) * x = b,
   * computed directly in the 2-D Fourier domain. Symbol entries whose modulus
   * reaches the rank threshold (see \ref rank) are inverted; the remaining ones
   * are treated as exact zeros, so the result is the pseudo-inverse applied to
   * \a b. For a non-singular operator this is the exact solution. Supports
   * multiple right-hand sides. */
  template <typename Rhs>
  Matrix<Scalar, RowsAtCompileTime, Rhs::ColsAtCompileTime> solve(const MatrixBase<Rhs>& b) const {
    EIGEN_STATIC_ASSERT(RowsAtCompileTime == Dynamic || Rhs::RowsAtCompileTime == Dynamic ||
                            int(RowsAtCompileTime) == int(Rhs::RowsAtCompileTime),
                        YOU_MIXED_MATRICES_OF_DIFFERENT_SIZES)
    const Index N = rows();
    eigen_assert(b.rows() == N && "right-hand side has the wrong number of rows");
    const ComplexArray s = symbol();
    RealArray mods;
    RealScalar tol;
    scaledModuli(s, mods, tol);
    ComplexArray sinv(s.rows(), s.cols());
    // Strictly-below-threshold entries are zeroed, matching SVDBase::rank(), so a
    // smallest-normal 1x1 operator stays invertible. A comparison with NaN is
    // false, so NaN symbol entries stay in the inverted set and propagate to the
    // output instead of being silently zeroed.
    // The packet path sanitizes discarded entries before division; exceptional
    // packets retain the scalar branch and balanced reciprocal.
    internal::structured_symbol_reciprocal(s.data(), mods.data(), tol, sinv.data(), s.size());
    Matrix<Scalar, RowsAtCompileTime, Rhs::ColsAtCompileTime> x(N, b.cols());
    if (!b.allFinite()) {
      // A non-finite right-hand side cannot go through the transforms (see
      // addProduct): apply the pseudo-inverse -- itself a BCCB operator,
      // generated by the 2-D inverse DFT of the thresholded reciprocal symbol --
      // through the direct kernel so Inf/NaN propagate entrywise.
      const GeneratorType pg = generatorFromSymbol(sinv);
      x.setZero();
      Bccb(pg, ComplexArray(), ComplexArray()).directProduct(x, b.derived(), Scalar(1));
      return x;
    }
    // The right-hand side is finite here (checked above), so no per-column
    // direct kernel can be needed.
    applySymbol(x, sinv, b.derived(), Scalar(1), /*accumulate=*/false,
                [](Index) { eigen_assert(false && "non-finite column requires a direct kernel"); });
    return x;
  }

  /** \returns the numerical rank: the number of symbol entries whose modulus is
   * no smaller than the threshold \c N * epsilon * max|symbol|, both evaluated
   * in an exactly rescaled frame so the moduli cannot overflow (see
   * scaledModuli()) and clamped from below like \c SVDBase::rank(). This is the
   * same threshold \ref solve uses to decide which Fourier components to invert,
   * and the comparison is strict like SVDBase's, so an entry sitting exactly on
   * the threshold still counts as non-zero. */
  Index rank() const {
    const ComplexArray s = symbol();
    RealArray mods;
    RealScalar tol;
    scaledModuli(s, mods, tol);
    return (!(mods.array() < tol)).count();  // negated so NaN entries count as non-zero
  }

  /** \returns the inverse of \c *this, itself a \c Bccb operator: the one
   * generated by the 2-D inverse DFT of the entrywise-inverted symbol.
   * \warning The operator must be non-singular; use \ref solve for a
   * pseudo-inverse solve of a rank-deficient operator. */
  Bccb inverse() const {
    ComplexArray sinv = symbol().cwiseInverse();
    const GeneratorType ginv = generatorFromSymbol(sinv);
    const bool cache = rows() > internal::structured_direct_threshold();
    return Bccb(ginv, cache ? sinv : ComplexArray(), cache ? computeProdSymbol(ginv) : ComplexArray());
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
    const ComplexArray s = symbol();
    Complex det(1);
    Index exponent = 0;
    for (Index k1 = 0; k1 < s.cols(); ++k1)
      for (Index k2 = 0; k2 < s.rows(); ++k2)
        det = internal::structured_balance(Complex(det * internal::structured_balance(s(k2, k1), exponent)), exponent);
    return toScalar(internal::structured_ldexp_clamped(det, exponent), std::is_same<RealScalar, Scalar>());
  }

  /** \returns the eigenvalues as the column-major flattening of the symbol:
   * eigenvalue \c f1*n2 + f2 is \c symbol()(f2, f1), and its (unit-norm)
   * eigenvector is the Kronecker product of the 1-D Fourier vectors of
   * frequencies \c f1 and \c f2, i.e. column \c f1*n2 + f2 of \ref eigenvectors.
   * Every BCCB matrix is diagonalized by this same 2-D Fourier basis. */
  ComplexVector eigenvalues() const { return symbol().reshaped(); }

  /** \returns the unitary matrix of eigenvectors: column \c f1*n2 + f2 is the
   * 2-D Fourier vector matching \c eigenvalues()[f1*n2 + f2].
   * \note The eigenvector matrix is materialized as a dense \c N x \c N matrix;
   * unlike the other methods of this class this costs O(N^2) storage. */
  ComplexMatrix eigenvectors() const {
    const Index n2 = blockSize(), n1 = numBlocks(), N = rows();
    ComplexMatrix F(N, N);
    for (Index f1 = 0; f1 < n1; ++f1)
      for (Index f2 = 0; f2 < n2; ++f2) fourierColumn(F, f1, f2, f1 * n2 + f2);
    return F;
  }

  /** \returns the singular values, sorted in decreasing order: the moduli of the
   * symbol entries. The ordering is shared with \ref matrixU and \ref matrixV,
   * so together they form the SVD \c *this = U * singularValues().asDiagonal() * V^H. */
  RealVector singularValues() const {
    const ComplexVector s = eigenvalues();
    const RealVector mods = s.cwiseAbs();
    const std::vector<Index> perm = internal::structured_svd_permutation(mods);
    RealVector sv(s.size());
    for (Index t = 0; t < s.size(); ++t) sv[t] = mods[perm[t]];
    return sv;
  }

  /** \returns the matrix of left singular vectors \c U: column \c t is the 2-D
   * Fourier vector of the t-th largest symbol entry, scaled by its phase (phase 1
   * for a zero entry). Dense \c N x \c N, see the note in \ref eigenvectors. */
  ComplexMatrix matrixU() const {
    const Index n2 = blockSize(), N = rows();
    const ComplexVector s = eigenvalues();
    const RealVector mods = s.cwiseAbs();
    const std::vector<Index> perm = internal::structured_svd_permutation(mods);
    ComplexMatrix U(N, N);
    for (Index t = 0; t < N; ++t) {
      fourierColumn(U, perm[t] / n2, perm[t] % n2, t);
      const RealScalar a = mods[perm[t]];
      if (a > RealScalar(0)) U.col(t) *= s[perm[t]] / a;
    }
    return U;
  }

  /** \returns the matrix of right singular vectors \c V: column \c t is the 2-D
   * Fourier vector of the t-th largest symbol entry. Dense \c N x \c N, see the
   * note in \ref eigenvectors. */
  ComplexMatrix matrixV() const {
    const Index n2 = blockSize(), N = rows();
    const ComplexVector s = eigenvalues();
    const std::vector<Index> perm = internal::structured_svd_permutation(RealVector(s.cwiseAbs()));
    ComplexMatrix V(N, N);
    for (Index t = 0; t < N; ++t) fourierColumn(V, perm[t] / n2, perm[t] % n2, t);
    return V;
  }

  /** \internal Writes the dense representation into \a dst: the (b1,c1) block is
   * the generator column (b1-c1) mod n1 rotated downwards by the within-block
   * column index, so only contiguous segment copies are involved. Invoked
   * through \c dense = bccb; */
  template <typename Dest>
  void evalTo(Dest& dst) const {
    assignTo<SetAssignment>(dst);
  }

  /** \internal Computes \c dst += (*this), see evalTo(). */
  template <typename Dest>
  void addTo(Dest& dst) const {
    assignTo<AddAssignment>(dst);
  }

  /** \internal Computes \c dst -= (*this), see evalTo(). */
  template <typename Dest>
  void subTo(Dest& dst) const {
    assignTo<SubAssignment>(dst);
  }

  /** \returns the product expression \c (*this) * \a x, evaluated through a fast
   * 2-D-FFT-based matrix-vector product. The expression carries the default
   * product tag, so assigning it behaves like any dense product: a temporary
   * resolves aliasing between the destination and \a x, and \c .noalias() skips
   * it. */
  template <typename Rhs>
  Product<Bccb, Rhs> operator*(const MatrixBase<Rhs>& x) const {
    EIGEN_STATIC_ASSERT(ColsAtCompileTime == Dynamic || Rhs::RowsAtCompileTime == Dynamic ||
                            int(ColsAtCompileTime) == int(Rhs::RowsAtCompileTime),
                        INVALID_MATRIX_PRODUCT)
    eigen_assert(x.rows() == cols() && "invalid product: dimensions do not match");
    return Product<Bccb, Rhs>(*this, x.derived());
  }

  /** \internal Computes \c dst += alpha * (*this) * rhs. \c ProductScalar is the
   * promoted scalar of the product (complex when a real operator is applied to a
   * complex right-hand side); the accumulation runs in the promoted type.
   *
   * Non-finite data takes the direct O(N^2) kernel: the transforms would smear a
   * single Inf/NaN into NaNs across the whole output, where the dense product
   * only propagates it through the dot products that touch it. A non-finite
   * generating array or cached symbol (which can overflow even for a finite
   * generator: the 2-D DFT accumulates up to N addends) routes the whole
   * product; a non-finite right-hand-side column is detected inside the FFT
   * loop -- in the same pass that derives its scaling exponent, so finite data
   * pays no extra scan -- and falls back per column. */
  template <typename Dest, typename Rhs, typename ProductScalar>
  void addProduct(Dest& dst, const Rhs& rhs, const ProductScalar& alpha) const {
    const Index N = rows();
    eigen_assert(rhs.rows() == N && "invalid product: dimensions do not match");
    if (N <= internal::structured_direct_threshold() || !m_fftUsable) {
      directProduct(dst, rhs, alpha);
      return;
    }
    // Products use the padded embedding symbol when a block dimension is not
    // 5-smooth (see computeProdSymbol()); the exact-size transform of such a
    // dimension runs through kissfft's quadratic generic butterfly.
    const ComplexArray& s = m_prodSymbol.size() > 0 ? m_prodSymbol : m_symbol;
    applySymbol(dst, s, rhs, alpha, /*accumulate=*/true, [&](Index k) { directProductColumn(dst, rhs, k, alpha); });
  }

 private:
  /** \internal Materializes the dense BCCB representation with the selected
   * assignment mode; shared by evalTo(), addTo() and subTo(). */
  template <DenseAssignment Assignment, typename Dest>
  void assignTo(Dest& dst) const {
    const Index n2 = blockSize(), n1 = numBlocks();
    for (Index c1 = 0; c1 < n1; ++c1)
      for (Index j2 = 0; j2 < n2; ++j2) {
        auto col = dst.col(c1 * n2 + j2);
        for (Index b1 = 0; b1 < n1; ++b1) {
          Index k1 = b1 - c1;
          if (k1 < 0) k1 += n1;
          EIGEN_IF_CONSTEXPR (Assignment == SetAssignment) {
            col.segment(b1 * n2, j2) = m_g.col(k1).tail(j2);
            col.segment(b1 * n2 + j2, n2 - j2) = m_g.col(k1).head(n2 - j2);
          } else EIGEN_IF_CONSTEXPR (Assignment == AddAssignment) {
            col.segment(b1 * n2, j2) += m_g.col(k1).tail(j2);
            col.segment(b1 * n2 + j2, n2 - j2) += m_g.col(k1).head(n2 - j2);
          } else {
            col.segment(b1 * n2, j2) -= m_g.col(k1).tail(j2);
            col.segment(b1 * n2 + j2, n2 - j2) -= m_g.col(k1).head(n2 - j2);
          }
        }
      }
  }

  /** \internal Builds an operator from a generating array and already-known
   * symbols (empty for small operators; \a prodSymbol also empty when both block
   * dimensions are 5-smooth, which needs no padded product embedding), skipping
   * the FFTs of the public constructor. Used by transpose(), conjugate(),
   * adjoint() and inverse(), whose symbols are cheap transformations of the
   * existing ones. */
  Bccb(const GeneratorType& g, const ComplexArray& symbol, const ComplexArray& prodSymbol)
      : m_g(g), m_symbol(symbol), m_prodSymbol(prodSymbol) {
    m_fftUsable = computeFftUsable();
  }

  /** \internal Whether products may take the FFT path: the generating array and
   * the cached symbols must be finite. The symbols accumulate up to N
   * (respectively p2*p1) addends, so they can overflow to Inf even for a finite
   * generator; such operators fall back to the direct kernel, which stays
   * exact. */
  bool computeFftUsable() const {
    return m_g.allFinite() && (m_symbol.size() == 0 || m_symbol.allFinite()) &&
           (m_prodSymbol.size() == 0 || m_prodSymbol.allFinite());
  }

  /** \internal \returns the symbol products should use for a generating array
   * \a g: empty when both block dimensions are 5-smooth (the exact-size
   * transforms are already fast), otherwise the 2-D DFT of the per-axis
   * circulant embedding of \a g. kissfft falls back to a quadratic generic
   * butterfly for prime factors other than 2, 3 and 5, and the row-column
   * algorithm pays that cost in every transform of the awkward axis, so each
   * such axis is padded to \c p_i = fft_next_good_size(2*n_i - 1) and the cyclic
   * convolution along it is evaluated as a padded linear convolution -- exactly
   * the Toeplitz embedding of a circulant, applied per dimension. Along a
   * padded axis the generator is laid out as [g; 0...0; g(1..)] (the wrapped
   * band g(1..) sits at the top indices), so that for reachable offsets
   * \c d = i - j in [-(n-1), n-1] the embedded array satisfies
   * \c Ge(d mod p) = g(d mod n); offsets outside that range only touch the zero
   * band. The two axes decouple, so the identity holds per entry of the 2-D
   * grid and the leading (n2, n1) block of the padded cyclic convolution equals
   * the exact one. A 5-smooth axis is kept at its exact size with no embedding.
   * The spectral operations (eigenvalues, solve, rank, determinant, SVD) keep
   * the exact-size symbol, whose entries are the eigenvalues. */
  ComplexArray computeProdSymbol(const GeneratorType& g) const {
    const Index n2 = g.rows(), n1 = g.cols();
    const Index p2 = internal::fft_next_good_size(n2) == n2 ? n2 : internal::fft_next_good_size(2 * n2 - 1);
    const Index p1 = internal::fft_next_good_size(n1) == n1 ? n1 : internal::fft_next_good_size(2 * n1 - 1);
    if (p2 == n2 && p1 == n1) return ComplexArray();
    ComplexArray Ge = ComplexArray::Zero(p2, p1);
    Ge.topLeftCorner(n2, n1) = g.template cast<Complex>();
    if (p2 != n2) Ge.bottomLeftCorner(n2 - 1, n1) = g.bottomRows(n2 - 1).template cast<Complex>();
    if (p1 != n1) Ge.topRightCorner(n2, n1 - 1) = g.rightCols(n1 - 1).template cast<Complex>();
    if (p2 != n2 && p1 != n1)
      Ge.bottomRightCorner(n2 - 1, n1 - 1) = g.bottomRightCorner(n2 - 1, n1 - 1).template cast<Complex>();
    fft2(Ge);
    return Ge;
  }

  /** \internal Direct O(N^2) kernel for column \a k of the right-hand side:
   * computes \c dst.col(k) += alpha * (*this) * rhs.col(k) without transforms,
   * as a plain scalar loop (the two-level segment-based middle tier of the 1-D
   * operators does not pay off here: the within-block segments are too short at
   * these sizes). Serves operators below the FFT threshold and any column
   * involving non-finite data, whose entrywise IEEE semantics the transforms
   * cannot preserve. */
  template <typename Dest, typename Rhs, typename ProductScalar>
  void directProductColumn(Dest& dst, const Rhs& rhs, Index k, const ProductScalar& alpha) const {
    const Index N = rows();
    // A unit alpha must not multiply: even the identity complex scalar (1,0)
    // pollutes an (Inf,0) value with NaN through the 0*Inf cross term.
    const bool unitAlpha = alpha == ProductScalar(1);
    for (Index i = 0; i < N; ++i) {
      ProductScalar acc(0);
      for (Index j = 0; j < N; ++j) acc += coeff(i, j) * rhs.coeff(j, k);
      dst.coeffRef(i, k) += unitAlpha ? acc : ProductScalar(alpha * acc);
    }
  }

  /** \internal Direct O(N^2) product kernel over every column, see
   * directProductColumn(). */
  template <typename Dest, typename Rhs, typename ProductScalar>
  void directProduct(Dest& dst, const Rhs& rhs, const ProductScalar& alpha) const {
    for (Index k = 0; k < rhs.cols(); ++k) directProductColumn(dst, rhs, k, alpha);
  }

  /** \internal Applies the operator whose 2-D symbol is \a s to every column of
   * \a rhs: reshape to n2 x n1 (column-major), 2-D FFT, multiply by \a s,
   * back-transform, take the \c ProductScalar part. Adds into \a dst when
   * \a accumulate, overwrites otherwise. The transforms run on the grid of
   * \a s itself: for the padded product symbol (see computeProdSymbol()) the
   * column is zero-padded into the (p2, p1) grid and the leading (n2, n1) block
   * of the back-transform is read out; for an exact-size symbol the grid is
   * (n2, n1) and no padding is involved.
   *
   * The transforms sum up to \c N inputs, so intermediates can overflow even
   * when every entry of the true result is representable. To keep the whole
   * pipeline overflow-free for finite inputs anywhere in the floating-point
   * range, the symbol and each right-hand-side column are rescaled by an exact
   * power of two that brings their maximum modulus below one -- huge generators
   * make huge symbols, hence both sides -- and the removed exponent is folded
   * back into the output through per-entry ldexp, which saturates cleanly to
   * zero or infinity if the true result itself leaves the representable range.
   * The exponents come from the component-wise magnitudes: the modulus of a
   * finite complex value near the overflow threshold is not representable,
   * which would silently disable the scaling exactly where it is needed.
   *
   * A single plain (fast-max) reduction pass per column yields the finiteness
   * routing, the zero-column shortcut and the scaling exponent (see
   * internal::structured_exponent_bound_finite() for why missing a NaN in the
   * fast max cannot change a routing decision). Non-finite columns are handed
   * to \a directColumn, the caller's per-column direct kernel, so the remaining
   * columns keep the fast path; a non-finite symbol is the caller's
   * responsibility (addProduct routes it to the direct kernel up front, and
   * solve() takes its dedicated pseudo-inverse fallback for non-finite
   * right-hand sides). Zero columns short-circuit to zero, but only under a
   * finite symbol (Inf*0 and NaN*0 are NaN entrywise) and after an exact
   * recheck: the fast max can miss a NaN hiding among zeros (an Inf always
   * surfaces), and such a column must fall back, not shortcut. */
  template <typename Dest, typename Rhs, typename ProductScalar, typename DirectColumn>
  void applySymbol(Dest& dst, const ComplexArray& s, const Rhs& rhs, const ProductScalar& alpha, bool accumulate,
                   DirectColumn&& directColumn) const {
    const Index n2 = blockSize(), n1 = numBlocks(), N = rows();
    const Index p2 = s.rows(), p1 = s.cols();  // the symbol's transform grid: (n2, n1) or the padded product grid
    const bool padded = p2 != n2 || p1 != n1;
    const bool sFinite = s.allFinite();
    // max|s| < 2^es; component bounds avoid overflow in complex moduli.
    const int es = internal::structured_exponent_bound(s);
    ComplexArray sScaled;
    if (es != 0) {
      sScaled = s;
      ldexpInPlace(sScaled, -es);
    }
    const ComplexArray& sUse = es != 0 ? sScaled : s;
    Matrix<ProductScalar, Dynamic, 1> xc(N);
    ComplexArray X(p2, p1), Xn;  // Xn: the leading-block extraction of a padded grid
    for (Index k = 0; k < rhs.cols(); ++k) {
      xc = rhs.col(k).template cast<ProductScalar>();
      const RealScalar m = xc.realView().cwiseAbs().maxCoeff();
      if (!(numext::isfinite)(m)) {
        directColumn(k);
        continue;
      }
      if (m == RealScalar(0)) {
        // The fast max cannot hide an Inf (those comparisons are ordered), but
        // it can miss a NaN among zeros: recheck exactly before shortcutting.
        if ((xc.array() == ProductScalar(0)).all()) {
          if (sFinite) {
            // An exactly zero column maps to an exactly zero column -- unless
            // the symbol holds Inf or NaN, whose products with zero are NaN;
            // falling through to the transforms produces exactly that.
            if (!accumulate) dst.col(k).setZero();
            continue;
          }
        } else {
          directColumn(k);
          continue;
        }
      }
      int ex = 0;  // stays 0 for an all-zero column: no scaling
      if (m > RealScalar(0)) {
        EIGEN_USING_STD(frexp);
        frexp(m, &ex);
        EIGEN_IF_CONSTEXPR (NumTraits<ProductScalar>::IsComplex) ++ex;
      }
      // reshaped() defaults to column-major traversal, the flattening the
      // two-level structure keys on, regardless of EIGEN_DEFAULT_TO_ROW_MAJOR.
      if (padded) {
        X.setZero();
        X.topLeftCorner(n2, n1) = xc.reshaped(n2, n1).template cast<Complex>();
      } else {
        X = xc.reshaped(n2, n1).template cast<Complex>();
      }
      ldexpInPlace(X, -ex);
      fft2(X);
      X.array() *= sUse.array();
      ifft2(X);
      if (padded) Xn = X.topLeftCorner(n2, n1);
      ComplexArray& out = padded ? Xn : X;
      ldexpInPlace(out, ex + es);
      if (accumulate)
        dst.col(k) += alpha * internal::structured_scalar_part_impl<ProductScalar>::run(out.reshaped());
      else
        dst.col(k) = alpha * internal::structured_scalar_part_impl<ProductScalar>::run(out.reshaped());
    }
  }

  /** \internal Multiplies every entry of \a X by 2^e, exactly. The per-entry
   * ldexp saturates to zero / infinity component-wise without ever forming the
   * (possibly unrepresentable) scale factor 2^e itself. */
  static void ldexpInPlace(ComplexArray& X, int e) {
    if (e == 0) return;
    X.realView() = X.realView().array().ldexp(e).matrix();
  }

  /** \internal In-place forward or inverse 2-D FFT by the row-column algorithm:
   * transform every column (length n2), then every row (length n1). Length-1
   * transforms are the identity (and unsupported by the kissfft backend), hence
   * the guards. */
  template <bool Inverse>
  void transform2(ComplexArray& X) const {
    auto&& fft = internal::structured_fft_engine<RealScalar>();
    const Index n2 = X.rows(), n1 = X.cols();
    // Both passes share one output buffer (resized by the engine) and one packed
    // input buffer, allocated at most twice for the whole transform. A column of
    // the column-major grid is already packed and goes to the engine as is; a row
    // is strided, and the engine would pack it into a fresh temporary per call.
    ComplexVector tmp, rowv;
    if (n2 > 1) {
      for (Index k1 = 0; k1 < n1; ++k1) {
        EIGEN_IF_CONSTEXPR (Inverse) {
          fft.inv(tmp, X.col(k1), n2);
        } else {
          fft.fwd(tmp, X.col(k1), n2);
        }
        X.col(k1) = tmp;
      }
    }
    if (n1 > 1) {
      for (Index k2 = 0; k2 < n2; ++k2) {
        rowv = X.row(k2).transpose();
        EIGEN_IF_CONSTEXPR (Inverse) {
          fft.inv(tmp, rowv, n1);
        } else {
          fft.fwd(tmp, rowv, n1);
        }
        X.row(k2) = tmp.transpose();
      }
    }
  }

  /** \internal In-place 2-D forward FFT. */
  void fft2(ComplexArray& X) const { transform2<false>(X); }

  /** \internal In-place 2-D inverse FFT. */
  void ifft2(ComplexArray& X) const { transform2<true>(X); }

  /** \internal Reconstructs the generating array from an exact-size symbol. */
  GeneratorType generatorFromSymbol(ComplexArray symbol) const {
    ifft2(symbol);
    GeneratorType generator = internal::structured_scalar_part_impl<Scalar>::run(symbol);
    return generator;
  }

  /** \internal \returns the 2-D DFT of the generating array. */
  ComplexArray computeSymbol() const {
    ComplexArray s = m_g.template cast<Complex>();
    fft2(s);
    return s;
  }

  /** \internal \returns \a M index-reversed in both dimensions:
   * result(k2, k1) = M((-k2) mod n2, (-k1) mod n1). Row 0 and column 0 stay in
   * place; the rest is a two-dimensional reversal. Empty input stays empty. */
  template <typename MatType>
  static MatType reverse2(const MatType& M) {
    const Index r = M.rows(), c = M.cols();
    MatType R(r, c);
    if (M.size() == 0) return R;
    R(0, 0) = M(0, 0);
    if (c > 1) R.row(0).tail(c - 1) = M.row(0).tail(c - 1).reverse();
    if (r > 1) R.col(0).tail(r - 1) = M.col(0).tail(r - 1).reverse();
    if (r > 1 && c > 1) R.bottomRightCorner(r - 1, c - 1) = M.bottomRightCorner(r - 1, c - 1).reverse();
    return R;
  }

  /** \internal Computes the moduli of the symbol entries and the matching
   * rank/pseudo-inversion threshold, both evaluated in an exactly rescaled frame
   * [5]: the entries are pre-scaled by a power of two chosen so no modulus can
   * overflow. A finite complex entry near the overflow threshold has a
   * non-representable modulus, which would otherwise turn the threshold into
   * infinity and misclassify every other entry (rank under-reported, solve()
   * zeroing valid Fourier modes). The rescaling is exact, so comparing scaled
   * moduli against the scaled threshold is equivalent to the unscaled
   * comparison. The threshold keeps the N * epsilon * max|s| convention of [3],
   * chapter 5.4, and the smallest-normal clamp -- carried into the scaled frame,
   * where its underflowing to zero for a huge frame is correct: no entry of such
   * a symbol can sit below the smallest normal number. Entries at or above the
   * threshold, in particular a smallest-normal entry of a moderate symbol, are
   * inverted (their reciprocals are finite). */
  static void scaledModuli(const ComplexArray& s, RealArray& mods, RealScalar& tol) {
    const int e = numext::maxi(internal::structured_exponent_bound(s), 0);
    // Two exact factors, as in Circulant::scaledModuli(): a single 2^-e is itself
    // subnormal once the frame exceeds the exponent range, and reads as zero under
    // flush-to-zero -- collapsing every scaled modulus, and the threshold with
    // them. Neither factor exceeds one, so no intermediate underflows on its own.
    const RealScalar down1 = numext::ldexp(RealScalar(1), -(e / 2)), down2 = numext::ldexp(RealScalar(1), -(e - e / 2));
    mods = ((s * down1) * down2).cwiseAbs();
    tol = numext::maxi(RealScalar(s.size()) * NumTraits<RealScalar>::epsilon() * mods.maxCoeff(),
                       ((std::numeric_limits<RealScalar>::min)() * down1) * down2);
  }

  /** \internal Writes the unit-norm 2-D Fourier eigenvector of frequencies
   * \c (f1, f2) into column \a dstCol of \a F: entry \c b1*n2 + i2 is
   * exp(2 pi i (b1 f1 / n1 + i2 f2 / n2)) / sqrt(N). All frequency products are
   * accumulated incrementally modulo their length, so the angles stay O(2 pi) at
   * full accuracy and no Index overflow can occur. */
  void fourierColumn(ComplexMatrix& F, Index f1, Index f2, Index dstCol) const {
    const Index n2 = blockSize(), n1 = numBlocks();
    const RealScalar scale = RealScalar(1) / numext::sqrt(RealScalar(rows()));
    ComplexVector w2(n2);
    Index jf = 0;  // i2 * f2 mod n2
    for (Index i2 = 0; i2 < n2; ++i2) {
      w2[i2] = std::polar(scale, RealScalar(2 * EIGEN_PI) * RealScalar(jf) / RealScalar(n2));
      jf += f2;
      if (jf >= n2) jf -= n2;
    }
    Index bf = 0;  // b1 * f1 mod n1
    for (Index b1 = 0; b1 < n1; ++b1) {
      const Complex w1 = std::polar(RealScalar(1), RealScalar(2 * EIGEN_PI) * RealScalar(bf) / RealScalar(n1));
      F.col(dstCol).segment(b1 * n2, n2) = w1 * w2;
      bf += f1;
      if (bf >= n1) bf -= n1;
    }
  }

  /** \internal Projects the complex determinant onto \c Scalar. */
  static Scalar toScalar(const Complex& z, std::true_type /*scalar_is_real*/) { return numext::real(z); }
  static Scalar toScalar(const Complex& z, std::false_type /*scalar_is_real*/) { return z; }

  GeneratorType m_g;
  ComplexArray m_symbol;
  // The padded embedding symbol products use when a block dimension is not
  // 5-smooth; empty otherwise. See computeProdSymbol().
  ComplexArray m_prodSymbol;
  bool m_fftUsable;
};

/** \ingroup StructuredMatrices_Module
 * \returns a \ref Bccb operator with generating array \a generator. The
 * compile-time dimensions of the operator are deduced from the array. */
template <typename Derived>
Bccb<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> makeBccb(
    const MatrixBase<Derived>& generator) {
  return Bccb<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>(generator);
}

namespace internal {

template <typename Scalar_, int BlockSize_, int NumBlocks_, typename Rhs, int ProductTag>
struct generic_product_impl<Bccb<Scalar_, BlockSize_, NumBlocks_>, Rhs, StructuredShape, DenseShape, ProductTag>
    : structured_product_impl<Bccb<Scalar_, BlockSize_, NumBlocks_>, Rhs> {};

}  // namespace internal

}  // namespace Eigen

#endif  // EIGEN_STRUCTURED_BCCB_H
