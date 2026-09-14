// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "main.h"

// Substitution is backward stable: the computed inverse X of a triangular T satisfies
// |T X - I| <= c*n*eps*|T|*|X| entrywise, for a small constant c (Higham, Accuracy and Stability of
// Numerical Algorithms, 2nd ed., Thm 8.5). The factors below absorb c and the entrywise-to-norm step;
// the largest ratio measured over the families tested here is 2, at n = 1.
static constexpr int kResidualFactor = 16;
// Forward error against the reference solve, bounded by the residual bound times cond(T) <= |T|*|X|.
static constexpr int kForwardFactor = 16;

// The part of the storage that a triangularView<mode>() does not own, and that an in-place kernel must
// therefore leave bit for bit unchanged: the opposite triangle, plus the diagonal under UnitDiag.
constexpr unsigned int untouched_mode(unsigned int mode) {
  return ((mode & Lower) != 0 ? unsigned(Upper) : unsigned(Lower)) | ((mode & UnitDiag) != 0 ? 0u : unsigned(ZeroDiag));
}

template <unsigned int Mode, typename MatrixType, typename OperandType>
void check_triangular_inverse(const MatrixType& t, OperandType& x, bool well_conditioned) {
  using Scalar = typename MatrixType::Scalar;
  using RealScalar = typename NumTraits<Scalar>::Real;
  using DenseType = Matrix<Scalar, Dynamic, Dynamic>;

  const Index n = t.rows();
  const RealScalar eps = NumTraits<RealScalar>::epsilon();

  x = t;
  x.template triangularView<Mode>().inverseInPlace();

  constexpr unsigned int kUntouched = untouched_mode(Mode);
  VERIFY_IS_CWISE_EQUAL(x.template triangularView<kUntouched>().toDenseMatrix(),
                        t.template triangularView<kUntouched>().toDenseMatrix());

  const DenseType td = t.template triangularView<Mode>().toDenseMatrix();
  const DenseType xd = x.template triangularView<Mode>().toDenseMatrix();
  const RealScalar residual = (td * xd - DenseType::Identity(n, n)).norm();
  const RealScalar residual_bound = RealScalar(kResidualFactor * n) * eps * td.norm() * xd.norm();
  VERIFY((numext::isfinite)(residual_bound));
  VERIFY(residual <= residual_bound);

  if (!well_conditioned) return;
  MatrixType reference = MatrixType::Identity(n, n);
  t.template triangularView<Mode>().solveInPlace(reference);
  const DenseType referenced = reference.template triangularView<Mode>().toDenseMatrix();
  const RealScalar forward_bound = RealScalar(kForwardFactor * n) * eps * td.norm() * xd.norm() * xd.norm();
  VERIFY((numext::isfinite)(forward_bound));
  VERIFY((xd - referenced).norm() <= forward_bound);
}

// Strictly diagonally dominant, so cond(T) stays O(1) and the forward comparison is meaningful. The
// scaling also holds with a unit diagonal, where the stored diagonal is ignored.
template <typename MatrixType>
MatrixType well_conditioned_triangular(Index n) {
  using Scalar = typename MatrixType::Scalar;
  using RealScalar = typename NumTraits<Scalar>::Real;
  MatrixType t = MatrixType::Random(n, n) / RealScalar(n + 1);
  t.diagonal().array() += Scalar(1);
  return t;
}

// Unscaled off-diagonals, so |T^-1| can grow exponentially in n; only the residual bound applies.
template <typename MatrixType>
MatrixType ill_conditioned_triangular(Index n) {
  using Scalar = typename MatrixType::Scalar;
  MatrixType t = MatrixType::Random(n, n);
  t.diagonal().array() += Scalar(2);
  return t;
}

template <typename MatrixType, typename OperandType>
void check_inverse_all_modes(const MatrixType& t, OperandType& x, bool well_conditioned) {
  check_triangular_inverse<Lower>(t, x, well_conditioned);
  check_triangular_inverse<Upper>(t, x, well_conditioned);
  check_triangular_inverse<UnitLower>(t, x, well_conditioned);
  check_triangular_inverse<UnitUpper>(t, x, well_conditioned);
}

template <typename MatrixType>
void triangular_inverse_all_modes(Index n) {
  MatrixType x(n, n);
  check_inverse_all_modes(well_conditioned_triangular<MatrixType>(n), x, /*well_conditioned=*/true);
  check_inverse_all_modes(ill_conditioned_triangular<MatrixType>(n), x, /*well_conditioned=*/false);
}

// The destination of LLT::inverse() need not be a plain object, so run the kernel through a block of a
// larger matrix: a non-zero offset and an outer stride that does not match the block width.
template <typename MatrixType>
void triangular_inverse_on_block(Index n) {
  const Index row_offset = 2, col_offset = 3;
  MatrixType host = MatrixType::Random(n + 3, n + 5);
  MatrixType expected = host;
  Block<MatrixType> x(host, row_offset, col_offset, n, n);

  check_triangular_inverse<Lower>(well_conditioned_triangular<MatrixType>(n), x, /*well_conditioned=*/true);

  // Nothing outside the block moved.
  expected.block(row_offset, col_offset, n, n) = x;
  VERIFY_IS_CWISE_EQUAL(host, expected);
}

// An operand whose inner stride is a run-time value, which is what a Map or Ref with a dynamic inner
// stride hands the blocked kernels: their TRSM and TRMM steps must reach each panel through that
// value, since the compile-time InnerStrideAtCompileTime they also carry is Dynamic.
template <typename MatrixType>
void triangular_inverse_on_strided_map(Index n) {
  using Scalar = typename MatrixType::Scalar;
  using VectorType = Matrix<Scalar, Dynamic, 1>;
  using StrideType = Stride<Dynamic, Dynamic>;
  const Index inner = 2, outer = 2 * n + 3;
  const Index buffer_size = numext::maxi(Index(1), (n - 1) * (outer + inner) + 1);
  VectorType buffer = VectorType::Random(buffer_size);
  const VectorType before = buffer;
  Map<MatrixType, 0, StrideType> x(buffer.data(), n, n, StrideType(outer, inner));

  check_triangular_inverse<Lower>(well_conditioned_triangular<MatrixType>(n), x, /*well_conditioned=*/true);
  check_triangular_inverse<Upper>(well_conditioned_triangular<MatrixType>(n), x, /*well_conditioned=*/true);

  // Nothing in the gaps between the mapped coefficients moved.
  std::vector<bool> mapped(buffer_size, false);
  for (Index j = 0; j < n; ++j)
    for (Index i = 0; i < n; ++i) mapped[static_cast<std::size_t>(&x.coeffRef(i, j) - buffer.data())] = true;
  for (Index k = 0; k < buffer_size; ++k)
    if (!mapped[k]) VERIFY_IS_EQUAL(buffer[k], before[k]);
}

// internal::triangular_adjoint_square_in_place() is what LLT::inverse() uses to turn L^-1 into
// L^-* L^-1. It is not public API, but it needs coverage of its own: a Cholesky factor always has a
// real diagonal, so the conjugations in the kernel are unobservable through LLT.
template <int UpLo, typename MatrixType>
void adjoint_square_in_place(Index n) {
  using Scalar = typename MatrixType::Scalar;
  using RealScalar = typename NumTraits<Scalar>::Real;
  using DenseType = Matrix<Scalar, Dynamic, Dynamic>;

  const RealScalar eps = NumTraits<RealScalar>::epsilon();
  const MatrixType t = ill_conditioned_triangular<MatrixType>(n);
  MatrixType a = t;
  internal::triangular_adjoint_square_in_place<UpLo>(a);
  VERIFY((a.diagonal().imag().array() == RealScalar(0)).all());

  const DenseType td = t.template triangularView<UpLo>().toDenseMatrix();
  const DenseType expected = UpLo == Lower ? DenseType(td.adjoint() * td) : DenseType(td * td.adjoint());
  const DenseType computed = a.template triangularView<UpLo>().toDenseMatrix();
  // n accumulations of products of entries of T, so the rounding error is bounded by ~n*eps*|T|^2.
  const RealScalar bound = RealScalar(kResidualFactor * n) * eps * td.norm() * td.norm();
  VERIFY((numext::isfinite)(bound));
  VERIFY((computed - expected.template triangularView<UpLo>().toDenseMatrix()).norm() <= bound);

  constexpr unsigned int kUntouched = untouched_mode(UpLo);
  VERIFY_IS_CWISE_EQUAL(a.template triangularView<kUntouched>().toDenseMatrix(),
                        t.template triangularView<kUntouched>().toDenseMatrix());
}

template <typename MatrixType>
void adjoint_square_both_triangles(Index n) {
  adjoint_square_in_place<Lower, MatrixType>(n);
  adjoint_square_in_place<Upper, MatrixType>(n);
}

EIGEN_DECLARE_TEST(triangular_in_place) {
  // 31/32 straddles the unblocked-to-blocked switch, 40 and 65 are not multiples of the block size,
  // and at 256 the heuristic first picks a block big enough for the nested kernels to block in turn.
  const Index sizes[] = {0, 1, 2, 3, 7, 16, 31, 32, 33, 40, 64, 65, 128, 256};
  for (int i = 0; i < g_repeat; i++) {
    for (Index n : sizes) {
      EIGEN_UNUSED_VARIABLE(n);  // the part-6 build compiles no subtest that takes n
      CALL_SUBTEST_1(triangular_inverse_all_modes<MatrixXd>(n));
      CALL_SUBTEST_2(triangular_inverse_all_modes<MatrixXcd>(n));
      CALL_SUBTEST_3(triangular_inverse_all_modes<MatrixXf>(n));
      CALL_SUBTEST_4((triangular_inverse_all_modes<Matrix<double, Dynamic, Dynamic, RowMajor>>(n)));
      CALL_SUBTEST_5(triangular_inverse_on_block<MatrixXd>(n));
      CALL_SUBTEST_5(triangular_inverse_on_strided_map<MatrixXd>(n));
      CALL_SUBTEST_7(adjoint_square_both_triangles<MatrixXd>(n));
      CALL_SUBTEST_8(adjoint_square_both_triangles<MatrixXcd>(n));
      CALL_SUBTEST_8((adjoint_square_both_triangles<Matrix<std::complex<double>, Dynamic, Dynamic, RowMajor>>(n)));
      CALL_SUBTEST_9(adjoint_square_both_triangles<MatrixXf>(n));
      CALL_SUBTEST_10((adjoint_square_both_triangles<Matrix<double, Dynamic, Dynamic, RowMajor>>(n)));
    }
    CALL_SUBTEST_6((triangular_inverse_all_modes<Matrix<double, 1, 1>>(1)));
    CALL_SUBTEST_6(triangular_inverse_all_modes<Matrix4d>(4));
    CALL_SUBTEST_6((triangular_inverse_all_modes<Matrix<std::complex<float>, 8, 8>>(8)));
  }
}
