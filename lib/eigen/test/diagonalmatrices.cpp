// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009 Benoit Jacob <jacob.benoit.1@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// discard stack allocation as that too bypasses malloc
#define EIGEN_STACK_ALLOCATION_LIMIT 0
// heap allocation will raise an assert if enabled at runtime
#define EIGEN_RUNTIME_NO_MALLOC

#include "main.h"
#include "random_for_arithmetic.h"
using namespace std;
template <typename MatrixType>
void diagonalmatrices(const MatrixType& m) {
  typedef typename MatrixType::Scalar Scalar;
  enum { Rows = MatrixType::RowsAtCompileTime, Cols = MatrixType::ColsAtCompileTime };
  typedef Matrix<Scalar, Rows, 1> VectorType;
  typedef Matrix<Scalar, 1, Cols> RowVectorType;
  typedef Matrix<Scalar, Rows, Rows> SquareMatrixType;
  typedef Matrix<Scalar, Dynamic, Dynamic> DynMatrixType;
  typedef DiagonalMatrix<Scalar, Rows> LeftDiagonalMatrix;
  typedef DiagonalMatrix<Scalar, Cols> RightDiagonalMatrix;
  typedef Matrix<Scalar, Rows == Dynamic ? Dynamic : 2 * Rows, Cols == Dynamic ? Dynamic : 2 * Cols> BigMatrix;
  Index rows = m.rows();
  Index cols = m.cols();

  MatrixType m1 = random_for_arithmetic<MatrixType>(rows, cols), m2 = random_for_arithmetic<MatrixType>(rows, cols);
  VectorType v1 = random_for_arithmetic<VectorType>(rows), v2 = random_for_arithmetic<VectorType>(rows);
  RowVectorType rv1 = random_for_arithmetic<RowVectorType>(cols), rv2 = random_for_arithmetic<RowVectorType>(cols);

  LeftDiagonalMatrix ldm1(v1), ldm2(v2);
  RightDiagonalMatrix rdm1(rv1), rdm2(rv2);

  Scalar s1 = random_scalar_for_arithmetic<Scalar>();

  SquareMatrixType sq_m1(v1.asDiagonal());
  VERIFY_IS_APPROX(sq_m1, v1.asDiagonal().toDenseMatrix());
  sq_m1 = v1.asDiagonal();
  VERIFY_IS_APPROX(sq_m1, v1.asDiagonal().toDenseMatrix());
  SquareMatrixType sq_m2 = v1.asDiagonal();
  VERIFY_IS_APPROX(sq_m1, sq_m2);

  ldm1 = v1.asDiagonal();
  LeftDiagonalMatrix ldm3(v1);
  VERIFY_IS_APPROX(ldm1.diagonal(), ldm3.diagonal());
  LeftDiagonalMatrix ldm4 = v1.asDiagonal();
  VERIFY_IS_APPROX(ldm1.diagonal(), ldm4.diagonal());

  sq_m1.block(0, 0, rows, rows) = ldm1;
  VERIFY_IS_APPROX(sq_m1, ldm1.toDenseMatrix());
  sq_m1.transpose() = ldm1;
  VERIFY_IS_APPROX(sq_m1, ldm1.toDenseMatrix());

  Index i = internal::random<Index>(0, rows - 1);
  Index j = internal::random<Index>(0, cols - 1);

  internal::set_is_malloc_allowed(false);
  VERIFY_IS_APPROX(((ldm1 * m1)(i, j)), ldm1.diagonal()(i) * m1(i, j));
  VERIFY_IS_APPROX(((ldm1 * (m1 + m2))(i, j)), ldm1.diagonal()(i) * (m1 + m2)(i, j));
  VERIFY_IS_APPROX(((m1 * rdm1)(i, j)), rdm1.diagonal()(j) * m1(i, j));
  VERIFY_IS_APPROX(((v1.asDiagonal() * m1)(i, j)), v1(i) * m1(i, j));
  VERIFY_IS_APPROX(((m1 * rv1.asDiagonal())(i, j)), rv1(j) * m1(i, j));
  VERIFY_IS_APPROX((((v1 + v2).asDiagonal() * m1)(i, j)), (v1 + v2)(i)*m1(i, j));
  VERIFY_IS_APPROX((((v1 + v2).asDiagonal() * (m1 + m2))(i, j)), (v1 + v2)(i) * (m1 + m2)(i, j));
  VERIFY_IS_APPROX(((m1 * (rv1 + rv2).asDiagonal())(i, j)), (rv1 + rv2)(j)*m1(i, j));
  VERIFY_IS_APPROX((((m1 + m2) * (rv1 + rv2).asDiagonal())(i, j)), (rv1 + rv2)(j) * (m1 + m2)(i, j));
  VERIFY_IS_APPROX((ldm1 * ldm1).diagonal()(i), ldm1.diagonal()(i) * ldm1.diagonal()(i));
  VERIFY_IS_APPROX((ldm1 * ldm1 * m1)(i, j), ldm1.diagonal()(i) * ldm1.diagonal()(i) * m1(i, j));
  VERIFY_IS_APPROX(((v1.asDiagonal() * v1.asDiagonal()).diagonal()(i)), v1(i) * v1(i));
  internal::set_is_malloc_allowed(true);

  if (rows > 1) {
    DynMatrixType tmp = m1.topRows(rows / 2), res;
    VERIFY_IS_APPROX((res = m1.topRows(rows / 2) * rv1.asDiagonal()), tmp * rv1.asDiagonal());
    VERIFY_IS_APPROX((res = v1.head(rows / 2).asDiagonal() * m1.topRows(rows / 2)),
                     v1.head(rows / 2).asDiagonal() * tmp);
  }

  BigMatrix big;
  big.setZero(2 * rows, 2 * cols);

  big.block(i, j, rows, cols) = m1;
  big.block(i, j, rows, cols) = v1.asDiagonal() * big.block(i, j, rows, cols);

  VERIFY_IS_APPROX((big.block(i, j, rows, cols)), v1.asDiagonal() * m1);

  big.block(i, j, rows, cols) = m1;
  big.block(i, j, rows, cols) = big.block(i, j, rows, cols) * rv1.asDiagonal();
  VERIFY_IS_APPROX((big.block(i, j, rows, cols)), m1 * rv1.asDiagonal());

  // products do not allocate memory
  MatrixType res(rows, cols);
  internal::set_is_malloc_allowed(false);
  res.noalias() = ldm1 * m1;
  res.noalias() = m1 * rdm1;
  res.noalias() = ldm1 * m1 * rdm1;
  res.noalias() = LeftDiagonalMatrix::Identity(rows) * m1 * RightDiagonalMatrix::Zero(cols);
  internal::set_is_malloc_allowed(true);

  // scalar multiple
  VERIFY_IS_APPROX(LeftDiagonalMatrix(ldm1 * s1).diagonal(), ldm1.diagonal() * s1);
  VERIFY_IS_APPROX(LeftDiagonalMatrix(s1 * ldm1).diagonal(), s1 * ldm1.diagonal());

  VERIFY_IS_APPROX(m1 * (rdm1 * s1), (m1 * rdm1) * s1);
  VERIFY_IS_APPROX(m1 * (s1 * rdm1), (m1 * rdm1) * s1);

  // Diagonal to dense
  sq_m1 = random_for_arithmetic<SquareMatrixType>(rows, rows);
  sq_m2 = sq_m1;
  VERIFY_IS_APPROX((sq_m1 += (s1 * v1).asDiagonal()), sq_m2 += (s1 * v1).asDiagonal().toDenseMatrix());
  VERIFY_IS_APPROX((sq_m1 -= (s1 * v1).asDiagonal()), sq_m2 -= (s1 * v1).asDiagonal().toDenseMatrix());
  VERIFY_IS_APPROX((sq_m1 = (s1 * v1).asDiagonal()), (s1 * v1).asDiagonal().toDenseMatrix());

  sq_m1 = random_for_arithmetic<SquareMatrixType>(rows, rows);
  sq_m2 = v1.asDiagonal();
  sq_m2 = sq_m1 * sq_m2;
  VERIFY_IS_APPROX((sq_m1 * v1.asDiagonal()).col(i), sq_m2.col(i));
  VERIFY_IS_APPROX((sq_m1 * v1.asDiagonal()).row(i), sq_m2.row(i));

  sq_m1 = v1.asDiagonal();
  sq_m2 = v2.asDiagonal();
  SquareMatrixType sq_m3 = v1.asDiagonal();
  VERIFY_IS_APPROX(sq_m3 = v1.asDiagonal() + v2.asDiagonal(), sq_m1 + sq_m2);
  VERIFY_IS_APPROX(sq_m3 = v1.asDiagonal() - v2.asDiagonal(), sq_m1 - sq_m2);
  VERIFY_IS_APPROX(sq_m3 = v1.asDiagonal() - 2 * v2.asDiagonal() + v1.asDiagonal(), sq_m1 - 2 * sq_m2 + sq_m1);

  // Zero and Identity
  LeftDiagonalMatrix zero = LeftDiagonalMatrix::Zero(rows);
  LeftDiagonalMatrix identity = LeftDiagonalMatrix::Identity(rows);
  VERIFY_IS_APPROX(identity.diagonal().sum(), Scalar(rows));
  VERIFY_IS_APPROX(zero.diagonal().sum(), Scalar(0));
  VERIFY_IS_APPROX((zero + 2 * LeftDiagonalMatrix::Identity(rows)).diagonal().sum(), Scalar(2 * rows));
}

template <typename MatrixType>
void as_scalar_product(const MatrixType& m) {
  typedef typename MatrixType::Scalar Scalar;
  typedef Matrix<Scalar, MatrixType::RowsAtCompileTime, 1> VectorType;
  typedef Matrix<Scalar, Dynamic, Dynamic> DynMatrixType;
  typedef Matrix<Scalar, Dynamic, 1> DynVectorType;
  typedef Matrix<Scalar, 1, Dynamic> DynRowVectorType;

  Index rows = m.rows();
  Index depth = internal::random<Index>(1, EIGEN_TEST_MAX_SIZE);

  VectorType v1 = random_for_arithmetic<VectorType>(rows);
  DynVectorType dv1 = random_for_arithmetic<DynVectorType>(depth);
  DynRowVectorType drv1 = random_for_arithmetic<DynRowVectorType>(depth);
  DynMatrixType dm1 = dv1;
  DynMatrixType drm1 = drv1;

  Scalar s = v1(0);

  VERIFY_IS_APPROX(v1.asDiagonal() * drv1, s * drv1);
  VERIFY_IS_APPROX(dv1 * v1.asDiagonal(), dv1 * s);

  VERIFY_IS_APPROX(v1.asDiagonal() * drm1, s * drm1);
  VERIFY_IS_APPROX(dm1 * v1.asDiagonal(), dm1 * s);
}

template <int>
void bug987() {
  Matrix3Xd points = Matrix3Xd::Random(3, 3);
  Vector2d diag = Vector2d::Random();
  Matrix2Xd tmp1 = points.topRows<2>(), res1, res2;
  VERIFY_IS_APPROX(res1 = diag.asDiagonal() * points.topRows<2>(), res2 = diag.asDiagonal() * tmp1);
  Matrix2d tmp2 = points.topLeftCorner<2, 2>();
  VERIFY_IS_APPROX((res1 = points.topLeftCorner<2, 2>() * diag.asDiagonal()), res2 = tmp2 * diag.asDiagonal());
}

template <int>
void bug2013() {
  Matrix3d m = Matrix3d::Random();
  Vector3d d = Vector3d::Random();

  Matrix3d ref_unit_lower = m.template triangularView<UnitLower>();
  Matrix3d ref_unit_upper = m.template triangularView<UnitUpper>();
  Matrix3d ref_lower = m.template triangularView<Lower>();
  Matrix3d ref_upper = m.template triangularView<Upper>();

  VERIFY_IS_APPROX((m.template triangularView<UnitLower>() * d.asDiagonal()).eval(), ref_unit_lower * d.asDiagonal());
  VERIFY_IS_APPROX((d.asDiagonal() * m.template triangularView<UnitLower>()).eval(), d.asDiagonal() * ref_unit_lower);

  VERIFY_IS_APPROX((m.template triangularView<UnitUpper>() * d.asDiagonal()).eval(), ref_unit_upper * d.asDiagonal());
  VERIFY_IS_APPROX((d.asDiagonal() * m.template triangularView<UnitUpper>()).eval(), d.asDiagonal() * ref_unit_upper);

  Matrix3d actual = Matrix3d::Random();
  Matrix3d expected = actual;
  actual = m;
  expected = m;
  actual.template triangularView<Upper>() = actual.template triangularView<Upper>() * d.asDiagonal();
  expected.template triangularView<Upper>() = Matrix3d(ref_upper * d.asDiagonal());
  VERIFY_IS_APPROX(actual, expected);

  actual = m;
  expected = m;
  actual.template triangularView<Lower>() = d.asDiagonal() * actual.template triangularView<Lower>();
  expected.template triangularView<Lower>() = Matrix3d(d.asDiagonal() * ref_lower);
  VERIFY_IS_APPROX(actual, expected);

  actual.setRandom();
  expected = actual;
  actual.noalias() += m.template triangularView<UnitLower>() * d.asDiagonal();
  expected.noalias() += ref_unit_lower * d.asDiagonal();
  VERIFY_IS_APPROX(actual, expected);

  actual.setRandom();
  expected = actual;
  actual.noalias() -= d.asDiagonal() * m.template triangularView<UnitUpper>();
  expected.noalias() -= d.asDiagonal() * ref_unit_upper;
  VERIFY_IS_APPROX(actual, expected);

  MatrixXd dynamic_m = MatrixXd::Random(4, 4);
  VectorXd dynamic_d = VectorXd::Random(4);
  MatrixXd dynamic_expected(4, 4);
  MatrixXd no_malloc_result(4, 4);

  dynamic_expected = MatrixXd(dynamic_m.template triangularView<UnitLower>()) * dynamic_d.asDiagonal();
  internal::set_is_malloc_allowed(false);
  no_malloc_result.noalias() = dynamic_m.template triangularView<UnitLower>() * dynamic_d.asDiagonal();
  internal::set_is_malloc_allowed(true);
  VERIFY_IS_APPROX(no_malloc_result, dynamic_expected);

  dynamic_expected = dynamic_d.asDiagonal() * MatrixXd(dynamic_m.template triangularView<UnitUpper>());
  internal::set_is_malloc_allowed(false);
  no_malloc_result.noalias() = dynamic_d.asDiagonal() * dynamic_m.template triangularView<UnitUpper>();
  internal::set_is_malloc_allowed(true);
  VERIFY_IS_APPROX(no_malloc_result, dynamic_expected);

  // Triangular destination assignment from a structured*diagonal product must go through the
  // lazy product_evaluator and avoid materializing a full PlainObject temporary.
  no_malloc_result = dynamic_m;
  dynamic_expected = dynamic_m;
  dynamic_expected.template triangularView<Upper>() =
      MatrixXd(dynamic_m.template triangularView<Upper>()) * dynamic_d.asDiagonal();
  internal::set_is_malloc_allowed(false);
  no_malloc_result.template triangularView<Upper>() =
      dynamic_m.template triangularView<Upper>() * dynamic_d.asDiagonal();
  internal::set_is_malloc_allowed(true);
  VERIFY_IS_APPROX(no_malloc_result, dynamic_expected);

  no_malloc_result = dynamic_m;
  dynamic_expected = dynamic_m;
  dynamic_expected.template triangularView<Lower>() =
      dynamic_d.asDiagonal() * MatrixXd(dynamic_m.template triangularView<Lower>());
  internal::set_is_malloc_allowed(false);
  no_malloc_result.template triangularView<Lower>() =
      dynamic_d.asDiagonal() * dynamic_m.template triangularView<Lower>();
  internal::set_is_malloc_allowed(true);
  VERIFY_IS_APPROX(no_malloc_result, dynamic_expected);
}

template <int>
void selfadjoint_diagonal_products() {
  Matrix3cd m = Matrix3cd::Random();
  m.diagonal() = m.diagonal().real();
  Vector3cd d = Vector3cd::Random();

  Matrix3cd ref_lower = m.template selfadjointView<Lower>();
  Matrix3cd ref_upper = m.template selfadjointView<Upper>();

  VERIFY_IS_APPROX((m.template selfadjointView<Lower>() * d.asDiagonal()).eval(), ref_lower * d.asDiagonal());
  VERIFY_IS_APPROX((d.asDiagonal() * m.template selfadjointView<Lower>()).eval(), d.asDiagonal() * ref_lower);

  VERIFY_IS_APPROX((m.template selfadjointView<Upper>() * d.asDiagonal()).eval(), ref_upper * d.asDiagonal());
  VERIFY_IS_APPROX((d.asDiagonal() * m.template selfadjointView<Upper>()).eval(), d.asDiagonal() * ref_upper);

  Matrix3cd actual = Matrix3cd::Random();
  Matrix3cd expected = actual;
  actual = m;
  expected = m;
  actual.template selfadjointView<Upper>() = actual.template selfadjointView<Upper>() * d.asDiagonal();
  expected.template triangularView<Upper>() = Matrix3cd(ref_upper * d.asDiagonal());
  VERIFY_IS_APPROX(actual, expected);

  actual = m;
  expected = m;
  actual.template selfadjointView<Lower>() = d.asDiagonal() * actual.template selfadjointView<Lower>();
  expected.template triangularView<Lower>() = Matrix3cd(d.asDiagonal() * ref_lower);
  VERIFY_IS_APPROX(actual, expected);

  actual.setRandom();
  expected = actual;
  actual.noalias() += m.template selfadjointView<Lower>() * d.asDiagonal();
  expected.noalias() += ref_lower * d.asDiagonal();
  VERIFY_IS_APPROX(actual, expected);

  actual.setRandom();
  expected = actual;
  actual.noalias() -= d.asDiagonal() * m.template selfadjointView<Upper>();
  expected.noalias() -= d.asDiagonal() * ref_upper;
  VERIFY_IS_APPROX(actual, expected);

  MatrixXcd dynamic_m = MatrixXcd::Random(4, 4);
  dynamic_m.diagonal() = dynamic_m.diagonal().real();
  VectorXcd dynamic_d = VectorXcd::Random(4);
  MatrixXcd dynamic_expected(4, 4);
  MatrixXcd no_malloc_result(4, 4);

  dynamic_expected = MatrixXcd(dynamic_m.template selfadjointView<Lower>()) * dynamic_d.asDiagonal();
  internal::set_is_malloc_allowed(false);
  no_malloc_result.noalias() = dynamic_m.template selfadjointView<Lower>() * dynamic_d.asDiagonal();
  internal::set_is_malloc_allowed(true);
  VERIFY_IS_APPROX(no_malloc_result, dynamic_expected);

  dynamic_expected = dynamic_d.asDiagonal() * MatrixXcd(dynamic_m.template selfadjointView<Upper>());
  internal::set_is_malloc_allowed(false);
  no_malloc_result.noalias() = dynamic_d.asDiagonal() * dynamic_m.template selfadjointView<Upper>();
  internal::set_is_malloc_allowed(true);
  VERIFY_IS_APPROX(no_malloc_result, dynamic_expected);

  // Triangular destination assignment from selfadjoint*diagonal must use the lazy
  // product_evaluator (with conjugate-mirror coeffs) and not materialize a temporary.
  no_malloc_result = dynamic_m;
  dynamic_expected = dynamic_m;
  dynamic_expected.template triangularView<Lower>() =
      MatrixXcd(dynamic_m.template selfadjointView<Upper>()) * dynamic_d.asDiagonal();
  internal::set_is_malloc_allowed(false);
  no_malloc_result.template triangularView<Lower>() =
      dynamic_m.template selfadjointView<Upper>() * dynamic_d.asDiagonal();
  internal::set_is_malloc_allowed(true);
  VERIFY_IS_APPROX(no_malloc_result, dynamic_expected);

  no_malloc_result = dynamic_m;
  dynamic_expected = dynamic_m;
  dynamic_expected.template triangularView<Upper>() =
      dynamic_d.asDiagonal() * MatrixXcd(dynamic_m.template selfadjointView<Lower>());
  internal::set_is_malloc_allowed(false);
  no_malloc_result.template triangularView<Upper>() =
      dynamic_d.asDiagonal() * dynamic_m.template selfadjointView<Lower>();
  internal::set_is_malloc_allowed(true);
  VERIFY_IS_APPROX(no_malloc_result, dynamic_expected);
}

// Exercise the block-tile path of the dense selfadjoint x diagonal kernel
// (BlockSize = 32 in ProductEvaluators.h). Picks a few sizes that hit
// full blocks (size = 64), partial blocks (size = 33, 65), and a tiny size
// that bypasses the block loop entirely (size = 8). Also verifies the
// overwrite path leaves no stale data when dst is pre-filled.
template <typename Scalar>
void selfadjoint_diagonal_products_at(Index n) {
  typedef Matrix<Scalar, Dynamic, Dynamic> MatType;
  typedef Matrix<Scalar, Dynamic, 1> VecType;

  MatType m = MatType::Random(n, n);
  m.diagonal() = m.diagonal().real().template cast<Scalar>();  // Hermitian diagonal
  VecType d = VecType::Random(n);

  MatType ref_lower = m.template selfadjointView<Lower>();
  MatType ref_upper = m.template selfadjointView<Upper>();

  // Plain assignment goes through evalTo (overwrite kernel).
  // Pre-fill dst with garbage to verify no stale entries remain.
  MatType dst = MatType::Constant(n, n, Scalar(42));
  dst.noalias() = m.template selfadjointView<Upper>() * d.asDiagonal();
  VERIFY_IS_APPROX(dst, ref_upper * d.asDiagonal());

  dst = MatType::Constant(n, n, Scalar(-7));
  dst.noalias() = m.template selfadjointView<Lower>() * d.asDiagonal();
  VERIFY_IS_APPROX(dst, ref_lower * d.asDiagonal());

  dst = MatType::Constant(n, n, Scalar(13));
  dst.noalias() = d.asDiagonal() * m.template selfadjointView<Upper>();
  VERIFY_IS_APPROX(dst, d.asDiagonal() * ref_upper);

  dst = MatType::Constant(n, n, Scalar(99));
  dst.noalias() = d.asDiagonal() * m.template selfadjointView<Lower>();
  VERIFY_IS_APPROX(dst, d.asDiagonal() * ref_lower);

  // Accumulating paths (scaleAndAddTo).
  MatType base = MatType::Random(n, n);
  dst = base;
  dst.noalias() += m.template selfadjointView<Upper>() * d.asDiagonal();
  VERIFY_IS_APPROX(dst, base + ref_upper * d.asDiagonal());

  dst = base;
  dst.noalias() -= d.asDiagonal() * m.template selfadjointView<Lower>();
  VERIFY_IS_APPROX(dst, base - d.asDiagonal() * ref_lower);

  // Scalar-scaled products: the "Dense ?= scalar * Product" rewriting rule
  // folds alpha into the SelfAdjointView. For a complex alpha that fold is
  // not Hermitian, so the dispatch must restore alpha rather than apply it
  // straight to the kernel — verify all four orientation x triangle combos.
  Scalar alpha = internal::random<Scalar>();
  dst = base;
  dst.noalias() += alpha * (m.template selfadjointView<Lower>() * d.asDiagonal());
  VERIFY_IS_APPROX(dst, base + alpha * (ref_lower * d.asDiagonal()));

  dst = base;
  dst.noalias() -= alpha * (m.template selfadjointView<Upper>() * d.asDiagonal());
  VERIFY_IS_APPROX(dst, base - alpha * (ref_upper * d.asDiagonal()));

  dst = base;
  dst.noalias() += alpha * (d.asDiagonal() * m.template selfadjointView<Upper>());
  VERIFY_IS_APPROX(dst, base + alpha * (d.asDiagonal() * ref_upper));

  dst = base;
  dst.noalias() -= alpha * (d.asDiagonal() * m.template selfadjointView<Lower>());
  VERIFY_IS_APPROX(dst, base - alpha * (d.asDiagonal() * ref_lower));

  // Overwrite-with-scalar path: hits evalTo's HasScalarFactor branch.
  dst = MatType::Constant(n, n, Scalar(17));
  dst.noalias() = alpha * (m.template selfadjointView<Lower>() * d.asDiagonal());
  VERIFY_IS_APPROX(dst, alpha * (ref_lower * d.asDiagonal()));

  dst = MatType::Constant(n, n, Scalar(-3));
  dst.noalias() = alpha * (d.asDiagonal() * m.template selfadjointView<Upper>());
  VERIFY_IS_APPROX(dst, alpha * (d.asDiagonal() * ref_upper));

  // Conjugated nested expressions go through the same blas_traits extraction
  // path. The extracted matrix must keep NeedToConjugate, otherwise the kernel
  // computes with m instead of m.conjugate().
  MatType conj_ref_lower = m.conjugate().template selfadjointView<Lower>();
  MatType conj_ref_upper = m.conjugate().template selfadjointView<Upper>();

  dst = MatType::Constant(n, n, Scalar(23));
  dst.noalias() = m.conjugate().template selfadjointView<Upper>() * d.asDiagonal();
  VERIFY_IS_APPROX(dst, conj_ref_upper * d.asDiagonal());

  dst = MatType::Constant(n, n, Scalar(-29));
  dst.noalias() = d.asDiagonal() * m.conjugate().template selfadjointView<Lower>();
  VERIFY_IS_APPROX(dst, d.asDiagonal() * conj_ref_lower);

  dst = base;
  dst.noalias() += alpha * (m.conjugate().template selfadjointView<Lower>() * d.asDiagonal());
  VERIFY_IS_APPROX(dst, base + alpha * (conj_ref_lower * d.asDiagonal()));

  dst = base;
  dst.noalias() -= alpha * (d.asDiagonal() * m.conjugate().template selfadjointView<Upper>());
  VERIFY_IS_APPROX(dst, base - alpha * (d.asDiagonal() * conj_ref_upper));
}

template <int>
void selfadjoint_diagonal_products_block_path() {
  selfadjoint_diagonal_products_at<double>(8);
  selfadjoint_diagonal_products_at<double>(33);  // partial off-diagonal block
  selfadjoint_diagonal_products_at<double>(64);  // exact multiple of BlockSize
  selfadjoint_diagonal_products_at<double>(65);  // off-by-one
  selfadjoint_diagonal_products_at<std::complex<double>>(33);
  selfadjoint_diagonal_products_at<std::complex<double>>(65);
}

// In-place patterns where the diagonal operand shares storage with the destination view.
// The fast path in triangular_product_assignment_dispatcher detects the run-time overlap
// and materializes a temporary, so the result must match a reference computed against a
// materialized source. The structured (triangular/selfadjoint) operand can safely share
// storage with dst because the kernel reads each cell before writing it.
template <unsigned int Mode, typename Mat>
void verify_triangular_in_place_with_aliased_diagonal(const Mat& m) {
  // diagonal * tri_view
  {
    Mat actual = m, expected = m;
    Mat ref_diag = actual.diagonal().asDiagonal();
    Mat ref_tri = actual.template triangularView<Mode>();
    expected.template triangularView<Mode>() = (ref_diag * ref_tri).eval();
    actual.template triangularView<Mode>() = actual.diagonal().asDiagonal() * actual.template triangularView<Mode>();
    VERIFY_IS_APPROX(actual, expected);
  }
  // tri_view * diagonal
  {
    Mat actual = m, expected = m;
    Mat ref_diag = actual.diagonal().asDiagonal();
    Mat ref_tri = actual.template triangularView<Mode>();
    expected.template triangularView<Mode>() = (ref_tri * ref_diag).eval();
    actual.template triangularView<Mode>() = actual.template triangularView<Mode>() * actual.diagonal().asDiagonal();
    VERIFY_IS_APPROX(actual, expected);
  }
}

template <unsigned int Mode, typename Mat>
void verify_selfadjoint_in_place_with_aliased_diagonal(const Mat& m) {
  // diagonal * sa_view
  {
    Mat actual = m, expected = m;
    Mat ref_diag = actual.diagonal().asDiagonal();
    Mat ref_sa = actual.template selfadjointView<Mode>();
    expected.template triangularView<Mode>() = (ref_diag * ref_sa).eval();
    actual.template selfadjointView<Mode>() = actual.diagonal().asDiagonal() * actual.template selfadjointView<Mode>();
    VERIFY_IS_APPROX(actual.template triangularView<Mode>().toDenseMatrix(),
                     expected.template triangularView<Mode>().toDenseMatrix());
  }
  // sa_view * diagonal
  {
    Mat actual = m, expected = m;
    Mat ref_diag = actual.diagonal().asDiagonal();
    Mat ref_sa = actual.template selfadjointView<Mode>();
    expected.template triangularView<Mode>() = (ref_sa * ref_diag).eval();
    actual.template selfadjointView<Mode>() = actual.template selfadjointView<Mode>() * actual.diagonal().asDiagonal();
    VERIFY_IS_APPROX(actual.template triangularView<Mode>().toDenseMatrix(),
                     expected.template triangularView<Mode>().toDenseMatrix());
  }
}

template <int>
void structured_diagonal_aliasing() {
  for (int n : {3, 5, 8, 17, 32, 33, 64, 65}) {
    MatrixXcd m = MatrixXcd::Random(n, n);
    m.diagonal() = m.diagonal().real();  // Hermitian-friendly diagonal

    verify_triangular_in_place_with_aliased_diagonal<Upper>(m);
    verify_triangular_in_place_with_aliased_diagonal<Lower>(m);
    verify_selfadjoint_in_place_with_aliased_diagonal<Upper>(m);
    verify_selfadjoint_in_place_with_aliased_diagonal<Lower>(m);
  }
}

EIGEN_DECLARE_TEST(diagonalmatrices) {
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(diagonalmatrices(Matrix<float, 1, 1>()));
    CALL_SUBTEST_1(as_scalar_product(Matrix<float, 1, 1>()));

    CALL_SUBTEST_2(diagonalmatrices(Matrix3f()));
    CALL_SUBTEST_3(diagonalmatrices(Matrix<double, 3, 3, RowMajor>()));
    CALL_SUBTEST_4(diagonalmatrices(Matrix4d()));
    CALL_SUBTEST_5(diagonalmatrices(Matrix<float, 4, 4, RowMajor>()));
    CALL_SUBTEST_6(diagonalmatrices(
        MatrixXcf(internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_6(as_scalar_product(MatrixXcf(1, 1)));
    CALL_SUBTEST_7(diagonalmatrices(
        MatrixXi(internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_8(diagonalmatrices(Matrix<double, Dynamic, Dynamic, RowMajor>(
        internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_9(diagonalmatrices(
        MatrixXf(internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_9(diagonalmatrices(MatrixXf(1, 1)));
    CALL_SUBTEST_9(as_scalar_product(MatrixXf(1, 1)));
  }
  CALL_SUBTEST_10(bug987<0>());
  CALL_SUBTEST_10(bug2013<0>());
  CALL_SUBTEST_10(selfadjoint_diagonal_products<0>());
  CALL_SUBTEST_10(selfadjoint_diagonal_products_block_path<0>());
  CALL_SUBTEST_10(structured_diagonal_aliasing<0>());
}
