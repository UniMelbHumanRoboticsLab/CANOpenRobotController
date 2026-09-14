// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2009 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// Enables the runtime malloc tracking that trsolve_no_malloc() needs. Allocation stays allowed
// by default; only the explicit set_is_malloc_allowed(false) windows below check it.
#define EIGEN_RUNTIME_NO_MALLOC

#include "main.h"

#define VERIFY_TRSM(TRI, XB)                             \
  {                                                      \
    (XB).setRandom();                                    \
    ref = (XB);                                          \
    (TRI).solveInPlace(XB);                              \
    VERIFY_IS_APPROX((TRI).toDenseMatrix() * (XB), ref); \
    (XB).setRandom();                                    \
    ref = (XB);                                          \
    (XB) = (TRI).solve(XB);                              \
    VERIFY_IS_APPROX((TRI).toDenseMatrix() * (XB), ref); \
  }

#define VERIFY_TRSM_ONTHERIGHT(TRI, XB)                                                      \
  {                                                                                          \
    (XB).setRandom();                                                                        \
    ref = (XB);                                                                              \
    (TRI).transpose().template solveInPlace<OnTheRight>(XB.transpose());                     \
    VERIFY_IS_APPROX((XB).transpose() * (TRI).transpose().toDenseMatrix(), ref.transpose()); \
    (XB).setRandom();                                                                        \
    ref = (XB);                                                                              \
    (XB).transpose() = (TRI).transpose().template solve<OnTheRight>(XB.transpose());         \
    VERIFY_IS_APPROX((XB).transpose() * (TRI).transpose().toDenseMatrix(), ref.transpose()); \
  }

template <typename Scalar, int Size, int Cols>
void trsolve(int size = Size, int cols = Cols) {
  typedef typename NumTraits<Scalar>::Real RealScalar;

  Matrix<Scalar, Size, Size, ColMajor> cmLhs(size, size);
  Matrix<Scalar, Size, Size, RowMajor> rmLhs(size, size);

  enum { colmajor = Size == 1 ? RowMajor : ColMajor, rowmajor = Cols == 1 ? ColMajor : RowMajor };
  Matrix<Scalar, Size, Cols, colmajor> cmRhs(size, cols);
  Matrix<Scalar, Size, Cols, rowmajor> rmRhs(size, cols);
  Matrix<Scalar, Dynamic, Dynamic, colmajor> ref(size, cols);

  cmLhs.setRandom();
  cmLhs *= static_cast<RealScalar>(0.1);
  cmLhs.diagonal().array() += static_cast<RealScalar>(1);
  rmLhs.setRandom();
  rmLhs *= static_cast<RealScalar>(0.1);
  rmLhs.diagonal().array() += static_cast<RealScalar>(1);

  VERIFY_TRSM(cmLhs.conjugate().template triangularView<Lower>(), cmRhs);
  VERIFY_TRSM(cmLhs.adjoint().template triangularView<Lower>(), cmRhs);
  VERIFY_TRSM(cmLhs.template triangularView<Upper>(), cmRhs);
  VERIFY_TRSM(cmLhs.template triangularView<Lower>(), rmRhs);
  VERIFY_TRSM(cmLhs.conjugate().template triangularView<Upper>(), rmRhs);
  VERIFY_TRSM(cmLhs.adjoint().template triangularView<Upper>(), rmRhs);

  VERIFY_TRSM(cmLhs.conjugate().template triangularView<UnitLower>(), cmRhs);
  VERIFY_TRSM(cmLhs.template triangularView<UnitUpper>(), rmRhs);

  VERIFY_TRSM(rmLhs.template triangularView<Lower>(), cmRhs);
  VERIFY_TRSM(rmLhs.conjugate().template triangularView<UnitUpper>(), rmRhs);

  VERIFY_TRSM_ONTHERIGHT(cmLhs.conjugate().template triangularView<Lower>(), cmRhs);
  VERIFY_TRSM_ONTHERIGHT(cmLhs.template triangularView<Upper>(), cmRhs);
  VERIFY_TRSM_ONTHERIGHT(cmLhs.template triangularView<Lower>(), rmRhs);
  VERIFY_TRSM_ONTHERIGHT(cmLhs.conjugate().template triangularView<Upper>(), rmRhs);

  VERIFY_TRSM_ONTHERIGHT(cmLhs.conjugate().template triangularView<UnitLower>(), cmRhs);
  VERIFY_TRSM_ONTHERIGHT(cmLhs.template triangularView<UnitUpper>(), rmRhs);

  VERIFY_TRSM_ONTHERIGHT(rmLhs.template triangularView<Lower>(), cmRhs);
  VERIFY_TRSM_ONTHERIGHT(rmLhs.conjugate().template triangularView<UnitUpper>(), rmRhs);

  int c = internal::random<int>(0, cols - 1);
  VERIFY_TRSM(rmLhs.template triangularView<Lower>(), rmRhs.col(c));
  VERIFY_TRSM(cmLhs.template triangularView<Lower>(), rmRhs.col(c));

  // destination with a non-default inner-stride
  // see bug 1741
  {
    typedef Matrix<Scalar, Dynamic, Dynamic> MatrixX;
    MatrixX buffer(2 * cmRhs.rows(), 2 * cmRhs.cols());
    Map<Matrix<Scalar, Size, Cols, colmajor>, 0, Stride<Dynamic, 2> > map1(
        buffer.data(), cmRhs.rows(), cmRhs.cols(), Stride<Dynamic, 2>(2 * cmRhs.outerStride(), 2));
    Map<Matrix<Scalar, Size, Cols, rowmajor>, 0, Stride<Dynamic, 2> > map2(
        buffer.data(), rmRhs.rows(), rmRhs.cols(), Stride<Dynamic, 2>(2 * rmRhs.outerStride(), 2));
    buffer.setZero();
    VERIFY_TRSM(cmLhs.conjugate().template triangularView<Lower>(), map1);
    buffer.setZero();
    VERIFY_TRSM(cmLhs.template triangularView<Lower>(), map2);

    // A runtime inner stride reaches the blocked kernels as OtherInnerStride == Dynamic, which the
    // compile-time 2 above does not; their panel offsets once multiplied by that constant.
    Map<Matrix<Scalar, Size, Cols, colmajor>, 0, Stride<Dynamic, Dynamic> > map3(
        buffer.data(), cmRhs.rows(), cmRhs.cols(), Stride<Dynamic, Dynamic>(2 * cmRhs.outerStride(), 2));
    Map<Matrix<Scalar, Size, Cols, rowmajor>, 0, Stride<Dynamic, Dynamic> > map4(
        buffer.data(), rmRhs.rows(), rmRhs.cols(), Stride<Dynamic, Dynamic>(2 * rmRhs.outerStride(), 2));
    buffer.setZero();
    VERIFY_TRSM(cmLhs.template triangularView<Lower>(), map3);
    buffer.setZero();
    VERIFY_TRSM(cmLhs.template triangularView<Upper>(), map4);
    buffer.setZero();
    VERIFY_TRSM_ONTHERIGHT(cmLhs.template triangularView<Lower>(), map3);
    buffer.setZero();
    VERIFY_TRSM_ONTHERIGHT(cmLhs.template triangularView<Upper>(), map4);
  }

  if (Size == Dynamic) {
    cmLhs.resize(0, 0);
    cmRhs.resize(0, cmRhs.cols());
    Matrix<Scalar, Size, Cols, colmajor> res = cmLhs.template triangularView<Lower>().solve(cmRhs);
    VERIFY_IS_EQUAL(res.rows(), 0);
    VERIFY_IS_EQUAL(res.cols(), cmRhs.cols());
    res = cmRhs;
    cmLhs.template triangularView<Lower>().solveInPlace(res);
    VERIFY_IS_EQUAL(res.rows(), 0);
    VERIFY_IS_EQUAL(res.cols(), cmRhs.cols());
  }
}

// Test triangular solve with non-unit inner stride at blocking boundary sizes.
// The scalar fallback path in trsmKernelR (TriangularSolverMatrix.h lines 156-166)
// is used when OtherInnerStride != 1. The existing bug 1741 test only uses
// InnerStride=2 at random sizes. This exercises the scalar path at sizes that
// trigger blocking transitions and tests additional configurations.
template <int>
void trsolve_strided_boundary() {
  typedef double Scalar;
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixX;

  const int sizes[] = {1, 2, 3, 4, 8, 12, 16, 24, 32, 47, 48, 49, 64};
  for (int si = 0; si < 13; ++si) {
    int n = sizes[si];

    MatrixX lhs = MatrixX::Random(n, n);
    lhs *= 0.1;
    lhs.diagonal().array() += 1.0;

    // InnerStride = 2: ColMajor RHS, OnTheLeft, Lower
    {
      int cols = 5;
      MatrixX buffer(2 * n, 2 * cols);
      Map<MatrixX, 0, Stride<Dynamic, 2> > map(buffer.data(), n, cols, Stride<Dynamic, 2>(2 * n, 2));
      MatrixX ref(n, cols);
      buffer.setZero();
      map.setRandom();
      ref = map;
      lhs.triangularView<Lower>().solveInPlace(map);
      VERIFY_IS_APPROX(lhs.triangularView<Lower>().toDenseMatrix() * MatrixX(map), ref);
    }

    // InnerStride = 2: Upper triangular
    {
      int cols = 5;
      MatrixX buffer(2 * n, 2 * cols);
      Map<MatrixX, 0, Stride<Dynamic, 2> > map(buffer.data(), n, cols, Stride<Dynamic, 2>(2 * n, 2));
      MatrixX ref(n, cols);
      buffer.setZero();
      map.setRandom();
      ref = map;
      lhs.triangularView<Upper>().solveInPlace(map);
      VERIFY_IS_APPROX(lhs.triangularView<Upper>().toDenseMatrix() * MatrixX(map), ref);
    }

    // Runtime inner stride (OtherInnerStride == Dynamic), both sides
    {
      int cols = 5;
      MatrixX buffer(2 * n, 2 * cols);
      Map<MatrixX, 0, Stride<Dynamic, Dynamic> > map(buffer.data(), n, cols, Stride<Dynamic, Dynamic>(2 * n, 2));
      MatrixX ref(n, cols);
      buffer.setZero();
      map.setRandom();
      ref = map;
      lhs.triangularView<Lower>().solveInPlace(map);
      VERIFY_IS_APPROX(lhs.triangularView<Lower>().toDenseMatrix() * MatrixX(map), ref);
      buffer.setZero();
      map.setRandom();
      ref = map;
      lhs.triangularView<Upper>().template solveInPlace<OnTheRight>(map.transpose());
      VERIFY_IS_APPROX(MatrixX(map.transpose()) * lhs.triangularView<Upper>().toDenseMatrix(), ref.transpose());
    }

    // InnerStride = 2: UnitLower (tests the UnitDiag path without diagonal scaling)
    {
      int cols = 3;
      MatrixX buffer(2 * n, 2 * cols);
      Map<MatrixX, 0, Stride<Dynamic, 2> > map(buffer.data(), n, cols, Stride<Dynamic, 2>(2 * n, 2));
      MatrixX ref(n, cols);
      buffer.setZero();
      map.setRandom();
      ref = map;
      lhs.triangularView<UnitLower>().solveInPlace(map);
      VERIFY_IS_APPROX(lhs.triangularView<UnitLower>().toDenseMatrix() * MatrixX(map), ref);
    }

    // InnerStride = 3: Less common stride to exercise the scalar path more thoroughly
    {
      int cols = 4;
      MatrixX buffer(3 * n, 3 * cols);
      Map<MatrixX, 0, Stride<Dynamic, 3> > map(buffer.data(), n, cols, Stride<Dynamic, 3>(3 * n, 3));
      MatrixX ref(n, cols);
      buffer.setZero();
      map.setRandom();
      ref = map;
      lhs.triangularView<Lower>().solveInPlace(map);
      VERIFY_IS_APPROX(lhs.triangularView<Lower>().toDenseMatrix() * MatrixX(map), ref);
    }

    // Vector RHS with InnerStride = 2
    {
      typedef Matrix<Scalar, Dynamic, 1> VecX;
      VecX buffer(2 * n);
      Map<VecX, 0, InnerStride<2> > map(buffer.data(), n, InnerStride<2>(2));
      buffer.setZero();
      map.setRandom();
      VecX ref = map;
      lhs.triangularView<Lower>().solveInPlace(map);
      VERIFY_IS_APPROX(lhs.triangularView<Lower>().toDenseMatrix() * VecX(map), ref);
    }
  }

  // Complex with non-unit stride: tests conjugation in the scalar fallback path.
  {
    typedef std::complex<double> CScalar;
    typedef Matrix<CScalar, Dynamic, Dynamic> CMatrixX;
    int n = 32;
    CMatrixX lhs = CMatrixX::Random(n, n);
    lhs *= CScalar(0.1);
    lhs.diagonal().array() += CScalar(1.0);

    int cols = 4;
    CMatrixX buffer(2 * n, 2 * cols);
    Map<CMatrixX, 0, Stride<Dynamic, 2> > map(buffer.data(), n, cols, Stride<Dynamic, 2>(2 * n, 2));
    CMatrixX ref(n, cols);

    // Conjugate Lower
    buffer.setZero();
    map.setRandom();
    ref = map;
    lhs.conjugate().triangularView<Lower>().solveInPlace(map);
    VERIFY_IS_APPROX(lhs.conjugate().triangularView<Lower>().toDenseMatrix() * CMatrixX(map), ref);

    // Adjoint Upper
    buffer.setZero();
    map.setRandom();
    ref = map;
    lhs.adjoint().triangularView<Lower>().solveInPlace(map);
    VERIFY_IS_APPROX(lhs.adjoint().triangularView<Lower>().toDenseMatrix() * CMatrixX(map), ref);
  }
}

void trsolve_indexed_view() {
  typedef Matrix<double, Dynamic, Dynamic> MatrixX;
  typedef Matrix<double, Dynamic, 1> VectorX;

  MatrixX lhs = MatrixX::Random(8, 8);
  lhs *= 0.1;
  lhs.diagonal().array() += 1.0;

  VectorX rhs = VectorX::Random(8);
  std::vector<int> indices{0, 1, 2, 7};

  MatrixX lhs_slice = lhs(indices, indices);
  VectorX rhs_slice = rhs(indices);
  VectorX expected = lhs_slice.triangularView<Upper>().solve(rhs_slice);

  VectorX actual = lhs(indices, indices).triangularView<Upper>().solve(rhs(indices));
  VERIFY_IS_APPROX(actual, expected);

  VectorX assigned = VectorX::Random(8);
  VectorX assigned_ref = assigned;
  assigned(indices) = lhs_slice.triangularView<Upper>().solve(rhs_slice);
  assigned_ref(indices) = expected;
  VERIFY_IS_APPROX(assigned, assigned_ref);

  VectorX inplace = rhs;
  VectorX inplace_ref = rhs;
  lhs_slice.triangularView<Upper>().solveInPlace(inplace(indices));
  inplace_ref(indices) = expected;
  VERIFY_IS_APPROX(inplace, inplace_ref);
}

#define VERIFY_TRSM_NO_MALLOC(TRI, XB, REF)                \
  {                                                        \
    (XB) = (REF);                                          \
    internal::set_is_malloc_allowed(false);                \
    (TRI).solveInPlace(XB);                                \
    internal::set_is_malloc_allowed(true);                 \
    VERIFY_IS_APPROX((TRI).toDenseMatrix() * (XB), (REF)); \
  }

#define VERIFY_TRSM_NO_MALLOC_ONTHERIGHT(TRI, XB, REF)     \
  {                                                        \
    (XB) = (REF);                                          \
    internal::set_is_malloc_allowed(false);                \
    (TRI).template solveInPlace<OnTheRight>(XB);           \
    internal::set_is_malloc_allowed(true);                 \
    VERIFY_IS_APPROX((XB) * (TRI).toDenseMatrix(), (REF)); \
  }

// Regression test for issue #3115: while malloc is disallowed, the AVX-512 trsm kernels delegate
// to the unspecialized kernel, which takes an upper-triangular panel by the opposite corner. The
// delegation used to keep the AVX-512 origin and solve outside the panel, reading and writing
// past the ends of both operands.
template <typename Scalar, int TriOptions>
void trsolve_no_malloc(int size, int cols) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, Dynamic, TriOptions> TriMatrix;
  typedef Matrix<Scalar, Dynamic, Dynamic> RhsMatrix;

  TriMatrix lhs = TriMatrix::Random(size, size) * RealScalar(0.1);
  lhs.diagonal().array() += RealScalar(1);

  RhsMatrix ref = RhsMatrix::Random(size, cols);
  RhsMatrix x(size, cols);

  VERIFY_TRSM_NO_MALLOC(lhs.template triangularView<Upper>(), x, ref);
  VERIFY_TRSM_NO_MALLOC(lhs.template triangularView<Lower>(), x, ref);
  VERIFY_TRSM_NO_MALLOC(lhs.template triangularView<UnitUpper>(), x, ref);
  VERIFY_TRSM_NO_MALLOC(lhs.template triangularView<UnitLower>(), x, ref);
  // The adjoint swaps the triangular operand's storage order.
  VERIFY_TRSM_NO_MALLOC(lhs.adjoint().template triangularView<Upper>(), x, ref);
  VERIFY_TRSM_NO_MALLOC(lhs.adjoint().template triangularView<Lower>(), x, ref);

  RhsMatrix refRight = RhsMatrix::Random(cols, size);
  RhsMatrix xRight(cols, size);

  VERIFY_TRSM_NO_MALLOC_ONTHERIGHT(lhs.template triangularView<Upper>(), xRight, refRight);
  VERIFY_TRSM_NO_MALLOC_ONTHERIGHT(lhs.template triangularView<Lower>(), xRight, refRight);
  VERIFY_TRSM_NO_MALLOC_ONTHERIGHT(lhs.adjoint().template triangularView<Upper>(), xRight, refRight);
  VERIFY_TRSM_NO_MALLOC_ONTHERIGHT(lhs.adjoint().template triangularView<Lower>(), xRight, refRight);
}

template <int>
void trsolve_no_malloc_all() {
  // The packed solver walks the triangular matrix in panels of max(mr, nr) columns; these sizes
  // straddle that width for both float and double. They also stay small enough that the solver's
  // blocking buffers come off the stack, since a heap allocation inside the windows below would
  // trip the guard for an unrelated reason.
  const int sizes[] = {1, 4, 17, 20, 24, 25, 40, 64};
  const int colCounts[] = {1, 5, 20};
  for (int size : sizes) {
    for (int cols : colCounts) {
      trsolve_no_malloc<float, ColMajor>(size, cols);
      trsolve_no_malloc<float, RowMajor>(size, cols);
      trsolve_no_malloc<double, ColMajor>(size, cols);
      trsolve_no_malloc<double, RowMajor>(size, cols);
      trsolve_no_malloc<std::complex<double>, ColMajor>(size, cols);
    }
  }
}

// The blocked solvers take their k-block depth, and a solve on the left its column panels, from the
// cache sizes (issue #3162). Both take effect only for operands larger than the cache, which test-sized
// solves are not on most hosts, so the caches are set here for a budget of 520 columns of the operand:
// 1100 right-hand sides then span two full panels and a partial one, and on x86 the depth rises above
// the blocking's on either side. Both storage orders of the right-hand side reach both kernels from
// either side, since the dispatcher transposes a row-major operand.
template <typename Scalar>
void trsolve_panels(int size, int cols, bool allCases) {
  using RealScalar = typename NumTraits<Scalar>::Real;
  using MatrixX = Matrix<Scalar, Dynamic, Dynamic>;
  using RowMatrixX = Matrix<Scalar, Dynamic, Dynamic, RowMajor>;

  const std::ptrdiff_t l1 = 32768, l2 = 65536;
  const std::ptrdiff_t panelL3 = 4 * 520 * std::ptrdiff_t(size) * std::ptrdiff_t(sizeof(Scalar));
  setCpuCacheSizes(l1, l2, panelL3);
  const std::ptrdiff_t budget = internal::triangular_solve_budget<Scalar>(l2, panelL3);
  // Reach: the budget holds fewer columns than the right-hand side has, and the operand exceeds it,
  // which deepens the k-blocks wherever the blocking leaves room.
  VERIFY(internal::triangular_solve_panel_columns(Index(size), Index(cols), budget,
                                                  Index(internal::gebp_traits<Scalar, Scalar>::nr)) < cols);
  VERIFY(std::ptrdiff_t(size) * cols > budget);

  // Scaling the off-diagonal part by 1/size keeps the triangles well conditioned whether the diagonal
  // is used or, for the unit views, taken as 1.
  MatrixX cmLhs = MatrixX::Random(size, size) / RealScalar(size);
  cmLhs.diagonal().array() += RealScalar(1);
  RowMatrixX rmLhs = cmLhs;
  MatrixX cmRhs(size, cols);
  RowMatrixX rmRhs(size, cols);
  MatrixX ref(size, cols);

  VERIFY_TRSM(cmLhs.template triangularView<Lower>(), cmRhs);
  VERIFY_TRSM(cmLhs.template triangularView<Upper>(), cmRhs);
  VERIFY_TRSM(cmLhs.template triangularView<Lower>(), rmRhs);
  VERIFY_TRSM(cmLhs.template triangularView<Upper>(), rmRhs);
  VERIFY_TRSM_ONTHERIGHT(cmLhs.template triangularView<Lower>(), cmRhs);
  VERIFY_TRSM_ONTHERIGHT(cmLhs.template triangularView<Upper>(), cmRhs);
  VERIFY_TRSM_ONTHERIGHT(cmLhs.template triangularView<Lower>(), rmRhs);
  VERIFY_TRSM_ONTHERIGHT(cmLhs.template triangularView<Upper>(), rmRhs);
  if (!allCases) return;

  VERIFY_TRSM(cmLhs.template triangularView<UnitLower>(), cmRhs);
  VERIFY_TRSM(cmLhs.adjoint().template triangularView<Upper>(), rmRhs);
  VERIFY_TRSM(rmLhs.template triangularView<Lower>(), cmRhs);
  VERIFY_TRSM(rmLhs.conjugate().template triangularView<UnitUpper>(), rmRhs);

  VERIFY_TRSM_ONTHERIGHT(cmLhs.template triangularView<UnitUpper>(), rmRhs);
  VERIFY_TRSM_ONTHERIGHT(cmLhs.adjoint().template triangularView<Lower>(), cmRhs);
  VERIFY_TRSM_ONTHERIGHT(rmLhs.template triangularView<Lower>(), rmRhs);
  VERIFY_TRSM_ONTHERIGHT(rmLhs.conjugate().template triangularView<UnitUpper>(), cmRhs);

  // A runtime inner stride, by which the panel origins must not be scaled.
  MatrixX buffer(2 * size, 2 * cols);
  Map<MatrixX, 0, Stride<Dynamic, Dynamic> > cmMap(buffer.data(), size, cols, Stride<Dynamic, Dynamic>(2 * size, 2));
  Map<RowMatrixX, 0, Stride<Dynamic, Dynamic> > rmMap(buffer.data(), size, cols, Stride<Dynamic, Dynamic>(2 * cols, 2));
  buffer.setZero();
  VERIFY_TRSM(cmLhs.template triangularView<Lower>(), cmMap);
  buffer.setZero();
  VERIFY_TRSM(cmLhs.template triangularView<Upper>(), rmMap);
  buffer.setZero();
  VERIFY_TRSM_ONTHERIGHT(cmLhs.template triangularView<Upper>(), cmMap);
  buffer.setZero();
  VERIFY_TRSM_ONTHERIGHT(cmLhs.template triangularView<Lower>(), rmMap);

  // A few rows beside a column-major triangle half of which exceeds the budget: only the triangle makes
  // this solve on the right deep.
  const std::ptrdiff_t triangleL3 = std::ptrdiff_t(size) * size * std::ptrdiff_t(sizeof(Scalar));
  setCpuCacheSizes(l1, l2, triangleL3);
  const std::ptrdiff_t triangleBudget = internal::triangular_solve_budget<Scalar>(l2, triangleL3);
  VERIFY(std::ptrdiff_t(size) * size / 2 > triangleBudget && std::ptrdiff_t(size) * 7 <= triangleBudget);
  RowMatrixX fewRows(size, 7);
  VERIFY_TRSM_ONTHERIGHT(rmLhs.template triangularView<Lower>(), fewRows);
  VERIFY_TRSM_ONTHERIGHT(rmLhs.template triangularView<Upper>(), fewRows);
}

template <int>
void trsolve_panels_all() {
  std::ptrdiff_t l1, l2, l3, l3_per_cpu;
  internal::manage_caching_sizes(GetAction, &l1, &l2, &l3, &l3_per_cpu);
  // 193 raises the depth of every x86 build except SSE2 float and double, which 391 raises; both leave a
  // partial last k-block at every depth they reach.
  trsolve_panels<float>(193, 1100, true);
  trsolve_panels<double>(193, 1100, true);
  trsolve_panels<std::complex<double> >(193, 1100, true);
  trsolve_panels<float>(391, 1100, false);
  trsolve_panels<double>(391, 1100, false);
  internal::manage_caching_sizes(SetAction, &l1, &l2, &l3, &l3_per_cpu);
}

EIGEN_DECLARE_TEST(product_trsolve) {
  for (int i = 0; i < g_repeat; i++) {
    // matrices
    CALL_SUBTEST_1((trsolve<float, Dynamic, Dynamic>(internal::random<int>(1, EIGEN_TEST_MAX_SIZE),
                                                     internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_2((trsolve<double, Dynamic, Dynamic>(internal::random<int>(1, EIGEN_TEST_MAX_SIZE),
                                                      internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_3((trsolve<std::complex<float>, Dynamic, Dynamic>(internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2),
                                                                   internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2))));
    CALL_SUBTEST_4((trsolve<std::complex<double>, Dynamic, Dynamic>(
        internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2), internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2))));

    // vectors
    CALL_SUBTEST_5((trsolve<float, Dynamic, 1>(internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_6((trsolve<double, Dynamic, 1>(internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_7((trsolve<std::complex<float>, Dynamic, 1>(internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_8((trsolve<std::complex<double>, Dynamic, 1>(internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));

    // meta-unrollers
    CALL_SUBTEST_9((trsolve<float, 4, 1>()));
    CALL_SUBTEST_10((trsolve<double, 4, 1>()));
    CALL_SUBTEST_11((trsolve<std::complex<float>, 4, 1>()));
    CALL_SUBTEST_12((trsolve<float, 1, 1>()));
    CALL_SUBTEST_13((trsolve<float, 1, 2>()));
    CALL_SUBTEST_14((trsolve<float, 3, 1>()));
  }

  // Strided solve at blocking boundaries (deterministic, outside g_repeat).
  CALL_SUBTEST_15(trsolve_strided_boundary<0>());
  CALL_SUBTEST_16(trsolve_indexed_view());
  CALL_SUBTEST_17(trsolve_no_malloc_all<0>());
  CALL_SUBTEST_18(trsolve_panels_all<0>());
}
