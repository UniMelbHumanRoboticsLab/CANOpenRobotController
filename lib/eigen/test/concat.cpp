// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Pavel Guzenfeld
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "main.h"

using namespace Eigen;

// ============================================================================
// Test 1: Dynamic-size matrix concatenation (basic correctness)
// ============================================================================
template <typename MatrixType>
void test_concat_dynamic(const MatrixType& m) {
  typedef typename MatrixType::Scalar Scalar;
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixX;

  Index rows = m.rows();
  Index cols = m.cols();

  MatrixType a = MatrixType::Random(rows, cols);
  MatrixType b = MatrixType::Random(rows, cols);

  // Vertical concatenation: stack rows
  {
    MatrixX expected(2 * rows, cols);
    expected.topRows(rows) = a;
    expected.bottomRows(rows) = b;
    VERIFY_IS_APPROX(expected, vcat(a, b).eval());

    // Also verify through assignment to MatrixX
    MatrixX result = vcat(a, b);
    VERIFY_IS_APPROX(expected, result);
  }

  // Horizontal concatenation: stack columns
  {
    MatrixX expected(rows, 2 * cols);
    expected.leftCols(cols) = a;
    expected.rightCols(cols) = b;
    VERIFY_IS_APPROX(expected, hcat(a, b).eval());

    MatrixX result = hcat(a, b);
    VERIFY_IS_APPROX(expected, result);
  }

  // Test with different-sized operands
  {
    MatrixX c = MatrixX::Random(rows + 2, cols);
    MatrixX vresult = vcat(a, c);
    VERIFY_IS_EQUAL(vresult.rows(), 2 * rows + 2);
    VERIFY_IS_EQUAL(vresult.cols(), cols);
    MatrixX vexpected(2 * rows + 2, cols);
    vexpected.topRows(rows) = a;
    vexpected.bottomRows(rows + 2) = c;
    VERIFY_IS_APPROX(vexpected, vresult);
  }

  {
    MatrixX d = MatrixX::Random(rows, cols + 3);
    MatrixX hresult = hcat(a, d);
    VERIFY_IS_EQUAL(hresult.rows(), rows);
    VERIFY_IS_EQUAL(hresult.cols(), 2 * cols + 3);
    MatrixX hexpected(rows, 2 * cols + 3);
    hexpected.leftCols(cols) = a;
    hexpected.rightCols(cols + 3) = d;
    VERIFY_IS_APPROX(hexpected, hresult);
  }
}

// ============================================================================
// Test 2: Expression inputs (not plain objects)
// Verifies that Concat works with CwiseBinaryOp, CwiseUnaryOp, etc.
// ============================================================================
template <typename MatrixType>
void test_concat_with_expressions(const MatrixType& m) {
  typedef typename MatrixType::Scalar Scalar;
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixX;

  Index rows = m.rows();
  Index cols = m.cols();

  MatrixType a = MatrixType::Random(rows, cols);
  MatrixType b = MatrixType::Random(rows, cols);

  // Concat with scalar-multiply expressions
  {
    MatrixX expected(2 * rows, cols);
    expected.topRows(rows) = 2.0 * a;
    expected.bottomRows(rows) = 3.0 * b;
    VERIFY_IS_APPROX(expected, vcat(2.0 * a, 3.0 * b).eval());
  }

  // Concat with sum/difference expressions
  {
    MatrixX expected(rows, 2 * cols);
    expected.leftCols(cols) = a + b;
    expected.rightCols(cols) = a - b;
    VERIFY_IS_APPROX(expected, hcat(a + b, a - b).eval());
  }

  // Concat with transpose expressions
  {
    MatrixX expected(cols, 2 * rows);
    expected.leftCols(rows) = a.transpose();
    expected.rightCols(rows) = b.transpose();
    VERIFY_IS_APPROX(expected, hcat(a.transpose(), b.transpose()).eval());
  }

  // Concat with block expressions
  {
    if (rows >= 2 && cols >= 2) {
      auto block_a = a.topLeftCorner(rows - 1, cols);
      auto block_b = b.bottomRightCorner(1, cols);
      MatrixX expected(rows, cols);
      expected.topRows(rows - 1) = block_a;
      expected.bottomRows(1) = block_b;
      VERIFY_IS_APPROX(expected, vcat(block_a, block_b).eval());
    }
  }
}

// ============================================================================
// Test 3: Fixed-size concat with compile-time dimension propagation
// ============================================================================
template <typename Scalar, int LhsRows, int CommonCols, int RhsRows>
void test_vcat_fixed() {
  typedef Matrix<Scalar, LhsRows, CommonCols> LhsType;
  typedef Matrix<Scalar, RhsRows, CommonCols> RhsType;

  LhsType a = LhsType::Random();
  RhsType b = RhsType::Random();

  auto vc = vcat(a, b);
  VERIFY_IS_EQUAL(vc.rows(), a.rows() + b.rows());
  VERIFY_IS_EQUAL(vc.cols(), a.cols());

  typedef Concat<Vertical, LhsType, RhsType> VConcatType;
  VERIFY((int(VConcatType::RowsAtCompileTime) == LhsRows + RhsRows));
  VERIFY((int(VConcatType::ColsAtCompileTime) == CommonCols));
  VERIFY((int(VConcatType::MaxRowsAtCompileTime) == LhsRows + RhsRows));
  VERIFY((int(VConcatType::MaxColsAtCompileTime) == CommonCols));

  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixX;
  MatrixX expected(a.rows() + b.rows(), a.cols());
  expected.topRows(a.rows()) = a;
  expected.bottomRows(b.rows()) = b;
  VERIFY_IS_APPROX(expected, MatrixX(vc));
}

template <typename Scalar, int CommonRows, int LhsCols, int RhsCols>
void test_hcat_fixed() {
  typedef Matrix<Scalar, CommonRows, LhsCols> LhsType;
  typedef Matrix<Scalar, CommonRows, RhsCols> RhsType;

  LhsType a = LhsType::Random();
  RhsType b = RhsType::Random();

  auto hc = hcat(a, b);
  VERIFY_IS_EQUAL(hc.rows(), a.rows());
  VERIFY_IS_EQUAL(hc.cols(), a.cols() + b.cols());

  typedef Concat<Horizontal, LhsType, RhsType> HConcatType;
  VERIFY((int(HConcatType::RowsAtCompileTime) == CommonRows));
  VERIFY((int(HConcatType::ColsAtCompileTime) == LhsCols + RhsCols));
  VERIFY((int(HConcatType::MaxRowsAtCompileTime) == CommonRows));
  VERIFY((int(HConcatType::MaxColsAtCompileTime) == LhsCols + RhsCols));

  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixX;
  MatrixX expected(a.rows(), a.cols() + b.cols());
  expected.leftCols(a.cols()) = a;
  expected.rightCols(b.cols()) = b;
  VERIFY_IS_APPROX(expected, MatrixX(hc));
}

// ============================================================================
// Test 4: Mixed fixed/dynamic dimension concat
// Verifies that compile-time dimensions become Dynamic when appropriate
// ============================================================================
template <typename Scalar>
void test_concat_mixed_fixed_dynamic() {
  typedef Matrix<Scalar, 3, 3> Fixed33;
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixX;
  typedef Matrix<Scalar, Dynamic, 3> MatrixX3;

  Fixed33 a = Fixed33::Random();
  MatrixX b = MatrixX::Random(4, 3);

  // Vertical: fixed + dynamic => dynamic rows, but cols known at compile time
  {
    auto vc = vcat(a, b);
    VERIFY_IS_EQUAL(vc.rows(), 7);
    VERIFY_IS_EQUAL(vc.cols(), 3);

    // Compile-time: rows should be Dynamic (fixed + dynamic = dynamic)
    typedef decltype(vc) VCType;
    VERIFY((int(VCType::RowsAtCompileTime) == Dynamic));
    // Cols: fixed 3 and dynamic => 3 (size_prefer_fixed picks the fixed one)
    VERIFY((int(VCType::ColsAtCompileTime) == 3));

    MatrixX expected(7, 3);
    expected.topRows(3) = a;
    expected.bottomRows(4) = b;
    VERIFY_IS_APPROX(expected, MatrixX(vc));
  }

  // Vertical: fixed + MatrixX3 (cols both = 3) => dynamic rows
  {
    MatrixX3 c = MatrixX3::Random(5, 3);
    auto vc = vcat(a, c);
    VERIFY_IS_EQUAL(vc.rows(), 8);
    VERIFY_IS_EQUAL(vc.cols(), 3);

    typedef decltype(vc) VCType;
    VERIFY((int(VCType::RowsAtCompileTime) == Dynamic));

    MatrixX expected(8, 3);
    expected.topRows(3) = a;
    expected.bottomRows(5) = c;
    VERIFY_IS_APPROX(expected, MatrixX(vc));
  }

  // Horizontal: fixed + dynamic
  {
    MatrixX d = MatrixX::Random(3, 5);
    auto hc = hcat(a, d);
    VERIFY_IS_EQUAL(hc.rows(), 3);
    VERIFY_IS_EQUAL(hc.cols(), 8);

    typedef decltype(hc) HCType;
    VERIFY((int(HCType::ColsAtCompileTime) == Dynamic));

    MatrixX expected(3, 8);
    expected.leftCols(3) = a;
    expected.rightCols(5) = d;
    VERIFY_IS_APPROX(expected, MatrixX(hc));
  }

  // Dynamic + fixed (reversed order)
  {
    MatrixX3 e = MatrixX3::Random(5, 3);
    auto vc = vcat(e, a);
    VERIFY_IS_EQUAL(vc.rows(), 8);
    VERIFY_IS_EQUAL(vc.cols(), 3);

    MatrixX expected(8, 3);
    expected.topRows(5) = e;
    expected.bottomRows(3) = a;
    VERIFY_IS_APPROX(expected, MatrixX(vc));
  }
}

// ============================================================================
// Test 5: Rvalue temporary lifetime safety
// Verifies ref_selector properly copies temporaries instead of dangling refs
// ============================================================================
template <typename Scalar>
void test_concat_rvalue_temporaries() {
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixX;

  MatrixX a = MatrixX::Random(3, 3);
  MatrixX b = MatrixX::Random(3, 3);

  // Test 5a: Product temporaries (Product expression creates temporaries)
  {
    MatrixX result = vcat(a * MatrixX::Identity(3, 3), b * MatrixX::Identity(3, 3));
    MatrixX expected(6, 3);
    expected.topRows(3) = a;
    expected.bottomRows(3) = b;
    VERIFY_IS_APPROX(expected, result);
  }

  // Test 5b: Additive temporaries
  {
    MatrixX result = hcat(a + MatrixX::Zero(3, 3), b + MatrixX::Zero(3, 3));
    MatrixX expected(3, 6);
    expected.leftCols(3) = a;
    expected.rightCols(3) = b;
    VERIFY_IS_APPROX(expected, result);
  }

  // Test 5c: Store expression with auto and evaluate later
  // This is the critical rvalue lifetime test: the expression must
  // keep the temporary alive until evaluation
  {
    auto expr = vcat(a * MatrixX::Identity(3, 3), b * MatrixX::Identity(3, 3));
    // At this point, the temporaries from a*I and b*I must still be valid
    MatrixX result = expr;
    MatrixX expected(6, 3);
    expected.topRows(3) = a;
    expected.bottomRows(3) = b;
    VERIFY_IS_APPROX(expected, result);
  }

  // Test 5d: Auto with hcat
  {
    auto expr = hcat(a + b, a - b);
    MatrixX result = expr;
    MatrixX expected(3, 6);
    expected.leftCols(3) = a + b;
    expected.rightCols(3) = a - b;
    VERIFY_IS_APPROX(expected, result);
  }

  // Test 5e: Nested auto expressions (chained concat of temporaries)
  {
    MatrixX c = MatrixX::Random(3, 3);
    auto inner = vcat(a + b, c * MatrixX::Identity(3, 3));
    auto outer = vcat(inner, a - b);
    MatrixX result = outer;
    VERIFY_IS_EQUAL(result.rows(), 9);
    VERIFY_IS_EQUAL(result.cols(), 3);
    MatrixX expected(9, 3);
    expected.topRows(3) = a + b;
    expected.middleRows(3, 3) = c;
    expected.bottomRows(3) = a - b;
    VERIFY_IS_APPROX(expected, result);
  }
}

// ============================================================================
// Test 6: Chained concatenation (binary API, no variadic needed)
// ============================================================================
template <typename Scalar>
void test_concat_chained() {
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixX;

  MatrixX a = MatrixX::Random(2, 3);
  MatrixX b = MatrixX::Random(2, 3);
  MatrixX c = MatrixX::Random(2, 3);

  // Chain: vcat(vcat(a, b), c) should be 6x3
  {
    MatrixX result = vcat(vcat(a, b), c);
    VERIFY_IS_EQUAL(result.rows(), 6);
    VERIFY_IS_EQUAL(result.cols(), 3);

    MatrixX expected(6, 3);
    expected.topRows(2) = a;
    expected.middleRows(2, 2) = b;
    expected.bottomRows(2) = c;
    VERIFY_IS_APPROX(expected, result);
  }

  // Chain: hcat(hcat(a, b), c) should be 2x9
  {
    MatrixX result = hcat(hcat(a, b), c);
    VERIFY_IS_EQUAL(result.rows(), 2);
    VERIFY_IS_EQUAL(result.cols(), 9);

    MatrixX expected(2, 9);
    expected.leftCols(3) = a;
    expected.middleCols(3, 3) = b;
    expected.rightCols(3) = c;
    VERIFY_IS_APPROX(expected, result);
  }

  // Mixed chain: vcat then hcat
  {
    MatrixX e = MatrixX::Random(4, 5);
    MatrixX vert = vcat(a, b);  // 4x3
    MatrixX result = hcat(vert, e);
    VERIFY_IS_EQUAL(result.rows(), 4);
    VERIFY_IS_EQUAL(result.cols(), 8);

    MatrixX expected(4, 8);
    expected.leftCols(3) = vert;
    expected.rightCols(5) = e;
    VERIFY_IS_APPROX(expected, result);
  }

  // 4-way chain
  {
    MatrixX d = MatrixX::Random(2, 3);
    MatrixX result = vcat(vcat(vcat(a, b), c), d);
    VERIFY_IS_EQUAL(result.rows(), 8);
    VERIFY_IS_EQUAL(result.cols(), 3);

    MatrixX expected(8, 3);
    expected.block(0, 0, 2, 3) = a;
    expected.block(2, 0, 2, 3) = b;
    expected.block(4, 0, 2, 3) = c;
    expected.block(6, 0, 2, 3) = d;
    VERIFY_IS_APPROX(expected, result);
  }
}

// ============================================================================
// Test 7: Vector concatenation
// ============================================================================
template <typename Scalar>
void test_concat_vectors() {
  typedef Matrix<Scalar, Dynamic, 1> VectorX;
  typedef Matrix<Scalar, 1, Dynamic> RowVectorX;
  typedef Matrix<Scalar, 3, 1> Vector3;
  typedef Matrix<Scalar, 4, 1> Vector4;
  typedef Matrix<Scalar, 1, 3> RowVector3;
  typedef Matrix<Scalar, 1, 4> RowVector4;

  // Column vectors: vcat
  {
    VectorX a = VectorX::Random(3);
    VectorX b = VectorX::Random(5);
    VectorX result = vcat(a, b);
    VERIFY_IS_EQUAL(result.rows(), 8);
    VERIFY_IS_EQUAL(result.cols(), 1);
    VectorX expected(8);
    expected.head(3) = a;
    expected.tail(5) = b;
    VERIFY_IS_APPROX(expected, result);
  }

  // Row vectors: hcat
  {
    RowVectorX a = RowVectorX::Random(3);
    RowVectorX b = RowVectorX::Random(5);
    RowVectorX result = hcat(a, b);
    VERIFY_IS_EQUAL(result.rows(), 1);
    VERIFY_IS_EQUAL(result.cols(), 8);
    RowVectorX expected(8);
    expected.head(3) = a;
    expected.tail(5) = b;
    VERIFY_IS_APPROX(expected, result);
  }

  // Fixed-size column vectors: compile-time dimensions
  {
    Vector3 a = Vector3::Random();
    Vector4 b = Vector4::Random();
    auto result = vcat(a, b);

    typedef Concat<Vertical, Vector3, Vector4> VConcatType;
    VERIFY((int(VConcatType::RowsAtCompileTime) == 7));
    VERIFY((int(VConcatType::ColsAtCompileTime) == 1));

    typedef Matrix<Scalar, Dynamic, 1> VecX;
    VecX expected(7);
    expected.head(3) = a;
    expected.tail(4) = b;
    VecX actual(result);
    VERIFY_IS_APPROX(expected, actual);
  }

  // Fixed-size row vectors: compile-time dimensions
  {
    RowVector3 a = RowVector3::Random();
    RowVector4 b = RowVector4::Random();
    auto result = hcat(a, b);

    typedef Concat<Horizontal, RowVector3, RowVector4> HConcatType;
    VERIFY((int(HConcatType::RowsAtCompileTime) == 1));
    VERIFY((int(HConcatType::ColsAtCompileTime) == 7));

    typedef Matrix<Scalar, 1, Dynamic> RowVecX;
    RowVecX expected(7);
    expected.head(3) = a;
    expected.tail(4) = b;
    RowVecX actual(result);
    VERIFY_IS_APPROX(expected, actual);
  }
}

// ============================================================================
// Test 8: Array concatenation (verifies XprKind propagation)
// ============================================================================
template <typename Scalar>
void test_concat_array() {
  typedef Array<Scalar, Dynamic, Dynamic> ArrayX;
  typedef Array<Scalar, 2, 3> FixedArray23;

  // Dynamic arrays
  {
    ArrayX a = ArrayX::Random(3, 4);
    ArrayX b = ArrayX::Random(3, 4);

    // vcat with arrays
    ArrayX expected(6, 4);
    expected.topRows(3) = a;
    expected.bottomRows(3) = b;
    ArrayX result = vcat(a, b);
    VERIFY_IS_APPROX(expected, result);
  }

  {
    ArrayX a = ArrayX::Random(3, 4);
    ArrayX b = ArrayX::Random(3, 4);

    // hcat with arrays
    ArrayX expected(3, 8);
    expected.leftCols(4) = a;
    expected.rightCols(4) = b;
    ArrayX result = hcat(a, b);
    VERIFY_IS_APPROX(expected, result);
  }

  // Fixed-size arrays
  {
    FixedArray23 a = FixedArray23::Random();
    FixedArray23 b = FixedArray23::Random();

    typedef Concat<Vertical, FixedArray23, FixedArray23> VConcatType;
    VERIFY((int(VConcatType::RowsAtCompileTime) == 4));
    VERIFY((int(VConcatType::ColsAtCompileTime) == 3));

    ArrayX expected(4, 3);
    expected.topRows(2) = a;
    expected.bottomRows(2) = b;
    ArrayX result = vcat(a, b);
    VERIFY_IS_APPROX(expected, result);
  }

  // Array expressions (coefficient-wise operations)
  {
    ArrayX a = ArrayX::Random(3, 4);
    ArrayX b = ArrayX::Random(3, 4);
    ArrayX result = vcat(a * b, a + b);
    VERIFY_IS_EQUAL(result.rows(), 6);
    VERIFY_IS_EQUAL(result.cols(), 4);
    ArrayX expected(6, 4);
    expected.topRows(3) = a * b;
    expected.bottomRows(3) = a + b;
    VERIFY_IS_APPROX(expected, result);
  }
}

// ============================================================================
// Test 9: Coefficient-level access verification
// ============================================================================
template <typename Scalar>
void test_concat_coeff_access() {
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixX;

  MatrixX a = MatrixX::Random(3, 4);
  MatrixX b = MatrixX::Random(2, 4);

  // Verify each coefficient of vcat
  {
    auto vc = vcat(a, b);
    for (Index j = 0; j < 4; ++j) {
      for (Index i = 0; i < 3; ++i) {
        VERIFY_IS_APPROX(vc(i, j), a(i, j));
      }
      for (Index i = 0; i < 2; ++i) {
        VERIFY_IS_APPROX(vc(3 + i, j), b(i, j));
      }
    }
  }

  // Verify each coefficient of hcat
  {
    MatrixX c = MatrixX::Random(3, 2);
    auto hc = hcat(a, c);
    for (Index i = 0; i < 3; ++i) {
      for (Index j = 0; j < 4; ++j) {
        VERIFY_IS_APPROX(hc(i, j), a(i, j));
      }
      for (Index j = 0; j < 2; ++j) {
        VERIFY_IS_APPROX(hc(i, 4 + j), c(i, j));
      }
    }
  }
}

// ============================================================================
// Test 10: Expression used in further Eigen operations
// Verifies Concat integrates with Eigen's expression machinery
// ============================================================================
template <typename Scalar>
void test_concat_in_expressions() {
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixX;
  typedef Matrix<Scalar, Dynamic, 1> VectorX;

  MatrixX a = MatrixX::Random(3, 3);
  MatrixX b = MatrixX::Random(3, 3);

  // Use concat result in matrix multiplication
  {
    MatrixX wide = hcat(a, b);  // 3x6
    VectorX v = VectorX::Random(6);
    VectorX result = wide * v;
    MatrixX wide_eval = wide;
    VectorX expected = wide_eval * v;
    VERIFY_IS_APPROX(expected, result);
  }

  // Use concat result in sum
  {
    auto vc = vcat(a, b);
    Scalar s = vc.sum();
    Scalar expected_sum = a.sum() + b.sum();
    VERIFY_IS_APPROX(expected_sum, s);
  }

  // Use concat in norm computation
  {
    VectorX va = VectorX::Random(4);
    VectorX vb = VectorX::Random(3);
    auto vc = vcat(va, vb);
    Scalar n2 = vc.squaredNorm();
    Scalar expected_n2 = va.squaredNorm() + vb.squaredNorm();
    VERIFY_IS_APPROX(expected_n2, n2);
  }

  // Concat result assigned to a block
  {
    MatrixX target = MatrixX::Zero(6, 3);
    target = vcat(a, b);
    MatrixX expected(6, 3);
    expected.topRows(3) = a;
    expected.bottomRows(3) = b;
    VERIFY_IS_APPROX(expected, target);
  }
}

// ============================================================================
// Test 11: Single row / single column edge cases
// ============================================================================
template <typename Scalar>
void test_concat_single_row_col() {
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixX;
  typedef Matrix<Scalar, 1, Dynamic> RowVectorX;
  typedef Matrix<Scalar, Dynamic, 1> VectorX;

  // Single-row matrices: vcat
  {
    RowVectorX a = RowVectorX::Random(5);
    RowVectorX b = RowVectorX::Random(5);
    MatrixX result = vcat(a, b);
    VERIFY_IS_EQUAL(result.rows(), 2);
    VERIFY_IS_EQUAL(result.cols(), 5);
    VERIFY_IS_APPROX(result.row(0), a);
    VERIFY_IS_APPROX(result.row(1), b);
  }

  // Single-column matrices: hcat
  {
    VectorX a = VectorX::Random(5);
    VectorX b = VectorX::Random(5);
    MatrixX result = hcat(a, b);
    VERIFY_IS_EQUAL(result.rows(), 5);
    VERIFY_IS_EQUAL(result.cols(), 2);
    VERIFY_IS_APPROX(result.col(0), a);
    VERIFY_IS_APPROX(result.col(1), b);
  }

  // 1x1 matrices
  {
    MatrixX a = MatrixX::Random(1, 1);
    MatrixX b = MatrixX::Random(1, 1);
    MatrixX vr = vcat(a, b);
    VERIFY_IS_EQUAL(vr.rows(), 2);
    VERIFY_IS_EQUAL(vr.cols(), 1);
    VERIFY_IS_APPROX(vr(0, 0), a(0, 0));
    VERIFY_IS_APPROX(vr(1, 0), b(0, 0));

    MatrixX hr = hcat(a, b);
    VERIFY_IS_EQUAL(hr.rows(), 1);
    VERIFY_IS_EQUAL(hr.cols(), 2);
    VERIFY_IS_APPROX(hr(0, 0), a(0, 0));
    VERIFY_IS_APPROX(hr(0, 1), b(0, 0));
  }
}

// ============================================================================
// Test 12: RowMajor matrices
// Verifies IsRowMajor flag logic in Concat traits
// ============================================================================
template <typename Scalar>
void test_concat_row_major() {
  typedef Matrix<Scalar, Dynamic, Dynamic, RowMajor> RowMajorMatrix;
  // Explicit ColMajor so the mixed-order cases below stay mixed under EIGEN_DEFAULT_TO_ROW_MAJOR.
  typedef Matrix<Scalar, Dynamic, Dynamic, ColMajor> ColMajorMatrix;
  typedef Matrix<Scalar, 3, 3, RowMajor> FixedRowMajor33;

  // RowMajor + RowMajor
  {
    RowMajorMatrix a = RowMajorMatrix::Random(3, 4);
    RowMajorMatrix b = RowMajorMatrix::Random(2, 4);
    ColMajorMatrix result = vcat(a, b);
    VERIFY_IS_EQUAL(result.rows(), 5);
    VERIFY_IS_EQUAL(result.cols(), 4);
    ColMajorMatrix expected(5, 4);
    expected.topRows(3) = a;
    expected.bottomRows(2) = b;
    VERIFY_IS_APPROX(expected, result);
  }

  {
    RowMajorMatrix a = RowMajorMatrix::Random(3, 4);
    RowMajorMatrix b = RowMajorMatrix::Random(3, 2);
    ColMajorMatrix result = hcat(a, b);
    VERIFY_IS_EQUAL(result.rows(), 3);
    VERIFY_IS_EQUAL(result.cols(), 6);
    ColMajorMatrix expected(3, 6);
    expected.leftCols(4) = a;
    expected.rightCols(2) = b;
    VERIFY_IS_APPROX(expected, result);
  }

  // Mixed: RowMajor + ColMajor
  {
    RowMajorMatrix a = RowMajorMatrix::Random(3, 4);
    ColMajorMatrix b = ColMajorMatrix::Random(2, 4);
    ColMajorMatrix result = vcat(a, b);
    VERIFY_IS_EQUAL(result.rows(), 5);
    VERIFY_IS_EQUAL(result.cols(), 4);
    ColMajorMatrix expected(5, 4);
    expected.topRows(3) = a;
    expected.bottomRows(2) = b;
    VERIFY_IS_APPROX(expected, result);
  }

  // Mixed storage-order operands with the destination matching the Concat's claimed order, so
  // the packet path engages. Regression: packets used to be loaded along the mismatched
  // operand's own inner direction, producing wrong results.
  {
    RowMajorMatrix a = RowMajorMatrix::Random(3, 11);
    ColMajorMatrix b = ColMajorMatrix::Random(2, 11);
    RowMajorMatrix result = vcat(a, b);
    RowMajorMatrix expected(5, 11);
    expected.topRows(3) = a;
    expected.bottomRows(2) = b;
    VERIFY_IS_APPROX(expected, result);

    STATIC_CHECK(!(internal::evaluator<std::decay_t<decltype(vcat(a, b))>>::Flags & PacketAccessBit));
    // With matching operand orders the Concat keeps packet access exactly when the operand
    // evaluators have it; comparing against the operand's own flag keeps this valid on backends
    // where this Scalar is not vectorizable.
    RowMajorMatrix c(2, 11);
    STATIC_CHECK((internal::evaluator<std::decay_t<decltype(vcat(a, c))>>::Flags & PacketAccessBit) ==
                 (internal::evaluator<RowMajorMatrix>::Flags & PacketAccessBit));
  }
  {
    ColMajorMatrix a = ColMajorMatrix::Random(5, 8);
    RowMajorMatrix b = RowMajorMatrix::Random(9, 8);
    ColMajorMatrix result = vcat(a, b);
    ColMajorMatrix expected(14, 8);
    expected.topRows(5) = a;
    expected.bottomRows(9) = b;
    VERIFY_IS_APPROX(expected, result);
  }
  {
    ColMajorMatrix a = ColMajorMatrix::Random(9, 3);
    RowMajorMatrix b = RowMajorMatrix::Random(9, 4);
    ColMajorMatrix result = hcat(a, b);
    ColMajorMatrix expected(9, 7);
    expected.leftCols(3) = a;
    expected.rightCols(4) = b;
    VERIFY_IS_APPROX(expected, result);
  }
  {
    RowMajorMatrix a = RowMajorMatrix::Random(6, 5);
    ColMajorMatrix b = ColMajorMatrix::Random(6, 7);
    RowMajorMatrix result = hcat(a, b);
    RowMajorMatrix expected(6, 12);
    expected.leftCols(5) = a;
    expected.rightCols(7) = b;
    VERIFY_IS_APPROX(expected, result);
  }

  // Fixed-size RowMajor
  {
    FixedRowMajor33 a = FixedRowMajor33::Random();
    FixedRowMajor33 b = FixedRowMajor33::Random();
    ColMajorMatrix result = vcat(a, b);
    VERIFY_IS_EQUAL(result.rows(), 6);
    VERIFY_IS_EQUAL(result.cols(), 3);
    ColMajorMatrix expected(6, 3);
    expected.topRows(3) = a;
    expected.bottomRows(3) = b;
    VERIFY_IS_APPROX(expected, result);
  }
}

// ============================================================================
// Test 13: Self-concatenation and aliasing
// ============================================================================
template <typename Scalar>
void test_concat_self() {
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixX;
  typedef Matrix<Scalar, Dynamic, 1> VectorX;

  // Self-concat: vcat(a, a)
  {
    MatrixX a = MatrixX::Random(3, 4);
    MatrixX result = vcat(a, a);
    VERIFY_IS_EQUAL(result.rows(), 6);
    VERIFY_IS_EQUAL(result.cols(), 4);
    VERIFY_IS_APPROX(result.topRows(3), a);
    VERIFY_IS_APPROX(result.bottomRows(3), a);
  }

  // Self-concat: hcat(a, a)
  {
    MatrixX a = MatrixX::Random(3, 4);
    MatrixX result = hcat(a, a);
    VERIFY_IS_EQUAL(result.rows(), 3);
    VERIFY_IS_EQUAL(result.cols(), 8);
    VERIFY_IS_APPROX(result.leftCols(4), a);
    VERIFY_IS_APPROX(result.rightCols(4), a);
  }

  // Self-concat vector
  {
    VectorX v = VectorX::Random(5);
    VectorX result = vcat(v, v);
    VERIFY_IS_EQUAL(result.rows(), 10);
    VERIFY_IS_APPROX(result.head(5), v);
    VERIFY_IS_APPROX(result.tail(5), v);
  }

  // Concat with self-expression: vcat(a, a.transpose().transpose())
  {
    MatrixX a = MatrixX::Random(3, 4);
    MatrixX result = vcat(a, a + MatrixX::Zero(3, 4));
    VERIFY_IS_EQUAL(result.rows(), 6);
    VERIFY_IS_APPROX(result.topRows(3), a);
    VERIFY_IS_APPROX(result.bottomRows(3), a);
  }
}

// ============================================================================
// Test 14: Runtime assertion for mismatched dimensions
// ============================================================================
template <typename Scalar>
void test_concat_mismatched_dimensions() {
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixX;

  MatrixX a(3, 4);
  MatrixX b(3, 5);
  VERIFY_RAISES_ASSERT(vcat(a, b));  // cols don't match

  MatrixX c(3, 4);
  MatrixX d(2, 4);
  VERIFY_RAISES_ASSERT(hcat(c, d));  // rows don't match
}

// ============================================================================
// Test 15: packetSegment path — covers tail of linear vectorized assignment.
//
// Regression: evaluator<Concat> originally provided packet() but not
// packetSegment(), so AssignEvaluator's tail loop failed to compile on AVX/
// AVX-512 (where has_packet_segment<PacketN> is true). This test exercises:
//   - sizes that are NOT a multiple of the packet width (forces tail loop)
//   - col-major vertical concat with lhs.rows not a multiple of packetSize
//     (forces boundary straddle on the main packet loop)
//   - row-major horizontal concat with lhs.cols not a multiple of packetSize
//     (same, along inner=cols)
//   - chained concat where the inner Concat itself serves as lhs/rhs
// ============================================================================
template <typename Scalar>
void test_concat_packet_segment() {
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixX;
  typedef Matrix<Scalar, Dynamic, Dynamic, RowMajor> RowMajorMatrix;
  typedef Matrix<Scalar, Dynamic, 1> VectorX;

  // Sweep sizes chosen to exercise: inside-lhs, straddle-boundary, inside-rhs
  // for every packet-width scenario (float: 4/8/16, double: 2/4/8).
  for (int lhsInner : {1, 2, 3, 5, 7, 9, 11, 13, 15, 17}) {
    for (int rhsInner : {1, 2, 3, 5, 7, 9, 11}) {
      const int outer = 3;

      // Vertical, col-major: inner=rows, packet extends along rows —
      // may straddle the row boundary at m_lhsRows.
      {
        MatrixX a = MatrixX::Random(lhsInner, outer);
        MatrixX b = MatrixX::Random(rhsInner, outer);
        MatrixX result = vcat(a, b);
        VERIFY_IS_EQUAL(result.rows(), lhsInner + rhsInner);
        VERIFY_IS_EQUAL(result.cols(), outer);
        VERIFY_IS_APPROX(result.topRows(lhsInner), a);
        VERIFY_IS_APPROX(result.bottomRows(rhsInner), b);
      }

      // Horizontal, col-major: inner=rows — packet extends along rows —
      // never crosses col boundary. Verifies non-straddle path still works.
      {
        MatrixX a = MatrixX::Random(lhsInner, 3);
        MatrixX b = MatrixX::Random(lhsInner, 5);
        MatrixX result = hcat(a, b);
        VERIFY_IS_EQUAL(result.rows(), lhsInner);
        VERIFY_IS_EQUAL(result.cols(), 8);
        VERIFY_IS_APPROX(result.leftCols(3), a);
        VERIFY_IS_APPROX(result.rightCols(5), b);
      }

      // Horizontal, row-major: inner=cols, packet extends along cols —
      // may straddle the col boundary at m_lhsCols.
      {
        RowMajorMatrix a = RowMajorMatrix::Random(outer, lhsInner);
        RowMajorMatrix b = RowMajorMatrix::Random(outer, rhsInner);
        RowMajorMatrix result = hcat(a, b);
        VERIFY_IS_EQUAL(result.rows(), outer);
        VERIFY_IS_EQUAL(result.cols(), lhsInner + rhsInner);
        VERIFY_IS_APPROX(result.leftCols(lhsInner), a);
        VERIFY_IS_APPROX(result.rightCols(rhsInner), b);
      }
    }
  }

  // Linear-access path: vcat of column vectors exposes LinearAccessBit,
  // so assignment goes through packetSegment(index, begin, count).
  for (int lhsLen : {1, 3, 5, 7, 9, 15, 17, 31, 33}) {
    for (int rhsLen : {1, 2, 4, 8, 11}) {
      VectorX a = VectorX::Random(lhsLen);
      VectorX b = VectorX::Random(rhsLen);
      VectorX result = vcat(a, b);
      VERIFY_IS_EQUAL(result.size(), lhsLen + rhsLen);
      VERIFY_IS_APPROX(result.head(lhsLen), a);
      VERIFY_IS_APPROX(result.tail(rhsLen), b);
    }
  }

  // Chained concat — packetSegment delegated through multiple Concat layers.
  {
    VectorX a = VectorX::Random(5);
    VectorX b = VectorX::Random(7);
    VectorX c = VectorX::Random(3);
    VectorX result = vcat(vcat(a, b), c);
    VERIFY_IS_EQUAL(result.size(), 15);
    VERIFY_IS_APPROX(result.segment(0, 5), a);
    VERIFY_IS_APPROX(result.segment(5, 7), b);
    VERIFY_IS_APPROX(result.segment(12, 3), c);
  }
}

EIGEN_DECLARE_TEST(concat) {
  for (int i = 0; i < g_repeat; i++) {
    // Dynamic-size matrix concat
    CALL_SUBTEST_1(test_concat_dynamic(MatrixXf(4, 3)));
    CALL_SUBTEST_1(test_concat_dynamic(MatrixXd(5, 7)));
    CALL_SUBTEST_1(test_concat_dynamic(MatrixXcf(3, 4)));

    // Expression inputs
    CALL_SUBTEST_2(test_concat_with_expressions(MatrixXf(4, 3)));
    CALL_SUBTEST_2(test_concat_with_expressions(MatrixXd(5, 7)));

    // Fixed-size concat with compile-time dimension checks
    CALL_SUBTEST_3((test_vcat_fixed<float, 2, 3, 4>()));   // vcat: 2x3 + 4x3
    CALL_SUBTEST_3((test_hcat_fixed<double, 3, 2, 5>()));  // hcat: 3x2 + 3x5
    CALL_SUBTEST_3((test_vcat_fixed<float, 4, 4, 4>()));   // square vcat: 4x4 + 4x4
    CALL_SUBTEST_3((test_hcat_fixed<float, 4, 4, 4>()));   // square hcat: 4x4 + 4x4

    // Mixed fixed/dynamic
    CALL_SUBTEST_4(test_concat_mixed_fixed_dynamic<float>());
    CALL_SUBTEST_4(test_concat_mixed_fixed_dynamic<double>());

    // Rvalue temporaries (lifetime safety)
    CALL_SUBTEST_5(test_concat_rvalue_temporaries<float>());
    CALL_SUBTEST_5(test_concat_rvalue_temporaries<double>());

    // Chained concat (binary API)
    CALL_SUBTEST_6(test_concat_chained<float>());
    CALL_SUBTEST_6(test_concat_chained<double>());

    // Vector concat
    CALL_SUBTEST_7(test_concat_vectors<float>());
    CALL_SUBTEST_7(test_concat_vectors<double>());

    // Array concat (XprKind propagation)
    CALL_SUBTEST_8(test_concat_array<float>());
    CALL_SUBTEST_8(test_concat_array<double>());

    // Coefficient-level access
    CALL_SUBTEST_9(test_concat_coeff_access<float>());
    CALL_SUBTEST_9(test_concat_coeff_access<double>());

    // Integration with other Eigen operations
    CALL_SUBTEST_10(test_concat_in_expressions<float>());
    CALL_SUBTEST_10(test_concat_in_expressions<double>());

    // Single row/column edge cases
    CALL_SUBTEST_11(test_concat_single_row_col<float>());
    CALL_SUBTEST_11(test_concat_single_row_col<double>());

    // RowMajor matrices
    CALL_SUBTEST_12(test_concat_row_major<float>());
    CALL_SUBTEST_12(test_concat_row_major<double>());

    // Self-concatenation and aliasing
    CALL_SUBTEST_13(test_concat_self<float>());
    CALL_SUBTEST_13(test_concat_self<double>());

    // Runtime assertion for mismatched dimensions
    CALL_SUBTEST_14(test_concat_mismatched_dimensions<float>());
    CALL_SUBTEST_14(test_concat_mismatched_dimensions<double>());

    // Packet segment tail path (AVX/AVX-512 has_packet_segment)
    CALL_SUBTEST_15(test_concat_packet_segment<float>());
    CALL_SUBTEST_15(test_concat_packet_segment<double>());
  }
}
