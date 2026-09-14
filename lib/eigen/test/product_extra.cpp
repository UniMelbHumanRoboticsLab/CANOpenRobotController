// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2006-2008 Benoit Jacob <jacob.benoit.1@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "main.h"

template <typename MatrixType>
void product_extra(const MatrixType& m) {
  typedef typename MatrixType::Scalar Scalar;
  typedef Matrix<Scalar, 1, Dynamic> RowVectorType;
  typedef Matrix<Scalar, Dynamic, 1> ColVectorType;
  typedef Matrix<Scalar, Dynamic, Dynamic, MatrixType::Flags & RowMajorBit> OtherMajorMatrixType;

  Index rows = m.rows();
  Index cols = m.cols();

  MatrixType m1 = MatrixType::Random(rows, cols), m2 = MatrixType::Random(rows, cols), m3(rows, cols),
             mzero = MatrixType::Zero(rows, cols), identity = MatrixType::Identity(rows, rows),
             square = MatrixType::Random(rows, rows), res = MatrixType::Random(rows, rows),
             square2 = MatrixType::Random(cols, cols), res2 = MatrixType::Random(cols, cols);
  RowVectorType v1 = RowVectorType::Random(rows), vrres(rows);
  ColVectorType vc2 = ColVectorType::Random(cols), vcres(cols);
  OtherMajorMatrixType tm1 = m1;

  Scalar s1 = internal::random<Scalar>(), s2 = internal::random<Scalar>(), s3 = internal::random<Scalar>();

  VERIFY_IS_APPROX(m3.noalias() = m1 * m2.adjoint(), m1 * m2.adjoint().eval());
  VERIFY_IS_APPROX(m3.noalias() = m1.adjoint() * square.adjoint(), m1.adjoint().eval() * square.adjoint().eval());
  VERIFY_IS_APPROX(m3.noalias() = m1.adjoint() * m2, m1.adjoint().eval() * m2);
  VERIFY_IS_APPROX(m3.noalias() = (s1 * m1.adjoint()) * m2, (s1 * m1.adjoint()).eval() * m2);
  VERIFY_IS_APPROX(m3.noalias() = ((s1 * m1).adjoint()) * m2, (numext::conj(s1) * m1.adjoint()).eval() * m2);
  VERIFY_IS_APPROX(m3.noalias() = (-m1.adjoint() * s1) * (s3 * m2), (-m1.adjoint() * s1).eval() * (s3 * m2).eval());
  VERIFY_IS_APPROX(m3.noalias() = (s2 * m1.adjoint() * s1) * m2, (s2 * m1.adjoint() * s1).eval() * m2);
  VERIFY_IS_APPROX(m3.noalias() = (-m1 * s2) * s1 * m2.adjoint(), (-m1 * s2).eval() * (s1 * m2.adjoint()).eval());

  // a very tricky case where a scale factor has to be automatically conjugated:
  VERIFY_IS_APPROX(m1.adjoint() * (s1 * m2).conjugate(), (m1.adjoint()).eval() * ((s1 * m2).conjugate()).eval());

  // test all possible conjugate combinations for the four matrix-vector product cases:

  VERIFY_IS_APPROX((-m1.conjugate() * s2) * (s1 * vc2), (-m1.conjugate() * s2).eval() * (s1 * vc2).eval());
  VERIFY_IS_APPROX((-m1 * s2) * (s1 * vc2.conjugate()), (-m1 * s2).eval() * (s1 * vc2.conjugate()).eval());
  VERIFY_IS_APPROX((-m1.conjugate() * s2) * (s1 * vc2.conjugate()),
                   (-m1.conjugate() * s2).eval() * (s1 * vc2.conjugate()).eval());

  VERIFY_IS_APPROX((s1 * vc2.transpose()) * (-m1.adjoint() * s2),
                   (s1 * vc2.transpose()).eval() * (-m1.adjoint() * s2).eval());
  VERIFY_IS_APPROX((s1 * vc2.adjoint()) * (-m1.transpose() * s2),
                   (s1 * vc2.adjoint()).eval() * (-m1.transpose() * s2).eval());
  VERIFY_IS_APPROX((s1 * vc2.adjoint()) * (-m1.adjoint() * s2),
                   (s1 * vc2.adjoint()).eval() * (-m1.adjoint() * s2).eval());

  VERIFY_IS_APPROX((-m1.adjoint() * s2) * (s1 * v1.transpose()),
                   (-m1.adjoint() * s2).eval() * (s1 * v1.transpose()).eval());
  VERIFY_IS_APPROX((-m1.transpose() * s2) * (s1 * v1.adjoint()),
                   (-m1.transpose() * s2).eval() * (s1 * v1.adjoint()).eval());
  VERIFY_IS_APPROX((-m1.adjoint() * s2) * (s1 * v1.adjoint()),
                   (-m1.adjoint() * s2).eval() * (s1 * v1.adjoint()).eval());

  VERIFY_IS_APPROX((s1 * v1) * (-m1.conjugate() * s2), (s1 * v1).eval() * (-m1.conjugate() * s2).eval());
  VERIFY_IS_APPROX((s1 * v1.conjugate()) * (-m1 * s2), (s1 * v1.conjugate()).eval() * (-m1 * s2).eval());
  VERIFY_IS_APPROX((s1 * v1.conjugate()) * (-m1.conjugate() * s2),
                   (s1 * v1.conjugate()).eval() * (-m1.conjugate() * s2).eval());

  VERIFY_IS_APPROX((-m1.adjoint() * s2) * (s1 * v1.adjoint()),
                   (-m1.adjoint() * s2).eval() * (s1 * v1.adjoint()).eval());

  // test the vector-matrix product with non aligned starts
  Index i = internal::random<Index>(0, m1.rows() - 2);
  Index j = internal::random<Index>(0, m1.cols() - 2);
  Index r = internal::random<Index>(1, m1.rows() - i);
  Index c = internal::random<Index>(1, m1.cols() - j);
  Index i2 = internal::random<Index>(0, m1.rows() - 1);
  Index j2 = internal::random<Index>(0, m1.cols() - 1);

  VERIFY_IS_APPROX(m1.col(j2).adjoint() * m1.block(0, j, m1.rows(), c),
                   m1.col(j2).adjoint().eval() * m1.block(0, j, m1.rows(), c).eval());
  VERIFY_IS_APPROX(m1.block(i, 0, r, m1.cols()) * m1.row(i2).adjoint(),
                   m1.block(i, 0, r, m1.cols()).eval() * m1.row(i2).adjoint().eval());

  // test negative strides
  {
    Map<MatrixType, Unaligned, Stride<Dynamic, Dynamic> > map1(&m1(rows - 1, cols - 1), rows, cols,
                                                               Stride<Dynamic, Dynamic>(-m1.outerStride(), -1));
    Map<MatrixType, Unaligned, Stride<Dynamic, Dynamic> > map2(&m2(rows - 1, cols - 1), rows, cols,
                                                               Stride<Dynamic, Dynamic>(-m2.outerStride(), -1));
    Map<RowVectorType, Unaligned, InnerStride<-1> > mapv1(&v1(v1.size() - 1), v1.size(), InnerStride<-1>(-1));
    Map<ColVectorType, Unaligned, InnerStride<-1> > mapvc2(&vc2(vc2.size() - 1), vc2.size(), InnerStride<-1>(-1));
    VERIFY_IS_APPROX(MatrixType(map1), m1.reverse());
    VERIFY_IS_APPROX(MatrixType(map2), m2.reverse());
    VERIFY_IS_APPROX(m3.noalias() = MatrixType(map1) * MatrixType(map2).adjoint(),
                     m1.reverse() * m2.reverse().adjoint());
    VERIFY_IS_APPROX(m3.noalias() = map1 * map2.adjoint(), m1.reverse() * m2.reverse().adjoint());
    VERIFY_IS_APPROX(map1 * vc2, m1.reverse() * vc2);
    VERIFY_IS_APPROX(m1 * mapvc2, m1 * mapvc2);
    VERIFY_IS_APPROX(map1.adjoint() * v1.transpose(), m1.adjoint().reverse() * v1.transpose());
    VERIFY_IS_APPROX(m1.adjoint() * mapv1.transpose(), m1.adjoint() * v1.reverse().transpose());
  }

  // regression test
  MatrixType tmp = m1 * m1.adjoint() * s1;
  VERIFY_IS_APPROX(tmp, m1 * m1.adjoint() * s1);

  // regression test for bug 1343, assignment to arrays
  Array<Scalar, Dynamic, 1> a1 = m1 * vc2;
  VERIFY_IS_APPROX(a1.matrix(), m1 * vc2);
  Array<Scalar, Dynamic, 1> a2 = s1 * (m1 * vc2);
  VERIFY_IS_APPROX(a2.matrix(), s1 * m1 * vc2);
  Array<Scalar, 1, Dynamic> a3 = v1 * m1;
  VERIFY_IS_APPROX(a3.matrix(), v1 * m1);
  Array<Scalar, Dynamic, Dynamic> a4 = m1 * m2.adjoint();
  VERIFY_IS_APPROX(a4.matrix(), m1 * m2.adjoint());
}

// Regression test for bug reported at http://forum.kde.org/viewtopic.php?f=74&t=96947
void mat_mat_scalar_scalar_product() {
  Eigen::Matrix2Xd dNdxy(2, 3);
  dNdxy << -0.5, 0.5, 0, -0.3, 0, 0.3;
  double det = 6.0, wt = 0.5;
  VERIFY_IS_APPROX(dNdxy.transpose() * dNdxy * det * wt, det * wt * dNdxy.transpose() * dNdxy);
}

template <typename MatrixType>
void zero_sized_objects(const MatrixType& m) {
  typedef typename MatrixType::Scalar Scalar;
  const int PacketSize = internal::packet_traits<Scalar>::size;
  constexpr int PacketSize1 = PacketSize > 1 ? PacketSize - 1 : 1;
  Index rows = m.rows();
  Index cols = m.cols();

  {
    MatrixType res, a(rows, 0), b(0, cols);
    VERIFY_IS_APPROX((res = a * b), MatrixType::Zero(rows, cols));
    VERIFY_IS_APPROX((res = a * a.transpose()), MatrixType::Zero(rows, rows));
    VERIFY_IS_APPROX((res = b.transpose() * b), MatrixType::Zero(cols, cols));
    VERIFY_IS_APPROX((res = b.transpose() * a.transpose()), MatrixType::Zero(cols, rows));
  }

  {
    MatrixType res, a(rows, cols), b(cols, 0);
    res = a * b;
    VERIFY(res.rows() == rows && res.cols() == 0);
    b.resize(0, rows);
    res = b * a;
    VERIFY(res.rows() == 0 && res.cols() == cols);
  }

  {
    Matrix<Scalar, PacketSize, 0> a;
    Matrix<Scalar, 0, 1> b;
    Matrix<Scalar, PacketSize, 1> res;
    VERIFY_IS_APPROX((res = a * b), MatrixType::Zero(PacketSize, 1));
    VERIFY_IS_APPROX((res = a.lazyProduct(b)), MatrixType::Zero(PacketSize, 1));
  }

  {
    Matrix<Scalar, PacketSize1, 0> a;
    Matrix<Scalar, 0, 1> b;
    Matrix<Scalar, PacketSize1, 1> res;
    VERIFY_IS_APPROX((res = a * b), MatrixType::Zero(PacketSize1, 1));
    VERIFY_IS_APPROX((res = a.lazyProduct(b)), MatrixType::Zero(PacketSize1, 1));
  }

  {
    Matrix<Scalar, PacketSize, Dynamic> a(PacketSize, 0);
    Matrix<Scalar, Dynamic, 1> b(0, 1);
    Matrix<Scalar, PacketSize, 1> res;
    VERIFY_IS_APPROX((res = a * b), MatrixType::Zero(PacketSize, 1));
    VERIFY_IS_APPROX((res = a.lazyProduct(b)), MatrixType::Zero(PacketSize, 1));
  }

  {
    Matrix<Scalar, PacketSize1, Dynamic> a(PacketSize1, 0);
    Matrix<Scalar, Dynamic, 1> b(0, 1);
    Matrix<Scalar, PacketSize1, 1> res;
    VERIFY_IS_APPROX((res = a * b), MatrixType::Zero(PacketSize1, 1));
    VERIFY_IS_APPROX((res = a.lazyProduct(b)), MatrixType::Zero(PacketSize1, 1));
  }
}

template <int>
void bug_127() {
  // Bug 127
  //
  // a product of the form lhs*rhs with
  //
  // lhs:
  // rows = 1, cols = 4
  // RowsAtCompileTime = 1, ColsAtCompileTime = -1
  // MaxRowsAtCompileTime = 1, MaxColsAtCompileTime = 5
  //
  // rhs:
  // rows = 4, cols = 0
  // RowsAtCompileTime = -1, ColsAtCompileTime = -1
  // MaxRowsAtCompileTime = 5, MaxColsAtCompileTime = 1
  //
  // was failing on a runtime assertion, because it had been mis-compiled as a dot product because Product.h was using
  // the max-sizes to detect size 1 indicating vectors, and that didn't account for 0-sized object with max-size 1.

  Matrix<float, 1, Dynamic, RowMajor, 1, 5> a(1, 4);
  Matrix<float, Dynamic, Dynamic, ColMajor, 5, 1> b(4, 0);
  a* b;
}

template <int>
void bug_817() {
  ArrayXXf B = ArrayXXf::Random(10, 10), C;
  VectorXf x = VectorXf::Random(10);
  C = (x.transpose() * B.matrix());
  B = (x.transpose() * B.matrix());
  VERIFY_IS_APPROX(B, C);
}

template <int>
void triangular_product_assignment_size_mismatch() {
  MatrixXd m1 = MatrixXd::Random(5, 5);
  MatrixXd m2(6, 6);
  VERIFY_RAISES_ASSERT(m2.triangularView<Lower>() = m1 * m1);
}

template <int>
void unaligned_objects() {
  // Regression test for the bug reported here:
  // http://forum.kde.org/viewtopic.php?f=74&t=107541
  // Recall the matrix*vector kernel avoid unaligned loads by loading two packets and then reassemble then.
  // There was a mistake in the computation of the valid range for fully unaligned objects: in some rare cases,
  // memory was read outside the allocated matrix memory. Though the values were not used, this might raise segfault.
  for (int m = 450; m < 460; ++m) {
    for (int n = 8; n < 12; ++n) {
      MatrixXf M(m, n);
      VectorXf v1(n), r1(500);
      RowVectorXf v2(m), r2(16);

      M.setRandom();
      v1.setRandom();
      v2.setRandom();
      for (int o = 0; o < 4; ++o) {
        r1.segment(o, m).noalias() = M * v1;
        VERIFY_IS_APPROX(r1.segment(o, m), M * MatrixXf(v1));
        r2.segment(o, n).noalias() = v2 * M;
        VERIFY_IS_APPROX(r2.segment(o, n), MatrixXf(v2) * M);
      }
    }
  }
}

template <typename T>
EIGEN_DONT_INLINE Index test_compute_block_size(Index m, Index n, Index k) {
  Index mc(m), nc(n), kc(k);
  internal::computeProductBlockingSizes<T, T>(kc, mc, nc);
  return kc + mc + nc;
}

template <typename T>
Index compute_block_size() {
  Index ret = 0;
  // Zero-sized inputs: verify they compile and don't crash.
  ret += test_compute_block_size<T>(0, 1, 1);
  ret += test_compute_block_size<T>(1, 0, 1);
  ret += test_compute_block_size<T>(1, 1, 0);
  ret += test_compute_block_size<T>(0, 0, 1);
  ret += test_compute_block_size<T>(0, 1, 0);
  ret += test_compute_block_size<T>(1, 0, 0);
  ret += test_compute_block_size<T>(0, 0, 0);

  // Sanity checks: blocking sizes must be positive and not exceed the original.
  {
    Index m = 200, n = 200, k = 200;
    Index mc = m, nc = n, kc = k;
    internal::computeProductBlockingSizes<T, T>(kc, mc, nc);
    VERIFY(kc > 0 && kc <= k);
    VERIFY(mc > 0 && mc <= m);
    VERIFY(nc > 0 && nc <= n);
  }
  // With EIGEN_DEBUG_SMALL_PRODUCT_BLOCKS (l1=9KB, l2=32KB, l3=512KB),
  // large sizes must be actually blocked (not returned as-is).
  {
    Index m = 500, n = 500, k = 500;
    Index mc = m, nc = n, kc = k;
    internal::computeProductBlockingSizes<T, T>(kc, mc, nc);
    VERIFY(kc < k);
  }

  return ret;
}

// Verify correctness of GEMM at sizes that require multiple blocking passes
// under EIGEN_DEBUG_SMALL_PRODUCT_BLOCKS (l1=9KB, l2=32KB, l3=512KB).
// The blocking early-return threshold is max(k,m,n) < 48, so sizes >= 48
// trigger actual multi-pass blocking with these tiny cache sizes.
// Verifies GEMM against column-by-column GEMV (a different code path).
template <int>
void test_small_block_correctness() {
  const int sizes[] = {48, 64, 96, 128, 200};
  for (int si = 0; si < 5; ++si) {
    int n = sizes[si];
    MatrixXd A = MatrixXd::Random(n, n);
    MatrixXd B = MatrixXd::Random(n, n);
    MatrixXd C(n, n);
    C.noalias() = A * B;
    MatrixXd Cref(n, n);
    for (int j = 0; j < n; ++j) Cref.col(j) = A * B.col(j);
    VERIFY_IS_APPROX(C, Cref);
  }
  // Non-square: exercise different blocking in m, n, k dimensions.
  {
    MatrixXd A = MatrixXd::Random(200, 64);
    MatrixXd B = MatrixXd::Random(64, 300);
    MatrixXd C(200, 300);
    C.noalias() = A * B;
    MatrixXd Cref(200, 300);
    for (int j = 0; j < 300; ++j) Cref.col(j) = A * B.col(j);
    VERIFY_IS_APPROX(C, Cref);
  }
}

template <typename>
void aliasing_with_resize() {
  Index m = internal::random<Index>(10, 50);
  Index n = internal::random<Index>(10, 50);
  MatrixXd A, B, C(m, n), D(m, m);
  VectorXd a, b, c(n);
  C.setRandom();
  D.setRandom();
  c.setRandom();
  double s = internal::random<double>(1, 10);

  A = C;
  B = A * A.transpose();
  A = A * A.transpose();
  VERIFY_IS_APPROX(A, B);

  A = C;
  B = (A * A.transpose()) / s;
  A = (A * A.transpose()) / s;
  VERIFY_IS_APPROX(A, B);

  A = C;
  B = (A * A.transpose()) + D;
  A = (A * A.transpose()) + D;
  VERIFY_IS_APPROX(A, B);

  A = C;
  B = D + (A * A.transpose());
  A = D + (A * A.transpose());
  VERIFY_IS_APPROX(A, B);

  A = C;
  B = s * (A * A.transpose());
  A = s * (A * A.transpose());
  VERIFY_IS_APPROX(A, B);

  A = C;
  a = c;
  b = (A * a) / s;
  a = (A * a) / s;
  VERIFY_IS_APPROX(a, b);
}

template <int>
void bug_1308() {
  int n = 10;
  MatrixXd r(n, n);
  VectorXd v = VectorXd::Random(n);
  r = v * RowVectorXd::Ones(n);
  VERIFY_IS_APPROX(r, v.rowwise().replicate(n));
  r = VectorXd::Ones(n) * v.transpose();
  VERIFY_IS_APPROX(r, v.rowwise().replicate(n).transpose());

  Matrix4d ones44 = Matrix4d::Ones();
  Matrix4d m44 = Matrix4d::Ones() * Matrix4d::Ones();
  VERIFY_IS_APPROX(m44, Matrix4d::Constant(4));
  VERIFY_IS_APPROX(m44.noalias() = ones44 * Matrix4d::Ones(), Matrix4d::Constant(4));
  VERIFY_IS_APPROX(m44.noalias() = ones44.transpose() * Matrix4d::Ones(), Matrix4d::Constant(4));
  VERIFY_IS_APPROX(m44.noalias() = Matrix4d::Ones() * ones44, Matrix4d::Constant(4));
  VERIFY_IS_APPROX(m44.noalias() = Matrix4d::Ones() * ones44.transpose(), Matrix4d::Constant(4));

  typedef Matrix<double, 4, 4, RowMajor> RMatrix4d;
  RMatrix4d r44 = Matrix4d::Ones() * Matrix4d::Ones();
  VERIFY_IS_APPROX(r44, Matrix4d::Constant(4));
  VERIFY_IS_APPROX(r44.noalias() = ones44 * Matrix4d::Ones(), Matrix4d::Constant(4));
  VERIFY_IS_APPROX(r44.noalias() = ones44.transpose() * Matrix4d::Ones(), Matrix4d::Constant(4));
  VERIFY_IS_APPROX(r44.noalias() = Matrix4d::Ones() * ones44, Matrix4d::Constant(4));
  VERIFY_IS_APPROX(r44.noalias() = Matrix4d::Ones() * ones44.transpose(), Matrix4d::Constant(4));
  VERIFY_IS_APPROX(r44.noalias() = ones44 * RMatrix4d::Ones(), Matrix4d::Constant(4));
  VERIFY_IS_APPROX(r44.noalias() = ones44.transpose() * RMatrix4d::Ones(), Matrix4d::Constant(4));
  VERIFY_IS_APPROX(r44.noalias() = RMatrix4d::Ones() * ones44, Matrix4d::Constant(4));
  VERIFY_IS_APPROX(r44.noalias() = RMatrix4d::Ones() * ones44.transpose(), Matrix4d::Constant(4));

  //   RowVector4d r4;
  m44.setOnes();
  r44.setZero();
  VERIFY_IS_APPROX(r44.noalias() += m44.row(0).transpose() * RowVector4d::Ones(), ones44);
  r44.setZero();
  VERIFY_IS_APPROX(r44.noalias() += m44.col(0) * RowVector4d::Ones(), ones44);
  r44.setZero();
  VERIFY_IS_APPROX(r44.noalias() += Vector4d::Ones() * m44.row(0), ones44);
  r44.setZero();
  VERIFY_IS_APPROX(r44.noalias() += Vector4d::Ones() * m44.col(0).transpose(), ones44);
}

// Regression test for issue #3059: GEBP asm register constraints fail
// for custom (non-vectorizable) scalar types. Type T has a non-trivial
// destructor (making sizeof(T) > sizeof(double)), while type U is a
// simple wrapper. Both must compile and produce correct products.
namespace issue_3059 {

class Ptr {
 public:
  ~Ptr() {}
  double* m_ptr = nullptr;
};

class T {
 public:
  T() = default;
  T(double v) : m_value(v) {}

  friend T operator*(const T& a, const T& b) { return T(a.m_value * b.m_value); }
  T& operator*=(const T& o) {
    m_value *= o.m_value;
    return *this;
  }
  friend T operator/(const T& a, const T& b) { return T(a.m_value / b.m_value); }
  T& operator/=(const T& o) {
    m_value /= o.m_value;
    return *this;
  }
  friend T operator+(const T& a, const T& b) { return T(a.m_value + b.m_value); }
  T& operator+=(const T& o) {
    m_value += o.m_value;
    return *this;
  }
  friend T operator-(const T& a, const T& b) { return T(a.m_value - b.m_value); }
  T& operator-=(const T& o) {
    m_value -= o.m_value;
    return *this;
  }
  friend T operator-(const T& a) { return T(-a.m_value); }

  bool operator==(const T& o) const { return m_value == o.m_value; }
  bool operator<(const T& o) const { return m_value < o.m_value; }
  bool operator<=(const T& o) const { return m_value <= o.m_value; }
  bool operator>(const T& o) const { return m_value > o.m_value; }
  bool operator>=(const T& o) const { return m_value >= o.m_value; }
  bool operator!=(const T& o) const { return m_value != o.m_value; }

  double value() const { return m_value; }

 private:
  double m_value = 0.0;
  Ptr m_ptr;  // Makes sizeof(T) > sizeof(double)
};

T sqrt(const T& x) { return T(std::sqrt(x.value())); }
T abs(const T& x) { return T(std::abs(x.value())); }
T abs2(const T& x) { return T(x.value() * x.value()); }

class U {
 public:
  U() = default;
  U(double v) : m_value(v) {}

  friend U operator*(const U& a, const U& b) { return U(a.m_value * b.m_value); }
  U& operator*=(const U& o) {
    m_value *= o.m_value;
    return *this;
  }
  friend U operator/(const U& a, const U& b) { return U(a.m_value / b.m_value); }
  U& operator/=(const U& o) {
    m_value /= o.m_value;
    return *this;
  }
  friend U operator+(const U& a, const U& b) { return U(a.m_value + b.m_value); }
  U& operator+=(const U& o) {
    m_value += o.m_value;
    return *this;
  }
  friend U operator-(const U& a, const U& b) { return U(a.m_value - b.m_value); }
  U& operator-=(const U& o) {
    m_value -= o.m_value;
    return *this;
  }
  friend U operator-(const U& a) { return U(-a.m_value); }

  bool operator==(const U& o) const { return m_value == o.m_value; }
  bool operator<(const U& o) const { return m_value < o.m_value; }
  bool operator<=(const U& o) const { return m_value <= o.m_value; }
  bool operator>(const U& o) const { return m_value > o.m_value; }
  bool operator>=(const U& o) const { return m_value >= o.m_value; }
  bool operator!=(const U& o) const { return m_value != o.m_value; }

  double value() const { return m_value; }

 private:
  double m_value = 0.0;
};

U sqrt(const U& x) { return U(std::sqrt(x.value())); }
U abs(const U& x) { return U(std::abs(x.value())); }
U abs2(const U& x) { return U(x.value() * x.value()); }

}  // namespace issue_3059

namespace Eigen {

template <>
struct NumTraits<issue_3059::T> : NumTraits<double> {
  using Real = issue_3059::T;
  using NonInteger = issue_3059::T;
  using Nested = issue_3059::T;
  enum { IsComplex = 0, RequireInitialization = 1 };
};

template <>
struct NumTraits<issue_3059::U> : NumTraits<double> {
  using Real = issue_3059::U;
  using NonInteger = issue_3059::U;
  using Nested = issue_3059::U;
  enum { IsComplex = 0, RequireInitialization = 0 };
};

}  // namespace Eigen

template <int>
void product_custom_scalar_types() {
  using namespace issue_3059;
  // Type T: has non-trivial destructor, sizeof(T) > sizeof(double)
  {
    Matrix<T, Dynamic, Dynamic> A(4, 4), B(4, 4), C(4, 4);
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j) {
        A(i, j) = T(static_cast<double>(i + 1));
        B(i, j) = T(static_cast<double>(j + 1));
      }
    C.noalias() = A * B;
    // A*B: C(i,j) = sum_k (i+1)*(k+1) * ... no, A(i,k)=(i+1), B(k,j)=(j+1)
    // so C(i,j) = sum_k (i+1)*(j+1) = 4*(i+1)*(j+1)
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j) VERIFY(C(i, j) == T(4.0 * (i + 1) * (j + 1)));
  }
  // Type U: simple wrapper, sizeof(U) == sizeof(double)
  {
    Matrix<U, Dynamic, Dynamic> A(4, 4), B(4, 4), C(4, 4);
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j) {
        A(i, j) = U(static_cast<double>(i + 1));
        B(i, j) = U(static_cast<double>(j + 1));
      }
    C.noalias() = A * B;
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j) VERIFY(C(i, j) == U(4.0 * (i + 1) * (j + 1)));
  }
  // Larger matrices to exercise GEBP blocking.
  {
    const int n = 33;
    Matrix<U, Dynamic, Dynamic> A(n, n), B(n, n), C(n, n);
    for (int i = 0; i < n; ++i)
      for (int j = 0; j < n; ++j) {
        A(i, j) = U(static_cast<double>((i * 7 + j * 3) % 13));
        B(i, j) = U(static_cast<double>((i * 5 + j * 11) % 17));
      }
    C.noalias() = A * B;
    // Verify against explicit triple loop.
    for (int i = 0; i < n; ++i)
      for (int j = 0; j < n; ++j) {
        double sum = 0;
        for (int k = 0; k < n; ++k) sum += A(i, k).value() * B(k, j).value();
        VERIFY(C(i, j) == U(sum));
      }
  }
}

// Test complex GEMV with all conjugation combinations at sizes that
// exercise full, half, and quarter packet code paths.
// The GEMV kernels in GeneralMatrixVector.h use conj_helper at three
// packet levels. The existing product_extra tests cover conjugation
// but only at random sizes, never systematically at packet boundaries.
template <int>
void gemv_complex_conjugate() {
  typedef std::complex<float> Scf;
  typedef std::complex<double> Scd;
  const Index PS_f = internal::packet_traits<Scf>::size;
  const Index PS_d = internal::packet_traits<Scd>::size;

  // Sizes chosen to exercise packet boundaries for both float and double.
  const Index sizes[] = {1, 2, 3, 4, 5, 7, 8, 9, 15, 16, 17, 31, 32, 33};

  for (int si = 0; si < 14; ++si) {
    Index m = sizes[si];
    // Test complex<float> GEMV with all conjugation combos.
    {
      typedef Matrix<Scf, Dynamic, Dynamic> Mat;
      typedef Matrix<Scf, Dynamic, 1> Vec;
      Mat A = Mat::Random(m, m);
      Vec v = Vec::Random(m);
      Vec res(m);

      // A * v (no conjugation)
      res.noalias() = A * v;
      VERIFY_IS_APPROX(res, (A.eval() * v.eval()).eval());

      // A.conjugate() * v
      res.noalias() = A.conjugate() * v;
      VERIFY_IS_APPROX(res, (A.conjugate().eval() * v.eval()).eval());

      // A * v.conjugate()
      res.noalias() = A * v.conjugate();
      VERIFY_IS_APPROX(res, (A.eval() * v.conjugate().eval()).eval());

      // A.conjugate() * v.conjugate()
      res.noalias() = A.conjugate() * v.conjugate();
      VERIFY_IS_APPROX(res, (A.conjugate().eval() * v.conjugate().eval()).eval());

      // A.adjoint() * v (transpose + conjugate of lhs)
      Vec res2(m);
      res2.noalias() = A.adjoint() * v;
      VERIFY_IS_APPROX(res2, (A.adjoint().eval() * v.eval()).eval());

      // Row-major complex GEMV
      typedef Matrix<Scf, Dynamic, Dynamic, RowMajor> RMat;
      RMat B = A;
      res.noalias() = B * v;
      VERIFY_IS_APPROX(res, (A.eval() * v.eval()).eval());

      res.noalias() = B.conjugate() * v;
      VERIFY_IS_APPROX(res, (A.conjugate().eval() * v.eval()).eval());
    }

    // Test complex<double> GEMV with conjugation.
    {
      typedef Matrix<Scd, Dynamic, Dynamic> Mat;
      typedef Matrix<Scd, Dynamic, 1> Vec;
      Mat A = Mat::Random(m, m);
      Vec v = Vec::Random(m);
      Vec res(m);

      res.noalias() = A.conjugate() * v;
      VERIFY_IS_APPROX(res, (A.conjugate().eval() * v.eval()).eval());

      res.noalias() = A * v.conjugate();
      VERIFY_IS_APPROX(res, (A.eval() * v.conjugate().eval()).eval());

      // Non-square: wide matrix × vector (exercises different cols path).
      Mat C = Mat::Random(m, m + 3);
      Vec w = Vec::Random(m + 3);
      Vec res3(m);
      res3.noalias() = C.conjugate() * w;
      VERIFY_IS_APPROX(res3, (C.conjugate().eval() * w.eval()).eval());
    }
  }
  (void)PS_f;
  (void)PS_d;
}

// Locks the BLAS contract that GEMM/GEMV leave the destination unchanged when
// alpha == 0, including under non-finite inputs in A/x/B that would otherwise
// taint the result via 0 * Inf = NaN.
// A zero scalar factor must leave the destination untouched for every
// assignment form: += and -= return early, while = zeroes the destination.
// Inf/NaN operands make a kernel that ran anyway visible as a NaN.
template <typename Scalar, int Order>
void alpha_zero_skips_gemm(Index m, Index k, Index n) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, Dynamic, Order> Mat;

  const Scalar pos_zero = Scalar(0);
  const Scalar neg_zero = Scalar(-RealScalar(0));

  Mat A = Mat::Random(m, k);
  Mat B = Mat::Random(k, n);
  A(0, 0) = Scalar(NumTraits<RealScalar>::infinity());
  B(1, 1) = Scalar(NumTraits<RealScalar>::quiet_NaN());

  const Mat C_ref = Mat::Random(m, n);
  const Mat zero = Mat::Zero(m, n);

  for (const Scalar& alpha : {pos_zero, neg_zero}) {
    Mat C = C_ref;
    C.noalias() += alpha * A * B;
    VERIFY_IS_CWISE_EQUAL(C, C_ref);

    C = C_ref;
    C.noalias() -= alpha * A * B;
    VERIFY_IS_CWISE_EQUAL(C, C_ref);

    C = C_ref;
    C.noalias() = alpha * A * B;
    VERIFY_IS_CWISE_EQUAL(C, zero);
  }
}

template <typename Scalar>
void alpha_zero_skips_kernel() {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, Dynamic, ColMajor> ColMat;
  typedef Matrix<Scalar, Dynamic, Dynamic, RowMajor> RowMat;
  typedef Matrix<Scalar, Dynamic, 1> Vec;

  const Index m = 17, k = 13, n = 11;
  const Scalar inf = Scalar(NumTraits<RealScalar>::infinity());
  const Scalar nan = Scalar(NumTraits<RealScalar>::quiet_NaN());
  const Scalar pos_zero = Scalar(0);
  const Scalar neg_zero = Scalar(-RealScalar(0));

  // GEMM, both storage orders. 17x13x11 sums to 41 and so takes the GEMM path
  // on a default build; 5x5x5 sums to 15 and reaches the coeff-based path,
  // which has its own zero-factor exit. Without the small shape the latter is
  // only covered where the threshold is raised (SME), i.e. by one nightly job.
  alpha_zero_skips_gemm<Scalar, ColMajor>(m, k, n);
  alpha_zero_skips_gemm<Scalar, ColMajor>(5, 5, 5);
  alpha_zero_skips_gemm<Scalar, RowMajor>(m, k, n);
  alpha_zero_skips_gemm<Scalar, RowMajor>(5, 5, 5);

  // GEMV col-major.
  {
    ColMat A = ColMat::Random(m, k);
    Vec x = Vec::Random(k);
    A(0, 0) = inf;
    x(1) = nan;

    Vec y = Vec::Random(m);
    const Vec y_ref = y;

    y.noalias() += pos_zero * (A * x);
    VERIFY_IS_CWISE_EQUAL(y, y_ref);

    y.noalias() += neg_zero * (A * x);
    VERIFY_IS_CWISE_EQUAL(y, y_ref);
  }

  // GEMV row-major.
  {
    RowMat A = RowMat::Random(m, k);
    Vec x = Vec::Random(k);
    A(0, 0) = inf;
    x(1) = nan;

    Vec y = Vec::Random(m);
    const Vec y_ref = y;

    y.noalias() += pos_zero * (A * x);
    VERIFY_IS_CWISE_EQUAL(y, y_ref);

    y.noalias() += neg_zero * (A * x);
    VERIFY_IS_CWISE_EQUAL(y, y_ref);
  }
}

EIGEN_DECLARE_TEST(product_extra) {
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(product_extra(
        MatrixXf(internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_2(product_extra(
        MatrixXd(internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_2(mat_mat_scalar_scalar_product());
    CALL_SUBTEST_3(product_extra(MatrixXcf(internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2),
                                           internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2))));
    CALL_SUBTEST_4(product_extra(MatrixXcd(internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2),
                                           internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2))));
    CALL_SUBTEST_1(zero_sized_objects(
        MatrixXf(internal::random<int>(1, EIGEN_TEST_MAX_SIZE), internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
  }
  CALL_SUBTEST_5(bug_127<0>());
  CALL_SUBTEST_5(triangular_product_assignment_size_mismatch<0>());
  CALL_SUBTEST_5(bug_817<0>());
  CALL_SUBTEST_5(bug_1308<0>());
  CALL_SUBTEST_6(unaligned_objects<0>());
  CALL_SUBTEST_7(compute_block_size<float>());
  CALL_SUBTEST_7(compute_block_size<double>());
  CALL_SUBTEST_7(compute_block_size<std::complex<double> >());
  CALL_SUBTEST_8(aliasing_with_resize<void>());
  CALL_SUBTEST_9(product_custom_scalar_types<0>());
  CALL_SUBTEST_10(test_small_block_correctness<0>());

  // Complex GEMV conjugation at varied sizes (deterministic, outside g_repeat).
  CALL_SUBTEST_11(gemv_complex_conjugate<0>());

  // alpha==0 fast path: GEMM/GEMV must leave the destination unchanged.
  CALL_SUBTEST_12(alpha_zero_skips_kernel<float>());
  CALL_SUBTEST_12(alpha_zero_skips_kernel<double>());
  CALL_SUBTEST_12(alpha_zero_skips_kernel<std::complex<float> >());
  CALL_SUBTEST_12(alpha_zero_skips_kernel<std::complex<double> >());
}
