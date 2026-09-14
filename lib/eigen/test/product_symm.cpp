// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2009 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "main.h"

template <typename Scalar, int Size, int OtherSize>
void symm(int size = Size, int othersize = OtherSize) {
  typedef Matrix<Scalar, Size, Size> MatrixType;
  typedef Matrix<Scalar, Size, OtherSize> Rhs1;
  typedef Matrix<Scalar, OtherSize, Size> Rhs2;
  enum { order = OtherSize == 1 ? 0 : RowMajor };
  typedef Matrix<Scalar, Size, OtherSize, order> Rhs3;

  Index rows = size;
  Index cols = size;

  MatrixType m1 = MatrixType::Random(rows, cols), m2 = MatrixType::Random(rows, cols), m3;

  m1 = (m1 + m1.adjoint()).eval();

  Rhs1 rhs1 = Rhs1::Random(cols, othersize), rhs12(cols, othersize), rhs13(cols, othersize);
  Rhs2 rhs2 = Rhs2::Random(othersize, rows), rhs22(othersize, rows), rhs23(othersize, rows);
  Rhs3 rhs3 = Rhs3::Random(cols, othersize), rhs32(cols, othersize), rhs33(cols, othersize);

  Scalar s1 = internal::random<Scalar>(), s2 = internal::random<Scalar>();

  m2 = m1.template triangularView<Lower>();
  m3 = m2.template selfadjointView<Lower>();
  VERIFY_IS_EQUAL(m1, m3);
  VERIFY_IS_APPROX(rhs12 = (s1 * m2).template selfadjointView<Lower>() * (s2 * rhs1), rhs13 = (s1 * m1) * (s2 * rhs1));

  VERIFY_IS_APPROX(rhs12 = (s1 * m2).transpose().template selfadjointView<Upper>() * (s2 * rhs1),
                   rhs13 = (s1 * m1.transpose()) * (s2 * rhs1));

  VERIFY_IS_APPROX(rhs12 = (s1 * m2).template selfadjointView<Lower>().transpose() * (s2 * rhs1),
                   rhs13 = (s1 * m1.transpose()) * (s2 * rhs1));

  VERIFY_IS_APPROX(rhs12 = (s1 * m2).conjugate().template selfadjointView<Lower>() * (s2 * rhs1),
                   rhs13 = (s1 * m1).conjugate() * (s2 * rhs1));

  VERIFY_IS_APPROX(rhs12 = (s1 * m2).template selfadjointView<Lower>().conjugate() * (s2 * rhs1),
                   rhs13 = (s1 * m1).conjugate() * (s2 * rhs1));

  VERIFY_IS_APPROX(rhs12 = (s1 * m2).adjoint().template selfadjointView<Upper>() * (s2 * rhs1),
                   rhs13 = (s1 * m1).adjoint() * (s2 * rhs1));

  VERIFY_IS_APPROX(rhs12 = (s1 * m2).template selfadjointView<Lower>().adjoint() * (s2 * rhs1),
                   rhs13 = (s1 * m1).adjoint() * (s2 * rhs1));

  m2 = m1.template triangularView<Upper>();
  rhs12.setRandom();
  rhs13 = rhs12;
  m3 = m2.template selfadjointView<Upper>();
  VERIFY_IS_EQUAL(m1, m3);
  VERIFY_IS_APPROX(rhs12 += (s1 * m2).template selfadjointView<Upper>() * (s2 * rhs1),
                   rhs13 += (s1 * m1) * (s2 * rhs1));

  m2 = m1.template triangularView<Lower>();
  VERIFY_IS_APPROX(rhs12 = (s1 * m2).template selfadjointView<Lower>() * (s2 * rhs2.adjoint()),
                   rhs13 = (s1 * m1) * (s2 * rhs2.adjoint()));

  m2 = m1.template triangularView<Upper>();
  VERIFY_IS_APPROX(rhs12 = (s1 * m2).template selfadjointView<Upper>() * (s2 * rhs2.adjoint()),
                   rhs13 = (s1 * m1) * (s2 * rhs2.adjoint()));

  m2 = m1.template triangularView<Upper>();
  VERIFY_IS_APPROX(rhs12 = (s1 * m2.adjoint()).template selfadjointView<Lower>() * (s2 * rhs2.adjoint()),
                   rhs13 = (s1 * m1.adjoint()) * (s2 * rhs2.adjoint()));

  // test row major = <...>
  m2 = m1.template triangularView<Lower>();
  rhs32.setRandom();
  rhs13 = rhs32;
  VERIFY_IS_APPROX(rhs32.noalias() -= (s1 * m2).template selfadjointView<Lower>() * (s2 * rhs3),
                   rhs13 -= (s1 * m1) * (s2 * rhs3));

  m2 = m1.template triangularView<Upper>();
  VERIFY_IS_APPROX(rhs32.noalias() = (s1 * m2.adjoint()).template selfadjointView<Lower>() * (s2 * rhs3).conjugate(),
                   rhs13 = (s1 * m1.adjoint()) * (s2 * rhs3).conjugate());

  m2 = m1.template triangularView<Upper>();
  rhs13 = rhs12;
  VERIFY_IS_APPROX(rhs12.noalias() += s1 * ((m2.adjoint()).template selfadjointView<Lower>() * (s2 * rhs3).conjugate()),
                   rhs13 += (s1 * m1.adjoint()) * (s2 * rhs3).conjugate());

  m2 = m1.template triangularView<Lower>();
  VERIFY_IS_APPROX(rhs22 = (rhs2) * (m2).template selfadjointView<Lower>(), rhs23 = (rhs2) * (m1));
  VERIFY_IS_APPROX(rhs22 = (s2 * rhs2) * (s1 * m2).template selfadjointView<Lower>(), rhs23 = (s2 * rhs2) * (s1 * m1));

  // destination with a non-default inner-stride
  // see bug 1741
  {
    typedef Matrix<Scalar, Dynamic, Dynamic> MatrixX;
    MatrixX buffer(2 * cols, 2 * othersize);
    // As for map2 below, the outer stride spans Rhs1's outer dimension, which is not rows() when
    // Rhs1 is row major.
    Map<Rhs1, 0, Stride<Dynamic, 2> > map1(buffer.data(), cols, othersize,
                                           Stride<Dynamic, 2>(2 * rhs13.outerStride(), 2));
    buffer.setZero();
    VERIFY_IS_APPROX(map1.noalias() = (s1 * m2).template selfadjointView<Lower>() * (s2 * rhs1),
                     rhs13 = (s1 * m1) * (s2 * rhs1));

    Map<Rhs2, 0, Stride<Dynamic, 2> > map2(buffer.data(), rhs22.rows(), rhs22.cols(),
                                           Stride<Dynamic, 2>(2 * rhs22.outerStride(), 2));
    buffer.setZero();
    VERIFY_IS_APPROX(map2 = (rhs2) * (m2).template selfadjointView<Lower>(), rhs23 = (rhs2) * (m1));
  }
}

// Physical RowMajor selfadjoint operand.  symm<> above always builds a ColMajor
// operand, so the RowMajor packers -- symm_pack_lhs/symm_pack_rhs specialized on
// RowMajor, including the SME versions whose transposed regions carry the
// two-pass trailing transpose -- are otherwise never reached through the public
// API.  Both operand positions (selfadjoint on the LHS and on the RHS) and both
// stored triangles are checked against a dense reference.
template <typename Scalar>
void symm_rowmajor_selfadjoint(Index size, Index othersize) {
  typedef Matrix<Scalar, Dynamic, Dynamic, RowMajor> RowMat;
  typedef Matrix<Scalar, Dynamic, Dynamic> ColMat;

  RowMat m1 = RowMat::Random(size, size);
  m1 = (m1 + m1.adjoint()).eval();  // exactly self-adjoint
  RowMat lo = m1.template triangularView<Lower>();
  RowMat up = m1.template triangularView<Upper>();

  // Selfadjoint on the LHS: packs the RowMajor operand via symm_pack_lhs.
  ColMat rhs = ColMat::Random(size, othersize);
  ColMat ref = m1 * rhs;
  VERIFY_IS_APPROX((lo.template selfadjointView<Lower>() * rhs).eval(), ref);
  VERIFY_IS_APPROX((up.template selfadjointView<Upper>() * rhs).eval(), ref);

  // Selfadjoint on the RHS: packs the RowMajor operand via symm_pack_rhs.
  ColMat lhs = ColMat::Random(othersize, size);
  ColMat ref2 = lhs * m1;
  VERIFY_IS_APPROX((lhs * lo.template selfadjointView<Lower>()).eval(), ref2);
  VERIFY_IS_APPROX((lhs * up.template selfadjointView<Upper>()).eval(), ref2);
}

// Test symmetric products at blocking boundary sizes.
// The existing test uses random sizes; these deterministic sizes exercise
// transitions in GEBP blocking (early-return at 48, block size changes).
template <int>
void product_symm_boundary() {
  const int sizes[] = {1, 2, 3, 4, 8, 16, 47, 48, 49, 64, 96, 128};
  for (int si = 0; si < 12; ++si) {
    int n = sizes[si];

    // double, matrix RHS
    symm<double, Dynamic, Dynamic>(n, 5);
    // double, vector RHS
    symm<double, Dynamic, 1>(n);
    // float, matrix RHS
    symm<float, Dynamic, Dynamic>(n, 7);
    // complex float, matrix RHS
    symm<std::complex<float>, Dynamic, Dynamic>(n, 3);
  }

  // RowMajor selfadjoint operand.  The partial last-panel widths in this list
  // drive the RowMajor packers' transposed regions through the two-pass trailing
  // transpose for streaming vector lengths from SVL=128 (svlw=4) up to SVL=2048
  // (svlw=64): a partial width w in (svlw, 2*svlw) needs two predicated passes.
  const int sa_sizes[] = {1, 5, 7, 17, 32, 33, 39, 45, 48, 49, 55, 57, 63, 64, 65, 96};
  for (int n : sa_sizes) {
    symm_rowmajor_selfadjoint<float>(n, 7);
    symm_rowmajor_selfadjoint<float>(n, 1);
    symm_rowmajor_selfadjoint<double>(n, 4);
  }
}

// Packed-buffer contract: symm_pack_lhs/rhs applied to a stored triangle must
// be bit-identical to gemm_pack_lhs/rhs applied to the reconstructed dense
// matrix -- gebp_kernel cannot tell the two apart. Sentinels in the unused
// triangle catch reads of the wrong half; a marker past the packed range (in
// both buffers) catches writing too much or too little.

template <typename Scalar>
Scalar symm_pack_sentinel() {
  return Scalar(typename NumTraits<Scalar>::Real(98765));
}

// Build an n x n Hermitian `full` and its triangle-only image `stored`
// (row >= col valid, sentinel elsewhere).
template <typename MatrixType>
void make_stored_triangle(Index n, MatrixType& stored, MatrixType& full) {
  full = MatrixType::Random(n, n);
  full = (full + full.adjoint()).eval();
  stored = MatrixType::Constant(n, n, symm_pack_sentinel<typename MatrixType::Scalar>());
  stored.template triangularView<Lower>() = full;
}

// A buffer holding `packed_size` packed entries followed by overrun markers.
template <typename Scalar>
Matrix<Scalar, Dynamic, 1> make_marked_buffer(Index packed_size) {
  return Matrix<Scalar, Dynamic, 1>::Constant(packed_size + 32, Scalar(typename NumTraits<Scalar>::Real(-31415)));
}

template <typename Scalar, int StorageOrder>
void check_symm_pack_lhs(Index kc) {
  using Traits = internal::gebp_traits<Scalar, Scalar>;
  using Mat = Matrix<Scalar, Dynamic, Dynamic, StorageOrder>;
  using Mapper = internal::const_blas_data_mapper<Scalar, Index, StorageOrder>;
  Mat stored, full;
  make_stored_triangle<Mat>(kc, stored, full);

  Matrix<Scalar, Dynamic, 1> packed = make_marked_buffer<Scalar>(kc * kc);
  Matrix<Scalar, Dynamic, 1> ref = packed;

  internal::symm_pack_lhs<Scalar, Index, Traits::mr, Traits::LhsProgress, StorageOrder>()(packed.data(), stored.data(),
                                                                                          stored.outerStride(), kc, kc);
  internal::gemm_pack_lhs<Scalar, Index, Mapper, Traits::mr, Traits::LhsProgress, typename Traits::LhsPacket4Packing,
                          StorageOrder, false, false>()(ref.data(), Mapper(full.data(), full.outerStride()), kc, kc);
  VERIFY_IS_EQUAL(packed, ref);
}

template <typename Scalar, int StorageOrder>
void check_symm_pack_rhs(Index n, Index rows, Index k2) {
  using Traits = internal::gebp_traits<Scalar, Scalar>;
  using Mat = Matrix<Scalar, Dynamic, Dynamic, StorageOrder>;
  using Mapper = internal::const_blas_data_mapper<Scalar, Index, StorageOrder>;
  Mat stored, full;
  make_stored_triangle<Mat>(n, stored, full);

  Matrix<Scalar, Dynamic, 1> packed = make_marked_buffer<Scalar>(rows * n);
  Matrix<Scalar, Dynamic, 1> ref = packed;

  internal::symm_pack_rhs<Scalar, Index, Traits::nr, StorageOrder>()(packed.data(), stored.data(), stored.outerStride(),
                                                                     rows, n, k2);
  internal::gemm_pack_rhs<Scalar, Index, Mapper, Traits::nr, StorageOrder, false, false>()(
      ref.data(), Mapper(full.data(), full.outerStride()).getSubMapper(k2, 0), rows, n);
  VERIFY_IS_EQUAL(packed, ref);
}

template <typename Scalar>
void symm_pack_buffers() {
  using Traits = internal::gebp_traits<Scalar, Scalar>;
  constexpr Index mr = Traits::mr;
  constexpr Index nr = Traits::nr;
  constexpr Index ps = internal::packet_traits<Scalar>::size;

  // LHS: every panel width up to a full panel plus a packet, then multi-panel
  // borders around the 2*mr/3*mr transitions and the half-packet tail.
  for (Index kc = 1; kc <= mr + ps + 2; ++kc) {
    check_symm_pack_lhs<Scalar, ColMajor>(kc);
    check_symm_pack_lhs<Scalar, RowMajor>(kc);
  }
  const Index lhs_borders[] = {2 * mr - 1, 2 * mr, 2 * mr + 1, 3 * mr, 3 * mr + ps / 2, 3 * mr + ps / 2 + 1, 97};
  for (Index kc : lhs_borders) {
    check_symm_pack_lhs<Scalar, ColMajor>(kc);
    check_symm_pack_lhs<Scalar, RowMajor>(kc);
  }

  // RHS: sizes on and off the 8/4-column panel grid, with every depth block
  // the driver's blocking can produce (k2 a multiple of 8; a block ends
  // 8-aligned or at the matrix edge).
  const Index rhs_sizes[] = {1,  2,  3,  nr - 1, nr, nr + 1, 2 * nr + 1, 15, 16, 17,
                             23, 24, 25, 31,     32, 33,     47,         48, 49, 97};
  for (Index n : rhs_sizes) {
    if (n < 1) continue;
    for (Index k2 = 0; k2 < n; k2 += 8) {
      check_symm_pack_rhs<Scalar, ColMajor>(n, n - k2, k2);
      check_symm_pack_rhs<Scalar, RowMajor>(n, n - k2, k2);
      for (Index rows = 8; k2 + rows <= (n / 8) * 8; rows += 8) {
        check_symm_pack_rhs<Scalar, ColMajor>(n, rows, k2);
        check_symm_pack_rhs<Scalar, RowMajor>(n, rows, k2);
      }
    }
  }
}

// A physically RowMajor selfadjoint operand. The symm<> tests above always
// build ColMajor operands, so without this the RowMajor symm_pack_lhs/rhs
// instantiations are never reached through the public API.
template <typename Scalar>
void symm_rowmajor_operand(Index n, Index m) {
  using RowMat = Matrix<Scalar, Dynamic, Dynamic, RowMajor>;
  using ColMat = Matrix<Scalar, Dynamic, Dynamic, ColMajor>;
  RowMat s = RowMat::Random(n, n);
  s = (s + s.adjoint()).eval();
  RowMat lo = s.template triangularView<Lower>();
  RowMat up = s.template triangularView<Upper>();
  ColMat b = ColMat::Random(n, m), c = ColMat::Random(m, n);

  ColMat ref = ColMat(s) * b;
  VERIFY_IS_APPROX(ColMat(lo.template selfadjointView<Lower>() * b), ref);
  VERIFY_IS_APPROX(ColMat(up.template selfadjointView<Upper>() * b), ref);
  ColMat ref2 = c * ColMat(s);
  VERIFY_IS_APPROX(ColMat(c * lo.template selfadjointView<Lower>()), ref2);
  VERIFY_IS_APPROX(ColMat(c * up.template selfadjointView<Upper>()), ref2);
}

template <int>
void symm_packers_and_rowmajor_operands() {
  symm_pack_buffers<float>();
  symm_pack_buffers<double>();
  symm_pack_buffers<std::complex<float> >();
  symm_pack_buffers<std::complex<double> >();

  const Index sizes[] = {1, 2, 7, 8, 9, 24, 25, 31, 32, 33, 47, 48, 49, 65};
  for (Index n : sizes) {
    for (Index m : {1, 3, 17}) {
      symm_rowmajor_operand<float>(n, m);
      symm_rowmajor_operand<double>(n, m);
      symm_rowmajor_operand<std::complex<float> >(n, m);
    }
  }
}

EIGEN_DECLARE_TEST(product_symm) {
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1((symm<float, Dynamic, Dynamic>(internal::random<int>(1, EIGEN_TEST_MAX_SIZE),
                                                  internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_2((symm<double, Dynamic, Dynamic>(internal::random<int>(1, EIGEN_TEST_MAX_SIZE),
                                                   internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_3((symm<std::complex<float>, Dynamic, Dynamic>(internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2),
                                                                internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2))));
    CALL_SUBTEST_4((symm<std::complex<double>, Dynamic, Dynamic>(internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2),
                                                                 internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2))));

    CALL_SUBTEST_5((symm<float, Dynamic, 1>(internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_6((symm<double, Dynamic, 1>(internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_7((symm<std::complex<float>, Dynamic, 1>(internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
    CALL_SUBTEST_8((symm<std::complex<double>, Dynamic, 1>(internal::random<int>(1, EIGEN_TEST_MAX_SIZE))));
  }

  // Deterministic blocking boundary tests (outside g_repeat).
  CALL_SUBTEST_9(product_symm_boundary<0>());

  // Packed-buffer contract checks and RowMajor selfadjoint operands.
  CALL_SUBTEST_10(symm_packers_and_rowmajor_operands<0>());
}
