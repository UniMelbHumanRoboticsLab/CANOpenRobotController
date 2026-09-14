// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2015 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// This unit test cannot be easily written to work with EIGEN_DEFAULT_TO_ROW_MAJOR
#ifdef EIGEN_DEFAULT_TO_ROW_MAJOR
#undef EIGEN_DEFAULT_TO_ROW_MAJOR
#endif

static long int nb_temporaries;

inline void on_temporary_creation() {
  // here's a great place to set a breakpoint when debugging failures in this test!
  nb_temporaries++;
}

#define EIGEN_SPARSE_CREATE_TEMPORARY_PLUGIN \
  { on_temporary_creation(); }

#include "main.h"
#include <Eigen/SparseCore>

#define VERIFY_EVALUATION_COUNT(XPR, N)                                                   \
  {                                                                                       \
    nb_temporaries = 0;                                                                   \
    CALL_SUBTEST(XPR);                                                                    \
    if (nb_temporaries != N) std::cerr << "nb_temporaries == " << nb_temporaries << "\n"; \
    VERIFY((#XPR) && nb_temporaries == N);                                                \
  }

template <typename PlainObjectType>
void check_const_correctness(const PlainObjectType &) {
  // verify that ref-to-const don't have LvalueBit
  typedef std::add_const_t<PlainObjectType> ConstPlainObjectType;
  VERIFY(!(internal::traits<Ref<ConstPlainObjectType>>::Flags & LvalueBit));
  VERIFY(!(internal::traits<Ref<ConstPlainObjectType, Aligned>>::Flags & LvalueBit));
  VERIFY(!(Ref<ConstPlainObjectType>::Flags & LvalueBit));
  VERIFY(!(Ref<ConstPlainObjectType, Aligned>::Flags & LvalueBit));
}

template <typename B>
EIGEN_DONT_INLINE void call_ref_1(Ref<SparseMatrix<float>> a, const B &b) {
  VERIFY_IS_EQUAL(a.toDense(), b.toDense());
}

template <typename B>
EIGEN_DONT_INLINE void call_ref_2(const Ref<const SparseMatrix<float>> &a, const B &b) {
  VERIFY_IS_EQUAL(a.toDense(), b.toDense());
}

template <typename B>
EIGEN_DONT_INLINE void call_ref_rm_1(Ref<SparseMatrix<float, RowMajor>> a, const B &b) {
  VERIFY_IS_EQUAL(a.toDense(), b.toDense());
}

template <typename B>
EIGEN_DONT_INLINE void call_ref_rm_2(const Ref<const SparseMatrix<float, RowMajor>> &a, const B &b) {
  VERIFY_IS_EQUAL(a.toDense(), b.toDense());
}

template <typename B>
EIGEN_DONT_INLINE void call_ref_3(const Ref<const SparseMatrix<float>, StandardCompressedFormat> &a, const B &b) {
  VERIFY(a.isCompressed());
  VERIFY_IS_EQUAL(a.toDense(), b.toDense());
}

template <typename B>
EIGEN_DONT_INLINE void call_ref_4(Ref<SparseVector<float>> a, const B &b) {
  VERIFY_IS_EQUAL(a.toDense(), b.toDense());
}

template <typename B>
EIGEN_DONT_INLINE void call_ref_5(const Ref<const SparseVector<float>> &a, const B &b) {
  VERIFY_IS_EQUAL(a.toDense(), b.toDense());
}

template <typename T>
typename T::Map make_map(T &xpr) {
  return typename T::Map(xpr.rows(), xpr.cols(), xpr.nonZeros(), xpr.outerIndexPtr(), xpr.innerIndexPtr(),
                         xpr.valuePtr(), xpr.innerNonZeroPtr());
}

template <typename T>
auto innerpanel(T &xpr, Index start, Index size)
    -> std::enable_if_t<(T::Flags & RowMajorBit) == RowMajorBit, decltype(xpr.middleRows(start, size))> {
  return xpr.middleRows(start, size);
}

template <typename T>
auto innerpanel(T &xpr, Index start, Index size)
    -> std::enable_if_t<(T::Flags & RowMajorBit) == 0, decltype(xpr.middleCols(start, size))> {
  return xpr.middleCols(start, size);
}

template <typename T>
auto innervector(T &xpr, Index index)
    -> std::enable_if_t<(T::Flags & RowMajorBit) == RowMajorBit, decltype(xpr.row(index))> {
  return xpr.row(index);
}

template <typename T>
auto innervector(T &xpr, Index index) -> std::enable_if_t<(T::Flags & RowMajorBit) == 0, decltype(xpr.col(index))> {
  return xpr.col(index);
}

template <typename XprType, typename ExpectedType>
void verify_coeffs(const XprType &xpr, const ExpectedType &expected) {
  auto coeffs = xpr.coeffs();
  VERIFY_IS_EQUAL(coeffs.size(), expected.size());
  for (Index i = 0; i < expected.size(); ++i) VERIFY_IS_EQUAL(coeffs[i], expected[i]);
}

template <int Options>
void init_diag(SparseMatrix<float, Options> &m) {
  m.resize(4, 4);
  m.insert(0, 0) = 10.f;
  m.insert(1, 1) = 20.f;
  m.insert(2, 2) = 30.f;
  m.insert(3, 3) = 40.f;
  m.makeCompressed();
}

template <typename MatrixType>
std::enable_if_t<(MatrixType::Flags & RowMajorBit) == 0> init_noncompressed_inner_panel(MatrixType &m) {
  m.resize(4, 4);
  m.reserve(VectorXi::Constant(4, 2));
  m.insert(1, 1) = 11.f;
  m.insert(2, 1) = 12.f;
  m.insert(0, 2) = 20.f;
  m.insert(2, 2) = 30.f;
  m.insert(3, 3) = 40.f;
}

template <typename MatrixType>
std::enable_if_t<(MatrixType::Flags & RowMajorBit) == RowMajorBit> init_noncompressed_inner_panel(MatrixType &m) {
  m.resize(4, 4);
  m.reserve(VectorXi::Constant(4, 2));
  m.insert(1, 1) = 11.f;
  m.insert(1, 2) = 12.f;
  m.insert(2, 0) = 20.f;
  m.insert(2, 2) = 30.f;
  m.insert(3, 3) = 40.f;
}

template <typename MatrixType, typename Init, typename Verify>
void verify_ref_and_map(const Init &init, const Verify &verify) {
  {
    MatrixType matrix;
    init(matrix);
    Ref<MatrixType> ref(matrix);
    verify(ref, matrix, 0);
  }

  {
    MatrixType matrix;
    init(matrix);
    auto map = make_map(matrix);
    verify(map, matrix, 1);
  }
}

template <typename XprType, typename MatrixType>
void verify_slice_coeffs(XprType &xpr, MatrixType &storage, int variant) {
  Array<float, 2, 1> expected;
  expected << 20.f, 30.f;

  Ref<MatrixType> middle(innerpanel(xpr, 1, 2));
  Ref<const MatrixType> const_middle(innerpanel(xpr, 1, 2));
  const float updated0 = 210.f + 10.f * variant;
  const float updated1 = 310.f + 10.f * variant;

  verify_coeffs(middle, expected);
  verify_coeffs(const_middle, expected);
  auto coeffs = middle.coeffs();
  coeffs[0] = updated0;
  coeffs[1] = updated1;
  VERIFY_IS_EQUAL(storage.coeff(0, 0), 10.f);
  VERIFY_IS_EQUAL(storage.coeff(1, 1), updated0);
  VERIFY_IS_EQUAL(storage.coeff(2, 2), updated1);
}

template <typename MatrixType, typename XprType>
float innerpanel_coeff(const XprType &xpr, Index outer, Index inner) {
  if ((MatrixType::Flags & RowMajorBit) == RowMajorBit) return xpr.coeff(outer, inner);
  return xpr.coeff(inner, outer);
}

template <typename MatrixType, typename XprType>
void set_innerpanel_coeff(XprType &xpr, Index outer, Index inner, float value) {
  if ((MatrixType::Flags & RowMajorBit) == RowMajorBit)
    xpr.coeffRef(outer, inner) = value;
  else
    xpr.coeffRef(inner, outer) = value;
}

template <typename MatrixType>
float storage_innerpanel_coeff(const MatrixType &storage, Index outer, Index inner) {
  if ((MatrixType::Flags & RowMajorBit) == RowMajorBit) return storage.coeff(outer + 1, inner);
  return storage.coeff(inner, outer + 1);
}

template <typename MatrixType>
float storage_innervector_coeff(const MatrixType &storage, Index outer, Index inner) {
  if ((MatrixType::Flags & RowMajorBit) == RowMajorBit) return storage.coeff(outer, inner);
  return storage.coeff(inner, outer);
}

template <typename XprType, typename MatrixType>
void verify_noncompressed_middle_binding(XprType &xpr, MatrixType &storage, int variant) {
  struct Entry {
    Index outer;
    Index inner;
    float value;
  };

  Ref<MatrixType> middle(innerpanel(xpr, 1, 2));
  Ref<const MatrixType> const_middle(innerpanel(xpr, 1, 2));
  const float updated0 = 111.f + variant;
  const float updated1 = 121.f + variant;
  const Entry expected[] = {{0, 1, 11.f}, {0, 2, 12.f}, {1, 0, 20.f}, {1, 2, 30.f}};

  VERIFY(!middle.isCompressed());
  VERIFY_IS_EQUAL(middle.nonZeros(), 4);
  VERIFY(!const_middle.isCompressed());
  VERIFY_IS_EQUAL(const_middle.nonZeros(), 4);
  for (const auto &entry : expected) {
    VERIFY_IS_EQUAL(innerpanel_coeff<MatrixType>(middle, entry.outer, entry.inner), entry.value);
    VERIFY_IS_EQUAL(innerpanel_coeff<MatrixType>(const_middle, entry.outer, entry.inner), entry.value);
  }

  set_innerpanel_coeff<MatrixType>(middle, 0, 1, updated0);
  set_innerpanel_coeff<MatrixType>(middle, 1, 0, updated1);
  VERIFY_IS_EQUAL(storage_innerpanel_coeff(storage, 0, 1), updated0);
  VERIFY_IS_EQUAL(storage_innerpanel_coeff(storage, 1, 0), updated1);
}

template <typename XprType, typename MatrixType>
void verify_noncompressed_inner_vector_binding(XprType &xpr, MatrixType &storage, int variant) {
  Array<float, 2, 1> expected;
  expected << 20.f, 30.f;

  Ref<SparseVector<float>> inner(innervector(xpr, 2));
  Ref<const SparseVector<float>> const_inner(innervector(xpr, 2));
  const float updated0 = 211.f + variant;
  const float updated1 = 221.f + variant;

  VERIFY(inner.isCompressed());
  VERIFY_IS_EQUAL(inner.rows(), 4);
  VERIFY_IS_EQUAL(inner.cols(), 1);
  VERIFY_IS_EQUAL(inner.nonZeros(), 2);
  VERIFY(const_inner.isCompressed());
  VERIFY_IS_EQUAL(const_inner.rows(), 4);
  VERIFY_IS_EQUAL(const_inner.cols(), 1);
  VERIFY_IS_EQUAL(const_inner.nonZeros(), 2);
  verify_coeffs(inner, expected);
  verify_coeffs(const_inner, expected);

  auto coeffs = inner.coeffs();
  coeffs[0] = updated0;
  coeffs[1] = updated1;
  VERIFY_IS_EQUAL(storage_innervector_coeff(storage, 2, 0), updated0);
  VERIFY_IS_EQUAL(storage_innervector_coeff(storage, 2, 2), updated1);
}

void check_ref_slice_coeffs() {
  verify_ref_and_map<SparseMatrix<float>>(
      [](SparseMatrix<float> &matrix) { init_diag(matrix); },
      [](auto &xpr, auto &matrix, int variant) { verify_slice_coeffs(xpr, matrix, variant); });
  verify_ref_and_map<SparseMatrix<float, RowMajor>>(
      [](SparseMatrix<float, RowMajor> &matrix) { init_diag(matrix); },
      [](auto &xpr, auto &matrix, int variant) { verify_slice_coeffs(xpr, matrix, variant); });
}

void check_noncompressed_ref_slices() {
  verify_ref_and_map<SparseMatrix<float>>(
      [](SparseMatrix<float> &matrix) { init_noncompressed_inner_panel(matrix); },
      [](auto &xpr, auto &matrix, int variant) { verify_noncompressed_middle_binding(xpr, matrix, variant); });
  verify_ref_and_map<SparseMatrix<float, RowMajor>>(
      [](SparseMatrix<float, RowMajor> &matrix) { init_noncompressed_inner_panel(matrix); },
      [](auto &xpr, auto &matrix, int variant) { verify_noncompressed_middle_binding(xpr, matrix, variant); });
}

void check_noncompressed_ref_inner_vectors() {
  verify_ref_and_map<SparseMatrix<float>>(
      [](SparseMatrix<float> &matrix) { init_noncompressed_inner_panel(matrix); },
      [](auto &xpr, auto &matrix, int variant) { verify_noncompressed_inner_vector_binding(xpr, matrix, variant); });
  verify_ref_and_map<SparseMatrix<float, RowMajor>>(
      [](SparseMatrix<float, RowMajor> &matrix) { init_noncompressed_inner_panel(matrix); },
      [](auto &xpr, auto &matrix, int variant) { verify_noncompressed_inner_vector_binding(xpr, matrix, variant); });
}

void call_ref() {
  SparseMatrix<float> A = MatrixXf::Random(10, 10).sparseView(0.5, 1);
  SparseMatrix<float, RowMajor> B = MatrixXf::Random(10, 10).sparseView(0.5, 1);
  SparseMatrix<float> C = MatrixXf::Random(10, 10).sparseView(0.5, 1);
  C.reserve(VectorXi::Constant(C.outerSize(), 2));
  const SparseMatrix<float> &Ac(A);
  SparseVector<float> vc = VectorXf::Random(10).sparseView(0.5, 1);
  SparseVector<float, RowMajor> vr = VectorXf::Random(10).sparseView(0.5, 1);
  SparseMatrix<float> AA = A * A;

  VERIFY_EVALUATION_COUNT(call_ref_1(A, A), 0);
  //   VERIFY_EVALUATION_COUNT( call_ref_1(Ac, Ac),  0); // does not compile on purpose
  VERIFY_EVALUATION_COUNT(call_ref_2(A, A), 0);
  VERIFY_EVALUATION_COUNT(call_ref_3(A, A), 0);
  VERIFY_EVALUATION_COUNT(call_ref_2(A.transpose(), A.transpose()), 1);
  VERIFY_EVALUATION_COUNT(call_ref_3(A.transpose(), A.transpose()), 1);
  VERIFY_EVALUATION_COUNT(call_ref_2(Ac, Ac), 0);
  VERIFY_EVALUATION_COUNT(call_ref_3(Ac, Ac), 0);
  VERIFY_EVALUATION_COUNT(call_ref_2(A + A, 2 * Ac), 1);
  VERIFY_EVALUATION_COUNT(call_ref_3(A + A, 2 * Ac), 1);
  VERIFY_EVALUATION_COUNT(call_ref_2(B, B), 1);
  VERIFY_EVALUATION_COUNT(call_ref_3(B, B), 1);
  VERIFY_EVALUATION_COUNT(call_ref_2(B.transpose(), B.transpose()), 0);
  VERIFY_EVALUATION_COUNT(call_ref_3(B.transpose(), B.transpose()), 0);
  VERIFY_EVALUATION_COUNT(call_ref_2(A * A, AA), 3);
  VERIFY_EVALUATION_COUNT(call_ref_3(A * A, AA), 3);

  VERIFY(!C.isCompressed());
  VERIFY_EVALUATION_COUNT(call_ref_3(C, C), 1);

  Ref<SparseMatrix<float>> Ar(A);
  VERIFY_IS_APPROX(Ar + Ar, A + A);
  VERIFY_EVALUATION_COUNT(call_ref_1(Ar, A), 0);
  VERIFY_EVALUATION_COUNT(call_ref_2(Ar, A), 0);
  VERIFY_EVALUATION_COUNT(call_ref_1(Ar.middleCols(1, 3), A.middleCols(1, 3)), 0);
  VERIFY_EVALUATION_COUNT(call_ref_2(Ar.middleCols(1, 3), A.middleCols(1, 3)), 0);
  VERIFY_EVALUATION_COUNT(call_ref_4(Ar.col(2), A.col(2)), 0);
  VERIFY_EVALUATION_COUNT(call_ref_5(Ar.col(2), A.col(2)), 0);

  Ref<SparseMatrix<float, RowMajor>> Br(B);
  VERIFY_EVALUATION_COUNT(call_ref_1(Br.transpose(), Br.transpose()), 0);
  VERIFY_EVALUATION_COUNT(call_ref_2(Br, Br), 1);
  VERIFY_EVALUATION_COUNT(call_ref_2(Br.transpose(), Br.transpose()), 0);
  VERIFY_EVALUATION_COUNT(call_ref_rm_1(Br.middleRows(1, 3), B.middleRows(1, 3)), 0);
  VERIFY_EVALUATION_COUNT(call_ref_rm_2(Br.middleRows(1, 3), B.middleRows(1, 3)), 0);
  VERIFY_EVALUATION_COUNT(call_ref_4(Br.row(2), B.row(2).transpose()), 0);
  VERIFY_EVALUATION_COUNT(call_ref_5(Br.row(2), B.row(2).transpose()), 0);

  auto Am = make_map(A);
  VERIFY_EVALUATION_COUNT(call_ref_1(Am.middleCols(1, 3), A.middleCols(1, 3)), 0);
  VERIFY_EVALUATION_COUNT(call_ref_2(Am.middleCols(1, 3), A.middleCols(1, 3)), 0);
  VERIFY_EVALUATION_COUNT(call_ref_4(Am.col(2), A.col(2)), 0);
  VERIFY_EVALUATION_COUNT(call_ref_5(Am.col(2), A.col(2)), 0);

  auto Bm = make_map(B);
  VERIFY_EVALUATION_COUNT(call_ref_rm_1(Bm.middleRows(1, 3), B.middleRows(1, 3)), 0);
  VERIFY_EVALUATION_COUNT(call_ref_rm_2(Bm.middleRows(1, 3), B.middleRows(1, 3)), 0);
  VERIFY_EVALUATION_COUNT(call_ref_4(Bm.row(2), B.row(2).transpose()), 0);
  VERIFY_EVALUATION_COUNT(call_ref_5(Bm.row(2), B.row(2).transpose()), 0);

  Ref<const SparseMatrix<float>> Arc(A);
  //   VERIFY_EVALUATION_COUNT( call_ref_1(Arc, Arc),  0); // does not compile on purpose
  VERIFY_EVALUATION_COUNT(call_ref_2(Arc, Arc), 0);

  VERIFY_EVALUATION_COUNT(call_ref_2(A.middleCols(1, 3), A.middleCols(1, 3)), 0);

  VERIFY_EVALUATION_COUNT(call_ref_2(A.col(2), A.col(2)), 0);
  VERIFY_EVALUATION_COUNT(call_ref_2(vc, vc), 0);
  VERIFY_EVALUATION_COUNT(call_ref_2(vr.transpose(), vr.transpose()), 0);
  VERIFY_EVALUATION_COUNT(call_ref_2(vr, vr.transpose()), 0);

  VERIFY_EVALUATION_COUNT(call_ref_2(A.block(1, 1, 3, 3), A.block(1, 1, 3, 3)),
                          1);  // should be 0 (allocate starts/nnz only)

  VERIFY_EVALUATION_COUNT(call_ref_4(vc, vc), 0);
  VERIFY_EVALUATION_COUNT(call_ref_4(vr, vr.transpose()), 0);
  VERIFY_EVALUATION_COUNT(call_ref_5(vc, vc), 0);
  VERIFY_EVALUATION_COUNT(call_ref_5(vr, vr.transpose()), 0);
  VERIFY_EVALUATION_COUNT(call_ref_4(A.col(2), A.col(2)), 0);
  VERIFY_EVALUATION_COUNT(call_ref_5(A.col(2), A.col(2)), 0);
  // VERIFY_EVALUATION_COUNT( call_ref_4(A.row(2), A.row(2).transpose()),  1); // does not compile on purpose
  VERIFY_EVALUATION_COUNT(call_ref_5(A.row(2), A.row(2).transpose()), 1);
}

EIGEN_DECLARE_TEST(sparse_ref) {
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(check_const_correctness(SparseMatrix<float>()));
    CALL_SUBTEST_1(check_const_correctness(SparseMatrix<double, RowMajor>()));
    CALL_SUBTEST_2(call_ref());
    CALL_SUBTEST_4(check_ref_slice_coeffs());
    CALL_SUBTEST_4(check_noncompressed_ref_slices());
    CALL_SUBTEST_4(check_noncompressed_ref_inner_vectors());

    CALL_SUBTEST_3(check_const_correctness(SparseVector<float>()));
    CALL_SUBTEST_3(check_const_correctness(SparseVector<double, RowMajor>()));
  }
}
