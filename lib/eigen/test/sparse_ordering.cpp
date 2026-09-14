// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "sparse.h"
#include <Eigen/OrderingMethods>

template <typename Idx>
void verify_is_permutation(const PermutationMatrix<Dynamic, Dynamic, Idx>& perm) {
  typedef Matrix<Idx, Dynamic, 1> Vec;
  Vec sorted = perm.indices();
  std::sort(sorted.data(), sorted.data() + sorted.size());
  Vec expected = Vec::LinSpaced(perm.size(), 0, Idx(perm.size() - 1));
  VERIFY_IS_EQUAL(sorted, expected);
}

template <typename Scalar>
void test_colamd_uncompressed() {
  typedef int Idx;
  typedef SparseMatrix<Scalar, ColMajor, Idx> SpMat;
  const int n = 32;
  SpMat A(n, n);
  A.reserve(VectorXi::Constant(n, 5));
  for (int j = 0; j < n; ++j) {
    A.insert(j, j) = Scalar(2) + Scalar(j);
    if (j > 0) A.insert(j - 1, j) = Scalar(-1);
    if (j + 1 < n) A.insert(j + 1, j) = Scalar(-1);
  }
  A.insert(0, n - 1) = Scalar(0.5);
  A.insert(n - 1, 0) = Scalar(0.25);
  VERIFY(!A.isCompressed());

  // Regression for #649: COLAMD on uncompressed input used to silently corrupt
  // the permutation in release builds.
  PermutationMatrix<Dynamic, Dynamic, Idx> perm_u, perm_c;
  COLAMDOrdering<Idx> ord;
  ord(A, perm_u);
  verify_is_permutation(perm_u);

  SpMat B = A;
  B.makeCompressed();
  ord(B, perm_c);
  verify_is_permutation(perm_c);

  VERIFY_IS_EQUAL(perm_u.indices(), perm_c.indices());
}

void test_colamd_mixed_index_types() {
  typedef SparseMatrix<double, ColMajor, int> SpMat;
  const int n = 8;
  SpMat A(n, n);
  A.reserve(VectorXi::Constant(n, 3));
  for (int j = 0; j < n; ++j) {
    A.insert(j, j) = 2.0 + j;
    if (j > 0) A.insert(j - 1, j) = -1.0;
    if (j + 1 < n) A.insert(j + 1, j) = -1.0;
  }
  A.makeCompressed();

  // COLAMDOrdering<StorageIndex> is a public API choice independent of the
  // source matrix's StorageIndex.  Keep accepting mixed index types.
  PermutationMatrix<Dynamic, Dynamic, long> perm;
  COLAMDOrdering<long> ord;
  ord(A, perm);
  verify_is_permutation(perm);
}

template <typename Scalar>
void test_amd_uncompressed() {
  typedef int Idx;
  typedef SparseMatrix<Scalar, ColMajor, Idx> SpMat;
  const int n = 24;
  SpMat A(n, n);
  A.reserve(VectorXi::Constant(n, 4));
  for (int j = 0; j < n; ++j) {
    A.insert(j, j) = Scalar(2) + Scalar(j);
    if (j > 0) {
      A.insert(j - 1, j) = Scalar(-1);
      A.insert(j, j - 1) = Scalar(-1);
    }
  }
  VERIFY(!A.isCompressed());

  PermutationMatrix<Dynamic, Dynamic, Idx> perm;
  AMDOrdering<Idx> ord;
  ord(A, perm);
  verify_is_permutation(perm);
}

EIGEN_DECLARE_TEST(sparse_ordering) {
  for (int i = 0; i < g_repeat; ++i) {
    CALL_SUBTEST_1(test_colamd_uncompressed<double>());
    CALL_SUBTEST_2(test_colamd_uncompressed<std::complex<double> >());
    CALL_SUBTEST_3(test_amd_uncompressed<double>());
    CALL_SUBTEST_4(test_colamd_mixed_index_types());
  }
}
