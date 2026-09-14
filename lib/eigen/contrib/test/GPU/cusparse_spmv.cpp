// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// Tests for gpu::SparseContext: GPU SpMV/SpMM via cuSPARSE.

#define EIGEN_USE_GPU
#include "main.h"
#include <Eigen/Sparse>
#include <contrib/Eigen/GPU>
#include "gpu_test_helpers.h"

using namespace Eigen;

// ---- Helper: build a random sparse matrix -----------------------------------

template <typename Scalar>
SparseMatrix<Scalar, ColMajor, int> make_sparse(Index rows, Index cols, double density = 0.1) {
  using SpMat = SparseMatrix<Scalar, ColMajor, int>;
  using RealScalar = typename NumTraits<Scalar>::Real;

  SpMat R(rows, cols);
  R.reserve(VectorXi::Constant(cols, static_cast<int>(rows * density) + 1));
  for (Index j = 0; j < cols; ++j) {
    for (Index i = 0; i < rows; ++i) {
      if ((std::rand() / double(RAND_MAX)) < density) {
        const RealScalar re = RealScalar(std::rand() / double(RAND_MAX) - 0.5);
        const RealScalar im = RealScalar(std::rand() / double(RAND_MAX) - 0.5);
        R.insert(i, j) = gpu_test::make_test_value<Scalar>(re, im);
      }
    }
  }
  R.makeCompressed();
  return R;
}

// ---- SpMV: y = A * x -------------------------------------------------------

template <typename Scalar>
void test_spmv(Index rows, Index cols) {
  using SpMat = SparseMatrix<Scalar, ColMajor, int>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  using RealScalar = typename NumTraits<Scalar>::Real;

  SpMat A = make_sparse<Scalar>(rows, cols);
  Vec x = Vec::Random(cols);

  gpu::SparseContext<Scalar> ctx;
  Vec y_gpu = ctx.multiply(A, x);
  Vec y_cpu = A * x;

  RealScalar tol = RealScalar(10) * RealScalar((std::max)(rows, cols)) * NumTraits<Scalar>::epsilon();
  VERIFY_IS_EQUAL(y_gpu.size(), rows);
  VERIFY((y_gpu - y_cpu).norm() / (y_cpu.norm() + RealScalar(1)) < tol);
}

// ---- SpMV with alpha/beta: y = alpha*A*x + beta*y ---------------------------

template <typename Scalar>
void test_spmv_alpha_beta(Index n) {
  using SpMat = SparseMatrix<Scalar, ColMajor, int>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  using RealScalar = typename NumTraits<Scalar>::Real;

  SpMat A = make_sparse<Scalar>(n, n);
  Vec x = Vec::Random(n);
  Vec y_init = Vec::Random(n);

  Scalar alpha(2);
  Scalar beta(3);

  Vec y_cpu = alpha * (A * x) + beta * y_init;

  gpu::SparseContext<Scalar> ctx;
  Vec y_gpu = y_init;
  ctx.multiply(A, x, y_gpu, alpha, beta);

  RealScalar tol = RealScalar(10) * RealScalar(n) * NumTraits<Scalar>::epsilon();
  VERIFY((y_gpu - y_cpu).norm() / (y_cpu.norm() + RealScalar(1)) < tol);
}

// ---- Transpose: y = A^T * x ------------------------------------------------

template <typename Scalar>
void test_spmv_transpose(Index rows, Index cols) {
  using SpMat = SparseMatrix<Scalar, ColMajor, int>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  using RealScalar = typename NumTraits<Scalar>::Real;

  SpMat A = make_sparse<Scalar>(rows, cols);
  Vec x = Vec::Random(rows);

  gpu::SparseContext<Scalar> ctx;
  Vec y_gpu = ctx.multiplyT(A, x);
  Vec y_cpu = A.transpose() * x;

  RealScalar tol = RealScalar(10) * RealScalar((std::max)(rows, cols)) * NumTraits<Scalar>::epsilon();
  VERIFY_IS_EQUAL(y_gpu.size(), cols);
  VERIFY((y_gpu - y_cpu).norm() / (y_cpu.norm() + RealScalar(1)) < tol);
}

// ---- SpMV adjoint: y = A^H * x ----------------------------------------------

template <typename Scalar>
void test_spmv_adjoint(Index rows, Index cols) {
  using SpMat = SparseMatrix<Scalar, ColMajor, int>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  using RealScalar = typename NumTraits<Scalar>::Real;

  SpMat A = make_sparse<Scalar>(rows, cols);
  Vec x = Vec::Random(rows);

  gpu::SparseContext<Scalar> ctx;
  Vec y_gpu = ctx.multiplyAdjoint(A, x);
  Vec y_cpu = A.adjoint() * x;

  RealScalar tol = RealScalar(10) * RealScalar((std::max)(rows, cols)) * NumTraits<Scalar>::epsilon();
  VERIFY_IS_EQUAL(y_gpu.size(), cols);
  VERIFY((y_gpu - y_cpu).norm() / (y_cpu.norm() + RealScalar(1)) < tol);
}

// ---- SpMM: Y = A * X (multiple RHS) ----------------------------------------

template <typename Scalar>
void test_spmm(Index rows, Index cols, Index nrhs) {
  using SpMat = SparseMatrix<Scalar, ColMajor, int>;
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using RealScalar = typename NumTraits<Scalar>::Real;

  SpMat A = make_sparse<Scalar>(rows, cols);
  Mat X = Mat::Random(cols, nrhs);

  gpu::SparseContext<Scalar> ctx;
  Mat Y_gpu = ctx.multiplyMat(A, X);
  Mat Y_cpu = A * X;

  RealScalar tol = RealScalar(10) * RealScalar((std::max)(rows, cols)) * NumTraits<Scalar>::epsilon();
  VERIFY_IS_EQUAL(Y_gpu.rows(), rows);
  VERIFY_IS_EQUAL(Y_gpu.cols(), nrhs);
  VERIFY((Y_gpu - Y_cpu).norm() / (Y_cpu.norm() + RealScalar(1)) < tol);
}

// ---- SpMM transpose: Y = A^T * X --------------------------------------------

template <typename Scalar>
void test_spmm_transpose(Index rows, Index cols, Index nrhs) {
  using SpMat = SparseMatrix<Scalar, ColMajor, int>;
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using RealScalar = typename NumTraits<Scalar>::Real;

  SpMat A = make_sparse<Scalar>(rows, cols);
  Mat X = Mat::Random(rows, nrhs);

  gpu::SparseContext<Scalar> ctx;
  Mat Y_gpu = ctx.multiplyMat(A, X, gpu::GpuOp::Trans);
  Mat Y_cpu = A.transpose() * X;

  RealScalar tol = RealScalar(10) * RealScalar((std::max)(rows, cols)) * NumTraits<Scalar>::epsilon();
  VERIFY_IS_EQUAL(Y_gpu.rows(), cols);
  VERIFY_IS_EQUAL(Y_gpu.cols(), nrhs);
  VERIFY((Y_gpu - Y_cpu).norm() / (Y_cpu.norm() + RealScalar(1)) < tol);
}

// ---- Identity matrix: I * x = x --------------------------------------------

template <typename Scalar>
void test_identity(Index n) {
  using SpMat = SparseMatrix<Scalar, ColMajor, int>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  using RealScalar = typename NumTraits<Scalar>::Real;

  // Build sparse identity.
  SpMat eye(n, n);
  eye.setIdentity();
  eye.makeCompressed();

  Vec x = Vec::Random(n);

  gpu::SparseContext<Scalar> ctx;
  Vec y = ctx.multiply(eye, x);

  RealScalar tol = NumTraits<Scalar>::epsilon();
  VERIFY((y - x).norm() < tol);
}

// ---- Pattern replacement at unchanged host pointers -------------------------

// Regression test: assigning a different same-shape/same-nnz sparsity pattern
// to the same SparseMatrix reuses its index allocations, so every host
// pointer the context saw before is unchanged. A structure cache keyed on
// pointer identity silently keeps the stale device indices and computes with
// the old pattern (here: returns x itself instead of the permuted x).
template <typename Scalar>
void test_pattern_replacement(Index n) {
  using SpMat = SparseMatrix<Scalar, ColMajor, int>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  using RealScalar = typename NumTraits<Scalar>::Real;

  SpMat A(n, n);
  A.setIdentity();
  A.makeCompressed();
  Vec x = Vec::Random(n);

  gpu::SparseContext<Scalar> ctx;
  Vec y1 = ctx.multiply(A, x);
  RealScalar tol = NumTraits<Scalar>::epsilon();
  VERIFY((y1 - x).norm() < tol);

  // Warm-up repeat: arms any structure cache keyed on the host arrays before
  // the pattern is replaced.
  Vec y1b = ctx.multiply(A, x);
  VERIFY((y1b - x).norm() < tol);

  // Value-only update at unchanged structure is picked up (values are
  // re-uploaded on every host-input call).
  A.coeffRef(0, 0) = Scalar(2);
  Vec y1c = ctx.multiply(A, x);
  Vec y1c_ref = A * x;
  VERIFY((y1c - y1c_ref).norm() < tol);

  // Replace the pattern with a cyclic permutation: one entry per column at
  // row (j+1) % n, all values 1, so shape and nnz match the identity and the
  // result differs from x only through the *index* arrays.
  const int* inner_before = A.innerIndexPtr();
  SpMat P(n, n);
  P.reserve(VectorXi::Constant(n, 1));
  for (Index j = 0; j < n; ++j) P.insert((j + 1) % n, j) = Scalar(1);
  P.makeCompressed();
  A = P;
  // Precondition this regression relies on: the assignment reused A's index
  // storage. If SparseMatrix assignment ever stops doing so, rewrite the
  // pattern in place (e.g. through innerIndexPtr()) to keep the coverage.
  VERIFY(A.innerIndexPtr() == inner_before);

  Vec y2 = ctx.multiply(A, x);
  Vec y2_ref = A * x;
  VERIFY((y2 - y2_ref).norm() < tol);
}

// ---- Context reuse ----------------------------------------------------------

template <typename Scalar>
void test_reuse(Index n) {
  using SpMat = SparseMatrix<Scalar, ColMajor, int>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  using RealScalar = typename NumTraits<Scalar>::Real;

  gpu::SparseContext<Scalar> ctx;
  RealScalar tol = RealScalar(10) * RealScalar(n) * NumTraits<Scalar>::epsilon();

  for (int trial = 0; trial < 3; ++trial) {
    SpMat A = make_sparse<Scalar>(n, n);
    Vec x = Vec::Random(n);
    Vec y_gpu = ctx.multiply(A, x);
    Vec y_cpu = A * x;
    VERIFY((y_gpu - y_cpu).norm() / (y_cpu.norm() + RealScalar(1)) < tol);
  }
}

// ---- Empty ------------------------------------------------------------------

template <typename Scalar>
void test_empty() {
  using SpMat = SparseMatrix<Scalar, ColMajor, int>;
  using Vec = Matrix<Scalar, Dynamic, 1>;

  SpMat A(0, 0);
  A.makeCompressed();
  Vec x(0);

  gpu::SparseContext<Scalar> ctx;
  Vec y = ctx.multiply(A, x);
  VERIFY_IS_EQUAL(y.size(), 0);
}

// ---- gpu::DeviceMatrix SpMV (no host roundtrip) ----------------------------------

template <typename Scalar>
void test_spmv_device(Index n) {
  using SpMat = SparseMatrix<Scalar, ColMajor, int>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  using RealScalar = typename NumTraits<Scalar>::Real;

  SpMat A = make_sparse<Scalar>(n, n);
  Vec x = Vec::Random(n);

  // Use shared gpu::Context for same-stream execution.
  gpu::Context gpu_ctx;
  gpu::SparseContext<Scalar> ctx(gpu_ctx);

  auto d_x = gpu::DeviceMatrix<Scalar>::fromHost(x, gpu_ctx.stream());
  gpu::DeviceMatrix<Scalar> d_y;

  ctx.multiply(A, d_x, d_y);

  Vec y_gpu = d_y.toHost(gpu_ctx.stream());
  Vec y_cpu = A * x;

  RealScalar tol = RealScalar(10) * RealScalar(n) * NumTraits<Scalar>::epsilon();
  VERIFY((y_gpu - y_cpu).norm() / (y_cpu.norm() + RealScalar(1)) < tol);
}

// ---- Expression syntax: d_y = d_A * d_x ------------------------------------

template <typename Scalar>
void test_spmv_expr(Index n) {
  using SpMat = SparseMatrix<Scalar, ColMajor, int>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  using RealScalar = typename NumTraits<Scalar>::Real;

  SpMat A = make_sparse<Scalar>(n, n);
  Vec x = Vec::Random(n);

  gpu::Context gpu_ctx;
  gpu::SparseContext<Scalar> ctx(gpu_ctx);

  // Upload sparse matrix and create device view.
  auto d_A = ctx.deviceView(A);

  // Upload x.
  auto d_x = gpu::DeviceMatrix<Scalar>::fromHost(x, gpu_ctx.stream());

  // Expression syntax: d_y = d_A * d_x
  gpu::DeviceMatrix<Scalar> d_y;
  d_y = d_A * d_x;

  // Also test with noalias():
  gpu::DeviceMatrix<Scalar> d_tmp;
  d_tmp.noalias() = d_A * d_x;

  Vec y_gpu = d_y.toHost(gpu_ctx.stream());
  Vec tmp_gpu = d_tmp.toHost(gpu_ctx.stream());
  Vec y_cpu = A * x;

  RealScalar tol = RealScalar(10) * RealScalar(n) * NumTraits<Scalar>::epsilon();
  VERIFY((y_gpu - y_cpu).norm() / (y_cpu.norm() + RealScalar(1)) < tol);
  VERIFY((tmp_gpu - y_cpu).norm() / (y_cpu.norm() + RealScalar(1)) < tol);
}

// ---- deviceView overwrite: second view replaces first -----------------------

template <typename Scalar>
void test_deviceview_overwrite(Index n) {
  using SpMat = SparseMatrix<Scalar, ColMajor, int>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  using RealScalar = typename NumTraits<Scalar>::Real;

  SpMat A1 = make_sparse<Scalar>(n, n);
  SpMat A2 = make_sparse<Scalar>(n, n);  // different random matrix

  Vec x = Vec::Random(n);

  gpu::Context gpu_ctx;
  gpu::SparseContext<Scalar> ctx(gpu_ctx);

  // First view: A1.
  auto d_A1 = ctx.deviceView(A1);
  auto d_x = gpu::DeviceMatrix<Scalar>::fromHost(x, gpu_ctx.stream());
  gpu::DeviceMatrix<Scalar> d_y1;
  d_y1 = d_A1 * d_x;
  Vec y1_gpu = d_y1.toHost(gpu_ctx.stream());
  Vec y1_cpu = A1 * x;
  RealScalar tol = RealScalar(10) * RealScalar(n) * NumTraits<Scalar>::epsilon();
  VERIFY((y1_gpu - y1_cpu).norm() / (y1_cpu.norm() + RealScalar(1)) < tol);

  // Second view overwrites first: now uses A2.
  auto d_A2 = ctx.deviceView(A2);
  gpu::DeviceMatrix<Scalar> d_y2;
  d_y2 = d_A2 * d_x;
  Vec y2_gpu = d_y2.toHost(gpu_ctx.stream());
  Vec y2_cpu = A2 * x;
  VERIFY((y2_gpu - y2_cpu).norm() / (y2_cpu.norm() + RealScalar(1)) < tol);
}

// ---- Device-resident SpMM + GpuOp device multiply -----------------------------

template <typename Scalar>
void test_device_spmm(Index n, Index nrhs) {
  using SpMat = SparseMatrix<Scalar, ColMajor, int>;
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using RealScalar = typename NumTraits<Scalar>::Real;

  SpMat A = make_sparse<Scalar>(n, n);
  Mat X = Mat::Random(n, nrhs);

  gpu::Context gctx;
  gpu::SparseContext<Scalar> ctx(gctx);
  auto view = ctx.deviceView(A);
  VERIFY(view.generation() == ctx.uploadGeneration());

  auto d_X = gpu::DeviceMatrix<Scalar>::fromHost(X, gctx.stream());
  gpu::DeviceMatrix<Scalar> d_Y = view * d_X;  // nrhs > 1 -> SpMM
  Mat Y_ref = A * X;
  RealScalar tol = RealScalar(10) * RealScalar(n) * NumTraits<Scalar>::epsilon();
  VERIFY((d_Y.toHost() - Y_ref).norm() / (Y_ref.norm() + RealScalar(1)) < tol);
}

template <typename Scalar>
void test_device_multiply_gpuop(Index n) {
  using SpMat = SparseMatrix<Scalar, ColMajor, int>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  using RealScalar = typename NumTraits<Scalar>::Real;

  SpMat A = make_sparse<Scalar>(n, n);
  Vec x = Vec::Random(n);

  gpu::Context gctx;
  gpu::SparseContext<Scalar> ctx(gctx);
  auto d_x = gpu::DeviceMatrix<Scalar>::fromHost(x, gctx.stream());
  gpu::DeviceMatrix<Scalar> d_y;
  ctx.multiply(A, d_x, d_y, Scalar(1), Scalar(0), gpu::GpuOp::Trans);
  Vec y_ref = A.transpose() * x;
  RealScalar tol = RealScalar(10) * RealScalar(n) * NumTraits<Scalar>::epsilon();
  VERIFY((d_y.toHost() - y_ref).norm() / (y_ref.norm() + RealScalar(1)) < tol);
}

// ---- Per-scalar driver ------------------------------------------------------

template <typename Scalar>
void test_scalar() {
  CALL_SUBTEST(test_device_spmm<Scalar>(64, 5));
  CALL_SUBTEST(test_device_multiply_gpuop<Scalar>(64));
  CALL_SUBTEST(test_spmv<Scalar>(64, 64));
  CALL_SUBTEST(test_spmv<Scalar>(128, 64));  // non-square
  CALL_SUBTEST(test_spmv<Scalar>(64, 128));  // wide
  CALL_SUBTEST(test_spmv_alpha_beta<Scalar>(64));
  CALL_SUBTEST(test_spmv_transpose<Scalar>(128, 64));
  // cuSPARSE < 12 cannot represent A^H * x for complex scalars with the
  // CSR-of-A^T trick used by SparseContext; SparseContext asserts in that
  // case. Real-scalar ConjTrans is demoted to Trans and works fine.
#if !defined(CUSPARSE_VERSION) || CUSPARSE_VERSION >= 12000
  CALL_SUBTEST(test_spmv_adjoint<Scalar>(128, 64));
#else
  if (!NumTraits<Scalar>::IsComplex) CALL_SUBTEST(test_spmv_adjoint<Scalar>(128, 64));
#endif
  CALL_SUBTEST(test_spmm<Scalar>(64, 64, 4));
  CALL_SUBTEST(test_spmm_transpose<Scalar>(128, 64, 4));
  CALL_SUBTEST(test_identity<Scalar>(64));
  CALL_SUBTEST(test_pattern_replacement<Scalar>(64));
  CALL_SUBTEST(test_reuse<Scalar>(64));
  CALL_SUBTEST(test_empty<Scalar>());
  CALL_SUBTEST(test_spmv_device<Scalar>(64));
  CALL_SUBTEST(test_spmv_expr<Scalar>(64));
  CALL_SUBTEST(test_deviceview_overwrite<Scalar>(64));
}

EIGEN_DECLARE_TEST(gpu_cusparse_spmv) {
  gpu_test::require_cusparse_context();

  // Split by scalar so each part compiles in parallel.
  CALL_SUBTEST_1(test_scalar<float>());
  CALL_SUBTEST_2(test_scalar<double>());
  CALL_SUBTEST_3(test_scalar<std::complex<float>>());
  CALL_SUBTEST_4(test_scalar<std::complex<double>>());
}
