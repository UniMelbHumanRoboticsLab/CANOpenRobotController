// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// Tests for gpu::DeviceMatrix and HostTransfer: typed RAII GPU memory wrapper.
// No cuSOLVER dependency — only CUDA runtime.

#define EIGEN_USE_GPU
#include "main.h"
#include <Eigen/Sparse>
#include <contrib/Eigen/GPU>

#include "./gpu_test_helpers.h"

using namespace Eigen;

// ---- Default construction ---------------------------------------------------

void test_default_construct() {
  gpu::DeviceMatrix<double> dm;
  VERIFY(dm.empty());
  VERIFY_IS_EQUAL(dm.rows(), 0);
  VERIFY_IS_EQUAL(dm.cols(), 0);
  VERIFY(dm.data() == nullptr);
  VERIFY_IS_EQUAL(dm.sizeInBytes(), size_t(0));
}

// ---- Allocate uninitialized -------------------------------------------------

template <typename Scalar>
void test_allocate(Index rows, Index cols) {
  gpu::DeviceMatrix<Scalar> dm(rows, cols);
  VERIFY(!dm.empty());
  VERIFY_IS_EQUAL(dm.rows(), rows);
  VERIFY_IS_EQUAL(dm.cols(), cols);
  VERIFY(dm.data() != nullptr);
  VERIFY_IS_EQUAL(dm.sizeInBytes(), size_t(rows) * size_t(cols) * sizeof(Scalar));
}

// ---- fromHost / toHost roundtrip (synchronous) ------------------------------

template <typename Scalar>
void test_roundtrip(Index rows, Index cols) {
  using MatrixType = Eigen::Matrix<Scalar, Dynamic, Dynamic>;
  MatrixType host = MatrixType::Random(rows, cols);

  auto dm = gpu::DeviceMatrix<Scalar>::fromHost(host);
  VERIFY_IS_EQUAL(dm.rows(), rows);
  VERIFY_IS_EQUAL(dm.cols(), cols);
  VERIFY(!dm.empty());

  MatrixType result = dm.toHost();
  VERIFY_IS_EQUAL(result.rows(), rows);
  VERIFY_IS_EQUAL(result.cols(), cols);
  VERIFY_IS_APPROX(result, host);
}

// ---- fromHost with a non-contiguous source ----------------------------------

// fromHost() binds its argument through Ref<const PlainMatrix>, whose default
// OuterStride<> matches any outer stride: a block of a larger matrix binds in
// place and keeps the parent's stride instead of being materialized. Uploading
// rows*cols contiguous elements from that pointer reads across column
// boundaries, so the upload must honour outerStride().
template <typename Scalar>
void test_fromhost_strided(Index rows, Index cols) {
  using MatrixType = Eigen::Matrix<Scalar, Dynamic, Dynamic, ColMajor>;
  using RowMajorMatrix = Eigen::Matrix<Scalar, Dynamic, Dynamic, RowMajor>;
  using ArrayType = Eigen::Array<Scalar, Dynamic, Dynamic, ColMajor>;

  const MatrixType host = MatrixType::Random(rows, cols);

  // outerStride() != rows(): binds in place with the parent's stride.
  MatrixType padded = MatrixType::Random(rows + 3, cols + 5);
  padded.block(1, 2, rows, cols) = host;
  VERIFY_IS_APPROX(gpu::DeviceMatrix<Scalar>::fromHost(padded.block(1, 2, rows, cols)).toHost(), host);

  // Eigen permits a negative outer stride, which cudaMemcpy2DAsync cannot
  // represent as a source pitch. Store the columns in reverse physical order
  // so the negatively-strided logical view is equal to host.
  MatrixType reversed_storage(rows, cols);
  for (Index col = 0; col < cols; ++col) reversed_storage.col(cols - 1 - col) = host.col(col);
  using NegativeStrideMap = Eigen::Map<const MatrixType, Eigen::Unaligned, Eigen::OuterStride<Dynamic>>;
  const NegativeStrideMap negative_stride(reversed_storage.data() + (cols - 1) * rows, rows, cols,
                                          Eigen::OuterStride<Dynamic>(-rows));
  VERIFY_IS_APPROX(gpu::DeviceMatrix<Scalar>::fromHost(negative_stride).toHost(), host);

  // Row-major and unevaluated expressions materialize into Ref's own storage,
  // which is contiguous; pinned here so the two paths stay covered together.
  const RowMajorMatrix row_major = host;
  VERIFY_IS_APPROX(gpu::DeviceMatrix<Scalar>::fromHost(row_major).toHost(), host);
  VERIFY_IS_APPROX(gpu::DeviceMatrix<Scalar>::fromHost(Scalar(2) * host).toHost(), MatrixType(Scalar(2) * host));

  // Array expressions are dense inputs too.
  const ArrayType array = host.array();
  VERIFY_IS_APPROX(gpu::DeviceMatrix<Scalar>::fromHost(array).toHost(), host);
}

// ---- fromHostAsync / toHostAsync roundtrip -----------------------------------

template <typename Scalar>
void test_roundtrip_async(Index rows, Index cols) {
  using MatrixType = Eigen::Matrix<Scalar, Dynamic, Dynamic>;
  MatrixType host = MatrixType::Random(rows, cols);

  cudaStream_t stream;
  EIGEN_CUDA_RUNTIME_CHECK(cudaStreamCreate(&stream));

  // Async upload from raw pointer.
  auto dm = gpu::DeviceMatrix<Scalar>::fromHostAsync(host.data(), rows, cols, stream);
  VERIFY_IS_EQUAL(dm.rows(), rows);
  VERIFY_IS_EQUAL(dm.cols(), cols);

  // Async download via HostTransfer future.
  auto transfer = dm.toHostAsync(stream);

  // get() blocks and returns the matrix.
  MatrixType result = transfer.get();
  VERIFY_IS_APPROX(result, host);

  EIGEN_CUDA_RUNTIME_CHECK(cudaStreamDestroy(stream));

  cudaStream_t producer_stream;
  cudaStream_t consumer_stream;
  EIGEN_CUDA_RUNTIME_CHECK(cudaStreamCreate(&producer_stream));
  EIGEN_CUDA_RUNTIME_CHECK(cudaStreamCreate(&consumer_stream));

  auto cross_stream_dm = gpu::DeviceMatrix<Scalar>::fromHostAsync(host.data(), rows, cols, producer_stream);
  auto cross_stream_transfer = cross_stream_dm.toHostAsync(consumer_stream);
  MatrixType cross_stream_result = cross_stream_transfer.get();
  VERIFY_IS_APPROX(cross_stream_result, host);

  EIGEN_CUDA_RUNTIME_CHECK(cudaStreamDestroy(consumer_stream));
  EIGEN_CUDA_RUNTIME_CHECK(cudaStreamDestroy(producer_stream));
}

// ---- HostTransfer::ready() and idempotent get() -----------------------------

void test_host_transfer_ready() {
  using MatrixType = Eigen::Matrix<double, Dynamic, Dynamic>;
  MatrixType host = MatrixType::Random(100, 100);

  auto dm = gpu::DeviceMatrix<double>::fromHost(host);
  auto transfer = dm.toHostAsync();

  // After get(), ready() must return true.
  MatrixType result = transfer.get();
  VERIFY(transfer.ready());
  VERIFY_IS_APPROX(result, host);

  // get() is idempotent.
  MatrixType& result2 = transfer.get();
  VERIFY_IS_APPROX(result2, host);
}

// ---- HostTransfer move ------------------------------------------------------

void test_host_transfer_move() {
  using MatrixType = Eigen::Matrix<double, Dynamic, Dynamic>;
  MatrixType host = MatrixType::Random(50, 50);

  auto dm = gpu::DeviceMatrix<double>::fromHost(host);
  auto transfer = dm.toHostAsync();

  gpu::HostTransfer<double> moved(std::move(transfer));
  MatrixType result = moved.get();
  VERIFY_IS_APPROX(result, host);
}

// ---- HostTransfer move assign -----------------------------------------------

// Move assignment destroys the destination's own transfer event before adopting
// the source's; move construction never reaches that branch because it starts
// from no event at all. Both destinations below hold a live event_, one already
// consumed by get() and one not, so the branch runs in both synced_ states.
void test_host_transfer_move_assign() {
  using MatrixType = Eigen::Matrix<double, Dynamic, Dynamic>;
  MatrixType host_a = MatrixType::Random(50, 50);
  MatrixType host_b = MatrixType::Random(50, 50);

  cudaStream_t stream;
  EIGEN_CUDA_RUNTIME_CHECK(cudaStreamCreate(&stream));

  auto dm_a = gpu::DeviceMatrix<double>::fromHost(host_a, stream);
  auto dm_b = gpu::DeviceMatrix<double>::fromHost(host_b, stream);

  // Destination already consumed: get() leaves event_ live and synced_ set.
  {
    auto dest = dm_a.toHostAsync(stream);
    VERIFY_IS_APPROX(dest.get(), host_a);

    auto src = dm_b.toHostAsync(stream);
    dest = std::move(src);

    // The adopted transfer still completes and yields the source's data.
    VERIFY_IS_APPROX(dest.get(), host_b);
    // The moved-from future owns no event and answers ready() without CUDA.
    VERIFY(src.ready());
  }

  // Destination never consumed: event_ live and synced_ still false. Sync first
  // so no copy is in flight into the staging buffer the assignment frees.
  {
    auto dest = dm_a.toHostAsync(stream);
    auto src = dm_b.toHostAsync(stream);
    EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(stream));

    dest = std::move(src);
    VERIFY_IS_APPROX(dest.get(), host_b);
    VERIFY(src.ready());
  }

  EIGEN_CUDA_RUNTIME_CHECK(cudaStreamDestroy(stream));
}

// ---- clone() produces independent copy --------------------------------------

template <typename Scalar>
void test_clone(Index rows, Index cols) {
  using MatrixType = Eigen::Matrix<Scalar, Dynamic, Dynamic>;
  MatrixType host = MatrixType::Random(rows, cols);

  auto dm = gpu::DeviceMatrix<Scalar>::fromHost(host);
  auto cloned = dm.clone();

  // Overwrite original with different data.
  MatrixType other = MatrixType::Random(rows, cols);
  dm = gpu::DeviceMatrix<Scalar>::fromHost(other);

  // Clone still holds the original data.
  MatrixType clone_result = cloned.toHost();
  VERIFY_IS_APPROX(clone_result, host);

  // Original holds the new data.
  MatrixType dm_result = dm.toHost();
  VERIFY_IS_APPROX(dm_result, other);
}

// ---- Move construct ---------------------------------------------------------

template <typename Scalar>
void test_move_construct(Index rows, Index cols) {
  using MatrixType = Eigen::Matrix<Scalar, Dynamic, Dynamic>;
  MatrixType host = MatrixType::Random(rows, cols);

  auto dm = gpu::DeviceMatrix<Scalar>::fromHost(host);
  gpu::DeviceMatrix<Scalar> moved(std::move(dm));

  VERIFY(dm.empty());
  VERIFY(dm.data() == nullptr);

  VERIFY_IS_EQUAL(moved.rows(), rows);
  VERIFY_IS_EQUAL(moved.cols(), cols);
  MatrixType result = moved.toHost();
  VERIFY_IS_APPROX(result, host);
}

// ---- Move assign ------------------------------------------------------------

template <typename Scalar>
void test_move_assign(Index rows, Index cols) {
  using MatrixType = Eigen::Matrix<Scalar, Dynamic, Dynamic>;
  MatrixType host = MatrixType::Random(rows, cols);

  auto dm = gpu::DeviceMatrix<Scalar>::fromHost(host);
  gpu::DeviceMatrix<Scalar> dest;
  dest = std::move(dm);

  VERIFY(dm.empty());
  VERIFY_IS_EQUAL(dest.rows(), rows);
  MatrixType result = dest.toHost();
  VERIFY_IS_APPROX(result, host);
}

// ---- Move assign over a live ready event ------------------------------------

// A destination produced asynchronously owns a ready event, which the
// assignment must destroy before adopting the source's. test_move_assign()
// cannot reach that branch: its destination is default-constructed and its
// source comes from the synchronous fromHost(), so neither records an event.
template <typename Scalar>
void test_move_assign_async(Index rows, Index cols) {
  using MatrixType = Eigen::Matrix<Scalar, Dynamic, Dynamic>;
  MatrixType host_a = MatrixType::Random(rows, cols);
  MatrixType host_b = MatrixType::Random(rows, cols);

  cudaStream_t producer_stream;
  cudaStream_t consumer_stream;
  EIGEN_CUDA_RUNTIME_CHECK(cudaStreamCreate(&producer_stream));
  EIGEN_CUDA_RUNTIME_CHECK(cudaStreamCreate(&consumer_stream));

  // Destination: upload finished, but its ready event is still live. Draining
  // the stream keeps the assignment from freeing a buffer still being written.
  auto dest = gpu::DeviceMatrix<Scalar>::fromHostAsync(host_a.data(), rows, cols, producer_stream);
  EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(producer_stream));

  // Source: upload still in flight, so the adopted event has real work behind it.
  auto src = gpu::DeviceMatrix<Scalar>::fromHostAsync(host_b.data(), rows, cols, producer_stream);
  dest = std::move(src);

  VERIFY(src.empty());
  VERIFY(src.data() == nullptr);
  VERIFY_IS_EQUAL(dest.rows(), rows);
  VERIFY_IS_EQUAL(dest.cols(), cols);

  // Reading back on the other stream waits on the adopted event, so an event
  // lost or left behind by the assignment shows up as wrong data.
  MatrixType result = dest.toHost(consumer_stream);
  VERIFY_IS_APPROX(result, host_b);

  EIGEN_CUDA_RUNTIME_CHECK(cudaStreamDestroy(consumer_stream));
  EIGEN_CUDA_RUNTIME_CHECK(cudaStreamDestroy(producer_stream));
}

// ---- resize() ---------------------------------------------------------------

void test_resize() {
  gpu::DeviceMatrix<double> dm(10, 20);
  VERIFY_IS_EQUAL(dm.rows(), 10);
  VERIFY_IS_EQUAL(dm.cols(), 20);

  dm.resize(50, 30);
  VERIFY_IS_EQUAL(dm.rows(), 50);
  VERIFY_IS_EQUAL(dm.cols(), 30);
  VERIFY(dm.data() != nullptr);

  // Resize to same dimensions is a no-op.
  double* ptr_before = dm.data();
  dm.resize(50, 30);
  VERIFY(dm.data() == ptr_before);
}

// ---- Empty / 0x0 matrix -----------------------------------------------------

void test_empty() {
  using MatrixType = Eigen::Matrix<double, Dynamic, Dynamic>;
  MatrixType empty_mat(0, 0);

  auto dm = gpu::DeviceMatrix<double>::fromHost(empty_mat);
  VERIFY(dm.empty());
  VERIFY_IS_EQUAL(dm.rows(), 0);
  VERIFY_IS_EQUAL(dm.cols(), 0);

  MatrixType result = dm.toHost();
  VERIFY_IS_EQUAL(result.rows(), 0);
  VERIFY_IS_EQUAL(result.cols(), 0);
}

// ---- Per-scalar driver ------------------------------------------------------

template <typename Scalar>
void test_scalar() {
  // Square.
  CALL_SUBTEST(test_roundtrip<Scalar>(1, 1));
  CALL_SUBTEST(test_roundtrip<Scalar>(64, 64));
  CALL_SUBTEST(test_roundtrip<Scalar>(256, 256));

  // Rectangular.
  CALL_SUBTEST(test_roundtrip<Scalar>(100, 7));
  CALL_SUBTEST(test_roundtrip<Scalar>(7, 100));

  // Async roundtrip.
  CALL_SUBTEST(test_roundtrip_async<Scalar>(64, 64));
  CALL_SUBTEST(test_roundtrip_async<Scalar>(100, 7));

  // Non-contiguous / non-column-major sources.
  CALL_SUBTEST(test_fromhost_strided<Scalar>(8, 8));
  CALL_SUBTEST(test_fromhost_strided<Scalar>(64, 17));

  CALL_SUBTEST(test_clone<Scalar>(64, 64));
  CALL_SUBTEST(test_move_construct<Scalar>(64, 64));
  CALL_SUBTEST(test_move_assign<Scalar>(64, 64));
  CALL_SUBTEST(test_move_assign_async<Scalar>(64, 64));
}

// ---- BLAS-1: dot product ----------------------------------------------------

template <typename Scalar>
void test_blas1(Index n) {
  using Vec = Matrix<Scalar, Dynamic, 1>;
  using RealScalar = typename NumTraits<Scalar>::Real;

  // All BLAS-1 ops share one gpu::Context — same stream, zero event overhead.
  gpu::Context ctx;

  RealScalar tol = RealScalar(10) * RealScalar(n) * NumTraits<Scalar>::epsilon();

  // dot
  {
    Vec a = Vec::Random(n);
    Vec b = Vec::Random(n);
    auto d_a = gpu::DeviceMatrix<Scalar>::fromHost(a, ctx.stream());
    auto d_b = gpu::DeviceMatrix<Scalar>::fromHost(b, ctx.stream());
    Scalar gpu_dot = d_a.dot(ctx, d_b);
    Scalar cpu_dot = a.dot(b);
    VERIFY(numext::abs(gpu_dot - cpu_dot) < tol * numext::abs(cpu_dot) + tol);
  }

  // norm / squaredNorm
  {
    Vec a = Vec::Random(n);
    auto d_a = gpu::DeviceMatrix<Scalar>::fromHost(a, ctx.stream());
    RealScalar gpu_norm = d_a.norm(ctx);
    RealScalar cpu_norm = a.norm();
    VERIFY(numext::abs(gpu_norm - cpu_norm) < tol * cpu_norm + tol);
    RealScalar gpu_sqnorm = d_a.squaredNorm(ctx);
    RealScalar cpu_sqnorm = a.squaredNorm();
    VERIFY(numext::abs(gpu_sqnorm - cpu_sqnorm) < tol * cpu_sqnorm + tol);
  }

  // addScaled (axpy)
  {
    Vec x = Vec::Random(n);
    Vec y = Vec::Random(n);
    Scalar alpha(2.5);
    Vec y_ref = y + alpha * x;
    auto d_y = gpu::DeviceMatrix<Scalar>::fromHost(y, ctx.stream());
    auto d_x = gpu::DeviceMatrix<Scalar>::fromHost(x, ctx.stream());
    d_y.addScaled(ctx, alpha, d_x);
    Vec y_gpu = d_y.toHost(ctx.stream());
    VERIFY((y_gpu - y_ref).norm() < tol * y_ref.norm() + tol);
  }

  // scale (scal)
  {
    Vec x = Vec::Random(n);
    Scalar alpha(3.0);
    Vec x_ref = alpha * x;
    auto d_x = gpu::DeviceMatrix<Scalar>::fromHost(x, ctx.stream());
    d_x.scale(ctx, alpha);
    Vec x_gpu = d_x.toHost(ctx.stream());
    VERIFY((x_gpu - x_ref).norm() < tol * x_ref.norm() + tol);
  }

  // copyFrom
  {
    Vec x = Vec::Random(n);
    auto d_x = gpu::DeviceMatrix<Scalar>::fromHost(x, ctx.stream());
    gpu::DeviceMatrix<Scalar> d_y;
    d_y.copyFrom(ctx, d_x);
    Vec y = d_y.toHost(ctx.stream());
    VERIFY_IS_APPROX(y, x);
  }

  // setZero
  {
    Vec x = Vec::Random(n);
    auto d_x = gpu::DeviceMatrix<Scalar>::fromHost(x, ctx.stream());
    d_x.setZero(ctx);
    Vec result = d_x.toHost(ctx.stream());
    VERIFY_IS_EQUAL(result, Vec::Zero(n));
  }
}

// ---- BLAS-1 operator overloads (CG-style) -----------------------------------

template <typename Scalar>
void test_cg_operators(Index n) {
  using Vec = Matrix<Scalar, Dynamic, 1>;
  using RealScalar = typename NumTraits<Scalar>::Real;

  RealScalar tol = RealScalar(10) * RealScalar(n) * NumTraits<Scalar>::epsilon();

  Vec x = Vec::Random(n);
  Vec p = Vec::Random(n);
  Vec tmp = Vec::Random(n);
  Vec z = Vec::Random(n);
  Scalar alpha(2.5);
  Scalar beta(0.7);

  // Test: x += alpha * p
  {
    Vec x_ref = x + alpha * p;
    auto d_x = gpu::DeviceMatrix<Scalar>::fromHost(x);
    auto d_p = gpu::DeviceMatrix<Scalar>::fromHost(p);
    d_x += alpha * d_p;
    Vec x_gpu = d_x.toHost();
    VERIFY((x_gpu - x_ref).norm() < tol * x_ref.norm() + tol);
  }

  // Test: r -= alpha * tmp
  {
    Vec r = Vec::Random(n);
    Vec r_ref = r - alpha * tmp;
    auto d_r = gpu::DeviceMatrix<Scalar>::fromHost(r);
    auto d_tmp = gpu::DeviceMatrix<Scalar>::fromHost(tmp);
    d_r -= alpha * d_tmp;
    Vec r_gpu = d_r.toHost();
    VERIFY((r_gpu - r_ref).norm() < tol * r_ref.norm() + tol);
  }

  // Test: p = z + beta * p  (cuBLAS geam)
  {
    Vec p_copy = p;
    Vec p_ref = z + beta * p_copy;
    auto d_p = gpu::DeviceMatrix<Scalar>::fromHost(p_copy);
    auto d_z = gpu::DeviceMatrix<Scalar>::fromHost(z);
    d_p = d_z + beta * d_p;
    Vec p_gpu = d_p.toHost();
    VERIFY((p_gpu - p_ref).norm() < tol * p_ref.norm() + tol);
  }

  // Test: operator+= and operator-= with gpu::DeviceMatrix (no scalar)
  {
    Vec a = Vec::Random(n);
    Vec b = Vec::Random(n);
    Vec a_ref = a + b;
    auto d_a = gpu::DeviceMatrix<Scalar>::fromHost(a);
    auto d_b = gpu::DeviceMatrix<Scalar>::fromHost(b);
    d_a += d_b;
    VERIFY((d_a.toHost() - a_ref).norm() < tol * a_ref.norm() + tol);
  }
}

// ---- gpu::DeviceScalar: deferred sync -------------------------------------------

template <typename Scalar>
void test_device_scalar() {
  using Vec = Matrix<Scalar, Dynamic, 1>;
  using RealScalar = typename NumTraits<Scalar>::Real;

  const Index n = 256;
  Vec a = Vec::Random(n);
  Vec b = Vec::Random(n);

  gpu::Context ctx;
  auto d_a = gpu::DeviceMatrix<Scalar>::fromHost(a, ctx.stream());
  auto d_b = gpu::DeviceMatrix<Scalar>::fromHost(b, ctx.stream());

  // dot() returns gpu::DeviceScalar — implicit conversion to Scalar syncs.
  Scalar gpu_dot = d_a.dot(ctx, d_b);
  Scalar cpu_dot = a.dot(b);
  RealScalar tol = RealScalar(10) * RealScalar(n) * NumTraits<Scalar>::epsilon();
  VERIFY(numext::abs(gpu_dot - cpu_dot) < tol * numext::abs(cpu_dot) + tol);

  // squaredNorm() returns host RealScalar directly (syncs internally).
  RealScalar gpu_sqnorm = d_a.squaredNorm(ctx);
  RealScalar cpu_sqnorm = a.squaredNorm();
  VERIFY(numext::abs(gpu_sqnorm - cpu_sqnorm) < tol * cpu_sqnorm + tol);

  // norm() returns gpu::DeviceScalar<RealScalar> — implicit conversion syncs.
  RealScalar gpu_norm = d_a.norm(ctx);
  RealScalar cpu_norm = a.norm();
  VERIFY(numext::abs(gpu_norm - cpu_norm) < tol * cpu_norm + tol);

  // Convenience overloads (thread-local context).
  gpu::Context::setThreadLocal(&ctx);
  Scalar gpu_dot2 = d_a.dot(d_b);
  VERIFY(numext::abs(gpu_dot2 - cpu_dot) < tol * numext::abs(cpu_dot) + tol);
  gpu::Context::setThreadLocal(nullptr);

  // Empty vectors: dot and norm must return zero.
  {
    gpu::DeviceMatrix<Scalar> d_empty(0, 1);
    gpu::DeviceMatrix<Scalar> d_empty2(0, 1);
    Scalar empty_dot = d_empty.dot(ctx, d_empty2);
    VERIFY_IS_EQUAL(empty_dot, Scalar(0));
    RealScalar empty_sqnorm = d_empty.squaredNorm(ctx);
    VERIFY_IS_EQUAL(empty_sqnorm, RealScalar(0));
    RealScalar empty_norm = d_empty.norm(ctx);
    VERIFY_IS_EQUAL(empty_norm, RealScalar(0));
  }
}

// ---- cwiseProduct -----------------------------------------------------------

template <typename Scalar>
void test_cwiseProduct() {
  using Vec = Matrix<Scalar, Dynamic, 1>;
  using RealScalar = typename NumTraits<Scalar>::Real;

  const Index n = 256;
  Vec a = Vec::Random(n);
  Vec b = Vec::Random(n);
  Vec ref = a.array() * b.array();

  gpu::Context ctx;
  auto d_a = gpu::DeviceMatrix<Scalar>::fromHost(a, ctx.stream());
  auto d_b = gpu::DeviceMatrix<Scalar>::fromHost(b, ctx.stream());
  auto d_c = d_a.cwiseProduct(ctx, d_b);
  Vec result = d_c.toHost(ctx.stream());

  RealScalar tol = RealScalar(10) * RealScalar(n) * NumTraits<Scalar>::epsilon();
  VERIFY((result - ref).norm() < tol * ref.norm() + tol);
}

EIGEN_DECLARE_TEST(gpu_device_matrix) {
  gpu_test::require_cuda_device();
  CALL_SUBTEST(test_default_construct());
  CALL_SUBTEST(test_empty());
  CALL_SUBTEST(test_resize());
  CALL_SUBTEST(test_host_transfer_ready());
  CALL_SUBTEST(test_host_transfer_move());
  CALL_SUBTEST(test_host_transfer_move_assign());
  CALL_SUBTEST((test_allocate<float>(100, 50)));
  CALL_SUBTEST((test_allocate<double>(100, 50)));
  CALL_SUBTEST(test_scalar<float>());
  CALL_SUBTEST(test_scalar<double>());
  CALL_SUBTEST(test_scalar<std::complex<float>>());
  CALL_SUBTEST(test_scalar<std::complex<double>>());
  CALL_SUBTEST(test_blas1<float>(256));
  CALL_SUBTEST(test_blas1<double>(256));
  CALL_SUBTEST(test_blas1<std::complex<float>>(256));
  CALL_SUBTEST(test_blas1<std::complex<double>>(256));
  CALL_SUBTEST(test_cg_operators<float>(256));
  CALL_SUBTEST(test_cg_operators<double>(256));
  CALL_SUBTEST(test_cg_operators<std::complex<float>>(256));
  CALL_SUBTEST(test_cg_operators<std::complex<double>>(256));
  CALL_SUBTEST(test_device_scalar<float>());
  CALL_SUBTEST(test_device_scalar<double>());
  CALL_SUBTEST(test_device_scalar<std::complex<float>>());
  CALL_SUBTEST(test_device_scalar<std::complex<double>>());
  CALL_SUBTEST(test_cwiseProduct<float>());
  CALL_SUBTEST(test_cwiseProduct<double>());
}
