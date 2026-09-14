// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// GPU sparse matrix-vector (SpMV) and sparse matrix-dense matrix (SpMM) multiply
// via cuSPARSE.
//
// SparseContext owns the cuSPARSE descriptors and device buffers. It takes
// SparseMatrix<Scalar, ColMajor> (CSC), implicitly converting RowMajor input, and
// can borrow a gpu::Context so that sparse products share a stream with BLAS-1
// operations — which removes the cross-stream event waits in solvers like CG.
//
// Caching: host-input calls re-upload the values *and* index arrays on every
// call. Host pointer identity cannot detect a sparsity pattern rewritten in
// place or assigned into the same allocations (SparseMatrix reuses them for
// same-shape assignments), so the structure is never assumed unchanged. What
// is cached are the cuSPARSE descriptors (sparse descriptor keyed on
// dimensions + nonzero count, dense descriptors keyed on shape) and the
// SpMV/SpMM workspace-size queries. For repeated products with no re-upload
// at all, use deviceView().
//
// Not thread-safe: concurrent multiply* calls on one instance race on the
// cuSPARSE handle, the bound stream, and the cached buffers. Use one per thread.

#ifndef EIGEN_GPU_SPARSE_CONTEXT_H
#define EIGEN_GPU_SPARSE_CONTEXT_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

#include <cstdint>

#include "./CuSparseSupport.h"
#include "./FwdDecl.h"

namespace Eigen {
namespace gpu {

/** Sparse product expression: DeviceSparseView * DeviceMatrix → SpMVExpr.
 * Evaluated by DeviceMatrix::operator=(SpMVExpr): dispatches to cusparseSpMV
 * when the dense operand has one column, cusparseSpMM otherwise. */
template <typename Scalar_>
class SpMVExpr {
 public:
  using Scalar = Scalar_;
  SpMVExpr(const DeviceSparseView<Scalar>& view, const DeviceMatrix<Scalar>& x) : view_(view), x_(x) {}
  const DeviceSparseView<Scalar>& view() const { return view_; }
  const DeviceMatrix<Scalar>& x() const { return x_; }

 private:
  const DeviceSparseView<Scalar>& view_;
  const DeviceMatrix<Scalar>& x_;
};

/** Device-resident sparse matrix view. Returned by SparseContext::deviceView().
 * Lightweight handle referencing the context's cached device data.
 *
 * \warning One SparseContext caches one sparse matrix at a time. Any later
 * upload through the same context — a second deviceView() or any host-input
 * multiply — replaces the cached data and invalidates earlier views; a stale
 * view is caught by an assert at evaluation time via a generation counter.
 * For multiple simultaneous sparse matrices, use separate SparseContext
 * instances (they can share a Context for same-stream execution).
 *
 * Supports `d_y = d_A * d_x` (SpMV) and `d_Y = d_A * d_X` (SpMM). */
template <typename Scalar_>
class DeviceSparseView {
 public:
  using Scalar = Scalar_;
  using SpMat = SparseMatrix<Scalar, ColMajor, int>;

  DeviceSparseView(SparseContext<Scalar>& ctx, Index rows, Index cols, uint64_t generation)
      : ctx_(ctx), rows_(rows), cols_(cols), generation_(generation) {}

  /** Sparse product expression: d_A * d_x. Evaluated by DeviceMatrix::operator=. */
  SpMVExpr<Scalar> operator*(const DeviceMatrix<Scalar>& x) const { return SpMVExpr<Scalar>(*this, x); }

  Index rows() const { return rows_; }
  Index cols() const { return cols_; }
  const SparseContext<Scalar>& context() const { return ctx_; }

  /** Upload generation this view was created against. Used to detect stale
   * views after the context has cached a different matrix. */
  uint64_t generation() const { return generation_; }

 private:
  SparseContext<Scalar>& ctx_;
  Index rows_;
  Index cols_;
  uint64_t generation_;
};

template <typename Scalar_>
class SparseContext {
 public:
  using Scalar = Scalar_;
  using RealScalar = typename NumTraits<Scalar>::Real;
  using StorageIndex = int;
  using SpMat = SparseMatrix<Scalar, ColMajor, StorageIndex>;
  using DenseVector = Matrix<Scalar, Dynamic, 1>;
  using DenseMatrix = Matrix<Scalar, Dynamic, Dynamic, ColMajor>;

  /** Standalone: creates own stream and cuSPARSE handle. */
  SparseContext() : owns_handle_(true) {
    EIGEN_CUDA_RUNTIME_CHECK(cudaStreamCreate(&stream_));
    owns_stream_ = true;
    EIGEN_CUSPARSE_CHECK(cusparseCreate(&handle_));
    EIGEN_CUSPARSE_CHECK(cusparseSetStream(handle_, stream_));
  }

  /** Borrow a Context: shares stream and cuSPARSE handle.
   * The Context must outlive this SparseContext. */
  explicit SparseContext(Context& ctx)
      : stream_(ctx.stream()), handle_(ctx.cusparseHandle()), owns_stream_(false), owns_handle_(false) {}

  ~SparseContext() {
    destroy_spmat_descriptor(/*checked=*/false);
    destroy_dense_descriptors();
    if (owns_handle_ && handle_) (void)cusparseDestroy(handle_);
    if (owns_stream_ && stream_) (void)cudaStreamDestroy(stream_);
  }

  SparseContext(const SparseContext&) = delete;
  SparseContext& operator=(const SparseContext&) = delete;

  /** Upload a sparse matrix to device and return a lightweight view.
   * The sparse data is uploaded immediately and cached in this context.
   * The returned view can be used for repeated SpMV/SpMM without re-uploading.
   * If the matrix values change, call deviceView() again to re-upload.
   *
   * \warning One context caches one matrix. Any later upload — another
   * deviceView() or any host-input multiply — overwrites the previous upload
   * and invalidates earlier views (asserted at evaluation time). For multiple
   * simultaneous matrices, use separate SparseContext instances sharing the
   * same Context.
   *
   * Supports `d_y = d_A * d_x` (SpMV) and `d_Y = d_A * d_X` (SpMM). */
  DeviceSparseView<Scalar> deviceView(const SpMat& A) {
    eigen_assert(A.isCompressed());
    upload_sparse(A);
    return DeviceSparseView<Scalar>(*this, A.rows(), A.cols(), generation_);
  }

  /** Generation counter of the currently cached sparse matrix. Bumped on
   * every sparse upload (deviceView() or a host-input multiply). */
  uint64_t uploadGeneration() const { return generation_; }

  /** Compute y = A * x. Returns y as a new dense vector. */
  template <typename InputType, typename Rhs>
  DenseVector multiply(const SparseMatrixBase<InputType>& A, const MatrixBase<Rhs>& x) {
    return multiply_host_return(A, x, GpuOp::NoTrans);
  }

  /** Compute y = alpha * op(A) * x + beta * y (in-place, host vectors). */
  template <typename InputType, typename Rhs, typename Dest>
  void multiply(const SparseMatrixBase<InputType>& A, const MatrixBase<Rhs>& x, MatrixBase<Dest>& y,
                Scalar alpha = Scalar(1), Scalar beta = Scalar(0), GpuOp op = GpuOp::NoTrans) {
    const InputType& input = A.derived();
    internal::check_storage_index_bounds<StorageIndex>(input.rows(), input.cols(), input.nonZeros());
    SpMat storage;
    const SpMat& mat = internal::bind_sparse<SpMat>(input, storage);
    multiply_host_impl(mat, x.derived(), y.derived(), alpha, beta, internal::to_cusparse_op<Scalar>(op));
  }

  /** Compute d_y = A * d_x. Device-resident dense vectors, no host transfer
   * for x/y. The sparse matrix (values and index arrays) is re-uploaded on
   * each call; for a fully device-resident sparse matrix use deviceView(). */
  template <typename InputType>
  void multiply(const SparseMatrixBase<InputType>& A, const DeviceMatrix<Scalar>& d_x, DeviceMatrix<Scalar>& d_y) {
    multiply(A, d_x, d_y, Scalar(1), Scalar(0), GpuOp::NoTrans);
  }

  /** Compute d_y = alpha * op(A) * d_x + beta * d_y (DeviceMatrix, in-place). */
  template <typename InputType>
  void multiply(const SparseMatrixBase<InputType>& A, const DeviceMatrix<Scalar>& d_x, DeviceMatrix<Scalar>& d_y,
                Scalar alpha, Scalar beta, GpuOp op = GpuOp::NoTrans) {
    const InputType& input = A.derived();
    internal::check_storage_index_bounds<StorageIndex>(input.rows(), input.cols(), input.nonZeros());
    SpMat storage;
    const SpMat& mat = internal::bind_sparse<SpMat>(input, storage);
    upload_sparse(mat);
    spmv_device_exec(d_x, d_y, alpha, beta, op);
  }

  /** Compute y = A^T * x (host vectors). */
  template <typename InputType, typename Rhs>
  DenseVector multiplyT(const SparseMatrixBase<InputType>& A, const MatrixBase<Rhs>& x) {
    return multiply_host_return(A, x, GpuOp::Trans);
  }

  /** Compute y = A^H * x (conjugate transpose). For real Scalar this is equivalent to multiplyT. */
  template <typename InputType, typename Rhs>
  DenseVector multiplyAdjoint(const SparseMatrixBase<InputType>& A, const MatrixBase<Rhs>& x) {
    return multiply_host_return(A, x, GpuOp::ConjTrans);
  }

  /** Compute Y = op(A) * X where X is a dense matrix (multiple RHS). Returns Y. */
  template <typename InputType, typename Rhs>
  DenseMatrix multiplyMat(const SparseMatrixBase<InputType>& A, const MatrixBase<Rhs>& X, GpuOp op = GpuOp::NoTrans) {
    const InputType& input = A.derived();
    internal::check_storage_index_bounds<StorageIndex>(input.rows(), input.cols(), input.nonZeros());
    SpMat storage;
    const SpMat& mat = internal::bind_sparse<SpMat>(input, storage);
    const DenseMatrix rhs(X.derived());

    const cusparseOperation_t cu_op = internal::to_cusparse_op<Scalar>(op);
    const Index m = (op == GpuOp::NoTrans) ? mat.rows() : mat.cols();
    const Index k = (op == GpuOp::NoTrans) ? mat.cols() : mat.rows();
    eigen_assert(k == rhs.rows());

    const Index n = rhs.cols();
    if (m == 0 || n == 0 || mat.nonZeros() == 0) return DenseMatrix::Zero(m, n);

    DenseMatrix Y = DenseMatrix::Zero(m, n);
    spmm_impl(mat, rhs, Y, Scalar(1), Scalar(0), cu_op);
    return Y;
  }

  cudaStream_t stream() const { return stream_; }

 private:
  cudaStream_t stream_ = nullptr;
  cusparseHandle_t handle_ = nullptr;
  bool owns_stream_ = false;
  bool owns_handle_ = false;

  // Cached device buffers for sparse matrix (grow-only).
  internal::DeviceBuffer d_outerPtr_;
  internal::DeviceBuffer d_innerIdx_;
  internal::DeviceBuffer d_values_;

  // Cached device buffers for host-API dense vectors (grow-only).
  internal::DeviceBuffer d_x_;
  internal::DeviceBuffer d_y_;

  mutable internal::DeviceBuffer d_workspace_;

  // Cached cuSPARSE sparse matrix descriptor.
  cusparseSpMatDescr_t spmat_desc_ = nullptr;
  Index cached_rows_ = -1;
  Index cached_cols_ = -1;
  Index cached_nnz_ = -1;

  // Bumped on every sparse upload; DeviceSparseViews record it at creation so
  // a stale view (its data replaced by a later upload) asserts at evaluation.
  uint64_t generation_ = 0;

  // Cached dense-vector/matrix descriptors, re-pointed per call and recreated
  // only when the shape changes.
  mutable cusparseDnVecDescr_t x_vec_desc_ = nullptr;
  mutable cusparseDnVecDescr_t y_vec_desc_ = nullptr;
  mutable int64_t x_vec_size_ = -1;
  mutable int64_t y_vec_size_ = -1;
  mutable cusparseDnMatDescr_t x_mat_desc_ = nullptr;
  mutable cusparseDnMatDescr_t y_mat_desc_ = nullptr;
  mutable int64_t x_mat_rows_ = -1, x_mat_cols_ = -1;
  mutable int64_t y_mat_rows_ = -1, y_mat_cols_ = -1;

  // Cached workspace-size query results, indexed by cusparseOperation_t.
  // Invalidated when the sparse descriptor or a dense-descriptor shape changes.
  static constexpr size_t kWsUnknown = static_cast<size_t>(-1);
  mutable size_t spmv_ws_size_[3] = {kWsUnknown, kWsUnknown, kWsUnknown};
  mutable size_t spmm_ws_size_[3] = {kWsUnknown, kWsUnknown, kWsUnknown};

  // Shared host-input SpMV entry: y = op(A) * x into a fresh vector.
  template <typename InputType, typename Rhs>
  DenseVector multiply_host_return(const SparseMatrixBase<InputType>& A, const MatrixBase<Rhs>& x, GpuOp op) {
    const InputType& input = A.derived();
    internal::check_storage_index_bounds<StorageIndex>(input.rows(), input.cols(), input.nonZeros());
    SpMat storage;
    const SpMat& mat = internal::bind_sparse<SpMat>(input, storage);
    DenseVector y((op == GpuOp::NoTrans) ? mat.rows() : mat.cols());
    y.setZero();
    multiply_host_impl(mat, x.derived(), y, Scalar(1), Scalar(0), internal::to_cusparse_op<Scalar>(op));
    return y;
  }

  template <typename RhsDerived, typename DestDerived>
  void multiply_host_impl(const SpMat& A, const RhsDerived& x, DestDerived& y, Scalar alpha, Scalar beta,
                          cusparseOperation_t op) {
    eigen_assert(A.isCompressed());

    const Index m = A.rows();
    const Index n = A.cols();
    const Index nnz = A.nonZeros();
    const Index x_size = (op == CUSPARSE_OPERATION_NON_TRANSPOSE) ? n : m;
    const Index y_size = (op == CUSPARSE_OPERATION_NON_TRANSPOSE) ? m : n;

    eigen_assert(x.size() == x_size);
    eigen_assert(y.size() == y_size);

    if (m == 0 || n == 0 || nnz == 0) {
      if (beta == Scalar(0))
        y.setZero();
      else
        y *= beta;
      return;
    }

    upload_sparse(A);

    ensure_buffer(d_x_, static_cast<size_t>(x_size) * sizeof(Scalar));
    // Ref binds in place when x is already a contiguous vector; only genuine
    // expressions are evaluated into the Ref's internal temporary.
    const Ref<const DenseVector> x_ref(x);
    EIGEN_CUDA_RUNTIME_CHECK(
        cudaMemcpyAsync(d_x_.get(), x_ref.data(), x_size * sizeof(Scalar), cudaMemcpyHostToDevice, stream_));

    ensure_buffer(d_y_, static_cast<size_t>(y_size) * sizeof(Scalar));
    if (beta != Scalar(0)) {
      const Ref<const DenseVector> y_ref(y);
      EIGEN_CUDA_RUNTIME_CHECK(
          cudaMemcpyAsync(d_y_.get(), y_ref.data(), y_size * sizeof(Scalar), cudaMemcpyHostToDevice, stream_));
    }

    exec_spmv(x_size, y_size, d_x_.get(), d_y_.get(), alpha, beta, op);

    EIGEN_CUDA_RUNTIME_CHECK(
        cudaMemcpyAsync(y.data(), d_y_.get(), y_size * sizeof(Scalar), cudaMemcpyDeviceToHost, stream_));
    EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(stream_));
  }

 public:
  /** Execute SpMV using the already-uploaded sparse matrix (no re-upload).
   * Used by SpMVExpr (d_y = d_A * d_x) for cached deviceView() paths.
   * The sparse matrix must have been uploaded via deviceView() or multiply(). */
  void spmv_device_exec(const DeviceMatrix<Scalar>& d_x, DeviceMatrix<Scalar>& d_y, Scalar alpha = Scalar(1),
                        Scalar beta = Scalar(0), GpuOp op = GpuOp::NoTrans) const {
    eigen_assert(spmat_desc_ && "sparse matrix not uploaded — call deviceView() or multiply() first");
    // cuSPARSE SpMV: y must not alias x (undefined behavior).
    eigen_assert(d_x.data() != d_y.data() && "SpMV: output aliases input vector");

    const cusparseOperation_t cu_op = internal::to_cusparse_op<Scalar>(op);
    const Index m = cached_rows_;
    const Index n = cached_cols_;
    const Index x_size = (cu_op == CUSPARSE_OPERATION_NON_TRANSPOSE) ? n : m;
    const Index y_size = (cu_op == CUSPARSE_OPERATION_NON_TRANSPOSE) ? m : n;

    eigen_assert(d_x.rows() * d_x.cols() == x_size);

    if (m == 0 || n == 0 || cached_nnz_ == 0) {
      // Empty A reduces SpMV to y <- beta*y; SparseContext owns no cuBLAS
      // handle for the scale, so the beta != 0 case must be handled by the caller.
      eigen_assert(beta == Scalar(0) && "SpMV with empty A and beta != 0 is unsupported; scale d_y externally");
      if (d_y.rows() * d_y.cols() != y_size) d_y.resize(y_size, 1);
      d_y.setZero(stream_);
      return;
    }

    // Ensure d_y is allocated.
    if (d_y.rows() * d_y.cols() != y_size) {
      d_y.resize(y_size, 1);
    }

    // Wait for input data to be ready on this stream.
    d_x.waitReady(stream_);
    d_y.waitReady(stream_);

    exec_spmv(x_size, y_size, const_cast<void*>(static_cast<const void*>(d_x.data())), static_cast<void*>(d_y.data()),
              alpha, beta, cu_op);

    d_y.recordReady(stream_);
  }

  /** Execute SpMM (d_Y = alpha * op(A) * d_X + beta * d_Y) using the
   * already-uploaded sparse matrix. d_X / d_Y are device-resident dense
   * column-major matrices. Used by SpMVExpr when the RHS has > 1 column. */
  void spmm_device_exec(const DeviceMatrix<Scalar>& d_X, DeviceMatrix<Scalar>& d_Y, Scalar alpha = Scalar(1),
                        Scalar beta = Scalar(0), GpuOp op = GpuOp::NoTrans) const {
    eigen_assert(spmat_desc_ && "sparse matrix not uploaded — call deviceView() or multiply() first");
    eigen_assert(d_X.data() != d_Y.data() && "SpMM: output aliases input matrix");

    const cusparseOperation_t cu_op = internal::to_cusparse_op<Scalar>(op);
    const bool transposed = (cu_op != CUSPARSE_OPERATION_NON_TRANSPOSE);
    const Index m_op = transposed ? cached_cols_ : cached_rows_;
    const Index k_op = transposed ? cached_rows_ : cached_cols_;
    const Index n = d_X.cols();

    eigen_assert(d_X.rows() == k_op);

    if (m_op == 0 || n == 0 || cached_nnz_ == 0) {
      eigen_assert(beta == Scalar(0) && "SpMM with empty A and beta != 0 is unsupported; scale d_Y externally");
      if (d_Y.rows() != m_op || d_Y.cols() != n) d_Y.resize(m_op, n);
      d_Y.setZero(stream_);
      return;
    }

    if (d_Y.rows() != m_op || d_Y.cols() != n) {
      d_Y.resize(m_op, n);
    }

    d_X.waitReady(stream_);
    d_Y.waitReady(stream_);

    exec_spmm(m_op, k_op, n, const_cast<void*>(static_cast<const void*>(d_X.data())), static_cast<void*>(d_Y.data()),
              alpha, beta, cu_op);

    d_Y.recordReady(stream_);
  }

 private:
  // cuSPARSE 11.x's cusparseSpMM rejects CSC for matA (CSC support landed in
  // CUDA 12.0). On 11.x we register the same buffers as CSR-of-A^T (dims
  // swapped) and invert the user-facing op before each cuSPARSE call. On 12+
  // we keep the natural CSC path so users pay no extra cost.
#if !defined(CUSPARSE_VERSION) || CUSPARSE_VERSION < 12000
  static constexpr bool kUseCsrOfTranspose = true;
  static constexpr cusparseSpMMAlg_t kSpMMAlg = CUSPARSE_SPMM_CSR_ALG2;
#else
  static constexpr bool kUseCsrOfTranspose = false;
  static constexpr cusparseSpMMAlg_t kSpMMAlg = CUSPARSE_SPMM_ALG_DEFAULT;
#endif

  // Map a user-facing op on A to the cuSPARSE op on the cached descriptor.
  // Identity on cuSPARSE 12+ (descriptor is CSC of A); inverted on 11.x
  // (descriptor is CSR of A^T).
  static cusparseOperation_t descriptor_op(cusparseOperation_t user_op) {
    EIGEN_IF_CONSTEXPR (!kUseCsrOfTranspose) return user_op;
    switch (user_op) {
      case CUSPARSE_OPERATION_NON_TRANSPOSE:
        return CUSPARSE_OPERATION_TRANSPOSE;
      case CUSPARSE_OPERATION_TRANSPOSE:
        return CUSPARSE_OPERATION_NON_TRANSPOSE;
      default:
        // CONJUGATE_TRANSPOSE on the CSR-of-A^T descriptor would compute
        // conj(A) * x, not A^H * x — not supported via this representation.
        eigen_assert(false && "CUSPARSE_OPERATION_CONJUGATE_TRANSPOSE not supported on cuSPARSE < 12.0");
        return user_op;
    }
  }

  static int op_index(cusparseOperation_t op) {
    switch (op) {
      case CUSPARSE_OPERATION_TRANSPOSE:
        return 1;
      case CUSPARSE_OPERATION_CONJUGATE_TRANSPOSE:
        return 2;
      default:
        return 0;
    }
  }

  void invalidate_ws_caches() const {
    for (int i = 0; i < 3; ++i) {
      spmv_ws_size_[i] = kWsUnknown;
      spmm_ws_size_[i] = kWsUnknown;
    }
  }

  // Recreate the descriptor when the size changes; otherwise just re-point it
  // at the new device buffer (cusparseDnVecSetValues is a host-side pointer
  // update, no GPU work).
  void update_dnvec(cusparseDnVecDescr_t& desc, int64_t& cur_size, int64_t size, void* ptr) const {
    if (!desc || cur_size != size) {
      if (desc) EIGEN_CUSPARSE_CHECK(cusparseDestroyDnVec(desc));
      EIGEN_CUSPARSE_CHECK(cusparseCreateDnVec(&desc, size, ptr, internal::cuda_data_type<Scalar>::value));
      cur_size = size;
      invalidate_ws_caches();
    } else {
      EIGEN_CUSPARSE_CHECK(cusparseDnVecSetValues(desc, ptr));
    }
  }

  void update_dnmat(cusparseDnMatDescr_t& desc, int64_t& cur_rows, int64_t& cur_cols, int64_t rows, int64_t cols,
                    void* ptr) const {
    if (!desc || cur_rows != rows || cur_cols != cols) {
      if (desc) EIGEN_CUSPARSE_CHECK(cusparseDestroyDnMat(desc));
      // Column-major with ld = rows.
      EIGEN_CUSPARSE_CHECK(cusparseCreateDnMat(&desc, rows, cols, rows, ptr, internal::cuda_data_type<Scalar>::value,
                                               CUSPARSE_ORDER_COL));
      cur_rows = rows;
      cur_cols = cols;
      invalidate_ws_caches();
    } else {
      EIGEN_CUSPARSE_CHECK(cusparseDnMatSetValues(desc, ptr));
    }
  }

  void destroy_dense_descriptors() {
    if (x_vec_desc_) (void)cusparseDestroyDnVec(x_vec_desc_);
    if (y_vec_desc_) (void)cusparseDestroyDnVec(y_vec_desc_);
    if (x_mat_desc_) (void)cusparseDestroyDnMat(x_mat_desc_);
    if (y_mat_desc_) (void)cusparseDestroyDnMat(y_mat_desc_);
    x_vec_desc_ = y_vec_desc_ = nullptr;
    x_mat_desc_ = y_mat_desc_ = nullptr;
  }

  void exec_spmv(Index x_size, Index y_size, void* d_x_ptr, void* d_y_ptr, Scalar alpha, Scalar beta,
                 cusparseOperation_t op) const {
    constexpr cudaDataType_t dtype = internal::cuda_data_type<Scalar>::value;
    const cusparseOperation_t cu_op = descriptor_op(op);
    update_dnvec(x_vec_desc_, x_vec_size_, x_size, d_x_ptr);
    update_dnvec(y_vec_desc_, y_vec_size_, y_size, d_y_ptr);

    size_t& ws_size = spmv_ws_size_[op_index(cu_op)];
    if (ws_size == kWsUnknown) {
      EIGEN_CUSPARSE_CHECK(cusparseSpMV_bufferSize(handle_, cu_op, &alpha, spmat_desc_, x_vec_desc_, &beta, y_vec_desc_,
                                                   dtype, CUSPARSE_SPMV_ALG_DEFAULT, &ws_size));
    }
    ensure_buffer(d_workspace_, ws_size);

    EIGEN_CUSPARSE_CHECK(cusparseSpMV(handle_, cu_op, &alpha, spmat_desc_, x_vec_desc_, &beta, y_vec_desc_, dtype,
                                      CUSPARSE_SPMV_ALG_DEFAULT, d_workspace_.get()));
  }

  void exec_spmm(Index m_op, Index k_op, Index n, void* d_x_ptr, void* d_y_ptr, Scalar alpha, Scalar beta,
                 cusparseOperation_t op) const {
    constexpr cudaDataType_t dtype = internal::cuda_data_type<Scalar>::value;
    const cusparseOperation_t cu_op = descriptor_op(op);
    // X is k_op x n, Y is m_op x n (column-major, post-op shapes).
    update_dnmat(x_mat_desc_, x_mat_rows_, x_mat_cols_, k_op, n, d_x_ptr);
    update_dnmat(y_mat_desc_, y_mat_rows_, y_mat_cols_, m_op, n, d_y_ptr);

    size_t& ws_size = spmm_ws_size_[op_index(cu_op)];
    if (ws_size == kWsUnknown) {
      EIGEN_CUSPARSE_CHECK(cusparseSpMM_bufferSize(handle_, cu_op, CUSPARSE_OPERATION_NON_TRANSPOSE, &alpha,
                                                   spmat_desc_, x_mat_desc_, &beta, y_mat_desc_, dtype, kSpMMAlg,
                                                   &ws_size));
    }
    ensure_buffer(d_workspace_, ws_size);

    EIGEN_CUSPARSE_CHECK(cusparseSpMM(handle_, cu_op, CUSPARSE_OPERATION_NON_TRANSPOSE, &alpha, spmat_desc_,
                                      x_mat_desc_, &beta, y_mat_desc_, dtype, kSpMMAlg, d_workspace_.get()));
  }

  void spmm_impl(const SpMat& A, const DenseMatrix& X, DenseMatrix& Y, Scalar alpha, Scalar beta,
                 cusparseOperation_t op) {
    eigen_assert(A.isCompressed());

    // For op != NON_TRANSPOSE, Y = op(A) * X. The dense X / Y descriptors must
    // describe the *post-op* shapes: X has k_op rows (= input dim of op(A)),
    // Y has m_op rows (= output dim of op(A)).
    const bool transposed = (op != CUSPARSE_OPERATION_NON_TRANSPOSE);
    const Index m_op = transposed ? A.cols() : A.rows();
    const Index k_op = transposed ? A.rows() : A.cols();
    const Index n = X.cols();
    const Index nnz = A.nonZeros();

    if (m_op == 0 || n == 0 || k_op == 0 || nnz == 0) {
      if (beta == Scalar(0))
        Y.setZero();
      else
        Y *= beta;
      return;
    }

    upload_sparse(A);

    // Upload X to device. X is k_op x n, Y is m_op x n (column-major).
    const size_t x_bytes = static_cast<size_t>(k_op) * static_cast<size_t>(n) * sizeof(Scalar);
    const size_t y_bytes = static_cast<size_t>(m_op) * static_cast<size_t>(n) * sizeof(Scalar);
    ensure_buffer(d_x_, x_bytes);
    ensure_buffer(d_y_, y_bytes);
    EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpyAsync(d_x_.get(), X.data(), x_bytes, cudaMemcpyHostToDevice, stream_));
    if (beta != Scalar(0)) {
      EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpyAsync(d_y_.get(), Y.data(), y_bytes, cudaMemcpyHostToDevice, stream_));
    }

    exec_spmm(m_op, k_op, n, d_x_.get(), d_y_.get(), alpha, beta, op);

    EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpyAsync(Y.data(), d_y_.get(), y_bytes, cudaMemcpyDeviceToHost, stream_));
    EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(stream_));
  }

  void upload_sparse(const SpMat& A) {
    // cuSPARSE 12.0+ accepts CSC directly. On cuSPARSE 11.x, cusparseSpMM
    // rejects CSC and CONJUGATE_TRANSPOSE on CSC+complex SpMV silently
    // demotes to TRANSPOSE. We register the same CSC buffers as CSR-of-A^T
    // (dims swapped) on 11.x and invert the op at exec time via
    // descriptor_op() — no transpose-copy required.
    upload_compressed_arrays(A.rows(), A.cols(), A.nonZeros(),
                             /*outer_count=*/A.cols() + 1, A.outerIndexPtr(), A.innerIndexPtr(), A.valuePtr());
  }

  void upload_compressed_arrays(Index m, Index n, Index nnz, Index outer_count, const StorageIndex* host_outer,
                                const StorageIndex* host_inner, const Scalar* host_values) {
    const size_t outer_bytes = static_cast<size_t>(outer_count) * sizeof(StorageIndex);
    const size_t inner_bytes = static_cast<size_t>(nnz) * sizeof(StorageIndex);
    const size_t val_bytes = static_cast<size_t>(nnz) * sizeof(Scalar);

    // Values *and* index arrays are re-uploaded unconditionally: host pointer
    // identity cannot detect a same-shape/same-nnz pattern rewritten in place
    // or assigned into the same allocations (SparseMatrix reuses them), so a
    // structure cache keyed on pointers would silently serve stale indices.
    // Only the cuSPARSE descriptor and the workspace-size queries are cached,
    // keyed on (rows, cols, nnz). Every upload invalidates outstanding
    // DeviceSparseViews via the generation counter.
    ++generation_;
    ensure_buffer(d_values_, val_bytes);
    ensure_buffer(d_outerPtr_, outer_bytes);
    ensure_buffer(d_innerIdx_, inner_bytes);
    EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpyAsync(d_values_.get(), host_values, val_bytes, cudaMemcpyHostToDevice, stream_));
    EIGEN_CUDA_RUNTIME_CHECK(
        cudaMemcpyAsync(d_outerPtr_.get(), host_outer, outer_bytes, cudaMemcpyHostToDevice, stream_));
    EIGEN_CUDA_RUNTIME_CHECK(
        cudaMemcpyAsync(d_innerIdx_.get(), host_inner, inner_bytes, cudaMemcpyHostToDevice, stream_));

    // Same shape and nnz: the grow-only device buffers cannot have been
    // reallocated, so the existing descriptor still points at the freshly
    // written data.
    if (m == cached_rows_ && n == cached_cols_ && nnz == cached_nnz_) return;

    destroy_spmat_descriptor(/*checked=*/true);

    constexpr cusparseIndexType_t idx_type = (sizeof(StorageIndex) == 4) ? CUSPARSE_INDEX_32I : CUSPARSE_INDEX_64I;
    constexpr cudaDataType_t val_type = internal::cuda_data_type<Scalar>::value;

    EIGEN_IF_CONSTEXPR (kUseCsrOfTranspose) {
      // cuSPARSE 11.x: cusparseSpMM rejects CSC for matA. CSC of A and CSR of
      // A^T share the same buffers, so register the data as CSR-of-A^T (dims
      // swapped) and invert the op in exec_spmv / spmm_impl via descriptor_op.
      EIGEN_CUSPARSE_CHECK(cusparseCreateCsr(&spmat_desc_, n, m, nnz, d_outerPtr_.get(), d_innerIdx_.get(),
                                             d_values_.get(), idx_type, idx_type, CUSPARSE_INDEX_BASE_ZERO, val_type));
    } else {
      EIGEN_CUSPARSE_CHECK(cusparseCreateCsc(&spmat_desc_, m, n, nnz, d_outerPtr_.get(), d_innerIdx_.get(),
                                             d_values_.get(), idx_type, idx_type, CUSPARSE_INDEX_BASE_ZERO, val_type));
    }
    cached_rows_ = m;
    cached_cols_ = n;
    cached_nnz_ = nnz;
  }

  // Destroy the sparse-matrix descriptor and reset the cache identity.
  // `checked` selects assert-on-failure (mid-lifetime rebuilds) vs swallow
  // (noexcept destructor).
  void destroy_spmat_descriptor(bool checked) {
    if (spmat_desc_) {
      cusparseStatus_t s = cusparseDestroySpMat(spmat_desc_);
      eigen_assert((!checked || s == CUSPARSE_STATUS_SUCCESS) && "cusparseDestroySpMat failed");
      EIGEN_UNUSED_VARIABLE(s);
      EIGEN_UNUSED_VARIABLE(checked);
      spmat_desc_ = nullptr;
    }
    cached_rows_ = -1;
    cached_cols_ = -1;
    cached_nnz_ = -1;
    invalidate_ws_caches();
  }

  void ensure_buffer(internal::DeviceBuffer& buf, size_t needed) const {
    if (needed > buf.size()) {
      if (buf) EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(stream_));
      buf = internal::DeviceBuffer(needed);
    }
  }
};

// Defined here because it needs the full SparseContext definition.

template <typename Scalar_>
DeviceMatrix<Scalar_>& DeviceMatrix<Scalar_>::operator=(const SpMVExpr<Scalar_>& expr) {
  // Uses the sparse matrix already uploaded by deviceView() — no re-upload on
  // repeated products with the same view. A stale view (the context has since
  // uploaded again, replacing the cached data) is caught here.
  eigen_assert(expr.view().generation() == expr.view().context().uploadGeneration() &&
               "DeviceSparseView is stale: its SparseContext has since uploaded another sparse matrix");
  if (expr.x().cols() <= 1) {
    expr.view().context().spmv_device_exec(expr.x(), *this, Scalar_(1), Scalar_(0), GpuOp::NoTrans);
  } else {
    expr.view().context().spmm_device_exec(expr.x(), *this, Scalar_(1), Scalar_(0), GpuOp::NoTrans);
  }
  return *this;
}

template <typename Scalar_>
DeviceMatrix<Scalar_>::DeviceMatrix(const SpMVExpr<Scalar_>& expr) : DeviceMatrix() {
  *this = expr;
}
}  // namespace gpu
}  // namespace Eigen

#endif  // EIGEN_GPU_SPARSE_CONTEXT_H
