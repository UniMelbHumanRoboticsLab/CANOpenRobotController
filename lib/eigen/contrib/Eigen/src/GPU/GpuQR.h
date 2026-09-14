// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// GPU QR decomposition using cuSOLVER, wrapping cusolverDnXgeqrf, cusolverDnXormqr
// (apply Q), and cublasXtrsm (triangular solve on R). Q is never formed
// explicitly.
//
// Both shapes are handled transparently: for m >= n the factorization is
// A = Q R and solve() is least-squares; for m < n it is A^H = Q R internally and
// solve() is minimum-norm.

#ifndef EIGEN_GPU_QR_H
#define EIGEN_GPU_QR_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

#include "./GpuSolverContext.h"

namespace Eigen {
namespace gpu {
template <typename Scalar_>
class QR {
 public:
  using Scalar = Scalar_;
  using RealScalar = typename NumTraits<Scalar>::Real;
  using PlainMatrix = Eigen::Matrix<Scalar, Dynamic, Dynamic, ColMajor>;

  QR() = default;

  /** Bind to \p ctx: run on its stream with its cuSOLVER/cuBLAS handles, so
   * solver work chains with other work on the same Context without
   * cross-stream event waits. \p ctx must outlive this object. */
  explicit QR(Context& ctx) : solver_ctx_(ctx) {}

  template <typename InputType>
  explicit QR(const DenseBase<InputType>& A) {
    compute(A);
  }

  explicit QR(const DeviceMatrix<Scalar>& d_A) { compute(d_A); }

  /** Factor a device-resident A immediately (adopt when m >= n, no copy). */
  explicit QR(DeviceMatrix<Scalar>&& d_A) { compute(std::move(d_A)); }

  /** Bind to \p ctx and factor A immediately. */
  template <typename InputType>
  QR(Context& ctx, const DenseBase<InputType>& A) : solver_ctx_(ctx) {
    compute(A);
  }

  /** Bind to \p ctx and factor a device-resident A (D2D copy). */
  QR(Context& ctx, const DeviceMatrix<Scalar>& d_A) : solver_ctx_(ctx) { compute(d_A); }

  ~QR() = default;

  QR(const QR&) = delete;
  QR& operator=(const QR&) = delete;

  QR(QR&& o) noexcept
      : solver_ctx_(std::move(o.solver_ctx_)),
        d_qr_(std::move(o.d_qr_)),
        d_tau_(std::move(o.d_tau_)),
        m_(o.m_),
        n_(o.n_),
        lda_(o.lda_),
        transposed_(o.transposed_) {
    o.m_ = 0;
    o.n_ = 0;
    o.lda_ = 0;
    o.transposed_ = false;
  }

  QR& operator=(QR&& o) noexcept {
    if (this != &o) {
      solver_ctx_ = std::move(o.solver_ctx_);
      d_qr_ = std::move(o.d_qr_);
      d_tau_ = std::move(o.d_tau_);
      m_ = o.m_;
      n_ = o.n_;
      lda_ = o.lda_;
      transposed_ = o.transposed_;
      o.m_ = 0;
      o.n_ = 0;
      o.lda_ = 0;
      o.transposed_ = false;
    }
    return *this;
  }

  template <typename InputType>
  QR& compute(const DenseBase<InputType>& A) {
    // Upload to device, then delegate to the adopting overload — the freshly
    // uploaded matrix is factored in place (geqrf overwrites its input), so no
    // second device copy is made. The wide-matrix transpose runs on the GPU
    // (via cublasXgeam) inside the device-input path; no host transpose.
    return compute(DeviceMatrix<Scalar>::fromHost(A.derived(), solver_ctx_.stream()));
  }

  QR& compute(const DeviceMatrix<Scalar>& d_A) {
    if (!begin_compute(d_A)) return *this;

    if (transposed_) {
      transpose_into_factor(d_A);
    } else {
      const size_t mat_bytes = factorBytes();
      allocate_factor_storage(mat_bytes);
      EIGEN_CUDA_RUNTIME_CHECK(
          cudaMemcpyAsync(d_qr_.get(), d_A.data(), mat_bytes, cudaMemcpyDeviceToDevice, solver_ctx_.stream()));
    }

    factorize();
    return *this;
  }

  /** Factor a device matrix (move). For m >= n the buffer is adopted and
   * factored in place — no copy; for m < n a transposed copy is unavoidable. */
  QR& compute(DeviceMatrix<Scalar>&& d_A) {
    if (!begin_compute(d_A)) return *this;

    if (transposed_) {
      transpose_into_factor(d_A);
    } else {
      d_qr_ = internal::DeviceBuffer::adopt(static_cast<void*>(d_A.release()), factorBytes());
    }

    factorize();
    return *this;
  }

  /** Solve A * X = B.
   * For m >= n (over-/exactly-determined): least-squares X = R^{-1} Q^H B (residual A^H r ≈ 0).
   * For m  < n (underdetermined):          minimum-norm  X = Q R^{-H} B (||X|| minimized). */
  template <typename Rhs>
  PlainMatrix solve(const MatrixBase<Rhs>& B) const {
    // Debug builds verify the factorization (info() synchronizes the stream on
    // the first call after compute()); release builds skip both the check and
    // the sync — use info() explicitly when failure must be detected.
    eigen_assert(solver_ctx_.info() == Success && "QR::solve called on a failed or uninitialized factorization");
    eigen_assert(B.rows() == m_);

    const Ref<const PlainMatrix> rhs(B.derived());
    const Index nrhs = rhs.cols();

    if (!transposed_) {
      return solve_overdetermined_host(rhs);
    }
    return solve_underdetermined_host(rhs, nrhs);
  }

  /** Solve with device-resident RHS. Returns n × nrhs DeviceMatrix. Fully
   * asynchronous; debug builds verify the factorization status first. */
  DeviceMatrix<Scalar> solve(const DeviceMatrix<Scalar>& d_B) const {
    eigen_assert(solver_ctx_.info() == Success && "QR::solve called on a failed or uninitialized factorization");
    eigen_assert(d_B.rows() == m_);
    d_B.waitReady(solver_ctx_.stream());

    if (!transposed_) {
      return solve_overdetermined_device(d_B);
    }
    return solve_underdetermined_device(d_B);
  }

  ComputationInfo info() const { return solver_ctx_.info(); }

  Index rows() const { return m_; }
  Index cols() const { return n_; }
  cudaStream_t stream() const { return solver_ctx_.stream(); }

  /** Upper-triangular factor R (k × n) of A = Q R. Available only for m >= n. */
  PlainMatrix matrixR() const {
    eigen_assert(solver_ctx_.info() == Success);
    eigen_assert(!transposed_ && "matrixR() not available when m < n (we factored A^H internally)");
    PlainMatrix qr_full(m_, n_);
    if (m_ > 0 && n_ > 0) {
      EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpy(qr_full.data(), d_qr_.get(),
                                          static_cast<size_t>(lda_) * static_cast<size_t>(n_) * sizeof(Scalar),
                                          cudaMemcpyDeviceToHost));
    }
    PlainMatrix R = qr_full.topRows(k()).template triangularView<Upper>();
    return R;
  }

 private:
  mutable internal::GpuSolverContext solver_ctx_;
  internal::DeviceBuffer d_qr_;   // grow-only; QR factors (reflectors below diag, R above)
  internal::DeviceBuffer d_tau_;  // grow-only; Householder scalars (length k)
  int64_t m_ = 0;                 // original A.rows()
  int64_t n_ = 0;                 // original A.cols()
  int64_t lda_ = 0;               // factor leading dim = max(m_, n_)
  bool transposed_ = false;       // true iff m_ < n_, i.e. A^H was factored

  // The factored matrix is always tall: rows >= cols.
  int64_t factor_rows() const { return transposed_ ? n_ : m_; }
  int64_t factor_cols() const { return transposed_ ? m_ : n_; }
  int64_t k() const { return (std::min)(m_, n_); }

  size_t factorBytes() const { return static_cast<size_t>(lda_) * static_cast<size_t>(factor_cols()) * sizeof(Scalar); }

  // Common compute() prologue: record shape, reset info, wait on the input.
  // Returns false (and clears stale factors) for empty input.
  bool begin_compute(const DeviceMatrix<Scalar>& d_A) {
    m_ = d_A.rows();
    n_ = d_A.cols();
    if (!solver_ctx_.begin_compute(m_ != 0 && n_ != 0)) {
      d_qr_ = internal::DeviceBuffer();
      d_tau_ = internal::DeviceBuffer();
      return false;
    }
    transposed_ = (m_ < n_);
    lda_ = static_cast<int64_t>(transposed_ ? n_ : m_);
    d_A.waitReady(solver_ctx_.stream());
    return true;
  }

  void allocate_factor_storage(size_t mat_bytes) { internal::ensure_sized(d_qr_, mat_bytes); }

  // Wide input (m < n): factor A^H, produced on device via cuBLAS geam.
  void transpose_into_factor(const DeviceMatrix<Scalar>& d_A) {
    allocate_factor_storage(factorBytes());
    Scalar alpha_one(1), beta_zero(0);
    EIGEN_CUBLAS_CHECK(internal::cublasXgeam(solver_ctx_.cublasHandle(), CUBLAS_OP_C, CUBLAS_OP_N, n_, m_, &alpha_one,
                                             d_A.data(), d_A.rows(), &beta_zero, static_cast<const Scalar*>(nullptr),
                                             n_, static_cast<Scalar*>(d_qr_.get()), n_));
  }

  void factorize() {
    constexpr cudaDataType_t dtype = internal::cusolver_data_type<Scalar>::value;

    solver_ctx_.mark_pending();

    internal::ensure_sized(d_tau_, static_cast<size_t>(k()) * sizeof(Scalar));

    const int64_t fm = factor_rows();
    const int64_t fn = factor_cols();
    size_t dev_ws = 0, host_ws = 0;
    EIGEN_CUSOLVER_CHECK(cusolverDnXgeqrf_bufferSize(solver_ctx_.cusolverHandle(), solver_ctx_.params_.p, fm, fn, dtype,
                                                     d_qr_.get(), lda_, dtype, d_tau_.get(), dtype, &dev_ws, &host_ws));

    solver_ctx_.ensure_scratch(dev_ws);
    solver_ctx_.h_workspace_.resize(host_ws);

    EIGEN_CUSOLVER_CHECK(
        cusolverDnXgeqrf(solver_ctx_.cusolverHandle(), solver_ctx_.params_.p, fm, fn, dtype, d_qr_.get(), lda_, dtype,
                         d_tau_.get(), dtype, solver_ctx_.scratch_workspace(), dev_ws,
                         host_ws > 0 ? solver_ctx_.h_workspace_.data() : nullptr, host_ws, solver_ctx_.scratch_info()));

    solver_ctx_.enqueue_info_copy();
  }

  // Applies Q (CUBLAS_OP_N) or Q^H (CUBLAS_OP_T for real, CUBLAS_OP_C for complex)
  // in place. Workspace comes from solver_ctx_'s grow-only scratch, so there is no
  // per-call malloc/free.
  void apply_Q(cublasOperation_t op, void* d_B, int64_t ldb, int64_t nrhs) const {
    const int im = internal::to_blas_int(factor_rows());
    const int in = internal::to_blas_int(nrhs);
    const int ik = internal::to_blas_int(k());
    const int ilda = internal::to_blas_int(lda_);
    const int ildb = internal::to_blas_int(ldb);

    int lwork = 0;
    EIGEN_CUSOLVER_CHECK(internal::cusolverDnXormqr_bufferSize(
        solver_ctx_.cusolverHandle(), CUBLAS_SIDE_LEFT, op, im, in, ik, static_cast<const Scalar*>(d_qr_.get()), ilda,
        static_cast<const Scalar*>(d_tau_.get()), static_cast<const Scalar*>(d_B), ildb, &lwork));

    solver_ctx_.ensure_scratch(static_cast<size_t>(lwork) * sizeof(Scalar));

    EIGEN_CUSOLVER_CHECK(internal::cusolverDnXormqr(
        solver_ctx_.cusolverHandle(), CUBLAS_SIDE_LEFT, op, im, in, ik, static_cast<const Scalar*>(d_qr_.get()), ilda,
        static_cast<const Scalar*>(d_tau_.get()), static_cast<Scalar*>(d_B), ildb,
        static_cast<Scalar*>(solver_ctx_.scratch_workspace()), lwork, solver_ctx_.scratch_info()));
  }

  void apply_QH(void* d_B, int64_t ldb, int64_t nrhs) const {
    constexpr cublasOperation_t trans = NumTraits<Scalar>::IsComplex ? CUBLAS_OP_C : CUBLAS_OP_T;
    apply_Q(trans, d_B, ldb, nrhs);
  }

  PlainMatrix solve_overdetermined_host(const Ref<const PlainMatrix>& rhs) const {
    const Index nrhs = rhs.cols();
    const size_t b_bytes = static_cast<size_t>(m_) * static_cast<size_t>(nrhs) * sizeof(Scalar);

    internal::DeviceBuffer d_B(b_bytes);
    internal::upload_host_matrix(static_cast<Scalar*>(d_B.get()), m_, rhs.data(), rhs.outerStride(), m_, nrhs,
                                 solver_ctx_.stream());

    apply_QH(d_B.get(), m_, nrhs);
    trsm_R(d_B.get(), m_, nrhs, /*op=*/CUBLAS_OP_N);

    PlainMatrix X(n_, nrhs);
    if (m_ == n_) {
      EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpyAsync(X.data(), d_B.get(),
                                               static_cast<size_t>(n_) * static_cast<size_t>(nrhs) * sizeof(Scalar),
                                               cudaMemcpyDeviceToHost, solver_ctx_.stream()));
    } else {
      EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpy2DAsync(X.data(), static_cast<size_t>(n_) * sizeof(Scalar), d_B.get(),
                                                 static_cast<size_t>(m_) * sizeof(Scalar),
                                                 static_cast<size_t>(n_) * sizeof(Scalar), static_cast<size_t>(nrhs),
                                                 cudaMemcpyDeviceToHost, solver_ctx_.stream()));
    }
    EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(solver_ctx_.stream()));
    return X;
  }

  DeviceMatrix<Scalar> solve_overdetermined_device(const DeviceMatrix<Scalar>& d_B) const {
    const Index nrhs = d_B.cols();
    const size_t b_bytes = static_cast<size_t>(m_) * static_cast<size_t>(nrhs) * sizeof(Scalar);

    internal::DeviceBuffer d_work(b_bytes);
    EIGEN_CUDA_RUNTIME_CHECK(
        cudaMemcpyAsync(d_work.get(), d_B.data(), b_bytes, cudaMemcpyDeviceToDevice, solver_ctx_.stream()));

    apply_QH(d_work.get(), m_, nrhs);
    trsm_R(d_work.get(), m_, nrhs, /*op=*/CUBLAS_OP_N);

    if (m_ == n_) {
      DeviceMatrix<Scalar> result =
          DeviceMatrix<Scalar>::adopt(static_cast<Scalar*>(d_work.release()), n_, static_cast<Index>(nrhs));
      result.recordReady(solver_ctx_.stream());
      return result;
    }
    DeviceMatrix<Scalar> result(n_, static_cast<Index>(nrhs));
    EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpy2DAsync(result.data(), static_cast<size_t>(n_) * sizeof(Scalar), d_work.get(),
                                               static_cast<size_t>(m_) * sizeof(Scalar),
                                               static_cast<size_t>(n_) * sizeof(Scalar), static_cast<size_t>(nrhs),
                                               cudaMemcpyDeviceToDevice, solver_ctx_.stream()));
    result.recordReady(solver_ctx_.stream());
    return result;
  }

  //
  // We factored A^H = Q R, so A = R^H Q^H. Solving A X = B for X with min ||X||:
  //   z = R^{-H} B            (m × nrhs, occupies top m rows of an n × nrhs buffer)
  //   X = Q [z; 0]            (n × nrhs)

  PlainMatrix solve_underdetermined_host(const Ref<const PlainMatrix>& rhs, Index nrhs) const {
    const size_t x_bytes = static_cast<size_t>(n_) * static_cast<size_t>(nrhs) * sizeof(Scalar);

    internal::DeviceBuffer d_X(x_bytes);
    // Zero the full n × nrhs buffer; B will overwrite the top m × nrhs block.
    EIGEN_CUDA_RUNTIME_CHECK(cudaMemsetAsync(d_X.get(), 0, x_bytes, solver_ctx_.stream()));

    // B (m × nrhs) into the top of d_X (leading dim n).
    internal::upload_host_matrix(static_cast<Scalar*>(d_X.get()), n_, rhs.data(), rhs.outerStride(), m_, nrhs,
                                 solver_ctx_.stream());

    trsm_R(d_X.get(), n_, nrhs, trsm_op_conj_trans());
    apply_Q(CUBLAS_OP_N, d_X.get(), n_, nrhs);

    PlainMatrix X(n_, nrhs);
    EIGEN_CUDA_RUNTIME_CHECK(
        cudaMemcpyAsync(X.data(), d_X.get(), x_bytes, cudaMemcpyDeviceToHost, solver_ctx_.stream()));
    EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(solver_ctx_.stream()));
    return X;
  }

  DeviceMatrix<Scalar> solve_underdetermined_device(const DeviceMatrix<Scalar>& d_B) const {
    const Index nrhs = d_B.cols();
    const size_t x_bytes = static_cast<size_t>(n_) * static_cast<size_t>(nrhs) * sizeof(Scalar);

    internal::DeviceBuffer d_X(x_bytes);
    EIGEN_CUDA_RUNTIME_CHECK(cudaMemsetAsync(d_X.get(), 0, x_bytes, solver_ctx_.stream()));

    if (m_ > 0 && nrhs > 0) {
      EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpy2DAsync(d_X.get(), static_cast<size_t>(n_) * sizeof(Scalar), d_B.data(),
                                                 static_cast<size_t>(m_) * sizeof(Scalar),
                                                 static_cast<size_t>(m_) * sizeof(Scalar), static_cast<size_t>(nrhs),
                                                 cudaMemcpyDeviceToDevice, solver_ctx_.stream()));
    }

    trsm_R(d_X.get(), n_, nrhs, trsm_op_conj_trans());
    apply_Q(CUBLAS_OP_N, d_X.get(), n_, nrhs);

    DeviceMatrix<Scalar> result =
        DeviceMatrix<Scalar>::adopt(static_cast<Scalar*>(d_X.release()), n_, static_cast<Index>(nrhs));
    result.recordReady(solver_ctx_.stream());
    return result;
  }

  static cublasOperation_t trsm_op_conj_trans() { return NumTraits<Scalar>::IsComplex ? CUBLAS_OP_C : CUBLAS_OP_T; }

  // X := op(R)^{-1} B, in place on B. The m >= n branch passes CUBLAS_OP_N to
  // solve R X = (Q^H B)[:k,:]; the m < n branch passes OP_T/OP_C to solve R^H z = B.
  void trsm_R(void* d_B, int64_t ldb, int64_t nrhs, cublasOperation_t op) const {
    Scalar alpha(1);
    EIGEN_CUBLAS_CHECK(internal::cublasXtrsm(
        solver_ctx_.cublasHandle(), CUBLAS_SIDE_LEFT, CUBLAS_FILL_MODE_UPPER, op, CUBLAS_DIAG_NON_UNIT, k(), nrhs,
        &alpha, static_cast<const Scalar*>(d_qr_.get()), lda_, static_cast<Scalar*>(d_B), ldb));
  }
};
}  // namespace gpu
}  // namespace Eigen

#endif  // EIGEN_GPU_QR_H
