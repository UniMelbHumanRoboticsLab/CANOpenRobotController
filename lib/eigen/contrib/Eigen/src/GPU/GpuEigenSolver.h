// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// GPU self-adjoint eigenvalue decomposition using cuSOLVER's divide-and-conquer
// cusolverDnXsyevd. Eigenvalues and eigenvectors stay on device.

#ifndef EIGEN_GPU_EIGENSOLVER_H
#define EIGEN_GPU_EIGENSOLVER_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

#include "./GpuSolverContext.h"

namespace Eigen {
namespace gpu {
template <typename Scalar_>
class SelfAdjointEigenSolver {
 public:
  using Scalar = Scalar_;
  using RealScalar = typename NumTraits<Scalar>::Real;
  using PlainMatrix = Eigen::Matrix<Scalar, Dynamic, Dynamic, ColMajor>;
  using RealVector = Eigen::Matrix<RealScalar, Dynamic, 1>;

  SelfAdjointEigenSolver() = default;

  /** Bind to \p ctx: run on its stream with its cuSOLVER/cuBLAS handles, so
   * solver work chains with other work on the same Context without
   * cross-stream event waits. \p ctx must outlive this object. */
  explicit SelfAdjointEigenSolver(Context& ctx) : solver_ctx_(ctx) {}

  /** \param options  Eigen::ComputeEigenvectors (default) or Eigen::EigenvaluesOnly. */
  template <typename InputType>
  explicit SelfAdjointEigenSolver(const DenseBase<InputType>& A, int options = ComputeEigenvectors) {
    compute(A, options);
  }

  explicit SelfAdjointEigenSolver(const DeviceMatrix<Scalar>& d_A, int options = ComputeEigenvectors) {
    compute(d_A, options);
  }

  /** Decompose a device-resident A immediately (adopt, no copy). */
  explicit SelfAdjointEigenSolver(DeviceMatrix<Scalar>&& d_A, int options = ComputeEigenvectors) {
    compute(std::move(d_A), options);
  }

  /** Bind to \p ctx and decompose A immediately. */
  template <typename InputType>
  SelfAdjointEigenSolver(Context& ctx, const DenseBase<InputType>& A, int options = ComputeEigenvectors)
      : solver_ctx_(ctx) {
    compute(A, options);
  }

  /** Bind to \p ctx and decompose a device-resident A (D2D copy). */
  SelfAdjointEigenSolver(Context& ctx, const DeviceMatrix<Scalar>& d_A, int options = ComputeEigenvectors)
      : solver_ctx_(ctx) {
    compute(d_A, options);
  }

  ~SelfAdjointEigenSolver() = default;

  SelfAdjointEigenSolver(const SelfAdjointEigenSolver&) = delete;
  SelfAdjointEigenSolver& operator=(const SelfAdjointEigenSolver&) = delete;

  SelfAdjointEigenSolver(SelfAdjointEigenSolver&& o) noexcept
      : solver_ctx_(std::move(o.solver_ctx_)),
        d_A_(std::move(o.d_A_)),
        d_W_(std::move(o.d_W_)),
        compute_eigenvectors_(o.compute_eigenvectors_),
        n_(o.n_),
        lda_(o.lda_) {
    o.compute_eigenvectors_ = true;
    o.n_ = 0;
    o.lda_ = 0;
  }

  SelfAdjointEigenSolver& operator=(SelfAdjointEigenSolver&& o) noexcept {
    if (this != &o) {
      solver_ctx_ = std::move(o.solver_ctx_);
      d_A_ = std::move(o.d_A_);
      d_W_ = std::move(o.d_W_);
      compute_eigenvectors_ = o.compute_eigenvectors_;
      n_ = o.n_;
      lda_ = o.lda_;
      o.compute_eigenvectors_ = true;
      o.n_ = 0;
      o.lda_ = 0;
    }
    return *this;
  }

  template <typename InputType>
  SelfAdjointEigenSolver& compute(const DenseBase<InputType>& A, int options = ComputeEigenvectors) {
    // Route through the adopting overload: the freshly uploaded matrix is
    // decomposed in place (syevd overwrites its input) — no second device copy.
    return compute(DeviceMatrix<Scalar>::fromHost(A.derived(), solver_ctx_.stream()), options);
  }

  SelfAdjointEigenSolver& compute(const DeviceMatrix<Scalar>& d_A, int options = ComputeEigenvectors) {
    if (!begin_compute(d_A, options)) return *this;

    const size_t mat_bytes = static_cast<size_t>(lda_) * static_cast<size_t>(n_) * sizeof(Scalar);
    internal::ensure_sized(d_A_, mat_bytes);
    EIGEN_CUDA_RUNTIME_CHECK(
        cudaMemcpyAsync(d_A_.get(), d_A.data(), mat_bytes, cudaMemcpyDeviceToDevice, solver_ctx_.stream()));

    factorize();
    return *this;
  }

  /** Decompose a device matrix (move): the buffer is adopted and overwritten
   * in place by syevd — no copy. */
  SelfAdjointEigenSolver& compute(DeviceMatrix<Scalar>&& d_A, int options = ComputeEigenvectors) {
    if (!begin_compute(d_A, options)) return *this;

    d_A_ = internal::DeviceBuffer::adopt(static_cast<void*>(d_A.release()),
                                         static_cast<size_t>(lda_) * static_cast<size_t>(n_) * sizeof(Scalar));

    factorize();
    return *this;
  }

  ComputationInfo info() const { return solver_ctx_.info(); }

  Index cols() const { return n_; }
  Index rows() const { return n_; }

  /** Eigenvalues in ascending order. Downloads from device. */
  RealVector eigenvalues() const {
    eigen_assert(solver_ctx_.info() == Success);
    RealVector W(n_);
    if (n_ > 0) {
      EIGEN_CUDA_RUNTIME_CHECK(
          cudaMemcpy(W.data(), d_W_.get(), static_cast<size_t>(n_) * sizeof(RealScalar), cudaMemcpyDeviceToHost));
    }
    return W;
  }

  /** Eigenvectors (columns). Downloads from device.
   * Requires ComputeEigenvectors mode. */
  PlainMatrix eigenvectors() const {
    eigen_assert(solver_ctx_.info() == Success);
    eigen_assert(compute_eigenvectors_ && "eigenvectors() requires ComputeEigenvectors option");
    PlainMatrix V(n_, n_);
    if (n_ > 0) {
      EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpy(V.data(), d_A_.get(),
                                          static_cast<size_t>(lda_) * static_cast<size_t>(n_) * sizeof(Scalar),
                                          cudaMemcpyDeviceToHost));
    }
    return V;
  }

  //
  // These return non-owning DeviceMatrix views over this solver's internal storage. The
  // view borrows the pointer: destruction does not free; this solver must outlive any
  // view derived from it. Both accessors are pure metadata — zero kernel launches.

  /** Eigenvalues as an n × 1 view on this solver's stream. No host sync in
   * release builds — the recorded event orders downstream consumers after
   * syevd; debug builds verify the factorization status. */
  DeviceMatrix<RealScalar> d_eigenvalues() const {
    eigen_assert(solver_ctx_.info() == Success);
    auto v = DeviceMatrix<RealScalar>::view(static_cast<RealScalar*>(d_W_.get()), n_, 1);
    v.recordReady(solver_ctx_.stream());
    return v;
  }

  /** Eigenvectors (columns) as an n × n view on this solver's stream. Same
   * synchronization contract as d_eigenvalues(). */
  DeviceMatrix<Scalar> d_eigenvectors() const {
    eigen_assert(solver_ctx_.info() == Success);
    eigen_assert(compute_eigenvectors_ && "d_eigenvectors() requires ComputeEigenvectors option");
    auto v = DeviceMatrix<Scalar>::view(static_cast<Scalar*>(d_A_.get()), n_, n_);
    v.recordReady(solver_ctx_.stream());
    return v;
  }

  cudaStream_t stream() const { return solver_ctx_.stream(); }

 private:
  mutable internal::GpuSolverContext solver_ctx_;
  internal::DeviceBuffer d_A_;  // grow-only; overwritten with eigenvectors by syevd
  internal::DeviceBuffer d_W_;  // grow-only; eigenvalues (RealScalar, length n)
  bool compute_eigenvectors_ = true;
  int64_t n_ = 0;
  int64_t lda_ = 0;

  // Common compute() prologue: validate, record shape, reset info, wait on
  // input. Returns false (and clears stale buffers) for empty input.
  bool begin_compute(const DeviceMatrix<Scalar>& d_A, int options) {
    eigen_assert(d_A.rows() == d_A.cols() && "SelfAdjointEigenSolver requires a square matrix");
    eigen_assert((options == ComputeEigenvectors || options == EigenvaluesOnly) &&
                 "options must be ComputeEigenvectors or EigenvaluesOnly");
    compute_eigenvectors_ = (options == ComputeEigenvectors);
    n_ = d_A.rows();
    if (!solver_ctx_.begin_compute(n_ != 0)) {
      d_A_ = internal::DeviceBuffer();
      d_W_ = internal::DeviceBuffer();
      return false;
    }
    lda_ = n_;
    d_A.waitReady(solver_ctx_.stream());
    return true;
  }

  void factorize() {
    constexpr cudaDataType_t dtype = internal::cusolver_data_type<Scalar>::value;
    constexpr cudaDataType_t rtype = internal::cuda_data_type<RealScalar>::value;

    solver_ctx_.mark_pending();

    internal::ensure_sized(d_W_, static_cast<size_t>(n_) * sizeof(RealScalar));

    const cusolverEigMode_t jobz = compute_eigenvectors_ ? CUSOLVER_EIG_MODE_VECTOR : CUSOLVER_EIG_MODE_NOVECTOR;

    constexpr cublasFillMode_t uplo = CUBLAS_FILL_MODE_LOWER;

    size_t dev_ws = 0, host_ws = 0;
    EIGEN_CUSOLVER_CHECK(cusolverDnXsyevd_bufferSize(solver_ctx_.cusolverHandle(), solver_ctx_.params_.p, jobz, uplo,
                                                     n_, dtype, d_A_.get(), lda_, rtype, d_W_.get(), dtype, &dev_ws,
                                                     &host_ws));

    solver_ctx_.ensure_scratch(dev_ws);
    solver_ctx_.h_workspace_.resize(host_ws);

    EIGEN_CUSOLVER_CHECK(cusolverDnXsyevd(solver_ctx_.cusolverHandle(), solver_ctx_.params_.p, jobz, uplo, n_, dtype,
                                          d_A_.get(), lda_, rtype, d_W_.get(), dtype, solver_ctx_.scratch_workspace(),
                                          dev_ws, host_ws > 0 ? solver_ctx_.h_workspace_.data() : nullptr, host_ws,
                                          solver_ctx_.scratch_info()));

    solver_ctx_.enqueue_info_copy();
  }
};
}  // namespace gpu
}  // namespace Eigen

#endif  // EIGEN_GPU_EIGENSOLVER_H
