// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// GPU SVD using cuSOLVER's divide-and-conquer cusolverDnXgesvd. U, S, and VT
// stay on device; solve() forms X = V * diag(D) * U^H * B with cuBLAS GEMM.
//
// cuSOLVER returns VT rather than V, so VT is what is stored; matrixV() adjoints
// it to match JacobiSVD and BDCSVD.

#ifndef EIGEN_GPU_SVD_H
#define EIGEN_GPU_SVD_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

#include "./GpuSolverContext.h"

namespace Eigen {
namespace gpu {
template <typename Scalar_>
class SVD {
 public:
  using Scalar = Scalar_;
  using RealScalar = typename NumTraits<Scalar>::Real;
  using PlainMatrix = Eigen::Matrix<Scalar, Dynamic, Dynamic, ColMajor>;
  using PlainVector = Eigen::Matrix<Scalar, Dynamic, 1>;
  using RealVector = Eigen::Matrix<RealScalar, Dynamic, 1>;

  SVD() = default;

  /** Bind to \p ctx: run on its stream with its cuSOLVER/cuBLAS handles, so
   * solver work chains with other work on the same Context without
   * cross-stream event waits. \p ctx must outlive this object. */
  explicit SVD(Context& ctx) : solver_ctx_(ctx) {}

  template <typename InputType>
  explicit SVD(const DenseBase<InputType>& A, unsigned int options = ComputeThinU | ComputeThinV) {
    compute(A, options);
  }

  explicit SVD(const DeviceMatrix<Scalar>& d_A, unsigned int options = ComputeThinU | ComputeThinV) {
    compute(d_A, options);
  }

  /** Decompose a device-resident A immediately (adopt when m >= n, no copy). */
  explicit SVD(DeviceMatrix<Scalar>&& d_A, unsigned int options = ComputeThinU | ComputeThinV) {
    compute(std::move(d_A), options);
  }

  /** Bind to \p ctx and decompose A immediately. */
  template <typename InputType>
  SVD(Context& ctx, const DenseBase<InputType>& A, unsigned int options = ComputeThinU | ComputeThinV)
      : solver_ctx_(ctx) {
    compute(A, options);
  }

  /** Bind to \p ctx and decompose a device-resident A (D2D copy). */
  SVD(Context& ctx, const DeviceMatrix<Scalar>& d_A, unsigned int options = ComputeThinU | ComputeThinV)
      : solver_ctx_(ctx) {
    compute(d_A, options);
  }

  ~SVD() = default;

  SVD(const SVD&) = delete;
  SVD& operator=(const SVD&) = delete;

  SVD(SVD&& o) noexcept
      : solver_ctx_(std::move(o.solver_ctx_)),
        d_A_(std::move(o.d_A_)),
        d_U_(std::move(o.d_U_)),
        d_S_(std::move(o.d_S_)),
        d_VT_(std::move(o.d_VT_)),
        d_D_(std::move(o.d_D_)),
        cached_diag_kk_(o.cached_diag_kk_),
        cached_diag_lambda_(o.cached_diag_lambda_),
        diag_valid_(o.diag_valid_),
        options_(o.options_),
        m_(o.m_),
        n_(o.n_),
        lda_(o.lda_),
        transposed_(o.transposed_) {
    o.diag_valid_ = false;
    o.options_ = 0;
    o.m_ = 0;
    o.n_ = 0;
    o.lda_ = 0;
    o.transposed_ = false;
  }

  SVD& operator=(SVD&& o) noexcept {
    if (this != &o) {
      solver_ctx_ = std::move(o.solver_ctx_);
      d_A_ = std::move(o.d_A_);
      d_U_ = std::move(o.d_U_);
      d_S_ = std::move(o.d_S_);
      d_VT_ = std::move(o.d_VT_);
      d_D_ = std::move(o.d_D_);
      cached_diag_kk_ = o.cached_diag_kk_;
      cached_diag_lambda_ = o.cached_diag_lambda_;
      diag_valid_ = o.diag_valid_;
      options_ = o.options_;
      m_ = o.m_;
      n_ = o.n_;
      lda_ = o.lda_;
      transposed_ = o.transposed_;
      o.diag_valid_ = false;
      o.options_ = 0;
      o.m_ = 0;
      o.n_ = 0;
      o.lda_ = 0;
      o.transposed_ = false;
    }
    return *this;
  }

  template <typename InputType>
  SVD& compute(const DenseBase<InputType>& A, unsigned int options = ComputeThinU | ComputeThinV) {
    // Upload to device, then delegate to the adopting overload — the freshly
    // uploaded matrix is consumed in place by gesvd, so no second device copy.
    // The wide-matrix transpose runs on the GPU (via cublasXgeam) inside the
    // device-input path; no host transpose.
    return compute(DeviceMatrix<Scalar>::fromHost(A.derived(), solver_ctx_.stream()), options);
  }

  SVD& compute(const DeviceMatrix<Scalar>& d_A, unsigned int options = ComputeThinU | ComputeThinV) {
    if (!begin_compute(d_A, options)) return *this;

    if (transposed_) {
      transpose_into_input(d_A);
    } else {
      const size_t mat_bytes = static_cast<size_t>(lda_) * static_cast<size_t>(n_) * sizeof(Scalar);
      d_A_ = internal::DeviceBuffer(mat_bytes);
      EIGEN_CUDA_RUNTIME_CHECK(
          cudaMemcpyAsync(d_A_.get(), d_A.data(), mat_bytes, cudaMemcpyDeviceToDevice, solver_ctx_.stream()));
    }

    factorize();
    return *this;
  }

  /** Decompose a device matrix (move). For m >= n the buffer is adopted and
   * consumed in place by gesvd — no copy; for m < n a transposed copy is
   * unavoidable. */
  SVD& compute(DeviceMatrix<Scalar>&& d_A, unsigned int options = ComputeThinU | ComputeThinV) {
    if (!begin_compute(d_A, options)) return *this;

    if (transposed_) {
      transpose_into_input(d_A);
    } else {
      const size_t a_bytes = d_A.sizeInBytes();
      d_A_ = internal::DeviceBuffer::adopt(static_cast<void*>(d_A.release()), a_bytes);
    }

    factorize();
    return *this;
  }

  ComputationInfo info() const { return solver_ctx_.info(); }

  Index rows() const { return transposed_ ? n_ : m_; }
  Index cols() const { return transposed_ ? m_ : n_; }

  /** Singular values (always available). Downloads from device on each call. */
  RealVector singularValues() const {
    eigen_assert(solver_ctx_.info() == Success);
    const Index k = (std::min)(m_, n_);
    RealVector S(k);
    EIGEN_CUDA_RUNTIME_CHECK(
        cudaMemcpy(S.data(), d_S_.get(), static_cast<size_t>(k) * sizeof(RealScalar), cudaMemcpyDeviceToHost));
    return S;
  }

  /** Left singular vectors U. */
  PlainMatrix matrixU() const {
    eigen_assert(solver_ctx_.info() == Success);
    eigen_assert((options_ & (ComputeThinU | ComputeFullU)) && "matrixU() requires ComputeThinU or ComputeFullU");
    const Index m_orig = transposed_ ? n_ : m_;
    const Index n_orig = transposed_ ? m_ : n_;
    const Index k = (std::min)(m_orig, n_orig);
    if (!transposed_) {
      const Index ucols = (options_ & ComputeFullU) ? m_ : k;
      PlainMatrix U(m_, ucols);
      EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpy(U.data(), d_U_.get(),
                                          static_cast<size_t>(m_) * static_cast<size_t>(ucols) * sizeof(Scalar),
                                          cudaMemcpyDeviceToHost));
      return U;
    } else {
      const Index vtrows = (options_ & ComputeFullU) ? m_orig : k;
      PlainMatrix VT_stored(vtrows, n_);
      EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpy(VT_stored.data(), d_VT_.get(),
                                          static_cast<size_t>(vtrows) * static_cast<size_t>(n_) * sizeof(Scalar),
                                          cudaMemcpyDeviceToHost));
      return VT_stored.adjoint();
    }
  }

  /** Right singular vectors V (matches host JacobiSVD/BDCSVD). */
  PlainMatrix matrixV() const { return matrixVT().adjoint(); }

  /** Right singular vectors transposed V^T. */
  PlainMatrix matrixVT() const {
    eigen_assert(solver_ctx_.info() == Success);
    eigen_assert((options_ & (ComputeThinV | ComputeFullV)) && "matrixVT() requires ComputeThinV or ComputeFullV");
    const Index m_orig = transposed_ ? n_ : m_;
    const Index n_orig = transposed_ ? m_ : n_;
    const Index k = (std::min)(m_orig, n_orig);
    if (!transposed_) {
      const Index vtrows = (options_ & ComputeFullV) ? n_ : k;
      PlainMatrix VT(vtrows, n_);
      EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpy(VT.data(), d_VT_.get(),
                                          static_cast<size_t>(vtrows) * static_cast<size_t>(n_) * sizeof(Scalar),
                                          cudaMemcpyDeviceToHost));
      return VT;
    } else {
      const Index ucols = (options_ & ComputeFullV) ? n_orig : k;
      PlainMatrix U_stored(m_, ucols);
      EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpy(U_stored.data(), d_U_.get(),
                                          static_cast<size_t>(m_) * static_cast<size_t>(ucols) * sizeof(Scalar),
                                          cudaMemcpyDeviceToHost));
      return U_stored.adjoint();
    }
  }

  //
  // These return non-owning DeviceMatrix views over the SVD's internal device storage.
  // The view borrows the pointer: destruction does not free; the SVD object must outlive
  // any view derived from it. For the common case (m >= n) all three accessors are pure
  // metadata: zero kernel launches, zero allocations.
  //
  // For wide matrices (m < n, internally factored as A^H), original U and V^T are the
  // adjoints of the stored buffers, so d_matrixU() / d_matrixVT() build them via a
  // cublasXgeam into an owning temporary. d_singularValues() remains zero-copy.

  /** Singular values as a k × 1 view on this solver's stream. */
  DeviceMatrix<RealScalar> d_singularValues() const {
    eigen_assert(solver_ctx_.info() == Success);
    const Index k = (std::min)(m_, n_);
    auto v = DeviceMatrix<RealScalar>::view(static_cast<RealScalar*>(d_S_.get()), k, 1);
    v.recordReady(solver_ctx_.stream());
    return v;
  }

  /** Left singular vectors U as a DeviceMatrix on this solver's stream.
   * For m >= n: zero-copy view. For m < n: owning (one cublasXgeam adjoint pass). */
  DeviceMatrix<Scalar> d_matrixU() const {
    eigen_assert(solver_ctx_.info() == Success);
    eigen_assert((options_ & (ComputeThinU | ComputeFullU)) && "d_matrixU() requires ComputeThinU or ComputeFullU");
    const Index m_orig = transposed_ ? n_ : m_;
    const Index n_orig = transposed_ ? m_ : n_;
    const Index k = (std::min)(m_orig, n_orig);
    if (!transposed_) {
      const Index ucols = (options_ & ComputeFullU) ? m_ : k;
      auto v = DeviceMatrix<Scalar>::view(static_cast<Scalar*>(d_U_.get()), m_, ucols);
      v.recordReady(solver_ctx_.stream());
      return v;
    }
    // transposed: U_orig = VT_stored^H -> conjugate-transpose via cublasXgeam.
    const Index vtrows_stored = (options_ & ComputeFullU) ? n_ : k;
    DeviceMatrix<Scalar> result(n_, vtrows_stored);
    if (n_ > 0 && vtrows_stored > 0) {
      Scalar alpha_one(1), beta_zero(0);
      EIGEN_CUBLAS_CHECK(internal::cublasXgeam(solver_ctx_.cublasHandle(), CUBLAS_OP_C, CUBLAS_OP_N, n_, vtrows_stored,
                                               &alpha_one, static_cast<const Scalar*>(d_VT_.get()), vtrows_stored,
                                               &beta_zero, static_cast<const Scalar*>(nullptr), n_, result.data(), n_));
      result.recordReady(solver_ctx_.stream());
    }
    return result;
  }

  /** Right singular vectors transposed V^T as a DeviceMatrix on this solver's stream.
   * For m >= n: zero-copy view. For m < n: owning (one cublasXgeam adjoint pass). */
  DeviceMatrix<Scalar> d_matrixVT() const {
    eigen_assert(solver_ctx_.info() == Success);
    eigen_assert((options_ & (ComputeThinV | ComputeFullV)) && "d_matrixVT() requires ComputeThinV or ComputeFullV");
    const Index m_orig = transposed_ ? n_ : m_;
    const Index n_orig = transposed_ ? m_ : n_;
    const Index k = (std::min)(m_orig, n_orig);
    if (!transposed_) {
      const Index vtrows = (options_ & ComputeFullV) ? n_ : k;
      auto v = DeviceMatrix<Scalar>::view(static_cast<Scalar*>(d_VT_.get()), vtrows, n_);
      v.recordReady(solver_ctx_.stream());
      return v;
    }
    // transposed: VT_orig = U_stored^H.
    const Index ucols = (options_ & ComputeFullV) ? n_orig : k;
    DeviceMatrix<Scalar> result(ucols, m_);
    if (ucols > 0 && m_ > 0) {
      Scalar alpha_one(1), beta_zero(0);
      EIGEN_CUBLAS_CHECK(internal::cublasXgeam(solver_ctx_.cublasHandle(), CUBLAS_OP_C, CUBLAS_OP_N, ucols, m_,
                                               &alpha_one, static_cast<const Scalar*>(d_U_.get()), m_, &beta_zero,
                                               static_cast<const Scalar*>(nullptr), ucols, result.data(), ucols));
      result.recordReady(solver_ctx_.stream());
    }
    return result;
  }

  /** Number of singular values above threshold. */
  Index rank(RealScalar threshold = RealScalar(-1)) const {
    RealVector S = singularValues();
    if (S.size() == 0) return 0;
    if (threshold < 0) {
      threshold = (std::max)(m_, n_) * S(0) * NumTraits<RealScalar>::epsilon();
    }
    return (S.array() > threshold).count();
  }

  /** Pseudoinverse solve: X = V * diag(1/S) * U^H * B. */
  template <typename Rhs>
  PlainMatrix solve(const MatrixBase<Rhs>& B) const {
    return solve_impl(B, (std::min)(m_, n_), RealScalar(0));
  }

  /** Truncated solve: use only top trunc singular triplets. */
  template <typename Rhs>
  PlainMatrix solve(const MatrixBase<Rhs>& B, Index trunc) const {
    eigen_assert(trunc > 0 && trunc <= (std::min)(m_, n_));
    return solve_impl(B, trunc, RealScalar(0));
  }

  /** Tikhonov-regularized solve: D_ii = S_i / (S_i^2 + lambda^2). */
  template <typename Rhs>
  PlainMatrix solve(const MatrixBase<Rhs>& B, RealScalar lambda) const {
    eigen_assert(lambda > 0);
    return solve_impl(B, (std::min)(m_, n_), lambda);
  }

  /** Pseudoinverse solve with device-resident RHS: returns an n × nrhs
   * DeviceMatrix that stays on device. The first solve for a given
   * (truncation, lambda) downloads the singular values (one host sync) to
   * build the inverse diagonal; subsequent solves reuse the cached device
   * diagonal and are fully asynchronous. */
  DeviceMatrix<Scalar> solve(const DeviceMatrix<Scalar>& d_B) const {
    return solve_device_impl(d_B, (std::min)(m_, n_), RealScalar(0));
  }

  /** Truncated device solve: use only top trunc singular triplets. */
  DeviceMatrix<Scalar> solve(const DeviceMatrix<Scalar>& d_B, Index trunc) const {
    eigen_assert(trunc > 0 && trunc <= (std::min)(m_, n_));
    return solve_device_impl(d_B, trunc, RealScalar(0));
  }

  /** Tikhonov-regularized device solve: D_ii = S_i / (S_i^2 + lambda^2). */
  DeviceMatrix<Scalar> solve(const DeviceMatrix<Scalar>& d_B, RealScalar lambda) const {
    eigen_assert(lambda > 0);
    return solve_device_impl(d_B, (std::min)(m_, n_), lambda);
  }

  cudaStream_t stream() const { return solver_ctx_.stream(); }

 private:
  mutable internal::GpuSolverContext solver_ctx_;
  internal::DeviceBuffer d_A_;   // gesvd input scratch; released after factorize()
  internal::DeviceBuffer d_U_;   // grow-only
  internal::DeviceBuffer d_S_;   // grow-only
  internal::DeviceBuffer d_VT_;  // grow-only
  // Cached inverse-diagonal for solve (built lazily, reused across solves; grow-only).
  mutable internal::DeviceBuffer d_D_;
  mutable Index cached_diag_kk_ = -1;
  mutable RealScalar cached_diag_lambda_ = RealScalar(-1);
  mutable bool diag_valid_ = false;
  unsigned int options_ = 0;
  int64_t m_ = 0;
  int64_t n_ = 0;
  int64_t lda_ = 0;
  bool transposed_ = false;

  // Common compute() prologue: record shape/options, reset info and cached
  // diagonal, wait on input. Returns false (clearing state) for empty input.
  bool begin_compute(const DeviceMatrix<Scalar>& d_A, unsigned int options) {
    options_ = options;
    m_ = d_A.rows();
    n_ = d_A.cols();
    lda_ = 0;
    transposed_ = false;
    diag_valid_ = false;
    if (!solver_ctx_.begin_compute(m_ != 0 && n_ != 0)) {
      d_A_ = internal::DeviceBuffer();
      d_U_ = internal::DeviceBuffer();
      d_S_ = internal::DeviceBuffer();
      d_VT_ = internal::DeviceBuffer();
      return false;
    }
    transposed_ = (m_ < n_);
    if (transposed_) {
      std::swap(m_, n_);
      lda_ = m_;
    } else {
      lda_ = static_cast<int64_t>(d_A.rows());
    }
    d_A.waitReady(solver_ctx_.stream());
    return true;
  }

  // Wide input (m < n): produce d_A_ = A^H on device via cuBLAS geam.
  void transpose_into_input(const DeviceMatrix<Scalar>& d_A) {
    const size_t mat_bytes = static_cast<size_t>(lda_) * static_cast<size_t>(n_) * sizeof(Scalar);
    d_A_ = internal::DeviceBuffer(mat_bytes);
    // geam: C(m×n) = alpha * op(A) + beta * op(B). beta=0, B=nullptr.
    Scalar alpha_one(1), beta_zero(0);
    EIGEN_CUBLAS_CHECK(internal::cublasXgeam(solver_ctx_.cublasHandle(), CUBLAS_OP_C, CUBLAS_OP_N, m_, n_, &alpha_one,
                                             d_A.data(), d_A.rows(), &beta_zero, static_cast<const Scalar*>(nullptr),
                                             m_, static_cast<Scalar*>(d_A_.get()), m_));
  }

  // Swap U↔V flags for the transposed case.
  static unsigned int swap_uv_options(unsigned int opts) {
    unsigned int result = 0;
    if (opts & ComputeThinU) result |= ComputeThinV;
    if (opts & ComputeFullU) result |= ComputeFullV;
    if (opts & ComputeThinV) result |= ComputeThinU;
    if (opts & ComputeFullV) result |= ComputeFullU;
    return result;
  }

  static signed char jobu(unsigned int opts) {
    if (opts & ComputeFullU) return 'A';
    if (opts & ComputeThinU) return 'S';
    return 'N';
  }

  static signed char jobvt(unsigned int opts) {
    if (opts & ComputeFullV) return 'A';
    if (opts & ComputeThinV) return 'S';
    return 'N';
  }

  void factorize() {
    constexpr cudaDataType_t dtype = internal::cusolver_data_type<Scalar>::value;
    constexpr cudaDataType_t rtype = internal::cuda_data_type<RealScalar>::value;
    const Index k = (std::min)(m_, n_);

    solver_ctx_.mark_pending();

    internal::ensure_sized(d_S_, static_cast<size_t>(k) * sizeof(RealScalar));

    const unsigned int int_opts = transposed_ ? swap_uv_options(options_) : options_;

    const Index ucols = (int_opts & ComputeFullU) ? m_ : ((int_opts & ComputeThinU) ? k : 0);
    const Index vtrows = (int_opts & ComputeFullV) ? n_ : ((int_opts & ComputeThinV) ? k : 0);
    const int64_t ldu = m_;
    const int64_t ldvt = vtrows > 0 ? vtrows : 1;

    if (ucols > 0) {
      internal::ensure_sized(d_U_, static_cast<size_t>(m_) * static_cast<size_t>(ucols) * sizeof(Scalar));
    }
    if (vtrows > 0) {
      internal::ensure_sized(d_VT_, static_cast<size_t>(vtrows) * static_cast<size_t>(n_) * sizeof(Scalar));
    }

    eigen_assert(m_ >= n_ && "Internal error: m_ < n_ should have been handled by transpose in compute()");
    size_t dev_ws = 0, host_ws = 0;
    EIGEN_CUSOLVER_CHECK(cusolverDnXgesvd_bufferSize(
        solver_ctx_.cusolverHandle(), solver_ctx_.params_.p, jobu(int_opts), jobvt(int_opts), m_, n_, dtype, d_A_.get(),
        lda_, rtype, d_S_.get(), dtype, ucols > 0 ? d_U_.get() : nullptr, ldu, dtype,
        vtrows > 0 ? d_VT_.get() : nullptr, ldvt, dtype, &dev_ws, &host_ws));

    solver_ctx_.ensure_scratch(dev_ws);
    solver_ctx_.h_workspace_.resize(host_ws);

    EIGEN_CUSOLVER_CHECK(cusolverDnXgesvd(
        solver_ctx_.cusolverHandle(), solver_ctx_.params_.p, jobu(int_opts), jobvt(int_opts), m_, n_, dtype, d_A_.get(),
        lda_, rtype, d_S_.get(), dtype, ucols > 0 ? d_U_.get() : nullptr, ldu, dtype,
        vtrows > 0 ? d_VT_.get() : nullptr, ldvt, dtype, solver_ctx_.scratch_workspace(), dev_ws,
        host_ws > 0 ? solver_ctx_.h_workspace_.data() : nullptr, host_ws, solver_ctx_.scratch_info()));

    solver_ctx_.enqueue_info_copy();

    // The input copy is pure gesvd scratch — release it now. The free is
    // stream-ordered (or synchronous on the fallback allocator), so it waits
    // for gesvd to retire; the memory returns to the pool instead of staying
    // resident for the solver's lifetime.
    d_A_ = internal::DeviceBuffer();
  }

  // Ensure d_D_ holds the kk-entry inverse diagonal for (kk, lambda).
  // Downloads S and synchronizes only when the cached diagonal doesn't match;
  // repeated solves with the same truncation/regularization are then free of
  // host syncs and H2D traffic for the diagonal.
  //
  // For lambda == 0 we mirror Eigen's SVDBase::_solve_impl: drop singular
  // values below S(0) * k * eps (numerical-rank truncation), so this
  // pseudoinverse solve agrees with CPU BDCSVD::solve on near-singular A.
  // dgmm wants the diagonal in the matrix scalar type — for complex Scalar
  // the diagonal is still real, so we build the real values then cast.
  void build_diag(Index kk, RealScalar lambda) const {
    if (diag_valid_ && cached_diag_kk_ == kk && cached_diag_lambda_ == lambda) return;

    const Index k = (std::min)(m_, n_);
    RealVector S(k);
    EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpyAsync(S.data(), d_S_.get(), static_cast<size_t>(k) * sizeof(RealScalar),
                                             cudaMemcpyDeviceToHost, solver_ctx_.stream()));
    EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(solver_ctx_.stream()));

    const RealScalar drop_threshold = S(0) * RealScalar(k) * NumTraits<RealScalar>::epsilon();
    auto S_head = S.head(kk).array();
    PlainVector D(kk);
    if (lambda == RealScalar(0)) {
      D = (S_head > drop_threshold).select(S_head.inverse(), RealScalar(0)).matrix().template cast<Scalar>();
    } else {
      D = (S_head / (S_head.square() + lambda * lambda)).matrix().template cast<Scalar>();
    }

    const size_t d_bytes = static_cast<size_t>(kk) * sizeof(Scalar);
    internal::ensure_sized(d_D_, d_bytes);
    EIGEN_CUDA_RUNTIME_CHECK(
        cudaMemcpyAsync(d_D_.get(), D.data(), d_bytes, cudaMemcpyHostToDevice, solver_ctx_.stream()));
    cached_diag_kk_ = kk;
    cached_diag_lambda_ = lambda;
    diag_valid_ = true;
  }

  // Shared pseudoinverse application: X = V_orig * diag(D) * U_orig^H * B,
  // entirely on device. Assumes build_diag(kk, ...) has run and B_dev/X_dev
  // are device pointers with leading dimensions m_orig / n_orig.
  void apply_pinv(const Scalar* B_dev, Index kk, Index nrhs, Scalar* X_dev) const {
    const Index m_orig = transposed_ ? n_ : m_;
    const Index n_orig = transposed_ ? m_ : n_;
    const Index k = (std::min)(m_, n_);

    auto* U_dev = static_cast<const Scalar*>(d_U_.get());
    auto* VT_dev = static_cast<const Scalar*>(d_VT_.get());

    Scalar scalars[2] = {Scalar(1), Scalar(0)};

    // Step 1: tmp = U_orig^H * B  (kk × nrhs).
    internal::DeviceBuffer d_tmp(static_cast<size_t>(kk) * static_cast<size_t>(nrhs) * sizeof(Scalar));
    auto* tmp_dev = static_cast<Scalar*>(d_tmp.get());
    if (!transposed_) {
      internal::cublaslt_gemm(solver_ctx_.cublasLtHandle(), solver_ctx_.cublasHandle(), CUBLAS_OP_C, CUBLAS_OP_N, kk,
                              nrhs, m_, &scalars[0], U_dev, m_, B_dev, m_orig, &scalars[1], tmp_dev, kk,
                              solver_ctx_.gemmWorkspace(), solver_ctx_.gemmPlanCache(),
                              solver_ctx_.cublasLtMaxWorkspaceBytes(), solver_ctx_.stream());
    } else {
      const Index vtrows_stored = (swap_uv_options(options_) & ComputeFullV) ? n_ : k;
      internal::cublaslt_gemm(solver_ctx_.cublasLtHandle(), solver_ctx_.cublasHandle(), CUBLAS_OP_N, CUBLAS_OP_N, kk,
                              nrhs, m_orig, &scalars[0], VT_dev, vtrows_stored, B_dev, m_orig, &scalars[1], tmp_dev, kk,
                              solver_ctx_.gemmWorkspace(), solver_ctx_.gemmPlanCache(),
                              solver_ctx_.cublasLtMaxWorkspaceBytes(), solver_ctx_.stream());
    }

    // Step 2: tmp = diag(D) * tmp on device via cublasXdgmm.
    EIGEN_CUBLAS_CHECK(internal::cublasXdgmm(solver_ctx_.cublasHandle(), CUBLAS_SIDE_LEFT, kk, nrhs, tmp_dev, kk,
                                             static_cast<const Scalar*>(d_D_.get()), 1, tmp_dev, kk));

    // Step 3: X = V_orig * tmp  (n_orig × nrhs).
    if (!transposed_) {
      const Index vtrows = (options_ & ComputeFullV) ? n_ : k;
      internal::cublaslt_gemm(solver_ctx_.cublasLtHandle(), solver_ctx_.cublasHandle(), CUBLAS_OP_C, CUBLAS_OP_N,
                              n_orig, nrhs, kk, &scalars[0], VT_dev, vtrows, tmp_dev, kk, &scalars[1], X_dev, n_orig,
                              solver_ctx_.gemmWorkspace(), solver_ctx_.gemmPlanCache(),
                              solver_ctx_.cublasLtMaxWorkspaceBytes(), solver_ctx_.stream());
    } else {
      internal::cublaslt_gemm(solver_ctx_.cublasLtHandle(), solver_ctx_.cublasHandle(), CUBLAS_OP_N, CUBLAS_OP_N,
                              n_orig, nrhs, kk, &scalars[0], U_dev, m_, tmp_dev, kk, &scalars[1], X_dev, n_orig,
                              solver_ctx_.gemmWorkspace(), solver_ctx_.gemmPlanCache(),
                              solver_ctx_.cublasLtMaxWorkspaceBytes(), solver_ctx_.stream());
    }
  }

  template <typename Rhs>
  PlainMatrix solve_impl(const MatrixBase<Rhs>& B, Index trunc, RealScalar lambda) const {
    eigen_assert(solver_ctx_.info() == Success && "SVD::solve called on a failed or uninitialized decomposition");
    eigen_assert((options_ & (ComputeThinU | ComputeFullU)) && "solve requires U");
    eigen_assert((options_ & (ComputeThinV | ComputeFullV)) && "solve requires V");

    const Index m_orig = transposed_ ? n_ : m_;
    const Index n_orig = transposed_ ? m_ : n_;
    eigen_assert(B.rows() == m_orig);

    const Index k = (std::min)(m_, n_);
    const Index kk = (std::min)(trunc, k);
    const Index nrhs = B.cols();

    // Empty problem: no rank, no RHS, or zero domain -> result is the zero matrix.
    if (kk == 0 || nrhs == 0 || n_orig == 0) {
      return PlainMatrix::Zero(n_orig, nrhs);
    }

    // Enqueue the B upload before build_diag: when the diagonal must be
    // (re)built, its S-download sync then also covers the in-flight upload —
    // one blocking wait instead of two.
    const Ref<const PlainMatrix> rhs(B.derived());
    internal::DeviceBuffer d_B(static_cast<size_t>(m_orig) * static_cast<size_t>(nrhs) * sizeof(Scalar));
    internal::upload_host_matrix(static_cast<Scalar*>(d_B.get()), m_orig, rhs.data(), rhs.outerStride(), m_orig, nrhs,
                                 solver_ctx_.stream());
    build_diag(kk, lambda);

    PlainMatrix X(n_orig, nrhs);
    internal::DeviceBuffer d_X(static_cast<size_t>(n_orig) * static_cast<size_t>(nrhs) * sizeof(Scalar));
    apply_pinv(static_cast<const Scalar*>(d_B.get()), kk, nrhs, static_cast<Scalar*>(d_X.get()));

    EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpyAsync(X.data(), d_X.get(),
                                             static_cast<size_t>(n_orig) * static_cast<size_t>(nrhs) * sizeof(Scalar),
                                             cudaMemcpyDeviceToHost, solver_ctx_.stream()));
    EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(solver_ctx_.stream()));

    return X;
  }

  DeviceMatrix<Scalar> solve_device_impl(const DeviceMatrix<Scalar>& d_B, Index trunc, RealScalar lambda) const {
    eigen_assert(solver_ctx_.info() == Success && "SVD::solve called on a failed or uninitialized decomposition");
    eigen_assert((options_ & (ComputeThinU | ComputeFullU)) && "solve requires U");
    eigen_assert((options_ & (ComputeThinV | ComputeFullV)) && "solve requires V");

    const Index m_orig = transposed_ ? n_ : m_;
    const Index n_orig = transposed_ ? m_ : n_;
    eigen_assert(d_B.rows() == m_orig);

    const Index k = (std::min)(m_, n_);
    const Index kk = (std::min)(trunc, k);
    const Index nrhs = d_B.cols();

    if (kk == 0 || nrhs == 0 || n_orig == 0) {
      DeviceMatrix<Scalar> X(n_orig, nrhs);
      X.setZero(solver_ctx_.stream());
      return X;
    }

    d_B.waitReady(solver_ctx_.stream());
    build_diag(kk, lambda);

    DeviceMatrix<Scalar> X(n_orig, nrhs);
    apply_pinv(d_B.data(), kk, nrhs, X.data());
    X.recordReady(solver_ctx_.stream());
    return X;
  }
};
}  // namespace gpu
}  // namespace Eigen

#endif  // EIGEN_GPU_SVD_H
