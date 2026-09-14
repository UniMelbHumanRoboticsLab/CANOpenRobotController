// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// GPU FFT via cuFFT: 1D and 2D C2C, R2C, and C2R transforms with plan caching.
//
// The stream and cuBLAS handle come from a bound gpu::Context, which defaults to
// Context::threadLocal() so an FFT shares a stream with the thread's other GPU
// work; pass an explicit Context to bind elsewhere.
//
// Inverse transforms are scaled by 1/n (1D) or 1/(n*m) (2D), so inv(fwd(x)) == x
// as in Eigen's CPU FFT.
//
// Not thread-safe: concurrent fwd/inv calls on one instance race on the cached
// plans and the bound Context. Use one FFT per thread.

#ifndef EIGEN_GPU_FFT_H
#define EIGEN_GPU_FFT_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

#include "./CuFftSupport.h"
#include "./CuBlasSupport.h"
#include "./GpuContext.h"

namespace Eigen {
namespace gpu {
// Default capacity of the per-FFT-instance cuFFT plan cache. Override by
// passing a capacity to the FFT constructor.
static constexpr std::size_t kDefaultCufftPlanCacheCapacity = 16;

template <typename Scalar_>
class FFT {
 public:
  using Scalar = Scalar_;
  using Complex = std::complex<Scalar>;
  using ComplexVector = Matrix<Complex, Dynamic, 1>;
  using RealVector = Matrix<Scalar, Dynamic, 1>;
  using ComplexMatrix = Matrix<Complex, Dynamic, Dynamic, ColMajor>;

  /** Construct an FFT bound to the calling thread's default Context.
   * The instance is thread-affine: it must not outlive the thread that
   * constructed it, since it borrows a pointer into thread-local storage.
   * For cross-thread lifetimes, pass an explicit Context.
   *
   * @param plan_cache_capacity max number of cuFFT plans to keep cached
   *   (silently clamped to at least 1). On overflow the least-recently-used
   *   plan is destroyed. */
  explicit FFT(std::size_t plan_cache_capacity = kDefaultCufftPlanCacheCapacity)
      : ctx_(&Context::threadLocal()), plans_(plan_cache_capacity > 0 ? plan_cache_capacity : 1) {}

  /** Construct an FFT bound to the given Context. The Context must outlive
   * this FFT instance; this object only borrows its stream and cuBLAS handle.
   *
   * @param plan_cache_capacity max number of cuFFT plans to keep cached
   *   (silently clamped to at least 1). On overflow the least-recently-used
   *   plan is destroyed. */
  explicit FFT(Context& ctx, std::size_t plan_cache_capacity = kDefaultCufftPlanCacheCapacity)
      : ctx_(&ctx), plans_(plan_cache_capacity > 0 ? plan_cache_capacity : 1) {}

  // Destructor is implicit: ~LruCache destroys each CufftPlan, which calls
  // cufftDestroy via CufftPlan's destructor.

  /** Capacity of the cuFFT plan cache (set at construction). */
  std::size_t plan_cache_capacity() const { return plans_.capacity(); }

  /** Current number of cached cuFFT plans. */
  std::size_t plan_cache_size() const { return plans_.size(); }

  FFT(const FFT&) = delete;
  FFT& operator=(const FFT&) = delete;

  /** Forward 1D C2C FFT. */
  template <typename Derived, std::enable_if_t<NumTraits<typename Derived::Scalar>::IsComplex>* = nullptr>
  ComplexVector fwd(const MatrixBase<Derived>& x) {
    const ComplexVector input(x.derived());
    const int n = static_cast<int>(input.size());
    if (n == 0) return ComplexVector(0);

    ensure_buffers(n * sizeof(Complex), n * sizeof(Complex));
    EIGEN_CUDA_RUNTIME_CHECK(
        cudaMemcpyAsync(d_in_.get(), input.data(), n * sizeof(Complex), cudaMemcpyHostToDevice, ctx_->stream()));

    cufftHandle plan = get_plan_1d(n, internal::cufft_c2c_type<Scalar>::value);
    EIGEN_CUFFT_CHECK(internal::cufftExecC2C_dispatch(plan, static_cast<Complex*>(d_in_.get()),
                                                      static_cast<Complex*>(d_out_.get()), CUFFT_FORWARD));

    ComplexVector result(n);
    EIGEN_CUDA_RUNTIME_CHECK(
        cudaMemcpyAsync(result.data(), d_out_.get(), n * sizeof(Complex), cudaMemcpyDeviceToHost, ctx_->stream()));
    EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(ctx_->stream()));
    return result;
  }

  /** Inverse 1D C2C FFT. Scaled by 1/n. */
  template <typename Derived>
  ComplexVector inv(const MatrixBase<Derived>& X) {
    static_assert(NumTraits<typename Derived::Scalar>::IsComplex, "inv() requires complex input");
    const ComplexVector input(X.derived());
    const int n = static_cast<int>(input.size());
    if (n == 0) return ComplexVector(0);

    ensure_buffers(n * sizeof(Complex), n * sizeof(Complex));
    EIGEN_CUDA_RUNTIME_CHECK(
        cudaMemcpyAsync(d_in_.get(), input.data(), n * sizeof(Complex), cudaMemcpyHostToDevice, ctx_->stream()));

    cufftHandle plan = get_plan_1d(n, internal::cufft_c2c_type<Scalar>::value);
    EIGEN_CUFFT_CHECK(internal::cufftExecC2C_dispatch(plan, static_cast<Complex*>(d_in_.get()),
                                                      static_cast<Complex*>(d_out_.get()), CUFFT_INVERSE));

    // Scale by 1/n.
    EIGEN_CUBLAS_CHECK(
        internal::cublasXscal(ctx_->cublasHandle(), n, Scalar(1) / Scalar(n), static_cast<Complex*>(d_out_.get()), 1));

    ComplexVector result(n);
    EIGEN_CUDA_RUNTIME_CHECK(
        cudaMemcpyAsync(result.data(), d_out_.get(), n * sizeof(Complex), cudaMemcpyDeviceToHost, ctx_->stream()));
    EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(ctx_->stream()));
    return result;
  }

  /** Forward 1D R2C FFT. Returns n/2+1 complex values (half-spectrum). */
  template <typename Derived, std::enable_if_t<!NumTraits<typename Derived::Scalar>::IsComplex>* = nullptr>
  ComplexVector fwd(const MatrixBase<Derived>& x) {
    const RealVector input(x.derived());
    const int n = static_cast<int>(input.size());
    if (n == 0) return ComplexVector(0);

    const int n_complex = n / 2 + 1;
    ensure_buffers(n * sizeof(Scalar), n_complex * sizeof(Complex));
    EIGEN_CUDA_RUNTIME_CHECK(
        cudaMemcpyAsync(d_in_.get(), input.data(), n * sizeof(Scalar), cudaMemcpyHostToDevice, ctx_->stream()));

    cufftHandle plan = get_plan_1d(n, internal::cufft_r2c_type<Scalar>::value);
    EIGEN_CUFFT_CHECK(
        internal::cufftExecR2C_dispatch(plan, static_cast<Scalar*>(d_in_.get()), static_cast<Complex*>(d_out_.get())));

    ComplexVector result(n_complex);
    EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpyAsync(result.data(), d_out_.get(), n_complex * sizeof(Complex),
                                             cudaMemcpyDeviceToHost, ctx_->stream()));
    EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(ctx_->stream()));
    return result;
  }

  /** Inverse 1D C2R FFT. Input is n/2+1 complex values, output is nfft real values.
   * Scaled by 1/nfft. Caller must specify nfft (original real signal length). */
  template <typename Derived>
  RealVector invReal(const MatrixBase<Derived>& X, Index nfft) {
    static_assert(NumTraits<typename Derived::Scalar>::IsComplex, "invReal() requires complex input");
    const ComplexVector input(X.derived());
    const int n = static_cast<int>(nfft);
    const int n_complex = n / 2 + 1;
    eigen_assert(input.size() == n_complex);
    if (n == 0) return RealVector(0);

    ensure_buffers(n_complex * sizeof(Complex), n * sizeof(Scalar));
    // cuFFT C2R may overwrite the input, so we copy to d_in_.
    EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpyAsync(d_in_.get(), input.data(), n_complex * sizeof(Complex),
                                             cudaMemcpyHostToDevice, ctx_->stream()));

    cufftHandle plan = get_plan_1d(n, internal::cufft_c2r_type<Scalar>::value);
    EIGEN_CUFFT_CHECK(
        internal::cufftExecC2R_dispatch(plan, static_cast<Complex*>(d_in_.get()), static_cast<Scalar*>(d_out_.get())));

    // Scale by 1/n.
    EIGEN_CUBLAS_CHECK(
        internal::cublasXscal(ctx_->cublasHandle(), n, Scalar(1) / Scalar(n), static_cast<Scalar*>(d_out_.get()), 1));

    RealVector result(n);
    EIGEN_CUDA_RUNTIME_CHECK(
        cudaMemcpyAsync(result.data(), d_out_.get(), n * sizeof(Scalar), cudaMemcpyDeviceToHost, ctx_->stream()));
    EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(ctx_->stream()));
    return result;
  }

  /** Forward 2D C2C FFT. Input and output are rows x cols complex matrices. */
  template <typename Derived>
  ComplexMatrix fwd2(const MatrixBase<Derived>& A) {
    static_assert(NumTraits<typename Derived::Scalar>::IsComplex, "fwd2() requires complex input");
    const ComplexMatrix input(A.derived());
    const int rows = static_cast<int>(input.rows());
    const int cols = static_cast<int>(input.cols());
    if (rows == 0 || cols == 0) return ComplexMatrix(rows, cols);

    const size_t total = static_cast<size_t>(rows) * static_cast<size_t>(cols) * sizeof(Complex);
    ensure_buffers(total, total);
    EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpyAsync(d_in_.get(), input.data(), total, cudaMemcpyHostToDevice, ctx_->stream()));

    cufftHandle plan = get_plan_2d(rows, cols, internal::cufft_c2c_type<Scalar>::value);
    EIGEN_CUFFT_CHECK(internal::cufftExecC2C_dispatch(plan, static_cast<Complex*>(d_in_.get()),
                                                      static_cast<Complex*>(d_out_.get()), CUFFT_FORWARD));

    ComplexMatrix result(rows, cols);
    EIGEN_CUDA_RUNTIME_CHECK(
        cudaMemcpyAsync(result.data(), d_out_.get(), total, cudaMemcpyDeviceToHost, ctx_->stream()));
    EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(ctx_->stream()));
    return result;
  }

  /** Inverse 2D C2C FFT. Scaled by 1/(rows*cols). */
  template <typename Derived>
  ComplexMatrix inv2(const MatrixBase<Derived>& A) {
    static_assert(NumTraits<typename Derived::Scalar>::IsComplex, "inv2() requires complex input");
    const ComplexMatrix input(A.derived());
    const int rows = static_cast<int>(input.rows());
    const int cols = static_cast<int>(input.cols());
    if (rows == 0 || cols == 0) return ComplexMatrix(rows, cols);

    const size_t total = static_cast<size_t>(rows) * static_cast<size_t>(cols) * sizeof(Complex);
    ensure_buffers(total, total);
    EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpyAsync(d_in_.get(), input.data(), total, cudaMemcpyHostToDevice, ctx_->stream()));

    cufftHandle plan = get_plan_2d(rows, cols, internal::cufft_c2c_type<Scalar>::value);
    EIGEN_CUFFT_CHECK(internal::cufftExecC2C_dispatch(plan, static_cast<Complex*>(d_in_.get()),
                                                      static_cast<Complex*>(d_out_.get()), CUFFT_INVERSE));

    // Scale by 1/(rows*cols).
    const int total_elems = rows * cols;
    EIGEN_CUBLAS_CHECK(internal::cublasXscal(ctx_->cublasHandle(), total_elems, Scalar(1) / Scalar(total_elems),
                                             static_cast<Complex*>(d_out_.get()), 1));

    ComplexMatrix result(rows, cols);
    EIGEN_CUDA_RUNTIME_CHECK(
        cudaMemcpyAsync(result.data(), d_out_.get(), total, cudaMemcpyDeviceToHost, ctx_->stream()));
    EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(ctx_->stream()));
    return result;
  }

  // DeviceMatrix in / DeviceMatrix out: no host transfer and no host
  // synchronization. Cross-stream safety follows the DeviceMatrix event
  // protocol (waitReady before reading, recordReady after enqueuing). 1D
  // overloads expect column vectors.

  /** Forward 1D C2C FFT, device-resident: d_X = fft(d_x). */
  void fwd(const DeviceMatrix<Complex>& d_x, DeviceMatrix<Complex>& d_X) {
    eigen_assert(d_x.cols() <= 1 && "device fwd(): expected a column vector");
    const int n = internal::to_blas_int(d_x.rows());
    prepare_out(d_x, d_X, n, 1);
    if (n == 0) return;
    cufftHandle plan = get_plan_1d(n, internal::cufft_c2c_type<Scalar>::value);
    EIGEN_CUFFT_CHECK(
        internal::cufftExecC2C_dispatch(plan, const_cast<Complex*>(d_x.data()), d_X.data(), CUFFT_FORWARD));
    d_X.recordReady(ctx_->stream());
  }

  /** Inverse 1D C2C FFT, device-resident. Scaled by 1/n. */
  void inv(const DeviceMatrix<Complex>& d_X, DeviceMatrix<Complex>& d_x) {
    eigen_assert(d_X.cols() <= 1 && "device inv(): expected a column vector");
    const int n = internal::to_blas_int(d_X.rows());
    prepare_out(d_X, d_x, n, 1);
    if (n == 0) return;
    cufftHandle plan = get_plan_1d(n, internal::cufft_c2c_type<Scalar>::value);
    EIGEN_CUFFT_CHECK(
        internal::cufftExecC2C_dispatch(plan, const_cast<Complex*>(d_X.data()), d_x.data(), CUFFT_INVERSE));
    EIGEN_CUBLAS_CHECK(internal::cublasXscal(ctx_->cublasHandle(), n, Scalar(1) / Scalar(n), d_x.data(), 1));
    d_x.recordReady(ctx_->stream());
  }

  /** Forward 1D R2C FFT, device-resident. Output is n/2+1 complex values. */
  void fwd(const DeviceMatrix<Scalar>& d_x, DeviceMatrix<Complex>& d_X) {
    eigen_assert(d_x.cols() <= 1 && "device fwd(): expected a column vector");
    const int n = internal::to_blas_int(d_x.rows());
    const int n_complex = n / 2 + 1;
    prepare_out(d_x, d_X, n == 0 ? 0 : n_complex, 1);
    if (n == 0) return;
    cufftHandle plan = get_plan_1d(n, internal::cufft_r2c_type<Scalar>::value);
    EIGEN_CUFFT_CHECK(internal::cufftExecR2C_dispatch(plan, const_cast<Scalar*>(d_x.data()), d_X.data()));
    d_X.recordReady(ctx_->stream());
  }

  /** Inverse 1D C2R FFT, device-resident. Input is nfft/2+1 complex values,
   * output nfft real values, scaled by 1/nfft. The input is preserved (cuFFT
   * C2R may overwrite its input, so it is staged through internal scratch). */
  void invReal(const DeviceMatrix<Complex>& d_X, DeviceMatrix<Scalar>& d_x, Index nfft) {
    eigen_assert(d_X.cols() <= 1 && "device invReal(): expected a column vector");
    const int n = internal::to_blas_int(nfft);
    const int n_complex = n / 2 + 1;
    eigen_assert(n == 0 || d_X.rows() == n_complex);
    prepare_out(d_X, d_x, n, 1);
    if (n == 0) return;
    // Stage through d_in_: cuFFT C2R may overwrite the input array.
    ensure_buffers(static_cast<size_t>(n_complex) * sizeof(Complex), 0);
    EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpyAsync(d_in_.get(), d_X.data(), n_complex * sizeof(Complex),
                                             cudaMemcpyDeviceToDevice, ctx_->stream()));
    cufftHandle plan = get_plan_1d(n, internal::cufft_c2r_type<Scalar>::value);
    EIGEN_CUFFT_CHECK(internal::cufftExecC2R_dispatch(plan, static_cast<Complex*>(d_in_.get()), d_x.data()));
    EIGEN_CUBLAS_CHECK(internal::cublasXscal(ctx_->cublasHandle(), n, Scalar(1) / Scalar(n), d_x.data(), 1));
    d_x.recordReady(ctx_->stream());
  }

  /** Forward 2D C2C FFT, device-resident. */
  void fwd2(const DeviceMatrix<Complex>& d_A, DeviceMatrix<Complex>& d_B) {
    const int rows = internal::to_blas_int(d_A.rows());
    const int cols = internal::to_blas_int(d_A.cols());
    prepare_out(d_A, d_B, rows, cols);
    if (rows == 0 || cols == 0) return;
    cufftHandle plan = get_plan_2d(rows, cols, internal::cufft_c2c_type<Scalar>::value);
    EIGEN_CUFFT_CHECK(
        internal::cufftExecC2C_dispatch(plan, const_cast<Complex*>(d_A.data()), d_B.data(), CUFFT_FORWARD));
    d_B.recordReady(ctx_->stream());
  }

  /** Inverse 2D C2C FFT, device-resident. Scaled by 1/(rows*cols). */
  void inv2(const DeviceMatrix<Complex>& d_A, DeviceMatrix<Complex>& d_B) {
    const int rows = internal::to_blas_int(d_A.rows());
    const int cols = internal::to_blas_int(d_A.cols());
    prepare_out(d_A, d_B, rows, cols);
    if (rows == 0 || cols == 0) return;
    cufftHandle plan = get_plan_2d(rows, cols, internal::cufft_c2c_type<Scalar>::value);
    EIGEN_CUFFT_CHECK(
        internal::cufftExecC2C_dispatch(plan, const_cast<Complex*>(d_A.data()), d_B.data(), CUFFT_INVERSE));
    const int total_elems = internal::to_blas_int(static_cast<int64_t>(rows) * static_cast<int64_t>(cols));
    EIGEN_CUBLAS_CHECK(
        internal::cublasXscal(ctx_->cublasHandle(), total_elems, Scalar(1) / Scalar(total_elems), d_B.data(), 1));
    d_B.recordReady(ctx_->stream());
  }

  /** The CUDA stream borrowed from the bound Context. */
  cudaStream_t stream() const { return ctx_->stream(); }

  /** The Context this FFT is bound to. */
  Context& context() const { return *ctx_; }

 private:
  Context* ctx_;
  Eigen::internal::LruCache<int64_t, internal::CufftPlan> plans_;
  internal::DeviceBuffer d_in_;
  internal::DeviceBuffer d_out_;
  size_t d_in_size_ = 0;
  size_t d_out_size_ = 0;

  // Common device-transform prologue: alias check, input/output event waits,
  // and (destructive) output resize.
  template <typename InScalar, typename OutScalar>
  void prepare_out(const DeviceMatrix<InScalar>& in, DeviceMatrix<OutScalar>& out, Index out_rows, Index out_cols) {
    eigen_assert(
        (in.data() == nullptr || static_cast<const void*>(in.data()) != static_cast<const void*>(out.data())) &&
        "device FFT: output must not alias input");
    in.waitReady(ctx_->stream());
    if (!out.empty()) out.waitReady(ctx_->stream());
    out.resize(out_rows, out_cols);
  }

  // Buffers grow but never shrink. The pre-realloc sync drains the *bound*
  // Context's stream — including unrelated GEMMs/solves/`device(ctx) = ...`
  // assignments queued on it — so callers running FFTs alongside other GPU
  // work on the same Context should size up front (call fwd/inv with the
  // largest expected n once) to avoid mid-pipeline stalls.
  void ensure_buffers(size_t in_bytes, size_t out_bytes) {
    if (in_bytes > d_in_size_) {
      if (d_in_) EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(ctx_->stream()));
      d_in_ = internal::DeviceBuffer(in_bytes);
      d_in_size_ = in_bytes;
    }
    if (out_bytes > d_out_size_) {
      if (d_out_) EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(ctx_->stream()));
      d_out_ = internal::DeviceBuffer(out_bytes);
      d_out_size_ = out_bytes;
    }
  }

  // Plan key encoding: rank (1 bit) | type (4 bits) | dims.
  // cufftType uses 7 bits; the top 3 (precision discriminator) are redundant
  // since Scalar fixes precision per FFT instance, so mask to 4 bits — without
  // it, e.g. plan_key_1d(5, C2C) and plan_key_1d(7, C2C) collide.
  static constexpr int64_t kTypeMask = 0xF;
  static constexpr int kCols2DBits = 30;  // bits 5..34
  static constexpr int kRows2DBits = 29;  // bits 35..63
  static int64_t plan_key_1d(int n, cufftType type) { return (int64_t(n) << 5) | (int64_t(type & kTypeMask) << 1) | 0; }

  static int64_t plan_key_2d(int rows, int cols, cufftType type) {
    eigen_assert(rows >= 0 && int64_t(rows) < (int64_t(1) << kRows2DBits) &&
                 "FFT plan rows exceed plan-key bit budget");
    eigen_assert(cols >= 0 && int64_t(cols) < (int64_t(1) << kCols2DBits) &&
                 "FFT plan cols exceed plan-key bit budget");
    return (int64_t(rows) << 35) | (int64_t(cols) << 5) | (int64_t(type & kTypeMask) << 1) | 1;
  }

  cufftHandle get_plan_1d(int n, cufftType type) {
    const int64_t key = plan_key_1d(n, type);
    if (internal::CufftPlan* hit = plans_.find(key)) return hit->get();

    cufftHandle plan;
    EIGEN_CUFFT_CHECK(cufftPlan1d(&plan, n, type, /*batch=*/1));
    EIGEN_CUFFT_CHECK(cufftSetStream(plan, ctx_->stream()));
    return plans_.insert(key, internal::CufftPlan(plan))->get();
  }

  cufftHandle get_plan_2d(int rows, int cols, cufftType type) {
    const int64_t key = plan_key_2d(rows, cols, type);
    if (internal::CufftPlan* hit = plans_.find(key)) return hit->get();

    // cuFFT uses row-major (C order) for 2D: first dim = rows, second = cols.
    // Eigen matrices are column-major, so we pass (cols, rows) to cuFFT
    // to get the correct 2D transform.
    cufftHandle plan;
    EIGEN_CUFFT_CHECK(cufftPlan2d(&plan, cols, rows, type));
    EIGEN_CUFFT_CHECK(cufftSetStream(plan, ctx_->stream()));
    return plans_.insert(key, internal::CufftPlan(plan))->get();
  }
};
}  // namespace gpu
}  // namespace Eigen

#endif  // EIGEN_GPU_FFT_H
