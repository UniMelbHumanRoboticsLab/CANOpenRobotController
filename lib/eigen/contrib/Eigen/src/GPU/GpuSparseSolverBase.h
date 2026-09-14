// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// Common base for the GPU sparse direct solvers (LLT, LDLT, LU) via cuDSS.
//
// All three share one analyzePattern → factorize → solve workflow and differ only
// in the cudssMatrixType_t and cudssMatrixViewType_t they pass to cuDSS, so this
// CRTP base implements the workflow and derived classes supply the two types.
//
// Not thread-safe: concurrent calls race on the cuDSS handle, the bound stream,
// and the cached scratch buffers. This includes concurrent solve() calls on one
// instance, const though solve() is. Use one solver per thread.

#ifndef EIGEN_GPU_SPARSE_SOLVER_BASE_H
#define EIGEN_GPU_SPARSE_SOLVER_BASE_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

#include "./CuSparseSupport.h"
#include "./CuDssSupport.h"

namespace Eigen {
namespace gpu {

// The named algorithms carry the cuDSS values, so apply_config() can
// static_cast them. cuDSS < 0.8 names none of them, so there they are not
// declared at all: selecting an algorithm the linked cuDSS cannot honor is a
// compile error rather than a runtime rejection, and EIGEN_HAS_CUDSS_SOLVER_CONFIG
// lets a caller branch on which case a build is in.
// Default is declared either way and is a sentinel rather than a cuDSS value:
// apply_config() skips its cudssConfigSet(), leaving the cuDSS default in place
// without depending on its numeric value.

/** Fill-reducing reordering algorithm, applied during analyzePattern().
 * Default lets cuDSS choose; Natural disables reordering. BtfColamd and
 * Colamd are valid for general (SparseLU) matrices only. The named
 * algorithms are declared only for cuDSS >= 0.8. */
enum class SparseReordering : int {
  Default = -1,
#if EIGEN_HAS_CUDSS_SOLVER_CONFIG
  BtfColamd = CUDSS_REORDERING_ALG_BTF_COLAMD,
  Colamd = CUDSS_REORDERING_ALG_COLAMD,
  Amd = CUDSS_REORDERING_ALG_AMD,
  NestedDissection = CUDSS_REORDERING_ALG_NESTED_DISSECTION,
  Natural = CUDSS_REORDERING_ALG_NONE,
#endif
};

/** Matching algorithm, applied during analyzePattern() to improve numerical
 * robustness. Off by cuDSS default; primarily useful for SparseLU on
 * indefinite or badly scaled systems. Auto lets cuDSS pick. The named
 * algorithms are declared only for cuDSS >= 0.8. */
enum class SparseMatching : int {
  Default = -1,
#if EIGEN_HAS_CUDSS_SOLVER_CONFIG
  None = CUDSS_MATCHING_ALG_NONE,
  MaxDiagCount = CUDSS_MATCHING_ALG_MAX_DIAG_COUNT,
  MaxMinDiag = CUDSS_MATCHING_ALG_MAX_MIN_DIAG,
  MaxMinDiagAlt = CUDSS_MATCHING_ALG_MAX_MIN_DIAG_ALT,
  MaxDiagSum = CUDSS_MATCHING_ALG_MAX_DIAG_SUM,
  MaxDiagProduct = CUDSS_MATCHING_ALG_MAX_DIAG_PRODUCT,
  Auto = CUDSS_MATCHING_ALG_AUTO,
#endif
};

/** Pivoting strategy, applied during factorize(). Default resolves per
 * matrix type. Validity of the other values depends on the matrix type and
 * reordering algorithm; see the cuDSS documentation for cudssPivotType_t.
 * The named strategies are declared only for cuDSS >= 0.8. */
enum class SparsePivoting : int {
  Default = -1,
#if EIGEN_HAS_CUDSS_SOLVER_CONFIG
  None = CUDSS_PIVOT_NONE,
  GlobalCol = CUDSS_PIVOT_GLOBAL_COL,
  GlobalRow = CUDSS_PIVOT_GLOBAL_ROW,
  Diagonal = CUDSS_PIVOT_DIAGONAL,
  LocalBlock = CUDSS_PIVOT_LOCAL_BLOCK,
#endif
};

/** Pass-through configuration for the cuDSS-backed sparse direct solvers
 * (SparseLLT, SparseLDLT, SparseLU). Fields left at their defaults keep the
 * corresponding cuDSS default, which is tuned for performance rather than
 * maximum robustness (e.g. matching is off). Non-default fields require
 * cuDSS >= 0.8, whose cudssReorderingAlg_t etc. name the algorithms with
 * stable values; earlier versions declare no algorithm enumerator and accept
 * only a default config, which EIGEN_HAS_CUDSS_SOLVER_CONFIG reports. */
struct SparseSolverConfig {
  SparseReordering reordering = SparseReordering::Default;
  SparseMatching matching = SparseMatching::Default;
  SparsePivoting pivoting = SparsePivoting::Default;
  /** Pivot admissibility threshold in [0, 1]; negative keeps the cuDSS default. */
  double pivotThreshold = -1.0;
  /** Perturbation applied to near-zero pivots; negative keeps the cuDSS default. */
  double pivotEpsilon = -1.0;
  /** Iterative-refinement steps during solve(); negative keeps the cuDSS default (0). */
  int refinementSteps = -1;
  /** Iterative-refinement stopping tolerance; negative keeps the cuDSS default. */
  double refinementTolerance = -1.0;
  /** Let factor data spill to host memory when device memory is insufficient.
   * Changes what analyzePattern() builds — set it before that phase. */
  bool hybridMemory = false;
  /** Device-memory budget in bytes for hybridMemory; negative keeps the cuDSS
   * default heuristic, which is what cuDSS itself spells as -1. Zero is a
   * budget of its own and is passed through. */
  int64_t hybridMemoryDeviceLimit = -1;
  /** Split factorization/solve work between host and device. cuDSS requires
   * this before analysis, which it changes — set it before analyzePattern(). */
  bool hybridExecute = false;

  /** True when every field keeps the cuDSS default. */
  bool isDefault() const {
    return reordering == SparseReordering::Default && matching == SparseMatching::Default &&
           pivoting == SparsePivoting::Default && pivotThreshold < 0 && pivotEpsilon < 0 && refinementSteps < 0 &&
           refinementTolerance < 0 && !hybridMemory && hybridMemoryDeviceLimit < 0 && !hybridExecute;
  }
};

namespace internal {

/** CRTP base for GPU sparse direct solvers.
 *
 * \tparam Scalar_  Element type (passed explicitly to avoid incomplete-type issues with CRTP).
 * \tparam Derived  The concrete solver class (SparseLLT, SparseLDLT, SparseLU).
 *                  Must provide:
 *                  - `static constexpr cudssMatrixType_t cudss_matrix_type()`
 *                  - `static constexpr cudssMatrixViewType_t cudss_matrix_view()`
 *                  - `static constexpr bool needs_csr_conversion()`
 */
template <typename Scalar_, typename Derived>
class SparseSolverBase {
 public:
  using Scalar = Scalar_;
  using RealScalar = typename NumTraits<Scalar>::Real;
  using StorageIndex = int;
  using SpMat = SparseMatrix<Scalar, ColMajor, StorageIndex>;
  using CsrMat = SparseMatrix<Scalar, RowMajor, StorageIndex>;
  using DenseVector = Matrix<Scalar, Dynamic, 1>;
  using DenseMatrix = Matrix<Scalar, Dynamic, Dynamic, ColMajor>;

  SparseSolverBase() { init_context(/*stream=*/nullptr, /*borrow_stream=*/false); }

  /** Borrow \p ctx's stream: solver work runs on the same stream as the
   * caller's other GPU operations (device-resident solves chain with SpMV /
   * cuBLAS work without cross-stream event waits). The cuDSS handle itself is
   * always owned by this solver. \p ctx must outlive this object. */
  explicit SparseSolverBase(Context& ctx) { init_context(ctx.stream(), /*borrow_stream=*/true); }

  ~SparseSolverBase() {
    destroy_cudss_objects();
    if (handle_) (void)cudssDestroy(handle_);
    if (owns_stream_ && stream_) (void)cudaStreamDestroy(stream_);
  }

  SparseSolverBase(const SparseSolverBase&) = delete;
  SparseSolverBase& operator=(const SparseSolverBase&) = delete;

  /** Apply \p cfg to this solver. Most knobs are consumed by the phase they
   * affect — reordering and matching by analyzePattern(), pivoting by
   * factorize(), refinement by solve() — so call setConfig() before the
   * first phase whose behavior it changes; phases already executed are
   * unaffected.
   *
   * The two hybrid *modes* are the exception. cuDSS builds different analysis
   * state for them, and documents that analysis has to be redone when hybrid
   * memory is enabled afterwards, so hybridMemory and hybridExecute must be
   * set before analyzePattern(). Changing either one after analysis
   * invalidates it: analyzePattern() has to be called again before
   * factorize(), which would otherwise consume analysis state built for a
   * different execution and memory mode. hybridMemoryDeviceLimit is only a
   * budget within hybridMemory and may be changed between factorizations.
   *
   * Non-default fields require cuDSS >= 0.8, which EIGEN_HAS_CUDSS_SOLVER_CONFIG
   * reports. Below that, the algorithm enumerators are not declared, so
   * selecting one does not compile; a non-default value of the remaining
   * fields is rejected rather than applied, asserting and leaving info()
   * reporting InvalidInput until the config is reset to default. */
  Derived& setConfig(const SparseSolverConfig& cfg) {
#if !EIGEN_HAS_CUDSS_SOLVER_CONFIG
    eigen_assert(cfg.isDefault() && "SparseSolverConfig knobs require cuDSS >= 0.8");
    if (!cfg.isDefault()) {
      // The assert above is compiled out in release builds. Record what was
      // asked for and fail through info() rather than factorizing with the
      // cuDSS defaults this call meant to replace. analyzePattern() keeps
      // failing while the stored config is one this cuDSS cannot honor.
      config_opts_ = cfg;
      analysis_done_ = false;
      info_ = InvalidInput;
      return derived();
    }
#endif

    const bool hybrid_mode_changed =
        cfg.hybridMemory != config_opts_.hybridMemory || cfg.hybridExecute != config_opts_.hybridExecute;
    eigen_assert((!hybrid_mode_changed || !analysis_done_) &&
                 "hybridMemory/hybridExecute change what analyzePattern() builds, so they must be set before it");

    config_opts_ = cfg;
    apply_config();

    if (hybrid_mode_changed && analysis_done_) {
      // The assert above is compiled out in release builds. Fail through
      // info() rather than factorizing against the wrong analysis.
      analysis_done_ = false;
      info_ = InvalidInput;
    }
    return derived();
  }

  /** The configuration most recently passed to setConfig(). */
  const SparseSolverConfig& config() const { return config_opts_; }

  /** Symbolic analysis + numeric factorization. */
  template <typename InputType>
  Derived& compute(const SparseMatrixBase<InputType>& A) {
    analyzePattern(A);
    if (info_ == Success) {
      factorize(A);
    }
    return derived();
  }

  /** Symbolic analysis only. Uploads sparsity structure to device.
   * This phase is synchronous (blocks until complete). */
  template <typename InputType>
  Derived& analyzePattern(const SparseMatrixBase<InputType>& A) {
#if !EIGEN_HAS_CUDSS_SOLVER_CONFIG
    if (!config_opts_.isDefault()) {
      // setConfig() refused this config; analyzing as if it had been applied
      // would hand factorize() state built for settings the caller replaced.
      info_ = InvalidInput;
      analysis_done_ = false;
      return derived();
    }
#endif

    const InputType& input = A.derived();
    check_storage_index_bounds<StorageIndex>(input.rows(), input.cols(), input.nonZeros());
    eigen_assert(input.rows() == input.cols() && "GpuSparseSolver requires a square matrix");

    n_ = input.rows();
    info_ = InvalidInput;
    analysis_done_ = false;

    if (n_ == 0) {
      nnz_ = 0;
      info_ = Success;
      analysis_done_ = true;
      return derived();
    }

    // For symmetric solvers, ColMajor CSC can be reinterpreted as CSR with
    // swapped triangle view — zero copy, except that Hermitian matrix types
    // need the value array conjugated (see csc_upload_values). For general
    // solvers, we must convert to actual RowMajor CSR so cuDSS sees the
    // correct matrix, not A^T.
    if (Derived::needs_csr_conversion()) {
      const CsrMat csr(input);
      nnz_ = csr.nonZeros();
      upload_compressed(csr.outerIndexPtr(), csr.innerIndexPtr(), csr.valuePtr());
    } else {
      // Bind by reference when the input is already a compressed CSC matrix —
      // no host copy.
      SpMat storage;
      const SpMat& csc = bind_sparse<SpMat>(input, storage);
      nnz_ = csc.nonZeros();
      upload_compressed(csc.outerIndexPtr(), csc.innerIndexPtr(), csc_upload_values(csc.valuePtr(), nnz_));
    }
    create_cudss_matrix();

    if (data_) EIGEN_CUDSS_CHECK(cudssDataDestroy(handle_, data_));
    EIGEN_CUDSS_CHECK(cudssDataCreate(handle_, &data_));

    create_placeholder_dense();

    EIGEN_CUDSS_CHECK(cudssExecute(handle_, CUDSS_PHASE_ANALYSIS, config_, data_, d_A_cudss_, d_x_cudss_, d_b_cudss_));

    analysis_done_ = true;
    info_ = Success;
    return derived();
  }

  /** Numeric factorization using the symbolic analysis from analyzePattern.
   *
   * \warning The sparsity pattern (outerIndexPtr, innerIndexPtr) must be
   * identical to the one passed to analyzePattern(). Only the numerical
   * values may change. Passing a different pattern is undefined behavior.
   * This matches the contract of CHOLMOD, UMFPACK, and cuDSS's own API.
   *
   * This phase is asynchronous — info() lazily synchronizes. */
  template <typename InputType>
  Derived& factorize(const SparseMatrixBase<InputType>& A) {
    eigen_assert(analysis_done_ && "factorize() requires analyzePattern() first");
    if (!analysis_done_) {
      // Reachable in release builds whenever setConfig() invalidated the
      // analysis rather than applying the request; see setConfig().
      info_ = InvalidInput;
      return derived();
    }

    if (n_ == 0) {
      info_ = Success;
      return derived();
    }

    // Convert to the same format used in analyzePattern. When the input is
    // already a compressed CSC matrix (symmetric solvers), it is bound by
    // reference — no host copy or conversion on refactorize (Hermitian types
    // re-conjugate the value array; the pattern stays zero-copy).
    // The temporaries must outlive the async memcpy (pageable H2D is actually
    // synchronous w.r.t. the host, but keep them alive for clarity).
    const InputType& input = A.derived();
    check_storage_index_bounds<StorageIndex>(input.rows(), input.cols(), input.nonZeros());
    eigen_assert(input.rows() == n_ && input.cols() == n_);

    const Scalar* value_ptr;
    Index value_nnz;
    CsrMat csr_tmp;
    SpMat csc_storage;
    if (Derived::needs_csr_conversion()) {
      csr_tmp = CsrMat(input);
      value_ptr = csr_tmp.valuePtr();
      value_nnz = csr_tmp.nonZeros();
    } else {
      const SpMat& csc = bind_sparse<SpMat>(input, csc_storage);
      value_ptr = csc_upload_values(csc.valuePtr(), csc.nonZeros());
      value_nnz = csc.nonZeros();
    }
    eigen_assert(value_nnz == nnz_);
    EIGEN_UNUSED_VARIABLE(value_nnz);

    EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpyAsync(d_values_.get(), value_ptr, static_cast<size_t>(nnz_) * sizeof(Scalar),
                                             cudaMemcpyHostToDevice, stream_));

    EIGEN_CUDSS_CHECK(cudssMatrixSetValues(d_A_cudss_, d_values_.get()));

    info_ = InvalidInput;
    info_synced_ = false;
    EIGEN_CUDSS_CHECK(
        cudssExecute(handle_, CUDSS_PHASE_FACTORIZATION, config_, data_, d_A_cudss_, d_x_cudss_, d_b_cudss_));

    return derived();
  }

  /** Solve A * X = B (host → host). Returns X as a dense matrix.
   * Supports single or multiple right-hand sides. */
  template <typename Rhs>
  DenseMatrix solve(const MatrixBase<Rhs>& B) const {
    // Debug builds verify the factorization (info() synchronizes on the first
    // call after factorize()); release builds skip both the check and the
    // sync — use info() explicitly when failure must be detected.
    eigen_assert(info() == Success && "GpuSparseSolver::solve requires a successful factorization");
    eigen_assert(B.rows() == n_);

    if (n_ == 0) return DenseMatrix(0, B.cols());

    const Ref<const DenseMatrix> rhs(B.derived());
    const int64_t nrhs = static_cast<int64_t>(rhs.cols());

    // Reuse cached d_b/d_x scratch to avoid cudaMalloc/cudaFree per solve.
    const size_t rhs_bytes = static_cast<size_t>(n_) * static_cast<size_t>(nrhs) * sizeof(Scalar);
    ensure_solve_buffer(d_b_solve_, rhs_bytes);
    ensure_solve_buffer(d_x_solve_, rhs_bytes);
    internal::upload_host_matrix(static_cast<Scalar*>(d_b_solve_.get()), n_, rhs.data(), rhs.outerStride(), n_, nrhs,
                                 stream_);

    update_solve_descriptors(nrhs, d_b_solve_.get(), d_x_solve_.get());
    EIGEN_CUDSS_CHECK(
        cudssExecute(handle_, CUDSS_PHASE_SOLVE, config_, data_, d_A_cudss_, x_solve_cudss_, b_solve_cudss_));

    DenseMatrix X(n_, rhs.cols());
    EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpyAsync(X.data(), d_x_solve_.get(), rhs_bytes, cudaMemcpyDeviceToHost, stream_));
    EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(stream_));

    return X;
  }

  /** Solve A * X = B with device-resident RHS. Returns an n × nrhs
   * DeviceMatrix that stays on device — no H2D/D2H transfer and no host
   * synchronization (debug builds verify the factorization status first,
   * which syncs once after each factorize()). Chain the result directly into
   * SpMV / cuBLAS work, e.g. for iterative refinement. */
  DeviceMatrix<Scalar> solve(const DeviceMatrix<Scalar>& d_B) const {
    eigen_assert(info() == Success && "GpuSparseSolver::solve requires a successful factorization");
    eigen_assert(d_B.rows() == n_);

    const int64_t nrhs = static_cast<int64_t>(d_B.cols());
    DeviceMatrix<Scalar> X(n_, d_B.cols());
    if (n_ == 0 || nrhs == 0) return X;

    d_B.waitReady(stream_);
    update_solve_descriptors(nrhs, const_cast<Scalar*>(d_B.data()), X.data());
    EIGEN_CUDSS_CHECK(
        cudssExecute(handle_, CUDSS_PHASE_SOLVE, config_, data_, d_A_cudss_, x_solve_cudss_, b_solve_cudss_));
    X.recordReady(stream_);
    return X;
  }

  ComputationInfo info() const {
    sync_info();
    return info_;
  }
  Index rows() const { return n_; }
  Index cols() const { return n_; }

  cudaStream_t stream() const { return stream_; }

 protected:
  cudaStream_t stream_ = nullptr;
  bool owns_stream_ = true;
  cudssHandle_t handle_ = nullptr;
  cudssConfig_t config_ = nullptr;
  cudssData_t data_ = nullptr;
  cudssMatrix_t d_A_cudss_ = nullptr;
  cudssMatrix_t d_x_cudss_ = nullptr;
  cudssMatrix_t d_b_cudss_ = nullptr;

  DeviceBuffer d_rowPtr_;
  DeviceBuffer d_colIdx_;
  DeviceBuffer d_values_;

  // Host staging for the conjugated value array (Hermitian zero-copy path
  // only; see csc_upload_values). Kept as a member so it outlives the async
  // H2D copy.
  DenseVector conj_values_;

  mutable DeviceBuffer d_b_solve_;
  mutable DeviceBuffer d_x_solve_;

  // Cached cuDSS dense descriptors for solve, re-pointed per call and
  // recreated only when nrhs changes.
  mutable cudssMatrix_t b_solve_cudss_ = nullptr;
  mutable cudssMatrix_t x_solve_cudss_ = nullptr;
  mutable int64_t solve_desc_nrhs_ = -1;

  int64_t n_ = 0;
  int64_t nnz_ = 0;
  mutable ComputationInfo info_ = InvalidInput;
  mutable bool info_synced_ = true;
  bool analysis_done_ = false;
  SparseSolverConfig config_opts_;

 private:
  Derived& derived() { return static_cast<Derived&>(*this); }
  const Derived& derived() const { return static_cast<const Derived&>(*this); }

  void init_context(cudaStream_t stream, bool borrow_stream) {
    if (borrow_stream) {
      // nullptr is CUDA's valid legacy default stream, so ownership cannot be
      // inferred from the stream value.
      stream_ = stream;
      owns_stream_ = false;
    } else {
      EIGEN_CUDA_RUNTIME_CHECK(cudaStreamCreate(&stream_));
      owns_stream_ = true;
    }
    EIGEN_CUDSS_CHECK(cudssCreate(&handle_));
    EIGEN_CUDSS_CHECK(cudssSetStream(handle_, stream_));
    EIGEN_CUDSS_CHECK(cudssConfigCreate(&config_));
  }

  // Rebuild config_ from config_opts_. Recreating rather than mutating lets a
  // field reset to Default restore the cuDSS default, whose numeric value is
  // not part of the cuDSS API contract.
  void apply_config() {
#if EIGEN_HAS_CUDSS_SOLVER_CONFIG
    if (config_) {
      // cuDSS reads the config during execution, and both factorize() and the
      // device-resident solve() return with their phase still queued. Retire
      // that work before the config it is reading goes away.
      EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(stream_));
      (void)cudssConfigDestroy(config_);
    }
    config_ = nullptr;
    EIGEN_CUDSS_CHECK(cudssConfigCreate(&config_));
    const SparseSolverConfig& c = config_opts_;
    if (c.reordering != SparseReordering::Default) {
      const cudssReorderingAlg_t v = static_cast<cudssReorderingAlg_t>(c.reordering);
      EIGEN_CUDSS_CHECK(cudssConfigSet(config_, CUDSS_CONFIG_REORDERING_ALG, &v, sizeof(v)));
    }
    if (c.matching != SparseMatching::Default) {
      const cudssMatchingAlg_t v = static_cast<cudssMatchingAlg_t>(c.matching);
      EIGEN_CUDSS_CHECK(cudssConfigSet(config_, CUDSS_CONFIG_MATCHING_ALG, &v, sizeof(v)));
    }
    if (c.pivoting != SparsePivoting::Default) {
      const cudssPivotType_t v = static_cast<cudssPivotType_t>(c.pivoting);
      EIGEN_CUDSS_CHECK(cudssConfigSet(config_, CUDSS_CONFIG_PIVOT_TYPE, &v, sizeof(v)));
    }
    if (c.pivotThreshold >= 0) {
      EIGEN_CUDSS_CHECK(
          cudssConfigSet(config_, CUDSS_CONFIG_PIVOT_THRESHOLD, &c.pivotThreshold, sizeof(c.pivotThreshold)));
    }
    if (c.pivotEpsilon >= 0) {
      EIGEN_CUDSS_CHECK(cudssConfigSet(config_, CUDSS_CONFIG_PIVOT_EPSILON, &c.pivotEpsilon, sizeof(c.pivotEpsilon)));
    }
    if (c.refinementSteps >= 0) {
      EIGEN_CUDSS_CHECK(
          cudssConfigSet(config_, CUDSS_CONFIG_IR_N_STEPS, &c.refinementSteps, sizeof(c.refinementSteps)));
    }
    if (c.refinementTolerance >= 0) {
      EIGEN_CUDSS_CHECK(
          cudssConfigSet(config_, CUDSS_CONFIG_IR_TOL, &c.refinementTolerance, sizeof(c.refinementTolerance)));
    }
    if (c.hybridMemory) {
      const int v = 1;
      EIGEN_CUDSS_CHECK(cudssConfigSet(config_, CUDSS_CONFIG_HYBRID_MEMORY_MODE, &v, sizeof(v)));
      if (c.hybridMemoryDeviceLimit >= 0) {
        const int64_t limit = c.hybridMemoryDeviceLimit;
        EIGEN_CUDSS_CHECK(cudssConfigSet(config_, CUDSS_CONFIG_HYBRID_DEVICE_MEMORY_LIMIT, &limit, sizeof(limit)));
      }
    }
    if (c.hybridExecute) {
      const int v = 1;
      EIGEN_CUDSS_CHECK(cudssConfigSet(config_, CUDSS_CONFIG_HYBRID_EXECUTE_MODE, &v, sizeof(v)));
    }
#else
    // setConfig() returns early for anything else, so there is nothing to apply.
    eigen_internal_assert(config_opts_.isDefault());
#endif
  }

  void ensure_solve_buffer(DeviceBuffer& buf, size_t needed) const {
    if (needed > buf.size()) {
      if (buf) EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(stream_));
      buf = DeviceBuffer(needed);
    }
  }

  // Recreate the solve descriptors when nrhs changes; otherwise just re-point
  // them (cudssMatrixSetValues is a host-side pointer update).
  void update_solve_descriptors(int64_t nrhs, void* b_ptr, void* x_ptr) const {
    constexpr cudss_value_type_t dtype = to_cudss_data_type(cuda_data_type<Scalar>::value);
    if (!b_solve_cudss_ || solve_desc_nrhs_ != nrhs) {
      destroy_solve_descriptors();
      EIGEN_CUDSS_CHECK(cudssMatrixCreateDn(&b_solve_cudss_, n_, nrhs, n_, b_ptr, dtype, CUDSS_LAYOUT_COL_MAJOR));
      EIGEN_CUDSS_CHECK(cudssMatrixCreateDn(&x_solve_cudss_, n_, nrhs, n_, x_ptr, dtype, CUDSS_LAYOUT_COL_MAJOR));
      solve_desc_nrhs_ = nrhs;
    } else {
      EIGEN_CUDSS_CHECK(cudssMatrixSetValues(b_solve_cudss_, b_ptr));
      EIGEN_CUDSS_CHECK(cudssMatrixSetValues(x_solve_cudss_, x_ptr));
    }
  }

  void destroy_solve_descriptors() const {
    if (b_solve_cudss_) {
      (void)cudssMatrixDestroy(b_solve_cudss_);
      b_solve_cudss_ = nullptr;
    }
    if (x_solve_cudss_) {
      (void)cudssMatrixDestroy(x_solve_cudss_);
      x_solve_cudss_ = nullptr;
    }
    solve_desc_nrhs_ = -1;
  }

  void sync_info() const {
    if (!info_synced_) {
      // cudssDataGet for CUDSS_DATA_INFO synchronizes the stream internally,
      // so an explicit cudaStreamSynchronize would be redundant.
      int cudss_info = 0;
      EIGEN_CUDSS_CHECK(cudssDataGet(handle_, data_, CUDSS_DATA_INFO, &cudss_info, sizeof(cudss_info), nullptr));
      info_ = (cudss_info == 0) ? Success : NumericalIssue;
      info_synced_ = true;
    }
  }

  // Destructor-only cleanup: there is no useful recovery path for failures.
  void destroy_cudss_objects() {
    // Same lifetime rule as apply_config(): the queued phase reads the data
    // handle and the matrix descriptors as well as the config. Unchecked
    // because this runs from the destructor.
    (void)cudaStreamSynchronize(stream_);
    destroy_solve_descriptors();
    if (d_A_cudss_) {
      (void)cudssMatrixDestroy(d_A_cudss_);
      d_A_cudss_ = nullptr;
    }
    if (d_x_cudss_) {
      (void)cudssMatrixDestroy(d_x_cudss_);
      d_x_cudss_ = nullptr;
    }
    if (d_b_cudss_) {
      (void)cudssMatrixDestroy(d_b_cudss_);
      d_b_cudss_ = nullptr;
    }
    if (data_) {
      (void)cudssDataDestroy(handle_, data_);
      data_ = nullptr;
    }
    if (config_) {
      (void)cudssConfigDestroy(config_);
      config_ = nullptr;
    }
  }

  // The zero-copy CSC-as-CSR reinterpretation hands cuDSS the stored triangle
  // of A^T. Real symmetric types are unaffected (A^T = A), but the Hermitian
  // types need the triangle of A, whose entries are the conjugates of A^T's.
  static constexpr bool needs_value_conjugation() {
    return Derived::cudss_matrix_type() == CUDSS_MTYPE_HPD || Derived::cudss_matrix_type() == CUDSS_MTYPE_HERMITIAN;
  }

  // Value array for the zero-copy CSC-as-CSR path: conjugated into
  // conj_values_ for Hermitian matrix types, passed through otherwise. The
  // pattern arrays remain zero-copy either way.
  const Scalar* csc_upload_values(const Scalar* values, Index nnz) {
    if (!needs_value_conjugation()) return values;
    conj_values_ = Map<const DenseVector>(values, nnz).conjugate();
    return conj_values_.data();
  }

  void upload_compressed(const StorageIndex* outer, const StorageIndex* inner, const Scalar* values) {
    const size_t rowptr_bytes = static_cast<size_t>(n_ + 1) * sizeof(StorageIndex);
    const size_t colidx_bytes = static_cast<size_t>(nnz_) * sizeof(StorageIndex);
    const size_t values_bytes = static_cast<size_t>(nnz_) * sizeof(Scalar);

    d_rowPtr_ = DeviceBuffer(rowptr_bytes);
    d_colIdx_ = DeviceBuffer(colidx_bytes);
    d_values_ = DeviceBuffer(values_bytes);

    EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpyAsync(d_rowPtr_.get(), outer, rowptr_bytes, cudaMemcpyHostToDevice, stream_));
    EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpyAsync(d_colIdx_.get(), inner, colidx_bytes, cudaMemcpyHostToDevice, stream_));
    EIGEN_CUDA_RUNTIME_CHECK(cudaMemcpyAsync(d_values_.get(), values, values_bytes, cudaMemcpyHostToDevice, stream_));
  }

  void create_cudss_matrix() {
    if (d_A_cudss_) EIGEN_CUDSS_CHECK(cudssMatrixDestroy(d_A_cudss_));

    constexpr cudss_value_type_t idx_type = to_cudss_data_type(cudss_index_type<StorageIndex>::value);
    constexpr cudss_value_type_t val_type = to_cudss_data_type(cuda_data_type<Scalar>::value);
    constexpr cudssMatrixType_t mtype = Derived::cudss_matrix_type();
    constexpr cudssMatrixViewType_t mview = Derived::cudss_matrix_view();

#if defined(CUDSS_VERSION) && CUDSS_VERSION >= 800
    // cuDSS 0.8 split the index type into separate offset/index type params.
    EIGEN_CUDSS_CHECK(cudssMatrixCreateCsr(&d_A_cudss_, n_, n_, nnz_, d_rowPtr_.get(),
                                           /*rowEnd=*/nullptr, d_colIdx_.get(), d_values_.get(), idx_type, idx_type,
                                           val_type, mtype, mview, CUDSS_BASE_ZERO));
#else
    EIGEN_CUDSS_CHECK(cudssMatrixCreateCsr(&d_A_cudss_, n_, n_, nnz_, d_rowPtr_.get(),
                                           /*rowEnd=*/nullptr, d_colIdx_.get(), d_values_.get(), idx_type, val_type,
                                           mtype, mview, CUDSS_BASE_ZERO));
#endif
  }

  void create_placeholder_dense() {
    // A new analysis may change n_, so the cached solve descriptors are stale.
    destroy_solve_descriptors();
    if (d_x_cudss_) EIGEN_CUDSS_CHECK(cudssMatrixDestroy(d_x_cudss_));
    if (d_b_cudss_) EIGEN_CUDSS_CHECK(cudssMatrixDestroy(d_b_cudss_));
    constexpr cudss_value_type_t dtype = to_cudss_data_type(cuda_data_type<Scalar>::value);
    EIGEN_CUDSS_CHECK(cudssMatrixCreateDn(&d_x_cudss_, n_, 1, n_, nullptr, dtype, CUDSS_LAYOUT_COL_MAJOR));
    EIGEN_CUDSS_CHECK(cudssMatrixCreateDn(&d_b_cudss_, n_, 1, n_, nullptr, dtype, CUDSS_LAYOUT_COL_MAJOR));
  }
};
}  // namespace internal
}  // namespace gpu
}  // namespace Eigen

#endif  // EIGEN_GPU_SPARSE_SOLVER_BASE_H
