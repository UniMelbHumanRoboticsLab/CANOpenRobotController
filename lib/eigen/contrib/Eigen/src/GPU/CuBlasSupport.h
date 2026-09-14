// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// cuBLAS-specific support types. Generic CUDA runtime utilities (DeviceBuffer,
// cuda_data_type) live in GpuSupport.h.

#ifndef EIGEN_GPU_CUBLAS_SUPPORT_H
#define EIGEN_GPU_CUBLAS_SUPPORT_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

#include "./GpuSupport.h"
#include <cublas_v2.h>
#include <cublasLt.h>
#include <cstring>
#include <utility>

namespace Eigen {
namespace gpu {
namespace internal {

#define EIGEN_CUBLAS_CHECK(expr)                                       \
  do {                                                                 \
    cublasStatus_t _s = (expr);                                        \
    eigen_assert(_s == CUBLAS_STATUS_SUCCESS && "cuBLAS call failed"); \
  } while (0)

constexpr cublasOperation_t to_cublas_op(GpuOp op) {
  switch (op) {
    case GpuOp::Trans:
      return CUBLAS_OP_T;
    case GpuOp::ConjTrans:
      return CUBLAS_OP_C;
    default:
      return CUBLAS_OP_N;
  }
}

// cuBLAS 12.0 added `_64` entry points taking int64_t dimensions across the
// whole wrapped surface, so one wrapper body serves both:
// EIGEN_CUBLAS_FN(cublasSgemm)(..., to_blas_dim(m), ...) becomes
// cublasSgemm_64(..., m, ...) there (the plain `_64` names are cublas_v2.h
// aliases for the real `_v2_64` symbols) and cublasSgemm(..., to_blas_int(m),
// ...) on older cuBLAS. to_blas_dim is separate from to_blas_int, which must
// keep narrowing for the cuSOLVER and cuFFT sites that have no `_64` API.

#if defined(CUBLAS_VERSION) && CUBLAS_VERSION >= 120000
#define EIGEN_CUBLAS_FN(name) name##_64
inline int64_t to_blas_dim(int64_t v) { return v; }
#else
#define EIGEN_CUBLAS_FN(name) name
inline int to_blas_dim(int64_t v) { return to_blas_int(v); }
#endif

// RAII cuBLAS / cuBLASLt handles; the ownership flag supports handles borrowed from a gpu::Context.
struct CublasHandleDeleter {
  bool owns = true;
  void operator()(cublasHandle_t h) const noexcept {
    if (owns && h) (void)cublasDestroy(h);
  }
};
using UniqueCublasHandle = std::unique_ptr<std::remove_pointer_t<cublasHandle_t>, CublasHandleDeleter>;

struct CublasLtHandleDeleter {
  void operator()(cublasLtHandle_t h) const noexcept {
    if (h) (void)cublasLtDestroy(h);
  }
};
using UniqueCublasLtHandle = std::unique_ptr<std::remove_pointer_t<cublasLtHandle_t>, CublasLtHandleDeleter>;

// cublasLtMatmul takes a compute type separate from the data type, which selects
// the precision policy:
//   - Default: tensor-core algorithms via the cublasLtMatmul heuristics. For
//     double, cuBLAS may use Ozaki emulation on sm_80+ tensor cores.
//   - EIGEN_CUDA_TF32: TF32 for float (~2x faster, 10-bit mantissa).
//   - EIGEN_NO_CUDA_TENSOR_OPS: pedantic compute types, no tensor cores, for
//     bit-exact reproducibility.
// Real and complex of the same precision share a compute type, so the
// specializations below just dispatch to a precision tag.
namespace cuda_compute_type_detail {
#if defined(EIGEN_NO_CUDA_TENSOR_OPS)
constexpr cublasComputeType_t kFloat = CUBLAS_COMPUTE_32F_PEDANTIC;
constexpr cublasComputeType_t kDouble = CUBLAS_COMPUTE_64F_PEDANTIC;
#elif defined(EIGEN_CUDA_TF32)
constexpr cublasComputeType_t kFloat = CUBLAS_COMPUTE_32F_FAST_TF32;
constexpr cublasComputeType_t kDouble = CUBLAS_COMPUTE_64F;
#else
constexpr cublasComputeType_t kFloat = CUBLAS_COMPUTE_32F;
constexpr cublasComputeType_t kDouble = CUBLAS_COMPUTE_64F;
#endif
}  // namespace cuda_compute_type_detail

template <typename Scalar>
struct cuda_compute_type;

template <>
struct cuda_compute_type<float> {
  static constexpr cublasComputeType_t value = cuda_compute_type_detail::kFloat;
};
template <>
struct cuda_compute_type<double> {
  static constexpr cublasComputeType_t value = cuda_compute_type_detail::kDouble;
};
template <>
struct cuda_compute_type<std::complex<float>> {
  static constexpr cublasComputeType_t value = cuda_compute_type_detail::kFloat;
};
template <>
struct cuda_compute_type<std::complex<double>> {
  static constexpr cublasComputeType_t value = cuda_compute_type_detail::kDouble;
};

#define EIGEN_CUBLASLT_CHECK(expr)                                       \
  do {                                                                   \
    cublasStatus_t _s = (expr);                                          \
    eigen_assert(_s == CUBLAS_STATUS_SUCCESS && "cuBLASLt call failed"); \
  } while (0)

// Maximum workspace the heuristic is allowed to consider. This is a preference
// ceiling, not an allocation — actual allocation matches the selected algorithm.
// Override at compile time via EIGEN_CUDA_CUBLASLT_MAX_WORKSPACE_BYTES.
#ifndef EIGEN_CUDA_CUBLASLT_MAX_WORKSPACE_BYTES
#define EIGEN_CUDA_CUBLASLT_MAX_WORKSPACE_BYTES (32 * 1024 * 1024)  // 32 MB
#endif
static constexpr size_t kCublasLtMaxWorkspaceBytes = EIGEN_CUDA_CUBLASLT_MAX_WORKSPACE_BYTES;

// Algorithm hint for the cublasGemmEx fallback path.
constexpr cublasGemmAlgo_t cuda_gemm_algo() {
#ifdef EIGEN_NO_CUDA_TENSOR_OPS
  return CUBLAS_GEMM_DEFAULT;
#else
  return CUBLAS_GEMM_DEFAULT_TENSOR_OP;
#endif
}

// Per-call descriptor creation and heuristic lookup cost 5-35% of total GEMM time
// for small and medium matrices, so plans are cached by shape. Typical workloads
// (CG iteration, chained solves) touch only 1-3 distinct shapes, hence the small
// capacity.
static constexpr std::size_t kCublasLtPlanCacheCapacity = 8;

struct CublasLtPlanKey {
  int64_t m, n, k;
  int64_t lda, ldb, ldc;
  cudaDataType_t dtype;
  cublasOperation_t transA, transB;

  bool operator==(const CublasLtPlanKey& o) const {
    return m == o.m && n == o.n && k == o.k && lda == o.lda && ldb == o.ldb && ldc == o.ldc && dtype == o.dtype &&
           transA == o.transA && transB == o.transB;
  }
};

struct CublasLtPlanKeyHash {
  std::size_t operator()(const CublasLtPlanKey& k) const noexcept {
    // boost-style hash_combine: mix each field into the rolling hash.
    auto mix = [](std::size_t a, std::size_t b) { return a ^ (b + 0x9e3779b97f4a7c15ULL + (a << 6) + (a >> 2)); };
    std::size_t r = std::hash<int64_t>{}(k.m);
    r = mix(r, std::hash<int64_t>{}(k.n));
    r = mix(r, std::hash<int64_t>{}(k.k));
    r = mix(r, std::hash<int64_t>{}(k.lda));
    r = mix(r, std::hash<int64_t>{}(k.ldb));
    r = mix(r, std::hash<int64_t>{}(k.ldc));
    r = mix(r, std::hash<int>{}(static_cast<int>(k.dtype)));
    r = mix(r, std::hash<int>{}(static_cast<int>(k.transA)));
    r = mix(r, std::hash<int>{}(static_cast<int>(k.transB)));
    return r;
  }
};

// A cached cuBLASLt matmul plan: the descriptor, three matrix layouts, and the
// heuristic-selected algorithm. Destruction releases all four cuBLASLt handles,
// so LruCache eviction is enough to manage entry lifetime. Move-only, since
// copying would alias the handles and double-destroy them.
class CublasLtPlanEntry {
 public:
  // If the heuristic returns no usable algorithm, use_cublaslt stays false and
  // the caller takes the cublasGemmEx fallback path. `max_workspace_bytes` is the
  // heuristic's ceiling — see gpu::Context::setCublasLtMaxWorkspaceBytes().
  CublasLtPlanEntry(cublasLtHandle_t lt_handle, const CublasLtPlanKey& key, cublasComputeType_t compute,
                    cudaDataType_t alpha_type, std::size_t max_workspace_bytes) {
    EIGEN_CUBLASLT_CHECK(cublasLtMatmulDescCreate(&matmul_desc, compute, alpha_type));
    EIGEN_CUBLASLT_CHECK(
        cublasLtMatmulDescSetAttribute(matmul_desc, CUBLASLT_MATMUL_DESC_TRANSA, &key.transA, sizeof(key.transA)));
    EIGEN_CUBLASLT_CHECK(
        cublasLtMatmulDescSetAttribute(matmul_desc, CUBLASLT_MATMUL_DESC_TRANSB, &key.transB, sizeof(key.transB)));

    // Layout dimensions are the physical (rows, cols) of the column-major operand;
    // the leading dimension is the actual stride between columns (lda/ldb/ldc),
    // which may exceed the active row count (e.g., a thin view of a wider buffer).
    const int64_t a_rows = (key.transA == CUBLAS_OP_N) ? key.m : key.k;
    const int64_t a_cols = (key.transA == CUBLAS_OP_N) ? key.k : key.m;
    const int64_t b_rows = (key.transB == CUBLAS_OP_N) ? key.k : key.n;
    const int64_t b_cols = (key.transB == CUBLAS_OP_N) ? key.n : key.k;
    EIGEN_CUBLASLT_CHECK(cublasLtMatrixLayoutCreate(&layout_A, key.dtype, a_rows, a_cols, key.lda));
    EIGEN_CUBLASLT_CHECK(cublasLtMatrixLayoutCreate(&layout_B, key.dtype, b_rows, b_cols, key.ldb));
    EIGEN_CUBLASLT_CHECK(cublasLtMatrixLayoutCreate(&layout_C, key.dtype, key.m, key.n, key.ldc));

    cublasLtMatmulPreference_t preference = nullptr;
    EIGEN_CUBLASLT_CHECK(cublasLtMatmulPreferenceCreate(&preference));
    EIGEN_CUBLASLT_CHECK(cublasLtMatmulPreferenceSetAttribute(preference, CUBLASLT_MATMUL_PREF_MAX_WORKSPACE_BYTES,
                                                              &max_workspace_bytes, sizeof(max_workspace_bytes)));

    cublasLtMatmulHeuristicResult_t result;
    int returned_results = 0;
    cublasStatus_t heuristic_status = cublasLtMatmulAlgoGetHeuristic(
        lt_handle, matmul_desc, layout_A, layout_B, layout_C, layout_C, preference, 1, &result, &returned_results);

    EIGEN_CUBLASLT_CHECK(cublasLtMatmulPreferenceDestroy(preference));

    // cublasLtMatmulAlgoGetHeuristic can return CUBLAS_STATUS_SUCCESS overall while
    // marking individual results NOT_SUPPORTED via result.state, so gate on both.
    if (heuristic_status == CUBLAS_STATUS_SUCCESS && returned_results > 0 && result.state == CUBLAS_STATUS_SUCCESS) {
      algo = result.algo;
      workspace_size = result.workspaceSize;
      use_cublaslt = true;
    }
  }

  ~CublasLtPlanEntry() { destroy(); }

  CublasLtPlanEntry(const CublasLtPlanEntry&) = delete;
  CublasLtPlanEntry& operator=(const CublasLtPlanEntry&) = delete;

  CublasLtPlanEntry(CublasLtPlanEntry&& o) noexcept
      : matmul_desc(o.matmul_desc),
        layout_A(o.layout_A),
        layout_B(o.layout_B),
        layout_C(o.layout_C),
        algo(o.algo),
        workspace_size(o.workspace_size),
        use_cublaslt(o.use_cublaslt) {
    o.matmul_desc = nullptr;
    o.layout_A = o.layout_B = o.layout_C = nullptr;
    o.use_cublaslt = false;
  }

  CublasLtPlanEntry& operator=(CublasLtPlanEntry&& o) noexcept {
    if (this != &o) {
      destroy();
      matmul_desc = o.matmul_desc;
      layout_A = o.layout_A;
      layout_B = o.layout_B;
      layout_C = o.layout_C;
      algo = o.algo;
      workspace_size = o.workspace_size;
      use_cublaslt = o.use_cublaslt;
      o.matmul_desc = nullptr;
      o.layout_A = o.layout_B = o.layout_C = nullptr;
      o.use_cublaslt = false;
    }
    return *this;
  }

  cublasLtMatmulDesc_t matmul_desc = nullptr;
  cublasLtMatrixLayout_t layout_A = nullptr;
  cublasLtMatrixLayout_t layout_B = nullptr;
  cublasLtMatrixLayout_t layout_C = nullptr;
  cublasLtMatmulAlgo_t algo{};
  std::size_t workspace_size = 0;
  bool use_cublaslt = false;

 private:
  void destroy() noexcept {
    if (layout_C) cublasLtMatrixLayoutDestroy(layout_C);
    if (layout_B) cublasLtMatrixLayoutDestroy(layout_B);
    if (layout_A) cublasLtMatrixLayoutDestroy(layout_A);
    if (matmul_desc) cublasLtMatmulDescDestroy(matmul_desc);
  }
};

using CublasLtPlanCache = Eigen::internal::LruCache<CublasLtPlanKey, CublasLtPlanEntry, CublasLtPlanKeyHash>;

// cublasLtMatmul with a shape-keyed plan cache, falling back to cublasGemmEx for
// shapes and types the cublasLt heuristic cannot serve. Dimensions are 64-bit on
// the cublasLt path. `workspace` grows monotonically to the selected algorithm's
// requirement; neither it nor `plan_cache` is thread-safe, so all calls sharing
// them must run on one stream.
template <typename Scalar>
void cublaslt_gemm(cublasLtHandle_t lt_handle, cublasHandle_t cublas_handle, cublasOperation_t transA,
                   cublasOperation_t transB, int64_t m, int64_t n, int64_t k, const Scalar* alpha, const Scalar* A,
                   int64_t lda, const Scalar* B, int64_t ldb, const Scalar* beta, Scalar* C, int64_t ldc,
                   DeviceBuffer& workspace, CublasLtPlanCache& plan_cache, std::size_t max_workspace_bytes,
                   cudaStream_t stream) {
  constexpr cudaDataType_t dtype = cuda_data_type<Scalar>::value;
  constexpr cublasComputeType_t compute = cuda_compute_type<Scalar>::value;
  constexpr cudaDataType_t alpha_type = cuda_data_type<Scalar>::value;

  // The key carries the leading dimensions so that strided views — e.g. SVD's
  // thin VT/U slices — get distinct cache entries.
  const CublasLtPlanKey key{m, n, k, lda, ldb, ldc, dtype, transA, transB};
  CublasLtPlanEntry* entry = plan_cache.find(key);
  if (!entry) {
    entry = plan_cache.insert(key, CublasLtPlanEntry(lt_handle, key, compute, alpha_type, max_workspace_bytes));
  }

  if (entry->use_cublaslt) {
    const size_t needed = entry->workspace_size;
    if (needed > workspace.size()) {
      // Sync only when freeing an existing buffer that may be in use.
      if (workspace.get()) EIGEN_CUDA_RUNTIME_CHECK(cudaStreamSynchronize(stream));
      workspace = DeviceBuffer(needed);
    }

    EIGEN_CUBLASLT_CHECK(cublasLtMatmul(lt_handle, entry->matmul_desc, alpha, A, entry->layout_A, B, entry->layout_B,
                                        beta, C, entry->layout_C, C, entry->layout_C, &entry->algo, workspace.get(),
                                        needed, stream));
  } else {
    // Fallback: cublasGemmEx for shapes/types that cublasLt cannot handle.
    EIGEN_CUBLAS_CHECK(EIGEN_CUBLAS_FN(cublasGemmEx)(
        cublas_handle, transA, transB, to_blas_dim(m), to_blas_dim(n), to_blas_dim(k), alpha, A, dtype,
        to_blas_dim(lda), B, dtype, to_blas_dim(ldb), beta, C, dtype, to_blas_dim(ldc), compute, cuda_gemm_algo()));
  }
}

// cuBLAS exposes one entry point per scalar type (Sgemm, Dgemm, ...). The
// cublasX* overloads below recover a type-generic interface over them.
inline cublasStatus_t cublasXgemm(cublasHandle_t h, cublasOperation_t transA, cublasOperation_t transB, int64_t m,
                                  int64_t n, int64_t k, const float* alpha, const float* A, int64_t lda, const float* B,
                                  int64_t ldb, const float* beta, float* C, int64_t ldc) {
  return EIGEN_CUBLAS_FN(cublasSgemm)(h, transA, transB, to_blas_dim(m), to_blas_dim(n), to_blas_dim(k), alpha, A,
                                      to_blas_dim(lda), B, to_blas_dim(ldb), beta, C, to_blas_dim(ldc));
}
inline cublasStatus_t cublasXgemm(cublasHandle_t h, cublasOperation_t transA, cublasOperation_t transB, int64_t m,
                                  int64_t n, int64_t k, const double* alpha, const double* A, int64_t lda,
                                  const double* B, int64_t ldb, const double* beta, double* C, int64_t ldc) {
  return EIGEN_CUBLAS_FN(cublasDgemm)(h, transA, transB, to_blas_dim(m), to_blas_dim(n), to_blas_dim(k), alpha, A,
                                      to_blas_dim(lda), B, to_blas_dim(ldb), beta, C, to_blas_dim(ldc));
}
static_assert(sizeof(cuComplex) == sizeof(std::complex<float>), "cuComplex and std::complex<float> layout mismatch");
static_assert(sizeof(cuDoubleComplex) == sizeof(std::complex<double>),
              "cuDoubleComplex and std::complex<double> layout mismatch");

// Complex alpha/beta are type-punned from std::complex<T>* to
// cuComplex*/cuDoubleComplex*. reinterpret_cast violates strict aliasing here:
// once inlined, clang/MSVC no longer see a read through the original type and
// elide the caller's store, which segfaults. std::memcpy is the standard-blessed
// pun. Device array pointers (A, B, C) are never dereferenced by the host
// compiler, so reinterpret_cast is safe for them.
inline cublasStatus_t cublasXgemm(cublasHandle_t h, cublasOperation_t transA, cublasOperation_t transB, int64_t m,
                                  int64_t n, int64_t k, const std::complex<float>* alpha, const std::complex<float>* A,
                                  int64_t lda, const std::complex<float>* B, int64_t ldb,
                                  const std::complex<float>* beta, std::complex<float>* C, int64_t ldc) {
  cuComplex a, b;
  std::memcpy(&a, alpha, sizeof(a));
  std::memcpy(&b, beta, sizeof(b));
  return EIGEN_CUBLAS_FN(cublasCgemm)(h, transA, transB, to_blas_dim(m), to_blas_dim(n), to_blas_dim(k), &a,
                                      reinterpret_cast<const cuComplex*>(A), to_blas_dim(lda),
                                      reinterpret_cast<const cuComplex*>(B), to_blas_dim(ldb), &b,
                                      reinterpret_cast<cuComplex*>(C), to_blas_dim(ldc));
}
inline cublasStatus_t cublasXgemm(cublasHandle_t h, cublasOperation_t transA, cublasOperation_t transB, int64_t m,
                                  int64_t n, int64_t k, const std::complex<double>* alpha,
                                  const std::complex<double>* A, int64_t lda, const std::complex<double>* B,
                                  int64_t ldb, const std::complex<double>* beta, std::complex<double>* C, int64_t ldc) {
  cuDoubleComplex a, b;
  std::memcpy(&a, alpha, sizeof(a));
  std::memcpy(&b, beta, sizeof(b));
  return EIGEN_CUBLAS_FN(cublasZgemm)(h, transA, transB, to_blas_dim(m), to_blas_dim(n), to_blas_dim(k), &a,
                                      reinterpret_cast<const cuDoubleComplex*>(A), to_blas_dim(lda),
                                      reinterpret_cast<const cuDoubleComplex*>(B), to_blas_dim(ldb), &b,
                                      reinterpret_cast<cuDoubleComplex*>(C), to_blas_dim(ldc));
}

inline cublasStatus_t cublasXtrsm(cublasHandle_t h, cublasSideMode_t side, cublasFillMode_t uplo,
                                  cublasOperation_t trans, cublasDiagType_t diag, int64_t m, int64_t n,
                                  const float* alpha, const float* A, int64_t lda, float* B, int64_t ldb) {
  return EIGEN_CUBLAS_FN(cublasStrsm)(h, side, uplo, trans, diag, to_blas_dim(m), to_blas_dim(n), alpha, A,
                                      to_blas_dim(lda), B, to_blas_dim(ldb));
}
inline cublasStatus_t cublasXtrsm(cublasHandle_t h, cublasSideMode_t side, cublasFillMode_t uplo,
                                  cublasOperation_t trans, cublasDiagType_t diag, int64_t m, int64_t n,
                                  const double* alpha, const double* A, int64_t lda, double* B, int64_t ldb) {
  return EIGEN_CUBLAS_FN(cublasDtrsm)(h, side, uplo, trans, diag, to_blas_dim(m), to_blas_dim(n), alpha, A,
                                      to_blas_dim(lda), B, to_blas_dim(ldb));
}
inline cublasStatus_t cublasXtrsm(cublasHandle_t h, cublasSideMode_t side, cublasFillMode_t uplo,
                                  cublasOperation_t trans, cublasDiagType_t diag, int64_t m, int64_t n,
                                  const std::complex<float>* alpha, const std::complex<float>* A, int64_t lda,
                                  std::complex<float>* B, int64_t ldb) {
  cuComplex a;
  std::memcpy(&a, alpha, sizeof(a));
  return EIGEN_CUBLAS_FN(cublasCtrsm)(h, side, uplo, trans, diag, to_blas_dim(m), to_blas_dim(n), &a,
                                      reinterpret_cast<const cuComplex*>(A), to_blas_dim(lda),
                                      reinterpret_cast<cuComplex*>(B), to_blas_dim(ldb));
}
inline cublasStatus_t cublasXtrsm(cublasHandle_t h, cublasSideMode_t side, cublasFillMode_t uplo,
                                  cublasOperation_t trans, cublasDiagType_t diag, int64_t m, int64_t n,
                                  const std::complex<double>* alpha, const std::complex<double>* A, int64_t lda,
                                  std::complex<double>* B, int64_t ldb) {
  cuDoubleComplex a;
  std::memcpy(&a, alpha, sizeof(a));
  return EIGEN_CUBLAS_FN(cublasZtrsm)(h, side, uplo, trans, diag, to_blas_dim(m), to_blas_dim(n), &a,
                                      reinterpret_cast<const cuDoubleComplex*>(A), to_blas_dim(lda),
                                      reinterpret_cast<cuDoubleComplex*>(B), to_blas_dim(ldb));
}

// SYMM: real → symm, complex → hemm.
inline cublasStatus_t cublasXsymm(cublasHandle_t h, cublasSideMode_t side, cublasFillMode_t uplo, int64_t m, int64_t n,
                                  const float* alpha, const float* A, int64_t lda, const float* B, int64_t ldb,
                                  const float* beta, float* C, int64_t ldc) {
  return EIGEN_CUBLAS_FN(cublasSsymm)(h, side, uplo, to_blas_dim(m), to_blas_dim(n), alpha, A, to_blas_dim(lda), B,
                                      to_blas_dim(ldb), beta, C, to_blas_dim(ldc));
}
inline cublasStatus_t cublasXsymm(cublasHandle_t h, cublasSideMode_t side, cublasFillMode_t uplo, int64_t m, int64_t n,
                                  const double* alpha, const double* A, int64_t lda, const double* B, int64_t ldb,
                                  const double* beta, double* C, int64_t ldc) {
  return EIGEN_CUBLAS_FN(cublasDsymm)(h, side, uplo, to_blas_dim(m), to_blas_dim(n), alpha, A, to_blas_dim(lda), B,
                                      to_blas_dim(ldb), beta, C, to_blas_dim(ldc));
}
inline cublasStatus_t cublasXsymm(cublasHandle_t h, cublasSideMode_t side, cublasFillMode_t uplo, int64_t m, int64_t n,
                                  const std::complex<float>* alpha, const std::complex<float>* A, int64_t lda,
                                  const std::complex<float>* B, int64_t ldb, const std::complex<float>* beta,
                                  std::complex<float>* C, int64_t ldc) {
  cuComplex a, b;
  std::memcpy(&a, alpha, sizeof(a));
  std::memcpy(&b, beta, sizeof(b));
  return EIGEN_CUBLAS_FN(cublasChemm)(
      h, side, uplo, to_blas_dim(m), to_blas_dim(n), &a, reinterpret_cast<const cuComplex*>(A), to_blas_dim(lda),
      reinterpret_cast<const cuComplex*>(B), to_blas_dim(ldb), &b, reinterpret_cast<cuComplex*>(C), to_blas_dim(ldc));
}
inline cublasStatus_t cublasXsymm(cublasHandle_t h, cublasSideMode_t side, cublasFillMode_t uplo, int64_t m, int64_t n,
                                  const std::complex<double>* alpha, const std::complex<double>* A, int64_t lda,
                                  const std::complex<double>* B, int64_t ldb, const std::complex<double>* beta,
                                  std::complex<double>* C, int64_t ldc) {
  cuDoubleComplex a, b;
  std::memcpy(&a, alpha, sizeof(a));
  std::memcpy(&b, beta, sizeof(b));
  return EIGEN_CUBLAS_FN(cublasZhemm)(h, side, uplo, to_blas_dim(m), to_blas_dim(n), &a,
                                      reinterpret_cast<const cuDoubleComplex*>(A), to_blas_dim(lda),
                                      reinterpret_cast<const cuDoubleComplex*>(B), to_blas_dim(ldb), &b,
                                      reinterpret_cast<cuDoubleComplex*>(C), to_blas_dim(ldc));
}

// GEAM: C = alpha * op(A) + beta * op(B).
inline cublasStatus_t cublasXgeam(cublasHandle_t h, cublasOperation_t transA, cublasOperation_t transB, int64_t m,
                                  int64_t n, const float* alpha, const float* A, int64_t lda, const float* beta,
                                  const float* B, int64_t ldb, float* C, int64_t ldc) {
  return EIGEN_CUBLAS_FN(cublasSgeam)(h, transA, transB, to_blas_dim(m), to_blas_dim(n), alpha, A, to_blas_dim(lda),
                                      beta, B, to_blas_dim(ldb), C, to_blas_dim(ldc));
}
inline cublasStatus_t cublasXgeam(cublasHandle_t h, cublasOperation_t transA, cublasOperation_t transB, int64_t m,
                                  int64_t n, const double* alpha, const double* A, int64_t lda, const double* beta,
                                  const double* B, int64_t ldb, double* C, int64_t ldc) {
  return EIGEN_CUBLAS_FN(cublasDgeam)(h, transA, transB, to_blas_dim(m), to_blas_dim(n), alpha, A, to_blas_dim(lda),
                                      beta, B, to_blas_dim(ldb), C, to_blas_dim(ldc));
}
inline cublasStatus_t cublasXgeam(cublasHandle_t h, cublasOperation_t transA, cublasOperation_t transB, int64_t m,
                                  int64_t n, const std::complex<float>* alpha, const std::complex<float>* A,
                                  int64_t lda, const std::complex<float>* beta, const std::complex<float>* B,
                                  int64_t ldb, std::complex<float>* C, int64_t ldc) {
  cuComplex a, b;
  std::memcpy(&a, alpha, sizeof(a));
  std::memcpy(&b, beta, sizeof(b));
  return EIGEN_CUBLAS_FN(cublasCgeam)(
      h, transA, transB, to_blas_dim(m), to_blas_dim(n), &a, reinterpret_cast<const cuComplex*>(A), to_blas_dim(lda),
      &b, reinterpret_cast<const cuComplex*>(B), to_blas_dim(ldb), reinterpret_cast<cuComplex*>(C), to_blas_dim(ldc));
}
inline cublasStatus_t cublasXgeam(cublasHandle_t h, cublasOperation_t transA, cublasOperation_t transB, int64_t m,
                                  int64_t n, const std::complex<double>* alpha, const std::complex<double>* A,
                                  int64_t lda, const std::complex<double>* beta, const std::complex<double>* B,
                                  int64_t ldb, std::complex<double>* C, int64_t ldc) {
  cuDoubleComplex a, b;
  std::memcpy(&a, alpha, sizeof(a));
  std::memcpy(&b, beta, sizeof(b));
  return EIGEN_CUBLAS_FN(cublasZgeam)(h, transA, transB, to_blas_dim(m), to_blas_dim(n), &a,
                                      reinterpret_cast<const cuDoubleComplex*>(A), to_blas_dim(lda), &b,
                                      reinterpret_cast<const cuDoubleComplex*>(B), to_blas_dim(ldb),
                                      reinterpret_cast<cuDoubleComplex*>(C), to_blas_dim(ldc));
}

// SYRK: real → syrk, complex → herk.
inline cublasStatus_t cublasXsyrk(cublasHandle_t h, cublasFillMode_t uplo, cublasOperation_t trans, int64_t n,
                                  int64_t k, const float* alpha, const float* A, int64_t lda, const float* beta,
                                  float* C, int64_t ldc) {
  return EIGEN_CUBLAS_FN(cublasSsyrk)(h, uplo, trans, to_blas_dim(n), to_blas_dim(k), alpha, A, to_blas_dim(lda), beta,
                                      C, to_blas_dim(ldc));
}
inline cublasStatus_t cublasXsyrk(cublasHandle_t h, cublasFillMode_t uplo, cublasOperation_t trans, int64_t n,
                                  int64_t k, const double* alpha, const double* A, int64_t lda, const double* beta,
                                  double* C, int64_t ldc) {
  return EIGEN_CUBLAS_FN(cublasDsyrk)(h, uplo, trans, to_blas_dim(n), to_blas_dim(k), alpha, A, to_blas_dim(lda), beta,
                                      C, to_blas_dim(ldc));
}
inline cublasStatus_t cublasXsyrk(cublasHandle_t h, cublasFillMode_t uplo, cublasOperation_t trans, int64_t n,
                                  int64_t k, const float* alpha, const std::complex<float>* A, int64_t lda,
                                  const float* beta, std::complex<float>* C, int64_t ldc) {
  return EIGEN_CUBLAS_FN(cublasCherk)(h, uplo, trans, to_blas_dim(n), to_blas_dim(k), alpha,
                                      reinterpret_cast<const cuComplex*>(A), to_blas_dim(lda), beta,
                                      reinterpret_cast<cuComplex*>(C), to_blas_dim(ldc));
}
inline cublasStatus_t cublasXsyrk(cublasHandle_t h, cublasFillMode_t uplo, cublasOperation_t trans, int64_t n,
                                  int64_t k, const double* alpha, const std::complex<double>* A, int64_t lda,
                                  const double* beta, std::complex<double>* C, int64_t ldc) {
  return EIGEN_CUBLAS_FN(cublasZherk)(h, uplo, trans, to_blas_dim(n), to_blas_dim(k), alpha,
                                      reinterpret_cast<const cuDoubleComplex*>(A), to_blas_dim(lda), beta,
                                      reinterpret_cast<cuDoubleComplex*>(C), to_blas_dim(ldc));
}

// SCAL: x = alpha * x, with real alpha even for complex x (Csscal/Zdscal), as
// needed by the intrinsically real 1/n inverse-FFT scaling.
inline cublasStatus_t cublasXscal(cublasHandle_t h, int64_t n, const float* alpha, float* x, int64_t incx) {
  return EIGEN_CUBLAS_FN(cublasSscal)(h, to_blas_dim(n), alpha, x, to_blas_dim(incx));
}
inline cublasStatus_t cublasXscal(cublasHandle_t h, int64_t n, const double* alpha, double* x, int64_t incx) {
  return EIGEN_CUBLAS_FN(cublasDscal)(h, to_blas_dim(n), alpha, x, to_blas_dim(incx));
}
inline cublasStatus_t cublasXscal(cublasHandle_t h, int64_t n, const float* alpha, std::complex<float>* x,
                                  int64_t incx) {
  return EIGEN_CUBLAS_FN(cublasCsscal)(h, to_blas_dim(n), alpha, reinterpret_cast<cuComplex*>(x), to_blas_dim(incx));
}
inline cublasStatus_t cublasXscal(cublasHandle_t h, int64_t n, const double* alpha, std::complex<double>* x,
                                  int64_t incx) {
  return EIGEN_CUBLAS_FN(cublasZdscal)(h, to_blas_dim(n), alpha, reinterpret_cast<cuDoubleComplex*>(x),
                                       to_blas_dim(incx));
}

// By-value alpha, for callers holding the scale as a scalar rather than a host
// pointer (e.g. inverse-FFT 1/n normalization).
inline cublasStatus_t cublasXscal(cublasHandle_t h, int64_t n, float alpha, float* x, int64_t incx) {
  return EIGEN_CUBLAS_FN(cublasSscal)(h, to_blas_dim(n), &alpha, x, to_blas_dim(incx));
}
inline cublasStatus_t cublasXscal(cublasHandle_t h, int64_t n, double alpha, double* x, int64_t incx) {
  return EIGEN_CUBLAS_FN(cublasDscal)(h, to_blas_dim(n), &alpha, x, to_blas_dim(incx));
}
inline cublasStatus_t cublasXscal(cublasHandle_t h, int64_t n, float alpha, std::complex<float>* x, int64_t incx) {
  return EIGEN_CUBLAS_FN(cublasCsscal)(h, to_blas_dim(n), &alpha, reinterpret_cast<cuComplex*>(x), to_blas_dim(incx));
}
inline cublasStatus_t cublasXscal(cublasHandle_t h, int64_t n, double alpha, std::complex<double>* x, int64_t incx) {
  return EIGEN_CUBLAS_FN(cublasZdscal)(h, to_blas_dim(n), &alpha, reinterpret_cast<cuDoubleComplex*>(x),
                                       to_blas_dim(incx));
}

// DGMM: C = A * diag(x) (side=RIGHT) or C = diag(x) * A (side=LEFT), applying a
// diagonal scaling without materialising diag(x). cuBLAS documents C == A as
// safe.
inline cublasStatus_t cublasXdgmm(cublasHandle_t h, cublasSideMode_t side, int64_t m, int64_t n, const float* A,
                                  int64_t lda, const float* x, int64_t incx, float* C, int64_t ldc) {
  return EIGEN_CUBLAS_FN(cublasSdgmm)(h, side, to_blas_dim(m), to_blas_dim(n), A, to_blas_dim(lda), x,
                                      to_blas_dim(incx), C, to_blas_dim(ldc));
}
inline cublasStatus_t cublasXdgmm(cublasHandle_t h, cublasSideMode_t side, int64_t m, int64_t n, const double* A,
                                  int64_t lda, const double* x, int64_t incx, double* C, int64_t ldc) {
  return EIGEN_CUBLAS_FN(cublasDdgmm)(h, side, to_blas_dim(m), to_blas_dim(n), A, to_blas_dim(lda), x,
                                      to_blas_dim(incx), C, to_blas_dim(ldc));
}
inline cublasStatus_t cublasXdgmm(cublasHandle_t h, cublasSideMode_t side, int64_t m, int64_t n,
                                  const std::complex<float>* A, int64_t lda, const std::complex<float>* x, int64_t incx,
                                  std::complex<float>* C, int64_t ldc) {
  return EIGEN_CUBLAS_FN(cublasCdgmm)(h, side, to_blas_dim(m), to_blas_dim(n), reinterpret_cast<const cuComplex*>(A),
                                      to_blas_dim(lda), reinterpret_cast<const cuComplex*>(x), to_blas_dim(incx),
                                      reinterpret_cast<cuComplex*>(C), to_blas_dim(ldc));
}
inline cublasStatus_t cublasXdgmm(cublasHandle_t h, cublasSideMode_t side, int64_t m, int64_t n,
                                  const std::complex<double>* A, int64_t lda, const std::complex<double>* x,
                                  int64_t incx, std::complex<double>* C, int64_t ldc) {
  return EIGEN_CUBLAS_FN(cublasZdgmm)(h, side, to_blas_dim(m), to_blas_dim(n),
                                      reinterpret_cast<const cuDoubleComplex*>(A), to_blas_dim(lda),
                                      reinterpret_cast<const cuDoubleComplex*>(x), to_blas_dim(incx),
                                      reinterpret_cast<cuDoubleComplex*>(C), to_blas_dim(ldc));
}

// The BLAS-1 wrappers below honour whichever pointer mode the caller set on the
// handle; under CUBLAS_POINTER_MODE_DEVICE the dot/nrm2 result pointers must
// address device memory.

// dot: result = x^T * y (real) or x^H * y (complex, conjugating x).
inline cublasStatus_t cublasXdot(cublasHandle_t h, int64_t n, const float* x, int64_t incx, const float* y,
                                 int64_t incy, float* result) {
  return EIGEN_CUBLAS_FN(cublasSdot)(h, to_blas_dim(n), x, to_blas_dim(incx), y, to_blas_dim(incy), result);
}
inline cublasStatus_t cublasXdot(cublasHandle_t h, int64_t n, const double* x, int64_t incx, const double* y,
                                 int64_t incy, double* result) {
  return EIGEN_CUBLAS_FN(cublasDdot)(h, to_blas_dim(n), x, to_blas_dim(incx), y, to_blas_dim(incy), result);
}
inline cublasStatus_t cublasXdot(cublasHandle_t h, int64_t n, const std::complex<float>* x, int64_t incx,
                                 const std::complex<float>* y, int64_t incy, std::complex<float>* result) {
  return EIGEN_CUBLAS_FN(cublasCdotc)(h, to_blas_dim(n), reinterpret_cast<const cuComplex*>(x), to_blas_dim(incx),
                                      reinterpret_cast<const cuComplex*>(y), to_blas_dim(incy),
                                      reinterpret_cast<cuComplex*>(result));
}
inline cublasStatus_t cublasXdot(cublasHandle_t h, int64_t n, const std::complex<double>* x, int64_t incx,
                                 const std::complex<double>* y, int64_t incy, std::complex<double>* result) {
  return EIGEN_CUBLAS_FN(cublasZdotc)(h, to_blas_dim(n), reinterpret_cast<const cuDoubleComplex*>(x), to_blas_dim(incx),
                                      reinterpret_cast<const cuDoubleComplex*>(y), to_blas_dim(incy),
                                      reinterpret_cast<cuDoubleComplex*>(result));
}

// nrm2: result = ||x||_2, always real.
inline cublasStatus_t cublasXnrm2(cublasHandle_t h, int64_t n, const float* x, int64_t incx, float* result) {
  return EIGEN_CUBLAS_FN(cublasSnrm2)(h, to_blas_dim(n), x, to_blas_dim(incx), result);
}
inline cublasStatus_t cublasXnrm2(cublasHandle_t h, int64_t n, const double* x, int64_t incx, double* result) {
  return EIGEN_CUBLAS_FN(cublasDnrm2)(h, to_blas_dim(n), x, to_blas_dim(incx), result);
}
inline cublasStatus_t cublasXnrm2(cublasHandle_t h, int64_t n, const std::complex<float>* x, int64_t incx,
                                  float* result) {
  return EIGEN_CUBLAS_FN(cublasScnrm2)(h, to_blas_dim(n), reinterpret_cast<const cuComplex*>(x), to_blas_dim(incx),
                                       result);
}
inline cublasStatus_t cublasXnrm2(cublasHandle_t h, int64_t n, const std::complex<double>* x, int64_t incx,
                                  double* result) {
  return EIGEN_CUBLAS_FN(cublasDznrm2)(h, to_blas_dim(n), reinterpret_cast<const cuDoubleComplex*>(x),
                                       to_blas_dim(incx), result);
}

// axpy: y += alpha * x.
inline cublasStatus_t cublasXaxpy(cublasHandle_t h, int64_t n, const float* alpha, const float* x, int64_t incx,
                                  float* y, int64_t incy) {
  return EIGEN_CUBLAS_FN(cublasSaxpy)(h, to_blas_dim(n), alpha, x, to_blas_dim(incx), y, to_blas_dim(incy));
}
inline cublasStatus_t cublasXaxpy(cublasHandle_t h, int64_t n, const double* alpha, const double* x, int64_t incx,
                                  double* y, int64_t incy) {
  return EIGEN_CUBLAS_FN(cublasDaxpy)(h, to_blas_dim(n), alpha, x, to_blas_dim(incx), y, to_blas_dim(incy));
}
inline cublasStatus_t cublasXaxpy(cublasHandle_t h, int64_t n, const std::complex<float>* alpha,
                                  const std::complex<float>* x, int64_t incx, std::complex<float>* y, int64_t incy) {
  cuComplex a;
  std::memcpy(&a, alpha, sizeof(a));
  return EIGEN_CUBLAS_FN(cublasCaxpy)(h, to_blas_dim(n), &a, reinterpret_cast<const cuComplex*>(x), to_blas_dim(incx),
                                      reinterpret_cast<cuComplex*>(y), to_blas_dim(incy));
}
inline cublasStatus_t cublasXaxpy(cublasHandle_t h, int64_t n, const std::complex<double>* alpha,
                                  const std::complex<double>* x, int64_t incx, std::complex<double>* y, int64_t incy) {
  cuDoubleComplex a;
  std::memcpy(&a, alpha, sizeof(a));
  return EIGEN_CUBLAS_FN(cublasZaxpy)(h, to_blas_dim(n), &a, reinterpret_cast<const cuDoubleComplex*>(x),
                                      to_blas_dim(incx), reinterpret_cast<cuDoubleComplex*>(y), to_blas_dim(incy));
}

// SCAL with complex alpha (Cscal/Zscal); the real-alpha forms are above.
inline cublasStatus_t cublasXscal(cublasHandle_t h, int64_t n, const std::complex<float>* alpha, std::complex<float>* x,
                                  int64_t incx) {
  cuComplex a;
  std::memcpy(&a, alpha, sizeof(a));
  return EIGEN_CUBLAS_FN(cublasCscal)(h, to_blas_dim(n), &a, reinterpret_cast<cuComplex*>(x), to_blas_dim(incx));
}
inline cublasStatus_t cublasXscal(cublasHandle_t h, int64_t n, const std::complex<double>* alpha,
                                  std::complex<double>* x, int64_t incx) {
  cuDoubleComplex a;
  std::memcpy(&a, alpha, sizeof(a));
  return EIGEN_CUBLAS_FN(cublasZscal)(h, to_blas_dim(n), &a, reinterpret_cast<cuDoubleComplex*>(x), to_blas_dim(incx));
}

// copy: y = x.
inline cublasStatus_t cublasXcopy(cublasHandle_t h, int64_t n, const float* x, int64_t incx, float* y, int64_t incy) {
  return EIGEN_CUBLAS_FN(cublasScopy)(h, to_blas_dim(n), x, to_blas_dim(incx), y, to_blas_dim(incy));
}
inline cublasStatus_t cublasXcopy(cublasHandle_t h, int64_t n, const double* x, int64_t incx, double* y, int64_t incy) {
  return EIGEN_CUBLAS_FN(cublasDcopy)(h, to_blas_dim(n), x, to_blas_dim(incx), y, to_blas_dim(incy));
}
inline cublasStatus_t cublasXcopy(cublasHandle_t h, int64_t n, const std::complex<float>* x, int64_t incx,
                                  std::complex<float>* y, int64_t incy) {
  return EIGEN_CUBLAS_FN(cublasCcopy)(h, to_blas_dim(n), reinterpret_cast<const cuComplex*>(x), to_blas_dim(incx),
                                      reinterpret_cast<cuComplex*>(y), to_blas_dim(incy));
}
inline cublasStatus_t cublasXcopy(cublasHandle_t h, int64_t n, const std::complex<double>* x, int64_t incx,
                                  std::complex<double>* y, int64_t incy) {
  return EIGEN_CUBLAS_FN(cublasZcopy)(h, to_blas_dim(n), reinterpret_cast<const cuDoubleComplex*>(x), to_blas_dim(incx),
                                      reinterpret_cast<cuDoubleComplex*>(y), to_blas_dim(incy));
}

}  // namespace internal
}  // namespace gpu
}  // namespace Eigen

#endif  // EIGEN_GPU_CUBLAS_SUPPORT_H
