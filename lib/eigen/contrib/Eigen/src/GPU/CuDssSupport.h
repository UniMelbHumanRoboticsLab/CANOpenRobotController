// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// cuDSS-specific support types. cuDSS is NVIDIA's sparse direct solver library
// (Cholesky, LDL^T, LU); it requires CUDA 12.0+ and ships separately from the
// CUDA Toolkit.

#ifndef EIGEN_GPU_CUDSS_SUPPORT_H
#define EIGEN_GPU_CUDSS_SUPPORT_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

#include "./GpuSupport.h"
#include <cudss.h>

/** 1 when the cuDSS headers this translation unit sees name the algorithms
 * gpu::SparseSolverConfig forwards (cuDSS >= 0.8), 0 otherwise. Where it is 0,
 * the algorithm enumerators are not declared and setConfig() rejects every
 * non-default field, so a caller that has to adapt its behavior branches on
 * this macro. */
#if defined(CUDSS_VERSION) && CUDSS_VERSION >= 800
#define EIGEN_HAS_CUDSS_SOLVER_CONFIG 1
#else
#define EIGEN_HAS_CUDSS_SOLVER_CONFIG 0
#endif

namespace Eigen {
namespace gpu {
namespace internal {
#define EIGEN_CUDSS_CHECK(x)                                              \
  do {                                                                    \
    cudssStatus_t _s = (x);                                               \
    eigen_assert(_s == CUDSS_STATUS_SUCCESS && "cuDSS call failed: " #x); \
    EIGEN_UNUSED_VARIABLE(_s);                                            \
  } while (0)

// cuDSS 0.8 changed cudssMatrixCreateDn/Csr from cudaDataType_t to the
// value-compatible cudssDataType_t (CUDSS_R_32F == CUDA_R_32F, ...). Map the
// module's cudaDataType_t traits through this alias so both API generations
// compile.
#if defined(CUDSS_VERSION) && CUDSS_VERSION >= 800
using cudss_value_type_t = cudssDataType_t;
#else
using cudss_value_type_t = cudaDataType_t;
#endif

constexpr cudss_value_type_t to_cudss_data_type(cudaDataType_t t) { return static_cast<cudss_value_type_t>(t); }

template <typename Scalar>
struct cudss_spd_type;

template <>
struct cudss_spd_type<float> {
  static constexpr cudssMatrixType_t value = CUDSS_MTYPE_SPD;
};
template <>
struct cudss_spd_type<double> {
  static constexpr cudssMatrixType_t value = CUDSS_MTYPE_SPD;
};
template <>
struct cudss_spd_type<std::complex<float>> {
  static constexpr cudssMatrixType_t value = CUDSS_MTYPE_HPD;
};
template <>
struct cudss_spd_type<std::complex<double>> {
  static constexpr cudssMatrixType_t value = CUDSS_MTYPE_HPD;
};

// Real → SYMMETRIC (A = A^T), complex → HERMITIAN (A = A^H). cuDSS also accepts
// CUDSS_MTYPE_SYMMETRIC for complex (A = A^T, no conjugation), but SparseLDLT
// implements Eigen's SimplicialLDLT semantics, which are Hermitian for complex.
// Complex symmetric would need a separate trait and solver mode.
template <typename Scalar>
struct cudss_hermitian_type;

template <>
struct cudss_hermitian_type<float> {
  static constexpr cudssMatrixType_t value = CUDSS_MTYPE_SYMMETRIC;
};
template <>
struct cudss_hermitian_type<double> {
  static constexpr cudssMatrixType_t value = CUDSS_MTYPE_SYMMETRIC;
};
template <>
struct cudss_hermitian_type<std::complex<float>> {
  static constexpr cudssMatrixType_t value = CUDSS_MTYPE_HERMITIAN;
};
template <>
struct cudss_hermitian_type<std::complex<double>> {
  static constexpr cudssMatrixType_t value = CUDSS_MTYPE_HERMITIAN;
};

template <typename StorageIndex>
struct cudss_index_type;

template <>
struct cudss_index_type<int> {
  static constexpr cudaDataType_t value = CUDA_R_32I;
};
template <>
struct cudss_index_type<int64_t> {
  static constexpr cudaDataType_t value = CUDA_R_64I;
};

// A symmetric matrix stored as CSC (ColMajor) reaches cuDSS as CSR of A^T. The
// data is identical since A = A^T, but the triangle view must be swapped.
template <int UpLo, int StorageOrder>
struct cudss_view_type;

template <>
struct cudss_view_type<Lower, ColMajor> {
  static constexpr cudssMatrixViewType_t value = CUDSS_MVIEW_UPPER;
};
template <>
struct cudss_view_type<Upper, ColMajor> {
  static constexpr cudssMatrixViewType_t value = CUDSS_MVIEW_LOWER;
};

template <>
struct cudss_view_type<Lower, RowMajor> {
  static constexpr cudssMatrixViewType_t value = CUDSS_MVIEW_LOWER;
};
template <>
struct cudss_view_type<Upper, RowMajor> {
  static constexpr cudssMatrixViewType_t value = CUDSS_MVIEW_UPPER;
};
}  // namespace internal
}  // namespace gpu
}  // namespace Eigen

#endif  // EIGEN_GPU_CUDSS_SUPPORT_H
