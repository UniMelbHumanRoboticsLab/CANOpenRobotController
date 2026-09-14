// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// cuSPARSE-specific support types.

#ifndef EIGEN_GPU_CUSPARSE_SUPPORT_H
#define EIGEN_GPU_CUSPARSE_SUPPORT_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

#include "./GpuSupport.h"
#include "./Meta.h"
#include <cusparse.h>

namespace Eigen {
namespace gpu {
namespace internal {

#define EIGEN_CUSPARSE_CHECK(x)                                                 \
  do {                                                                          \
    cusparseStatus_t _s = (x);                                                  \
    eigen_assert(_s == CUSPARSE_STATUS_SUCCESS && "cuSPARSE call failed: " #x); \
    EIGEN_UNUSED_VARIABLE(_s);                                                  \
  } while (0)

// cuSPARSE rejects CUSPARSE_OPERATION_CONJUGATE_TRANSPOSE for real scalar
// types; for real Scalar, ConjTrans is mathematically equivalent to Trans and
// is silently demoted. Complex Scalar passes through unchanged.
template <typename Scalar>
constexpr cusparseOperation_t to_cusparse_op(GpuOp op) {
  const auto op_ = (op == GpuOp::ConjTrans && !NumTraits<Scalar>::IsComplex) ? GpuOp::Trans : op;
  switch (op_) {
    case GpuOp::Trans:
      return CUSPARSE_OPERATION_TRANSPOSE;
    case GpuOp::ConjTrans:
      return CUSPARSE_OPERATION_CONJUGATE_TRANSPOSE;
    default:
      return CUSPARSE_OPERATION_NON_TRANSPOSE;
  }
}

// Bind without copying when the input already has the target sparse format
// and is compressed; otherwise convert/compress into `storage` and return
// that. Avoids a full host copy + format conversion on the common
// already-matching path. The SFINAE split (rather than plain overloading on
// `const SpMatType&`) is load-bearing: an exact-match overload would bind a
// converted temporary for other input types and return a dangling reference.
template <typename SpMatType, typename InputType, require_same_t<SpMatType, InputType> = 0>
const SpMatType& bind_sparse(const InputType& A, SpMatType& storage) {
  if (A.isCompressed()) return A;
  storage = A;
  storage.makeCompressed();
  return storage;
}

template <typename SpMatType, typename InputType, require_not_same_t<SpMatType, InputType> = 0>
const SpMatType& bind_sparse(const InputType& A, SpMatType& storage) {
  storage = A;
  storage.makeCompressed();
  return storage;
}

// Shared guard for the 32-bit index limits of the CUDA sparse libraries
// (cuSPARSE, cuDSS): matrix dimensions and nonzero count must fit StorageIndex.
template <typename StorageIndex>
inline void check_storage_index_bounds(Index rows, Index cols, Index nnz) {
  const Index max_storage_index = static_cast<Index>((std::numeric_limits<StorageIndex>::max)());
  eigen_assert(rows <= max_storage_index && cols <= max_storage_index && nnz <= max_storage_index &&
               "matrix dimensions or nonzeros exceed the index range supported by the CUDA sparse libraries");
  EIGEN_UNUSED_VARIABLE(rows);
  EIGEN_UNUSED_VARIABLE(cols);
  EIGEN_UNUSED_VARIABLE(nnz);
  EIGEN_UNUSED_VARIABLE(max_storage_index);
}

}  // namespace internal
}  // namespace gpu
}  // namespace Eigen

#endif  // EIGEN_GPU_CUSPARSE_SUPPORT_H
