/*
 Copyright (c) 2011, Intel Corporation. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of Intel Corporation nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ********************************************************************************
 *   Content : Eigen bindings to Intel(R) MKL
 *   Scalar and index types of the external BLAS/LAPACK interface, split out of MKL_support.h
 ********************************************************************************
*/
// SPDX-License-Identifier: BSD-3-Clause

#ifndef EIGEN_BLAS_TYPES_H
#define EIGEN_BLAS_TYPES_H

/* Scalar and index types of the external BLAS/LAPACK interface. These are used pervasively by the BLAS and LAPACKE
 * backends, so they live in a backend-neutral header rather than in MKL_support.h. BlasIndex follows the integer
 * width selected there (MKL_INT for MKL, EIGEN_BLAS_INT otherwise), so this header must be included after
 * MKL_support.h has set those up; MKL_support.h includes it for that reason.
 */

// IWYU pragma: private
#include "../InternalHeaderCheck.h"

namespace Eigen {

using dcomplex = std::complex<double>;
using scomplex = std::complex<float>;

#if defined(EIGEN_USE_MKL)
using BlasIndex = MKL_INT;
// Plain static_assert (not EIGEN_STATIC_ASSERT): like the LAPACKE/BLAS cross-check it must not be suppressible.
#if defined(EIGEN_64BIT_BLAS)
static_assert(sizeof(MKL_INT) == 8,
              "EIGEN_64BIT_BLAS is defined but MKL_INT is 32-bit. Define MKL_ILP64 and link the MKL *_ilp64 "
              "libraries, or undefine EIGEN_64BIT_BLAS.");
#else
static_assert(sizeof(MKL_INT) == 4,
              "MKL_INT is 64-bit but EIGEN_64BIT_BLAS is not defined. Define EIGEN_64BIT_BLAS to match MKL_ILP64, or "
              "link the MKL *_lp64 libraries.");
#endif
#else
using BlasIndex = EIGEN_BLAS_INT;
#endif

}  // end namespace Eigen

#endif  // EIGEN_BLAS_TYPES_H
