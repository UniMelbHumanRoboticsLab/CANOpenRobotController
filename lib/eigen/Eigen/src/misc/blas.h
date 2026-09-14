// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_MISC_BLAS_H
#define EIGEN_MISC_BLAS_H

extern "C" {

/* EIGEN_BLAS_SYM applies EIGEN_BLAS_SYMBOL_SUFFIX; both come from Core/util/MKL_support.h, this header's only
 * includer. The declarations here must stay in step with the BLAS call sites under Core/products.
 */
#define BLASFUNC(FUNC) EIGEN_BLAS_SYM(FUNC)

/* Level 1 routines */

void BLASFUNC(saxpy)(const EIGEN_BLAS_INT *, const float *, const float *, const EIGEN_BLAS_INT *, float *,
                     const EIGEN_BLAS_INT *);
void BLASFUNC(daxpy)(const EIGEN_BLAS_INT *, const double *, const double *, const EIGEN_BLAS_INT *, double *,
                     const EIGEN_BLAS_INT *);
void BLASFUNC(caxpy)(const EIGEN_BLAS_INT *, const float *, const float *, const EIGEN_BLAS_INT *, float *,
                     const EIGEN_BLAS_INT *);
void BLASFUNC(zaxpy)(const EIGEN_BLAS_INT *, const double *, const double *, const EIGEN_BLAS_INT *, double *,
                     const EIGEN_BLAS_INT *);

/* Level 2 routines */

void BLASFUNC(sgemv)(const char *, const EIGEN_BLAS_INT *, const EIGEN_BLAS_INT *, const float *, const float *,
                     const EIGEN_BLAS_INT *, const float *, const EIGEN_BLAS_INT *, const float *, float *,
                     const EIGEN_BLAS_INT *);
void BLASFUNC(dgemv)(const char *, const EIGEN_BLAS_INT *, const EIGEN_BLAS_INT *, const double *, const double *,
                     const EIGEN_BLAS_INT *, const double *, const EIGEN_BLAS_INT *, const double *, double *,
                     const EIGEN_BLAS_INT *);
void BLASFUNC(cgemv)(const char *, const EIGEN_BLAS_INT *, const EIGEN_BLAS_INT *, const float *, const float *,
                     const EIGEN_BLAS_INT *, const float *, const EIGEN_BLAS_INT *, const float *, float *,
                     const EIGEN_BLAS_INT *);
void BLASFUNC(zgemv)(const char *, const EIGEN_BLAS_INT *, const EIGEN_BLAS_INT *, const double *, const double *,
                     const EIGEN_BLAS_INT *, const double *, const EIGEN_BLAS_INT *, const double *, double *,
                     const EIGEN_BLAS_INT *);

void BLASFUNC(strmv)(const char *, const char *, const char *, const EIGEN_BLAS_INT *, const float *,
                     const EIGEN_BLAS_INT *, float *, const EIGEN_BLAS_INT *);
void BLASFUNC(dtrmv)(const char *, const char *, const char *, const EIGEN_BLAS_INT *, const double *,
                     const EIGEN_BLAS_INT *, double *, const EIGEN_BLAS_INT *);
void BLASFUNC(ctrmv)(const char *, const char *, const char *, const EIGEN_BLAS_INT *, const float *,
                     const EIGEN_BLAS_INT *, float *, const EIGEN_BLAS_INT *);
void BLASFUNC(ztrmv)(const char *, const char *, const char *, const EIGEN_BLAS_INT *, const double *,
                     const EIGEN_BLAS_INT *, double *, const EIGEN_BLAS_INT *);

void BLASFUNC(ssymv)(const char *, const EIGEN_BLAS_INT *, const float *, const float *, const EIGEN_BLAS_INT *,
                     const float *, const EIGEN_BLAS_INT *, const float *, float *, const EIGEN_BLAS_INT *);
void BLASFUNC(dsymv)(const char *, const EIGEN_BLAS_INT *, const double *, const double *, const EIGEN_BLAS_INT *,
                     const double *, const EIGEN_BLAS_INT *, const double *, double *, const EIGEN_BLAS_INT *);

void BLASFUNC(chemv)(const char *, const EIGEN_BLAS_INT *, const float *, const float *, const EIGEN_BLAS_INT *,
                     const float *, const EIGEN_BLAS_INT *, const float *, float *, const EIGEN_BLAS_INT *);
void BLASFUNC(zhemv)(const char *, const EIGEN_BLAS_INT *, const double *, const double *, const EIGEN_BLAS_INT *,
                     const double *, const EIGEN_BLAS_INT *, const double *, double *, const EIGEN_BLAS_INT *);

/* Level 3 routines */

void BLASFUNC(sgemm)(const char *, const char *, const EIGEN_BLAS_INT *, const EIGEN_BLAS_INT *, const EIGEN_BLAS_INT *,
                     const float *, const float *, const EIGEN_BLAS_INT *, const float *, const EIGEN_BLAS_INT *,
                     const float *, float *, const EIGEN_BLAS_INT *);
void BLASFUNC(dgemm)(const char *, const char *, const EIGEN_BLAS_INT *, const EIGEN_BLAS_INT *, const EIGEN_BLAS_INT *,
                     const double *, const double *, const EIGEN_BLAS_INT *, const double *, const EIGEN_BLAS_INT *,
                     const double *, double *, const EIGEN_BLAS_INT *);
void BLASFUNC(cgemm)(const char *, const char *, const EIGEN_BLAS_INT *, const EIGEN_BLAS_INT *, const EIGEN_BLAS_INT *,
                     const float *, const float *, const EIGEN_BLAS_INT *, const float *, const EIGEN_BLAS_INT *,
                     const float *, float *, const EIGEN_BLAS_INT *);
void BLASFUNC(zgemm)(const char *, const char *, const EIGEN_BLAS_INT *, const EIGEN_BLAS_INT *, const EIGEN_BLAS_INT *,
                     const double *, const double *, const EIGEN_BLAS_INT *, const double *, const EIGEN_BLAS_INT *,
                     const double *, double *, const EIGEN_BLAS_INT *);

void BLASFUNC(strsm)(const char *, const char *, const char *, const char *, const EIGEN_BLAS_INT *,
                     const EIGEN_BLAS_INT *, const float *, const float *, const EIGEN_BLAS_INT *, float *,
                     const EIGEN_BLAS_INT *);
void BLASFUNC(dtrsm)(const char *, const char *, const char *, const char *, const EIGEN_BLAS_INT *,
                     const EIGEN_BLAS_INT *, const double *, const double *, const EIGEN_BLAS_INT *, double *,
                     const EIGEN_BLAS_INT *);
void BLASFUNC(ctrsm)(const char *, const char *, const char *, const char *, const EIGEN_BLAS_INT *,
                     const EIGEN_BLAS_INT *, const float *, const float *, const EIGEN_BLAS_INT *, float *,
                     const EIGEN_BLAS_INT *);
void BLASFUNC(ztrsm)(const char *, const char *, const char *, const char *, const EIGEN_BLAS_INT *,
                     const EIGEN_BLAS_INT *, const double *, const double *, const EIGEN_BLAS_INT *, double *,
                     const EIGEN_BLAS_INT *);

void BLASFUNC(strmm)(const char *, const char *, const char *, const char *, const EIGEN_BLAS_INT *,
                     const EIGEN_BLAS_INT *, const float *, const float *, const EIGEN_BLAS_INT *, float *,
                     const EIGEN_BLAS_INT *);
void BLASFUNC(dtrmm)(const char *, const char *, const char *, const char *, const EIGEN_BLAS_INT *,
                     const EIGEN_BLAS_INT *, const double *, const double *, const EIGEN_BLAS_INT *, double *,
                     const EIGEN_BLAS_INT *);
void BLASFUNC(ctrmm)(const char *, const char *, const char *, const char *, const EIGEN_BLAS_INT *,
                     const EIGEN_BLAS_INT *, const float *, const float *, const EIGEN_BLAS_INT *, float *,
                     const EIGEN_BLAS_INT *);
void BLASFUNC(ztrmm)(const char *, const char *, const char *, const char *, const EIGEN_BLAS_INT *,
                     const EIGEN_BLAS_INT *, const double *, const double *, const EIGEN_BLAS_INT *, double *,
                     const EIGEN_BLAS_INT *);

void BLASFUNC(ssymm)(const char *, const char *, const EIGEN_BLAS_INT *, const EIGEN_BLAS_INT *, const float *,
                     const float *, const EIGEN_BLAS_INT *, const float *, const EIGEN_BLAS_INT *, const float *,
                     float *, const EIGEN_BLAS_INT *);
void BLASFUNC(dsymm)(const char *, const char *, const EIGEN_BLAS_INT *, const EIGEN_BLAS_INT *, const double *,
                     const double *, const EIGEN_BLAS_INT *, const double *, const EIGEN_BLAS_INT *, const double *,
                     double *, const EIGEN_BLAS_INT *);

void BLASFUNC(ssyrk)(const char *, const char *, const EIGEN_BLAS_INT *, const EIGEN_BLAS_INT *, const float *,
                     const float *, const EIGEN_BLAS_INT *, const float *, float *, const EIGEN_BLAS_INT *);
void BLASFUNC(dsyrk)(const char *, const char *, const EIGEN_BLAS_INT *, const EIGEN_BLAS_INT *, const double *,
                     const double *, const EIGEN_BLAS_INT *, const double *, double *, const EIGEN_BLAS_INT *);

void BLASFUNC(chemm)(const char *, const char *, const EIGEN_BLAS_INT *, const EIGEN_BLAS_INT *, const float *,
                     const float *, const EIGEN_BLAS_INT *, const float *, const EIGEN_BLAS_INT *, const float *,
                     float *, const EIGEN_BLAS_INT *);
void BLASFUNC(zhemm)(const char *, const char *, const EIGEN_BLAS_INT *, const EIGEN_BLAS_INT *, const double *,
                     const double *, const EIGEN_BLAS_INT *, const double *, const EIGEN_BLAS_INT *, const double *,
                     double *, const EIGEN_BLAS_INT *);

void BLASFUNC(cherk)(const char *, const char *, const EIGEN_BLAS_INT *, const EIGEN_BLAS_INT *, const float *,
                     const float *, const EIGEN_BLAS_INT *, const float *, float *, const EIGEN_BLAS_INT *);
void BLASFUNC(zherk)(const char *, const char *, const EIGEN_BLAS_INT *, const EIGEN_BLAS_INT *, const double *,
                     const double *, const EIGEN_BLAS_INT *, const double *, double *, const EIGEN_BLAS_INT *);

#undef BLASFUNC
}

#endif
