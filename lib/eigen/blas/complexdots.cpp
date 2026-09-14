// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

// C++ replacements for the f2c complex dot product wrappers.
// These are thin wrappers around the worker functions (cdotcw_, etc.)
// defined in level1_cplx_impl.h.
//
// Note: blas.h declares these as void, but gfortran expects complex functions
// to return by value. We define the correct signatures here and do not include
// blas.h to avoid the conflicting declarations.

#if defined(_WIN32)
#if defined(EIGEN_BLAS_BUILD_DLL)
#define EIGEN_BLAS_CDOT_API __declspec(dllexport)
#else
#define EIGEN_BLAS_CDOT_API
#endif
#elif ((defined(__GNUC__) && __GNUC__ >= 4) || defined(__clang__)) && defined(EIGEN_BLAS_BUILD_DLL)
#define EIGEN_BLAS_CDOT_API __attribute__((visibility("default")))
#else
#define EIGEN_BLAS_CDOT_API
#endif

extern "C" {

// Worker function declarations (defined in level1_cplx_impl.h via complex_single.cpp / complex_double.cpp).
void cdotcw_(int *n, float *cx, int *incx, float *cy, int *incy, float *res);
void cdotuw_(int *n, float *cx, int *incx, float *cy, int *incy, float *res);
void zdotcw_(int *n, double *cx, int *incx, double *cy, int *incy, double *res);
void zdotuw_(int *n, double *cx, int *incx, double *cy, int *incy, double *res);

// POD complex types for C-compatible return values (matches Fortran complex layout).
struct eigen_blas_complex_float {
  float r, i;
};
struct eigen_blas_complex_double {
  double r, i;
};

// CDOTC computes the conjugated dot product of two single-precision complex vectors.
EIGEN_BLAS_CDOT_API eigen_blas_complex_float cdotc_(int *n, float *cx, int *incx, float *cy, int *incy) {
  eigen_blas_complex_float res = {0.0f, 0.0f};
  cdotcw_(n, cx, incx, cy, incy, &res.r);
  return res;
}

// CDOTU computes the unconjugated dot product of two single-precision complex vectors.
EIGEN_BLAS_CDOT_API eigen_blas_complex_float cdotu_(int *n, float *cx, int *incx, float *cy, int *incy) {
  eigen_blas_complex_float res = {0.0f, 0.0f};
  cdotuw_(n, cx, incx, cy, incy, &res.r);
  return res;
}

// ZDOTC computes the conjugated dot product of two double-precision complex vectors.
EIGEN_BLAS_CDOT_API eigen_blas_complex_double zdotc_(int *n, double *cx, int *incx, double *cy, int *incy) {
  eigen_blas_complex_double res = {0.0, 0.0};
  zdotcw_(n, cx, incx, cy, incy, &res.r);
  return res;
}

// ZDOTU computes the unconjugated dot product of two double-precision complex vectors.
EIGEN_BLAS_CDOT_API eigen_blas_complex_double zdotu_(int *n, double *cx, int *incx, double *cy, int *incy) {
  eigen_blas_complex_double res = {0.0, 0.0};
  zdotuw_(n, cx, incx, cy, incy, &res.r);
  return res;
}

}  // extern "C"
