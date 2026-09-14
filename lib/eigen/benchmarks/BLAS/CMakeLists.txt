# Benchmarks for Eigen's built-in BLAS implementation.
# Compiles the Eigen BLAS sources directly into the benchmark executable
# so there is no external BLAS dependency.
# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

set(EIGEN_BLAS_SRCS
  ${EIGEN_SOURCE_DIR}/blas/single.cpp
  ${EIGEN_SOURCE_DIR}/blas/double.cpp
  ${EIGEN_SOURCE_DIR}/blas/complex_single.cpp
  ${EIGEN_SOURCE_DIR}/blas/complex_double.cpp
  ${EIGEN_SOURCE_DIR}/blas/xerbla.cpp
  ${EIGEN_SOURCE_DIR}/blas/lsame.cpp
  ${EIGEN_SOURCE_DIR}/blas/complexdots.cpp
)

eigen_add_benchmark(bench_blas bench_blas.cpp)
target_sources(bench_blas PRIVATE ${EIGEN_BLAS_SRCS})
target_include_directories(bench_blas PRIVATE ${EIGEN_SOURCE_DIR}/blas)
