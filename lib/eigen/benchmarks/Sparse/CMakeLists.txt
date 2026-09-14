# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

eigen_add_benchmark(bench_spmm bench_spmm.cpp)
eigen_add_benchmark(bench_sparse_transpose bench_sparse_transpose.cpp)
eigen_add_benchmark(bench_sparseview_assign bench_sparseview_assign.cpp)
eigen_add_benchmark(bench_sparse_solvers bench_sparse_solvers.cpp)
eigen_add_benchmark(bench_sparseqr_lookahead bench_sparseqr_lookahead.cpp)
eigen_add_benchmark(bench_threaded_spmv bench_threaded_spmv.cpp LIBRARIES Threads::Threads)
eigen_add_benchmark(bench_block_sparse bench_block_sparse.cpp)
