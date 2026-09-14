# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

eigen_add_benchmark(bench_blocking_sizes bench_blocking_sizes.cpp)
eigen_add_benchmark(bench_aocl bench_aocl.cpp)

add_executable(print_blocking print_blocking.cpp)
target_include_directories(print_blocking PRIVATE ${EIGEN_SOURCE_DIR})
target_compile_options(print_blocking PRIVATE -O3 -DNDEBUG)
