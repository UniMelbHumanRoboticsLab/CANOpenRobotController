# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

find_package(Threads REQUIRED)
eigen_add_benchmark(bench_fork_join bench_fork_join.cpp LIBRARIES Threads::Threads)
eigen_add_benchmark(bench_schedule bench_schedule.cpp LIBRARIES Threads::Threads)
