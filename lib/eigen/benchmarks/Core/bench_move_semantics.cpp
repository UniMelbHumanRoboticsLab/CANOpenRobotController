// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Core>
#include "../../test/MovableScalar.h"
#include <utility>

template <typename MatrixType>
void copy_matrix(MatrixType& m) {
  MatrixType tmp(m);
  m = tmp;
}

template <typename MatrixType>
void move_matrix(MatrixType&& m) {
  MatrixType tmp(std::move(m));
  m = std::move(tmp);
}

template <typename Scalar>
static void BM_CopySemantics(benchmark::State& state) {
  using MatrixType = Eigen::Matrix<Eigen::MovableScalar<Scalar>, 1, 10>;
  MatrixType data = MatrixType::Random().eval();
  for (auto _ : state) {
    copy_matrix(data);
    benchmark::DoNotOptimize(data.data());
  }
}

template <typename Scalar>
static void BM_MoveSemantics(benchmark::State& state) {
  using MatrixType = Eigen::Matrix<Eigen::MovableScalar<Scalar>, 1, 10>;
  MatrixType data = MatrixType::Random().eval();
  for (auto _ : state) {
    move_matrix(std::move(data));
    benchmark::DoNotOptimize(data.data());
  }
}

BENCHMARK(BM_CopySemantics<float>);
BENCHMARK(BM_MoveSemantics<float>);
BENCHMARK(BM_CopySemantics<double>);
BENCHMARK(BM_MoveSemantics<double>);
