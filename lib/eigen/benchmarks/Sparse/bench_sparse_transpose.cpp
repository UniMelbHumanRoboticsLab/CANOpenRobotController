// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/Sparse>
#include <set>

using namespace Eigen;

typedef double Scalar;
typedef SparseMatrix<Scalar> SpMat;

static void fillMatrix(float density, int rows, int cols, SpMat& dst) {
  dst.resize(rows, cols);
  dst.reserve(static_cast<int>(rows * cols * density));
  for (int j = 0; j < cols; ++j) {
    for (int i = 0; i < rows; ++i) {
      if (internal::random<float>(0, 1) < density) {
        dst.insert(i, j) = internal::random<Scalar>();
      }
    }
  }
  dst.makeCompressed();
}

static void BM_SparseTranspose(benchmark::State& state) {
  int n = state.range(0);
  float density = state.range(1) / 10000.0f;
  SpMat sm(n, n), result(n, n);
  fillMatrix(density, n, n, sm);
  for (auto _ : state) {
    result = sm.transpose();
    benchmark::DoNotOptimize(result.valuePtr());
  }
  state.counters["nnz"] = sm.nonZeros();
  state.counters["density%"] = density * 100;
}

// Args: {size, density*10000}: 1%, 0.5%, 0.1%, 0.04%
BENCHMARK(BM_SparseTranspose)->ArgsProduct({{1000, 10000}, {100, 50, 10, 4}});
