// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 The Eigen Authors
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>

#include <Eigen/OrderingMethods>
#include <Eigen/SparseQR>
#include <vector>

using namespace Eigen;

typedef double Scalar;
typedef SparseMatrix<Scalar, ColMajor> SpMat;

static SpMat makeWeakPivotMatrix(Index rows) {
  eigen_assert(rows >= 2);
  std::vector<Triplet<Scalar> > triplets;
  triplets.reserve(static_cast<std::size_t>(2 * rows));

  triplets.emplace_back(0, 0, 1.0);
  triplets.emplace_back(0, 1, 1.0);
  triplets.emplace_back(1, 1, 5e-13);
  for (Index row = 1; row < rows; ++row) {
    triplets.emplace_back(row, row + 1, 1.0);
  }

  SpMat matrix(rows, rows + 1);
  matrix.setFromTriplets(triplets.begin(), triplets.end());
  matrix.makeCompressed();
  return matrix;
}

static SpMat makeRegularBandedMatrix(Index rows) {
  std::vector<Triplet<Scalar> > triplets;
  triplets.reserve(static_cast<std::size_t>(3 * rows));
  for (Index row = 0; row < rows; ++row) {
    triplets.emplace_back(row, row, 2.0);
    if (row + 1 < rows) {
      triplets.emplace_back(row + 1, row, -0.5);
      triplets.emplace_back(row, row + 1, 0.25);
    }
  }

  SpMat matrix(rows, rows);
  matrix.setFromTriplets(triplets.begin(), triplets.end());
  matrix.makeCompressed();
  return matrix;
}

static void BM_SparseQRWeakPivotLookAhead(benchmark::State& state) {
  const Index rows = state.range(0);
  const SpMat matrix = makeWeakPivotMatrix(rows);

  for (auto _ : state) {
    SparseQR<SpMat, NaturalOrdering<int> > solver;
    solver.compute(matrix);
    benchmark::DoNotOptimize(solver.rank());
    benchmark::DoNotOptimize(solver.lastPivotLookAheadSkipped());
  }
  state.SetItemsProcessed(state.iterations() * rows);
}

static void BM_SparseQRRegularBanded(benchmark::State& state) {
  const Index rows = state.range(0);
  const SpMat matrix = makeRegularBandedMatrix(rows);

  for (auto _ : state) {
    SparseQR<SpMat, NaturalOrdering<int> > solver;
    solver.compute(matrix);
    benchmark::DoNotOptimize(solver.rank());
  }
  state.SetItemsProcessed(state.iterations() * rows);
}

BENCHMARK(BM_SparseQRWeakPivotLookAhead)->Arg(8)->Arg(32)->Arg(64)->Arg(128);
BENCHMARK(BM_SparseQRRegularBanded)->Arg(8)->Arg(32)->Arg(64)->Arg(128);
