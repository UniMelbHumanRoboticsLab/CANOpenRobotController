// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>
#include <Eigen/SVD>

#include <complex>
#include <cstdlib>

template <typename Scalar, int Options>
static void BM_JacobiSVDRotations(benchmark::State& state) {
  using Matrix = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
  using RealScalar = typename Eigen::NumTraits<Scalar>::Real;
  const Eigen::Index rows = state.range(0);
  const Eigen::Index cols = state.range(1);
  Matrix matrix(rows, cols);
  std::srand(5489);
  matrix.setRandom();
  benchmark::DoNotOptimize(matrix.data());
  benchmark::ClobberMemory();

  Eigen::JacobiSVD<Matrix, Eigen::ComputeThinU | Eigen::ComputeThinV> reference(matrix);
  const RealScalar tolerance = RealScalar(128 * rows) * Eigen::NumTraits<RealScalar>::epsilon();
  const Matrix reconstructed =
      reference.matrixU() * reference.singularValues().asDiagonal() * reference.matrixV().adjoint();
  if (reference.info() != Eigen::Success || !((matrix - reconstructed).norm() <= tolerance * matrix.norm()) ||
      !((reference.matrixU().adjoint() * reference.matrixU() - Matrix::Identity(cols, cols)).norm() <= tolerance) ||
      !((reference.matrixV().adjoint() * reference.matrixV() - Matrix::Identity(cols, cols)).norm() <= tolerance)) {
    state.SkipWithError("JacobiSVD failed reconstruction or unitarity check");
    return;
  }

  Eigen::JacobiSVD<Matrix, Options> svd(rows, cols);
  svd.compute(matrix);
  if (svd.info() != Eigen::Success ||
      !((svd.singularValues() - reference.singularValues()).norm() <= tolerance * matrix.norm())) {
    state.SkipWithError("JacobiSVD failed singular-value check");
    return;
  }
  for (auto _ : state) {
    benchmark::ClobberMemory();
    svd.compute(matrix);
    benchmark::DoNotOptimize(svd.singularValues().data());
    EIGEN_IF_CONSTEXPR (Options != 0) {
      benchmark::DoNotOptimize(svd.matrixU().data());
      benchmark::DoNotOptimize(svd.matrixV().data());
    }
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations());
}

#define JACOBI_ROTATION_SHAPES ->Args({3, 3})->Args({16, 16})->Args({64, 64})->Args({64, 16})

BENCHMARK_TEMPLATE(BM_JacobiSVDRotations, float, 0) JACOBI_ROTATION_SHAPES;
BENCHMARK_TEMPLATE(BM_JacobiSVDRotations, double, 0) JACOBI_ROTATION_SHAPES;
BENCHMARK_TEMPLATE(BM_JacobiSVDRotations, std::complex<float>, 0) JACOBI_ROTATION_SHAPES;
BENCHMARK_TEMPLATE(BM_JacobiSVDRotations, std::complex<double>, 0) JACOBI_ROTATION_SHAPES;
BENCHMARK_TEMPLATE(BM_JacobiSVDRotations, float, Eigen::ComputeThinU | Eigen::ComputeThinV) JACOBI_ROTATION_SHAPES;
BENCHMARK_TEMPLATE(BM_JacobiSVDRotations, double, Eigen::ComputeThinU | Eigen::ComputeThinV) JACOBI_ROTATION_SHAPES;
BENCHMARK_TEMPLATE(BM_JacobiSVDRotations, std::complex<float>, Eigen::ComputeThinU | Eigen::ComputeThinV)
JACOBI_ROTATION_SHAPES;
BENCHMARK_TEMPLATE(BM_JacobiSVDRotations, std::complex<double>, Eigen::ComputeThinU | Eigen::ComputeThinV)
JACOBI_ROTATION_SHAPES;

#undef JACOBI_ROTATION_SHAPES
