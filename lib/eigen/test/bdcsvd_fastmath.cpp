// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include "main.h"
#include <Eigen/SVD>

EIGEN_DECLARE_TEST(bdcsvd_fastmath) {
  typedef Matrix<double, 6, 6> Matrix6d;
  const double kTolerance = 16 * Matrix6d::RowsAtCompileTime * NumTraits<double>::epsilon();

  // A finite singular-vector coefficient grows to about 2^570. Its ordinary
  // squared norm overflows, and GCC's -ffast-math assumes isfinite() is true.
  Matrix6d matrix = Matrix6d::Zero();
  using std::ldexp;
  matrix.diagonal() << 0.0, 0.0, ldexp(1.0, -487), -1.0, 0.0, 0.0;
  matrix.diagonal(1) << 0.0, ldexp(1.0, -453), -ldexp(1.0, -627), 0.0, 0.0;

  BDCSVD<Matrix6d, ComputeFullU | ComputeFullV> svd;
  svd.setSwitchSize(3);
  svd.compute(matrix);

  VERIFY(svd.info() == Success);
  const Matrix6d reconstruction = svd.matrixU() * svd.singularValues().asDiagonal() * svd.matrixV().transpose();
  VERIFY((reconstruction - matrix).stableNorm() <= kTolerance * matrix.stableNorm());

  const Matrix6d identity = Matrix6d::Identity();
  VERIFY((svd.matrixU().transpose() * svd.matrixU() - identity).stableNorm() <= kTolerance);
  VERIFY((svd.matrixV().transpose() * svd.matrixV() - identity).stableNorm() <= kTolerance);
}
