// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include "../contrib/Eigen/StructuredMatrices"

#ifdef EIGEN_SHOULD_FAIL_TO_BUILD
using Scalar = int;
#else
using Scalar = double;
#endif

int main() {
  Eigen::Matrix<Scalar, 2, 1> nodes;
  nodes << Scalar(1), Scalar(2);
  Eigen::Vandermonde<Scalar, 2, 2> vandermonde(nodes);
  return static_cast<int>(vandermonde.rows());
}
