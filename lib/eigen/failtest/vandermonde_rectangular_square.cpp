// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include "../contrib/Eigen/StructuredMatrices"

#ifdef EIGEN_SHOULD_FAIL_TO_BUILD
constexpr int Cols = 3;
#else
constexpr int Cols = 2;
#endif

int main() {
  Eigen::Vector2d nodes;
  nodes << 0.0, 1.0;
  Eigen::Vandermonde<double, 2, Cols> vandermonde(nodes);
  return static_cast<int>(vandermonde.cols());
}
