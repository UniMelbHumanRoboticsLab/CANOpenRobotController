// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

// Consumer program shared by the build-system integration tests.  It exists to
// prove that the Eigen a scenario assembled is usable, so it exercises a
// header, a template instantiation, and a computed result rather than only
// including <Eigen/Dense>.  The marker on the last line is what the driver
// matches; a run that ends early fails the scenario.

#include <Eigen/Dense>

#include <cstdio>

int main() {
  Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
  m(0, 1) = 2.0;
  m(2, 0) = -1.0;

  // Unit triangular up to the (2,0) entry, so the determinant is exactly one
  // in binary floating point and needs no tolerance.
  if (m.determinant() != 1.0) {
    std::printf("determinant was %f, expected 1\n", m.determinant());
    return 1;
  }

  Eigen::Vector3d v = m * Eigen::Vector3d(1.0, 1.0, 1.0);
  if (v(0) != 3.0) {
    std::printf("product gave %f, expected 3\n", v(0));
    return 1;
  }

  std::printf("eigen-ok\n");
  return 0;
}
