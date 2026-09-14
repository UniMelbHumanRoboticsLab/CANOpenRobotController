// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <iostream>
#include <Eigen/Dense>

int main() {
  Eigen::Matrix4d m;
  m.resize(4, 4);  // no operation
  std::cout << "The matrix m is of size " << m.rows() << "x" << m.cols() << std::endl;
}
