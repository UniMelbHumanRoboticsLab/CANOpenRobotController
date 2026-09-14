// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <Eigen/Core>
#include <contrib/Eigen/SpecialFunctions>
#include <iostream>
int main() {
  Eigen::Array4d v(0.5, 10, 0, -1);
  std::cout << v.lgamma() << std::endl;
}
