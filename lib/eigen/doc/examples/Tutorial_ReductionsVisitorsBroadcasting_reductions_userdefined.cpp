// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

struct GreaterAbs {
  double operator()(double a, double b) const {
    using std::abs;
    return abs(a) < abs(b) ? b : a;
  }
};

int main() {
  Eigen::MatrixXd m(2, 3);
  m << -2, 1, 4, 3, -5, 2;

  std::cout << "matrix:\n" << m << "\n\n";
  std::cout << "largest absolute value: " << m.redux(GreaterAbs()) << "\n";
  std::cout << "column-wise:\n" << m.colwise().redux(GreaterAbs()) << "\n";
}
