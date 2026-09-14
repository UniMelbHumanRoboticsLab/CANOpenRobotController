// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <contrib/Eigen/AutoDiff>

template <typename Scalar>
Scalar f(const Scalar& x, const Scalar& y) {
  using std::sin;
  return x * x * y + sin(y);
}

int main() {
  typedef Eigen::AutoDiffScalar<Eigen::Vector2d> ADScalar;

  ADScalar x(2.0, Eigen::Vector2d::UnitX());
  ADScalar y(0.5, Eigen::Vector2d::UnitY());
  ADScalar z = f(x, y);

  std::cout << "f(x,y) = " << z.value() << "\n";
  std::cout << "df/dx  = " << z.derivatives()[0] << "\n";
  std::cout << "df/dy  = " << z.derivatives()[1] << "\n";
}
