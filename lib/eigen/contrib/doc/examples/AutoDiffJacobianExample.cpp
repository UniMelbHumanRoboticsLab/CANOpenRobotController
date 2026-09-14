// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <contrib/Eigen/AutoDiff>

struct ExampleFunctor {
  typedef Eigen::Vector2d InputType;
  typedef Eigen::Vector2d ValueType;

  template <typename Scalar>
  void operator()(const Eigen::Matrix<Scalar, 2, 1>& x, Eigen::Matrix<Scalar, 2, 1>* y) const {
    using std::sin;
    (*y)[0] = x[0] * x[0] + x[1];
    (*y)[1] = x[0] * sin(x[1]);
  }
};

int main() {
  Eigen::AutoDiffJacobian<ExampleFunctor> functor;
  Eigen::Vector2d x;
  Eigen::Vector2d y;
  Eigen::AutoDiffJacobian<ExampleFunctor>::JacobianType jacobian;

  x << 2.0, 0.5;
  functor(x, &y, &jacobian);

  std::cout << "value:\n" << y << "\n\n";
  std::cout << "jacobian:\n" << jacobian << "\n";
}
