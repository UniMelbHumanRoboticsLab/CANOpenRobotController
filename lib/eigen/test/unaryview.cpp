// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2021 Andrew Johnson <andrew.johnson@arjohnsonau.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include <type_traits>
#include "main.h"

struct UnaryViewBox {
  explicit UnaryViewBox(double v = 0.) : value(v) {}

  double value;
};

namespace Eigen {
template <>
struct NumTraits<UnaryViewBox> : NumTraits<double> {
  typedef UnaryViewBox Real;
  typedef UnaryViewBox NonInteger;
  typedef UnaryViewBox Nested;
  enum {
    IsComplex = 0,
    IsInteger = 0,
    IsSigned = 1,
    RequireInitialization = 1,
    ReadCost = NumTraits<double>::ReadCost,
    AddCost = NumTraits<double>::AddCost,
    MulCost = NumTraits<double>::MulCost
  };
};

namespace internal {
struct unaryview_box_ref_op {
  EIGEN_EMPTY_STRUCT_CTOR(unaryview_box_ref_op)

  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE double& operator()(const UnaryViewBox& box) const {
    return numext::real_ref(*const_cast<double*>(&box.value));
  }
};

template <>
struct functor_traits<unaryview_box_ref_op> {
  enum { Cost = 0, PacketAccess = false };
};
}  // namespace internal
}  // namespace Eigen

template <int OuterStride, int InnerStride, typename VectorType>
void unaryview_stride(const VectorType& m) {
  typedef typename VectorType::Scalar Scalar;
  Index rows = m.rows();
  Index cols = m.cols();
  VectorType vec = VectorType::Random(rows, cols);

  struct view_op {
    EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const Scalar& operator()(const Scalar& v) const { return v; }
  };

  CwiseUnaryView<view_op, VectorType, Stride<OuterStride, InnerStride>> vec_view(vec);
  VERIFY(vec_view.outerStride() == (OuterStride == 0 ? 0 : OuterStride));
  VERIFY(vec_view.innerStride() == (InnerStride == 0 ? 1 : InnerStride));
}

void test_mutable_unaryview() {
  struct Vec3 {
    double x;
    double y;
    double z;
  };

  Eigen::Vector<Vec3, 3> m;
  auto x_view = m.unaryViewExpr([](Vec3& v) -> double& { return v.x; });
  auto y_view = m.unaryViewExpr([](Vec3& v) -> double& { return v.y; });
  auto z_view = m.unaryViewExpr([](Vec3& v) -> double& { return v.z; });

  x_view.setConstant(1);
  y_view.setConstant(2);
  z_view.setConstant(3);

  for (int i = 0; i < m.size(); ++i) {
    VERIFY_IS_EQUAL(m(i).x, 1);
    VERIFY_IS_EQUAL(m(i).y, 2);
    VERIFY_IS_EQUAL(m(i).z, 3);
  }
}

void test_unaryview_pointer_result() {
  struct Payload {
    double value;
  };

  struct Holder {
    Payload* ptr;
  };

  struct pointer_view_op {
    Payload*& operator()(Holder& x) const { return x.ptr; }
    Payload* const& operator()(const Holder& x) const { return x.ptr; }
  };

  Payload p0 = {0.0};
  Payload p1 = {1.0};
  Payload p2 = {2.0};

  Eigen::Vector<Holder, 2> holders;
  holders(0).ptr = &p0;
  holders(1).ptr = &p1;

  auto view = holders.unaryViewExpr(pointer_view_op());
  VERIFY((std::is_same<typename Eigen::internal::traits<decltype(view)>::Scalar, Payload*>::value));

  Eigen::Vector<Payload*, 2> out = view;
  VERIFY_IS_EQUAL(out(0), &p0);
  VERIFY_IS_EQUAL(out(1), &p1);

  view(0) = &p2;
  VERIFY_IS_EQUAL(holders(0).ptr, &p2);
}

void test_unaryview_solve() {
  // Random upper-triangular system.
  Eigen::MatrixXd A = Eigen::MatrixXd::Random(5, 5);
  A.triangularView<Eigen::Lower>().setZero();
  A.diagonal().setRandom();
  Eigen::VectorXd b = Eigen::VectorXd::Random(5);

  struct trivial_view_op {
    double& operator()(double& x) const { return x; }
    const double& operator()(const double& x) const { return x; }
  };

  // Non-const view:
  {
    auto b_view = b.unaryViewExpr(trivial_view_op());
    b_view(0) = 1;  // Allows modification.
    Eigen::VectorXd x = A.triangularView<Eigen::Upper>().solve(b_view);
    VERIFY_IS_APPROX(A * x, b);
  }

  // Const view:
  {
    const auto b_view = b.unaryViewExpr(trivial_view_op());
    Eigen::VectorXd x = A.triangularView<Eigen::Upper>().solve(b_view);
    VERIFY_IS_APPROX(A * x, b);
  }

  // Non-const view of const matrix:
  {
    const Eigen::VectorXd const_b = b;
    auto b_view = const_b.unaryViewExpr(trivial_view_op());
    Eigen::VectorXd x = A.triangularView<Eigen::Upper>().solve(b_view);
    VERIFY_IS_APPROX(A * x, b);
  }

  // Const view of const matrix:
  {
    const Eigen::VectorXd const_b = b;
    const auto b_view = const_b.unaryViewExpr(trivial_view_op());
    Eigen::VectorXd x = A.triangularView<Eigen::Upper>().solve(b_view);
    VERIFY_IS_APPROX(A * x, b);
  }

  // Eigen::MatrixXd out =
  //       mat_in.real()
  //             .triangularView<Eigen::Upper>()
  //             .solve(mat_in.unaryViewExpr([&](const auto& x){ return std::real(x); }));
}

void test_unaryview_direct_access_product() {
  typedef Matrix<UnaryViewBox, Dynamic, Dynamic> BoxMatrix;

  BoxMatrix a(4, 4), b(4, 4);
  MatrixXd a_values(4, 4), b_values(4, 4);

  for (Index j = 0; j < a.cols(); ++j) {
    for (Index i = 0; i < a.rows(); ++i) {
      a_values(i, j) = 1. + 0.5 * i + 0.25 * j;
      b_values(i, j) = -0.75 + 0.125 * i - 0.5 * j;
      a(i, j).value = a_values(i, j);
      b(i, j).value = b_values(i, j);
    }
  }

  CwiseUnaryView<internal::unaryview_box_ref_op, BoxMatrix> a_view(a);
  CwiseUnaryView<internal::unaryview_box_ref_op, BoxMatrix> b_view(b);

  const auto& const_a_view = a_view;
  const auto& const_b_view = b_view;
  VERIFY_IS_EQUAL(&const_a_view.coeffRef(0, 0), &a(0, 0).value);
  VERIFY_IS_EQUAL(&const_b_view.coeffRef(0, 0), &b(0, 0).value);

  MatrixXd actual = a_view * b_view;
  VERIFY_IS_APPROX(actual, a_values * b_values);
}

EIGEN_DECLARE_TEST(unaryviewstride) {
  CALL_SUBTEST_1((unaryview_stride<1, 2>(MatrixXf())));
  CALL_SUBTEST_1((unaryview_stride<0, 0>(MatrixXf())));
  CALL_SUBTEST_2((unaryview_stride<1, 2>(VectorXf())));
  CALL_SUBTEST_2((unaryview_stride<0, 0>(VectorXf())));
  CALL_SUBTEST_3((unaryview_stride<1, 2>(RowVectorXf())));
  CALL_SUBTEST_3((unaryview_stride<0, 0>(RowVectorXf())));
  CALL_SUBTEST_4(test_mutable_unaryview());
  CALL_SUBTEST_4(test_unaryview_pointer_result());
  CALL_SUBTEST_4(test_unaryview_solve());
  CALL_SUBTEST_4(test_unaryview_direct_access_product());
}
