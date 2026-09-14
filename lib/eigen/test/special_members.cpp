// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include "main.h"

#include <Eigen/Geometry>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SVD>
#include <Eigen/SparseCholesky>

static_assert(std::is_trivially_destructible<AlignedBox<double, 3>>::value,
              "AlignedBox should have a trivial destructor");
static_assert(std::is_trivially_destructible<Hyperplane<double, 3>>::value,
              "Hyperplane should have a trivial destructor");
static_assert(std::is_trivially_destructible<ParametrizedLine<double, 3>>::value,
              "ParametrizedLine should have a trivial destructor");

static_assert(std::is_move_constructible<SimplicialLLT<SparseMatrix<double>>>::value,
              "SimplicialLLT should be move constructible");
static_assert(std::is_move_constructible<ConjugateGradient<SparseMatrix<double>, Lower | Upper>>::value,
              "ConjugateGradient should be move constructible");
static_assert(std::is_move_constructible<BiCGSTAB<SparseMatrix<double>>>::value,
              "BiCGSTAB should be move constructible");

using StorageVal = internal::StorageVal<double, int>;
static_assert(std::is_copy_constructible<StorageVal>::value, "StorageVal should remain copy constructible");
static_assert(std::is_move_constructible<StorageVal>::value, "StorageVal should remain move constructible");
static_assert(!std::is_copy_assignable<StorageVal>::value, "StorageVal should not become copy assignable");
static_assert(!std::is_move_assignable<StorageVal>::value, "StorageVal should not become move assignable");

SparseMatrix<double> make_spd_matrix() {
  SparseMatrix<double> matrix(3, 3);
  std::vector<Triplet<double>> triplets;
  triplets.push_back(Triplet<double>(0, 0, 4.0));
  triplets.push_back(Triplet<double>(0, 1, -1.0));
  triplets.push_back(Triplet<double>(1, 0, -1.0));
  triplets.push_back(Triplet<double>(1, 1, 4.0));
  triplets.push_back(Triplet<double>(1, 2, -1.0));
  triplets.push_back(Triplet<double>(2, 1, -1.0));
  triplets.push_back(Triplet<double>(2, 2, 3.0));
  matrix.setFromTriplets(triplets.begin(), triplets.end());
  matrix.makeCompressed();
  return matrix;
}

template <typename Solver, typename MatrixType>
Solver move_computed_solver(const MatrixType& matrix) {
  Solver source;
  source.compute(matrix);
  VERIFY_IS_EQUAL(source.info(), Success);
  Solver moved(std::move(source));
  return moved;
}

void test_move_initialized_bdcsvd() {
  MatrixXd matrix(3, 3);
  matrix << 4.0, 1.0, 0.0, 1.0, 3.0, 1.0, 0.0, 1.0, 2.0;
  VectorXd expected(3);
  expected << 1.0, -2.0, 3.0;
  const VectorXd rhs = matrix * expected;

  typedef BDCSVD<MatrixXd, ComputeThinU | ComputeThinV> Solver;
  Solver solver = move_computed_solver<Solver>(matrix);
  const VectorXd actual = solver.solve(rhs);

  VERIFY_IS_EQUAL(solver.info(), Success);
  VERIFY_IS_APPROX(actual, expected);
}

void test_move_initialized_simplicial_llt() {
  const SparseMatrix<double> matrix = make_spd_matrix();
  VectorXd expected(3);
  expected << 1.0, -2.0, 3.0;
  const VectorXd rhs = matrix * expected;

  typedef SimplicialLLT<SparseMatrix<double>> Solver;
  Solver solver = move_computed_solver<Solver>(matrix);
  const VectorXd actual = solver.solve(rhs);

  VERIFY_IS_EQUAL(solver.info(), Success);
  VERIFY_IS_APPROX(actual, expected);
}

template <typename Solver>
void test_move_initialized_iterative_solver() {
  const SparseMatrix<double> matrix = make_spd_matrix();
  VectorXd expected(3);
  expected << 1.0, -2.0, 3.0;
  const VectorXd rhs = matrix * expected;

  Solver solver = move_computed_solver<Solver>(matrix);
  solver.setMaxIterations(100);
  solver.setTolerance(NumTraits<double>::epsilon() * 100);
  const VectorXd actual = solver.solve(rhs);

  VERIFY_IS_EQUAL(solver.info(), Success);
  VERIFY_IS_APPROX(actual, expected);
}

EIGEN_DECLARE_TEST(special_members) {
  CALL_SUBTEST_1(test_move_initialized_bdcsvd());
  CALL_SUBTEST_2(test_move_initialized_simplicial_llt());
  CALL_SUBTEST_3((test_move_initialized_iterative_solver<ConjugateGradient<SparseMatrix<double>, Lower | Upper>>()));
  CALL_SUBTEST_4((test_move_initialized_iterative_solver<BiCGSTAB<SparseMatrix<double>>>()));
}
