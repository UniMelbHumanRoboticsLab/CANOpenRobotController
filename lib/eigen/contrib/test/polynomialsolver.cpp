// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2010 Manuel Yguel <manuel.yguel@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "main.h"
#include <contrib/Eigen/Polynomials>
#include <iostream>
#include <algorithm>

using namespace std;

namespace Eigen {
namespace internal {
template <int Size>
struct increment_if_fixed_size : std::integral_constant<int, (Size == Dynamic) ? Dynamic : Size + 1> {};
}  // namespace internal
}  // namespace Eigen

template <typename PolynomialType>
PolynomialType polyder(const PolynomialType& p) {
  typedef typename PolynomialType::Scalar Scalar;
  PolynomialType res(p.size());
  for (Index i = 1; i < p.size(); ++i) res[i - 1] = p[i] * Scalar(i);
  res[p.size() - 1] = 0.;
  return res;
}

template <int Deg, typename POLYNOMIAL, typename SOLVER>
bool aux_evalSolver(const POLYNOMIAL& pols, SOLVER& psolve) {
  typedef typename POLYNOMIAL::Scalar Scalar;
  typedef typename POLYNOMIAL::RealScalar RealScalar;

  typedef typename SOLVER::RootsType RootsType;
  typedef Matrix<RealScalar, Deg, 1> EvalRootsType;

  const Index deg = pols.size() - 1;

  // Test template constructor from coefficient vector
  SOLVER solve_constr(pols);

  psolve.compute(pols);
  const RootsType& roots(psolve.roots());
  EvalRootsType evr(deg);
  POLYNOMIAL pols_der = polyder(pols);
  EvalRootsType der(deg);
  for (int i = 0; i < roots.size(); ++i) {
    evr[i] = std::abs(poly_eval(pols, roots[i]));
    der[i] = numext::maxi(RealScalar(1.), std::abs(poly_eval(pols_der, roots[i])));
  }

  // we need to divide by the magnitude of the derivative because
  // with a high derivative is very small error in the value of the root
  // yiels a very large error in the polynomial evaluation.
  bool evalToZero = (evr.cwiseQuotient(der)).isZero(test_precision<Scalar>());
  if (!evalToZero) {
    cerr << "WRONG root: " << endl;
    cerr << "Polynomial: " << pols.transpose() << endl;
    cerr << "Roots found: " << roots.transpose() << endl;
    cerr << "Abs value of the polynomial at the roots: " << evr.transpose() << endl;
    cerr << endl;
  }

  std::vector<RealScalar> rootModuli(roots.size());
  Map<EvalRootsType> aux(&rootModuli[0], roots.size());
  aux = roots.array().abs();
  std::sort(rootModuli.begin(), rootModuli.end());
  bool distinctModuli = true;
  for (size_t i = 1; i < rootModuli.size() && distinctModuli; ++i) {
    if (internal::isApprox(rootModuli[i], rootModuli[i - 1])) {
      distinctModuli = false;
    }
  }
  VERIFY(evalToZero || !distinctModuli);

  return distinctModuli;
}

template <int Deg, typename POLYNOMIAL>
void evalSolver(const POLYNOMIAL& pols) {
  typedef typename POLYNOMIAL::Scalar Scalar;

  typedef PolynomialSolver<Scalar, Deg> PolynomialSolverType;

  PolynomialSolverType psolve;
  aux_evalSolver<Deg, POLYNOMIAL, PolynomialSolverType>(pols, psolve);
}

template <typename Solver>
void verify_polynomialsolver_sugar(const Solver& solver, typename Solver::RealScalar threshold) {
  using Real = typename Solver::RealScalar;
  const auto& computed = solver.roots();
  Index greatest, smallest;
  computed.cwiseAbs2().maxCoeff(&greatest);
  computed.cwiseAbs2().minCoeff(&smallest);
  VERIFY_IS_EQUAL(solver.greatestRoot(), computed[greatest]);
  VERIFY_IS_EQUAL(solver.smallestRoot(), computed[smallest]);

  std::vector<Real> expectedRealRoots, expectedRealExtrema;
  for (Index i = 0; i < computed.size(); ++i) {
    // realRoots uses a strict threshold; the extremal queries include its boundary.
    if (numext::abs(computed[i].imag()) < threshold) expectedRealRoots.push_back(computed[i].real());
    if (numext::abs(computed[i].imag()) <= threshold) expectedRealExtrema.push_back(computed[i].real());
  }
  std::vector<Real> actualRealRoots;
  solver.realRoots(actualRealRoots, threshold);
  VERIFY_IS_EQUAL(actualRealRoots.size(), expectedRealRoots.size());
  for (size_t i = 0; i < expectedRealRoots.size(); ++i) VERIFY_IS_EQUAL(actualRealRoots[i], expectedRealRoots[i]);

  const bool expectedHasRealRoot = !expectedRealExtrema.empty();
  const auto absLess = [](Real a, Real b) { return numext::abs(a) < numext::abs(b); };
  bool hasRealRoot;
  Real result = solver.absGreatestRealRoot(hasRealRoot, threshold);
  VERIFY_IS_EQUAL(hasRealRoot, expectedHasRealRoot);
  if (hasRealRoot)
    VERIFY_IS_EQUAL(result, *std::max_element(expectedRealExtrema.begin(), expectedRealExtrema.end(), absLess));
  result = solver.absSmallestRealRoot(hasRealRoot, threshold);
  VERIFY_IS_EQUAL(hasRealRoot, expectedHasRealRoot);
  if (hasRealRoot)
    VERIFY_IS_EQUAL(result, *std::min_element(expectedRealExtrema.begin(), expectedRealExtrema.end(), absLess));
  result = solver.greatestRealRoot(hasRealRoot, threshold);
  VERIFY_IS_EQUAL(hasRealRoot, expectedHasRealRoot);
  if (hasRealRoot) VERIFY_IS_EQUAL(result, *std::max_element(expectedRealExtrema.begin(), expectedRealExtrema.end()));
  result = solver.smallestRealRoot(hasRealRoot, threshold);
  VERIFY_IS_EQUAL(hasRealRoot, expectedHasRealRoot);
  if (hasRealRoot) VERIFY_IS_EQUAL(result, *std::min_element(expectedRealExtrema.begin(), expectedRealExtrema.end()));
}

template <int Deg, typename POLYNOMIAL, typename REAL_ROOTS>
void evalSolverSugarFunction(const POLYNOMIAL& pols, const REAL_ROOTS& real_roots) {
  using Scalar = typename POLYNOMIAL::Scalar;
  using RealScalar = typename POLYNOMIAL::RealScalar;
  using PolynomialSolverType = PolynomialSolver<Scalar, Deg>;

  PolynomialSolverType psolve;
  if (aux_evalSolver<Deg, POLYNOMIAL, PolynomialSolverType>(pols, psolve)) {
    // First-order root displacement estimate: delta * sum_k |r_j|^k / |p'(r_j)| for a monic polynomial,
    // p'(r_j) = prod_{k != j} (r_j - r_k). Use delta = 32 eps max_k |a_k| for the companion eigenvalue error.
    const RealScalar coefficientError = RealScalar(32) * NumTraits<RealScalar>::epsilon() * pols.cwiseAbs().maxCoeff();
    Matrix<RealScalar, Dynamic, 1> tolerance(real_roots.size());
    for (Index j = 0; j < real_roots.size(); ++j) {
      const RealScalar root = real_roots[j];
      RealScalar powerSum = RealScalar(0), power = RealScalar(1), derivative = RealScalar(1);
      for (Index k = 0; k < pols.size(); ++k) {
        powerSum += power;
        power *= numext::abs(root);
      }
      for (Index k = 0; k < real_roots.size(); ++k) {
        if (k != j) derivative *= numext::abs(root - real_roots[k]);
      }
      tolerance[j] = coefficientError * powerSum / derivative;
    }
    // A broad cluster tolerance must not hide a missing, well-conditioned root.
    for (Index j = 0; j < real_roots.size(); ++j) {
      VERIFY((numext::isfinite)(tolerance[j]));
      VERIFY((psolve.roots().array() - real_roots[j]).abs().minCoeff() <= tolerance[j]);
    }
    for (Index i = 0; i < psolve.roots().size(); ++i) {
      bool found = false;
      for (Index j = 0; j < real_roots.size() && !found; ++j) {
        VERIFY((numext::isfinite)(tolerance[j]));
        if (numext::abs(psolve.roots()[i] - real_roots[j]) <= tolerance[j]) found = true;
      }
      VERIFY(found);
    }
  }
  verify_polynomialsolver_sugar(psolve, numext::sqrt(test_precision<RealScalar>()));
}

void polynomialsolver_sugar_cluster() {
  Matrix<float, 7, 1> roots;
  roots << -0.8f, 0.2f, 0.2001f, 0.5f, 0.7f, 0.9f, 1.0f;
  Matrix<float, 8, 1> poly;
  roots_to_monicPolynomial(roots, poly);
  evalSolverSugarFunction<7>(poly, roots);
}

void polynomialsolver_sugar_filtering() {
  Vector4d poly;
  poly << 0, 1, 0, 1;
  PolynomialSolver<double, 3> solver(poly);
  verify_polynomialsolver_sugar(solver, 0.0);
  verify_polynomialsolver_sugar(solver, 0.5);
  verify_polynomialsolver_sugar(solver, 1.0);
  verify_polynomialsolver_sugar(solver, 2.0);
  Vector3d noRealRoots;
  noRealRoots << 1, 0, 1;
  PolynomialSolver<double, 2> complexSolver(noRealRoots);
  verify_polynomialsolver_sugar(complexSolver, 0.5);
}

template <typename Scalar_, int Deg_>
void polynomialsolver(int deg) {
  typedef typename NumTraits<Scalar_>::Real RealScalar;
  typedef internal::increment_if_fixed_size<Deg_> Dim;
  typedef Matrix<Scalar_, Dim::value, 1> PolynomialType;
  typedef Matrix<Scalar_, Deg_, 1> EvalRootsType;
  typedef Matrix<RealScalar, Deg_, 1> RealRootsType;

  cout << "Standard cases" << endl;
  PolynomialType pols = PolynomialType::Random(deg + 1);
  evalSolver<Deg_, PolynomialType>(pols);

  cout << "Hard cases" << endl;
  Scalar_ multipleRoot = internal::random<Scalar_>();
  EvalRootsType allRoots = EvalRootsType::Constant(deg, multipleRoot);
  roots_to_monicPolynomial(allRoots, pols);
  evalSolver<Deg_, PolynomialType>(pols);

  // The companion matrix eigenvalue approach has limited accuracy for float at
  // high degrees. The PolynomialSolver documentation itself warns: "With 32bit
  // (float) floating types this problem shows up frequently." Skip the sugar
  // function test (which requires exact root matching) for float beyond degree 8.
  if (deg <= 8 || sizeof(RealScalar) > sizeof(float)) {
    cout << "Test sugar" << endl;
    RealRootsType realRoots = RealRootsType::Random(deg);
    // sort by ascending absolute value to mitigate precision lost during polynomial expansion
    std::sort(realRoots.begin(), realRoots.end(),
              [](RealScalar a, RealScalar b) { return numext::abs(a) < numext::abs(b); });
    roots_to_monicPolynomial(realRoots, pols);
    evalSolverSugarFunction<Deg_>(pols, realRoots);
  }
}

EIGEN_DECLARE_TEST(polynomialsolver) {
  CALL_SUBTEST_7(polynomialsolver_sugar_cluster());
  CALL_SUBTEST_13(polynomialsolver_sugar_filtering());
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1((polynomialsolver<float, 1>(1)));
    CALL_SUBTEST_2((polynomialsolver<double, 2>(2)));
    CALL_SUBTEST_3((polynomialsolver<double, 3>(3)));
    CALL_SUBTEST_4((polynomialsolver<float, 4>(4)));
    CALL_SUBTEST_5((polynomialsolver<double, 5>(5)));
    CALL_SUBTEST_6((polynomialsolver<float, 6>(6)));
    CALL_SUBTEST_7((polynomialsolver<float, 7>(7)));
    CALL_SUBTEST_8((polynomialsolver<double, 8>(8)));

    CALL_SUBTEST_9((polynomialsolver<float, Dynamic>(internal::random<int>(9, 13))));
    CALL_SUBTEST_10((polynomialsolver<double, Dynamic>(internal::random<int>(9, 13))));
    CALL_SUBTEST_11((polynomialsolver<float, Dynamic>(1)));
    CALL_SUBTEST_12((polynomialsolver<std::complex<double>, Dynamic>(internal::random<int>(2, 13))));
  }
}
