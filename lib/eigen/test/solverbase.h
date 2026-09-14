// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#ifndef TEST_SOLVERBASE_H
#define TEST_SOLVERBASE_H

template <typename DstType, typename RhsType, typename MatrixType, typename SolverType>
void check_solverbase(const MatrixType& matrix, const SolverType& solver, Index rows, Index cols, Index cols2) {
  // solve
  DstType m2 = DstType::Random(cols, cols2);
  RhsType m3 = matrix * m2;
  DstType solver_solution = DstType::Random(cols, cols2);
  solver._solve_impl(m3, solver_solution);
  VERIFY_IS_APPROX(m3, matrix * solver_solution);
  solver_solution = DstType::Random(cols, cols2);
  solver_solution = solver.solve(m3);
  VERIFY_IS_APPROX(m3, matrix * solver_solution);
  // test solve with transposed
  m3 = RhsType::Random(rows, cols2);
  m2 = matrix.transpose() * m3;
  RhsType solver_solution2 = RhsType::Random(rows, cols2);
  solver.template _solve_impl_transposed<false>(m2, solver_solution2);
  VERIFY_IS_APPROX(m2, matrix.transpose() * solver_solution2);
  solver_solution2 = RhsType::Random(rows, cols2);
  solver_solution2 = solver.transpose().solve(m2);
  VERIFY_IS_APPROX(m2, matrix.transpose() * solver_solution2);
  // test solve with conjugate transposed
  m3 = RhsType::Random(rows, cols2);
  m2 = matrix.adjoint() * m3;
  solver_solution2 = RhsType::Random(rows, cols2);
  solver.template _solve_impl_transposed<true>(m2, solver_solution2);
  VERIFY_IS_APPROX(m2, matrix.adjoint() * solver_solution2);
  solver_solution2 = RhsType::Random(rows, cols2);
  solver_solution2 = solver.adjoint().solve(m2);
  VERIFY_IS_APPROX(m2, matrix.adjoint() * solver_solution2);
  // test with temporary expression as rhs
  m2 = DstType::Random(cols, cols2);
  solver_solution = solver.solve(matrix * m2);
  VERIFY_IS_APPROX(matrix * m2, matrix * solver_solution);
}

// Checks the four determinant accessors of a decomposition against a reference determinant \a det and a
// reference \a logabsdet = log|det| formed independently of it. Callers must keep \a det itself in range.
template <typename SolverType, typename Scalar>
void check_determinant(const SolverType& solver, const Scalar& det, const typename NumTraits<Scalar>::Real& logabsdet) {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  VERIFY_IS_APPROX(solver.determinant(), det);
  VERIFY_IS_APPROX(solver.absDeterminant(), numext::abs(det));
  // log|det| passes through zero, where a relative comparison says nothing; bound the error absolutely.
  VERIFY_IS_MUCH_SMALLER_THAN(solver.logAbsDeterminant() - logabsdet, RealScalar(1));
  VERIFY_IS_APPROX(solver.signDeterminant(), numext::sign(det));
}

// True when |det| has left the representable range in the direction it was expected to: exactly infinity
// where the determinant overflowed, exactly zero where it underflowed. logAbsDeterminant() is still
// meaningful there; determinant() and absDeterminant() are not. A NaN is neither, and testing !isfinite()
// would accept one -- with it any inf - inf or 0 * inf artifact.
template <typename RealScalar>
bool determinant_out_of_range(const RealScalar& absdet, bool overflow) {
  return overflow ? numext::equal_strict(absdet, NumTraits<RealScalar>::infinity()) : numext::is_exactly_zero(absdet);
}

#endif  // TEST_SOLVERBASE_H
