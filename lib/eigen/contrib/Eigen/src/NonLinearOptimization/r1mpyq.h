// IWYU pragma: private
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_NONLINEAROPTIMIZATION_R1MPYQ_H
#define EIGEN_NONLINEAROPTIMIZATION_R1MPYQ_H

#include "./InternalHeaderCheck.h"

namespace Eigen {

namespace internal {

// TODO : move this to GivensQR once there's such a thing in Eigen

// Each rotation mixes a column of a with its last column, so a row vector is
// passed as the transpose of the column vector holding it.
template <typename Scalar, typename Derived>
void r1mpyq(MatrixBase<Derived> &a_, const std::vector<JacobiRotation<Scalar> > &v_givens,
            const std::vector<JacobiRotation<Scalar> > &w_givens) {
  Derived &a = a_.derived();
  const Index n = a.cols();

  /*     apply the first set of givens rotations to a. */
  for (Index j = n - 2; j >= 0; --j) a.applyOnTheRight(j, n - 1, v_givens[j]);
  /*     apply the second set of givens rotations to a. */
  for (Index j = 0; j < n - 1; ++j) a.applyOnTheRight(j, n - 1, w_givens[j].transpose());
}

}  // end namespace internal

}  // end namespace Eigen

#endif  // EIGEN_NONLINEAROPTIMIZATION_R1MPYQ_H
