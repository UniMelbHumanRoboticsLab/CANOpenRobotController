// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include "main.h"
#include "fp_control.h"

#include <limits>

#include <Eigen/Eigenvalues>

template <typename RealScalar>
void wilkinson_shift_avoids_squared_subdiagonal() {
  using std::sqrt;

  // Keep sqrt(min)/16 and all dependent expressions out of constant folding so they observe the active FTZ mode.
  volatile RealScalar normal_min = (std::numeric_limits<RealScalar>::min)();
  const RealScalar subdiag_before = sqrt(normal_min) / RealScalar(16);
  VERIFY(numext::is_exactly_zero(subdiag_before * subdiag_before));
  for (int td_sign = -1; td_sign <= 1; td_sign += 2) {
    for (int subdiag_sign = -1; subdiag_sign <= 1; subdiag_sign += 2) {
      const RealScalar td = RealScalar(td_sign) * subdiag_before * NumTraits<RealScalar>::epsilon();
      RealScalar diag[2] = {td, -td};
      RealScalar subdiag[1] = {RealScalar(subdiag_sign) * subdiag_before};

      internal::tridiagonal_qr_step<RealScalar, RealScalar, int>(diag, subdiag, 0, 1, nullptr, 2);

      // A Wilkinson shift nearly diagonalizes this 2x2 block in one step. Reassociating its fallback through e^2
      // under fast-math makes the correction underflow under FTZ and instead leaves the off-diagonal unchanged.
      const RealScalar kDeflated = RealScalar(8) * NumTraits<RealScalar>::epsilon() * subdiag_before;
      VERIFY(numext::abs(subdiag[0]) <= kDeflated);
    }
  }
}

template <typename RealScalar>
void selfadjoint_eigensolver_preserves_tiny_block() {
  using std::sqrt;

  const RealScalar epsilon = NumTraits<RealScalar>::epsilon();
  volatile RealScalar normal_min = (std::numeric_limits<RealScalar>::min)();
  const RealScalar e = sqrt(normal_min) / RealScalar(16);
  // e survives deflation, (e/epsilon)^2 > 2*abs(td), while e^2 underflows.
  // Keep the ratio separate so fast-math cannot reassociate the test setup itself through e^2.
  volatile RealScalar ratio = (e / (epsilon * epsilon)) / RealScalar(16);
  const RealScalar td = e * ratio;
  VERIFY(numext::is_exactly_zero(e * e));
  volatile RealScalar scaled_e = e / epsilon;
  VERIFY(scaled_e * scaled_e > RealScalar(2) * numext::abs(td));
  Matrix<RealScalar, 3, 3> matrix;
  matrix << RealScalar(1), RealScalar(0), RealScalar(0), RealScalar(0), td, e, RealScalar(0), e, -td;

  SelfAdjointEigenSolver<Matrix<RealScalar, 3, 3> > solver(matrix, EigenvaluesOnly);
  VERIFY_IS_EQUAL(solver.info(), Success);
  const RealScalar kRelative = RealScalar(8) * epsilon;
  VERIFY(numext::abs(solver.eigenvalues()(0) + e) <= kRelative * e);
  VERIFY(numext::abs(solver.eigenvalues()(1) - e) <= kRelative * e);
}

EIGEN_DECLARE_TEST(eigensolver_selfadjoint_fastmath) {
  Eigen::ScopedFlushToZero flush_to_zero;
  if (flush_to_zero.isSupported()) {
    CALL_SUBTEST(selfadjoint_eigensolver_preserves_tiny_block<float>());
    CALL_SUBTEST(selfadjoint_eigensolver_preserves_tiny_block<double>());
    CALL_SUBTEST(wilkinson_shift_avoids_squared_subdiagonal<float>());
    CALL_SUBTEST(wilkinson_shift_avoids_squared_subdiagonal<double>());
  }
}
