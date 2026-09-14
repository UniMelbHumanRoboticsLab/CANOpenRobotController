#include "../Eigen/Core"

// Reproduces the "storage-order footgun" on !2336: get<I>() uses coeffRef(Index),
// which is linear in storage order, so a RowMajor matrix would silently flip
// decomposition order vs. the column-major default. We static_assert against
// RowMajor until the semantics are agreed upon — this failtest guards that.
#ifdef EIGEN_SHOULD_FAIL_TO_BUILD
#define STORAGE_ORDER Eigen::RowMajor
#else
#define STORAGE_ORDER Eigen::ColMajor
#endif

int main() {
  Eigen::Matrix<double, 2, 2, STORAGE_ORDER> m;
  m << 1, 2, 3, 4;
  (void)Eigen::get<0>(m);
  return 0;  // Test should pass.
}
