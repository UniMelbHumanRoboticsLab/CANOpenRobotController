#include "../Eigen/Core"

// Reproduces the "Dynamic-sized Matrix breaks tuple_size" bug reported on !2336.
// tuple_size<MatrixXd> is a SFINAE-friendly empty specialization (issue #3103),
// so naming ::value outside a SFINAE context must still fail to compile.
#ifdef EIGEN_SHOULD_FAIL_TO_BUILD
#define ROWS Eigen::Dynamic
#define COLS Eigen::Dynamic
#else
#define ROWS 3
#define COLS 1
#endif

#include <tuple>

int main() {
  (void)std::tuple_size<Eigen::Matrix<double, ROWS, COLS>>::value;
  return 0;  // Test should pass.
}
