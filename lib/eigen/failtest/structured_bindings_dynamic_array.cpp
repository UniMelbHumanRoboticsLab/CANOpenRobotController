#include "../Eigen/Core"

// Reproduces the "Dynamic-sized Array breaks tuple_size" bug: the Array
// specialization behaves like the Matrix one — a SFINAE-friendly empty
// tuple_size (issue #3103), so naming ::value must still fail to compile.
#ifdef EIGEN_SHOULD_FAIL_TO_BUILD
#define ROWS Eigen::Dynamic
#define COLS Eigen::Dynamic
#else
#define ROWS 3
#define COLS 1
#endif

#include <tuple>

int main() {
  (void)std::tuple_size<Eigen::Array<double, ROWS, COLS>>::value;
  return 0;  // Test should pass.
}
