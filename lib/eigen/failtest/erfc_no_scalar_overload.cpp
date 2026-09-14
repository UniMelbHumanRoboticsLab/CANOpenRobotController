#include "../contrib/Eigen/SpecialFunctions"

// A custom scalar that does not provide erfc. Eigen's approximations are tuned
// for float and double and are not valid for it, so numext::erfc must be
// rejected at compile time instead of computing a wrong result.
namespace custom_scalar {
struct CustomReal {
  double value;
};
#ifndef EIGEN_SHOULD_FAIL_TO_BUILD
inline CustomReal erfc(const CustomReal& x) { return CustomReal{std::erfc(x.value)}; }
#endif
}  // namespace custom_scalar

namespace Eigen {
template <>
struct NumTraits<custom_scalar::CustomReal> : NumTraits<double> {
  using Real = custom_scalar::CustomReal;
  using NonInteger = custom_scalar::CustomReal;
  using Nested = custom_scalar::CustomReal;
};
}  // namespace Eigen

int main() { return Eigen::numext::erfc(custom_scalar::CustomReal{0.5}).value > 0.0 ? 0 : 1; }
