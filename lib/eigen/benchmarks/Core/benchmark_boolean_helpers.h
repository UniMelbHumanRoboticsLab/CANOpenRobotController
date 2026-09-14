// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_BENCHMARK_BOOLEAN_HELPERS_H
#define EIGEN_BENCHMARK_BOOLEAN_HELPERS_H

#include <cstdint>

#include <Eigen/Core>

namespace Eigen {
namespace benchmark_detail {

using BoolArray = Array<bool, Dynamic, 1>;

enum InputPattern { Random, AllFalse, AllTrue, Sparse, FirstTrue, LastTrue, FirstFalse, LastFalse };

template <InputPattern Pattern>
void fill_input(BoolArray& input, std::uint32_t state) {
  for (Index i = 0; i < input.size(); ++i) {
    state = state * 1664525u + 1013904223u;
    input[i] = Pattern == AllTrue || Pattern == FirstFalse || Pattern == LastFalse ||
               (Pattern == Sparse && state % 10 == 0) || (Pattern == Random && (state >> 31) != 0);
  }
  if (input.size() == 0) return;
  if (Pattern == FirstTrue) input[0] = true;
  if (Pattern == LastTrue) input[input.size() - 1] = true;
  if (Pattern == FirstFalse) input[0] = false;
  if (Pattern == LastFalse) input[input.size() - 1] = false;
}

}  // namespace benchmark_detail
}  // namespace Eigen

#endif  // EIGEN_BENCHMARK_BOOLEAN_HELPERS_H
