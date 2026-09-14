// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_GPU_META_H
#define EIGEN_GPU_META_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"
#include <type_traits>

namespace Eigen {
namespace gpu {
namespace internal {

template <typename T>
using require_t = std::enable_if_t<std::decay_t<T>::value, int>;

template <typename T>
using require_not_t = std::enable_if_t<!std::decay_t<T>::value, int>;

template <typename T1, typename T2>
using require_same_t = require_t<std::is_same<std::decay_t<T1>, std::decay_t<T2>>>;

template <typename T1, typename T2>
using require_not_same_t = require_not_t<std::is_same<std::decay_t<T1>, std::decay_t<T2>>>;

template <bool... Values>
using require_all_t = require_t<Eigen::internal::reduce_all<Values...>>;

}  // namespace internal
}  // namespace gpu
}  // namespace Eigen

#endif  // EIGEN_GPU_META_H
