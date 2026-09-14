// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

// The same checks with assertions compiled out: the runtime checks have to fire in a release build, where the
// gpu_assert() calls they replace were empty.
#define EIGEN_NO_DEBUG 1
#include "tensor_gpu_runtime_check.cu"
