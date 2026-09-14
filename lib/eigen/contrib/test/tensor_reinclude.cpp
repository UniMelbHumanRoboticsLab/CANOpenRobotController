// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include "main.h"
#include <unsupported/Eigen/CXX11/Tensor>

// Re-inclusion after enabling a backend must complete its device type. Backend
// evaluation still requires the macro before the first Eigen include.
#define EIGEN_USE_THREADS
#include <unsupported/Eigen/CXX11/Tensor>

EIGEN_DECLARE_TEST(tensor_reinclude) {
  Eigen::ThreadPool pool(1);
  Eigen::ThreadPoolDevice device(&pool, 1);
  VERIFY_IS_EQUAL(device.numThreads(), 1);
}
