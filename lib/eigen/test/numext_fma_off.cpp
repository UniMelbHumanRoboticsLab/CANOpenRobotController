// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#undef EIGEN_SCALAR_MADD_USE_FMA
#define EIGEN_SCALAR_MADD_USE_FMA 0
#include "numext.cpp"  // NOLINT(bugprone-suspicious-include): Compile the suite with a different configuration.
