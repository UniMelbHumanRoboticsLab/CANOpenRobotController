// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <cctype>

#include "blas.h"

// LSAME returns true if ca and cb are the same letter, regardless of case.
extern "C" EIGEN_BLAS_API int lsame_(const char *ca, const char *cb) {
  return std::toupper(static_cast<unsigned char>(*ca)) == std::toupper(static_cast<unsigned char>(*cb));
}
