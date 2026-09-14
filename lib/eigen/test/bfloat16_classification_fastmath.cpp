// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

// The same checks compiled with the compiler's fast-math flags (test/CMakeLists.txt), under which a classification
// of the widened float folds to false: isinf/isnan/isfinite of a bfloat16 have to test its bits.
#include "bfloat16_classification.cpp"  // NOLINT(bugprone-suspicious-include): the same suite under fast-math flags.
