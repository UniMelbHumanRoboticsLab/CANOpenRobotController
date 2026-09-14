// Force the generic clang vector backend with 64-byte vectors.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0
#define EIGEN_VECTORIZE_GENERIC 1
#define EIGEN_GENERIC_VECTOR_SIZE_BYTES 64
#include "packetmath.cpp"
