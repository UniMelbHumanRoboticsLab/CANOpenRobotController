// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

int array[24];
for (int i = 0; i < 24; ++i) array[i] = i;

cout << "Original column-major matrix:\n" << Map<Matrix<int, 6, 4> >(array) << endl;
cout << "Every other row:\n" << Map<Matrix<int, 3, 4>, Unaligned, InnerStride<2> >(array) << endl;
