// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

Matrix2d a, b, c;
a << 1, 2, 3, 4;
b << 5, 6, 7, 8;
c.noalias() = a * b;  // this computes the product directly to c
cout << c << endl;
