// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

Matrix3d m = 10000 * Matrix3d::Identity();
m(0, 2) = 1;
cout << "Here's the matrix m:" << endl << m << endl;
cout << "m.isDiagonal() returns: " << m.isDiagonal() << endl;
cout << "m.isDiagonal(1e-3) returns: " << m.isDiagonal(1e-3) << endl;
