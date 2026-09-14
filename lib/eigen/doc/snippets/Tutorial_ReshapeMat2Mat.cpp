// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

MatrixXf M1(2, 6);  // Column-major storage
M1 << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12;

cout << "M2:" << endl << M1.reshaped(6, 2) << endl;
