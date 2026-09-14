// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

MatrixXf M1(3, 3);  // Column-major storage
M1 << 1, 2, 3, 4, 5, 6, 7, 8, 9;

cout << "v1:" << endl << M1.reshaped().transpose() << endl;

Matrix<float, Dynamic, Dynamic, RowMajor> M2(M1);
cout << "v2:" << endl << M2.reshaped<AutoOrder>().transpose() << endl;
