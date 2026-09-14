// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

MatrixXi m = Matrix4i::Random();
cout << "Here is the matrix m:" << endl << m << endl;
cout << "Here is m.reshaped(2, 8):" << endl << m.reshaped(2, 8) << endl;
m.resize(2, 8);
cout << "Here is the matrix m after m.resize(2,8):" << endl << m << endl;
