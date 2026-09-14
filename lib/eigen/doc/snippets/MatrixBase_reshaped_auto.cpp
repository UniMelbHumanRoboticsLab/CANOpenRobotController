// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

Matrix4i m = Matrix4i::Random();
cout << "Here is the matrix m:" << endl << m << endl;
cout << "Here is m.reshaped(2, AutoSize):" << endl << m.reshaped(2, AutoSize) << endl;
cout << "Here is m.reshaped<RowMajor>(AutoSize, fix<8>):" << endl << m.reshaped<RowMajor>(AutoSize, fix<8>) << endl;
