// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

Matrix4i m = Matrix4i::Random();
cout << "Here is the matrix m:" << endl << m << endl;
cout << "Here are the coefficients on the 1st super-diagonal and 2nd sub-diagonal of m:" << endl
     << m.diagonal(1).transpose() << endl
     << m.diagonal(-2).transpose() << endl;
