// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

RowVector4i v = RowVector4i::Random();
cout << "Here is the vector v:" << endl << v << endl;
cout << "Here is v.head(2):" << endl << v.head<2>() << endl;
v.head<2>().setZero();
cout << "Now the vector v is:" << endl << v << endl;
