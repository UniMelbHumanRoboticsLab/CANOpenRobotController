// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

MatrixXi m = MatrixXi::Random(2, 3);
cout << "Here is the matrix m:" << endl << m << endl;
cout << "m.colwise().replicate<3>() = ..." << endl;
cout << m.colwise().replicate<3>() << endl;
