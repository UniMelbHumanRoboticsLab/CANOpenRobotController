// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

MatrixXd m(3, 4);
m.resize(NoChange, 5);
cout << "m: " << m.rows() << " rows, " << m.cols() << " cols" << endl;
