// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

MatrixXd ones = MatrixXd::Ones(3, 3);
VectorXcd eivals = ones.eigenvalues();
cout << "The eigenvalues of the 3x3 matrix of ones are:" << endl << eivals << endl;
