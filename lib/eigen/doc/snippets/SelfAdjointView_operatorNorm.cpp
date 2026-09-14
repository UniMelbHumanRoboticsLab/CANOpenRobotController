// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

MatrixXd ones = MatrixXd::Ones(3, 3);
cout << "The operator norm of the 3x3 matrix of ones is " << ones.selfadjointView<Lower>().operatorNorm() << endl;
