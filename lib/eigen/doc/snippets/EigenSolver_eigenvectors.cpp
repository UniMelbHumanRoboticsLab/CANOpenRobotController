// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

MatrixXd ones = MatrixXd::Ones(3, 3);
EigenSolver<MatrixXd> es(ones);
cout << "The first eigenvector of the 3x3 matrix of ones is:" << endl << es.eigenvectors().col(0) << endl;
