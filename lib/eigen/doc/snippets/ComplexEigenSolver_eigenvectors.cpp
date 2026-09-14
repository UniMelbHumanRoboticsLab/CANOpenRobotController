// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

MatrixXcf ones = MatrixXcf::Ones(3, 3);
ComplexEigenSolver<MatrixXcf> ces(ones);
cout << "The first eigenvector of the 3x3 matrix of ones is:" << endl << ces.eigenvectors().col(0) << endl;
