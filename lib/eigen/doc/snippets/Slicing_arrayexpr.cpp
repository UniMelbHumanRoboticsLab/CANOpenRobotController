// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

ArrayXi ind(5);
ind << 4, 2, 5, 5, 3;
MatrixXi A = MatrixXi::Random(4, 6);
cout << "Initial matrix A:\n" << A << "\n\n";
cout << "A(all,ind-1):\n" << A(Eigen::placeholders::all, ind - 1) << "\n\n";
