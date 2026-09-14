// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

std::vector<int> ind{4, 2, 5, 5, 3};
MatrixXi A = MatrixXi::Random(4, 6);
cout << "Initial matrix A:\n" << A << "\n\n";
cout << "A(all,ind):\n" << A(Eigen::placeholders::all, ind) << "\n\n";
