// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

MatrixXd m(2, 3);
m << 2, 0.5, 1, 3, 0.25, 1;
cout << m.cwiseInverse() << endl;
