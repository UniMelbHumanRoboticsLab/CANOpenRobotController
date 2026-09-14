// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

MatrixXd m(2, 3);
m << 2, -4, 6, -5, 1, 0;
cout << m.cwiseAbs() << endl;
