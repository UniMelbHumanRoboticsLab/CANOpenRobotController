// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

MatrixXcf v = MatrixXcf::Random(2, 3);
cout << v << endl << endl;
cout << v.cwiseArg() << endl;