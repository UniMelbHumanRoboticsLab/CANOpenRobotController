// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

Matrix2d md = Matrix2d::Identity() * 0.45;
Matrix2f mf = Matrix2f::Identity();
cout << md + mf.cast<double>() << endl;
