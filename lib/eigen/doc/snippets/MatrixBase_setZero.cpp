// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

Matrix4i m = Matrix4i::Random();
m.row(1).setZero();
cout << m << endl;
