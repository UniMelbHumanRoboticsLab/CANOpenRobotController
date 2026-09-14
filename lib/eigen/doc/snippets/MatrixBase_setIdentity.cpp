// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

Matrix4i m = Matrix4i::Zero();
m.block<3, 3>(1, 0).setIdentity();
cout << m << endl;
