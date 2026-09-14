// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

Matrix3d m = Matrix3d::Identity();
m.col(1) = Vector3d(4, 5, 6);
cout << m << endl;
