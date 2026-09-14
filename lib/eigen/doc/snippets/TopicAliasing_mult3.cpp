// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

MatrixXf matA(2, 2);
matA << 2, 0, 0, 2;
matA.noalias() = matA * matA;
cout << matA;
