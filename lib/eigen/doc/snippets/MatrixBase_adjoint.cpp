// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

Matrix2cf m = Matrix2cf::Random();
cout << "Here is the 2x2 complex matrix m:" << endl << m << endl;
cout << "Here is the adjoint of m:" << endl << m.adjoint() << endl;
