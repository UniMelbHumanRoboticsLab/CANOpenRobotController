// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

MatrixXf a(2, 2);
std::cout << "a is of size " << a.rows() << "x" << a.cols() << std::endl;
MatrixXf b(3, 3);
a = b;
std::cout << "a is now of size " << a.rows() << "x" << a.cols() << std::endl;
