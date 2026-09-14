// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

Array33i a = Array33i::Random(), b = Array33i::Random();
Array33i c = a * b;
cout << "a:\n" << a << "\nb:\n" << b << "\nc:\n" << c << endl;
