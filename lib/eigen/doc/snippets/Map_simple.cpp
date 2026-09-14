// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

int array[9];
for (int i = 0; i < 9; ++i) array[i] = i;
cout << Map<Matrix3i>(array) << endl;
