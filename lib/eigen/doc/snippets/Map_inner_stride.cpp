// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

int array[12];
for (int i = 0; i < 12; ++i) array[i] = i;
cout << Map<VectorXi, 0, InnerStride<2> >(array, 6)  // the inner stride has already been passed as template parameter
     << endl;
