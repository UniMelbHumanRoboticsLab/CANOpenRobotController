# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

# doc/TopicCMakeGuide.dox lists EIGEN_BUILD_PKGCONFIG among the options an
# embedding project turns off.  find_package already pins that a default
# top-level install ships eigen3.pc, so this pins that the option suppresses
# it, and that the headers are unaffected either way.

bs_install_eigen(-DEIGEN_BUILD_PKGCONFIG=OFF)

bs_assert_not_installed("eigen3\\.pc$" "eigen3.pc")
bs_assert_eigen_headers_installed()
