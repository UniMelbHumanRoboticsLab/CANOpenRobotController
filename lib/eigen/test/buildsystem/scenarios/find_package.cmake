# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

# doc/TopicCMakeGuide.dox: an installed Eigen is consumed with
# find_package(Eigen3 REQUIRED NO_MODULE) and target_link_libraries against
# Eigen3::Eigen.  Also pins what a default top-level install contains, since
# every later scenario asserting an absence is only meaningful against it.

bs_install_eigen()

bs_assert_eigen_headers_installed()
bs_assert_installed("Eigen3Config\\.cmake$" "Eigen3 CMake package")
bs_assert_installed("Eigen3ConfigVersion\\.cmake$" "Eigen3 package version file")
bs_assert_installed("Eigen3Targets\\.cmake$" "Eigen3 exported targets")
bs_assert_installed("/Eigen/Version$" "generated Eigen/Version header")

# pkgconfig_off only asserts that eigen3.pc is absent, so without the positive
# case here it would stay green even if pkg-config installation broke for every
# configuration.  EIGEN_BUILD_PKGCONFIG, and with it eigen3.pc, exists only
# where the top-level CMakeLists offers the option.
if(NOT WIN32 OR NOT CMAKE_HOST_SYSTEM_NAME MATCHES Windows)
  bs_assert_installed("eigen3\\.pc$" "eigen3.pc")
endif()

bs_configure("consumer" "${BS_CONSUMER_DIR}/installed" "${WORK_DIR}/consumer"
             "-DCMAKE_PREFIX_PATH=${BS_PREFIX}"
             "-DEIGEN_EXPECTED_PREFIX=${BS_PREFIX}"
             ${BS_FIND_PACKAGE_ISOLATION})
bs_build_and_run_consumer("consumer" "${WORK_DIR}/consumer")
