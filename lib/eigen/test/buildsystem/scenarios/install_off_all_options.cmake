# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

# doc/TopicCMakeGuide.dox: with EIGEN_INSTALL set to OFF, "Eigen then places no
# file at all under CMAKE_INSTALL_PREFIX".
#
# install_off exercises that claim where a project meets it, embedded, and there
# EIGEN_BUILD_CMAKE_PACKAGE, EIGEN_BUILD_PKGCONFIG, EIGEN_BUILD_BLAS and
# EIGEN_BUILD_LAPACK all default to OFF -- so the guards around those install
# rules are never reached and could regress unnoticed.  Enable every one of them
# in a top-level configure instead, which is also the only configuration that
# reaches the install-doc target.

bs_configure("Eigen with EIGEN_INSTALL=OFF and every install option on"
             "${EIGEN_SOURCE_DIR}" "${WORK_DIR}/eigen-build"
             "-DCMAKE_INSTALL_PREFIX=${BS_PREFIX}"
             -DEIGEN_BUILD_TESTING=OFF
             -DEIGEN_BUILD_DOC=OFF
             -DEIGEN_BUILD_DEMOS=OFF
             -DEIGEN_BUILD_BLAS=ON
             -DEIGEN_BUILD_LAPACK=ON
             -DEIGEN_BUILD_CMAKE_PACKAGE=ON
             -DEIGEN_BUILD_PKGCONFIG=ON
             -DEIGEN_INSTALL=OFF)
bs_install("Eigen with EIGEN_INSTALL=OFF" "${WORK_DIR}/eigen-build")

bs_assert_no_eigen_installed()
bs_assert_not_installed("eigen_blas" "the BLAS library")
bs_assert_not_installed("eigen_lapack" "the LAPACK library")

# Naming the artifacts above still lets an unguarded install rule for something
# else through, and here nothing at all is expected under the prefix.
bs_installed_files(installed)
if(installed)
  bs_installed_summary(summary)
  bs_fail("expected an empty install prefix; ${summary}")
endif()

# install-doc copies the generated documentation into CMAKE_INSTALL_FULL_DOCDIR,
# so it is an install rule in every sense that matters here and must not exist
# either.  Asking to build it is enough: the documentation is never generated,
# because a target that is gone cannot run.
bs_configure("Eigen with EIGEN_INSTALL=OFF and documentation enabled"
             "${EIGEN_SOURCE_DIR}" "${WORK_DIR}/eigen-doc-build"
             "-DCMAKE_INSTALL_PREFIX=${BS_PREFIX}"
             -DEIGEN_BUILD_TESTING=OFF
             -DEIGEN_BUILD_DOC=ON
             -DEIGEN_BUILD_DEMOS=OFF
             -DEIGEN_BUILD_BLAS=OFF
             -DEIGEN_BUILD_LAPACK=OFF
             -DEIGEN_INSTALL=OFF)
bs_run(WHAT "build install-doc with EIGEN_INSTALL=OFF" EXPECT_RESULT FAILURE
       OUTPUT_VARIABLE output
       COMMAND ${CMAKE_COMMAND} --build "${WORK_DIR}/eigen-doc-build"
               --config "${BUILD_CONFIG}" --target install-doc)
if(NOT output MATCHES "install-doc")
  bs_fail("the install-doc build failed for some other reason\n----\n${output}\n----")
endif()
