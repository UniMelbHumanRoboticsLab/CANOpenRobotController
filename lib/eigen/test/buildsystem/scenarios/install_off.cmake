# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

# doc/TopicCMakeGuide.dox: with EIGEN_INSTALL set to OFF, "Eigen then places no
# file at all under CMAKE_INSTALL_PREFIX", while the embedding project's own
# install rules keep working.  This is issue #3088.

bs_configure("embedding consumer with EIGEN_INSTALL=OFF" "${BS_CONSUMER_DIR}/subproject"
             "${WORK_DIR}/consumer"
             "-DEIGEN_SOURCE_DIR=${EIGEN_SOURCE_DIR}"
             "-DCMAKE_INSTALL_PREFIX=${BS_PREFIX}"
             "-DEIGEN_INSTALL=OFF")
bs_build_and_run_consumer("embedding consumer with EIGEN_INSTALL=OFF" "${WORK_DIR}/consumer")
bs_install("embedding consumer with EIGEN_INSTALL=OFF" "${WORK_DIR}/consumer")

bs_assert_no_eigen_installed()
bs_assert_installed("bin/consumer" "the consumer's own executable")

# Nothing but the consumer's own executable may appear, so a future install
# rule added outside the EIGEN_INSTALL guard is caught here rather than only
# by the named checks above.
bs_installed_files(installed)
list(LENGTH installed count)
if(NOT count EQUAL 1)
  bs_installed_summary(summary)
  bs_fail("expected only the consumer's executable; ${summary}")
endif()
