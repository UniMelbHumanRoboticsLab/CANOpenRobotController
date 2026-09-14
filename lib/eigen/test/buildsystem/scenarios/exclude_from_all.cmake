# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

# doc/TopicCMakeGuide.dox: passing EXCLUDE_FROM_ALL to add_subdirectory "drops
# Eigen's install rules".  CMake documents this on the EXCLUDE_FROM_ALL
# directory property rather than on add_subdirectory, and the wording only
# appeared in recent releases, so the behavior is worth pinning at whatever
# CMake version is running the suite.
#
# The guide's separate warning -- that FetchContent_Declare ignores the keyword
# before CMake 3.28 -- cannot be checked here, because a scenario only ever has
# the one CMake that launched it.

bs_configure("embedding consumer with EXCLUDE_FROM_ALL" "${BS_CONSUMER_DIR}/subproject"
             "${WORK_DIR}/consumer"
             "-DEIGEN_SOURCE_DIR=${EIGEN_SOURCE_DIR}"
             "-DCMAKE_INSTALL_PREFIX=${BS_PREFIX}"
             "-DEIGEN_USE_EXCLUDE_FROM_ALL=ON")
bs_build_and_run_consumer("embedding consumer with EXCLUDE_FROM_ALL" "${WORK_DIR}/consumer")
bs_install("embedding consumer with EXCLUDE_FROM_ALL" "${WORK_DIR}/consumer")

bs_assert_no_eigen_installed()
bs_assert_installed("bin/consumer" "the consumer's own executable")
