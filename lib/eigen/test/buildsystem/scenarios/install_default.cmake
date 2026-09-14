# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

# doc/TopicCMakeGuide.dox: "Eigen's install rules are enabled even when it is
# built as a sub-project."  This is the baseline that makes install_off and
# exclude_from_all meaningful -- without it, a build that installed nothing at
# all would satisfy both of them.

bs_configure("embedding consumer" "${BS_CONSUMER_DIR}/subproject" "${WORK_DIR}/consumer"
             "-DEIGEN_SOURCE_DIR=${EIGEN_SOURCE_DIR}"
             "-DCMAKE_INSTALL_PREFIX=${BS_PREFIX}")
bs_build("embedding consumer" "${WORK_DIR}/consumer")
bs_install("embedding consumer" "${WORK_DIR}/consumer")

bs_assert_eigen_headers_installed()
bs_assert_installed("bin/consumer" "the consumer's own executable")
