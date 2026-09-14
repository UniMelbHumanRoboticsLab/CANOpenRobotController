# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

# doc/TopicCMakeGuide.dox: Eigen can be embedded without installing it first,
# applying the guide's "Disabling Eigen build options" snippet beforehand.  The
# consumer project sets exactly those options, so this covers both claims:
# Eigen3::Eigen is usable from a sub-project, and an embedded Eigen contributes
# nothing to the enclosing project's tests.

bs_configure("embedding consumer" "${BS_CONSUMER_DIR}/subproject" "${WORK_DIR}/consumer"
             "-DEIGEN_SOURCE_DIR=${EIGEN_SOURCE_DIR}")
bs_build_and_run_consumer("embedding consumer" "${WORK_DIR}/consumer")

# Embedding Eigen must leave the enclosing project's test list alone: the
# consumer registers one test of its own, and that must be all CTest reports.
#
# CTest gained --test-dir in CMake 3.20.  On the 3.17 Eigen supports it is
# accepted and ignored, leaving CTest to inspect its own working directory --
# which is the outer Eigen build tree, whose tests would then be reported as
# the consumer's.  Select the consumer build directory by changing into it.
bs_run(WHAT "list tests of the embedding consumer" OUTPUT_VARIABLE tests
       COMMAND ${CMAKE_COMMAND} -E chdir "${WORK_DIR}/consumer"
               ${CMAKE_CTEST_COMMAND} -N)
if(NOT tests MATCHES "consumer_runs")
  bs_fail("CTest did not inspect the consumer build directory\n----\n${tests}\n----")
endif()
if(NOT tests MATCHES "Total Tests: 1")
  bs_fail("embedding Eigen changed the enclosing project's test list:\n----\n${tests}\n----")
endif()
