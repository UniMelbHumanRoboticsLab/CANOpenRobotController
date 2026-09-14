# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

# The complement of find_package_version_range: without a rejection case, a
# version file that accepted everything would pass that test.
#
# The non-range branch of cmake/Eigen3ConfigVersion.cmake.in accepts a package
# whose version is at least the requested one and below the next major, so a
# bare "3.4" spans >=3.4 to <4.0.0 rather than 3.4.z.  Rejecting a request one
# major above the installed package would hold under either reading, so the
# cases below pin the implemented one: both bounds, plus -- whenever the
# installed version leaves room to state it -- a minor below the installed one
# that must still be accepted.

bs_install_eigen()
bs_eigen_package_version(major minor patch)

if(major LESS 4)
  bs_fail("this scenario assumes Eigen's major version is at least 4, got ${major}")
endif()

# Upper bound: an earlier major must not answer, whatever its minor.
bs_configure_expect_failure("consumer pinned to 3.4" "${BS_CONSUMER_DIR}/installed"
                            "${WORK_DIR}/consumer-old" output
                            "-DCMAKE_PREFIX_PATH=${BS_PREFIX}"
                            "-DEIGEN_EXPECTED_PREFIX=${BS_PREFIX}"
                            "-DEIGEN_VERSION_SPEC=3.4"
                            ${BS_FIND_PACKAGE_ISOLATION})
if(NOT output MATCHES "Eigen3")
  bs_fail("configure failed for some reason other than the version check\n----\n${output}\n----")
endif()

math(EXPR too_new "${major}+1")
bs_configure_expect_failure("consumer requiring ${too_new}" "${BS_CONSUMER_DIR}/installed"
                            "${WORK_DIR}/consumer-new" output
                            "-DCMAKE_PREFIX_PATH=${BS_PREFIX}"
                            "-DEIGEN_EXPECTED_PREFIX=${BS_PREFIX}"
                            "-DEIGEN_VERSION_SPEC=${too_new}"
                            ${BS_FIND_PACKAGE_ISOLATION})
if(NOT output MATCHES "Eigen3")
  bs_fail("configure failed for some reason other than the version check\n----\n${output}\n----")
endif()

# Lower bound: a patch above the installed one is inside the same major and must
# still be rejected, which a file that only compared majors would not do.
math(EXPR next_patch "${patch}+1")
bs_configure_expect_failure("consumer requiring ${major}.${minor}.${next_patch}"
                            "${BS_CONSUMER_DIR}/installed"
                            "${WORK_DIR}/consumer-patch" output
                            "-DCMAKE_PREFIX_PATH=${BS_PREFIX}"
                            "-DEIGEN_EXPECTED_PREFIX=${BS_PREFIX}"
                            "-DEIGEN_VERSION_SPEC=${major}.${minor}.${next_patch}"
                            ${BS_FIND_PACKAGE_ISOLATION})
if(NOT output MATCHES "Eigen3")
  bs_fail("configure failed for some reason other than the version check\n----\n${output}\n----")
endif()

# The case that separates ">=X.Y, <X+1.0.0" from "X.Y.z": an older minor of the
# installed major must be accepted.  Only statable once the release series has
# moved past .0, so it is a control the scenario adds when it can rather than a
# requirement on the version number.
if(minor GREATER 0)
  math(EXPR previous_minor "${minor}-1")
  bs_configure("consumer requiring ${major}.${previous_minor}" "${BS_CONSUMER_DIR}/installed"
               "${WORK_DIR}/consumer-minor"
               "-DCMAKE_PREFIX_PATH=${BS_PREFIX}"
               "-DEIGEN_EXPECTED_PREFIX=${BS_PREFIX}"
               "-DEIGEN_VERSION_SPEC=${major}.${previous_minor}"
               ${BS_FIND_PACKAGE_ISOLATION})
  bs_build_and_run_consumer("consumer requiring an older minor" "${WORK_DIR}/consumer-minor")
endif()
