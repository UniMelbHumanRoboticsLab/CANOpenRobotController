# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

# Checks the variables that project(Eigen3 VERSION ...) is responsible for
# populating. Their values as observed during Eigen's own configure are baked
# into the test command, so a variable that project() left unset arrives here as
# an empty string rather than as an undefined one.

if(NOT DEFINED EIGEN_EXPECTED_VERSION OR EIGEN_EXPECTED_VERSION STREQUAL "")
  message(FATAL_ERROR "EIGEN_EXPECTED_VERSION was not passed to this script.")
endif()

set(names PROJECT_VERSION Eigen3_VERSION)
# CMAKE_PROJECT_VERSION belongs to the outermost project, which is Eigen3 only
# when Eigen is not being consumed as a sub-project.
if(EIGEN_IS_TOP_LEVEL)
  list(APPEND names CMAKE_PROJECT_VERSION)
endif()

foreach(name IN LISTS names)
  if(NOT "${OBSERVED_${name}}" STREQUAL "${EIGEN_EXPECTED_VERSION}")
    message(FATAL_ERROR "${name} was \"${OBSERVED_${name}}\", expected "
                        "\"${EIGEN_EXPECTED_VERSION}\": the top-level project() call "
                        "must declare VERSION.")
  endif()
endforeach()
