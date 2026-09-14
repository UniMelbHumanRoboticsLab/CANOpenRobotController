# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

# Driver for the build-system integration tests.  Run in script mode:
#
#   cmake -DEIGEN_SOURCE_DIR=<src> -DSCENARIO=<name> -DWORK_DIR=<dir>
#         -DGENERATOR=<gen> -DCXX_COMPILER=<path> -DBUILD_CONFIG=<cfg>
#         -P run_scenario.cmake
#
# The driver defines the helpers below and then includes
# scenarios/${SCENARIO}.cmake, which performs the assertions.  A failed
# assertion aborts with FATAL_ERROR, which CTest reports as a test failure.

cmake_minimum_required(VERSION 3.17)

foreach(var EIGEN_SOURCE_DIR SCENARIO WORK_DIR GENERATOR BUILD_CONFIG)
  if(NOT DEFINED ${var})
    message(FATAL_ERROR "run_scenario.cmake requires -D${var}=<value>")
  endif()
endforeach()

set(BS_CONSUMER_DIR "${CMAKE_CURRENT_LIST_DIR}/consumers")
set(BS_PREFIX "${WORK_DIR}/prefix")

# The parts of Eigen's own build that no scenario inspects are slow to
# configure and drag in tools a scenario should not depend on.
set(BS_EIGEN_QUIET_OPTIONS
    -DEIGEN_BUILD_TESTING=OFF
    -DEIGEN_BUILD_DOC=OFF
    -DEIGEN_BUILD_DEMOS=OFF
    -DEIGEN_BUILD_BLAS=OFF
    -DEIGEN_BUILD_LAPACK=OFF
    )

# Eigen calls export(PACKAGE Eigen3), so CMake's user package registry names
# every Eigen build tree on the machine.  A find_package scenario that let the
# registry answer would pass without ever reading the install tree it just
# built, so every such configure disables both registries and additionally
# asserts, in the consumer project, that the package came from BS_PREFIX.
set(BS_FIND_PACKAGE_ISOLATION
    -DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF
    -DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF
    )

# Start from an empty work directory so a rerun cannot observe a previous
# run's cache or install tree.
file(REMOVE_RECURSE "${WORK_DIR}")
file(MAKE_DIRECTORY "${WORK_DIR}")

function(bs_fail message)
  message(FATAL_ERROR "[${SCENARIO}] ${message}")
endfunction()

function(bs_assert_streq actual expected what)
  if(NOT actual STREQUAL expected)
    bs_fail("${what}:\n  expected [${expected}]\n  actual   [${actual}]")
  endif()
endfunction()

# Runs a command, failing the scenario with its merged output unless the exit
# status matches EXPECT_RESULT.  EXPECT_RESULT accepts a number or the word
# FAILURE, which admits any non-zero status.
function(bs_run)
  cmake_parse_arguments(BS "" "WHAT;EXPECT_RESULT;OUTPUT_VARIABLE" "COMMAND" ${ARGN})
  if(NOT DEFINED BS_EXPECT_RESULT)
    set(BS_EXPECT_RESULT 0)
  endif()
  execute_process(COMMAND ${BS_COMMAND}
                  RESULT_VARIABLE result
                  OUTPUT_VARIABLE output
                  ERROR_VARIABLE output)
  if(BS_EXPECT_RESULT STREQUAL "FAILURE")
    if(result EQUAL 0)
      bs_fail("${BS_WHAT} was expected to fail but succeeded\n----\n${output}\n----")
    endif()
  elseif(NOT result EQUAL BS_EXPECT_RESULT)
    bs_fail("${BS_WHAT} exited ${result}, expected ${BS_EXPECT_RESULT}\n----\n${output}\n----")
  endif()
  if(BS_OUTPUT_VARIABLE)
    set(${BS_OUTPUT_VARIABLE} "${output}" PARENT_SCOPE)
  endif()
endfunction()

# Eigen's own project() call enables C as well as CXX, so a nested configure of
# Eigen needs both compilers.  An image whose compilers are versioned names such
# as gcc-10/g++-10, with no plain cc on PATH, has none to detect on its own.
function(bs_cmake_compiler_args out_var)
  set(args "")
  if(C_COMPILER)
    list(APPEND args "-DCMAKE_C_COMPILER=${C_COMPILER}")
  endif()
  if(CXX_COMPILER)
    list(APPEND args "-DCMAKE_CXX_COMPILER=${CXX_COMPILER}")
  endif()
  set(${out_var} "${args}" PARENT_SCOPE)
endfunction()

# Configures <src> into <bin>.  Remaining arguments are passed to CMake.
function(bs_configure what src bin)
  bs_cmake_compiler_args(compiler_args)
  bs_run(WHAT "configure ${what}"
         COMMAND ${CMAKE_COMMAND} -S "${src}" -B "${bin}" -G "${GENERATOR}"
                 "-DCMAKE_BUILD_TYPE=${BUILD_CONFIG}" ${compiler_args} ${ARGN})
endfunction()

# Configures <src> expecting failure, returning the output through <output_var>.
function(bs_configure_expect_failure what src bin output_var)
  bs_cmake_compiler_args(compiler_args)
  bs_run(WHAT "configure ${what}" EXPECT_RESULT FAILURE OUTPUT_VARIABLE output
         COMMAND ${CMAKE_COMMAND} -S "${src}" -B "${bin}" -G "${GENERATOR}"
                 "-DCMAKE_BUILD_TYPE=${BUILD_CONFIG}" ${compiler_args} ${ARGN})
  set(${output_var} "${output}" PARENT_SCOPE)
endfunction()

function(bs_build what bin)
  bs_run(WHAT "build ${what}"
         COMMAND ${CMAKE_COMMAND} --build "${bin}" --config "${BUILD_CONFIG}" ${ARGN})
endfunction()

function(bs_install what bin)
  bs_run(WHAT "install ${what}"
         COMMAND ${CMAKE_COMMAND} --install "${bin}" --config "${BUILD_CONFIG}")
endfunction()

# Configures Eigen as a top-level project and installs it into BS_PREFIX.
# Remaining arguments are passed to the Eigen configure.
function(bs_install_eigen)
  bs_configure("Eigen" "${EIGEN_SOURCE_DIR}" "${WORK_DIR}/eigen-build"
               "-DCMAKE_INSTALL_PREFIX=${BS_PREFIX}"
               ${BS_EIGEN_QUIET_OPTIONS} ${ARGN})
  bs_install("Eigen" "${WORK_DIR}/eigen-build")
endfunction()

# Every file in the install prefix, as paths relative to it.  Matching against
# this list rather than globbing absolute paths keeps the assertions
# independent of how deep INCLUDE_INSTALL_DIR happens to be.
function(bs_installed_files out_var)
  set(found "")
  if(EXISTS "${BS_PREFIX}")
    file(GLOB_RECURSE absolute LIST_DIRECTORIES false "${BS_PREFIX}/*")
    foreach(path IN LISTS absolute)
      file(RELATIVE_PATH relative "${BS_PREFIX}" "${path}")
      list(APPEND found "${relative}")
    endforeach()
  endif()
  list(SORT found)
  set(${out_var} "${found}" PARENT_SCOPE)
endfunction()

# A readable excerpt of the install tree for failure messages.
function(bs_installed_summary out_var)
  bs_installed_files(all)
  list(LENGTH all count)
  set(excerpt "${all}")
  if(count GREATER 20)
    list(SUBLIST excerpt 0 20 excerpt)
    list(APPEND excerpt "... and ${count} files in total")
  endif()
  string(REPLACE ";" "\n  " excerpt "${excerpt}")
  set(${out_var} "${count} file(s) installed:\n  ${excerpt}" PARENT_SCOPE)
endfunction()

function(bs_matching_installed_files regex out_var)
  bs_installed_files(all)
  set(hits "")
  foreach(path IN LISTS all)
    if(path MATCHES "${regex}")
      list(APPEND hits "${path}")
    endif()
  endforeach()
  set(${out_var} "${hits}" PARENT_SCOPE)
endfunction()

function(bs_assert_installed regex what)
  bs_matching_installed_files("${regex}" hits)
  if(NOT hits)
    bs_installed_summary(summary)
    bs_fail("expected ${what} matching /${regex}/ in the install tree; ${summary}")
  endif()
endfunction()

function(bs_assert_not_installed regex what)
  bs_matching_installed_files("${regex}" hits)
  if(hits)
    string(REPLACE ";" "\n  " listing "${hits}")
    bs_fail("expected no ${what} in the install tree, found:\n  ${listing}")
  endif()
endfunction()

# Asserts the install tree holds nothing Eigen contributes: headers from the
# supported and contributed trees, the legacy unsupported/ shims, the signature
# file, eigen3.pc, and the CMake package files.
function(bs_assert_no_eigen_installed)
  bs_assert_not_installed("/Eigen/Core$" "supported Eigen header")
  bs_assert_not_installed("/contrib/Eigen/Tensor$" "contributed Eigen header")
  bs_assert_not_installed("/unsupported/Eigen/Tensor$" "legacy unsupported Eigen shim")
  bs_assert_not_installed("signature_of_eigen3_matrix_library$" "Eigen signature file")
  bs_assert_not_installed("eigen3\\.pc$" "eigen3.pc")
  bs_assert_not_installed("Eigen3Config\\.cmake$" "Eigen3 CMake package")
  bs_assert_not_installed("Eigen3Targets.*\\.cmake$" "Eigen3 CMake targets")
endfunction()

# Asserts the install tree holds the headers a consumer needs.
function(bs_assert_eigen_headers_installed)
  bs_assert_installed("/Eigen/Core$" "supported Eigen header")
  bs_assert_installed("/Eigen/Dense$" "Eigen/Dense umbrella")
  bs_assert_installed("/contrib/Eigen/Tensor$" "contributed Eigen header")
  bs_assert_installed("/unsupported/Eigen/Tensor$" "legacy unsupported Eigen shim")
  bs_assert_installed("signature_of_eigen3_matrix_library$" "Eigen signature file")
endfunction()

# The version Eigen3ConfigVersion.cmake will carry, read from the same header
# the top-level CMakeLists parses.  Deriving the version constraints a scenario
# tests from this keeps them meaningful across a release bump instead of
# pinning them to whatever version happened to be current.
function(bs_eigen_package_version out_major out_minor out_patch)
  file(READ "${EIGEN_SOURCE_DIR}/Eigen/Version" header)
  foreach(component MAJOR MINOR PATCH)
    # Test the match itself, not CMAKE_MATCH_1: a legitimate component value of
    # 0 is false to if().
    string(REGEX MATCH "define[ \t]+EIGEN_${component}_VERSION[ \t]+([0-9]+)" matched "${header}")
    if(NOT matched)
      bs_fail("could not read EIGEN_${component}_VERSION from Eigen/Version")
    endif()
    set(value_${component} "${CMAKE_MATCH_1}")
  endforeach()
  set(${out_major} "${value_MAJOR}" PARENT_SCOPE)
  set(${out_minor} "${value_MINOR}" PARENT_SCOPE)
  set(${out_patch} "${value_PATCH}" PARENT_SCOPE)
endfunction()

function(bs_find_consumer_executable bin out_var)
  foreach(candidate
          "${bin}/consumer" "${bin}/consumer.exe"
          "${bin}/${BUILD_CONFIG}/consumer" "${bin}/${BUILD_CONFIG}/consumer.exe")
    if(EXISTS "${candidate}")
      set(${out_var} "${candidate}" PARENT_SCOPE)
      return()
    endif()
  endforeach()
  set(${out_var} "" PARENT_SCOPE)
endfunction()

# Builds a consumer project and runs it, failing unless it prints the marker
# its source promises.  Running it is what proves the headers a scenario
# assembled are usable, not merely present.
function(bs_build_and_run_consumer what bin)
  bs_build("${what}" "${bin}")
  bs_find_consumer_executable("${bin}" consumer_exe)
  if(NOT consumer_exe)
    bs_fail("${what}: built no consumer executable under ${bin}")
  endif()
  bs_run(WHAT "run ${what}" OUTPUT_VARIABLE output COMMAND "${consumer_exe}")
  if(NOT output MATCHES "eigen-ok")
    bs_fail("${what}: consumer ran but did not print its marker\n----\n${output}\n----")
  endif()
endfunction()

set(BS_SCENARIO_FILE "${CMAKE_CURRENT_LIST_DIR}/scenarios/${SCENARIO}.cmake")
if(NOT EXISTS "${BS_SCENARIO_FILE}")
  message(FATAL_ERROR "no such scenario: ${BS_SCENARIO_FILE}")
endif()
include("${BS_SCENARIO_FILE}")
