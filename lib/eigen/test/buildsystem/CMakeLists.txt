# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

# Build-system integration tests.  Each scenario asserts a claim about Eigen's
# own CMake surface -- most by driving a real configure, build, and install of
# Eigen -- so documented or relied-upon CMake behavior cannot drift from the
# tree without a test failing.
#
# Registered only for a top-level, non-cross-compiling build: a scenario
# configures and runs a nested project, which needs a usable host toolchain
# rather than the artifacts of the enclosing build.
#
# CMAKE_CROSSCOMPILING alone does not catch the CI's emulator builds, which
# select a cross compiler without a toolchain file or CMAKE_SYSTEM_NAME.  There
# the enclosing suite runs through the CROSSCOMPILING_EMULATOR target property,
# but the nested consumers inherit only the (cross) compiler, so the scenarios
# that run one exec a foreign-architecture binary on the host.

if(NOT PROJECT_IS_TOP_LEVEL OR CMAKE_CROSSCOMPILING OR CMAKE_CROSSCOMPILING_EMULATOR)
  return()
endif()

set(EIGEN_BUILDSYSTEM_SCENARIOS
    find_package
    find_package_version_reject
    add_subdirectory
    install_default
    install_off
    install_off_all_options
    exclude_from_all
    cuda_launcher_wrapper
    suitesparse_reconfigure
    )

# find_package version ranges are CMake 3.19 syntax.  On the 3.17 Eigen still
# supports, "3.4...5" reaches Eigen3ConfigVersion.cmake as one unparsable
# version string and the consumer configure fails, so the claim this scenario
# tests cannot be stated there at all.
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.19)
  list(APPEND EIGEN_BUILDSYSTEM_SCENARIOS find_package_version_range)
endif()

# EIGEN_BUILD_PKGCONFIG only exists off Windows, so the scenario asserting
# that it suppresses eigen3.pc has nothing to assert there.
if(NOT WIN32 OR NOT CMAKE_HOST_SYSTEM_NAME MATCHES Windows)
  list(APPEND EIGEN_BUILDSYSTEM_SCENARIOS pkgconfig_off)
endif()

set(EIGEN_BUILDSYSTEM_CONFIG "${CMAKE_BUILD_TYPE}")
if(NOT EIGEN_BUILDSYSTEM_CONFIG)
  set(EIGEN_BUILDSYSTEM_CONFIG "Release")
endif()

foreach(scenario IN LISTS EIGEN_BUILDSYSTEM_SCENARIOS)
  add_test(NAME buildsystem_${scenario}
    COMMAND ${CMAKE_COMMAND}
      "-DEIGEN_SOURCE_DIR=${Eigen_SOURCE_DIR}"
      "-DSCENARIO=${scenario}"
      "-DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/${scenario}"
      "-DGENERATOR=${CMAKE_GENERATOR}"
      "-DC_COMPILER=${CMAKE_C_COMPILER}"
      "-DCXX_COMPILER=${CMAKE_CXX_COMPILER}"
      "-DBUILD_CONFIG=${EIGEN_BUILDSYSTEM_CONFIG}"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/run_scenario.cmake")
  # Each scenario runs several nested CMake invocations; the default 1500s
  # ceiling is ample, but a hung nested build should fail rather than hang.
  set_tests_properties(buildsystem_${scenario} PROPERTIES
                       LABELS "buildsystem" TIMEOUT 900)
endforeach()
