# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

# Consumes an installed Eigen exactly the way doc/TopicCMakeGuide.dox opens:
# find_package plus target_link_libraries against Eigen3::Eigen.
# EIGEN_VERSION_SPEC carries the optional version argument so one project can
# stand in for the plain, ranged, and rejected forms the guide documents.

cmake_minimum_required(VERSION 3.17)
project(eigen_installed_consumer CXX)

find_package(Eigen3 ${EIGEN_VERSION_SPEC} REQUIRED NO_MODULE)

# Prove the package came from the install tree the scenario built. Eigen calls
# export(PACKAGE Eigen3), so an unrelated build tree recorded in CMake's user
# package registry can satisfy find_package and leave the scenario asserting
# nothing about what it installed.
if(EIGEN_EXPECTED_PREFIX)
  get_filename_component(eigen_found_dir "${Eigen3_DIR}" REALPATH)
  get_filename_component(eigen_expected_prefix "${EIGEN_EXPECTED_PREFIX}" REALPATH)
  string(FIND "${eigen_found_dir}" "${eigen_expected_prefix}/" eigen_prefix_position)
  if(NOT eigen_prefix_position EQUAL 0)
    message(FATAL_ERROR
            "found Eigen3 at ${eigen_found_dir}, expected it under ${eigen_expected_prefix}")
  endif()
endif()

add_executable(consumer ../main.cpp)
target_link_libraries(consumer Eigen3::Eigen)
