# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

# Embeds Eigen the way doc/TopicCMakeGuide.dox describes for FetchContent:
# the options are forced before Eigen is added, then Eigen3::Eigen is linked.
# FetchContent_MakeAvailable reduces to this add_subdirectory call, so driving
# add_subdirectory directly tests the same path without a network fetch.
#
# EIGEN_USE_EXCLUDE_FROM_ALL selects the variant the guide's EXCLUDE_FROM_ALL
# section describes.

cmake_minimum_required(VERSION 3.17)
project(eigen_subproject_consumer CXX)

# The consumer runs a test of its own, so the scenario can require that CTest
# in this build directory sees exactly that one test.  Without a test to find,
# "Eigen contributed nothing" and "CTest read the wrong directory" produce the
# same empty listing.  enable_testing() precedes add_subdirectory so that any
# test Eigen registered would be generated into this tree too.
enable_testing()

set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)

if(EIGEN_USE_EXCLUDE_FROM_ALL)
  add_subdirectory("${EIGEN_SOURCE_DIR}" eigen-build EXCLUDE_FROM_ALL)
else()
  add_subdirectory("${EIGEN_SOURCE_DIR}" eigen-build)
endif()

add_executable(consumer ../main.cpp)
target_link_libraries(consumer Eigen3::Eigen)
add_test(NAME consumer_runs COMMAND consumer)

# The consumer's own install rule must keep working whatever Eigen's does, so
# the scenarios can tell "Eigen installed nothing" apart from "nothing ran".
install(TARGETS consumer RUNTIME DESTINATION bin)
