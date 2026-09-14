# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

# Stands in for Eigen's use of Find{CHOLMOD,UMFPACK,KLU,SPQR}.cmake: it asks for
# the legacy variables those modules define and checks that they are still
# usable, which is what a second configure of the same build tree used to break.
# The suitesparse_reconfigure scenario supplies EIGEN_CMAKE_DIR, a <pkg>_DIR per
# package pointing at a fake SuiteSparse >= 7 config package, and the include
# directory those packages export.

cmake_minimum_required(VERSION 3.17)
project(suitesparse_consumer NONE)

list(APPEND CMAKE_MODULE_PATH "${EIGEN_CMAKE_DIR}")

foreach(pkg IN ITEMS CHOLMOD UMFPACK KLU SPQR)
  find_package(${pkg})

  if(NOT ${pkg}_FOUND)
    message(FATAL_ERROR "${pkg} was not found")
  endif()

  # The legacy *_LIBRARIES value names an imported target, so the config package
  # has to have been loaded by this configure run, not merely by the one that
  # filled the cache.
  if(NOT ${pkg}_LIBRARIES STREQUAL "SuiteSparse::${pkg}")
    message(FATAL_ERROR "${pkg}_LIBRARIES is '${${pkg}_LIBRARIES}', expected SuiteSparse::${pkg}")
  endif()
  if(NOT TARGET ${${pkg}_LIBRARIES})
    message(FATAL_ERROR "${pkg}_LIBRARIES names ${${pkg}_LIBRARIES}, which is not a target here")
  endif()

  # Asserting the exact directory the fake config package exports also proves
  # the config-mode branch ran, rather than a SuiteSparse that happens to be
  # installed on the host answering the manual fallback search.
  if(NOT ${pkg}_INCLUDES STREQUAL "${EXPECTED_INCLUDE_DIR}")
    message(FATAL_ERROR "${pkg}_INCLUDES is '${${pkg}_INCLUDES}', expected ${EXPECTED_INCLUDE_DIR}")
  endif()
  # A generator expression is not a directory, and consumers that read include
  # directories at configure time -- FindCUDA's cuda_add_executable() among them
  # -- cannot resolve one.
  foreach(dir IN LISTS ${pkg}_INCLUDES)
    if(NOT IS_DIRECTORY "${dir}")
      message(FATAL_ERROR "${pkg}_INCLUDES holds '${dir}', which is not a usable directory")
    endif()
  endforeach()
endforeach()
