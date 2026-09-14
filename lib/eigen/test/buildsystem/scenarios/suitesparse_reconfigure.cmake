# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

# Find{CHOLMOD,UMFPACK,KLU,SPQR}.cmake cache values that are only meaningful
# while the SuiteSparse >= 7 config package is loaded: *_LIBRARIES names an
# imported target and *_INCLUDES comes from that target's
# INTERFACE_INCLUDE_DIRECTORIES.  So the claim under test is specifically that
# the second configure of a build tree behaves like the first -- the first one
# always worked -- and, separately, that no generator expression reaches the
# cache, since SuiteSparse >= 7 lists one beside the real include directory.
#
# No SuiteSparse installation is involved: each package is a fake config file
# exporting a target of the shape SuiteSparse installs.

set(fake_root "${WORK_DIR}/suitesparse")
set(include_dir "${fake_root}/include")
file(MAKE_DIRECTORY "${include_dir}")

set(package_args "")
foreach(pkg IN ITEMS CHOLMOD UMFPACK KLU SPQR)
  set(pkg_dir "${fake_root}/${pkg}")
  file(MAKE_DIRECTORY "${pkg_dir}")
  file(WRITE "${pkg_dir}/${pkg}Config.cmake"
"# Stand-in for the config package SuiteSparse >= 7 installs.  The guard is
# needed because Find${pkg}.cmake loads this file on every configure.
if(NOT TARGET SuiteSparse::${pkg})
  add_library(SuiteSparse::${pkg} INTERFACE IMPORTED)
  set_target_properties(SuiteSparse::${pkg} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
    \"${include_dir};$<TARGET_PROPERTY:SuiteSparse::SuiteSparseConfig,INTERFACE_INCLUDE_DIRECTORIES>\")
endif()
")
  list(APPEND package_args "-D${pkg}_DIR=${pkg_dir}")
endforeach()

set(consumer_src "${BS_CONSUMER_DIR}/suitesparse")
set(consumer_bin "${WORK_DIR}/consumer")

bs_configure("the SuiteSparse consumer, cold cache" "${consumer_src}" "${consumer_bin}"
             "-DEIGEN_CMAKE_DIR=${EIGEN_SOURCE_DIR}/cmake"
             "-DEXPECTED_INCLUDE_DIR=${include_dir}"
             ${package_args})
bs_configure("the SuiteSparse consumer, warm cache" "${consumer_src}" "${consumer_bin}"
             "-DEIGEN_CMAKE_DIR=${EIGEN_SOURCE_DIR}/cmake"
             "-DEXPECTED_INCLUDE_DIR=${include_dir}"
             ${package_args})
# What a build tool triggers after an edit: a re-configure carrying nothing but
# the cache the previous runs wrote.
bs_configure("the SuiteSparse consumer, re-generated from the cache alone"
             "${consumer_src}" "${consumer_bin}")

file(READ "${consumer_bin}/CMakeCache.txt" cache)
foreach(pkg IN ITEMS CHOLMOD UMFPACK KLU SPQR)
  if(NOT cache MATCHES "\n${pkg}_INCLUDES:[A-Z]+=([^\n]*)")
    bs_fail("${pkg}_INCLUDES was never cached")
  endif()
  if(NOT CMAKE_MATCH_1 STREQUAL "${include_dir}")
    bs_fail("${pkg}_INCLUDES cached as '${CMAKE_MATCH_1}', expected ${include_dir}")
  endif()
endforeach()

# --- Upgrade path -------------------------------------------------------------
# The scenario above starts from an empty cache, which is not how anyone meets
# this bug: they have a build tree configured *before* the fix, whose cache
# already holds the unusable generator-expression value.  `set(... CACHE ...)`
# does nothing when the entry exists, so without FORCE the find modules would
# leave that value in place and reconfiguring would never heal the tree.
#
# Seed exactly that state and require one configure to repair it.
set(stale_bin "${WORK_DIR}/consumer-stale")
file(MAKE_DIRECTORY "${stale_bin}")
set(stale_genex "$<TARGET_PROPERTY:SuiteSparse::SuiteSparseConfig,INTERFACE_INCLUDE_DIRECTORIES>")
set(seed "")
foreach(pkg IN ITEMS CHOLMOD UMFPACK KLU SPQR)
  string(APPEND seed "${pkg}_INCLUDES:PATH=${include_dir};${stale_genex}\n")
  string(APPEND seed "${pkg}_LIBRARIES:STRING=SuiteSparse::${pkg}\n")
endforeach()
file(WRITE "${stale_bin}/CMakeCache.txt" "${seed}")

bs_configure("the SuiteSparse consumer, cache poisoned by a pre-fix configure"
             "${consumer_src}" "${stale_bin}"
             "-DEIGEN_CMAKE_DIR=${EIGEN_SOURCE_DIR}/cmake"
             "-DEXPECTED_INCLUDE_DIR=${include_dir}"
             ${package_args})

file(READ "${stale_bin}/CMakeCache.txt" stale_cache)
foreach(pkg IN ITEMS CHOLMOD UMFPACK KLU SPQR)
  if(NOT stale_cache MATCHES "\n${pkg}_INCLUDES:[A-Z]+=([^\n]*)")
    bs_fail("${pkg}_INCLUDES was never cached in the poisoned tree")
  endif()
  # Hold the capture: any later MATCHES would overwrite CMAKE_MATCH_1.
  set(healed "${CMAKE_MATCH_1}")
  if(healed MATCHES "\\$<")
    bs_fail("${pkg}_INCLUDES still holds a generator expression after re-configure: '${healed}'")
  endif()
  if(NOT healed STREQUAL "${include_dir}")
    bs_fail("${pkg}_INCLUDES healed to '${healed}', expected ${include_dir}")
  endif()
endforeach()
