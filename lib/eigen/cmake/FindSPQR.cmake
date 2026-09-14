# SPQR lib usually requires linking to a blas and lapack library.
# It is up to the user of this module to find a BLAS and link to it.
#
# SPQR lib requires Cholmod, colamd and amd as well.
# FindCholmod.cmake can be used to find those packages before finding spqr
#
# This module first tries to find SPQR via its CMake config-mode package
# (shipped with SuiteSparse >= 7.0). If that fails, it falls back to a
# manual header/library search for compatibility with older installations.
# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

if (SPQR_INCLUDES AND SPQR_LIBRARIES)
  set(SPQR_FIND_QUIETLY TRUE)
endif ()

# --- Try config-mode first (SuiteSparse >= 7.0) ---
# Run this on every configure, not only when the cache is cold.  The variables
# set below name an imported target, which exists only if the config package has
# been loaded in the *current* run.  Returning early on a warm cache left
# SPQR_LIBRARIES naming a target that was never defined, so the first
# configure of a build tree succeeded and every later one failed.
find_package(SPQR CONFIG QUIET)
if(SPQR_FOUND AND TARGET SuiteSparse::SPQR)
  # Extract include dirs and libraries from the imported target so that the
  # legacy variables expected by Eigen's build system are populated.
  get_target_property(_spqr_inc SuiteSparse::SPQR INTERFACE_INCLUDE_DIRECTORIES)
  # SuiteSparse >= 7 exports these as a generator expression, e.g.
  # $<TARGET_PROPERTY:SuiteSparse::SuiteSparseConfig,INTERFACE_INCLUDE_DIRECTORIES>.
  # A generator expression cannot be cached and handed to callers that inspect
  # include directories at configure time, so keep only literal paths and let
  # the manual search below handle the rest.
  set(_spqr_inc_dirs "")
  if(_spqr_inc)
    foreach(_dir IN LISTS _spqr_inc)
      if(NOT _dir MATCHES "\\$<")
        list(APPEND _spqr_inc_dirs "${_dir}")
      endif()
    endforeach()
  endif()
  if(_spqr_inc_dirs)
    # FORCE because a plain `set(... CACHE ...)` is a no-op when the entry
    # already exists.  A build tree first configured before this fix still
    # holds the unusable generator-expression value, and reconfiguring it
    # would otherwise keep it forever.
    set(SPQR_INCLUDES "${_spqr_inc_dirs}" CACHE PATH "SPQR include directory" FORCE)
    set(SPQR_LIBRARIES SuiteSparse::SPQR CACHE STRING "SPQR libraries" FORCE)
    # Mark as found and return early -- no need for the manual search below.
    mark_as_advanced(SPQR_INCLUDES SPQR_LIBRARIES)
    return()
  endif()
endif()

# --- Fallback: manual search (SuiteSparse < 7.0 or no config package) ---
find_path(SPQR_INCLUDES
  NAMES
  SuiteSparseQR.hpp
  PATHS
  $ENV{SPQRDIR}
  ${INCLUDE_INSTALL_DIR}
  PATH_SUFFIXES
  suitesparse
  ufsparse
)

find_library(SPQR_LIBRARIES spqr $ENV{SPQRDIR} ${LIB_INSTALL_DIR})

if(SPQR_LIBRARIES)

  find_library(SUITESPARSE_LIBRARY SuiteSparse PATHS $ENV{SPQRDIR} ${LIB_INSTALL_DIR})
  if (SUITESPARSE_LIBRARY)
    set(SPQR_LIBRARIES ${SPQR_LIBRARIES} ${SUITESPARSE_LIBRARY})
  endif()

  find_library(CHOLMOD_LIBRARY cholmod PATHS $ENV{UMFPACK_LIBDIR} $ENV{UMFPACKDIR} ${LIB_INSTALL_DIR})
  if(CHOLMOD_LIBRARY)
    set(SPQR_LIBRARIES ${SPQR_LIBRARIES} ${CHOLMOD_LIBRARY})
  endif()

endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SPQR DEFAULT_MSG SPQR_INCLUDES SPQR_LIBRARIES)

mark_as_advanced(SPQR_INCLUDES SPQR_LIBRARIES)
