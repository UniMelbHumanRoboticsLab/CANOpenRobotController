# Umfpack lib usually requires linking to a blas library.
# It is up to the user of this module to find a BLAS and link to it.
#
# This module first tries to find UMFPACK via its CMake config-mode package
# (shipped with SuiteSparse >= 7.0). If that fails, it falls back to a
# manual header/library search for compatibility with older installations.
# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

if (UMFPACK_INCLUDES AND UMFPACK_LIBRARIES)
  set(UMFPACK_FIND_QUIETLY TRUE)
endif ()

# --- Try config-mode first (SuiteSparse >= 7.0) ---
# Run this on every configure, not only when the cache is cold.  The variables
# set below name an imported target, which exists only if the config package has
# been loaded in the *current* run.  Returning early on a warm cache left
# UMFPACK_LIBRARIES naming a target that was never defined, so the first
# configure of a build tree succeeded and every later one failed.
find_package(UMFPACK CONFIG QUIET)
if(UMFPACK_FOUND AND TARGET SuiteSparse::UMFPACK)
  # Extract include dirs and libraries from the imported target so that the
  # legacy variables expected by Eigen's build system are populated.
  get_target_property(_umfpack_inc SuiteSparse::UMFPACK INTERFACE_INCLUDE_DIRECTORIES)
  # SuiteSparse >= 7 exports these as a generator expression, e.g.
  # $<TARGET_PROPERTY:SuiteSparse::SuiteSparseConfig,INTERFACE_INCLUDE_DIRECTORIES>.
  # A generator expression cannot be cached and handed to callers that inspect
  # include directories at configure time, so keep only literal paths and let
  # the manual search below handle the rest.
  set(_umfpack_inc_dirs "")
  if(_umfpack_inc)
    foreach(_dir IN LISTS _umfpack_inc)
      if(NOT _dir MATCHES "\\$<")
        list(APPEND _umfpack_inc_dirs "${_dir}")
      endif()
    endforeach()
  endif()
  if(_umfpack_inc_dirs)
    # FORCE because a plain `set(... CACHE ...)` is a no-op when the entry
    # already exists.  A build tree first configured before this fix still
    # holds the unusable generator-expression value, and reconfiguring it
    # would otherwise keep it forever.
    set(UMFPACK_INCLUDES "${_umfpack_inc_dirs}" CACHE PATH "UMFPACK include directory" FORCE)
    set(UMFPACK_LIBRARIES SuiteSparse::UMFPACK CACHE STRING "UMFPACK libraries" FORCE)
    # Mark as found and return early -- no need for the manual search below.
    mark_as_advanced(UMFPACK_INCLUDES UMFPACK_LIBRARIES)
    return()
  endif()
endif()

# --- Fallback: manual search (SuiteSparse < 7.0 or no config package) ---
find_path(UMFPACK_INCLUDES
  NAMES
  umfpack.h
  PATHS
  $ENV{UMFPACKDIR}
  ${INCLUDE_INSTALL_DIR}
  PATH_SUFFIXES
  suitesparse
  ufsparse
)

find_library(UMFPACK_LIBRARIES umfpack PATHS $ENV{UMFPACKDIR} ${LIB_INSTALL_DIR})

if(UMFPACK_LIBRARIES)

  if(NOT UMFPACK_LIBDIR)
    get_filename_component(UMFPACK_LIBDIR ${UMFPACK_LIBRARIES} PATH)
  endif()

  find_library(COLAMD_LIBRARY colamd PATHS ${UMFPACK_LIBDIR} $ENV{UMFPACKDIR} ${LIB_INSTALL_DIR})
  if(COLAMD_LIBRARY)
    set(UMFPACK_LIBRARIES ${UMFPACK_LIBRARIES} ${COLAMD_LIBRARY})
  endif ()

  find_library(AMD_LIBRARY amd PATHS ${UMFPACK_LIBDIR} $ENV{UMFPACKDIR} ${LIB_INSTALL_DIR})
  if(AMD_LIBRARY)
    set(UMFPACK_LIBRARIES ${UMFPACK_LIBRARIES} ${AMD_LIBRARY})
  endif ()

  find_library(SUITESPARSE_LIBRARY SuiteSparse PATHS ${UMFPACK_LIBDIR} $ENV{UMFPACKDIR} ${LIB_INSTALL_DIR})
  if(SUITESPARSE_LIBRARY)
    set(UMFPACK_LIBRARIES ${UMFPACK_LIBRARIES} ${SUITESPARSE_LIBRARY})
  endif ()

  find_library(CHOLMOD_LIBRARY cholmod PATHS $ENV{UMFPACK_LIBDIR} $ENV{UMFPACKDIR} ${LIB_INSTALL_DIR})
  if(CHOLMOD_LIBRARY)
    set(UMFPACK_LIBRARIES ${UMFPACK_LIBRARIES} ${CHOLMOD_LIBRARY})
  endif()

endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(UMFPACK DEFAULT_MSG
                                  UMFPACK_INCLUDES UMFPACK_LIBRARIES)

mark_as_advanced(UMFPACK_INCLUDES UMFPACK_LIBRARIES AMD_LIBRARY COLAMD_LIBRARY CHOLMOD_LIBRARY SUITESPARSE_LIBRARY)
