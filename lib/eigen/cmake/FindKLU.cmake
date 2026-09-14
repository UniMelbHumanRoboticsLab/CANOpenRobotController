# KLU lib usually requires linking to a blas library.
# It is up to the user of this module to find a BLAS and link to it.
#
# This module first tries to find KLU via its CMake config-mode package
# (shipped with SuiteSparse >= 7.0). If that fails, it falls back to a
# manual header/library search for compatibility with older installations.
# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

if (KLU_INCLUDES AND KLU_LIBRARIES)
  set(KLU_FIND_QUIETLY TRUE)
endif ()

# --- Try config-mode first (SuiteSparse >= 7.0) ---
# Run this on every configure, not only when the cache is cold.  The variables
# set below name an imported target, which exists only if the config package has
# been loaded in the *current* run.  Returning early on a warm cache left
# KLU_LIBRARIES naming a target that was never defined, so the first
# configure of a build tree succeeded and every later one failed.
find_package(KLU CONFIG QUIET)
if(KLU_FOUND AND TARGET SuiteSparse::KLU)
  # Extract include dirs and libraries from the imported target so that the
  # legacy variables expected by Eigen's build system are populated.
  get_target_property(_klu_inc SuiteSparse::KLU INTERFACE_INCLUDE_DIRECTORIES)
  # SuiteSparse >= 7 exports these as a generator expression, e.g.
  # $<TARGET_PROPERTY:SuiteSparse::SuiteSparseConfig,INTERFACE_INCLUDE_DIRECTORIES>.
  # A generator expression cannot be cached and handed to callers that inspect
  # include directories at configure time, so keep only literal paths and let
  # the manual search below handle the rest.
  set(_klu_inc_dirs "")
  if(_klu_inc)
    foreach(_dir IN LISTS _klu_inc)
      if(NOT _dir MATCHES "\\$<")
        list(APPEND _klu_inc_dirs "${_dir}")
      endif()
    endforeach()
  endif()
  if(_klu_inc_dirs)
    # FORCE because a plain `set(... CACHE ...)` is a no-op when the entry
    # already exists.  A build tree first configured before this fix still
    # holds the unusable generator-expression value, and reconfiguring it
    # would otherwise keep it forever.
    set(KLU_INCLUDES "${_klu_inc_dirs}" CACHE PATH "KLU include directory" FORCE)
    set(KLU_LIBRARIES SuiteSparse::KLU CACHE STRING "KLU libraries" FORCE)
    # Mark as found and return early -- no need for the manual search below.
    mark_as_advanced(KLU_INCLUDES KLU_LIBRARIES)
    return()
  endif()
endif()

# --- Fallback: manual search (SuiteSparse < 7.0 or no config package) ---
find_path(KLU_INCLUDES
  NAMES
  klu.h
  PATHS
  $ENV{KLUDIR}
  ${INCLUDE_INSTALL_DIR}
  PATH_SUFFIXES
  suitesparse
  ufsparse
)

find_library(KLU_LIBRARIES klu PATHS $ENV{KLUDIR} ${LIB_INSTALL_DIR})

if(KLU_LIBRARIES)

  if(NOT KLU_LIBDIR)
    get_filename_component(KLU_LIBDIR ${KLU_LIBRARIES} PATH)
  endif()

  find_library(COLAMD_LIBRARY colamd PATHS ${KLU_LIBDIR} $ENV{KLUDIR} ${LIB_INSTALL_DIR})
  if(COLAMD_LIBRARY)
    set(KLU_LIBRARIES ${KLU_LIBRARIES} ${COLAMD_LIBRARY})
  endif ()

  find_library(AMD_LIBRARY amd PATHS ${KLU_LIBDIR} $ENV{KLUDIR} ${LIB_INSTALL_DIR})
  if(AMD_LIBRARY)
    set(KLU_LIBRARIES ${KLU_LIBRARIES} ${AMD_LIBRARY})
  endif ()

  find_library(BTF_LIBRARY btf PATHS $ENV{KLU_LIBDIR} $ENV{KLUDIR} ${LIB_INSTALL_DIR})
  if(BTF_LIBRARY)
    set(KLU_LIBRARIES ${KLU_LIBRARIES} ${BTF_LIBRARY})
  endif()

endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(KLU DEFAULT_MSG
                                  KLU_INCLUDES KLU_LIBRARIES)

mark_as_advanced(KLU_INCLUDES KLU_LIBRARIES AMD_LIBRARY COLAMD_LIBRARY BTF_LIBRARY)
