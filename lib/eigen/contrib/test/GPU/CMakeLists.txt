# GPU module tests. Included from contrib/test/CMakeLists.txt inside the
# if(CUDA_FOUND AND EIGEN_TEST_CUDA) block.
# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

find_package(CUDAToolkit QUIET)

# These tests compile the GPU headers against the test harness, where
# eigen_assert throws. A noexcept function that reaches such an assert therefore
# calls std::terminate; the headers destroy CUDA events unchecked in their
# noexcept move assignments for exactly that reason. Both compilers already
# diagnose the pattern by default, under their own spelling; promoting it to an
# error keeps it from returning unnoticed. The move assignments are templates,
# so this binds the instantiations the tests below create, not the definitions.
set(EIGEN_GPU_TEST_CXX_FLAGS "")
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(EIGEN_GPU_TEST_CXX_FLAGS "-Werror=terminate")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(EIGEN_GPU_TEST_CXX_FLAGS "-Werror=exceptions")
endif()

# ei_add_gpu_test(<name> [EXTRA_LIBS <lib>...] [EXTRA_INCLUDES <dir>...] [EXTRA_DEFINES <def>...])
#
# Thin wrapper around ei_add_test for GPU module tests. Delegates to
# ei_add_test so CALL_SUBTEST_N splitting and EIGEN_TEST_MAX_SIZE handling
# work the same way as the rest of the test suite, then layers on the
# GPU-specific properties (EXTRA_{LIBS,INCLUDES,DEFINES}, the
# "Official;gpu" labels, SKIP_RETURN_CODE=77, buildtests_gpu dependency).
#
# GPU module tests are plain .cpp (host-compiled) by design: they
# orchestrate NVIDIA library calls and never launch kernels themselves,
# which avoids NVCC instantiating Eigen CPU packet ops for CUDA vector
# types. That keeps them outside ei_add_test's "is_gpu_test" path (which
# triggers on EIGEN_ADD_TEST_FILENAME_EXTENSION=cu), so we set the
# SKIP_RETURN_CODE and labels ourselves.
function(ei_add_gpu_test test_name)
  cmake_parse_arguments(ARG "" "" "EXTRA_LIBS;EXTRA_INCLUDES;EXTRA_DEFINES" ${ARGN})

  # Delegate to ei_add_test. The third arg is a semicolon-separated library
  # list; CUDA::cudart_static is always needed for any host test that
  # touches a device-allocated buffer.
  ei_add_test(${test_name} "${EIGEN_GPU_TEST_CXX_FLAGS}" "CUDA::cudart_static;${ARG_EXTRA_LIBS}")

  # Discover targets ei_add_test created. The scan mirrors ei_add_test's own
  # scan so we see the same set of suffixes: one target per distinct
  # CALL_SUBTEST_N on EIGEN_SPLIT_LARGE_TESTS, else a single target.
  file(READ "${test_name}.cpp" _src)
  string(REGEX MATCHALL "CALL_SUBTEST_[0-9]+" _matches "${_src}")
  string(REGEX REPLACE "CALL_SUBTEST_" "" _suffixes "${_matches}")
  list(REMOVE_DUPLICATES _suffixes)
  set(_targets)
  if(EIGEN_SPLIT_LARGE_TESTS AND _suffixes)
    foreach(s ${_suffixes})
      list(APPEND _targets "${test_name}_${s}")
    endforeach()
  else()
    list(APPEND _targets "${test_name}")
  endif()

  foreach(t ${_targets})
    # SYSTEM for the toolkit and backend headers: the -Werror above is aimed at
    # Eigen's own noexcept functions, not at whatever those ship.
    target_include_directories(${t} SYSTEM PRIVATE
      "${CUDA_TOOLKIT_ROOT_DIR}/include"
      ${ARG_EXTRA_INCLUDES})
    target_include_directories(${t} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
    if(ARG_EXTRA_DEFINES)
      target_compile_definitions(${t} PRIVATE ${ARG_EXTRA_DEFINES})
    endif()
    add_dependencies(buildtests_gpu ${t})
    set_property(TEST ${t} APPEND PROPERTY LABELS "Official;gpu")
    set_property(TEST ${t} PROPERTY SKIP_RETURN_CODE 77)
  endforeach()
endfunction()

# Compile-time GPU type-trait surface (static_assert only; links like device_matrix
# because it pulls the full <contrib/Eigen/GPU> umbrella).
ei_add_gpu_test(device_traits
  EXTRA_LIBS CUDA::cublas CUDA::cusolver CUDA::npps CUDA::nppc)

# DeviceMatrix core: CUDA runtime + cuBLAS + cuSOLVER (for BLAS-1 ops via GpuContext).
ei_add_gpu_test(device_matrix
  EXTRA_LIBS CUDA::cublas CUDA::cusolver CUDA::npps CUDA::nppc)
ei_add_gpu_test(cublas_64bit EXTRA_LIBS CUDA::cublas)

# DeviceBufferPool release ordering: CUDA runtime only, plus std::thread for the stream gate.
find_package(Threads REQUIRED)
ei_add_gpu_test(device_buffer_pool EXTRA_LIBS Threads::Threads)

# cuBLAS integration. OFF by default; later MRs in the stack opt in.
option(EIGEN_TEST_CUBLAS "Test cuBLAS integration" OFF)
if(EIGEN_TEST_CUBLAS AND TARGET CUDA::cublas)
  ei_add_gpu_test(cublas EXTRA_LIBS CUDA::cublas CUDA::cusolver)
endif()

# cuSOLVER dense factorizations (LLT, LU, QR, SVD, self-adjoint eigensolver).
option(EIGEN_TEST_CUSOLVER "Test cuSOLVER integration" OFF)
if(EIGEN_TEST_CUSOLVER AND TARGET CUDA::cusolver)
  find_package(Threads REQUIRED)
  ei_add_gpu_test(cusolver_llt EXTRA_LIBS CUDA::cusolver CUDA::cublas Threads::Threads)
  foreach(_cusolver_test IN ITEMS cusolver_lu cusolver_qr cusolver_svd cusolver_eigen)
    ei_add_gpu_test(${_cusolver_test} EXTRA_LIBS CUDA::cusolver CUDA::cublas)
  endforeach()
endif()

# cuFFT (part of the CUDA toolkit, no separate option needed).
if(TARGET CUDA::cufft)
  ei_add_gpu_test(cufft EXTRA_LIBS CUDA::cufft CUDA::cublas)
endif()

# cuSPARSE SpMV (part of the CUDA toolkit) and the end-to-end CG test that
# exercises Eigen::ConjugateGradient with DeviceMatrix + device-resident SpMV.
if(TARGET CUDA::cusparse)
  ei_add_gpu_test(cusparse_spmv
    EXTRA_LIBS CUDA::cusparse CUDA::cublas CUDA::cusolver)
  ei_add_gpu_test(cg
    EXTRA_LIBS CUDA::cusparse CUDA::cublas CUDA::cusolver CUDA::npps CUDA::nppc)
endif()

# cuDSS sparse direct solvers -- distributed separately from the CUDA Toolkit.
option(EIGEN_TEST_CUDSS "Test cuDSS sparse solver integration" OFF)
if(EIGEN_TEST_CUDSS)
  find_path(CUDSS_INCLUDE_DIR cudss.h
    HINTS ${CUDSS_DIR}/include ${CUDA_TOOLKIT_ROOT_DIR}/include /usr/include)
  find_library(CUDSS_LIBRARY cudss
    HINTS ${CUDSS_DIR}/lib ${CUDSS_DIR}/lib64 ${CUDA_TOOLKIT_ROOT_DIR}/lib64 /usr/lib/x86_64-linux-gnu)
  if(CUDSS_INCLUDE_DIR AND CUDSS_LIBRARY)
    message(STATUS "cuDSS found: ${CUDSS_LIBRARY}")
    foreach(_cudss_test IN ITEMS cudss_llt cudss_ldlt cudss_lu)
      ei_add_gpu_test(${_cudss_test}
        EXTRA_LIBS CUDA::cusolver CUDA::cublas ${CUDSS_LIBRARY}
        EXTRA_INCLUDES ${CUDSS_INCLUDE_DIR}
        EXTRA_DEFINES EIGEN_CUDSS=1)
    endforeach()
  else()
    message(WARNING "EIGEN_TEST_CUDSS=ON but cuDSS not found. Set CUDSS_DIR.")
  endif()
endif()
