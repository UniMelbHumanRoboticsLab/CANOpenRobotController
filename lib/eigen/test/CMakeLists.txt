# The file split_test_helper.h was generated at first run,
# it is now included in test/
# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0
if(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/split_test_helper.h)
  file(REMOVE ${CMAKE_CURRENT_BINARY_DIR}/split_test_helper.h)
endif()

# check if we have a Fortran compiler
include(CheckLanguage)
check_language(Fortran)
if(CMAKE_Fortran_COMPILER)
  enable_language(Fortran)
  set(EIGEN_Fortran_COMPILER_WORKS ON)
else()
  set(EIGEN_Fortran_COMPILER_WORKS OFF)
  # search for a default Lapack library to complete Eigen's one
  find_package(LAPACK QUIET)
endif()

# TODO do the same for EXTERNAL_LAPACK
option(EIGEN_TEST_EXTERNAL_BLAS "Use external BLAS library for testsuite" OFF)
option(EIGEN_TEST_EXTERNAL_BLAS_INT64 "Use the 64-bit integer (ILP64) interface of the external BLAS" OFF)
set(EIGEN_TEST_EXTERNAL_BLAS_SYMBOL_SUFFIX "" CACHE STRING
    "Symbol suffix of the external BLAS 64-bit interface, e.g. _64 for dgemm_64_")
# Check that the BLAS integer width Eigen is configured for is the one the linked library actually uses.
# Fortran BLAS takes every dimension by pointer, so a mismatch is invisible to the compiler and the linker and
# only shows up as corrupted arguments at run time.
#
# The probe calls dgemm passing one 8-byte slot for every integer argument, holding 1 in its low word and -1 in
# its high word. A 32-bit callee reads 1 and computes 3*5; a 64-bit callee reads a negative extent and bails out
# through xerbla without touching memory. Detection is by output rather than exit status, because the reference
# xerbla ends in a Fortran STOP, which skips the rest of main and still exits 0.
function(eigen_check_external_blas_integer_size)
  if(CMAKE_CROSSCOMPILING)
    message(STATUS "External BLAS: cross-compiling, BLAS integer width not checked")
    return()
  endif()
  set(probe_dir "${CMAKE_CURRENT_BINARY_DIR}/eigen_blas_int_probe")
  set(probe_src "${probe_dir}/probe.cpp")
  # Bracket argument: the suffix arrives as a compile definition, so nothing here needs escaping.
  file(WRITE "${probe_src}" [==[
#include <cstdint>
#include <cstdio>
#define EIGEN_PROBE_CAT2(a, b) a##b
#define EIGEN_PROBE_CAT(a, b) EIGEN_PROBE_CAT2(a, b)
#define EIGEN_PROBE_DGEMM EIGEN_PROBE_CAT(EIGEN_PROBE_CAT(dgemm, EIGEN_PROBE_SUFFIX), _)
extern "C" void EIGEN_PROBE_DGEMM(const void*, const void*, const void*, const void*, const void*, const double*,
                                  const double*, const void*, const double*, const void*, const double*, double*,
                                  const void*);
int main() {
  std::int64_t slot = std::int64_t(1) | (std::int64_t(0xFFFFFFFF) << 32);
  double alpha = 1.0, beta = 0.0, a[1] = {3.0}, b[1] = {5.0}, c[1] = {-999.0};
  std::printf("EIGEN_PROBE_RAN\n");
  std::fflush(stdout);
  EIGEN_PROBE_DGEMM("N", "N", &slot, &slot, &slot, &alpha, a, &slot, b, &slot, &beta, c, &slot);
  if (c[0] == 15.0) {
    std::printf("EIGEN_PROBE_IS32\n");
    std::fflush(stdout);
  }
  return 0;
}
]==])
  try_run(probe_run probe_built "${probe_dir}" "${probe_src}"
          COMPILE_DEFINITIONS "-DEIGEN_PROBE_SUFFIX=${EIGEN_TEST_EXTERNAL_BLAS_SYMBOL_SUFFIX}"
          LINK_LIBRARIES ${BLAS_LIBRARIES}
          COMPILE_OUTPUT_VARIABLE probe_build_output
          RUN_OUTPUT_VARIABLE probe_output)
  if(NOT probe_built)
    message(FATAL_ERROR
            "External BLAS: could not link dgemm${EIGEN_TEST_EXTERNAL_BLAS_SYMBOL_SUFFIX}_ against "
            "${BLAS_LIBRARIES}. Check EIGEN_TEST_EXTERNAL_BLAS_SYMBOL_SUFFIX.\n${probe_build_output}")
  endif()
  if(NOT probe_output MATCHES "EIGEN_PROBE_RAN")
    message(WARNING "External BLAS: integer-width probe did not run; BLAS integer width not checked")
    return()
  endif()
  if(probe_output MATCHES "EIGEN_PROBE_IS32")
    set(library_is_int64 FALSE)
  else()
    set(library_is_int64 TRUE)
  endif()
  if(library_is_int64 AND NOT EIGEN_TEST_EXTERNAL_BLAS_INT64)
    message(FATAL_ERROR "External BLAS: ${BLAS_LIBRARIES} uses 64-bit integers. Set "
                        "EIGEN_TEST_EXTERNAL_BLAS_INT64=ON, or point BLAS_LIBRARIES at an LP64 library.")
  elseif(NOT library_is_int64 AND EIGEN_TEST_EXTERNAL_BLAS_INT64)
    message(FATAL_ERROR "External BLAS: ${BLAS_LIBRARIES} uses 32-bit integers. Set "
                        "EIGEN_TEST_EXTERNAL_BLAS_INT64=OFF, or point BLAS_LIBRARIES at an ILP64 library.")
  endif()
  if(library_is_int64)
    message(STATUS "External BLAS: confirmed 64-bit (ILP64) integer interface")
  else()
    message(STATUS "External BLAS: confirmed 32-bit (LP64) integer interface")
  endif()
endfunction()

if(EIGEN_TEST_EXTERNAL_BLAS)
  find_package(BLAS REQUIRED)
  message(STATUS "BLAS_COMPILER_FLAGS: ${BLAS_COMPILER_FLAGS}")
  add_definitions("-DEIGEN_USE_BLAS") # is adding  ${BLAS_COMPILER_FLAGS} necessary?
  if(EIGEN_TEST_EXTERNAL_BLAS_INT64)
    add_definitions("-DEIGEN_64BIT_BLAS")
  endif()
  if(NOT EIGEN_TEST_EXTERNAL_BLAS_SYMBOL_SUFFIX STREQUAL "")
    add_definitions("-DEIGEN_BLAS_SYMBOL_SUFFIX=${EIGEN_TEST_EXTERNAL_BLAS_SYMBOL_SUFFIX}")
  endif()
  eigen_check_external_blas_integer_size()
  list(APPEND EXTERNAL_LIBS "${BLAS_LIBRARIES}")
endif()

# configure blas/lapack (use Eigen's ones)
set(EIGEN_BLAS_LIBRARIES eigen_blas)
set(EIGEN_LAPACK_LIBRARIES eigen_lapack)

set(EIGEN_TEST_MATRIX_DIR "" CACHE STRING "Enable testing of realword sparse matrices contained in the specified path")
if(EIGEN_TEST_MATRIX_DIR)
  if(NOT WIN32)
    message(STATUS "Test realworld sparse matrices: ${EIGEN_TEST_MATRIX_DIR}")
    add_definitions( -DTEST_REAL_CASES="${EIGEN_TEST_MATRIX_DIR}" )
  else()
    message(STATUS "REAL CASES CAN NOT BE CURRENTLY TESTED ON WIN32")
  endif()
endif()

set(SPARSE_LIBS " ")

find_package(CHOLMOD)
if(CHOLMOD_FOUND AND EIGEN_BUILD_BLAS AND EIGEN_BUILD_LAPACK)
  add_definitions("-DEIGEN_CHOLMOD_SUPPORT")
  include_directories(${CHOLMOD_INCLUDES})
  set(SPARSE_LIBS ${SPARSE_LIBS} ${CHOLMOD_LIBRARIES} ${EIGEN_BLAS_LIBRARIES} ${EIGEN_LAPACK_LIBRARIES})
  set(CHOLMOD_ALL_LIBS  ${CHOLMOD_LIBRARIES} ${EIGEN_BLAS_LIBRARIES} ${EIGEN_LAPACK_LIBRARIES})
  ei_add_property(EIGEN_TESTED_BACKENDS "CHOLMOD, ")

  ei_add_test(cholmod_support "" "${CHOLMOD_ALL_LIBS}")
else()
  ei_add_property(EIGEN_MISSING_BACKENDS "CHOLMOD, ")
endif()

find_package(UMFPACK)
if(UMFPACK_FOUND AND EIGEN_BUILD_BLAS)
  add_definitions("-DEIGEN_UMFPACK_SUPPORT")
  include_directories(${UMFPACK_INCLUDES})
  set(SPARSE_LIBS ${SPARSE_LIBS} ${UMFPACK_LIBRARIES} ${EIGEN_BLAS_LIBRARIES})
  set(UMFPACK_ALL_LIBS ${UMFPACK_LIBRARIES} ${EIGEN_BLAS_LIBRARIES})
  ei_add_property(EIGEN_TESTED_BACKENDS "UMFPACK, ")

  ei_add_test(umfpack_support "" "${UMFPACK_ALL_LIBS}")
else()
  ei_add_property(EIGEN_MISSING_BACKENDS "UMFPACK, ")
endif()

find_package(KLU)
if(KLU_FOUND AND EIGEN_BUILD_BLAS)
  add_definitions("-DEIGEN_KLU_SUPPORT")
  include_directories(${KLU_INCLUDES})
  set(SPARSE_LIBS ${SPARSE_LIBS} ${KLU_LIBRARIES} ${EIGEN_BLAS_LIBRARIES})
  set(KLU_ALL_LIBS ${KLU_LIBRARIES} ${EIGEN_BLAS_LIBRARIES})
  ei_add_property(EIGEN_TESTED_BACKENDS "KLU, ")

  ei_add_test(klu_support "" "${KLU_ALL_LIBS}")
else()
  ei_add_property(EIGEN_MISSING_BACKENDS "KLU, ")
endif()

find_package(SuperLU 4.0)
if(SuperLU_FOUND AND EIGEN_BUILD_BLAS)
  add_definitions("-DEIGEN_SUPERLU_SUPPORT")
  include_directories(${SUPERLU_INCLUDES})
  set(SPARSE_LIBS ${SPARSE_LIBS} ${SUPERLU_LIBRARIES} ${EIGEN_BLAS_LIBRARIES})
  set(SUPERLU_ALL_LIBS ${SUPERLU_LIBRARIES} ${EIGEN_BLAS_LIBRARIES})
  ei_add_property(EIGEN_TESTED_BACKENDS  "SuperLU, ")

  ei_add_test(superlu_support "" "${SUPERLU_ALL_LIBS}")
else()
  ei_add_property(EIGEN_MISSING_BACKENDS  "SuperLU, ")
endif()


find_package(PASTIX QUIET COMPONENTS METIS SEQ)
# check that the PASTIX found is a version without MPI
find_path(PASTIX_pastix_nompi.h_INCLUDE_DIRS
  NAMES pastix_nompi.h
  HINTS ${PASTIX_INCLUDE_DIRS}
)
if (NOT PASTIX_pastix_nompi.h_INCLUDE_DIRS)
  message(STATUS "A version of Pastix has been found but pastix_nompi.h does not exist in the include directory."
                 " Because Eigen tests require a version without MPI, we disable the Pastix backend.")
endif()
if(PASTIX_FOUND AND PASTIX_pastix_nompi.h_INCLUDE_DIRS)
  add_definitions("-DEIGEN_PASTIX_SUPPORT")
  include_directories(${PASTIX_INCLUDE_DIRS_DEP})
  if(SCOTCH_FOUND)
    include_directories(${SCOTCH_INCLUDE_DIRS})
    set(PASTIX_LIBRARIES ${PASTIX_LIBRARIES} ${SCOTCH_LIBRARIES})
  elseif(METIS_FOUND)
    include_directories(${METIS_INCLUDE_DIRS})
    set(PASTIX_LIBRARIES ${PASTIX_LIBRARIES} ${METIS_LIBRARIES})
  else()
    ei_add_property(EIGEN_MISSING_BACKENDS  "PaStiX, ")
  endif()
  set(SPARSE_LIBS ${SPARSE_LIBS} ${PASTIX_LIBRARIES_DEP} ${ORDERING_LIBRARIES})
  set(PASTIX_ALL_LIBS ${PASTIX_LIBRARIES_DEP})
  ei_add_property(EIGEN_TESTED_BACKENDS  "PaStiX, ")
else()
  ei_add_property(EIGEN_MISSING_BACKENDS  "PaStiX, ")
endif()

if(METIS_FOUND)
  add_definitions("-DEIGEN_METIS_SUPPORT")
  include_directories(${METIS_INCLUDE_DIRS})
  ei_add_property(EIGEN_TESTED_BACKENDS "METIS, ")
else()
  ei_add_property(EIGEN_MISSING_BACKENDS "METIS, ")
endif()

find_package(SPQR)
if(SPQR_FOUND AND CHOLMOD_FOUND AND EIGEN_BUILD_BLAS AND EIGEN_BUILD_LAPACK AND (EIGEN_Fortran_COMPILER_WORKS OR LAPACK_FOUND) )
  add_definitions("-DEIGEN_SPQR_SUPPORT")
  include_directories(${SPQR_INCLUDES})
  set(SPQR_ALL_LIBS ${SPQR_LIBRARIES} ${CHOLMOD_LIBRARIES} ${EIGEN_LAPACK_LIBRARIES} ${EIGEN_BLAS_LIBRARIES} ${LAPACK_LIBRARIES})
  set(SPARSE_LIBS ${SPARSE_LIBS} ${SPQR_ALL_LIBS})
  ei_add_property(EIGEN_TESTED_BACKENDS "SPQR, ")
else()
  ei_add_property(EIGEN_MISSING_BACKENDS "SPQR, ")
endif()

find_package(Accelerate)
if(Accelerate_FOUND)
  add_definitions("-DEIGEN_ACCELERATE_SUPPORT")
  include_directories(${Accelerate_INCLUDES})
  set(SPARSE_LIBS ${SPARSE_LIBS} ${Accelerate_LIBRARIES})
  set(Accelerate_ALL_LIBS ${Accelerate_LIBRARIES})
  ei_add_property(EIGEN_TESTED_BACKENDS "Accelerate, ")
else()
  ei_add_property(EIGEN_MISSING_BACKENDS "Accelerate, ")
endif()

option(EIGEN_TEST_NOQT "Disable Qt support in unit tests" OFF)
if(NOT EIGEN_TEST_NOQT)
  find_package(Qt4)
  if(QT4_FOUND)
    include(${QT_USE_FILE})
    ei_add_property(EIGEN_TESTED_BACKENDS  "Qt4 support, ")
  else()
    ei_add_property(EIGEN_MISSING_BACKENDS  "Qt4 support, ")
  endif()
endif()

if(TEST_LIB)
  add_definitions("-DEIGEN_EXTERN_INSTANTIATIONS=1")
endif()

set_property(GLOBAL PROPERTY EIGEN_CURRENT_SUBPROJECT "Official")
add_custom_target(BuildOfficial)

# Not an ei_add_test: this checks values produced by the configure step itself,
# so it compares what the enclosing project() call left in this scope.
add_test(NAME cmake_project_version
         COMMAND ${CMAKE_COMMAND}
                 "-DEIGEN_EXPECTED_VERSION=${EIGEN_VERSION_NUMBER}"
                 "-DEIGEN_IS_TOP_LEVEL=${PROJECT_IS_TOP_LEVEL}"
                 "-DOBSERVED_PROJECT_VERSION=${PROJECT_VERSION}"
                 "-DOBSERVED_Eigen3_VERSION=${Eigen3_VERSION}"
                 "-DOBSERVED_CMAKE_PROJECT_VERSION=${CMAKE_PROJECT_VERSION}"
                 -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake_project_version.cmake")
set_property(TEST cmake_project_version PROPERTY LABELS "Official")

ei_add_test(clz)
ei_add_test(rand)
ei_add_test(realview)
ei_add_test(meta)
ei_add_test(preprocessor_directives)
ei_add_test(lru_cache)
ei_add_test(maxsizevector)
ei_add_test(numext)
ei_add_test(numext_fma_off)
# Reuse the existing GCC AVX2 configuration's -mfma for a small scalar contraction regression.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND EIGEN_TEST_AVX2)
  ei_add_test(numext_fma_contraction "-ffp-contract=fast -DEIGEN_DONT_VECTORIZE")
endif()
ei_add_test(safe_scaling)
ei_add_test(sizeof)
ei_add_test(dynalloc)
ei_add_test(nomalloc)
ei_add_test(stack_allocation_limit)
ei_add_test(first_aligned)
ei_add_test(cache_sizes)
ei_add_test(type_alias)
ei_add_test(nullary)
ei_add_test(mixingtypes)
ei_add_test(float_conversion)
ei_add_test(io)
ei_add_test(packetmath "-DEIGEN_FAST_MATH=1")
# Generic clang vector backend tests for different vector sizes.
include(CheckCXXSourceCompiles)
check_cxx_source_compiles("
  typedef float v4sf __attribute__((ext_vector_type(4)));
  int main() { return __builtin_vectorelements(v4sf{}); }
" COMPILER_SUPPORTS_VECTOR_EXTENSIONS)
if(COMPILER_SUPPORTS_VECTOR_EXTENSIONS)
  ei_add_test(packetmath_generic_16 "-DEIGEN_FAST_MATH=1")
  ei_add_test(packetmath_generic_32 "-DEIGEN_FAST_MATH=1")
  ei_add_test(packetmath_generic_64 "-DEIGEN_FAST_MATH=1")
  ei_add_test(mixingtypes_generic_32)
  ei_add_test(mixingtypes_generic_64)
endif()
ei_add_test(packet_segment)
ei_add_test(vectorization_logic)
ei_add_test(basicstuff)
ei_add_test(constexpr)
ei_add_test(constructor)
ei_add_test(linearstructure)
ei_add_test(integer_types)
ei_add_test(unalignedcount)
if(NOT EIGEN_TEST_NO_EXCEPTIONS AND NOT EIGEN_TEST_OPENMP)
  ei_add_test(exceptions)
endif()
ei_add_test(redux)
ei_add_test(visitor)
ei_add_test(block)
ei_add_test(corners)
ei_add_test(symbolic_index)
ei_add_test(indexed_view)
ei_add_test(reshape)
ei_add_test(swap)
ei_add_test(resize)
ei_add_test(no_automatic_resizing)
ei_add_test(conservative_resize)
ei_add_test(product_small)
ei_add_test(product_large)
if(EIGEN_TEST_SME)
  # EIGEN_TEST_SME defines EIGEN_ARM64_USE_SME (root CMakeLists.txt), which turns
  # a toolchain without SME2 into a build error rather than a NEON fallback; the
  # toolchain must still supply an SME2 -march/-mcpu.
  ei_add_test(product_sme)
endif()
if(EIGEN_TEST_SVE)
  # EIGEN_TEST_SVE defines EIGEN_ARM64_USE_SVE and -msve-vector-bits (root
  # CMakeLists.txt); the toolchain must still supply an SVE -march/-mcpu.
  # This guards every other SVE result: if the runner's vector length does not
  # match the one the packets were built for, the rest of the suite is vacuous.
  ei_add_test(sve_vector_length)
endif()
ei_add_test(product_extra)
ei_add_test(diagonalmatrices)
ei_add_test(diagonalview)
ei_add_test(skew_symmetric_matrix3)
ei_add_test(adjoint)
ei_add_test(diagonal)
ei_add_test(miscmatrices)
ei_add_test(commainitializer)
ei_add_test(smallvectors)
ei_add_test(mapped_matrix)
ei_add_test(mapstride)
ei_add_test(unaryview)
ei_add_test(mapstaticmethods)
ei_add_test(array_cwise)
ei_add_test(matrix_cwise)
ei_add_test(array_for_matrix)
ei_add_test(array_replicate)
ei_add_test(concat)
ei_add_test(array_reverse)
ei_add_test(ref)
ei_add_test(is_same_dense)
ei_add_test(triangular)
ei_add_test(triangular_in_place)
ei_add_test(selfadjoint)
ei_add_test(product_selfadjoint)
ei_add_test(product_symm)
ei_add_test(product_syrk)
ei_add_test(product_trmv)
ei_add_test(product_trmm)
ei_add_test(product_trsolve)
ei_add_test(product_mmtr)
ei_add_test(product_notemporary)
ei_add_test(product_threaded "-pthread" "${CMAKE_THREAD_LIBS_INIT}")
ei_add_test(stable_norm)
ei_add_test(permutationmatrices)
ei_add_test(bandmatrix)
ei_add_test(cholesky)
ei_add_test(bunchkaufman)
ei_add_test(condition_estimator)
ei_add_test(lu)
ei_add_test(determinant)
ei_add_test(inverse)
ei_add_test(qr)
ei_add_test(qr_colpivoting)
ei_add_test(qr_rand_colpivoting)
ei_add_test(qr_fullpivoting)
ei_add_test(upperbidiagonalization)
ei_add_test(hessenberg)
ei_add_test(schur_real)
ei_add_test(schur_complex)
ei_add_test(eigensolver_selfadjoint)
ei_add_test(tridiagonal_eigensolver)
ei_add_test(eigensolver_generic)
ei_add_test(eigensolver_complex)
ei_add_test(real_qz)
ei_add_test(complex_qz)
ei_add_test(eigensolver_generalized_real)
ei_add_test(jacobi)
ei_add_test(jacobisvd)
ei_add_test(bdcsvd)
ei_add_test(householder)
ei_add_test(geo_orthomethods)
ei_add_test(geo_quaternion)
ei_add_test(geo_eulerangles)
ei_add_test(geo_parametrizedline)
ei_add_test(geo_alignedbox)
ei_add_test(geo_hyperplane)
ei_add_test(geo_transformations)
ei_add_test(geo_homogeneous)
ei_add_test(geo_misc)
ei_add_test(stdvector)
ei_add_test(stdvector_overload)
ei_add_test(stdlist)
ei_add_test(stdlist_overload)
ei_add_test(stddeque)
ei_add_test(stddeque_overload)
ei_add_test(block_sparse_matrix)
ei_add_test(sparse_basic)
ei_add_test(sparse_block)
ei_add_test(sparse_vector)
ei_add_test(sparse_product)
ei_add_test(sparse_threaded_product "-pthread" "${CMAKE_THREAD_LIBS_INIT}")
ei_add_test(sparse_ref)
ei_add_test(sparse_solvers)
ei_add_test(sparse_permutations)
ei_add_test(simplicial_cholesky)
ei_add_test(conjugate_gradient)
ei_add_test(incomplete_cholesky)
ei_add_test(incomplete_LUT)
ei_add_test(bicgstab)
ei_add_test(lscg)
ei_add_test(lsmr)
ei_add_test(gmres)
ei_add_test(dgmres)
ei_add_test(minres)
ei_add_test(idrs)
ei_add_test(bicgstabl)
ei_add_test(idrstabl)
ei_add_test(sparselu)
ei_add_test(sparseqr)
ei_add_test(sparse_ordering)
ei_add_test(umeyama)
ei_add_test(nesting_ops "${CMAKE_CXX_FLAGS_DEBUG}")
ei_add_test(nestbyvalue)
ei_add_test(zerosized)
ei_add_test(dontalign)
ei_add_test(evaluators)
if(NOT EIGEN_TEST_NO_EXCEPTIONS)
  ei_add_test(sizeoverflow)
endif()
ei_add_test(prec_inverse_4x4)
ei_add_test(vectorwiseop)
ei_add_test(special_numbers)
ei_add_test(rvalue_types)
ei_add_test(dense_storage)
ei_add_test(ctorleak)
ei_add_test(inplace_decomposition)
ei_add_test(half_float)
ei_add_test(bfloat16_float)
ei_add_test(bfloat16_classification)
ei_add_test(array_of_string)
ei_add_test(num_dimensions)
ei_add_test(stl_iterators)
ei_add_test(blasutil)
ei_add_test(random_matrix)
ei_add_test(initializer_list_construction)
ei_add_test(diagonal_matrix_variadic_ctor)
ei_add_test(serializer)
ei_add_test(special_members)
ei_add_test(tuple_test)
ei_add_test(structured_bindings)
ei_add_test(threads_eventcount "-pthread" "${CMAKE_THREAD_LIBS_INIT}")
ei_add_test(threads_runqueue "-pthread" "${CMAKE_THREAD_LIBS_INIT}")
ei_add_test(threads_non_blocking_thread_pool "-pthread" "${CMAKE_THREAD_LIBS_INIT}")
ei_add_test(threads_fork_join "-pthread" "${CMAKE_THREAD_LIBS_INIT}")
ei_add_test(assignment_threaded "-pthread" "${CMAKE_THREAD_LIBS_INIT}")
add_executable(bug1213 bug1213.cpp bug1213_main.cpp)
target_link_libraries(bug1213 Eigen3::Eigen)
# ei_add_test applies EIGEN_TEST_CUSTOM_CXX_FLAGS to every target it registers;
# a bare add_executable has to ask. Without it this is the one test target built
# at the configuration's default -march, which is not what the job selected.
if(EIGEN_TEST_CUSTOM_CXX_FLAGS)
  target_compile_options(bug1213 PRIVATE ${EIGEN_TEST_CUSTOM_CXX_FLAGS})
endif()

check_cxx_compiler_flag("-ffast-math" COMPILER_SUPPORT_FASTMATH)
if(COMPILER_SUPPORT_FASTMATH)
  set(EIGEN_FASTMATH_FLAGS "-ffast-math")
else()
  check_cxx_compiler_flag("/fp:fast" COMPILER_SUPPORT_FPFAST)
  if(COMPILER_SUPPORT_FPFAST)
    set(EIGEN_FASTMATH_FLAGS "/fp:fast")
  endif()
endif()
# The fastmath test intentionally uses NaN/infinity under -ffast-math.
# Suppress the clang warning about this being technically undefined.
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  check_cxx_compiler_flag("-Wno-nan-infinity-disabled" COMPILER_SUPPORT_WNO_NAN_INF)
  if(COMPILER_SUPPORT_WNO_NAN_INF)
    set(EIGEN_FASTMATH_FLAGS "${EIGEN_FASTMATH_FLAGS} -Wno-nan-infinity-disabled")
  endif()
endif()

if(COMPILER_SUPPORT_FASTMATH)
  ei_add_test(packetmath_fastmath "-O2 ${EIGEN_FASTMATH_FLAGS}")
elseif(COMPILER_SUPPORT_FPFAST)
  ei_add_test(packetmath_fastmath "${EIGEN_FASTMATH_FLAGS}")
endif()
if(COMPILER_SUPPORT_FASTMATH AND COMPILER_SUPPORTS_VECTOR_EXTENSIONS)
  ei_add_test(packetmath_fastmath_generic_16 "-O2 ${EIGEN_FASTMATH_FLAGS} -Werror")
  ei_add_test(packetmath_fastmath_generic_32 "-O2 ${EIGEN_FASTMATH_FLAGS} -Werror")
  ei_add_test(packetmath_fastmath_generic_64 "-O2 ${EIGEN_FASTMATH_FLAGS} -Werror")
endif()

ei_add_test(fastmath "${EIGEN_FASTMATH_FLAGS}")
ei_add_test(bdcsvd_fastmath "${EIGEN_FASTMATH_FLAGS}")
ei_add_test(bfloat16_classification_fastmath "${EIGEN_FASTMATH_FLAGS}")
ei_add_test(eigensolver_selfadjoint_fastmath "${EIGEN_FASTMATH_FLAGS}")
ei_add_test(stable_norm_fastmath "${EIGEN_FASTMATH_FLAGS}")
ei_add_test(householder_fastmath "${EIGEN_FASTMATH_FLAGS}")

# # ei_add_test(denseLM)

if(QT4_FOUND)
  ei_add_test(qtvector "" "${QT_QTCORE_LIBRARY}")
endif()

if(PARDISO_FOUND)
  ei_add_test(pardiso_support "" "${PARDISO_ALL_LIBS}")
endif()

if(PASTIX_FOUND AND (SCOTCH_FOUND OR METIS_FOUND))
  ei_add_test(pastix_support "" "${PASTIX_ALL_LIBS}")
endif()

if(SPQR_FOUND AND CHOLMOD_FOUND AND EIGEN_BUILD_BLAS AND EIGEN_BUILD_LAPACK)
  ei_add_test(spqr_support "" "${SPQR_ALL_LIBS}")
endif()

if(METIS_FOUND)
ei_add_test(metis_support "" "${METIS_LIBRARIES}")
endif()

if(Accelerate_FOUND)
  ei_add_test(accelerate_support "" "${Accelerate_ALL_LIBS}")
endif()

string(TOLOWER "${CMAKE_CXX_COMPILER}" cmake_cxx_compiler_tolower)
if(cmake_cxx_compiler_tolower MATCHES "qcc")
  set(CXX_IS_QCC "ON")
endif()

ei_add_property(EIGEN_TESTING_SUMMARY "CXX:               ${CMAKE_CXX_COMPILER}\n")
if(CMAKE_COMPILER_IS_GNUCXX AND NOT CXX_IS_QCC)
  execute_process(COMMAND ${CMAKE_CXX_COMPILER} --version COMMAND head -n 1 OUTPUT_VARIABLE EIGEN_CXX_VERSION_STRING OUTPUT_STRIP_TRAILING_WHITESPACE)
  ei_add_property(EIGEN_TESTING_SUMMARY "CXX_VERSION:       ${EIGEN_CXX_VERSION_STRING}\n")
endif()
ei_add_property(EIGEN_TESTING_SUMMARY "CXX_FLAGS:         ${CMAKE_CXX_FLAGS}\n")
if (EIGEN_TEST_CUSTOM_CXX_FLAGS)
  ei_add_property(EIGEN_TESTING_SUMMARY "Custom CXX flags:  ${EIGEN_TEST_CUSTOM_CXX_FLAGS}\n")
endif()
ei_add_property(EIGEN_TESTING_SUMMARY "Sparse lib flags:  ${SPARSE_LIBS}\n")

# boost MP unit test
find_package(Boost 1.53.0 CONFIG)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
  ei_add_test(boostmultiprec "" "${Boost_LIBRARIES}")
  ei_add_property(EIGEN_TESTED_BACKENDS "Boost.Multiprecision, ")
else()
  ei_add_property(EIGEN_MISSING_BACKENDS "Boost.Multiprecision, ")
endif()


# CUDA unit tests
option(EIGEN_TEST_CUDA "Enable CUDA support in unit tests" OFF)
option(EIGEN_TEST_CUDA_CLANG "Use clang instead of nvcc to compile the CUDA tests" OFF)
option(EIGEN_TEST_CUDA_NVC "Use nvc++ (NVHPC) instead of nvcc to compile the CUDA tests" OFF)

if(EIGEN_TEST_CUDA_CLANG AND NOT CMAKE_CXX_COMPILER MATCHES "clang")
  message(WARNING "EIGEN_TEST_CUDA_CLANG is set, but CMAKE_CXX_COMPILER does not appear to be clang.")
endif()

if(EIGEN_TEST_CUDA_NVC AND NOT CMAKE_CXX_COMPILER_ID MATCHES "NVHPC")
  message(WARNING "EIGEN_TEST_CUDA_NVC is set, but CMAKE_CXX_COMPILER does not appear to be nvc++.")
endif()

find_package(CUDA 11.4)
if(CUDA_FOUND AND EIGEN_TEST_CUDA)
  # Make sure to compile without the -pedantic, -Wundef, -Wnon-virtual-dtor
  # and -fno-check-new flags since they trigger thousands of compilation warnings
  # in the CUDA runtime
  string(REPLACE "-pedantic" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  string(REPLACE "-Wundef" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  string(REPLACE "-Wnon-virtual-dtor" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  string(REPLACE "-fno-check-new" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

  ei_cuda_resolve_compute_arch()

  if(EIGEN_TEST_CUDA_CLANG)
    string(APPEND CMAKE_CXX_FLAGS " --cuda-path=${CUDA_TOOLKIT_ROOT_DIR}")
    foreach(GPU IN LISTS EIGEN_CUDA_COMPUTE_ARCH)
      string(APPEND CMAKE_CXX_FLAGS " --cuda-gpu-arch=sm_${GPU}")
    endforeach()
    string(APPEND CMAKE_CXX_FLAGS " ${EIGEN_CUDA_CXX_FLAGS}")
  elseif(EIGEN_TEST_CUDA_NVC)
    string(APPEND CMAKE_CXX_FLAGS " -cuda")
    foreach(GPU IN LISTS EIGEN_CUDA_COMPUTE_ARCH)
      string(APPEND CMAKE_CXX_FLAGS " -gpu=cc${GPU}")
    endforeach()
    string(APPEND CMAKE_CXX_FLAGS " ${EIGEN_CUDA_CXX_FLAGS}")
  else()
    set(NVCC_ARCH_FLAGS)
    # Define an -arch=sm_<arch>, otherwise if GPU does not exactly match one of
    # those in the arch list for -gencode, the kernels will fail to run with
    #    cudaErrorNoKernelImageForDevice
    # This can happen with newer cards (e.g. sm_75) and compiling with older
    # versions of nvcc (e.g. 9.2) that do not support their specific arch.
    list(LENGTH EIGEN_CUDA_COMPUTE_ARCH EIGEN_CUDA_COMPUTE_ARCH_SIZE)
    if(EIGEN_CUDA_COMPUTE_ARCH_SIZE)
      list(GET EIGEN_CUDA_COMPUTE_ARCH 0 EIGEN_CUDA_COMPUTE_DEFAULT)
      set(NVCC_ARCH_FLAGS " -arch=sm_${EIGEN_CUDA_COMPUTE_DEFAULT}")
    endif()
    foreach(ARCH IN LISTS EIGEN_CUDA_COMPUTE_ARCH)
      string(APPEND NVCC_ARCH_FLAGS " -gencode arch=compute_${ARCH},code=sm_${ARCH}")
    endforeach()
    set(CUDA_NVCC_FLAGS  "--expt-relaxed-constexpr -Xcudafe \"--display_error_number\" ${NVCC_ARCH_FLAGS} ${CUDA_NVCC_FLAGS} ${EIGEN_CUDA_CXX_FLAGS}")
    cuda_include_directories("${CMAKE_CURRENT_BINARY_DIR}" "${CUDA_TOOLKIT_ROOT_DIR}/include")
  endif()

  set(EIGEN_ADD_TEST_FILENAME_EXTENSION  "cu")

  ei_add_test(gpu_example)
  ei_add_test(gpu_basic)

  unset(EIGEN_ADD_TEST_FILENAME_EXTENSION)

endif()


# HIP unit tests
option(EIGEN_TEST_HIP "Add HIP support." OFF)
if (EIGEN_TEST_HIP)

  set(ROCM_PATH "/opt/rocm" CACHE STRING "Path to the ROCm installation.")

  if (EXISTS ${ROCM_PATH}/hip)
    set(HIP_PATH ${ROCM_PATH}/hip)
    list(APPEND CMAKE_MODULE_PATH ${HIP_PATH}/cmake)
  elseif (EXISTS ${ROCM_PATH}/lib/cmake/hip)
    set(HIP_PATH ${ROCM_PATH})
    list(APPEND CMAKE_MODULE_PATH ${HIP_PATH}/lib/cmake/hip)
  else ()
    message(FATAL_ERROR "EIGEN_TEST_HIP is ON, but could not find the ROCm installation under ${ROCM_PATH}")
  endif()

  find_package(HIP REQUIRED)
  if (HIP_FOUND AND HIP_VERSION VERSION_LESS "5.6")
    message(FATAL_ERROR "Eigen requires ROCm/HIP >= 5.6, found ${HIP_VERSION}")
  endif()
  if (HIP_FOUND)
    execute_process(COMMAND ${HIP_PATH}/bin/hipconfig --platform OUTPUT_VARIABLE HIP_PLATFORM
                    OUTPUT_STRIP_TRAILING_WHITESPACE)

    if (HIP_PLATFORM STREQUAL "amd")

      include_directories(${HIP_PATH}/include)

      set(EIGEN_ADD_TEST_FILENAME_EXTENSION  "cu")
      ei_add_test(gpu_basic)
      ei_add_test(gpu_example)
      unset(EIGEN_ADD_TEST_FILENAME_EXTENSION)

    elseif (HIP_PLATFORM STREQUAL "nvcc" OR HIP_PLATFORM STREQUAL "nvidia")
      message(FATAL_ERROR "HIP_PLATFORM = nvcc is not supported within Eigen")
    else ()
      message(FATAL_ERROR "Unknown HIP_PLATFORM = ${HIP_PLATFORM}")
    endif()
  endif()
endif()

if(EIGEN_TEST_SYCL)
  set(EIGEN_SYCL ON)
  include(SyclConfigureTesting)

  ei_add_test(sycl_basic)
  set(EIGEN_SYCL OFF)
endif()

cmake_dependent_option(EIGEN_TEST_BUILD_DOCUMENTATION "Test building the doxygen documentation" OFF "EIGEN_BUILD_DOC" OFF)
if(EIGEN_TEST_BUILD_DOCUMENTATION)
  add_dependencies(buildtests doc)
endif()

# ULP accuracy measurement tool (see test/ulp_accuracy/README.md)
find_package(MPFR)
find_package(GMP)
find_package(Threads REQUIRED)
add_executable(ulp_accuracy ulp_accuracy/ulp_accuracy.cpp)
# Threads::Threads rather than a literal -pthread and pthread: it carries the
# right compile and link flags per toolchain and expands to nothing on MSVC,
# where there is no pthread.lib to open.
target_link_libraries(ulp_accuracy Eigen3::Eigen Threads::Threads)
if(EIGEN_TEST_CUSTOM_CXX_FLAGS)
  target_compile_options(ulp_accuracy PRIVATE ${EIGEN_TEST_CUSTOM_CXX_FLAGS})
endif()
if(MPFR_FOUND AND GMP_FOUND)
  target_include_directories(ulp_accuracy PRIVATE ${MPFR_INCLUDES} ${GMP_INCLUDES})
  target_link_libraries(ulp_accuracy ${MPFR_LIBRARIES} ${GMP_LIBRARIES})
  target_compile_definitions(ulp_accuracy PRIVATE EIGEN_HAS_MPFR)
endif()
