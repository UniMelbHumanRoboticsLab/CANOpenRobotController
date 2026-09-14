// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2006-2008 Benoit Jacob <jacob.benoit.1@gmail.com>
// Copyright (C) 2008 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include <cstdlib>
#include <cerrno>
#include <ctime>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <limits>
#include <string>
#include <sstream>
#include <vector>
#include <typeinfo>
#include <type_traits>
#include <functional>
#ifdef EIGEN_USE_SYCL
#include <CL/sycl.hpp>
#endif

// The following includes of STL headers have to be done _before_ the
// definition of macros min() and max().  The reason is that many STL
// implementations will not work properly as the min and max symbols collide
// with the STL functions std::min() and std::max().  The STL headers may check
// for the macro definition of min/max and issue a warning or undefine the
// macros.
//
// Still, Windows defines min() and max() in windef.h as part of the regular
// Windows system interfaces and many other Windows APIs depend on these
// macros being available.  To prevent the macro expansion of min/max and to
// make Eigen compatible with the Windows environment all function calls of
// std::min() and std::max() have to be written with parenthesis around the
// function name.
//
// All STL headers used by Eigen should be included here.  Because main.h is
// included before any Eigen header and because the STL headers are guarded
// against multiple inclusions, no STL header will see our own min/max macro
// definitions.
#include <limits>
#include <algorithm>
// Disable ICC's std::complex operator specializations so we can use our own.
#define _OVERRIDE_COMPLEX_SPECIALIZATION_ 1
#include <complex>
#include <deque>
#include <queue>
#include <cassert>
#include <list>
#include <random>
#include <chrono>
// libstdc++'s <unordered_map> uses std::numeric_limits<size_t>::max() internally
// (in <bits/hashtable_policy.h>); pre-include here so the macro definitions of
// min/max below don't break it on toolchains where it isn't pulled in by
// another pre-included header (e.g. cuda-11.5 / gcc-10).
#include <unordered_map>
#if __cplusplus > 201703L
// libstdc++ 9's <memory> indirectly uses max() via <bit>.
// libstdc++ 10's <memory> indirectly uses max() via ranges headers.
#include <memory>
// libstdc++ 11's <thread> indirectly uses max() via semaphore headers.
#include <thread>
#endif

// Configure GPU.
#if defined(EIGEN_USE_HIP)
#if defined(__HIPCC__) && !defined(EIGEN_NO_HIP)
#define EIGEN_HIPCC __HIPCC__
#include <hip/hip_runtime.h>
#include <hip/hip_runtime_api.h>
#endif
#elif defined(__CUDACC__) && !defined(EIGEN_NO_CUDA)
#define EIGEN_CUDACC __CUDACC__
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <cuda_fp16.h>
#endif

#if defined(EIGEN_CUDACC) || defined(EIGEN_HIPCC)
#define EIGEN_TEST_NO_LONGDOUBLE
#define EIGEN_DEFAULT_DENSE_INDEX_TYPE int
#endif

// To test that all calls from Eigen code to std::min() and std::max() are
// protected by parenthesis against macro expansion, the min()/max() macros
// are defined here and any not-parenthesized min/max call will cause a
// compiler error.
#if !defined(__HIPCC__) && !defined(EIGEN_USE_SYCL) && !defined(EIGEN_POCKETFFT_DEFAULT)
//
// HIP header files include the following files
//  <thread>
//  <regex>
//  <unordered_map>
// which seem to contain not-parenthesized calls to "max"/"min", triggering the following check and causing the compile
// to fail
//
// Including those header files before the following macro definition for "min" / "max", only partially resolves the
// issue This is because other HIP header files also define "isnan" / "isinf" / "isfinite" functions, which are needed
// in other headers.
//
// So instead choosing to simply disable this check for HIP
//
#define min(A, B) please_protect_your_min_with_parentheses
#define max(A, B) please_protect_your_max_with_parentheses
#define isnan(X) please_protect_your_isnan_with_parentheses
#define isinf(X) please_protect_your_isinf_with_parentheses
#define isfinite(X) please_protect_your_isfinite_with_parentheses
#endif

// test possible conflicts
struct real {};
struct imag {};

#ifdef M_PI
#undef M_PI
#endif
#define M_PI please_use_EIGEN_PI_instead_of_M_PI

#define FORBIDDEN_IDENTIFIER \
  (this_identifier_is_forbidden_to_avoid_clashes) this_identifier_is_forbidden_to_avoid_clashes
// B0 is defined in POSIX header termios.h
#define B0 FORBIDDEN_IDENTIFIER
#define I FORBIDDEN_IDENTIFIER

// _res is defined by resolv.h
#define _res FORBIDDEN_IDENTIFIER

// Shrink the cache model used by computeProductBlockingSizes so that test-sized
// products (see EIGEN_TEST_MAX_SIZE) exercise multi-pass GEMM blocking; with real
// cache sizes nothing in the suite would ever block. Any new architecture-specific
// blocking heuristic must honor this macro, or the blocking sentinels in
// product_extra.cpp (compute_block_size) will fail on that target.
// Exception: tests linking the eigen_blas/eigen_lapack shim libraries (the
// *_support sparse-solver tests) must opt out via EIGEN_NO_DEBUG_SMALL_PRODUCT_BLOCKS.
// Those libraries are compiled without this macro, and mixing the two configurations
// in one binary is an ODR violation on the inline blocking/kernel code.
#ifndef EIGEN_NO_DEBUG_SMALL_PRODUCT_BLOCKS
#define EIGEN_DEBUG_SMALL_PRODUCT_BLOCKS
#endif

// shuts down ICC's remark #593: variable "XXX" was set but never used
#define TEST_SET_BUT_UNUSED_VARIABLE(X) EIGEN_UNUSED_VARIABLE(X)

#ifdef TEST_ENABLE_TEMPORARY_TRACKING

static long int nb_temporaries;
static long int nb_temporaries_on_assert = -1;

#ifdef TEST_IGNORE_STACK_ALLOCATED_TEMPORARY
inline void on_temporary_creation(long int size, int SizeAtCompileTime) {
  // ignore stack-allocated temporaries
  if (SizeAtCompileTime != -1) return;
#else
inline void on_temporary_creation(long int size, int) {
#endif
  // here's a great place to set a breakpoint when debugging failures in this test!
  if (size != 0) nb_temporaries++;
  if (nb_temporaries_on_assert > 0) assert(nb_temporaries < nb_temporaries_on_assert);
}

#define EIGEN_DENSE_STORAGE_CTOR_PLUGIN \
  { on_temporary_creation(size, Size); }

#define VERIFY_EVALUATION_COUNT(XPR, N)                            \
  {                                                                \
    nb_temporaries = 0;                                            \
    XPR;                                                           \
    if (nb_temporaries != (N)) {                                   \
      std::cerr << "nb_temporaries == " << nb_temporaries << "\n"; \
    }                                                              \
    VERIFY((#XPR) && nb_temporaries == (N));                       \
  }

#endif

#include "split_test_helper.h"

#ifdef NDEBUG
#undef NDEBUG
#endif

// On windows CE, NDEBUG is automatically defined <assert.h> if NDEBUG is not defined.
#ifndef DEBUG
#define DEBUG
#endif

#define DEFAULT_REPEAT 10

namespace Eigen {
static std::vector<std::string> g_test_stack;
// level == 0 <=> return 1 if test fail
// level >= 1 <=> warning message to std::cerr if test fail
static int g_test_level = 0;
static int g_repeat = 1;
static unsigned int g_seed = 0;
static bool g_has_set_repeat = false, g_has_set_seed = false;

class EigenTest {
 public:
  EigenTest() : m_func(0) {}
  EigenTest(const char* a_name, void (*func)(void)) : m_name(a_name), m_func(func) {
    get_registered_tests().push_back(this);
  }
  const std::string& name() const { return m_name; }
  void operator()() const { m_func(); }

  static const std::vector<EigenTest*>& all() { return get_registered_tests(); }

 protected:
  static std::vector<EigenTest*>& get_registered_tests() {
    static std::vector<EigenTest*>* ms_registered_tests = new std::vector<EigenTest*>();
    return *ms_registered_tests;
  }
  std::string m_name;
  void (*m_func)(void);
};

// Declare and register a test, e.g.:
//    EIGEN_DECLARE_TEST(mytest) { ... }
// will create a function:
//    void test_mytest() { ... }
// that will be automatically called.
#define EIGEN_DECLARE_TEST(X)                                                              \
  void EIGEN_CAT(test_, X)();                                                              \
  static EigenTest EIGEN_CAT(test_handler_, X)(EIGEN_MAKESTRING(X), &EIGEN_CAT(test_, X)); \
  void EIGEN_CAT(test_, X)()
}  // namespace Eigen

#define EIGEN_DEFAULT_IO_FORMAT IOFormat(4, 0, "  ", "\n", "", "", "", "")

#if (defined(_CPPUNWIND) || defined(__EXCEPTIONS)) && !defined(__CUDA_ARCH__) && !defined(__HIP_DEVICE_COMPILE__) && \
    !defined(__SYCL_DEVICE_ONLY__)
#define EIGEN_EXCEPTIONS
#endif

#ifndef EIGEN_NO_ASSERTION_CHECKING

namespace Eigen {
static const bool should_raise_an_assert = false;

// Used to avoid to raise two exceptions at a time in which
// case the exception is not properly caught.
// This may happen when a second exceptions is triggered in a destructor.
static bool no_more_assert = false;
static bool report_on_cerr_on_assert_failure = true;

struct eigen_assert_exception {
  eigen_assert_exception(void) {}
  ~eigen_assert_exception() { Eigen::no_more_assert = false; }
};

struct eigen_static_assert_exception {
  eigen_static_assert_exception(void) {}
  ~eigen_static_assert_exception() { Eigen::no_more_assert = false; }
};
}  // namespace Eigen
// If EIGEN_DEBUG_ASSERTS is defined and if no assertion is triggered while
// one should have been, then the list of executed assertions is printed out.
//
// EIGEN_DEBUG_ASSERTS is not enabled by default as it
// significantly increases the compilation time
// and might even introduce side effects that would hide
// some memory errors.
#ifdef EIGEN_DEBUG_ASSERTS

namespace Eigen {
namespace internal {
static bool push_assert = false;
}
static std::vector<std::string> eigen_assert_list;
}  // namespace Eigen
#define eigen_assert(a)                                                                                             \
  if ((!(a)) && (!no_more_assert)) {                                                                                \
    if (report_on_cerr_on_assert_failure) std::cerr << #a << " " __FILE__ << "(" << __LINE__ << ")\n";              \
    Eigen::no_more_assert = true;                                                                                   \
    EIGEN_THROW_X(Eigen::eigen_assert_exception());                                                                 \
  } else if (Eigen::internal::push_assert) {                                                                        \
    eigen_assert_list.push_back(std::string(EIGEN_MAKESTRING(__FILE__) " (" EIGEN_MAKESTRING(__LINE__) ") : " #a)); \
  }

#ifdef EIGEN_EXCEPTIONS
#define VERIFY_RAISES_ASSERT(a)                                                                                  \
  {                                                                                                              \
    Eigen::no_more_assert = false;                                                                               \
    Eigen::eigen_assert_list.clear();                                                                            \
    Eigen::internal::push_assert = true;                                                                         \
    Eigen::report_on_cerr_on_assert_failure = false;                                                             \
    try {                                                                                                        \
      a;                                                                                                         \
      std::cerr << "One of the following asserts should have been triggered:\n";                                 \
      for (uint ai = 0; ai < eigen_assert_list.size(); ++ai) std::cerr << "  " << eigen_assert_list[ai] << "\n"; \
      VERIFY(Eigen::should_raise_an_assert&& #a);                                                                \
    } catch (Eigen::eigen_assert_exception) {                                                                    \
      Eigen::internal::push_assert = false;                                                                      \
      VERIFY(true);                                                                                              \
    }                                                                                                            \
    Eigen::report_on_cerr_on_assert_failure = true;                                                              \
    Eigen::internal::push_assert = false;                                                                        \
  }
#endif  // EIGEN_EXCEPTIONS

#elif !defined(__CUDACC__) && !defined(__HIPCC__) && !defined(__SYCL_DEVICE_ONLY__)  // EIGEN_DEBUG_ASSERTS
#define eigen_assert(a)                               \
  if ((!(a)) && (!no_more_assert)) {                  \
    Eigen::no_more_assert = true;                     \
    if (report_on_cerr_on_assert_failure) {           \
      eigen_plain_assert(a);                          \
    } else {                                          \
      EIGEN_THROW_X(Eigen::eigen_assert_exception()); \
    }                                                 \
  }

#ifdef EIGEN_EXCEPTIONS
#define VERIFY_RAISES_ASSERT(a)                      \
  {                                                  \
    Eigen::no_more_assert = false;                   \
    Eigen::report_on_cerr_on_assert_failure = false; \
    try {                                            \
      a;                                             \
      VERIFY(Eigen::should_raise_an_assert&& #a);    \
    } catch (Eigen::eigen_assert_exception&) {       \
      VERIFY(true);                                  \
    }                                                \
    Eigen::report_on_cerr_on_assert_failure = true;  \
  }
#endif  // EIGEN_EXCEPTIONS
#endif  // EIGEN_DEBUG_ASSERTS

#ifndef VERIFY_RAISES_ASSERT
#define VERIFY_RAISES_ASSERT(a) std::cout << "Can't VERIFY_RAISES_ASSERT( " #a " ) with exceptions disabled\n";
#endif

#else  // EIGEN_NO_ASSERTION_CHECKING

#define VERIFY_RAISES_ASSERT(a) \
  {}

#endif  // EIGEN_NO_ASSERTION_CHECKING

#if !defined(EIGEN_TESTING_CONSTEXPR) && !defined(EIGEN_TESTING_PLAINOBJECT_CTOR)
#define EIGEN_INTERNAL_DEBUGGING
#endif
#include <Eigen/Core>

inline void verify_impl(bool condition, const char* testname, const char* file, int line,
                        const char* condition_as_string) {
  if (!condition) {
    if (Eigen::g_test_level > 0) std::cerr << "WARNING: ";
    std::cerr << "Test " << testname << " failed in " << file << " (" << line << ")" << std::endl
              << "    " << condition_as_string << std::endl;
    std::cerr << "Stack:\n";
    const int test_stack_size = static_cast<int>(Eigen::g_test_stack.size());
    for (int i = test_stack_size - 1; i >= 0; --i) std::cerr << "  - " << Eigen::g_test_stack[i] << "\n";
    std::cerr << "\n";
    if (Eigen::g_test_level == 0) exit(1);
  }
}

#define VERIFY(a) ::verify_impl(a, g_test_stack.back().c_str(), __FILE__, __LINE__, EIGEN_MAKESTRING(a))

#define VERIFY_GE(a, b) ::verify_impl(a >= b, g_test_stack.back().c_str(), __FILE__, __LINE__, EIGEN_MAKESTRING(a >= b))
#define VERIFY_LE(a, b) ::verify_impl(a <= b, g_test_stack.back().c_str(), __FILE__, __LINE__, EIGEN_MAKESTRING(a <= b))

#define STATIC_CHECK(COND) EIGEN_STATIC_ASSERT((COND), EIGEN_INTERNAL_ERROR_PLEASE_FILE_A_BUG_REPORT)

#define CALL_SUBTEST(FUNC)                          \
  do {                                              \
    g_test_stack.push_back(EIGEN_MAKESTRING(FUNC)); \
    FUNC;                                           \
    g_test_stack.pop_back();                        \
  } while (0)

#include "numerical_test_helpers.h"

#include "random_matrix_helper.h"

#include "type_test_helpers.h"

using namespace Eigen;

/**
 * Set number of repetitions for unit test from input string.
 *
 * @param str input string
 */
inline void set_repeat_from_string(const char* str) {
  errno = 0;
  g_repeat = int(strtoul(str, 0, 10));
  if (errno || g_repeat <= 0) {
    std::cout << "Invalid repeat value " << str << std::endl;
    exit(EXIT_FAILURE);
  }
  g_has_set_repeat = true;
}

/**
 * Set seed for randomized unit tests from input string.
 *
 * @param str input string
 */
inline void set_seed_from_string(const char* str) {
  errno = 0;
  g_seed = int(strtoul(str, 0, 10));
  if (errno || g_seed == 0) {
    std::cout << "Invalid seed value " << str << std::endl;
    exit(EXIT_FAILURE);
  }
  g_has_set_seed = true;
}

inline void set_seed_from_time() {
  using namespace std::chrono;
  long long ns = duration_cast<nanoseconds>(high_resolution_clock::now().time_since_epoch()).count();
  g_seed = static_cast<decltype(g_seed)>(ns);
}

// Reports the test as skipped and exits with 77, the code CTest maps to "skipped"
// (SKIP_RETURN_CODE in cmake/EigenTesting.cmake).
[[noreturn]] inline void skip_test(const char* reason) {
  std::cout << "SKIP: " << reason << std::endl;
  std::exit(77);
}

#if defined(EIGEN_USE_GPU)
inline int maybe_skip_gpu_tests() {
#if defined(EIGEN_USE_HIP)
  int device_count = 0;
  hipError_t status = hipGetDeviceCount(&device_count);
  if (status != hipSuccess) {
    std::cout << "SKIP: HIP GPU tests require a visible ROCm device. hipGetDeviceCount failed with: "
              << hipGetErrorString(status) << std::endl;
    return 77;
  }
  if (device_count <= 0) {
    std::cout << "SKIP: HIP GPU tests require a visible ROCm device." << std::endl;
    return 77;
  }
#elif defined(EIGEN_CUDACC)
  int device_count = 0;
  cudaError_t status = cudaGetDeviceCount(&device_count);
  if (status != cudaSuccess) {
    std::cout << "SKIP: CUDA GPU tests require a visible CUDA device. cudaGetDeviceCount failed with: "
              << cudaGetErrorString(status) << std::endl;
    return 77;
  }
  if (device_count <= 0) {
    std::cout << "SKIP: CUDA GPU tests require a visible CUDA device." << std::endl;
    return 77;
  }
#endif
  return 0;
}
#endif

// GCC's AddressSanitizer instrumentation relocates a function's locals into the fake stack it uses
// for stack-use-after-return detection, and it lays that frame out on a 32-byte grid. A frame GCC
// aligned to more than that then faults on its own aligned vector accesses, since vmovdqa64 raises
// #GP off an address that is not 64-byte aligned. An AVX-512 build under ASan therefore dies with
// a SEGV unrelated to the code under test, in whichever frame the layout happens to hit. Opt those
// builds out of the fake stack; clang aligns its fake frames correctly and keeps the detector.
// ASAN_OPTIONS from the environment still overrides this.
//
// The condition asks what alignment compiled stack objects require, which is
// EIGEN_MAX_STATIC_ALIGN_BYTES rather than the dynamic-allocation bound EIGEN_MAX_ALIGN_BYTES --
// the latter is only an upper bound for it, so it is 64 even in a default-ISA build that aligns no
// local past 16. EIGEN_VECTORIZE_AVX512 is checked as well because the AVX-512 backend declares
// EIGEN_ALIGN64 locals and spills 64-byte vectors unconditionally, whatever the user caps the
// Eigen alignment macros at.
#if defined(__SANITIZE_ADDRESS__) && EIGEN_COMP_GNUC_STRICT && \
    (EIGEN_MAX_STATIC_ALIGN_BYTES > 32 || defined(EIGEN_VECTORIZE_AVX512))
extern "C" const char* __asan_default_options() { return "detect_stack_use_after_return=0"; }
#endif

int main(int argc, char* argv[]) {
  g_has_set_repeat = false;
  g_has_set_seed = false;
  bool need_help = false;

  for (int i = 1; i < argc; i++) {
    if (argv[i][0] == 'r') {
      if (g_has_set_repeat) {
        std::cout << "Argument " << argv[i] << " conflicting with a former argument" << std::endl;
        return 1;
      }
      set_repeat_from_string(argv[i] + 1);
    } else if (argv[i][0] == 's') {
      if (g_has_set_seed) {
        std::cout << "Argument " << argv[i] << " conflicting with a former argument" << std::endl;
        return 1;
      }
      set_seed_from_string(argv[i] + 1);
    } else {
      need_help = true;
    }
  }

  if (need_help) {
    std::cout << "This test application takes the following optional arguments:" << std::endl;
    std::cout << "  rN     Repeat each test N times (default: " << DEFAULT_REPEAT << ")" << std::endl;
    std::cout << "  sN     Use N as seed for random numbers (default: based on current time)" << std::endl;
    std::cout << std::endl;
    std::cout << "If defined, the environment variables EIGEN_REPEAT and EIGEN_SEED" << std::endl;
    std::cout << "will be used as default values for these parameters." << std::endl;
    return 1;
  }

  char* env_EIGEN_REPEAT = getenv("EIGEN_REPEAT");
  if (!g_has_set_repeat && env_EIGEN_REPEAT) set_repeat_from_string(env_EIGEN_REPEAT);
  char* env_EIGEN_SEED = getenv("EIGEN_SEED");
  if (!g_has_set_seed && env_EIGEN_SEED) set_seed_from_string(env_EIGEN_SEED);

  if (!g_has_set_seed) set_seed_from_time();
  if (!g_has_set_repeat) g_repeat = DEFAULT_REPEAT;

  std::cout << "Initializing random number generator with seed " << g_seed << std::endl;
  std::stringstream ss;
  ss << "Seed: " << g_seed;
  g_test_stack.push_back(ss.str());
  srand(g_seed);
  std::cout << "Repeating each test " << g_repeat << " times" << std::endl;

#if defined(EIGEN_USE_GPU)
  {
    const int skip_code = maybe_skip_gpu_tests();
    if (skip_code != 0) return skip_code;
  }
#endif

  VERIFY(EigenTest::all().size() > 0);

  for (std::size_t i = 0; i < EigenTest::all().size(); ++i) {
    const EigenTest& current_test = *EigenTest::all()[i];
    Eigen::g_test_stack.push_back(current_test.name());
    current_test();
    Eigen::g_test_stack.pop_back();
  }

  return 0;
}

// These warning are disabled here such that they are still ON when parsing Eigen's header files.
#if defined __INTEL_COMPILER
// remark #383: value copied to temporary, reference to temporary used
//  -> this warning is raised even for legal usage as: g_test_stack.push_back("foo"); where g_test_stack is a
//  std::vector<std::string>
// remark #1418: external function definition with no prior declaration
//  -> this warning is raised for all our test functions. Declaring them static would fix the issue.
// warning #279: controlling expression is constant
// remark #1572: floating-point equality and inequality comparisons are unreliable
#pragma warning disable 279 383 1418 1572
#endif

#ifdef _MSC_VER
// 4503 - decorated name length exceeded, name was truncated
#pragma warning(disable : 4503)
#endif

#include "gpu_test_helper.h"

#ifndef EIGEN_TEST_MAX_SIZE
#define EIGEN_TEST_MAX_SIZE 320
#endif
