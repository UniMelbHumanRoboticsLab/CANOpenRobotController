#!/bin/bash
#
# Run clang-tidy on files changed in the current MR.
#
# Usage: run-clang-tidy.sh <base_sha> <build_dir>
#
# <base_sha>   The merge-base commit to diff against.
# <build_dir>  Path to a CMake build directory containing compile_commands.json.
#
# For header files under Eigen/src/<Module>/, the script generates a driver
# that includes the parent module header first, so InternalHeaderCheck.h does
# not #error out, and then the changed header itself. The explicit second
# include covers new headers that are not exported by the umbrella yet. The
# umbrella name is read from the header's own `#error "Please include <X>"`
# directive (or a sibling InternalHeaderCheck.h), with a fallback to the
# heuristic <root>/<Module> for deeply-nested files (e.g. arch-specific
# backends) that don't carry their own directive.
#
# ISA backends under arch/<ISA>/ are the exception: they only compile with the
# -march/-mcpu flags that select them, which this job does not pass, so forcing
# them into the driver turns every such change into a wall of clang-diagnostic
# errors from arm_sve.h / immintrin.h rather than a lint result. Those get the
# umbrella include alone, which lints them when the host target selects the
# backend and reports nothing when it does not.
# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

set -euo pipefail

BASE_SHA="${1:?Usage: run-clang-tidy.sh <base_sha> <build_dir>}"
BUILD_DIR="${2:?Usage: run-clang-tidy.sh <base_sha> <build_dir>}"

if [ ! -f "${BUILD_DIR}/compile_commands.json" ]; then
  echo "ERROR: ${BUILD_DIR}/compile_commands.json not found."
  echo "Run cmake with -DCMAKE_EXPORT_COMPILE_COMMANDS=ON first."
  exit 1
fi

REPO_ROOT="$(git rev-parse --show-toplevel)"

# Compiler arguments for the sources clang-tidy parses outside the
# compilation database: the generated header drivers and the failtest sources.
# Where a CUDA toolkit is installed the GPU module's headers parse in full;
# the CI image has none, and third_party_include_missing_from() then marks
# those files as partially checked.
DRIVER_COMPILE_ARGS=(-std=c++14 -I"${REPO_ROOT}")
for cuda_root in "${CUDAToolkit_ROOT:-}" "${CUDA_HOME:-}" "${CUDA_PATH:-}" /usr/local/cuda; do
  if [ -n "${cuda_root}" ] && [ -f "${cuda_root}/include/cuda_runtime.h" ]; then
    DRIVER_COMPILE_ARGS+=(-isystem "${cuda_root}/include")
    break
  fi
done

# Get changed files (Added, Modified, Renamed) without losing whitespace in
# repository paths.
mapfile -d '' -t CHANGED_FILES < <(git diff --name-only -z --diff-filter=AMR "${BASE_SHA}" HEAD)

if [ "${#CHANGED_FILES[@]}" -eq 0 ]; then
  echo "No changed files to check."
  exit 0
fi

TIDY_TMPDIR=$(mktemp -d)
trap 'rm -rf "${TIDY_TMPDIR}"' EXIT

ERRORS=0
# Generated drivers live outside the checkout, where clang-tidy cannot discover
# Eigen's configuration.  The job is advisory (`allow_failure`), so promoting
# warnings makes GitLab mark findings without blocking the pipeline.
TIDY_CONFIG_ARG="--config-file=${REPO_ROOT}/.clang-tidy"
TIDY_ARGS=(
  "${TIDY_CONFIG_ARG}"
  "--warnings-as-errors=*"
)

# Determine which umbrella header to include when linting a given source-tree
# header. The source of truth is the `#error "Please include <X>"` directive
# carried either by the header itself (e.g. Eigen/src/StlSupport/StdDeque.h
# -> Eigen/StdDeque) or by its sibling InternalHeaderCheck.h (the common
# case for Eigen/src/<Module>/*.h).
module_include_for_header() {
  local header="$1"
  local module
  local hint
  local candidate

  # Restrict to header files inside the src trees.
  if [[ "${header}" =~ ^Eigen/src/([^/]+)/ ]]; then
    module="${BASH_REMATCH[1]}"
  elif [[ "${header}" =~ ^contrib/Eigen/src/([^/]+)/ ]]; then
    module="${BASH_REMATCH[1]}"
  else
    return 1
  fi

  # Parse `#error "Please include <X>"` from the header or its sibling
  # InternalHeaderCheck.h.
  for candidate in "${REPO_ROOT}/${header}" \
                   "${REPO_ROOT}/$(dirname "${header}")/InternalHeaderCheck.h"; do
    if [ -f "${candidate}" ]; then
      hint=$(grep "Please include" "${candidate}" 2>/dev/null \
             | sed -nE 's/.*"Please include ([^ "]+).*/\1/p' \
             | head -n1)
      if [ -n "${hint}" ] && [ -f "${REPO_ROOT}/${hint}" ]; then
        echo "${hint}"
        return 0
      fi
    fi
  done

  # Fallback: route through <root>/<Module> if it exists. This catches files
  # nested deeper than the module's top-level src/ (e.g. arch-specific
  # backends under Eigen/src/Core/arch/<ISA>/) that don't carry their own
  # `#error` directive.
  if [[ "${header}" =~ ^contrib/ ]]; then
    hint="contrib/Eigen/${module}"
  else
    hint="Eigen/${module}"
  fi
  if [ -f "${REPO_ROOT}/${hint}" ]; then
    echo "${hint}"
    return 0
  fi

  # No parseable directive and no matching umbrella file — likely a
  # utility/details file shared across umbrellas (e.g. StlSupport/details.h).
  # Skip silently.
  return 1
}

# A split test contributes one compilation-database entry per EIGEN_TEST_PART,
# and clang-tidy parses the file once for every entry that names it: 17 for
# test/eigensolver_selfadjoint.cpp, 41 for test/array_cwise.cpp, at roughly a
# minute and 3 GB each. That exhausts the job timeout on a single file and
# leaves every later file unchecked, silently. Narrow the database to the
# entries the changed lines need, which scripts/tidy_compile_db.py selects:
# one per distinct compiler configuration, and within a configuration split
# into parts, the parts that compile the added lines.
#
# Writes the reduced database to <outdir>/compile_commands.json, reports on
# stdout what it left out, and succeeds only when the file is present in the
# full database. CMake normally records absolute source paths, so establishing
# membership takes path resolution rather than a textual search.
reduced_database() {
  python3 "${REPO_ROOT}/scripts/tidy_compile_db.py" \
          "${BUILD_DIR}/compile_commands.json" "${REPO_ROOT}/$1" "$2" "$3"
}

# Some modules reach a third-party header the image does not install:
# <cholmod.h> from CholmodSupport, <cuda_runtime.h> from every header in the
# GPU module that includes GpuSupport.h. Clang reports the missing include,
# drops it along with every declaration behind it, and suppresses the
# diagnostics that would otherwise follow, so clang-tidy goes on to analyze a
# truncated AST: sound over the parts of the header that do not touch the
# absent API, unreliable over the parts that do.
#
# Name the header the driver could not resolve, so the caller can label that
# file's findings accordingly. Probing the driver rather than consulting a
# list of modules also checks the module in full wherever the dependency is
# installed, and leaves the headers that never reach it -- five of the GPU
# module's twenty-nine -- checked as they already are.
#
# Probe with clang++ rather than ${CXX}: the answer has to be the one
# clang-tidy's own parse produces, and g++ neither searches the same
# directories nor words the diagnostic the same way.
third_party_include_missing_from() {
  local output diag location spelling directive
  output=$(clang++ "${DRIVER_COMPILE_ARGS[@]}" -E -P -o /dev/null "$1" 2>&1) || true
  diag=$(printf '%s\n' "${output}" \
         | grep -m1 -E "fatal error: '[^']+' file not found") || return 0

  # <path>:<line>:<column>: fatal error: '<spelling>' file not found
  spelling=${diag##*fatal error: \'}
  spelling=${spelling%%\'*}
  location=${diag%%: fatal error:*}
  location=${location%:*}

  # An include the repository is expected to satisfy is a defect in the
  # change, not a missing dependency; leave it to clang-tidy to report as the
  # error it is. Eigen spells its own headers relative to the file that
  # includes them, so the form of the directive is what separates the two:
  # <cuda_runtime.h> and <cholmod.h> come from outside the tree, while
  # "./InternalHeaderCheck.h" and "GenericPacketMathPow.h" do not, and no
  # quoted third-party include in the tree is reachable without an
  # EIGEN_USE_* macro this job does not define.
  case "${spelling}" in
    Eigen/*|contrib/*|unsupported/*|./*|../*) return 0 ;;
  esac
  directive=$(sed -n "${location##*:}p" "${location%:*}" 2>/dev/null)
  if [[ "${directive}" =~ ^[[:space:]]*#[[:space:]]*include[[:space:]]*\" ]]; then
    return 0
  fi

  printf '%s\n' "${spelling}"
}

# Restrict diagnostics to the lines this merge request adds. Without it the
# style checks in .clang-tidy (modernize-use-nullptr, modernize-use-using)
# would report every pre-existing occurrence in a touched file — Eigen/src
# holds ~4100 typedefs — rather than the ones under review. An empty filter
# means "no filtering" to clang-tidy, so a file with no added lines is skipped
# rather than linted whole.
line_filter_for() {
  python3 "${REPO_ROOT}/scripts/style_common.py" --line-filter "${BASE_SHA}" "$1"
}

echo "Checking changed files with clang-tidy..."
echo "Base SHA: ${BASE_SHA}"
echo ""

for file in "${CHANGED_FILES[@]}"; do
  LINE_FILTER=$(line_filter_for "${file}")
  if [ -z "${LINE_FILTER}" ] || [ "${LINE_FILTER}" = "[]" ]; then
    # Renamed or mode-only change: nothing added to report on.
    continue
  fi

  # Only check C++ source and header files.
  case "${file}" in
    failtest/*.cpp)
      # The compilation database carries both the successful and intentionally
      # broken variants.  Parse the ordinary variant directly so the _ko
      # command cannot turn every changed failtest into a clang diagnostic.
      echo "=== ${file} ==="
      if ! clang-tidy \
            "${TIDY_ARGS[@]}" \
            --line-filter="${LINE_FILTER}" \
            "${file}" \
            -- "${DRIVER_COMPILE_ARGS[@]}" 2>&1; then
        ERRORS=$((ERRORS + 1))
      fi
      ;;
    *.cpp|*.cc|*.cxx)
      # Source file: run clang-tidy directly if it's in the compilation database.
      FILE_DB="${TIDY_TMPDIR}/db_${file//\//_}"
      if SELECTION=$(reduced_database "${file}" "${FILE_DB}" "${LINE_FILTER}"); then
        echo "=== ${file} ===${SELECTION:+ ${SELECTION}}"
        if ! clang-tidy \
              -p "${FILE_DB}" \
              "${TIDY_ARGS[@]}" \
              --line-filter="${LINE_FILTER}" \
              "${file}" 2>&1; then
          ERRORS=$((ERRORS + 1))
        fi
      else
        STATUS=$?
        if [ "${STATUS}" -gt 1 ]; then
          exit "${STATUS}"
        fi
      fi
      ;;
    *.h|*.hpp)
      # Header file: include the right module first, then force the changed
      # header into the translation unit even if the umbrella omits it.
      MODULE_INCLUDE=$(module_include_for_header "${file}" || true)
      if [ -z "${MODULE_INCLUDE}" ]; then
        # Not a recognized module header.
        continue
      fi

      # arch/<ISA>/ backends need their own -march to parse at all; arch/Default
      # is generic and stays force-included.
      NOTE=""
      FORCE_INCLUDE="#include <${file}>"
      if [[ "${file}" =~ /arch/([^/]+)/ ]] && [ "${BASH_REMATCH[1]}" != "Default" ]; then
        NOTE=" [${BASH_REMATCH[1]} backend: linted only if the host target selects it]"
        FORCE_INCLUDE=""
      elif [[ "${file}" =~ /Tensor/[^/]*Sycl[^/]*\.h$ ]]; then
        NOTE=" [SYCL backend: linted only when EIGEN_USE_SYCL is enabled]"
        FORCE_INCLUDE=""
      fi

      DRIVER="${TIDY_TMPDIR}/tidy_driver_${file//\//_}.cpp"
      cat > "${DRIVER}" <<EOF
#include <${MODULE_INCLUDE}>
${FORCE_INCLUDE}
EOF
      HEADER_FILTER="$(echo "${file}" | sed 's/[.[\*^$()+?{|]/\\&/g')"

      # Without the dependency clang drops the include and everything it
      # declares, then suppresses the diagnostics that would follow. What
      # remains of the header is still worth checking -- most of it does not
      # touch the third-party API -- but a check that reaches the truncated
      # part can be wrong either way: the switch over cublasOperation_t in
      # CuBlasSupport.h reads as three identical branches once CUBLAS_OP_N,
      # _T and _C are gone. Report those findings without promoting them to
      # errors, so they inform a reader without failing a job that has no
      # way to confirm them.
      MISSING_INCLUDE=$(third_party_include_missing_from "${DRIVER}")
      if [ -n "${MISSING_INCLUDE}" ]; then
        echo "=== ${file} (via ${MODULE_INCLUDE})${NOTE} — partial: <${MISSING_INCLUDE}> is not installed ==="
        clang-tidy \
          "${TIDY_CONFIG_ARG}" \
          --header-filter="${HEADER_FILTER}" \
          --line-filter="${LINE_FILTER}" \
          "${DRIVER}" \
          -- "${DRIVER_COMPILE_ARGS[@]}" 2>&1 || true
        continue
      fi

      echo "=== ${file} (via ${MODULE_INCLUDE})${NOTE} ==="
      if ! clang-tidy \
            "${TIDY_ARGS[@]}" \
            --header-filter="${HEADER_FILTER}" \
            --line-filter="${LINE_FILTER}" \
            "${DRIVER}" \
            -- "${DRIVER_COMPILE_ARGS[@]}" 2>&1; then
        ERRORS=$((ERRORS + 1))
      fi
      ;;
  esac
done

if [ ${ERRORS} -gt 0 ]; then
  echo ""
  echo "clang-tidy reported issues in ${ERRORS} file(s)."
  exit 1
else
  echo ""
  echo "clang-tidy: all clean."
  exit 0
fi
