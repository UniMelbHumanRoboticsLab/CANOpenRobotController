#!/bin/bash
# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

set -x

rootdir=`pwd`
mkdir -p ${EIGEN_CI_BUILDDIR}
cd ${EIGEN_CI_BUILDDIR}

# Compile through ccache when the job enables it and the image provides it.
# The GitLab cache holds ${CCACHE_DIR}, keyed on content and compiler, so it
# still hits after the fresh per-job clone re-stamps every source mtime
# (which makes any cached ninja state rebuild from scratch).
launchers=""
if [[ "${EIGEN_CI_CCACHE}" == "on" ]] && command -v ccache >/dev/null 2>&1; then
  launchers="-DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache"
  ccache --zero-stats
fi

cmake -G Ninja                                                   \
  -DCMAKE_CXX_COMPILER=${EIGEN_CI_CXX_COMPILER}                  \
  -DCMAKE_C_COMPILER=${EIGEN_CI_C_COMPILER}                      \
  -DCMAKE_CXX_COMPILER_TARGET=${EIGEN_CI_CXX_COMPILER_TARGET}    \
  ${launchers}                                                   \
  ${EIGEN_CI_ADDITIONAL_ARGS} ${rootdir}

# The affected-tests tier (see scripts/affected_tests.py) passes its selection
# as a file rather than a variable so the list is not bounded by CI variable
# limits.  The file holds "NONE" or one target per line; the full-suite form is
# "buildtests" plus the targets it does not aggregate, and so takes the same
# path as any other list.
# Targets that this configuration did not register (optional dependencies such
# as CHOLMOD, CUDA or SYCL) are dropped here: ninja aborts on an unknown target,
# and this is the first point that knows what CMake actually configured.
selected_targets=""
if [[ -n "${EIGEN_CI_BUILD_TARGET_FILE}" ]]; then
  target_file="${EIGEN_CI_BUILD_TARGET_FILE}"
  [[ "${target_file}" = /* ]] || target_file="${rootdir}/${target_file}"
  # Fail loudly rather than falling through to the default target: a missing
  # selection would otherwise silently build the entire test suite.
  if [[ ! -f "${target_file}" ]]; then
    echo "EIGEN_CI_BUILD_TARGET_FILE=${EIGEN_CI_BUILD_TARGET_FILE} does not exist." >&2
    echo "The select:tests artifact is missing; refusing to guess a build target." >&2
    exit 1
  fi
  requested=$(cat "${target_file}")
  if [[ "${requested}" == "NONE" ]]; then
    echo "No tests are affected by this merge request; nothing to build."
    cd ${rootdir}
    set +x
    return 0 2>/dev/null || exit 0
  else
    { set +x; } 2>/dev/null
    # The runner sources this script under `set -eo pipefail`, so every command
    # substitution below has to end in a success status: a failed ninja, or a
    # grep that legitimately counts zero lines, would otherwise abandon the job
    # before the checks that are meant to report it.
    configured=$(ninja -t targets all 2>/dev/null | sed -n 's/^\([A-Za-z_0-9]*\): phony$/\1/p' | sort -u || true)
    # An empty query means ninja is unusable, not that nothing is configured.
    # Without this the intersection below would be empty and the job would
    # trivially "succeed" having built nothing.
    if [[ -z "${configured}" ]]; then
      echo "Could not enumerate configured targets via 'ninja -t targets'." >&2
      exit 1
    fi
    requested_targets=$(echo "${requested}" | sort -u)
    selected_targets=$(comm -12 <(echo "${requested_targets}") <(echo "${configured}"))
    unconfigured=$(comm -23 <(echo "${requested_targets}") <(echo "${configured}"))
    nrequested=$(echo "${requested_targets}" | grep -c . || true)
    nselected=$(echo "${selected_targets}" | grep -c . || true)
    echo "Affected selection: ${nselected} of ${nrequested} requested targets are configured here."
    if [[ -n "${unconfigured}" ]]; then
      echo "Not configured in this build: $(echo "${unconfigured}" | tr '\n' ' ')"
    fi
    set -x
    if [[ -z "${selected_targets}" ]]; then
      echo "None of the affected tests exist in this configuration; nothing to build."
      cd ${rootdir}
      set +x
      return 0 2>/dev/null || exit 0
    fi
    EIGEN_CI_BUILD_TARGET=$(echo "${selected_targets}" | tr '\n' ' ')
  fi
fi

target=""
if [[ ${EIGEN_CI_BUILD_TARGET} ]]; then
  target="--target ${EIGEN_CI_BUILD_TARGET}"
fi

# Builds (particularly gcc) sometimes get killed, potentially when running
# out of resources.  In that case, keep trying to build the remaining
# targets (k0), then retry with reduced parallelism to minimize resource use.
# EIGEN_CI_BUILD_JOBS can be set to limit parallelism for memory-hungry
# compilers (e.g. NVHPC).  When unset, leave -j off so ninja picks its
# own default (NPROC + 2).  njobs is still needed for batch-size sizing.
njobs=${EIGEN_CI_BUILD_JOBS:-${NPROC}}
jobs=""
if [[ -n "${EIGEN_CI_BUILD_JOBS}" ]]; then
  jobs="-j${EIGEN_CI_BUILD_JOBS}"
fi

# Fallback parallelism for retry builds after a failure (default: 2).
fallback_jobs="-j${EIGEN_CI_FALLBACK_JOBS:-2}"

# For phony meta-targets (e.g. buildtests), shuffle the dependency list and
# build in batches so that memory-hungry compilations (like bdcsvd with
# nvc++) are spread out instead of all running at once.  Ninja ignores the
# command-line target order and schedules by its dependency graph, so we
# must feed it batches to actually influence scheduling.
#
# Every batch is a barrier, so the batch has to hold enough work to keep all
# cores busy until the slowest target in it finishes.  Per-target compile times
# are heavily right-skewed (median around 10s, worst case several minutes), so a
# batch only a small multiple of njobs drains almost immediately and then runs
# down to a single straggler.  Measured runner occupancy on the 2026-08-02
# nightly, at the old depth of max(2 * njobs, 48):
#
#   32 vCPU, batch 64   depth 1.9x   39% busy
#    8 vCPU, batch 48   depth 4.8x   66% busy
#    4 vCPU, batch 48   depth 8.0x   78% busy
#
# Peak concurrency, and hence peak memory, is set by -j and is unchanged by the
# batch size; the batch only controls which targets may co-run.  Override with
# EIGEN_CI_BUILD_BATCH_SIZE for fine-grained control.
default_batch=$((njobs * 8))
default_batch=$((default_batch > 96 ? default_batch : 96))
batch_size=${EIGEN_CI_BUILD_BATCH_SIZE:-${default_batch}}
shuffled=false
# An affected-tests selection names CMake targets, and most of those are
# aggregates: "buildtests", and the parent of every split test (bdcsvd depends
# on bdcsvd_1..bdcsvd_41).  The batch loop can only spread apart the targets it
# is handed, so an unexpanded parent puts a whole test family in one batch and
# lets all of its parts co-run.  expand_to_leaves resolves them first.
#
# CMake emits every aggregate as a phony edge, so descending through phony
# edges alone reaches the compiles and links without walking into object files:
# buildtests -> test/bdcsvd_1, bdcsvd -> test/bdcsvd -> test/bdcsvd_1, and a
# plain executable such as bug1213 -> test/bug1213, which is already a link
# edge and stays as it is.
expand_to_leaves() {
  local scratch depth
  scratch=$(mktemp -d)
  printf '%s\n' ${1} | awk 'NF' | sort -u > "${scratch}/frontier"
  : > "${scratch}/leaves"
  # Every level costs one ninja invocation over the whole frontier (about 0.1s
  # for the full suite).  Three levels are enough for the graph above; the cap
  # only bounds an unexpected cycle.
  for depth in 1 2 3 4 5 6 7 8; do
    [[ -s "${scratch}/frontier" ]] || break
    # A query that cannot be answered is not a reason to drop targets: leave
    # the frontier unexpanded and let ninja resolve it during the build.
    ninja -t query $(cat "${scratch}/frontier") > "${scratch}/query" 2>/dev/null || break
    awk '
      function flush() {
        if (name == "") return
        if (rule == "phony" && ninputs > 0) {
          for (i = 1; i <= ninputs; i++) print "N", inputs[i]
        } else {
          print "L", name
        }
        name = ""; rule = ""; ninputs = 0; section = ""
      }
      /^[^ ]/       { flush(); name = $0; sub(/:$/, "", name); next }
      /^  input:/   { rule = $2; section = "input"; next }
      /^  outputs:/ { section = ""; next }
      section == "input" && /^    / { inputs[++ninputs] = $1 }
      END { flush() }
    ' "${scratch}/query" > "${scratch}/classified"
    sed -n 's/^L //p' "${scratch}/classified" >> "${scratch}/leaves"
    sed -n 's/^N //p' "${scratch}/classified" | sort -u > "${scratch}/next"
    mv "${scratch}/next" "${scratch}/frontier"
  done
  # Whatever is still unexpanded -- a failed query or a chain past the cap --
  # is built as named.  Empty after a normal walk.
  cat "${scratch}/frontier" >> "${scratch}/leaves"
  sort -u "${scratch}/leaves"
  rm -rf "${scratch}"
}

deps=""
if [[ -n "${selected_targets}" ]] && command -v ninja >/dev/null 2>&1; then
  { set +x; } 2>/dev/null
  deps=$(expand_to_leaves "${selected_targets}")
# The meta-target path below quotes EIGEN_CI_BUILD_TARGET into `ninja -t query`,
# so a space-separated list (the SME cross-build's product_* targets) would go in
# as one bogus name and make the query fail, aborting the job before any build
# runs.  Skip batching for a list and let the plain `cmake --build --target t1 t2
# ...` at the end build it directly; it handles multiple targets.
elif [[ -n "${EIGEN_CI_BUILD_TARGET}" && "${EIGEN_CI_BUILD_TARGET}" != *[[:space:]]* ]] && command -v ninja >/dev/null 2>&1; then
  # Suppress xtrace while extracting and shuffling the target list
  # to avoid dumping ~1200 lines to the CI log.
  { set +x; } 2>/dev/null
  deps=$(ninja -t query "${EIGEN_CI_BUILD_TARGET}" 2>/dev/null \
         | awk '/^  input:/{found=1; next} /^  outputs:/{found=0} found && /^    /{print $1}')
  # CMake custom targets like BuildOfficial have an intermediate phony
  # (e.g. test/BuildOfficial) that holds the real dependencies.  If we
  # got exactly one dep, resolve it one more level.
  if [[ $(echo "$deps" | wc -l) -eq 1 ]] && [[ -n "$deps" ]]; then
    inner=$(ninja -t query "$deps" 2>/dev/null \
            | awk '/^  input:/{found=1; next} /^  outputs:/{found=0} found && /^    /{print $1}')
    if [[ -n "$inner" ]]; then
      deps="$inner"
    fi
  fi
fi

if [[ -n "${deps}" ]]; then
  # Deterministic shuffle: hash each target name and sort by hash.
  # Stable across runs (helps ninja's .ninja_log and build caches),
  # portable (no shuf dependency), and spreads same-family targets apart.
  # Uses Knuth's multiplicative hash (golden-ratio prime 2654435761) for
  # good avalanche — similar names like bdcsvd_1..bdcsvd_51 get widely
  # dispersed instead of clustering together.
  shuffled_deps=$(echo "$deps" | awk '
    BEGIN { for(i=0;i<128;i++) ord[sprintf("%c",i)]=i }
    { h=0
      for(i=1;i<=length($0);i++) h=(h+ord[substr($0,i,1)])*2654435761%2147483647
      printf "%010d %s\n",h,$0 }' | sort | sed 's/^[^ ]* //')
  if [[ -n "$shuffled_deps" ]]; then
    ndeps=$(echo "$shuffled_deps" | wc -l)
    echo "Building ${ndeps} targets in batches of ${batch_size} (njobs=${njobs})"
    shuffled=true
    # Build in batches: ninja parallelises within each batch, but batches
    # run sequentially so memory-hungry targets from different families
    # don't pile up simultaneously.  Track failures so we can report the
    # right exit code at the end.
    # Note: xtrace stays off to avoid dumping the full target list.
    # Use process substitution so the while loop runs in the current
    # shell and build_failed propagates.
    batch_num=0
    build_failed=false
    while IFS= read -r batch; do
      batch_num=$((batch_num + 1))
      echo "=== Batch ${batch_num} ==="
      ninja -k0 ${jobs} ${batch} || ninja -k0 ${fallback_jobs} ${batch} || build_failed=true
    done < <(echo "$shuffled_deps" | xargs -n "${batch_size}")
    if [[ "$build_failed" == "true" ]]; then
      echo "Some batches failed."
      # The cache is pushed even on failure (cache:when: always), so the
      # stats still describe what the next attempt can reuse.
      if [[ -n "${launchers}" ]]; then
        ccache --show-stats
      fi
      exit 1
    fi
  fi
  set -x
fi

if [[ "$shuffled" != "true" ]]; then
  cmake --build . ${target} -- -k0 ${jobs} || cmake --build . ${target} -- -k0 ${fallback_jobs}
fi

# Hit/miss summary for judging what the cache pays for on this job.
if [[ -n "${launchers}" ]]; then
  ccache --show-stats
fi

cd ${rootdir}

set +x
