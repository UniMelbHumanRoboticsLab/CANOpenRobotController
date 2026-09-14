#!/bin/bash
# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

set -x

rootdir=`pwd`

# The affected-tests tier (see scripts/affected_tests.py) passes its CTest
# filter as a file rather than a variable so the regex is not bounded by CI
# variable limits.  "ALL" means run everything the paired build produced,
# "NONE" means the merge request affects no test at all.
if [[ -n "${EIGEN_CI_CTEST_REGEX_FILE}" ]]; then
  regex_file="${EIGEN_CI_CTEST_REGEX_FILE}"
  [[ "${regex_file}" = /* ]] || regex_file="${rootdir}/${regex_file}"
  # Fail loudly rather than falling through: a missing selection would
  # otherwise silently run the whole suite against a partial build.
  if [[ ! -f "${regex_file}" ]]; then
    echo "EIGEN_CI_CTEST_REGEX_FILE=${EIGEN_CI_CTEST_REGEX_FILE} does not exist." >&2
    echo "The select:tests artifact is missing; refusing to guess a test filter." >&2
    exit 1
  fi
  selection=$(cat "${regex_file}")
  case "${selection}" in
    NONE)
      echo "No tests are affected by this merge request; nothing to run."
      set +x
      return 0 2>/dev/null || exit 0
      ;;
    ALL)
      EIGEN_CI_CTEST_REGEX=""
      ;;
    *)
      EIGEN_CI_CTEST_REGEX="${selection}"
      ;;
  esac
fi

cd ${EIGEN_CI_BUILDDIR}

target=""
if [[ ${EIGEN_CI_CTEST_REGEX} ]]; then
  target="-R ${EIGEN_CI_CTEST_REGEX}"
elif [[ ${EIGEN_CI_CTEST_LABEL} ]]; then
  target="-L ${EIGEN_CI_CTEST_LABEL}"
fi

exclude=""
if [[ -n "${EIGEN_CI_CTEST_EXCLUDE}" ]]; then
  exclude="-E ${EIGEN_CI_CTEST_EXCLUDE}"
fi

set +x

# Content-addressed pass cache (see ci/scripts/test_cache.py, which owns
# the keying and fingerprint policy): skip tests whose executable,
# emulator, CTest definition, and environment fingerprint match a
# first-attempt pass recorded by an earlier run of this job.
# Merge-request pipelines only: scheduled and web runs keep re-running
# identical binaries so the clock-seeded RNG keeps exploring fresh seeds.
# Sharded jobs must not skip -- dropping tests from the filtered list
# would shift the `-I index,,total` partition and could leave tests unrun
# in every shard; the ctest-args check fails closed for jobs that shard by
# hand rather than through `parallel:`.
testcache_active=false
if [[ "${EIGEN_CI_TEST_CACHE}" == "on" \
      && "${CI_PIPELINE_SOURCE:-}" == "merge_request_event" \
      && "${CI_NODE_TOTAL:-1}" -le 1 \
      && "${EIGEN_CI_CTEST_ARGS:-}" != *-I* \
      && -n "${EIGEN_CI_TEST_CACHE_DIR:-}" ]] && command -v python3 >/dev/null 2>&1; then
  mkdir -p "${EIGEN_CI_TEST_CACHE_DIR}"
  testcache_tmp=$(mktemp -d)
  if ctest --show-only=json-v1 ${target} ${exclude} \
       | python3 "${rootdir}/ci/scripts/test_cache.py" plan \
           --tests-json - \
           --manifest "${EIGEN_CI_TEST_CACHE_DIR}/passed.txt" \
           --config-root "${rootdir}" \
           --skip-regex-out "${testcache_tmp}/skip_regex.txt" \
           --keys-out "${testcache_tmp}/keys.txt"; then
    testcache_active=true
    skip_regex=$(cat "${testcache_tmp}/skip_regex.txt" 2>/dev/null || true)
    if [[ -n "${skip_regex}" ]]; then
      exclude="-E ${EIGEN_CI_CTEST_EXCLUDE:+(${EIGEN_CI_CTEST_EXCLUDE})|}${skip_regex}"
    fi
  fi
fi

EIGEN_CI_CTEST_PARALLEL=${EIGEN_CI_CTEST_PARALLEL:-${NPROC}}
# A `saas-*` tag names an instance type, but the pool behind it also holds
# self-hosted runners that carry the tag with fewer cores than that type has, so
# a job-supplied cap can land above the host it runs on.  Clamp rather than
# oversubscribe: the qemu jobs pick their cap to leave TCG contention headroom
# under EIGEN_CI_CTEST_ARGS' --timeout, and exceeding the core count spends
# exactly that headroom.  Both operands are digit-tested first: `-gt` evaluates
# its operands arithmetically, so a malformed value reaches ctest unclamped
# behind a bash diagnostic rather than being caught here.
if [[ "${EIGEN_CI_CTEST_PARALLEL}" =~ ^[0-9]+$ && "${NPROC}" =~ ^[0-9]+$ \
      && "${EIGEN_CI_CTEST_PARALLEL}" -gt "${NPROC}" ]]; then
  EIGEN_CI_CTEST_PARALLEL=${NPROC}
fi
# Total attempts for flaky tests (passed to ctest --repeat until-pass:N).
EIGEN_CI_CTEST_REPEAT=${EIGEN_CI_CTEST_REPEAT:-3}
# Per-test timeout for the retry phase. Retries exist to absorb seed-dependent
# flakes, which pass quickly when they pass; a test that hit the initial
# per-test timeout can never pass a full-length retry and would only burn up
# to EIGEN_CI_CTEST_REPEAT more timeouts (this pushed the qemu-emulated jobs
# past their job caps). A later --timeout on the ctest command line overrides
# an earlier one from EIGEN_CI_CTEST_ARGS.
EIGEN_CI_CTEST_RETRY_TIMEOUT=${EIGEN_CI_CTEST_RETRY_TIMEOUT:-600}
ctest_cmd="ctest ${EIGEN_CI_CTEST_ARGS} --parallel ${EIGEN_CI_CTEST_PARALLEL} --output-on-failure --no-compress-output --build-noclean ${target} ${exclude}"

echo "Running initial tests..."
# The job sources this script and GitLab Runner runs it under errexit, so
# a bare failing ctest would abort the job before the retry logic; capture
# the status through || instead.
initial_exit=0
${ctest_cmd} -T test || initial_exit=$?

# Fold first-attempt results into the pass cache.  The dashboard run's
# Test.xml is the authoritative status source: only "passed" is recorded,
# so failures, timeouts and SKIP_RETURN_CODE skips ("failed"/"notrun")
# never enter the manifest, and a ctest that died without writing results
# records nothing.  Passes obtained in the retry phase below are
# deliberately not recorded: a seed-flaky test keeps re-running.
if [[ "${testcache_active}" == "true" ]]; then
  python3 "${rootdir}/ci/scripts/test_cache.py" record \
      --manifest "${EIGEN_CI_TEST_CACHE_DIR}/passed.txt" \
      --keys "${testcache_tmp}/keys.txt" \
      --testing-dir Testing || true
fi

if [[ ${initial_exit} -eq 0 ]]; then
  echo "Tests passed on the first attempt."
  exit_code=0
else
  echo "Initial tests failed with exit code ${initial_exit}. Retrying up to ${EIGEN_CI_CTEST_REPEAT} times..."
  if ${ctest_cmd} --rerun-failed --repeat until-pass:${EIGEN_CI_CTEST_REPEAT} --timeout ${EIGEN_CI_CTEST_RETRY_TIMEOUT}; then
    echo "Tests passed on retry."
    # 42 = passed-on-retry; .test:linux / .test:windows whitelist it via
    # allow_failure.exit_codes so the job is marked as a soft warning.
    exit_code=42
  else
    exit_code=$?
  fi
fi

set -x

cd ${rootdir}

set +x

exit $exit_code
