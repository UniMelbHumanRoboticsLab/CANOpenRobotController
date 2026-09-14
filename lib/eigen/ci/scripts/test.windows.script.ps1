# Change to build directory.
# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0
$rootdir = Get-Location

# The affected-tests tier (see scripts/affected_tests.py) passes its CTest
# filter as a file rather than a variable so the regex is not bounded by CI
# variable limits.  "ALL" means run everything the paired build produced,
# "NONE" means the merge request affects no test at all.
if (${EIGEN_CI_CTEST_REGEX_FILE}) {
  $regex_file = ${EIGEN_CI_CTEST_REGEX_FILE}
  if (-Not [System.IO.Path]::IsPathRooted($regex_file)) {
    $regex_file = Join-Path ${rootdir} $regex_file
  }
  # Fail loudly rather than falling through: a missing selection would
  # otherwise silently run the whole suite against a partial build.
  if (-Not (Test-Path $regex_file)) {
    Write-Error ("EIGEN_CI_CTEST_REGEX_FILE=${EIGEN_CI_CTEST_REGEX_FILE} does not exist. " +
                 "The select:tests artifact is missing; refusing to guess a test filter.")
    Exit 1
  }
  $selection = (Get-Content $regex_file -Raw).Trim()
  if ($selection -eq "NONE") {
    Write-Host "No tests are affected by this merge request; nothing to run."
    Exit 0
  } elseif ($selection -eq "ALL") {
    $EIGEN_CI_CTEST_REGEX = ""
  } else {
    $EIGEN_CI_CTEST_REGEX = $selection
  }
}

cd $EIGEN_CI_BUILDDIR

# Determine number of processors for parallel tests.
$NPROC=${Env:NUMBER_OF_PROCESSORS}

# Total attempts for flaky tests, matching test.linux.script.sh.  No job sets
# this, and an empty value made the retry below spell "until-pass:", which ctest
# rejects as a usage error -- so the retry never ran and its exit status masked
# the test failure that triggered it.
if (-Not ${EIGEN_CI_CTEST_REPEAT}) { $EIGEN_CI_CTEST_REPEAT = 3 }

# @(), not "": an "ALL" selection leaves no regex and no label, so the
# no-filter case is reachable and must contribute no argument to ctest.
$target = @()
if (${EIGEN_CI_CTEST_REGEX}) {
  $target = "-R","${EIGEN_CI_CTEST_REGEX}"
} elseif (${EIGEN_CI_CTEST_LABEL}) {
  $target = "-L","${EIGEN_CI_CTEST_LABEL}"
}

# Same knob as test.linux.script.sh, for tests this runner cannot execute.
$exclude = @()
if (${EIGEN_CI_CTEST_EXCLUDE}) {
  $exclude = "-E","${EIGEN_CI_CTEST_EXCLUDE}"
}

$ctest_cmd = { ctest ${EIGEN_CI_CTEST_ARGS} --parallel ${NPROC} --output-on-failure --no-compress-output --build-noclean ${target} ${exclude} @args }

Write-Host "Running initial tests..."

& $ctest_cmd "-T" "test"
$exit_code = $LASTEXITCODE

if ($exit_code -eq 0) {
  Write-Host "Tests passed on the first attempt."
}
else {
  Write-Host "Initial tests failed with exit code $exit_code. Retrying up to $EIGEN_CI_CTEST_REPEAT times..."
  & $ctest_cmd "--rerun-failed" "--repeat" "until-pass:$EIGEN_CI_CTEST_REPEAT"
  $exit_code = $LASTEXITCODE

  if ($exit_code -eq 0) {
    Write-Host "Tests passed on retry."
    $exit_code = 42
  }
}

# Return to root directory.
cd ${rootdir}

Exit $exit_code
