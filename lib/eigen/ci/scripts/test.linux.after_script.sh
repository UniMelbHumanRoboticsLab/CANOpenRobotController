#!/bin/bash
# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

set -x

rootdir=`pwd`
cd ${EIGEN_CI_BUILDDIR}

# Generate test results. xsltproc is installed by common.linux.before_script.sh.
#
# An affected-tier job whose selection is NONE returns before ctest runs, so
# there is no dashboard run to convert and a zero-byte file would be reported
# as a JUnit parse error.  (A fully cached job is not this case: ctest runs and
# an empty <TestList> converts to a valid tests="0" suite.)  No `exit` -- this
# file is sourced.
if [[ -r Testing/TAG ]]; then
  tag=$(head -n 1 < Testing/TAG)
  if [[ -r "Testing/${tag}/Test.xml" ]]; then
    xsltproc ${rootdir}/ci/CTest2JUnit.xsl "Testing/${tag}/Test.xml" > "JUnitTestResults_$CI_JOB_ID.xml" \
      || rm -f "JUnitTestResults_$CI_JOB_ID.xml"
  fi
fi

cd ${rootdir}

set +x
