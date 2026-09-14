# Change to build directory.
# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0
$rootdir = Get-Location
cd ${EIGEN_CI_BUILDDIR}

# No dashboard run to convert when ctest never reached its test phase; an
# empty report is a parse error, so write none.  See the Linux after_script.
$TEST_TAG = ""
if (Test-Path Testing\TAG) { $TEST_TAG = Get-Content Testing\TAG | select -first 1 }
if ($TEST_TAG -and (Test-Path Testing\$TEST_TAG\Test.xml)) {
  # PowerShell equivalent to xsltproc:
  $XSL_FILE = Resolve-Path "..\ci\CTest2JUnit.xsl"
  $INPUT_FILE = Resolve-Path Testing\$TEST_TAG\Test.xml
  $OUTPUT_FILE = Join-Path -Path $pwd -ChildPath JUnitTestResults_$CI_JOB_ID.xml
  # Transform() opens the output before it parses the input, so a Test.xml
  # truncated by a ctest the job timeout killed would leave a partial file
  # for the runner to register.  Same reason as the Linux `|| rm -f`.
  try {
    $xslt = New-Object System.Xml.Xsl.XslCompiledTransform;
    $xslt.Load($XSL_FILE)
    $xslt.Transform($INPUT_FILE,$OUTPUT_FILE)
  } catch {
    Write-Host "Could not convert ${INPUT_FILE}: $_"
    Remove-Item -Force -ErrorAction SilentlyContinue $OUTPUT_FILE
  }
}

# Return to root directory.
cd ${rootdir}
