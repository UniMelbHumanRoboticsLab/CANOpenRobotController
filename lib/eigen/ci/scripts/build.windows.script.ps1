# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

$rootdir = Get-Location

# The affected-tests tier (see scripts/affected_tests.py) passes its selection
# as a file rather than a variable so the list is not bounded by CI variable
# limits.  The file holds "NONE" or one target per line; the full-suite form is
# "buildtests" plus the targets it does not aggregate, and so takes the same
# path as any other list.
#
# Read first, before vcvarsall, the ccache provisioning and the CMake configure:
# a diff that reaches no test at all selects "NONE", and such a job should cost
# a checkout rather than a full configure.  Declining to schedule it at all
# would have to go through `rules:`, which GitLab evaluates when the pipeline is
# created -- before select:tests has produced this file -- so exiting early is
# the cheapest answer available within one pipeline.
$requested = @()
if (${EIGEN_CI_BUILD_TARGET_FILE}) {
  $target_file = ${EIGEN_CI_BUILD_TARGET_FILE}
  if (-Not [System.IO.Path]::IsPathRooted($target_file)) {
    $target_file = Join-Path ${rootdir} $target_file
  }
  # Fail loudly rather than falling through to the default target: a missing
  # selection would otherwise silently build the entire test suite.
  if (-Not (Test-Path $target_file)) {
    Write-Error ("EIGEN_CI_BUILD_TARGET_FILE=${EIGEN_CI_BUILD_TARGET_FILE} does not exist. " +
                 "The select:tests artifact is missing; refusing to guess a build target.")
    Exit 1
  }
  $requested = @(Get-Content $target_file | ForEach-Object { $_.Trim() } |
                 Where-Object { $_ } | Sort-Object -Unique)
  if ($requested -contains "NONE") {
    Write-Host "No tests are affected by this merge request; nothing to build."
    Exit 0
  }
}

# Find Visual Studio installation directory. -products * includes Build
# Tools installs, which vswhere's default product filter skips; without it a
# Build Tools-only runner gets an empty path here, vcvarsall never runs, and
# the configure step fails with "No CMAKE_CXX_COMPILER could be found".
$VS_INSTALL_DIR = &"${Env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -property installationPath

# Run VCVarsAll.bat initialization script and extract environment variables.
# http://allen-mack.blogspot.com/2008/03/replace-visual-studio-command-prompt.html
cmd.exe /c "`"${VS_INSTALL_DIR}\VC\Auxiliary\Build\vcvarsall.bat`" $EIGEN_CI_MSVC_ARCH -vcvars_ver=$EIGEN_CI_MSVC_VER & set" |
  foreach {
    if ($_ -match "^([^=]+)=(.*)$") {
      set-item -force -LiteralPath "ENV:\$($Matches[1])" -value "$($Matches[2])"
    }
  }

# Create and enter build directory.
if (-Not (Test-Path ${EIGEN_CI_BUILDDIR})) {
    mkdir $EIGEN_CI_BUILDDIR
}
cd $EIGEN_CI_BUILDDIR

# We need to split EIGEN_CI_ADDITIONAL_ARGS, otherwise they are interpreted
# as a single argument.  Split by space, unless double-quoted.
$split_args = [regex]::Split(${EIGEN_CI_ADDITIONAL_ARGS}, ' (?=(?:[^"]|"[^"]*")*$)' )

# Locate ccache when the job enables it (mirrors build.linux.script.sh: the
# GitLab cache holds ${CCACHE_DIR}, keyed on file content and compiler, so it
# hits even after a re-checkout re-stamps every source mtime). Prefer a
# runner-installed ccache, then the copy restored from the .ccache-bin cache;
# otherwise download the release pinned below and cache it for subsequent
# jobs on this runner. Trust is established at every use, not at download
# time: a runner-installed ccache must report at least the version required
# for MSVC support (older ones would break compiles rather than degrade), and
# a restored or freshly extracted binary must match the pinned SHA-256 — the
# GitLab cache is writable job output, so restoring it does not authenticate
# it. Re-verifying per use also means a version bump below invalidates stale
# cached copies by itself. Any failure only means building without ccache,
# never running an unverified binary. Only /Z7 or no debug information is
# cacheable by ccache; these MinSizeRel builds emit none.
$ccache_exe = ""
if ("${EIGEN_CI_CCACHE}" -eq "on") {
  $ccache_version = "4.13.6"
  $ccache_zip_sha256 = "3d7cebb05850ad704e197b3f1d3f0f924ab6c9fdfc561578e146184fe9d89380"
  $ccache_exe_sha256 = "1f285645553e61463b000c6c440301724894b4ba0174dbef6b4818d54b54b68d"
  # Caching MSVC needs ccache >= 4.8.
  $ccache_min_version = [version]"4.8"
  $ccache_bindir = Join-Path ${rootdir} ".ccache-bin"
  $cached_exe = Join-Path $ccache_bindir "ccache.exe"

  $system_ccache = Get-Command ccache -ErrorAction SilentlyContinue
  if ($system_ccache) {
    $system_version = ""
    try {
      $system_version = (& $system_ccache.Source --version | Select-Object -First 1)
    } catch {}
    if (($system_version -match "ccache version (\d+(\.\d+)+)") -and
        ([version]$Matches[1] -ge $ccache_min_version)) {
      $ccache_exe = $system_ccache.Source
    } else {
      Write-Warning ("Ignoring system ccache at $($system_ccache.Source) " +
                     "('${system_version}'; need >= ${ccache_min_version}).")
    }
  }

  if ((-Not $ccache_exe) -and (Test-Path $cached_exe)) {
    if ((Get-FileHash $cached_exe -Algorithm SHA256).Hash -eq $ccache_exe_sha256) {
      $ccache_exe = $cached_exe
    } else {
      Write-Warning "Discarding cached ccache.exe that failed SHA-256 verification."
      Remove-Item $cached_exe -Force
    }
  }

  if (-Not $ccache_exe) {
    $zip = Join-Path ([System.IO.Path]::GetTempPath()) "ccache-${ccache_version}.zip"
    try {
      $ProgressPreference = "SilentlyContinue"
      Invoke-WebRequest "https://github.com/ccache/ccache/releases/download/v${ccache_version}/ccache-${ccache_version}-windows-x86_64.zip" -OutFile $zip
      if ((Get-FileHash $zip -Algorithm SHA256).Hash -eq $ccache_zip_sha256) {
        $unpack = Join-Path ([System.IO.Path]::GetTempPath()) "ccache-unpack"
        Expand-Archive $zip -DestinationPath $unpack -Force
        New-Item -ItemType Directory -Force -Path $ccache_bindir | Out-Null
        Copy-Item (Join-Path $unpack "ccache-${ccache_version}-windows-x86_64/ccache.exe") $cached_exe
        if ((Get-FileHash $cached_exe -Algorithm SHA256).Hash -eq $ccache_exe_sha256) {
          $ccache_exe = $cached_exe
        } else {
          Write-Warning "Extracted ccache.exe failed SHA-256 verification; building without ccache."
          Remove-Item $cached_exe -Force
        }
      } else {
        Write-Warning "ccache download failed SHA-256 verification; building without ccache."
      }
    } catch {
      Write-Warning "ccache download failed ($_); building without ccache."
    }
  }
}

$launchers = @()
if ($ccache_exe) {
  # Forward slashes: CMake treats the launcher as a path-valued cache entry.
  $ccache_cmake = $ccache_exe -replace '\\', '/'
  $launchers = "-DCMAKE_C_COMPILER_LAUNCHER=${ccache_cmake}",
               "-DCMAKE_CXX_COMPILER_LAUNCHER=${ccache_cmake}"
  & $ccache_exe --zero-stats
}

# Configure build.
cmake -G Ninja -DCMAKE_BUILD_TYPE=MinSizeRel `
      -DEIGEN_TEST_CUSTOM_CXX_FLAGS="${EIGEN_CI_TEST_CUSTOM_CXX_FLAGS}" `
      ${launchers} ${split_args} "${rootdir}"

# Targets that this configuration did not register (optional dependencies such
# as CHOLMOD or CUDA) are dropped from the selection read above: ninja aborts on
# an unknown target, and this is the first point that knows what CMake actually
# configured.
$selected_targets = @()
if (${EIGEN_CI_BUILD_TARGET_FILE}) {
  # Not redirected with 2>: the runner sets $ErrorActionPreference to Stop, and
  # redirecting a native command's stderr promotes each line it writes to a
  # terminating error, which would abort the job ahead of the report below.
  $configured = @{}
  try {
    foreach ($line in (ninja -t targets all)) {
      if ($line -match '^([A-Za-z_0-9]+): phony$') { $configured[$Matches[1]] = $true }
    }
  } catch {
    Write-Warning "Enumerating configured targets failed: $_"
  }
  # An empty query means ninja is unusable, not that nothing is configured.
  # Without this the intersection below would be empty and the job would
  # trivially "succeed" having built nothing.
  if ($configured.Count -eq 0) {
    Write-Error "Could not enumerate configured targets via 'ninja -t targets'."
    cd ${rootdir}
    Exit 1
  }
  $selected_targets = @($requested | Where-Object { $configured.ContainsKey($_) })
  $unconfigured = @($requested | Where-Object { -Not $configured.ContainsKey($_) })
  Write-Host ("Affected selection: {0} of {1} requested targets are configured here." -f
              $selected_targets.Count, $requested.Count)
  if ($unconfigured.Count -gt 0) {
    Write-Host ("Not configured in this build: " + ($unconfigured -join " "))
  }
  if ($selected_targets.Count -eq 0) {
    Write-Host "None of the affected tests exist in this configuration; nothing to build."
    cd ${rootdir}
    Exit 0
  }
}

# Built as an array, not a string: PowerShell hands a single string containing
# spaces to a native command as one argument, which cmake rejects as an unknown
# target name.  An affected selection is always a list, and the SME-style
# multi-target EIGEN_CI_BUILD_TARGET is one too.
$target = @()
if ($selected_targets.Count -gt 0) {
  $target = @("--target") + $selected_targets
} elseif (${EIGEN_CI_BUILD_TARGET}) {
  $target = @("--target") + @(${EIGEN_CI_BUILD_TARGET} -split '\s+' | Where-Object { $_ })
}

# Windows builds sometimes fail due heap errors. In that case, try
# building the rest, then try to build again with a single thread.
cmake --build . ${target} -- -k0 || cmake --build . ${target} -- -k0 -j1

$success = $LASTEXITCODE

# Hit/miss summary for judging what the cache pays for on this job. Runs on
# failures too: the cache is pushed even then (cache:when: always), so the
# stats still describe what the next attempt can reuse.
if ($ccache_exe) {
  & $ccache_exe --show-stats
}

# Return to root directory.
cd ${rootdir}

# Explicitly propagate exit code to indicate pass/failure of build command.
if($success -ne 0) { Exit $success }
