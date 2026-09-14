#!/bin/bash
# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

# Configures Eigen and runs only the build-system integration tests under
# test/buildsystem.
#
# This job builds and consumes no artifact, unlike the paired build/test jobs:
# each scenario drives its own nested configure, build, and install, so the
# enclosing build directory holds nothing but the CTest definitions.  For the
# same reason the job stays off the content-addressed pass cache, which keys on
# a test's executable; these tests have none, so a recorded pass would outlive
# any change to Eigen's CMake files.

set -ex

rootdir=$(pwd)
mkdir -p "${EIGEN_CI_BUILDDIR}"
cd "${EIGEN_CI_BUILDDIR}"

# The scenarios configure Eigen themselves with whatever options they need, so
# this configure only has to register the tests.
#
# EIGEN_CI_ADDITIONAL_ARGS carries several -D options in a single variable and
# has to word-split; ci/build.linux.gitlab-ci.yml documents that contract for
# the jobs that set it.  Every other expansion here is a single word.
# shellcheck disable=SC2086
cmake -G Ninja                                    \
  "-DCMAKE_C_COMPILER=${EIGEN_CI_C_COMPILER}"     \
  "-DCMAKE_CXX_COMPILER=${EIGEN_CI_CXX_COMPILER}" \
  -DEIGEN_BUILD_TESTING=ON                        \
  -DEIGEN_BUILD_DOC=OFF                           \
  -DEIGEN_BUILD_DEMOS=OFF                         \
  -DEIGEN_BUILD_BLAS=OFF                          \
  -DEIGEN_BUILD_LAPACK=OFF                        \
  ${EIGEN_CI_ADDITIONAL_ARGS} "${rootdir}"

# --no-tests=error matters more here than in the paired test jobs: everything
# that registers these tests is itself CMake, so a guard in
# test/buildsystem/CMakeLists.txt that stops matching would otherwise leave
# ctest reporting success over an empty selection.
ctest -L buildsystem --output-on-failure --no-tests=error "-j${NPROC}"

cd "${rootdir}"

set +x
