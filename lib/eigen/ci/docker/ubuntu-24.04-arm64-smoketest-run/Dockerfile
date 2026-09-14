# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0
#
# Minimal CI image for running aarch64 MR smoke tests (Ubuntu 24.04).
# Test-only: no compilers. Binaries are built natively on arm64 runners
# using ubuntu-24.04-arm64-smoketest-build and transferred as artifacts.
#
# Packages:
#   cmake     - provides ctest
#   xsltproc  - converts CTest XML to JUnit in test.linux.after_script.sh
#   libgomp1  - OpenMP runtime for test binaries (libstdc++6/libgcc-s1
#               are already present in the Ubuntu 24.04 base image)
#   python3   - drives the test pass cache (ci/scripts/test_cache.py)
#
# Rebuild and push when this file changes:
#   docker buildx build --platform linux/arm64 \
#     -t registry.gitlab.com/libeigen/eigen/ubuntu-24.04-arm64-smoketest-run:latest \
#     --push ci/docker/ubuntu-24.04-arm64-smoketest-run/

FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y && \
    apt-get install -y --no-install-recommends \
      cmake \
      xsltproc \
      libgomp1 \
      python3 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
