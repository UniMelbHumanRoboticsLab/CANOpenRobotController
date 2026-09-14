# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0
#
# CI image for riscv64 build jobs -- full builds and MR smoke tests (Ubuntu 26.04).
# This is an amd64 image: these builds run on an amd64 CI runner and
# cross-compile to riscv64, so it carries the riscv64 cross toolchain
# (g++-15-riscv64-linux-gnu, which also provides the riscv64 libstdc++) plus
# clang-21 (which cross-compiles via --target). cmake/ninja/git/ccache are baked
# in too so the jobs can run with EIGEN_CI_SKIP_APT=true and skip apt entirely.
#
# Rebuild and push when this file changes:
#   docker buildx build --platform linux/amd64 \
#     -t registry.gitlab.com/libeigen/eigen/ubuntu-26.04-riscv64-smoketest-build:latest \
#     --push ci/docker/ubuntu-26.04-riscv64-smoketest-build/

FROM ubuntu:26.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y && \
    apt-get install -y --no-install-recommends \
      cmake ninja-build git ccache \
      g++-15-riscv64-linux-gnu \
      clang-21 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
