# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0
#
# Slim CI image for amd64 MR smoke test jobs (Ubuntu 24.04).
#
# Rebuild and push when this file changes:
#   docker buildx build --platform linux/amd64 \
#     -t registry.gitlab.com/libeigen/eigen/ubuntu-24.04-amd64-smoketest-build:latest \
#     --push ci/docker/ubuntu-24.04-amd64-smoketest-build/

FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y && \
    apt-get install -y --no-install-recommends \
      cmake ninja-build git ccache \
      gcc-10 g++-10 \
      g++-10-aarch64-linux-gnu \
      clang-14 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
