# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0
#
# Minimal CI image for running riscv64 MR smoke tests (Ubuntu 26.04).
# Test-only: no compilers. Binaries are cross-built on an amd64 runner and
# transferred as artifacts; this image just runs them on the riscv64 runner.
#
# Packages overlaid onto the riscv64 base:
#   cmake     - provides ctest
#   xsltproc  - converts CTest XML to JUnit in test.linux.after_script.sh
#   libgomp1  - OpenMP runtime for test binaries (libstdc++6/libgcc-s1
#               are already present in the Ubuntu 26.04 base image)
#   python3   - drives the test pass cache (ci/scripts/test_cache.py)
#
# This image is assembled WITHOUT emulation: the first stage runs natively on
# the amd64 build host, downloads the riscv64 .debs (and their dependency
# closure) and unpacks them into a rootfs; the final riscv64 stage only copies
# those files in (a single COPY), so no riscv64 code runs at build time. That
# avoids the qemu-emulated apt-get that otherwise hangs when building a riscv64
# image on an amd64 runner.
#
# Rebuild and push when this file changes:
#   docker buildx build --platform linux/riscv64 \
#     -t registry.gitlab.com/libeigen/eigen/ubuntu-26.04-riscv64-smoketest-run:latest \
#     --push ci/docker/ubuntu-26.04-riscv64-smoketest-run/

# --- fetch stage: runs natively on the build platform (amd64), no emulation ---
FROM --platform=$BUILDPLATFORM ubuntu:26.04 AS fetch

ARG DEBIAN_FRONTEND=noninteractive

# Enable the riscv64 architecture. riscv64 lives on ports.ubuntu.com, so pin the
# stock (amd64) sources to amd64 and add a riscv64 source pointing at the ports
# mirror. The suite codename is read from the base image so it tracks the FROM.
RUN dpkg --add-architecture riscv64 && \
    sed -i '/^Types:/a Architectures: amd64' /etc/apt/sources.list.d/ubuntu.sources && \
    . /etc/os-release && \
    printf 'Types: deb\nURIs: http://ports.ubuntu.com/ubuntu-ports\nSuites: %s %s-updates %s-security\nComponents: main universe\nArchitectures: riscv64\nSigned-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg\n' \
      "$VERSION_CODENAME" "$VERSION_CODENAME" "$VERSION_CODENAME" \
      > /etc/apt/sources.list.d/riscv64-ports.sources && \
    apt-get update -y

# Download the riscv64 packages and their full dependency closure (no install,
# no maintainer scripts -> no execution), then unpack each .deb into /rootfs.
RUN apt-get install -y --no-install-recommends --download-only \
      cmake:riscv64 \
      xsltproc:riscv64 \
      libgomp1:riscv64 \
      python3:riscv64 && \
    mkdir -p /rootfs && \
    for deb in /var/cache/apt/archives/*.deb; do dpkg-deb -x "$deb" /rootfs; done

# --- final image: riscv64, assembled by copying the unpacked files in ---
FROM ubuntu:26.04

COPY --from=fetch /rootfs/ /
