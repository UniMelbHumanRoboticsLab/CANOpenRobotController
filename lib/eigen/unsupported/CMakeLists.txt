# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

# Backward-compatibility shell for the legacy unsupported/ tree. The real
# content moved to ../contrib/ in Eigen 5.x; only forwarding headers remain
# here so that existing user code with #include <unsupported/Eigen/...> keeps
# building. Tests, docs, and benchmarks live under ../contrib/.

add_subdirectory(Eigen)
