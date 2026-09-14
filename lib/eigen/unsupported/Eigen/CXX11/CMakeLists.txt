# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0
#
# Forwarding shims for the legacy <unsupported/Eigen/CXX11/...> include paths.

set(Eigen_LegacyShim_CXX11_HEADERS
  Tensor
  TensorSymmetry
  ThreadPool
  )

if(EIGEN_INSTALL)
  install(FILES
    ${Eigen_LegacyShim_CXX11_HEADERS}
    DESTINATION ${INCLUDE_INSTALL_DIR}/unsupported/Eigen/CXX11 COMPONENT Devel
    )
endif()
