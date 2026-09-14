# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

file(GLOB Eigen_EulerAngles_SRCS "*.h")

if(EIGEN_INSTALL)
  install(FILES
    ${Eigen_EulerAngles_SRCS}
    DESTINATION ${INCLUDE_INSTALL_DIR}/contrib/Eigen/src/EulerAngles COMPONENT Devel
    )
endif()
