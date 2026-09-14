# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0
#
# Forwarding shims for the legacy <unsupported/Eigen/...> include paths.
# The actual module headers live under ../../contrib/Eigen/.

set(Eigen_LegacyShim_HEADERS
  AdolcForward
  AlignedVector3
  ArpackSupport
  AutoDiff
  BVH
  EulerAngles
  FFT
  GPU
  IterativeSolvers
  KroneckerProduct
  LevenbergMarquardt
  MatrixFunctions
  MPRealSupport
  NNLS
  NonLinearOptimization
  NumericalDiff
  OpenGLSupport
  Polynomials
  SparseExtra
  SpecialFunctions
  Splines
  StructuredMatrices
  Tensor
  TensorSymmetry
  ThreadPool
  )

if(EIGEN_INSTALL)
  install(FILES
    ${Eigen_LegacyShim_HEADERS}
    DESTINATION ${INCLUDE_INSTALL_DIR}/unsupported/Eigen COMPONENT Devel
    )
endif()

add_subdirectory(CXX11)
