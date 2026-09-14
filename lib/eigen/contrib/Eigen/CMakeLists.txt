# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

# When adding a new umbrella header here, also add a corresponding
# forwarding shim under unsupported/Eigen/ (and update the
# Eigen_LegacyShim_HEADERS list in unsupported/Eigen/CMakeLists.txt) so
# the legacy <unsupported/Eigen/...> include path keeps working.
set(Eigen_HEADERS
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
  )

if(EIGEN_INSTALL)
  install(FILES
    ${Eigen_HEADERS}
    DESTINATION ${INCLUDE_INSTALL_DIR}/contrib/Eigen COMPONENT Devel
    )

  install(DIRECTORY src DESTINATION ${INCLUDE_INSTALL_DIR}/contrib/Eigen COMPONENT Devel FILES_MATCHING PATTERN "*.h")
endif()
