// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2013 Gauthier Brun <brun.gauthier@gmail.com>
// Copyright (C) 2013 Nicolas Carre <nicolas.carre@ensimag.fr>
// Copyright (C) 2013 Jean Ceccato <jean.ceccato@ensimag.fr>
// Copyright (C) 2013 Pierre Zoppitelli <pierre.zoppitelli@ensimag.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/
// SPDX-License-Identifier: MPL-2.0

// discard stack allocation as that too bypasses malloc
#define EIGEN_STACK_ALLOCATION_LIMIT 0
#define EIGEN_RUNTIME_NO_MALLOC

#include "main.h"
#include "fp_control.h"
#include "tridiag_test_matrices.h"
#include <Eigen/SVD>
#include <cstdlib>

#define SVD_DEFAULT(M) BDCSVD<M>
#define SVD_FOR_MIN_NORM(M) BDCSVD<M>
#define SVD_STATIC_OPTIONS(M, O) BDCSVD<M, O>
#include "svd_common.h"

template <typename MatrixType>
void bdcsvd_method() {
  enum { Size = MatrixType::RowsAtCompileTime };
  typedef typename MatrixType::RealScalar RealScalar;
  typedef Matrix<RealScalar, Size, 1> RealVecType;
  MatrixType m = MatrixType::Identity();
  VERIFY_IS_APPROX(m.bdcSvd().singularValues(), RealVecType::Ones());
  VERIFY_RAISES_ASSERT(m.bdcSvd().matrixU());
  VERIFY_RAISES_ASSERT(m.bdcSvd().matrixV());
}

// compare the Singular values returned with Jacobi and Bdc
template <typename MatrixType>
void compare_bdc_jacobi(const MatrixType& a = MatrixType(), int algoswap = 16, bool random = true) {
  MatrixType m = random ? MatrixType::Random(a.rows(), a.cols()) : a;

  BDCSVD<MatrixType> bdc_svd(m.rows(), m.cols());
  bdc_svd.setSwitchSize(algoswap);
  bdc_svd.compute(m);

  JacobiSVD<MatrixType> jacobi_svd(m);
  VERIFY_IS_APPROX(bdc_svd.singularValues(), jacobi_svd.singularValues());
}

#if defined(EIGEN_TEST_PART_46) || defined(EIGEN_TEST_PART_47) || defined(EIGEN_TEST_PART_48) || \
    defined(EIGEN_TEST_PART_49) || defined(EIGEN_TEST_PART_ALL)
// Verifies total deflation is **not** triggered.
void compare_bdc_jacobi_instance(bool structure_as_m, int algoswap = 16) {
  MatrixXd m(4, 3);
  if (structure_as_m) {
    // The first 3 rows are the reduced form of Matrix 1 as shown below, and it
    // has nonzero elements in the first column and diagonals only.
    m << 1.056293, 0, 0, -0.336468, 0.907359, 0, -1.566245, 0, 0.149150, -0.1, 0, 0;
  } else {
    // Matrix 1.
    m << 0.882336, 18.3914, -26.7921, -5.58135, 17.1931, -24.0892, -20.794, 8.68496, -4.83103, -8.4981, -10.5451,
        23.9072;
  }
  compare_bdc_jacobi(m, algoswap, false);
}
#endif

template <typename MatrixType>
void bdcsvd_thin_full_options(const MatrixType& input = MatrixType()) {
  svd_thin_full_option_checks<MatrixType, 0>(input);
  svd_verify_constructor_options_assert<BDCSVD<MatrixType>>(input);
}

template <typename MatrixType>
void bdcsvd_asserts(const MatrixType& input = MatrixType()) {
  MatrixType m(input.rows(), input.cols());
  svd_fill_random(m);

  svd_verify_assert<MatrixType>(m);
  svd_verify_constructor_options_assert<BDCSVD<MatrixType>>(m);
}

template <typename MatrixType>
void bdcsvd_check_convergence(const MatrixType& input) {
  BDCSVD<MatrixType, Eigen::ComputeThinU | Eigen::ComputeThinV> svd(input);
  VERIFY(svd.info() == Eigen::Success);
  MatrixType D = svd.matrixU() * svd.singularValues().asDiagonal() * svd.matrixV().transpose();
  VERIFY_IS_APPROX(input, D);
}

// Verify SVD of bidiagonal matrix given as diagonal + superdiagonal vectors.
template <typename RealScalar>
void verify_bidiagonal_svd(const Matrix<RealScalar, Dynamic, 1>& diag,
                           const Matrix<RealScalar, Dynamic, 1>& superdiag) {
  typedef Matrix<RealScalar, Dynamic, Dynamic> MatrixXr;
  typedef Matrix<RealScalar, Dynamic, 1> VectorXr;
  const Index n = diag.size();

  BDCSVD<MatrixXr, ComputeFullU | ComputeFullV> bdcsvd(diag, superdiag);
  VERIFY(bdcsvd.info() == Success);

  const VectorXr& sv = bdcsvd.singularValues();

  // Singular values must be non-negative.
  for (Index i = 0; i < sv.size(); ++i) {
    VERIFY(sv(i) >= RealScalar(0));
  }

  // Singular values must be sorted descending.
  for (Index i = 1; i < sv.size(); ++i) {
    VERIFY(sv(i - 1) >= sv(i));
  }

  // Orthogonality of U and V.
  VERIFY_IS_APPROX(bdcsvd.matrixU().transpose() * bdcsvd.matrixU(), MatrixXr::Identity(n, n));
  VERIFY_IS_APPROX(bdcsvd.matrixV().transpose() * bdcsvd.matrixV(), MatrixXr::Identity(n, n));

  // Reconstruction: U * S * V^T should equal the original bidiagonal.
  MatrixXr B = MatrixXr::Zero(n, n);
  B.diagonal() = diag;
  if (n > 1) B.diagonal(1) = superdiag;
  MatrixXr recon = bdcsvd.matrixU() * sv.asDiagonal() * bdcsvd.matrixV().transpose();
  VERIFY_IS_APPROX(recon, B);

  // Cross-validate singular values against JacobiSVD.
  JacobiSVD<MatrixXr> jacobi(B);
  VERIFY_IS_APPROX(sv, jacobi.singularValues());
}

// Verify that bidiagonal API and matrix API produce matching singular values.
template <typename RealScalar>
void verify_bidiagonal_vs_matrix_svd(const Matrix<RealScalar, Dynamic, 1>& diag,
                                     const Matrix<RealScalar, Dynamic, 1>& superdiag) {
  typedef Matrix<RealScalar, Dynamic, Dynamic> MatrixXr;
  const Index n = diag.size();

  // Build dense bidiagonal matrix.
  MatrixXr B = MatrixXr::Zero(n, n);
  B.diagonal() = diag;
  if (n > 1) B.diagonal(1) = superdiag;

  BDCSVD<MatrixXr> bidiag_svd(diag, superdiag);
  BDCSVD<MatrixXr> matrix_svd(B);

  VERIFY(bidiag_svd.info() == Success);
  VERIFY(matrix_svd.info() == Success);
  VERIFY_IS_APPROX(bidiag_svd.singularValues(), matrix_svd.singularValues());
}

template <typename RealScalar>
void bdcsvd_bidiagonal_hard_cases() {
  Eigen::internal::set_is_malloc_allowed(true);

  // Use the shared tridiagonal test matrix generators.
  // Each generator fills (diag, offdiag) which we treat as (diagonal, superdiagonal)
  // of a bidiagonal matrix.
  test::for_all_tridiag_test_matrices<RealScalar>(
      [](const auto& diag, const auto& offdiag) { verify_bidiagonal_svd<RealScalar>(diag, offdiag); });

  // Additional SVD-specific test: identity with cross-validation against full matrix SVD.
  test::for_tridiag_sizes<RealScalar>([](auto& diag, auto& offdiag) {
    test::tridiag_identity(diag, offdiag);
    verify_bidiagonal_vs_matrix_svd<RealScalar>(diag, offdiag);
  });

  // Additional SVD-specific test: scalar for n=1.
  {
    typedef Matrix<RealScalar, Dynamic, 1> VectorXr;
    VectorXr diag(1), offdiag(0);
    diag(0) = RealScalar(3.14);
    verify_bidiagonal_svd<RealScalar>(diag, offdiag);
  }
}

#if defined(EIGEN_TEST_PART_6) || defined(EIGEN_TEST_PART_ALL)
void bdcsvd_mixed_option_enum_regression() {
  using NoQrFullSVD = BDCSVD<MatrixXd, NoQRPreconditioner | ComputeFullU | ComputeFullV>;
  using ReversedMixedSVD = BDCSVD<MatrixXd, ComputeThinU | DisableQRDecomposition | ComputeFullV>;

  STATIC_CHECK((int(NoQrFullSVD::QRDecomposition) == int(NoQRPreconditioner)));
  STATIC_CHECK((NoQrFullSVD::ComputationOptions == (ComputeFullU | ComputeFullV)));

  STATIC_CHECK((int(ReversedMixedSVD::QRDecomposition) == int(DisableQRDecomposition)));
  STATIC_CHECK((ReversedMixedSVD::ComputationOptions == (ComputeThinU | ComputeFullV)));
}
#endif

#if defined(EIGEN_TEST_PART_53) || defined(EIGEN_TEST_PART_ALL)
void bdcsvd_extreme_scale_regressions() {
  typedef Matrix<double, 6, 6> Matrix6d;
  const double kTolerance = 16 * Matrix6d::RowsAtCompileTime * NumTraits<double>::epsilon();

  const auto verify_decomposition = [kTolerance](const Matrix6d& matrix) {
    BDCSVD<Matrix6d, ComputeFullU | ComputeFullV> svd;
    svd.setSwitchSize(3);
    svd.compute(matrix);

    VERIFY(svd.info() == Success);

    const Matrix6d reconstruction = svd.matrixU() * svd.singularValues().asDiagonal() * svd.matrixV().transpose();
    VERIFY((reconstruction - matrix).stableNorm() <= kTolerance * matrix.stableNorm());

    const Matrix6d identity = Matrix6d::Identity();
    VERIFY((svd.matrixU().transpose() * svd.matrixU() - identity).stableNorm() <= kTolerance);
    VERIFY((svd.matrixV().transpose() * svd.matrixV() - identity).stableNorm() <= kTolerance);

    // Also exercise the values-only path, which uses a compact m_naiveU and
    // linear workspace during divide-and-conquer merges.
    BDCSVD<Matrix6d> valuesOnlySvd;
    valuesOnlySvd.setSwitchSize(3);
    valuesOnlySvd.compute(matrix);
    VERIFY(valuesOnlySvd.info() == Success);
    VERIFY_IS_APPROX(valuesOnlySvd.singularValues(), svd.singularValues());
  };

  Matrix6d matrix = Matrix6d::Zero();
  const double kSubnormal1040 = std::numeric_limits<double>::denorm_min() * 17179869184.0;  // 2^-1040
  const double kSubnormal1060 = std::numeric_limits<double>::denorm_min() * 16384.0;        // 2^-1060
  const double kSmallestNormal = (std::numeric_limits<double>::min)();                      // 2^-1022
  const double kNormal1000 = kSmallestNormal * 4194304.0;                                   // 2^-1000

  // The merge combines normal and subnormal couplings. Squaring the two
  // coupling terms directly used to underflow, which later left perturbCol0
  // without a predecessor and made the decomposition report NumericalIssue.
  matrix.diagonal() << kSubnormal1040, -kSubnormal1060, kSmallestNormal, 0.5, 1.0, kSmallestNormal;
  matrix.diagonal(1) << -kNormal1000, kNormal1000, kSubnormal1040, kSubnormal1060, -8.0;
  verify_decomposition(matrix);

  // A singular-vector coefficient grows to about 2^570 here. Its squared
  // norm overflows even though the vector has a finite, well-scaled
  // normalization.
  using std::ldexp;
  matrix.setZero();
  matrix.diagonal() << 0.0, 0.0, ldexp(1.0, -487), -1.0, 0.0, 0.0;
  matrix.diagonal(1) << 0.0, ldexp(1.0, -453), -ldexp(1.0, -627), 0.0, 0.0;
  verify_decomposition(matrix);
}
#endif

#if defined(EIGEN_TEST_PART_54) || defined(EIGEN_TEST_PART_ALL)
void bdcsvd_fast_math_regression_1588() {
  const Index n = 500;
  MatrixXd matrix = MatrixXd::Zero(n, n);

  std::srand(1);
  for (Index k = 0; k < 5000; ++k) {
    const Index row = std::rand() % n;
    const Index col = std::rand() % n;
    matrix(row, col) = static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX);
  }
  matrix = matrix * matrix;

  BDCSVD<MatrixXd, ComputeThinU | ComputeThinV> svd(matrix);
  VERIFY(svd.info() == Success);

  MatrixXd reconstruction = svd.matrixU() * svd.singularValues().asDiagonal() * svd.matrixV().transpose();
  const double relative_error = (reconstruction - matrix).norm() / matrix.norm();
  // Deterministic input (fixed seed); the reconstruction is backward stable, so the relative error stays near eps.
  VERIFY(relative_error < 64 * NumTraits<double>::epsilon());
}
#endif

void bdcsvd_power_of_two_scaling() {
  // Reciprocal scaling rounds the smaller singular value down by one ULP in both entry paths.
  const Index size = 20;
  VectorXf diagonal = VectorXf::Zero(size);
  diagonal(0) = numext::bit_cast<float>(numext::uint32_t(0x13afd4a1));
  diagonal(1) = numext::bit_cast<float>(numext::uint32_t(0x11525720));

  MatrixXf matrix = MatrixXf::Zero(size, size + 1);
  matrix.leftCols(size).diagonal() = diagonal;
  BDCSVD<MatrixXf> denseSvd;
  denseSvd.setSwitchSize(8);
  denseSvd.compute(matrix);
  VERIFY_IS_EQUAL(denseSvd.singularValues()(0), diagonal(0));
  VERIFY_IS_EQUAL(denseSvd.singularValues()(1), diagonal(1));

  BDCSVD<MatrixXf> bidiagonalSvd;
  bidiagonalSvd.setSwitchSize(8);
  bidiagonalSvd.compute(diagonal, VectorXf::Zero(size - 1));
  VERIFY_IS_EQUAL(bidiagonalSvd.singularValues()(0), diagonal(0));
  VERIFY_IS_EQUAL(bidiagonalSvd.singularValues()(1), diagonal(1));

  // A subnormal coupling remains significant relative to a uniformly tiny bidiagonal matrix. Scale it without first
  // feeding the subnormal operand to packet arithmetic, which flushes it on ARMv7 NEON.
  const auto checkSubnormalCoupling = [&]() {
    volatile float normalMin = (std::numeric_limits<float>::min)();
    const float largest = 256.0f * normalMin;
    // Construct min_normal / 2 without arithmetic that FTZ would flush before it reaches the solver.
    const float coupling = numext::bit_cast<float>(numext::uint32_t(0x00400000));
    diagonal.setZero();
    diagonal(0) = largest;
    VectorXf superdiagonal = VectorXf::Zero(size - 1);
    superdiagonal(0) = coupling;
    VERIFY_IS_EQUAL(numext::bit_cast<numext::uint32_t>(superdiagonal(0)), numext::uint32_t(0x00400000));
    const float expectedRatio = numext::sqrt(1.0f + 1.0f / (512.0f * 512.0f));
    for (int switchSize : {8, 32}) {
      bidiagonalSvd.setSwitchSize(switchSize);
      bidiagonalSvd.compute(diagonal, superdiagonal);
      VERIFY(numext::abs(bidiagonalSvd.singularValues()(0) / largest - expectedRatio) <=
             2.0f * NumTraits<float>::epsilon());
    }
  };
  checkSubnormalCoupling();
  ScopedFlushToZero ftz;
  checkSubnormalCoupling();
}

EIGEN_DECLARE_TEST(bdcsvd) {
  CALL_SUBTEST_1((bdcsvd_asserts<Matrix3f>()));
  CALL_SUBTEST_2((bdcsvd_asserts<Matrix4d>()));
  CALL_SUBTEST_3((bdcsvd_asserts<Matrix<float, 10, 7>>()));
  CALL_SUBTEST_4((bdcsvd_asserts<Matrix<float, 7, 10>>()));
  CALL_SUBTEST_5((bdcsvd_asserts<Matrix<std::complex<double>, 6, 9>>()));
  CALL_SUBTEST_6((bdcsvd_mixed_option_enum_regression()));

  CALL_SUBTEST_7((bdcsvd_thin_full_options<Matrix2cd>()));
  CALL_SUBTEST_9((bdcsvd_thin_full_options<Matrix2d>()));

  for (int i = 0; i < g_repeat; i++) {
    int r = internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2), c = internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2);

    TEST_SET_BUT_UNUSED_VARIABLE(r);
    TEST_SET_BUT_UNUSED_VARIABLE(c);

    CALL_SUBTEST_11((compare_bdc_jacobi<MatrixXf>(MatrixXf(r, c))));
    CALL_SUBTEST_12((compare_bdc_jacobi<MatrixXd>(MatrixXd(r, c))));
    CALL_SUBTEST_13((compare_bdc_jacobi<MatrixXcd>(MatrixXcd(r, c))));
    // Test on inf/nan matrix
    CALL_SUBTEST_14((svd_inf_nan<MatrixXf>()));
    CALL_SUBTEST_15((svd_inf_nan<MatrixXd>()));

    // Verify some computations using all combinations of the Options template parameter.
    CALL_SUBTEST_16((bdcsvd_thin_full_options<Matrix3f>()));
    CALL_SUBTEST_18((bdcsvd_thin_full_options<Matrix<float, 2, 3>>()));
    CALL_SUBTEST_20((bdcsvd_thin_full_options<MatrixXd>(MatrixXd(20, 17))));
    CALL_SUBTEST_22((bdcsvd_thin_full_options<MatrixXd>(MatrixXd(17, 20))));
    CALL_SUBTEST_24((bdcsvd_thin_full_options<Matrix<double, Dynamic, 15>>(Matrix<double, Dynamic, 15>(r, 15))));
    CALL_SUBTEST_26((bdcsvd_thin_full_options<Matrix<double, 13, Dynamic>>(Matrix<double, 13, Dynamic>(13, c))));
    CALL_SUBTEST_28((bdcsvd_thin_full_options<MatrixXf>(MatrixXf(r, c))));
    CALL_SUBTEST_30((bdcsvd_thin_full_options<MatrixXcd>(MatrixXcd(r, c))));
    CALL_SUBTEST_32((bdcsvd_thin_full_options<MatrixXd>(MatrixXd(r, c))));
    CALL_SUBTEST_34((bdcsvd_thin_full_options<Matrix<double, Dynamic, Dynamic, RowMajor>>(
        Matrix<double, Dynamic, Dynamic, RowMajor>(20, 27))));
    CALL_SUBTEST_36((bdcsvd_thin_full_options<Matrix<double, Dynamic, Dynamic, RowMajor>>(
        Matrix<double, Dynamic, Dynamic, RowMajor>(27, 20))));
    CALL_SUBTEST_38((
        svd_check_max_size_matrix<Matrix<float, Dynamic, Dynamic, ColMajor, 20, 35>, ColPivHouseholderQRPreconditioner>(
            r, c)));
    CALL_SUBTEST_39(
        (svd_check_max_size_matrix<Matrix<float, Dynamic, Dynamic, ColMajor, 35, 20>, HouseholderQRPreconditioner>(r,
                                                                                                                   c)));
    CALL_SUBTEST_40((
        svd_check_max_size_matrix<Matrix<float, Dynamic, Dynamic, RowMajor, 20, 35>, ColPivHouseholderQRPreconditioner>(
            r, c)));
    CALL_SUBTEST_41(
        (svd_check_max_size_matrix<Matrix<float, Dynamic, Dynamic, RowMajor, 35, 20>, HouseholderQRPreconditioner>(r,
                                                                                                                   c)));
  }

  // test matrixbase method
  CALL_SUBTEST_42((bdcsvd_method<Matrix2cd>()));
  CALL_SUBTEST_43((bdcsvd_method<Matrix3f>()));

  // Test problem size constructors
  CALL_SUBTEST_44(BDCSVD<MatrixXf>(10, 10));

  // Check that preallocation avoids subsequent mallocs
  // Disabled because not supported by BDCSVD
  // CALL_SUBTEST_9( svd_preallocate<void>() );

  CALL_SUBTEST_45(svd_underoverflow<void>());
  CALL_SUBTEST_45(bdcsvd_power_of_two_scaling());

  // Without total deflation issues.
  CALL_SUBTEST_46((compare_bdc_jacobi_instance(true)));
  CALL_SUBTEST_47((compare_bdc_jacobi_instance(false)));

  // With total deflation issues before, when it shouldn't be triggered.
  CALL_SUBTEST_48((compare_bdc_jacobi_instance(true, 3)));
  CALL_SUBTEST_49((compare_bdc_jacobi_instance(false, 3)));

  // Convergence for large constant matrix (https://gitlab.com/libeigen/eigen/-/issues/2491)
  CALL_SUBTEST_50(bdcsvd_check_convergence<MatrixXf>(MatrixXf::Constant(500, 500, 1)));

  // Bidiagonal SVD hard test cases
  CALL_SUBTEST_51((bdcsvd_bidiagonal_hard_cases<float>()));
  CALL_SUBTEST_52((bdcsvd_bidiagonal_hard_cases<double>()));
  CALL_SUBTEST_53((bdcsvd_extreme_scale_regressions()));
  CALL_SUBTEST_54((bdcsvd_fast_math_regression_1588()));
}
