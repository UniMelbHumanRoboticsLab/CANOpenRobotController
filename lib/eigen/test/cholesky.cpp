// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#define TEST_ENABLE_TEMPORARY_TRACKING
#define EIGEN_RUNTIME_NO_MALLOC

#include "main.h"
#include <Eigen/Cholesky>
#include <Eigen/QR>
#include "solverbase.h"

template <typename MatrixType, int UpLo>
typename MatrixType::RealScalar matrix_l1_norm(const MatrixType& m) {
  if (m.cols() == 0) return typename MatrixType::RealScalar(0);
  MatrixType symm = m.template selfadjointView<UpLo>();
  return symm.cwiseAbs().colwise().sum().maxCoeff();
}

template <typename MatrixType, template <typename, int> class CholType>
void test_chol_update(const MatrixType& symm) {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename MatrixType::RealScalar RealScalar;
  typedef Matrix<Scalar, MatrixType::RowsAtCompileTime, 1> VectorType;

  MatrixType symmLo = symm.template triangularView<Lower>();
  MatrixType symmUp = symm.template triangularView<Upper>();
  MatrixType symmCpy = symm;

  CholType<MatrixType, Lower> chollo(symmLo);
  CholType<MatrixType, Upper> cholup(symmUp);

  for (int k = 0; k < 10; ++k) {
    VectorType vec = VectorType::Random(symm.rows());
    RealScalar sigma = internal::random<RealScalar>();
    symmCpy += sigma * vec * vec.adjoint();

    // we are doing some downdates, so it might be the case that the matrix is not SPD anymore
    CholType<MatrixType, Lower> chol(symmCpy);
    if (chol.info() != Success) break;

    chollo.rankUpdate(vec, sigma);
    VERIFY_IS_APPROX(symmCpy, chollo.reconstructedMatrix());

    cholup.rankUpdate(vec, sigma);
    VERIFY_IS_APPROX(symmCpy, cholup.reconstructedMatrix());
  }
}

// A^-1 from an LLT factorization. The kernels fill one triangle and mirror it, so the result is
// exactly self-adjoint, and accuracy is checked as the backward error of the corresponding solve.
template <typename MatrixType, int UpLo>
void check_llt_inverse(const MatrixType& symm) {
  using Scalar = typename MatrixType::Scalar;
  using RealScalar = typename NumTraits<Scalar>::Real;
  using DynMatrixType = Matrix<Scalar, Dynamic, Dynamic>;
  using DynVectorType = Matrix<Scalar, Dynamic, 1>;
  const Index n = symm.rows();

  const MatrixType tri = symm.template triangularView<UpLo>();
  LLT<MatrixType, UpLo> llt(tri);
  VERIFY(llt.info() == Success);

  const MatrixType inv = llt.inverse();
  VERIFY_IS_EQUAL(inv.rows(), n);
  VERIFY_IS_EQUAL(inv.cols(), n);
  VERIFY_IS_CWISE_EQUAL(inv, inv.adjoint());

  // |A X - I| <= c*n*eps*|A|*|X| for a Cholesky solve (Higham, Accuracy and Stability of Numerical
  // Algorithms, 2nd ed., Thm 10.4). The factor 16 absorbs c and the entrywise-to-norm step; the
  // largest ratio measured over this family is 2, at n = 1.
  const RealScalar eps = NumTraits<RealScalar>::epsilon();
  const RealScalar residual_bound = RealScalar(16 * n) * eps * symm.norm() * inv.norm();
  VERIFY((numext::isfinite)(residual_bound));
  VERIFY((symm * inv - MatrixType::Identity(n, n)).norm() <= residual_bound);

  // The same answer as solving against an explicit identity, to within the forward error, which the
  // residual bound multiplied by cond(A) <= |A|*|X| bounds in turn.
  const MatrixType reference = llt.solve(MatrixType::Identity(n, n));
  const RealScalar forward_bound = residual_bound * inv.norm();
  VERIFY((numext::isfinite)(forward_bound));
  VERIFY((inv - reference).norm() <= forward_bound);

  // The destination need not be a plain object.
  DynMatrixType host = DynMatrixType::Random(n + 2, n + 2);
  host.bottomRightCorner(n, n) = llt.inverse();
  VERIFY((symm * host.bottomRightCorner(n, n) - MatrixType::Identity(n, n)).norm() <= residual_bound);

  // Nor need its inner stride be known at compile time. extract_data() is null for such a destination,
  // which leaves the alias with the factor unknown rather than excluded, so this takes POTRI at every
  // size, and the gaps between the mapped coefficients hold the kernels to the storage they were given.
  using StrideType = Stride<Dynamic, Dynamic>;
  const Index inner = 2, outer = 2 * n + 3;
  const Index buffer_size = numext::maxi(Index(1), (n - 1) * (outer + inner) + 1);
  DynVectorType buffer = DynVectorType::Random(buffer_size);
  const DynVectorType before = buffer;
  Map<MatrixType, 0, StrideType> strided(buffer.data(), n, n, StrideType(outer, inner));
  strided = llt.inverse();
  VERIFY_IS_CWISE_EQUAL(strided, strided.adjoint());
  VERIFY((symm * strided - MatrixType::Identity(n, n)).norm() <= residual_bound);
  std::vector<bool> mapped(buffer_size, false);
  for (Index j = 0; j < n; ++j)
    for (Index i = 0; i < n; ++i) mapped[static_cast<std::size_t>(&strided.coeffRef(i, j) - buffer.data())] = true;
  for (Index k = 0; k < buffer_size; ++k)
    if (!mapped[k]) VERIFY_IS_EQUAL(buffer[k], before[k]);

  // An in-place decomposition may overwrite its own factor, whichever arm the size selects: the solve
  // fallback would read the factor after setIdentity() had destroyed it, so the alias takes POTRI.
  MatrixType storage = tri;
  LLT<Ref<MatrixType>, UpLo> inplace(storage);
  VERIFY(inplace.info() == Success);
  storage = inplace.inverse();
  VERIFY_IS_CWISE_EQUAL(storage, storage.adjoint());
  VERIFY((symm * storage - MatrixType::Identity(n, n)).norm() <= residual_bound);
}

// LLT::inverse() dispatches on EIGEN_LLT_INVERSE_POTRI_THRESHOLD, so straddle it deterministically:
// the random sizes above reach only whichever side of it EIGEN_TEST_MAX_SIZE happens to allow. Only
// real scalars are thresholded, so only they have a boundary to straddle.
template <typename Scalar>
void llt_inverse_threshold_boundary() {
  static_assert(!NumTraits<Scalar>::IsComplex, "only real scalars are thresholded");
  using RealScalar = typename NumTraits<Scalar>::Real;
  using MatrixType = Matrix<Scalar, Dynamic, Dynamic>;

  const Index threshold = EIGEN_LLT_INVERSE_POTRI_THRESHOLD;
  const Index sizes[] = {threshold - 1, threshold, threshold + 1};
  for (Index n : sizes) {
    MatrixType r = MatrixType::Random(n, n);
    MatrixType symm = r * r.adjoint();
    symm.diagonal().array() += RealScalar(n);

    check_llt_inverse<MatrixType, Lower>(symm);
    check_llt_inverse<MatrixType, Upper>(symm);
  }
}

template <typename MatrixType>
void cholesky(const MatrixType& m) {
  /* this test covers the following files:
     LLT.h LDLT.h
  */
  Index rows = m.rows();
  Index cols = m.cols();

  typedef typename MatrixType::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, MatrixType::RowsAtCompileTime, MatrixType::RowsAtCompileTime> SquareMatrixType;
  typedef Matrix<Scalar, MatrixType::RowsAtCompileTime, 1> VectorType;

  MatrixType a0 = MatrixType::Random(rows, cols);
  VectorType vecB = VectorType::Random(rows), vecX(rows);
  MatrixType matB = MatrixType::Random(rows, cols), matX(rows, cols);
  SquareMatrixType symm = a0 * a0.adjoint();
  // let's make sure the matrix is not singular or near singular
  for (int k = 0; k < 3; ++k) {
    MatrixType a1 = MatrixType::Random(rows, cols);
    symm += a1 * a1.adjoint();
  }

  {
    STATIC_CHECK((std::is_same<typename LLT<MatrixType, Lower>::StorageIndex, int>::value));
    STATIC_CHECK((std::is_same<typename LLT<MatrixType, Upper>::StorageIndex, int>::value));

    SquareMatrixType symmUp = symm.template triangularView<Upper>();
    SquareMatrixType symmLo = symm.template triangularView<Lower>();

    LLT<SquareMatrixType, Lower> chollo(symmLo);
    VERIFY_IS_APPROX(symm, chollo.reconstructedMatrix());

    check_solverbase<VectorType, VectorType>(symm, chollo, rows, rows, 1);
    check_solverbase<MatrixType, MatrixType>(symm, chollo, rows, cols, rows);

    const MatrixType symmLo_inverse = chollo.solve(MatrixType::Identity(rows, cols));
    RealScalar rcond =
        (RealScalar(1) / matrix_l1_norm<MatrixType, Lower>(symmLo)) / matrix_l1_norm<MatrixType, Lower>(symmLo_inverse);
    RealScalar rcond_est = chollo.rcond();
    // Verify that the estimated condition number is within a factor of 10 of the
    // truth.
    VERIFY(rcond_est >= rcond / 10 && rcond_est <= rcond * 10);

    // test the upper mode
    LLT<SquareMatrixType, Upper> cholup(symmUp);
    VERIFY_IS_APPROX(symm, cholup.reconstructedMatrix());
    vecX = cholup.solve(vecB);
    VERIFY_IS_APPROX(symm * vecX, vecB);
    matX = cholup.solve(matB);
    VERIFY_IS_APPROX(symm * matX, matB);

    // Verify that the estimated condition number is within a factor of 10 of the
    // truth.
    const MatrixType symmUp_inverse = cholup.solve(MatrixType::Identity(rows, cols));
    rcond =
        (RealScalar(1) / matrix_l1_norm<MatrixType, Upper>(symmUp)) / matrix_l1_norm<MatrixType, Upper>(symmUp_inverse);
    rcond_est = cholup.rcond();
    VERIFY(rcond_est >= rcond / 10 && rcond_est <= rcond * 10);

    check_llt_inverse<SquareMatrixType, Lower>(symm);
    check_llt_inverse<SquareMatrixType, Upper>(symm);

    MatrixType neg = -symmLo;
    chollo.compute(neg);
    VERIFY(neg.size() == 0 || chollo.info() == NumericalIssue);

    VERIFY_IS_APPROX(MatrixType(chollo.matrixL().transpose().conjugate()), MatrixType(chollo.matrixU()));
    VERIFY_IS_APPROX(MatrixType(chollo.matrixU().transpose().conjugate()), MatrixType(chollo.matrixL()));
    VERIFY_IS_APPROX(MatrixType(cholup.matrixL().transpose().conjugate()), MatrixType(cholup.matrixU()));
    VERIFY_IS_APPROX(MatrixType(cholup.matrixU().transpose().conjugate()), MatrixType(cholup.matrixL()));

    // test some special use cases of SelfCwiseBinaryOp:
    MatrixType m1 = MatrixType::Random(rows, cols), m2(rows, cols);
    m2 = m1;
    m2 += symmLo.template selfadjointView<Lower>().llt().solve(matB);
    VERIFY_IS_APPROX(m2, m1 + symmLo.template selfadjointView<Lower>().llt().solve(matB));
    m2 = m1;
    m2 -= symmLo.template selfadjointView<Lower>().llt().solve(matB);
    VERIFY_IS_APPROX(m2, m1 - symmLo.template selfadjointView<Lower>().llt().solve(matB));
    m2 = m1;
    m2.noalias() += symmLo.template selfadjointView<Lower>().llt().solve(matB);
    VERIFY_IS_APPROX(m2, m1 + symmLo.template selfadjointView<Lower>().llt().solve(matB));
    m2 = m1;
    m2.noalias() -= symmLo.template selfadjointView<Lower>().llt().solve(matB);
    VERIFY_IS_APPROX(m2, m1 - symmLo.template selfadjointView<Lower>().llt().solve(matB));
  }

  // LDLT
  {
    STATIC_CHECK((std::is_same<typename LDLT<MatrixType, Lower>::StorageIndex, int>::value));
    STATIC_CHECK((std::is_same<typename LDLT<MatrixType, Upper>::StorageIndex, int>::value));

    int sign = internal::random<int>() % 2 ? 1 : -1;

    if (sign == -1) {
      symm = -symm;  // test a negative matrix
    }

    SquareMatrixType symmUp = symm.template triangularView<Upper>();
    SquareMatrixType symmLo = symm.template triangularView<Lower>();

    LDLT<SquareMatrixType, Lower> ldltlo(symmLo);
    VERIFY(ldltlo.info() == Success);
    VERIFY_IS_APPROX(symm, ldltlo.reconstructedMatrix());

    check_solverbase<VectorType, VectorType>(symm, ldltlo, rows, rows, 1);
    check_solverbase<MatrixType, MatrixType>(symm, ldltlo, rows, cols, rows);

    const MatrixType symmLo_inverse = ldltlo.solve(MatrixType::Identity(rows, cols));
    RealScalar rcond =
        (RealScalar(1) / matrix_l1_norm<MatrixType, Lower>(symmLo)) / matrix_l1_norm<MatrixType, Lower>(symmLo_inverse);
    RealScalar rcond_est = ldltlo.rcond();
    // Verify that the estimated condition number is within a factor of 10 of the
    // truth.
    VERIFY(rcond_est >= rcond / 10 && rcond_est <= rcond * 10);

    LDLT<SquareMatrixType, Upper> ldltup(symmUp);
    VERIFY(ldltup.info() == Success);
    VERIFY_IS_APPROX(symm, ldltup.reconstructedMatrix());
    vecX = ldltup.solve(vecB);
    VERIFY_IS_APPROX(symm * vecX, vecB);
    matX = ldltup.solve(matB);
    VERIFY_IS_APPROX(symm * matX, matB);

    // Verify that the estimated condition number is within a factor of 10 of the
    // truth.
    const MatrixType symmUp_inverse = ldltup.solve(MatrixType::Identity(rows, cols));
    rcond =
        (RealScalar(1) / matrix_l1_norm<MatrixType, Upper>(symmUp)) / matrix_l1_norm<MatrixType, Upper>(symmUp_inverse);
    rcond_est = ldltup.rcond();
    VERIFY(rcond_est >= rcond / 10 && rcond_est <= rcond * 10);

    VERIFY_IS_APPROX(MatrixType(ldltlo.matrixL().transpose().conjugate()), MatrixType(ldltlo.matrixU()));
    VERIFY_IS_APPROX(MatrixType(ldltlo.matrixU().transpose().conjugate()), MatrixType(ldltlo.matrixL()));
    VERIFY_IS_APPROX(MatrixType(ldltup.matrixL().transpose().conjugate()), MatrixType(ldltup.matrixU()));
    VERIFY_IS_APPROX(MatrixType(ldltup.matrixU().transpose().conjugate()), MatrixType(ldltup.matrixL()));

    if (MatrixType::RowsAtCompileTime == Dynamic) {
      // note : each inplace permutation requires a small temporary vector (mask)

      // check inplace solve
      matX = matB;
      VERIFY_EVALUATION_COUNT(matX = ldltlo.solve(matX), 0);
      VERIFY_IS_APPROX(matX, ldltlo.solve(matB).eval());

      matX = matB;
      VERIFY_EVALUATION_COUNT(matX = ldltup.solve(matX), 0);
      VERIFY_IS_APPROX(matX, ldltup.solve(matB).eval());
    }

    // restore
    if (sign == -1) symm = -symm;

    // check matrices coming from linear constraints with Lagrange multipliers
    if (rows >= 3) {
      SquareMatrixType A = symm;
      Index c = internal::random<Index>(0, rows - 2);
      A.bottomRightCorner(c, c).setZero();
      // Make sure a solution exists:
      vecX.setRandom();
      vecB = A * vecX;
      vecX.setZero();
      ldltlo.compute(A);
      VERIFY_IS_APPROX(A, ldltlo.reconstructedMatrix());
      vecX = ldltlo.solve(vecB);
      VERIFY_IS_APPROX(A * vecX, vecB);
    }

    // check non-full rank matrices
    if (rows >= 3) {
      Index r = internal::random<Index>(1, rows - 1);
      Matrix<Scalar, Dynamic, Dynamic> a = Matrix<Scalar, Dynamic, Dynamic>::Random(rows, r);
      SquareMatrixType A = a * a.adjoint();
      // Make sure a solution exists:
      vecX.setRandom();
      vecB = A * vecX;
      vecX.setZero();
      ldltlo.compute(A);
      VERIFY_IS_APPROX(A, ldltlo.reconstructedMatrix());
      vecX = ldltlo.solve(vecB);
      VERIFY_IS_APPROX(A * vecX, vecB);
    }

    // check matrices with a wide spectrum
    if (rows >= 3) {
      using std::pow;
      using std::sqrt;
      RealScalar s = (std::min)(16, std::numeric_limits<RealScalar>::max_exponent10 / 8);
      Matrix<Scalar, Dynamic, Dynamic> a = Matrix<Scalar, Dynamic, Dynamic>::Random(rows, rows);
      Matrix<RealScalar, Dynamic, 1> d = Matrix<RealScalar, Dynamic, 1>::Random(rows);
      for (Index k = 0; k < rows; ++k) d(k) = d(k) * pow(RealScalar(10), internal::random<RealScalar>(-s, s));
      SquareMatrixType A = a * d.asDiagonal() * a.adjoint();
      // Make sure a solution exists:
      vecX.setRandom();
      vecB = A * vecX;
      vecX.setZero();
      ldltlo.compute(A);
      VERIFY_IS_APPROX(A, ldltlo.reconstructedMatrix());
      vecX = ldltlo.solve(vecB);

      if (ldltlo.vectorD().real().cwiseAbs().minCoeff() > RealScalar(0)) {
        VERIFY_IS_APPROX(A * vecX, vecB);
      } else {
        RealScalar large_tol = sqrt(test_precision<RealScalar>());
        VERIFY((A * vecX).isApprox(vecB, large_tol));

        ++g_test_level;
        VERIFY_IS_APPROX(A * vecX, vecB);
        --g_test_level;
      }
    }
  }

  // update/downdate
  CALL_SUBTEST((test_chol_update<SquareMatrixType, LLT>(symm)));
  CALL_SUBTEST((test_chol_update<SquareMatrixType, LDLT>(symm)));
}

template <typename MatrixType>
void cholesky_cplx(const MatrixType& m) {
  // classic test
  cholesky(m);

  // test mixing real/scalar types

  Index rows = m.rows();
  Index cols = m.cols();

  typedef typename MatrixType::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<RealScalar, MatrixType::RowsAtCompileTime, MatrixType::RowsAtCompileTime> RealMatrixType;
  typedef Matrix<Scalar, MatrixType::RowsAtCompileTime, 1> VectorType;

  RealMatrixType a0 = RealMatrixType::Random(rows, cols);
  VectorType vecB = VectorType::Random(rows), vecX(rows);
  MatrixType matB = MatrixType::Random(rows, cols), matX(rows, cols);
  RealMatrixType symm = a0 * a0.adjoint();
  // let's make sure the matrix is not singular or near singular
  for (int k = 0; k < 3; ++k) {
    RealMatrixType a1 = RealMatrixType::Random(rows, cols);
    symm += a1 * a1.adjoint();
  }

  {
    RealMatrixType symmLo = symm.template triangularView<Lower>();

    LLT<RealMatrixType, Lower> chollo(symmLo);
    VERIFY_IS_APPROX(symm, chollo.reconstructedMatrix());

    check_solverbase<VectorType, VectorType>(symm, chollo, rows, rows, 1);
    // check_solverbase<MatrixType, MatrixType>(symm, chollo, rows, cols, rows);
  }

  // LDLT
  {
    int sign = internal::random<int>() % 2 ? 1 : -1;

    if (sign == -1) {
      symm = -symm;  // test a negative matrix
    }

    RealMatrixType symmLo = symm.template triangularView<Lower>();

    LDLT<RealMatrixType, Lower> ldltlo(symmLo);
    VERIFY(ldltlo.info() == Success);
    VERIFY_IS_APPROX(symm, ldltlo.reconstructedMatrix());

    check_solverbase<VectorType, VectorType>(symm, ldltlo, rows, rows, 1);
    // check_solverbase<MatrixType, MatrixType>(symm, ldltlo, rows, cols, rows);
  }
}

// regression test for bug 241
template <typename MatrixType>
void cholesky_bug241(const MatrixType& m) {
  eigen_assert(m.rows() == 2 && m.cols() == 2);

  typedef typename MatrixType::Scalar Scalar;
  typedef Matrix<Scalar, MatrixType::RowsAtCompileTime, 1> VectorType;

  MatrixType matA;
  matA << 1, 1, 1, 1;
  VectorType vecB;
  vecB << 1, 1;
  VectorType vecX = matA.ldlt().solve(vecB);
  VERIFY_IS_APPROX(matA * vecX, vecB);
}

// LDLT is not guaranteed to work for indefinite matrices, but happens to work fine if matrix is diagonal.
// This test checks that LDLT reports correctly that matrix is indefinite.
// See http://forum.kde.org/viewtopic.php?f=74&t=106942 and bug 736
template <typename MatrixType>
void cholesky_definiteness(const MatrixType& m) {
  eigen_assert(m.rows() == 2 && m.cols() == 2);
  MatrixType mat;
  LDLT<MatrixType> ldlt(2);

  {
    mat << 1, 0, 0, -1;
    ldlt.compute(mat);
    VERIFY(ldlt.info() == Success);
    VERIFY(!ldlt.isNegative());
    VERIFY(!ldlt.isPositive());
    VERIFY_IS_APPROX(mat, ldlt.reconstructedMatrix());
  }
  {
    mat << 1, 2, 2, 1;
    ldlt.compute(mat);
    VERIFY(ldlt.info() == Success);
    VERIFY(!ldlt.isNegative());
    VERIFY(!ldlt.isPositive());
    VERIFY_IS_APPROX(mat, ldlt.reconstructedMatrix());
  }
  {
    mat << 0, 0, 0, 0;
    ldlt.compute(mat);
    VERIFY(ldlt.info() == Success);
    VERIFY(ldlt.isNegative());
    VERIFY(ldlt.isPositive());
    VERIFY_IS_APPROX(mat, ldlt.reconstructedMatrix());
  }
  {
    mat << 0, 0, 0, 1;
    ldlt.compute(mat);
    VERIFY(ldlt.info() == Success);
    VERIFY(!ldlt.isNegative());
    VERIFY(ldlt.isPositive());
    VERIFY_IS_APPROX(mat, ldlt.reconstructedMatrix());
  }
  {
    mat << -1, 0, 0, 0;
    ldlt.compute(mat);
    VERIFY(ldlt.info() == Success);
    VERIFY(ldlt.isNegative());
    VERIFY(!ldlt.isPositive());
    VERIFY_IS_APPROX(mat, ldlt.reconstructedMatrix());
  }
}

template <typename>
void cholesky_faillure_cases() {
  MatrixXd mat;
  LDLT<MatrixXd> ldlt;

  {
    mat.resize(2, 2);
    mat << 0, 1, 1, 0;
    ldlt.compute(mat);
    VERIFY_IS_NOT_APPROX(mat, ldlt.reconstructedMatrix());
    VERIFY(ldlt.info() == NumericalIssue);
  }
#if (!EIGEN_ARCH_i386) || defined(EIGEN_VECTORIZE_SSE2)
  {
    mat.resize(3, 3);
    mat << -1, -3, 3, -3, -8.9999999999999999999, 1, 3, 1, 0;
    ldlt.compute(mat);
    VERIFY(ldlt.info() == NumericalIssue);
    VERIFY_IS_NOT_APPROX(mat, ldlt.reconstructedMatrix());
  }
#endif
  {
    mat.resize(3, 3);
    mat << 1, 2, 3, 2, 4, 1, 3, 1, 0;
    ldlt.compute(mat);
    VERIFY(ldlt.info() == NumericalIssue);
    VERIFY_IS_NOT_APPROX(mat, ldlt.reconstructedMatrix());
  }

  {
    mat.resize(8, 8);
    mat << 0.1, 0, -0.1, 0, 0, 0, 1, 0, 0, 4.24667, 0, 2.00333, 0, 0, 0, 0, -0.1, 0, 0.2, 0, -0.1, 0, 0, 0, 0, 2.00333,
        0, 8.49333, 0, 2.00333, 0, 0, 0, 0, -0.1, 0, 0.1, 0, 0, 1, 0, 0, 0, 2.00333, 0, 4.24667, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    ldlt.compute(mat);
    VERIFY(ldlt.info() == NumericalIssue);
    VERIFY_IS_NOT_APPROX(mat, ldlt.reconstructedMatrix());
  }

  // bug 1479
  {
    mat.resize(4, 4);
    mat << 1, 2, 0, 1, 2, 4, 0, 2, 0, 0, 0, 1, 1, 2, 1, 1;
    ldlt.compute(mat);
    VERIFY(ldlt.info() == NumericalIssue);
    VERIFY_IS_NOT_APPROX(mat, ldlt.reconstructedMatrix());
  }
}

// Accumulates I + sum_k w_k w_k^* out of rank-1 updates alone, which is the path rankUpdate() takes while the
// decomposition holds no factorization, and checks the reported status against the factorization it produced.
template <typename MatrixType, int UpLo>
void check_ldlt_rankupdate_from_scratch(LDLT<MatrixType, UpLo>& ldlt, Index size) {
  using Scalar = typename MatrixType::Scalar;
  using RealScalar = typename NumTraits<Scalar>::Real;
  using VectorType = Matrix<Scalar, MatrixType::RowsAtCompileTime, 1>;

  // Build up the identity first so that the accumulated matrix is well conditioned whatever the random terms are.
  MatrixType ref = MatrixType::Identity(size, size);
  for (Index k = 0; k < size; ++k) ldlt.rankUpdate(VectorType::Unit(size, k), RealScalar(1));

  for (int k = 0; k < 3; ++k) {
    VectorType w = VectorType::Random(size);
    ldlt.rankUpdate(w, RealScalar(1));
    ref += w * w.adjoint();
  }

  VERIFY(ldlt.info() == Success);
  // Success has to be earned: check that the factorization really is one, so that reporting Success unconditionally
  // would not pass either.
  VERIFY_IS_APPROX(ref, ldlt.reconstructedMatrix());
  VectorType vecB = VectorType::Random(size);
  VectorType vecX = ldlt.solve(vecB);
  VERIFY_IS_APPROX(ref * vecX, vecB);
}

// LDLT::rankUpdate() takes one of two paths: it updates the factorization the decomposition already holds, or, when
// there is none, it builds one from scratch. The from-scratch path left the status untouched, so info() reported
// whatever was left over -- InvalidInput from construction, or a failure from an earlier compute() -- for a
// factorization that was in fact sound. Updating an existing factorization keeps a failure already reported for it.
template <typename MatrixType>
void cholesky_ldlt_rankupdate_info(const MatrixType& m) {
  using Scalar = typename MatrixType::Scalar;
  using RealScalar = typename MatrixType::RealScalar;
  using VectorType = Matrix<Scalar, MatrixType::RowsAtCompileTime, 1>;

  const Index size = m.rows();
  eigen_assert(size >= 2);

  // A zero diagonal with non-zero off-diagonal entries makes the first pivot invalid while the matrix is not: this
  // is the 2x2 failure case of cholesky_faillure_cases() at size `size`.
  const MatrixType indefinite = MatrixType::Ones(size, size) - MatrixType::Identity(size, size);

  // The decomposition has never factored anything, so its status is the constructor's InvalidInput.
  {
    LDLT<MatrixType, Lower> ldltlo(size);
    check_ldlt_rankupdate_from_scratch(ldltlo, size);

    LDLT<MatrixType, Upper> ldltup(size);
    check_ldlt_rankupdate_from_scratch(ldltup, size);
  }

  // The decomposition is reused after a compute() that failed, and setZero() discards that factorization, so the
  // stale failure must not be reported for the one the rank updates build.
  {
    LDLT<MatrixType, Lower> ldltlo(indefinite);
    VERIFY(ldltlo.info() == NumericalIssue);
    ldltlo.setZero();
    check_ldlt_rankupdate_from_scratch(ldltlo, size);

    LDLT<MatrixType, Upper> ldltup(indefinite);
    VERIFY(ldltup.info() == NumericalIssue);
    ldltup.setZero();
    check_ldlt_rankupdate_from_scratch(ldltup, size);
  }

  // Without setZero() the update applies to the failed factorization, whose status stands.
  {
    VectorType vec = VectorType::Random(size);

    LDLT<MatrixType, Lower> ldltlo(indefinite);
    VERIFY(ldltlo.info() == NumericalIssue);
    ldltlo.rankUpdate(vec, RealScalar(1));
    VERIFY(ldltlo.info() == NumericalIssue);

    LDLT<MatrixType, Upper> ldltup(indefinite);
    VERIFY(ldltup.info() == NumericalIssue);
    ldltup.rankUpdate(vec, RealScalar(1));
    VERIFY(ldltup.info() == NumericalIssue);
  }
}

// Applies a single rank update that has to succeed, and checks the factorization it left behind rather than the
// reported status alone, so that reporting Success unconditionally would not pass either.
template <typename MatrixType, int UpLo, typename VectorType>
void check_ldlt_rankupdate(LDLT<MatrixType, UpLo>& ldlt, const VectorType& w,
                           const typename MatrixType::RealScalar& sigma, const MatrixType& ref) {
  ldlt.rankUpdate(w, sigma);
  VERIFY(ldlt.info() == Success);
  VERIFY_IS_APPROX(ref, ldlt.reconstructedMatrix());
}

// A component of w that is exactly zero contributes nothing, but the update used to compute that contribution as a
// division whose denominator can be zero at the same position: the pivot is zero all along the from-scratch path, and
// the running alpha reaches zero when a downdate cancels the matrix exactly. The resulting 0/0 either landed straight
// in the factorization or reached the low-rank termination, which read it as a signal to abandon the rest of the
// update. The update has to survive a zero in any position, including the first.
template <typename MatrixType>
void cholesky_ldlt_rankupdate_zero_components(const MatrixType& m) {
  using Scalar = typename MatrixType::Scalar;
  using RealScalar = typename NumTraits<Scalar>::Real;
  using VectorType = Matrix<Scalar, MatrixType::RowsAtCompileTime, 1>;

  const Index size = m.rows();
  const MatrixType identity = MatrixType::Identity(size, size);

  for (Index zero_at = 0; zero_at < size; ++zero_at) {
    VectorType w = VectorType::Random(size);
    w(zero_at) = Scalar(0);
    const MatrixType ref = w * w.adjoint();

    LDLT<MatrixType, Lower> ldltlo(size);
    check_ldlt_rankupdate(ldltlo, w, RealScalar(1), ref);

    LDLT<MatrixType, Upper> ldltup(size);
    check_ldlt_rankupdate(ldltup, w, RealScalar(1), ref);
  }

  // Consecutive zeros have to be skipped one after another, not collapse the update at the first of them.
  {
    VectorType w = VectorType::Random(size);
    w.head(size / 2).setZero();
    const MatrixType ref = w * w.adjoint();

    LDLT<MatrixType, Lower> ldltlo(size);
    check_ldlt_rankupdate(ldltlo, w, RealScalar(1), ref);

    LDLT<MatrixType, Upper> ldltup(size);
    check_ldlt_rankupdate(ldltup, w, RealScalar(1), ref);
  }

  // The hazard is not confined to the from-scratch path: compute() accepts a rank-deficient matrix, and a zero of w
  // meeting one of its zero pivots is the same 0/0. An existing factorization whose pivots are all non-zero, such as
  // the identity, cannot reach it -- the zero of w always meets a non-zero denominator there.
  if (size >= 3) {
    MatrixType rank_deficient = MatrixType::Zero(size, size);
    rank_deficient(0, 0) = Scalar(1);

    // Leave a non-zero component after the zeroed one, whose contribution the 0/0 used to discard.
    for (Index zero_at = 1; zero_at + 1 < size; ++zero_at) {
      VectorType w = VectorType::Random(size);
      w(zero_at) = Scalar(0);
      const MatrixType ref = rank_deficient + w * w.adjoint();

      LDLT<MatrixType, Lower> ldlt(rank_deficient);
      VERIFY(ldlt.info() == Success);
      check_ldlt_rankupdate(ldlt, w, RealScalar(1), ref);
    }
  }

  // Downdating the identity by one of its own basis directions drives alpha to exactly zero, so every later position
  // -- where w is zero -- met a zero denominator on an ordinary, fully initialized factorization. The factorization
  // came back full of NaN, and info() still reported Success.
  for (Index k = 0; k < size; ++k) {
    const VectorType w = VectorType::Unit(size, k);
    const MatrixType ref = identity - w * w.adjoint();

    LDLT<MatrixType, Lower> ldltlo(identity);
    check_ldlt_rankupdate(ldltlo, w, RealScalar(-1), ref);

    LDLT<MatrixType, Upper> ldltup(identity);
    check_ldlt_rankupdate(ldltup, w, RealScalar(-1), ref);
  }
}

// A = Q D Q^*, with Q unitary and D real, is Hermitian with det(A) = prod(D_ii).
// Drawing the |D_ii| from an annulus keeps A well conditioned and log|det(A)| away from zero.
template <typename MatrixType>
void cholesky_determinant(Index size) {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<RealScalar, Dynamic, 1> RealVectorType;

  MatrixType q = MatrixType::Random(size, size).householderQr().householderQ();
  RealVectorType d(size);
  for (Index i = 0; i < size; ++i) d(i) = internal::random<RealScalar>(RealScalar(1.2), RealScalar(2.8));

  const RealScalar absdet = d.prod();
  const RealScalar logabsdet = d.array().log().sum();
  MatrixType spd = q * d.template cast<Scalar>().asDiagonal() * q.adjoint();

  LLT<MatrixType, Lower> lltlo(spd);
  VERIFY(lltlo.info() == Success);
  check_determinant(lltlo, Scalar(absdet), logabsdet);

  LLT<MatrixType, Upper> lltup(spd);
  VERIFY(lltup.info() == Success);
  check_determinant(lltup, Scalar(absdet), logabsdet);

  // Negating D leaves |det(A)| alone and makes A negative definite, so det(A) picks up a factor (-1)^n.
  // LDLT's D is purely diagonal and so does not cover the indefinite case; BunchKaufman does, and
  // test/bunchkaufman.cpp checks the same identities there.
  d = -d;
  const RealScalar det = d.prod();
  const MatrixType negdef = q * d.template cast<Scalar>().asDiagonal() * q.adjoint();

  LDLT<MatrixType, Lower> ldltlo(negdef);
  VERIFY(ldltlo.info() == Success);
  check_determinant(ldltlo, Scalar(det), logabsdet);

  LDLT<MatrixType, Upper> ldltup(negdef);
  VERIFY(ldltup.info() == Success);
  check_determinant(ldltup, Scalar(det), logabsdet);
}

// The determinant of an empty matrix is the empty product, 1.
template <typename MatrixType>
void cholesky_determinant_empty() {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;

  MatrixType empty(0, 0);

  LLT<MatrixType> llt(empty);
  VERIFY_IS_EQUAL(llt.determinant(), Scalar(1));
  VERIFY_IS_EQUAL(llt.absDeterminant(), RealScalar(1));
  VERIFY_IS_EQUAL(llt.logAbsDeterminant(), RealScalar(0));
  VERIFY_IS_EQUAL(llt.signDeterminant(), Scalar(1));

  LDLT<MatrixType> ldlt(empty);
  VERIFY_IS_EQUAL(ldlt.determinant(), Scalar(1));
  VERIFY_IS_EQUAL(ldlt.absDeterminant(), RealScalar(1));
  VERIFY_IS_EQUAL(ldlt.logAbsDeterminant(), RealScalar(0));
  VERIFY_IS_EQUAL(ldlt.signDeterminant(), Scalar(1));
}

// logAbsDeterminant() exists to survive the range where the determinant itself does not: with n = 200 and
// a diagonal of 10^4, det = 10^800 overflows every supported float type while log|det| = 800 log 10 does not.
template <typename MatrixType>
void cholesky_determinant_overflow() {
  typedef typename MatrixType::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;

  const Index size = 200;
  for (bool overflow : {true, false}) {
    const RealScalar scale = overflow ? RealScalar(1e4) : RealScalar(1e-4);
    const MatrixType a = MatrixType::Identity(size, size) * Scalar(scale);
    const RealScalar logabsdet = RealScalar(size) * numext::log(scale);

    LLT<MatrixType> llt(a);
    VERIFY(llt.info() == Success);
    VERIFY(determinant_out_of_range(llt.absDeterminant(), overflow));
    VERIFY_IS_APPROX(llt.logAbsDeterminant(), logabsdet);
    VERIFY_IS_EQUAL(llt.signDeterminant(), Scalar(1));

    LDLT<MatrixType> ldlt(a);
    VERIFY(ldlt.info() == Success);
    VERIFY(determinant_out_of_range(ldlt.absDeterminant(), overflow));
    VERIFY_IS_APPROX(ldlt.logAbsDeterminant(), logabsdet);
    VERIFY_IS_EQUAL(ldlt.signDeterminant(), Scalar(1));
  }
}

// A failed factorization does not represent the input, so the four accessors assert rather than answer from
// it: for [[0,1],[1,0]] LDLT reports NumericalIssue with D = 0, where det = -1. m_isInitialized alone does
// not catch that, since compute() sets it either way.
template <typename MatrixType>
void cholesky_determinant_failed_factorization(Index size) {
  eigen_assert(size >= 2);

  // A zero diagonal with non-zero off-diagonal entries makes the first pivot invalid while the matrix is
  // not; both factorizations give up on it.
  const MatrixType indefinite = MatrixType::Ones(size, size) - MatrixType::Identity(size, size);

  LDLT<MatrixType, Lower> ldltlo(indefinite);
  VERIFY(ldltlo.info() == NumericalIssue);
  VERIFY(!ldltlo.reconstructedMatrix().isApprox(indefinite));
  VERIFY_RAISES_ASSERT(ldltlo.determinant())
  VERIFY_RAISES_ASSERT(ldltlo.absDeterminant())
  VERIFY_RAISES_ASSERT(ldltlo.logAbsDeterminant())
  VERIFY_RAISES_ASSERT(ldltlo.signDeterminant())

  LDLT<MatrixType, Upper> ldltup(indefinite);
  VERIFY(ldltup.info() == NumericalIssue);
  VERIFY_RAISES_ASSERT(ldltup.determinant())
  VERIFY_RAISES_ASSERT(ldltup.absDeterminant())
  VERIFY_RAISES_ASSERT(ldltup.logAbsDeterminant())
  VERIFY_RAISES_ASSERT(ldltup.signDeterminant())

  LLT<MatrixType, Lower> lltlo(indefinite);
  VERIFY(lltlo.info() == NumericalIssue);
  VERIFY_RAISES_ASSERT(lltlo.determinant())
  VERIFY_RAISES_ASSERT(lltlo.absDeterminant())
  VERIFY_RAISES_ASSERT(lltlo.logAbsDeterminant())
  VERIFY_RAISES_ASSERT(lltlo.signDeterminant())
}

template <typename MatrixType>
void cholesky_verify_assert() {
  MatrixType tmp;

  LLT<MatrixType> llt;
  VERIFY_RAISES_ASSERT(llt.matrixL())
  VERIFY_RAISES_ASSERT(llt.matrixU())
  VERIFY_RAISES_ASSERT(llt.solve(tmp))
  VERIFY_RAISES_ASSERT(llt.transpose().solve(tmp))
  VERIFY_RAISES_ASSERT(llt.adjoint().solve(tmp))
  VERIFY_RAISES_ASSERT(llt.solveInPlace(tmp))
  VERIFY_RAISES_ASSERT(llt.determinant())
  VERIFY_RAISES_ASSERT(llt.absDeterminant())
  VERIFY_RAISES_ASSERT(llt.logAbsDeterminant())
  VERIFY_RAISES_ASSERT(llt.signDeterminant())

  LDLT<MatrixType> ldlt;
  VERIFY_RAISES_ASSERT(ldlt.matrixL())
  VERIFY_RAISES_ASSERT(ldlt.transpositionsP())
  VERIFY_RAISES_ASSERT(ldlt.vectorD())
  VERIFY_RAISES_ASSERT(ldlt.isPositive())
  VERIFY_RAISES_ASSERT(ldlt.isNegative())
  VERIFY_RAISES_ASSERT(ldlt.solve(tmp))
  VERIFY_RAISES_ASSERT(ldlt.transpose().solve(tmp))
  VERIFY_RAISES_ASSERT(ldlt.adjoint().solve(tmp))
  VERIFY_RAISES_ASSERT(ldlt.solveInPlace(tmp))
  VERIFY_RAISES_ASSERT(ldlt.determinant())
  VERIFY_RAISES_ASSERT(ldlt.absDeterminant())
  VERIFY_RAISES_ASSERT(ldlt.logAbsDeterminant())
  VERIFY_RAISES_ASSERT(ldlt.signDeterminant())
}

// Test Cholesky decomposition at blocking and vectorization boundaries.
// LLT uses blocks of size max(8, min(size/8 rounded to 16, 128)).
// Sizes near these boundaries exercise the transition between full
// blocked and unblocked paths, including triangular solve boundaries.
template <typename Scalar>
void cholesky_blocking_boundary() {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixType;
  typedef Matrix<Scalar, Dynamic, 1> VectorType;

  const Index PS = internal::packet_traits<Scalar>::size;
  const Index sizes[] = {1, 2, 3,  PS - 1, PS, PS + 1, 2 * PS - 1, 2 * PS, 2 * PS + 1, 4 * PS, 4 * PS + 1, 7,
                         8, 9, 15, 16,     17, 31,     32,         33,     63,         64,     65};
  for (Index si = 0; si < Index(sizeof(sizes) / sizeof(sizes[0])); ++si) {
    Index n = sizes[si];
    if (n < 1) continue;

    // Create a symmetric positive definite matrix: A = R'*R + n*I
    MatrixType R = MatrixType::Random(n, n);
    MatrixType m = R.adjoint() * R;
    m.diagonal().array() += RealScalar(n);

    // LLT
    LLT<MatrixType> llt(m);
    VERIFY(llt.info() == Success);
    VERIFY_IS_APPROX(m, llt.reconstructedMatrix());
    VectorType rhs = VectorType::Random(n);
    VectorType x = llt.solve(rhs);
    VERIFY_IS_APPROX(m * x, rhs);

    // LDLT
    LDLT<MatrixType> ldlt(m);
    VERIFY(ldlt.info() == Success);
    VERIFY_IS_APPROX(m, ldlt.reconstructedMatrix());
    x = ldlt.solve(rhs);
    VERIFY_IS_APPROX(m * x, rhs);
  }
}

// Test Cholesky with RowMajor storage at blocking boundaries.
template <typename Scalar>
void cholesky_rowmajor_boundary() {
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, Dynamic, Dynamic, RowMajor> RowMatrixType;

  const Index sizes[] = {7, 8, 9, 15, 16, 17, 31, 32, 33};
  for (Index si = 0; si < Index(sizeof(sizes) / sizeof(sizes[0])); ++si) {
    Index n = sizes[si];
    RowMatrixType R = RowMatrixType::Random(n, n);
    RowMatrixType m = R.adjoint() * R;
    m.diagonal().array() += RealScalar(n);

    LLT<RowMatrixType> llt(m);
    VERIFY(llt.info() == Success);
    VERIFY_IS_APPROX(m, llt.reconstructedMatrix());

    LDLT<RowMatrixType> ldlt(m);
    VERIFY(ldlt.info() == Success);
    VERIFY_IS_APPROX(m, ldlt.reconstructedMatrix());
  }
}

// Preallocated decompositions of dynamic-size matrices stay allocation-free, including the 1-norm
// they take for rcond(), whose workspace comes from the stack.
template <typename Scalar>
void cholesky_dynamic_preallocated_no_malloc() {
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixType;
  Index size = 8;
  MatrixType A = MatrixType::Random(size, size);
  MatrixType spd = A * A.adjoint() + MatrixType::Identity(size, size) * Scalar(size);
  LLT<MatrixType> llt(size);
  LDLT<MatrixType> ldlt(size);
  internal::set_is_malloc_allowed(false);
  llt.compute(spd);
  VERIFY_IS_EQUAL(llt.info(), Success);
  ldlt.compute(spd);
  VERIFY_IS_EQUAL(ldlt.info(), Success);
  VERIFY(spd.template selfadjointView<Lower>().l1Norm() > 0);
  internal::set_is_malloc_allowed(true);
  VERIFY(llt.rcond() > 0);
}

EIGEN_DECLARE_TEST(cholesky) {
  int s = 0;
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(cholesky(Matrix<double, 1, 1>()));
    CALL_SUBTEST_1(cholesky_dynamic_preallocated_no_malloc<double>());
    CALL_SUBTEST_1(cholesky_dynamic_preallocated_no_malloc<std::complex<double> >());
    CALL_SUBTEST_3(cholesky(Matrix2d()));
    CALL_SUBTEST_3(cholesky_bug241(Matrix2d()));
    CALL_SUBTEST_3(cholesky_definiteness(Matrix2d()));
    CALL_SUBTEST_4(cholesky(Matrix3f()));
    CALL_SUBTEST_5(cholesky(Matrix4d()));

    CALL_SUBTEST_3(cholesky_ldlt_rankupdate_info(Matrix2d()));
    CALL_SUBTEST_4(cholesky_ldlt_rankupdate_info(Matrix3f()));
    CALL_SUBTEST_5(cholesky_ldlt_rankupdate_info(Matrix4d()));

    CALL_SUBTEST_1(cholesky_ldlt_rankupdate_zero_components(Matrix<double, 1, 1>()));
    CALL_SUBTEST_3(cholesky_ldlt_rankupdate_zero_components(Matrix2d()));
    CALL_SUBTEST_4(cholesky_ldlt_rankupdate_zero_components(Matrix3f()));
    CALL_SUBTEST_5(cholesky_ldlt_rankupdate_zero_components(Matrix4d()));

    s = internal::random<int>(1, EIGEN_TEST_MAX_SIZE);
    CALL_SUBTEST_2(cholesky(MatrixXd(s, s)));
    TEST_SET_BUT_UNUSED_VARIABLE(s);

    s = internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 2);
    CALL_SUBTEST_6(cholesky_cplx(MatrixXcd(s, s)));
    TEST_SET_BUT_UNUSED_VARIABLE(s);

    // cholesky_ldlt_rankupdate_info() needs at least two rows for its NumericalIssue case.
    s = internal::random<int>(2, EIGEN_TEST_MAX_SIZE / 4);
    CALL_SUBTEST_2(cholesky_ldlt_rankupdate_info(MatrixXd(s, s)));
    TEST_SET_BUT_UNUSED_VARIABLE(s);

    s = internal::random<int>(2, EIGEN_TEST_MAX_SIZE / 8);
    CALL_SUBTEST_6(cholesky_ldlt_rankupdate_info(MatrixXcd(s, s)));
    TEST_SET_BUT_UNUSED_VARIABLE(s);

    // Quadratic in the size, since every position of the zero is tried in turn.
    s = internal::random<int>(1, EIGEN_TEST_MAX_SIZE / 16);
    CALL_SUBTEST_2(cholesky_ldlt_rankupdate_zero_components(MatrixXd(s, s)));
    CALL_SUBTEST_6(cholesky_ldlt_rankupdate_zero_components(MatrixXcd(s, s)));
    TEST_SET_BUT_UNUSED_VARIABLE(s);

    // Bounded so that the determinant itself, not just its logarithm, stays in range.
    s = internal::random<int>(1, 30);
    CALL_SUBTEST_2(cholesky_determinant<MatrixXd>(s));
    CALL_SUBTEST_8(cholesky_determinant<MatrixXf>(s));
    CALL_SUBTEST_6(cholesky_determinant<MatrixXcd>(s));
    TEST_SET_BUT_UNUSED_VARIABLE(s);
  }
  // empty matrix, regression test for Bug 785:
  CALL_SUBTEST_2(cholesky(MatrixXd(0, 0)));
  CALL_SUBTEST_2(cholesky_determinant_empty<MatrixXd>());
  CALL_SUBTEST_8(cholesky_determinant_overflow<MatrixXf>());
  CALL_SUBTEST_3(cholesky_determinant_failed_factorization<Matrix2d>(2));
  CALL_SUBTEST_2(cholesky_determinant_failed_factorization<MatrixXd>(internal::random<int>(2, 20)));
  CALL_SUBTEST_6(cholesky_determinant_failed_factorization<MatrixXcd>(internal::random<int>(2, 20)));

  // This does not work yet:
  // CALL_SUBTEST_2( cholesky(Matrix<double,0,0>()) );

  CALL_SUBTEST_4(cholesky_verify_assert<Matrix3f>());
  CALL_SUBTEST_7(cholesky_verify_assert<Matrix3d>());
  CALL_SUBTEST_8(cholesky_verify_assert<MatrixXf>());
  CALL_SUBTEST_2(cholesky_verify_assert<MatrixXd>());

  // Test problem size constructors
  CALL_SUBTEST_9(LLT<MatrixXf>(10));
  CALL_SUBTEST_9(LDLT<MatrixXf>(10));

  CALL_SUBTEST_2(cholesky_faillure_cases<void>());

  // Blocking and vectorization boundary tests (deterministic, outside g_repeat).
  CALL_SUBTEST_2(cholesky_blocking_boundary<double>());
  CALL_SUBTEST_8(cholesky_blocking_boundary<float>());
  CALL_SUBTEST_6(cholesky_blocking_boundary<std::complex<double> >());
  CALL_SUBTEST_2(cholesky_rowmajor_boundary<double>());
  CALL_SUBTEST_8(cholesky_rowmajor_boundary<float>());
  CALL_SUBTEST_2(llt_inverse_threshold_boundary<double>());
  CALL_SUBTEST_8(llt_inverse_threshold_boundary<float>());

  TEST_SET_BUT_UNUSED_VARIABLE(nb_temporaries);
}
