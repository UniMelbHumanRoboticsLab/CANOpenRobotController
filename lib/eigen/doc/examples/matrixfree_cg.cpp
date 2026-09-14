// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <iostream>
#include <Eigen/IterativeLinearSolvers>

class MatrixReplacement;
using Eigen::SparseMatrix;

namespace Eigen {
namespace internal {
// MatrixReplacement looks-like a SparseMatrix, so let's inherit its traits:
template <>
struct traits<MatrixReplacement> : public Eigen::internal::traits<Eigen::SparseMatrix<double> > {};
}  // namespace internal
}  // namespace Eigen

// Example of a matrix-free wrapper from a user type to Eigen's compatible type
// For the sake of simplicity, this example simply wrap a Eigen::SparseMatrix.
class MatrixReplacement : public Eigen::EigenBase<MatrixReplacement> {
 public:
  // Required types, constants, and methods:
  using Scalar = double;
  using RealScalar = double;
  using StorageIndex = int;
  static constexpr int ColsAtCompileTime = Eigen::Dynamic;
  static constexpr int MaxColsAtCompileTime = Eigen::Dynamic;
  static constexpr bool IsRowMajor = false;

  Index rows() const { return mp_mat->rows(); }
  Index cols() const { return mp_mat->cols(); }

  template <typename Rhs>
  Eigen::Product<MatrixReplacement, Rhs, Eigen::AliasFreeProduct> operator*(const Eigen::MatrixBase<Rhs>& x) const {
    return Eigen::Product<MatrixReplacement, Rhs, Eigen::AliasFreeProduct>(*this, x.derived());
  }

  // Custom API:
  MatrixReplacement() = default;

  void attachMyMatrix(const SparseMatrix<double>& mat) { mp_mat = &mat; }
  const SparseMatrix<double>& my_matrix() const { return *mp_mat; }

 private:
  const SparseMatrix<double>* mp_mat = nullptr;
};

// Implementation of MatrixReplacement * Eigen::DenseVector though a specialization of internal::generic_product_impl:
namespace Eigen {
namespace internal {

template <typename Rhs>
struct generic_product_impl<MatrixReplacement, Rhs, SparseShape, DenseShape,
                            GemvProduct>  // GEMV stands for matrix-vector
    : generic_product_impl_base<MatrixReplacement, Rhs, generic_product_impl<MatrixReplacement, Rhs> > {
  using Scalar = typename Product<MatrixReplacement, Rhs>::Scalar;

  template <typename Dest>
  static void scaleAndAddTo(Dest& dst, const MatrixReplacement& lhs, const Rhs& rhs, const Scalar& alpha) {
    // This method should implement "dst += alpha * lhs * rhs" inplace,
    // however, for iterative solvers, alpha is always equal to 1, so let's not bother about it.
    eigen_assert(alpha == Scalar(1) && "scaling is not implemented");
    EIGEN_ONLY_USED_FOR_DEBUG(alpha);

    // Here we could simply call dst.noalias() += lhs.my_matrix() * rhs,
    // but let's do something fancier (and less efficient):
    for (Index i = 0; i < lhs.cols(); ++i) dst += rhs(i) * lhs.my_matrix().col(i);
  }
};

}  // namespace internal
}  // namespace Eigen

// Solve Ax = b with the given matrix-free solver and report the outcome. The default tolerance,
// NumTraits<double>::epsilon(), is out of reach for CG and MINRES on this matrix, so pick an attainable one.
template <typename Solver>
void solve(const char* name, Solver& solver, const MatrixReplacement& A, const Eigen::VectorXd& b) {
  solver.setTolerance(1e-10);
  solver.compute(A);
  Eigen::VectorXd x = solver.solve(b);
  std::cout << name << (solver.info() == Eigen::Success ? " converged" : " did not converge") << " after "
            << solver.iterations() << " iterations, estimated error: " << solver.error() << std::endl;
}

int main() {
  Eigen::Index n = 10;
  Eigen::SparseMatrix<double> S = Eigen::MatrixXd::Random(n, n).sparseView(0.5, 1);
  S = S.transpose() * S;

  MatrixReplacement A;
  A.attachMyMatrix(S);

  Eigen::VectorXd b(n);
  b.setRandom();

  // Solve Ax = b using various iterative solver with matrix-free version:
  Eigen::ConjugateGradient<MatrixReplacement, Eigen::Lower | Eigen::Upper, Eigen::IdentityPreconditioner> cg;
  solve("CG:      ", cg, A, b);

  Eigen::BiCGSTAB<MatrixReplacement, Eigen::IdentityPreconditioner> bicgstab;
  solve("BiCGSTAB:", bicgstab, A, b);

  Eigen::GMRES<MatrixReplacement, Eigen::IdentityPreconditioner> gmres;
  solve("GMRES:   ", gmres, A, b);

  Eigen::DGMRES<MatrixReplacement, Eigen::IdentityPreconditioner> dgmres;
  solve("DGMRES:  ", dgmres, A, b);

  Eigen::MINRES<MatrixReplacement, Eigen::Lower | Eigen::Upper, Eigen::IdentityPreconditioner> minres;
  solve("MINRES:  ", minres, A, b);
}
