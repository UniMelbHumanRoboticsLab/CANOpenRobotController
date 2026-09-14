# Eigen GPU Module (`contrib/Eigen/GPU`)

GPU-accelerated linear algebra for Eigen users, dispatching to NVIDIA CUDA
Math Libraries (cuBLAS, cuSOLVER, cuFFT, cuSPARSE, cuDSS). Requires CUDA 11.4+;
cuDSS features require CUDA 12.0+ and a separate cuDSS install. Header-only.

This module dispatches rather than reimplements, so numerical behavior,
supported shapes and scalar types, and performance characteristics are the
vendor libraries'. Their documentation is the reference for anything this file
does not state:

| Library | Used for | Documentation |
|---------|----------|---------------|
| CUDA Toolkit | streams, memory, error codes | <https://docs.nvidia.com/cuda/> |
| cuBLAS (incl. cuBLASLt) | `DeviceMatrix` products, BLAS-1 | <https://docs.nvidia.com/cuda/cublas/> |
| cuSOLVER | dense LLT / LU / QR / SVD / EVD | <https://docs.nvidia.com/cuda/cusolver/> |
| cuSPARSE | SpMV / SpMM | <https://docs.nvidia.com/cuda/cusparse/> |
| cuFFT | `gpu::FFT` | <https://docs.nvidia.com/cuda/cufft/> |
| NPP | device-side scalar and coefficient-wise arithmetic | <https://docs.nvidia.com/cuda/npp/> |
| cuDSS | sparse direct solvers (separate install) | <https://docs.nvidia.com/cuda/cudss/> |

## Why this module

Eigen is the linear algebra foundation for a large ecosystem of C++ projects
in robotics (ROS, Drake, MoveIt, Pinocchio), computer vision (OpenCV, COLMAP,
Open3D), scientific computing (Ceres, Stan), and beyond. Many of these
projects run on GPU-equipped hardware but cannot use GPUs for Eigen operations
without dropping down to raw CUDA library APIs.

GPU sparse solvers are a particularly acute gap. Sparse factorization is the
bottleneck in SLAM, bundle adjustment, FEM, and nonlinear optimization --
exactly the workloads where GPU acceleration matters most. Downstream projects
like [Ceres](https://github.com/ceres-solver/ceres-solver/issues/1151) and
[COLMAP](https://github.com/colmap/colmap/issues/4018) have open requests for
GPU-accelerated sparse solvers, and third-party projects like
[cholespy](https://github.com/rgl-epfl/cholespy) exist specifically because
Eigen lacks them. The `contrib/Eigen/GPU` module provides GPU sparse Cholesky, LDL^T,
and LU factorization via cuDSS, alongside dense solvers (cuSOLVER), matrix
products (cuBLAS), FFT (cuFFT), and sparse matrix-vector products (cuSPARSE).

Existing Eigen users should be able to move performance-critical dense or
sparse linear algebra to the GPU with minimal code changes and without
learning CUDA library APIs directly.

## Design philosophy

**CPU and GPU coexist.** There is no global compile-time switch that replaces
CPU implementations (unlike `EIGEN_USE_LAPACKE`). Users choose GPU solvers
explicitly -- `gpu::LLT<double>` vs `Eigen::LLT<MatrixXd>`,
`gpu::SparseLLT<double>` vs `SimplicialLLT<SparseMatrix<double>>` -- and both
coexist in the same binary.
This also lets users keep the factored matrix on device across multiple solves,
something impossible with compile-time replacement.

**Familiar syntax.** GPU operations use the same expression patterns as CPU
Eigen. Here is a side-by-side comparison:

```cpp
// ---- CPU (Eigen) ----               // ---- GPU (contrib/Eigen/GPU) ----
#include <Eigen/Dense>                  #define EIGEN_USE_GPU
                                        #include <contrib/Eigen/GPU>

// Dense
MatrixXd A = ...;                       auto d_A = gpu::DeviceMatrix<double>::fromHost(A);
MatrixXd B = ...;                       auto d_B = gpu::DeviceMatrix<double>::fromHost(B);

MatrixXd C = A * B;                     gpu::DeviceMatrix<double> d_C = d_A * d_B;
MatrixXd X = A.llt().solve(B);          gpu::DeviceMatrix<double> d_X = d_A.llt().solve(d_B);

                                        MatrixXd X = d_X.toHost();

// Sparse (using SpMat = SparseMatrix<double>)
SimplicialLLT<SpMat> llt(A);            gpu::SparseLLT<double> llt(A);
VectorXd x = llt.solve(b);              VectorXd x = llt.solve(b);
```

The GPU version reads like CPU Eigen with explicit upload/download for dense
operations, and an almost identical API for sparse solvers. Expressions can
copy-initialize a `DeviceMatrix` directly (as above), scalar factors accept
plain literals (`2 * d_A`, `d_A / 2`, `-d_A`), and unsupported expressions are
compile errors.

**Standalone module.** `contrib/Eigen/GPU` does not modify or depend on Eigen's Core
expression template system (`MatrixBase`, `CwiseBinaryOp`, etc.).
`DeviceMatrix` is not an Eigen expression type and does not inherit from
`MatrixBase`. The expression layer is a thin compile-time dispatch where every
supported expression maps to a single NVIDIA library call. There is no
coefficient-level evaluation, lazy fusion, or packet operations.

**Interoperability where useful.** `DeviceMatrix` provides the same operator
signatures as `Matrix` for common vector operations: `+=`, `-=`, `*=`,
`dot()`, `squaredNorm()`, `norm()`, `setZero()`, and `noalias()`. This makes
`DeviceMatrix` usable as a drop-in `VectorType` in Eigen algorithm templates
that rely on these operations. For example, Eigen's `conjugate_gradient()`
template works with `DeviceMatrix` with a single typedef change -- no
modifications to the algorithm or the expression template system. Conjugate
gradient is just the motivating example; we are open to expanding operator
coverage as needed to support other high-level Eigen algorithms on the GPU.

**Explicit over implicit.** Host-device transfers, stream management, and
library handle lifetimes are visible in the API. There are no hidden
allocations or synchronizations except where documented (e.g., `toHost()` must
synchronize to deliver data to the host).

## Key concepts

### `gpu::DeviceMatrix<Scalar>`

A typed RAII wrapper for a dense column-major matrix in GPU device memory.
This is the GPU counterpart of Eigen's `MatrixX<Scalar>`. A vector is simply
a `DeviceMatrix` with one column. All public GPU classes live in `namespace
Eigen::gpu`.

```cpp
// Upload from host
auto d_A = gpu::DeviceMatrix<double>::fromHost(A);

// Allocate uninitialized
gpu::DeviceMatrix<double> d_C(m, n);

// Download to host
MatrixXd C = d_C.toHost();

// Async download (returns a future)
auto transfer = d_C.toHostAsync();
// ... do other work ...
MatrixXd C = transfer.get();
```

`DeviceMatrix` supports expression methods that mirror Eigen's API:
`adjoint()`, `transpose()`, `triangularView<UpLo>()`,
`selfadjointView<UpLo>()`, `llt()`, `lu()`. These return lightweight
expression objects that are evaluated when assigned.

For BLAS Level-1 operations, `DeviceMatrix` also provides `dot()`, `norm()`,
`squaredNorm()`, `setZero()`, `noalias()`, and arithmetic operators
(`+=`, `-=`, `*=`) that dispatch to cuBLAS `axpy`, `nrm2`, `dot`, `scal`,
and `geam`. These are the operations needed by iterative solvers.

### `gpu::DeviceScalar<Scalar>`

A device-resident scalar value. Reductions like `dot()`, `norm()`, and
`squaredNorm()` return `DeviceScalar` instead of a host scalar, deferring
the host synchronization until the value is actually needed:

```cpp
auto dot_val = d_x.dot(d_y);          // DeviceScalar -- no sync
auto norm_sq = d_r.squaredNorm();      // DeviceScalar -- no sync
Scalar alpha = dot_val / norm_sq;      // sync here (implicit conversion)
d_x += alpha * d_p;                    // host scalar * DeviceMatrix (axpy)
```

Division between `DeviceScalar` values (real types only) is performed on
device via NPP, avoiding extra synchronizations. Small device allocations
(including `DeviceScalar`) go through the stream-ordered allocator like every
other block when the device has memory pools; on the `cudaMalloc` fallback
path they are recycled through a thread-local `DeviceBufferPool` instead, to
avoid `cudaMalloc`/`cudaFree` overhead in tight loops.
Pool contract: a released block is recycled only after the device has retired
every operation enqueued before the release on any blocking stream, so pooled
buffers may move between the streams of one thread. The release is tracked by
an event on the legacy default stream, the same ordering the stream-ordered
allocator relies on. The pool is thread-local, so sharing a pooled buffer
across threads needs external synchronization, and `cudaStreamNonBlocking`
streams are outside the guarantee.

### `gpu::Context`

Every GPU operation needs a CUDA stream and library handles (cuBLAS eagerly,
cuSOLVER / cuBLASLt / cuSPARSE lazily on first use). `gpu::Context` bundles
these together. A single `Context` is not thread-safe -- use one per thread
(or external synchronization), since the underlying NVIDIA library handles
are not thread-safe per handle.

For simple usage, you don't need to create one -- a per-thread default context
is created lazily on first use:

```cpp
// These use the thread-local default context automatically
d_C = d_A * d_B;
d_X = d_A.llt().solve(d_B);
```

For concurrent multi-stream execution, create explicit contexts:

```cpp
gpu::Context ctx1, ctx2;
d_C1.device(ctx1) = d_A1 * d_B1;   // runs on stream 1
d_C2.device(ctx2) = d_A2 * d_B2;   // runs on stream 2 (concurrently)
```

To integrate with existing CUDA code, borrow an existing stream:

```cpp
gpu::Context ctx(my_existing_stream);  // wraps stream, does not take ownership
```

To override the thread-local default (e.g., in CG where all ops share one
context):

```cpp
gpu::Context ctx;
gpu::Context::setThreadLocal(&ctx);    // all threadLocal() calls return ctx
// ... GPU operations ...
gpu::Context::setThreadLocal(nullptr); // restore lazy-created default
```

### Linking {#eigen_gpu_linking}

The module is header-only, but each feature pulls in the corresponding NVIDIA
library at link time. cuSOLVER, cuBLASLt, and cuSPARSE are created lazily on
first use, so a translation unit that only uses cuBLAS or cuFFT does not need
to link the others:

| Feature                                 | Link flags                |
|-----------------------------------------|---------------------------|
| `DeviceMatrix`, GEMM, TRSM, SYMM, SYRK  | `-lcublas -lcublasLt`     |
| Dense solvers (LLT, LU, QR, SVD, EVD)   | `-lcusolver -lcublas`     |
| FFT (`gpu::FFT`)                        | `-lcufft -lcublas`        |
| SpMV / SpMM (`gpu::SparseContext`)      | `-lcusparse -lcublas`     |
| Sparse direct solvers (cuDSS)           | `-lcudss -lcublas`        |

cuBLAS is required by `DeviceMatrix` itself (every `Context` creates a cuBLAS
handle eagerly) and is also a runtime dependency of cuDSS, so it is the one
constant. cuDSS additionally requires `EIGEN_CUDSS` to be defined before
including `contrib/Eigen/GPU`.

## Usage

### Matrix operations (cuBLAS)

Products dispatch to [cuBLAS](https://docs.nvidia.com/cuda/cublas/), GEMM
through its cuBLASLt API; see [Precision control](#eigen_gpu_precision) for which
compute type that selects.

```cpp
auto d_A = gpu::DeviceMatrix<double>::fromHost(A);
auto d_B = gpu::DeviceMatrix<double>::fromHost(B);

// GEMM: C = A * B, C = A^H * B, C = A * B^T, ...
gpu::DeviceMatrix<double> d_C = d_A * d_B;
d_C = d_A.adjoint() * d_B;
d_C = d_A * d_B.transpose();

// Scaled and accumulated
d_C += 2.0 * d_A * d_B;             // alpha=2, beta=1
d_C -= d_A * d_B;                   // alpha=-1, beta=1
d_C.device(ctx) -= d_A * d_B;       // same, on an explicit stream

// Triangular solve (TRSM)
d_X = d_A.triangularView<Lower>().solve(d_B);

// Symmetric/Hermitian multiply (SYMM/HEMM)
d_C = d_A.selfadjointView<Lower>() * d_B;

// Rank-k update (SYRK/HERK)
d_C.selfadjointView<Lower>().rankUpdate(d_A);  // C += A * A^H
```

### BLAS Level-1 operations

Dot products, norms and vector arithmetic map to the corresponding
[cuBLAS](https://docs.nvidia.com/cuda/cublas/) Level-1 routines, except for
device-side scalar arithmetic, which uses the signal-processing functions of
[NPP](https://docs.nvidia.com/cuda/npp/).

```cpp
// Dot product and norms (return DeviceScalar -- no sync until read)
auto dot_val = d_x.dot(d_y);          // cublasDdot / cublasCdotc
auto norm_val = d_r.norm();            // cublasDnrm2
double n = norm_val;                   // implicit conversion triggers sync

// Vector arithmetic (cuBLAS axpy / geam)
d_x += alpha * d_p;                    // axpy: x = x + alpha * p
d_x -= alpha * d_p;                    // axpy: x = x - alpha * p
d_x *= alpha;                          // scal: x = alpha * x
d_r.setZero();                         // cudaMemsetAsync

// DeviceScalar arithmetic (stays on device, real types only)
auto alpha = absNew / dot_val;         // device-side division via NPP
d_x += alpha * d_p;                    // DeviceScalar * DeviceMatrix (axpy with device pointer)

// Matrix add/subtract/scale (cuBLAS geam)
gpu::DeviceMatrix<double> d_C = d_A + d_B;       // C = A + B
d_C = d_A + 2.0 * d_B;                          // C = A + 2*B
d_C = d_A - d_B;                                 // C = A - B
d_C = 2 * d_A - 3 * d_B;                        // scaled both sides, int literals fine
d_C = -d_A;                                      // unary minus
d_C = d_A / 2.0;                                 // divide by scalar
d_C = 0.5 * d_C;                                 // in-place rescale (aliasing-safe)
```

### Dense solvers (cuSOLVER)

Backed by the dense part of [cuSOLVER](https://docs.nvidia.com/cuda/cusolver/)
(cuSolverDN), whose documentation defines what each factorization returns and
when it reports a numerical failure.

**One-shot expression syntax** -- "one-shot" means factorization and solve
run as a single fused call with no persistent factorization object; each
evaluation re-factorizes:

```cpp
// Cholesky solve (potrf + potrs)
gpu::DeviceMatrix<double> d_X = d_A.llt().solve(d_B);

// LU solve (getrf + getrs)
d_Y = d_A.lu().solve(d_B);
```

Scratch for the one-shot form (factor copy, cuSOLVER workspace, info words)
lives in the `gpu::Context` and grows monotonically — repeated one-shot solves
perform no per-call allocations. In debug builds each call verifies the
factorization status (one stream synchronization); release builds
(`EIGEN_NO_DEBUG`/`NDEBUG`) skip the check *and* the sync, making the
expression fully asynchronous — use the cached `gpu::LLT` / `gpu::LU` classes
and `info()` when numerical failure must be detected.

**Cached factorization** -- Factor once, solve many times:

```cpp
gpu::LLT<double> llt;
llt.compute(d_A);                    // factorize (async)
if (llt.info() != Success) { ... }   // lazy sync on first info() call
auto d_X1 = llt.solve(d_B1);        // reuses factor (async)
auto d_X2 = llt.solve(d_B2);        // reuses factor (async)
auto d_X3 = llt.solve(std::move(d_B3));  // in-place: consumes RHS, no copy/alloc
MatrixXd X2 = d_X2.toHost();

// Bind a solver to an existing Context: work runs on ctx's stream with its
// handles, so it chains with GEMM/SpMV on the same Context without
// cross-stream event waits. All five dense solvers (LLT, LU, QR, SVD,
// SelfAdjointEigenSolver) support this.
gpu::Context ctx;
gpu::LLT<double> llt_ctx(ctx, d_A);

// LU with transpose solve
gpu::LU<double> lu;
lu.compute(d_A);
auto d_Y = lu.solve(d_B, gpu::GpuOp::Trans);           // A^T Y = B

// QR solve (overdetermined least squares)
gpu::QR<double> qr;
qr.compute(d_A);                     // factorize on device (async)
auto d_X = qr.solve(d_B);           // Q^H * B via ormqr, then trsm on R
MatrixXd X = d_X.toHost();

// SVD (results downloaded on access)
gpu::SVD<double> svd;
svd.compute(d_A, ComputeThinU | ComputeThinV);
VectorXd S = svd.singularValues();   // downloads to host
MatrixXd U = svd.matrixU();          // downloads to host
MatrixXd V = svd.matrixV();          // V (matches JacobiSVD)
MatrixXd VT = svd.matrixVT();        // V^T (matches cuSOLVER)

// SVD: device-side views (no D2H transfer; svd must outlive the views)
auto d_S = svd.d_singularValues();   // DeviceMatrix view of singular values
auto d_U = svd.d_matrixU();          // DeviceMatrix view of U
auto d_VT = svd.d_matrixVT();        // DeviceMatrix view of V^T

// Self-adjoint eigenvalue decomposition
gpu::SelfAdjointEigenSolver<double> es;
es.compute(d_A);
VectorXd eigenvals = es.eigenvalues();    // downloads to host
MatrixXd eigenvecs = es.eigenvectors();   // downloads to host
auto d_W = es.d_eigenvalues();            // DeviceMatrix view of eigenvalues
auto d_V = es.d_eigenvectors();           // DeviceMatrix view of eigenvectors
```

The cached API keeps the factored matrix on device, avoiding redundant
host-device transfers and re-factorizations. All five solvers accept
`compute(DeviceMatrix&&)` to adopt the input and factor it in place with no
copy (for QR/SVD with m < n the internal transpose still copies), and all five
can bind to a `gpu::Context` to share its stream and handles. All solvers also
accept host dense expressions directly as a convenience (e.g.,
`gpu::LLT<double> llt(A)` or `qr.solve(B)`), which handles upload/download
internally. Host `compute()` finishes its upload before returning, while
factorization remains asynchronous. The `d_*` accessors
on `gpu::SVD` and `gpu::SelfAdjointEigenSolver` return non-owning
`DeviceMatrix` views so downstream cuBLAS/cuSOLVER work can chain without
round-tripping through host memory.

### Sparse direct solvers (cuDSS)

Requires [cuDSS](https://docs.nvidia.com/cuda/cudss/) (separate install, CUDA
12.0+), which is distributed outside the CUDA Toolkit and versioned separately
from it. Define `EIGEN_CUDSS` before including `contrib/Eigen/GPU`; see
[Linking](#eigen_gpu_linking) for link flags.

```cpp
SparseMatrix<double> A = ...;  // symmetric positive definite
VectorXd b = ...;

// Sparse Cholesky -- one-liner
gpu::SparseLLT<double> llt(A);
VectorXd x = llt.solve(b);

// Three-phase workflow for repeated solves with the same sparsity pattern
gpu::SparseLLT<double> llt;
llt.analyzePattern(A);               // symbolic analysis (once)
llt.factorize(A);                    // numeric factorization
VectorXd x = llt.solve(b);
llt.factorize(A_new_values);         // refactorize (reuses symbolic analysis)
VectorXd x2 = llt.solve(b);

// Sparse LDL^T (symmetric indefinite)
gpu::SparseLDLT<double> ldlt(A);
VectorXd x = ldlt.solve(b);

// Sparse LU (general non-symmetric)
gpu::SparseLU<double> lu(A);
VectorXd x = lu.solve(b);

// Bind to an existing Context (same stream as SpMV / cuBLAS work) and solve
// with a device-resident RHS — result stays on device, no host sync:
gpu::Context ctx;
gpu::SparseLLT<double> llt_ctx(ctx, A);
auto d_b = gpu::DeviceMatrix<double>::fromHost(b, ctx.stream());
gpu::DeviceMatrix<double> d_x = llt_ctx.solve(d_b);
```

#### Solver configuration (cuDSS >= 0.8)

`gpu::SparseSolverConfig` passes cuDSS tuning knobs through to the solver:
fill-reducing reordering, matching, pivoting strategy / threshold / epsilon,
iterative refinement, and the hybrid host/device memory and execute modes.
Fields left at their defaults keep the cuDSS defaults, which favor speed over
maximum robustness — for badly scaled or nearly singular systems, consider
enabling matching and iterative refinement:

```cpp
gpu::SparseSolverConfig cfg;
cfg.reordering = gpu::SparseReordering::Amd;
cfg.matching = gpu::SparseMatching::Auto;   // off by cuDSS default
cfg.refinementSteps = 2;                    // iterative refinement in solve()
gpu::SparseLU<double> lu;
lu.setConfig(cfg);                          // before compute(): reordering and
lu.compute(A);                              // matching apply at analysis time
VectorXd x = lu.solve(b);
```

Each knob is consumed by the phase it affects (reordering/matching by
`analyzePattern()`, pivoting by `factorize()`, refinement by `solve()`), so
`setConfig()` must run before the first phase whose behavior it changes.

Every field is a pass-through, so which values are admissible for a given matrix
type — and what each one does — is cuDSS's contract, not ours:
[cuDSS Data Types](https://docs.nvidia.com/cuda/cudss/types.html) documents
`cudssConfigParam_t` and the `cudssReorderingAlg_t` / `cudssMatchingAlg_t` /
`cudssPivotType_t` values these enums mirror, and
[cuDSS Advanced Features](https://docs.nvidia.com/cuda/cudss/advanced_features.html)
describes the hybrid host/device memory and execute modes.

cuDSS < 0.8 names none of these algorithms. There the `SparseReordering`,
`SparseMatching` and `SparsePivoting` enumerators other than `Default` are not
declared, so selecting one is a compile error rather than a request the linked
cuDSS cannot honor. The remaining fields — thresholds, refinement, the hybrid
modes — still exist, and `setConfig()` refuses any non-default value of them: it
asserts, and `info()` reports `InvalidInput` until the config is reset to
default, so the request cannot be silently downgraded to the cuDSS defaults.
`EIGEN_HAS_CUDSS_SOLVER_CONFIG` is 1 or 0 accordingly, for callers that need to
branch at compile time.

### FFT (cuFFT)

Plans and data layouts are [cuFFT](https://docs.nvidia.com/cuda/cufft/)'s. The
scaling convention is not: cuFFT leaves its transforms unnormalized, and
`gpu::FFT` applies the 1/n on the inverse so that `inv(fwd(x)) == x`, matching
`contrib/Eigen/FFT`.

```cpp
gpu::FFT<float> fft;                // shares stream + cuBLAS with the
                                    // thread-local default Context
gpu::Context ctx;
gpu::FFT<float> fft_on_ctx(ctx);    // share stream + cuBLAS with an
                                    // explicit Context (e.g. for
                                    // multi-stream pipelines)

// 1D complex-to-complex
VectorXcf X = fft.fwd(x);           // forward
VectorXcf y = fft.inv(X);           // inverse (scaled by 1/n)

// 1D real-to-complex / complex-to-real
VectorXcf R = fft.fwd(r);           // returns n/2+1 complex (half-spectrum)
VectorXf  s = fft.invReal(R, n);    // C2R inverse, caller specifies n

// 2D complex-to-complex
MatrixXcf B = fft.fwd2(A);         // 2D forward
MatrixXcf C = fft.inv2(B);         // 2D inverse (scaled by 1/(rows*cols))

// Plans are cached and reused across calls with the same size/type.

// Device-resident transforms: DeviceMatrix in/out, no host transfer, no sync.
gpu::DeviceMatrix<std::complex<float>> d_X, d_y;
fft.fwd(d_x, d_X);                  // 1D C2C forward (d_x: complex column vector)
fft.inv(d_X, d_y);                  // 1D C2C inverse (scaled by 1/n)
fft.fwd(d_r, d_R);                  // 1D R2C (d_r: real column vector)
fft.invReal(d_R, d_s, n);           // 1D C2R (input preserved)
fft.fwd2(d_A, d_B);                 // 2D C2C forward
fft.inv2(d_B, d_C);                 // 2D C2C inverse
```

### Sparse matrix-vector multiply (cuSPARSE)

Uses the [cuSPARSE](https://docs.nvidia.com/cuda/cusparse/) generic API
(`cusparseSpMV` / `cusparseSpMM`), which fixes the supported index and value
type combinations.

```cpp
SparseMatrix<double> A = ...;
VectorXd x = ...;

// Host vectors (upload/download handled internally)
gpu::SparseContext<double> spmv;
VectorXd y = spmv.multiply(A, x);           // y = A * x
VectorXd z = spmv.multiplyT(A, x);          // z = A^T * x
spmv.multiply(A, x, y, 2.0, 1.0);           // y = 2*A*x + y
spmv.multiply(A, x, y, 1.0, 0.0,            // y = A^H * x (Hermitian SpMV)
              gpu::GpuOp::ConjTrans);

// Multiple RHS (SpMM)
MatrixXd Y = spmv.multiplyMat(A, X);                      // Y = A * X
MatrixXd Z = spmv.multiplyMat(A, X, gpu::GpuOp::Trans);   // Z = A^T * X

// Device-resident SpMV / SpMM (sparse matrix cached on device)
gpu::Context ctx;
gpu::SparseContext<double> spmv_dev(ctx);   // share gpu::Context for same-stream
auto d_A = spmv_dev.deviceView(A);          // upload sparse matrix once
d_y = d_A * d_x;                            // SpMV, stays on device
d_Y = d_A * d_X;                            // SpMM when the RHS has > 1 column
```

Host-input calls re-upload the sparse values *and* index arrays on every call
(host pointer identity cannot detect a pattern rewritten in place or assigned
into the same allocations, so the structure is never assumed unchanged). The
cuSPARSE descriptors and workspace-size queries are cached across calls with
matching shapes; `deviceView()` is the upload-once path. A `DeviceSparseView`
carries a generation counter — using a view after any later upload through its
context asserts instead of silently multiplying by the wrong matrix.

### Eigen algorithm interop (example: Conjugate gradient)

The BLAS-1 operators and `DeviceSparseView` make `DeviceMatrix` usable as a
vector type in GPU implementations of algorithms like conjugate gradient.
Conjugate gradient is the motivating example -- the GPU CG mirrors Eigen's
`conjugate_gradient()` line for line, with only one host sync per iteration
(the convergence check). All scalar intermediates (`alpha`, `beta`, `absNew`)
stay on device as `DeviceScalar` values:

```cpp
gpu::Context ctx;
gpu::Context::setThreadLocal(&ctx);
gpu::SparseContext<double> spmv(ctx);
auto mat = spmv.deviceView(A);              // upload sparse matrix once

auto rhs = gpu::DeviceMatrix<double>::fromHost(b, ctx.stream());
gpu::DeviceMatrix<double> x(n, 1);
x.setZero();
gpu::DeviceMatrix<double> residual(n, 1);
residual.copyFrom(ctx, rhs);                // r = b (x=0)
gpu::DeviceMatrix<double> p(n, 1);
p.copyFrom(ctx, residual);                  // p = r
gpu::DeviceMatrix<double> z(n, 1), tmp(n, 1);

auto absNew = residual.dot(p);              // DeviceScalar -- no sync

while (i < maxIters) {
  tmp.noalias() = mat * p;                   // SpMV, device-resident

  auto alpha = absNew / p.dot(tmp);          // DeviceScalar / DeviceScalar -- no sync

  x += alpha * p;                            // DeviceScalar * DeviceMatrix axpy -- no sync
  residual -= alpha * tmp;                   // DeviceScalar * DeviceMatrix axpy -- no sync

  residualNorm2 = residual.squaredNorm();    // THE one sync per iteration
  if (residualNorm2 < threshold) break;

  z.copyFrom(ctx, residual);                 // no preconditioner: z = r
  auto absOld = std::move(absNew);           // no sync, no alloc
  absNew = residual.dot(z);                  // DeviceScalar -- no sync
  auto beta = absNew / absOld;               // DeviceScalar / DeviceScalar -- no sync

  p *= beta;                                 // DeviceScalar scal -- no sync
  p += z;                                    // axpy -- no sync
}
MatrixXd result = x.toHost();
```

### Precision control {#eigen_gpu_precision}

GEMM dispatch routes through `cublasLtMatmul`. The compute type is selected
per scalar via the `cuda_compute_type` trait in `CuBlasSupport.h`, gated by
two compile-time macros:

| Macro | Effect |
|---|---|
| (default) | `CUBLAS_COMPUTE_32F` / `CUBLAS_COMPUTE_64F`. cublasLt heuristics may pick tensor-core algorithms; on `sm_80+` doubles can land on Ozaki-emulated tensor cores. |
| `EIGEN_CUDA_TF32` | `CUBLAS_COMPUTE_32F_FAST_TF32` for `float` and `complex<float>` (~2x faster, 10-bit mantissa). No effect on `double` / `complex<double>`. |
| `EIGEN_NO_CUDA_TENSOR_OPS` | Pedantic compute types (`CUBLAS_COMPUTE_*_PEDANTIC`) for every scalar — disables tensor-core algorithms. Use for bit-exact reproducibility. Takes precedence over `EIGEN_CUDA_TF32`. |

These are independent of cuBLAS's runtime `cublasSetMathMode()` /
`CUBLAS_TF32_OVERRIDE` controls; the cublasLt path keys off the compile-time
compute type instead. The `cublasGemmEx` fallback (used when cublasLt's
heuristic returns no candidate) honors `EIGEN_NO_CUDA_TENSOR_OPS` via its
algorithm hint (`CUBLAS_GEMM_DEFAULT` vs `CUBLAS_GEMM_DEFAULT_TENSOR_OP`).

### Stream control and async execution

Operations are asynchronous by default. The compute-solve chain runs without
host synchronization until you need a result on the host:

```text
fromHost(A) --sync-->  compute() --async-->  solve() --async-->  toHost()
   H2D                  potrf                 potrs                D2H
                                                                   sync
```

Mandatory sync points:
- `fromHost()` -- Synchronizes to complete the upload before returning
- `toHost()` / `HostTransfer::get()` -- Must deliver data to host
- `info()` -- Must read the factorization status
- `DeviceScalar` implicit conversion -- Downloads scalar from device

Debug-only sync points (compiled out under `EIGEN_NO_DEBUG`/`NDEBUG`): every
solver `solve()` and accessor verifies `info() == Success` via `eigen_assert`,
which forces one stream synchronization the first time after each
`compute()`/`factorize()`. Release builds perform no such check — call
`info()` explicitly where failure detection matters. `gpu::SVD`'s device
solve additionally downloads the singular values once per (truncation,
lambda) setting to build its cached inverse diagonal.

**Cross-stream safety** is automatic. `DeviceMatrix` tracks write completion
via CUDA events. When a matrix written on stream A is read on stream B, the
module automatically inserts `cudaStreamWaitEvent`. Same-stream operations
skip the wait (CUDA guarantees in-order execution within a stream).

**Device memory allocation is stream-ordered.** All module allocations go
through `cudaMallocAsync` / `cudaFreeAsync` on devices that support memory
pools (detected at runtime; `cudaMalloc`/`cudaFree` fallback otherwise, or
force the fallback with `EIGEN_GPU_NO_STREAM_ORDERED_ALLOC` — required when
borrowing `cudaStreamNonBlocking` streams, which do not synchronize with the
legacy stream the allocator uses for ordering). Consequences:

- Allocating/destroying `DeviceMatrix` temporaries no longer performs a
  device-wide synchronization; freed blocks recycle through the driver pool
  (the pool's release threshold is raised so steady-state loops reallocate at
  user-space speed).
- Destroying a solver (or `DeviceMatrix`) with work still in flight is safe
  *and* async: the stream-ordered free waits for previously enqueued work
  without stalling the host.
- `DeviceMatrix::resize()` is capacity-aware: shrinking or same-size reshapes
  reuse the existing allocation (contents are still discarded).

## Reference

### Supported scalar types

`float`, `double`, `std::complex<float>`, `std::complex<double>` (unless
noted otherwise).

### Expression -> library call mapping

| DeviceMatrix expression | Library call | Parameters |
|---|---|---|
| `C = A * B` | `cublasLtMatmul` (with `cublasGemmEx` fallback) | transA=N, transB=N, alpha=1, beta=0 |
| `C = A.adjoint() * B` | `cublasLtMatmul` | transA=C, transB=N |
| `C = A.transpose() * B` | `cublasLtMatmul` | transA=T, transB=N |
| `C = A * B.adjoint()` | `cublasLtMatmul` | transA=N, transB=C |
| `C = A * B.transpose()` | `cublasLtMatmul` | transA=N, transB=T |
| `C = alpha * A * B` | `cublasLtMatmul` | alpha from LHS |
| `C = A * (alpha * B)` | `cublasLtMatmul` | alpha from RHS |
| `C += A * B` | `cublasLtMatmul` | alpha=1, beta=1 |
| `C -= A * B` | `cublasLtMatmul` | alpha=-1, beta=1 |
| `X = A.llt().solve(B)` | `cusolverDnXpotrf` + `Xpotrs` | uplo, n, nrhs |
| `X = A.llt<Upper>().solve(B)` | same | uplo=Upper |
| `X = A.lu().solve(B)` | `cusolverDnXgetrf` + `Xgetrs` | n, nrhs |
| `X = A.triangularView<L>().solve(B)` | `cublasXtrsm` | side=L, uplo, diag=NonUnit |
| `C = A.selfadjointView<L>() * B` | `cublasXsymm` / `cublasXhemm` | side=L, uplo |
| `C.selfadjointView<L>().rankUpdate(A)` | `cublasXsyrk` / `cublasXherk` | uplo, trans=N |
| `C = A + B` | `cublasXgeam` | alpha=1, beta=1 |
| `C = A + alpha * B` | `cublasXgeam` | alpha=1, beta from scaled |
| `C = A - B` | `cublasXgeam` | alpha=1, beta=-1 |
| `C = A - alpha * B` | `cublasXgeam` | alpha=1, beta=-scaled |
| `C = alpha * A + beta * B` | `cublasXgeam` | both sides scaled |
| `C = alpha * A`, `C = -A`, `C = A / alpha` | `cublasXgeam` | beta=0, aliasing-safe |
| `x += alpha * y` | `cublasXaxpy` | alpha (host scalar) |
| `x += dAlpha * y` | `cublasXaxpy` | alpha (DeviceScalar, device pointer mode) |
| `x -= alpha * y` | `cublasXaxpy` | alpha negated |
| `x *= alpha` | `cublasXscal` | alpha (host or DeviceScalar) |
| `x.dot(y)` | `cublasXdot` / `cublasXdotc` | returns `DeviceScalar` |
| `x.norm()` | `cublasXnrm2` | returns `DeviceScalar<RealScalar>` |
| `x.squaredNorm()` | `cublasXdot(x, x)` | returns `DeviceScalar<RealScalar>` |
| `d_y = view * d_x` | `cusparseSpMV` | device-resident SpMV |
| `d_Y = view * d_X` | `cusparseSpMM` | device-resident SpMM (RHS with >1 column) |

### `DeviceMatrix<Scalar>`

Typed RAII wrapper for a dense column-major matrix in GPU device memory.
Always dense (leading dimension = rows). A vector is a `DeviceMatrix` with
one column.

```cpp
// Construction
DeviceMatrix<Scalar>()                                   // Empty (0x0)
DeviceMatrix<Scalar>(Index n)                            // Allocate column vector (n x 1)
DeviceMatrix<Scalar>(rows, cols)                         // Allocate uninitialized
DeviceMatrix<Scalar>(expr)                               // Copy-init from any supported expression
                                                         // (GEMM, geam/scaled, LLT/LU solve, TRSM,
                                                         //  SYMM, SpMV/SpMM)

// Upload / download / pointer adoption
static DeviceMatrix fromHost(matrix, stream=nullptr)           // -> DeviceMatrix (syncs)
static DeviceMatrix fromHostAsync(ptr, rows, cols, stream)         // -> DeviceMatrix (no sync, caller manages ptr lifetime)
static DeviceMatrix adopt(Scalar* device_ptr, rows, cols)          // Owning wrapper over a raw device pointer
static DeviceMatrix view(Scalar* device_ptr, rows, cols)           // Non-owning view (does not free on destruction)
PlainMatrix        toHost(stream=nullptr)                      // -> host Matrix (syncs)
HostTransfer       toHostAsync(stream=nullptr)                 // -> HostTransfer future (no sync)
DeviceMatrix       clone(stream=nullptr)                       // -> DeviceMatrix (D2D copy, async)

// Dimensions and access
Index   rows()
Index   cols()
size_t  sizeInBytes()
bool    empty()
Scalar* data()                                           // Raw device pointer
void    resize(Index rows, Index cols)                   // Discard contents; keeps the allocation
                                                         // when it is already large enough

// Expression builders (return lightweight views, evaluated on assignment)
AdjointView       adjoint()                              // GEMM with ConjTrans
TransposeView     transpose()                            // GEMM with Trans
LltExpr            llt() / llt<UpLo>()                   // -> .solve(d_B) -> DeviceMatrix
LuExpr             lu()                                  // -> .solve(d_B) -> DeviceMatrix
TriangularView     triangularView<UpLo>()                // -> .solve(d_B) -> DeviceMatrix (TRSM)
SelfAdjointView    selfadjointView<UpLo>()               // -> * d_B (SYMM), .rankUpdate(d_A) (SYRK)
Assignment   device(gpu::Context& ctx)                // Bind assignment to explicit stream
DeviceMatrix&      noalias()                             // No-op (all ops are implicitly noalias)

// BLAS Level-1 (all have overloads with explicit gpu::Context& parameter)
DeviceScalar<Scalar>     dot(const DeviceMatrix& other)  // cuBLAS dot/dotc -> DeviceScalar
DeviceScalar<RealScalar> norm()                          // cuBLAS nrm2 -> DeviceScalar
DeviceScalar<RealScalar>  squaredNorm()                    // dot(self, self) -> DeviceScalar (no sync)
void                     setZero()                       // cudaMemsetAsync
void                     addScaled(gpu::Context&, Scalar alpha, const DeviceMatrix& x)  // this += alpha * x (axpy)
void                     scale(gpu::Context&, Scalar alpha)                              // this *= alpha (scal)
void                     copyFrom(gpu::Context&, const DeviceMatrix& other)              // this = other (D2D copy)
DeviceMatrix& operator+=(const Scaled<DeviceMatrix>&)    // axpy; spelled `mat += alpha * other`
DeviceMatrix& operator-=(const Scaled<DeviceMatrix>&)    // axpy negated; spelled `mat -= alpha * other`
DeviceMatrix& operator+=(const DeviceMatrix&)            // cuBLAS axpy (alpha=1)
DeviceMatrix& operator-=(const DeviceMatrix&)            // cuBLAS axpy (alpha=-1)
DeviceMatrix& operator+=(const DeviceScaledDevice<Scalar>&)  // axpy with device scalar; spelled `mat += d_alpha * other`
DeviceMatrix& operator-=(const DeviceScaledDevice<Scalar>&)  // negated; spelled `mat -= d_alpha * other`
DeviceMatrix& operator*=(Scalar)                         // cuBLAS scal (host pointer mode)
DeviceMatrix& operator*=(const DeviceScalar<Scalar>&)    // cuBLAS scal (device pointer mode, no host sync)
DeviceMatrix  cwiseProduct(gpu::Context&, const DeviceMatrix&)            // NPP nppsMul (float/double only)
void          cwiseProduct(gpu::Context&, const DeviceMatrix&, const DeviceMatrix&)  // in-place: this = a .* b

// geam expressions (evaluated on assignment)
DeviceMatrix& operator=(const DeviceAddExpr&)            // C = A + B, C = A + alpha*B, C = A - B, etc.
```

### `DeviceScalar<Scalar>`

Device-resident scalar. Returned by `dot()`, `norm()`, and `squaredNorm()`.
Implicit conversion to `Scalar` triggers `cudaStreamSynchronize` + download.

```cpp
DeviceScalar(cudaStream_t stream = nullptr)              // Allocate uninitialized
DeviceScalar(Scalar host_val, cudaStream_t stream)       // Upload host value

Scalar         get()                                     // Download (syncs stream)
               operator Scalar()                         // Implicit conversion (syncs)
Scalar*        devicePtr()                               // Raw device pointer
cudaStream_t   stream()

// Device-side arithmetic (no host sync, real types only)
DeviceScalar   operator/(DeviceScalar, DeviceScalar)     // NPP nppsDiv
DeviceScalar   operator/(Scalar, DeviceScalar)           // upload + div
DeviceScalar   operator/(DeviceScalar, Scalar)           // upload + div
DeviceScalar   operator-()                               // NPP nppsMulC(-1)
```

### `gpu::Context`

Unified GPU execution context owning a CUDA stream and library handles. Not
thread-safe -- use one `Context` per thread, or external synchronization
across threads.

```cpp
gpu::Context()                                             // Creates dedicated stream + cuBLAS handle
                                                           // (cuSOLVER / cuBLASLt / cuSPARSE handles
                                                           // are created lazily on first use)
gpu::Context(cudaStream_t stream)                          // Borrow existing stream (not owned)
static gpu::Context& threadLocal()                         // Per-thread default (lazy-created)
static void        setThreadLocal(gpu::Context* ctx)       // Override thread-local default (nullptr restores)

cudaStream_t       stream()
cublasHandle_t     cublasHandle()
cusolverDnHandle_t cusolverHandle()                        // Lazy: creates the handle on first call
cublasLtHandle_t   cublasLtHandle()                        // Lazy-initialized
cusparseHandle_t   cusparseHandle()                        // Lazy-initialized

internal::DeviceBuffer&          gemmWorkspace()            // cublasLtMatmul scratch (lazy-grown per context)
internal::CublasLtPlanCache&     gemmPlanCache()            // shape-keyed plan cache (per context, ~8-entry LRU)
internal::OneShotSolverScratch&  oneshotSolverScratch()     // LLT/LU expression scratch (lazy-grown per context)
```

Non-copyable, non-movable (owns library handles). Translation units that
never call `cusolverHandle()` do not pull cuSOLVER symbols at link time --
see [Linking](#eigen_gpu_linking).

### `gpu::LLT<Scalar, UpLo>` -- Dense Cholesky (cuSOLVER)

Caches the Cholesky factor on device for repeated solves.

```cpp
gpu::LLT()                                                // Default construct, then call compute()
gpu::LLT(Context& ctx)                                    // Bind to ctx's stream + handles
gpu::LLT(const DenseBase<D>& A)                           // Convenience: upload + factorize
gpu::LLT(const DeviceMatrix& d_A)                         // Convenience: D2D copy + factorize
gpu::LLT(DeviceMatrix&& d_A)                              // Convenience: adopt + factorize
gpu::LLT(Context& ctx, ...)                               // Bind + factorize in one step

gpu::LLT&            compute(const DenseBase<D>& A)       // Upload + factorize
gpu::LLT&            compute(const DeviceMatrix& d_A)     // D2D copy + factorize
gpu::LLT&            compute(DeviceMatrix&& d_A)          // Adopt + factorize (no copy)

PlainMatrix        solve(const MatrixBase<D>& B)         // -> host Matrix (syncs)
DeviceMatrix       solve(const DeviceMatrix& d_B)        // -> DeviceMatrix (async, stays on device)
DeviceMatrix       solve(DeviceMatrix&& d_B)             // In-place: consumes RHS, no copy/alloc

ComputationInfo    info()                                // Lazy sync on first call: Success or NumericalIssue
Index              rows() / cols()
cudaStream_t       stream()
```

### `gpu::LU<Scalar>` -- Dense LU (cuSOLVER)

Same pattern as `gpu::LLT`. Adds a `gpu::GpuOp` parameter on `solve()`.

```cpp
PlainMatrix        solve(const MatrixBase<D>& B, GpuOp op = GpuOp::NoTrans)  // -> host Matrix
DeviceMatrix       solve(const DeviceMatrix& d_B, GpuOp op = GpuOp::NoTrans) // -> DeviceMatrix
```

`gpu::GpuOp`: `NoTrans`, `Trans`, `ConjTrans`.

### `gpu::QR<Scalar>` -- Dense QR (cuSOLVER)

QR factorization via `cusolverDnXgeqrf`. Solve uses ORMQR (apply Q^H) + TRSM
(back-substitute on R) -- Q is never formed explicitly.

```cpp
gpu::QR()                                                  // Default construct
gpu::QR(const DenseBase<D>& A)                             // Convenience: upload + factorize

gpu::QR&             compute(const DenseBase<D>& A)        // Upload + factorize
gpu::QR&             compute(const DeviceMatrix& d_A)      // D2D copy + factorize

PlainMatrix        solve(const MatrixBase<D>& B)         // -> host Matrix (syncs)
DeviceMatrix       solve(const DeviceMatrix& d_B)        // -> DeviceMatrix (async)
PlainMatrix        matrixR()                             // -> host Matrix (m >= n only)

ComputationInfo    info()                                // Lazy sync
Index              rows() / cols()
cudaStream_t       stream()
```

### `gpu::SVD<Scalar>` -- Dense SVD (cuSOLVER)

SVD via `cusolverDnXgesvd`. Supports `ComputeThinU | ComputeThinV`,
`ComputeFullU | ComputeFullV`, or `0` (values only). Wide matrices (m < n)
handled by internal transpose.

```cpp
gpu::SVD()                                                 // Default construct, then call compute()
gpu::SVD(const DenseBase<D>& A, unsigned options = ComputeThinU | ComputeThinV)  // Convenience

gpu::SVD&            compute(const DenseBase<D>& A, unsigned options = ComputeThinU | ComputeThinV)
gpu::SVD&            compute(const DeviceMatrix& d_A, unsigned options = ComputeThinU | ComputeThinV)

RealVector         singularValues()                      // -> host vector (syncs, downloads)
PlainMatrix        matrixU()                             // -> host Matrix (syncs, downloads)
PlainMatrix        matrixV()                             // -> host Matrix (V = VT^H, matches JacobiSVD)
PlainMatrix        matrixVT()                            // -> host Matrix (syncs, downloads V^T)

DeviceMatrix       d_singularValues()                    // -> DeviceMatrix view (zero-copy)
DeviceMatrix       d_matrixU()                           // -> DeviceMatrix view (zero-copy when m >= n)
DeviceMatrix       d_matrixVT()                          // -> DeviceMatrix view (zero-copy when m >= n)

PlainMatrix        solve(const MatrixBase<D>& B)         // -> host Matrix (pseudoinverse)
PlainMatrix        solve(const MatrixBase<D>& B, Index k)       // Truncated (top k triplets)
PlainMatrix        solve(const MatrixBase<D>& B, RealScalar l)  // Tikhonov regularized
DeviceMatrix       solve(const DeviceMatrix& d_B)        // Device-resident pseudoinverse solve
DeviceMatrix       solve(const DeviceMatrix& d_B, Index k)      // Truncated, device-resident
DeviceMatrix       solve(const DeviceMatrix& d_B, RealScalar l) // Tikhonov, device-resident

Index              rank(RealScalar threshold = -1)
ComputationInfo    info()                                // Lazy sync
Index              rows() / cols()
cudaStream_t       stream()
```

**Note:** `singularValues()`, `matrixU()`, `matrixV()`, and `matrixVT()`
download to host on each call. The `d_*` accessors return non-owning
`DeviceMatrix` views into the solver's internal buffers; the `gpu::SVD` object
must outlive any view derived from it. For wide matrices (m < n) the U/V^T
views are owning (one `cublasXgeam` adjoint pass).

### `gpu::SelfAdjointEigenSolver<Scalar>` -- Eigendecomposition (cuSOLVER)

Symmetric/Hermitian eigenvalue decomposition via `cusolverDnXsyevd`.
`ComputeMode` enum: `EigenvaluesOnly`, `ComputeEigenvectors`.

```cpp
gpu::SelfAdjointEigenSolver()                              // Default construct, then call compute()
gpu::SelfAdjointEigenSolver(const DenseBase<D>& A, ComputeMode mode = ComputeEigenvectors)  // Convenience

gpu::SelfAdjointEigenSolver& compute(const DenseBase<D>& A, ComputeMode mode = ComputeEigenvectors)
gpu::SelfAdjointEigenSolver& compute(const DeviceMatrix& d_A, ComputeMode mode = ComputeEigenvectors)

RealVector         eigenvalues()                         // -> host vector (syncs, downloads, ascending order)
PlainMatrix        eigenvectors()                        // -> host Matrix (syncs, downloads, columns)

DeviceMatrix       d_eigenvalues()                       // -> DeviceMatrix view (zero-copy)
DeviceMatrix       d_eigenvectors()                      // -> DeviceMatrix view (zero-copy, requires ComputeEigenvectors)

ComputationInfo    info()                                // Lazy sync
Index              rows() / cols()
cudaStream_t       stream()
```

**Note:** `eigenvalues()` and `eigenvectors()` download to host on each call.
The `d_*` accessors return non-owning `DeviceMatrix` views into the solver's
internal buffers; the `gpu::SelfAdjointEigenSolver` object must outlive any
view derived from it.

### `HostTransfer<Scalar>`

Future for async device-to-host transfer. Returned by
`DeviceMatrix::toHostAsync()`.

```cpp
PlainMatrix&       get()                                 // Block until complete, return host Matrix ref. Idempotent.
bool               ready()                               // Non-blocking poll
```

### `gpu::SparseLLT<Scalar, UpLo>` -- Sparse Cholesky (cuDSS)

Requires cuDSS (CUDA 12.0+, `#define EIGEN_CUDSS`). Three-phase workflow
with symbolic reuse. Accepts `SparseMatrix<Scalar, ColMajor, int>` (CSC).
Matrix dimensions and nonzero count must fit in `int` (cuDSS limitation;
debug builds assert).

```cpp
gpu::SparseLLT()                                           // Default construct
gpu::SparseLLT(const SparseMatrixBase<D>& A)               // Analyze + factorize

gpu::SparseLLT&      analyzePattern(const SparseMatrixBase<D>& A)  // Symbolic analysis (reusable)
gpu::SparseLLT&      factorize(const SparseMatrixBase<D>& A)       // Numeric factorization
gpu::SparseLLT&      compute(const SparseMatrixBase<D>& A)         // analyzePattern + factorize

DenseMatrix        solve(const MatrixBase<D>& B)         // -> host Matrix (syncs)
DeviceMatrix       solve(const DeviceMatrix& d_B)        // -> DeviceMatrix (async, stays on device)

gpu::SparseLLT&      setConfig(const SparseSolverConfig&) // cuDSS knobs (>= 0.8); call before the affected phase
const SparseSolverConfig& config()                        // Last configuration set

ComputationInfo    info()                                // Lazy sync
Index              rows() / cols()
cudaStream_t       stream()
```

All three cuDSS solvers also accept a `gpu::Context&` as first constructor
argument to borrow its stream (`gpu::SparseLLT<double> llt(ctx)` or
`llt(ctx, A)`).

### `gpu::SparseLDLT<Scalar, UpLo>` -- Sparse LDL^T (cuDSS)

Symmetric indefinite. Same API as `gpu::SparseLLT`.

### `gpu::SparseLU<Scalar>` -- Sparse LU (cuDSS)

General non-symmetric. Same API as `gpu::SparseLLT` (without `UpLo`).

### `gpu::FFT<Scalar>` -- FFT (cuFFT)

Plans cached by (size, type) in a bounded LRU and reused; the
least-recently-used plan is destroyed via `cufftDestroy` on overflow. Cache
capacity is set at construction (default
`kDefaultCufftPlanCacheCapacity = 16`). Inverse transforms scaled so
`inv(fwd(x)) == x`. Supported scalars: `float`, `double`. Stream and cuBLAS
handle borrowed from a `gpu::Context` (default: `Context::threadLocal()`),
so by default the FFT shares a stream with other GPU operations on the same
thread.

```cpp
gpu::FFT(std::size_t plan_cache_capacity = kDefaultCufftPlanCacheCapacity)
                                        // bind to Context::threadLocal()
gpu::FFT(gpu::Context& ctx,
         std::size_t plan_cache_capacity = kDefaultCufftPlanCacheCapacity)
                                        // bind to an explicit Context

// 1D transforms (host vectors in and out)
ComplexVector      fwd(const MatrixBase<D>& x)           // C2C forward (complex input)
ComplexVector      fwd(const MatrixBase<D>& x)           // R2C forward (real input, returns n/2+1)
ComplexVector      inv(const MatrixBase<D>& X)           // C2C inverse, scaled by 1/n
RealVector         invReal(const MatrixBase<D>& X, Index n)  // C2R inverse, scaled by 1/n

// 2D transforms (host matrices in and out)
ComplexMatrix      fwd2(const MatrixBase<D>& A)         // 2D C2C forward
ComplexMatrix      inv2(const MatrixBase<D>& A)         // 2D C2C inverse, scaled by 1/(rows*cols)

// Device-resident transforms (DeviceMatrix in/out, async, no host transfer)
void               fwd(const DeviceMatrix<Complex>&, DeviceMatrix<Complex>&)      // 1D C2C
void               inv(const DeviceMatrix<Complex>&, DeviceMatrix<Complex>&)      // 1D C2C inverse
void               fwd(const DeviceMatrix<Scalar>&, DeviceMatrix<Complex>&)       // 1D R2C
void               invReal(const DeviceMatrix<Complex>&, DeviceMatrix<Scalar>&, Index nfft)  // 1D C2R
void               fwd2 / inv2(const DeviceMatrix<Complex>&, DeviceMatrix<Complex>&)  // 2D C2C

cudaStream_t       stream()             // borrowed from the bound Context
gpu::Context&      context()            // the bound Context
std::size_t        plan_cache_capacity() // configured capacity
std::size_t        plan_cache_size()     // currently cached plan count
```

All FFT methods accept host data and return host data. Upload/download is
handled internally. The C2C and R2C overloads of `fwd()` are distinguished by
the input scalar type (complex vs real).

### `gpu::SparseContext<Scalar>` -- SpMV/SpMM (cuSPARSE)

Accepts `SparseMatrix<Scalar, ColMajor>`. Host-input methods accept host data
and return host data; device-input methods (`deviceView()`, `multiply(A, d_x,
d_y)`) operate on `DeviceMatrix`. Matrix dimensions and nonzero count must fit
in `int` (cuSPARSE limitation; debug builds assert).

```cpp
gpu::SparseContext()                                       // Creates own stream + cuSPARSE handle
gpu::SparseContext(gpu::Context& ctx)                        // Borrow gpu::Context for same-stream execution

// Host data in/out
DenseVector        multiply(A, x)                                       // y = A * x
void               multiply(A, x, y, alpha=1, beta=0,                   // y = alpha*op(A)*x + beta*y
                     op=GpuOp::NoTrans)
DenseVector        multiplyT(A, x)                                      // y = A^T * x
DenseVector        multiplyAdjoint(A, x)                                // y = A^H * x
DenseMatrix        multiplyMat(A, X, op=GpuOp::NoTrans)                 // Y = op(A) * X (SpMM)

// DeviceMatrix in/out (sparse matrix re-uploaded per call)
void               multiply(A, d_x, d_y)                                // SpMV with device vectors
void               multiply(A, d_x, d_y, alpha, beta, op=GpuOp::NoTrans)

// Device-resident sparse matrix (upload once, reuse)
DeviceSparseView   deviceView(A)                                        // Upload sparse matrix, return view
uint64_t           uploadGeneration()                                   // Generation of the cached upload

// Advanced: run SpMV/SpMM against the already-uploaded matrix
void               spmv_device_exec(d_x, d_y, alpha=1, beta=0, op=GpuOp::NoTrans)
void               spmm_device_exec(d_X, d_Y, alpha=1, beta=0, op=GpuOp::NoTrans)

cudaStream_t       stream()
```

### `DeviceSparseView<Scalar>` -- Device-resident sparse matrix

Returned by `gpu::SparseContext::deviceView()`. Holds a sparse matrix on device
for repeated SpMV without re-uploading.

```cpp
SpMVExpr           operator*(const DeviceMatrix& d_x)    // d_y = view * d_x (evaluated on assignment;
                                                         // dispatches to SpMM when d_x has > 1 column)
uint64_t           generation()                          // Upload generation (stale views assert)
```

### Aliasing

Unlike Eigen's `Matrix`, where omitting `.noalias()` triggers a copy to a
temporary, DeviceMatrix dispatches directly to NVIDIA library calls which have
no built-in aliasing protection. All operations are implicitly noalias.
The caller must ensure operands don't alias the destination for GEMM, TRSM,
SYMM/HEMM, and SYRK/HERK. Debug builds assert on these violations before
dispatching to cuBLAS. `geam` expressions (`d_C = d_A + alpha * d_B`) are
safe with aliasing. The `.noalias()` method exists as a no-op for Eigen
template compatibility.

## Future work

- **Reassess host-input vs. device-input API surface.** Each solver currently
  exposes both host-input (`compute(MatrixXd)`, `solve(MatrixXd)`) and
  device-input (`compute(DeviceMatrix)`, `solve(DeviceMatrix)`) overloads, plus
  host- and device-side accessors (`matrixU()` vs `d_matrixU()`). This eases
  migration from CPU Eigen but may invite accidental host ↔ device round-trips
  when users mix the two without realising the cost. Revisit once the module
  is in users' hands; if the convenience overloads cause more confusion than
  they save, narrow toward a single explicit `fromHost` / `toHost` boundary.

- **Robustness-oriented cuDSS defaults.** `gpu::SparseSolverConfig` exposes
  matching, pivoting, refinement, and the hybrid modes, but a
  default-constructed solver still runs with cuDSS's performance-tuned
  defaults (matching off). Consider flipping the shipped defaults toward
  robustness now that users can override them.

- **cuDSS threading layer for host-side reordering.** As of cuDSS 0.7.1
  fill-reducing reordering runs on the CPU. cuDSS supports a "threading
  layer" plugin that parallelises this stage; for reordering-dominated
  problems it can materially close the gap with multithreaded CPU sparse
  direct solvers. We don't currently configure a threading layer.

- **Complex symmetric (non-Hermitian) sparse LDL^T.** `gpu::SparseLDLT`
  treats complex inputs as Hermitian (matching `Eigen::SimplicialLDLT`).
  cuDSS also supports `CUDSS_MTYPE_SYMMETRIC` for complex matrices
  (A = A^T, no conjugation); exposing this would need a separate solver
  mode.

## File layout

| File | Depends on | Contents |
|------|-----------|----------|
| `GpuSupport.h` | `<cuda_runtime.h>` | Error macro, `DeviceBuffer`, `DeviceBufferPool`, `cuda_data_type<>` |
| `DeviceMatrix.h` | `GpuSupport.h` | `gpu::DeviceMatrix<>`, `gpu::HostTransfer<>` |
| `DeviceExpr.h` | `DeviceMatrix.h` | GEMM, geam, and device-scalar expression wrappers |
| `DeviceBlasExpr.h` | `DeviceMatrix.h` | TRSM, SYMM, SYRK expression wrappers |
| `DeviceSolverExpr.h` | `DeviceMatrix.h` | Solver expression wrappers (LLT, LU) |
| `DeviceScalar.h` | `GpuSupport.h`, `DeviceScalarOps.h` | `gpu::DeviceScalar<>` (device-resident scalar) |
| `DeviceScalarOps.h` | `<npps_*.h>` | Scalar div/neg/cwiseProduct via NPP |
| `DeviceDispatch.h` | all above | All dispatch functions, BLAS-1 out-of-line defs, `gpu::Assignment` |
| `GpuContext.h` | `CuBlasSupport.h`, `CuSolverSupport.h` | `gpu::Context` |
| `CuBlasSupport.h` | `GpuSupport.h`, `<cublas_v2.h>`, `<cublasLt.h>` | cuBLAS error macro, type-specific wrappers |
| `CuSolverSupport.h` | `GpuSupport.h`, `<cusolverDn.h>` | cuSOLVER params, fill-mode mapping |
| `GpuSolverContext.h` | `CuSolverSupport.h`, `CuBlasSupport.h` | Shared solver context (stream, handles, scratch) |
| `GpuLLT.h` | `GpuSolverContext.h` | `gpu::LLT<>` -- Cached dense Cholesky factorization |
| `GpuLU.h` | `GpuSolverContext.h` | `gpu::LU<>` -- Cached dense LU factorization |
| `GpuQR.h` | `GpuSolverContext.h` | `gpu::QR<>` -- Dense QR decomposition |
| `GpuSVD.h` | `GpuSolverContext.h` | `gpu::SVD<>` -- Dense SVD decomposition |
| `GpuEigenSolver.h` | `GpuSolverContext.h` | `gpu::SelfAdjointEigenSolver<>` |
| `CuFftSupport.h` | `GpuSupport.h`, `<cufft.h>` | cuFFT error macro, type-dispatch wrappers |
| `GpuFFT.h` | `CuFftSupport.h`, `CuBlasSupport.h`, `GpuContext.h` | `gpu::FFT<>` -- 1D/2D FFT with plan caching |
| `CuSparseSupport.h` | `GpuSupport.h`, `<cusparse.h>` | cuSPARSE error macro |
| `GpuSparseContext.h` | `CuSparseSupport.h` | `gpu::SparseContext<>`, `gpu::DeviceSparseView<>` |
| `CuDssSupport.h` | `GpuSupport.h`, `<cudss.h>` | cuDSS error macro, type traits (optional) |
| `GpuSparseSolverBase.h` | `CuDssSupport.h` | CRTP base for sparse solvers (optional) |
| `GpuSparseLLT.h` | `GpuSparseSolverBase.h` | `gpu::SparseLLT<>` -- Sparse Cholesky via cuDSS (optional) |
| `GpuSparseLDLT.h` | `GpuSparseSolverBase.h` | `gpu::SparseLDLT<>` -- Sparse LDL^T via cuDSS (optional) |
| `GpuSparseLU.h` | `GpuSparseSolverBase.h` | `gpu::SparseLU<>` -- Sparse LU via cuDSS (optional) |

## Building and testing

```bash
cmake -G Ninja -B build -S . \
  -DEIGEN_TEST_CUDA=ON \
  -DEIGEN_CUDA_COMPUTE_ARCH="70" \
  -DEIGEN_TEST_CUBLAS=ON \
  -DEIGEN_TEST_CUSOLVER=ON

cmake --build build --target cublas cusolver_llt cusolver_lu \
  cusolver_qr cusolver_svd cusolver_eigen \
  device_matrix cufft cusparse_spmv cg
ctest --test-dir build -L gpu --output-on-failure

# Sparse solvers (cuDSS -- separate install required)
cmake -G Ninja -B build -S . \
  -DEIGEN_TEST_CUDA=ON \
  -DEIGEN_CUDA_COMPUTE_ARCH="70" \
  -DEIGEN_TEST_CUDSS=ON

cmake --build build --target cudss_llt cudss_ldlt cudss_lu
ctest --test-dir build -R '^cudss_' --output-on-failure
```

## Future enhancements

- **Batched API (`DeviceBatchMatrix`).** A strided batch of N identical-size
  matrices dispatching to cuBLAS/cuSOLVER batched APIs (`cublasDgemmBatched`,
  `cusolverDnXpotrfBatched`, etc.). This enables robotics and model-predictive
  control workloads where many small independent systems are solved in
  parallel.
- **cuTENSOR for Tensor module.** Replace the hand-written GPU tensor
  contraction and reduction kernels (~2300 lines in
  `TensorContractionGpu.h` / `TensorReductionGpu.h`) with cuTENSOR dispatch,
  following the same library-dispatch pattern used by `contrib/Eigen/GPU`.
- **Unified/zero-copy memory for Jetson.** Use `cudaMallocManaged` or
  `cudaHostAllocMapped` to eliminate `fromHost()` / `toHost()` copies on
  integrated GPUs (Jetson) where CPU and GPU share DRAM.
- **Device-side Eigen interop.** Bridge between host-side `DeviceMatrix`
  dispatch and device-side Eigen expression templates (Core + Tensor) running
  inside CUDA kernels. Raw-pointer + `Map` / `TensorMap` as the zero-copy
  interop surface.
- **Per-stream CUDA memory pools.** Allocation is now stream-ordered through
  the device's default memory pool. Attaching a dedicated `cudaMemPool_t` per
  stream (`cudaDeviceSetMempool` / `cudaMallocFromPoolAsync`) could further
  reduce cross-stream allocator contention for workloads that fan out many
  concurrent solves.
