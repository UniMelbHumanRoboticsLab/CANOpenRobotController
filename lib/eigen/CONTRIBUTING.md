# Contributing to Eigen

Eigen is written and maintained by volunteers. Contributions — code, documentation, bug triage, tests on uncommon platforms, design feedback — are all welcome.

This document is the human-facing on-ramp. [`AGENTS.md`](AGENTS.md) is the AI-facing repository contract and routes
readers to focused guides for testing, numerical work, performance, SIMD/GPU, Tensor/ThreadPool, and CI. Those guides
are also useful to human contributors working in the corresponding areas.

## Where Eigen lives

- **Upstream repository:** <https://gitlab.com/libeigen/eigen> — bugs, merge requests, and discussion all happen here. The GitHub mirror is read-only.
- **Issue tracker:** <https://gitlab.com/libeigen/eigen/-/issues>
- **CI pipelines:** <https://gitlab.com/libeigen/eigen/-/pipelines>
- **API documentation (nightly):** <https://libeigen.gitlab.io/eigen/docs-nightly>
- **Project site:** <https://libeigen.gitlab.io> (note: some pages predate current practice — when in doubt, prefer
  this file and the guide map in `AGENTS.md`)
- **Chat:** [Discord](https://discord.com/channels/777904510169382942/777904791136370758) — the primary support and discussion channel.

## Ways to contribute

You don't have to write C++ to help.

- **Answer questions** on Discord. New users often ask things that point at gaps in our documentation — patches to fill those gaps are very welcome.
- **Triage bugs.** Reproduce reports, ask reporters for missing details (compiler, OS, ISA, minimal example), and weigh in on issues labelled [`decision needed`](https://gitlab.com/libeigen/eigen/-/issues?label_name%5B%5D=decision+needed). Issues labelled [`contributions welcome`](https://gitlab.com/libeigen/eigen/-/issues?label_name%5B%5D=contributions+welcome) are good entry points.
- **Run the test suite** on rarely-tested compilers, OSes, or architectures and report failures. The nightly matrix is visible in the CI pipelines link above.
- **Improve documentation** — both the Doxygen reference (comments in headers) and the prose pages at <https://gitlab.com/libeigen/libeigen.gitlab.io>.
- **Contribute code** — bug fixes, new features, new tests, new benchmarks. See below.

## Submitting a merge request

1. **Sign in to GitLab** and fork <https://gitlab.com/libeigen/eigen>.
2. **Clone your fork** and add `libeigen/eigen` as an upstream remote (use the `https://` clone URL if you don't have an SSH key set up):
   ```bash
   git clone git@gitlab.com:<your-username>/eigen.git    # or: https://gitlab.com/<your-username>/eigen.git
   cd eigen
   git remote add upstream https://gitlab.com/libeigen/eigen.git
   ```
3. **Create a topic branch** off `master` for your change:
   ```bash
   git fetch upstream
   git checkout -b my-topic upstream/master
   ```
4. **Make your changes**, with tests (and a benchmark if it's a performance-sensitive change — see [Tests and benchmarks](#tests-and-benchmarks)).
5. **Format** with `clang-format-17` (see [Coding standards](#coding-standards)).
6. **Commit** with an `Area: short imperative subject` message (see [Commit messages](#commit-messages)). Stage files by path — don't use `git add -A`.
7. **Push** to your fork and open a merge request against `libeigen/eigen` `master`.
8. **Be patient and responsive.** Reviewers are volunteers; reviews can take a while. If your MR is being ignored for a long time, don't be shy about pinging it.

If you're planning a **large change** (new module, API redesign, significant refactor), open an issue or start a discussion first. Aligning on direction before you write code avoids wasted work for everyone.

## Build and test

Eigen is **header-only**, so consumers don't need to build the library itself — but contributors do need to build the test suite. CMake configure happens out-of-source, and tests are intentionally *not* in the default `all` target — drive everything through the named targets.

```bash
cmake -S . -B build -G Ninja
cmake --build build --target buildtests        # build all unit tests
cmake --build build --target check             # build + run all tests
cmake --build build --target buildsmoketests   # the smoke set CI gates MRs on
ctest --test-dir build --parallel --output-on-failure --no-tests=error
```

Other useful targets: `BuildOfficial` (just `test/`), `BuildContrib` (just `contrib/test/`), `buildtests_gpu`, `check_gpu`. Once configured, the build dir also exposes `./buildtests.sh <regex>` and `./check.sh <regex>` for narrowing by test-name regex.

For focused build recipes, test registration, split-test mechanics, and assertion guidance, see
[`.agents/testing.md`](.agents/testing.md). Packet and accelerator validation is covered by
[`.agents/simd-gpu.md`](.agents/simd-gpu.md). The checked-out CMake files remain authoritative for current options.

> **In-flight migration:** [MR 2159](https://gitlab.com/libeigen/eigen/-/merge_requests/2159) is migrating the test framework from Eigen's own `EIGEN_DECLARE_TEST` / `CALL_SUBTEST_N` macros to Google Test. Until it lands, write new tests with the existing framework (`test/main.h`, `VERIFY_*`, `EIGEN_DECLARE_TEST`, `ei_add_test` in the matching `CMakeLists.txt`).

## Tests and benchmarks

- **Every new feature lands with tests.** Bug fixes should add a regression test that fails before the fix and passes after.
- **Performance-sensitive changes land with a benchmark** under `benchmarks/` or `contrib/benchmarks/`. Don't defer benchmarks to a follow-up MR.
- **Cover numerical corner cases**, not just typical inputs: ±0, ±∞, NaN, subnormals, domain boundaries, near-overflow / underflow values. Decomposition and solver tests should exercise ill-conditioned and structured matrices (Hilbert, Vandermonde, rank-deficient, near-singular, Jordan blocks, …). The bar is matching LAPACK on conditioning, pivoting strategy, and backward stability. Standard references: Higham's *Accuracy and Stability of Numerical Algorithms* / *Functions of Matrices*, Golub & van Loan's *Matrix Computations*.
- **New SIMD intrinsics need a `test/packetmath.cpp` entry** (or `contrib/test/special_packetmath.cpp` for special functions), and a specialization in every architecture backend that supports the type. `Eigen/src/Core/arch/Default/` holds generic SIMD implementations shared across backends; scalar fallbacks live in `Eigen/src/Core/GenericPacketMath.h` and `Eigen/src/Core/MathFunctions.h`.
- **Benchmarking discipline.** Benchmarks on a loaded system are meaningless. Before running one: check `uptime` shows a low load average, finish or cancel background builds, and never run two benchmark binaries in the same shell invocation (parallel or chained with `&&`). Run each in its own shell, take medians within a binary, and optionally alternate (A, B, A, B) across separate invocations to detect drift.

## Coding standards

Eigen has a few hard rules that catch contributors most often. The repository-wide rules are in
[`AGENTS.md`](AGENTS.md#non-negotiable-rules), which also routes to task-specific guidance. The short version:

1. **Header-only contract.** Never `#include` anything under `Eigen/src/...` or `contrib/Eigen/src/...` — `InternalHeaderCheck.h` makes that a hard compile error. User code reaches implementation only through the umbrella headers (`Eigen/Core`, `Eigen/Dense`, `Eigen/SVD`, …).
2. **Preserve `EIGEN_DEVICE_FUNC`** on coefficient-level methods. Dropping it silently breaks CUDA / HIP / SYCL builds and rarely shows up in local testing.
3. **Format with `clang-format-17` exactly** — Google base, 120 columns, configured in `.clang-format`. Newer or older
   `clang-format` will diff against CI. CI checks changed lines (`git clang-format --diff --commit <base>`), and the
   tree is not uniformly clang-format-17 clean. Inspect the selected files' diffs to exclude unrelated changes, then
   run `git clang-format --binary clang-format-17 --force <base-sha> -- <files>`; `--force` permits unstaged edits.
   Format new, untracked files with `clang-format-17 -i <files>` because Git's diff omits them. Follow the checks in
   [`.agents/ci.md`](.agents/ci.md). `scripts/format.sh` is for an intentional whole-tree pass.
4. **Don't apply general C++ "modernize" cleanups.** No `modernize-*` / `cppcoreguidelines-*` clang-tidy fixes, no replacing `EIGEN_STRONG_INLINE` with `inline`, no reordering includes (`.clang-format` has `SortIncludes: false`). Eigen's conventions are encoded in `.clang-format`'s `StatementMacros` and `AttributeMacros` lists — don't "fix" their indentation.
5. **Tests use `test/main.h`, not gtest** (today — see migration note above). `VERIFY_*` assertions, `CALL_SUBTEST_N` / `EIGEN_TEST_PART_N` for splitting, `EIGEN_DECLARE_TEST(<name>) { ... }` as the entry point.
6. **`auto` traps.** `auto x = A + B;` captures a lazy expression holding references that can dangle. Use `.eval()` or an explicit type. Similarly, `.noalias()` is a promise from the caller — only use it when the destination doesn't appear on the right side (so `mat.noalias() = mat * mat` is **wrong**).
7. **Stage commits explicitly.** Don't `git add -A` / `git add .` — the repo root accumulates `.vscode/`, `.idea/`, `build*/`, and other untracked files that must not enter commits. Add by path or with targeted globs.

Naming conventions:
- Classes: PascalCase (`Matrix`, `BDCSVD`).
- Methods: camelCase (`coeffRef`, `applyOnTheLeft`).
- Macros and compile-time constants: `EIGEN_UPPER_CASE` (`EIGEN_DEVICE_FUNC`, `EIGEN_STRONG_INLINE`).
- Internal implementation lives in the `Eigen::internal` namespace; public API stays in `Eigen::` or module namespaces.
- Use `eigen_assert(cond)` for runtime preconditions (not raw `assert`) and `eigen_internal_assert` for internal-only
  invariants gated on `EIGEN_INTERNAL_DEBUGGING`. Compile-time API preconditions use the appropriate
  `EIGEN_STATIC_ASSERT_*` helper or `EIGEN_STATIC_ASSERT(cond, MSG_TOKEN)` to honor `EIGEN_NO_STATIC_ASSERT` and user
  overrides. Use `static_assert(cond, "message")` for unconditional implementation invariants.

The canonical class layout is visible in recent additions such as `contrib/Eigen/src/GPU/DeviceMatrix.h`. Template parameters that get re-exported as public aliases carry a trailing underscore (`template <typename Scalar_, ...>` paired with `using Scalar = Scalar_;` — prefer `using` over `typedef` in new code), member variables use an `m_` prefix (`m_matrix`, `m_isInitialized`), implementation headers under `Eigen/src/...` begin with `// IWYU pragma: private` and `#include "./InternalHeaderCheck.h"`, header guards follow `EIGEN_<NAME>_H`, and implementation detail lives inside nested `namespace Eigen { namespace internal { ... } }`. Public classes carry Doxygen blocks (`\class`, `\brief`, `\tparam`, `\sa`) and link to runnable snippets from `doc/snippets/` via `\include` / `\verbinclude`. When in doubt, copy the style from a recent neighbouring file rather than inventing a new convention.

### License and SPDX/REUSE headers

Every new source file must carry an inline copyright + license header. The `checkformat:reuse` CI job (`reuse lint`) blocks otherwise. Two header styles are in active use; pick whichever fits.

Attributed to an individual contributor:

```cpp
// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) <year> <Your Name> <your.email@example.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0
```

Attributed collectively (common for files written by multiple contributors, or where individual attribution isn't useful):

```cpp
// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) <year> The Eigen Authors.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0
```

Markdown docs, generated files (`*.in`), and binary assets that can't carry inline headers are covered by path annotations in `REUSE.toml` — add your path there if you're creating one.

### Provenance and attribution

This is non-negotiable.

- Eigen code must be **original**, or derived from publicly published **MPL-2.0-compatible** material.
- **Do not copy** — verbatim, paraphrased, or "translated" to another syntax — from incompatibly-licensed sources (proprietary, NDA-encumbered, prior-employer internal, GPL/AGPL). Paraphrasing is still a derivative work. The same discipline applies to AI-suggested code: cite the source, or rewrite from a known reference and cite that, or drop it. Ideas aren't copyrightable; specific expressions are — learn from the source, write your own, and credit it.
- **Cite published references inline** when they inform an implementation: LAPACK / LAWN, ACM TOMS / SIAM papers, Higham, Golub & van Loan, textbook algorithms, Boost components, vendor application notes — by name (author, year, identifier). A comment or Doxygen `\note` block is enough.
- When copying permissively-licensed code into an MPL-2.0 file, follow Mozilla's [Guidelines for Developers](https://www.mozilla.org/MPL/2.0/permissive-code-into-mpl.html).

### Copyright and credit

- **No copyright assignment.** Everyone keeps copyright on their own contributions. There is no CLA.
- **Commit under your own identity.** Make sure `user.name` and `user.email` in your git config are set to credit you correctly.
- When contributing on behalf of your employer, **commit in your own name** where possible rather than your employer's.
- For substantial improvements to a source file, feel free to add yourself to its copyright line — but it isn't required; you get copyright implicitly on the code you wrote. The collective `The Eigen Authors` attribution is fine when individual credit isn't useful.

## Commit messages

Format: `Area: short imperative subject`.

Examples (from recent history):
- `BDCSVD: fix edge case`
- `GPU: Drop direct LruCache.h include from CuBlasSupport.h`
- `ci: drop CodeRabbit summary`
- `TriangularView: alias-aware fallback for structured-diagonal product fast path`

Keep the subject under ~72 characters; use the body for the *why*, not the *what* — the diff already shows what changed. Reference issues by `#<n>` when relevant.

## Quality bar

Eigen aspires to state-of-the-art on two axes — **performance** and **numerical accuracy / IEEE-754 conformance** — and treats them as separate goals that are sometimes in tension.

- **Performance.** Eigen Core is mostly optimized for single-core throughput via per-architecture packet backends (SIMD) and cache-aware blocking. Multi-core in Core is opt-in (OpenMP or `EIGEN_GEMM_THREADPOOL`) and covers a subset of operations. The Tensor module is multi-core by design via `ThreadPoolDevice`.
- **Numerical accuracy.** LAPACK-level for linear algebra (decompositions, solvers — backward stability, pivoting, conditioning) and C++ standard-library level for standard math functions on scalars. On regular inputs, a few ULPs of error in vectorized math is the long-standing trade-off for SIMD throughput; larger deviations need explicit justification.
- **IEEE-754 / special values.** For special values (NaN, ±0, ±∞, subnormals, singularities, branch boundaries — e.g. `log(0)`, `pow(0, 0)`) the bar is **exact conformance** to IEEE 754 / ISO C / C++ specifications, with cppreference as the authoritative spec. Special-value handling is **not** subject to the few-ULPs trade-off.

For decompositions and solvers specifically, the bar is matching LAPACK on conditioning, pivoting strategy, and backward stability. Don't trade numerical robustness for speed in those code paths without explicit sign-off.

## Load-bearing modules

Some `contrib/` modules carry stability guarantees beyond what the directory name suggests:

- **`contrib/Eigen/Tensor`** and **`Eigen/ThreadPool`** together form TensorFlow's core compute backend. The `contrib/` location means *looser API-stability guarantees* — it does **not** mean low-traffic or low-stakes. Breaking changes to header layout, signatures, semantics, or contraction / reduction kernel performance ripple into every TensorFlow build. Prefer additive changes, keep header paths stable, run `contrib/test/tensor_*` before submitting, and call out any behaviour change prominently in the MR description.

## Communication

- **Discord** is the primary chat channel ([link above](#where-eigen-lives)). Most informal design discussion happens there.
- **GitLab issues** are the place for bug reports, feature requests, and threaded design discussions tied to a specific topic.
- For larger contributions, **discuss the plan first** to avoid duplicated effort and to align on API decisions before implementation.

## Further reading

- [`AGENTS.md`](AGENTS.md) — repository-wide agent contract, standard workflow, essential Eigen hazards, and the map
  to task-specific guidance.
- Task guides: [testing](.agents/testing.md), [numerical code](.agents/numerics.md),
  [benchmarking](.agents/benchmarking.md), [SIMD/GPU](.agents/simd-gpu.md),
  [Tensor/ThreadPool](.agents/tensor-threadpool.md), [documentation](.agents/docs.md), and
  [formatting/CI](.agents/ci.md).
- [`README.md`](README.md) — high-level project description and pointers to the websites.
- [`CHANGELOG.md`](CHANGELOG.md) — release-by-release notes.
- API reference (nightly): <https://libeigen.gitlab.io/eigen/docs-nightly>.

Thank you for contributing to Eigen!
