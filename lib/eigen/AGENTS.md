# AGENTS.md

Guidance for AI coding agents working in Eigen. Human contributors should start with
[`README.md`](README.md) and the project documentation it links to. Per-tool files such as `CLAUDE.md` should import
this file and contain only tool-specific additions.

## Scope and precedence

Follow the user's task, then the nearest applicable `AGENTS.md`, then repository documentation and established local
patterns. The checked-out source, tests, CMake files, and CI configuration are authoritative for current mechanics. If
this guide disagrees with the tree, follow the tree, report the discrepancy, and update the guidance when that is in
scope.

Read this file for every task. Then read every row below that matches the work; do not load unrelated guides by
default.

| Work area | Additional guidance |
|---|---|
| Any new or rewritten code | [`.agents/conventions.md`](.agents/conventions.md) |
| Tests and CMake test targets | [`.agents/testing.md`](.agents/testing.md) |
| Numerical kernels, decompositions, solvers, accuracy | [`.agents/numerics.md`](.agents/numerics.md) |
| Sparse matrices, sparse solvers, external sparse backends | [`.agents/sparse.md`](.agents/sparse.md) |
| Performance changes and benchmarks | [`.agents/benchmarking.md`](.agents/benchmarking.md) |
| Packet math, CUDA, HIP, SYCL, `contrib/Eigen/GPU` | [`.agents/simd-gpu.md`](.agents/simd-gpu.md) |
| Tensor, ThreadPool, and multithreading | [`.agents/tensor-threadpool.md`](.agents/tensor-threadpool.md) |
| Formatting, lint, and GitLab CI | [`.agents/ci.md`](.agents/ci.md) |
| Doxygen blocks, `doc/` pages, snippets, and examples | [`.agents/docs.md`](.agents/docs.md) |
| Changes under `ci/`, `.gitlab-ci.yml`, or the test-selection and cache scripts | [`.agents/ci-internals.md`](.agents/ci-internals.md) |
| Writing or updating a merge request description | [`.agents/merge-requests.md`](.agents/merge-requests.md) |
| Answering merge request review comments | [`.agents/review-response.md`](.agents/review-response.md) |
| Expression templates or evaluator internals | [`doc/TopicLazyEvaluation.dox`](doc/TopicLazyEvaluation.dox), [`doc/NewExpressionType.dox`](doc/NewExpressionType.dox), and [`doc/ClassHierarchy.dox`](doc/ClassHierarchy.dox) |

## Non-negotiable rules

1. **Preserve existing work.** Start with `git status --short`. Never discard, overwrite, reformat, or stage unrelated
   user changes. Do not use destructive Git commands unless the user explicitly requests that operation. Stage named
   paths, never `git add .` or `git add -A`.
2. **Keep provenance clean.** Code must be original or derived from source material whose license is compatible with
   Eigen's MPL-2.0 distribution. Do not copy, paraphrase, or translate code from proprietary, NDA-covered, internal, or
   incompatibly licensed sources. Published papers, standards, textbooks, and algorithm descriptions may inform an
   independent implementation; cite them inline when they materially inform it. A citation does not make copied code
   permissible. Never invent an attribution for AI-generated code; a `Co-Authored-By` trailer naming the model that
   actually produced the change is accurate attribution, not an invented one, and is permitted.
3. **Respect the header-only and C++14 contracts.** Supported headers must compile as C++14 unless a guarded backend has
   a documented newer requirement. User code, examples, and public-behavior tests include umbrella headers such as
   `Eigen/Core` or `Eigen/SVD`, not files below `Eigen/src/` or `contrib/Eigen/src/`. Focused tests of private
   utilities may follow an established direct-include pattern, but those paths remain private even where a header is not
   mechanically guarded. Definitions in public headers must have valid header linkage and avoid ODR violations.
4. **Protect compatibility.** Treat supported public names, signatures, header paths, semantics, and ABI-affecting
   configuration as compatibility surfaces. Prefer additive changes and deprecation over removal. When moving private
   implementation headers, update the public umbrella and remove the old private file rather than adding a private-path
   forwarding shim. ABI-affecting Eigen macros must be consistent across translation units.
5. **Preserve Eigen annotations and style.** Do not drop `EIGEN_DEVICE_FUNC` from coefficient-level or device-callable
   functions. Do not replace `EIGEN_STRONG_INLINE` with `inline`, reorder includes, normalize Eigen macro layout, or
   apply broad `modernize-*` or `cppcoreguidelines-*` rewrites. The repository's conventions and `.clang-format` take
   precedence over generic C++ advice. This protects code you are not otherwise changing; it does not license writing
   new code in a superseded form. Write new declarations in the form
   [`.agents/conventions.md`](.agents/conventions.md) records.
6. **Gate a fast path on the property it needs.** State the exact precondition a new specialization, capability flag,
   or enable condition depends on and test for that, not for an adjacent capability, an overload's existence, or a
   property the built-in types merely happen to share. New opt-in traits default to the conservative answer;
   user-specializable extension points must stay correct while unannotated.
7. **Ship verification with behavior.** New functionality includes focused tests. Bug fixes include a regression test
   that fails without the fix when practical. Performance-sensitive changes include an appropriate benchmark. Scale
   broader coverage to the affected scalar types, storage orders, backends, and public contracts. Confirm the new test
   fails at the parent commit when practical; otherwise demonstrate that it reaches the changed path by construction.
   See [`.agents/testing.md`](.agents/testing.md).
8. **Treat external writes as deliberate actions.** Unless the user already asked for them, pause after the local commit
   before pushing, opening or updating a merge request, commenting on an issue, or making another external-system write.
   Recommend `affected-tests` with the relevant platform labels, or with `all-platforms` for broader coverage; see
   [`.agents/ci.md`](.agents/ci.md). Do not add `all-tests` without the user's explicit permission for that label.

## Standard workflow

1. Inspect `git status --short`, the current branch, and the diff. Separate pre-existing work from the requested change.
2. As applicable, read the public header, implementation, nearby tests, registration in `CMakeLists.txt`, and relevant
   task guides before deciding on an implementation. Search with `rg` or `rg --files` (`--hidden` reaches `.agents/`).
   Before writing a helper, check `numext`, `NumTraits`, `MathFunctions.h`, `Meta.h`, `XprHelper.h`, and the
   `test/*_helpers.h` headers for an existing one; if it exists but lacks needed hardening, fix it there rather than
   adding a local copy.
3. Keep the patch within the owning module and established patterns. Avoid opportunistic refactors and generated or
   metadata churn.
4. Add or update applicable tests and benchmarks in the same patch. Test public behavior through its umbrella header so
   missing exports are caught; follow nearby patterns for focused private-internal tests.
5. Format the task's changed lines with `git clang-format --binary clang-format-17 --force <base-sha> -- <files>`.
   Inspect the selected files' diffs first to exclude unrelated changes; `--force` permits unstaged edits. Untracked
   files are absent from the diff, so format task-created files with `clang-format-17 -i <files>`. Whole-file formatting
   of existing files and `scripts/format.sh` also rewrite pre-existing lines that are not clang-format-17 clean, so use
   them only when that churn is intended. See [`.agents/ci.md`](.agents/ci.md) for the matching check.
6. Build and run the narrowest relevant test first, then widen validation according to the change's risk. Use separate
   build directories for materially different CMake configurations.
7. Review `git diff --check`, `git diff`, and `git status --short`. Report the exact validation run and any unavailable
   compiler, ISA, GPU, dependency, or downstream coverage.
8. When review comments arrive, follow [`.agents/review-response.md`](.agents/review-response.md).

## Repository essentials

Eigen is a header-only expression-template library. Consumers include module headers under `Eigen/` or
`contrib/Eigen/`. The top-level CMake project builds tests, documentation, demos, and BLAS/LAPACK shims rather than
a core Eigen library; benchmarks use separate CMake projects. `Eigen/Dense` aggregates the dense modules, while
`Eigen/Eigen` includes `Dense` and `Sparse`. External backend support modules and `Eigen/ThreadPool` remain separate
includes. The upstream project is on GitLab; its GitHub repository is a read-only mirror.

The supported implementation is under `Eigen/src/`; tests are under `test/`. Modules with looser API-stability
guarantees are under `contrib/Eigen/`, with tests under `contrib/test/`. Legacy `unsupported/Eigen/...` include paths
remain valid: one-line forwarding shims under `unsupported/Eigen/` point at the `contrib/` headers and are installed
alongside them. "Contrib" does not imply low impact: Tensor is a foundational TensorFlow dependency. Public umbrella
headers are the source of truth for a module's exported internals.

The `lapack/*.f` files are vendored netlib LAPACK reference sources and are read-only here: do not edit them ad hoc,
and flag a merge request that changes one unless it is an explicit refresh from a named netlib release, in which case
check the diff against that release. Tree-wide clang-format and SPDX-tagging commits are listed in
`.git-blame-ignore-revs`; pass that file to `git blame` with `--ignore-revs-file` to see the history beneath them.

Every new source file needs accurate REUSE metadata; [`.agents/conventions.md`](.agents/conventions.md) records the
required header form and the `REUSE.toml` rules for files that cannot carry an inline tag.

## Essential Eigen hazards

### Expressions, lifetimes, and aliasing

Eigen expressions are lazy and frequently retain references. Consumption can occur through assignment, construction,
coefficient access, reductions, or `.eval()`.

- `auto x = A + B;` stores a lazy expression whose references may dangle. Materialize with `(A + B).eval()` or use an
  appropriate plain-object type when ownership is required.
- `.noalias()` is a promise, not a runtime check. Use it only when the destination cannot appear in the right-hand side.
  `mat = mat * mat` is protected by product evaluation; `mat.noalias() = mat * mat` is wrong.
- Prefer Eigen expressions when they express the operation clearly and avoid repeated evaluation. Keep a scalar loop
  when it represents control flow better, avoids an unnecessary temporary, or has measured performance benefits.
- Prefer block and view expressions when a uniform operation or existing Eigen method applies to a submatrix; for
  example, scale a 2-by-2 block or call its `determinant()` instead of spelling out its coefficients. Preserve known
  extents with fixed-size accessors such as `block<Rows, Cols>(i, j)`; in dependent template code, write
  `m.template block<Rows, Cols>(i, j)`. Use runtime extents only when they are genuinely dynamic, and use individual
  coefficient access when entries require different operations. Blocks remain lazy, non-owning views, so the lifetime
  and overlap rules above still apply.
- The two arms of `?:` must have a common C++ type; distinct Eigen expression types often do not. Use `if`/`else` when
  necessary.
- Declare dynamically sized matrix and vector workspaces outside the loop that fills them: a plain object named inside
  the loop body allocates on every iteration, as does every subexpression that materializes a temporary into it.

### Scalar, index, and storage genericity

Use `Eigen::Index` for dimensions and counts, but remember that its underlying type is configurable. Use `NumTraits` for
scalar properties and Eigen's `numext` helpers when custom-scalar or device support matters. Do not store sizes or loop
counts in `Scalar`, hard-code `float`/`double` without an API reason, or narrow to a vendor API's `int` without checking
the range. Test real, complex, integer, and narrow/custom scalar types according to the operation's documented domain.
An algebraic property that holds for the built-in types — commutativity, exactness, tie behavior of `min`/`max` — is
not a property of every `Scalar`; establish it per scalar category and leave custom scalars on the conservative path.

Propagate storage-order and expression flags deliberately. `RowMajorBit`, fixed versus dynamic dimensions, alignment,
and vectorization eligibility affect evaluators and fast paths. Eigen alignment depends on configuration and
architecture; do not encode a presumed byte value. Include configuration-sensitive behavior in tests when it changes
semantics or ABI.

### Public APIs and diagnostics

For generic APIs, accept the least restrictive established Eigen base (`EigenBase`, `DenseBase`, `MatrixBase`,
`ArrayBase`, or a suitable `Ref`) that preserves the intended semantics. Follow nearby established patterns for writable
expression arguments; do not cast away constness from genuinely const storage. Public-header additions with non-template
definitions or objects deserve a multiple-translation-unit link test when an ODR regression is plausible.

The supported C++14 configurations cannot rely on C++17 over-aligned value passing. Pass fixed-size vectorizable Eigen
objects by reference rather than by value; see [`doc/PassingByValue.dox`](doc/PassingByValue.dox).

Use `eigen_assert` for runtime preconditions that belong to Eigen's public debug behavior and `eigen_internal_assert`
for internal invariants gated by `EIGEN_INTERNAL_DEBUGGING`. Use the local compile-time assertion style that gives the
clearest diagnostic. Comments should explain non-obvious mathematics, invariants, compatibility constraints, or
provenance rather than narrating the code. Keep comments concise and proportional to the code's complexity. Avoid
tutorial-style prose, section-by-section narration, and comments that restate identifiers or control flow. Longer
comments are justified only when that rationale cannot be expressed clearly in code. Reviewers here read mathematics
and code faster than English: where a formula, a recurrence, an error bound, or two lines of pseudo-code state the
point more precisely than a paragraph, write that instead. The same preference applies to merge request descriptions
and review comments; [`.agents/merge-requests.md`](.agents/merge-requests.md) records the KaTeX syntax GitLab renders.

## Quick build and test

By default, tests are not part of the `all` target, although that target may build configured auxiliary libraries. A
typical focused workflow is:

```bash
cmake -G Ninja -S . -B build
cmake --build build --target <test-name>
ctest --test-dir build -R '^<test-name>$' --output-on-failure --no-tests=error
```

For a split test such as `foo_3`, build that exact target and match it exactly with CTest, keeping `--no-tests=error`:
a filter that matches nothing otherwise exits 0. The generated `buildtests.sh` and `check.sh` wrappers accept
source/test-name regexes and are useful for building all matching parts.
Use `buildtests`, `BuildOfficial`, `BuildContrib`, `buildsmoketests`, or `check` only when the requested validation
warrants that scope. See [`.agents/testing.md`](.agents/testing.md) for the current test framework, split rules,
configuration variants, and failure-test workflow.

## Completion checklist

Before declaring the task complete:

- The diff contains only intentional changes and preserves pre-existing work.
- New public implementation is reachable through the intended umbrella header.
- New files have correct REUSE metadata and no generated or local-tool files are staged.
- Changed source lines and task-created source files pass the clang-format-17 checks in `.agents/ci.md`;
  `git diff --check` is clean.
- Documentation describing the changed behavior — the Doxygen block above a changed declaration, the module `README`,
  and nearby comments naming a value or precondition the change moved — is updated with it.
- Focused regression tests pass, with broader tests or benchmarks run when the risk warrants them.
- Numerical, aliasing, scalar, storage-order, device, threading, and ABI implications have been considered where
  relevant.
- The final report names validation performed, residual risk, and anything that could not be tested locally.

Commit subjects normally use `Category: Short description`, for example
`Core: Fix alias handling in product assignment`.
