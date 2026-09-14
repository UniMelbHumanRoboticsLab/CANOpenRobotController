#!/usr/bin/env python3
# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

"""Select the tests affected by a set of changed files.

Eigen is header-only, so a test is affected by a change exactly when its
translation unit textually includes the changed file.  This script builds the
include graph over ``Eigen/``, ``contrib/Eigen/``, ``unsupported/Eigen/`` and the test trees, then
maps changed paths to the CMake test targets that reach them.

The graph follows every ``#include`` regardless of preprocessor guards, so the
closure is a strict superset of the true compile dependency and no test is
dropped because a conditional branch was not taken.  Over-approximation is the
safe direction here: the point is to widen coverage relative to the fixed smoke
list, not to minimise work.  Changes that invalidate the mapping itself (CMake,
CI, the BLAS/LAPACK shims) fall back to the full ``buildtests`` target.

Two output files are written, both consumed by the build and test scripts of each
platform in the tier -- ``ci/scripts/build.linux.script.sh`` and
``ci/scripts/test.linux.script.sh``, and their ``.windows.script.ps1``
counterparts:

  targets.txt       ``NONE``, a newline-separated target list, or the
                    full-suite list of ``buildtests`` and the targets it does
                    not aggregate
  ctest_regex.txt   ``ALL``, ``NONE``, or a CTest ``-R`` regex

The selected names are CMake target names, not CTest test names: a split test
``foo`` registers ``foo_1``..``foo_N`` as tests but a single ``foo`` target that
aggregates them, so selecting ``foo`` builds and runs every part.  Targets that
a given configuration does not register (optional dependencies such as CHOLMOD
or SYCL) are filtered out by the build scripts, the only place that knows what
CMake actually configured.

Two registrations do not fit that shape:

* Each compile-failure test under ``failtest/`` compiles its own target from
  inside CTest, so those are selected as CTest names and never handed to the
  build job.
* ``buildtests`` aggregates the ``ei_add_test`` targets only.  A bare
  ``add_executable`` such as ``bug1213`` is attached to nothing, so the
  full-suite mode has to name those targets next to ``buildtests``.

A registered target need not compile a ``.cpp``, and need not be named by a
literal: ``ei_add_test`` takes the source extension from
``EIGEN_ADD_TEST_FILENAME_EXTENSION``, which the GPU blocks set to ``cu``, and
the GPU module registers families of tests from ``foreach`` item lists.  Both
are read out of the CMake source, so a translation unit under a test root with
no registration the parser can see is an error rather than an assumption.
Those targets are selected like any other; the build script drops them in
configurations that did not register them.  ``test/buildsystem/`` is skipped:
its consumer projects are separate CMake projects configured by their own CI
job, so an ``add_executable`` there is not a test registration.
"""

import argparse
import collections
import fnmatch
import os
import re
import subprocess
import sys

# Directories scanned to build the include graph.
# unsupported/Eigen holds only the legacy umbrella shims now, but they are
# still headers a change can propagate through, so the graph keeps them.
SCAN_ROOTS = ("Eigen", "contrib/Eigen", "unsupported/Eigen", "test", "contrib/test", "failtest")

# Directories whose .cpp files are test translation units.
TEST_ROOTS = ("test", "contrib/test")

# Subtrees of TEST_ROOTS that are not part of this build.  test/buildsystem/
# holds standalone CMake projects that test:linux:buildsystem configures on
# their own, so their add_executable() calls register targets no configuration
# here has, and their sources are not test translation units.
EXCLUDED_TEST_DIRS = ("test/buildsystem",)

# Compile-failure suite.  ei_add_failtest registers <name>_ok and <name>_ko as
# CTest tests whose test action is a build of an EXCLUDE_FROM_ALL target.
FAILTEST_ROOT = "failtest"

# Extensions a registered test translation unit can have.  ".cu" comes from the
# GPU registrations; see CMAKE_REGISTRATION_RE.
TEST_SOURCE_SUFFIXES = (".cpp", ".cu")

# Share of the test suite above which an explicit selection is replaced by the
# full ``buildtests`` target.
DEFAULT_MAX_FRACTION = 0.85

# Changes matching these patterns cannot affect which tests exist or what they
# cover, so they select nothing.
IGNORED_PATTERNS = (
    ".gitattributes",
    ".gitignore",
    ".clang-format",
    ".clang-tidy",
    "*.md",
    "*.dox",
    "AGENTS.md",
    "COPYING*",
    "INSTALL",
    "README*",
    "REUSE.toml",
    ".agents/*",
    ".gitlab/*",
    "LICENSES/*",
    "benchmarks/*",
    # The CI YAML is orchestration -- rules, tags, artifacts, job names -- and
    # cannot change which test includes which header, which is the only mapping
    # this selector maintains.  Ignored rather than merely dropped from
    # FULL_REBUILD_PATTERNS below: these files are not under SCAN_ROOTS, so an
    # un-forced path would instead fall through to "not in the include graph"
    # and force the full suite anyway.
    "ci/*.gitlab-ci.yml",
    "ci/CTest2JUnit.xsl",
    "debug/*",
    "demos/*",
    "doc/*",
    "contrib/benchmarks/*",
    "contrib/doc/*",
)

# Changes matching these patterns invalidate the include-graph mapping itself
# (test registration, split counts, the CI drivers, or shim libraries whose
# tests are not modelled here), so they force the full test suite.  Checked
# after IGNORED_PATTERNS, so a benchmark's or the docs' own CMakeLists.txt does
# not drag in the whole suite.
FULL_REBUILD_PATTERNS = (
    "CMakeLists.txt",
    "*/CMakeLists.txt",
    "*.cmake",
    "*.cmake.in",
    # Keeps forcing: it defines the selector-to-driver contract itself
    # (EIGEN_CI_BUILD_TARGET_FILE, EIGEN_CI_CTEST_REGEX_FILE).
    ".gitlab-ci.yml",
    # ci/ splits: the scripts and the image definitions can change what a build
    # produces or how a test runs.  The *.gitlab-ci.yml files cannot, and are
    # ignored above.
    "ci/scripts/*",
    "ci/docker/*",
    "cmake/*",
    "scripts/*",
    "blas/*",
    "lapack/*",
)

INCLUDE_RE = re.compile(r'^[ \t]*#[ \t]*include[ \t]*[<"]([^>"]+)[>"]', re.MULTILINE)
# One pass over a test CMakeLists.txt, in source order, because what a
# registration means depends on the state at that point: the source extension
# comes from EIGEN_ADD_TEST_FILENAME_EXTENSION, which the CUDA and HIP blocks
# set to "cu" and unset again, so gpu_basic is test/gpu_basic.cu while its
# neighbours are .cpp; and a name spelled ${var} resolves against the item list
# of the enclosing foreach(), which is how the GPU module registers its
# cusolver_* and cudss_* tests.
CMAKE_REGISTRATION_RE = re.compile(
    r"^[ \t]*(?:"
    r'(?P<scope>set|unset)\([ \t]*EIGEN_ADD_TEST_FILENAME_EXTENSION[ \t]*"?(?P<extension>[A-Za-z_0-9]*)"?'
    r"|(?:ei_add_test|ei_add_gpu_test)\([ \t]*"
    r"(?P<test>[A-Za-z_][A-Za-z_0-9]*|\$\{[A-Za-z_][A-Za-z_0-9]*\})"
    r"|add_executable\([ \t]*(?P<executable>[A-Za-z_][A-Za-z_0-9]*)[ \t\r\n]+(?P<sources>[^)]*)\)"
    r"|foreach\((?P<loop>[^)]*)\)"
    r"|(?P<endloop>endforeach)\("
    r")",
    re.MULTILINE,
)
# A bare CMake identifier, used to recognise a foreach loop variable.
CMAKE_NAME_RE = re.compile(r"[A-Za-z_][A-Za-z_0-9]*\Z")
CMAKE_FAILTEST_RE = re.compile(
    r'^[ \t]*ei_add_failtest\([ \t]*"?([A-Za-z_][A-Za-z_0-9]*)"?',
    re.MULTILINE,
)


def _matches(path, patterns):
    return any(fnmatch.fnmatch(path, p) for p in patterns)


def _under(path, roots):
    return any(path.startswith(root + "/") for root in roots)


class IncludeGraph:
    """Textual ``#include`` graph over the scanned source roots."""

    def __init__(self, source_dir):
        self.source_dir = source_dir
        self.files = set()
        self._direct = {}
        self._by_suffix = {}
        self._scan()

    def _scan(self):
        for root in SCAN_ROOTS:
            abs_root = os.path.join(self.source_dir, root)
            if not os.path.isdir(abs_root):
                continue
            for dirpath, dirnames, filenames in os.walk(abs_root):
                dirnames[:] = [d for d in dirnames if not d.startswith(".")]
                for name in filenames:
                    rel = os.path.relpath(os.path.join(dirpath, name), self.source_dir)
                    self.files.add(rel)
        # Index every path suffix so that an include spelled relative to a
        # directory outside the scanned roots still resolves.  Ambiguous
        # suffixes are dropped rather than guessed.
        candidates = {}
        for rel in self.files:
            parts = rel.split("/")
            for i in range(len(parts)):
                candidates.setdefault("/".join(parts[i:]), []).append(rel)
        self._by_suffix = {suffix: matches[0]
                           for suffix, matches in candidates.items() if len(matches) == 1}

    def read_text(self, rel):
        """Contents of a file in the tree, or ``''`` if it cannot be read."""
        try:
            with open(os.path.join(self.source_dir, rel), "r", errors="ignore") as handle:
                return handle.read()
        except OSError:
            return ""

    def direct_includes(self, rel):
        """Resolved includes of a single file."""
        cached = self._direct.get(rel)
        if cached is not None:
            return cached
        resolved = set()
        self._direct[rel] = resolved  # placed first: the graph has cycles
        directory = os.path.dirname(rel)
        for spelling in INCLUDE_RE.findall(self.read_text(rel)):
            candidate = os.path.normpath(os.path.join(directory, spelling))
            if candidate in self.files:
                resolved.add(candidate)
            elif spelling in self.files:
                resolved.add(spelling)
            elif spelling in self._by_suffix:
                resolved.add(self._by_suffix[spelling])
        return resolved

    def closure(self, rel):
        """Every file reachable from ``rel`` through includes."""
        seen = set()
        stack = [rel]
        while stack:
            for nxt in self.direct_includes(stack.pop()):
                if nxt not in seen:
                    seen.add(nxt)
                    stack.append(nxt)
        return seen


# targets     -- test translation unit -> the CMake target that compiles it
# standalone  -- targets the ``buildtests`` aggregate does not depend on
# failtests   -- failtest translation unit -> the CTest names that compile it
Registrations = collections.namedtuple("Registrations", "targets standalone failtests")


def _loop_binding(arguments):
    """Names a ``foreach(...)`` binds, or ``None`` when they are not literal."""
    tokens = [token.strip('"') for token in arguments.split()]
    if not tokens or not CMAKE_NAME_RE.match(tokens[0]):
        return None
    variable, items = tokens[0], tokens[1:]
    if items[:2] == ["IN", "ITEMS"]:
        items = items[2:]
    elif items[:1] == ["IN"]:
        # IN LISTS and IN ZIP_LISTS iterate variables, not literal names.
        return None
    return variable, [item for item in items if "$" not in item]


def _loop_expand(token, loops):
    """Resolve a registration name against the enclosing ``foreach`` bindings."""
    if not token.startswith("$"):
        return [token]
    name = token[2:-1]
    for binding in reversed(loops):
        if binding is not None and binding[0] == name:
            return binding[1]
    return []


def test_registrations(graph):
    """Map registered translation units to what CI has to build or run."""
    source_targets = {}
    standalone = set()

    def register(source, target):
        previous = source_targets.get(source)
        if previous is not None and previous != target:
            raise ValueError("%s is registered by both %s and %s" % (source, previous, target))
        source_targets[source] = target

    cmake_files = sorted(
        rel
        for rel in graph.files
        if os.path.basename(rel) == "CMakeLists.txt"
        and _under(rel, TEST_ROOTS)
        and not _under(rel, EXCLUDED_TEST_DIRS)
    )
    for cmake_file in cmake_files:
        directory = os.path.dirname(cmake_file)
        extension = "cpp"
        # foreach() bindings in effect, innermost last.  A loop over anything
        # but a literal item list pushes None so endforeach() stays balanced.
        loops = []
        for match in CMAKE_REGISTRATION_RE.finditer(graph.read_text(cmake_file)):
            if match.group("scope"):
                # unset(), or a set() with no value, restores the default.
                extension = match.group("extension") if match.group("scope") == "set" else ""
                extension = extension or "cpp"
                continue
            if match.group("loop") is not None:
                loops.append(_loop_binding(match.group("loop")))
                continue
            if match.group("endloop"):
                if loops:
                    loops.pop()
                continue
            if match.group("test"):
                for target in _loop_expand(match.group("test"), loops):
                    source = os.path.normpath(
                        os.path.join(directory, "%s.%s" % (target, extension)))
                    if source in graph.files:
                        register(source, target)
                continue
            target = match.group("executable")
            for token in re.findall(r'"[^"]*"|[^\s]+', match.group("sources")):
                token = token.strip('"')
                if not token.endswith(TEST_SOURCE_SUFFIXES) or "$" in token:
                    continue
                source = os.path.normpath(os.path.join(directory, token))
                if source in graph.files:
                    register(source, target)
                    standalone.add(target)

    failtests = {}
    for name in CMAKE_FAILTEST_RE.findall(graph.read_text(FAILTEST_ROOT + "/CMakeLists.txt")):
        source = "%s/%s.cpp" % (FAILTEST_ROOT, name)
        if source in graph.files:
            failtests[source] = (name + "_ok", name + "_ko")

    return Registrations(source_targets, standalone, failtests)


def full_suite(graph, reasons):
    """Full-suite selection, naming the targets ``buildtests`` does not build."""
    try:
        standalone = test_registrations(graph).standalone
    except ValueError:
        # A broken registration is reported by the paths that depend on the
        # mapping; the full suite stays available without it.
        standalone = ()
    return Selection("all", reasons=reasons, standalone=standalone)


def reverse_map(graph, sources):
    """Map each included file to the test sources that reach it."""
    reverse = {}
    for src in sources:
        for dep in graph.closure(src):
            reverse.setdefault(dep, set()).add(src)
    return reverse


class Selection:
    """Outcome: the full suite, explicit targets, no tests, or an error."""

    def __init__(self, mode, targets=(), reasons=(), ctest_names=(), standalone=()):
        self.mode = mode  # "all", "targets", "none", or "error"
        self.targets = set(targets)
        self.reasons = list(reasons)
        # CTest names with no build target of their own.
        self.ctest_names = set(ctest_names)
        # Targets to name alongside ``buildtests`` in "all" mode.
        self.standalone = set(standalone)

    @property
    def targets_file(self):
        if self.mode == "error":
            raise ValueError("an invalid selection has no target file")
        if self.mode == "all":
            return "".join(name + "\n" for name in ["buildtests"] + sorted(self.standalone))
        if self.mode == "none":
            return "NONE\n"
        return "".join(name + "\n" for name in sorted(self.targets))

    @property
    def regex_file(self):
        if self.mode == "error":
            raise ValueError("an invalid selection has no regex file")
        if self.mode == "all":
            return "ALL\n"
        if self.mode == "none":
            return "NONE\n"
        names = sorted(self.targets) + sorted(self.ctest_names)
        return "^(%s)(_[0-9]+)?$\n" % "|".join(re.escape(name) for name in names)


def select(graph, changed_files, max_fraction=DEFAULT_MAX_FRACTION):
    """Map changed paths to the tests that must run."""
    paths = []
    for path in changed_files:
        path = path.strip()
        if path and not _matches(path, IGNORED_PATTERNS):
            paths.append(path)
    if not paths:
        return Selection("none", reasons=["no change reaches a test"])
    for path in paths:
        if _matches(path, FULL_REBUILD_PATTERNS):
            return full_suite(graph, ["%s forces the full suite" % path])

    try:
        registered = test_registrations(graph)
    except ValueError as error:
        return Selection("error", reasons=[str(error)])
    sources = sorted(registered.targets)
    reverse = reverse_map(graph, sources)
    failtest_reverse = reverse_map(graph, sorted(registered.failtests))

    selected = set()
    selected_failtests = set()
    reasons = []
    for path in paths:
        reached_by = set(reverse.get(path, ()))
        if path in registered.targets:
            reached_by.add(path)
        failtests = set(failtest_reverse.get(path, ()))
        if path in registered.failtests:
            failtests.add(path)
        if reached_by or failtests:
            selected |= reached_by
            selected_failtests |= failtests
            continue
        if path in graph.files:
            # A source file in the tree that nothing includes: either a new
            # header not yet wired up or an unregistered translation unit.
            roots = TEST_ROOTS + (FAILTEST_ROOT,)
            if (_under(path, roots) and not _under(path, EXCLUDED_TEST_DIRS)
                    and path.endswith(TEST_SOURCE_SUFFIXES)):
                return Selection("error", reasons=["%s has no CMake test target" % path])
            reasons.append("%s is in the tree but reaches no test" % path)
            continue
        # Deleted, renamed, or outside every scanned root: the graph cannot say
        # what it affected, so do not guess.
        return full_suite(graph, ["%s is not in the include graph" % path])

    if not selected and not selected_failtests:
        return Selection("none", reasons=reasons or ["no change reaches a test"])

    if len(selected) > max_fraction * len(sources):
        reasons.append(
            "%d of %d test sources selected (>%.0f%%)"
            % (len(selected), len(sources), 100 * max_fraction)
        )
        return full_suite(graph, reasons)

    ctest_names = set()
    for source in sorted(selected_failtests):
        ctest_names.update(registered.failtests[source])
    if ctest_names:
        reasons.append("%d compile-failure test(s) build from inside CTest"
                       % len(selected_failtests))
    return Selection("targets", (registered.targets[s] for s in selected), reasons,
                     ctest_names=ctest_names)


def changed_files_from_git(source_dir, base_sha, head="HEAD"):
    """Paths changed between ``base_sha`` and ``head``."""
    result = subprocess.run(
        ["git", "diff", "--no-renames", "--name-only", "%s...%s" % (base_sha, head)],
        cwd=source_dir,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError("git diff failed: %s" % result.stderr.strip())
    return [line for line in result.stdout.splitlines() if line.strip()]


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    default_source = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser.add_argument("--source-dir", default=default_source,
                        help="Eigen source tree (default: the tree containing this script)")
    parser.add_argument("--base-sha",
                        help="compute changed files from 'git diff BASE...HEAD'")
    parser.add_argument("--head", default="HEAD", help="head revision for --base-sha")
    parser.add_argument("--changed-files",
                        help="read newline-separated changed paths from this file ('-' for stdin)")
    parser.add_argument("--output-dir",
                        help="write targets.txt and ctest_regex.txt here")
    parser.add_argument("--max-fraction", type=float, default=DEFAULT_MAX_FRACTION,
                        help="degrade to the full suite above this fraction (default: %(default)s)")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    if args.changed_files:
        if args.changed_files == "-":
            changed = sys.stdin.read().splitlines()
        else:
            with open(args.changed_files) as handle:
                changed = handle.read().splitlines()
    elif args.base_sha:
        try:
            changed = changed_files_from_git(args.source_dir, args.base_sha, args.head)
        except RuntimeError as error:
            # Without a usable diff there is no basis for narrowing.
            print("%s; selecting the full suite" % error, file=sys.stderr)
            changed = None
    else:
        print("one of --base-sha or --changed-files is required", file=sys.stderr)
        return 2

    graph = IncludeGraph(args.source_dir)
    if changed is None:
        selection = full_suite(graph, ["the merge-base diff is unavailable"])
    else:
        selection = select(graph, changed, args.max_fraction)

    print("mode: %s" % selection.mode, file=sys.stderr)
    for reason in selection.reasons:
        print("  %s" % reason, file=sys.stderr)
    if selection.mode == "error":
        return 1
    if selection.mode == "targets":
        print("  %d targets: %s" % (len(selection.targets),
                                    " ".join(sorted(selection.targets))), file=sys.stderr)
        if selection.ctest_names:
            print("  %d CTest-only: %s" % (len(selection.ctest_names),
                                           " ".join(sorted(selection.ctest_names))),
                  file=sys.stderr)

    if args.output_dir:
        os.makedirs(args.output_dir, exist_ok=True)
        with open(os.path.join(args.output_dir, "targets.txt"), "w") as handle:
            handle.write(selection.targets_file)
        with open(os.path.join(args.output_dir, "ctest_regex.txt"), "w") as handle:
            handle.write(selection.regex_file)
    else:
        sys.stdout.write(selection.targets_file)

    return 0


if __name__ == "__main__":
    sys.exit(main())
