#!/usr/bin/env python3
# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

"""Run clang-tidy on the lines an edit added.

``.clang-tidy`` is the authority on the conventions clang-tidy can express, so
rather than restate them this hook runs the real thing on the edited file and
reports only diagnostics on added lines (``--line-filter``).  Without that
restriction the checks would be unusable interactively: ``Eigen/src`` holds
~4100 ``typedef``s, and whole-file output buries the two lines just written.

Editing a header under ``Eigen/src`` cannot be compiled directly — the module's
``InternalHeaderCheck.h`` ``#error``s out — so a driver includes the module
umbrella first and then the edited header.  The umbrella name is read from the
``#error "Please include <X>"`` directive.  This mirrors what
``ci/scripts/run-clang-tidy.sh`` does for merge requests.  Including the edited
header explicitly also covers a new header that the umbrella does not export
yet.

LLVM's ``clang-tidy-diff.py`` cannot replace this routing: it invokes changed
headers directly, which trips Eigen's internal-header guard, and a PostToolUse
payload is not a unified diff.

No compilation database is required: ``-std=c++14 -I<repo>`` parses any file
in the C++14 trees, with ``test/`` added for the test sources and
``contrib/`` for the contrib ones, which keeps the hook usable in a
fresh checkout with no build directory.

The hook fails open — a missing clang-tidy, an unparsable file, a translation
unit that does not compile, or any unexpected error exits 0 rather than blocking
the harness. Only real findings on added lines block.
"""

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from style_common import (REPO_ROOT, diff_added_lines, hook_edit_snippets,  # noqa: E402
                          hook_post_image_and_added, is_cxx_path, line_filter_json)

# Parsing an umbrella costs about a second, so keep the hook off trees whose
# conventions these checks do not describe (bench/, demos/, doc/ examples).
# unsupported/Eigen/ is kept for the legacy umbrella shims that still live
# there; the implementation tree itself is contrib/, which has no shims and
# so needs the src-tree rules.
CHECKED_TREES = ("Eigen/", "contrib/Eigen/", "unsupported/Eigen/", "test/", "contrib/test/", "failtest/", "blas/",
                 "lapack/")
SRC_TREES = ("Eigen/src/", "contrib/Eigen/src/")
# Modules whose umbrella needs a third-party header we cannot assume is present.
EXTERNAL_DEP_MODULES = ("AccelerateSupport", "CholmodSupport", "KLUSupport", "MetisSupport",
                        "PaStiXSupport", "PardisoSupport", "SPQRSupport", "SuperLUSupport",
                        "UmfPackSupport")
PLEASE_INCLUDE = re.compile(r'"Please include ([^ "]+)')
TIMEOUT_SECONDS = 30


def module_of(rel_path):
    """Return the ``Eigen/src/<Module>`` name owning ``rel_path``, else None."""
    m = re.match(r"^(?:contrib/)?Eigen/src/([^/]+)/", rel_path)
    return m.group(1) if m else None


def umbrella_for(rel_path, root=REPO_ROOT):
    """Return the module umbrella to include when linting a src-tree header.

    The source of truth is the ``#error "Please include <X>"`` directive on the
    header itself or on its sibling ``InternalHeaderCheck.h``; the
    ``<root>/<Module>`` heuristic covers arch-specific backends nested deeper
    than the module root, which carry no directive of their own.
    """
    module = module_of(rel_path)
    if module is None or module in EXTERNAL_DEP_MODULES:
        return None
    for candidate in (rel_path, os.path.join(os.path.dirname(rel_path), "InternalHeaderCheck.h")):
        try:
            with open(os.path.join(root, candidate), encoding="utf-8", errors="replace") as handle:
                hit = PLEASE_INCLUDE.search(handle.read())
        except OSError:
            continue
        if hit and os.path.isfile(os.path.join(root, hit.group(1))):
            return hit.group(1)
    fallback = ("contrib/Eigen/" if rel_path.startswith("contrib/") else "Eigen/") + module
    return fallback if os.path.isfile(os.path.join(root, fallback)) else None


def cuda_include_dir(default_root="/usr/local/cuda"):
    """Include directory of an installed CUDA toolkit, or None if there is none.

    Every header in the GPU module that includes ``GpuSupport.h`` reaches
    ``<cuda_runtime.h>``. Without it clang drops the include and every
    declaration behind it, leaving an AST too truncated to check the module
    against; with it the header is checked like any other.

    ``default_root`` is where the toolkit is looked for once the environment
    names none; a test passes a directory that holds no toolkit so the
    negative case does not depend on the host.
    """
    roots = [os.environ.get(name) for name in ("CUDAToolkit_ROOT", "CUDA_HOME", "CUDA_PATH")]
    roots.append(default_root)
    for root in roots:
        if root and os.path.isfile(os.path.join(root, "include", "cuda_runtime.h")):
            return os.path.join(root, "include")
    return None


def compile_args(rel_path, root=REPO_ROOT):
    """Include paths sufficient to parse ``rel_path`` without a compile database.

    Test sources reach ``main.h`` through ``test/``; the contrib ones also
    include their module umbrellas as ``<Eigen/Tensor>``, which resolves only
    with ``contrib/`` on the path.  These are the directories
    ``contrib/test/CMakeLists.txt`` adds for them.
    """
    args = ["-std=c++14", "-I" + root]
    if rel_path.startswith(("test/", "contrib/test/")):
        args += ["-I" + os.path.join(root, "test")]
    if rel_path.startswith("contrib/test/"):
        args += ["-I" + os.path.join(root, "contrib")]
    cuda = cuda_include_dir()
    if cuda:
        args += ["-isystem", cuda]
    return args


def tidy_target(rel_path, tmpdir, root=REPO_ROOT):
    """Return the path clang-tidy should be pointed at, or None to skip.

    Source files are linted directly; src-tree headers go through a generated
    driver that includes their module umbrella followed by the header itself.
    """
    if rel_path.startswith(SRC_TREES) or not os.path.splitext(rel_path)[1]:
        include = umbrella_for(rel_path, root) if rel_path.startswith(SRC_TREES) else rel_path
        if include is None:
            return None
        driver = os.path.join(tmpdir, "tidy_driver_" + rel_path.replace("/", "_") + ".cpp")
        with open(driver, "w", encoding="utf-8") as handle:
            handle.write("#include <%s>\n" % include)
            if rel_path.startswith(SRC_TREES):
                handle.write("#include <%s>\n" % rel_path)
        return driver
    if os.path.splitext(rel_path)[1].lower() in (".cpp", ".cc", ".cxx"):
        return os.path.join(root, rel_path)
    return None  # a .h outside the src trees has no reliable standalone TU


def run_clang_tidy(per_file, root=REPO_ROOT, tidy=None):
    """Lint the added lines of each file in ``per_file``.

    Returns (diagnostics, skipped): ``diagnostics`` are clang-tidy's warning
    lines, ``skipped`` a list of (path, reason) for files that could not be
    checked.  A file whose translation unit does not compile is one of those:
    clang-tidy emits no useful check output for it, and reporting it as clean
    would claim a check that never ran.
    """
    tidy = shutil.which(tidy or "clang-tidy")
    if tidy is None:
        return None, []
    diagnostics, skipped = [], []
    with tempfile.TemporaryDirectory() as tmpdir:
        for rel_path, added in sorted(per_file.items()):
            if not added or not is_cxx_path(rel_path) or not rel_path.startswith(CHECKED_TREES):
                continue
            target = tidy_target(rel_path, tmpdir, root)
            if target is None:
                skipped.append((rel_path, "no standalone translation unit"))
                continue
            cmd = [tidy, "--quiet",
                   "--config-file=" + os.path.join(root, ".clang-tidy"),
                   "--header-filter=" + re.escape(rel_path),
                   "--line-filter=" + line_filter_json({rel_path: added}),
                   target, "--"] + compile_args(rel_path, root)
            try:
                done = subprocess.run(cmd, capture_output=True, text=True, timeout=TIMEOUT_SECONDS)
            except OSError:
                skipped.append((rel_path, "clang-tidy could not be run"))
                continue
            except subprocess.TimeoutExpired:
                skipped.append((rel_path, "clang-tidy timed out after %ds" % TIMEOUT_SECONDS))
                continue
            broke = False
            output = "\n".join(part for part in (done.stdout, done.stderr) if part)
            for line in output.splitlines():
                if not re.search(r": (warning|error): ", line):
                    continue
                if "clang-diagnostic-error" in line or ": error: " in line:
                    broke = True
                    continue
                diagnostics.append(re.sub(r"^" + re.escape(root) + "/", "", line))
            if broke:
                skipped.append((rel_path, "translation unit did not compile"))
            elif done.returncode != 0:
                skipped.append((rel_path, "clang-tidy failed"))
    return diagnostics, skipped


def run_hook_mode(tidy=None):
    try:
        payload = json.load(sys.stdin)
    except Exception:
        return 0
    rel_path, snippets = hook_edit_snippets(payload)
    if rel_path is None or not is_cxx_path(rel_path) or not rel_path.startswith(CHECKED_TREES):
        return 0
    _, added, exact = hook_post_image_and_added(payload, rel_path, snippets)
    if not exact or not added:
        # Without real line numbers a line filter would point at the wrong
        # lines; silence beats a misplaced diagnostic.
        return 0
    diagnostics, skipped = run_clang_tidy({rel_path: added}, tidy=tidy)
    if not diagnostics:
        broken = [reason for path, reason in (skipped or []) if reason == "translation unit did not compile"]
        if broken:
            # A header edited mid-refactor routinely fails to parse on its own,
            # so show the user a non-blocking notice rather than feeding Claude
            # an error with no finding to act on.
            message = "clang-tidy (%s): not checked — the translation unit did not compile." % rel_path
            sys.stdout.write(json.dumps({"systemMessage": message}) + "\n")
            return 0
        return 0
    sys.stderr.write("clang-tidy (%s) — added lines only:\n" % rel_path)
    for line in diagnostics[:8]:
        sys.stderr.write("  %s\n" % line)
    if len(diagnostics) > 8:
        sys.stderr.write("  ... and %d more\n" % (len(diagnostics) - 8))
    sys.stderr.write("Advisory: these are Eigen conventions from .clang-tidy; keep a flagged construct "
                     "only if it is justified, and say why at the construct.\n")
    return 2


def run_diff_mode(base, tidy=None):
    per_file = diff_added_lines(base)
    diagnostics, skipped = run_clang_tidy(per_file, tidy=tidy)
    if diagnostics is None:
        sys.stderr.write("%s not found; skipping.\n" % (tidy or "clang-tidy"))
        return 0
    for line in diagnostics:
        print(line)
    for rel_path, reason in skipped:
        print("%s: not checked (%s)" % (rel_path, reason))
    if diagnostics:
        print("\n%d clang-tidy finding(s) on added lines." % len(diagnostics))
    return 1 if diagnostics else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--diff", metavar="BASE", help="lint lines added since merge-base(BASE, HEAD)")
    mode.add_argument("--claude-hook", action="store_true", help="run as a Claude Code PostToolUse hook")
    parser.add_argument("-b", "--binary", metavar="CLANG_TIDY",
                        help="clang-tidy executable to use, as a PATH name or a full path "
                             "(e.g. clang-tidy-19 from LLVM's version-suffixed packages); "
                             "default: clang-tidy")
    args = parser.parse_args()
    if args.claude_hook:
        try:
            return run_hook_mode(tidy=args.binary)
        except Exception:
            return 0  # a broken hook must not block the harness
    if args.binary and shutil.which(args.binary) is None:
        # In diff mode an explicitly named binary that is absent is a usage
        # error; silently skipping would claim a check that never ran.  Hook
        # mode above keeps the fail-open contract instead.
        parser.error("--binary %s not found" % args.binary)
    return run_diff_mode(args.diff, tidy=args.binary)


if __name__ == "__main__":
    sys.exit(main())
