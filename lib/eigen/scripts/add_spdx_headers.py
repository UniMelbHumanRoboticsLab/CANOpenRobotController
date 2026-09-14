#!/usr/bin/env python3
# This file is part of Eigen, a lightweight C++ template library
# for linear algebra.
#
# Copyright (C) 2026 The Eigen Authors
#
# This Source Code Form is subject to the terms of the Mozilla
# Public License v. 2.0. If a copy of the MPL was not distributed
# with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
# SPDX-License-Identifier: MPL-2.0
# REUSE-IgnoreStart
"""
Insert per-file SPDX license tags across the Eigen tree.

Idempotent: files already containing an SPDX-License-Identifier tag are left
untouched. Re-running after a clean run produces no changes.

The license id assigned to each file is determined by `classify(path)`:
- LAPACKE/BLAS bridges with Intel copyright    -> BSD-3-Clause
- MORSE-derived cmake/Find*.cmake modules      -> BSD-3-Clause
- Reference-BLAS Fortran tests blas/testing/*.f -> BSD-3-Clause
- LevenbergMarquardt MINPACK-derived headers   -> MPL-2.0 AND LicenseRef-MINPACK
- Everything else under repo                    -> MPL-2.0

The comment style is determined by the file extension:
- C / C++ / CUDA / Inc / Fortran-fixed-form ('//', '#', '*' etc.)

Files explicitly excluded from inline SPDX (handled instead by REUSE.toml):
- LICENSES/, COPYING.*, LICENSE                                (license texts)
- *.dox, *.in, *.md, *.dat, *.png, README*, INSTALL,           (assets/templates)
  *.def, *.natvis, *.css, *.xsl, *.preamble, *.traits, *.krazy
- .git*, .clang*, .coderabbit*, .gitlab/                       (config dotfiles)
- failtest/*.cpp                                               (trivial stubs)
- signature_of_eigen3_matrix_library                           (sentinel file)
"""
# REUSE-IgnoreEnd

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path
from typing import Iterable

REPO_ROOT = Path(__file__).resolve().parent.parent

SPDX_RE = re.compile(r"SPDX-License-Identifier\s*:")
# Match REUSE's own copyright detection (case-sensitive `Copyright`); lowercase
# `copyright` lines (typo in some headers) won't be recognised, so we'll still
# inject `SPDX-FileCopyrightText` for those files.
COPYRIGHT_RE = re.compile(r"Copyright\b")


# ---------------------------------------------------------------------------
# Path classification
# ---------------------------------------------------------------------------

# Files containing an Intel BSD-3-Clause copyright header (LAPACKE/BLAS bridges).
# Match by content rather than path so this stays correct as files are added.
INTEL_BSD_MARKER = b"Copyright (c) 2011, Intel Corporation"
INTEL_BSD_MARKER_2010 = b"Copyright (c) 2010, Intel Corp"

# MORSE-derived cmake modules (Univ. of Tennessee 2009-2014).
MORSE_MARKER = b"@copyright (c) 2009-2014 The University of Tennessee"

# Header marker for the LevenbergMarquardt MINPACK-derived files.
MINPACK_MARKER = b"Minpack license"

# Reference-BLAS / LAPACK Fortran (Univ. of Tennessee/Berkeley/Colorado Denver).
REFBLAS_MARKERS = (
    b"Reference BLAS is a software package",
    b"LAPACK is a software package provided by Univ. of Tennessee",
)

# Apache-2.0 — TensorFlow-derived BFloat16, Codeplay FindComputeCpp.cmake.
APACHE_MARKER = b"Licensed under the Apache License"


def in_failtest(rel: Path) -> bool:
    return rel.parts and rel.parts[0] == "failtest"


# Bulk-mapped via REUSE.toml — never insert inline SPDX into these.
SKIP_BY_NAME = {
    "CHANGELOG.md",
    "INSTALL",
    "README.md",
    "signature_of_eigen3_matrix_library",
    "CLAUDE.md",
}

SKIP_BY_SUFFIX = {
    ".dox",
    ".in",
    ".dat",
    ".png",
    ".svg",
    ".jpg",
    ".jpeg",
    ".gif",
    ".md",
    ".txt",        # README.txt, *.txt config — handled by REUSE.toml
    ".def",        # blas/eigen_blas.def, lapack/eigen_lapack*.def
    ".natvis",
    ".css",
    ".xsl",
    ".preamble",
    ".traits",
    ".krazy",
    ".yaml",       # .coderabbit.yaml — REUSE.toml
    ".entry",      # doc/examples/make_circulant.cpp.entry / .evaluator / ...
    ".evaluator",
    ".expression",
    ".main",
}

# Top-level / specially-handled excluded paths (relative to repo root).
SKIP_PATHS = {
    ".clang-format",
    ".clang-tidy",
    ".coderabbit.yaml",
    ".git-blame-ignore-revs",
    ".gitattributes",
    ".gitignore",
    "LICENSE",
    "REUSE.toml",
}


def is_skipped(rel: Path) -> bool:
    name = rel.name
    posix = rel.as_posix()
    if posix in SKIP_PATHS:
        return True
    if posix.startswith("LICENSES/") or posix.startswith("COPYING"):
        return True
    if posix.startswith(".gitlab/") or posix.startswith(".git/"):
        return True
    if in_failtest(rel) and rel.suffix == ".cpp":
        return True
    # Recognise CMakeLists.txt by name BEFORE the .txt suffix-skip below.
    if name == "CMakeLists.txt":
        return False
    if name in SKIP_BY_NAME:
        return True
    if rel.suffix in SKIP_BY_SUFFIX:
        return True
    return False


def classify(rel: Path, content: bytes) -> str:
    """Return the SPDX-License-Identifier expression for a file."""
    if INTEL_BSD_MARKER in content or INTEL_BSD_MARKER_2010 in content:
        return "BSD-3-Clause"
    if MORSE_MARKER in content:
        return "BSD-3-Clause"
    if MINPACK_MARKER in content:
        return "MPL-2.0 AND LicenseRef-MINPACK"
    if any(m in content for m in REFBLAS_MARKERS):
        return "BSD-3-Clause"
    if APACHE_MARKER in content:
        return "Apache-2.0"
    return "MPL-2.0"


# ---------------------------------------------------------------------------
# Comment-style dispatch
# ---------------------------------------------------------------------------

# (suffix -> comment prefix). Files without a known suffix are handled by name.
SLASH_LANGUAGES = {".h", ".hh", ".cpp", ".cc", ".c", ".cu", ".inc", ".js"}
HASH_LANGUAGES = {".cmake", ".sh", ".py", ".ps1", ".yml", ".cfg"}
FORTRAN_LANGUAGES = {".f", ".F", ".f90"}


def is_eigen_umbrella_header(rel: Path) -> bool:
    """`Eigen/Core`, `contrib/Eigen/Tensor`, etc. — extensionless C++ headers."""
    parts = rel.parts
    if rel.suffix:
        return False
    if len(parts) >= 2 and parts[0] == "Eigen" and parts[-1][:1].isupper():
        return len(parts) == 2
    if (
        len(parts) >= 3
        and parts[0] in ("contrib", "unsupported")
        and parts[1] == "Eigen"
        and parts[-1][:1].isupper()
    ):
        # contrib/Eigen/Tensor (real), unsupported/Eigen/Tensor (legacy shim), ...
        return True
    return False


def has_shebang(path: Path) -> bool:
    try:
        with path.open("rb") as f:
            return f.read(2) == b"#!"
    except OSError:
        return False


def comment_prefix_for(rel: Path, full: Path | None = None) -> str | None:
    if rel.suffix in SLASH_LANGUAGES:
        return "//"
    if rel.suffix in HASH_LANGUAGES:
        return "#"
    if rel.suffix in FORTRAN_LANGUAGES:
        return "*>"
    name = rel.name
    if name == "CMakeLists.txt":
        return "#"
    if is_eigen_umbrella_header(rel):
        return "//"
    # Extensionless executable scripts (e.g. scripts/eigen_gen_docs).
    if not rel.suffix and full is not None and has_shebang(full):
        return "#"
    return None


# ---------------------------------------------------------------------------
# Insertion strategy
# ---------------------------------------------------------------------------

def insert_spdx(
    path: Path, content: bytes, spdx_id: str, comment: str
) -> bytes | None:
    """Return new file bytes with the SPDX line inserted, or None for no-op."""
    full_text = content.decode("utf-8", errors="replace")
    if SPDX_RE.search(full_text):
        return None  # Already tagged.

    head = full_text[:4096]
    has_copyright = COPYRIGHT_RE.search(head) is not None

    # Match the file's line-ending style (CRLF vs LF) so we don't introduce
    # mixed endings that clang-format will then "fix".
    eol = "\r\n" if b"\r\n" in content[:4096] else "\n"

    # REUSE-IgnoreStart
    spdx_lines = []
    if not has_copyright:
        spdx_lines.append(f"{comment} SPDX-FileCopyrightText: The Eigen Authors{eol}")
    spdx_lines.append(f"{comment} SPDX-License-Identifier: {spdx_id}{eol}")
    spdx_line = "".join(spdx_lines).encode()
    # REUSE-IgnoreEnd

    lines = full_text.splitlines(keepends=True)

    # Two header families to consider:
    #
    # (a) Existing C-line-comment MPL prose ("// This file is part of Eigen, ...
    #     // ... obtain one at http://mozilla.org/MPL/2.0/."): append SPDX
    #     directly after the prose block (last consecutive comment line at
    #     top-of-file).
    #
    # (b) Existing C-block-comment header (Intel "/* Copyright (c) 2011, Intel
    #     Corporation ... */"): append a comment line right after the closing
    #     "*/".
    #
    # (c) Hash-comment prose (.cmake / .gitlab-ci.yml / etc.): append after the
    #     last consecutive '#'-comment line at top.
    #
    # (d) Fortran '*'-comment prose (Reference BLAS): append a `*>` line after
    #     the last consecutive '*'-prefixed line at top.
    #
    # (e) No header: insert SPDX as the very first line, then a blank line.
    #
    # Files starting with `#!shebang` or a `#ifndef` guard — we always insert
    # the SPDX line above the guard but below any shebang.

    insert_at = 0
    has_shebang = lines and lines[0].startswith("#!")
    if has_shebang:
        insert_at = 1

    # (b) C-block comment opening at index `insert_at`.
    if insert_at < len(lines) and lines[insert_at].lstrip().startswith("/*"):
        for i in range(insert_at, len(lines)):
            if "*/" in lines[i]:
                insert_at = i + 1
                # Skip a single blank line after the block, if any.
                break
        new_block = spdx_line.decode()
        # We want SPDX as a // line after the */.
        return ("".join(lines[:insert_at]) + new_block + "".join(lines[insert_at:])).encode()

    # (a, c, d) Detect a top-of-file run of comment lines and insert after.
    end_of_block = insert_at
    if comment == "//":
        marker_prefixes = ("//",)
    elif comment == "#":
        marker_prefixes = ("#",)
    elif comment == "*>":
        # Fortran free-form file headers begin with `*` or `*>`.
        marker_prefixes = ("*",)
    else:
        marker_prefixes = (comment,)

    while end_of_block < len(lines):
        s = lines[end_of_block].lstrip()
        if any(s.startswith(p) for p in marker_prefixes):
            end_of_block += 1
            continue
        break

    if end_of_block > insert_at:
        # Append SPDX after the existing prose block.
        return (
            "".join(lines[:end_of_block])
            + spdx_line.decode()
            + "".join(lines[end_of_block:])
        ).encode()

    # (e) No header — insert at the top (after shebang if present), and
    # follow with a single blank line *unless* the file already starts with
    # one (otherwise clang-format collapses the duplicate).
    next_is_blank = (
        insert_at < len(lines) and lines[insert_at].strip() == ""
    )
    blank = "" if next_is_blank else eol
    return (
        "".join(lines[:insert_at])
        + spdx_line.decode()
        + blank
        + "".join(lines[insert_at:])
    ).encode()


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def tracked_files() -> Iterable[Path]:
    out = subprocess.check_output(
        ["git", "ls-files"], cwd=REPO_ROOT, text=True
    )
    for line in out.splitlines():
        if line:
            yield Path(line)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--check",
        action="store_true",
        help="Don't write; exit 1 if any file would change.",
    )
    p.add_argument(
        "--paths",
        nargs="*",
        help="Limit to these paths (relative to repo root). Default: all tracked.",
    )
    p.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Print every file decision (skip / classify / change).",
    )
    args = p.parse_args()

    if args.paths:
        targets = [Path(p) for p in args.paths]
    else:
        targets = list(tracked_files())

    changed = []
    for rel in targets:
        if is_skipped(rel):
            if args.verbose:
                print(f"[skip-rule] {rel}")
            continue

        full = REPO_ROOT / rel
        prefix = comment_prefix_for(rel, full)
        if prefix is None:
            if args.verbose:
                print(f"[skip-ext]  {rel}")
            continue
        try:
            content = full.read_bytes()
        except (FileNotFoundError, IsADirectoryError):
            continue

        spdx_id = classify(rel, content)
        new_content = insert_spdx(rel, content, spdx_id, prefix)
        if new_content is None:
            if args.verbose:
                print(f"[ok]        {rel}  ({spdx_id})")
            continue

        if args.check:
            print(f"[would-change] {rel}  ({spdx_id})")
            changed.append(rel)
            continue

        full.write_bytes(new_content)
        print(f"[changed]   {rel}  ({spdx_id})")
        changed.append(rel)

    if args.check and changed:
        print(f"\n{len(changed)} file(s) need SPDX headers.", file=sys.stderr)
        return 1
    if not args.check:
        print(f"\nUpdated {len(changed)} file(s).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
