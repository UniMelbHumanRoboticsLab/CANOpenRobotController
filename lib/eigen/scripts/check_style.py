#!/usr/bin/env python3
# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

"""Advisory style check for added C++ code, for what clang-tidy cannot say.

Flags the problems that recur in this repository's code reviews and that no
clang-tidy check states: narration-comment verbosity (see the comment rules in
``AGENTS.md``), ``enum`` constant blocks, ``std::integral_constant<bool,...>``,
C++17-and-later constructs in the C++14 trees, and ``std::`` math in library
headers where ``numext::`` is required.  Conventions clang-tidy *can* state
live in ``.clang-tidy`` and are checked by ``clang_tidy_hook.py`` against the
same added lines.  Only lines a change ADDS are reported, but each file's
complete post-image is lexed so surrounding context — an enclosing block
comment, a Doxygen continuation, a multi-line initializer — is classified
correctly.

The findings are advisory: a flagged construct may be justified, in which
case keep it and state the reason where the construct is.

Modes:
  --diff BASE     check lines added relative to ``merge-base(BASE, HEAD)``,
                  including uncommitted changes and untracked C++ files;
                  exit 1 if findings
  --claude-hook   run as a Claude Code PostToolUse hook: read the tool-call
                  JSON from stdin and check the text the edit added; exit 2
                  if findings so the harness feeds them back to the model

The Claude Code hook is registered for this repository in
``.claude/settings.json``.  Other agents and humans can run the diff mode
directly, e.g. ``python3 scripts/check_style.py --diff origin/master``.
"""

import argparse
import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from style_common import (REPO_ROOT, added_lines_from_diff, added_from_structured_patch,  # noqa: E402,F401
                          diff_added_lines, hook_added_line_numbers, hook_edit_snippets,
                          hook_post_image_and_added, is_cxx_path, read_post_image)

# Trees whose headers and tests must compile as C++14 (see AGENTS.md rule 3).
# unsupported/Eigen/ is kept for the legacy umbrella shims that still live there;
# the implementation tree itself is contrib/.
CXX14_TREES = ("Eigen/", "contrib/Eigen/", "unsupported/Eigen/", "test/", "contrib/test/", "failtest/", "blas/",
               "lapack/")
# Guarded exceptions with a documented newer requirement: SYCL sources build
# only in the C++17 SYCL configurations, while duccfft and the structured-
# binding failtests explicitly select C++17 in their test CMakeLists files.
CXX17_PREFIXES = ("Eigen/src/Core/arch/SYCL/",)
CXX17_FILES = {
    "test/sycl_basic.cpp",
    "contrib/Eigen/src/FFT/duccfft_impl.h",
    "contrib/test/duccfft.cpp",
    "failtest/structured_bindings_dynamic_matrix.cpp",
    "failtest/structured_bindings_dynamic_array.cpp",
    "failtest/structured_bindings_rowmajor.cpp",
}
# Library implementation headers, where numext:: is required over std:: math.
LIBRARY_SRC_TREES = ("Eigen/src/", "contrib/Eigen/src/")

DOXYGEN = re.compile(r"^\s*(/\*\*|/\*!|///|//!)")
LICENSE = re.compile(r"SPDX|Copyright|License|Mozilla Public", re.I)
# A bibliography is provenance, which AGENTS.md keeps, and it is long by nature: one entry per
# work cited already exceeds the block threshold. Matched on the block's own header line so that
# ordinary prose mentioning a reference is still counted.
REFERENCES = re.compile(r"^\s*(?://|/\*)?\s*References:\s*$", re.M)
STD_MATH = (
    r"abs|arg|conj|real|imag|sqrt|cbrt|isnan|isinf|isfinite|signbit|exp|exp2|expm1|log|log1p|log2|pow|"
    r"sin|cos|tan|asin|acos|atan|atan2|asinh|acosh|atanh|sinh|cosh|tanh|hypot|fma|copysign|ldexp|floor|"
    r"ceil|round|rint|trunc|fmod"
)

# Convention checks applied per added code line (comments and literals blanked).
#
# Only conventions clang-tidy cannot currently state belong here; NULL and
# typedef moved to modernize-use-nullptr and modernize-use-using in
# .clang-tidy, which clang_tidy_hook.py and ci/scripts/run-clang-tidy.sh run
# against the same added lines.  The entries below are the remainder: those
# needing a CustomChecks query (parked in .clang-tidy until CI's clang-tidy
# reaches 22) and those no AST matcher reaches at all.
#
# One known gap in that handover: modernize-use-using in clang-tidy 18 (the
# checkformat:clangtidy toolchain) reports typedefs at class and namespace
# scope but not function-local ones (measured — Block.h 17 of 18, Redux.h 24
# of 25, GeneralMatrixMatrix.h 21 of 28).  That was an upstream defect,
# llvm/llvm-project#72179, fixed in the 19.x release, so the gap closes when
# the CI image's clang-tidy moves; until then a function-local typedef in new
# code goes unreported.  Reintroducing a regex for it would bring back the
# false positives this handover removes.  modernize-use-nullptr has no gap.
CODE_CHECKS = [
    # Awaiting the parked eigen-bool-constant query.
    (r"std::integral_constant<\s*bool\b", "std::integral_constant<bool,...>: use Eigen's bool_constant (Meta.h)"),
    # Awaiting the parked eigen-enum-constant query.  No stock check states
    # this rule: cppcoreguidelines-use-enum-class asks for `enum class`,
    # whereas Eigen wants static constexpr for trait constants.
    (r"^\s*enum\s*(?::[^{}]+)?\{", "enum constant block: trait/evaluator constants are static constexpr in new "
                                   "code; Flags is `unsigned int`"),
]
CXX14_CHECKS = [
    # Awaiting the parked eigen-if-constexpr query.  A C++14 build does not
    # reliably reject this on its own: clang accepts `if constexpr` as an
    # extension and only warns under -Wc++17-extensions.
    (r"\bif\s+constexpr\b", "if constexpr: supported code compiles as C++14; use EIGEN_IF_CONSTEXPR (...) with a "
                            "condition that is valid either way"),
    # These do fail the default C++14 build; flagging them at edit time just
    # shortens the loop, and covers headers no built target happens to include.
    (r"\bstd::(any|span|optional|variant|string_view|byte|filesystem|void_t)\b",
     "C++17/20 library facility: the supported baseline is C++14"),
]
# Also C++14-tree only (CMakeLists.txt defaults CMAKE_CXX_STANDARD to 14), and
# not expressible as a matcher — designated initializers do not parse in C++14,
# so there is no AST for a check to match.
DESIGNATED_MSG = "designated initializer: C++20-only; use aggregate assignment with /*name=*/ comments"


RAW_PREFIX = re.compile(r"(?:^|[^0-9A-Za-z_])(?:u8|u|U|L)?R$")


def is_digit_separator(line, i):
    """Return whether the apostrophe at ``i`` belongs to a C++ number."""
    if i + 1 >= len(line) or not line[i + 1].isalnum():
        return False
    start = i
    while start > 0 and (line[start - 1].isalnum() or line[start - 1] in "._'"):
        start -= 1
    token = line[start:i]
    return bool(token) and (token[0].isdigit() or (len(token) > 1 and token[0] == "." and token[1].isdigit()))


def scan_quoted(line, j, quote):
    """Scan a quoted literal's remainder from position ``j``.

    Returns (next_position, closed, spliced): ``spliced`` is True when the
    line ends with a backslash inside the literal, so the literal continues
    on the next physical line.
    """
    n = len(line)
    while j < n:
        if line[j] == "\\":
            if j == n - 1:
                return n, False, True
            j += 2
            continue
        if line[j] == quote:
            return j + 1, True, False
        j += 1
    return n, False, False


def lex_lines(lines):
    """Lex a file's complete contiguous text.

    Returns (code_lines, comment_only, doxygen_only) as parallel lists: per
    line, the code with comments and string/character literals blanked, the
    stripped comment text when the line holds no code (else None), and whether
    that comment belongs to Doxygen.  Block comments, raw string literals, and
    backslash-spliced ordinary literals carry their state across physical
    lines.
    """
    code_lines, comment_only, doxygen_only = [], [], []
    in_block = False
    block_doxygen = False
    raw_terminator = None  # e.g. )delim" while inside a raw string literal
    open_quote = None      # quote character of a spliced ordinary literal
    for line in lines:
        out, i, n = [], 0, len(line)
        had_code = False
        had_comment = in_block
        line_doxygen = block_doxygen if in_block else False
        literal_at_start = raw_terminator is not None or open_quote is not None
        while i < n:
            if in_block:
                j = line.find("*/", i)
                if j < 0:
                    i = n
                else:
                    in_block = False
                    block_doxygen = False
                    i = j + 2
                continue
            if raw_terminator is not None:
                j = line.find(raw_terminator, i)
                if j < 0:
                    i = n
                else:
                    i = j + len(raw_terminator)
                    raw_terminator = None
                continue
            if open_quote is not None:
                i, closed, spliced = scan_quoted(line, i, open_quote)
                if closed or not spliced:  # an unspliced open literal is ill-formed; close at EOL
                    open_quote = None
                continue
            c = line[i]
            if c == "/" and i + 1 < n and line[i + 1] == "/":
                had_comment = True
                line_doxygen = line_doxygen or bool(DOXYGEN.match(line[i:]))
                break
            if c == "/" and i + 1 < n and line[i + 1] == "*":
                had_comment = True
                block_doxygen = line.startswith(("/**", "/*!"), i)
                line_doxygen = line_doxygen or block_doxygen
                in_block = True
                i += 2
                continue
            if c == '"' and RAW_PREFIX.search(line[:i]):
                paren = line.find("(", i + 1)
                delimiter = line[i + 1:paren] if paren >= 0 else None
                if delimiter is not None and len(delimiter) <= 16 and not re.search(r"[()\\\s]", delimiter):
                    out.append('""')
                    had_code = True
                    raw_terminator = ")" + delimiter + '"'
                    j = line.find(raw_terminator, paren + 1)
                    if j < 0:
                        i = n
                    else:
                        i = j + len(raw_terminator)
                        raw_terminator = None
                    continue
            if c == "'" and is_digit_separator(line, i):
                # C++14 digit separator inside a preprocessing-number, not a
                # character-literal delimiter (for example 1'000 or 0xFF'00).
                out.append(c)
                had_code = True
                i += 1
                continue
            if c in "\"'":
                out.append(c + c)
                had_code = True
                i, closed, spliced = scan_quoted(line, i + 1, c)
                if not closed and spliced:
                    open_quote = c
                continue
            out.append(c)
            if not c.isspace():
                had_code = True
            i += 1
        stripped = line.strip()
        is_comment = (not had_code) and had_comment and not literal_at_start
        code_lines.append("".join(out))
        comment_only.append(stripped if is_comment else None)
        doxygen_only.append(line_doxygen if is_comment else None)
    return code_lines, comment_only, doxygen_only


def check_comments(comment_only, doxygen_only, added, findings):
    """Flag comment blocks that gain six or more added narration lines.

    Blocks are formed over the full file, so an addition inside an existing
    Doxygen, license or bibliography block inherits that block's exemption, and
    only the ADDED lines count toward the threshold — extending a pre-existing
    block by a line or two is not reported.
    """
    def finish_block(block_start, block_end, is_doxygen):
        block = comment_only[block_start:block_end]
        added_in_block = [line_no for line_no in range(block_start + 1, block_end + 1) if line_no in added]
        text = "\n".join(block)
        exempt = is_doxygen or LICENSE.search(text) or REFERENCES.search(text)
        if added_in_block and len(added_in_block) >= 6 and not exempt:
            findings.append((added_in_block[0], "%d added lines of non-Doxygen comment: AGENTS.md keeps only "
                                                "mathematics, invariants, compatibility constraints, provenance, "
                                                "or the reason a deliberate construct must not be simplified"
                                                % len(added_in_block)))

    block_start, block_doxygen = None, None
    for idx in range(len(comment_only) + 1):
        comment = comment_only[idx] if idx < len(comment_only) else None
        if comment is not None:
            if block_start is None:
                block_start = idx
                block_doxygen = doxygen_only[idx]
            elif doxygen_only[idx] != block_doxygen:
                finish_block(block_start, idx, block_doxygen)
                block_start = idx
                block_doxygen = doxygen_only[idx]
        elif block_start is not None:
            finish_block(block_start, idx, block_doxygen)
            block_start = None


def check_designated_initializer(code_lines, line_no, findings):
    code = code_lines[line_no - 1]
    hit = re.search(r"[{,]\s*\.\w+\s*=", code)
    if not hit and re.match(r"\s*\.\w+\s*=", code):
        # A line-leading designator continues a braced initializer only when
        # the previous code line ends with `{` or `,`; anything else is a
        # wrapped member access or assignment.
        for prev in range(line_no - 2, -1, -1):
            prev_code = code_lines[prev].rstrip()
            if prev_code:
                hit = prev_code.endswith("{") or prev_code.endswith(",")
                break
    if hit:
        findings.append((line_no, DESIGNATED_MSG))
        return True
    return False


def check_conventions(rel_path, code_lines, added, findings):
    checks = list(CODE_CHECKS)
    cxx17_path = (
        rel_path in CXX17_FILES
        or rel_path.startswith(CXX17_PREFIXES)
        or (rel_path.startswith("contrib/Eigen/src/Tensor/") and rel_path.endswith("Sycl.h"))
        or (rel_path.startswith("contrib/test/") and rel_path.endswith("_sycl.cpp"))
    )
    if rel_path.startswith(CXX14_TREES) and not cxx17_path:
        checks += CXX14_CHECKS
    if rel_path.startswith(LIBRARY_SRC_TREES):
        checks.append((r"(?:\(\s*)?\bstd::(?:%s)\s*(?:\)\s*)?\(" % STD_MATH,
                       "std:: math call in a library header: use the numext:: equivalent "
                       "(device- and custom-scalar-aware)"))
        checks.append((r"(?:\(\s*)?\bstd::frexp\s*(?:\)\s*)?\(",
                       "std::frexp call in a library header: use EIGEN_USING_STD(frexp) and an unqualified call "
                       "for ADL and device/custom-scalar support"))
    added_sorted = sorted(added)
    for pattern, message in checks:
        rx = re.compile(pattern)
        for line_no in added_sorted:
            if rx.search(code_lines[line_no - 1]):
                findings.append((line_no, message))
                break  # one report per pattern per file
    for line_no in added_sorted:
        if check_designated_initializer(code_lines, line_no, findings):
            break


def find_findings(rel_path, lines, added):
    """Check one file: ``lines`` is the complete post-image, ``added`` the set
    of 1-based line numbers the change added.  Returns [(line_no, message)]."""
    if not is_cxx_path(rel_path):
        return []
    added = {n for n in added if 1 <= n <= len(lines) and lines[n - 1].strip()}
    if not added:
        return []
    code_lines, comment_only, doxygen_only = lex_lines(lines)
    findings = []
    check_comments(comment_only, doxygen_only, added, findings)
    check_conventions(rel_path, code_lines, added, findings)
    findings.sort()
    return findings


def run_diff_mode(base, root=REPO_ROOT):
    """Return [(rel_path, line_no, message)] for lines added since
    merge-base(base, HEAD), including untracked C++ files."""
    per_file = diff_added_lines(base, root)
    results = []
    for rel_path in sorted(per_file):
        text = read_post_image(root, rel_path)
        if text is None:  # deleted or unreadable
            continue
        for line_no, message in find_findings(rel_path, text.splitlines(), per_file[rel_path]):
            results.append((rel_path, line_no, message))
    return results


def run_hook_mode():
    try:
        payload = json.load(sys.stdin)
    except Exception:
        return 0
    rel_path, snippets = hook_edit_snippets(payload)
    if rel_path is None:
        return 0
    # The hook runs after the edit, so the file on disk is the post-image; it is
    # lexed whole so enclosing comments and initializers classify correctly.
    lines, added, _ = hook_post_image_and_added(payload, rel_path, snippets)
    findings = find_findings(rel_path, lines, added)
    if findings:
        sys.stderr.write("style check (%s) — review before proceeding:\n" % rel_path)
        for _, message in findings[:8]:
            sys.stderr.write("  - %s\n" % message)
        sys.stderr.write("Advisory: keep a flagged construct only if it is justified, and say why at the "
                         "construct.\n")
        return 2
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--diff", metavar="BASE", help="check lines added relative to merge-base(BASE, HEAD)")
    mode.add_argument("--claude-hook", action="store_true", help="run as a Claude Code PostToolUse hook")
    args = parser.parse_args()
    if args.claude_hook:
        try:
            return run_hook_mode()
        except Exception:
            return 0  # a broken hook must not block the harness
    results = run_diff_mode(args.diff)
    for rel_path, line_no, message in results:
        print("%s:%d: %s" % (rel_path, line_no, message))
    if results:
        print("\n%d advisory finding(s) in added lines. Keep a flagged construct only if it is justified, "
              "and say why at the construct." % len(results))
    return 1 if results else 0


if __name__ == "__main__":
    sys.exit(main())
