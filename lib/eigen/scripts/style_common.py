#!/usr/bin/env python3
# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

"""Shared plumbing for the added-line style checks.

Both ``check_style.py`` (textual conventions) and ``clang_tidy_hook.py``
(clang-tidy conventions) need the same two things: the set of line numbers a
change ADDED to each file, and the ability to turn a Claude Code PostToolUse
payload into that set.  That logic lives here so the two checkers cannot drift
apart in what they consider an added line.

Run directly to emit a clang-tidy ``--line-filter`` argument for the files a
diff touches, which is how ``ci/scripts/run-clang-tidy.sh`` restricts an MR's
diagnostics to its own lines:

    python3 scripts/style_common.py --line-filter BASE [--] [path ...]
"""

import argparse
import json
import os
import re
import subprocess
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CXX_EXT = {".h", ".hpp", ".hxx", ".cpp", ".cc", ".cxx", ".cu", ".cuh", ".inc"}
# unsupported/Eigen/ still holds the legacy umbrella shims, which are
# extensionless headers too.
EXTENSIONLESS_HEADER_TREES = ("Eigen/", "contrib/Eigen/", "unsupported/Eigen/")


def is_cxx_path(rel_path):
    """Return whether ``rel_path`` is C++ source, including Eigen's
    extensionless public module headers such as ``Eigen/Core``."""
    return (os.path.splitext(rel_path)[1].lower() in CXX_EXT
            or (not os.path.splitext(os.path.basename(rel_path))[1]
                and rel_path.startswith(EXTENSIONLESS_HEADER_TREES)))


def git(root, *args):
    return subprocess.run(["git"] + list(args), cwd=root, capture_output=True, text=True)


def read_post_image(root, rel_path):
    try:
        with open(os.path.join(root, rel_path), encoding="utf-8", errors="replace") as handle:
            return handle.read()
    except OSError:
        return None


def added_lines_from_diff(diff_text):
    """Parse ``git diff -U0`` output into {relative_path: set(added line numbers)}."""
    files = {}
    path, new_line = None, 0
    for raw in diff_text.splitlines():
        if raw.startswith("+++ "):
            name = raw[4:]
            path = None if name == "/dev/null" else name[2:] if name.startswith("b/") else name
        elif raw.startswith("@@"):
            m = re.search(r"\+(\d+)(?:,(\d+))?", raw)
            new_line = int(m.group(1)) if m else 0
        elif raw.startswith("+") and not raw.startswith("+++"):
            if path is not None:
                files.setdefault(path, set()).add(new_line)
            new_line += 1
        elif not raw.startswith("-") and not raw.startswith("\\"):
            # "\ No newline at end of file" is a marker, not a context line.
            new_line += 1
    return files


def added_from_structured_patch(tool_response):
    """Derive added line numbers from the tool response's structured patch,
    which records the exact edited hunks.  A valid deletion-only patch returns
    an empty set; None means absent or malformed data and enables fallback."""
    try:
        added = set()
        for hunk in tool_response["structuredPatch"]:
            line_no = int(hunk["newStart"])
            for entry in hunk["lines"]:
                if entry.startswith("+"):
                    added.add(line_no)
                    line_no += 1
                elif not entry.startswith("-") and not entry.startswith("\\"):
                    # "\ No newline at end of file" is a marker, not a context line.
                    line_no += 1
        return added
    except Exception:
        return None


def hook_added_line_numbers(content, snippets):
    """Map the strings an edit added onto post-image line numbers.

    Each snippet must occur exactly once — a repeated occurrence cannot be
    told apart from pre-existing identical text, and would mark unrelated
    lines.  A snippet's span excludes the line after its trailing newline.
    Returns None when any snippet is absent or ambiguous; the caller then
    checks the snippet text standalone."""
    added = set()
    for snippet in snippets:
        if not snippet:
            continue
        start = content.find(snippet)
        if start < 0 or content.find(snippet, start + 1) >= 0:
            return None
        first = content.count("\n", 0, start) + 1
        span = snippet.count("\n") + (0 if snippet.endswith("\n") else 1)
        added.update(range(first, first + max(span, 1)))
    return added


def hook_edit_snippets(payload):
    """Return (rel_path, snippets) for an edit payload, or (None, None) when
    the tool call is not an in-repository C++ edit this checker handles."""
    tool_input = payload.get("tool_input", {}) or {}
    tool_name = payload.get("tool_name", "")
    path = tool_input.get("file_path", "") or ""
    if not path:
        return None, None
    rel_path = os.path.relpath(os.path.abspath(path), REPO_ROOT).replace(os.sep, "/")
    if rel_path.startswith(".."):
        return None, None
    if tool_name == "Write":
        snippets = [tool_input.get("content", "")]
    elif tool_name == "Edit":
        snippets = [tool_input.get("new_string", "")]
    elif tool_name == "MultiEdit":
        snippets = [e.get("new_string", "") for e in tool_input.get("edits", [])]
    else:
        return None, None
    if not any(s.strip() for s in snippets):
        return None, None
    return rel_path, snippets


def hook_post_image_and_added(payload, rel_path, snippets):
    """Resolve the post-image lines and added line numbers for an edit.

    The hook runs after the edit, so the file on disk is the post-image.  The
    edited lines come from the tool response's structured patch when present,
    else from locating a uniquely occurring added string.  Returns
    (lines, added, exact): ``exact`` is False when the lines could not be
    located in the real file and the snippet text is being checked standalone.
    """
    text = read_post_image(REPO_ROOT, rel_path)
    added = None
    lines = None
    if text is not None:
        lines = text.splitlines()
        added = added_from_structured_patch(payload.get("tool_response") or {})
        if added is None:
            if payload.get("tool_name") == "Write":
                added = set(range(1, len(lines) + 1))
            else:
                added = hook_added_line_numbers(text, snippets)
    if added is None:
        lines = "\n".join(snippets).splitlines()
        return lines, set(range(1, len(lines) + 1)), False
    return lines, added, True


def diff_added_lines(base, root=REPO_ROOT, include_untracked=True):
    """Return {rel_path: set(added line numbers)} for everything added since
    merge-base(base, HEAD), optionally including whole untracked C++ files."""
    merge_base = git(root, "merge-base", base, "HEAD")
    resolved = merge_base.stdout.strip() if merge_base.returncode == 0 else base
    diff = git(root, "diff", "-U0", "--no-color", resolved)
    if diff.returncode not in (0, 1):
        sys.stderr.write(diff.stderr)
        raise SystemExit(2)
    per_file = added_lines_from_diff(diff.stdout)
    if include_untracked:
        untracked = git(root, "ls-files", "--others", "--exclude-standard")
        for rel_path in untracked.stdout.splitlines():
            if is_cxx_path(rel_path):
                text = read_post_image(root, rel_path)
                if text is not None:
                    per_file.setdefault(rel_path, set()).update(range(1, len(text.splitlines()) + 1))
    return per_file


def as_ranges(numbers):
    """Collapse a set of line numbers into sorted inclusive [start, end] pairs."""
    ranges = []
    for number in sorted(numbers):
        if ranges and number == ranges[-1][1] + 1:
            ranges[-1][1] = number
        else:
            ranges.append([number, number])
    return ranges


def line_filter_json(per_file):
    """Build clang-tidy's --line-filter argument from {rel_path: line numbers}.

    clang-tidy matches ``name`` as a path suffix and, when the filter is
    non-empty, reports nothing for files it does not list — so the filter
    doubles as the file filter.  Repository-relative paths are used rather
    than bare basenames, which would collide across modules.
    """
    return json.dumps([{"name": rel_path, "lines": as_ranges(lines)}
                       for rel_path, lines in sorted(per_file.items()) if lines])


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--line-filter", metavar="BASE", required=True,
                        help="emit a clang-tidy --line-filter for lines added since merge-base(BASE, HEAD)")
    parser.add_argument("paths", nargs="*", help="restrict the filter to these repository-relative paths")
    args = parser.parse_args()
    per_file = diff_added_lines(args.line_filter, include_untracked=False)
    if args.paths:
        wanted = set(args.paths)
        per_file = {p: lines for p, lines in per_file.items() if p in wanted}
    print(line_filter_json(per_file))
    return 0


if __name__ == "__main__":
    sys.exit(main())
