#!/usr/bin/env python3
# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

"""Narrow a compilation database to the entries a changed source needs.

``ci/scripts/run-clang-tidy.sh`` checks a changed source through the build
directory's ``compile_commands.json``.  clang-tidy parses the file once per
entry naming it, and a split test contributes one entry per ``EIGEN_TEST_PART``
-- 17 for ``test/eigensolver_selfadjoint.cpp``, 41 for ``test/array_cwise.cpp``,
at roughly a minute and 3 GB each.  A merge request touching such a file spends
the job's entire timeout on it, and every file after it goes unchecked without
saying so.

Two entries are interchangeable only when they differ solely in which
``-DEIGEN_TEST_PART_<n>`` they define.  ``blas/complexdots.cpp`` is compiled
both into the static library and, with ``-DEIGEN_BLAS_BUILD_DLL``, into the
shared one; code that only the second configuration enables is parsed only by
the second entry.  Entries are therefore grouped by the arguments that affect
parsing, every group keeps a representative, and only inside a group is a part
chosen.

Within a group the part matters exactly where the source asks for it.  A line
inside a ``CALL_SUBTEST_<n>(...)`` invocation or an
``#if defined(EIGEN_TEST_PART_<n>)`` guard exists only when part ``<n>`` is
defined -- the split helper expands the invocation to nothing otherwise.  Every
other line is parsed under any part, because the checks ``.clang-tidy`` enables
are syntactic: they fire on declarations rather than on template
instantiations.  So each added line is mapped to the parts that can compile it,
and the smallest set of entries covering all of them is retained, up to
``MAX_PARTS``.  Whatever that leaves unchecked is named on stdout: reporting a
file clean without having parsed it is the failure this replaces.

Usage: tidy_compile_db.py <database> <source> <outdir> <line-filter>

``<line-filter>`` is the clang-tidy ``--line-filter`` argument that
``scripts/style_common.py`` builds for the same source.  Writes the reduced
database to ``<outdir>/compile_commands.json`` and prints a one-line summary
when it dropped anything.  Exits 0 when the source is in the database, 1 when
it is not, and 2 when the database cannot be read.
"""

import bisect
import json
import os
import re
import shlex
import sys

# A part costs about a minute and 2 GB, against a 15 minute job budget shared
# with every other changed file, so cover as many parts as a diff needs only up
# to this many.  Beyond it the summary line names the parts left unchecked
# rather than letting the job run into its timeout, which is the silent failure
# this whole reduction exists to remove.
MAX_PARTS = 4

# Arguments that name an output rather than describe the parse. They differ per
# split part (`bdcsvd_1.dir` against `bdcsvd_2.dir`) and would defeat grouping.
_VALUE_ARGS = ("-o", "-MT", "-MF", "-MQ", "-MJ")
_OUTPUT_FLAGS = ("-c", "-MD", "-MMD", "-MP")
_PART_DEFINE = re.compile(r"^-DEIGEN_TEST_PART_([0-9]+)(=.*)?$")
_PART_ARG = re.compile(r"-DEIGEN_TEST_PART_([0-9]+)\b")
_DIRECTIVE = re.compile(r"[ \t]*#[ \t]*(if|ifdef|ifndef|elif|else|endif)\b(.*)")
_PART_MENTION = re.compile(r"(!\s*)?EIGEN_TEST_PART_([0-9]+|ALL)\b")
_DEFINED_CALL = re.compile(r"defined\s*\(\s*([A-Za-z_]\w*)\s*\)")
_DEFINED_BARE = re.compile(r"defined\s+([A-Za-z_]\w*)")
_CALL_SUBTEST = re.compile(r"\bCALL_SUBTEST_([0-9]+)\s*\(")


def arguments_of(entry):
    """The compile command of a database entry, as an argument list."""
    command = entry.get("command")
    if command:
        return shlex.split(command)
    return list(entry.get("arguments", ()))


def part_of(entry):
    """The ``EIGEN_TEST_PART`` an entry defines, as a string, or None."""
    found = _PART_ARG.search(entry.get("command") or " ".join(entry.get("arguments", ())))
    return found.group(1) if found else None


def configuration_of(entry):
    """The arguments of an entry that affect how the source is parsed.

    Output paths and the part define are excluded, so entries that differ only
    in which part they build compare equal and nothing else does.
    """
    configuration = []
    skip = False
    for argument in arguments_of(entry):
        if skip:
            skip = False
        elif argument in _VALUE_ARGS:
            skip = True
        elif argument in _OUTPUT_FLAGS or (argument.startswith("-o") and len(argument) > 2):
            continue
        elif not _PART_DEFINE.match(argument):
            configuration.append(argument)
    return tuple(configuration)


def entries_for(commands, source):
    """The database entries compiling ``source``, in database order."""
    source = os.path.realpath(source)
    matches = []
    for entry in commands:
        if not isinstance(entry, dict):
            continue
        path = entry.get("file")
        if not path:
            continue
        if not os.path.isabs(path):
            path = os.path.join(entry.get("directory", ""), path)
        if os.path.realpath(path) == source:
            matches.append(entry)
    return matches


def _part_mentions(condition, negated=False):
    """The parts a preprocessor condition names, split by sense.

    ``defined`` is elided first so a mention is negated exactly when a ``!``
    precedes it.  ``EIGEN_TEST_PART_ALL`` is one of the mentions: no entry of a
    split test defines it, so a branch that only it can reach is compiled by no
    part rather than by every part.
    """
    text = _DEFINED_CALL.sub(r"\1", condition)
    text = _DEFINED_BARE.sub(r"\1", text)
    positive, negative = set(), set()
    for mention in _PART_MENTION.finditer(text):
        target = negative if bool(mention.group(1)) != negated else positive
        target.add(mention.group(2))
    return positive, negative


def _branch(positive, negative, universe):
    """The parts under which a branch with these mentions is compiled.

    A positive mention is the binding one: ``defined(EIGEN_TEST_PART_3) && X``
    still needs part 3, and ``defined(EIGEN_TEST_PART_3) || X`` is at least
    compiled under part 3, which is all the caller needs to reach the line.
    """
    if positive:
        return universe & positive
    if negative:
        return universe - negative
    return universe


def guard_constraints(lines, universe):
    """Map each 1-based line number to the parts its ``#if`` guards allow."""
    per_line = [universe] * (len(lines) + 2)
    cumulative = [universe]
    # Parts already claimed by an earlier branch of each open conditional, so
    # that an `#else` can exclude them.
    taken = []
    index = 0
    while index < len(lines):
        first = index
        directive = _DIRECTIVE.match(lines[index])
        condition = ""
        if directive:
            condition = directive.group(2)
            while condition.rstrip().endswith("\\") and index + 1 < len(lines):
                index += 1
                condition = condition.rstrip()[:-1] + lines[index]
        for number in range(first + 1, index + 2):
            per_line[number] = cumulative[-1]
        if directive:
            keyword = directive.group(1)
            if keyword in ("if", "ifdef", "ifndef"):
                positive, negative = _part_mentions(condition, negated=keyword == "ifndef")
                branch = _branch(positive, negative, universe)
                taken.append(branch if positive or negative else frozenset())
                cumulative.append(cumulative[-1] & branch)
            elif keyword in ("elif", "else") and taken:
                cumulative.pop()
                if keyword == "elif":
                    positive, negative = _part_mentions(condition)
                    branch = _branch(positive, negative, universe)
                    taken[-1] = taken[-1] | (branch if positive or negative else frozenset())
                else:
                    branch = universe - taken[-1]
                cumulative.append(cumulative[-1] & branch)
            elif keyword == "endif" and taken:
                taken.pop()
                cumulative.pop()
        index += 1
    return per_line


def call_constraints(text, universe):
    """Map each 1-based line number to the parts its ``CALL_SUBTEST`` allows.

    A line can carry more than one invocation, in which case either part
    compiles part of it.
    """
    starts = [0] + [newline.end() for newline in re.finditer(r"\n", text)]
    per_line = {}
    for call in _CALL_SUBTEST.finditer(text):
        depth, offset = 1, call.end()
        while offset < len(text) and depth:
            if text[offset] == "(":
                depth += 1
            elif text[offset] == ")":
                depth -= 1
            offset += 1
        first = bisect.bisect_right(starts, call.start())
        last = bisect.bisect_right(starts, offset - 1)
        allowed = universe & {call.group(1)}
        for number in range(first, last + 1):
            per_line[number] = per_line.get(number, frozenset()) | allowed
    return per_line


def required_parts(text, lines, universe):
    """The parts that can compile each of ``lines``, as a list of sets."""
    source_lines = text.splitlines()
    guards = guard_constraints(source_lines, universe)
    calls = call_constraints(text, universe)
    return [guards[number] & calls.get(number, universe)
            for number in lines if number < len(guards)]


def select_parts(required, order, limit=MAX_PARTS):
    """The smallest set of parts covering ``required``, capped at ``limit``.

    Returns the chosen parts and the requirements the cap left uncovered. Ties
    go to the part the database lists first, which is what a diff that names no
    part at all gets.
    """
    chosen = []
    uncovered = [need for need in required if need]
    while uncovered and len(chosen) < limit:
        counts = {}
        for need in uncovered:
            for part in need:
                counts[part] = counts.get(part, 0) + 1
        best = min(counts, key=lambda part: (-counts[part], order.index(part)))
        chosen.append(best)
        uncovered = [need for need in uncovered if best not in need]
    return chosen, uncovered


def reduce_entries(matches, text, lines):
    """The entries of ``matches`` needed to parse ``lines`` of ``text``.

    Returns the retained entries in database order, the parts the cap left out,
    and how many added lines no configured part compiles at all.
    """
    groups = {}
    for index, entry in enumerate(matches):
        groups.setdefault(configuration_of(entry), []).append(index)

    keep, skipped, unreachable = set(), set(), 0
    for group in groups.values():
        parts = [part_of(matches[index]) for index in group]
        if len(group) == 1 or None in parts or len(set(parts)) != len(parts):
            # Not one configuration split into parts: every entry here parses
            # something the others do not.
            keep.update(group)
            continue
        universe = frozenset(parts)
        required = required_parts(text, lines, universe)
        unreachable = max(unreachable, sum(1 for need in required if not need))
        chosen, uncovered = select_parts(required, parts)
        skipped |= {part for need in uncovered for part in need}
        keep.update(index for index, part in zip(group, parts) if part in chosen)
    return [matches[index] for index in sorted(keep)], sorted(skipped, key=int), unreachable


def added_lines(line_filter):
    """The line numbers a clang-tidy ``--line-filter`` argument marks."""
    lines = []
    for item in json.loads(line_filter or "[]"):
        for first, last in item.get("lines", ()):
            lines.extend(range(first, last + 1))
    return sorted(set(lines))


def summarize(matches, keep, skipped, unreachable):
    """A one-line report of what the reduction dropped, or an empty string.

    What the reduction leaves out is stated rather than implied: a job that
    prints "all clean" for a file it never parsed is the failure this replaces.
    """
    if len(keep) == len(matches) and not skipped and not unreachable:
        return ""
    parts = sorted((part for part in (part_of(entry) for entry in keep) if part), key=int)
    if not parts:
        checking = "%d entries" % len(keep)
    else:
        checking = "part%s %s" % ("" if len(parts) == 1 else "s", ", ".join(parts))
    report = "(%d compilation-database entries; checking %s" % (len(matches), checking)
    if skipped:
        report += "; NOT CHECKED: added lines also need part%s %s" % (
            "" if len(skipped) == 1 else "s", ", ".join(skipped))
    if unreachable:
        report += "; NOT CHECKED: %d added line(s) belong to no configured part" % unreachable
    return report + ")"


def main(argv):
    database, source, outdir, line_filter = argv[1:5]
    try:
        with open(database, encoding="utf-8") as handle:
            commands = json.load(handle)
    except (OSError, ValueError) as error:
        print("ERROR: could not read %s: %s" % (database, error), file=sys.stderr)
        return 2
    if not isinstance(commands, list):
        print("ERROR: %s does not contain a JSON array" % database, file=sys.stderr)
        return 2

    matches = entries_for(commands, source)
    if not matches:
        return 1

    with open(source, encoding="utf-8", errors="replace") as handle:
        text = handle.read()
    keep, skipped, unreachable = reduce_entries(matches, text, added_lines(line_filter))

    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, "compile_commands.json"), "w", encoding="utf-8") as handle:
        json.dump(keep, handle)
    report = summarize(matches, keep, skipped, unreachable)
    if report:
        print(report)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
