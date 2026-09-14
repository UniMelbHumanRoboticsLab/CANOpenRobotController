#!/usr/bin/env python3
# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

"""Unit tests for scripts/tidy_compile_db.py.

The reduction decides what clang-tidy parses, so the cases that matter are the
ones where dropping an entry would drop an added line with it: a split part
selected by a guard rather than by the added text, several parts named at once,
and a second build configuration of the same source.  Synthetic databases keep
the tests independent of a configured build directory; one case reads
`test/bdcsvd.cpp` from the tree so the guard shapes stay real.

Usage: python3 scripts/test_tidy_compile_db.py
"""

import json
import os
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from style_common import REPO_ROOT  # noqa: E402
from tidy_compile_db import (MAX_PARTS, added_lines, configuration_of, part_of,  # noqa: E402
                             reduce_entries, required_parts, summarize)

SCRIPT = os.path.join(REPO_ROOT, "scripts", "tidy_compile_db.py")


def entry(source, part=None, extra=(), output="obj"):
    """A compilation-database entry the way CMake's Ninja generator writes one."""
    flags = " ".join(list(extra) + (["-DEIGEN_TEST_PART_%s=1" % part] if part else []))
    return {"directory": "/build", "file": source,
            "command": "/usr/bin/c++ %s -I/repo -std=c++14 -o %s -c %s" % (flags, output, source),
            "output": output}


def parts_kept(matches, text, lines):
    keep, _, _ = reduce_entries(matches, text, lines)
    return [part_of(kept) for kept in keep]


def test_part_define_does_not_distinguish_configurations():
    """Entries differing only in the part share one configuration."""
    first = entry("/repo/test/t.cpp", part="1", output="t_1.o")
    second = entry("/repo/test/t.cpp", part="2", output="t_2.o")
    assert configuration_of(first) == configuration_of(second)
    assert part_of(first) == "1" and part_of(second) == "2"
    # A macro that is not a part define does distinguish them, and so does no
    # define at all.
    shared = entry("/repo/blas/b.cpp", extra=["-DEIGEN_BLAS_BUILD_DLL", "-fPIC"])
    assert configuration_of(shared) != configuration_of(entry("/repo/blas/b.cpp"))


def test_unguarded_lines_collapse_to_one_part():
    matches = [entry("/repo/test/t.cpp", part=str(n), output="t_%d.o" % n) for n in (1, 2, 3)]
    text = "void f() {\n  int i = 0;\n}\n"
    assert parts_kept(matches, text, [2]) == ["1"], parts_kept(matches, text, [2])


def test_guarded_line_selects_its_part():
    """The added text names no part; the guard around it does.

    This is the case a scan of the diff alone gets wrong: with part 1 selected,
    the preprocessor removes the whole changed region.
    """
    matches = [entry("/repo/test/t.cpp", part=str(n), output="t_%d.o" % n) for n in (1, 2, 3)]
    text = ("#if defined(EIGEN_TEST_PART_3) || defined(EIGEN_TEST_PART_ALL)\n"
            "void guarded() { int* p = 0; }\n"
            "#endif\n")
    assert parts_kept(matches, text, [2]) == ["3"]


def test_every_named_part_is_retained():
    """Additions naming two parts keep both, not just the first."""
    matches = [entry("/repo/test/t.cpp", part=str(n), output="t_%d.o" % n) for n in (1, 2, 3)]
    text = ("void run() {\n"
            "  CALL_SUBTEST_1(f<int>());\n"
            "  CALL_SUBTEST_2(g<int>());\n"
            "}\n")
    assert parts_kept(matches, text, [2, 3]) == ["1", "2"]


def test_multi_line_call_subtest_covers_its_continuation():
    matches = [entry("/repo/test/t.cpp", part=str(n), output="t_%d.o" % n) for n in (1, 2)]
    text = ("void run() {\n"
            "  CALL_SUBTEST_2((f<Matrix<float, 1, 1>,\n"
            "                    int>()));\n"
            "}\n")
    # Line 3 carries no part of its own; it is inside part 2's invocation.
    assert parts_kept(matches, text, [3]) == ["2"]


def test_elif_chain_and_else_branch():
    text = ("#if defined EIGEN_TEST_PART_1\n"
            "int first;\n"
            "#elif defined EIGEN_TEST_PART_2\n"
            "int second;\n"
            "#else\n"
            "int rest;\n"
            "#endif\n")
    universe = frozenset(["1", "2", "3"])
    assert required_parts(text, [2], universe) == [frozenset(["1"])]
    assert required_parts(text, [4], universe) == [frozenset(["2"])]
    # The `#else` is compiled by every part the branches above did not claim.
    assert required_parts(text, [6], universe) == [frozenset(["3"])]


def test_guard_condition_spanning_continuation_lines():
    text = ("#if defined(EIGEN_TEST_PART_2) || \\\n"
            "    defined(EIGEN_TEST_PART_3)\n"
            "int guarded;\n"
            "#endif\n")
    universe = frozenset(["1", "2", "3"])
    assert required_parts(text, [3], universe) == [frozenset(["2", "3"])]


def test_condition_that_only_restricts_other_macros():
    """A guard naming no part constrains no part, inside or after it."""
    text = ("#ifdef __clang__\n"
            "int a;\n"
            "#else\n"
            "int b;\n"
            "#endif\n"
            "int c;\n")
    universe = frozenset(["1", "2"])
    assert required_parts(text, [2, 4, 6], universe) == [universe, universe, universe]


def test_nested_guards_intersect():
    text = ("#if defined(EIGEN_TEST_PART_1) || defined(EIGEN_TEST_PART_2)\n"
            "#ifndef EIGEN_TEST_PART_2\n"
            "int only_one;\n"
            "#endif\n"
            "#endif\n")
    assert required_parts(text, [3], frozenset(["1", "2", "3"])) == [frozenset(["1"])]


def test_cap_reports_the_parts_it_leaves_out():
    count = MAX_PARTS + 2
    matches = [entry("/repo/test/t.cpp", part=str(n), output="t_%d.o" % n)
               for n in range(1, count + 1)]
    text = "void run() {\n" + "".join("  CALL_SUBTEST_%d(f());\n" % n
                                      for n in range(1, count + 1)) + "}\n"
    lines = list(range(2, count + 2))
    keep, skipped, unreachable = reduce_entries(matches, text, lines)
    assert len(keep) == MAX_PARTS, len(keep)
    assert len(skipped) == count - MAX_PARTS, skipped
    assert not unreachable
    # A capped run must say so rather than report the file as clean.
    report = summarize(matches, keep, skipped, unreachable)
    assert "NOT CHECKED" in report and skipped[0] in report, report


def test_lines_no_configured_part_compiles_are_reported():
    matches = [entry("/repo/test/t.cpp", part=str(n), output="t_%d.o" % n) for n in (1, 2)]
    text = ("#if defined(EIGEN_TEST_PART_ALL)\n"
            "int unbuilt;\n"
            "#endif\n")
    keep, skipped, unreachable = reduce_entries(matches, text, [2])
    assert unreachable == 1 and not skipped
    assert "no configured part" in summarize(matches, keep, skipped, unreachable)


def test_distinct_configurations_are_all_kept():
    """The static and shared builds of a BLAS source parse different code."""
    static = entry("/repo/blas/b.cpp", output="static.o")
    shared = entry("/repo/blas/b.cpp", extra=["-DEIGEN_BLAS_BUILD_DLL", "-fPIC"], output="shared.o")
    matches = [static, shared]
    text = ("#if defined(EIGEN_BLAS_BUILD_DLL)\n"
            "typedef int OnlyInTheSharedBuild;\n"
            "#endif\n")
    keep, skipped, unreachable = reduce_entries(matches, text, [2])
    assert keep == matches, keep
    assert not skipped and not unreachable
    assert summarize(matches, keep, skipped, unreachable) == ""


def test_configurations_and_parts_combine():
    """Each configuration keeps its own part selection."""
    matches = ([entry("/repo/test/t.cpp", part=str(n), output="plain_%d.o" % n) for n in (1, 2)]
               + [entry("/repo/test/t.cpp", part=str(n), extra=["-DEIGEN_DONT_VECTORIZE"],
                        output="novec_%d.o" % n) for n in (1, 2)])
    text = ("void run() {\n"
            "  CALL_SUBTEST_2(f());\n"
            "}\n")
    keep, _, _ = reduce_entries(matches, text, [2])
    assert [(part_of(kept), kept["output"]) for kept in keep] == [("2", "plain_2.o"),
                                                                  ("2", "novec_2.o")]


def test_bdcsvd_guard_in_the_real_tree():
    """The tree's own `EIGEN_TEST_PART_53` block, as the review reported it."""
    path = os.path.join(REPO_ROOT, "test", "bdcsvd.cpp")
    with open(path, encoding="utf-8") as handle:
        text = handle.read()
    lines = text.splitlines()
    guarded = next(number for number, line in enumerate(lines, 1)
                   if line.startswith("void bdcsvd_extreme_scale_regressions"))
    universe = frozenset(str(n) for n in range(1, 60))
    assert required_parts(text, [guarded], universe) == [frozenset(["53"])], guarded


def test_added_lines_expands_the_line_filter():
    assert added_lines('[{"name": "test/t.cpp", "lines": [[3, 5], [9, 9]]}]') == [3, 4, 5, 9]
    assert added_lines("[]") == []


def test_command_line_writes_the_reduced_database():
    with tempfile.TemporaryDirectory(prefix="tidy_compile_db_test_") as tmp:
        source = os.path.join(tmp, "t.cpp")
        with open(source, "w") as handle:
            handle.write("#if defined(EIGEN_TEST_PART_3)\nint guarded;\n#endif\n")
        database = os.path.join(tmp, "compile_commands.json")
        with open(database, "w") as handle:
            json.dump([entry(source, part=str(n), output="t_%d.o" % n) for n in (1, 2, 3)], handle)
        outdir = os.path.join(tmp, "reduced")

        line_filter = json.dumps([{"name": "t.cpp", "lines": [[2, 2]]}])
        done = subprocess.run([sys.executable, SCRIPT, database, source, outdir, line_filter],
                              capture_output=True, text=True)
        assert done.returncode == 0, done
        with open(os.path.join(outdir, "compile_commands.json")) as handle:
            reduced = json.load(handle)
        assert [part_of(kept) for kept in reduced] == ["3"], reduced
        assert "checking part 3" in done.stdout, done.stdout

        # A source outside the database is reported by exit status, not output.
        done = subprocess.run([sys.executable, SCRIPT, database, os.path.join(tmp, "other.cpp"),
                               outdir, line_filter], capture_output=True, text=True)
        assert done.returncode == 1, done

        # An unreadable database is fatal, so the caller can stop rather than
        # silently check nothing.
        done = subprocess.run([sys.executable, SCRIPT, source, source, outdir, line_filter],
                              capture_output=True, text=True)
        assert done.returncode == 2, done


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for test in tests:
        test()
        print("PASS %s" % test.__name__)
    print("%d tests passed" % len(tests))
    return 0


if __name__ == "__main__":
    sys.exit(main())
