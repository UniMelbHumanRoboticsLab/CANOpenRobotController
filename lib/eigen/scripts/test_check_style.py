#!/usr/bin/env python3
# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

"""Unit tests for scripts/check_style.py.

Runs the checks on synthetic post-images, the diff parser on a crafted diff,
and the diff mode against a temporary git repository, so the expectations do
not depend on the checked-out tree.

Usage: python3 scripts/test_check_style.py
"""

import os
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from check_style import (
    added_from_structured_patch,
    added_lines_from_diff,
    find_findings,
    hook_added_line_numbers,
    run_diff_mode,
)


def messages(rel_path, text, added=None):
    lines = text.splitlines()
    if added is None:
        added = set(range(1, len(lines) + 1))
    return [m for _, m in find_findings(rel_path, lines, added)]


def assert_flags(rel_path, text, *fragments, **kwargs):
    got = messages(rel_path, text, kwargs.get("added"))
    for fragment in fragments:
        assert any(fragment in m for m in got), "expected %r in findings for %s, got %r" % (fragment, rel_path, got)


def assert_clean(rel_path, text, added=None):
    got = messages(rel_path, text, added)
    assert not got, "expected no findings for %s, got %r" % (rel_path, got)


def test_conventions_flagged():
    assert_flags("Eigen/src/Core/Foo.h", "std::integral_constant<bool, true> b;\n", "bool_constant")
    assert_flags("Eigen/src/Core/Foo.h", "enum { Flags = 0 };\n", "static constexpr")
    assert_flags("Eigen/src/Core/Foo.h", "enum : unsigned int { Flags = 0 };\n", "static constexpr")
    assert_flags("Eigen/src/Core/Foo.h", "enum : std::uint32_t { Flags = 0 };\n", "static constexpr")
    assert_flags("Eigen/src/Core/Foo.h", "S s = {.size = 3};\n", "designated initializer")
    assert_flags("Eigen/src/Core/Foo.h", "if constexpr (kSize > 4) {}\n", "EIGEN_IF_CONSTEXPR")
    assert_flags("test/foo.cpp", "std::optional<int> x;\n", "C++17/20 library facility")
    assert_flags("test/foo.cpp", "std::any x;\n", "C++17/20 library facility")
    assert_flags("test/foo.cpp", "std::filesystem::path p;\n", "C++17/20 library facility")
    assert_flags("Eigen/src/Core/Foo.h", "double x = std::sqrt(2.0);\n", "numext::")
    assert_flags("Eigen/src/Core/Foo.h", "double x = (std::sqrt)(2.0);\n", "numext::")
    assert_flags("Eigen/src/Core/Foo.h", "double x = std::exp2(2.0);\n", "numext::")
    assert_flags("Eigen/src/Core/Foo.h", "double x = std::cbrt(2.0);\n", "numext::")
    assert_flags("Eigen/src/Core/Foo.h", "double x = std::acosh(2.0);\n", "numext::")
    assert_flags("Eigen/src/Core/Foo.h", "auto x = std::conj(value);\n", "numext::")
    assert_flags("Eigen/src/Core/Foo.h", "double x = std::frexp(value, &exponent);\n", "EIGEN_USING_STD")


def test_scoping():
    # Eigen's public module headers are C++ despite having no extension.
    assert_flags("Eigen/Core", "enum { Flags = 0 };\n", "static constexpr")
    assert_flags("unsupported/Eigen/FFT", "if constexpr (kSize > 4) {}\n", "EIGEN_IF_CONSTEXPR")
    # std:: math is only flagged in library implementation headers.
    assert_clean("test/foo.cpp", "double x = std::sqrt(2.0);\n")
    # C++14 checks do not apply outside the C++14 trees.
    assert_clean("benchmarks/Core/foo.cpp", "if constexpr (kSize > 4) {}\n")
    # Non-C++ files are ignored entirely.
    assert_clean("AGENTS.md", "enum { Flags = 0 };\nif constexpr (x) {}\n")
    # Only ADDED lines are reported: the same violations on unchanged lines stay silent.
    assert_clean("Eigen/src/Core/Foo.h", "enum { Flags = 0 };\nint x = 1;\n", added={2})
    # Sources with a documented C++17 requirement are exempt from the C++14 checks...
    assert_clean("test/sycl_basic.cpp", "if constexpr (kSize > 4) {}\n")
    assert_clean("Eigen/src/Core/arch/SYCL/PacketMath.h", "if constexpr (kSize > 4) {}\n")
    assert_clean("contrib/Eigen/src/Tensor/TensorDeviceSycl.h", "if constexpr (kSize > 4) {}\n")
    assert_clean("contrib/test/tensor_sycl.cpp", "if constexpr (kSize > 4) {}\n")
    assert_clean("contrib/Eigen/src/FFT/duccfft_impl.h", "if constexpr (kSize > 4) {}\n")
    assert_clean("contrib/test/duccfft.cpp", "if constexpr (kSize > 4) {}\n")
    assert_clean("failtest/structured_bindings_dynamic_matrix.cpp", "if constexpr (kSize > 4) {}\n")
    assert_clean("failtest/structured_bindings_dynamic_array.cpp", "if constexpr (kSize > 4) {}\n")
    assert_clean("failtest/structured_bindings_rowmajor.cpp", "if constexpr (kSize > 4) {}\n")
    # A coincidental substring is not a documented C++17 requirement.
    assert_flags("test/not_sycl_related.cpp", "if constexpr (kSize > 4) {}\n", "EIGEN_IF_CONSTEXPR")
    assert_flags("contrib/Eigen/src/FFT/kissfft_impl.h", "if constexpr (kSize > 4) {}\n", "EIGEN_IF_CONSTEXPR")
    # ...but not from the other conventions.
    assert_flags("test/sycl_basic.cpp", "std::integral_constant<bool, true> b;\n", "bool_constant")


def test_false_positive_probes():
    assert_clean("Eigen/src/Core/Foo.h", "opts.size = 4;\nfoo(a.b, c.d);\n")           # member access, not init
    assert_clean("Eigen/src/Core/Foo.h", "double v[] = {.5, 1.5};\n")                  # float literal, not init
    assert_clean("Eigen/src/Core/Foo.h", "enum class Kind { A, B };\n")                # scoped enums are fine
    assert_clean("Eigen/src/Core/Foo.h", "enum Kind : unsigned int { A, B };\n")       # named enums are fine
    assert_clean("Eigen/src/Core/Foo.h", 'auto s = "if constexpr integral_constant<bool,";\n')  # in a string
    assert_clean("Eigen/src/Core/Foo.h", "const char c = 'N';\n")                      # character literal
    assert_clean("Eigen/src/Core/Foo.h", "const wchar_t c = L'N';\n")                  # prefixed character literal
    assert_clean("Eigen/src/Core/Foo.h", "// if constexpr discussed in a comment\nint x = 1;\n")
    # A C++14 digit separator is not the start of a character literal; code
    # later on the same line must remain visible to convention checks.
    assert_flags("Eigen/src/Core/Foo.h", "auto n = 1'000; std::integral_constant<bool, true> b;\n",
                 "bool_constant")
    assert_flags("Eigen/src/Core/Foo.h", "auto n = 0xFF'00; std::integral_constant<bool, true> b;\n",
                 "bool_constant")
    assert_clean("Eigen/src/Core/Foo.h", "using MyInt = int;\nstatic constexpr unsigned int Flags = 0;\n"
                                         "const char* p = nullptr;\nEIGEN_IF_CONSTEXPR (kSize > 4) {}\n"
                                         "double x = numext::sqrt(2.0);\n")


def test_multiline_designated_initializer():
    # The designator on its own line after `{` or `,` is still a C++20 designated initializer.
    assert_flags("Eigen/src/Core/Foo.h", "S s = {\n    .size = 3,\n};\n", "designated initializer")
    assert_flags("Eigen/src/Core/Foo.h", "S s = {\n    .a = 1,\n    .b = 2,\n};\n", "designated initializer")
    # A wrapped member assignment is not: the previous code line does not end with `{` or `,`.
    assert_clean("Eigen/src/Core/Foo.h", "obj\n    .member = value;\n")
    assert_clean("Eigen/src/Core/Foo.h", "foo(bar)\n    .field = 1;\n")


def test_context_across_diff_gaps():
    # An added continuation line inside an existing block comment is a comment,
    # not code, even though the surrounding lines were not added.
    text = "/* existing block\n * Do not use if constexpr here\n */\nint x = 1;\n"
    assert_clean("Eigen/src/Core/Foo.h", text, added={2})
    # Same for an existing Doxygen block: additions inherit its exemption.
    doxy = "/** \\brief Existing docs.\n" + "\n".join(" * added line %d" % i for i in range(8)) + "\n */\nint x;\n"
    assert_clean("Eigen/src/Core/Foo.h", doxy, added=set(range(2, 10)))
    # A string on an unchanged line does not leak its content into added lines.
    text = 'const char* s = "no /* here";\nstd::integral_constant<bool, true> b;\n'
    assert_flags("Eigen/src/Core/Foo.h", text, "bool_constant", added={2})


def test_comment_verbosity():
    narration = "\n".join("// narration line %d" % i for i in range(6)) + "\nint x = 1;\n"
    assert_flags("Eigen/src/Core/Foo.h", narration, "non-Doxygen comment")
    # License headers and Doxygen blocks are exempt however long they are.
    license_header = "\n".join("// SPDX-License-Identifier: MPL-2.0" if i == 0 else "// Copyright notice %d" % i
                               for i in range(8)) + "\nint x = 1;\n"
    assert_clean("Eigen/src/Core/Foo.h", license_header)
    # A bibliography under its own header is provenance, exempt however long it runs.
    refs = "// References:\n" + "\n".join("//  [%d] Author, \"Title\", 20%02d." % (i, i) for i in range(8))
    assert_clean("Eigen/src/Core/Foo.h", refs + "\nint x = 1;\n")
    # Prose that merely mentions references is still narration.
    prose = "\n".join("// see the References: section for line %d" % i for i in range(6)) + "\nint x = 1;\n"
    assert_flags("Eigen/src/Core/Foo.h", prose, "non-Doxygen comment")
    # Five added lines stay under the threshold.
    assert_clean("Eigen/src/Core/Foo.h", "\n".join("// l%d" % i for i in range(5)) + "\nint x = 1;\n")
    # Extending an existing narration block by two lines is not reported: only
    # ADDED lines count toward the threshold.
    block = "\n".join("// old line %d" % i for i in range(10)) + "\n// new a\n// new b\nint x = 1;\n"
    assert_clean("Eigen/src/Core/Foo.h", block, added={11, 12})
    # Six added lines inside an existing non-Doxygen block are reported.
    block = "// old line\n" + "\n".join("// new %d" % i for i in range(6)) + "\nint x = 1;\n"
    assert_flags("Eigen/src/Core/Foo.h", block, "non-Doxygen comment", added=set(range(2, 8)))
    # Blank physical lines and a closing line without a leading `*` remain
    # inside their lexical block, but the blank line does not count as prose.
    block = "/* first\nsecond\nthird\n\nfourth\nfifth\nsixth */\nint x = 1;\n"
    assert_flags("Eigen/src/Core/Foo.h", block, "non-Doxygen comment")
    # A blank line inside Doxygen must not split off the remaining lines and
    # lose the exemption inherited from the opening delimiter.
    doxy = "/** docs\n\nline one\nline two\nline three\nline four\nline five\nline six */\nint x = 1;\n"
    assert_clean("Eigen/src/Core/Foo.h", doxy)
    # An adjacent ordinary heading is a separate comment run and must not
    # strip the exemption from the Doxygen block that follows it.
    doxy = "// API documentation follows\n/** docs\nline one\nline two\nline three\nline four\nline five\nline six */\n"
    assert_clean("Eigen/src/Core/Foo.h", doxy)
    # The Doxygen kind is carried even when the opener follows code and is not
    # itself a comment-only line.
    trailing_doxy = ("int value; /**< docs\n * line one\n * line two\n * line three\n"
                     " * line four\n * line five\n * line six */\n")
    assert_clean("Eigen/src/Core/Foo.h", trailing_doxy)
    # Conversely, ordinary narration immediately after Doxygen does not
    # inherit its exemption.
    narration = "/** short docs */\n" + "\n".join("// narration %d" % i for i in range(6)) + "\n"
    assert_flags("Eigen/src/Core/Foo.h", narration, "non-Doxygen comment")


def test_multiline_literals():
    # Contents of a raw string spanning lines are literal, not code.
    raw = 'const char* prog = R"cl(\nif constexpr (true) { std::optional<int> x; }\n)cl";\nint y = 1;\n'
    assert_clean("Eigen/src/Core/Foo.h", raw)
    # Code after the raw string closes is lexed again.
    raw_then_code = 'auto s = R"(\ntext\n)"; std::integral_constant<bool, true> b;\n'
    assert_flags("Eigen/src/Core/Foo.h", raw_then_code, "bool_constant")
    # A backslash-spliced ordinary string stays a literal on its continuation lines.
    spliced = 'const char* s = "first \\\nif constexpr (x) \\\nlast";\nint z = 1;\n'
    assert_clean("Eigen/src/Core/Foo.h", spliced)
    # A raw-string-looking suffix of an identifier is an ordinary string.
    assert_clean("Eigen/src/Core/Foo.h", 'auto v = myR"(not raw)";\n')
    # Single-line raw strings close on the same line.
    assert_clean("Eigen/src/Core/Foo.h", 'auto s = R"(if constexpr NULL typedef)";\n')


def test_hook_line_mapping():
    content = "int x;\nint changed;\nconst char* p = NULL;\n"
    # A snippet with a trailing newline covers only its own line, not the next.
    assert hook_added_line_numbers(content, ["int changed;\n"]) == {2}
    assert hook_added_line_numbers(content, ["int changed;"]) == {2}
    # Multi-line snippets cover their span.
    assert hook_added_line_numbers(content, ["int x;\nint changed;\n"]) == {1, 2}
    # An ambiguous snippet cannot be located; the caller must fall back.
    assert hook_added_line_numbers("true\nif constexpr (true)\n", ["true"]) is None
    # An absent snippet likewise.
    assert hook_added_line_numbers(content, ["not present"]) is None


def test_structured_patch():
    response = {"structuredPatch": [
        {"oldStart": 4, "oldLines": 2, "newStart": 5, "newLines": 3,
         "lines": [" context", "-old line", "+new one", "+new two", " context"]},
    ]}
    assert added_from_structured_patch(response) == {6, 7}
    deletion_only = {"structuredPatch": [
        {"oldStart": 1, "oldLines": 3, "newStart": 1, "newLines": 2,
         "lines": [" context", "-removed", " context"]},
    ]}
    # A valid empty result must not fall back and mark the retained context.
    deletion_added = added_from_structured_patch(deletion_only)
    assert deletion_added == set()
    assert_clean("Eigen/src/Core/Foo.h", "int keep;\nstd::integral_constant<bool, true> b;\n",
                 added=deletion_added)
    assert added_from_structured_patch({}) is None
    assert added_from_structured_patch({"structuredPatch": "bogus"}) is None


def test_diff_parser():
    diff = (
        "diff --git a/Eigen/src/Core/Foo.h b/Eigen/src/Core/Foo.h\n"
        "--- a/Eigen/src/Core/Foo.h\n"
        "+++ b/Eigen/src/Core/Foo.h\n"
        "@@ -10,0 +11,2 @@ context\n"
        "+const char* p = NULL;\n"
        "+int y = 2;\n"
        "@@ -20,1 +23,1 @@ context\n"
        "-old line\n"
        "+typedef int T;\n"
        "diff --git a/gone.cpp b/gone.cpp\n"
        "--- a/gone.cpp\n"
        "+++ /dev/null\n"
    )
    files = added_lines_from_diff(diff)
    assert set(files) == {"Eigen/src/Core/Foo.h"}, files
    assert files["Eigen/src/Core/Foo.h"] == {11, 12, 23}, files


def test_no_newline_marker():
    # Replacing an unterminated last line: the "\ No newline" markers must not
    # advance the line counter, or the addition maps past the post-image.
    diff = (
        "--- a/Eigen/src/Core/Foo.h\n"
        "+++ b/Eigen/src/Core/Foo.h\n"
        "@@ -1 +1 @@\n"
        "-int old;\n"
        "\\ No newline at end of file\n"
        "+const char* p = NULL;\n"
        "\\ No newline at end of file\n"
    )
    files = added_lines_from_diff(diff)
    assert files["Eigen/src/Core/Foo.h"] == {1}, files
    assert messages("Eigen/src/Core/Foo.h", "std::integral_constant<bool, true> b;", added={1})
    response = {"structuredPatch": [
        {"oldStart": 1, "oldLines": 1, "newStart": 1, "newLines": 1,
         "lines": ["-int old;", "\\ No newline at end of file",
                   "+const char* p = NULL;", "\\ No newline at end of file"]},
    ]}
    assert added_from_structured_patch(response) == {1}


def test_diff_mode_merge_base_and_untracked():
    with tempfile.TemporaryDirectory(prefix="check_style_test_") as tmp:
        def sh(*args):
            subprocess.run(args, cwd=tmp, check=True, capture_output=True)

        def write(rel, text):
            path = os.path.join(tmp, rel)
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w") as handle:
                handle.write(text)

        sh("git", "init", "-q", "-b", "main")
        # The base carries a one-line file without a trailing newline, so the
        # feature diff emits "\ No newline at end of file" markers.
        write("Eigen/src/Core/NoEol.h", "int old;")
        sh("git", "add", "Eigen/src/Core/NoEol.h")
        sh("git", "-c", "user.email=t@t", "-c", "user.name=t", "commit", "-q", "-m", "base")
        sh("git", "branch", "target")
        # Feature branch adds a violating file and replaces the unterminated line.
        write("Eigen/src/Core/Added.h", "std::integral_constant<bool, true> b;\n")
        write("Eigen/src/Core/NoEol.h", "enum { Flags = 0 };")
        sh("git", "add", "Eigen/src/Core/Added.h", "Eigen/src/Core/NoEol.h")
        sh("git", "-c", "user.email=t@t", "-c", "user.name=t", "commit", "-q", "-m", "feature")
        # Target advances independently with its own violating file: a
        # two-tree diff against `target` would report it in reverse.
        sh("git", "checkout", "-q", "target")
        write("Eigen/src/Core/TargetOnly.h", "std::integral_constant<bool, false> legacy;\n")
        sh("git", "add", "Eigen/src/Core/TargetOnly.h")
        sh("git", "-c", "user.email=t@t", "-c", "user.name=t", "commit", "-q", "-m", "target-only")
        sh("git", "checkout", "-q", "main")
        # An untracked new file must be scanned even though git diff omits it.
        write("Eigen/src/Core/Untracked.h", "std::integral_constant<bool, true> b;\n")
        write("Eigen/NewModule", "enum { Flags = 0 };\n")

        results = run_diff_mode("target", root=tmp)
        paths = {rel_path for rel_path, _, _ in results}
        assert "Eigen/src/Core/Added.h" in paths, results
        assert "Eigen/src/Core/Untracked.h" in paths, results
        assert "Eigen/NewModule" in paths, results
        assert "Eigen/src/Core/TargetOnly.h" not in paths, results
        # The unterminated replacement maps to line 1 despite the markers.
        noeol = [(l, m) for p, l, m in results if p == "Eigen/src/Core/NoEol.h"]
        assert noeol and noeol[0][0] == 1 and "static constexpr" in noeol[0][1], results


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for test in tests:
        test()
        print("PASS %s" % test.__name__)
    print("%d tests passed" % len(tests))
    return 0


if __name__ == "__main__":
    sys.exit(main())
