#!/usr/bin/env python3
# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

"""Unit tests for scripts/clang_tidy_hook.py and scripts/style_common.py.

The line-filter construction, umbrella resolution and target selection are
tested against the real tree without invoking clang-tidy.  The end-to-end
check does invoke it, and is skipped when clang-tidy is not installed.

Usage: python3 scripts/test_clang_tidy_hook.py
"""

import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import clang_tidy_hook
from clang_tidy_hook import (compile_args, cuda_include_dir, module_of, run_clang_tidy,
                             tidy_target, umbrella_for)
from style_common import REPO_ROOT, as_ranges, line_filter_json


def test_as_ranges():
    assert as_ranges(set()) == []
    assert as_ranges({5}) == [[5, 5]]
    # Consecutive numbers collapse; gaps start a new range.
    assert as_ranges({1, 2, 3}) == [[1, 3]]
    assert as_ranges({3, 1, 2, 7, 8, 20}) == [[1, 3], [7, 8], [20, 20]]


def test_line_filter_json():
    payload = json.loads(line_filter_json({"Eigen/src/Core/Block.h": {10, 11, 40}}))
    assert payload == [{"name": "Eigen/src/Core/Block.h", "lines": [[10, 11], [40, 40]]}], payload
    # Repository-relative paths, not basenames: `Block.h` alone would match any
    # module's Block.h, since clang-tidy compares the name as a path suffix.
    assert "/" in payload[0]["name"]
    # A file with no added lines must not appear: an empty entry would be a
    # filter that silences the file rather than one that reports nothing.
    assert json.loads(line_filter_json({"a.h": set(), "b.h": {1}})) == [{"name": "b.h", "lines": [[1, 1]]}]


def test_module_of():
    assert module_of("Eigen/src/Core/Block.h") == "Core"
    assert module_of("Eigen/src/Core/arch/AVX/PacketMath.h") == "Core"
    assert module_of("contrib/Eigen/src/Tensor/TensorBlock.h") == "Tensor"
    assert module_of("test/block.cpp") is None
    assert module_of("Eigen/Core") is None


def test_umbrella_resolution():
    # Read from the module's InternalHeaderCheck.h `#error` directive.
    assert umbrella_for("Eigen/src/Core/Block.h") == "Eigen/Core"
    # The Tensor module's umbrella lives outside its own directory tree.
    assert umbrella_for("contrib/Eigen/src/Tensor/TensorBlock.h") == "contrib/Eigen/Tensor"
    # Arch backends nested below the module root carry no directive of their
    # own and fall back to <root>/<Module>.
    assert umbrella_for("Eigen/src/Core/arch/AVX/PacketMath.h") == "Eigen/Core"
    # Modules needing a third-party header are skipped rather than guessed at.
    assert umbrella_for("Eigen/src/CholmodSupport/CholmodSupport.h") is None
    # Every module in the tree must resolve, or the hook silently skips files.
    unresolved = []
    for tree in ("Eigen/src", "contrib/Eigen/src"):
        for module in sorted(os.listdir(os.path.join(REPO_ROOT, tree))):
            rel = "%s/%s/InternalHeaderCheck.h" % (tree, module)
            if not os.path.isfile(os.path.join(REPO_ROOT, rel)):
                continue
            if umbrella_for(rel) is None and module not in (
                    "AccelerateSupport", "CholmodSupport", "KLUSupport", "MetisSupport",
                    "PaStiXSupport", "PardisoSupport", "SPQRSupport", "SuperLUSupport",
                    "UmfPackSupport"):
                unresolved.append(rel)
    assert not unresolved, unresolved


def test_tidy_target():
    with tempfile.TemporaryDirectory(prefix="tidy_target_test_") as tmp:
        # A src-tree header is linted after its umbrella.  The explicit second
        # include is needed for a new header the umbrella does not export yet.
        driver = tidy_target("Eigen/src/Core/Block.h", tmp)
        assert driver is not None and driver.startswith(tmp)
        assert open(driver).read() == ("#include <Eigen/Core>\n"
                                       "#include <Eigen/src/Core/Block.h>\n")
        # An extensionless public module header is included directly.
        driver = tidy_target("Eigen/Core", tmp)
        assert open(driver).read() == "#include <Eigen/Core>\n"
        # A test source is linted in place, not through a driver.
        assert tidy_target("test/block.cpp", tmp) == os.path.join(REPO_ROOT, "test/block.cpp")
        # A header outside the src trees has no reliable standalone TU.
        assert tidy_target("test/main.h", tmp) is None
        # An unresolvable module is skipped.
        assert tidy_target("Eigen/src/CholmodSupport/CholmodSupport.h", tmp) is None


def test_compile_args():
    args = compile_args("Eigen/src/Core/Block.h")
    assert "-std=c++14" in args and "-I" + REPO_ROOT in args
    # Test sources include main.h from test/.
    assert any(a.endswith("/test") for a in compile_args("test/block.cpp"))
    assert any(a.endswith("/test") for a in compile_args("contrib/test/tensor_block_access.cpp"))
    assert not any(a.endswith("/test") for a in compile_args("Eigen/src/Core/Block.h"))
    # Contrib test sources include their umbrellas as <Eigen/Tensor>, which
    # only resolves with contrib/ on the path; nothing else needs it.
    assert any(a.endswith("/contrib") for a in compile_args("contrib/test/tensor_block_access.cpp"))
    assert not any(a.endswith("/contrib") for a in compile_args("test/block.cpp"))
    assert not any(a.endswith("/contrib") for a in compile_args("contrib/Eigen/src/Tensor/TensorBlock.h"))


def test_cuda_include_dir():
    saved = {name: os.environ.pop(name, None)
             for name in ("CUDAToolkit_ROOT", "CUDA_HOME", "CUDA_PATH")}
    try:
        with tempfile.TemporaryDirectory() as tmpdir:
            include = os.path.join(tmpdir, "include")
            os.makedirs(include)
            absent = os.path.join(tmpdir, "absent")
            os.environ["CUDA_HOME"] = tmpdir
            assert cuda_include_dir(absent) is None, "an empty toolkit root must not be accepted"
            with open(os.path.join(include, "cuda_runtime.h"), "w", encoding="utf-8") as handle:
                handle.write("\n")
            assert cuda_include_dir(absent) == include
            assert compile_args("contrib/Eigen/src/GPU/DeviceMatrix.h")[-2:] == ["-isystem", include]
    finally:
        for name, value in saved.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value


def test_end_to_end_line_filter():
    """clang-tidy must report an added NULL and stay silent on legacy ones."""
    if shutil.which("clang-tidy") is None:
        print("SKIP test_end_to_end_line_filter (clang-tidy not installed)")
        return
    with tempfile.TemporaryDirectory(prefix="tidy_e2e_test_") as tmp:
        header = os.path.join(tmp, "probe.h")
        with open(header, "w") as handle:
            handle.write("struct Probe {\n"
                         "  typedef int Legacy;\n"      # line 2: pre-existing
                         "  typedef int Added;\n"       # line 3: "added"
                         "};\n")
        source = os.path.join(tmp, "probe.cpp")
        with open(source, "w") as handle:
            handle.write('#include "probe.h"\n')
        cmd = ["clang-tidy", "--quiet", "--checks=-*,modernize-use-using",
               "--header-filter=probe\\.h",
               "--line-filter=" + json.dumps([{"name": "probe.h", "lines": [[3, 3]]}]),
               source, "--", "-std=c++14", "-I" + tmp]
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=60).stdout
        hits = [line for line in out.splitlines() if "modernize-use-using" in line]
        assert len(hits) == 1, "expected exactly the added line, got %r" % (hits,)
        assert "probe.h:3:" in hits[0], hits


def test_broken_translation_unit_is_not_reported_clean():
    """A file that does not compile must be named, not silently passed."""
    if shutil.which("clang-tidy") is None:
        print("SKIP test_broken_translation_unit_is_not_reported_clean (clang-tidy not installed)")
        return
    with tempfile.TemporaryDirectory(prefix="tidy_broken_test_") as tmp:
        os.makedirs(os.path.join(tmp, "test"))
        # run_clang_tidy reads <root>/.clang-tidy, so the fake root needs one.
        shutil.copyfile(os.path.join(REPO_ROOT, ".clang-tidy"), os.path.join(tmp, ".clang-tidy"))
        source = os.path.join(tmp, "test", "broken.cpp")
        with open(source, "w") as handle:
            handle.write("int valid = 0;\nthis is not valid C++ at all @@@;\n")
        diagnostics, skipped = run_clang_tidy({"test/broken.cpp": {2}}, root=tmp)
        assert diagnostics == [], diagnostics
        reasons = [reason for path, reason in skipped if path == "test/broken.cpp"]
        assert reasons == ["translation unit did not compile"], skipped


def test_hook_mode_reports_broken_unit_without_blocking():
    """An uncompiled unit produces a non-blocking user notice, while a real
    finding on an added line still blocks."""
    payload = {"tool_name": "Write",
               "tool_input": {"file_path": os.path.join(REPO_ROOT, "test", "probe.cpp"),
                              "content": "int probe = 0;\n"}}

    def run(diagnostics, skipped):
        saved = (sys.stdin, sys.stdout, sys.stderr, clang_tidy_hook.hook_post_image_and_added,
                 clang_tidy_hook.run_clang_tidy)
        sys.stdin, sys.stdout, sys.stderr = (io.StringIO(json.dumps(payload)), io.StringIO(), io.StringIO())
        clang_tidy_hook.hook_post_image_and_added = lambda *args: (["int probe = 0;"], {1}, True)
        clang_tidy_hook.run_clang_tidy = lambda per_file, **kwargs: (diagnostics, skipped)
        try:
            return clang_tidy_hook.run_hook_mode(), sys.stdout.getvalue(), sys.stderr.getvalue()
        finally:
            (sys.stdin, sys.stdout, sys.stderr, clang_tidy_hook.hook_post_image_and_added,
             clang_tidy_hook.run_clang_tidy) = saved

    code, out, err = run([], [("test/probe.cpp", "translation unit did not compile")])
    notice = json.loads(out)
    assert code == 0 and "did not compile" in notice["systemMessage"] and not err, (code, out, err)
    code, out, err = run(["test/probe.cpp:1:1: warning: use 'using' [modernize-use-using]"], [])
    assert code == 2 and not out and "modernize-use-using" in err, (code, out, err)


def test_new_src_header_is_checked():
    """A private header absent from its umbrella must still be parsed."""
    if shutil.which("clang-tidy") is None:
        print("SKIP test_new_src_header_is_checked (clang-tidy not installed)")
        return
    with tempfile.TemporaryDirectory(prefix="tidy_new_header_test_") as tmp:
        src = os.path.join(tmp, "Eigen", "src", "Core")
        os.makedirs(src)
        with open(os.path.join(tmp, ".clang-tidy"), "w") as handle:
            handle.write("Checks: '-*,modernize-use-using'\n")
        with open(os.path.join(tmp, "Eigen", "Core"), "w") as handle:
            handle.write("#define EIGEN_CORE_MODULE_H\n")
        with open(os.path.join(src, "InternalHeaderCheck.h"), "w") as handle:
            handle.write("#ifndef EIGEN_CORE_MODULE_H\n"
                         "#error \"Please include Eigen/Core instead of this file directly.\"\n"
                         "#endif\n")
        with open(os.path.join(src, "Added.h"), "w") as handle:
            handle.write('#include "InternalHeaderCheck.h"\n'
                         "typedef int AddedAlias;\n")

        diagnostics, skipped = run_clang_tidy({"Eigen/src/Core/Added.h": {2}}, root=tmp)
        assert not skipped, skipped
        assert len(diagnostics) == 1 and "modernize-use-using" in diagnostics[0], diagnostics


def test_contrib_test_source_is_checked():
    """A contrib test source includes its module umbrella as <Eigen/X>,
    which resolves only through contrib/; without that path every such
    file was skipped as a translation unit that did not compile."""
    if shutil.which("clang-tidy") is None:
        print("SKIP test_contrib_test_source_is_checked (clang-tidy not installed)")
        return
    with tempfile.TemporaryDirectory(prefix="tidy_contrib_test_") as tmp:
        os.makedirs(os.path.join(tmp, "contrib", "Eigen"))
        os.makedirs(os.path.join(tmp, "contrib", "test"))
        with open(os.path.join(tmp, ".clang-tidy"), "w") as handle:
            handle.write("Checks: '-*,modernize-use-using'\n")
        with open(os.path.join(tmp, "contrib", "Eigen", "Probe"), "w") as handle:
            handle.write("#define EIGEN_PROBE_MODULE_H\n")
        with open(os.path.join(tmp, "contrib", "test", "probe.cpp"), "w") as handle:
            handle.write("#include <Eigen/Probe>\n"
                         "typedef int AddedAlias;\n")

        diagnostics, skipped = run_clang_tidy({"contrib/test/probe.cpp": {2}}, root=tmp)
        assert not skipped, skipped
        assert len(diagnostics) == 1 and "modernize-use-using" in diagnostics[0], diagnostics


def test_binary_flag():
    """--binary must be honored: a usage error in diff mode, fail-open in hook mode."""
    script = os.path.join(REPO_ROOT, "scripts", "clang_tidy_hook.py")
    done = subprocess.run([sys.executable, script, "--diff", "HEAD", "--binary", "no-such-clang-tidy"],
                          capture_output=True, text=True)
    assert done.returncode == 2 and "no-such-clang-tidy" in done.stderr, (done.returncode, done.stderr)
    done = subprocess.run([sys.executable, script, "--claude-hook", "--binary", "no-such-clang-tidy"],
                          input="{}", capture_output=True, text=True)
    assert done.returncode == 0, (done.returncode, done.stderr)
    # A resolvable path is used as given: /bin/false runs and fails, which
    # run_clang_tidy reports per file (test_clang_tidy_failure_is_not_reported_clean).


def test_clang_tidy_failure_is_not_reported_clean():
    with tempfile.TemporaryDirectory(prefix="tidy_failure_test_") as tmp:
        os.makedirs(os.path.join(tmp, "test"))
        with open(os.path.join(tmp, ".clang-tidy"), "w") as handle:
            handle.write("Checks: '-*'\n")
        with open(os.path.join(tmp, "test", "probe.cpp"), "w") as handle:
            handle.write("int probe;\n")
        diagnostics, skipped = run_clang_tidy({"test/probe.cpp": {1}}, root=tmp, tidy="/bin/false")
        assert diagnostics == [], diagnostics
        assert skipped == [("test/probe.cpp", "clang-tidy failed")], skipped


def test_failtest_is_checked_without_failure_macro():
    """Failtests are linted as their successful compile, not their _ko target."""
    if shutil.which("clang-tidy") is None:
        print("SKIP test_failtest_is_checked_without_failure_macro (clang-tidy not installed)")
        return
    with tempfile.TemporaryDirectory(prefix="tidy_failtest_test_") as tmp:
        os.makedirs(os.path.join(tmp, "failtest"))
        with open(os.path.join(tmp, ".clang-tidy"), "w") as handle:
            handle.write("Checks: '-*,modernize-use-using'\n")
        with open(os.path.join(tmp, "failtest", "probe.cpp"), "w") as handle:
            handle.write("#ifdef EIGEN_SHOULD_FAIL_TO_BUILD\n"
                         "this is intentionally invalid;\n"
                         "#endif\n"
                         "typedef int AddedAlias;\n")

        diagnostics, skipped = run_clang_tidy({"failtest/probe.cpp": {4}}, root=tmp)
        assert not skipped, skipped
        assert len(diagnostics) == 1 and "modernize-use-using" in diagnostics[0], diagnostics


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for test in tests:
        test()
        print("PASS %s" % test.__name__)
    print("%d tests passed" % len(tests))
    return 0


if __name__ == "__main__":
    sys.exit(main())
