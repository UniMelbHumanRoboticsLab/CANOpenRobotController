#!/usr/bin/env python3
# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0
#
# Content-addressed pass cache for CI test jobs (driven by
# test.linux.script.sh, run from inside the build directory).
#
# `plan` reads a `ctest --show-only=json-v1` dump (stdin via
# `--tests-json -`) and keys every enumerated test by an environment
# fingerprint plus the content of its definition: command elements that are
# files (the test binary, and the emulator for cross-compiled jobs) are
# replaced by a digest of their bytes, remaining elements and the CTest
# properties are taken literally with build-directory paths normalized away.
# Tests whose key the manifest already records as a first-attempt pass
# become the anchored exclusion regex in --skip-regex-out; every keyable
# test goes to --keys-out for the later `record` step.
#
# The environment fingerprint covers everything that can change a test's
# outcome while its binary stays identical: the image and its C library, the
# CONFIG_PATHS subtrees of the checked-in CI configuration, the dpkg version
# state of every lib* package (a superset of anything a test binary can
# dynamically load, cross sysroots included), and KEYED_ENV_PREFIXES variables
# read directly for values set outside the tree.  Job-to-job differences are
# already isolated by the per-job-name GitLab cache key; the fingerprint guards
# against one job's environment drifting over time.
# A tool that is genuinely absent (e.g. no dpkg on the hand-assembled
# riscv64 image) contributes a fixed sentinel so images with and without it
# never share keys; unexpected errors propagate, `plan` fails, and the
# driving script then runs the full selection.
#
# `record` folds the run's first-attempt results back into the manifest,
# using the dashboard run's Test.xml as the authoritative status source:
# only tests with Status="passed" are recorded ("failed" and "notrun" cover
# failures, timeouts, missing executables, and SKIP_RETURN_CODE skips).
# New passes are appended, entries that proved useful again (skipped or
# re-passed) move to the end, and the file is trimmed to --max-entries, so
# it ages out roughly least-recently-useful first.
#
# A test is keyable only if its command ends in an executable inside the
# build directory.  Anything else (e.g. a test that invokes the compiler on
# tree sources) has an outcome that depends on more than the artifact's
# content and must always run.

import argparse
import functools
import hashlib
import json
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ElementTree

# Paths under the config root whose content can change a test's outcome while
# its binary stays identical.  Deliberately not the whole ci/ tree: the
# *.gitlab-ci.yml files are orchestration, and everything in them that reaches
# a test outcome reaches this key by value already -- job variables through
# KEYED_ENV_PREFIXES, the image through CI_JOB_IMAGE, compiler flags and the
# cross emulator through the digests of the files in the test's command, ctest
# timeouts through the properties hash.  Hashing the file that sets them only
# means every CI-maintenance merge request throws away every job's manifest.
#
# ci/docker/ stays: those images are referenced by a moving :latest tag, so
# CI_JOB_IMAGE cannot see a rebuild, and the dpkg-query below covers only lib*
# packages, not a non-lib* one such as qemu-user.
#
# This is a whole-directory walk rather than a list of the scripts on the test
# path, because a list is complete only until someone adds a script and forgets
# to name it here -- and that failure mode is a silently reused stale pass.
#
# The exchange for the narrowing: a job's tags: are now invisible to the key.
# Moving a job to a runner pool whose CPU differs (an AVX-512 job off the
# avx512 pool, say) will not invalidate its manifest, so pair such a move with
# a cache clear.  Nothing today distinguishes two hosts within one tag pool
# either, so this widens an accepted hole rather than opening a new one.
CONFIG_PATHS = ("ci/scripts", "ci/docker")

# Environment variables (matched by prefix) that can change a test's outcome
# without changing its binary.  Since the CI YAML is no longer hashed, this
# tuple is load-bearing: a new job variable that can change a test's outcome
# must be added here, because setting it in the YAML alone no longer keys it.
KEYED_ENV_PREFIXES = (
    "EIGEN_REPEAT",
    "EIGEN_SEED",
    "EIGEN_CI_CTEST_ARGS",
    "QEMU_",
    "ASAN_",
    "UBSAN_",
    "LSAN_",
    "TSAN_",
    "MSAN_",
    "LD_LIBRARY_PATH",
    "LD_PRELOAD",
    # Host-derived values that legitimately vary across runners of one job
    # (EIGEN_CI_CTEST_PARALLEL, NPROC) are deliberately absent.
)


@functools.lru_cache(maxsize=None)
def file_digest(path):
    with open(path, "rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def fingerprint(config_root):
    parts = ["image:" + os.environ.get("CI_JOB_IMAGE", "")]
    try:
        parts.append("libc:" + (os.confstr("CS_GNU_LIBC_VERSION") or ""))
    except (ValueError, OSError):
        parts.append("libc:unavailable")
    config = hashlib.sha256()
    for top in CONFIG_PATHS:
        top_path = os.path.join(config_root, top)
        if os.path.isfile(top_path):
            config.update(top.encode() + b"\0" + file_digest(top_path).encode() + b"\0")
            continue
        for dirpath, dirnames, filenames in os.walk(top_path):
            dirnames.sort()
            for filename in sorted(filenames):
                path = os.path.join(dirpath, filename)
                rel = os.path.relpath(path, config_root)
                config.update(rel.encode() + b"\0" + file_digest(path).encode() + b"\0")
    parts.append("config:" + config.hexdigest())
    try:
        packages = subprocess.run(["dpkg-query", "-W", "lib*"], check=True, capture_output=True, text=True).stdout
        parts.append("libs:" + hashlib.sha256(packages.encode()).hexdigest())
    except (FileNotFoundError, subprocess.CalledProcessError):
        parts.append("libs:unavailable")
    env = ("%s=%s" % (k, v) for k, v in os.environ.items() if k.startswith(KEYED_ENV_PREFIXES))
    parts.append("env:" + ",".join(sorted(env)))
    return "|".join(parts)


def normalize(text, builddir):
    """Strips the build-directory prefix so the key does not depend on where
    the checkout happens to live."""
    return text.replace(builddir, ".")


def test_key(test, fingerprint_bytes, builddir):
    """Digest of the fingerprint and the test's definition, or None if the
    command does not end in an executable under the build directory."""
    command = test.get("command")
    if not command:
        return None
    last = os.path.realpath(command[-1])
    if not (last.startswith(builddir + os.sep) and os.path.isfile(last) and os.access(last, os.X_OK)):
        return None
    h = hashlib.sha256()
    h.update(fingerprint_bytes)
    for element in command:
        path = os.path.realpath(element)
        if os.path.isfile(path):
            h.update(b"file:" + file_digest(path).encode() + b"\0")
        else:
            h.update(b"arg:" + normalize(element, builddir).encode() + b"\0")
    properties = sorted(test.get("properties", []), key=lambda p: str(p.get("name")))
    h.update(b"props:" + normalize(json.dumps(properties, sort_keys=True), builddir).encode())
    return h.hexdigest()


def read_manifest(path):
    """The manifest as an insertion-ordered {key: testname} dict."""
    entries = {}
    if os.path.exists(path):
        with open(path) as f:
            for line in f:
                fields = line.split()
                if fields:
                    entries[fields[0]] = fields[1] if len(fields) > 1 else ""
    return entries


def write_manifest(path, entries, max_entries):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.writelines("%s %s\n" % item for item in list(entries.items())[-max_entries:])
    os.replace(tmp, path)


def plan(args):
    with (sys.stdin if args.tests_json == "-" else open(args.tests_json)) as f:
        tests = json.load(f)["tests"]
    manifest = read_manifest(args.manifest)
    builddir = os.path.realpath(os.getcwd())
    fingerprint_bytes = fingerprint(args.config_root).encode() + b"\0"
    keys = {}
    skip = []
    for test in tests:
        key = test_key(test, fingerprint_bytes, builddir)
        if key is None:
            continue
        keys[test["name"]] = key
        if key in manifest:
            skip.append(test["name"])
    with open(args.keys_out, "w") as f:
        f.writelines("%s %s\n" % (key, name) for name, key in keys.items())
    with open(args.skip_regex_out, "w") as f:
        if skip:
            f.write("^(%s)$" % "|".join(re.escape(name) for name in skip))
    print(
        "test cache plan: %d tests selected, %d cached passes to skip, %d to run (%d not keyable)"
        % (len(tests), len(skip), len(tests) - len(skip), len(tests) - len(keys))
    )


def record(args):
    keys = {}
    with open(args.keys) as f:
        for line in f:
            fields = line.split()
            if len(fields) == 2:
                keys[fields[1]] = fields[0]
    # The dashboard <Test> elements carry a Status attribute ("passed",
    # "failed", or "notrun"); the bare <TestList> entries do not and are
    # ignored.  Streamed with iterparse because --no-compress-output embeds
    # every test's stdout in the file.  A missing or unparsable Test.xml
    # means ctest died without completing the test phase: record nothing
    # rather than trust it.
    passed = set()
    try:
        with open(os.path.join(args.testing_dir, "TAG")) as f:
            tag = f.readline().strip()
        for _, element in ElementTree.iterparse(os.path.join(args.testing_dir, tag, "Test.xml")):
            if element.tag != "Test":
                continue
            if element.get("Status") == "passed":
                name = element.find("Name")
                if name is not None:
                    passed.add(name.text)
            element.clear()
    except (OSError, ElementTree.ParseError) as error:
        print("test cache record: cannot read test results (%s); recording nothing" % error)
        return
    manifest = read_manifest(args.manifest)
    new = refreshed = 0
    for name, key in keys.items():
        if name in passed:
            if manifest.pop(key, None) is None:
                new += 1
            else:
                refreshed += 1
            manifest[key] = name
        elif manifest.pop(key, None) is not None:
            # Not run because this entry made plan skip it; move the entry to
            # the back so trimming ages out unused entries first.
            manifest[key] = name
            refreshed += 1
    write_manifest(args.manifest, manifest, args.max_entries)
    print(
        "test cache record: %d new first-attempt passes, %d entries refreshed, %d entries total"
        % (new, refreshed, len(manifest))
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("plan")
    p.add_argument("--tests-json", required=True, help="json-v1 dump, or - for stdin")
    p.add_argument("--manifest", required=True)
    p.add_argument("--config-root", required=True)
    p.add_argument("--skip-regex-out", required=True)
    p.add_argument("--keys-out", required=True)
    p.set_defaults(func=plan)

    p = sub.add_parser("record")
    p.add_argument("--manifest", required=True)
    p.add_argument("--keys", required=True)
    p.add_argument("--testing-dir", required=True)
    p.add_argument("--max-entries", type=int, default=20000)
    p.set_defaults(func=record)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
