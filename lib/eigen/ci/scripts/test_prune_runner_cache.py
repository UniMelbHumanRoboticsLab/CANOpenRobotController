#!/usr/bin/env python3
# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

"""Unit tests for ci/scripts/prune_runner_cache.py.

Runs against synthetic cache trees, so the expectations do not depend on any
runner's disk.  The load-bearing checks are test_generations, which pins that
only indices strictly below the current one are dropped, and test_min_age,
which pins that an archive still being written is never removed -- both are
what keep a stray --stale-index-below or an aggressive cap from deleting a
pool a running job is about to restore.

Usage: python3 ci/scripts/test_prune_runner_cache.py
"""

import os
import shutil
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import prune_runner_cache
from prune_runner_cache import cache_index, human

FAILURES = []


def check(condition, message):
    if condition:
        return
    FAILURES.append(message)
    print("FAIL: %s" % message)


class Args(object):
    """The attributes remove() reads, without going through argparse."""

    def __init__(self, root, dry_run=False):
        self.root = root
        self.dry_run = dry_run


def write_archive(root, key, megabytes, age_days, project="libeigen/eigen"):
    path = os.path.join(root, project, key, "cache.zip")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as handle:
        handle.write(b"\0" * int(megabytes * 1000 * 1000))
    when = time.time() - age_days * 86400
    os.utime(path, (when, when))
    return path


def keys_on_disk(root):
    found = set()
    for dirpath, _, filenames in os.walk(root):
        for name in filenames:
            found.add(os.path.basename(dirpath))
    return found


def test_cache_index():
    cases = {
        "build-linux-docs-ccache-11-protected": 11,
        "build-linux-docs-ccache-3-non_protected": 3,
        "build-windows-x86-64-msvc-14-29-default-BUILD-10-protected": 10,
        "pool-mr2995-ccache-11-protected": 11,
        "windows-ccache-bin": None,
        "no-index-here": None,
    }
    for key, want in cases.items():
        got = cache_index(os.path.join("/cache", "libeigen/eigen", key, "cache.zip"))
        check(got == want, "cache_index(%s) = %r, expected %r" % (key, got, want))


def test_generations(root):
    write_archive(root, "pool-ccache-9-protected", 3, 300)
    write_archive(root, "pool-ccache-10-protected", 3, 200)
    write_archive(root, "pool-ccache-11-protected", 3, 30)
    write_archive(root, "pool-mr1-ccache-11-non_protected", 3, 1)
    write_archive(root, "unindexed-key", 3, 400)

    args = Args(root, dry_run=True)
    entries = list(prune_runner_cache.entries(root, ("cache.zip",)))
    for path, size, mtime in entries:
        index = cache_index(path)
        if index is not None and index < 11:
            prune_runner_cache.remove(path, size, mtime, args, "test")
    check(len(keys_on_disk(root)) == 5, "--dry-run removed something")

    args.dry_run = False
    for path, size, mtime in entries:
        index = cache_index(path)
        if index is not None and index < 11:
            prune_runner_cache.remove(path, size, mtime, args, "test")
    left = keys_on_disk(root)
    check("pool-ccache-9-protected" not in left, "generation 9 survived")
    check("pool-ccache-10-protected" not in left, "generation 10 survived")
    check("pool-ccache-11-protected" in left, "current generation was dropped")
    check("pool-mr1-ccache-11-non_protected" in left,
          "current generation was dropped for the other protection suffix")
    check("unindexed-key" in left, "a key with no index was dropped")


def test_lru_order(root):
    write_archive(root, "pool-a-ccache-11-protected", 4, 10)
    write_archive(root, "pool-b-ccache-11-protected", 4, 5)
    write_archive(root, "pool-c-ccache-11-protected", 4, 1)

    args = Args(root)
    entries = sorted(prune_runner_cache.entries(root, ("cache.zip",)), key=lambda e: e[2])
    total = sum(size for _, size, _ in entries)
    cap = 9 * 1000 * 1000
    freed = 0
    for path, size, mtime in entries:
        if total - freed <= cap:
            break
        prune_runner_cache.remove(path, size, mtime, args, "test")
        freed += size
    left = keys_on_disk(root)
    check("pool-a-ccache-11-protected" not in left, "oldest archive survived the cap")
    check("pool-c-ccache-11-protected" in left, "newest archive was evicted first")
    check(total - freed <= cap, "cap not reached: %d left" % (total - freed))


def run_main(argv):
    """Call main() as the CLI does; returns its exit status."""
    saved = sys.argv
    sys.argv = ["prune_runner_cache.py"] + argv
    try:
        return prune_runner_cache.main()
    except SystemExit as exit_status:
        return exit_status.code
    finally:
        sys.argv = saved


def test_min_age(root):
    write_archive(root, "pool-old-ccache-11-protected", 2, 40)
    write_archive(root, "pool-fresh-ccache-11-protected", 2, 0)
    run_main(["--root", root, "--max-gb", "0"])
    left = keys_on_disk(root)
    check("pool-old-ccache-11-protected" not in left,
          "an aged archive survived a zero cap")
    check("pool-fresh-ccache-11-protected" in left,
          "an archive newer than --min-age-minutes was deleted; a job may still "
          "be uploading it")


def test_report_changes_nothing(root):
    write_archive(root, "pool-ccache-9-protected", 2, 300)
    status = run_main(["--root", root, "--report", "--stale-index-below", "11"])
    check(status in (0, None), "--report exited %r" % status)
    check("pool-ccache-9-protected" in keys_on_disk(root), "--report deleted an archive")


def test_requires_an_action(root):
    write_archive(root, "pool-ccache-11-protected", 1, 1)
    check(run_main(["--root", root]) != 0, "a no-op invocation reported success")
    check(run_main(["--root", os.path.join(root, "absent")]) != 0,
          "a missing --root reported success")


def test_empty_directories_removed(root):
    path = write_archive(root, "pool-gone-ccache-9-protected", 1, 100)
    prune_runner_cache.remove(path, 1, time.time() - 100 * 86400, Args(root), "test")
    check(not os.path.exists(os.path.dirname(path)), "empty key directory left behind")
    check(os.path.isdir(root), "the cache root itself was removed")


def test_human():
    cases = [(0, "0 B"), (512, "512 B"), (2500, "2.5 kB"),
             (3_000_000, "3.0 MB"), (4_000_000_000, "4.0 GB")]
    for value, want in cases:
        check(human(value) == want, "human(%d) = %s, expected %s" % (value, human(value), want))


def main():
    test_cache_index()
    test_human()
    for case in (test_generations, test_lru_order, test_min_age,
                 test_report_changes_nothing, test_requires_an_action,
                 test_empty_directories_removed):
        root = tempfile.mkdtemp(prefix="eigen-prune-cache-")
        try:
            case(root)
        finally:
            shutil.rmtree(root, ignore_errors=True)

    if FAILURES:
        print("\n%d check(s) failed" % len(FAILURES))
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
