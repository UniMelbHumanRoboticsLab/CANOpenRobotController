#!/usr/bin/env python3
# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

"""Prune a GitLab Runner local cache directory.

For self-hosted runners without a distributed cache backend
(`[runners.cache]`), where `Checking cache for <key>` is followed by
`No URL provided ... a local version of cache will be extracted`.  Those
archives live on the runner's disk and GitLab Runner never removes one: a
`cache:key` change and the "Clear runner caches" button both orphan the old
archive rather than reclaiming it.  Runners backed by a distributed cache need
none of this; the object store's lifecycle policy covers them.

Two passes, either usable alone:

--stale-index-below N drops superseded cache generations.  GitLab appends
`-<index>-protected` / `-<index>-non_protected` to every key, where the index
is the project's "Clear runner caches" counter, so a job can only read the
current index and anything below it is unreadable by construction.  Read the
current index off any job log's `Checking cache for <key>-<index>-<protection>`
line.

--max-gb caps the total, dropping the oldest archive by mtime until it fits.
Every job that restores a pool also rewrites it (`cache:policy` defaults to
pull-push), so mtime tracks last use and oldest-first is an LRU, not a FIFO.

--report prints what is on disk, by generation, and changes nothing.

Usage, as the user that owns the cache directory:

    prune_runner_cache.py --root /var/lib/gitlab-runner/cache --report
    prune_runner_cache.py --root /var/lib/gitlab-runner/cache \\
        --stale-index-below 11 --max-gb 120

With the docker executor the archives live inside the runner's cache volume
unless `[runners] cache_dir` names a host path that is also listed in
`[runners.docker] volumes`; point --root at that path, or at the volume's
Mountpoint from `docker volume inspect`.
"""

import argparse
import os
import re
import sys
import time

DEFAULT_PATTERNS = ("cache.zip", "cache.tgz", "cache.tar.zst")
INDEX = re.compile(r"-(\d+)-(?:protected|non_protected)$")


def entries(root, patterns):
    for dirpath, _, filenames in os.walk(root):
        for name in filenames:
            if name in patterns:
                path = os.path.join(dirpath, name)
                try:
                    st = os.stat(path)
                except OSError:
                    continue
                yield path, st.st_size, st.st_mtime


def human(n):
    for unit, scale in (("GB", 1e9), ("MB", 1e6), ("kB", 1e3)):
        if n >= scale:
            return "%.1f %s" % (n / scale, unit)
    return "%d B" % n


def cache_index(path):
    """GitLab's cache generation, from the key directory holding the archive."""
    match = INDEX.search(os.path.basename(os.path.dirname(path)))
    return int(match.group(1)) if match else None


def remove(path, size, mtime, args, why):
    rel = os.path.relpath(path, args.root)
    age_days = (time.time() - mtime) / 86400
    print("%s %s (%s, %.1f d, %s)" % ("would remove" if args.dry_run else "removing",
                                      rel, human(size), age_days, why))
    if args.dry_run:
        return
    try:
        os.remove(path)
    except OSError as exc:
        print("  failed: %s" % exc, file=sys.stderr)
        return
    parent = os.path.dirname(path)
    while os.path.abspath(parent) != os.path.abspath(args.root):
        try:
            os.rmdir(parent)
        except OSError:
            break
        parent = os.path.dirname(parent)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--root", required=True, help="cache directory to prune")
    p.add_argument("--max-gb", type=float, help="keep total archive bytes below this")
    p.add_argument("--stale-index-below", type=int, metavar="N",
                   help="delete archives whose key carries a cache index below N; "
                        "N is the index current jobs use")
    p.add_argument("--report", action="store_true",
                   help="print a per-generation breakdown and exit")
    p.add_argument("--min-age-minutes", type=float, default=30.0,
                   help="never delete an archive written this recently; avoids racing "
                        "a job that is still uploading (default: 30)")
    p.add_argument("--pattern", action="append",
                   help="archive file name to consider (repeatable; default: %s)"
                        % ", ".join(DEFAULT_PATTERNS))
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()

    patterns = tuple(args.pattern) if args.pattern else DEFAULT_PATTERNS
    if not os.path.isdir(args.root):
        sys.exit("not a directory: %s" % args.root)
    if not args.report and args.max_gb is None and args.stale_index_below is None:
        sys.exit("nothing to do: pass --report, --max-gb or --stale-index-below")

    found = sorted(entries(args.root, patterns), key=lambda e: e[2])
    total = sum(size for _, size, _ in found)
    print("%d archives, %s" % (len(found), human(total)))

    if args.report:
        by_gen = {}
        for path, size, mtime in found:
            gen = cache_index(path)
            count, nbytes, newest = by_gen.get(gen, (0, 0, 0))
            by_gen[gen] = (count + 1, nbytes + size, max(newest, mtime))
        for gen in sorted(by_gen, key=lambda g: (g is None, g)):
            count, nbytes, newest = by_gen[gen]
            print("  index %-5s %4d archives  %9s  newest %s"
                  % ("none" if gen is None else gen, count, human(nbytes),
                     time.strftime("%Y-%m-%d", time.localtime(newest))))
        return

    cutoff = time.time() - args.min_age_minutes * 60
    freed = 0

    if args.stale_index_below is not None:
        for entry in list(found):
            path, size, mtime = entry
            gen = cache_index(path)
            if gen is None or gen >= args.stale_index_below or mtime > cutoff:
                continue
            remove(path, size, mtime, args, "superseded generation %d" % gen)
            freed += size
            found.remove(entry)
        print("%s %s from superseded generations"
              % ("would free" if args.dry_run else "freed", human(freed)))

    if args.max_gb is None:
        return

    cap = int(args.max_gb * 1e9)
    print("cap %s, %s in use" % (human(cap), human(total - freed)))
    for path, size, mtime in found:
        if total - freed <= cap:
            break
        if mtime > cutoff:
            continue
        remove(path, size, mtime, args, "least recently used")
        freed += size

    print("%s %s in total; %s remains"
          % ("would free" if args.dry_run else "freed", human(freed), human(total - freed)))
    if total - freed > cap:
        print("still over cap: every remaining archive is newer than "
              "--min-age-minutes", file=sys.stderr)


if __name__ == "__main__":
    main()
