# Releasing Eigen

Checklist for cutting an Eigen release (major / minor / patch) from the
upstream repository at <https://gitlab.com/libeigen/eigen>.

The mechanical steps below are the *what*. The decisions around them
(timing, scope, what counts as a breaking change) are a maintainer call
and intentionally not encoded here.

## Versioning

Eigen follows [Semantic Versioning 2.0.0](https://semver.org/) as of
5.0 — see the transition table in [`CHANGELOG.md`](CHANGELOG.md) under
the 5.0.0 entry. Versions are `MAJOR.MINOR.PATCH`:

- Bump `MAJOR` for backward-incompatible API or ABI changes.
- Bump `MINOR` for backward-compatible feature additions.
- Bump `PATCH` for backward-compatible bug fixes only.

The legacy `WORLD` field is frozen at `3` for posterity and plays no
role in release decisions; `Eigen/Version` keeps the `#define
EIGEN_WORLD_VERSION 3` line, but only `MAJOR`/`MINOR`/`PATCH` move.

## Scope

Two flows are described:

- **Major / minor release** (`X.Y.0`) — cuts a new `X.Y` release branch
  from `master`, ships the accumulated `[Unreleased]` work. Tag `5.0.0`
  is the most recent example.
- **Patch release** (`X.Y.Z`, `Z ≥ 1`) — adds cherry-picked fixes to an
  existing `X.Y` release branch. Tag `5.0.1` is the most recent example.

Choosing between them is a judgment call (SemVer is the rule, but
"large enough" varies). See work item
[#3051](https://gitlab.com/libeigen/eigen/-/work_items/3051) for a
recent case where a proposed 5.0.2 patch was reconsidered as a minor
release once maintainers reviewed scope. Resolve this on the mailing
list / Discord before any branch or version work.

## Prerequisites

- Push rights to `upstream` (`https://gitlab.com/libeigen/eigen.git`).
- A GitLab personal access token with `api` scope, exported as
  `GITLAB_PRIVATE_TOKEN`. All `scripts/gitlab_api_*.py` helpers fall
  back to this environment variable.
- A clean working tree on `master` (for a major / minor cut) or on the
  release branch (for a patch).

## Version source of truth

Everything reads from [`Eigen/Version`](Eigen/Version). `Macros.h` does
not hard-code version numbers; `CMakeLists.txt` parses the
`#define` lines from `Eigen/Version` at configure time. Editing
`Eigen/Version` is the single version bump.

Field conventions observed in history:

| State                                  | `PATCH` | `PRERELEASE` | `BUILD`   | `VERSION_STRING`        |
| -------------------------------------- | ------- | ------------ | --------- | ----------------------- |
| At the release tag                     | `Z`     | `""`         | `""`      | `"X.Y.Z"`               |
| Release branch after the tag (dev)     | `Z+1`   | `"dev"`      | `"X.Y"`   | `"X.Y.(Z+1)-dev+X.Y"`   |
| `master` between releases (dev)        | `Z`     | `"dev"`      | `"master"`| `"X.Y.Z-dev+master"`    |

Reference commits:

- `151b95d07` — `bump to 5.0.0` (set the released form).
- `0db477863` — `Set 5.0.1 release version` (same pattern, for a patch).
- `4abf3bd54` / `ccde35bcd` — post-release dev bump on the release
  branch and on `master` respectively.

## 1. Pre-release (shared)

Gather the changelog material and label the included MRs / issues so
the `release::X.Y.Z` query links in `CHANGELOG.md` resolve.

```sh
export GITLAB_PRIVATE_TOKEN=...

# 1. Dump everything that closed / was merged since the last release
#    (parallel). The scripts filter by `updated_at` (closest available
#    proxy for merge/close time), which can over-include — the human
#    narrows down in step 3.
python3 scripts/gitlab_api_mrs.py \
  --state merged \
  --updated_after  YYYY-MM-DD \
  --updated_before YYYY-MM-DD \
  --related_issues --closes_issues > mrs.json &
python3 scripts/gitlab_api_issues.py \
  --state closed \
  --updated_after  YYYY-MM-DD \
  --updated_before YYYY-MM-DD > issues.json &
wait

# 2. Map commits to their MRs / issues.
git log --pretty=%H <prev-tag>..<head> > commits.txt
python3 scripts/git_commit_mrs_and_issues.py \
  --merge_requests_file mrs.json \
  --commits commits.txt > commit_map.json

# 3. Decide the final included set and write it to filtered files
#    (selected_mrs.json / selected_issues.json) — typically by walking
#    `commit_map.json` and dropping anything out of scope. Then label
#    only that set; the CHANGELOG label-query links in sections 2c / 3c
#    depend on these labels.
python3 scripts/gitlab_api_labeller.py release::X.Y.Z \
  --mrs    $(jq -r '.[].iid' selected_mrs.json) \
  --issues $(jq -r '.[].iid' selected_issues.json)
```

## 2. Major / minor release (`X.Y.0`)

All steps are on the upstream repo. The release branch is just `X.Y`
(no `release/` prefix; matches existing `3.4`, `5.0`).

a. **Cut the release branch from `master`.**

   ```sh
   git fetch upstream
   git checkout -b X.Y upstream/master
   git push upstream X.Y
   ```

b. **On the release branch, set the released form of `Eigen/Version`.**

   ```c
   #define EIGEN_MAJOR_VERSION X
   #define EIGEN_MINOR_VERSION Y
   #define EIGEN_PATCH_VERSION 0
   #define EIGEN_PRERELEASE_VERSION ""
   #define EIGEN_BUILD_VERSION ""
   #define EIGEN_VERSION_STRING "X.Y.0"
   ```

c. **Promote `[Unreleased]` in `CHANGELOG.md` to `## [X.Y.0] - YYYY-MM-DD`.**
   Use the [5.0.0 entry](CHANGELOG.md) as the template — typical
   sub-sections include `### Versioning`, `### Breaking changes`, then
   per-area sections (`### Elementwise math functions`, `### Dense matrix
   decompositions`, etc.). Include label-query links of the form
   `https://gitlab.com/libeigen/eigen/-/issues?state=all&label_name%5B%5D=release%3A%3AX.Y.0`
   and the analogous merge-requests query.

d. **Commit and tag.** Tags are lightweight (no `v` prefix).

   ```sh
   git commit Eigen/Version CHANGELOG.md -m "Set X.Y.0 release version."
   git tag X.Y.0
   git push upstream X.Y X.Y.0
   ```

e. **Post-tag dev bump of the release branch.** Set `PATCH=1`,
   `PRERELEASE="dev"`, `BUILD="X.Y"`,
   `VERSION_STRING="X.Y.1-dev+X.Y"`. Commit subject:
   `Update dev version number.`

f. **Post-tag bookkeeping on `master`.** There is no rigid convention
   for how `master`'s `Eigen/Version` advances after a release — today
   master tracks the next patch (`5.0.1-dev+master`); for a minor or
   major release decide with maintainers whether to bump
   `MAJOR`/`MINOR` on `master` as well.

## 3. Patch release (`X.Y.Z`, `Z ≥ 1`)

a. **Cherry-pick fixes from `master` to the `X.Y` release branch.**

   ```sh
   git checkout X.Y
   git pull upstream X.Y
   git cherry-pick -x <sha>           # -x records the source SHA
   ```

   Use `cherry-pick -x` consistently — `git_commit_mrs_and_issues.py`
   walks `(cherry picked from commit ...)` trailers to attribute work
   back to its original MR. Re-run CI on the branch after each pick (or
   after each batch) so a bad pick can be reverted in isolation.

   Driving this by hand is tedious. Steve Bronder's
   [`apply_patches.py`](https://gist.github.com/SteveBronder/474845f6673100e9928872a407244362)
   (linked from
   [#3051](https://gitlab.com/libeigen/eigen/-/work_items/3051)) is
   prior art: it reads a CSV of SHAs, cherry-picks each onto a fresh
   branch, and runs configure / build / tests after each pick. The repo
   does not yet vendor an equivalent; if you write one, put it under
   `scripts/`.

b. **On the release branch, set the released form of `Eigen/Version`.**
   Bump `PATCH` to `Z`, clear `PRERELEASE` and `BUILD`, update
   `VERSION_STRING` to `"X.Y.Z"` (model: `0db477863`).

c. **Add a `## [X.Y.Z] - YYYY-MM-DD` section to `CHANGELOG.md`.**
   The [5.0.1 entry](CHANGELOG.md) is the template: a short intro plus
   a bulleted list of fixes with `[#nnnn]` / `[!nnnn]` references, then
   the two `release::X.Y.Z` label-query links.

d. **Commit and tag.**

   ```sh
   git commit Eigen/Version CHANGELOG.md -m "Set X.Y.Z release version."
   git tag X.Y.Z
   git push upstream X.Y X.Y.Z
   ```

e. **Post-tag dev bump of the release branch.** Set `PATCH=Z+1`,
   `PRERELEASE="dev"`, `BUILD="X.Y"`,
   `VERSION_STRING="X.Y.(Z+1)-dev+X.Y"`. Commit subject:
   `Update dev version number.`

## 4. Publish archives to the GitLab package registry

After the tag is pushed, mirror GitLab's auto-generated tag archives
into the project's generic package registry with SHA-256 checksums:

```sh
python3 scripts/gitlab_api_deploy_package.py --version X.Y.Z
```

The script downloads `eigen-X.Y.Z.{tar.gz,tar.bz2,tar,zip}` from
`https://gitlab.com/libeigen/eigen/-/archive/X.Y.Z/`, computes
SHA-256 sums, and uploads each archive + its `.sha256` companion to
`projects/15462818/packages/generic/eigen/X.Y.Z/`.

## 5. Create the GitLab Release object

Manual step in the GitLab UI (no CI automation today): **Project →
Deploy → Releases → New release**. Pick the tag, write a short
description (link to the matching `CHANGELOG.md` section), and add
the package-registry archive URLs from step 4 as release assets.

## 6. Documentation

Per-branch Doxygen output lands on GitLab Pages under
`https://libeigen.gitlab.io/eigen/docs-<branch>` via the `deploy:docs`
job in [`ci/deploy.gitlab-ci.yml`](ci/deploy.gitlab-ci.yml). The job
fires on schedule, on web-triggered pipelines, and on push to the
default branch, only inside the `libeigen` namespace.

To publish docs for a release branch / tag, trigger a **Web** pipeline
on the release branch from **Build → Pipelines → Run pipeline**. The
`PAGES_PREFIX` becomes `docs-<branch>` and the URL becomes
`https://libeigen.gitlab.io/eigen/docs-<branch>`.

Manual build fallback (developer machine, when CI Pages isn't an
option):

```sh
mkdir -p build && cd build
cmake .. && make doc
# output: build/doc/html/
```

The `scripts/eigen_gen_docs` shell script is obsolete (it rsyncs to
`ssh.tuxfamily.org`, the pre-GitLab docs host). Do not use it.

## 7. Announcements

All manual; no tooling in-repo.

- Eigen mailing list.
- Discord `#announcements`.
- Project website news / wiki page (verify still maintained before
  posting).
- Downstream packagers as best-effort: Homebrew, major Linux distros,
  Compiler Explorer (godbolt).

## 8. Cleanup

- Open the `release::X.Y.Z` issue and MR query links from the new
  `CHANGELOG.md` entry and confirm they return non-empty results.
- Close the GitLab milestone for this release if one was used.
- On `master`, ensure `CHANGELOG.md`'s `[Unreleased]` section is empty
  (or re-create it) so the next release's notes start clean.
