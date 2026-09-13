# Maintaining polka across ROS 2 distros

polka supports five ROS 2 distributions, one branch each:

| Distro  | Codename  | Ubuntu | LTS | Branch    |
|---------|-----------|--------|-----|-----------|
| Humble  | Hawksbill | 22.04  | yes | `humble`  |
| Iron    | Irwini    | 22.04  | no  | `iron`    |
| Jazzy   | Jalisco   | 24.04  | yes | `jazzy`   |
| Kilted  | Kaiju     | 24.04  | no  | `kilted`  |
| Lyrical | Luth      | 26.04  | yes | `lyrical` |

The branches hold identical code on purpose. This document is about keeping them that
way without doing the same work five times.

## One source of truth, fanned out

```
            feature PR
                │
                ▼
            humble  ──────────────  source of truth (develop here)
                │  scripts/sync-distros.sh   (merge → build-verify → push)
    ┌───────────┼───────────┬───────────┬───────────┐
    ▼           ▼           ▼           ▼           ▼
  iron        jazzy       kilted      lyrical    (+humble)
 22.04        24.04       24.04        26.04
    └────────────  .github/workflows/ci.yml builds all 5 per push/PR  ───────────┘
```

**Why develop on `humble`, the oldest distro?**
Newer distros are almost always backward-compatible, so code that builds on Humble
builds forward to Lyrical. Going the other way, it's far too easy to reach for a
new-only API and break the older branches without noticing. So the oldest distro wins.

## Day to day

1. Branch off `humble`:  `git checkout humble && git checkout -b panav/feat/my-thing`
2. Write and test it on Humble, then open a PR into `humble`. CI builds it on **all five** distros.
3. Once it merges, fan it out:
   ```bash
   scripts/sync-distros.sh            # merge humble → iron/jazzy/kilted/lyrical, build, push
   scripts/sync-distros.sh --dry-run  # preview first
   scripts/sync-distros.sh --no-build # skip docker builds (CI still verifies)
   ```

Don't hand-create `-jazzy` / `-kilted` sibling feature branches any more. The sync
script is the fan-out.

## When a distro really does need different code

Two options, in order of preference.

### 1. Compile-time guards — preferred

Branch *inside the shared source file* so the same file builds everywhere and the
branches stay byte-identical, which keeps sync a trivial fast-forward:

```cpp
// Header that moved between distros (cv_bridge .h -> .hpp in Jazzy+):
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#  include <cv_bridge/cv_bridge.hpp>   // Jazzy / Kilted / Lyrical
#else
#  include <cv_bridge/cv_bridge.h>      // Humble / Iron
#endif

// API that changed by version:
#include <rclcpp/rclcpp.hpp>
#if RCLCPP_VERSION_GTE(17, 0, 0)
  // newer API
#else
  // Humble-era API
#endif
```

Reach for `__has_include`, `RCLCPP_VERSION_GTE(major, minor, patch)` or a CMake-provided
`POLKA_ROS_DISTRO` define before you fork branch history.

> polka still includes `<pcl_conversions/pcl_conversions.h>`, which resolves on all five
> distros. If a future distro moves it, wrap it in `__has_include` rather than letting
> the branches diverge.

### 2. A thin per-distro overlay — only if a guard can't work

Some differences won't fit in an `#if`: a `package.xml` version pin, a
`cmake_minimum_required` bump, a distro-only dependency name. Keep those as a small,
stable, clearly labeled set of commits at the tip of that distro branch. The sync
script merges `humble` underneath them, so conflicts land exactly on the lines that
genuinely differ, which is the signal you want. Nothing is auto-resolved.

## CI

[`.github/workflows/ci.yml`](.github/workflows/ci.yml) runs a `fail-fast: false` matrix
that builds and tests polka in `osrf/ros:<distro>-desktop` containers for all five
distros, on every push and PR. That's what catches a break on, say, Lyrical, whose
`ament_target_dependencies()` removal is invisible to someone working on Humble
(handled in `CMakeLists.txt` behind an `if(COMMAND ...)` guard).

## Adding a new distro

1. `git checkout <nearest-existing-sibling> && git checkout -b <newdistro>` — match the
   Ubuntu lineage, so a 26.04 release branches from `lyrical`.
2. Add it to `ALL_DISTROS` and `BUILD_DISTRO` in `scripts/sync-distros.sh`.
3. Add a matrix entry in `.github/workflows/ci.yml` and to the `on:` branch lists.
4. Add the row to the support table in `README.md`, plus a badge.
5. Run `scripts/sync-distros.sh` to bring it current, then push.
