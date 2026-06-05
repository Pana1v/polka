# Maintaining polka across ROS 2 distros

polka supports five ROS 2 distributions, each on its own branch:

| Distro  | Codename  | Ubuntu | LTS | Branch    |
|---------|-----------|--------|-----|-----------|
| Humble  | Hawksbill | 22.04  | yes | `humble`  |
| Iron    | Irwini    | 22.04  | no  | `iron`    |
| Jazzy   | Jalisco   | 24.04  | yes | `jazzy`   |
| Kilted  | Kaiju     | 24.04  | no  | `kilted`  |
| Lyrical | Luth      | 26.04  | yes | `lyrical` |

The branches are intentionally **code-identical**. The whole point of this document is to
keep them that way with as little duplicated effort as possible.

## The model: single source of truth + fan-out

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

**Why develop on `humble` (the oldest supported distro)?**
Newer distros are overwhelmingly backward-compatible, so code that compiles on Humble
almost always compiles forward to Lyrical. Developing on the newest distro and
back-porting is the opposite — it's easy to reach for a new-only API that breaks the
older branches. So the oldest supported distro is the source of truth.

## Day-to-day contributor workflow

1. Branch off `humble`:  `git checkout humble && git checkout -b panav/feat/my-thing`
2. Implement + test on Humble. Open a PR into `humble`. CI builds it on **all five** distros.
3. After merge, fan the change out:
   ```bash
   scripts/sync-distros.sh            # merge humble → iron/jazzy/kilted/lyrical, build, push
   scripts/sync-distros.sh --dry-run  # preview first
   scripts/sync-distros.sh --no-build # skip docker builds (CI will still verify)
   ```

That's it. **Do not** hand-create `-jazzy` / `-kilted` sibling feature branches anymore —
the sync script is the fan-out mechanism.

## When a distro genuinely needs different code

Two layered techniques, in order of preference:

### 1. Compile-time guards (keep branches identical) — preferred

Branch *inside the shared source file* so the same file compiles everywhere and the
branches stay byte-identical (sync remains a trivial fast-forward). Examples:

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

Prefer `__has_include`, `RCLCPP_VERSION_GTE(major, minor, patch)`, or a CMake-provided
`POLKA_ROS_DISTRO` define over forking the branch history.

> Note: polka currently includes `<pcl_conversions/pcl_conversions.h>`, which still
> resolves on all five distros. If a future distro moves it, wrap it with `__has_include`
> rather than diverging the branches.

### 2. Thin per-distro overlay — only when a guard is impossible

Some differences can't live in `#if` blocks — e.g. a `package.xml` dependency version
pin, a `cmake_minimum_required` bump, or a distro-only dependency name. Keep these as a
**small, stable, clearly-labeled set of commits at the tip of that distro branch**. The
sync script merges `humble` underneath them; conflicts surface exactly at the lines that
legitimately differ (the signal you want) and are never auto-resolved.

## CI: the safety net

[`.github/workflows/ci.yml`](.github/workflows/ci.yml) runs a `fail-fast: false` matrix
that builds and tests `polka` in `osrf/ros:<distro>-desktop` containers for all five
distros on every push and PR. This is what automatically catches a break on, say,
Lyrical — whose `ament_target_dependencies()` removal would otherwise be invisible to an
author working on Humble (handled in `CMakeLists.txt` with a `if(COMMAND ...)` guard).

## Releasing a new distro (the next "M" release)

1. `git checkout <nearest-existing-sibling> && git checkout -b <newdistro>` (match the
   Ubuntu lineage — e.g. a 26.04 release branches from `lyrical`).
2. Add it to `ALL_DISTROS` and `BUILD_DISTRO` in `scripts/sync-distros.sh`.
3. Add a matrix entry in `.github/workflows/ci.yml` and the `on:` branch lists.
4. Add the row to the support table in `README.md` and a distro badge.
5. `scripts/sync-distros.sh` to bring it current; push.
