#!/usr/bin/env bash
#
# sync-distros.sh — fan out the source-of-truth branch to every ROS 2 distro branch.
#
# polka maintains one branch per supported ROS 2 distro. They are intentionally
# code-identical: development happens on a single SOURCE branch (humble, the oldest
# supported distro), and this script merges that branch forward into each downstream
# distro branch, building each one to catch per-distro API/syntax drift before pushing.
#
# Where a distro genuinely needs different code, prefer a compile-time guard in the
# shared source (see MAINTAINING.md) so branches stay identical and merges stay clean.
# If a branch carries a deliberate per-distro overlay, this script merges on top of it;
# conflicts are surfaced (never auto-resolved) for a human to handle.
#
# Usage:
#   scripts/sync-distros.sh              # merge + build-verify + push all distro branches
#   scripts/sync-distros.sh --dry-run    # show what would happen; no merge/push
#   scripts/sync-distros.sh --no-build   # skip the colcon build verification
#   scripts/sync-distros.sh --no-push    # merge + build locally, do not push
#   SOURCE=jazzy scripts/sync-distros.sh # override the source-of-truth branch
#
set -euo pipefail

SOURCE="${SOURCE:-humble}"
# Downstream branches to sync (everything except SOURCE).
ALL_DISTROS=(humble iron jazzy kilted lyrical)
# ROS distro used to build each branch.
declare -A BUILD_DISTRO=( [humble]=humble [iron]=iron [jazzy]=jazzy [kilted]=kilted [lyrical]=lyrical )

DRY_RUN=0; DO_BUILD=1; DO_PUSH=1
for arg in "$@"; do
  case "$arg" in
    --dry-run)  DRY_RUN=1 ;;
    --no-build) DO_BUILD=0 ;;
    --no-push)  DO_PUSH=0 ;;
    -h|--help)  sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "unknown option: $arg" >&2; exit 2 ;;
  esac
done

log()  { printf '\033[1;34m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m!!\033[0m %s\n' "$*" >&2; }
run()  { if [[ $DRY_RUN -eq 1 ]]; then echo "  [dry-run] $*"; else "$@"; fi; }

# Refuse to run on a dirty tree — we switch branches and merge.
if [[ -n "$(git status --porcelain)" ]]; then
  warn "working tree is dirty; commit or stash before syncing."; exit 1
fi

START_BRANCH="$(git rev-parse --abbrev-ref HEAD)"
restore() { git checkout --quiet "$START_BRANCH" 2>/dev/null || true; }
trap restore EXIT

log "Source of truth: $SOURCE"
git checkout --quiet "$SOURCE"

FAILED=()
for distro in "${ALL_DISTROS[@]}"; do
  [[ "$distro" == "$SOURCE" ]] && continue
  log "Syncing $distro  <-  $SOURCE"

  if ! git rev-parse --verify --quiet "$distro" >/dev/null; then
    warn "branch '$distro' does not exist locally — skipping (create it first)."
    FAILED+=("$distro:missing"); continue
  fi

  run git checkout --quiet "$distro"
  if [[ $DRY_RUN -eq 0 ]]; then
    if ! git merge --no-edit "$SOURCE"; then
      warn "MERGE CONFLICT on '$distro' — resolve manually, then re-run. Aborting this branch."
      git merge --abort
      FAILED+=("$distro:conflict"); git checkout --quiet "$SOURCE"; continue
    fi
  else
    echo "  [dry-run] git merge --no-edit $SOURCE"
  fi

  if [[ $DO_BUILD -eq 1 ]]; then
    bd="${BUILD_DISTRO[$distro]}"
    log "Build-verify '$distro' in osrf/ros:${bd}-desktop"
    run docker run --rm -v "$PWD/../..:/ws" -w /ws "osrf/ros:${bd}-desktop" \
      bash -lc "source /opt/ros/${bd}/setup.bash && colcon build --packages-select polka"
  fi

  if [[ $DO_PUSH -eq 1 ]]; then
    run git push origin "$distro"
  fi
done

git checkout --quiet "$SOURCE"

if [[ ${#FAILED[@]} -gt 0 ]]; then
  warn "completed with issues: ${FAILED[*]}"; exit 1
fi
log "All distro branches synced from '$SOURCE'."
