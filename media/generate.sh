#!/usr/bin/env bash
# Orchestrate the 3-panel + perf-panel demo:
#   1. emit per-config YAMLs
#   2. capture polka runs (bag + metrics.json) for B/C/D tiles and E_{cuda,cpu}
#   3. render per-panel composite PNGs (parallel)
#   4. render perf bar chart
#   5. compose into pipeline_demo.gif/.mp4
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
WS_ROOT="$(cd "$HERE/../../../.." && pwd)"
BAG="${POLKA_DEMO_BAG:-$WS_ROOT/bags/polka_test/polka_test.mcap}"
CONFIGS="$HERE/configs"
WORK="$HERE/work"
FRAMES="$HERE/frames_panels"

DURATION="${REC_DURATION:-12}"
PERF_DURATION="${PERF_DURATION:-15}"
PERF_REPEATS="${PERF_REPEATS:-3}"

mkdir -p "$WORK" "$FRAMES"

if [[ ! -f "$BAG" ]]; then
  echo "ERROR: bag not found: $BAG" >&2
  exit 1
fi

# 1. Emit per-config YAMLs
python3 "$CONFIGS/generate_panels.py"

set +u
source /opt/ros/humble/setup.bash
source "$WS_ROOT/aux_ws/install/setup.bash"
set -u

cleanup() {
  # pkill returns 1 when no targets match, which is fine; mask it from set -e.
  pkill -9 -f "ros2 bag play"   2>/dev/null || true
  pkill -9 -f "ros2 bag record" 2>/dev/null || true
  pkill -9 -f polka_node        2>/dev/null || true
  return 0
}
trap cleanup EXIT

# 2. Capture
visual_configs=(B_single B_merged
                C_range C_angular C_box C_height
                C2_keep_front C2_exclude_front
                C3_no_self C3_self
                D_voxel D_random_grid
                F_dual
                G_cuda_live G_cpu_live)

for cfg in "${visual_configs[@]}"; do
  out="$WORK/$cfg"
  if [[ -f "$out/metrics.json" ]] && compgen -G "$out/run/*.mcap" > /dev/null; then
    echo "skip $cfg (already captured)"
    continue
  fi
  rm -rf "$out"
  echo "================================================================"
  echo "CAPTURE $cfg"
  echo "================================================================"
  python3 "$HERE/run_capture.py" "$CONFIGS/$cfg.yaml" "$out" \
    --bag "$BAG" --duration "$DURATION" --repeats 1
done

# Deskew panel: synthetic publisher injects 1 rad/s yaw skew so the effect is
# visible regardless of the real bag's motion content.
DESKEW_OMEGA="${DESKEW_OMEGA:-1.0}"
for cfg in A_raw A_deskewed; do
  out="$WORK/$cfg"
  if [[ -f "$out/metrics.json" ]] && compgen -G "$out/run/*.mcap" > /dev/null; then
    echo "skip $cfg (already captured)"
    continue
  fi
  rm -rf "$out"
  echo "================================================================"
  echo "CAPTURE $cfg  (synthetic ω=${DESKEW_OMEGA} rad/s yaw)"
  echo "================================================================"

  # Start the synthetic-skew publisher. It owns /synthetic/lidar/points,
  # /synthetic/imu, and re-latches /tf_static.  5 Hz keeps Python's per-frame
  # numpy work below the publish period (vectorised skew is ~95ms per cloud).
  python3 "$HERE/synthetic_skew_publisher.py" \
    --bag "$BAG" --omega "$DESKEW_OMEGA" --rate 5 \
    --duration "$((DURATION + 6))" \
    > "$WORK/${cfg}.publisher.log" 2>&1 &
  PUB_PID=$!
  # Give the publisher a head start so polka sees /tf_static + first clouds.
  sleep 2

  python3 "$HERE/run_capture.py" "$CONFIGS/$cfg.yaml" "$out" \
    --bag "$BAG" --duration "$DURATION" --skip-bag-play \
    --repeats 1 || true

  kill -INT "$PUB_PID" 2>/dev/null || true
  sleep 1
  kill -9 "$PUB_PID" 2>/dev/null || true
  wait "$PUB_PID" 2>/dev/null || true
done

# Perf panels: multiple repeats
for cfg in E_cuda E_cpu; do
  out="$WORK/$cfg"
  if [[ -d "$out" ]] && [[ -d "$out/rep_0$((PERF_REPEATS - 1))" ]]; then
    echo "skip $cfg (already captured ${PERF_REPEATS} reps)"
    continue
  fi
  rm -rf "$out"
  echo "================================================================"
  echo "CAPTURE $cfg  (×$PERF_REPEATS)"
  echo "================================================================"
  python3 "$HERE/run_capture.py" "$CONFIGS/$cfg.yaml" "$out" \
    --bag "$BAG" --duration "$PERF_DURATION" --repeats "$PERF_REPEATS"
done

# 3a. Render title slide (no capture)
echo "================================================================"
echo "RENDER TITLE"
echo "================================================================"
python3 "$HERE/render_title.py"

# 3b. Render visual panels (B / C / C2 / C3 / D / F / G)
echo "================================================================"
echo "RENDER VISUAL PANELS"
echo "================================================================"
python3 "$HERE/render_panels.py"

# 4. Render perf panel
echo "================================================================"
echo "RENDER PANEL E"
echo "================================================================"
python3 "$HERE/render_perf.py"

# 5. Compose into GIF + MP4
echo "================================================================"
echo "COMPOSE"
echo "================================================================"
bash "$HERE/compose.sh"
