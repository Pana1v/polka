#!/usr/bin/env bash
# Compose per-panel PNG directories into pipeline_demo.gif/.mp4.
# Layout: title slide → B → C → C2 → C3 → D → F → G → E perf chart.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
FRAMES="$HERE/frames_panels"
WORK="$HERE/work"
COMBINED="$WORK/combined_panel_frames"
mkdir -p "$WORK"
rm -rf "$COMBINED"
mkdir -p "$COMBINED"

FPS=15
PANEL_HOLD_FRAMES=8    # 0.5s hold at end of each panel (15fps)
PERF_HOLD_FRAMES=75    # 5s hold on perf panel
TITLE_HOLD_FRAMES=37   # 2.5s hold on title

idx=0

# Title slide
title_png="$FRAMES/panel_A_title/f_0000.png"
if [[ -f "$title_png" ]]; then
  echo "  panel_A_title: held $TITLE_HOLD_FRAMES frames"
  for _ in $(seq 1 "$TITLE_HOLD_FRAMES"); do
    ln -sf "$title_png" "$COMBINED/$(printf '%06d' $idx).png"
    idx=$((idx + 1))
  done
fi

# Visual panels in story order
for panel in panel_A panel_B panel_C panel_C2 panel_C3 panel_D panel_F panel_G; do
  dir="$FRAMES/$panel"
  count=$(ls "$dir"/f_*.png 2>/dev/null | wc -l)
  if [[ "$count" == "0" ]]; then
    echo "skip $panel (no frames)" >&2
    continue
  fi
  echo "  $panel: $count frames"
  for f in "$dir"/f_*.png; do
    ln -sf "$f" "$COMBINED/$(printf '%06d' $idx).png"
    idx=$((idx + 1))
  done
  last=$(ls "$dir"/f_*.png | tail -1)
  for _ in $(seq 1 "$PANEL_HOLD_FRAMES"); do
    ln -sf "$last" "$COMBINED/$(printf '%06d' $idx).png"
    idx=$((idx + 1))
  done
done

# Perf panel — single PNG repeated.
perf_png="$FRAMES/panel_E/f_0000.png"
if [[ -f "$perf_png" ]]; then
  echo "  panel_E: held $PERF_HOLD_FRAMES frames"
  for _ in $(seq 1 "$PERF_HOLD_FRAMES"); do
    ln -sf "$perf_png" "$COMBINED/$(printf '%06d' $idx).png"
    idx=$((idx + 1))
  done
fi

echo "total: $idx frames @ ${FPS}fps"

MP4="$HERE/pipeline_demo.mp4"
GIF="$HERE/pipeline_demo.gif"

# --- MP4 ---
echo "rendering $MP4"
ffmpeg -y -framerate "$FPS" -i "$COMBINED/%06d.png" \
  -vf "scale=960:-2,format=yuv420p" \
  -c:v mpeg4 -q:v 9 \
  "$MP4" 2> "$WORK/ffmpeg-mp4.log"

# --- GIF (two-pass via rawvideo intermediate) ---
# Direct PNG sequence → palettegen → paletteuse triggers an ffmpeg "Internal
# bug" with `select`/`fps` filters mid-graph in this container's ffmpeg 4.4.
# Workaround: bake the frame-subsampling + scale into a rawvideo nut, then
# generate + apply the palette in two clean passes.
echo "rendering $GIF"
TMPRAW="$WORK/gif_input.nut"
PALETTE="$WORK/palette.png"

# Sub-sample to 8fps and 560px wide, every other frame
ffmpeg -y -framerate "$FPS" -i "$COMBINED/%06d.png" \
  -vf "select='not(mod(n,2))',setpts=N/8/TB,scale=560:-2:flags=lanczos,format=rgb24" \
  -c:v rawvideo -pix_fmt rgb24 -f nut "$TMPRAW" 2> "$WORK/ffmpeg-tmp.log"

ffmpeg -y -i "$TMPRAW" -vf "palettegen=max_colors=48" "$PALETTE" \
  2> "$WORK/ffmpeg-palette.log"

ffmpeg -y -i "$TMPRAW" -i "$PALETTE" \
  -lavfi "paletteuse=dither=bayer:bayer_scale=5" \
  "$GIF" 2> "$WORK/ffmpeg-gif.log"

rm -f "$TMPRAW"

echo ""
echo "outputs:"
ls -lh "$MP4" "$GIF"
