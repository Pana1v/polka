#!/usr/bin/env bash
# Encode each rendered panel's PNG sequence into a separate GIF with gifski.
# gifski isn't in apt and cargo isn't installed; install the prebuilt binary
# into ~/.local/bin (no sudo) if it's missing.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
FRAMES="$HERE/frames"
GIFS="$HERE/gifs"
FPS="${GIF_FPS:-11}"
WIDTH="${GIF_WIDTH:-1000}"
QUALITY="${GIF_QUALITY:-85}"
STRIDE="${GIF_STRIDE:-1}"   # use every Nth frame (near-static scene tolerates >1)

mkdir -p "$GIFS"

if ! command -v gifski >/dev/null 2>&1 && [[ ! -x "$HOME/.local/bin/gifski" ]]; then
  echo "installing gifski to ~/.local/bin ..."
  tmp="$(mktemp -d)"
  curl -sSL -o "$tmp/gifski.deb" \
    "https://github.com/ImageOptim/gifski/releases/download/1.32.0/gifski_1.32.0-1_amd64.deb"
  dpkg-deb -x "$tmp/gifski.deb" "$tmp/x"
  mkdir -p "$HOME/.local/bin"
  cp "$(find "$tmp/x" -name gifski -type f | head -1)" "$HOME/.local/bin/gifski"
  chmod +x "$HOME/.local/bin/gifski"
  rm -rf "$tmp"
fi
GIFSKI="$(command -v gifski || echo "$HOME/.local/bin/gifski")"

for dir in "$FRAMES"/*/; do
  panel="$(basename "$dir")"
  if ! compgen -G "$dir/f_*.png" > /dev/null; then
    echo "skip $panel (no frames)"; continue
  fi
  echo "encoding $panel ..."
  mapfile -t allf < <(ls "$dir"/f_*.png)
  sel=()
  for ((i = 0; i < ${#allf[@]}; i += STRIDE)); do sel+=("${allf[i]}"); done
  "$GIFSKI" --fps "$FPS" --quality "$QUALITY" --width "$WIDTH" \
    -o "$GIFS/$panel.gif" "${sel[@]}" > /dev/null 2>&1
done

echo "---"
ls -lh "$GIFS"
