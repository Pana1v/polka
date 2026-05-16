#!/usr/bin/env python3
"""Render the title slide PNG (single frame held at the start of the GIF).

Uses the actual polka logo from images/polka.png; falls back to a text wordmark
if the logo file is missing.
"""
import pathlib

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.image as mpimg  # noqa: E402

BG = "#161616"
W, H = 1920, 1080
DPI = 100

LOGO_PATH = pathlib.Path(__file__).resolve().parent.parent / "images" / "polka.png"

FEATURES = [
    "multi-LiDAR fusion  (any mix of PointCloud2 + LaserScan)",
    "IMU-based per-point deskew",
    "per-source range / angular / box filters",
    "output filters  (range, angular, box, height, footprint)",
    "downsample  (voxel centroid  •  random_grid real points)",
    "dual outputs  (PointCloud2  +  LaserScan)",
    "CUDA-accelerated merge pipeline  (CPU fallback)",
    "runtime parameter reload",
]


def render(out_path):
    fig = plt.figure(figsize=(W / DPI, H / DPI), dpi=DPI, facecolor=BG)

    # ---- top zone: logo ----
    ax_logo = fig.add_axes([0.18, 0.55, 0.64, 0.40])
    ax_logo.set_facecolor(BG)
    ax_logo.set_xticks([]); ax_logo.set_yticks([])
    for s in ax_logo.spines.values():
        s.set_visible(False)
    if LOGO_PATH.is_file():
        img = mpimg.imread(str(LOGO_PATH))
        ax_logo.imshow(img, aspect="equal")
    else:
        ax_logo.text(0.5, 0.5, "polka", color="white", fontsize=130,
                     family="monospace", weight="bold",
                     ha="center", va="center")

    # ---- tagline below logo ----
    fig.text(0.5, 0.52, "multi-LiDAR fusion node for ROS 2",
             color="#aaaaaa", fontsize=24, family="monospace",
             ha="center", va="center")

    # ---- feature list ----
    ax_list = fig.add_axes([0, 0, 1, 0.45])
    ax_list.set_facecolor(BG)
    ax_list.set_xticks([]); ax_list.set_yticks([])
    for s in ax_list.spines.values():
        s.set_visible(False)
    ax_list.set_xlim(0, 1)
    ax_list.set_ylim(0, 1)

    list_x = 0.22
    list_y_top = 0.92
    line_spacing = 0.105
    for i, feat in enumerate(FEATURES):
        ax_list.text(list_x, list_y_top - i * line_spacing,
                     f"  ◾  {feat}",
                     color="#dddddd", fontsize=18, family="monospace",
                     ha="left", va="center")

    fig.text(0.5, 0.02, "github.com/Pan-Navigator/polka",
             color="#666666", fontsize=13, family="monospace",
             ha="center", va="center", style="italic")

    fig.savefig(out_path, dpi=DPI, facecolor=BG)
    plt.close(fig)


def main():
    out = pathlib.Path(__file__).parent / "frames_panels" / "panel_A_title"
    out.mkdir(parents=True, exist_ok=True)
    path = out / "f_0000.png"
    render(path)
    print(f"wrote {path}")


if __name__ == "__main__":
    main()
