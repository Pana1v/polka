#!/usr/bin/env python3
"""Render Panel B/C/D composite frames from captured polka bags.

Panel B: fusion        = [B_single | B_merged]
Panel C: filters       = 2x2 grid [C_range, C_angular, C_box, C_height]
Panel D: downsample    = [D_voxel | D_random_grid]

Pairing of tiles is keyed by header.stamp with a 50ms tolerance (independent
polka runs don't have matching enumeration indices).
"""
import argparse
import multiprocessing as mp
import pathlib
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from rclpy.serialization import deserialize_message  # noqa: E402
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

CLOUD_TOPIC = "/polka/merged_cloud"

FRAME_W, FRAME_H = 1920, 1080
DPI = 100
BG = "#161616"

# View shared across tiles
XLIM = (-12, 12)
YLIM = (-8, 8)
ZLIM = (-1, 3)
ELEV = 28
AZIM = -58
POINT_SIZE = 6.0
MAX_POINTS = 60000

STAMP_TOLERANCE_NS = 50_000_000  # 50 ms

PANELS = {
    "A": {
        "title": "IMU-based per-point deskew  (synthetic ω = 1.0 rad/s yaw)",
        "layout": "overlay_deskew",
        "align": "index",
        "tiles": [("A_raw",      "raw  (red)"),
                  ("A_deskewed", "deskewed  (green)")],
    },
    "B": {
        "title": "fusion: 1 lidar  →  2 lidars merged",
        "layout": "h2",
        "tiles": [("B_single", "single front lidar"),
                  ("B_merged",  "front + back merged")],
    },
    "C": {
        "title": "output filters (each applied alone)",
        "layout": "g22",
        "tiles": [("C_range",   "range filter (0.5-8m)"),
                  ("C_angular", "angular filter (±45° front)"),
                  ("C_box",     "box filter (±5m x ±3m)"),
                  ("C_height", "height cap (0-1.2m)")],
    },
    "C2": {
        "title": "angular filter: invert flag toggles keep ↔ exclude",
        "layout": "h2",
        "tiles": [("C2_keep_front",    "keep ±45° front"),
                  ("C2_exclude_front", "exclude ±45° front")],
    },
    "C3": {
        "title": "self-filter: exclude ego-body footprint",
        "layout": "h2",
        "tiles": [("C3_no_self", "no self-filter"),
                  ("C3_self",    "chassis box excluded")],
    },
    "D": {
        "title": "downsample: voxel  vs  random_grid  (leaf=0.25m)",
        "layout": "h2",
        "tiles": [("D_voxel",       "voxel (synthetic centroid)"),
                  ("D_random_grid", "random_grid (real point)")],
    },
    "F": {
        "title": "dual outputs: PointCloud2  +  LaserScan",
        "layout": "h2_cloud_scan",
        "tiles": [("F_dual", "merged_cloud (3D)"),
                  ("F_dual", "merged_scan (2D top-down)")],
    },
    "G": {
        "title": "CUDA-accelerated merge pipeline  (same config, enable_gpu toggle)",
        "layout": "h2",
        "tiles": [("G_cuda_live", "CUDA path"),
                  ("G_cpu_live",  "CPU path")],
    },
}


def read_bag(bag_dir: pathlib.Path):
    """Load all PointCloud2 messages from a bag's `run` subdir, return list of
    (stamp_ns, msg) tuples sorted by stamp."""
    actual_bag = bag_dir / "run"
    if not actual_bag.is_dir():
        actual_bag = bag_dir
    so = StorageOptions(uri=str(actual_bag), storage_id="mcap")
    co = ConverterOptions("", "")
    r = SequentialReader()
    r.open(so, co)
    topic_types = {t.name: t.type for t in r.get_all_topics_and_types()}
    if CLOUD_TOPIC not in topic_types:
        return []
    msg_cls = get_message(topic_types[CLOUD_TOPIC])
    out = []
    while r.has_next():
        topic, data, ts = r.read_next()
        if topic == CLOUD_TOPIC:
            msg = deserialize_message(data, msg_cls)
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            out.append((stamp_ns, msg))
    out.sort(key=lambda x: x[0])
    return out


def pc2_xyzi(msg):
    fields = {f.name: f for f in msg.fields}
    if not {"x", "y", "z"}.issubset(fields):
        return np.zeros((0, 4), dtype=np.float32)
    step = msg.point_step
    n = msg.width * msg.height
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)

    def col(name):
        f = fields[name]
        return np.frombuffer(raw[:, f.offset:f.offset + 4].tobytes(), dtype=np.float32)

    xyz = np.stack([col("x"), col("y"), col("z")], axis=1)
    intensity = col("intensity") if "intensity" in fields else np.ones(n, dtype=np.float32)
    return np.concatenate([xyz, intensity.reshape(-1, 1)], axis=1)


def align_by_index(streams):
    """Yield tuples of msgs paired by enumeration index. Use when the input
    sources are synchronized externally (e.g. a publisher feeding clouds in
    deterministic order) but their wall-clock stamps don't overlap."""
    if not streams or any(len(s) == 0 for s in streams):
        return
    n = min(len(s) for s in streams)
    for i in range(n):
        yield tuple(streams[k][i][1] for k in range(len(streams)))


def align_by_stamp(streams):
    """streams = list of [(stamp_ns, msg)] for each tile. Yield aligned tuples
    of (matched_msg_for_each_tile,) keyed by the longest stream's stamps."""
    if not streams or any(len(s) == 0 for s in streams):
        return
    # Use longest stream as the timeline (most frames)
    longest_idx = max(range(len(streams)), key=lambda i: len(streams[i]))
    longest = streams[longest_idx]

    # Sorted stamp arrays for binary search
    others = [(i, np.array([s[0] for s in stream])) for i, stream in enumerate(streams)
              if i != longest_idx]

    for stamp, msg_l in longest:
        match = [None] * len(streams)
        match[longest_idx] = msg_l
        ok = True
        for i, arr in others:
            idx = int(np.searchsorted(arr, stamp))
            best = None
            best_d = STAMP_TOLERANCE_NS + 1
            for j in (idx - 1, idx):
                if 0 <= j < len(arr):
                    d = abs(int(arr[j]) - stamp)
                    if d < best_d:
                        best_d = d
                        best = j
            if best is None or best_d > STAMP_TOLERANCE_NS:
                ok = False
                break
            match[i] = streams[i][best][1]
        if ok:
            yield tuple(match)


def setup_tile(ax, points, sublabel):
    ax.set_facecolor(BG)
    if points is not None and points.shape[0] > 0:
        if points.shape[0] > MAX_POINTS:
            stride = points.shape[0] // MAX_POINTS + 1
            points = points[::stride]
        ax.scatter(points[:, 0], points[:, 1], points[:, 2],
                   c=points[:, 2], cmap="turbo", s=POINT_SIZE,
                   marker=".", linewidths=0, alpha=0.95)
    ax.set_xlim(XLIM); ax.set_ylim(YLIM); ax.set_zlim(ZLIM)
    ax.set_box_aspect((1.5, 1.0, 0.5))
    ax.view_init(elev=ELEV, azim=AZIM)
    ax.set_axis_off()
    ax.text2D(0.02, 0.92, sublabel, transform=ax.transAxes,
              color="white", fontsize=18, family="monospace", weight="bold")


def render_h2(title, tiles, msgs, out_path):
    fig = plt.figure(figsize=(FRAME_W / DPI, FRAME_H / DPI), dpi=DPI, facecolor=BG)
    fig.text(0.5, 0.95, title, color="white", fontsize=28, family="monospace",
             weight="bold", ha="center")
    fig.text(0.99, 0.02, "polka", color="#666", fontsize=14, family="monospace",
             ha="right", weight="bold")
    for i, (msg, (_, sublabel)) in enumerate(zip(msgs, tiles)):
        ax = fig.add_subplot(1, 2, i + 1, projection="3d")
        pts = pc2_xyzi(msg) if msg is not None else None
        count = (msg.width * msg.height) if msg is not None else 0
        setup_tile(ax, pts, sublabel)
        ax.text2D(0.02, 0.05, f"{count:,} pts", transform=ax.transAxes,
                  color="#ccc", fontsize=14, family="monospace")
    fig.subplots_adjust(left=0.01, right=0.99, top=0.92, bottom=0.02,
                        wspace=0.02, hspace=0.02)
    fig.savefig(out_path, dpi=DPI, facecolor=BG)
    plt.close(fig)


def render_g22(title, tiles, msgs, out_path):
    fig = plt.figure(figsize=(FRAME_W / DPI, FRAME_H / DPI), dpi=DPI, facecolor=BG)
    fig.text(0.5, 0.96, title, color="white", fontsize=28, family="monospace",
             weight="bold", ha="center")
    fig.text(0.99, 0.02, "polka", color="#666", fontsize=14, family="monospace",
             ha="right", weight="bold")
    for i, (msg, (_, sublabel)) in enumerate(zip(msgs, tiles)):
        ax = fig.add_subplot(2, 2, i + 1, projection="3d")
        pts = pc2_xyzi(msg) if msg is not None else None
        count = (msg.width * msg.height) if msg is not None else 0
        setup_tile(ax, pts, sublabel)
        ax.text2D(0.02, 0.05, f"{count:,} pts", transform=ax.transAxes,
                  color="#ccc", fontsize=12, family="monospace")
    fig.subplots_adjust(left=0.005, right=0.995, top=0.93, bottom=0.01,
                        wspace=0.02, hspace=0.05)
    fig.savefig(out_path, dpi=DPI, facecolor=BG)
    plt.close(fig)


def render_h2_deskew(title, tiles, msgs, out_path):
    """Side-by-side raw vs deskewed cloud with a tight near-field zoom so the
    per-point smear correction is visible on a moving lidar."""
    fig = plt.figure(figsize=(FRAME_W / DPI, FRAME_H / DPI), dpi=DPI, facecolor=BG)
    fig.text(0.5, 0.95, title, color="white", fontsize=24, family="monospace",
             weight="bold", ha="center")
    fig.text(0.99, 0.02, "polka", color="#666", fontsize=14, family="monospace",
             ha="right", weight="bold")

    # Tight zoom centered on robot — front lidar sees forward/right of base_link.
    DXLIM = (0, 8)
    DYLIM = (-4, 4)
    DZLIM = (0, 3)

    for i, (msg, (_, sublabel)) in enumerate(zip(msgs, tiles)):
        ax = fig.add_subplot(1, 2, i + 1, projection="3d")
        ax.set_facecolor(BG)
        if msg is not None:
            pts = pc2_xyzi(msg)
            if pts.shape[0] > 0:
                # Mask to the near-field crop for visibility
                m = ((pts[:, 0] >= DXLIM[0]) & (pts[:, 0] <= DXLIM[1]) &
                     (pts[:, 1] >= DYLIM[0]) & (pts[:, 1] <= DYLIM[1]) &
                     (pts[:, 2] >= DZLIM[0]) & (pts[:, 2] <= DZLIM[1]))
                pts = pts[m]
                if pts.shape[0] > MAX_POINTS:
                    pts = pts[::pts.shape[0] // MAX_POINTS + 1]
                if pts.shape[0] > 0:
                    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2],
                               c=pts[:, 2], cmap="turbo", s=POINT_SIZE,
                               marker=".", linewidths=0, alpha=0.95)
        ax.set_xlim(DXLIM); ax.set_ylim(DYLIM); ax.set_zlim(DZLIM)
        ax.set_box_aspect((1.0, 1.0, 0.5))
        ax.view_init(elev=20, azim=-65)
        ax.set_axis_off()
        ax.text2D(0.02, 0.92, sublabel, transform=ax.transAxes,
                  color="white", fontsize=18, family="monospace", weight="bold")
        count = (msg.width * msg.height) if msg is not None else 0
        ax.text2D(0.02, 0.05, f"{count:,} pts  ·  near-field crop x∈[0,8]m",
                  transform=ax.transAxes,
                  color="#ccc", fontsize=12, family="monospace")

    fig.subplots_adjust(left=0.01, right=0.99, top=0.92, bottom=0.02,
                        wspace=0.02, hspace=0.02)
    fig.savefig(out_path, dpi=DPI, facecolor=BG)
    plt.close(fig)


def render_h2_cloud_scan(title, tiles, msgs, out_path):
    """3D cloud on left, top-down 2D scan-style view on right.

    Polka's actual LaserScan is a 1D ranges array; we synthesize the
    equivalent view by projecting the height-clipped cloud to 2D polar.
    """
    fig = plt.figure(figsize=(FRAME_W / DPI, FRAME_H / DPI), dpi=DPI, facecolor=BG)
    fig.text(0.5, 0.95, title, color="white", fontsize=28, family="monospace",
             weight="bold", ha="center")
    fig.text(0.99, 0.02, "polka", color="#666", fontsize=14, family="monospace",
             ha="right", weight="bold")

    msg = msgs[0]  # same source for both tiles
    pts = pc2_xyzi(msg) if msg is not None else None
    count = (msg.width * msg.height) if msg is not None else 0

    # Left tile: regular 3D cloud
    ax3 = fig.add_subplot(1, 2, 1, projection="3d")
    setup_tile(ax3, pts, tiles[0][1])
    ax3.text2D(0.02, 0.05, f"{count:,} pts", transform=ax3.transAxes,
               color="#ccc", fontsize=14, family="monospace")

    # Right tile: 2D top-down "scan-like" projection.
    ax2 = fig.add_subplot(1, 2, 2)
    ax2.set_facecolor(BG)
    ax2.set_xticks([]); ax2.set_yticks([])
    for s in ax2.spines.values():
        s.set_color("#444")
    if pts is not None and pts.shape[0] > 0:
        # Height-band slice ≈ what LaserScan flatten would emit (z_min..z_max)
        mask = (pts[:, 2] > -0.5) & (pts[:, 2] < 0.5)
        scan_pts = pts[mask]
        if scan_pts.shape[0] > 0:
            if scan_pts.shape[0] > MAX_POINTS:
                scan_pts = scan_pts[::scan_pts.shape[0] // MAX_POINTS + 1]
            r = np.hypot(scan_pts[:, 0], scan_pts[:, 1])
            ax2.scatter(scan_pts[:, 0], scan_pts[:, 1], c=r,
                        cmap="plasma", s=POINT_SIZE,
                        marker=".", linewidths=0, alpha=0.95)
    # Robot at origin
    ax2.plot(0, 0, marker="^", color="white", markersize=18)
    ax2.set_xlim(-15, 15); ax2.set_ylim(-10, 10)
    ax2.set_aspect("equal")
    ax2.text(0.02, 0.92, tiles[1][1], transform=ax2.transAxes,
             color="white", fontsize=18, family="monospace", weight="bold")
    ax2.text(0.02, 0.05, "z ∈ [-0.5, 0.5]m  ·  bird's-eye projection",
             transform=ax2.transAxes,
             color="#ccc", fontsize=12, family="monospace")

    fig.subplots_adjust(left=0.01, right=0.99, top=0.92, bottom=0.02,
                        wspace=0.02, hspace=0.02)
    fig.savefig(out_path, dpi=DPI, facecolor=BG)
    plt.close(fig)


def render_overlay_deskew(title, tiles, msgs, out_path):
    """Overlay raw (red) and deskewed (green) clouds on the SAME 3D axes so the
    smear shows up as a directly comparable red↔green offset.

    Top-down (bird's-eye) view, tight crop on the near-field where the yaw
    smear is largest in linear distance.
    """
    fig = plt.figure(figsize=(FRAME_W / DPI, FRAME_H / DPI), dpi=DPI, facecolor=BG)
    fig.text(0.5, 0.95, title, color="white", fontsize=26, family="monospace",
             weight="bold", ha="center")
    fig.text(0.99, 0.02, "polka", color="#666", fontsize=14, family="monospace",
             ha="right", weight="bold")

    # Tight near-field x∈[1.5,5]m, y∈[-2.5,2.5]m. Top-down 2D so red↔green
    # offset along the tangential (y) axis is unambiguous.
    DXLIM = (1.5, 5.0)
    DYLIM = (-2.5, 2.5)
    DZLIM = (-0.5, 2.5)

    ax = fig.add_axes([0.05, 0.08, 0.9, 0.80])
    ax.set_facecolor(BG)
    ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values():
        s.set_color("#444")

    colors_alphas = [("#ff4040", 0.55), ("#40ff60", 0.55)]
    counts = []
    for (msg, (_, sublabel)), (color, alpha) in zip(zip(msgs, tiles), colors_alphas):
        if msg is None:
            counts.append(0)
            continue
        pts = pc2_xyzi(msg)
        counts.append(msg.width * msg.height)
        if pts.shape[0] == 0:
            continue
        m = ((pts[:, 0] >= DXLIM[0]) & (pts[:, 0] <= DXLIM[1]) &
             (pts[:, 1] >= DYLIM[0]) & (pts[:, 1] <= DYLIM[1]) &
             (pts[:, 2] >= DZLIM[0]) & (pts[:, 2] <= DZLIM[1]))
        pts = pts[m]
        if pts.shape[0] > MAX_POINTS:
            pts = pts[::pts.shape[0] // MAX_POINTS + 1]
        if pts.shape[0] > 0:
            # Top-down view: scatter x vs y, colored by uniform red/green
            ax.scatter(pts[:, 0], pts[:, 1], c=color, s=8.0,
                       marker=".", linewidths=0, alpha=alpha)

    ax.set_xlim(DXLIM); ax.set_ylim(DYLIM)
    ax.set_aspect("equal")

    # Robot at origin (bottom of view)
    ax.plot(0, 0, marker="^", color="white", markersize=18, clip_on=False)

    # Legend overlays
    ax.text(0.02, 0.95, "■  raw  (1 rad/s yaw smear)",
            transform=ax.transAxes,
            color="#ff4040", fontsize=22, family="monospace", weight="bold")
    ax.text(0.02, 0.89, "■  deskewed  (SE(3) per-point correction)",
            transform=ax.transAxes,
            color="#40ff60", fontsize=22, family="monospace", weight="bold")
    ax.text(0.02, 0.04, f"bird's-eye  ·  x∈[1.5,5]m  ·  red {counts[0]:,} pts  ·  green {counts[1]:,} pts",
            transform=ax.transAxes,
            color="#bbb", fontsize=14, family="monospace")

    fig.savefig(out_path, dpi=DPI, facecolor=BG)
    plt.close(fig)


def _worker(args):
    panel_id, title, tiles, layout, msgs, out_path = args
    if layout == "h2":
        render_h2(title, tiles, msgs, out_path)
    elif layout == "h2_cloud_scan":
        render_h2_cloud_scan(title, tiles, msgs, out_path)
    elif layout == "h2_deskew":
        render_h2_deskew(title, tiles, msgs, out_path)
    elif layout == "overlay_deskew":
        render_overlay_deskew(title, tiles, msgs, out_path)
    else:
        render_g22(title, tiles, msgs, out_path)


def render_panel(panel_id, work_dir, out_dir, pool):
    spec = PANELS[panel_id]
    print(f"[{panel_id}] loading bags...", flush=True)
    cache = {}
    streams = []
    for tile_name, _ in spec["tiles"]:
        if tile_name in cache:
            streams.append(cache[tile_name])
            continue
        bag_dir = work_dir / tile_name
        s = read_bag(bag_dir)
        if not s:
            print(f"  WARN: empty stream for {tile_name}", flush=True)
        cache[tile_name] = s
        streams.append(s)
    print(f"[{panel_id}] streams: {[len(s) for s in streams]}", flush=True)

    out_panel_dir = out_dir / f"panel_{panel_id}"
    out_panel_dir.mkdir(parents=True, exist_ok=True)

    align = spec.get("align", "stamp")
    aligner = align_by_index if align == "index" else align_by_stamp

    tasks = []
    for i, matched in enumerate(aligner(streams)):
        out_path = out_panel_dir / f"f_{i:04d}.png"
        tasks.append((panel_id, spec["title"], spec["tiles"], spec["layout"],
                      matched, str(out_path)))

    if not tasks:
        print(f"[{panel_id}] no aligned frames", flush=True)
        return 0
    print(f"[{panel_id}] rendering {len(tasks)} frames...", flush=True)
    for _ in pool.imap_unordered(_worker, tasks, chunksize=4):
        pass
    return len(tasks)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--work", type=pathlib.Path,
                    default=pathlib.Path(__file__).parent / "work")
    ap.add_argument("--out", type=pathlib.Path,
                    default=pathlib.Path(__file__).parent / "frames_panels")
    ap.add_argument("--only", choices=list(PANELS.keys()), default=None)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    workers = max(1, mp.cpu_count() // 2)
    print(f"workers: {workers}", flush=True)
    pool = mp.Pool(processes=workers)
    try:
        panels = [args.only] if args.only else list(PANELS.keys())
        for pid in panels:
            render_panel(pid, args.work, args.out, pool)
    finally:
        pool.close()
        pool.join()


if __name__ == "__main__":
    main()
