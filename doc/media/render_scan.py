#!/usr/bin/env python3
"""
Render the 2D LaserScan merge panel.

Left:  each lidar's own flattened scan, overlaid and color-coded.
Right: the merged scan, where every beam is colored by the sensor that *won*
       that direction (provided the nearest return). This is exactly polka's
       merge rule (closest obstacle per angular bin) and makes it visible that
       all three sensors contribute to the single merged scan.

Reads /polka/merged_scan from the single-source captures (ouster_scan,
avia_scan, mid360_scan); the merged scan is reconstructed + attributed from
them (identical to polka's closest-per-bin merge). Frames pair by index (the
bag is near-stationary, so index pairing is sufficient).
"""
import argparse
import pathlib

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from rclpy.serialization import deserialize_message  # noqa: E402
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

SCAN_TOPIC = '/polka/merged_scan'
BG = '#121214'
LIM = 6.5
PT = 9
# fixed order -> stable colour mapping / winner index
SOURCES = [('ouster_scan', 'Ouster', '#2bd4d4'),
           ('avia_scan', 'Avia', '#ff9a3c'),
           ('mid360_scan', 'Mid-360', '#7CFC54')]
COLORS = [c for _, _, c in SOURCES]


def read_scan_ranges(run_dir):
    """
    Return per-frame range arrays plus scan geometry.

    Returns (frames, angle_min, angle_increment), where frames is a list of
    per-frame range arrays with NaN for invalid bins.
    """
    d = run_dir / 'run'
    if not d.is_dir():
        d = run_dir
    r = SequentialReader()
    r.open(StorageOptions(uri=str(d), storage_id='mcap'), ConverterOptions('', ''))
    frames, amin, ainc = [], None, None
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic != SCAN_TOPIC:
            continue
        m = deserialize_message(data, LaserScan)
        amin, ainc = m.angle_min, m.angle_increment
        rng = np.asarray(m.ranges, dtype=np.float32)
        bad = ~(np.isfinite(rng) & (rng > m.range_min) & (rng < m.range_max))
        rng = rng.copy()
        rng[bad] = np.nan
        frames.append(rng)
    return frames, amin, ainc


def style(ax, title):
    ax.set_facecolor(BG)
    ax.set_xlim(-LIM, LIM)
    ax.set_ylim(-LIM, LIM)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_color('#333')
    ax.plot(0, 0, marker='^', color='white', markersize=15, zorder=5)
    ax.text(0.03, 0.96, title, transform=ax.transAxes, color='white',
            fontsize=20, family='monospace', weight='bold', va='top')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--work', type=pathlib.Path,
                    default=pathlib.Path(__file__).parent / 'work')
    ap.add_argument('--out', type=pathlib.Path,
                    default=pathlib.Path(__file__).parent / 'frames' / 'scan_merge')
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    for old in args.out.glob('f_*.png'):
        old.unlink()

    srcs = []
    amin = ainc = None
    for cfg, label, color in SOURCES:
        frames, am, ai = read_scan_ranges(args.work / f'cap_{cfg}')
        srcs.append((label, color, frames))
        amin, ainc = am, ai

    n = min(len(f) for _, _, f in srcs)
    nbins = len(srcs[0][2][0])
    ang = amin + np.arange(nbins) * ainc
    cos, sin = np.cos(ang), np.sin(ang)
    print(f'frames -> {n}, bins={nbins}', flush=True)

    for i in range(n):
        # (3, nbins) range matrix, NaN where a source has no return
        R = np.vstack([srcs[k][2][i] for k in range(3)])
        fig = plt.figure(figsize=(19.2, 10.0), dpi=100, facecolor=BG)
        fig.suptitle('2D LaserScan merge:  nearest return per direction wins',
                     color='white', fontsize=28, family='monospace',
                     weight='bold', y=0.965)

        # Left: each sensor's full scan overlaid, alpha-blended so overlapping
        # coverage shows as a mix instead of whichever colour is drawn last.
        axL = fig.add_subplot(1, 2, 1)
        style(axL, 'individual scans')
        for k, (label, color, _) in enumerate(srcs):
            v = np.isfinite(R[k])
            axL.scatter(R[k][v] * cos[v], R[k][v] * sin[v], s=PT, c=color,
                        linewidths=0, alpha=0.5)
            axL.text(0.03, 0.90 - 0.05 * k, f'■ {label}', transform=axL.transAxes,
                     color=color, fontsize=16, family='monospace', weight='bold')

        # Right: merged = nearest per bin, colored by the winning sensor
        axR = fig.add_subplot(1, 2, 2)
        style(axR, 'merged scan  (nearest return wins)')
        anyvalid = np.isfinite(R).any(axis=0)
        Rfill = np.where(np.isfinite(R), R, np.inf)
        winner = np.argmin(Rfill, axis=0)
        mrange = np.min(Rfill, axis=0)
        wins = []
        for k, (label, color, _) in enumerate(srcs):
            sel = anyvalid & (winner == k)
            axR.scatter(mrange[sel] * cos[sel], mrange[sel] * sin[sel],
                        s=PT + 1, c=color, linewidths=0)
            wins.append((label, color, int(sel.sum())))
        tot = max(1, sum(w[2] for w in wins))
        x0 = 0.03
        for label, color, c in wins:
            axR.text(x0, 0.05, f'{label} {round(100*c/tot)}%', transform=axR.transAxes,
                     color=color, fontsize=15, family='monospace', weight='bold')
            x0 += 0.013 * (len(label) + 5)

        fig.text(0.99, 0.015, 'polka', color='#666', fontsize=15,
                 family='monospace', ha='right', weight='bold')
        fig.subplots_adjust(left=0.01, right=0.99, top=0.92, bottom=0.01, wspace=0.02)
        fig.savefig(args.out / f'f_{i:04d}.png', facecolor=BG)
        plt.close(fig)
    print(f'wrote {n} frames to {args.out}', flush=True)


if __name__ == '__main__':
    main()
