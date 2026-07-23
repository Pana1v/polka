#!/usr/bin/env python3
"""
Render per-feature demo panels with Open3D offscreen + matplotlib compositing.

Open3D's GPU offscreen renderer draws the point-cloud pixels (depth, AA); a thin
matplotlib layer composes the divided panel with titles/labels. One panel = a
left/right comparison of two captured polka runs (or cloud+scan for dual output).

  python3 render_o3d.py --work <work_dir> --out <frames_dir> [--only fusion]

<work_dir> holds cap_<config>/run mcap bags (from run_capture.py).
"""
import argparse
import pathlib

import matplotlib
matplotlib.use('Agg')
import matplotlib.cm as cm  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import open3d as o3d  # noqa: E402
from open3d.visualization import rendering  # noqa: E402

from rclpy.serialization import deserialize_message  # noqa: E402
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions  # noqa: E402
from sensor_msgs.msg import LaserScan, PointCloud2  # noqa: E402

CLOUD_TOPIC = '/polka/merged_cloud'
SCAN_TOPIC = '/polka/merged_scan'
STAMP_TOL_NS = 60_000_000  # 60 ms

TILE = 1100               # px per tile (square)
BG = (0.07, 0.07, 0.08, 1.0)
POINT_SIZE = 2.6
MAX_POINTS = 130000       # stride-subsample above this (speed; 1100px can't resolve more)
ZCOLOR = (-1.5, 4.0)      # z range mapped to the colormap (output frame, os_sensor)
CMAP = cm.get_cmap('turbo')

# panel -> (layout, left_cfg, right_cfg, title, left_label, right_label)
#   layout "cc" = cloud|cloud, "cs" = cloud|scan
PANELS = {
    'fusion':         ('cc', 'single', 'merged', 'Multi-LiDAR fusion',
                       'Ouster only', 'Ouster + Avia + Mid-360 merged'),
    'filter_range':   ('cc', 'merged', 'filter_range', 'Output filter: range',
                       'merged', 'range 0.5-6 m'),
    'filter_angular': ('cc', 'merged', 'filter_angular', 'Output filter: angular',
                       'merged', 'keep +/-45 deg front'),
    'filter_box':     ('cc', 'merged', 'filter_box', 'Output filter: box',
                       'merged', '+/-3 m AABB'),
    'filter_height':  ('cc', 'merged', 'filter_height', 'Output filter: height cap',
                       'merged', 'z in [-0.6, 1.0] m'),
    'angular_invert': ('cc', 'invert_keep', 'invert_exclude', 'Angular filter: invert flag',
                       'keep +/-45 deg', 'exclude +/-45 deg'),
    'self_filter':    ('cc', 'merged', 'self_on', 'Self / footprint filter',
                       'no self-filter', 'chassis box excluded'),
    'voxel':          ('cc', 'merged', 'voxel_on', 'Voxel downsample',
                       'full density', 'voxel leaf 0.2 m'),
    'cpu_vs_cuda':    ('cc', 'cpu', 'merged', 'CPU vs CUDA pipeline',
                       'CPU path', 'CUDA path'),
    'dual_output':    ('cs', 'dual', 'dual', 'Dual output: cloud + scan',
                       'merged_cloud (3D)', 'merged_scan (2D top-down)'),
}


def _bag_dir(work, cfg):
    d = work / f'cap_{cfg}' / 'run'
    return d if d.is_dir() else work / f'cap_{cfg}'


def _reader(bag_dir):
    r = SequentialReader()
    r.open(StorageOptions(uri=str(bag_dir), storage_id='mcap'), ConverterOptions('', ''))
    return r


def pc2_xyz(msg):
    fields = {f.name: f for f in msg.fields}
    n = msg.width * msg.height
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)

    def col(name):
        f = fields[name]
        return np.frombuffer(raw[:, f.offset:f.offset + 4].tobytes(), dtype=np.float32)

    if not {'x', 'y', 'z'}.issubset(fields):
        return np.zeros((0, 3), np.float32)
    return np.stack([col('x'), col('y'), col('z')], axis=1)


def read_clouds(bag_dir):
    if not pathlib.Path(bag_dir).exists():
        return []
    r = _reader(bag_dir)
    out = []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == CLOUD_TOPIC:
            m = deserialize_message(data, PointCloud2)
            ns = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
            xyz = pc2_xyz(m)
            xyz = xyz[np.isfinite(xyz).all(axis=1)]
            out.append((ns, xyz))
    out.sort(key=lambda t: t[0])
    return out


def read_scans(bag_dir):
    r = _reader(bag_dir)
    out = []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == SCAN_TOPIC:
            m = deserialize_message(data, LaserScan)
            ns = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
            ranges = np.asarray(m.ranges, dtype=np.float32)
            ang = m.angle_min + np.arange(ranges.size) * m.angle_increment
            valid = np.isfinite(ranges) & (ranges > m.range_min) & (ranges < m.range_max)
            x = ranges[valid] * np.cos(ang[valid])
            y = ranges[valid] * np.sin(ang[valid])
            out.append((ns, np.stack([x, y], axis=1)))
    out.sort(key=lambda t: t[0])
    return out


def align(stream_a, stream_b):
    """Yield (msg_a, msg_b) pairs matched by stamp within tolerance."""
    if not stream_a or not stream_b:
        return
    b_ns = np.array([t[0] for t in stream_b])
    for ns, a in stream_a:
        j = int(np.searchsorted(b_ns, ns))
        best, best_d = None, STAMP_TOL_NS + 1
        for k in (j - 1, j):
            if 0 <= k < len(b_ns):
                d = abs(int(b_ns[k]) - ns)
                if d < best_d:
                    best_d, best = d, k
        if best is not None and best_d <= STAMP_TOL_NS:
            yield a, stream_b[best][1]


class Renderer:

    def __init__(self, eye, center, up, fov=55.0):
        self.r = rendering.OffscreenRenderer(TILE, TILE)
        self.r.scene.set_background(list(BG))
        self.r.scene.scene.set_sun_light([0.3, 0.3, -1.0], [1, 1, 1], 60000)
        self.mat = rendering.MaterialRecord()
        self.mat.shader = 'defaultUnlit'
        self.mat.point_size = POINT_SIZE
        self.eye, self.center, self.up, self.fov = eye, center, up, fov

    def cloud_img(self, xyz):
        self.r.scene.clear_geometry()
        if xyz.shape[0] > MAX_POINTS:  # deterministic stride keeps density uniform/stable
            xyz = xyz[::(xyz.shape[0] // MAX_POINTS + 1)]
        if xyz.shape[0] > 0:
            pc = o3d.geometry.PointCloud()
            pc.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
            t = np.clip((xyz[:, 2] - ZCOLOR[0]) / (ZCOLOR[1] - ZCOLOR[0]), 0, 1)
            pc.colors = o3d.utility.Vector3dVector(CMAP(t)[:, :3])
            self.r.scene.add_geometry('pc', pc, self.mat)
        self.r.setup_camera(self.fov,
                            np.asarray(self.center, np.float32),
                            np.asarray(self.eye, np.float32),
                            np.asarray(self.up, np.float32))
        return np.asarray(self.r.render_to_image())


def scan_img(xy, lim=12.0):
    """Top-down 2D scan render via matplotlib, returned as an RGB array."""
    fig = plt.figure(figsize=(TILE / 100, TILE / 100), dpi=100, facecolor=BG[:3])
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_facecolor(BG[:3])
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect('equal')
    ax.axis('off')
    if xy.shape[0] > 0:
        rr = np.hypot(xy[:, 0], xy[:, 1])
        ax.scatter(xy[:, 0], xy[:, 1], c=rr, cmap='turbo', s=6, linewidths=0)
    ax.plot(0, 0, marker='^', color='white', markersize=16)
    fig.canvas.draw()
    w, h = fig.canvas.get_width_height()
    buf = np.frombuffer(fig.canvas.buffer_rgba(), np.uint8).reshape(h, w, 4)[:, :, :3].copy()
    plt.close(fig)
    return buf


def compose(imgL, imgR, title, labelL, labelR, footL, footR, out_png):
    fig = plt.figure(figsize=(19.2, 10.0), dpi=100, facecolor=BG[:3])
    fig.suptitle(title, color='white', fontsize=30, family='monospace',
                 weight='bold', y=0.965)
    for i, (img, lab, foot) in enumerate([(imgL, labelL, footL), (imgR, labelR, footR)]):
        ax = fig.add_subplot(1, 2, i + 1)
        ax.imshow(img)
        ax.axis('off')
        ax.text(0.03, 0.95, lab, transform=ax.transAxes, color='white',
                fontsize=20, family='monospace', weight='bold', va='top')
        ax.text(0.03, 0.04, foot, transform=ax.transAxes, color='#bbb',
                fontsize=15, family='monospace')
    fig.text(0.99, 0.015, 'polka', color='#666', fontsize=15, family='monospace',
             ha='right', weight='bold')
    fig.subplots_adjust(left=0.005, right=0.995, top=0.92, bottom=0.01, wspace=0.01)
    fig.savefig(out_png, facecolor=BG[:3])
    plt.close(fig)


def compute_camera(work):
    """Pick a fixed 3/4 view from the merged cloud's horizontal extent."""
    clouds = read_clouds(_bag_dir(work, 'merged'))
    pts = np.concatenate([c for _, c in clouds[:5]], axis=0) if clouds else np.zeros((1, 3))
    lo = np.percentile(pts, 2, axis=0)
    hi = np.percentile(pts, 98, axis=0)
    center = (lo + hi) / 2.0
    center[2] = np.clip(center[2], -0.5, 1.0)
    span = float(np.linalg.norm(hi[:2] - lo[:2])) or 10.0
    d = span * 1.1
    eye = center + np.array([-d * 0.75, -d * 0.55, d * 0.55])
    return eye.tolist(), center.tolist(), [0, 0, 1]


def render_panel(name, work, out, renderer):
    layout, lcfg, rcfg, title, llab, rlab = PANELS[name]
    out_dir = out / name
    out_dir.mkdir(parents=True, exist_ok=True)
    for old in out_dir.glob('f_*.png'):  # clear stale frames from a previous run
        old.unlink()

    if layout == 'cc':
        L = read_clouds(_bag_dir(work, lcfg))
        R = read_clouds(_bag_dir(work, rcfg))
        pairs = list(align(L, R))
        print(f'[{name}] {lcfg}={len(L)} {rcfg}={len(R)} aligned={len(pairs)}', flush=True)
        for i, (a, b) in enumerate(pairs):
            imgL = renderer.cloud_img(a)
            imgR = renderer.cloud_img(b)
            compose(imgL, imgR, title, llab, rlab,
                    f'{a.shape[0]:,} pts', f'{b.shape[0]:,} pts',
                    out_dir / f'f_{i:04d}.png')
        return len(pairs)

    # cloud | scan from the same dual capture
    C = read_clouds(_bag_dir(work, lcfg))
    S = read_scans(_bag_dir(work, rcfg))
    pairs = list(align(C, S))
    print(f'[{name}] cloud={len(C)} scan={len(S)} aligned={len(pairs)}', flush=True)
    for i, (c, s) in enumerate(pairs):
        imgL = renderer.cloud_img(c)
        imgR = scan_img(s)
        compose(imgL, imgR, title, llab, rlab,
                f'{c.shape[0]:,} pts', f'{s.shape[0]:,} beams',
                out_dir / f'f_{i:04d}.png')
    return len(pairs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--work', type=pathlib.Path,
                    default=pathlib.Path(__file__).parent / 'work')
    ap.add_argument('--out', type=pathlib.Path,
                    default=pathlib.Path(__file__).parent / 'frames')
    ap.add_argument('--only', default=None)
    args = ap.parse_args()

    eye, center, up = compute_camera(args.work)
    eye_r = [round(v, 2) for v in eye]
    center_r = [round(v, 2) for v in center]
    print(f'camera eye={eye_r} center={center_r}', flush=True)
    renderer = Renderer(eye, center, up)

    names = [args.only] if args.only else list(PANELS.keys())
    for name in names:
        render_panel(name, args.work, args.out, renderer)


if __name__ == '__main__':
    main()
