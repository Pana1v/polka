# Polka feature-demo GIFs — design

- **Date:** 2026-06-28
- **Branch:** `panav/viz/feature-demo-gifs` (off `origin/humble`)
- **Status:** Approved design — pending implementation plan

## 1. Goal

Produce **separate, high-quality animated GIFs, one per Polka feature**, generated
from the TIERS multi-LiDAR `Calibration.bag` running the latest **Humble** Polka.

Each GIF runs **two or more Polka instances with different configs side-by-side in a
divided panel** (the "multiple instances, one functionality each" idea), rendered with
**Open3D offscreen** and encoded with **gifski**.

This replaces the existing single low-quality montage (`media/pipeline_demo.gif`:
560 px / 8 fps / 48 colors / matplotlib scatter) with per-feature, visually clean GIFs.

## 2. Input data

TIERS Multi-LiDAR Multi-UAV dataset — <https://github.com/TIERS/multi_lidar_multi_uav_dataset>.

`~/Downloads/Calibration.bag` — ROS 1 bag, 1.32 GB, **9.3 s**, 2321 messages.

| Topic | Type | Rate | Frame ID | Use |
|---|---|---|---|---|
| `/ouster/points` | `sensor_msgs/PointCloud2` | 20 Hz | `os_sensor` | **base reference**, direct |
| `/avia/livox/lidar` | `livox_ros_driver/CustomMsg` | 100 Hz | `avia_frame` | convert → PointCloud2 |
| `/mid360/livox/lidar` | `livox_ros_driver2/CustomMsg` | 100 Hz | `mid360_frame` | convert → PointCloud2 |
| `/camera/depth/color/points` | `sensor_msgs/PointCloud2` | 30 Hz | `camera_depth_optical_frame` | direct (not in fusion) |

No IMU, no LaserScan, no TF in the bag. OS1 (Ouster) is the calibration base frame.

### Extrinsics (sensor frame → `os_sensor`, `[x y z roll pitch yaw]`, from dataset README)

| Sensor | Frame | Extrinsic |
|---|---|---|
| Livox Avia | `avia_frame` | `0.149354 0.0423582 -0.0524961  3.13419 -3.13908 -3.13281` |
| Livox Mid-360 | `mid360_frame` | `0.125546 -0.0554536 -0.20206  0.00467344 0.0270294 0.0494959` |
| RealSense D435 | `camera_depth_optical_frame` | `-0.172863 0.11895 -0.101785  1.55222 3.11188 1.60982` |
| Ouster OS1-64 | `os_sensor` | identity (base) |

## 3. Locked decisions

| Decision | Choice | Rationale |
|---|---|---|
| Renderer | **Open3D offscreen** | GPU-accurate, anti-aliased, full density, scriptable/headless. Verified working on RTX 2050 via EGL/Filament/OpenGL 4.6. |
| Fusion scope | **3 LiDARs** (Ouster + Avia + Mid-360) | Clean, intuitive multi-LiDAR story. RealSense excluded (noisier, narrow FoV). |
| Livox ingest | **Offline bag rewriter** | Avoids building two Livox driver msg packages + a live converter. |
| Features | **All** (fusion, 4 separate filter GIFs, angular-invert, self-filter, voxel, CPU-vs-CUDA + perf, dual-output) | — |
| Filter GIFs | **Separate GIF per filter** | Matches "one feature each" literally. |
| Encoder | **gifski**, installed to `~/.local/bin` via `dpkg-deb -x` (no sudo) | Best GIF quality; consumes PNG frames directly, no ffmpeg needed for GIFs. |
| Branch | **New branch off `origin/humble`** | Live branch (CI, 5-distro support). |

Out of scope: IMU-based deskew / motion compensation (no IMU in bag; bag is
near-stationary calibration data). Heterogeneous 2D+3D fusion (no LaserScan source).

## 4. Pipeline

```
Calibration.bag (ROS1, 1.32 GB, 9.3 s)
  └─[A] rosbags-convert ──────────────▶ calibration_ros2/ (mcap)
  └─[B] livox_to_pc2.py (offline) ─────▶ /avia/points, /mid360/points as PointCloud2
  └─[C] static_transform_publisher ────▶ avia/mid360 → os_sensor (README extrinsics)
[D] build Polka @ origin/humble, CPU + CUDA (WITH_CUDA=ON default; RTX 2050 = sm_86)
[E] run_capture.py: per feature-variant, launch polka + replay bag → output mcap + metrics.json
[F] render_o3d.py (NEW): load per-variant clouds, align by header.stamp, render divided-panel PNGs
[G] make_gifs.sh: gifski PNG frames → one .gif per feature
```

## 5. Components

### New
- **`media/livox_to_pc2.py`** — offline ROS 2 bag rewriter. Reads `CustomMsg`
  (registered from the bag's embedded message definitions), emits `PointCloud2`
  with fields `x,y,z,intensity` (intensity = reflectivity), preserving `header.stamp`
  and `frame_id`. New topics `/avia/points`, `/mid360/points`.
- **`media/render_o3d.py`** — replaces matplotlib `render_panels.py`. Open3D
  `rendering.OffscreenRenderer`: shared camera per panel, dark studio background,
  height/intensity colormap, full point density, MSAA. Composites N tiles into one
  divided-panel PNG per aligned frame. Multiprocessing over frames.
- **`media/make_gifs.sh`** — installs gifski to `~/.local/bin` (extract `.deb` with
  `dpkg-deb -x`, no sudo), then encodes each feature's PNG sequence into a separate
  `media/gifs/<feature>.gif` (palette-optimized, ~512–720 px, 15–20 fps).

### Ported from `panav/viz/demo-generator` (re-pointed at real topics)
- **`run_capture.py`** — launches polka subprocess, finds `polka_node` PID, samples
  CPU%/RSS (psutil) and GPU% (pynvml if present), subscribes to `/polka/merged_cloud`,
  replays bag with `qos_override.yaml`, writes `<out>/run/*.mcap` + `metrics.json`.
- **`qos_override.yaml`** — latches `/tf_static` for late-joining subscribers.
- **Config templates** — adapted to real topics: sources `/ouster/points`,
  `/avia/points`, `/mid360/points`; `output_frame_id: os_sensor`.

### Removed / not ported
- matplotlib `render_panels.py`, `compose.sh` montage, `synthetic_skew_publisher.py`
  (deskew out of scope), `render_title.py`, `generate.sh` montage orchestration.

## 6. Deliverables — the GIFs (each a separate divided-panel file in `media/gifs/`)

| File | Panels | Shows |
|---|---|---|
| `fusion.gif` | Ouster-only ▸ 3-LiDAR merged | the headline merge |
| `filter_range.gif` | merged ▸ range-filtered | spherical distance crop |
| `filter_angular.gif` | merged ▸ angular sector | yaw-sector keep |
| `filter_box.gif` | merged ▸ box-cropped | AABB crop |
| `filter_height.gif` | merged ▸ height-capped | z-clip |
| `angular_invert.gif` | keep ±45° ▸ exclude ±45° | the `invert` flag |
| `self_filter.gif` | no self-filter ▸ chassis box excluded | ego-body exclusion |
| `voxel.gif` | full density ▸ voxel-downsampled | with live point counts |
| `cpu_vs_cuda.gif` + `perf.png` | CPU path ▸ CUDA path; bar chart | engine parity + perf |
| `dual_output.gif` | merged_cloud (3D) + merged_scan (2D top-down) | dual output |

## 7. Risks & error handling

- **No TF in bag** → publish static transforms from README extrinsics (launch with
  `static_transform_publisher` per sensor, parent `os_sensor`).
- **Two distinct Livox custom types** (`livox_ros_driver` vs `livox_ros_driver2`) →
  handled by the offline rewriter using each connection's embedded msgdef; no driver
  package builds.
- **9.3 s clip** → short; GIFs use the full clip (loop in viewer). Capture duration
  capped at bag length.
- **gifski not in apt, no cargo, no sudo** → prebuilt binary extracted to
  `~/.local/bin`. GIFs need only gifski (renderer emits PNGs). ffmpeg (apt, needs
  password) only if MP4s are also wanted — not a core deliverable.
- **Open3D EGL device binding on hybrid GPU** → verified rendering a real frame
  headlessly; if a future run produces black frames, force the NVIDIA EGL device via
  `__EGL_VENDOR_LIBRARY_FILENAMES` / `__NV_PRIME_RENDER_OFFLOAD`.
- **Camera intrinsics consistency across tiles** → `render_o3d.py` uses one fixed
  camera pose/intrinsic for all tiles in a panel so side-by-side comparison is honest.

## 8. Validation

- **Smoke:** `livox_to_pc2.py` on a 1 s slice → assert non-empty `PointCloud2`, correct
  `frame_id`, plausible point counts vs `CustomMsg.point_num`.
- **Fusion alignment:** with static TFs applied, the 3 merged clouds visually coincide
  on shared structure (walls/targets) — the calibration's own success criterion.
- **Per-feature sanity:** each capture's output point count moves in the expected
  direction (filters reduce; voxel reduces; self-filter removes the chassis box; CPU
  and CUDA outputs match within tolerance).
- **Render:** every panel produces ≥1 non-black PNG; frame count > 0 after stamp
  alignment.
- **Encode:** each `media/gifs/*.gif` exists, non-trivial size, opens.

## 9. Deliverable layout

```
media/
├── configs/            # per-feature-variant polka YAMLs (real topics, os_sensor)
├── livox_to_pc2.py     # NEW offline CustomMsg→PointCloud2 rewriter
├── render_o3d.py       # NEW Open3D offscreen divided-panel renderer
├── run_capture.py      # ported, re-pointed
├── make_gifs.sh        # NEW gifski driver (+ userspace install)
├── qos_override.yaml   # ported
├── launch/tf_static.launch.py  # NEW static extrinsics
└── gifs/               # OUTPUT: one .gif per feature
```

## 10. Open items / dependencies (resolved at plan time)

- Confirm Livox `CustomMsg` field layout from the bag's embedded msgdef (offset_time,
  reflectivity, tag, line) before writing `livox_to_pc2.py`.
- Decide capture frame rate / point budget per tile for render performance.
- gifski release URL/version for the userspace install.
