# Polka pipeline media

This directory holds the scripts that generate the project's hero `pipeline_demo.gif` / `.mp4` from a recorded `polka_test.mcap` bag. The branch [`panav/viz/demo-generator`](https://github.com/Pana1v/polka/tree/panav/viz/demo-generator) preserves them; they are not part of the runtime package.

> **Note on the `random_grid` and `A_*` (deskew) configs:** Panels D2 and A reference a `random_grid` output downsampler and a `synthetic_skew_publisher.py`-fed deskew path that were used during development but are not yet merged into `humble`/`jazzy`. To regenerate the GIF on a stock polka, either pin to the commit of polka that the demo was captured against, or skip those configs.

`pipeline_demo.gif` / `pipeline_demo.mp4` showcase polka's pipeline as 7 side-by-side visual panels followed by a CUDA-vs-CPU performance bar chart, opened by a title slide.

## Panels

| # | Panel | Layout | What it shows |
|---|---|---|---|
| title | static slide | wordmark + feature checklist (2.5 s hold) |
| A | deskew | 1×2 side-by-side | raw cloud (1 rad/s yaw smear) vs polka's per-point SE(3) correction. Driven by `synthetic_skew_publisher.py` because no available bag has both per-point timestamps and visible motion. |
| B | fusion | 1×2 side-by-side | single front lidar vs both lidars merged |
| C | filters | 2×2 grid | range / angular / box / height_cap — each applied alone |
| C2 | invert flag | 1×2 side-by-side | angular filter `invert=false` (keep ±45°) vs `invert=true` (exclude ±45°) |
| C3 | self-filter | 1×2 side-by-side | no self-filter vs chassis-box ego-body exclusion |
| D | downsample | 1×2 side-by-side | voxel (centroid) vs random_grid (real points), leaf=0.25m |
| F | dual outputs | 1×2 (3D + 2D top-down) | merged_cloud (3D) alongside a bird's-eye projection illustrating the LaserScan output |
| G | CUDA pipeline | 1×2 side-by-side | same config with `enable_gpu: true` (CUDA path) vs `false` (CPU path), animated live |
| E | perf | bar chart | CUDA vs CPU on msg rate, CPU%, RAM, GPU% (3 reps each, 5 s hold) |

### Panel A — synthetic skew demo

[`synthetic_skew_publisher.py`](synthetic_skew_publisher.py) reads `/lidar/front/rslidar_points` from `polka_test.mcap`, rotates each point's XY by `omega × (t_point - t_scan_start)` (default `omega=1.0` rad/s) using the per-point `timestamp` field, and republishes on `/synthetic/lidar/points`. A matching `/synthetic/imu` at `angular_velocity.z = omega` is published alongside. Polka subscribes to both: with `motion_compensation.enabled=false` (A_raw) the output keeps the smear; with `motion_compensation.enabled=true` (A_deskewed) polka's per-point SE(3) correction reverses it. Visual difference is subtle at the near-field crop because 1 rad/s × 0.1 s scan = ~5.7° of yaw, smaller than the demo's azimuth resolution; the **mechanism** is what's being shown.

## Regenerate

```bash
bash media/generate.sh
```

Environment overrides:
- `POLKA_DEMO_BAG=/path/to/bag.mcap` — default `bags/polka_test/polka_test.mcap`
- `REC_DURATION=12` — seconds of capture per visual stage
- `PERF_DURATION=15` — seconds of capture per perf run
- `PERF_REPEATS=3` — repeats per CUDA / CPU run

Re-runs reuse captured bags if they already exist; delete `media/work/<stage>/` to force recapture.

## How it works

1. `configs/generate_panels.py` emits one YAML per panel tile by mutating `_base.yaml`.
2. `run_capture.py` for each config:
   - launches polka (subprocess)
   - finds the polka_node child PID
   - spawns a psutil sampler thread (CPU% / RSS, 1 Hz)
   - spawns a pynvml sampler thread (GPU%, 1 Hz; skipped if pynvml missing)
   - subscribes to `/polka/merged_cloud` with `use_sim_time` matching polka, computing per-message latency as `now - header.stamp`
   - plays the input bag with `qos_override.yaml` so `/tf_static` arrives
   - writes `<out>/run/*.mcap` + `<out>/metrics.json`
3. `render_panels.py`:
   - loads each tile's bag → list of `(header.stamp_ns, PointCloud2)` tuples
   - **aligns tiles by `header.stamp` with a 50 ms tolerance** (independent polka runs don't have matching enumeration indices)
   - renders one composite PNG per aligned frame group, parallel via `multiprocessing.Pool`
4. `render_perf.py` aggregates `metrics.json` from CUDA + CPU runs (3 reps each) into a single bar chart PNG with mean ± std error bars.
5. `compose.sh` symlinks per-panel frames into a single contiguous stream (with a 0.5 s hold at each panel's end and 5 s on perf), then ffmpeg → MP4 + GIF.

## Caveats on the performance numbers

- **Latency is end-to-end on bag replay**, not pure polka compute. The bag player's buffering adds delay outside of polka's control. The relative CUDA-vs-CPU comparison is still meaningful — both paths share the same overhead.
- **CPU%** is the polka_node process's percentage from `psutil.cpu_percent()`, summed across threads, normalized to 100 % per core. Higher = more CPU work.
- **RAM** is RSS in MB. CUDA always has higher RSS due to GPU buffer allocations.
- **GPU%** uses `pynvml.nvmlDeviceGetUtilizationRates(handle).gpu`, a 1-second rolling average. The merge engine is small enough that GPU utilization is in the low single digits.
- Each run drops the first ~3 s of samples to skip CUDA initialization / cuBLAS warmup.

## Stage configs

`configs/_base.yaml` is the shared base. `configs/generate_panels.py` mutates it to produce:

| File | Used in |
|---|---|
| `A_raw.yaml`, `A_deskewed.yaml` | Panel A (deskew, fed by `synthetic_skew_publisher.py`) |
| `B_single.yaml`, `B_merged.yaml` | Panel B |
| `C_range.yaml`, `C_angular.yaml`, `C_box.yaml`, `C_height.yaml` | Panel C |
| `C2_keep_front.yaml`, `C2_exclude_front.yaml` | Panel C2 (invert flag) |
| `C3_no_self.yaml`, `C3_self.yaml` | Panel C3 (self-filter) |
| `D_voxel.yaml`, `D_random_grid.yaml` | Panel D |
| `F_dual.yaml` | Panel F (cloud + scan visualization) |
| `G_cuda_live.yaml`, `G_cpu_live.yaml` | Panel G (live CUDA vs CPU) |
| `E_cuda.yaml`, `E_cpu.yaml` | Panel E (each × `PERF_REPEATS`) |

Edit `_base.yaml` and re-run `generate.sh` to apply globally.
