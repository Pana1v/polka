# Demo media

Per-feature demo GIFs in [`gifs/`](gifs/) and the headless toolchain that builds
them. Source data is the [TIERS multi-LiDAR dataset](https://github.com/TIERS/multi_lidar_multi_uav_dataset)
(Ouster OS1 + Livox Avia + Mid-360); clouds render with Open3D offscreen, panels
compose with matplotlib, and GIFs encode with [gifski](https://gif.ski/).

`work/` and `frames/` are generated intermediates and are gitignored.

## Pipeline

```
Calibration.bag (ROS 1)
  → prepare_bag.py        ROS 1 → ROS 2 mcap; Livox CustomMsg → PointCloud2;
                          sliding-window accumulation densifies the solid-state scans
  → gen_configs.py        emit one polka config per feature variant into configs/
  → run_capture.py        launch polka + static TFs, replay bag, record output mcap (+ metrics)
                          (capture_all.py runs every config; demo_bringup.launch.py adds the TFs)
  → render_o3d.py         Open3D offscreen → divided-panel PNG frames per feature
    render_scan.py        2D LaserScan merge panel (individual vs merged, by winning sensor)
  → make_gifs.sh          gifski → one gifs/<feature>.gif per panel
```

## Regenerate

```bash
# 1. convert the ROS 1 bag (edit --in / --out if needed)
python3 prepare_bag.py --in ~/Downloads/Calibration.bag --out ~/ros2_ws/bags/calibration_ros2
# 2. configs + captures (each config is isolated by ROS_DOMAIN_ID)
python3 gen_configs.py
python3 capture_all.py
# 3. render + encode
python3 render_o3d.py --work work --out frames
python3 render_scan.py
bash make_gifs.sh
```

Extrinsics (sensor → `os_sensor` base frame) live in `demo_bringup.launch.py`,
taken from the dataset README. The `cpu_vs_cuda` panel needs a working CUDA
runtime; without it polka falls back to CPU and that panel is skipped.
