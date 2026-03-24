# POLKA

<p align="center">
  <img src="images/polka.png" alt="Polka" width="300"/>
</p>

**Multi-LiDAR fusion node for ROS 2** — merges any mix of PointCloud2 and LaserScan sources into unified output, with optional CUDA GPU acceleration.

Replaces multi-node pipelines (`relay → filter → transform → merge → downsample`) with a single composable node.

## Features

- **Heterogeneous source fusion**: mix PointCloud2 and LaserScan sensors freely
- **Dual output**: publish merged PointCloud2, LaserScan, or both
- **Per-source filtering**: range, angular, box filters applied before merge
- **Output filtering**: range, angular, box, height, footprint (ego-body exclusion), voxel downsample — applied in fixed order after merge
- **IMU deskewing**: per-point SE(3) motion correction using IMU angular velocity and acceleration; auto-detects per-point timestamp fields (`time`, `t`, `timestamp`, etc.)
- **CUDA acceleration**: optional GPU merge engine with fused kernels and pre-allocated buffers
- **TF2 fallback**: automatic transform lookup; falls back to last known good on dropout
- **Composable node**: standalone or loaded into a component container
- **Fully parameterized**: every feature runtime-configurable via ROS 2 parameters

## Dependencies

| Package | Purpose |
|---|---|
| `rclcpp` / `rclcpp_components` | ROS 2 node framework |
| `sensor_msgs` | PointCloud2, LaserScan, Imu |
| `tf2_ros` / `tf2_eigen` | Frame transforms |
| `pcl_conversions` | PCL ↔ ROS message conversion |
| `laser_geometry` | LaserScan → PointCloud2 projection |
| CUDA toolkit | **Optional** — GPU merge engine only |

## Build

```bash
# CPU only
colcon build --packages-select polka

# With CUDA
colcon build --packages-select polka --cmake-args -DPOLKA_ENABLE_CUDA=ON
```

## Quick Start

```bash
cp config/example_params.yaml config/my_robot.yaml
# Edit: set output_frame_id, list sensors under source_names, configure per-source topics/filters
ros2 launch polka polka.launch.py params_file:=config/my_robot.yaml
```

Ensure TF is published from each sensor's `frame_id` to your `output_frame_id`.

## Configuration

All parameters live under the `polka` namespace. See [config/example_params.yaml](config/example_params.yaml) for the full annotated reference.

### Key Parameters

| Parameter | Default | Description |
|---|---|---|
| `output_frame_id` | `"base_link"` | Target frame for merged output |
| `output_rate` | `20.0` | Merge + publish rate (Hz) |
| `source_timeout` | `0.5` | Drop source if no data within this window (s) |
| `timestamp_strategy` | `"earliest"` | Output stamp: `earliest`, `latest`, `average`, or `local` |

### Motion Compensation (IMU Deskewing)

Per-point deskewing uses the SE(3) exponential map with constant-acceleration + constant-angular-velocity, applied per point based on its timestamp. Inter-source alignment corrects timing offsets between sensors.

Motion model inspired by [rko_lio](https://github.com/TixiaoShan/rko_lio) (Malladi et al., 2025).

```yaml
motion_compensation:
  enabled: true
  imu_topic: "/imu/data"
  max_imu_age: 0.2
  imu_buffer_size: 200            # ring buffer (~1s at 200Hz)
  per_point_deskew: true
  deskew_timestamp_field: "auto"  # auto-detects 'time', 't', 'timestamp', etc.
```

### Output Filters

Applied after merge in this order: output filters (range/angular/box) → footprint filter → height filter → voxel downsample.

```yaml
outputs:
  cloud:
    height_filter:
      enabled: true
      z_min: -1.0
      z_max: 3.0
    voxel:
      enabled: true
      leaf_size: 0.05
    footprint_filter:
      enabled: true
      box_names: ["chassis"]
      chassis:
        x_min: -0.30
        x_max:  0.30
        y_min: -0.25
        y_max:  0.25
        z_min: -0.10
        z_max:  0.50
```

## Pipeline Comparison

### polka (1 node)

```mermaid
graph LR
    subgraph Drivers
        D1[lidar driver · front]
        D2[odom / cmd_vel]
        D3[lidar driver · back]
    end

    P[<strong>polka</strong>]

    subgraph Consumers
        C1[mapping / reconstruction<br/>~/merged_cloud]
        C2[localization / navigation<br/>~/merged_scan]
    end

    D1 --> P
    D2 -.-> P
    D3 --> P
    P --> C1
    P --> C2
```

### pcl_ros chain (7+ nodes)

Cloud path:

```mermaid
graph LR
    subgraph Drivers
        D1[lidar driver · front]
        D2[lidar driver · back]
    end

    CAT[pcl_ros::<br/>ConcatenatePointCloud<br/>+ ApproxTimeSynchronizer]
    CF[custom node<br/>cloud filters]
    MAP[mapping node]

    D1 --> CAT
    D2 --> CAT
    CAT --> CF -->|merged_cloud| MAP
```

Scan path:

```mermaid
graph LR
    subgraph Drivers
        D1[lidar driver · front]
        D2[lidar driver · back]
    end

    P2L1[pointcloud_to_laserscan<br/>· front]
    P2L2[pointcloud_to_laserscan<br/>· back]
    IRA[ira_laser_tools::<br/>LaserscanMerger]
    SF[custom node<br/>scan filters]
    NAV[localization / navigation]

    D1 --> P2L1
    D2 --> P2L2
    P2L1 --> IRA
    P2L2 --> IRA
    IRA --> SF -->|merged_scan| NAV
```

## Architecture

```mermaid
graph LR
    subgraph Sources
        PC[PointCloud2<br/>/front/points]
        LS[LaserScan<br/>/rear/scan]
    end

    subgraph Per-Source Filters
        PF1[Range / Angular /<br/>Box Filter]
        PF2[Range / Angular /<br/>Box Filter]
    end

    subgraph Merge Engine
        ME[CPU or CUDA<br/>Merge]
    end

    subgraph Output Pipeline
        OF[Range / Angular /<br/>Box Filter]
        FF[Footprint Filter]
        HF[Height Filter]
        VX[Voxel Downsample]
    end

    PC --> PF1 --> ME
    LS --> PF2 --> ME
    ME --> OF --> FF --> HF --> VX
    VX --> OUT_PC[PointCloud2]
    VX --> OUT_LS[LaserScan]
```

## File Structure

```
polka/
├── config/example_params.yaml
├── launch/polka.launch.py
├── include/polka/
│   ├── polka_node.hpp              # Main composable node
│   ├── types.hpp                   # Config structs and type definitions
│   ├── config_loader.hpp           # Parameter loading and hot-reload
│   ├── source_adapter.hpp          # Sensor subscription and conversion
│   ├── filters/
│   │   ├── i_filter.hpp            # Filter interface
│   │   ├── range_filter.hpp
│   │   ├── angular_filter.hpp
│   │   └── box_filter.hpp          # Also used inverted for footprint filter
│   └── merge_engine/
│       ├── i_merge_engine.hpp      # Merge engine interface
│       ├── cpu_merge_engine.hpp
│       ├── cuda_merge_engine.hpp
│       └── cuda_types.cuh
└── src/
    ├── main.cpp
    ├── polka_node.cpp
    ├── config_loader.cpp
    ├── source_adapter.cpp
    ├── filters/
    └── merge_engine/
```

## Acknowledgments

Per-point deskewing motion model inspired by rko_lio:

```bibtex
@article{malladi2025arxiv,
  author  = {M.V.R. Malladi and T. Guadagnino and L. Lobefaro and C. Stachniss},
  title   = {A Robust Approach for LiDAR-Inertial Odometry Without Sensor-Specific Modeling},
  journal = {arXiv preprint},
  year    = {2025},
  volume  = {arXiv:2509.06593},
  url     = {https://arxiv.org/pdf/2509.06593},
}
```

## License

Apache-2.0
