# POLKA

<p align="center">
  <img src="images/polka.png" alt="Polka" width="300"/>
</p>

**Multi-LiDAR fusion node for ROS 2** that merges any mix of PointCloud2 and LaserScan sources into a unified output, with optional CUDA GPU acceleration.

Polka replaces multi-node pipelines (relay -> filter -> transform -> merge -> downsample) with a single composable node, dramatically reducing latency, CPU overhead, and configuration complexity.

## Why Polka?

Managing multiple LiDAR sensors in ROS 2 typically requires a chain of separate nodes, each adding overhead, latency, and failure points. Polka collapses this entire pipeline into one composable node:

- **Deep per-source filtering**: every sensor gets its own range, angular, and box filter pass before any data enters the merge stage, so you never waste bandwidth merging garbage
- **Multi-modal merging**: fuse 3D PointCloud2 and 2D LaserScan sources together in a single merge step, no separate projection or relay nodes needed
- **Unified output**: emit merged PointCloud2, LaserScan, or both simultaneously from a single node
- **Rich output filtering**: after merge, apply range, angular, box, height filter, footprint filter (ego-body exclusion), and voxel downsampling in a defined, consistent order
- **CUDA GPU acceleration**: the merge engine can run entirely on GPU with fused kernels and pre-allocated buffers, cutting merge latency significantly on sensor-dense platforms
- **IMU-based deskewing**: per-point motion correction using the SE(3) exponential map removes intra-scan distortion, plus inter-source alignment eliminates ghosting artifacts during robot motion
- **TF2 integration**: transforms are resolved automatically, with fallback to the last known good transform so a momentary TF dropout does not drop the entire output

## Features

- **Heterogeneous source fusion**: mix 3D PointCloud2 and 2D LaserScan sensors freely
- **Dual output**: publish merged PointCloud2, LaserScan, or both simultaneously
- **Per-source filtering**: range, angular, and box filters applied before merge
- **Output filtering**: range, angular, box, height filter, footprint filter (ego-body exclusion), voxel downsampling
- **IMU-based deskewing**: per-point SE(3) motion correction using IMU angular velocity and acceleration, with auto-detection of per-point timestamp fields
- **CUDA acceleration**: optional GPU merge engine with fused kernels and pre-allocated buffers
- **TF2 integration**: automatic transform lookup with fallback to last known good transform
- **Fully parameterized**: every feature is runtime-configurable via ROS 2 parameters
- **Composable node**: runs standalone or loaded into a component container

## Dependencies

| Package | Purpose |
|---|---|
| `rclcpp` / `rclcpp_components` | ROS 2 node framework |
| `sensor_msgs` | PointCloud2, LaserScan messages |
| `sensor_msgs` (Imu) | IMU data for motion compensation / deskewing |
| `tf2_ros` / `tf2_eigen` | Frame transforms |
| `pcl_conversions` | PCL <-> ROS message conversion |
| `laser_geometry` | LaserScan -> PointCloud2 projection |
| CUDA toolkit | **Optional** -- only needed for GPU merge engine |

## Build

```bash
# CPU only
cd ~/ros2_ws
colcon build --packages-select polka

# With CUDA support
colcon build --packages-select polka --cmake-args -DPOLKA_ENABLE_CUDA=ON
```

## Quick Start

1. Copy and edit the example config:
   ```bash
   cp config/example_params.yaml config/my_robot.yaml
   ```

2. Set `output_frame_id` to your robot's base frame (e.g. `base_link`)

3. List your sensors under `source_names` and configure each source's topic, type, and filters

4. Ensure TF is published from each sensor's `frame_id` to `output_frame_id`

5. Launch:
   ```bash
   ros2 launch polka polka.launch.py params_file:=config/my_robot.yaml
   ```

## Configuration

All parameters live under the `polka` namespace. See [config/example_params.yaml](config/example_params.yaml) for the full annotated reference.

### Key Parameters

| Parameter | Default | Description |
|---|---|---|
| `output_frame_id` | `"base_link"` | Target frame for all merged output |
| `output_rate` | `20.0` | Merge + publish rate (Hz) |
| `source_timeout` | `0.5` | Drop source if no data within this window (s) |
| `timestamp_strategy` | `"earliest"` | Output stamp: `earliest`, `latest`, `average`, or `local` |

### Motion Compensation (IMU Deskewing)

Corrects for robot motion during LiDAR scans using IMU data. Per-point deskewing uses the SE(3) exponential map motion model with angular velocity and linear acceleration from IMU, applied to each point based on its per-point timestamp. Inter-source alignment corrects for timing offsets between different sensors.

The motion model is inspired by [rko_lio](https://github.com/TixiaoShan/rko_lio) (Malladi et al., 2025).

```yaml
motion_compensation:
  enabled: true
  imu_topic: "/imu/data"          # sensor_msgs/Imu topic
  max_imu_age: 0.2                # seconds - reject stale IMU
  imu_buffer_size: 200            # ring buffer (~1s at 200Hz)
  per_point_deskew: true          # per-point correction within each scan
  deskew_timestamp_field: "auto"  # auto-detects 'time', 't', 'timestamp', etc.
```

### Output Filters

Applied to the merged cloud before publishing, in this order:

1. **Output filters** (range / angular / box)
2. **Footprint filter** -- removes points inside robot body exclusion zones
3. **Height filter** -- clips to `[z_min, z_max]`
4. **Voxel downsample** -- reduces density via VoxelGrid

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
├── config/example_params.yaml      # Full annotated config reference
├── images/polka.png                # Project image
├── launch/polka.launch.py          # Launch file
├── include/polka/
│   ├── polka_node.hpp              # Main composable node
│   ├── types.hpp                   # Config structs and type definitions
│   ├── config_loader.hpp           # Parameter loading and hot-reload
│   ├── source_adapter.hpp          # Subscribes to and converts sensor data
│   ├── filters/
│   │   ├── i_filter.hpp            # Filter interface
│   │   ├── range_filter.hpp        # Min/max distance filter
│   │   ├── angular_filter.hpp      # Angular sector filter
│   │   └── box_filter.hpp          # Axis-aligned box filter (+ invert for footprint filter)
│   └── merge_engine/
│       ├── i_merge_engine.hpp      # Merge engine interface
│       ├── cpu_merge_engine.hpp    # CPU merge implementation
│       ├── cuda_merge_engine.hpp   # CUDA GPU merge implementation
│       └── cuda_types.cuh          # GPU type definitions
└── src/
    ├── main.cpp                    # Entry point
    ├── polka_node.cpp              # Node implementation
    ├── config_loader.cpp           # Parameter loading logic
    ├── source_adapter.cpp          # Source subscription logic
    ├── filters/                    # Filter implementations
    └── merge_engine/               # Merge engine implementations
```

## Acknowledgments

The per-point deskewing motion model (SE(3) exponential map with constant-acceleration + constant-angular-velocity) is inspired by rko_lio:

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
