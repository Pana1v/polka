# Polka

<p align="center">
  <img src="polka.png" alt="Polka" width="300"/>
</p>

**Multi-LiDAR fusion node for ROS 2** — merges any mix of PointCloud2 and LaserScan sources into a unified output, with optional CUDA GPU acceleration.

Polka replaces multi-node pipelines (relay → filter → transform → merge → downsample) with a single composable node.

## Features

- **Heterogeneous source fusion** — mix 3D PointCloud2 and 2D LaserScan sensors freely
- **Dual output** — publish merged PointCloud2, LaserScan, or both simultaneously
- **Per-source filtering** — range, angular, and box filters applied before merge
- **Output filtering** — range, angular, box, height cap, self-filter (ego-body exclusion), voxel downsampling
- **Motion compensation** — velocity-based correction for inter-scan time differences (Odometry or TwistStamped)
- **CUDA acceleration** — optional GPU merge engine with fused kernels and pre-allocated buffers
- **TF2 integration** — automatic transform lookup with fallback to last known good transform
- **Fully parameterized** — every feature is runtime-configurable via ROS 2 parameters
- **Composable node** — runs standalone or loaded into a component container

## Dependencies

| Package | Purpose |
|---|---|
| `rclcpp` / `rclcpp_components` | ROS 2 node framework |
| `sensor_msgs` | PointCloud2, LaserScan messages |
| `nav_msgs` / `geometry_msgs` | Odometry, TwistStamped for motion compensation |
| `tf2_ros` / `tf2_eigen` | Frame transforms |
| `pcl_conversions` | PCL <-> ROS message conversion |
| `laser_geometry` | LaserScan -> PointCloud2 projection |
| CUDA toolkit | **Optional** — only needed for GPU merge engine |

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

### Motion Compensation

Corrects for robot motion between source scan timestamps using first-order velocity approximation.

```yaml
motion_compensation:
  enabled: true
  velocity_topic: "~/odom"
  velocity_type: "odometry"    # or "twist_stamped"
  max_velocity_age: 0.2
```

### Output Filters

Applied to the merged cloud before publishing, in this order:

1. **Output filters** (range / angular / box)
2. **Self-filter** — removes points inside robot body exclusion zones
3. **Height cap** — clips to `[z_min, z_max]`
4. **Voxel downsample** — reduces density via VoxelGrid

```yaml
outputs:
  cloud:
    height_cap:
      enabled: true
      z_min: -1.0
      z_max: 3.0
    voxel:
      enabled: true
      leaf_size: 0.05
    self_filter:
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

## Architecture

```
Sources (N sensors)          Merge Engine           Output Pipeline
┌─────────────┐
│ PointCloud2  │──► per-source ──►┐
│ /front/points│    filters       │
└─────────────┘                   │    ┌──────────┐    ┌────────────┐
                                  ├───►│ CPU  or  │───►│ Filters    │──► PointCloud2
┌─────────────┐                   │    │ CUDA     │    │ Self-filter│──► LaserScan
│ LaserScan   │──► per-source ──►┘    │ merge    │    │ Height cap │
│ /rear/scan  │    filters            └──────────┘    │ Voxel      │
└─────────────┘                                       └────────────┘
```

## File Structure

```
polka/
├── config/example_params.yaml      # Full annotated config reference
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
│   │   └── box_filter.hpp          # Axis-aligned box filter (+ invert for self-filter)
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

## License

Apache-2.0
