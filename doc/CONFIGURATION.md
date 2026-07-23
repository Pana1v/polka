# Configuration

All parameters live under the `polka` namespace. [`config/example_params.yaml`](../config/example_params.yaml) is a minimal starter; [`config/detailed_params.yaml`](../config/detailed_params.yaml) is the full annotated reference of every parameter.

## Minimal config

```yaml
polka:
  ros__parameters:
    output_frame_id: "base_link"
    output_rate: 20.0
    source_names: ["front_3d", "rear_2d"]
    sources:
      front_3d:
        topic: "/front_lidar/points"
        type: "pointcloud2"
      rear_2d:
        topic: "/rear_lidar/scan"
        type: "laserscan"
    outputs:
      cloud:
        enabled: true
      scan:
        enabled: true
```

Everything else has sensible defaults. Add filters, deskewing, and GPU acceleration as needed.

## Key parameters

| Parameter | Default | Description |
|---|---|---|
| `output_frame_id` | `"base_link"` | Target frame for all merged output |
| `output_rate` | `20.0` | Merge and publish rate (Hz) |
| `source_timeout` | `0.5` | Drop a source if no data arrives within this window (s) |
| `enable_gpu` | `true` | Use the CUDA merge engine when available (falls back to CPU) |
| `timestamp_strategy` | `"earliest"` | Output stamp: `earliest`, `latest`, `average`, or `local` |

## Per-source parameters

| Parameter | Default | Description |
|---|---|---|
| `sources.<name>.topic` | `""` | Subscription topic (required) |
| `sources.<name>.type` | `"pointcloud2"` | `"pointcloud2"` or `"laserscan"` |
| `sources.<name>.imu_topic` | `""` | Per-source IMU override (empty uses the global IMU) |
| `sources.<name>.qos_reliability` | `"best_effort"` | `"best_effort"` or `"reliable"` |
| `sources.<name>.qos_history_depth` | `1` | QoS queue depth |

> Tip: large clouds (e.g. a 64-beam spinning lidar) can be dropped under `best_effort` when the executor is busy. Use `"reliable"` with a depth of `10` for those sources.

## Output filters

Applied to the merged cloud before publishing, in this order:

1. **Output filters** (range, angular, box)
2. **Footprint filter**: removes points inside robot-body exclusion zones
3. **Height filter**: clips to `[z_min, z_max]`
4. **Voxel downsample**: reduces density via VoxelGrid

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

> Note: any positive voxel `leaf_size` enables voxel downsampling, even with `enabled: false`. Leave `leaf_size` at `0.0` (or omit it) when you do not want voxelization.

## Motion compensation (IMU deskewing)

Corrects for robot motion during a LiDAR scan using IMU data. Per-point deskewing uses the SE(3) exponential map motion model with angular velocity and linear acceleration from the IMU, applied to each point by its per-point timestamp. Inter-source alignment corrects for timing offsets between sensors. The motion model is inspired by [rko_lio](https://github.com/TixiaoShan/rko_lio) (Malladi et al., 2025).

```yaml
motion_compensation:
  enabled: true
  imu_topic: "/imu/data"          # sensor_msgs/Imu topic (global, used by all sources)
  max_imu_age: 0.2                # seconds, reject stale IMU
  imu_buffer_size: 200            # ring buffer (~1 s at 200 Hz)
  per_point_deskew: true          # per-point correction within each scan
  deskew_timestamp_field: "auto"  # auto-detects 'time', 't', 'timestamp', etc.
```

**Per-point timestamp auto-detect.** With `deskew_timestamp_field: "auto"`, polka scans each `PointCloud2` for one of: `time`, `t`, `timestamp`, `time_stamp`, `offset_time`, `timeStamp`. Set a specific name if your driver differs; if no usable field is present, polka logs once and falls back to whole-scan deskewing for that source.

**Gravity subtraction.** Gravity is subtracted from `linear_acceleration` only when the IMU publishes a valid orientation (`orientation_covariance[0] >= 0` and a non-degenerate quaternion). Otherwise acceleration is zeroed and deskewing is rotation-only: still useful, but translation during the scan is not corrected.

### Per-source IMU override

Articulated platforms (hinged vehicles, manipulators, humanoids, rotating turrets) can override the IMU per source: each moving sensor reads an IMU rigidly mounted to its moving body, while fixed sensors share the global platform IMU. polka uses TF to rotate both angular velocity and linear acceleration from the IMU frame into each sensor frame, so `robot_state_publisher` must keep the IMU-to-sensor transform current. A dynamic transform (e.g. driven by joint_states from a turret encoder) works out of the box.

```yaml
motion_compensation:
  enabled: true
  imu_topic: "/imu/data"          # global fallback IMU

sources:
  turret_lidar:
    topic: "/turret/points"
    imu_topic: "/turret/imu/data" # per-source override
  chassis_lidar:
    topic: "/chassis/points"
    # imu_topic omitted, falls back to /imu/data
```

A working two-source setup is in [`config/example_articulated_imu.yaml`](../config/example_articulated_imu.yaml). The global `motion_compensation.imu_topic` remains the recommended path for fully rigid platforms.

## Rosbag / simulation playback

The default configuration targets **live sensor data** on the system (wall) clock. The `source_timeout` staleness check compares each message's header stamp against the node clock, so a replayed bag (whose stamps are historical) makes every source look stale and **nothing is published**. Most people hit this the first time they test with a bag.

Enable simulated time and play the bag with `--clock` so the node clock tracks bag time:

```bash
ros2 launch polka polka.launch.py use_sim_time:=true
ros2 bag play <bag> --clock
```

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `false` | Set `true` for rosbag/simulation replay; the node then uses ROS time driven by `/clock`. |

If polka detects a clock/timestamp mismatch it prints one actionable warning naming the fix:

- Bag stamps far behind the system clock (`use_sim_time` left `false`): set `use_sim_time:=true` and play with `--clock`.
- `use_sim_time:=true` but nothing publishes `/clock`: add `--clock`. (The clock is frozen otherwise, so clouds may still flow on an undefined stamp. Always pass `--clock` for correct timing.)
