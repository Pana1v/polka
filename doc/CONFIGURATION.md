# Configuration

Every parameter lives under the `polka` namespace. [`config/example_params.yaml`](../config/example_params.yaml) is a minimal starting point; [`config/detailed_params.yaml`](../config/detailed_params.yaml) is the annotated reference with all of them.

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

Everything else has a default. Add filters, deskewing and GPU acceleration when you need them.

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

> Big clouds (a 64-beam spinning lidar, say) get dropped under `best_effort` when the executor is busy. Use `"reliable"` with a depth of `10` for those sources.

## Output filters

These run on the merged cloud before it is published, in this order:

1. **Output filters** (range, angular, box)
2. **Footprint filter**: drops points inside the robot-body exclusion zones
3. **Height filter**: clips to `[z_min, z_max]`
4. **Voxel downsample**: thins density with a VoxelGrid

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

> Any positive voxel `leaf_size` turns voxel downsampling on, even with `enabled: false`. Leave `leaf_size` at `0.0` (or drop it) if you don't want voxelization.

## Motion compensation (IMU deskewing)

Corrects for the robot moving while a LiDAR scan is being collected. Per-point deskewing runs an SE(3) exponential-map motion model off the IMU's angular velocity and linear acceleration, applied to each point by that point's own timestamp. Inter-source alignment handles timing offsets between sensors. The motion model is inspired by [rko_lio](https://github.com/PRBonn/rko_lio) (Malladi et al., 2025).

```yaml
motion_compensation:
  enabled: true
  imu_topic: "/imu/data"          # sensor_msgs/Imu topic (global, used by all sources)
  max_imu_age: 0.2                # seconds, reject stale IMU
  imu_buffer_size: 200            # ring buffer (~1 s at 200 Hz)
  per_point_deskew: true          # per-point correction within each scan
  deskew_timestamp_field: "auto"  # auto-detects 'time', 't', 'timestamp', etc.
```

**Per-point timestamp auto-detect.** With `deskew_timestamp_field: "auto"`, polka checks each `PointCloud2` for one of `time`, `t`, `timestamp`, `time_stamp`, `offset_time`, `timeStamp`. Name the field yourself if your driver calls it something else. If there is no usable field, polka logs once and falls back to whole-scan deskewing for that source.

**Gravity subtraction.** Gravity comes out of `linear_acceleration` only when the IMU publishes a valid orientation (`orientation_covariance[0] >= 0` and a non-degenerate quaternion). Otherwise acceleration is zeroed and deskewing is rotation-only, which still helps, but translation during the scan goes uncorrected.

### Per-source IMU override

On articulated platforms — hinged vehicles, manipulators, humanoids, rotating turrets — each moving sensor can read an IMU bolted to its own moving body while the fixed sensors share the platform IMU. polka uses TF to rotate both angular velocity and linear acceleration from the IMU frame into each sensor frame, so `robot_state_publisher` has to keep the IMU-to-sensor transform current. A dynamic transform, driven by joint_states from a turret encoder for instance, works as is.

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

[`config/example_articulated_imu.yaml`](../config/example_articulated_imu.yaml) has a working two-source setup. On a fully rigid platform, stick with the global `motion_compensation.imu_topic`.

## Rosbag / simulation playback

The defaults assume live sensors on the wall clock. The `source_timeout` staleness check compares each message's header stamp against the node clock, so replaying a bag — whose stamps are from whenever it was recorded — makes every source look stale and **nothing gets published**. Almost everyone hits this the first time they test against a bag.

Turn on simulated time and play the bag with `--clock` so the node clock follows bag time:

```bash
ros2 launch polka polka.launch.py use_sim_time:=true
ros2 bag play <bag> --clock
```

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `false` | Set `true` for bag or simulation replay; the node then uses ROS time driven by `/clock`. |

When polka spots a clock/timestamp mismatch it prints one warning that names the fix:

- Bag stamps far behind the system clock, `use_sim_time` left `false`: set `use_sim_time:=true` and play with `--clock`.
- `use_sim_time:=true` but nothing is publishing `/clock`: add `--clock`. The clock is frozen without it, so clouds may still flow but on an undefined stamp. Always pass `--clock`.
