# POLKA

<p align="center">
  <a href="https://github.com/Pana1v/polka/tree/humble"><img src="https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white" alt="ROS 2 Humble"/></a>
  <a href="https://github.com/Pana1v/polka/tree/iron"><img src="https://img.shields.io/badge/ROS_2-Iron-22314E?logo=ros&logoColor=white" alt="ROS 2 Iron"/></a>
  <a href="https://github.com/Pana1v/polka/tree/jazzy"><img src="https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros&logoColor=white" alt="ROS 2 Jazzy"/></a>
  <a href="https://github.com/Pana1v/polka/tree/kilted"><img src="https://img.shields.io/badge/ROS_2-Kilted-22314E?logo=ros&logoColor=white" alt="ROS 2 Kilted"/></a>
  <a href="https://github.com/Pana1v/polka/tree/lyrical"><img src="https://img.shields.io/badge/ROS_2-Lyrical-22314E?logo=ros&logoColor=white" alt="ROS 2 Lyrical"/></a>
  <br/>
  <img src="https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white" alt="Ubuntu 22.04"/>
  <img src="https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white" alt="Ubuntu 24.04"/>
  <img src="https://img.shields.io/badge/Ubuntu-26.04-E95420?logo=ubuntu&logoColor=white" alt="Ubuntu 26.04"/>
  <br/>
  <img src="https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white" alt="C++17"/>
  <img src="https://img.shields.io/badge/CUDA-optional-76B900?logo=nvidia&logoColor=white" alt="CUDA optional"/>
  <img src="https://img.shields.io/badge/PCL-point_cloud-2a9d8f" alt="PCL"/>
  <img src="https://img.shields.io/badge/build-colcon-blue?logo=ros&logoColor=white" alt="colcon"/>
  <br/>
  <a href="LICENSE"><img src="https://img.shields.io/github/license/Pana1v/polka?color=blue" alt="License: Apache-2.0"/></a>
  <a href="https://github.com/Pana1v/polka/stargazers"><img src="https://img.shields.io/github/stars/Pana1v/polka?style=flat" alt="GitHub stars"/></a>
  <a href="https://github.com/Pana1v/polka/issues"><img src="https://img.shields.io/github/issues/Pana1v/polka" alt="GitHub issues"/></a>
  <a href="https://github.com/Pana1v/polka/commits"><img src="https://img.shields.io/github/last-commit/Pana1v/polka" alt="Last commit"/></a>
</p>

<p align="center">
  <img src="doc/images/polka.png" alt="Polka" width="640"/>
</p>

<p align="center">
  <img src="doc/media/gifs/fusion.gif" alt="Polka multi-LiDAR fusion" width="760"/>
  <br/>
  <em>Multi-LiDAR fusion: one spinning Ouster vs. Ouster + Livox Avia + Mid-360 merged into a single cloud.</em>
</p>

**Multi-LiDAR fusion node for ROS 2.** Merges any mix of PointCloud2 and LaserScan sources into a unified PointCloud2 and/or LaserScan, with per-source and output filtering, IMU deskewing, and optional CUDA acceleration. One composable node replaces a relay / filter / transform / merge / downsample chain.

## Features in action

Each clip runs polka with a different config on the [TIERS multi-LiDAR dataset](https://github.com/TIERS/multi_lidar_multi_uav_dataset) (Ouster OS1 + Livox Avia + Mid-360), rendered headless with Open3D. See [`doc/media/`](doc/media/) to regenerate.

<table>
<tr>
<td width="50%"><img src="doc/media/gifs/filter_range.gif" alt="range filter"/><br/><em>Range filter: keep points within a distance shell</em></td>
<td width="50%"><img src="doc/media/gifs/filter_angular.gif" alt="angular filter"/><br/><em>Angular filter: keep a yaw sector</em></td>
</tr>
<tr>
<td width="50%"><img src="doc/media/gifs/filter_box.gif" alt="box filter"/><br/><em>Box filter: crop to an axis-aligned box</em></td>
<td width="50%"><img src="doc/media/gifs/filter_height.gif" alt="height cap"/><br/><em>Height cap: clip to a z-range</em></td>
</tr>
<tr>
<td width="50%"><img src="doc/media/gifs/angular_invert.gif" alt="angular invert flag"/><br/><em>Angular <code>invert</code> flag: keep vs. exclude a sector</em></td>
<td width="50%"><img src="doc/media/gifs/self_filter.gif" alt="self filter"/><br/><em>Self-filter: remove the robot's own footprint</em></td>
</tr>
<tr>
<td width="50%"><img src="doc/media/gifs/voxel.gif" alt="voxel downsample"/><br/><em>Voxel downsample: 69k to 5k points</em></td>
<td width="50%"><img src="doc/media/gifs/dual_output.gif" alt="dual output"/><br/><em>Dual output: merged cloud plus flattened 2D scan</em></td>
</tr>
<tr>
<td colspan="2" align="center"><img src="doc/media/gifs/scan_merge.gif" alt="2D LaserScan merge" width="780"/><br/><em>2D LaserScan merge: each beam colored by the sensor with the nearest return</em></td>
</tr>
</table>

## Features

- **Heterogeneous fusion**: mix 3D PointCloud2 and 2D LaserScan sources freely
- **Dual output**: merged PointCloud2, LaserScan, or both at once
- **Per-source and output filtering**: range, angular, box, height cap, footprint (ego-body) exclusion, voxel downsample
- **IMU deskewing**: per-point SE(3) motion correction, with per-point timestamp auto-detect
- **CUDA acceleration**: optional GPU merge engine, falls back to CPU
- **TF2 integration**: automatic lookup with last-known-good fallback
- **Fully parameterized and composable**: runtime ROS 2 params; standalone or in a component container

## Install

Each ROS 2 distro has its own code-identical branch:

| Distro | Ubuntu | Branch |
|--------|--------|--------|
| Humble | 22.04 | [`humble`](../../tree/humble) |
| Iron | 22.04 | [`iron`](../../tree/iron) |
| Jazzy | 24.04 | [`jazzy`](../../tree/jazzy) |
| Kilted | 24.04 | [`kilted`](../../tree/kilted) |
| Lyrical | 26.04 | [`lyrical`](../../tree/lyrical) |

```bash
git clone -b humble https://github.com/Pana1v/polka.git ~/ros2_ws/src/polka
cd ~/ros2_ws && colcon build --packages-select polka
# add  --cmake-args -DWITH_CUDA=ON  for the GPU merge engine
```

## Quick start

```bash
cp config/example_params.yaml config/my_robot.yaml      # edit topics + output_frame_id
ros2 launch polka polka.launch.py config_file:=config/my_robot.yaml
```

Set `output_frame_id` to your base frame, list sensors under `source_names`, and make sure TF resolves each sensor `frame_id` to `output_frame_id`. Replaying a bag? Pass `use_sim_time:=true` and play with `--clock` (see [Configuration](doc/CONFIGURATION.md#rosbag--simulation-playback)).

## Documentation

- **[Configuration](doc/CONFIGURATION.md)**: every parameter, filters, IMU deskewing, rosbag playback
- **[Pipeline and architecture](doc/PIPELINE.md)**: what polka replaces, internal stages, file layout
- **[Maintaining distro branches](MAINTAINING.md)**: single-source-of-truth sync across the five branches

## License and credits

Apache-2.0. The per-point deskewing motion model is inspired by [rko_lio](https://github.com/TixiaoShan/rko_lio) (Malladi et al., 2025, [arXiv:2509.06593](https://arxiv.org/pdf/2509.06593)).
