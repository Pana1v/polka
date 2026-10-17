# POLKA

> **Documentation: [polka-ros.github.io][docs].** This README covers install and a first launch;
> configuration, the pipeline, architecture, deskew and performance are on the site.

<p align="center">
  <a href="https://polka-ros.github.io/"><img src="https://img.shields.io/badge/docs-polka--ros.github.io-3F51B5?logo=materialformkdocs&logoColor=white" alt="Documentation"/></a>
  <br/>
  <a href="https://github.com/Pana1v/polka/tree/humble"><img src="https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white" alt="ROS 2 Humble"/></a>
  <a href="https://github.com/Pana1v/polka/tree/iron"><img src="https://img.shields.io/badge/ROS_2-Iron-22314E?logo=ros&logoColor=white" alt="ROS 2 Iron"/></a>
  <a href="https://github.com/Pana1v/polka/tree/jazzy"><img src="https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros&logoColor=white" alt="ROS 2 Jazzy"/></a>
  <a href="https://github.com/Pana1v/polka/tree/kilted"><img src="https://img.shields.io/badge/ROS_2-Kilted-22314E?logo=ros&logoColor=white" alt="ROS 2 Kilted"/></a>
  <a href="https://github.com/Pana1v/polka/tree/lyrical"><img src="https://img.shields.io/badge/ROS_2-Lyrical-22314E?logo=ros&logoColor=white" alt="ROS 2 Lyrical"/></a>
  <a href="https://github.com/Pana1v/polka/tree/rolling"><img src="https://img.shields.io/badge/ROS_2-Rolling-22314E?logo=ros&logoColor=white" alt="ROS 2 Rolling"/></a>
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
  <a href="https://polka-ros.github.io/"><img src="https://polka-ros.github.io/images/polka.png" alt="Polka" width="640"/></a>
</p>

**Multi-LiDAR fusion node for ROS 2.** Takes any mix of PointCloud2 and LaserScan sources and publishes one merged PointCloud2, one merged LaserScan, or both. Filters per source and on the output, deskews with IMU and odometry, and uses CUDA when an NVIDIA GPU is present. One node instead of a relay, filter, transform, merge and downsample chain.

<p align="center">
  <img src="https://polka-ros.github.io/gifs/filter_box.gif" width="400" alt="box filter"/>
  <img src="https://polka-ros.github.io/gifs/deskew_3d.gif" width="400" alt="deskew"/>
</p>

## Install

### From apt

CUDA-enabled packages for Humble, Jazzy, Kilted, Lyrical and Rolling (amd64). They need only
the NVIDIA driver, not the CUDA toolkit, and run on the CPU on machines without a usable GPU.
Lyrical needs driver 580 or newer, the rest 525 or newer.

```bash
sudo curl -fsSL https://polka-ros.github.io/polka.gpg -o /usr/share/keyrings/polka.gpg
echo "deb [signed-by=/usr/share/keyrings/polka.gpg] https://polka-ros.github.io $(. /etc/os-release && echo $VERSION_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/polka.list
sudo apt update && sudo apt install ros-$ROS_DISTRO-polka
```

### From source

One branch per ROS 2 distro, same code on each:

| Distro | Ubuntu | Branch |
|--------|--------|--------|
| Humble | 22.04 | [`humble`](../../tree/humble) |
| Iron | 22.04 | [`iron`](../../tree/iron) |
| Jazzy | 24.04 | [`jazzy`](../../tree/jazzy) |
| Kilted | 24.04 | [`kilted`](../../tree/kilted) |
| Lyrical | 26.04 | [`lyrical`](../../tree/lyrical) |
| Rolling | 26.04 | [`rolling`](../../tree/rolling) |

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

Point `output_frame_id` at your base frame, list your sensors under `source_names`, and check that TF resolves every sensor `frame_id` to `output_frame_id`. Playing a bag? Pass `use_sim_time:=true` and play with `--clock` (see [Configuration: bag playback][bag]).

Branch maintenance across distros: [MAINTAINING.md](MAINTAINING.md).

## Documentation

Everything beyond install lives on **[polka-ros.github.io][docs]**:

- **[Install][install]**: apt packages with CUDA built in, or build from source
- **[Configuration][configuration]**: every parameter, filters, IMU deskewing, bag playback
- **[Pipeline][pipeline]**: what polka replaces, the internal stages, the file layout
- **[Architecture][architecture]**: the CPU and CUDA merge engines, filters, source adapters, threading
- **[Deskew][deskew]**: how big skew gets, and what deskew fixes on public KITTI data
- **[Performance][performance]**: deskew cost against GLIM, rko_lio and LIO-SAM, the CPU/CUDA crossover

## License and credits

Apache-2.0. The per-point deskewing motion model is inspired by [rko_lio](https://github.com/PRBonn/rko_lio) (Malladi et al., 2025, [arXiv:2509.06593](https://arxiv.org/pdf/2509.06593)).

[docs]: https://polka-ros.github.io/
[install]: https://polka-ros.github.io/install.html
[configuration]: https://polka-ros.github.io/configuration.html
[bag]: https://polka-ros.github.io/configuration.html#rosbag--simulation-playback
[pipeline]: https://polka-ros.github.io/pipeline.html
[architecture]: https://polka-ros.github.io/architecture.html
[deskew]: https://polka-ros.github.io/deskew.html
[performance]: https://polka-ros.github.io/performance.html
