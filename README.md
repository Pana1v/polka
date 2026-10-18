# POLKA

<p align="center">
  <a href="https://polka-ros.github.io/"><img src="https://img.shields.io/badge/docs-polka--ros.github.io-3F51B5?logo=materialformkdocs&logoColor=white" alt="Documentation"/></a>
  <img src="https://img.shields.io/badge/ROS_2-Humble_|_Jazzy_|_Kilted_|_Lyrical_|_Rolling-22314E?logo=ros&logoColor=white" alt="ROS 2 distros"/>
  <img src="https://img.shields.io/badge/CUDA-optional-76B900?logo=nvidia&logoColor=white" alt="CUDA optional"/>
  <a href="LICENSE"><img src="https://img.shields.io/github/license/Pana1v/polka?color=blue" alt="License: Apache-2.0"/></a>
</p>

<p align="center">
  <a href="https://polka-ros.github.io/"><img src="https://polka-ros.github.io/images/polka.png" alt="Polka" width="640"/></a>
</p>

**Multi-LiDAR fusion node for ROS 2.** Merges any mix of PointCloud2 and LaserScan sources into one cloud, one scan, or both, with per-source and output filters, IMU/odometry deskew, and CUDA when an NVIDIA GPU is present. It replaces a relay, filter, transform, merge and downsample chain.

**Documentation: [polka-ros.github.io][docs]**

<p align="center">
  <img src="https://polka-ros.github.io/gifs/filter_box.gif" width="400" alt="box filter"/>
  <img src="https://polka-ros.github.io/gifs/deskew_3d.gif" width="400" alt="deskew"/>
</p>

## Install

From apt, with CUDA built in (amd64; needs only the NVIDIA driver, falls back to CPU without a GPU):

```bash
sudo curl -fsSL https://polka-ros.github.io/polka.gpg -o /usr/share/keyrings/polka.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/polka.gpg] https://polka-ros.github.io $(. /etc/os-release && echo $VERSION_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/polka.list
sudo apt update && sudo apt install ros-${ROS_DISTRO:?source your ROS setup.bash first}-polka
```

From source, using the branch named after your distro:

```bash
git clone -b humble https://github.com/Pana1v/polka.git ~/ros2_ws/src/polka
cd ~/ros2_ws && colcon build --packages-select polka   # CUDA engine is built when nvcc is found
```

Driver versions, distro branches and details: [Install][install].

## Quick start

```bash
cp config/example_params.yaml config/my_robot.yaml   # set your topics and output_frame_id
ros2 launch polka polka.launch.py config_file:=config/my_robot.yaml
```

Then see [Configuration][configuration].

## License

Apache-2.0. The per-point deskew model is inspired by [rko_lio](https://github.com/PRBonn/rko_lio) ([arXiv:2509.06593](https://arxiv.org/pdf/2509.06593)). Maintainers: [MAINTAINING.md](MAINTAINING.md).

[docs]: https://polka-ros.github.io/
[install]: https://polka-ros.github.io/install.html
[configuration]: https://polka-ros.github.io/configuration.html
