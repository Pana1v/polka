^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package polka
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2026-05-28)
------------------
* Add CHANGELOG.rst (REP 132) and release metadata to package.xml (website / repository / bugtracker URLs, author tag); bump version to 0.3.0.
* Prettify logs: startup banner, ``polka:`` prefix, unified throttle constants.
* Warn once per source on missing ``intensity`` field instead of throttled-repeat.
* Embed pipeline demo GIF in README; refactor README formatting; add minimal config example and multi-LiDAR IMU deskew example.

0.2.0 (2026-04-30)
------------------
* Add Jazzy (Ubuntu 24.04) distro support; remove ``ManualByNode`` liveliness QoS unsupported on Jazzy rclcpp.
* Per-source IMU topic override for articulated platforms (turrets, hinged vehicles, manipulators).
* Gravity-aware IMU deskew: subtract gravity from linear acceleration using orientation when covariance is valid; fall back to rotation-only deskew otherwise.
* Fix IMU→sensor frame rotation in deskew and inter-source alignment.
* Fix degenerate-quaternion fallthrough in the SE(3) exponential map.
* Add throttled warning for inter-source IMU→sensor TF lookup failure.
* Fix thread safety, stale timestamps, dead code, config duplication; add CUDA error checking.
* Configurable output QoS.
* Warn on missing ``intensity`` field instead of silently zeroing.

0.1.0 (2026-03-31)
------------------
* Initial release of polka — composable multi-LiDAR fusion node.
* Heterogeneous source fusion: mix PointCloud2 and LaserScan inputs in a single merge step.
* Per-source filters (range / angular / box) applied before merge.
* Output filters: range / angular / box, footprint (ego-body exclusion), height clip, voxel downsample — applied in a defined order.
* Dual output: merged PointCloud2, LaserScan, or both.
* IMU-based per-point deskewing using the SE(3) exponential map (constant angular velocity + constant acceleration motion model).
* Optional CUDA GPU merge engine with fused kernels and pre-allocated buffers; CPU fallback when unavailable.
* TF2 integration with last-known-good transform fallback.
* Default Release build configuration.
* Pipeline comparison documentation (polka vs. multi-node pcl_ros chain).
