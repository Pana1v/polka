^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package polka
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Add per-feature demo GIFs (multi-LiDAR fusion, output filters, angular invert, self-filter, voxel downsample, dual output, 2D scan merge) generated from the TIERS multi-LiDAR dataset with a headless Open3D + gifski toolchain.
* Slim the README to badges, demo gallery, and quick start; move parameter and pipeline detail into ``doc/CONFIGURATION.md`` and ``doc/PIPELINE.md``.
* Consolidate documentation assets (docs, images, demo media) under ``doc/``.
* Remove the superseded ``pipeline_demo.gif``.

0.5.0 (2026-07-25)
------------------
* Add ``polka_monitor`` terminal diagnostics dashboard and ``dashboard`` launch arg.
* Two-phase runtime reconfigure and ``/diagnostics`` stats with timing and rate drift flags.
* CPU and CUDA merge performance optimizations, plus bounded stale-source reuse.
* Coarse-stride SE(3) rotation interpolation for faster per-point deskew.
* Estimate body-frame gravity via EMA when the IMU lacks orientation.
* Detect rosbag/clock misconfiguration and expose ``use_sim_time``.
* Per-point timestamp passthrough with a duplicate-timestamp guard.
* Add Iron, Kilted, and Lyrical distro support with a per-distro CI build matrix.
* Behavior change: ``suppress_duplicate_timestamps`` and ``diagnostics.enabled`` now default to true.
* ``example_params.yaml`` slimmed to a minimal example; full reference moved to ``config/detailed_params.yaml``.

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
