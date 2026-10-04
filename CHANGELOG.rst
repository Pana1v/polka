^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package polka
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fix the first deskewed scan of each source using the gyro in the IMU's own frame instead of the lidar's. It reversed the turn for upside-down (NED) IMUs, as built into many lidars: 13 cm off on a real scan.
* Add a real-data deskew test on 0.3 s of the articulated-rig bag from #2 (``test/data/airy_turn.mcap``, 8.4 MB). New test dependencies: ``rosbag2_cpp``, ``rosbag2_storage_mcap``, ``tf2_msgs``.
* Deskew cost no longer depends on point order. Exact rotations are built once per scan at evenly spaced angles, and each point steps from the nearest one. Vertically sweeping lidars stored column by column, and unordered clouds, now cost the same as spinning ones: on public 85k-point Airy clouds, shuffled order went from 9.9 to 4.4 ns per point (SE(3)) and 11.9 to 3.3 (rotation). Accuracy is unchanged.
* Add ``motion_compensation.translation`` (``none`` | ``imu_accel`` | ``odometry``) and ``motion_compensation.odom_topic``. Deskew had no velocity term, so steady motion went uncorrected (1.5 m/s: 15 cm of skew per 0.1 s scan); ``odometry`` takes ``v * dt`` from a ``nav_msgs/Odometry`` twist, moved to each lidar's origin with its lever arm. ``none`` deskews rotation only. ``imu_accel`` stays the default. New dependency: ``nav_msgs``.
* Behavior change: deskew and inter-source alignment average the IMU over the scan's own time span instead of using the newest sample. On a real AMR IMU stream, the newest-sample acceleration term was worse than none in 76% of 0.1 s windows while moving; one 8 m/s^2 shock after a scan shifted it 4 cm.
* The SE(3) deskew path re-anchors often enough to stay under 0.1 mm at road speed, where translation reaches metres per scan. 4.3 to 4.4 ns per point.
* ``motion_compensation.max_imu_age`` and ``imu_frame`` never had an effect. They still load, with a startup warning, and are gone from the example configs.
* Example configs list only non-default values and load to the same parameters as before. ``detailed_params.yaml`` now shows the real defaults for ``outputs.scan`` (``enabled``, ``z_min``, ``z_max``, ``range_max``) and ``outputs.cloud.voxel.leaf_size``; its old ``leaf_size: 0.05`` silently turned voxelization on.
* Interpolate the SE(3) deskew path between exact anchors, as the rotation-only path already does: 6.9 to 4.3 ns per point on a 66k-point scan. Accuracy stays under 1 mm, including ring-major clouds whose point times jump back at every ring.
* Fix per-point deskew direction. Points were moved by the inverse of the sensor motion, which doubled intra-scan skew instead of removing it (0.6 rad/s yaw: 25 cm RMS raw, 50 cm after deskew, under 0.01 cm now).
* Make the rotation-only deskew path reachable. It required IMU acceleration of exactly zero, which no real IMU produces since the EMA gravity estimate, so every scan took the slow SE(3) path. It now runs whenever the scan-wide translation from ``0.5 * a * dt^2`` is under 1 mm, and re-anchors across time jumps such as ring boundaries.
* Closed-form SE(3) deskew path. Per-point math now runs in float. Per point on a 66k-point scan: 41.5 to 3.3 ns on the rotation path, 41.5 to 6.9 ns on SE(3).
* Support Ouster-style per-point timestamps: a ``UINT32`` time field is now accepted and read as a nanosecond offset from ``header.stamp``. Previously the datatype was rejected outright, so Ouster sources ran with per-point deskewing and timestamp passthrough silently disabled.
* Resolve per-point time units and epoch from the field's declared datatype instead of a value-magnitude test. The magnitude test now applies only to ``FLOAT64``, the one ambiguous case; RoboSense and Velodyne behavior is unchanged.
* Reject per-point times that decode to more than 10 s from the header stamp, with a diagnostic warning, rather than deskewing on misread units.

0.5.0 (2026-07-25)
------------------
* Add ``polka_monitor`` terminal diagnostics dashboard and ``dashboard`` launch arg.
* Two-phase runtime reconfigure and ``/diagnostics`` stats with timing and rate drift flags.
* CPU and CUDA merge performance optimizations, plus bounded stale-source reuse.
* CPU angular filter replaces per-point atan2 with a precomputed cross-product half-plane test, about 3x faster (10.47 to 3.55 ms per tick at 259k points).
* Coarse-stride SE(3) rotation interpolation cuts per-source deskew latency from ~9.8 ms to ~1.6 ms (about 6.2x) with negligible accuracy loss (max error ~1.6e-7 cm).
* Estimate body-frame gravity via EMA when the IMU lacks orientation.
* Detect rosbag/clock misconfiguration and expose ``use_sim_time``.
* Per-point timestamp passthrough with a duplicate-timestamp guard.
* Add Iron, Kilted, and Lyrical distro support with a per-distro CI build matrix.
* Modular refactor of the node internals: headers reorganized into subdirectories, with the output path split into ``OutputPipeline`` and ``ScanBuilder``.
* Behavior change: ``suppress_duplicate_timestamps`` and ``diagnostics.enabled`` now default to true.
* Add per-feature demo GIFs (multi-LiDAR fusion, output filters, angular invert, self-filter, voxel downsample, dual output, 2D scan merge) generated from the TIERS multi-LiDAR dataset with a headless Open3D + gifski toolchain.
* Slim the README to badges, demo gallery, and quick start; move parameter and pipeline detail into ``doc/CONFIGURATION.md`` and ``doc/PIPELINE.md``.
* Consolidate documentation assets (docs, images, demo media) under ``doc/``.
* Remove the superseded ``pipeline_demo.gif``.
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
