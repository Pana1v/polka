# Test data

`airy_turn.mcap`: 0.3 s of a real articulated (center-steered) vehicle at its sharpest
turn (0.66 rad/s at 0.45 m/s). zstd-compressed MCAP, readable on every supported distro.

| Topic | Content |
|---|---|
| `/pointcloud/airy_{front,rear,left_front}` | RoboSense Airy, 96 x 900 organized, FLOAT64 absolute `timestamp`, 3 scans each |
| `/imu/airy_{front,rear,left_front}` | each lidar's built-in 200 Hz IMU (NED frame, no orientation) |
| `/articulated_steering_controller/odom` | wheel odometry, twist in `base_footprint` |
| `/tf`, `/tf_static`, `/joint_states` | articulation joint and extrinsics |

Cut from the recording shared by [@kyrie2to11](https://github.com/kyrie2to11) for polka
testing in [#2](https://github.com/Pana1v/polka/issues/2). Thanks!
