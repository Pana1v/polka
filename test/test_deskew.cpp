// Copyright 2025 Panav Arpit Raaj <praajarpit@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Per-point deskew against exact truth, through the real SourceAdapter.
//
//   static world points ──► sensor moves with a known twist
//                       ──► point i seen at dt_i in the moving frame (skewed)
//                       ──► SourceAdapter deskews to header time
//                       ──► must land back on the world points
//
// The world frame is the sensor frame at header time, so truth needs no TF.

#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <tf2_ros/buffer.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <numeric>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "polka/input/source_adapter.hpp"
#include "polka/util/se3_exp.hpp"

using namespace std::chrono_literals;

namespace polka
{

namespace
{

constexpr char kTopic[] = "/deskew_test_points";
constexpr char kImuTopic[] = "/deskew_test_imu";
constexpr double kImuRate = 400.0;     // Hz, Xsens-class
constexpr double kGravity = 9.80665;
constexpr int kImuBufferSize = 200;
constexpr size_t kPoints = 400;
constexpr size_t kDensePoints = 8000;  // ~ a 16-ring lidar's share per 0.1 s
constexpr size_t kRings = 8;
constexpr size_t kSweepRows = 64;      // vertically sweeping solid-state lidar
constexpr size_t kSweepCols = 125;
constexpr double kScanPeriod = 0.1;   // 10 Hz lidar
constexpr double kTolerance = 1e-3;   // 1 mm, well under any real skew below
const rclcpp::Time kHeaderStamp(1700000000, 0, RCL_ROS_TIME);

struct Scan
{
  std::vector<Eigen::Vector3d> world;   // truth, header-time sensor frame
  std::vector<Eigen::Vector3d> seen;    // what the moving sensor measured
  std::vector<double> dt;               // seconds after header (may be negative)
};

// Static points on a ring, swept once per ring. 'motion' maps dt to the sensor pose
// at dt relative to header time. With rings > 1 the cloud is ring-major, as many
// drivers emit it: each ring sweeps the whole scan period, so dt jumps back at
// every ring start.
template<typename Motion>
Scan make_scan(double dt_first, Motion motion, size_t points = kPoints, size_t rings = 1)
{
  Scan s;
  const size_t per_ring = points / rings;
  for (size_t i = 0; i < per_ring * rings; ++i) {
    const size_t ring = i / per_ring;
    const double frac = static_cast<double>(i % per_ring) / (per_ring - 1);
    const double azimuth = -M_PI + 2.0 * M_PI * frac;
    const double range = 5.0 + 15.0 * frac;
    const double dt = dt_first + kScanPeriod * frac;

    const Eigen::Vector3d p(
      range * std::cos(azimuth), range * std::sin(azimuth), 0.5 + 0.3 * ring);
    s.world.push_back(p);
    s.seen.push_back(motion(dt).inverse() * p);
    s.dt.push_back(dt);
  }
  return s;
}

// Vertically sweeping lidar (rows bottom to top over the scan period): every point
// in a row shares one time, while memory order is column-major, so consecutive
// points hop between rows and times.
template<typename Motion>
Scan make_vertical_sweep(Motion motion)
{
  Scan s;
  for (size_t col = 0; col < kSweepCols; ++col) {
    for (size_t row = 0; row < kSweepRows; ++row) {
      const double u = static_cast<double>(col) / (kSweepCols - 1);
      const double v = static_cast<double>(row) / (kSweepRows - 1);
      const double azimuth = -1.0 + 2.0 * u;          // +-57 deg
      const double elevation = -0.35 + 0.7 * v;       // +-20 deg
      const double range = 5.0 + 15.0 * u;
      const double dt = kScanPeriod * v;

      const Eigen::Vector3d p = range * Eigen::Vector3d(
        std::cos(elevation) * std::cos(azimuth), std::cos(elevation) * std::sin(azimuth),
        std::sin(elevation));
      s.world.push_back(p);
      s.seen.push_back(motion(dt).inverse() * p);
      s.dt.push_back(dt);
    }
  }
  return s;
}

// Same points in a fixed pseudo-random order, as unordered or non-repetitive
// (Livox-style) clouds arrive.
Scan shuffled(const Scan & in)
{
  std::vector<size_t> order(in.dt.size());
  std::iota(order.begin(), order.end(), 0);
  std::shuffle(order.begin(), order.end(), std::mt19937(7));
  Scan out;
  for (size_t i : order) {
    out.world.push_back(in.world[i]);
    out.seen.push_back(in.seen[i]);
    out.dt.push_back(in.dt[i]);
  }
  return out;
}

// FLOAT64 absolute-epoch 'timestamp' field, as RoboSense and Hesai drivers emit it.
sensor_msgs::msg::PointCloud2 to_msg(const Scan & s)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = "lidar";
  msg.header.stamp = kHeaderStamp;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(s.seen.size());

  sensor_msgs::PointCloud2Modifier mod(msg);
  mod.setPointCloud2Fields(
    5,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
    "timestamp", 1, sensor_msgs::msg::PointField::FLOAT64);
  mod.resize(s.seen.size());

  sensor_msgs::PointCloud2Iterator<float> ix(msg, "x"), iy(msg, "y"), iz(msg, "z"),
  ii(msg, "intensity");
  sensor_msgs::PointCloud2Iterator<double> it(msg, "timestamp");
  for (size_t i = 0; i < s.seen.size(); ++i) {
    *ix = static_cast<float>(s.seen[i].x());
    *iy = static_cast<float>(s.seen[i].y());
    *iz = static_cast<float>(s.seen[i].z());
    *ii = 1.0f;
    *it = kHeaderStamp.seconds() + s.dt[i];
    ++ix; ++iy; ++iz; ++ii; ++it;
  }
  return msg;
}

Eigen::Isometry3d yaw_motion(double omega_z, double dt)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(omega_z * dt, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return T;
}

// Sensor motion over dt for a constant body twist of the sensor itself.
Eigen::Isometry3d twist_motion(const Eigen::Vector3d & w, const Eigen::Vector3d & v, double dt)
{
  return se3_exp(v * dt, w * dt);
}

BodyTwist make_twist(const Eigen::Vector3d & v, const Eigen::Vector3d & w, const char * frame)
{
  BodyTwist t;
  t.linear = v;
  t.angular = w;
  t.frame_id = frame;
  return t;
}

// 0.3 s of a real articulated vehicle at its sharpest turn: three RoboSense Airy
// lidars with built-in IMUs, wheel odometry and TF. See test/data/README.md.
constexpr char kRealBag[] = POLKA_TEST_DATA "/airy_turn.mcap";
constexpr char kRealLidar[] = "/pointcloud/airy_rear";
constexpr char kRealImu[] = "/imu/airy_rear";
constexpr char kRealOdom[] = "/articulated_steering_controller/odom";

struct RealData
{
  std::vector<sensor_msgs::msg::PointCloud2> clouds;
  std::vector<sensor_msgs::msg::Imu> imu;
  std::vector<nav_msgs::msg::Odometry> odom;
  std::vector<geometry_msgs::msg::TransformStamped> tf_static;
};

template<typename T>
T decode(const rosbag2_storage::SerializedBagMessage & bag_msg)
{
  T msg;
  rclcpp::SerializedMessage raw(*bag_msg.serialized_data);
  rclcpp::Serialization<T>().deserialize_message(&raw, &msg);
  return msg;
}

RealData read_real_bag()
{
  rosbag2_storage::StorageOptions options;
  options.uri = kRealBag;
  options.storage_id = "mcap";
  rosbag2_cpp::Reader reader;
  reader.open(options);

  RealData d;
  while (reader.has_next()) {
    const auto m = reader.read_next();
    if (m->topic_name == kRealLidar) {
      d.clouds.push_back(decode<sensor_msgs::msg::PointCloud2>(*m));
    } else if (m->topic_name == kRealImu) {
      d.imu.push_back(decode<sensor_msgs::msg::Imu>(*m));
    } else if (m->topic_name == kRealOdom) {
      d.odom.push_back(decode<nav_msgs::msg::Odometry>(*m));
    } else if (m->topic_name == "/tf_static") {
      for (const auto & t : decode<tf2_msgs::msg::TFMessage>(*m).transforms) {
        d.tf_static.push_back(t);
      }
    }
  }
  return d;
}

double stamp_sec(const builtin_interfaces::msg::Time & t)
{
  return rclcpp::Time(t).seconds();
}

}  // namespace

class DeskewTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("deskew_test");
    exec_.add_node(node_);
  }

  void TearDown() override {exec_.remove_node(node_);}

  // Feeds one scan through a fresh SourceAdapter holding a fixed IMU reading
  // (and, for ODOMETRY, a fixed body twist), returns the deskewed cloud (null on
  // timeout).
  CloudT::ConstPtr deskew(
    const Scan & scan, const Eigen::Vector3d & angular_vel, const Eigen::Vector3d & accel,
    TranslationMode translation = TranslationMode::IMU_ACCEL,
    const BodyTwist & body_twist = BodyTwist(),
    std::shared_ptr<tf2_ros::Buffer> tf = nullptr)
  {
    AveragedImu imu;
    imu.angular_vel = angular_vel;
    imu.linear_accel = accel;
    imu.valid = true;
    return deskew_msg(to_msg(scan), imu, translation, body_twist, tf);
  }

  // Feeds one PointCloud2 through a fresh SourceAdapter holding a fixed IMU reading
  // and body twist, returns the deskewed cloud (null on timeout).
  CloudT::ConstPtr deskew_msg(
    const sensor_msgs::msg::PointCloud2 & msg, const AveragedImu & imu_reading,
    TranslationMode translation, const BodyTwist & body_twist,
    std::shared_ptr<tf2_ros::Buffer> tf)
  {
    auto imu = std::make_shared<AveragedImu>(imu_reading);

    SourceConfig cfg;
    cfg.name = "test";
    cfg.topic = kTopic;
    cfg.qos_reliability = "reliable";
    cfg.qos_history_depth = 5;

    auto twist = std::make_shared<BodyTwist>(body_twist);
    twist->valid = true;

    SourceAdapter adapter(
      node_.get(), cfg, false,
      [imu](const rclcpp::Time &, const rclcpp::Time &) {return imu;}, true, "auto",
      tf, kImuBufferSize, translation,
      [twist](const rclcpp::Time &, const rclcpp::Time &) {return twist;});

    auto pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(kTopic, 5);
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (!adapter.received() && std::chrono::steady_clock::now() < deadline) {
      pub->publish(msg);
      exec_.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    return adapter.received() ? adapter.get_latest() : nullptr;
  }

  // Same as deskew(), but through a real per-source ImuBuffer fed with an IMU
  // stream: gyro 'gyro(t)' and constant 'accel' over [t_from, t_to] (relative to
  // header), with the newest sample's accel replaced by 'last_accel'.
  CloudT::ConstPtr deskew_streamed(
    const Scan & scan, const std::function<Eigen::Vector3d(double)> & gyro,
    const Eigen::Vector3d & accel, double t_from, double t_to,
    const Eigen::Vector3d & last_accel)
  {
    SourceConfig cfg;
    cfg.name = "test";
    cfg.topic = kTopic;
    cfg.imu_topic = kImuTopic;
    cfg.qos_reliability = "reliable";
    cfg.qos_history_depth = 5;

    SourceAdapter adapter(node_.get(), cfg, false, nullptr, true, "auto");

    // Level orientation, so polka removes exactly +g along z.
    auto imu_pub = node_->create_publisher<sensor_msgs::msg::Imu>(
      kImuTopic, rclcpp::SensorDataQoS().keep_last(1000));
    const int samples = static_cast<int>((t_to - t_from) * kImuRate) + 1;
    for (int k = 0; k < samples; ++k) {
      const double t = t_from + k / kImuRate;
      const Eigen::Vector3d angular_vel = gyro(t);
      sensor_msgs::msg::Imu imu;
      imu.header.stamp = kHeaderStamp + rclcpp::Duration::from_seconds(t);
      imu.orientation.w = 1.0;
      imu.orientation_covariance[0] = 0.0;
      const Eigen::Vector3d a = (k == samples - 1 ? last_accel : accel) +
        Eigen::Vector3d(0.0, 0.0, kGravity);
      imu.linear_acceleration.x = a.x();
      imu.linear_acceleration.y = a.y();
      imu.linear_acceleration.z = a.z();
      imu.angular_velocity.x = angular_vel.x();
      imu.angular_velocity.y = angular_vel.y();
      imu.angular_velocity.z = angular_vel.z();
      imu_pub->publish(imu);
      exec_.spin_some();
    }
    for (int k = 0; k < 20; ++k) {
      exec_.spin_some();
      std::this_thread::sleep_for(5ms);
    }

    auto pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(kTopic, 5);
    const auto msg = to_msg(scan);
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (!adapter.received() && std::chrono::steady_clock::now() < deadline) {
      pub->publish(msg);
      exec_.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    return adapter.received() ? adapter.get_latest() : nullptr;
  }

  static double max_error(const CloudT & cloud, const Scan & scan)
  {
    double worst = 0.0;
    for (size_t i = 0; i < cloud.size(); ++i) {
      const Eigen::Vector3d p(cloud[i].x, cloud[i].y, cloud[i].z);
      worst = std::max(worst, (p - scan.world[i]).norm());
    }
    return worst;
  }

  rclcpp::executors::SingleThreadedExecutor exec_;
  std::shared_ptr<rclcpp::Node> node_;
};

TEST_F(DeskewTest, YawOnlyLandsOnTruth)
{
  // AMR turning at its cap. Raw skew at 20 m is about 1.2 m.
  const Eigen::Vector3d w(0.0, 0.0, 0.6);
  const auto scan = make_scan(0.0, [&](double dt) {return yaw_motion(w.z(), dt);});

  const auto out = deskew(scan, w, Eigen::Vector3d::Zero());
  ASSERT_TRUE(out) << "adapter received nothing";
  ASSERT_EQ(out->size(), kPoints);
  EXPECT_LT(max_error(*out, scan), kTolerance);
}

TEST_F(DeskewTest, YawWithAccelLandsOnTruth)
{
  // Translation large enough to force the full SE(3) path.
  const Eigen::Vector3d w(0.0, 0.0, 0.6);
  const Eigen::Vector3d a(2.0, 0.5, 0.0);
  const auto scan = make_scan(0.0, [&](double dt) {return compute_motion_delta(w, a, dt);});

  const auto out = deskew(scan, w, a);
  ASSERT_TRUE(out) << "adapter received nothing";
  ASSERT_EQ(out->size(), kPoints);
  EXPECT_LT(max_error(*out, scan), kTolerance);
}

TEST_F(DeskewTest, HeaderAtScanEndLandsOnTruth)
{
  // Drivers that stamp the header at scan end give negative dt.
  const Eigen::Vector3d w(0.0, 0.0, -0.6);
  const auto scan = make_scan(-kScanPeriod, [&](double dt) {return yaw_motion(w.z(), dt);});

  const auto out = deskew(scan, w, Eigen::Vector3d::Zero());
  ASSERT_TRUE(out) << "adapter received nothing";
  ASSERT_EQ(out->size(), kPoints);
  EXPECT_LT(max_error(*out, scan), kTolerance);
}

TEST_F(DeskewTest, ImuNoiseAccelStillLandsOnTruth)
{
  // A real IMU never reports exactly zero accel after gravity removal.
  // 0.05 m/s^2 moves a point under 0.3 mm over one scan.
  const Eigen::Vector3d w(0.0, 0.0, 0.6);
  const Eigen::Vector3d a(0.05, -0.03, 0.02);
  const auto scan = make_scan(0.0, [&](double dt) {return compute_motion_delta(w, a, dt);});

  const auto out = deskew(scan, w, a);
  ASSERT_TRUE(out) << "adapter received nothing";
  ASSERT_EQ(out->size(), kPoints);
  EXPECT_LT(max_error(*out, scan), kTolerance);
}

TEST_F(DeskewTest, DenseFastTurnWithAccelLandsOnTruth)
{
  // Dense like a real lidar, so any per-point shortcut is exercised, and a tilted
  // axis turning fast enough to stress interpolation.
  const Eigen::Vector3d w(0.1, -0.05, 2.0);
  const Eigen::Vector3d a(3.0, 1.0, 0.5);
  const auto scan = make_scan(
    0.0, [&](double dt) {return compute_motion_delta(w, a, dt);}, kDensePoints);

  const auto out = deskew(scan, w, a);
  ASSERT_TRUE(out) << "adapter received nothing";
  ASSERT_EQ(out->size(), kDensePoints);
  EXPECT_LT(max_error(*out, scan), kTolerance);
}

TEST_F(DeskewTest, RingMajorWithAccelLandsOnTruth)
{
  // Point time jumps back at every ring start: nothing may carry over across it.
  const Eigen::Vector3d w(0.0, 0.0, 0.6);
  const Eigen::Vector3d a(2.0, 0.0, 0.0);
  const auto scan = make_scan(
    0.0, [&](double dt) {return compute_motion_delta(w, a, dt);}, kDensePoints, kRings);

  const auto out = deskew(scan, w, a);
  ASSERT_TRUE(out) << "adapter received nothing";
  ASSERT_EQ(out->size(), kDensePoints);
  EXPECT_LT(max_error(*out, scan), kTolerance);
}

TEST_F(DeskewTest, RingMajorYawOnlyLandsOnTruth)
{
  const Eigen::Vector3d w(0.0, 0.0, 0.6);
  const auto scan = make_scan(
    0.0, [&](double dt) {return yaw_motion(w.z(), dt);}, kDensePoints, kRings);

  const auto out = deskew(scan, w, Eigen::Vector3d::Zero());
  ASSERT_TRUE(out) << "adapter received nothing";
  ASSERT_EQ(out->size(), kDensePoints);
  EXPECT_LT(max_error(*out, scan), kTolerance);
}

TEST_F(DeskewTest, VerticalSweepLandsOnTruth)
{
  const Eigen::Vector3d w(0.1, -0.05, 2.0);
  const Eigen::Vector3d a(3.0, 1.0, 0.5);
  const auto scan = make_vertical_sweep([&](double dt) {return compute_motion_delta(w, a, dt);});

  const auto out = deskew(scan, w, a);
  ASSERT_TRUE(out) << "adapter received nothing";
  ASSERT_EQ(out->size(), kSweepRows * kSweepCols);
  EXPECT_LT(max_error(*out, scan), kTolerance);
}

TEST_F(DeskewTest, ShuffledOrderLandsOnTruth)
{
  const Eigen::Vector3d w(0.1, -0.05, 2.0);
  const auto scan = shuffled(
    make_scan(0.0, [&](double dt) {return yaw_motion(w.z(), dt);}, kDensePoints, kRings));

  const auto out = deskew(scan, Eigen::Vector3d(0.0, 0.0, w.z()), Eigen::Vector3d::Zero());
  ASSERT_TRUE(out) << "adapter received nothing";
  ASSERT_EQ(out->size(), kDensePoints);
  EXPECT_LT(max_error(*out, scan), kTolerance);
}

TEST_F(DeskewTest, TimeOutsideSampledRangeLandsOnTruth)
{
  // The scan's time span is estimated from a sample of points. A late point the
  // sample skips must still deskew exactly.
  const Eigen::Vector3d w(0.0, 0.0, 2.0);
  const Eigen::Vector3d a(2.0, 0.0, 0.0);
  auto motion = [&](double dt) {return compute_motion_delta(w, a, dt);};
  auto scan = make_scan(0.0, motion);
  constexpr size_t kUnsampled = 3;   // 400 points are sampled every 7th
  scan.dt[kUnsampled] = 1.5 * kScanPeriod;
  scan.seen[kUnsampled] = motion(scan.dt[kUnsampled]).inverse() * scan.world[kUnsampled];

  const auto out = deskew(scan, w, a);
  ASSERT_TRUE(out) << "adapter received nothing";
  ASSERT_EQ(out->size(), kPoints);
  EXPECT_LT(max_error(*out, scan), kTolerance);
}

TEST_F(DeskewTest, TranslationNoneIgnoresAccel)
{
  // Rotation-only mode: a large IMU acceleration must not move any point.
  const Eigen::Vector3d w(0.0, 0.0, 0.6);
  const Eigen::Vector3d a(3.0, 1.0, 0.0);
  const auto scan = make_scan(0.0, [&](double dt) {return yaw_motion(w.z(), dt);});

  const auto out = deskew(scan, w, a, TranslationMode::NONE);
  ASSERT_TRUE(out) << "adapter received nothing";
  ASSERT_EQ(out->size(), kPoints);
  EXPECT_LT(max_error(*out, scan), kTolerance);
}

TEST_F(DeskewTest, OdometryVelocityLandsOnTruth)
{
  // Steady 1.5 m/s while turning, zero acceleration: 15 cm of skew that
  // IMU_ACCEL cannot see.
  const Eigen::Vector3d w(0.0, 0.0, 0.3);
  const Eigen::Vector3d v(1.5, 0.2, 0.0);
  const auto scan = make_scan(0.0, [&](double dt) {return twist_motion(w, v, dt);});

  const auto out = deskew(
    scan, w, Eigen::Vector3d::Zero(), TranslationMode::ODOMETRY, make_twist(v, w, "lidar"));
  ASSERT_TRUE(out) << "adapter received nothing";
  ASSERT_EQ(out->size(), kPoints);
  EXPECT_LT(max_error(*out, scan), kTolerance);
}

TEST_F(DeskewTest, OdometryFastForwardAndTurnLandsOnTruth)
{
  // 20 m/s with a hard turn on a sparse scan: interpolation steps near the 2 mrad
  // re-anchor bound while translation reaches 2 m, so holding the translation
  // Jacobian between anchors alone would err by about 2 mm.
  const Eigen::Vector3d w(0.0, 0.0, 0.45);
  const Eigen::Vector3d v(20.0, 0.0, 0.0);
  const auto scan = make_scan(0.0, [&](double dt) {return twist_motion(w, v, dt);});

  const auto out = deskew(
    scan, w, Eigen::Vector3d::Zero(), TranslationMode::ODOMETRY, make_twist(v, w, "lidar"));
  ASSERT_TRUE(out) << "adapter received nothing";
  ASSERT_EQ(out->size(), kPoints);
  EXPECT_LT(max_error(*out, scan), kTolerance);
}

TEST_F(DeskewTest, OdometryLeverArmLandsOnTruth)
{
  // Base spins in place; the lidar sits 0.6 m forward, 0.3 m left, yawed 90 deg.
  // The base has no velocity, yet the lidar origin moves at w x r, about 0.4 m/s.
  const Eigen::Vector3d w_base(0.0, 0.0, 0.6);
  Eigen::Isometry3d T_base_lidar = Eigen::Isometry3d::Identity();
  T_base_lidar.translate(Eigen::Vector3d(0.6, 0.3, 0.5));
  T_base_lidar.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));

  auto tf = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  geometry_msgs::msg::TransformStamped tf_msg = tf2::eigenToTransform(T_base_lidar);
  tf_msg.header.frame_id = "base_link";
  tf_msg.child_frame_id = "lidar";
  tf->setTransform(tf_msg, "test", true);

  // Lidar motion = base motion seen from the lidar.
  const auto scan = make_scan(
    0.0, [&](double dt) {
      return T_base_lidar.inverse() * yaw_motion(w_base.z(), dt) * T_base_lidar;
    });

  const Eigen::Vector3d w_lidar = T_base_lidar.linear().transpose() * w_base;
  const auto out = deskew(
    scan, w_lidar, Eigen::Vector3d::Zero(), TranslationMode::ODOMETRY,
    make_twist(Eigen::Vector3d::Zero(), w_base, "base_link"), tf);
  ASSERT_TRUE(out) << "adapter received nothing";
  ASSERT_EQ(out->size(), kPoints);
  EXPECT_LT(max_error(*out, scan), kTolerance);
}

TEST_F(DeskewTest, FlippedImuFrameFirstScanLandsOnTruth)
{
  // IMU mounted upside down relative to the lidar (NED-style, 180 deg about x), as
  // on lidars with built-in IMUs. Even a source's very first scan must rotate the
  // gyro into the lidar frame before deskewing.
  const Eigen::Vector3d w_lidar(0.0, 0.0, 0.6);
  const Eigen::Isometry3d T_lidar_imu(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  auto tf = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  geometry_msgs::msg::TransformStamped tf_msg = tf2::eigenToTransform(T_lidar_imu);
  tf_msg.header.frame_id = "lidar";
  tf_msg.child_frame_id = "imu_ned";
  tf->setTransform(tf_msg, "test", true);

  const auto scan = make_scan(0.0, [&](double dt) {return yaw_motion(w_lidar.z(), dt);});
  AveragedImu imu;
  imu.angular_vel = T_lidar_imu.linear().transpose() * w_lidar;
  imu.frame_id = "imu_ned";
  imu.valid = true;

  const auto out = deskew_msg(to_msg(scan), imu, TranslationMode::NONE, BodyTwist(), tf);
  ASSERT_TRUE(out) << "adapter received nothing";
  ASSERT_EQ(out->size(), kPoints);
  EXPECT_LT(max_error(*out, scan), kTolerance);
}

TEST_F(DeskewTest, RealTurningScanMatchesExactDeskew)
{
  // A real rear-lidar scan at 0.66 rad/s: organized 96 x 900 Airy layout with
  // no-return NaNs and absolute FLOAT64 point times. polka's anchored float deskew
  // must match an exact double-precision SE(3) built from the same IMU, odometry
  // and TF.
  const RealData d = read_real_bag();
  ASSERT_FALSE(d.clouds.empty());
  ASSERT_FALSE(d.tf_static.empty());
  const auto & msg = d.clouds[d.clouds.size() / 2];
  const double header = stamp_sec(msg.header.stamp);

  // Exact per-point times, for the scan window and the reference.
  std::vector<double> dt;
  for (sensor_msgs::PointCloud2ConstIterator<double> it(msg, "timestamp"); it != it.end(); ++it) {
    dt.push_back(*it - header);
  }
  const auto [dt_min, dt_max] = std::minmax_element(dt.begin(), dt.end());

  // Motion over the scan: IMU mean (in the IMU frame), odometry mean (base frame).
  AveragedImu imu;
  int imu_n = 0;
  for (const auto & m : d.imu) {
    const double t = stamp_sec(m.header.stamp) - header;
    if (t < *dt_min || t > *dt_max) {continue;}
    imu.angular_vel += Eigen::Vector3d(
      m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z);
    imu.frame_id = m.header.frame_id;
    ++imu_n;
  }
  ASSERT_GE(imu_n, 2);
  imu.angular_vel /= imu_n;
  imu.valid = true;

  BodyTwist twist;
  int odom_n = 0;
  for (const auto & m : d.odom) {
    const double t = stamp_sec(m.header.stamp) - header;
    if (t < *dt_min || t > *dt_max) {continue;}
    const auto & tw = m.twist.twist;
    twist.linear += Eigen::Vector3d(tw.linear.x, tw.linear.y, tw.linear.z);
    twist.angular += Eigen::Vector3d(tw.angular.x, tw.angular.y, tw.angular.z);
    twist.frame_id = m.child_frame_id;
    ++odom_n;
  }
  ASSERT_GE(odom_n, 1);
  twist.linear /= odom_n;
  twist.angular /= odom_n;

  auto tf = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  for (const auto & t : d.tf_static) {
    tf->setTransform(t, "bag", true);
  }

  // Reference: exact SE(3) per point in double, motion in the lidar frame.
  const Eigen::Matrix3d R_lidar_imu = tf2::transformToEigen(
    tf->lookupTransform(msg.header.frame_id, imu.frame_id, tf2::TimePointZero)).rotation();
  const Eigen::Isometry3d T_base_lidar = tf2::transformToEigen(
    tf->lookupTransform(twist.frame_id, msg.header.frame_id, tf2::TimePointZero));
  const Eigen::Vector3d w = R_lidar_imu * imu.angular_vel;
  const Eigen::Vector3d v = velocity_at_frame(twist.linear, twist.angular, T_base_lidar);

  const auto out = deskew_msg(msg, imu, TranslationMode::ODOMETRY, twist, tf);
  ASSERT_TRUE(out) << "adapter received nothing";
  ASSERT_EQ(out->size(), dt.size());

  double worst = 0.0, moved = 0.0;
  size_t i = 0;
  for (sensor_msgs::PointCloud2ConstIterator<float> it(msg, "x"); it != it.end(); ++it, ++i) {
    const Eigen::Vector3d p(it[0], it[1], it[2]);
    if (!p.allFinite()) {continue;}
    const Eigen::Vector3d truth = compute_motion_delta(w, v, Eigen::Vector3d::Zero(), dt[i]) * p;
    const Eigen::Vector3d got((*out)[i].x, (*out)[i].y, (*out)[i].z);
    worst = std::max(worst, (got - truth).norm());
    moved = std::max(moved, (truth - p).norm());
  }
  EXPECT_GT(moved, 0.05) << "fixture scan should carry real skew";
  EXPECT_LT(worst, kTolerance);
}

TEST_F(DeskewTest, ImuShockAfterScanIsNotApplied)
{
  // The IMU stream is steady over the scan, but its newest sample (after the
  // scan ends) is a 8 m/s^2 shock, as a bump or a rail gap gives. Deskewing on
  // that one sample would shift points by about 4 cm.
  const Eigen::Vector3d w(0.0, 0.0, 0.6);
  const Eigen::Vector3d a(2.0, 0.5, 0.0);
  const Eigen::Vector3d shock = a + Eigen::Vector3d(8.0, 0.0, 0.0);
  const auto scan = make_scan(0.0, [&](double dt) {return compute_motion_delta(w, a, dt);});

  const auto out = deskew_streamed(
    scan, [&](double) {return w;}, a, -0.05, kScanPeriod + 0.02, shock);
  ASSERT_TRUE(out) << "adapter received nothing";
  ASSERT_EQ(out->size(), kPoints);
  EXPECT_LT(max_error(*out, scan), kTolerance);
}

TEST_F(DeskewTest, GyroBiasLearnedAtStandstill)
{
  // The front Airy on the polka#2 rig reads 24.6 mrad/s while parked. Deskewing a
  // turn with it skews points by bias * dt * range: 5 cm at 20 m. polka must learn
  // the bias while the vehicle stands still and subtract it.
  const Eigen::Vector3d bias(0.0136, 0.0151, 0.0143);
  const Eigen::Vector3d w(0.0, 0.0, 0.6);
  constexpr double kStillFrom = -1.6;   // s before the header: parked
  constexpr double kTurnFrom = -0.1;    // s: starts turning
  const auto scan = make_scan(0.0, [&](double dt) {return yaw_motion(w.z(), dt);});

  const auto out = deskew_streamed(
    scan, [&](double t) {return t < kTurnFrom ? bias : Eigen::Vector3d(w + bias);},
    Eigen::Vector3d::Zero(), kStillFrom, kScanPeriod + 0.02, Eigen::Vector3d::Zero());
  ASSERT_TRUE(out) << "adapter received nothing";
  ASSERT_EQ(out->size(), kPoints);
  const double error = max_error(*out, scan);
  RecordProperty("max_error_mm", std::to_string(error * 1e3));
  EXPECT_LT(error, kTolerance);
}

TEST_F(DeskewTest, SlowSteadyTurnIsNotLearnedAsBias)
{
  // A smooth turn looks still to a gyro (no spread). Above the largest plausible
  // bias it must be kept as motion.
  const Eigen::Vector3d w(0.0, 0.0, 0.05);
  const auto scan = make_scan(0.0, [&](double dt) {return yaw_motion(w.z(), dt);});

  const auto out = deskew_streamed(
    scan, [&](double) {return w;}, Eigen::Vector3d::Zero(), -1.6, kScanPeriod + 0.02,
    Eigen::Vector3d::Zero());
  ASSERT_TRUE(out) << "adapter received nothing";
  ASSERT_EQ(out->size(), kPoints);
  EXPECT_LT(max_error(*out, scan), kTolerance);
}

}  // namespace polka

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
