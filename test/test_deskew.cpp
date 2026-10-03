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

#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "polka/input/source_adapter.hpp"
#include "polka/util/se3_exp.hpp"

using namespace std::chrono_literals;

namespace polka
{

namespace
{

constexpr char kTopic[] = "/deskew_test_points";
constexpr size_t kPoints = 400;
constexpr double kScanPeriod = 0.1;   // 10 Hz lidar
constexpr double kTolerance = 1e-3;   // 1 mm, well under any real skew below
const rclcpp::Time kHeaderStamp(1700000000, 0, RCL_ROS_TIME);

struct Scan
{
  std::vector<Eigen::Vector3d> world;   // truth, header-time sensor frame
  std::vector<Eigen::Vector3d> seen;    // what the moving sensor measured
  std::vector<double> dt;               // seconds after header (may be negative)
};

// Static points on a ring, swept once. 'motion' maps dt to the sensor pose at dt
// relative to header time.
template<typename Motion>
Scan make_scan(double dt_first, Motion motion)
{
  Scan s;
  for (size_t i = 0; i < kPoints; ++i) {
    const double frac = static_cast<double>(i) / (kPoints - 1);
    const double azimuth = -M_PI + 2.0 * M_PI * frac;
    const double range = 5.0 + 15.0 * frac;
    const double dt = dt_first + kScanPeriod * frac;

    const Eigen::Vector3d p(range * std::cos(azimuth), range * std::sin(azimuth), 0.5);
    s.world.push_back(p);
    s.seen.push_back(motion(dt).inverse() * p);
    s.dt.push_back(dt);
  }
  return s;
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

  // Feeds one scan through a fresh SourceAdapter holding a fixed IMU reading,
  // returns the deskewed cloud (null on timeout).
  CloudT::ConstPtr deskew(
    const Scan & scan, const Eigen::Vector3d & angular_vel, const Eigen::Vector3d & accel)
  {
    auto imu = std::make_shared<AveragedImu>();
    imu->angular_vel = angular_vel;
    imu->linear_accel = accel;
    imu->valid = true;

    SourceConfig cfg;
    cfg.name = "test";
    cfg.topic = kTopic;
    cfg.qos_reliability = "reliable";
    cfg.qos_history_depth = 5;

    SourceAdapter adapter(
      node_.get(), cfg, false, [imu]() {return imu;}, true, "auto");

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

}  // namespace polka

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
