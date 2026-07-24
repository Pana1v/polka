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

#ifndef POLKA__INPUT__IMU_BUFFER_HPP_
#define POLKA__INPUT__IMU_BUFFER_HPP_

#include <Eigen/Core>

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace polka
{

struct ImuSample
{
  double wx = 0.0, wy = 0.0, wz = 0.0;   // angular velocity (rad/s)
  double ax = 0.0, ay = 0.0, az = 0.0;    // linear acceleration (m/s²)
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

struct AveragedImu
{
  Eigen::Vector3d angular_vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_accel = Eigen::Vector3d::Zero();
  bool valid = false;
  std::string frame_id;
};

class ImuBuffer
{
public:
  ImuBuffer(rclcpp::Node * node, const std::string & topic, int buffer_size);

  std::shared_ptr<const AveragedImu> snapshot() const;

  // Diagnostics accessors - independent of snapshot()/valid, so a diagnostics
  // tick can report "never received" vs. "receiving but degenerate" separately.
  const std::string & topic() const {return topic_;}
  uint64_t msg_count() const {return msg_count_.load(std::memory_order_relaxed);}
  rclcpp::Time last_stamp() const;

private:
  void callback(sensor_msgs::msg::Imu::ConstSharedPtr msg);

  std::string topic_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  std::deque<ImuSample> buffer_;
  mutable std::mutex mutex_;
  std::shared_ptr<const AveragedImu> snapshot_;
  std::atomic<uint64_t> msg_count_{0};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  int max_size_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  // 体坐标系重力 EMA（无 orientation 时用静止比力均值估计）
  Eigen::Vector3d g_body_ema_ = Eigen::Vector3d::Zero();
  bool g_body_initialized_ = false;
};

}  // namespace polka

#endif  // POLKA__INPUT__IMU_BUFFER_HPP_
