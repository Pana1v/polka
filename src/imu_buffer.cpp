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

#include "polka/imu_buffer.hpp"
#include <Eigen/Geometry>
#include <cmath>

namespace polka {

ImuBuffer::ImuBuffer(rclcpp::Node * node, const std::string & topic, int buffer_size)
: max_size_(buffer_size), logger_(node->get_logger()), clock_(node->get_clock())
{
  sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
    topic, rclcpp::SensorDataQoS(),
    std::bind(&ImuBuffer::callback, this, std::placeholders::_1));
  RCLCPP_INFO(logger_, "IMU buffer: subscribing to '%s' (capacity %d)", topic.c_str(), buffer_size);
}

std::shared_ptr<const AveragedImu> ImuBuffer::snapshot() const
{
  return std::atomic_load(&snapshot_);
}

void ImuBuffer::callback(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  const auto & a = msg->linear_acceleration;
  const auto & w = msg->angular_velocity;
  if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z) ||
      !std::isfinite(w.x) || !std::isfinite(w.y) || !std::isfinite(w.z)) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
      "IMU buffer: non-finite values, ignoring");
    return;
  }

  Eigen::Vector3d accel(a.x, a.y, a.z);
  if (msg->orientation_covariance[0] >= 0.0) {
    const auto & q = msg->orientation;
    Eigen::Quaterniond ori(q.w, q.x, q.y, q.z);
    if (ori.squaredNorm() > 0.5) {
      ori.normalize();
      constexpr double kGravity = 9.80665;
      Eigen::Vector3d g_imu =
        ori.toRotationMatrix().transpose() * Eigen::Vector3d(0.0, 0.0, kGravity);
      accel -= g_imu;
    } else {
      accel.setZero();
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000,
        "IMU buffer: degenerate orientation quaternion, translation deskew disabled");
    }
  } else {
    accel.setZero();
    RCLCPP_WARN_ONCE(logger_,
      "IMU buffer: IMU has no orientation — cannot subtract gravity, "
      "translation deskew disabled (rotation-only)");
  }

  ImuSample sample;
  sample.wx = w.x;  sample.wy = w.y;  sample.wz = w.z;
  sample.ax = accel.x();  sample.ay = accel.y();  sample.az = accel.z();
  sample.stamp = rclcpp::Time(msg->header.stamp);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.push_back(sample);
    while (static_cast<int>(buffer_.size()) > max_size_)
      buffer_.pop_front();
  }

  auto avg = std::make_shared<AveragedImu>();
  avg->angular_vel = Eigen::Vector3d(w.x, w.y, w.z);
  avg->linear_accel = accel;
  avg->valid = true;
  avg->frame_id = msg->header.frame_id;
  std::atomic_store(&snapshot_, std::const_pointer_cast<const AveragedImu>(avg));
}

}  // namespace polka
