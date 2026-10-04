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

#include "polka/input/odom_buffer.hpp"

#include <cmath>

namespace polka
{

namespace
{
// About 2 s of history at a typical 100 Hz odometry rate.
constexpr size_t kMaxSamples = 200;
// The newest twist stands in for an empty scan window only while this fresh.
constexpr std::chrono::milliseconds kMaxFallbackAge{500};

bool finite(const geometry_msgs::msg::Vector3 & v)
{
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
}  // namespace

OdomBuffer::OdomBuffer(rclcpp::Node * node, const std::string & topic)
: topic_(topic), logger_(node->get_logger())
{
  sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    topic, rclcpp::SensorDataQoS(),
    std::bind(&OdomBuffer::callback, this, std::placeholders::_1));
  RCLCPP_INFO(logger_, "polka: odometry twist from '%s' for translation deskew", topic.c_str());
}

void OdomBuffer::callback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const auto & tw = msg->twist.twist;
  if (!finite(tw.linear) || !finite(tw.angular)) {return;}

  if (msg->child_frame_id.empty()) {
    RCLCPP_WARN_ONCE(
      logger_, "polka: odometry on '%s' has no child_frame_id; its twist is taken "
      "as already in each lidar's frame", topic_.c_str());
  }

  std::lock_guard<std::mutex> lock(mutex_);
  samples_.push_back(
    {rclcpp::Time(msg->header.stamp),
      Eigen::Vector3d(tw.linear.x, tw.linear.y, tw.linear.z),
      Eigen::Vector3d(tw.angular.x, tw.angular.y, tw.angular.z)});
  while (samples_.size() > kMaxSamples) {
    samples_.pop_front();
  }
  frame_id_ = msg->child_frame_id;
  last_receipt_ = std::chrono::steady_clock::now();
}

std::shared_ptr<const BodyTwist> OdomBuffer::average(
  const rclcpp::Time & from, const rclcpp::Time & to) const
{
  auto twist = std::make_shared<BodyTwist>();
  std::lock_guard<std::mutex> lock(mutex_);
  if (samples_.empty()) {return twist;}
  twist->frame_id = frame_id_;

  int count = 0;
  for (const auto & s : samples_) {
    if (s.stamp.get_clock_type() != from.get_clock_type() || s.stamp < from || s.stamp > to) {
      continue;
    }
    twist->linear += s.linear;
    twist->angular += s.angular;
    ++count;
  }
  if (count > 0) {
    twist->linear /= count;
    twist->angular /= count;
    twist->valid = true;
    return twist;
  }

  // Window empty: odometry slower than the scan, or on another clock.
  if (std::chrono::steady_clock::now() - last_receipt_ > kMaxFallbackAge) {return twist;}
  twist->linear = samples_.back().linear;
  twist->angular = samples_.back().angular;
  twist->valid = true;
  return twist;
}

}  // namespace polka
