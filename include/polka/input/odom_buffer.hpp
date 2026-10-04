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

#ifndef POLKA__INPUT__ODOM_BUFFER_HPP_
#define POLKA__INPUT__ODOM_BUFFER_HPP_

#include <Eigen/Core>

#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace polka
{

// Rigid-body twist of 'frame_id' (an odometry child_frame_id), expressed in it.
struct BodyTwist
{
  Eigen::Vector3d linear = Eigen::Vector3d::Zero();    // m/s
  Eigen::Vector3d angular = Eigen::Vector3d::Zero();   // rad/s
  std::string frame_id;
  bool valid = false;
};

// Recent twists from a nav_msgs/Odometry topic (wheel odometry, an EKF). Must not
// be computed from the clouds it deskews, or the loop feeds its own error back.
class OdomBuffer
{
public:
  OdomBuffer(rclcpp::Node * node, const std::string & topic);

  // Mean twist over the messages stamped in [from, to]. With none inside, the
  // newest twist if it arrived recently, else invalid.
  std::shared_ptr<const BodyTwist> average(
    const rclcpp::Time & from, const rclcpp::Time & to) const;

  const std::string & topic() const {return topic_;}

private:
  struct Sample
  {
    rclcpp::Time stamp;
    Eigen::Vector3d linear;
    Eigen::Vector3d angular;
  };

  void callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);

  std::string topic_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  mutable std::mutex mutex_;
  std::deque<Sample> samples_;
  std::string frame_id_;
  std::chrono::steady_clock::time_point last_receipt_;
  rclcpp::Logger logger_;
};

}  // namespace polka

#endif  // POLKA__INPUT__ODOM_BUFFER_HPP_
