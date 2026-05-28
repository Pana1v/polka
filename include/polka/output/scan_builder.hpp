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

#ifndef POLKA__OUTPUT__SCAN_BUILDER_HPP_
#define POLKA__OUTPUT__SCAN_BUILDER_HPP_

#include "polka/types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <string>
#include <vector>

namespace polka {

/// Builds sensor_msgs::LaserScan from a merged PointCloud2 or a pre-computed range vector.
/// Centralises header assembly so both CPU and GPU paths share the same stamp/frame logic.
class ScanBuilder {
public:
  void configure(const ScanOutputConfig & cfg, double output_rate,
                 const std::string & frame_id);

  /// Project a 3D cloud to a 2D LaserScan using the configured flatten parameters.
  sensor_msgs::msg::LaserScan from_cloud(
    CloudT::ConstPtr cloud, const rclcpp::Time & stamp) const;

  /// Wrap a pre-computed range vector (GPU output) into a LaserScan message.
  sensor_msgs::msg::LaserScan from_ranges(
    const std::vector<float> & ranges, const rclcpp::Time & stamp) const;

private:
  sensor_msgs::msg::LaserScan make_header(const rclcpp::Time & stamp) const;

  ScanOutputConfig config_;
  float scan_time_ = 0.0f;
  std::string frame_id_;
};

}  // namespace polka

#endif  // POLKA__OUTPUT__SCAN_BUILDER_HPP_
