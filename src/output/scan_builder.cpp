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

#include "polka/output/scan_builder.hpp"

#include <cmath>
#include <limits>

namespace polka
{

void ScanBuilder::configure(
  const ScanOutputConfig & cfg, double output_rate, const std::string & frame_id)
{
  config_ = cfg;
  frame_id_ = frame_id;
  scan_time_ = (output_rate > 0.0) ? 1.0f / static_cast<float>(output_rate) : 0.0f;
}

sensor_msgs::msg::LaserScan ScanBuilder::make_header(const rclcpp::Time & stamp) const
{
  const auto & fp = config_.flatten;
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = frame_id_;
  scan.header.stamp = stamp;
  scan.angle_min = static_cast<float>(fp.angle_min);
  scan.angle_max = static_cast<float>(fp.angle_max);
  scan.angle_increment = static_cast<float>(fp.angle_increment);
  scan.range_min = static_cast<float>(fp.range_min);
  scan.range_max = static_cast<float>(fp.range_max);
  scan.time_increment = 0.0f;
  scan.scan_time = scan_time_;
  return scan;
}

sensor_msgs::msg::LaserScan ScanBuilder::from_cloud(
  CloudT::ConstPtr cloud, const rclcpp::Time & stamp) const
{
  const auto & fp = config_.flatten;
  const float z_min = static_cast<float>(fp.z_min);
  const float z_max = static_cast<float>(fp.z_max);
  const float a_min = static_cast<float>(fp.angle_min);
  const float a_max = static_cast<float>(fp.angle_max);
  const float a_inc = static_cast<float>(fp.angle_increment);
  const float r_min = static_cast<float>(fp.range_min);
  const float r_max = static_cast<float>(fp.range_max);
  const int n = fp.n_bins;

  auto scan = make_header(stamp);
  scan.ranges.assign(n, std::numeric_limits<float>::infinity());

  for (const auto & p : *cloud) {
    if (p.z < z_min || p.z > z_max) {continue;}
    float az = std::atan2(p.y, p.x);
    if (az < a_min || az > a_max) {continue;}
    int bin = static_cast<int>((az - a_min) / a_inc);
    if (bin < 0 || bin >= n) {continue;}
    float range = std::sqrt(p.x * p.x + p.y * p.y);
    if (range < r_min || range > r_max) {continue;}
    scan.ranges[bin] = std::min(scan.ranges[bin], range);
  }
  return scan;
}

sensor_msgs::msg::LaserScan ScanBuilder::from_ranges(
  const std::vector<float> & ranges, const rclcpp::Time & stamp) const
{
  auto scan = make_header(stamp);
  scan.ranges = ranges;
  return scan;
}

}  // namespace polka
