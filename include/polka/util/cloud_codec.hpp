// Copyright 2026 Panav Arpit Raaj <praajarpit@gmail.com>
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

#ifndef POLKA__UTIL__CLOUD_CODEC_HPP_
#define POLKA__UTIL__CLOUD_CODEC_HPP_

#include <cstdint>

#include "polka/types.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace polka
{

// Whether the output cloud carries the float64 per-point 'time' field.
enum class PointTimeField { OMIT, INCLUDE };

// Packed output layout, the same bytes PCL >= 1.14.1 emits for pcl::toROSMsg:
//   OMIT:    x y z intensity       (float32)          point_step 16
//   INCLUDE: x y z intensity time  (float32, float64) point_step 24
// Older PCL (Humble..Kilted) pads to 32 bytes per point; writing the layout
// directly halves the payload there and skips the intermediate PCLPointCloud2.
constexpr uint32_t kPackedStepXYZI = 16;
constexpr uint32_t kPackedStepXYZIT = 24;

// One pass from the merged cloud into 'msg'. Header is left to the caller.
void cloud_to_msg(
  const CloudT & cloud, PointTimeField time, sensor_msgs::msg::PointCloud2 & msg);

// pcl::fromROSMsg without its extra full-payload copy on PCL < 1.13.1 (Humble):
// fields are matched exactly as PCL matches them, then copied straight from 'msg'.
void cloud_from_msg(const sensor_msgs::msg::PointCloud2 & msg, CloudT & cloud);

}  // namespace polka

#endif  // POLKA__UTIL__CLOUD_CODEC_HPP_
