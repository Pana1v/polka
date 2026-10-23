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

// Fields, dimensions and steps of the packed layout (see kPackedStepXYZI in
// types.hpp) for 'width' x 'height' points. Leaves header and data alone.
void packed_cloud_layout(
  PointTimeField time, uint32_t width, uint32_t height, bool is_dense,
  sensor_msgs::msg::PointCloud2 & msg);

// One pass from the merged cloud into 'msg', in the packed layout. Header is
// left to the caller.
void cloud_to_msg(
  const CloudT & cloud, PointTimeField time, sensor_msgs::msg::PointCloud2 & msg);

// pcl::fromROSMsg without its extra full-payload copy on PCL < 1.13.1 (Humble):
// fields are matched exactly as PCL matches them, then copied straight from 'msg'.
void cloud_from_msg(const sensor_msgs::msg::PointCloud2 & msg, CloudT & cloud);

}  // namespace polka

#endif  // POLKA__UTIL__CLOUD_CODEC_HPP_
