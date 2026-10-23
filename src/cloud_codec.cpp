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

#include "polka/util/cloud_codec.hpp"

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cstring>
#include <vector>

#include <sensor_msgs/msg/point_field.hpp>

namespace polka
{

namespace
{

void add_field(
  sensor_msgs::msg::PointCloud2 & msg, const char * name, uint32_t offset, uint8_t datatype)
{
  sensor_msgs::msg::PointField f;
  f.name = name;
  f.offset = offset;
  f.datatype = datatype;
  f.count = 1;
  msg.fields.push_back(f);
}

}  // namespace

void cloud_to_msg(
  const CloudT & cloud, PointTimeField time, sensor_msgs::msg::PointCloud2 & msg)
{
  using sensor_msgs::msg::PointField;
  const bool with_time = time == PointTimeField::INCLUDE;
  const uint32_t step = with_time ? kPackedStepXYZIT : kPackedStepXYZI;

  msg.fields.clear();
  add_field(msg, "x", 0, PointField::FLOAT32);
  add_field(msg, "y", 4, PointField::FLOAT32);
  add_field(msg, "z", 8, PointField::FLOAT32);
  add_field(msg, "intensity", 12, PointField::FLOAT32);
  if (with_time) {
    add_field(msg, "time", 16, PointField::FLOAT64);
  }

  // Trust width x height only when it describes every point; otherwise publish
  // the points as one unorganized row so the message is always self-consistent.
  const size_t n = cloud.size();
  const bool organized = static_cast<size_t>(cloud.width) * cloud.height == n;
  msg.height = organized ? cloud.height : 1;
  msg.width = organized ? cloud.width : static_cast<uint32_t>(n);
  msg.is_bigendian = false;
  msg.is_dense = cloud.is_dense;
  msg.point_step = step;
  msg.row_step = step * msg.width;
  msg.data.resize(n * step);

  // PointXYZIT: x y z pad (16 B) | intensity | pad | time. Repack without the pads.
  uint8_t * out = msg.data.data();
  for (const auto & p : cloud.points) {
    std::memcpy(out, p.data, 3 * sizeof(float));
    std::memcpy(out + 12, &p.intensity, sizeof(float));
    if (with_time) {
      std::memcpy(out + 16, &p.time, sizeof(double));
    }
    out += step;
  }
}

void cloud_from_msg(const sensor_msgs::msg::PointCloud2 & msg, CloudT & cloud)
{
  std::vector<pcl::PCLPointField> fields;
  pcl_conversions::toPCL(msg.fields, fields);
  pcl::MsgFieldMap field_map;
  pcl::createMapping<PointT>(fields, field_map);

  pcl_conversions::toPCL(msg.header, cloud.header);
  cloud.resize(static_cast<size_t>(msg.width) * msg.height);
  cloud.width = msg.width;
  cloud.height = msg.height;
  cloud.is_dense = msg.is_dense;

  const uint8_t * rows = msg.data.data();
  auto * out = reinterpret_cast<uint8_t *>(cloud.points.data());
  for (uint32_t r = 0; r < msg.height; ++r) {
    const uint8_t * in = rows + static_cast<size_t>(r) * msg.row_step;
    for (uint32_t c = 0; c < msg.width; ++c) {
      for (const auto & m : field_map) {
        std::memcpy(out + m.struct_offset, in + m.serialized_offset, m.size);
      }
      in += msg.point_step;
      out += sizeof(PointT);
    }
  }
}

}  // namespace polka
