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

#include <gtest/gtest.h>

#include <pcl/common/io.h>
#include <pcl/console/print.h>
#include <pcl/pcl_config.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "polka/util/cloud_codec.hpp"

namespace polka
{
namespace
{

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

CloudT make_cloud(size_t n)
{
  CloudT cloud;
  cloud.resize(n);
  for (size_t i = 0; i < n; ++i) {
    auto & p = cloud[i];
    p.x = 0.5f * static_cast<float>(i);
    p.y = -1.25f * static_cast<float>(i);
    p.z = 3.0f + static_cast<float>(i);
    p.intensity = static_cast<float>(i % 256);
    p.time = 1750000000.123456789 + 1e-6 * static_cast<double>(i);
  }
  return cloud;
}

// Airy-like source: x y z intensity | ring (uint16) | pad | timestamp (float64).
PointCloud2 make_airy_msg(uint32_t height, uint32_t width, uint32_t row_pad = 0)
{
  PointCloud2 msg;
  msg.height = height;
  msg.width = width;
  auto field = [](const char * name, uint32_t offset, uint8_t type) {
      PointField f;
      f.name = name;
      f.offset = offset;
      f.datatype = type;
      f.count = 1;
      return f;
    };
  msg.fields = {field("x", 0, PointField::FLOAT32), field("y", 4, PointField::FLOAT32),
    field("z", 8, PointField::FLOAT32), field("intensity", 12, PointField::FLOAT32),
    field("ring", 16, PointField::UINT16), field("timestamp", 24, PointField::FLOAT64)};
  msg.point_step = 32;
  msg.row_step = msg.point_step * width + row_pad;
  msg.is_dense = true;
  msg.data.resize(static_cast<size_t>(msg.row_step) * height);
  for (size_t i = 0; i < msg.data.size(); ++i) {
    msg.data[i] = static_cast<uint8_t>((i * 37 + 11) & 0xff);
  }
  return msg;
}

void expect_same_points(const CloudT & a, const CloudT & b)
{
  ASSERT_EQ(a.size(), b.size());
  EXPECT_EQ(a.width, b.width);
  EXPECT_EQ(a.height, b.height);
  EXPECT_EQ(a.is_dense, b.is_dense);
  for (size_t i = 0; i < a.size(); ++i) {
    // Bitwise: the codec copies bytes, so NaN payloads must survive too.
    ASSERT_EQ(0, std::memcmp(&a[i].x, &b[i].x, 3 * sizeof(float))) << "point " << i;
    ASSERT_EQ(0, std::memcmp(&a[i].intensity, &b[i].intensity, sizeof(float))) << "point " << i;
    ASSERT_EQ(0, std::memcmp(&a[i].time, &b[i].time, sizeof(double))) << "point " << i;
  }
}

class CloudCodec : public ::testing::Test
{
protected:
  void SetUp() override {pcl::console::setVerbosityLevel(pcl::console::L_ERROR);}
};

TEST_F(CloudCodec, PackedXyziLayoutAndValues)
{
  const CloudT cloud = make_cloud(7);
  PointCloud2 msg;
  cloud_to_msg(cloud, PointTimeField::OMIT, msg);

  ASSERT_EQ(msg.fields.size(), 4u);
  const char * names[] = {"x", "y", "z", "intensity"};
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_EQ(msg.fields[i].name, names[i]);
    EXPECT_EQ(msg.fields[i].offset, 4 * i);
    EXPECT_EQ(msg.fields[i].datatype, PointField::FLOAT32);
    EXPECT_EQ(msg.fields[i].count, 1u);
  }
  EXPECT_EQ(msg.point_step, kPackedStepXYZI);
  EXPECT_EQ(msg.width, 7u);
  EXPECT_EQ(msg.height, 1u);
  EXPECT_EQ(msg.row_step, kPackedStepXYZI * 7);
  EXPECT_EQ(msg.data.size(), kPackedStepXYZI * 7);
  EXPECT_FALSE(msg.is_bigendian);

  sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), i(msg, "intensity");
  for (size_t k = 0; k < cloud.size(); ++k, ++x, ++i) {
    EXPECT_EQ(x[0], cloud[k].x);
    EXPECT_EQ(x[1], cloud[k].y);
    EXPECT_EQ(x[2], cloud[k].z);
    EXPECT_EQ(*i, cloud[k].intensity);
  }
}

TEST_F(CloudCodec, PackedXyzitKeepsFullTimePrecision)
{
  const CloudT cloud = make_cloud(5);
  PointCloud2 msg;
  cloud_to_msg(cloud, PointTimeField::INCLUDE, msg);

  ASSERT_EQ(msg.fields.size(), 5u);
  EXPECT_EQ(msg.fields[4].name, "time");
  EXPECT_EQ(msg.fields[4].offset, 16u);
  EXPECT_EQ(msg.fields[4].datatype, PointField::FLOAT64);
  EXPECT_EQ(msg.point_step, kPackedStepXYZIT);
  EXPECT_EQ(msg.data.size(), kPackedStepXYZIT * 5);

  sensor_msgs::PointCloud2ConstIterator<double> t(msg, "time");
  for (size_t k = 0; k < cloud.size(); ++k, ++t) {
    EXPECT_EQ(*t, cloud[k].time);
  }
}

TEST_F(CloudCodec, StaleWidthStillGivesConsistentMessage)
{
  CloudT cloud = make_cloud(6);
  cloud.width = 99;  // a filter that forgot to update the dimensions
  cloud.height = 1;
  PointCloud2 msg;
  cloud_to_msg(cloud, PointTimeField::OMIT, msg);
  EXPECT_EQ(static_cast<size_t>(msg.width) * msg.height, cloud.size());
  EXPECT_EQ(msg.data.size(), static_cast<size_t>(msg.row_step) * msg.height);
}

TEST_F(CloudCodec, EmptyCloud)
{
  PointCloud2 msg;
  cloud_to_msg(CloudT(), PointTimeField::INCLUDE, msg);
  EXPECT_EQ(msg.width, 0u);
  EXPECT_TRUE(msg.data.empty());
  EXPECT_EQ(msg.fields.size(), 5u);
}

#if PCL_VERSION_COMPARE(>=, 1, 14, 1)
// Where PCL already packs, the codec must emit exactly PCL's bytes, so Lyrical
// and Rolling output is unchanged by it.
TEST_F(CloudCodec, MatchesPclToRosMsgBytes)
{
  CloudT cloud = make_cloud(1000);
  cloud.is_dense = false;

  PointCloud2 ours, pcl_msg;
  cloud_to_msg(cloud, PointTimeField::INCLUDE, ours);
  pcl::toROSMsg(cloud, pcl_msg);
  EXPECT_EQ(ours.fields, pcl_msg.fields);
  EXPECT_EQ(ours.point_step, pcl_msg.point_step);
  EXPECT_EQ(ours.row_step, pcl_msg.row_step);
  EXPECT_EQ(ours.is_dense, pcl_msg.is_dense);
  EXPECT_EQ(ours.data, pcl_msg.data);

  pcl::PointCloud<pcl::PointXYZI> xyzi;
  pcl::copyPointCloud(cloud, xyzi);
  cloud_to_msg(cloud, PointTimeField::OMIT, ours);
  pcl::toROSMsg(xyzi, pcl_msg);
  EXPECT_EQ(ours.fields, pcl_msg.fields);
  EXPECT_EQ(ours.point_step, pcl_msg.point_step);
  EXPECT_EQ(ours.data, pcl_msg.data);
}
#endif

TEST_F(CloudCodec, FromMsgMatchesPclOrganizedWithRowPadding)
{
  const PointCloud2 msg = make_airy_msg(4, 9, 8);
  CloudT ours, theirs;
  cloud_from_msg(msg, ours);
  pcl::fromROSMsg(msg, theirs);
  expect_same_points(ours, theirs);
  EXPECT_EQ(ours.height, 4u);
}

TEST_F(CloudCodec, FromMsgMatchesPclWhenSourceHasTimeField)
{
  PointCloud2 msg = make_airy_msg(1, 50);
  msg.fields[5].name = "time";  // float64 'time' maps onto PointXYZIT::time
  CloudT ours, theirs;
  cloud_from_msg(msg, ours);
  pcl::fromROSMsg(msg, theirs);
  expect_same_points(ours, theirs);
}

TEST_F(CloudCodec, FromMsgLeavesUnmatchedFieldsZero)
{
  PointCloud2 msg = make_airy_msg(1, 20);
  msg.fields[3].datatype = PointField::UINT16;  // PCL skips a type mismatch
  CloudT ours;
  cloud_from_msg(msg, ours);
  ASSERT_EQ(ours.size(), 20u);
  for (const auto & p : ours) {
    EXPECT_EQ(p.intensity, 0.0f);
    EXPECT_EQ(p.time, 0.0);
  }
}

TEST_F(CloudCodec, RoundTrip)
{
  const CloudT cloud = make_cloud(64);
  PointCloud2 msg;
  cloud_to_msg(cloud, PointTimeField::INCLUDE, msg);
  CloudT back;
  cloud_from_msg(msg, back);
  expect_same_points(cloud, back);
}

}  // namespace
}  // namespace polka
