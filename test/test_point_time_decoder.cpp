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

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "polka/input/point_time_decoder.hpp"

namespace polka
{
namespace
{

// Scan start, chosen to look like a real Unix stamp so the absolute-vs-offset
// decision is exercised the way it is in the field.
constexpr double kHeaderSec = 1750000000.0;

// Builds a cloud whose per-point time field has the given name and datatype.
// 'raw_times' are written verbatim in the field's own units - the point of these
// tests is what the decoder makes of the raw bytes.
sensor_msgs::msg::PointCloud2 make_timed_cloud(
  const std::string & time_name, uint8_t time_datatype,
  const std::vector<double> & raw_times)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = "lidar";
  msg.height = 1;
  msg.width = static_cast<uint32_t>(raw_times.size());

  sensor_msgs::PointCloud2Modifier mod(msg);
  mod.setPointCloud2Fields(
    5,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
    time_name.c_str(), 1, time_datatype);
  mod.resize(raw_times.size());

  uint32_t time_offset = 0;
  for (const auto & field : msg.fields) {
    if (field.name == time_name) {time_offset = field.offset;}
  }

  for (size_t i = 0; i < raw_times.size(); ++i) {
    uint8_t * pt = msg.data.data() + i * msg.point_step;
    switch (time_datatype) {
      case sensor_msgs::msg::PointField::UINT32: {
          const uint32_t v = static_cast<uint32_t>(raw_times[i]);
          std::memcpy(pt + time_offset, &v, sizeof(v));
          break;
        }
      case sensor_msgs::msg::PointField::FLOAT64: {
          const double v = raw_times[i];
          std::memcpy(pt + time_offset, &v, sizeof(v));
          break;
        }
      default: {
          const float v = static_cast<float>(raw_times[i]);
          std::memcpy(pt + time_offset, &v, sizeof(v));
          break;
        }
    }
  }

  return msg;
}

// dt of point i, for a cloud the decoder has already been matched against.
double dt_of(
  const PointTimeDecoder & decoder, const sensor_msgs::msg::PointCloud2 & msg, size_t i)
{
  return decoder.dt(msg.data.data() + i * msg.point_step, kHeaderSec);
}

}  // namespace

// --- Ouster: 't', UINT32, nanoseconds since header.stamp ---------------------

TEST(PointTimeDecoder, DetectsOusterUint32Field)
{
  const auto msg = make_timed_cloud("t", sensor_msgs::msg::PointField::UINT32, {0.0});

  std::string matched;
  const auto decoder = detect_point_time_field(msg.fields, "auto", &matched);

  ASSERT_TRUE(decoder.has_value());
  EXPECT_EQ(matched, "t");
  EXPECT_EQ(decoder->datatype, sensor_msgs::msg::PointField::UINT32);
}

TEST(PointTimeDecoder, OusterNanosecondsDecodeToSeconds)
{
  // 0 ms, 50 ms and 99.9 ms into a 10 Hz scan.
  const auto msg = make_timed_cloud(
    "t", sensor_msgs::msg::PointField::UINT32, {0.0, 50e6, 99.9e6});

  const auto decoder = detect_point_time_field(msg.fields, "auto");
  ASSERT_TRUE(decoder.has_value());

  EXPECT_NEAR(dt_of(*decoder, msg, 0), 0.0, 1e-9);
  EXPECT_NEAR(dt_of(*decoder, msg, 1), 0.05, 1e-9);
  EXPECT_NEAR(dt_of(*decoder, msg, 2), 0.0999, 1e-9);
}

TEST(PointTimeDecoder, ExplicitHintStillResolvesUint32)
{
  // Regression for the "just set deskew_timestamp_field" non-fix: the hint
  // overrides the name only, so it never rescued a rejected datatype.
  const auto msg = make_timed_cloud("t", sensor_msgs::msg::PointField::UINT32, {50e6});

  const auto decoder = detect_point_time_field(msg.fields, "t");

  ASSERT_TRUE(decoder.has_value());
  EXPECT_NEAR(dt_of(*decoder, msg, 0), 0.05, 1e-9);
}

TEST(PointTimeDecoder, OusterMaxUint32StaysWithinGuard)
{
  // A UINT32 ns field is capped by its own type at 2^32 ns = 4.295 s, so it can
  // never trip the plausibility guard however corrupt the payload is.
  const auto msg = make_timed_cloud(
    "t", sensor_msgs::msg::PointField::UINT32, {static_cast<double>(UINT32_MAX)});

  const auto decoder = detect_point_time_field(msg.fields, "auto");
  ASSERT_TRUE(decoder.has_value());

  const double max_abs = point_time_max_abs_dt(
    *decoder, msg.data.data(), msg.point_step, 1, kHeaderSec);

  EXPECT_LT(max_abs, 4.3);
  EXPECT_LT(max_abs, kMaxPlausibleAbsDtSec);
}

// --- Existing sensors: these must not move ----------------------------------

TEST(PointTimeDecoder, RoboSenseFloat64AbsoluteUnchanged)
{
  const auto msg = make_timed_cloud(
    "timestamp", sensor_msgs::msg::PointField::FLOAT64,
    {kHeaderSec, kHeaderSec + 0.05});

  std::string matched;
  const auto decoder = detect_point_time_field(msg.fields, "auto", &matched);
  ASSERT_TRUE(decoder.has_value());
  EXPECT_EQ(matched, "timestamp");

  EXPECT_NEAR(dt_of(*decoder, msg, 0), 0.0, 1e-6);
  EXPECT_NEAR(dt_of(*decoder, msg, 1), 0.05, 1e-6);
}

TEST(PointTimeDecoder, VelodyneFloat32OffsetUnchanged)
{
  const auto msg = make_timed_cloud(
    "time", sensor_msgs::msg::PointField::FLOAT32, {0.0, 0.05});

  std::string matched;
  const auto decoder = detect_point_time_field(msg.fields, "auto", &matched);
  ASSERT_TRUE(decoder.has_value());
  EXPECT_EQ(matched, "time");

  EXPECT_NEAR(dt_of(*decoder, msg, 0), 0.0, 1e-6);
  EXPECT_NEAR(dt_of(*decoder, msg, 1), 0.05, 1e-6);
}

TEST(PointTimeDecoder, NoTimeFieldIsNotDetected)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.height = 1;
  msg.width = 1;
  sensor_msgs::PointCloud2Modifier mod(msg);
  mod.setPointCloud2Fields(
    3,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  mod.resize(1);

  EXPECT_FALSE(detect_point_time_field(msg.fields, "auto").has_value());
}

// --- Plausibility guard ------------------------------------------------------

TEST(PointTimeDecoder, ImplausibleOffsetIsCaught)
{
  // What a nanosecond field read as seconds looks like: 100 ms -> 1e8 s.
  const auto msg = make_timed_cloud(
    "time", sensor_msgs::msg::PointField::FLOAT32, {0.0, 1e8});

  const auto decoder = detect_point_time_field(msg.fields, "auto");
  ASSERT_TRUE(decoder.has_value());

  const double max_abs = point_time_max_abs_dt(
    *decoder, msg.data.data(), msg.point_step, 2, kHeaderSec);

  EXPECT_GT(max_abs, kMaxPlausibleAbsDtSec);
}

TEST(PointTimeDecoder, ClockSkewIsCaughtByMaxAbsButNotBySpan)
{
  // Absolute stamps running 100 s ahead of the header. The span across the scan
  // is only 0.05 s, so a span-based check sails through; max|dt| does not.
  const auto msg = make_timed_cloud(
    "timestamp", sensor_msgs::msg::PointField::FLOAT64,
    {kHeaderSec + 100.0, kHeaderSec + 100.05});

  const auto decoder = detect_point_time_field(msg.fields, "auto");
  ASSERT_TRUE(decoder.has_value());

  const double max_abs = point_time_max_abs_dt(
    *decoder, msg.data.data(), msg.point_step, 2, kHeaderSec);

  const double span = dt_of(*decoder, msg, 1) - dt_of(*decoder, msg, 0);
  EXPECT_LT(span, 1.0) << "span is offset-invariant, which is exactly the blind spot";
  EXPECT_GT(max_abs, kMaxPlausibleAbsDtSec);
}

TEST(PointTimeDecoder, NormalScanClearsTheGuard)
{
  const auto msg = make_timed_cloud(
    "t", sensor_msgs::msg::PointField::UINT32, {0.0, 50e6, 100e6});

  const auto decoder = detect_point_time_field(msg.fields, "auto");
  ASSERT_TRUE(decoder.has_value());

  const double max_abs = point_time_max_abs_dt(
    *decoder, msg.data.data(), msg.point_step, 3, kHeaderSec);

  EXPECT_NEAR(max_abs, 0.1, 1e-9);
  EXPECT_LT(max_abs, kMaxPlausibleAbsDtSec);
}

TEST(PointTimeDecoder, MaxAbsDtSamplesTheEndOfALargeScan)
{
  // The extreme sits at the last point, which a stride can step over.
  std::vector<double> times(2000);
  for (size_t i = 0; i < times.size(); ++i) {
    times[i] = static_cast<double>(i) * 50000.0;  // ns, 0 .. ~0.1 s
  }
  const auto msg = make_timed_cloud("t", sensor_msgs::msg::PointField::UINT32, times);

  const auto decoder = detect_point_time_field(msg.fields, "auto");
  ASSERT_TRUE(decoder.has_value());

  const double max_abs = point_time_max_abs_dt(
    *decoder, msg.data.data(), msg.point_step, times.size(), kHeaderSec);

  EXPECT_NEAR(max_abs, times.back() * 1e-9, 1e-9);
}

}  // namespace polka
