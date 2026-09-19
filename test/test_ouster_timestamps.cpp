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

// End-to-end cover for an Ouster-shaped source: a UINT32 't' field holding
// nanoseconds since header.stamp has to survive detection, the plausibility
// guard and populate_point_time, and come out of the node as per-point times
// that actually differ from each other.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "polka/polka_node.hpp"

using namespace std::chrono_literals;

namespace polka
{

namespace
{

// Nanosecond offsets spanning a 10 Hz scan, as ouster_ros emits them.
const std::vector<uint32_t> kOusterOffsetsNs = {0u, 25000000u, 50000000u, 99000000u};

sensor_msgs::msg::PointCloud2 make_ouster_cloud(const rclcpp::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = "base_link";  // target frame: TF lookup is identity
  msg.header.stamp = stamp;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(kOusterOffsetsNs.size());

  sensor_msgs::PointCloud2Modifier mod(msg);
  mod.setPointCloud2Fields(
    5,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
    "t", 1, sensor_msgs::msg::PointField::UINT32);
  mod.resize(kOusterOffsetsNs.size());

  uint32_t t_offset = 0;
  for (const auto & field : msg.fields) {
    if (field.name == "t") {t_offset = field.offset;}
  }

  sensor_msgs::PointCloud2Iterator<float> ix(msg, "x"), iy(msg, "y"), iz(msg, "z"),
  ii(msg, "intensity");
  for (size_t i = 0; i < kOusterOffsetsNs.size(); ++i) {
    *ix = 1.0f + static_cast<float>(i);
    *iy = 0.0f;
    *iz = 0.0f;
    *ii = 1.0f;
    ++ix; ++iy; ++iz; ++ii;

    std::memcpy(
      msg.data.data() + i * msg.point_step + t_offset,
      &kOusterOffsetsNs[i], sizeof(uint32_t));
  }

  return msg;
}

// A FLOAT32 'time' field holding what a nanosecond value looks like when it is
// mistaken for seconds. The plausibility guard has to refuse this.
sensor_msgs::msg::PointCloud2 make_implausible_cloud(const rclcpp::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = "base_link";
  msg.header.stamp = stamp;
  msg.height = 1;
  msg.width = 2;

  sensor_msgs::PointCloud2Modifier mod(msg);
  mod.setPointCloud2Fields(
    5,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
    "time", 1, sensor_msgs::msg::PointField::FLOAT32);
  mod.resize(2);

  sensor_msgs::PointCloud2Iterator<float> ix(msg, "x"), iy(msg, "y"), iz(msg, "z"),
  ii(msg, "intensity"), it(msg, "time");
  const float times[2] = {0.0f, 1e8f};
  for (size_t i = 0; i < 2; ++i) {
    *ix = 1.0f + static_cast<float>(i);
    *iy = 0.0f;
    *iz = 0.0f;
    *ii = 1.0f;
    *it = times[i];
    ++ix; ++iy; ++iz; ++ii; ++it;
  }

  return msg;
}

}  // namespace

class OusterTimestampTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides(
      {
        rclcpp::Parameter("source_names", std::vector<std::string>{"s1"}),
        rclcpp::Parameter("sources.s1.topic", "/ouster_points"),
        rclcpp::Parameter("enable_gpu", false),
        rclcpp::Parameter("output_frame_id", "base_link"),
        rclcpp::Parameter("outputs.cloud.topic", "/merged_test"),
        rclcpp::Parameter("output_rate", 20.0),
        rclcpp::Parameter("source_timeout", 5.0),
        rclcpp::Parameter("source_stale_reuse_window", 6.0),
        // Emit the per-point 'time' field, raw Unix seconds so the assertion
        // does not depend on which stamp the output tick happens to pick.
        rclcpp::Parameter("point_timestamps.enabled", true),
        rclcpp::Parameter("point_timestamps.mode", std::string("absolute")),
      });
    node_ = std::make_shared<PolkaNode>(opts);
    helper_ = std::make_shared<rclcpp::Node>("ouster_helper");
    exec_.add_node(node_);
    exec_.add_node(helper_);
  }

  void TearDown() override
  {
    exec_.remove_node(node_);
    exec_.remove_node(helper_);
  }

  template<typename Pred>
  bool spin_until(Pred pred, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (pred()) {return true;}
      exec_.spin_some();
      std::this_thread::sleep_for(2ms);
    }
    return pred();
  }

  // Publishes clouds built by 'make' with an advancing stamp (so
  // suppress_duplicate_timestamps does not swallow the tick) until the node
  // emits a merged cloud. Null if nothing arrived in time.
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pump(
    const std::function<sensor_msgs::msg::PointCloud2(const rclcpp::Time &)> & make)
  {
    auto pub = helper_->create_publisher<sensor_msgs::msg::PointCloud2>("/ouster_points", 10);

    sensor_msgs::msg::PointCloud2::ConstSharedPtr received;
    auto sub = helper_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/merged_test", 10,
      [&received](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {received = msg;});

    if (!spin_until([&] {return helper_->count_subscribers("/ouster_points") > 0;}, 2000ms)) {
      return nullptr;
    }

    rclcpp::Time stamp = helper_->now();
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!received && std::chrono::steady_clock::now() < deadline) {
      pub->publish(make(stamp));
      stamp = stamp + rclcpp::Duration(0, 100000000);  // +100 ms
      exec_.spin_some();
      std::this_thread::sleep_for(20ms);
    }

    return received;
  }

  static std::vector<double> point_times(const sensor_msgs::msg::PointCloud2 & msg)
  {
    std::vector<double> times;
    for (sensor_msgs::PointCloud2ConstIterator<double> it(msg, "time");
      it != it.end(); ++it)
    {
      times.push_back(*it);
    }
    return times;
  }

  rclcpp::executors::SingleThreadedExecutor exec_;
  std::shared_ptr<PolkaNode> node_;
  std::shared_ptr<rclcpp::Node> helper_;
};

TEST_F(OusterTimestampTest, Uint32NanosecondFieldReachesTheOutput)
{
  auto received = pump(make_ouster_cloud);
  ASSERT_TRUE(received) << "no merged cloud was published";

  bool has_time_field = false;
  for (const auto & field : received->fields) {
    if (field.name == "time") {has_time_field = true;}
  }
  ASSERT_TRUE(has_time_field) << "output carries no per-point 'time' field";

  const auto times = point_times(*received);
  ASSERT_EQ(times.size(), kOusterOffsetsNs.size());

  // The regression: before UINT32 was accepted every point inherited the header
  // stamp, so all of these were identical.
  const double span = times.back() - times.front();
  const double expected_span =
    static_cast<double>(kOusterOffsetsNs.back() - kOusterOffsetsNs.front()) * 1e-9;
  EXPECT_NEAR(span, expected_span, 1e-6) << "per-point times are flat or mis-scaled";

  // Spacing must match the nanosecond offsets point for point, which pins the
  // units rather than merely proving the values differ.
  for (size_t i = 1; i < times.size(); ++i) {
    const double gap = times[i] - times[i - 1];
    const double expected_gap =
      static_cast<double>(kOusterOffsetsNs[i] - kOusterOffsetsNs[i - 1]) * 1e-9;
    EXPECT_NEAR(gap, expected_gap, 1e-6) << "gap " << i << " is wrong";
  }
}

TEST_F(OusterTimestampTest, ImplausibleTimesFallBackToTheHeaderStamp)
{
  // Covers the coupling called out in the design: tripping the guard clears
  // has_timestamp_field_, which disables per-point timestamp passthrough too.
  auto received = pump(make_implausible_cloud);
  ASSERT_TRUE(received) << "no merged cloud was published";

  const auto times = point_times(*received);
  ASSERT_EQ(times.size(), 2u);

  // Guard tripped, so every point inherits the header stamp instead of the
  // 1e8 s offset the field claimed.
  EXPECT_DOUBLE_EQ(times[0], times[1]);
  EXPECT_LT(std::abs(times[0] - rclcpp::Time(received->header.stamp).seconds()), 1.0);
}

}  // namespace polka

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
