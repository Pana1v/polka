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

#include <array>
#include <chrono>
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

sensor_msgs::msg::PointCloud2 make_cloud(
  const std::vector<std::array<float, 3>> & points, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = "base_link";  // target frame: TF lookup is identity
  msg.header.stamp = stamp;
  msg.height = 1;
  msg.width = points.size();
  sensor_msgs::PointCloud2Modifier mod(msg);
  mod.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  mod.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> ix(msg, "x"), iy(msg, "y"), iz(msg, "z"),
  ii(msg, "intensity");
  for (const auto & p : points) {
    *ix = p[0]; *iy = p[1]; *iz = p[2]; *ii = 1.0f;
    ++ix; ++iy; ++iz; ++ii;
  }
  return msg;
}

}  // namespace

class RuntimeReconfigureTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides(
      {
        rclcpp::Parameter("source_names", std::vector<std::string>{"s1", "s2"}),
        rclcpp::Parameter("sources.s1.topic", "/t1"),
        rclcpp::Parameter("sources.s2.topic", "/t2"),
        rclcpp::Parameter("enable_gpu", false),
        rclcpp::Parameter("outputs.cloud.topic", "/merged_test"),
        rclcpp::Parameter("output_rate", 20.0),
        // Generous so a slow CI machine cannot stale-out mid-test.
        rclcpp::Parameter("source_timeout", 5.0),
      });
    node_ = std::make_shared<PolkaNode>(opts);
    helper_ = std::make_shared<rclcpp::Node>("helper");
    exec_.add_node(node_);
    exec_.add_node(helper_);
  }

  void TearDown() override
  {
    exec_.remove_node(node_);
    exec_.remove_node(helper_);
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      exec_.spin_some();
      std::this_thread::sleep_for(2ms);
    }
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

  rclcpp::executors::SingleThreadedExecutor exec_;
  std::shared_ptr<PolkaNode> node_;
  std::shared_ptr<rclcpp::Node> helper_;
};

TEST_F(RuntimeReconfigureTest, AddPendingSourceThenActivateThenRemove)
{
  // Add a third name without a topic: accepted, but no subscription yet.
  auto result = node_->set_parameter(
    rclcpp::Parameter("source_names", std::vector<std::string>{"s1", "s2", "extra"}));
  ASSERT_TRUE(result.successful) << result.reason;
  spin_for(100ms);
  EXPECT_EQ(helper_->count_subscribers("/t3"), 0u);

  // Setting its topic activates it: the subscription must appear on the graph.
  result = node_->set_parameter(rclcpp::Parameter("sources.extra.topic", "/t3"));
  ASSERT_TRUE(result.successful) << result.reason;
  ASSERT_TRUE(
    spin_until(
      [&]() {return helper_->count_subscribers("/t3") == 1u;}, 2000ms));

  // Removing the name tears the subscription down again.
  result = node_->set_parameter(
    rclcpp::Parameter("source_names", std::vector<std::string>{"s1", "s2"}));
  ASSERT_TRUE(result.successful) << result.reason;
  ASSERT_TRUE(
    spin_until(
      [&]() {return helper_->count_subscribers("/t3") == 0u;}, 2000ms));
}

TEST_F(RuntimeReconfigureTest, TopicChangeRecreatesSubscription)
{
  ASSERT_EQ(helper_->count_subscribers("/t1"), 1u);
  auto result = node_->set_parameter(rclcpp::Parameter("sources.s1.topic", "/t1b"));
  ASSERT_TRUE(result.successful) << result.reason;
  ASSERT_TRUE(
    spin_until(
      [&]() {
        return helper_->count_subscribers("/t1") == 0u &&
        helper_->count_subscribers("/t1b") == 1u;
      }, 2000ms));
}

TEST_F(RuntimeReconfigureTest, InvalidSetIsRejectedWithReason)
{
  auto result = node_->set_parameter(
    rclcpp::Parameter("sources.s1.filters.range.min", -1.0));
  EXPECT_FALSE(result.successful);
  EXPECT_NE(result.reason.find("min_range"), std::string::npos) << result.reason;
  // The rejected value must not have been committed.
  EXPECT_DOUBLE_EQ(
    node_->get_parameter("sources.s1.filters.range.min").as_double(), 0.1);
}

TEST_F(RuntimeReconfigureTest, EnableGpuIsReadOnly)
{
  // rclcpp itself rejects sets of read-only parameters (failed result with
  // a reason, no exception), before polka's validation callback runs.
  auto result = node_->set_parameter(rclcpp::Parameter("enable_gpu", true));
  EXPECT_FALSE(result.successful);
  EXPECT_NE(result.reason.find("read-only"), std::string::npos) << result.reason;
  EXPECT_FALSE(node_->get_parameter("enable_gpu").as_bool());
}

// End-to-end regression for the Humble stale-read bug: the FIRST runtime set
// of a filter bound must affect the very next merged cloud. Under the old
// single-phase reconfigure the callback re-read pre-commit storage, so the
// first set silently applied the previous values.
TEST_F(RuntimeReconfigureTest, FirstFilterSetAffectsNextOutput)
{
  auto pub = helper_->create_publisher<sensor_msgs::msg::PointCloud2>("/t1", 10);

  sensor_msgs::msg::PointCloud2::SharedPtr merged;
  auto sub = helper_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/merged_test", 10,
    [&merged](sensor_msgs::msg::PointCloud2::SharedPtr msg) {merged = msg;});

  // Baseline: both points (2 m and 6 m from origin) survive.
  pub->publish(make_cloud({{{2.0f, 0.0f, 0.0f}}, {{6.0f, 0.0f, 0.0f}}}, node_->now()));
  ASSERT_TRUE(spin_until([&]() {return merged != nullptr;}, 3000ms));
  EXPECT_EQ(merged->width, 2u);

  // First-ever filter set: enable the range filter and cap it at 4 m.
  auto result = node_->set_parameters_atomically(
    {
      rclcpp::Parameter("sources.s1.filters.range.enabled", true),
      rclcpp::Parameter("sources.s1.filters.range.max", 4.0),
    });
  ASSERT_TRUE(result.successful) << result.reason;
  spin_for(100ms);  // let the deferred apply run

  merged.reset();
  pub->publish(make_cloud({{{2.0f, 0.0f, 0.0f}}, {{6.0f, 0.0f, 0.0f}}}, node_->now()));
  ASSERT_TRUE(spin_until([&]() {return merged != nullptr;}, 3000ms));
  EXPECT_EQ(merged->width, 1u);  // the 6 m point is gone on the FIRST set
}

// Defect regression: disabling motion compensation at runtime used to leave
// adapters holding a getter that dereferenced the reset ImuBuffer.
TEST_F(RuntimeReconfigureTest, MotionCompToggleThenTrafficDoesNotCrash)
{
  auto result = node_->set_parameters_atomically(
    {
      rclcpp::Parameter("motion_compensation.enabled", true),
      rclcpp::Parameter("motion_compensation.imu_topic", "/imu"),
    });
  ASSERT_TRUE(result.successful) << result.reason;
  spin_for(100ms);

  result = node_->set_parameter(rclcpp::Parameter("motion_compensation.enabled", false));
  ASSERT_TRUE(result.successful) << result.reason;
  spin_for(100ms);

  auto pub = helper_->create_publisher<sensor_msgs::msg::PointCloud2>("/t1", 10);
  pub->publish(make_cloud({{{1.0f, 0.0f, 0.0f}}}, node_->now()));
  spin_for(300ms);  // would crash here before the fix
  SUCCEED();
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
