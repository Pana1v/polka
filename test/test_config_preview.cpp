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
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

#include "polka/config/config_loader.hpp"

namespace polka {

namespace {

rclcpp::NodeOptions two_source_options()
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
    rclcpp::Parameter("source_names", std::vector<std::string>{"a", "b"}),
    rclcpp::Parameter("sources.a.topic", "/a/points"),
    rclcpp::Parameter("sources.b.topic", "/b/points"),
  });
  return opts;
}

}  // namespace

class ConfigPreviewTest : public ::testing::Test {
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("polka_cfg_test", two_source_options());
    loader_ = std::make_unique<ConfigLoader>(node_.get());
    loader_->load();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<ConfigLoader> loader_;
};

// Regression for the Humble stale-read bug: the on-set callback fires before
// the commit, so validation must read the *proposed* values, not storage.
TEST_F(ConfigPreviewTest, OverlayReadsProposedNotStoredValues)
{
  std::vector<rclcpp::Parameter> proposed = {
    rclcpp::Parameter("sources.a.filters.range.enabled", true),
    rclcpp::Parameter("sources.a.filters.range.max", 4.0),
  };
  auto cfg = loader_->preview(proposed, {"a", "b"});
  ASSERT_EQ(cfg.sources.size(), 2u);
  EXPECT_TRUE(cfg.sources[0].filter_params.range_filter_enabled);
  EXPECT_DOUBLE_EQ(cfg.sources[0].filter_params.max_range, 4.0);
  // Storage is untouched: preview never commits or declares.
  EXPECT_DOUBLE_EQ(
    node_->get_parameter("sources.a.filters.range.max").as_double(), 100.0);
  EXPECT_FALSE(
    node_->get_parameter("sources.a.filters.range.enabled").as_bool());
}

TEST_F(ConfigPreviewTest, RejectionReasonNamesTheOffendingPrefix)
{
  std::vector<rclcpp::Parameter> proposed = {
    rclcpp::Parameter("sources.a.filters.range.min", -1.0),
  };
  try {
    loader_->preview(proposed, {"a", "b"});
    FAIL() << "expected invalid range.min to throw";
  } catch (const std::exception & ex) {
    const std::string reason = ex.what();
    EXPECT_NE(reason.find("sources.a.filters"), std::string::npos) << reason;
    EXPECT_NE(reason.find("min_range"), std::string::npos) << reason;
  }
}

TEST_F(ConfigPreviewTest, EmptySourceListRejected)
{
  EXPECT_THROW(loader_->preview({}, {}), std::exception);
}

TEST_F(ConfigPreviewTest, DuplicateSourceNamesRejected)
{
  try {
    loader_->preview({}, {"a", "a"});
    FAIL() << "expected duplicate names to throw";
  } catch (const std::exception & ex) {
    EXPECT_NE(std::string(ex.what()).find("duplicate"), std::string::npos);
  }
}

TEST_F(ConfigPreviewTest, InvalidTimestampStrategyRejected)
{
  std::vector<rclcpp::Parameter> proposed = {
    rclcpp::Parameter("timestamp_strategy", "bogus"),
  };
  try {
    loader_->preview(proposed, {"a", "b"});
    FAIL() << "expected invalid timestamp_strategy to throw";
  } catch (const std::exception & ex) {
    EXPECT_NE(std::string(ex.what()).find("timestamp_strategy"), std::string::npos);
  }
}

TEST_F(ConfigPreviewTest, InvalidDiagnosticsConfigRejected)
{
  std::vector<rclcpp::Parameter> proposed = {
    rclcpp::Parameter("diagnostics.timing_drift.ewma_alpha", 2.0),
  };
  try {
    loader_->preview(proposed, {"a", "b"});
    FAIL() << "expected invalid ewma_alpha to throw";
  } catch (const std::exception & ex) {
    EXPECT_NE(std::string(ex.what()).find("ewma_alpha"), std::string::npos);
  }
}

// A source just added to source_names has no declared parameters yet; preview
// must fall back to defaults (empty topic == pending) instead of throwing,
// and reload() must then declare it so later sets work.
TEST_F(ConfigPreviewTest, NewSourceIsPendingInPreviewAndDeclaredByReload)
{
  auto cfg = loader_->preview({}, {"a", "b", "extra"});
  ASSERT_EQ(cfg.sources.size(), 3u);
  EXPECT_TRUE(cfg.sources[2].topic.empty());
  EXPECT_FALSE(node_->has_parameter("sources.extra.topic"));

  auto reloaded = loader_->reload({"a", "b", "extra"});
  ASSERT_EQ(reloaded.sources.size(), 3u);
  EXPECT_TRUE(node_->has_parameter("sources.extra.topic"));
}

TEST_F(ConfigPreviewTest, ReloadAllowsShrinkingButNotEmptying)
{
  auto cfg = loader_->reload({"a"});
  EXPECT_EQ(cfg.sources.size(), 1u);
  EXPECT_THROW(loader_->reload({}), std::exception);
}

TEST(ConfigLoaderStartup, EmptyTopicRejectedAtStartup)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
    rclcpp::Parameter("source_names", std::vector<std::string>{"a"}),
    // sources.a.topic deliberately left at its "" default
  });
  auto node = std::make_shared<rclcpp::Node>("polka_cfg_strict", opts);
  ConfigLoader loader(node.get());
  EXPECT_THROW(loader.load(), std::exception);
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
