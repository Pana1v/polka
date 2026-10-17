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

#include <cuda_runtime.h>
#include <gtest/gtest.h>

#include <vector>

#include "polka/merge_engine/cuda_merge_engine.hpp"

namespace polka
{

namespace
{

// One point inside the default range limits (0.1 to 100 m), one closer, one beyond.
std::vector<MergeInput> three_ranges(bool range_enabled)
{
  auto cloud = std::make_shared<CloudT>();
  for (float x : {0.05f, 10.0f, 150.0f}) {
    PointT p;
    p.x = x;
    p.y = 0.0f;
    p.z = 0.0f;
    p.intensity = 1.0f;
    p.time = 0.0;
    cloud->push_back(p);
  }
  MergeInput in;
  in.cloud = cloud;
  in.transform = Eigen::Isometry3d::Identity();
  in.filter_params.range_filter_enabled = range_enabled;
  in.filter_params.min_range = 0.1;
  in.filter_params.max_range = 100.0;
  in.filter_params.compute_squared_ranges();
  return {in};
}

class CudaRangeFilter : public ::testing::Test
{
protected:
  void SetUp() override
  {
    int devices = 0;
    if (cudaGetDeviceCount(&devices) != cudaSuccess || devices == 0) {
      GTEST_SKIP() << "no CUDA device";
    }
    MergeConfig config;
    config.sources.resize(1);
    engine_ = std::make_unique<CudaMergeEngine>(config);
  }

  std::unique_ptr<CudaMergeEngine> engine_;
};

TEST_F(CudaRangeFilter, DisabledKeepsEveryPoint)
{
  auto result = engine_->merge_pipeline(three_ranges(false), PipelineConfig{});
  EXPECT_EQ(result.cloud->size(), 3u);
}

TEST_F(CudaRangeFilter, EnabledKeepsOnlyPointsInRange)
{
  auto result = engine_->merge_pipeline(three_ranges(true), PipelineConfig{});
  ASSERT_EQ(result.cloud->size(), 1u);
  EXPECT_FLOAT_EQ(result.cloud->points[0].x, 10.0f);
}

}  // namespace

}  // namespace polka
