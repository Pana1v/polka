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

#include <cstdint>
#include <vector>

#include "polka/merge_engine/cuda_merge_engine.hpp"
#include "polka/util/cloud_codec.hpp"

namespace polka
{

namespace
{

constexpr double kStamp = 1750000000.25;

// An odd count, so 24-byte packed points start on 8- but not 16-byte boundaries.
std::vector<MergeInput> one_source(size_t n)
{
  auto cloud = std::make_shared<CloudT>();
  for (size_t i = 0; i < n; ++i) {
    PointT p;
    p.x = 0.5f + 0.01f * static_cast<float>(i);
    p.y = -2.0f + 0.003f * static_cast<float>(i);
    p.z = 0.25f * static_cast<float>(i % 7);
    p.intensity = static_cast<float>(i % 255);
    p.time = kStamp - 0.1 + 1e-6 * static_cast<double>(i);
    cloud->push_back(p);
  }
  MergeInput in;
  in.cloud = cloud;
  in.transform = Eigen::Isometry3d::Identity();
  in.transform.translation() << 1.0, 0.0, 0.5;
  return {in};
}

class CudaPack : public ::testing::Test
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

  // Bytes pack_result() writes for the last pipeline result.
  std::vector<uint8_t> pack(size_t count, PointTimeField time, double time_base)
  {
    const size_t step = time == PointTimeField::INCLUDE ? kPackedStepXYZIT : kPackedStepXYZI;
    std::vector<uint8_t> host(count * step);
    uint8_t * dev = nullptr;
    EXPECT_EQ(cudaMalloc(&dev, host.size()), cudaSuccess);
    engine_->pack_result(dev, time, time_base);
    EXPECT_EQ(cudaStreamSynchronize(engine_->stream()), cudaSuccess);
    EXPECT_EQ(cudaMemcpy(host.data(), dev, host.size(), cudaMemcpyDeviceToHost), cudaSuccess);
    cudaFree(dev);
    return host;
  }

  std::unique_ptr<CudaMergeEngine> engine_;
};

// The device path must publish exactly what the host path would, including the
// OFFSET rebase of 'time' that PolkaNode::rebase_point_time applies on the host.
TEST_F(CudaPack, MatchesHostCodecWithTimeOffset)
{
  auto result = engine_->merge_pipeline(one_source(1001), PipelineConfig{});
  ASSERT_EQ(result.cloud->size(), 1001u);
  const auto device = pack(result.cloud->size(), PointTimeField::INCLUDE, kStamp);

  for (auto & p : *result.cloud) {
    p.time -= kStamp;
  }
  sensor_msgs::msg::PointCloud2 host;
  cloud_to_msg(*result.cloud, PointTimeField::INCLUDE, host);
  EXPECT_EQ(device, std::vector<uint8_t>(host.data.begin(), host.data.end()));
}

TEST_F(CudaPack, MatchesHostCodecWithoutTime)
{
  auto result = engine_->merge_pipeline(one_source(333), PipelineConfig{});
  const auto device = pack(result.cloud->size(), PointTimeField::OMIT, 0.0);

  sensor_msgs::msg::PointCloud2 host;
  cloud_to_msg(*result.cloud, PointTimeField::OMIT, host);
  EXPECT_EQ(device, std::vector<uint8_t>(host.data.begin(), host.data.end()));
}

}  // namespace

}  // namespace polka
