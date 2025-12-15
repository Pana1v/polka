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

#ifndef POLKA__MERGE_ENGINE__I_MERGE_ENGINE_HPP
#define POLKA__MERGE_ENGINE__I_MERGE_ENGINE_HPP

#include "polka/types.hpp"
#include <vector>
#include <Eigen/Geometry>

namespace polka {

struct MergeInput {
  CloudT::ConstPtr cloud;
  Eigen::Isometry3d transform;
  FilterParams filter_params;
};

// Full GPU pipeline configuration - captures everything needed post-merge
struct PipelineConfig {
  FilterParams output_filters;
  bool self_filter_enabled = false;
  std::vector<ExclusionBox> self_filter_boxes;
  HeightCapConfig height_cap;
  VoxelConfig voxel;
  bool scan_enabled = false;
  FlattenParams flatten;
};

struct PipelineResult {
  CloudT::Ptr cloud;
  std::vector<float> scan_ranges;  // non-empty only when GPU produces scan
};

class IMergeEngine {
public:
  virtual CloudT::Ptr merge(const std::vector<MergeInput> & sources) = 0;
  virtual bool is_gpu() const = 0;

  // Full pipeline: merge + output filters + voxel + flatten, all on GPU.
  // Default implementation falls back to merge-only (caller handles rest on CPU).
  virtual PipelineResult merge_pipeline(
    const std::vector<MergeInput> & sources,
    const PipelineConfig & /*config*/)
  {
    return {merge(sources), {}};
  }

  virtual ~IMergeEngine() = default;
};

}  // namespace polka

#endif  // POLKA__MERGE_ENGINE__I_MERGE_ENGINE_HPP
