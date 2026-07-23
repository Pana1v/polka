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

#ifndef POLKA__OUTPUT__OUTPUT_PIPELINE_HPP_
#define POLKA__OUTPUT__OUTPUT_PIPELINE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "polka/types.hpp"
#include "polka/filters/i_filter.hpp"
#include "polka/merge_engine/i_merge_engine.hpp"

namespace polka
{

/// CPU post-merge processing pipeline: output filters → height cap → voxel downsample.
/// Also builds PipelineConfig for the GPU engine path.
class OutputPipeline
{
public:
  void configure(const CloudOutputConfig & cfg);

  /// Rebuild filter chain after a reconfigure (same config, new filter objects).
  void rebuild_filters();

  /// Run CPU pipeline in-place on a merged cloud.
  void process(CloudT & cloud, const std::string & frame_id) const;

  /// Build PipelineConfig for the GPU path (merge_pipeline).
  PipelineConfig to_pipeline_config(bool scan_enabled, const FlattenParams & flatten) const;

private:
  void build_filters();

  CloudOutputConfig config_;
  std::vector<std::unique_ptr<IFilter>> filters_;  // range + angular + box + self-filter boxes
};

}  // namespace polka

#endif  // POLKA__OUTPUT__OUTPUT_PIPELINE_HPP_
