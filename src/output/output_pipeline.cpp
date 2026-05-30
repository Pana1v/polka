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

#include "polka/output/output_pipeline.hpp"
#include "polka/filters/filter_chain.hpp"
#include "polka/filters/box_filter.hpp"

#include <pcl/filters/voxel_grid.h>
// PointXYZIT is a custom point type, so VoxelGrid/PCLBase are not pre-instantiated
// in libpcl; pull in the template implementations to instantiate them here.
#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>

namespace polka {

void OutputPipeline::configure(const CloudOutputConfig & cfg)
{
  config_ = cfg;
  build_filters();
}

void OutputPipeline::rebuild_filters()
{
  build_filters();
}

void OutputPipeline::build_filters()
{
  filters_ = build_filter_chain(config_.filters);
  if (config_.self_filter.enabled) {
    for (const auto & eb : config_.self_filter.boxes)
      filters_.push_back(std::make_unique<BoxFilter>(eb.min, eb.max, true));
  }
}

void OutputPipeline::process(CloudT & cloud, const std::string & frame_id) const
{
  for (const auto & filter : filters_)
    filter->apply(cloud, frame_id);

  if (config_.height_cap.enabled) {
    const float z_min = static_cast<float>(config_.height_cap.z_min);
    const float z_max = static_cast<float>(config_.height_cap.z_max);
    size_t j = 0;
    for (size_t i = 0; i < cloud.size(); ++i) {
      if (cloud[i].z >= z_min && cloud[i].z <= z_max)
        cloud[j++] = cloud[i];
    }
    cloud.resize(j);
    cloud.width = static_cast<uint32_t>(j);
    cloud.height = 1;
    cloud.is_dense = true;
  }

  if (config_.voxel.enabled) {
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud.makeShared());
    vg.setLeafSize(config_.voxel.leaf_x, config_.voxel.leaf_y, config_.voxel.leaf_z);
    CloudT filtered;
    vg.filter(filtered);
    cloud = std::move(filtered);
  }
}

PipelineConfig OutputPipeline::to_pipeline_config(
  bool scan_enabled, const FlattenParams & flatten) const
{
  PipelineConfig pcfg;
  pcfg.output_filters = config_.filters;
  pcfg.self_filter_enabled = config_.self_filter.enabled;
  pcfg.self_filter_boxes = config_.self_filter.boxes;
  pcfg.height_cap = config_.height_cap;
  pcfg.voxel = config_.voxel;
  pcfg.scan_enabled = scan_enabled;
  if (scan_enabled)
    pcfg.flatten = flatten;
  return pcfg;
}

}  // namespace polka
