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

#include "polka/merge_engine/cpu_merge_engine.hpp"
#include <cmath>

namespace polka {

CloudT::Ptr CpuMergeEngine::merge(const std::vector<MergeInput> & sources)
{
  auto output = std::make_shared<CloudT>();
  size_t total = 0;
  for (const auto & src : sources)
    total += src.cloud->size();
  output->resize(total);

  size_t out_idx = 0;
  for (const auto & src : sources) {
    Eigen::Affine3f tf = src.transform.cast<float>();
    for (size_t i = 0; i < src.cloud->size(); ++i) {
      const auto & p = (*src.cloud)[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        continue;
      auto & o = (*output)[out_idx++];
      Eigen::Vector3f out = tf * Eigen::Vector3f(p.x, p.y, p.z);
      o.x = out.x();
      o.y = out.y();
      o.z = out.z();
      o.intensity = p.intensity;
    }
  }

  output->resize(out_idx);
  output->width = static_cast<uint32_t>(out_idx);
  output->height = 1;
  output->is_dense = true;
  return output;
}

}  // namespace polka
