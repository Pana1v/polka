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

#include "polka/filters/box_filter.hpp"

namespace polka {

BoxFilter::BoxFilter(const Eigen::Vector3d & box_min, const Eigen::Vector3d & box_max,
                     bool invert)
: bx_min_(static_cast<float>(box_min.x())), bx_max_(static_cast<float>(box_max.x())),
  by_min_(static_cast<float>(box_min.y())), by_max_(static_cast<float>(box_max.y())),
  bz_min_(static_cast<float>(box_min.z())), bz_max_(static_cast<float>(box_max.z())),
  invert_(invert)
{
}

void BoxFilter::apply(CloudT & cloud, const std::string & /*frame_id*/)
{
  size_t j = 0;
  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto & p = cloud[i];
    bool inside = p.x >= bx_min_ && p.x <= bx_max_ &&
                  p.y >= by_min_ && p.y <= by_max_ &&
                  p.z >= bz_min_ && p.z <= bz_max_;
    if (inside != invert_) {
      cloud[j++] = p;
    }
  }
  cloud.resize(j);
  cloud.width = static_cast<uint32_t>(j);
  cloud.height = 1;
  cloud.is_dense = true;
}

}  // namespace polka
