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

#include "polka/filters/range_filter.hpp"

namespace polka {

RangeFilter::RangeFilter(double min_range, double max_range)
: min_range_sq_(static_cast<float>(min_range * min_range)),
  max_range_sq_(static_cast<float>(max_range * max_range))
{
}

void RangeFilter::apply(CloudT & cloud, const std::string & /*frame_id*/)
{
  size_t j = 0;
  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto & p = cloud[i];
    float r2 = p.x * p.x + p.y * p.y + p.z * p.z;
    if (r2 >= min_range_sq_ && r2 <= max_range_sq_) {
      cloud[j++] = p;
    }
  }
  cloud.resize(j);
  cloud.width = static_cast<uint32_t>(j);
  cloud.height = 1;
  cloud.is_dense = true;
}

}  // namespace polka
