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

#include "polka/filters/angular_filter.hpp"
#include <cmath>

namespace polka
{

AngularFilter::AngularFilter(
  const std::vector<std::pair<double, double>> & ranges_deg, bool invert)
: invert_(invert)
{
  bounds_.reserve(ranges_deg.size());
  for (const auto & r : ranges_deg) {
    double lo_rad = r.first * M_PI / 180.0;
    double hi_rad = r.second * M_PI / 180.0;
    double span = (r.first <= r.second) ? (r.second - r.first) : (360.0 - r.first + r.second);
    bounds_.push_back(
      {
        static_cast<float>(std::cos(lo_rad)), static_cast<float>(std::sin(lo_rad)),
        static_cast<float>(std::cos(hi_rad)), static_cast<float>(std::sin(hi_rad)),
        span > 180.0
      });
  }
}

bool AngularFilter::in_ranges(float x, float y) const
{
  for (const auto & b : bounds_) {
    float cross_lo = b.lo_x * y - b.lo_y * x;
    float cross_hi = b.hi_x * y - b.hi_y * x;
    bool inside = b.wide ? (cross_lo >= 0.0f || cross_hi <= 0.0f) :
      (cross_lo >= 0.0f && cross_hi <= 0.0f);
    if (inside) {return true;}
  }
  return false;
}

void AngularFilter::apply(CloudT & cloud, const std::string & /*frame_id*/)
{
  size_t j = 0;
  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto & p = cloud[i];
    bool match = in_ranges(p.x, p.y);
    bool keep = invert_ ? !match : match;
    if (keep) {
      cloud[j++] = p;
    }
  }
  cloud.resize(j);
  cloud.width = static_cast<uint32_t>(j);
  cloud.height = 1;
  cloud.is_dense = true;
}

}  // namespace polka
