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

#ifndef POLKA__FILTERS__ANGULAR_FILTER_HPP_
#define POLKA__FILTERS__ANGULAR_FILTER_HPP_

#include <string>
#include <vector>
#include <utility>

#include "polka/filters/i_filter.hpp"

namespace polka
{

class AngularFilter : public IFilter
{
public:
  AngularFilter(const std::vector<std::pair<double, double>> & ranges_deg, bool invert);
  void apply(CloudT & cloud, const std::string & frame_id) override;

private:
  // Precomputed bound vectors for a cross-product half-plane test, one per range:
  // (cos(lo), sin(lo), cos(hi), sin(hi)). Mirrors cuda_merge_engine.cu's
  // pass_angular(), which avoids atan2f per point; ported here so the CPU path
  // gets the same win (measured ~3x: PERF_AUDIT.md finding #4).
  struct Bound { float lo_x, lo_y, hi_x, hi_y; bool wide; };
  std::vector<Bound> bounds_;
  bool invert_;
  bool in_ranges(float x, float y) const;
};

}  // namespace polka

#endif  // POLKA__FILTERS__ANGULAR_FILTER_HPP_
