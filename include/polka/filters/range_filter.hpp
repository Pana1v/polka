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

#ifndef POLKA__FILTERS__RANGE_FILTER_HPP
#define POLKA__FILTERS__RANGE_FILTER_HPP

#include "polka/filters/i_filter.hpp"

namespace polka {

class RangeFilter : public IFilter {
public:
  RangeFilter(double min_range, double max_range);
  void apply(CloudT & cloud, const std::string & frame_id) override;

private:
  float min_range_sq_;
  float max_range_sq_;
};

}  // namespace polka

#endif  // POLKA__FILTERS__RANGE_FILTER_HPP
