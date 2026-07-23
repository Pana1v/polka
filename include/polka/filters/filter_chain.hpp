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

#ifndef POLKA__FILTERS__FILTER_CHAIN_HPP_
#define POLKA__FILTERS__FILTER_CHAIN_HPP_

#include <memory>
#include <vector>

#include "polka/types.hpp"
#include "polka/filters/i_filter.hpp"
#include "polka/filters/range_filter.hpp"
#include "polka/filters/angular_filter.hpp"
#include "polka/filters/box_filter.hpp"

namespace polka
{

/// Build a filter chain from a FilterParams config.
/// Returns an ordered vector: range → angular → box (enabled filters only).
inline std::vector<std::unique_ptr<IFilter>> build_filter_chain(const FilterParams & fp)
{
  std::vector<std::unique_ptr<IFilter>> chain;
  if (fp.range_filter_enabled) {
    chain.push_back(std::make_unique<RangeFilter>(fp.min_range, fp.max_range));
  }
  if (fp.angular_filter_enabled && !fp.angular_ranges.empty()) {
    chain.push_back(std::make_unique<AngularFilter>(fp.angular_ranges, fp.angular_invert));
  }
  if (fp.box_filter_enabled) {
    chain.push_back(std::make_unique<BoxFilter>(fp.box_min, fp.box_max));
  }
  return chain;
}

}  // namespace polka

#endif  // POLKA__FILTERS__FILTER_CHAIN_HPP_
