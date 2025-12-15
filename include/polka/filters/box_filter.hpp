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

#ifndef POLKA__FILTERS__BOX_FILTER_HPP
#define POLKA__FILTERS__BOX_FILTER_HPP

#include "polka/filters/i_filter.hpp"
#include <Eigen/Core>

namespace polka {

class BoxFilter : public IFilter {
public:
  BoxFilter(const Eigen::Vector3d & box_min, const Eigen::Vector3d & box_max,
            bool invert = false);
  void apply(CloudT & cloud, const std::string & frame_id) override;

private:
  float bx_min_, bx_max_, by_min_, by_max_, bz_min_, bz_max_;
  bool invert_;
};

}  // namespace polka

#endif  // POLKA__FILTERS__BOX_FILTER_HPP
