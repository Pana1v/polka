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

#ifndef POLKA__CONFIG_LOADER_HPP
#define POLKA__CONFIG_LOADER_HPP

#include "polka/types.hpp"
#include <rclcpp/rclcpp.hpp>

namespace polka {

class ConfigLoader {
public:
  explicit ConfigLoader(rclcpp::Node * node);
  MergeConfig load();
  MergeConfig reload(const std::vector<std::string> & source_names);

private:
  rclcpp::Node * node_;
  rclcpp::Logger logger_;

  void declare_defaults();
  MergeConfig read_common_params();
  FilterParams load_filter_params(const std::string & prefix);
  OutputQosConfig load_output_qos(const std::string & prefix);
  SelfFilterConfig load_self_filter_config(const std::string & prefix);
  void validate(const MergeConfig & config);
};

}  // namespace polka

#endif  // POLKA__CONFIG_LOADER_HPP
