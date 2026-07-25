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

#ifndef POLKA__CONFIG__CONFIG_LOADER_HPP_
#define POLKA__CONFIG__CONFIG_LOADER_HPP_

#include <string>
#include <vector>

#include "polka/types.hpp"
#include <rclcpp/rclcpp.hpp>

namespace polka
{

class ConfigLoader
{
public:
  explicit ConfigLoader(rclcpp::Node * node);

  // Startup load: strict validation (every source needs a topic).
  MergeConfig load();

  // Runtime re-read of committed parameter storage. Declares parameters for
  // source names not seen before and tolerates "pending" sources whose topic
  // is still empty, so sources can be added incrementally at runtime.
  MergeConfig reload(const std::vector<std::string> & source_names);

  // Validation pass over *proposed* parameter values, overlaid on committed
  // storage. Never mutates node state (no declares). Throws with a
  // human-readable, prefix-qualified reason on invalid config - the caller
  // surfaces it as SetParametersResult.reason.
  //
  // This exists because Humble's on-set callback fires BEFORE the new values
  // are committed: reading get_parameter() there returns the old values, so
  // validation must work from the proposed list instead.
  MergeConfig preview(
    const std::vector<rclcpp::Parameter> & proposed,
    const std::vector<std::string> & source_names);

private:
  rclcpp::Node * node_;
  rclcpp::Logger logger_;
  // When set, parameter reads resolve from this list before node storage.
  const std::vector<rclcpp::Parameter> * overlay_ = nullptr;

  void declare_defaults();
  void declare_source_params(const std::string & name);
  MergeConfig read_config(const std::vector<std::string> & source_names, bool allow_pending);
  MergeConfig read_common_params();
  SourceConfig read_source_config(const std::string & name);
  FilterParams load_filter_params(const std::string & prefix);
  OutputQosConfig load_output_qos(const std::string & prefix);
  SelfFilterConfig load_self_filter_config(const std::string & prefix, bool declare_missing);
  void validate(MergeConfig & config, bool allow_pending);

  // overlay -> committed storage; throws if the parameter was never declared.
  rclcpp::Parameter param(const std::string & name) const;
  // overlay -> committed storage -> fallback default. For parameters that may
  // not be declared yet (a new source's params during preview).
  template<typename T>
  T param_or(const std::string & name, const T & def) const;
};

}  // namespace polka

#endif  // POLKA__CONFIG__CONFIG_LOADER_HPP_
