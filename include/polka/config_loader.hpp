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
  FilterParams load_filter_params(const std::string & prefix);
  SelfFilterConfig load_self_filter_config(const std::string & prefix);
  void validate(const MergeConfig & config);
};

}  // namespace polka

#endif  // POLKA__CONFIG_LOADER_HPP
