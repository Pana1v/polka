#ifndef POLKA__FILTERS__ANGULAR_FILTER_HPP
#define POLKA__FILTERS__ANGULAR_FILTER_HPP

#include "polka/filters/i_filter.hpp"
#include <vector>
#include <utility>

namespace polka {

class AngularFilter : public IFilter {
public:
  AngularFilter(const std::vector<std::pair<double, double>> & ranges_deg, bool invert);
  void apply(CloudT & cloud, const std::string & frame_id) override;

private:
  std::vector<std::pair<double, double>> ranges_deg_;
  bool invert_;
  bool in_ranges(double angle_deg) const;
};

}  // namespace polka

#endif  // POLKA__FILTERS__ANGULAR_FILTER_HPP
