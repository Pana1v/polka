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
