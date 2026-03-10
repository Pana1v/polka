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
