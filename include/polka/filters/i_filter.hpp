#ifndef POLKA__FILTERS__I_FILTER_HPP
#define POLKA__FILTERS__I_FILTER_HPP

#include "polka/types.hpp"
#include <string>

namespace polka {

class IFilter {
public:
  virtual void apply(CloudT & cloud, const std::string & frame_id) = 0;
  virtual ~IFilter() = default;
};

}  // namespace polka

#endif  // POLKA__FILTERS__I_FILTER_HPP
