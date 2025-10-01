#ifndef POLKA__MERGE_ENGINE__CPU_MERGE_ENGINE_HPP_
#define POLKA__MERGE_ENGINE__CPU_MERGE_ENGINE_HPP_

#include "polka/merge_engine/i_merge_engine.hpp"

namespace polka {

class CpuMergeEngine : public IMergeEngine {
public:
  CloudT::Ptr merge(const std::vector<MergeInput> & sources) override;
  bool is_gpu() const override { return false; }
};

}  // namespace polka

#endif  // POLKA__MERGE_ENGINE__CPU_MERGE_ENGINE_HPP_
