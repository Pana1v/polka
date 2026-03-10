#ifndef POLKA__MERGE_ENGINE__CUDA_MERGE_ENGINE_HPP
#define POLKA__MERGE_ENGINE__CUDA_MERGE_ENGINE_HPP

#ifdef POLKA_CUDA_ENABLED

#include "polka/merge_engine/i_merge_engine.hpp"

namespace polka {

class CudaMergeEngine : public IMergeEngine {
public:
  explicit CudaMergeEngine(const MergeConfig & config);
  ~CudaMergeEngine() override;

  CloudT::Ptr merge(const std::vector<MergeInput> & sources) override;
  bool is_gpu() const override { return true; }

private:
  struct Impl;
  Impl * impl_;
};

}  // namespace polka

#endif  // POLKA_CUDA_ENABLED
#endif  // POLKA__MERGE_ENGINE__CUDA_MERGE_ENGINE_HPP
