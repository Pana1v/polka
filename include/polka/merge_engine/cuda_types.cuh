#ifndef POLKA__MERGE_ENGINE__CUDA_TYPES_CUH
#define POLKA__MERGE_ENGINE__CUDA_TYPES_CUH

#define POLKA_MAX_ANGULAR_RANGES 8

struct GpuFilterParams {
  float min_range_sq;
  float max_range_sq;
  bool angular_enabled;
  bool invert;
  int n_angular_ranges;
  float2 angular_ranges[POLKA_MAX_ANGULAR_RANGES];
  bool box_enabled;
  float3 box_min;
  float3 box_max;
};

#endif  // POLKA__MERGE_ENGINE__CUDA_TYPES_CUH
