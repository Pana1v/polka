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

#ifndef POLKA__MERGE_ENGINE__CUDA_TYPES_CUH
#define POLKA__MERGE_ENGINE__CUDA_TYPES_CUH

#define POLKA_MAX_ANGULAR_RANGES 8
#define POLKA_MAX_SELF_BOXES 8
#define POLKA_VOXEL_TABLE_SIZE (1 << 18)
#define POLKA_MAX_SCAN_BINS 4096

struct GpuFilterParams {
  float min_range_sq;
  float max_range_sq;
  bool angular_enabled;
  bool invert;
  int n_angular_ranges;
  float4 angular_bounds[POLKA_MAX_ANGULAR_RANGES];
  bool angular_wide[POLKA_MAX_ANGULAR_RANGES];
  bool box_enabled;
  float3 box_min;
  float3 box_max;
};

struct GpuOutputFilterParams {
  bool range_enabled;
  float min_range_sq;
  float max_range_sq;

  bool angular_enabled;
  bool angular_invert;
  int n_angular_ranges;
  float4 angular_bounds[POLKA_MAX_ANGULAR_RANGES];
  bool angular_wide[POLKA_MAX_ANGULAR_RANGES];

  bool box_enabled;
  float3 box_min;
  float3 box_max;

  int n_self_boxes;
  float3 self_boxes_min[POLKA_MAX_SELF_BOXES];
  float3 self_boxes_max[POLKA_MAX_SELF_BOXES];

  bool height_cap_enabled;
  float z_min;
  float z_max;
};

struct GpuFlattenParams {
  float z_min;
  float z_max;
  float a_min;
  float a_max;
  float a_inc;
  float r_min;
  float r_max;
  int n_bins;
};

#endif  // POLKA__MERGE_ENGINE__CUDA_TYPES_CUH
