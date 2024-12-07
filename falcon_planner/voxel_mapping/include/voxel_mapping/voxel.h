#ifndef VOXEL_H
#define VOXEL_H

#include "exploration_types.h"

namespace voxel_mapping {

enum class OccupancyType {
  UNKNOWN,
  OCCUPIED,
  FREE
};

struct OccupancyVoxel {
  OccupancyType value = OccupancyType::UNKNOWN;
};

struct TSDFVoxel {
  FloatingPoint value = 0.0, weight = 0.0;
};

struct ESDFVoxel {
  FloatingPoint value = std::numeric_limits<FloatingPoint>::max();
};

} // namespace voxel_mapping

#endif // VOXEL_H