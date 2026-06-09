#pragma once

#include "common/octree.h"
#include <cstdint>
#include <string>

/// Exporta solo la superficie exterior de los vóxeles sólidos como malla PLY.
void exportVoxelSurfacePLY(const SparseOctree& octree, uint32_t d,
                           const std::string& outputPath);
