#pragma once

#include "common/octree.h"
#include <cstdint>
#include <string>

/// Exporta los vóxeles sólidos como una nube de puntos en formato PLY.
void exportVoxelsPLY(const SparseOctree& octree, uint32_t d,
                     const std::string& outputPath);
