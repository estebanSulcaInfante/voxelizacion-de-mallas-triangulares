#pragma once

#include "common/octree.h"
#include <cstdint>
#include <string>

/// Exporta los vóxeles como un array binario denso de d x d x d bytes.
void exportVoxelsRAW(const SparseOctree& octree, uint32_t d,
                     const std::string& outputPath);
