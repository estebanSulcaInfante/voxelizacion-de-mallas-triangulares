#pragma once

#include "common/mesh.h"
#include "common/octree.h"
#include <vector>
#include <cstdint>

std::vector<uint32_t> determineActiveLevel1Nodes(const Mesh& mesh, uint32_t d);
SparseOctree buildSparseOctree(const std::vector<uint32_t>& actNodes1, int height);
void voxelizeIntoOctree(const Mesh& mesh, SparseOctree& octree, int height);
void propagateInsideOutside(SparseOctree& octree);
uint64_t countSolidVoxels(const SparseOctree& octree);
