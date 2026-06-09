#include "parallel/voxelizer_omp.h"

uint64_t countSolidVoxels(const SparseOctree& octree) {
    uint64_t count = 0;
    for (const auto* leaf : octree.levels[0]) {
        for (int x = 0; x < 4; ++x) {
            for (int y = 0; y < 4; ++y) {
                for (int z = 0; z < 4; ++z) {
                    if (leaf->SG[x][y][z]) {
                        count++;
                    }
                }
            }
        }
    }
    return count;
}
