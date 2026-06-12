#include "sequential/voxelizer_sequential.h"

namespace {

uint64_t countSolidNode(const OctreeNode* node) {
    if (!node) {
        return 0;
    }

    if (node->level == 0) {
        uint64_t count = 0;
        for (int x = 0; x < 4; ++x) {
            for (int y = 0; y < 4; ++y) {
                for (int z = 0; z < 4; ++z) {
                    count += node->SG[x][y][z] ? 1ull : 0ull;
                }
            }
        }
        return count;
    }

    bool hasChildren = false;
    for (const auto* child : node->children) {
        if (child) {
            hasChildren = true;
            break;
        }
    }

    if (!hasChildren) {
        return node->inside ? (1ull << (3 * (node->level + 2))) : 0ull;
    }

    uint64_t count = 0;
    for (const auto* child : node->children) {
        count += countSolidNode(child);
    }
    return count;
}

} // namespace

uint64_t countSolidVoxels(const SparseOctree& octree) {
    if (octree.levels.empty() || octree.levels[octree.height].empty()) {
        return 0;
    }
    return countSolidNode(octree.levels[octree.height].front());
}
