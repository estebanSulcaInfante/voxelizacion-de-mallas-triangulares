#include "parallel/voxelizer_omp.h"
#include "common/morton.h"
#include "common/octree.h"

#include <iostream>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace par {

SparseOctree buildSparseOctree(const std::vector<uint32_t>& actNodes1, int height) {
    SparseOctree octree;
    octree.height = height;

    octree.levels.resize(height + 1);
    octree.nodeMaps.resize(height + 1);

    std::cout << "  Algoritmo 2: altura=" << height << std::endl;

    createChildNodes(octree, actNodes1);

    std::vector<uint32_t> actNodes = actNodes1;

    for (int level = 1; level <= height; ++level) {
        std::vector<uint32_t> nextActNodes;
        nextActNodes.reserve(actNodes.size());
        std::vector<uint32_t> parentCodes(actNodes.size());

#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
        for (size_t i = 0; i < actNodes.size(); ++i) {
            parentCodes[i] = parentMorton(actNodes[i]);
        }

        for (size_t i = 0; i < actNodes.size(); ++i) {
            uint32_t parent_m = parentCodes[i];
            if (nextActNodes.empty() || nextActNodes.back() != parent_m) {
                nextActNodes.push_back(parent_m);
                allocateParentGroup(octree, parent_m, level);
            }
        }

#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
        for (size_t i = 0; i < actNodes.size(); ++i) {
            const uint32_t parent_m = parentCodes[i];
            const uint32_t j = childIndex(actNodes[i]);

            auto childIt = octree.nodeMaps[level - 1].find(actNodes[i]);
            if (childIt != octree.nodeMaps[level - 1].end()) {
                OctreeNode* child = childIt->second;
                OctreeNode* parent = octree.nodeMaps[level][parent_m];
                parent->children[j] = child;
                child->parent = parent;
            }
        }

        std::cout << "  Nivel " << level << ": " << nextActNodes.size() << " nodos" << std::endl;
        actNodes = nextActNodes;
    }

    for (int level = height; level >= 1; --level) {
        for (auto* node : octree.levels[level]) {
            propagateXNeighbors(node);
        }
    }

    std::cout << "  Vecindad en X propagada" << std::endl;
    return octree;
}

} // namespace par
