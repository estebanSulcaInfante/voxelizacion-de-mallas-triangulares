#include "parallel/voxelizer_omp.h"
#include "common/morton.h"
#include "common/octree.h"

#include <deque>
#include <iostream>
#include <vector>

namespace {

size_t gridIndex(uint32_t d, uint32_t x, uint32_t y, uint32_t z) {
    return (static_cast<size_t>(z) * d + y) * d + x;
}

uint32_t voxelResolutionFromHeight(int height) {
    return 1u << (height + 2);
}

void enqueueExteriorVoxel(std::deque<uint32_t>& queue,
                          std::vector<uint8_t>& states,
                          uint32_t d,
                          uint32_t x, uint32_t y, uint32_t z) {
    size_t index = gridIndex(d, x, y, z);
    if (states[index] == 0) {
        states[index] = 2;
        queue.push_back(static_cast<uint32_t>(index));
    }
}

} // namespace

void propagateInsideOutside(SparseOctree& octree) {
    const uint32_t d = voxelResolutionFromHeight(octree.height);
    std::vector<uint8_t> states(static_cast<size_t>(d) * d * d, 0);

    std::cout << "  Algoritmo 4: reconstruccion de volumen solido y recompresion sparse" << std::endl;

    for (const auto* leaf : octree.levels[0]) {
        uint32_t bx, by, bz;
        decodeMorton3D(leaf->mortonCode, bx, by, bz);

        for (uint32_t z = 0; z < 4; ++z) {
            for (uint32_t y = 0; y < 4; ++y) {
                for (uint32_t x = 0; x < 4; ++x) {
                    if (leaf->SG[x][y][z]) {
                        uint32_t gx = bx * 4 + x;
                        uint32_t gy = by * 4 + y;
                        uint32_t gz = bz * 4 + z;
                        states[gridIndex(d, gx, gy, gz)] = 1;
                    }
                }
            }
        }
    }

    std::deque<uint32_t> queue;

    for (uint32_t x = 0; x < d; ++x) {
        for (uint32_t y = 0; y < d; ++y) {
            enqueueExteriorVoxel(queue, states, d, x, y, 0);
            enqueueExteriorVoxel(queue, states, d, x, y, d - 1);
        }
    }
    for (uint32_t x = 0; x < d; ++x) {
        for (uint32_t z = 0; z < d; ++z) {
            enqueueExteriorVoxel(queue, states, d, x, 0, z);
            enqueueExteriorVoxel(queue, states, d, x, d - 1, z);
        }
    }
    for (uint32_t y = 0; y < d; ++y) {
        for (uint32_t z = 0; z < d; ++z) {
            enqueueExteriorVoxel(queue, states, d, 0, y, z);
            enqueueExteriorVoxel(queue, states, d, d - 1, y, z);
        }
    }

    while (!queue.empty()) {
        uint32_t index = queue.front();
        queue.pop_front();

        uint32_t gx = index % d;
        uint32_t gy = (index / d) % d;
        uint32_t gz = index / (d * d);

        if (gx > 0) enqueueExteriorVoxel(queue, states, d, gx - 1, gy, gz);
        if (gx + 1 < d) enqueueExteriorVoxel(queue, states, d, gx + 1, gy, gz);
        if (gy > 0) enqueueExteriorVoxel(queue, states, d, gx, gy - 1, gz);
        if (gy + 1 < d) enqueueExteriorVoxel(queue, states, d, gx, gy + 1, gz);
        if (gz > 0) enqueueExteriorVoxel(queue, states, d, gx, gy, gz - 1);
        if (gz + 1 < d) enqueueExteriorVoxel(queue, states, d, gx, gy, gz + 1);
    }

    size_t interiorCount = 0;
    size_t surfaceCount = 0;
    std::vector<uint8_t> solidGrid(states.size(), 0);
    for (size_t i = 0; i < states.size(); ++i) {
        if (states[i] == 1) {
            solidGrid[i] = 1;
            surfaceCount++;
        } else if (states[i] == 0) {
            solidGrid[i] = 1;
            interiorCount++;
        }
    }

    std::cout << "    Voxeles de superficie: " << surfaceCount << std::endl;
    std::cout << "    Voxeles interiores rellenados: " << interiorCount << std::endl;

    rebuildSparseOctreeFromDenseGrid(octree, solidGrid, d);
    std::cout << "    Octree recompreso desde volumen denso" << std::endl;
}
