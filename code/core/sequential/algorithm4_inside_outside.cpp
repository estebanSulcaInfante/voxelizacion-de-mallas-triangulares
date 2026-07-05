#include "sequential/voxelizer_sequential.h"
#include "common/morton.h"
#include "common/octree.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <vector>

namespace {

class IndexQueue {
public:
    explicit IndexQueue(size_t initialCapacity) {
        buffer.resize(std::max<size_t>(initialCapacity, 1));
    }

    bool empty() const {
        return count == 0;
    }

    void push(uint32_t value) {
        if (count == buffer.size()) {
            grow();
        }
        buffer[tail] = value;
        tail = (tail + 1) % buffer.size();
        ++count;
    }

    uint32_t pop() {
        const uint32_t value = buffer[head];
        head = (head + 1) % buffer.size();
        --count;
        return value;
    }

private:
    void grow() {
        std::vector<uint32_t> expanded(buffer.size() * 2);
        for (size_t i = 0; i < count; ++i) {
            expanded[i] = buffer[(head + i) % buffer.size()];
        }
        buffer.swap(expanded);
        head = 0;
        tail = count;
    }

    std::vector<uint32_t> buffer;
    size_t head = 0;
    size_t tail = 0;
    size_t count = 0;
};

size_t clippedIndex(uint32_t nx, uint32_t ny, uint32_t x, uint32_t y, uint32_t z) {
    return (static_cast<size_t>(z) * ny + y) * nx + x;
}

uint32_t voxelResolutionFromHeight(int height) {
    return 1u << (height + 2);
}

void enqueueExteriorVoxel(IndexQueue& queue,
                          std::vector<uint8_t>& states,
                          uint32_t nx,
                          uint32_t ny,
                          uint32_t x, uint32_t y, uint32_t z) {
    size_t index = clippedIndex(nx, ny, x, y, z);
    if (states[index] == 0) {
        states[index] = 2;
        queue.push(static_cast<uint32_t>(index));
    }
}

} // namespace

namespace seq {

void propagateInsideOutside(SparseOctree& octree) {
    const uint32_t d = voxelResolutionFromHeight(octree.height);

    std::cout << "  Algoritmo 4: reconstruccion de volumen solido y recompresion sparse" << std::endl;

    uint32_t minX = std::numeric_limits<uint32_t>::max();
    uint32_t minY = std::numeric_limits<uint32_t>::max();
    uint32_t minZ = std::numeric_limits<uint32_t>::max();
    uint32_t maxX = 0;
    uint32_t maxY = 0;
    uint32_t maxZ = 0;

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
                        minX = std::min(minX, gx);
                        minY = std::min(minY, gy);
                        minZ = std::min(minZ, gz);
                        maxX = std::max(maxX, gx);
                        maxY = std::max(maxY, gy);
                        maxZ = std::max(maxZ, gz);
                    }
                }
            }
        }
    }

    if (minX == std::numeric_limits<uint32_t>::max()) {
        std::cout << "    Voxeles de superficie: 0" << std::endl;
        std::cout << "    Voxeles interiores rellenados: 0" << std::endl;
        return;
    }

    minX = (minX == 0) ? 0 : minX - 1u;
    minY = (minY == 0) ? 0 : minY - 1u;
    minZ = (minZ == 0) ? 0 : minZ - 1u;
    maxX = std::min(d - 1u, maxX + 1u);
    maxY = std::min(d - 1u, maxY + 1u);
    maxZ = std::min(d - 1u, maxZ + 1u);

    const uint32_t nx = maxX - minX + 1u;
    const uint32_t ny = maxY - minY + 1u;
    const uint32_t nz = maxZ - minZ + 1u;
    std::vector<uint8_t> states(static_cast<size_t>(nx) * ny * nz, 0);

    for (const auto* leaf : octree.levels[0]) {
        uint32_t bx, by, bz;
        decodeMorton3D(leaf->mortonCode, bx, by, bz);

        for (uint32_t z = 0; z < 4; ++z) {
            for (uint32_t y = 0; y < 4; ++y) {
                for (uint32_t x = 0; x < 4; ++x) {
                    if (leaf->SG[x][y][z]) {
                        const uint32_t gx = bx * 4 + x;
                        const uint32_t gy = by * 4 + y;
                        const uint32_t gz = bz * 4 + z;
                        states[clippedIndex(nx, ny, gx - minX, gy - minY, gz - minZ)] = 1;
                    }
                }
            }
        }
    }

    IndexQueue queue(static_cast<size_t>(nx) * ny);

    for (uint32_t x = 0; x < nx; ++x) {
        for (uint32_t y = 0; y < ny; ++y) {
            enqueueExteriorVoxel(queue, states, nx, ny, x, y, 0);
            enqueueExteriorVoxel(queue, states, nx, ny, x, y, nz - 1);
        }
    }
    for (uint32_t x = 0; x < nx; ++x) {
        for (uint32_t z = 0; z < nz; ++z) {
            enqueueExteriorVoxel(queue, states, nx, ny, x, 0, z);
            enqueueExteriorVoxel(queue, states, nx, ny, x, ny - 1, z);
        }
    }
    for (uint32_t y = 0; y < ny; ++y) {
        for (uint32_t z = 0; z < nz; ++z) {
            enqueueExteriorVoxel(queue, states, nx, ny, 0, y, z);
            enqueueExteriorVoxel(queue, states, nx, ny, nx - 1, y, z);
        }
    }

    while (!queue.empty()) {
        uint32_t index = queue.pop();

        uint32_t gx = index % nx;
        uint32_t gy = (index / nx) % ny;
        uint32_t gz = index / (nx * ny);

        if (gx > 0) enqueueExteriorVoxel(queue, states, nx, ny, gx - 1, gy, gz);
        if (gx + 1 < nx) enqueueExteriorVoxel(queue, states, nx, ny, gx + 1, gy, gz);
        if (gy > 0) enqueueExteriorVoxel(queue, states, nx, ny, gx, gy - 1, gz);
        if (gy + 1 < ny) enqueueExteriorVoxel(queue, states, nx, ny, gx, gy + 1, gz);
        if (gz > 0) enqueueExteriorVoxel(queue, states, nx, ny, gx, gy, gz - 1);
        if (gz + 1 < nz) enqueueExteriorVoxel(queue, states, nx, ny, gx, gy, gz + 1);
    }

    size_t interiorCount = 0;
    size_t surfaceCount = 0;
    for (size_t i = 0; i < states.size(); ++i) {
        if (states[i] == 1) {
            surfaceCount++;
        } else if (states[i] == 0) {
            interiorCount++;
        }
    }

    std::cout << "    Voxeles de superficie: " << surfaceCount << std::endl;
    std::cout << "    Voxeles interiores rellenados: " << interiorCount << std::endl;

    rebuildSparseOctreeFromClippedVoxelStates(octree, states, d, minX, minY, minZ, maxX, maxY, maxZ);
    std::cout << "    Octree recompreso bottom-up desde estados de flood-fill" << std::endl;
}

} // namespace seq
