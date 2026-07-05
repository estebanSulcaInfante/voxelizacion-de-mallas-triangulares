#include "parallel/voxelizer_omp.h"
#include "common/morton.h"
#include "common/octree.h"

#include <algorithm>
#include <atomic>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace {

size_t clippedIndex(uint32_t nx, uint32_t ny, uint32_t x, uint32_t y, uint32_t z) {
    return (static_cast<size_t>(z) * ny + y) * nx + x;
}

uint32_t voxelResolutionFromHeight(int height) {
    return 1u << (height + 2);
}

} // namespace

namespace par {

void propagateInsideOutside(SparseOctree& octree) {
    const uint32_t d = voxelResolutionFromHeight(octree.height);

    std::cout << "  Algoritmo 4: reconstruccion de volumen solido y recompresion sparse" << std::endl;

    const auto& leaves = octree.levels[0];

    uint32_t minX = std::numeric_limits<uint32_t>::max();
    uint32_t minY = std::numeric_limits<uint32_t>::max();
    uint32_t minZ = std::numeric_limits<uint32_t>::max();
    uint32_t maxX = 0;
    uint32_t maxY = 0;
    uint32_t maxZ = 0;

    for (const auto* leaf : leaves) {
        uint32_t bx, by, bz;
        decodeMorton3D(leaf->mortonCode, bx, by, bz);

        for (uint32_t z = 0; z < 4; ++z) {
            for (uint32_t y = 0; y < 4; ++y) {
                for (uint32_t x = 0; x < 4; ++x) {
                    if (leaf->SG[x][y][z]) {
                        const uint32_t gx = bx * 4u + x;
                        const uint32_t gy = by * 4u + y;
                        const uint32_t gz = bz * 4u + z;
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
    const size_t totalVoxels = static_cast<size_t>(nx) * ny * nz;

    std::unique_ptr<std::atomic<uint8_t>[]> states(new std::atomic<uint8_t>[totalVoxels]);

#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (size_t i = 0; i < totalVoxels; ++i) {
        states[i].store(0, std::memory_order_relaxed);
    }

#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (size_t leafIndex = 0; leafIndex < leaves.size(); ++leafIndex) {
        const auto* leaf = leaves[leafIndex];
        uint32_t bx, by, bz;
        decodeMorton3D(leaf->mortonCode, bx, by, bz);

        for (uint32_t z = 0; z < 4; ++z) {
            for (uint32_t y = 0; y < 4; ++y) {
                for (uint32_t x = 0; x < 4; ++x) {
                    if (leaf->SG[x][y][z]) {
                        const uint32_t gx = bx * 4u + x;
                        const uint32_t gy = by * 4u + y;
                        const uint32_t gz = bz * 4u + z;
                        states[clippedIndex(nx, ny, gx - minX, gy - minY, gz - minZ)]
                            .store(1, std::memory_order_relaxed);
                    }
                }
            }
        }
    }

    std::vector<uint32_t> currentFront;

    auto enqueueSeed = [&](uint32_t x, uint32_t y, uint32_t z) {
        const size_t index = clippedIndex(nx, ny, x, y, z);
        uint8_t expected = 0;
        if (states[index].compare_exchange_strong(expected, 2, std::memory_order_relaxed)) {
            currentFront.push_back(static_cast<uint32_t>(index));
        }
    };

    for (uint32_t x = 0; x < nx; ++x) {
        for (uint32_t y = 0; y < ny; ++y) {
            enqueueSeed(x, y, 0);
            enqueueSeed(x, y, nz - 1u);
        }
    }
    for (uint32_t x = 0; x < nx; ++x) {
        for (uint32_t z = 0; z < nz; ++z) {
            enqueueSeed(x, 0, z);
            enqueueSeed(x, ny - 1u, z);
        }
    }
    for (uint32_t y = 0; y < ny; ++y) {
        for (uint32_t z = 0; z < nz; ++z) {
            enqueueSeed(0, y, z);
            enqueueSeed(nx - 1u, y, z);
        }
    }

    std::vector<uint32_t> nextFront;

#ifdef _OPENMP
    const int maxThreads = omp_get_max_threads();
#else
    const int maxThreads = 1;
#endif
    std::vector<std::vector<uint32_t>> threadLocalNext(static_cast<size_t>(maxThreads));

    while (!currentFront.empty()) {
        for (auto& local : threadLocalNext) {
            local.clear();
        }

#ifdef _OPENMP
#pragma omp parallel
#endif
        {
#ifdef _OPENMP
            const int tid = omp_get_thread_num();
#else
            const int tid = 0;
#endif
            auto& localNext = threadLocalNext[static_cast<size_t>(tid)];

#ifdef _OPENMP
#pragma omp for schedule(dynamic, 1024)
#endif
            for (size_t i = 0; i < currentFront.size(); ++i) {
                const uint32_t index = currentFront[i];
                const uint32_t gx = index % nx;
                const uint32_t gy = (index / nx) % ny;
                const uint32_t gz = index / (nx * ny);

                auto tryEnqueue = [&](uint32_t tx, uint32_t ty, uint32_t tz) {
                    const size_t nIdx = clippedIndex(nx, ny, tx, ty, tz);
                    uint8_t expected = 0;
                    if (states[nIdx].compare_exchange_strong(expected, 2, std::memory_order_relaxed)) {
                        localNext.push_back(static_cast<uint32_t>(nIdx));
                    }
                };

                if (gx > 0) tryEnqueue(gx - 1u, gy, gz);
                if (gx + 1u < nx) tryEnqueue(gx + 1u, gy, gz);
                if (gy > 0) tryEnqueue(gx, gy - 1u, gz);
                if (gy + 1u < ny) tryEnqueue(gx, gy + 1u, gz);
                if (gz > 0) tryEnqueue(gx, gy, gz - 1u);
                if (gz + 1u < nz) tryEnqueue(gx, gy, gz + 1u);
            }
        }

        size_t totalNew = 0;
        for (const auto& local : threadLocalNext) {
            totalNew += local.size();
        }
        nextFront.clear();
        nextFront.reserve(totalNew);
        for (auto& local : threadLocalNext) {
            nextFront.insert(nextFront.end(), local.begin(), local.end());
        }

        std::swap(currentFront, nextFront);
    }

    size_t interiorCount = 0;
    size_t surfaceCount = 0;

#ifdef _OPENMP
#pragma omp parallel for reduction(+:interiorCount, surfaceCount) schedule(static)
#endif
    for (size_t i = 0; i < totalVoxels; ++i) {
        const uint8_t s = states[i].load(std::memory_order_relaxed);
        if (s == 1) {
            surfaceCount++;
        } else if (s == 0) {
            interiorCount++;
        }
    }

    std::cout << "    Voxeles de superficie: " << surfaceCount << std::endl;
    std::cout << "    Voxeles interiores rellenados: " << interiorCount << std::endl;

    rebuildSparseOctreeFromClippedAtomicVoxelStates(octree, states.get(), d,
                                                    minX, minY, minZ,
                                                    maxX, maxY, maxZ);
    std::cout << "    Octree recompreso bottom-up desde estados de flood-fill" << std::endl;
}

} // namespace par
