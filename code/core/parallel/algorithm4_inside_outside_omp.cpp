#include "parallel/voxelizer_omp.h"
#include "common/morton.h"
#include "common/octree.h"

#include <iostream>
#include <vector>
#include <atomic>
#include <memory>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace {

size_t gridIndex(uint32_t d, uint32_t x, uint32_t y, uint32_t z) {
    return (static_cast<size_t>(z) * d + y) * d + x;
}

uint32_t voxelResolutionFromHeight(int height) {
    return 1u << (height + 2);
}

} // namespace

namespace par {

void propagateInsideOutside(SparseOctree& octree) {
    const uint32_t d = voxelResolutionFromHeight(octree.height);
    const size_t totalVoxels = static_cast<size_t>(d) * d * d;

    // Usamos std::atomic para evitar race conditions al marcar los vóxeles descubiertos.
    std::unique_ptr<std::atomic<uint8_t>[]> states(new std::atomic<uint8_t>[totalVoxels]);

#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (size_t i = 0; i < totalVoxels; ++i) {
        states[i].store(0, std::memory_order_relaxed);
    }

    std::cout << "  Algoritmo 4: reconstruccion de volumen solido y recompresion sparse" << std::endl;

    const auto& leaves = octree.levels[0];

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
                        uint32_t gx = bx * 4 + x;
                        uint32_t gy = by * 4 + y;
                        uint32_t gz = bz * 4 + z;
                        states[gridIndex(d, gx, gy, gz)].store(1, std::memory_order_relaxed);
                    }
                }
            }
        }
    }

    std::vector<uint32_t> currentFront;
    
    // Función auxiliar atómica para encolar semillas
    auto enqueueSeed = [&](uint32_t x, uint32_t y, uint32_t z) {
        size_t index = gridIndex(d, x, y, z);
        uint8_t expected = 0;
        if (states[index].compare_exchange_strong(expected, 2, std::memory_order_relaxed)) {
            currentFront.push_back(static_cast<uint32_t>(index));
        }
    };

    // Inicializar semillas en las caras del volumen
    for (uint32_t x = 0; x < d; ++x) {
        for (uint32_t y = 0; y < d; ++y) {
            enqueueSeed(x, y, 0);
            enqueueSeed(x, y, d - 1);
        }
    }
    for (uint32_t x = 0; x < d; ++x) {
        for (uint32_t z = 0; z < d; ++z) {
            enqueueSeed(x, 0, z);
            enqueueSeed(x, d - 1, z);
        }
    }
    for (uint32_t y = 0; y < d; ++y) {
        for (uint32_t z = 0; z < d; ++z) {
            enqueueSeed(0, y, z);
            enqueueSeed(d - 1, y, z);
        }
    }

    std::vector<uint32_t> nextFront;

    // BFS Paralelo por Niveles (Level-Synchronous BFS)
    while (!currentFront.empty()) {
        std::vector<std::vector<uint32_t>> threadLocalNext;

#ifdef _OPENMP
        int maxThreads = omp_get_max_threads();
#else
        int maxThreads = 1;
#endif
        threadLocalNext.resize(maxThreads);

#ifdef _OPENMP
#pragma omp parallel
#endif
        {
#ifdef _OPENMP
            int tid = omp_get_thread_num();
#else
            int tid = 0;
#endif
            auto& localNext = threadLocalNext[tid];

            // Procesar el frente actual en paralelo
#ifdef _OPENMP
#pragma omp for schedule(dynamic, 1024)
#endif
            for (size_t i = 0; i < currentFront.size(); ++i) {
                uint32_t index = currentFront[i];
                uint32_t gx = index % d;
                uint32_t gy = (index / d) % d;
                uint32_t gz = index / (d * d);

                // Función auxiliar para encolar vecinos desde los hilos
                auto tryEnqueue = [&](uint32_t nx, uint32_t ny, uint32_t nz) {
                    size_t nIdx = gridIndex(d, nx, ny, nz);
                    uint8_t expected = 0;
                    if (states[nIdx].compare_exchange_strong(expected, 2, std::memory_order_relaxed)) {
                        localNext.push_back(static_cast<uint32_t>(nIdx));
                    }
                };

                if (gx > 0) tryEnqueue(gx - 1, gy, gz);
                if (gx + 1 < d) tryEnqueue(gx + 1, gy, gz);
                if (gy > 0) tryEnqueue(gx, gy - 1, gz);
                if (gy + 1 < d) tryEnqueue(gx, gy + 1, gz);
                if (gz > 0) tryEnqueue(gx, gy, gz - 1);
                if (gz + 1 < d) tryEnqueue(gx, gy, gz + 1);
            }
        } // fin de la region paralela

        // Unir todos los buffers locales en el nuevo frente de onda
        size_t totalNew = 0;
        for (const auto& local : threadLocalNext) {
            totalNew += local.size();
        }
        nextFront.reserve(totalNew);
        for (auto& local : threadLocalNext) {
            nextFront.insert(nextFront.end(), local.begin(), local.end());
        }

        std::swap(currentFront, nextFront);
        nextFront.clear();
    }

    size_t interiorCount = 0;
    size_t surfaceCount = 0;

#ifdef _OPENMP
#pragma omp parallel for reduction(+:interiorCount, surfaceCount) schedule(static)
#endif
    for (size_t i = 0; i < totalVoxels; ++i) {
        uint8_t s = states[i].load(std::memory_order_relaxed);
        if (s == 1) {
            surfaceCount++;
        } else if (s == 0) {
            interiorCount++;
        }
    }

    std::cout << "    Voxeles de superficie: " << surfaceCount << std::endl;
    std::cout << "    Voxeles interiores rellenados: " << interiorCount << std::endl;

    rebuildSparseOctreeFromAtomicVoxelStates(octree, states.get(), d);
    std::cout << "    Octree recompreso bottom-up desde estados de flood-fill" << std::endl;
}

} // namespace par

