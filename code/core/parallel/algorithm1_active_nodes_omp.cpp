#include "parallel/voxelizer_omp.h"
#include "common/geometry.h"
#include "common/morton.h"

#include <iostream>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace par {

std::vector<uint32_t> determineActiveLevel1Nodes(const Mesh& mesh, uint32_t d) {
    uint32_t d1 = d / 8;
    uint32_t N = d1 * d1 * d1;

    std::cout << "  Algoritmo 1: d1=" << d1 << ", N=" << N << std::endl;

    std::vector<uint8_t> A(N, 0);
    const float voxelSize = 8.0f;

    if (mesh.triangles.empty()) {
        std::cout << "  Nodos activos nivel 1: 0 de " << N << std::endl;
        return {};
    }

#ifdef _OPENMP
    const int threadCount = omp_get_max_threads();
    std::vector<std::vector<uint8_t>> localMasks(static_cast<size_t>(threadCount),
                                                 std::vector<uint8_t>(N, 0));

#pragma omp parallel
    {
        const int threadId = omp_get_thread_num();
        std::vector<uint8_t>& localA = localMasks[static_cast<size_t>(threadId)];

#pragma omp for schedule(dynamic, 16)
        for (size_t triIndex = 0; triIndex < mesh.triangles.size(); ++triIndex) {
            const auto& tri = mesh.triangles[triIndex];
            uint32_t x_min, y_min, z_min, x_max, y_max, z_max;
            computeTriangleAABBLevel1(tri, d1, x_min, y_min, z_min, x_max, y_max, z_max);

            for (uint32_t z1 = z_min; z1 <= z_max; ++z1) {
                for (uint32_t y1 = y_min; y1 <= y_max; ++y1) {
                    for (uint32_t x1 = x_min; x1 <= x_max; ++x1) {
                        if (conservativeOverlapModified(tri, x1, y1, z1, voxelSize)) {
                            const uint32_t m1 = encodeMorton3D(x1, y1, z1);
                            if (m1 < N) {
                                localA[m1] = 1;
                            }
                        }
                    }
                }
            }
        }
    }

    for (uint32_t m1 = 0; m1 < N; ++m1) {
        for (int threadId = 0; threadId < threadCount; ++threadId) {
            if (localMasks[static_cast<size_t>(threadId)][m1]) {
                A[m1] = 1;
                break;
            }
        }
    }
#else
    for (const auto& tri : mesh.triangles) {
        uint32_t x_min, y_min, z_min, x_max, y_max, z_max;
        computeTriangleAABBLevel1(tri, d1, x_min, y_min, z_min, x_max, y_max, z_max);

        for (uint32_t z1 = z_min; z1 <= z_max; ++z1) {
            for (uint32_t y1 = y_min; y1 <= y_max; ++y1) {
                for (uint32_t x1 = x_min; x1 <= x_max; ++x1) {
                    if (conservativeOverlapModified(tri, x1, y1, z1, voxelSize)) {
                        const uint32_t m1 = encodeMorton3D(x1, y1, z1);
                        if (m1 < N) {
                            A[m1] = 1;
                        }
                    }
                }
            }
        }
    }
#endif

    std::vector<uint32_t> actNodes1;
    actNodes1.reserve(N);
    for (uint32_t m1 = 0; m1 < N; ++m1) {
        if (A[m1] == 1) {
            actNodes1.push_back(m1);
        }
    }

    std::cout << "  Nodos activos nivel 1: " << actNodes1.size() << " de " << N << std::endl;
    return actNodes1;
}

} // namespace par
