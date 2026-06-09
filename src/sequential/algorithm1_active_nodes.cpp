#include "sequential/voxelizer_sequential.h"
#include "common/geometry.h"
#include "common/morton.h"

#include <iostream>

std::vector<uint32_t> determineActiveLevel1Nodes(const Mesh& mesh, uint32_t d) {
    uint32_t d1 = d / 8;
    uint32_t N = d1 * d1 * d1;

    std::cout << "  Algoritmo 1: d1=" << d1 << ", N=" << N << std::endl;

    std::vector<uint8_t> A(N, 0);
    float voxelSize = 8.0f;

    for (const auto& tri : mesh.triangles) {
        uint32_t x_min, y_min, z_min, x_max, y_max, z_max;
        computeTriangleAABBLevel1(tri, d1, x_min, y_min, z_min, x_max, y_max, z_max);

        for (uint32_t z1 = z_min; z1 <= z_max; ++z1) {
            for (uint32_t y1 = y_min; y1 <= y_max; ++y1) {
                for (uint32_t x1 = x_min; x1 <= x_max; ++x1) {
                    if (conservativeOverlapModified(tri, x1, y1, z1, voxelSize)) {
                        uint32_t m1 = encodeMorton3D(x1, y1, z1);
                        if (m1 < N) {
                            A[m1] = 1;
                        }
                    }
                }
            }
        }
    }

    std::vector<uint32_t> actNodes1;
    for (uint32_t m1 = 0; m1 < N; ++m1) {
        if (A[m1] == 1) {
            actNodes1.push_back(m1);
        }
    }

    std::cout << "  Nodos activos nivel 1: " << actNodes1.size() << " de " << N << std::endl;
    return actNodes1;
}
