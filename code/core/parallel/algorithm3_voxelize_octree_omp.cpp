#include "parallel/voxelizer_omp.h"
#include "common/geometry.h"
#include "common/morton.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace {

uint32_t subgridBitIndex(uint32_t lx, uint32_t ly, uint32_t lz) {
    return lx + 4u * (ly + 4u * lz);
}

void applyLeafMask(OctreeNode* leaf, uint64_t mask) {
    if (mask == 0) {
        return;
    }

    for (uint32_t z = 0; z < 4; ++z) {
        for (uint32_t y = 0; y < 4; ++y) {
            for (uint32_t x = 0; x < 4; ++x) {
                const uint32_t bit = subgridBitIndex(x, y, z);
                if (mask & (1ull << bit)) {
                    leaf->SG[x][y][z] = 1;
                }
            }
        }
    }
}

} // namespace

namespace par {

void voxelizeIntoOctree(const Mesh& mesh, SparseOctree& octree, int height) {
    (void)height;
    std::cout << "  Algoritmo 3: voxelizacion conservativa de superficie para "
              << mesh.triangles.size() << " triangulos" << std::endl;
    const uint32_t processedTriangles = static_cast<uint32_t>(mesh.triangles.size());

    if (mesh.triangles.empty() || octree.levels.empty() || octree.levels[0].empty()) {
        std::cout << "  Triangulos procesados: " << processedTriangles << std::endl;
        return;
    }

    const auto& leaves = octree.levels[0];
    std::unordered_map<uint32_t, size_t> leafIndex;
    leafIndex.reserve(leaves.size());
    for (size_t i = 0; i < leaves.size(); ++i) {
        leafIndex[leaves[i]->mortonCode] = i;
    }

#ifdef _OPENMP
    const int threadCount = omp_get_max_threads();
    std::vector<std::vector<uint64_t>> localMasks(static_cast<size_t>(threadCount),
                                                  std::vector<uint64_t>(leaves.size(), 0));

#pragma omp parallel
    {
        const int threadId = omp_get_thread_num();
        std::vector<uint64_t>& localLeafMasks = localMasks[static_cast<size_t>(threadId)];

#pragma omp for schedule(dynamic, 8)
        for (size_t triIndex = 0; triIndex < mesh.triangles.size(); ++triIndex) {
            const auto& tri = mesh.triangles[triIndex];

            Vec3 tmin = Vec3::min(tri.v0, Vec3::min(tri.v1, tri.v2));
            Vec3 tmax = Vec3::max(tri.v0, Vec3::max(tri.v1, tri.v2));

            int x_min = std::max(0, static_cast<int>(std::floor(tmin.x)));
            int y_min = std::max(0, static_cast<int>(std::floor(tmin.y)));
            int z_min = std::max(0, static_cast<int>(std::floor(tmin.z)));
            int x_max = static_cast<int>(std::floor(tmax.x));
            int y_max = static_cast<int>(std::floor(tmax.y));
            int z_max = static_cast<int>(std::floor(tmax.z));

            for (int vz = z_min; vz <= z_max; ++vz) {
                for (int vy = y_min; vy <= y_max; ++vy) {
                    for (int vx = x_min; vx <= x_max; ++vx) {
                        if (conservativeOverlapModified(tri,
                                                        static_cast<uint32_t>(vx),
                                                        static_cast<uint32_t>(vy),
                                                        static_cast<uint32_t>(vz),
                                                        1.0f)) {
                            const uint32_t leafMorton = encodeMorton3D(
                                static_cast<uint32_t>(vx / 4),
                                static_cast<uint32_t>(vy / 4),
                                static_cast<uint32_t>(vz / 4));
                            auto it = leafIndex.find(leafMorton);
                            if (it == leafIndex.end()) {
                                continue;
                            }

                            const uint32_t bit = subgridBitIndex(
                                static_cast<uint32_t>(vx % 4),
                                static_cast<uint32_t>(vy % 4),
                                static_cast<uint32_t>(vz % 4));
                            localLeafMasks[it->second] |= (1ull << bit);
                        }
                    }
                }
            }
        }
    }

    for (size_t leafIdx = 0; leafIdx < leaves.size(); ++leafIdx) {
        uint64_t mergedMask = 0;
        for (int threadId = 0; threadId < threadCount; ++threadId) {
            mergedMask |= localMasks[static_cast<size_t>(threadId)][leafIdx];
        }
        applyLeafMask(leaves[leafIdx], mergedMask);
    }
#else
    for (const auto& tri : mesh.triangles) {
        Vec3 tmin = Vec3::min(tri.v0, Vec3::min(tri.v1, tri.v2));
        Vec3 tmax = Vec3::max(tri.v0, Vec3::max(tri.v1, tri.v2));

        int x_min = std::max(0, static_cast<int>(std::floor(tmin.x)));
        int y_min = std::max(0, static_cast<int>(std::floor(tmin.y)));
        int z_min = std::max(0, static_cast<int>(std::floor(tmin.z)));
        int x_max = static_cast<int>(std::floor(tmax.x));
        int y_max = static_cast<int>(std::floor(tmax.y));
        int z_max = static_cast<int>(std::floor(tmax.z));

        for (int vz = z_min; vz <= z_max; ++vz) {
            for (int vy = y_min; vy <= y_max; ++vy) {
                for (int vx = x_min; vx <= x_max; ++vx) {
                    if (conservativeOverlapModified(tri,
                                                    static_cast<uint32_t>(vx),
                                                    static_cast<uint32_t>(vy),
                                                    static_cast<uint32_t>(vz),
                                                    1.0f)) {
                        const uint32_t leafMorton = encodeMorton3D(
                            static_cast<uint32_t>(vx / 4),
                            static_cast<uint32_t>(vy / 4),
                            static_cast<uint32_t>(vz / 4));
                        auto it = octree.nodeMaps[0].find(leafMorton);
                        if (it == octree.nodeMaps[0].end()) {
                            continue;
                        }

                        it->second->SG[vx % 4][vy % 4][vz % 4] = 1;
                    }
                }
            }
        }
    }
#endif

    std::cout << "  Triangulos procesados: " << processedTriangles << std::endl;
}

} // namespace par
