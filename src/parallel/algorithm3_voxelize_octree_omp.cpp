#include "parallel/voxelizer_omp.h"
#include "common/geometry.h"
#include "common/morton.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace {

void applyParityAtVoxel(SparseOctree& octree, uint32_t vx, uint32_t vy, uint32_t vz) {
    uint32_t m0 = encodeMorton3D(vx / 4, vy / 4, vz / 4);
    auto it = octree.nodeMaps[0].find(m0);
    if (it == octree.nodeMaps[0].end()) {
        return;
    }

    OctreeNode* leaf = it->second;
    uint32_t xlocal = vx % 4;
    uint32_t ylocal = vy % 4;
    uint32_t zlocal = vz % 4;

    for (uint32_t x = xlocal; x < 4; ++x) {
        leaf->SG[x][ylocal][zlocal] ^= 1;
    }
}

void voxelizeTriangleFallbackXParity(const Triangle& tri, SparseOctree& octree) {
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
                    applyParityAtVoxel(octree,
                                       static_cast<uint32_t>(vx),
                                       static_cast<uint32_t>(vy),
                                       static_cast<uint32_t>(vz));
                }
            }
        }
    }
}

} // namespace

void voxelizeIntoOctree(const Mesh& mesh, SparseOctree& octree, int height) {
    (void)height;
    std::cout << "  Algoritmo 3: voxelizando " << mesh.triangles.size() << " triangulos" << std::endl;

    float tileSize = 4.0f;
    uint32_t processedTriangles = 0;

    for (const auto& tri : mesh.triangles) {
        processedTriangles++;

        if (dominantAxisOfTriangle(tri) != DominantAxis::X) {
            voxelizeTriangleFallbackXParity(tri, octree);
            continue;
        }

        float y_min_f, y_max_f, z_min_f, z_max_f;
        computeBoundingBoxYZ(tri, y_min_f, y_max_f, z_min_f, z_max_f);

        int y0_min = static_cast<int>(std::max(0.0f, std::floor(y_min_f / tileSize)));
        int y0_max = static_cast<int>(std::floor(y_max_f / tileSize));
        int z0_min = static_cast<int>(std::max(0.0f, std::floor(z_min_f / tileSize)));
        int z0_max = static_cast<int>(std::floor(z_max_f / tileSize));

        TriangleEdgeFunctions eyz = setupEdgeFunctionsYZ(tri);

        for (int z0 = z0_min; z0 <= z0_max; ++z0) {
            for (int y0 = y0_min; y0 <= y0_max; ++y0) {
                for (int sz = 0; sz < 4; ++sz) {
                    for (int sy = 0; sy < 4; ++sy) {
                        float cy = y0 * tileSize + sy + 0.5f;
                        float cz = z0 * tileSize + sz + 0.5f;

                        if (centerInsideProjectionYZ(cy, cz, tri, eyz)) {
                            float x_tilde = projectCenterToTrianglePlaneX(cy, cz, tri);
                            if (x_tilde < 0) continue;

                            uint32_t vx = static_cast<uint32_t>(std::floor(x_tilde));
                            uint32_t vy = static_cast<uint32_t>(y0 * 4 + sy);
                            uint32_t vz = static_cast<uint32_t>(z0 * 4 + sz);
                            applyParityAtVoxel(octree, vx, vy, vz);
                        }
                    }
                }
            }
        }
    }

    std::cout << "  Triangulos procesados: " << processedTriangles << std::endl;
}
