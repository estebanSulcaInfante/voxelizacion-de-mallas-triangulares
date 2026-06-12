#include "sequential/voxelizer_sequential.h"
#include "common/geometry.h"
#include "common/morton.h"

#include <algorithm>
#include <cmath>
#include <iostream>
namespace {

void markSurfaceVoxel(SparseOctree& octree, uint32_t vx, uint32_t vy, uint32_t vz) {
    uint32_t m0 = encodeMorton3D(vx / 4, vy / 4, vz / 4);
    auto it = octree.nodeMaps[0].find(m0);
    if (it == octree.nodeMaps[0].end()) {
        return;
    }

    OctreeNode* leaf = it->second;
    leaf->SG[vx % 4][vy % 4][vz % 4] = 1;
}

} // namespace

void voxelizeIntoOctree(const Mesh& mesh, SparseOctree& octree, int height) {
    (void)height;
    std::cout << "  Algoritmo 3: voxelizacion conservativa de superficie para "
              << mesh.triangles.size() << " triangulos" << std::endl;

    uint32_t processedTriangles = 0;

    for (const auto& tri : mesh.triangles) {
        processedTriangles++;

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
                        markSurfaceVoxel(octree,
                                         static_cast<uint32_t>(vx),
                                         static_cast<uint32_t>(vy),
                                         static_cast<uint32_t>(vz));
                    }
                }
            }
        }
    }

    std::cout << "  Triangulos procesados: " << processedTriangles << std::endl;
}
