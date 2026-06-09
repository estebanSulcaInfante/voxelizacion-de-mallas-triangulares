#include "io/voxel_export_surface_ply.h"
#include "common/morton.h"

#include <array>
#include <fstream>
#include <iostream>
#include <vector>

static bool isSolidVoxel(const SparseOctree& octree, uint32_t d,
                         int gx, int gy, int gz) {
    if (gx < 0 || gy < 0 || gz < 0) return false;
    if (gx >= static_cast<int>(d) || gy >= static_cast<int>(d) || gz >= static_cast<int>(d)) return false;

    uint32_t bx = static_cast<uint32_t>(gx / 4);
    uint32_t by = static_cast<uint32_t>(gy / 4);
    uint32_t bz = static_cast<uint32_t>(gz / 4);
    uint32_t leafMorton = encodeMorton3D(bx, by, bz);

    auto it = octree.nodeMaps[0].find(leafMorton);
    if (it == octree.nodeMaps[0].end()) {
        return false;
    }

    const OctreeNode* leaf = it->second;
    return leaf->SG[gx % 4][gy % 4][gz % 4] != 0;
}

void exportVoxelSurfacePLY(const SparseOctree& octree, uint32_t d,
                           const std::string& outputPath) {
    std::vector<Vec3> vertices;
    std::vector<std::array<uint32_t, 3>> faces;

    auto addQuad = [&](const Vec3& v0, const Vec3& v1, const Vec3& v2, const Vec3& v3) {
        uint32_t base = static_cast<uint32_t>(vertices.size());
        vertices.push_back(v0);
        vertices.push_back(v1);
        vertices.push_back(v2);
        vertices.push_back(v3);
        faces.push_back({base + 0, base + 1, base + 2});
        faces.push_back({base + 0, base + 2, base + 3});
    };

    for (const auto* leaf : octree.levels[0]) {
        uint32_t bx, by, bz;
        decodeMorton3D(leaf->mortonCode, bx, by, bz);

        for (int x = 0; x < 4; ++x) {
            for (int y = 0; y < 4; ++y) {
                for (int z = 0; z < 4; ++z) {
                    if (!leaf->SG[x][y][z]) {
                        continue;
                    }

                    int gx = static_cast<int>(bx * 4 + x);
                    int gy = static_cast<int>(by * 4 + y);
                    int gz = static_cast<int>(bz * 4 + z);

                    float x0 = static_cast<float>(gx);
                    float x1 = x0 + 1.0f;
                    float y0 = static_cast<float>(gy);
                    float y1 = y0 + 1.0f;
                    float z0 = static_cast<float>(gz);
                    float z1 = z0 + 1.0f;

                    if (!isSolidVoxel(octree, d, gx - 1, gy, gz)) {
                        addQuad({x0, y0, z0}, {x0, y0, z1}, {x0, y1, z1}, {x0, y1, z0});
                    }
                    if (!isSolidVoxel(octree, d, gx + 1, gy, gz)) {
                        addQuad({x1, y0, z0}, {x1, y1, z0}, {x1, y1, z1}, {x1, y0, z1});
                    }
                    if (!isSolidVoxel(octree, d, gx, gy - 1, gz)) {
                        addQuad({x0, y0, z0}, {x1, y0, z0}, {x1, y0, z1}, {x0, y0, z1});
                    }
                    if (!isSolidVoxel(octree, d, gx, gy + 1, gz)) {
                        addQuad({x0, y1, z0}, {x0, y1, z1}, {x1, y1, z1}, {x1, y1, z0});
                    }
                    if (!isSolidVoxel(octree, d, gx, gy, gz - 1)) {
                        addQuad({x0, y0, z0}, {x0, y1, z0}, {x1, y1, z0}, {x1, y0, z0});
                    }
                    if (!isSolidVoxel(octree, d, gx, gy, gz + 1)) {
                        addQuad({x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1});
                    }
                }
            }
        }
    }

    std::ofstream out(outputPath);
    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << vertices.size() << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "element face " << faces.size() << "\n";
    out << "property list uchar int vertex_indices\n";
    out << "end_header\n";

    for (const auto& v : vertices) {
        out << v.x << " " << v.y << " " << v.z << "\n";
    }
    for (const auto& f : faces) {
        out << "3 " << f[0] << " " << f[1] << " " << f[2] << "\n";
    }

    out.close();
    std::cout << "Exportado superficie voxelizada PLY: " << outputPath
              << " (" << vertices.size() << " vertices, "
              << faces.size() << " triangulos)" << std::endl;
}
