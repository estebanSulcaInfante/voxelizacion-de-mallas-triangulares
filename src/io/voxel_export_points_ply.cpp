#include "io/voxel_export_points_ply.h"
#include "common/morton.h"

#include <fstream>
#include <iostream>
#include <vector>

void exportVoxelsPLY(const SparseOctree& octree, uint32_t d,
                     const std::string& outputPath) {
    (void)d;
    std::vector<Vec3> points;

    for (const auto* leaf : octree.levels[0]) {
        uint32_t bx, by, bz;
        decodeMorton3D(leaf->mortonCode, bx, by, bz);

        for (int x = 0; x < 4; ++x) {
            for (int y = 0; y < 4; ++y) {
                for (int z = 0; z < 4; ++z) {
                    if (leaf->SG[x][y][z]) {
                        float gx = bx * 4.0f + x + 0.5f;
                        float gy = by * 4.0f + y + 0.5f;
                        float gz = bz * 4.0f + z + 0.5f;
                        points.push_back({gx, gy, gz});
                    }
                }
            }
        }
    }

    std::ofstream out(outputPath);
    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << points.size() << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "end_header\n";

    for (const auto& p : points) {
        out << p.x << " " << p.y << " " << p.z << "\n";
    }

    out.close();
    std::cout << "Exportado PLY: " << outputPath << " (" << points.size() << " puntos)" << std::endl;
}
