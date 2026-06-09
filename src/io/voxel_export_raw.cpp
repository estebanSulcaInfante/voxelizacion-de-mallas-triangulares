#include "io/voxel_export_raw.h"
#include "common/morton.h"

#include <fstream>
#include <iostream>
#include <vector>

void exportVoxelsRAW(const SparseOctree& octree, uint32_t d,
                     const std::string& outputPath) {
    std::vector<uint8_t> grid(static_cast<size_t>(d) * d * d, 0);

    for (const auto* leaf : octree.levels[0]) {
        uint32_t bx, by, bz;
        decodeMorton3D(leaf->mortonCode, bx, by, bz);

        for (int x = 0; x < 4; ++x) {
            for (int y = 0; y < 4; ++y) {
                for (int z = 0; z < 4; ++z) {
                    if (leaf->SG[x][y][z]) {
                        uint32_t gx = bx * 4 + x;
                        uint32_t gy = by * 4 + y;
                        uint32_t gz = bz * 4 + z;
                        if (gx < d && gy < d && gz < d) {
                            grid[static_cast<size_t>(gz) * d * d + gy * d + gx] = 1;
                        }
                    }
                }
            }
        }
    }

    std::ofstream out(outputPath, std::ios::binary);
    out.write(reinterpret_cast<const char*>(grid.data()), grid.size());
    out.close();

    std::cout << "Exportado RAW: " << outputPath << " (" << d << "^3 = "
              << grid.size() << " bytes)" << std::endl;
}
