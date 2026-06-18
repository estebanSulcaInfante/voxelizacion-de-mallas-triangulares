#include "io/voxel_export_octree_vtu.h"
#include "common/morton.h"

#include <array>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <vector>

namespace {

struct VTUCell {
    std::array<float, 24> points;
    int depth;
    int level;
    uint32_t span;
};

uint32_t nodeSpanVoxels(int level) {
    return 4u << level;
}

bool hasChildren(const OctreeNode* node) {
    for (const auto* child : node->children) {
        if (child) {
            return true;
        }
    }
    return false;
}

void appendHexahedron(std::vector<VTUCell>& cells,
                      float x0, float y0, float z0,
                      float x1, float y1, float z1,
                      int depth, int level, uint32_t span) {
    cells.push_back({{
        x0, y0, z0,
        x1, y0, z0,
        x1, y1, z0,
        x0, y1, z0,
        x0, y0, z1,
        x1, y0, z1,
        x1, y1, z1,
        x0, y1, z1
    }, depth, level, span});
}

bool subgridIsUniform(const OctreeNode* node,
                      uint32_t ox, uint32_t oy, uint32_t oz,
                      uint32_t span, bool& solidValue) {
    solidValue = node->SG[ox][oy][oz] != 0;
    for (uint32_t z = oz; z < oz + span; ++z) {
        for (uint32_t y = oy; y < oy + span; ++y) {
            for (uint32_t x = ox; x < ox + span; ++x) {
                if ((node->SG[x][y][z] != 0) != solidValue) {
                    return false;
                }
            }
        }
    }
    return true;
}

void appendLeafSubgridCells(const OctreeNode* node,
                            uint32_t baseX, uint32_t baseY, uint32_t baseZ,
                            uint32_t ox, uint32_t oy, uint32_t oz,
                            uint32_t span, int depth,
                            std::vector<VTUCell>& cells) {
    bool solidValue = false;
    if (subgridIsUniform(node, ox, oy, oz, span, solidValue)) {
        if (!solidValue) {
            return;
        }

        const float x0 = static_cast<float>(baseX + ox);
        const float y0 = static_cast<float>(baseY + oy);
        const float z0 = static_cast<float>(baseZ + oz);
        const float x1 = x0 + static_cast<float>(span);
        const float y1 = y0 + static_cast<float>(span);
        const float z1 = z0 + static_cast<float>(span);
        appendHexahedron(cells, x0, y0, z0, x1, y1, z1, depth, 0, span);
        return;
    }

    const uint32_t half = span / 2;
    for (uint32_t childIndex = 0; childIndex < 8; ++childIndex) {
        const uint32_t childOx = ox + ((childIndex & 1u) ? half : 0u);
        const uint32_t childOy = oy + ((childIndex & 2u) ? half : 0u);
        const uint32_t childOz = oz + ((childIndex & 4u) ? half : 0u);
        appendLeafSubgridCells(node, baseX, baseY, baseZ,
                               childOx, childOy, childOz,
                               half, depth + 1, cells);
    }
}

void appendOctreeCells(const OctreeNode* node, int depth,
                       std::vector<VTUCell>& cells) {
    if (!node) {
        return;
    }

    if (node->level == 0) {
        uint32_t bx, by, bz;
        decodeMorton3D(node->mortonCode, bx, by, bz);
        appendLeafSubgridCells(node, bx * 4u, by * 4u, bz * 4u,
                               0u, 0u, 0u, 4u, depth, cells);
        return;
    }

    if (!hasChildren(node)) {
        if (node->inside != 1) {
            return;
        }

        uint32_t bx, by, bz;
        decodeMorton3D(node->mortonCode, bx, by, bz);
        const uint32_t span = nodeSpanVoxels(node->level);
        const float x0 = static_cast<float>(bx * span);
        const float y0 = static_cast<float>(by * span);
        const float z0 = static_cast<float>(bz * span);
        const float x1 = x0 + static_cast<float>(span);
        const float y1 = y0 + static_cast<float>(span);
        const float z1 = z0 + static_cast<float>(span);
        appendHexahedron(cells, x0, y0, z0, x1, y1, z1, depth, node->level, span);
        return;
    }

    for (const auto* child : node->children) {
        appendOctreeCells(child, depth + 1, cells);
    }
}

template <typename T>
void writeValues(std::ofstream& out, const std::vector<T>& values, int perLine = 12) {
    for (size_t i = 0; i < values.size(); ++i) {
        if (i % static_cast<size_t>(perLine) == 0) {
            out << "          ";
        }
        out << values[i];
        if (i + 1 != values.size()) {
            out << " ";
        }
        if ((i + 1) % static_cast<size_t>(perLine) == 0 || i + 1 == values.size()) {
            out << "\n";
        }
    }
}

} // namespace

void exportSparseOctreeVTU(const SparseOctree& octree, uint32_t d,
                           const std::string& outputPath) {
    if (octree.levels.empty() || octree.levels[octree.height].empty()) {
        std::cerr << "No hay raiz valida para exportar VTU." << std::endl;
        return;
    }

    std::vector<VTUCell> cells;
    appendOctreeCells(octree.levels[octree.height].front(), 0, cells);

    std::vector<float> points;
    std::vector<int> connectivity;
    std::vector<int> offsets;
    std::vector<int> types;
    std::vector<int> depths;
    std::vector<int> levels;
    std::vector<uint32_t> spans;

    points.reserve(cells.size() * 24);
    connectivity.reserve(cells.size() * 8);
    offsets.reserve(cells.size());
    types.reserve(cells.size());
    depths.reserve(cells.size());
    levels.reserve(cells.size());
    spans.reserve(cells.size());

    int pointOffset = 0;
    for (const auto& cell : cells) {
        points.insert(points.end(), cell.points.begin(), cell.points.end());
        for (int i = 0; i < 8; ++i) {
            connectivity.push_back(pointOffset + i);
        }
        pointOffset += 8;
        offsets.push_back(pointOffset);
        types.push_back(12); // VTK_HEXAHEDRON
        depths.push_back(cell.depth);
        levels.push_back(cell.level);
        spans.push_back(cell.span);
    }

    std::ofstream out(outputPath);
    out << "<?xml version=\"1.0\"?>\n";
    out << "<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" byte_order=\"LittleEndian\">\n";
    out << "  <UnstructuredGrid>\n";
    out << "    <Piece NumberOfPoints=\"" << points.size() / 3
        << "\" NumberOfCells=\"" << cells.size() << "\">\n";
    out << "      <Points>\n";
    out << "        <DataArray type=\"Float32\" NumberOfComponents=\"3\" format=\"ascii\">\n";
    for (size_t i = 0; i < points.size(); i += 3) {
        out << "          " << points[i] << " " << points[i + 1] << " " << points[i + 2] << "\n";
    }
    out << "        </DataArray>\n";
    out << "      </Points>\n";
    out << "      <Cells>\n";
    out << "        <DataArray type=\"Int32\" Name=\"connectivity\" format=\"ascii\">\n";
    writeValues(out, connectivity, 16);
    out << "        </DataArray>\n";
    out << "        <DataArray type=\"Int32\" Name=\"offsets\" format=\"ascii\">\n";
    writeValues(out, offsets, 16);
    out << "        </DataArray>\n";
    out << "        <DataArray type=\"UInt8\" Name=\"types\" format=\"ascii\">\n";
    writeValues(out, types, 16);
    out << "        </DataArray>\n";
    out << "      </Cells>\n";
    out << "      <CellData>\n";
    out << "        <DataArray type=\"Int32\" Name=\"depth\" format=\"ascii\">\n";
    writeValues(out, depths, 16);
    out << "        </DataArray>\n";
    out << "        <DataArray type=\"Int32\" Name=\"level\" format=\"ascii\">\n";
    writeValues(out, levels, 16);
    out << "        </DataArray>\n";
    out << "        <DataArray type=\"UInt32\" Name=\"cell_span\" format=\"ascii\">\n";
    writeValues(out, spans, 16);
    out << "        </DataArray>\n";
    out << "      </CellData>\n";
    out << "    </Piece>\n";
    out << "  </UnstructuredGrid>\n";
    out << "</VTKFile>\n";

    out.close();
    std::cout << "Exportado Sparse Octree VTU: " << outputPath
              << " (" << cells.size() << " celdas, d=" << d << ")" << std::endl;
}
