#include "common/octree.h"
#include "common/morton.h"
#include <iostream>
#include <algorithm>
#include <cmath>

namespace {

uint32_t nodeSpanVoxels(int level) {
    return 4u << level;
}

size_t gridIndex(uint32_t d, uint32_t x, uint32_t y, uint32_t z) {
    return (static_cast<size_t>(z) * d + y) * d + x;
}

OctreeNode* buildNodeFromDenseGrid(SparseOctree& octree,
                                   const std::vector<uint8_t>& solidGrid,
                                   uint32_t d,
                                   int level,
                                   uint32_t bx,
                                   uint32_t by,
                                   uint32_t bz) {
    const uint32_t span = nodeSpanVoxels(level);
    const uint32_t ox = bx * span;
    const uint32_t oy = by * span;
    const uint32_t oz = bz * span;

    bool allZero = true;
    bool allOne = true;

    for (uint32_t z = oz; z < oz + span; ++z) {
        for (uint32_t y = oy; y < oy + span; ++y) {
            for (uint32_t x = ox; x < ox + span; ++x) {
                bool solid = solidGrid[gridIndex(d, x, y, z)] != 0;
                allZero &= !solid;
                allOne &= solid;
                if (!allZero && !allOne) {
                    goto block_state_decided;
                }
            }
        }
    }

block_state_decided:
    if (allZero) {
        return nullptr;
    }

    OctreeNode* node = new OctreeNode();
    node->mortonCode = encodeMorton3D(bx, by, bz);
    node->level = level;
    octree.levels[level].push_back(node);
    octree.nodeMaps[level][node->mortonCode] = node;

    if (level == 0) {
        for (uint32_t z = 0; z < 4; ++z) {
            for (uint32_t y = 0; y < 4; ++y) {
                for (uint32_t x = 0; x < 4; ++x) {
                    node->SG[x][y][z] = solidGrid[gridIndex(d, ox + x, oy + y, oz + z)] ? 1 : 0;
                }
            }
        }
        node->inside = allOne ? 1 : 0;
        return node;
    }

    if (allOne) {
        node->inside = 1;
        return node;
    }

    for (uint32_t childIndexValue = 0; childIndexValue < 8; ++childIndexValue) {
        uint32_t childBx = bx * 2 + (childIndexValue & 1u);
        uint32_t childBy = by * 2 + ((childIndexValue >> 1) & 1u);
        uint32_t childBz = bz * 2 + ((childIndexValue >> 2) & 1u);
        OctreeNode* child = buildNodeFromDenseGrid(octree, solidGrid, d, level - 1, childBx, childBy, childBz);
        if (child) {
            linkNodeToParent(child, node, childIndexValue);
        }
    }

    return node;
}

} // namespace

// ============================================================================
// Destructor del octree
// ============================================================================

SparseOctree::~SparseOctree() {
    resetSparseOctree(*this);
}

// ============================================================================
// Creación de nodos hoja (nivel 0) a partir de nodos activos de nivel 1
// ============================================================================

void createChildNodes(SparseOctree& octree, const std::vector<uint32_t>& actNodes1) {
    // Cada nodo activo de nivel 1 tiene un código Morton m1.
    // Sus 8 hijos en nivel 0 tienen códigos Morton: m1*8 + j, j=0..7
    // Cada hijo es una hoja con una sub-grid de 4x4x4 vóxeles.

    auto& leaves = octree.levels[0];
    auto& leafMap = octree.nodeMaps[0];

    for (uint32_t m1 : actNodes1) {
        for (uint32_t j = 0; j < 8; ++j) {
            uint32_t childMorton = m1 * 8 + j;

            OctreeNode* leaf = new OctreeNode();
            leaf->mortonCode = childMorton;
            leaf->level = 0;

            leaves.push_back(leaf);
            leafMap[childMorton] = leaf;
        }
    }

    std::cout << "  Hojas creadas (nivel 0): " << leaves.size() << std::endl;
}

// ============================================================================
// Alocación de un nodo padre
// ============================================================================

OctreeNode* allocateParentGroup(SparseOctree& octree, uint32_t pMorton, int level) {
    OctreeNode* parent = new OctreeNode();
    parent->mortonCode = pMorton;
    parent->level = level;

    octree.levels[level].push_back(parent);
    octree.nodeMaps[level][pMorton] = parent;

    return parent;
}

// ============================================================================
// Enlace hijo -> padre
// ============================================================================

void linkNodeToParent(OctreeNode* child, OctreeNode* parent, uint32_t j) {
    parent->children[j] = child;
    child->parent = parent;
}

// ============================================================================
// Propagación de vecindad en X (top-down)
// ============================================================================

void propagateXNeighbors(OctreeNode* node) {
    // Para cada hijo de este nodo, determinar su vecino en +x y -x
    // basándose en la estructura del octree y la vecindad del padre.
    //
    // En un octree, los 8 hijos se organizan según su posición relativa
    // en x, y, z (bits 0, 1, 2 del childIndex).
    // El bit 0 del childIndex corresponde a x:
    //   bit 0 = 0 → mitad -x
    //   bit 0 = 1 → mitad +x

    for (int j = 0; j < 8; ++j) {
        OctreeNode* child = node->children[j];
        if (!child) continue;

        int xBit = j & 1;  // Posición en x (0 o 1)
        int yzBits = j & 6; // Bits y, z

        if (xBit == 0) {
            // Hijo en la mitad -x
            // Su vecino +x es el hermano con xBit=1 (mismo yz)
            OctreeNode* sibling = node->children[yzBits | 1];
            if (sibling) {
                child->xNeighborPos = sibling;
                sibling->xNeighborNeg = child;
            }

            // Su vecino -x viene del vecino -x del padre
            if (node->xNeighborNeg) {
                OctreeNode* parentNeighbor = node->xNeighborNeg;
                // El hijo correspondiente del vecino del padre (con xBit=1, mismo yz)
                OctreeNode* neighborChild = parentNeighbor->children[yzBits | 1];
                if (neighborChild) {
                    child->xNeighborNeg = neighborChild;
                    neighborChild->xNeighborPos = child;
                }
            }
        } else {
            // Hijo en la mitad +x (xBit=1)
            // Su vecino +x viene del vecino +x del padre
            if (node->xNeighborPos) {
                OctreeNode* parentNeighbor = node->xNeighborPos;
                // El hijo correspondiente del vecino del padre (con xBit=0, mismo yz)
                OctreeNode* neighborChild = parentNeighbor->children[yzBits | 0];
                if (neighborChild) {
                    child->xNeighborPos = neighborChild;
                    neighborChild->xNeighborNeg = child;
                }
            }
            // Su vecino -x ya fue asignado cuando procesamos el hermano con xBit=0
        }
    }
}

// ============================================================================
// Descenso completo desde la raíz hasta la hoja
// ============================================================================

OctreeNode* fullDescent(SparseOctree& octree, uint32_t morton0, int height) {
    // Buscar directamente en el mapa de hojas (nivel 0)
    // El código Morton de la hoja se obtiene del morton0 del vóxel
    // dividido por 4 en cada eje (ya que la SG es 4x4x4).
    //
    // morton0 corresponde a las coordenadas del vóxel en la grilla fina.
    // El código Morton del nodo hoja (nivel 0) corresponde a las coordenadas
    // del bloque de 4x4x4 que contiene este vóxel.

    // Extraer coordenadas del vóxel
    uint32_t vx, vy, vz;
    decodeMorton3D(morton0, vx, vy, vz);

    // Coordenadas del bloque nivel 0 (cada bloque = 4 vóxeles por eje)
    uint32_t bx = vx / 4;
    uint32_t by = vy / 4;
    uint32_t bz = vz / 4;

    uint32_t leafMorton = encodeMorton3D(bx, by, bz);

    auto it = octree.nodeMaps[0].find(leafMorton);
    if (it != octree.nodeMaps[0].end()) {
        return it->second;
    }

    return nullptr; // No existe la hoja (región vacía)
}

void resetSparseOctree(SparseOctree& octree) {
    for (auto& level : octree.levels) {
        for (auto* node : level) {
            delete node;
        }
        level.clear();
    }
    for (auto& nodeMap : octree.nodeMaps) {
        nodeMap.clear();
    }
}

bool isVoxelSolid(const SparseOctree& octree, uint32_t d,
                  uint32_t gx, uint32_t gy, uint32_t gz) {
    if (gx >= d || gy >= d || gz >= d) {
        return false;
    }

    for (int level = octree.height; level >= 0; --level) {
        uint32_t span = nodeSpanVoxels(level);
        uint32_t bx = gx / span;
        uint32_t by = gy / span;
        uint32_t bz = gz / span;
        uint32_t morton = encodeMorton3D(bx, by, bz);

        auto it = octree.nodeMaps[level].find(morton);
        if (it == octree.nodeMaps[level].end()) {
            return false;
        }

        const OctreeNode* node = it->second;
        if (level == 0) {
            return node->SG[gx % 4][gy % 4][gz % 4] != 0;
        }

        bool hasChildren = false;
        for (const auto* child : node->children) {
            if (child) {
                hasChildren = true;
                break;
            }
        }

        if (!hasChildren) {
            return node->inside == 1;
        }
    }

    return false;
}

void rebuildSparseOctreeFromDenseGrid(SparseOctree& octree,
                                      const std::vector<uint8_t>& solidGrid,
                                      uint32_t d) {
    resetSparseOctree(octree);

    octree.height = std::max(1, static_cast<int>(std::log2(d)) - 2);
    octree.levels.resize(octree.height + 1);
    octree.nodeMaps.resize(octree.height + 1);

    OctreeNode* root = buildNodeFromDenseGrid(octree, solidGrid, d, octree.height, 0, 0, 0);
    if (!root) {
        std::cerr << "Advertencia: la grilla densa no contiene voxeles solidos." << std::endl;
    }
}
