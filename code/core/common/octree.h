#pragma once

#include "common/mesh.h"
#include <vector>
#include <cstdint>
#include <unordered_map>
#include <cstring>

// ============================================================================
// Estructura del Octree Disperso para voxelización sólida
//
// El octree almacena la geometría de forma jerárquica:
// - Nivel 0 (hojas): Contienen sub-grids (SG) de 4x4x4 vóxeles
// - Niveles 1..h: Nodos intermedios que agrupan 8 hijos
//
// Cada nodo hoja tiene un sub-grid de 4x4x4 = 64 vóxeles.
// Esto significa que el nivel 0 del octree corresponde a bloques de 4^3
// vóxeles en la grilla final de resolución d.
// ============================================================================

struct OctreeNode {
    uint32_t mortonCode;         // Código Morton de este nodo en su nivel
    int level;                   // Nivel en el octree (0 = hoja)

    OctreeNode* children[8];     // Punteros a hijos (nullptr si no existe)
    OctreeNode* parent;          // Puntero al padre

    OctreeNode* xNeighborPos;    // Vecino en dirección +x
    OctreeNode* xNeighborNeg;    // Vecino en dirección -x

    // Sub-grid de 4x4x4 vóxeles (solo para hojas, nivel 0)
    // SG[x][y][z] almacena la paridad de intersecciones
    uint8_t SG[4][4][4];

    // Flags para propagación interior/exterior (niveles > 0)
    int flip;
    int inside;

    OctreeNode() : mortonCode(0), level(0), parent(nullptr),
                   xNeighborPos(nullptr), xNeighborNeg(nullptr),
                   flip(0), inside(0) {
        std::memset(children, 0, sizeof(children));
        std::memset(SG, 0, sizeof(SG));
    }
};

struct SparseOctree {
    // levels[i] contiene los nodos del nivel i
    // levels[0] = hojas (con sub-grids)
    // levels[h] = raíz (o nivel más alto)
    std::vector<std::vector<OctreeNode*>> levels;

    int height;  // Altura del árbol (h)

    // Mapas de búsqueda rápida: mortonCode -> nodo, por nivel
    std::vector<std::unordered_map<uint32_t, OctreeNode*>> nodeMaps;

    SparseOctree() : height(0) {}
    ~SparseOctree();
};

// ============================================================================
// Funciones de construcción y navegación del octree
// ============================================================================

/// Crea los nodos hoja (nivel 0) a partir de los nodos activos de nivel 1.
/// Cada nodo activo de nivel 1 genera 8 nodos hoja (sus hijos).
void createChildNodes(SparseOctree& octree, const std::vector<uint32_t>& actNodes1);

/// Aloca un grupo de padre con espacio para 8 hijos en el nivel dado.
OctreeNode* allocateParentGroup(SparseOctree& octree, uint32_t parentMorton, int level);

/// Enlaza un nodo hijo a su padre en la posición j (0..7).
void linkNodeToParent(OctreeNode* child, OctreeNode* parent, uint32_t j);

/// Propaga las relaciones de vecindad en X hacia los hijos (top-down).
void propagateXNeighbors(OctreeNode* node);

/// Descenso completo desde la raíz hasta la hoja correspondiente a un código Morton.
/// Retorna el nodo hoja que contiene el vóxel, o nullptr si no existe.
OctreeNode* fullDescent(SparseOctree& octree, uint32_t morton0, int height);

/// Elimina todos los nodos y reinicia la estructura interna del octree.
void resetSparseOctree(SparseOctree& octree);

/// Retorna true si el vóxel fino global (gx, gy, gz) está marcado como sólido.
bool isVoxelSolid(const SparseOctree& octree, uint32_t d,
                  uint32_t gx, uint32_t gy, uint32_t gz);

/// Reconstruye un octree disperso a partir de una grilla binaria densa.
void rebuildSparseOctreeFromDenseGrid(SparseOctree& octree,
                                      const std::vector<uint8_t>& solidGrid,
                                      uint32_t d);
