#include "voxelizer.h"
#include "morton.h"
#include "geometry.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <cstring>

// ============================================================================
// Algoritmo 1: Determinación de nodos activos de nivel 1
//
// Pseudocódigo referencia (Algorithm 1_Secuencial):
// 1. d1 = d / 8, N = d1^3
// 2. A[N] ← 0 (grilla implícita)
// 3. Para cada triángulo T: calcular AABB, test conservativo, marcar A[morton] = 1
// 4. Compactar A en lista ordenada de Morton codes activos
// ============================================================================

std::vector<uint32_t> determineActiveLevel1Nodes(const Mesh& mesh, uint32_t d) {
    // Paso 1: Resolución en nivel 1
    uint32_t d1 = d / 8;  // d1 = d / 2^3
    uint32_t N = d1 * d1 * d1;  // Cantidad total de nodos en nivel 1

    std::cout << "  Algoritmo 1: d1=" << d1 << ", N=" << N << std::endl;

    // Paso 3: Reservar memoria para la grilla implícita
    std::vector<uint8_t> A(N, 0);

    float voxelSize = 8.0f; // Tamaño del vóxel en nivel 1 (en coordenadas de grilla fina)

    // Pasos 6-11: Voxelización conservativa por triángulo
    for (const auto& tri : mesh.triangles) {
        // Paso 7: BoundingBox del triángulo en coordenadas de grilla nivel 1
        uint32_t x_min, y_min, z_min, x_max, y_max, z_max;
        computeTriangleAABBLevel1(tri, d1, x_min, y_min, z_min, x_max, y_max, z_max);

        // Paso 8-11: Para cada vóxel en el AABB, test conservativo
        for (uint32_t z1 = z_min; z1 <= z_max; ++z1) {
            for (uint32_t y1 = y_min; y1 <= y_max; ++y1) {
                for (uint32_t x1 = x_min; x1 <= x_max; ++x1) {
                    // Paso 9: Test conservativo
                    if (conservativeOverlapModified(tri, x1, y1, z1, voxelSize)) {
                        // Paso 10-11: Codificar Morton y marcar como activo
                        uint32_t m1 = encodeMorton3D(x1, y1, z1);
                        if (m1 < N) {
                            A[m1] = 1;
                        }
                    }
                }
            }
        }
    }

    // Pasos 12-15: Compactación secuencial
    std::vector<uint32_t> actNodes1;
    for (uint32_t m1 = 0; m1 < N; ++m1) {
        if (A[m1] == 1) {
            actNodes1.push_back(m1);
        }
    }

    std::cout << "  Nodos activos nivel 1: " << actNodes1.size() << " de " << N << std::endl;

    return actNodes1;
}

// ============================================================================
// Algoritmo 2: Construcción del octree disperso
//
// Pseudocódigo referencia (Algorithm 2_Secuencial):
// 1. Crear nodos hoja (nivel 0) a partir de actNodes1
// 2. Construcción ascendente: agrupar por parent_morton, crear padres
// 3. Propagación descendente de vecinos en X
// ============================================================================

SparseOctree buildSparseOctree(const std::vector<uint32_t>& actNodes1, int height) {
    SparseOctree octree;
    octree.height = height;

    // Inicializar niveles y mapas
    octree.levels.resize(height + 1);
    octree.nodeMaps.resize(height + 1);

    std::cout << "  Algoritmo 2: altura=" << height << std::endl;

    // Paso 1: Generar nodos hoja (nivel 0)
    createChildNodes(octree, actNodes1);

    // Paso 2: Construcción ascendente (Bottom-Up)
    std::vector<uint32_t> actNodes = actNodes1;

    for (int level = 1; level <= height; ++level) {
        std::vector<uint32_t> nextActNodes;

        // Pasos 7-18: Agrupación secuencial directa
        for (size_t i = 0; i < actNodes.size(); ++i) {
            uint32_t parent_m = parentMorton(actNodes[i]);

            // Paso 11: Si es el primer hijo de un nuevo padre
            if (nextActNodes.empty() || nextActNodes.back() != parent_m) {
                nextActNodes.push_back(parent_m);
                // Paso 14: Crear estructura del padre
                allocateParentGroup(octree, parent_m, level);
            }

            // Paso 17-18: Enlazar hijo al padre
            uint32_t j = childIndex(actNodes[i]);

            // Buscar el nodo hijo en el nivel inferior
            auto it = octree.nodeMaps[level - 1].find(actNodes[i]);
            if (it != octree.nodeMaps[level - 1].end()) {
                OctreeNode* child = it->second;
                OctreeNode* parent = octree.nodeMaps[level][parent_m];
                linkNodeToParent(child, parent, j);
            }
        }

        std::cout << "  Nivel " << level << ": " << nextActNodes.size() << " nodos" << std::endl;
        actNodes = nextActNodes;
    }

    // Pasos 21-23: Propagación descendente de vecinos en X
    for (int level = height; level >= 1; --level) {
        for (auto* node : octree.levels[level]) {
            propagateXNeighbors(node);
        }
    }

    std::cout << "  Vecindad en X propagada" << std::endl;

    return octree;
}

// ============================================================================
// Algoritmo 3: Voxelización dentro del octree
//
// Pseudocódigo referencia (Algorithm 3_Secuencial):
// Para cada triángulo: proyectar en YZ, encontrar tiles, test de solapamiento,
// para cada sub-grid vóxel: test de centro, proyectar X, XOR en la hoja.
// ============================================================================

void voxelizeIntoOctree(const Mesh& mesh, SparseOctree& octree, int height) {
    std::cout << "  Algoritmo 3: voxelizando " << mesh.triangles.size() << " triangulos" << std::endl;

    // Tamaño de un tile de nivel 0: 4 vóxeles por eje
    float tileSize = 4.0f;

    uint32_t processedTriangles = 0;

    // Paso 1: Para cada triángulo
    for (const auto& tri : mesh.triangles) {
        processedTriangles++;

        // Paso 2: Bounding box en YZ
        float y_min_f, y_max_f, z_min_f, z_max_f;
        computeBoundingBoxYZ(tri, y_min_f, y_max_f, z_min_f, z_max_f);

        // Paso 3: Rango de tiles nivel 0 en YZ
        int y0_min = static_cast<int>(std::max(0.0f, std::floor(y_min_f / tileSize)));
        int y0_max = static_cast<int>(std::floor(y_max_f / tileSize));
        int z0_min = static_cast<int>(std::max(0.0f, std::floor(z_min_f / tileSize)));
        int z0_max = static_cast<int>(std::floor(z_max_f / tileSize));

        // Paso 4: Edge functions en YZ
        TriangleEdgeFunctions eyz = setupEdgeFunctionsYZ(tri);

        // Paso 6: Para cada tile en el rango YZ
        for (int z0 = z0_min; z0 <= z0_max; ++z0) {
            for (int y0 = y0_min; y0 <= y0_max; ++y0) {
                // Pasos 10-15: Para cada sub-vóxel del tile (s = 0..15)
                // sy = s/4, sz = s%4 → recorre las 16 posiciones en YZ del sub-grid
                for (int sz = 0; sz < 4; ++sz) {
                    for (int sy = 0; sy < 4; ++sy) {
                        // Paso 13: Centro del sub-grid vóxel en coordenadas globales
                        float cy = y0 * tileSize + sy + 0.5f;
                        float cz = z0 * tileSize + sz + 0.5f;

                        // Paso 15: Test de centro dentro de la proyección YZ
                        if (centerInsideProjectionYZ(cy, cz, tri, eyz)) {
                            // Paso 16: Proyectar centro al plano del triángulo en X
                            float x_tilde = projectCenterToTrianglePlaneX(cy, cz, tri);

                            if (x_tilde < 0) continue; // Fuera del dominio

                            // Paso 17: Código Morton del vóxel en nivel 0
                            uint32_t vx = static_cast<uint32_t>(std::floor(x_tilde));
                            uint32_t vy = static_cast<uint32_t>(y0 * 4 + sy);
                            uint32_t vz = static_cast<uint32_t>(z0 * 4 + sz);

                            // Paso 18: Descenso completo para encontrar la hoja
                            uint32_t m0 = encodeMorton3D(vx / 4, vy / 4, vz / 4);
                            // Buscar directamente en el mapa de hojas
                            auto it = octree.nodeMaps[0].find(m0);
                            if (it == octree.nodeMaps[0].end()) continue;

                            OctreeNode* leaf = it->second;

                            // Paso 19: Posición local dentro de la sub-grid
                            uint32_t xlocal = vx % 4;

                            // Pasos 21-23: XOR desde xlocal hasta 3
                            // Esto marca la paridad de intersecciones a lo largo de x
                            for (uint32_t x = xlocal; x < 4; ++x) {
                                leaf->SG[x][sy][sz] ^= 1;
                            }
                        }
                    }
                }
            }
        }
    }

    std::cout << "  Triangulos procesados: " << processedTriangles << std::endl;
}

// ============================================================================
// Forward declarations para funciones auxiliares de propagación
// ============================================================================
static bool childBoundaryRequiresFlipImpl(OctreeNode* node, SparseOctree& octree);
static void flipInsideStateImpl(OctreeNode* node);

// ============================================================================
// Algoritmo 4: Propagación jerárquica interior/exterior
//
// Pseudocódigo referencia (Algorithm 4_Secuencial):
// Fase 1: Propagación en nivel 0 (cadenas de hojas en +x)
// Fase 2: Propagación ascendente con flip/inside
// Fase 3: Propagación descendente (interior hacia hojas)
// ============================================================================

void propagateInsideOutside(SparseOctree& octree) {
    int h = octree.height;

    std::cout << "  Algoritmo 4: propagacion I/E" << std::endl;

    // ---- Fase 1: Propagación en nivel 0 (columnas en x) ----
    // Para cada hoja que NO tiene vecino en -x (inicio de cadena):
    // recorrer la cadena en +x y propagar la paridad.

    int chainsProcessed = 0;
    for (auto* leaf : octree.levels[0]) {
        if (leaf->xNeighborNeg == nullptr) {
            // Inicio de cadena
            chainsProcessed++;
            OctreeNode* curr = leaf;
            while (curr->xNeighborPos != nullptr) {
                OctreeNode* next = curr->xNeighborPos;
                if (hasActiveBoundaryX(curr->SG)) {
                    flipAllSGVoxels(next->SG);
                }
                curr = next;
            }
        }
    }
    std::cout << "    Fase 1: " << chainsProcessed << " cadenas procesadas" << std::endl;

    // ---- Fase 2: Propagación jerárquica ascendente ----
    for (int level = 1; level <= h; ++level) {
        // Inicialización y verificación en un solo bucle
        for (auto* node : octree.levels[level]) {
            node->flip = 0;
            node->inside = 0;

            // Verificar si algún hijo de frontera requiere flip
            // Un hijo en la posición +x (childIndex con bit 0 = 1) que tiene
            // paridad activa propagará el flip hacia arriba.
            if (childBoundaryRequiresFlipImpl(node, octree)) {
                node->flip = 1;
            }
        }

        // Propagación en el nivel actual a lo largo del eje x
        for (auto* head : octree.levels[level]) {
            if (head->xNeighborNeg == nullptr) {
                OctreeNode* curr = head;
                while (curr->xNeighborPos != nullptr) {
                    OctreeNode* next = curr->xNeighborPos;
                    if (curr->flip == 1) {
                        next->flip = 1 - next->flip;
                        next->inside = 1 - next->inside;
                    }
                    curr = next;
                }
            }
        }

        std::cout << "    Fase 2, nivel " << level << ": " << octree.levels[level].size() << " nodos" << std::endl;
    }

    // ---- Fase 3: Propagación descendente (interior hacia hojas) ----
    for (int level = h; level >= 1; --level) {
        for (auto* node : octree.levels[level]) {
            if (node->inside == 1) {
                for (int j = 0; j < 8; ++j) {
                    if (node->children[j]) {
                        flipInsideStateImpl(node->children[j]);
                    }
                }
            }
        }
    }

    std::cout << "    Fase 3: propagacion descendente completa" << std::endl;
}

// ============================================================================
// Funciones auxiliares para la propagación
// ============================================================================

/// Verifica si la frontera de los hijos de un nodo requiere flip.
/// Un nodo requiere flip si la paridad acumulada en su borde +x es impar.
static bool childBoundaryRequiresFlipImpl(OctreeNode* node, SparseOctree& octree) {
    // Para un nodo en nivel > 0, verificar los hijos en la posición +x
    // (los que tienen bit 0 = 1 en su childIndex)
    for (int j = 0; j < 8; ++j) {
        if ((j & 1) == 1) { // Hijos en la mitad +x
            OctreeNode* child = node->children[j];
            if (child) {
                if (child->level == 0) {
                    // Es una hoja, verificar su SG
                    if (hasActiveBoundaryX(child->SG)) {
                        return true;
                    }
                } else {
                    // Nivel intermedio, verificar recursivamente
                    if (child->flip == 1) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

/// Invierte el estado interior de un nodo (usado en la Fase 3 descendente).
static void flipInsideStateImpl(OctreeNode* node) {
    if (node->level == 0) {
        // Hoja: invertir toda la sub-grid
        flipAllSGVoxels(node->SG);
    } else {
        // Nodo intermedio: invertir flag de inside
        node->inside = 1 - node->inside;
    }
}



// ============================================================================
// Exportación de resultados
// ============================================================================

void exportVoxelsPLY(const SparseOctree& octree, uint32_t d,
                      const std::string& outputPath) {
    // Recolectar todos los vóxeles sólidos
    std::vector<Vec3> points;

    for (const auto* leaf : octree.levels[0]) {
        // Obtener coordenadas del bloque
        uint32_t bx, by, bz;
        decodeMorton3D(leaf->mortonCode, bx, by, bz);

        for (int x = 0; x < 4; ++x) {
            for (int y = 0; y < 4; ++y) {
                for (int z = 0; z < 4; ++z) {
                    if (leaf->SG[x][y][z]) {
                        // Centro del vóxel en coordenadas globales
                        float gx = bx * 4.0f + x + 0.5f;
                        float gy = by * 4.0f + y + 0.5f;
                        float gz = bz * 4.0f + z + 0.5f;
                        points.push_back({gx, gy, gz});
                    }
                }
            }
        }
    }

    // Escribir PLY
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

void exportVoxelsRAW(const SparseOctree& octree, uint32_t d,
                      const std::string& outputPath) {
    // Array denso d x d x d
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

uint64_t countSolidVoxels(const SparseOctree& octree) {
    uint64_t count = 0;
    for (const auto* leaf : octree.levels[0]) {
        for (int x = 0; x < 4; ++x) {
            for (int y = 0; y < 4; ++y) {
                for (int z = 0; z < 4; ++z) {
                    if (leaf->SG[x][y][z]) {
                        count++;
                    }
                }
            }
        }
    }
    return count;
}
