#pragma once

#include "mesh.h"
#include "octree.h"
#include <vector>
#include <cstdint>

// ============================================================================
// Interfaz principal de los 4 algoritmos de voxelización secuencial
// Basados en Schwarz & Seidel (2010), adaptados a ejecución secuencial.
// ============================================================================

/// Algoritmo 1: Determinación de nodos activos de nivel 1.
/// Recorre todos los triángulos y marca los vóxeles de la grilla de nivel 1
/// que se solapan con al menos un triángulo (voxelización conservativa).
/// Retorna una lista ordenada (por Morton code) de los nodos activos.
std::vector<uint32_t> determineActiveLevel1Nodes(const Mesh& mesh, uint32_t d);

/// Algoritmo 2: Construcción del octree disperso.
/// A partir de la lista de nodos activos de nivel 1, construye el octree
/// bottom-up y propaga las relaciones de vecindad en X top-down.
SparseOctree buildSparseOctree(const std::vector<uint32_t>& actNodes1, int height);

/// Algoritmo 3: Voxelización dentro del octree.
/// Proyecta los triángulos sobre las hojas del octree para marcar las
/// intersecciones a lo largo del eje X usando la regla de paridad (XOR).
void voxelizeIntoOctree(const Mesh& mesh, SparseOctree& octree, int height);

/// Algoritmo 4: Propagación jerárquica interior/exterior.
/// Reconstruye el volumen sólido usando la regla de paridad acumulada:
/// Fase 1: Propagación en nivel 0 (cadenas de hojas en +x)
/// Fase 2: Propagación ascendente (bottom-up)
/// Fase 3: Propagación descendente (top-down hacia hojas)
void propagateInsideOutside(SparseOctree& octree);

// ============================================================================
// Utilidades de exportación
// ============================================================================

/// Exporta los vóxeles sólidos como una nube de puntos en formato PLY.
/// Cada vóxel sólido se representa como un punto en su centro.
void exportVoxelsPLY(const SparseOctree& octree, uint32_t d,
                      const std::string& outputPath);

/// Exporta los vóxeles como un array binario denso de d x d x d bytes.
/// 1 = sólido, 0 = vacío.
void exportVoxelsRAW(const SparseOctree& octree, uint32_t d,
                      const std::string& outputPath);

/// Cuenta el número total de vóxeles marcados como sólidos.
uint64_t countSolidVoxels(const SparseOctree& octree);
