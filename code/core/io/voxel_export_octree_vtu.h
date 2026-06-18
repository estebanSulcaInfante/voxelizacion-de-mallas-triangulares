#pragma once

#include "common/octree.h"
#include <string>

/// Exporta el volumen disperso como un UnstructuredGrid (.vtu) de hexaedros.
/// Cada celda representa un bloque sólido adaptativo del octree.
/// La salida incluye arreglos de celda:
/// - depth: profundidad visual dentro de la jerarquía exportada
/// - level: nivel original del octree
/// - cell_span: tamaño del bloque en vóxeles
void exportSparseOctreeVTU(const SparseOctree& octree, uint32_t d,
                           const std::string& outputPath);
