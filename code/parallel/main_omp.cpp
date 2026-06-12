#include "common/mesh.h"
#include "common/timer.h"
#include "io/mesh_loader.h"
#include "io/voxel_export_octree_vtu.h"
#include "parallel/voxelizer_omp.h"

#include <iostream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <filesystem>

namespace fs = std::filesystem;

namespace {

bool isPathInsideDirectory(const fs::path& candidate, const fs::path& directory) {
    fs::path normalizedCandidate = fs::weakly_canonical(candidate);
    fs::path normalizedDirectory = fs::weakly_canonical(directory);

    auto dirIt = normalizedDirectory.begin();
    auto dirEnd = normalizedDirectory.end();
    auto candidateIt = normalizedCandidate.begin();
    auto candidateEnd = normalizedCandidate.end();

    for (; dirIt != dirEnd && candidateIt != candidateEnd; ++dirIt, ++candidateIt) {
        if (*dirIt != *candidateIt) {
            return false;
        }
    }

    return dirIt == dirEnd;
}

std::string resolveInputPath(const std::string& userPath) {
    const fs::path inputRoot = "data/input";
    const fs::path candidate = userPath;

    if (!fs::exists(candidate)) {
        throw std::runtime_error("el archivo de entrada no existe: " + userPath);
    }
    if (!isPathInsideDirectory(candidate, inputRoot)) {
        throw std::runtime_error("solo se aceptan modelos dentro de data/input");
    }

    return candidate.string();
}

std::string resolveOutputPath(const std::string& userPath) {
    const fs::path outputRoot = "data/output";
    fs::create_directories(outputRoot);

    fs::path requested(userPath);
    fs::path filename = requested.filename();
    if (filename.empty()) {
        throw std::runtime_error("la ruta de salida debe incluir un nombre de archivo");
    }

    return (outputRoot / filename).string();
}

} // namespace

// ============================================================================
// Punto de entrada principal
// 
// Uso: voxelizer <mesh.obj> <resolution> [--export-octree-vtu output.vtu]
//
// Ejecuta los 4 algoritmos de voxelización secuencial y reporta tiempos.
// ============================================================================

void printUsage(const char* progName) {
    std::cout << "Uso: " << progName << " <mesh.obj> <resolution> [opciones]" << std::endl;
    std::cout << std::endl;
    std::cout << "Restricciones:" << std::endl;
    std::cout << "  - La entrada debe estar dentro de data/input" << std::endl;
    std::cout << "  - Toda salida se guarda dentro de data/output" << std::endl;
    std::cout << std::endl;
    std::cout << "Opciones:" << std::endl;
    std::cout << "  --export-octree-vtu <archivo> Exportar el Sparse Octree como hexaedros adaptativos VTU" << std::endl;
    std::cout << std::endl;
    std::cout << "Ejemplo:" << std::endl;
    std::cout << "  " << progName << " data/input/nefertiti.obj 256 --export-octree-vtu nefertiti_256_sparse.vtu" << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        printUsage(argv[0]);
        return 1;
    }

    std::string meshPath;
    uint32_t d = static_cast<uint32_t>(std::atoi(argv[2]));

    // Validar que d sea potencia de 2 y múltiplo de 8
    if (d == 0 || (d & (d - 1)) != 0) {
        std::cerr << "Error: la resolucion debe ser potencia de 2 (e.g., 32, 64, 128, 256, 512)" << std::endl;
        return 1;
    }
    if (d < 8) {
        std::cerr << "Error: la resolucion minima es 8" << std::endl;
        return 1;
    }

    // Parsear opciones
    std::string exportOctreeVTU;
    for (int i = 3; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--export-octree-vtu" && i + 1 < argc) {
            exportOctreeVTU = resolveOutputPath(argv[++i]);
        }
    }

    // Calcular altura del octree
    // h = log2(d) - 2, porque las hojas tienen sub-grids de 4^3 = (2^2)^3
    // El nivel 0 del octree tiene bloques de 4x4x4 vóxeles
    // El nivel 1 tiene bloques de 8x8x8 = (d/d1) donde d1 = d/8
    int h = static_cast<int>(std::log2(d)) - 2;
    if (h < 1) h = 1;

    std::cout << "========================================" << std::endl;
    std::cout << "  Voxelizacion Secuencial - Benchmark" << std::endl;
    std::cout << "========================================" << std::endl;

    // ---- Paso 0: Cargar malla ----
    Timer timerLoad;
    timerLoad.start();
    meshPath = resolveInputPath(argv[1]);
    Mesh mesh = loadMesh(meshPath);
    normalizeMesh(mesh, d);
    timerLoad.stop();

    std::cout << std::endl;
    std::cout << "Malla: " << meshPath << " (" << mesh.triangles.size() << " triangulos)" << std::endl;
    std::cout << "Resolucion: d = " << d << std::endl;
    std::cout << "Altura octree: h = " << h << std::endl;
    std::cout << "Carga + normalizacion: " << timerLoad.elapsedMs() << " ms" << std::endl;
    std::cout << std::endl;

    // ---- Algoritmo 1: Nodos activos nivel 1 ----
    Timer timer1;
    timer1.start();
    std::vector<uint32_t> actNodes1 = determineActiveLevel1Nodes(mesh, d);
    timer1.stop();

    std::cout << std::endl;

    // ---- Algoritmo 2: Construcción del octree disperso ----
    Timer timer2;
    timer2.start();
    SparseOctree octree = buildSparseOctree(actNodes1, h);
    timer2.stop();

    std::cout << std::endl;

    // ---- Algoritmo 3: Voxelización en el octree ----
    Timer timer3;
    timer3.start();
    voxelizeIntoOctree(mesh, octree, h);
    timer3.stop();

    std::cout << std::endl;

    // ---- Algoritmo 4: Propagación interior/exterior ----
    Timer timer4;
    timer4.start();
    propagateInsideOutside(octree);
    timer4.stop();

    // ---- Resultados ----
    double totalTs = timer1.elapsedMs() + timer2.elapsedMs() + timer3.elapsedMs() + timer4.elapsedMs();
    uint64_t solidVoxels = countSolidVoxels(octree);

    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Resultados del Benchmark" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Malla: " << meshPath << " (" << mesh.triangles.size() << " triangulos)" << std::endl;
    std::cout << "Resolucion: d = " << d << std::endl;
    std::cout << "Altura octree: h = " << h << std::endl;
    std::cout << std::endl;

    // Formateo alineado de tiempos
    auto printTime = [](const std::string& label, double ms) {
        std::cout << label;
        int padding = 45 - static_cast<int>(label.size());
        for (int i = 0; i < padding; ++i) std::cout << " ";
        std::cout << ms << " ms" << std::endl;
    };

    printTime("Algoritmo 1 (Nodos activos L1):", timer1.elapsedMs());
    printTime("Algoritmo 2 (Construccion octree):", timer2.elapsedMs());
    printTime("Algoritmo 3 (Voxelizacion):", timer3.elapsedMs());
    printTime("Algoritmo 4 (Propagacion I/E):", timer4.elapsedMs());
    std::cout << "----------------------------------------" << std::endl;
    printTime("Total Ts:", totalTs);
    std::cout << std::endl;
    std::cout << "Voxeles solidos: " << solidVoxels << std::endl;
    std::cout << "========================================" << std::endl;

    // ---- Exportación ----
    if (!exportOctreeVTU.empty()) {
        exportSparseOctreeVTU(octree, d, exportOctreeVTU);
    }

    return 0;
}
