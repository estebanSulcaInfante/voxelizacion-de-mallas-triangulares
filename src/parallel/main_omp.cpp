#include "common/mesh.h"
#include "common/timer.h"
#include "io/mesh_loader.h"
#include "io/voxel_export_points_ply.h"
#include "io/voxel_export_raw.h"
#include "io/voxel_export_surface_ply.h"
#include "parallel/voxelizer_omp.h"

#include <iostream>
#include <string>
#include <cmath>
#include <cstdlib>

// ============================================================================
// Punto de entrada principal
// 
// Uso: voxelizer <mesh.{ply,obj,off}> <resolution> [--export-ply output.ply]
//                 [--export-raw output.raw] [--export-voxel-ply output_surface.ply]
//
// Ejecuta los 4 algoritmos de voxelización secuencial y reporta tiempos.
// ============================================================================

void printUsage(const char* progName) {
    std::cout << "Uso: " << progName << " <mesh.{ply,obj,off}> <resolution> [opciones]" << std::endl;
    std::cout << std::endl;
    std::cout << "Opciones:" << std::endl;
    std::cout << "  --export-ply <archivo>   Exportar voxeles como nube de puntos PLY" << std::endl;
    std::cout << "  --export-raw <archivo>   Exportar voxeles como array binario denso" << std::endl;
    std::cout << "  --export-voxel-ply <archivo> Exportar solo caras externas como malla PLY" << std::endl;
    std::cout << std::endl;
    std::cout << "Ejemplo:" << std::endl;
    std::cout << "  " << progName << " data/bunny/reconstruction/bun_zipper.ply 256 --export-ply output.ply" << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        printUsage(argv[0]);
        return 1;
    }

    std::string meshPath = argv[1];
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
    std::string exportPLY, exportRAW, exportVoxelPLY;
    for (int i = 3; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--export-ply" && i + 1 < argc) {
            exportPLY = argv[++i];
        } else if (arg == "--export-raw" && i + 1 < argc) {
            exportRAW = argv[++i];
        } else if (arg == "--export-voxel-ply" && i + 1 < argc) {
            exportVoxelPLY = argv[++i];
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
    if (!exportPLY.empty()) {
        exportVoxelsPLY(octree, d, exportPLY);
    }
    if (!exportRAW.empty()) {
        exportVoxelsRAW(octree, d, exportRAW);
    }
    if (!exportVoxelPLY.empty()) {
        exportVoxelSurfacePLY(octree, d, exportVoxelPLY);
    }

    return 0;
}
