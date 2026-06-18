#include "experiments/benchmark/benchmark.h"
#include "common/mesh.h"
#include "common/timer.h"
#include "io/mesh_loader.h"
#include "parallel/voxelizer_omp.h"
#include "sequential/voxelizer_sequential.h"

#include <cmath>
#include <fstream>
#include <stdexcept>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace {

int octreeHeightFromResolution(uint32_t d) {
    int h = static_cast<int>(std::log2(d)) - 2;
    return h < 1 ? 1 : h;
}

void validateResolution(uint32_t resolution) {
    if (resolution == 0 || (resolution & (resolution - 1)) != 0 || resolution < 8) {
        throw std::runtime_error("la resolucion debe ser potencia de 2 y al menos 8");
    }
}

double nowOmpMs() {
#ifdef _OPENMP
    return omp_get_wtime() * 1000.0;
#else
    return 0.0;
#endif
}

BenchmarkResult initializeResult(const std::string& meshPath,
                                 uint32_t resolution,
                                 int threads,
                                 const std::string& implementation) {
    BenchmarkResult result;
    result.model = meshPath;
    result.implementation = implementation;
    result.resolution = resolution;
    result.threads = threads;
    return result;
}

Mesh loadNormalizedMesh(const std::string& meshPath, uint32_t resolution, double& elapsedMs) {
    Timer timerLoad;
    timerLoad.start();
    Mesh mesh = loadMesh(meshPath);
    normalizeMesh(mesh, resolution);
    timerLoad.stop();
    elapsedMs = timerLoad.elapsedMs();
    return mesh;
}

} // namespace

BenchmarkResult runSequentialBenchmark(const std::string& meshPath, uint32_t resolution) {
    validateResolution(resolution);

    BenchmarkResult result = initializeResult(meshPath, resolution, 1, "seq");
    const int height = octreeHeightFromResolution(resolution);

    Mesh mesh = loadNormalizedMesh(meshPath, resolution, result.loadMs);

    Timer timer1;
    timer1.start();
    std::vector<uint32_t> actNodes1 = seq::determineActiveLevel1Nodes(mesh, resolution);
    timer1.stop();
    result.alg1Ms = timer1.elapsedMs();

    Timer timer2;
    timer2.start();
    SparseOctree octree = seq::buildSparseOctree(actNodes1, height);
    timer2.stop();
    result.alg2Ms = timer2.elapsedMs();

    Timer timer3;
    timer3.start();
    seq::voxelizeIntoOctree(mesh, octree, height);
    timer3.stop();
    result.alg3Ms = timer3.elapsedMs();

    Timer timer4;
    timer4.start();
    seq::propagateInsideOutside(octree);
    timer4.stop();
    result.alg4Ms = timer4.elapsedMs();

    result.totalMs = result.alg1Ms + result.alg2Ms + result.alg3Ms + result.alg4Ms;
    result.solidVoxels = seq::countSolidVoxels(octree);
    return result;
}

BenchmarkResult runParallelBenchmark(const std::string& meshPath, uint32_t resolution, int threads) {
    validateResolution(resolution);

#ifdef _OPENMP
    omp_set_num_threads(threads);
#endif

    BenchmarkResult result = initializeResult(meshPath, resolution, threads, "omp");
    const int height = octreeHeightFromResolution(resolution);

    Mesh mesh = loadNormalizedMesh(meshPath, resolution, result.loadMs);

    double t0 = nowOmpMs();
    std::vector<uint32_t> actNodes1 = par::determineActiveLevel1Nodes(mesh, resolution);
    double t1 = nowOmpMs();
    result.alg1Ms = t1 - t0;

    t0 = nowOmpMs();
    SparseOctree octree = par::buildSparseOctree(actNodes1, height);
    t1 = nowOmpMs();
    result.alg2Ms = t1 - t0;

    t0 = nowOmpMs();
    par::voxelizeIntoOctree(mesh, octree, height);
    t1 = nowOmpMs();
    result.alg3Ms = t1 - t0;

    t0 = nowOmpMs();
    par::propagateInsideOutside(octree);
    t1 = nowOmpMs();
    result.alg4Ms = t1 - t0;

    result.totalMs = result.alg1Ms + result.alg2Ms + result.alg3Ms + result.alg4Ms;
    result.solidVoxels = par::countSolidVoxels(octree);
    return result;
}

void writeBenchmarkCSV(const std::string& outputPath, const std::vector<BenchmarkResult>& results) {
    std::ofstream out(outputPath);
    if (!out) {
        throw std::runtime_error("no se pudo abrir el archivo CSV de salida: " + outputPath);
    }

    out << "model,implementation,threads,resolution,load_ms,alg1_ms,alg2_ms,alg3_ms,alg4_ms,total_ms,solid_voxels\n";
    for (const auto& result : results) {
        out << result.model << ','
            << result.implementation << ','
            << result.threads << ','
            << result.resolution << ','
            << result.loadMs << ','
            << result.alg1Ms << ','
            << result.alg2Ms << ','
            << result.alg3Ms << ','
            << result.alg4Ms << ','
            << result.totalMs << ','
            << result.solidVoxels << '\n';
    }
}
