#include "experiments/benchmark/benchmark.h"

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

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
    const fs::path outputRoot = "data/output/benchmarks";
    fs::create_directories(outputRoot);

    fs::path requested(userPath);
    fs::path filename = requested.filename();
    if (filename.empty()) {
        throw std::runtime_error("la ruta de salida debe incluir un nombre de archivo");
    }

    return (outputRoot / filename).string();
}

void printUsage(const char* progName) {
    std::cout << "Uso: " << progName << " [--model <archivo.obj>] [--output <archivo.csv>]" << std::endl;
    std::cout << std::endl;
    std::cout << "Defaults:" << std::endl;
    std::cout << "  model  = data/input/happy.obj" << std::endl;
    std::cout << "  output = data/output/benchmarks/benchmark_happy.csv" << std::endl;
    std::cout << "  threads = 1,2,4,8,16" << std::endl;
    std::cout << "  resolutions = 64,128,256,512,1024" << std::endl;
}

} // namespace

int main(int argc, char* argv[]) {
    try {
        std::string modelPath = "data/input/happy.obj";
        std::string csvPath = "benchmark_happy.csv";

        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg == "--model" && i + 1 < argc) {
                modelPath = argv[++i];
            } else if (arg == "--output" && i + 1 < argc) {
                csvPath = argv[++i];
            } else if (arg == "--help" || arg == "-h") {
                printUsage(argv[0]);
                return 0;
            }
        }

        modelPath = resolveInputPath(modelPath);
        csvPath = resolveOutputPath(csvPath);

        const std::vector<int> threadCounts = {1, 2, 4, 8, 16};
        const std::vector<uint32_t> resolutions = {64, 128, 256, 512, 1024};

        std::vector<BenchmarkResult> results;
        results.reserve(threadCounts.size() * resolutions.size());

        for (uint32_t resolution : resolutions) {
            for (int threads : threadCounts) {
                std::cout << "[BENCH] model=" << modelPath
                          << " resolution=" << resolution
                          << " threads=" << threads << std::endl;

                BenchmarkResult result = (threads == 1)
                    ? runSequentialBenchmark(modelPath, resolution)
                    : runParallelBenchmark(modelPath, resolution, threads);

                results.push_back(result);
            }
        }

        writeBenchmarkCSV(csvPath, results);
        std::cout << "[BENCH] CSV generado en: " << csvPath << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
