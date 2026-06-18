#pragma once

#include <cstdint>
#include <string>
#include <vector>

struct BenchmarkResult {
    std::string model;
    std::string implementation;
    uint32_t resolution = 0;
    int threads = 1;
    double loadMs = 0.0;
    double alg1Ms = 0.0;
    double alg2Ms = 0.0;
    double alg3Ms = 0.0;
    double alg4Ms = 0.0;
    double totalMs = 0.0;
    uint64_t solidVoxels = 0;
};

BenchmarkResult runSequentialBenchmark(const std::string& meshPath, uint32_t resolution);
BenchmarkResult runParallelBenchmark(const std::string& meshPath, uint32_t resolution, int threads);
void writeBenchmarkCSV(const std::string& outputPath, const std::vector<BenchmarkResult>& results);
