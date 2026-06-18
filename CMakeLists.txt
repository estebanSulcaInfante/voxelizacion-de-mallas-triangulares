cmake_minimum_required(VERSION 3.16)
project(VoxelizacionDeMallasTriangulares LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O2")

find_package(OpenMP QUIET)

add_library(voxel_common
    code/core/common/mesh.cpp
    code/core/common/morton.cpp
    code/core/common/geometry.cpp
    code/core/common/octree.cpp
)

add_library(voxel_io
    code/core/io/mesh_loader.cpp
    code/core/io/voxel_export_octree_vtu.cpp
)

add_library(voxel_seq
    code/core/sequential/algorithm1_active_nodes.cpp
    code/core/sequential/algorithm2_sparse_octree.cpp
    code/core/sequential/algorithm3_voxelize_octree.cpp
    code/core/sequential/algorithm4_inside_outside.cpp
    code/core/sequential/voxelizer_sequential.cpp
)

add_library(voxel_omp
    code/core/parallel/algorithm1_active_nodes_omp.cpp
    code/core/parallel/algorithm2_sparse_octree_omp.cpp
    code/core/parallel/algorithm3_voxelize_octree_omp.cpp
    code/core/parallel/algorithm4_inside_outside_omp.cpp
    code/core/parallel/voxelizer_omp.cpp
)

target_include_directories(voxel_common PUBLIC
    ${CMAKE_SOURCE_DIR}/code
    ${CMAKE_SOURCE_DIR}/code/core
)

target_include_directories(voxel_io PUBLIC
    ${CMAKE_SOURCE_DIR}/code
    ${CMAKE_SOURCE_DIR}/code/core
)

target_include_directories(voxel_seq PUBLIC
    ${CMAKE_SOURCE_DIR}/code
    ${CMAKE_SOURCE_DIR}/code/core
)

target_include_directories(voxel_omp PUBLIC
    ${CMAKE_SOURCE_DIR}/code
    ${CMAKE_SOURCE_DIR}/code/core
)

target_link_libraries(voxel_io PUBLIC voxel_common)
target_link_libraries(voxel_seq PUBLIC voxel_common)
target_link_libraries(voxel_omp PUBLIC voxel_common)

if(OpenMP_CXX_FOUND)
    target_link_libraries(voxel_omp PUBLIC OpenMP::OpenMP_CXX)
endif()

add_executable(voxelizer_seq
    code/apps/voxelizer_seq_main.cpp
)

add_executable(voxelizer_omp
    code/apps/voxelizer_omp_main.cpp
)

add_executable(benchmark_runner
    code/experiments/benchmark/benchmark.cpp
    code/experiments/benchmark/main_benchmark.cpp
)

add_executable(plot_benchmark_totals
    code/experiments/benchmark/plot_benchmark_totals.cpp
)

target_link_libraries(voxelizer_seq PRIVATE voxel_seq voxel_io)
target_link_libraries(voxelizer_omp PRIVATE voxel_omp voxel_io)
target_link_libraries(benchmark_runner PRIVATE voxel_seq voxel_omp voxel_io)

if(MSVC)
    target_compile_options(voxel_common PRIVATE /O2 /W3)
    target_compile_options(voxel_io PRIVATE /O2 /W3)
    target_compile_options(voxel_seq PRIVATE /O2 /W3)
    target_compile_options(voxel_omp PRIVATE /O2 /W3)
    target_compile_options(voxelizer_seq PRIVATE /O2 /W3)
    target_compile_options(voxelizer_omp PRIVATE /O2 /W3)
    set(CMAKE_CXX_FLAGS_RELEASE "/O2")
endif()
