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
    src/common/mesh.cpp
    src/common/morton.cpp
    src/common/geometry.cpp
    src/common/octree.cpp
)

add_library(voxel_io
    src/io/mesh_loader.cpp
    src/io/voxel_export_points_ply.cpp
    src/io/voxel_export_raw.cpp
    src/io/voxel_export_surface_ply.cpp
)

add_library(voxel_seq
    src/sequential/algorithm1_active_nodes.cpp
    src/sequential/algorithm2_sparse_octree.cpp
    src/sequential/algorithm3_voxelize_octree.cpp
    src/sequential/algorithm4_inside_outside.cpp
    src/sequential/voxelizer_sequential.cpp
)

add_library(voxel_omp
    src/parallel/algorithm1_active_nodes_omp.cpp
    src/parallel/algorithm2_sparse_octree_omp.cpp
    src/parallel/algorithm3_voxelize_octree_omp.cpp
    src/parallel/algorithm4_inside_outside_omp.cpp
    src/parallel/voxelizer_omp.cpp
)

target_include_directories(voxel_common PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/extern
)

target_include_directories(voxel_io PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/extern
)

target_include_directories(voxel_seq PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/extern
)

target_include_directories(voxel_omp PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/extern
)

target_link_libraries(voxel_io PUBLIC voxel_common)
target_link_libraries(voxel_seq PUBLIC voxel_common)
target_link_libraries(voxel_omp PUBLIC voxel_common)

if(OpenMP_CXX_FOUND)
    target_link_libraries(voxel_omp PUBLIC OpenMP::OpenMP_CXX)
endif()

add_executable(voxelizer_seq
    src/sequential/main_seq.cpp
)

add_executable(voxelizer_omp
    src/parallel/main_omp.cpp
)

target_link_libraries(voxelizer_seq PRIVATE voxel_seq voxel_io)
target_link_libraries(voxelizer_omp PRIVATE voxel_omp voxel_io)

if(MSVC)
    target_compile_options(voxel_common PRIVATE /O2 /W3)
    target_compile_options(voxel_io PRIVATE /O2 /W3)
    target_compile_options(voxel_seq PRIVATE /O2 /W3)
    target_compile_options(voxel_omp PRIVATE /O2 /W3)
    target_compile_options(voxelizer_seq PRIVATE /O2 /W3)
    target_compile_options(voxelizer_omp PRIVATE /O2 /W3)
    set(CMAKE_CXX_FLAGS_RELEASE "/O2")
endif()
