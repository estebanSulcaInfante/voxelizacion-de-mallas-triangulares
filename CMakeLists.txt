cmake_minimum_required(VERSION 3.16)
project(VoxelizadorSecuencial LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Optimización para benchmarking realista
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# Ejecutable principal
add_executable(voxelizer
    src/main.cpp
    src/mesh.cpp
    src/morton.cpp
    src/geometry.cpp
    src/octree.cpp
    src/voxelizer.cpp
)

# Incluir headers propios y la dependencia externa (happly)
target_include_directories(voxelizer PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/extern
)

# En MSVC, habilitar optimizaciones equivalentes
if(MSVC)
    target_compile_options(voxelizer PRIVATE /O2 /W3)
    set(CMAKE_CXX_FLAGS_RELEASE "/O2")
endif()
