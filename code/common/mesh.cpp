#include "common/mesh.h"
#include <iostream>
#include <algorithm>
#include <limits>

// ============================================================================
// Normalización de la malla al cubo [0, d)^3
// ============================================================================

void normalizeMesh(Mesh& mesh, uint32_t d) {
    // Calcular el tamaño del AABB
    Vec3 size = mesh.bounds.max - mesh.bounds.min;

    // Encontrar la dimensión máxima para mantener proporciones
    float maxDim = std::max({size.x, size.y, size.z});

    if (maxDim < 1e-10f) {
        std::cerr << "Advertencia: malla degenerada (tamaño ~0)" << std::endl;
        return;
    }

    // Factor de escala: mapear la dimensión más grande a [0, d)
    // Dejamos un pequeño margen (0.01) para evitar que vóxeles caigan fuera del rango
    float scale = (static_cast<float>(d) - 0.02f) / maxDim;

    // Centrar la malla en el cubo
    Vec3 offset = mesh.bounds.min;

    // Offset adicional para centrar en las dimensiones menores
    float padX = (static_cast<float>(d) - 0.02f - size.x * scale) / 2.0f;
    float padY = (static_cast<float>(d) - 0.02f - size.y * scale) / 2.0f;
    float padZ = (static_cast<float>(d) - 0.02f - size.z * scale) / 2.0f;

    for (auto& tri : mesh.triangles) {
        auto transform = [&](Vec3& v) {
            v.x = (v.x - offset.x) * scale + padX + 0.01f;
            v.y = (v.y - offset.y) * scale + padY + 0.01f;
            v.z = (v.z - offset.z) * scale + padZ + 0.01f;
        };
        transform(tri.v0);
        transform(tri.v1);
        transform(tri.v2);
    }

    // Actualizar AABB global
    mesh.bounds.min = {std::numeric_limits<float>::max(),
                       std::numeric_limits<float>::max(),
                       std::numeric_limits<float>::max()};
    mesh.bounds.max = {std::numeric_limits<float>::lowest(),
                       std::numeric_limits<float>::lowest(),
                       std::numeric_limits<float>::lowest()};

    for (const auto& tri : mesh.triangles) {
        mesh.bounds.min = Vec3::min(mesh.bounds.min, Vec3::min(tri.v0, Vec3::min(tri.v1, tri.v2)));
        mesh.bounds.max = Vec3::max(mesh.bounds.max, Vec3::max(tri.v0, Vec3::max(tri.v1, tri.v2)));
    }

    std::cout << "Malla normalizada a [0, " << d << ")^3" << std::endl;
    std::cout << "  AABB min: (" << mesh.bounds.min.x << ", " << mesh.bounds.min.y << ", " << mesh.bounds.min.z << ")" << std::endl;
    std::cout << "  AABB max: (" << mesh.bounds.max.x << ", " << mesh.bounds.max.y << ", " << mesh.bounds.max.z << ")" << std::endl;
}
