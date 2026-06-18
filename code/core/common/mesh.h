#pragma once

#include <vector>
#include <string>
#include <cmath>
#include <array>
#include <cstdint>

// ============================================================================
// Estructuras fundamentales para la malla triangular
// ============================================================================

struct Vec3 {
    float x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }

    // Producto cruz
    Vec3 cross(const Vec3& o) const {
        return {y * o.z - z * o.y,
                z * o.x - x * o.z,
                x * o.y - y * o.x};
    }

    // Producto punto
    float dot(const Vec3& o) const {
        return x * o.x + y * o.y + z * o.z;
    }

    // Componente mínima/máxima por eje
    static Vec3 min(const Vec3& a, const Vec3& b) {
        return {std::fmin(a.x, b.x), std::fmin(a.y, b.y), std::fmin(a.z, b.z)};
    }
    static Vec3 max(const Vec3& a, const Vec3& b) {
        return {std::fmax(a.x, b.x), std::fmax(a.y, b.y), std::fmax(a.z, b.z)};
    }
};

struct Triangle {
    Vec3 v0, v1, v2;
};

struct AABB {
    Vec3 min, max;
};

struct Mesh {
    std::vector<Triangle> triangles;
    AABB bounds;  // AABB global de toda la malla
};

// ============================================================================
// Funciones de normalización
// ============================================================================

/// Normaliza la malla al cubo [0, d)^3, manteniendo proporciones
void normalizeMesh(Mesh& mesh, uint32_t d);
