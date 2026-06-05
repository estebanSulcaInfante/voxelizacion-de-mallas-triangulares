#include "geometry.h"
#include <algorithm>
#include <cmath>

// ============================================================================
// Test conservativo de solapamiento triángulo-vóxel
// Basado en Akenine-Möller (2001) "Fast 3D Triangle-Box Overlap Testing"
// con ajustes conservativos de Schwarz-Seidel (2010).
//
// El test verifica tres condiciones (SAT — Separating Axis Theorem):
// 1. AABB del triángulo vs AABB del vóxel
// 2. Plano del triángulo vs vóxel
// 3. Ejes cruzados (aristas del triángulo × ejes del vóxel)
// ============================================================================

// ---- Helpers internos ----

/// Verifica si dos intervalos [a_min, a_max] y [b_min, b_max] se solapan
static inline bool intervalsOverlap(float a_min, float a_max, float b_min, float b_max) {
    return a_min <= b_max && b_min <= a_max;
}

/// Proyecta los 3 vértices del triángulo sobre un eje y verifica solapamiento
/// con el intervalo [-halfSize, halfSize] (vóxel centrado en el origen).
static inline bool axisTestSAT(float v0_proj, float v1_proj, float v2_proj, float r) {
    float minP = std::min({v0_proj, v1_proj, v2_proj});
    float maxP = std::max({v0_proj, v1_proj, v2_proj});
    return !(minP > r || maxP < -r);
}

// ============================================================================
// Algoritmo 1: AABB del triángulo en coordenadas de grilla nivel 1
// ============================================================================

void computeTriangleAABBLevel1(const Triangle& tri, uint32_t d1,
                                uint32_t& x_min, uint32_t& y_min, uint32_t& z_min,
                                uint32_t& x_max, uint32_t& y_max, uint32_t& z_max) {
    // Cada celda de nivel 1 tiene tamaño 8 (= 2^3) en coordenadas de la grilla fina
    // Así que dividimos las coordenadas del triángulo entre 8 para obtener
    // coordenadas de la grilla de nivel 1.
    float voxelSize = 8.0f; // Tamaño del vóxel en nivel 1

    Vec3 tmin = Vec3::min(tri.v0, Vec3::min(tri.v1, tri.v2));
    Vec3 tmax = Vec3::max(tri.v0, Vec3::max(tri.v1, tri.v2));

    // Convertir a coordenadas de grilla nivel 1
    x_min = static_cast<uint32_t>(std::max(0.0f, std::floor(tmin.x / voxelSize)));
    y_min = static_cast<uint32_t>(std::max(0.0f, std::floor(tmin.y / voxelSize)));
    z_min = static_cast<uint32_t>(std::max(0.0f, std::floor(tmin.z / voxelSize)));

    x_max = static_cast<uint32_t>(std::min(static_cast<float>(d1 - 1), std::floor(tmax.x / voxelSize)));
    y_max = static_cast<uint32_t>(std::min(static_cast<float>(d1 - 1), std::floor(tmax.y / voxelSize)));
    z_max = static_cast<uint32_t>(std::min(static_cast<float>(d1 - 1), std::floor(tmax.z / voxelSize)));
}

// ============================================================================
// Test conservativo de solapamiento (Akenine-Möller + Schwarz-Seidel)
// ============================================================================

bool conservativeOverlapModified(const Triangle& tri,
                                  uint32_t vx, uint32_t vy, uint32_t vz,
                                  float voxelSize) {
    // Centro del vóxel
    float cx = (vx + 0.5f) * voxelSize;
    float cy = (vy + 0.5f) * voxelSize;
    float cz = (vz + 0.5f) * voxelSize;

    float halfSize = voxelSize * 0.5f;

    // Trasladar triángulo al sistema de coordenadas del vóxel (centrado en 0)
    Vec3 v0 = {tri.v0.x - cx, tri.v0.y - cy, tri.v0.z - cz};
    Vec3 v1 = {tri.v1.x - cx, tri.v1.y - cy, tri.v1.z - cz};
    Vec3 v2 = {tri.v2.x - cx, tri.v2.y - cy, tri.v2.z - cz};

    // ---- Test 1: AABB del triángulo vs vóxel ----
    // (Pre-filtro rápido)
    {
        Vec3 tmin = Vec3::min(v0, Vec3::min(v1, v2));
        Vec3 tmax = Vec3::max(v0, Vec3::max(v1, v2));

        if (tmin.x > halfSize || tmax.x < -halfSize) return false;
        if (tmin.y > halfSize || tmax.y < -halfSize) return false;
        if (tmin.z > halfSize || tmax.z < -halfSize) return false;
    }

    // ---- Test 2: Plano del triángulo vs vóxel ----
    {
        Vec3 e0 = v1 - v0;
        Vec3 e1 = v2 - v1;
        Vec3 normal = e0.cross(e1);

        float d = normal.dot(v0);

        // Radio de proyección del vóxel sobre la normal
        float r = halfSize * (std::abs(normal.x) + std::abs(normal.y) + std::abs(normal.z));

        if (d > r || d < -r) return false;
    }

    // ---- Test 3: Ejes cruzados (9 tests) ----
    // Para cada arista del triángulo (e0, e1, e2) cruzada con cada eje del vóxel (X, Y, Z)
    Vec3 edges[3] = {v1 - v0, v2 - v1, v0 - v2};

    for (int i = 0; i < 3; ++i) {
        Vec3 e = edges[i];

        // Eje X × arista
        {
            float p0 = e.z * v0.y - e.y * v0.z;
            float p1 = e.z * v1.y - e.y * v1.z;
            float p2 = e.z * v2.y - e.y * v2.z;
            float r = halfSize * (std::abs(e.z) + std::abs(e.y));
            if (!axisTestSAT(p0, p1, p2, r)) return false;
        }

        // Eje Y × arista
        {
            float p0 = -e.z * v0.x + e.x * v0.z;
            float p1 = -e.z * v1.x + e.x * v1.z;
            float p2 = -e.z * v2.x + e.x * v2.z;
            float r = halfSize * (std::abs(e.z) + std::abs(e.x));
            if (!axisTestSAT(p0, p1, p2, r)) return false;
        }

        // Eje Z × arista
        {
            float p0 = e.y * v0.x - e.x * v0.y;
            float p1 = e.y * v1.x - e.x * v1.y;
            float p2 = e.y * v2.x - e.x * v2.y;
            float r = halfSize * (std::abs(e.y) + std::abs(e.x));
            if (!axisTestSAT(p0, p1, p2, r)) return false;
        }
    }

    return true; // Todos los tests pasaron → solapamiento confirmado
}

// ============================================================================
// Algoritmo 3: Funciones para voxelización en el octree
// ============================================================================

void computeBoundingBoxYZ(const Triangle& tri,
                           float& y_min, float& y_max,
                           float& z_min, float& z_max) {
    y_min = std::min({tri.v0.y, tri.v1.y, tri.v2.y});
    y_max = std::max({tri.v0.y, tri.v1.y, tri.v2.y});
    z_min = std::min({tri.v0.z, tri.v1.z, tri.v2.z});
    z_max = std::max({tri.v0.z, tri.v1.z, tri.v2.z});
}

TriangleEdgeFunctions setupEdgeFunctionsYZ(const Triangle& tri) {
    TriangleEdgeFunctions eyz;

    // Proyección del triángulo en el plano YZ
    // Arista 0: v0 -> v1
    // Edge function: e(y,z) = a*y + b*z + c
    // donde (a,b) es la normal 2D de la arista (perpendicular a la arista, apuntando hacia dentro)

    // Para un triángulo con vértices proyectados (v0.y, v0.z), (v1.y, v1.z), (v2.y, v2.z):
    // La arista v0->v1 tiene dirección (v1.y-v0.y, v1.z-v0.z)
    // La normal interior es (-(v1.z-v0.z), (v1.y-v0.y)) o su negativa,
    // dependiendo de la orientación.

    // Arista 0: v0 -> v1
    float dy01 = tri.v1.y - tri.v0.y;
    float dz01 = tri.v1.z - tri.v0.z;
    eyz.edges[0].a = -dz01;
    eyz.edges[0].b = dy01;
    eyz.edges[0].c = -(eyz.edges[0].a * tri.v0.y + eyz.edges[0].b * tri.v0.z);

    // Arista 1: v1 -> v2
    float dy12 = tri.v2.y - tri.v1.y;
    float dz12 = tri.v2.z - tri.v1.z;
    eyz.edges[1].a = -dz12;
    eyz.edges[1].b = dy12;
    eyz.edges[1].c = -(eyz.edges[1].a * tri.v1.y + eyz.edges[1].b * tri.v1.z);

    // Arista 2: v2 -> v0
    float dy20 = tri.v0.y - tri.v2.y;
    float dz20 = tri.v0.z - tri.v2.z;
    eyz.edges[2].a = -dz20;
    eyz.edges[2].b = dy20;
    eyz.edges[2].c = -(eyz.edges[2].a * tri.v2.y + eyz.edges[2].b * tri.v2.z);

    // Verificar orientación: el punto opuesto a cada arista debe tener e >= 0
    // Si no, invertir la edge function
    if (eyz.edges[0].evaluate(tri.v2.y, tri.v2.z) < 0) {
        eyz.edges[0].a = -eyz.edges[0].a;
        eyz.edges[0].b = -eyz.edges[0].b;
        eyz.edges[0].c = -eyz.edges[0].c;
    }
    if (eyz.edges[1].evaluate(tri.v0.y, tri.v0.z) < 0) {
        eyz.edges[1].a = -eyz.edges[1].a;
        eyz.edges[1].b = -eyz.edges[1].b;
        eyz.edges[1].c = -eyz.edges[1].c;
    }
    if (eyz.edges[2].evaluate(tri.v1.y, tri.v1.z) < 0) {
        eyz.edges[2].a = -eyz.edges[2].a;
        eyz.edges[2].b = -eyz.edges[2].b;
        eyz.edges[2].c = -eyz.edges[2].c;
    }

    return eyz;
}

bool centerInsideProjectionYZ(float cy, float cz,
                               const Triangle& tri,
                               const TriangleEdgeFunctions& eyz) {
    // Un punto está dentro de la proyección YZ del triángulo si está
    // en el semiplano positivo de las 3 edge functions
    (void)tri; // No necesario aquí, la info ya está en eyz

    for (int i = 0; i < 3; ++i) {
        if (eyz.edges[i].evaluate(cy, cz) < 0) {
            return false;
        }
    }
    return true;
}

float projectCenterToTrianglePlaneX(float cy, float cz, const Triangle& tri) {
    // El plano del triángulo se define por: n · (p - v0) = 0
    // donde n = (v1-v0) × (v2-v0)
    //
    // Dado (cy, cz), encontramos x tal que n · ((x, cy, cz) - v0) = 0
    // nx*(x - v0.x) + ny*(cy - v0.y) + nz*(cz - v0.z) = 0
    // x = v0.x - (ny*(cy - v0.y) + nz*(cz - v0.z)) / nx

    Vec3 e1 = tri.v1 - tri.v0;
    Vec3 e2 = tri.v2 - tri.v0;
    Vec3 n = e1.cross(e2);

    // Si nx ≈ 0, el triángulo es casi paralelo al eje X
    // En este caso, retornamos la coordenada x promedio del triángulo
    if (std::abs(n.x) < 1e-10f) {
        return (tri.v0.x + tri.v1.x + tri.v2.x) / 3.0f;
    }

    float x = tri.v0.x - (n.y * (cy - tri.v0.y) + n.z * (cz - tri.v0.z)) / n.x;
    return x;
}

// ============================================================================
// Algoritmo 4: Funciones de propagación
// ============================================================================

bool hasActiveBoundaryX(const uint8_t SG[4][4][4]) {
    // Verificar si la última capa en +x (x=3) tiene algún vóxel con paridad impar.
    // Esto indica que hay una frontera de superficie que requiere "flip" del siguiente nodo.
    // Sumamos la paridad de toda la columna x para esa posición (y,z).
    for (int y = 0; y < 4; ++y) {
        for (int z = 0; z < 4; ++z) {
            // Acumular paridad a lo largo de x
            uint8_t parity = 0;
            for (int x = 0; x < 4; ++x) {
                parity ^= SG[x][y][z];
            }
            if (parity != 0) {
                return true;
            }
        }
    }
    return false;
}

void flipAllSGVoxels(uint8_t SG[4][4][4]) {
    for (int x = 0; x < 4; ++x) {
        for (int y = 0; y < 4; ++y) {
            for (int z = 0; z < 4; ++z) {
                SG[x][y][z] ^= 1;
            }
        }
    }
}
