#pragma once

#include "common/mesh.h"
#include <cstdint>

// ============================================================================
// Funciones geométricas para la voxelización conservativa
// Basadas en el test de intersección triángulo-caja de Akenine-Möller (2001)
// y el test conservativo de Schwarz-Seidel (2010).
// ============================================================================

/// Estructura para funciones de borde 2D (edge functions).
/// Una edge function e(p) = a*px + b*py + c define un semiplano.
/// Un punto está "dentro" si e(p) >= 0 para las tres aristas del triángulo.
struct EdgeFunction2D {
    float a, b, c;

    float evaluate(float px, float py) const {
        return a * px + b * py + c;
    }
};

/// Estructura con las 3 edge functions de un triángulo proyectado en un plano 2D.
struct TriangleEdgeFunctions {
    EdgeFunction2D edges[3];
};

enum class DominantAxis {
    X,
    Y,
    Z
};

// --- Algoritmo 1: Nodos activos de nivel 1 ---

/// Calcula el AABB de un triángulo en coordenadas de grilla de nivel 1.
/// Retorna las esquinas mín/máx en coordenadas enteras [x_min..x_max, y_min..y_max, z_min..z_max].
void computeTriangleAABBLevel1(const Triangle& tri, uint32_t d1,
                                uint32_t& x_min, uint32_t& y_min, uint32_t& z_min,
                                uint32_t& x_max, uint32_t& y_max, uint32_t& z_max);

/// Test conservativo de solapamiento triángulo-vóxel (Schwarz-Seidel modificado).
/// Verifica si un triángulo se solapa con un vóxel centrado en (vx, vy, vz)
/// de tamaño unitario en coordenadas de grilla del nivel dado.
/// 
/// El test evalúa:
/// 1. Solapamiento del AABB del triángulo con el vóxel
/// 2. Intersección del plano del triángulo con el vóxel  
/// 3. Solapamiento de las proyecciones 2D (XY, XZ, YZ) usando edge functions
bool conservativeOverlapModified(const Triangle& tri,
                                  uint32_t vx, uint32_t vy, uint32_t vz,
                                  float voxelSize);

/// Retorna el eje dominante de la normal del triángulo.
DominantAxis dominantAxisOfTriangle(const Triangle& tri);

// --- Algoritmo 3: Voxelización en el octree ---

/// Calcula el AABB del triángulo proyectado en el plano YZ.
void computeBoundingBoxYZ(const Triangle& tri,
                           float& y_min, float& y_max,
                           float& z_min, float& z_max);

/// Calcula las edge functions del triángulo proyectado en YZ.
TriangleEdgeFunctions setupEdgeFunctionsYZ(const Triangle& tri);

/// Verifica si el centro (cy, cz) está dentro de la proyección YZ del triángulo
/// usando las edge functions.
bool centerInsideProjectionYZ(float cy, float cz,
                               const Triangle& tri,
                               const TriangleEdgeFunctions& eyz);

/// Proyecta un punto (cy, cz) sobre el plano del triángulo y retorna la coordenada X.
/// Dado un punto en YZ que cae dentro de la proyección del triángulo,
/// encuentra la x correspondiente en el plano del triángulo.
float projectCenterToTrianglePlaneX(float cy, float cz, const Triangle& tri);

// --- Algoritmo 4: Propagación interior/exterior ---

/// Verifica si la columna x de la sub-grid tiene un bit activo (frontera).
/// Retorna true si la última capa en +x de la SG tiene paridad impar.
bool hasActiveBoundaryX(const uint8_t SG[4][4][4]);

/// Invierte (flip) todos los vóxeles en la sub-grid.
void flipAllSGVoxels(uint8_t SG[4][4][4]);
