#pragma once

#include <cstdint>

// ============================================================================
// Codificación/Decodificación Morton 3D (Z-order curve)
// 
// Soporta hasta 10 bits por eje (coordenadas 0..1023), produciendo
// códigos Morton de hasta 30 bits en un uint32_t.
// ============================================================================

/// Codifica coordenadas 3D (x, y, z) en un código Morton intercalando bits.
/// Orden de intercalación: x en bits 0,3,6,...  y en bits 1,4,7,...  z en bits 2,5,8,...
uint32_t encodeMorton3D(uint32_t x, uint32_t y, uint32_t z);

/// Decodifica un código Morton en sus coordenadas 3D originales.
void decodeMorton3D(uint32_t morton, uint32_t& x, uint32_t& y, uint32_t& z);

/// Obtiene el código Morton del padre (desplazamiento de 3 bits a la derecha).
/// En el octree, el padre agrupa 8 hijos cuyo Morton difiere solo en los 3 bits bajos.
inline uint32_t parentMorton(uint32_t morton) {
    return morton >> 3;
}

/// Obtiene la posición del hijo dentro de su padre (0..7).
inline uint32_t childIndex(uint32_t morton) {
    return morton & 0x7;
}
