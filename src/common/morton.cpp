#include "common/morton.h"

// ============================================================================
// Codificación Morton 3D mediante bit-twiddling ("magic bits")
//
// Para 3D, intercalamos los bits de x, y, z de modo que:
//   morton = ...z2 y2 x2 z1 y1 x1 z0 y0 x0
//
// Esto crea una curva de llenado de espacio (Z-order curve / Morton curve)
// que preserva la localidad espacial y es fundamental para el octree.
//
// Soporta hasta 10 bits por coordenada (valores 0..1023),
// produciendo códigos Morton de hasta 30 bits.
// ============================================================================

/// Separa los bits de un valor de 10 bits, insertando 2 ceros entre cada bit.
/// Ejemplo: 0b1010 -> 0b001_000_001_000
static uint32_t spreadBits(uint32_t v) {
    v &= 0x000003FFu;  // Solo usar 10 bits
    v = (v | (v << 16)) & 0x030000FFu;
    v = (v | (v <<  8)) & 0x0300F00Fu;
    v = (v | (v <<  4)) & 0x030C30C3u;
    v = (v | (v <<  2)) & 0x09249249u;
    return v;
}

/// Compacta los bits separados, extrayendo cada tercer bit.
/// Operación inversa de spreadBits.
static uint32_t compactBits(uint32_t v) {
    v &= 0x09249249u;
    v = (v | (v >>  2)) & 0x030C30C3u;
    v = (v | (v >>  4)) & 0x0300F00Fu;
    v = (v | (v >>  8)) & 0x030000FFu;
    v = (v | (v >> 16)) & 0x000003FFu;
    return v;
}

uint32_t encodeMorton3D(uint32_t x, uint32_t y, uint32_t z) {
    return spreadBits(x) | (spreadBits(y) << 1) | (spreadBits(z) << 2);
}

void decodeMorton3D(uint32_t morton, uint32_t& x, uint32_t& y, uint32_t& z) {
    x = compactBits(morton >> 0);
    y = compactBits(morton >> 1);
    z = compactBits(morton >> 2);
}
