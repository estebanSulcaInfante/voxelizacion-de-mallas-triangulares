#pragma once

#include "common/mesh.h"

#include <string>

/// Carga una malla triangular desde un archivo PLY (ASCII o binario).
Mesh loadPLY(const std::string& path);

/// Carga una malla triangular desde un archivo OBJ.
Mesh loadOBJ(const std::string& path);

/// Carga una malla triangular desde un archivo OFF.
Mesh loadOFF(const std::string& path);

/// Detecta la extensión del archivo y delega al loader adecuado.
Mesh loadMesh(const std::string& path);
