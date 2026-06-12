#pragma once

#include "common/mesh.h"

#include <string>

/// Carga una malla triangular desde un archivo OBJ.
Mesh loadOBJ(const std::string& path);

/// Valida la extensión del archivo y carga una malla triangular OBJ.
Mesh loadMesh(const std::string& path);
