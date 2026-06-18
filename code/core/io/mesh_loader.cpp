#include "io/mesh_loader.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void computeBounds(Mesh& mesh) {
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
}

void printMeshSummary(const std::string& path, size_t vertexCount, const Mesh& mesh) {
    std::cout << "Malla cargada: " << path << std::endl;
    std::cout << "  Vertices: " << vertexCount << std::endl;
    std::cout << "  Triangulos: " << mesh.triangles.size() << std::endl;
    std::cout << "  AABB min: (" << mesh.bounds.min.x << ", " << mesh.bounds.min.y << ", " << mesh.bounds.min.z << ")" << std::endl;
    std::cout << "  AABB max: (" << mesh.bounds.max.x << ", " << mesh.bounds.max.y << ", " << mesh.bounds.max.z << ")" << std::endl;
}

std::string toLower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

std::string getExtension(const std::string& path) {
    const size_t dotPos = path.find_last_of('.');
    if (dotPos == std::string::npos) {
        return "";
    }
    return toLower(path.substr(dotPos + 1));
}

int parseOBJIndex(const std::string& token, int vertexCount) {
    const size_t slashPos = token.find('/');
    const std::string indexPart = token.substr(0, slashPos);
    if (indexPart.empty()) {
        throw std::runtime_error("cara OBJ con indice de vertice vacio");
    }

    int index = std::stoi(indexPart);
    if (index > 0) {
        return index - 1;
    }
    if (index < 0) {
        return vertexCount + index;
    }
    throw std::runtime_error("OBJ usa indice 0 no soportado");
}

Triangle makeTriangle(const std::vector<Vec3>& vertices, int i0, int i1, int i2) {
    if (i0 < 0 || i1 < 0 || i2 < 0 ||
        i0 >= static_cast<int>(vertices.size()) ||
        i1 >= static_cast<int>(vertices.size()) ||
        i2 >= static_cast<int>(vertices.size())) {
        throw std::runtime_error("indice de vertice fuera de rango");
    }

    Triangle tri;
    tri.v0 = vertices[i0];
    tri.v1 = vertices[i1];
    tri.v2 = vertices[i2];
    return tri;
}

} // namespace

Mesh loadOBJ(const std::string& path) {
    Mesh mesh;
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("no se pudo abrir archivo OBJ");
    }

    std::vector<Vec3> vertices;
    std::string line;

    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;

        if (prefix == "v") {
            Vec3 vertex;
            iss >> vertex.x >> vertex.y >> vertex.z;
            vertices.push_back(vertex);
        } else if (prefix == "f") {
            std::vector<int> faceIndices;
            std::string token;
            while (iss >> token) {
                faceIndices.push_back(parseOBJIndex(token, static_cast<int>(vertices.size())));
            }

            if (faceIndices.size() < 3) {
                continue;
            }

            for (size_t i = 1; i + 1 < faceIndices.size(); ++i) {
                mesh.triangles.push_back(makeTriangle(vertices, faceIndices[0], faceIndices[i], faceIndices[i + 1]));
            }
        }
    }

    if (mesh.triangles.empty()) {
        throw std::runtime_error("OBJ sin triangulos validos");
    }

    computeBounds(mesh);
    printMeshSummary(path, vertices.size(), mesh);
    return mesh;
}

Mesh loadMesh(const std::string& path) {
    const std::string extension = getExtension(path);

    if (extension == "obj") {
        return loadOBJ(path);
    }

    throw std::runtime_error("solo se soportan mallas OBJ. Extension recibida: " + extension);
}
