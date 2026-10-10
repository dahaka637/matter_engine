#include "Engine/Modeling/Mesh/Mesh.hpp"

#include <algorithm>
#include <cmath>
#include <set>

namespace MatterEngine::Modeling {

bool MeshPart::connectedWith(const MeshPart& other) const {
    return std::find(connectedTo.begin(), connectedTo.end(), other.name) != connectedTo.end()
        || std::find(other.connectedTo.begin(), other.connectedTo.end(), name) != other.connectedTo.end();
}

Box3 Mesh::bounds() const {
    Box3 box;
    for (const Vec3& position : positions) box.include(position);
    return box;
}

Vec3 Mesh::normal(std::size_t triangle) const {
    const Triangle& t = triangles[triangle];
    return triangleNormal(positions[t[0]], positions[t[1]], positions[t[2]]);
}

float Mesh::area(std::size_t triangle) const {
    const Triangle& t = triangles[triangle];
    return triangleArea(positions[t[0]], positions[t[1]], positions[t[2]]);
}

Vec3 Mesh::center(std::size_t triangle) const {
    const Triangle& t = triangles[triangle];
    return (positions[t[0]] + positions[t[1]] + positions[t[2]]) / 3.0f;
}

int Mesh::findPart(std::string_view name) const {
    for (std::size_t i = 0; i < parts.size(); ++i)
        if (parts[i].name == name) return static_cast<int>(i);
    return -1;
}

int Mesh::findMaterial(std::string_view name) const {
    for (std::size_t i = 0; i < materials.size(); ++i)
        if (materials[i].name == name) return static_cast<int>(i);
    return -1;
}

int Mesh::partOf(std::size_t triangle) const {
    return triangle < trianglePart.size() ? trianglePart[triangle] : -1;
}

Box3 Mesh::partBounds(int part) const {
    Box3 box;
    for (std::size_t i = 0; i < triangles.size(); ++i)
        if (partOf(i) == part)
            for (const std::uint32_t vertex : triangles[i]) box.include(positions[vertex]);
    return box;
}

std::vector<std::uint32_t> Mesh::partVertices(int part) const {
    std::set<std::uint32_t> vertices;
    for (std::size_t i = 0; i < triangles.size(); ++i)
        if (partOf(i) == part) vertices.insert(triangles[i].begin(), triangles[i].end());
    return { vertices.begin(), vertices.end() };
}

std::vector<std::uint32_t> Mesh::partTriangles(int part) const {
    std::vector<std::uint32_t> list;
    for (std::size_t i = 0; i < triangles.size(); ++i)
        if (partOf(i) == part) list.push_back(static_cast<std::uint32_t>(i));
    return list;
}

std::size_t Mesh::partTriangleCount(int part) const {
    return static_cast<std::size_t>(std::count(trianglePart.begin(), trianglePart.end(), part));
}

float Mesh::signedVolume() const {
    double volume = 0.0;
    for (const Triangle& t : triangles)
        volume += dot(positions[t[0]], cross(positions[t[1]], positions[t[2]]));
    return static_cast<float>(volume / 6.0);
}

float Mesh::surfaceArea() const {
    double total = 0.0;
    for (std::size_t i = 0; i < triangles.size(); ++i) total += area(i);
    return static_cast<float>(total);
}

std::vector<Vec3> Mesh::posedPositions(std::span<const Transform> skinning) const {
    if (!hasSkin() || skinning.empty()) return positions;
    std::vector<Vec3> result(positions.size());
    for (std::size_t v = 0; v < positions.size(); ++v) {
        Vec3 sum {};
        float total = 0.0f;
        for (std::size_t i = 0; i < 4; ++i) {
            const float weight = skin[v].weights[i];
            const std::uint16_t joint = skin[v].joints[i];
            if (weight <= 0.0f || joint >= skinning.size()) continue;
            sum += skinning[joint].apply(positions[v]) * weight;
            total += weight;
        }
        result[v] = total > 1e-6f ? sum / total : positions[v];
    }
    return result;
}

std::vector<std::string> Mesh::validateLayout() const {
    std::vector<std::string> problems;
    if (trianglePart.size() != triangles.size()) problems.push_back("trianglePart com tamanho diferente");
    if (triangleMaterial.size() != triangles.size()) problems.push_back("triangleMaterial com tamanho diferente");
    if (!skin.empty() && skin.size() != positions.size()) problems.push_back("skin com tamanho diferente");
    if (!triangleUv.empty() && triangleUv.size() != triangles.size()) problems.push_back("triangleUv com tamanho diferente");
    for (const MeshMaterial& material : materials)
        if (material.texture >= static_cast<int>(textures.size()))
            problems.push_back("material " + material.name + " com textura inexistente");
    for (std::size_t i = 0; i < triangles.size(); ++i) {
        for (const std::uint32_t vertex : triangles[i])
            if (vertex >= positions.size()) {
                problems.push_back("triangulo " + std::to_string(i) + " com vertice fora");
                break;
            }
        if (i < trianglePart.size() && trianglePart[i] >= parts.size())
            problems.push_back("triangulo " + std::to_string(i) + " com parte inexistente");
        if (i < triangleMaterial.size() && triangleMaterial[i] >= materials.size())
            problems.push_back("triangulo " + std::to_string(i) + " com material inexistente");
        if (problems.size() > 20) break;
    }
    return problems;
}

} // namespace MatterEngine::Modeling
