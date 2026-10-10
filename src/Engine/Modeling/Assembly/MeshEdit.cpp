#include "Engine/Modeling/Assembly/MeshEdit.hpp"

#include "Engine/Modeling/Mesh/MeshBuilder.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine::Modeling {

void transformPositions(Mesh& mesh, float scale, Vec3 offset) {
    for (Vec3& p : mesh.positions) p = p * scale + offset;
}

CutResult keepRightHalf(Mesh& mesh) {
    return cutByPlane(mesh, { { 0.0f, 0.0f, 0.0f }, { 0.0f, -1.0f, 0.0f } });
}

std::uint32_t appendMesh(Mesh& into, const Mesh& from) {
    const std::uint32_t first = static_cast<std::uint32_t>(into.positions.size());
    // Os pesos: os de "from" se tiver (os de "into" ficam vazios se nao
    // tinha), senao vazios.
    if (from.hasSkin() || !into.skin.empty()) into.skin.resize(first);
    into.positions.insert(into.positions.end(), from.positions.begin(), from.positions.end());
    if (from.hasSkin()) into.skin.insert(into.skin.end(), from.skin.begin(), from.skin.end());
    else if (!into.skin.empty()) into.skin.resize(into.positions.size());
    MeshBuilder builder(into);
    std::vector<std::uint16_t> partOf(from.parts.size()), materialOf(from.materials.size());
    for (std::size_t p = 0; p < from.parts.size(); ++p) {
        const MeshPart& part = from.parts[p];
        builder.part(part.name, part.region);
        MeshPart& target = into.parts[static_cast<std::size_t>(builder.currentPart())];
        for (const std::string& other : part.connectedTo)
            if (std::find(target.connectedTo.begin(), target.connectedTo.end(), other) == target.connectedTo.end())
                target.connectedTo.push_back(other);
        for (const std::string& opening : part.openings) target.openings.push_back(opening);
        target.root = target.root || part.root;
        partOf[p] = static_cast<std::uint16_t>(builder.currentPart());
    }
    for (std::size_t m = 0; m < from.materials.size(); ++m) {
        const int existing = into.findMaterial(from.materials[m].name);
        if (existing >= 0) materialOf[m] = static_cast<std::uint16_t>(existing);
        else {
            into.materials.push_back(from.materials[m]);
            materialOf[m] = static_cast<std::uint16_t>(into.materials.size() - 1);
        }
    }
    for (std::size_t t = 0; t < from.triangles.size(); ++t) {
        const Triangle& triangle = from.triangles[t];
        into.triangles.push_back({ triangle[0] + first, triangle[1] + first, triangle[2] + first });
        into.trianglePart.push_back(t < from.trianglePart.size() && from.trianglePart[t] < partOf.size() ? partOf[from.trianglePart[t]] : 0);
        into.triangleMaterial.push_back(
            t < from.triangleMaterial.size() && from.triangleMaterial[t] < materialOf.size() ? materialOf[from.triangleMaterial[t]] : 0);
    }
    return first;
}

CutPlane tiltedPlane(float height, float tiltDegrees, float centerX, bool keepAbove) {
    const float tilt = tiltDegrees * DegreesToRadians;
    const Vec3 up { std::sin(tilt), 0.0f, std::cos(tilt) };
    return { { centerX, 0.0f, height }, keepAbove ? up : -up };
}

std::vector<std::uint32_t> partsVertices(const Mesh& mesh, std::span<const int> parts) {
    std::vector<bool> used(mesh.positions.size(), false);
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
        if (std::find(parts.begin(), parts.end(), mesh.partOf(t)) != parts.end())
            for (const std::uint32_t v : mesh.triangles[t]) used[v] = true;
    std::vector<std::uint32_t> vertices;
    for (std::size_t v = 0; v < used.size(); ++v)
        if (used[v]) vertices.push_back(static_cast<std::uint32_t>(v));
    return vertices;
}

} // namespace MatterEngine::Modeling
