#include "Engine/Modeling/Objects/ObjectMesh.hpp"

#include <cmath>
#include <map>
#include <tuple>

namespace MatterEngine::Modeling {

bool buildTexturedBox(Mesh& mesh, Vec3 size, const BoxFaceUvs& uvs, std::string_view part, std::string_view material,
    std::string& error) {
    if (!(size.x > 0.0f && size.y > 0.0f && size.z > 0.0f) || !std::isfinite(size.x + size.y + size.z)) {
        error = "bloco com medida invalida";
        return false;
    }
    const Vec3 half = size * 0.5f;
    std::uint16_t partIndex = 0, materialIndex = 0;
    {
        const int found = mesh.findPart(part);
        if (found < 0) {
            MeshPart created;
            created.name = std::string(part);
            created.root = true;
            mesh.parts.push_back(created);
            partIndex = static_cast<std::uint16_t>(mesh.parts.size() - 1);
        } else {
            partIndex = static_cast<std::uint16_t>(found);
        }
        const int existing = mesh.findMaterial(material);
        if (existing < 0) {
            mesh.materials.push_back({ std::string(material), rgb(255, 255, 255), -1 });
            materialIndex = static_cast<std::uint16_t>(mesh.materials.size() - 1);
        } else {
            materialIndex = static_cast<std::uint16_t>(existing);
        }
    }
    // Os 8 cantos, um vertice cada (as faces dividem as arestas: a malha e
    // fechada).
    std::map<std::tuple<int, int, int>, std::uint32_t> corners;
    const auto corner = [&](Vec3 p) {
        const auto key = std::make_tuple(p.x > 0.0f ? 1 : -1, p.y > 0.0f ? 1 : -1, p.z > 0.0f ? 1 : -1);
        const auto found = corners.find(key);
        if (found != corners.end()) return found->second;
        mesh.positions.push_back(p);
        const auto index = static_cast<std::uint32_t>(mesh.positions.size() - 1);
        corners.emplace(key, index);
        return index;
    };
    // Cada face vista de fora: a normal, o "em cima" e as meias-medidas no
    // direito e no em cima.
    struct Face {
        Vec3 normal;
        Vec3 up;
    };
    const Face faces[6] { { { 1.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } }, { { -1.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } },
        { { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } }, { { 0.0f, -1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } },
        { { 0.0f, 0.0f, 1.0f }, { 0.0f, 1.0f, 0.0f } }, { { 0.0f, 0.0f, -1.0f }, { 0.0f, 1.0f, 0.0f } } };
    const auto extent = [&](Vec3 axis) { return std::abs(axis.x) * half.x + std::abs(axis.y) * half.y + std::abs(axis.z) * half.z; };
    const bool textured = mesh.hasUv() || mesh.triangles.empty();
    for (int f = 0; f < 6; ++f) {
        const Face& face = faces[f];
        const Vec3 right = cross(face.normal * -1.0f, face.up);
        const Vec3 center = face.normal * extent(face.normal);
        const Vec3 r = right * extent(right), u = face.up * extent(face.up);
        const std::uint32_t topLeft = corner(center - r + u), bottomLeft = corner(center - r - u);
        const std::uint32_t bottomRight = corner(center + r - u), topRight = corner(center + r + u);
        const UvRect& rect = uvs.faces[static_cast<std::size_t>(f)];
        const Uv tl { rect.u0 + uvs.inset, rect.v0 + uvs.inset }, tr { rect.u1 - uvs.inset, rect.v0 + uvs.inset };
        const Uv bl { rect.u0 + uvs.inset, rect.v1 - uvs.inset }, br { rect.u1 - uvs.inset, rect.v1 - uvs.inset };
        // Anti-horario visto de fora: em cima a esquerda, embaixo a esquerda,
        // embaixo a direita, em cima a direita.
        mesh.triangles.push_back({ topLeft, bottomLeft, bottomRight });
        mesh.triangles.push_back({ topLeft, bottomRight, topRight });
        for (int k = 0; k < 2; ++k) {
            mesh.trianglePart.push_back(partIndex);
            mesh.triangleMaterial.push_back(materialIndex);
        }
        if (textured) {
            mesh.triangleUv.push_back({ tl, bl, br });
            mesh.triangleUv.push_back({ tl, br, tr });
        }
    }
    return true;
}

} // namespace MatterEngine::Modeling
