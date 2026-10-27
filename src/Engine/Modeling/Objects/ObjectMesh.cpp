#include "Engine/Modeling/Objects/ObjectMesh.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <tuple>

namespace MatterEngine::Modeling {

namespace {

// A parte (raiz) e o material pelo nome, criados se faltarem.
std::pair<std::uint16_t, std::uint16_t> partAndMaterial(Mesh& mesh, std::string_view part, std::string_view material) {
    int partIndex = mesh.findPart(part);
    if (partIndex < 0) {
        MeshPart created;
        created.name = std::string(part);
        created.root = true;
        mesh.parts.push_back(created);
        partIndex = static_cast<int>(mesh.parts.size() - 1);
    }
    int materialIndex = mesh.findMaterial(material);
    if (materialIndex < 0) {
        mesh.materials.push_back({ std::string(material), rgb(255, 255, 255), -1 });
        materialIndex = static_cast<int>(mesh.materials.size() - 1);
    }
    return { static_cast<std::uint16_t>(partIndex), static_cast<std::uint16_t>(materialIndex) };
}

Vec3 unit(Vec3 v) {
    const float length = std::sqrt(v.lengthSquared());
    return length > 1e-12f ? v * (1.0f / length) : Vec3 {};
}

// Gira v em volta do eixo unitario.
Vec3 rotateAround(Vec3 v, Vec3 axis, float angle) {
    const float c = std::cos(angle), s = std::sin(angle);
    return v * c + cross(axis, v) * s + axis * (dot(axis, v) * (1.0f - c));
}

} // namespace

bool buildTube(Mesh& mesh, std::span<const Vec3> path, float radius, int sides, std::span<const float> bendRadii,
    std::string_view part, std::string_view material, std::string& error, std::vector<Vec3>* axisOut) {
    if (path.size() < 2 || !(radius > 0.0f) || sides < 3) {
        error = "tubo invalido";
        return false;
    }
    // O eixo com as dobras: cada canto com raio vira um arco tangente aos dois
    // trechos; os sem raio ficam como canto (meia-esquadria).
    struct AxisPoint {
        Vec3 position;
        // Canto soldado: as direcoes de chegada e de saida.
        bool miter = false;
        Vec3 in;
        Vec3 out;
    };
    std::vector<AxisPoint> axis;
    axis.push_back({ path.front(), false, {}, {} });
    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
        const Vec3 in = unit(path[i] - path[i - 1]), out = unit(path[i + 1] - path[i]);
        const float turn = std::acos(std::clamp(dot(in, out), -1.0f, 1.0f));
        const float bend = i - 1 < bendRadii.size() ? bendRadii[i - 1] : 0.0f;
        if (turn < 1e-4f) continue;
        if (bend <= 0.0f) {
            axis.push_back({ path[i], true, in, out });
            continue;
        }
        // A tangente do arco nao passa da metade dos trechos.
        const float before = std::sqrt((path[i] - path[i - 1]).lengthSquared());
        const float after = std::sqrt((path[i + 1] - path[i]).lengthSquared());
        const float tangent = std::min({ bend * std::tan(turn * 0.5f), 0.5f * before, 0.5f * after });
        const float arcRadius = tangent / std::tan(turn * 0.5f);
        const Vec3 start = path[i] - in * tangent;
        const Vec3 inward = unit(out - in);
        const Vec3 center = path[i] + inward * (arcRadius / std::cos(turn * 0.5f));
        const Vec3 pivot = unit(cross(in, out));
        const int steps = std::max(2, static_cast<int>(std::ceil(turn / (3.14159265f / 12.0f))));
        for (int k = 0; k <= steps; ++k)
            axis.push_back({ center + rotateAround(start - center, pivot, turn * static_cast<float>(k) / static_cast<float>(steps)), false, {}, {} });
    }
    axis.push_back({ path.back(), false, {}, {} });
    // A direcao em cada ponto (a media dos trechos; na meia-esquadria, a
    // bissetriz) e um referencial que nao torce (transporte paralelo).
    const std::size_t count = axis.size();
    std::vector<Vec3> tangents(count);
    for (std::size_t k = 0; k < count; ++k) {
        const Vec3 back = k > 0 ? unit(axis[k].position - axis[k - 1].position) : Vec3 {};
        const Vec3 ahead = k + 1 < count ? unit(axis[k + 1].position - axis[k].position) : Vec3 {};
        tangents[k] = unit(back + ahead);
    }
    Vec3 normal = std::abs(tangents[0].z) < 0.9f ? Vec3 { 0.0f, 0.0f, 1.0f } : Vec3 { 1.0f, 0.0f, 0.0f };
    normal = unit(normal - tangents[0] * dot(normal, tangents[0]));
    const auto [partIndex, materialIndex] = partAndMaterial(mesh, part, material);
    const auto base = static_cast<std::uint32_t>(mesh.positions.size());
    for (std::size_t k = 0; k < count; ++k) {
        if (k > 0) {
            normal = unit(normal - tangents[k] * dot(normal, tangents[k]));
            if (normal.lengthSquared() < 0.5f) normal = unit(cross(tangents[k], Vec3 { 0.0f, 1.0f, 0.0f }));
        }
        const Vec3 binormal = cross(tangents[k], normal);
        for (int j = 0; j < sides; ++j) {
            const float angle = 2.0f * 3.14159265f * static_cast<float>(j) / static_cast<float>(sides);
            Vec3 point = axis[k].position + (normal * std::cos(angle) + binormal * std::sin(angle)) * radius;
            if (axis[k].miter) {
                // O anel do trecho de chegada levado ao plano da bissetriz.
                const Vec3 in = axis[k].in;
                const Vec3 ringNormal = unit(normal - in * dot(normal, in));
                const Vec3 ringBinormal = cross(in, ringNormal);
                const Vec3 ring = (ringNormal * std::cos(angle) + ringBinormal * std::sin(angle)) * radius;
                const float slide = -dot(ring, tangents[k]) / dot(in, tangents[k]);
                point = axis[k].position + ring + in * slide;
            }
            mesh.positions.push_back(point);
        }
    }
    const auto addTriangle = [&](std::uint32_t a, std::uint32_t b, std::uint32_t c) {
        mesh.triangles.push_back({ a, b, c });
        mesh.trianglePart.push_back(partIndex);
        mesh.triangleMaterial.push_back(materialIndex);
        if (!mesh.triangleUv.empty()) mesh.triangleUv.push_back({});
    };
    const auto ring = [&](std::size_t k, int j) {
        return base + static_cast<std::uint32_t>(k * static_cast<std::size_t>(sides) + static_cast<std::size_t>((j + sides) % sides));
    };
    for (std::size_t k = 0; k + 1 < count; ++k)
        for (int j = 0; j < sides; ++j) {
            // Para fora: o anel gira anti-horario em volta da tangente.
            addTriangle(ring(k, j), ring(k, j + 1), ring(k + 1, j + 1));
            addTriangle(ring(k, j), ring(k + 1, j + 1), ring(k + 1, j));
        }
    // As tampas: um centro em cada ponta.
    const auto cap = [&](std::size_t k, bool start) {
        mesh.positions.push_back(axis[k].position);
        const auto center = static_cast<std::uint32_t>(mesh.positions.size() - 1);
        for (int j = 0; j < sides; ++j) {
            if (start) addTriangle(center, ring(k, j + 1), ring(k, j));
            else addTriangle(center, ring(k, j), ring(k, j + 1));
        }
    };
    cap(0, true);
    cap(count - 1, false);
    if (axisOut != nullptr) {
        axisOut->clear();
        for (const AxisPoint& point : axis) axisOut->push_back(point.position);
    }
    return true;
}

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
