#include "Engine/Modeling/Mesh/ShadingNormals.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace MatterEngine::Modeling {

namespace {

std::uint32_t root(std::vector<std::uint32_t>& parent, std::uint32_t i) {
    while (parent[i] != i) {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }
    return i;
}

// O angulo do canto em "at" (radianos), entre as arestas que saem dele.
float cornerAngle(Vec3 at, Vec3 a, Vec3 b) {
    const Vec3 u = a - at, v = b - at;
    const float lengths = u.length() * v.length();
    if (lengths < 1e-20f) return 0.0f;
    return std::acos(std::clamp(dot(u, v) / lengths, -1.0f, 1.0f));
}

Vec3 faceNormal(Vec3 a, Vec3 b, Vec3 c) {
    const Vec3 n = cross(b - a, c - a);
    const float length = n.length();
    return length > 1e-20f ? n / length : Vec3 {};
}

} // namespace

ShadingNormals::ShadingNormals(const Mesh& mesh, float creaseDegrees) : m_crease(creaseDegrees) {
    const std::size_t count = mesh.triangles.size();
    std::vector<std::uint32_t> parent(count * 3);
    std::iota(parent.begin(), parent.end(), 0u);
    std::vector<Vec3> normals(count);
    for (std::size_t t = 0; t < count; ++t) {
        const Triangle& tri = mesh.triangles[t];
        normals[t] = faceNormal(mesh.positions[tri[0]], mesh.positions[tri[1]], mesh.positions[tri[2]]);
    }
    // As arestas (o par de vertices) com o triangulo e o canto de onde saem;
    // juntas, as que tem dois triangulos so.
    struct HalfEdge {
        std::uint64_t key;
        std::uint32_t triangle;
    };
    std::vector<HalfEdge> edges;
    edges.reserve(count * 3);
    for (std::size_t t = 0; t < count; ++t)
        for (int k = 0; k < 3; ++k) {
            const std::uint32_t a = mesh.triangles[t][static_cast<std::size_t>(k)], b = mesh.triangles[t][static_cast<std::size_t>((k + 1) % 3)];
            edges.push_back({ (static_cast<std::uint64_t>(std::min(a, b)) << 32) | std::max(a, b), static_cast<std::uint32_t>(t) });
        }
    std::sort(edges.begin(), edges.end(), [](const HalfEdge& x, const HalfEdge& y) { return x.key < y.key || (x.key == y.key && x.triangle < y.triangle); });
    const float limit = std::cos(std::clamp(creaseDegrees, 0.0f, 180.0f) * 0.017453292f);
    const auto cornerOf = [&](std::uint32_t triangle, std::uint32_t vertex) {
        const Triangle& tri = mesh.triangles[triangle];
        const std::uint32_t k = tri[0] == vertex ? 0u : tri[1] == vertex ? 1u : 2u;
        return triangle * 3 + k;
    };
    for (std::size_t i = 0; i < edges.size();) {
        std::size_t j = i;
        while (j < edges.size() && edges[j].key == edges[i].key) ++j;
        // Uma aresta de duas faces sem quina: os cantos de cada ponta juntos.
        if (j - i == 2 && edges[i].triangle != edges[i + 1].triangle
            && dot(normals[edges[i].triangle], normals[edges[i + 1].triangle]) >= limit) {
            for (const std::uint32_t vertex : { static_cast<std::uint32_t>(edges[i].key >> 32), static_cast<std::uint32_t>(edges[i].key & 0xffffffffu) }) {
                const std::uint32_t x = root(parent, cornerOf(edges[i].triangle, vertex)), y = root(parent, cornerOf(edges[i + 1].triangle, vertex));
                if (x != y) parent[std::max(x, y)] = std::min(x, y);
            }
        }
        i = j;
    }
    m_cornerGroup.resize(count * 3);
    std::vector<std::uint32_t> id(count * 3, std::numeric_limits<std::uint32_t>::max());
    for (std::uint32_t corner = 0; corner < count * 3; ++corner) {
        const std::uint32_t r = root(parent, corner);
        if (id[r] == std::numeric_limits<std::uint32_t>::max()) id[r] = m_groupCount++;
        m_cornerGroup[corner] = id[r];
    }
}

void ShadingNormals::compute(std::span<const Vec3> positions, const std::vector<Triangle>& triangles, float facet, std::vector<Vec3>& corners) const {
    const std::size_t count = triangles.size();
    corners.resize(count * 3);
    std::vector<Vec3> faces(count);
    for (std::size_t t = 0; t < count; ++t) {
        const Triangle& tri = triangles[t];
        faces[t] = faceNormal(positions[tri[0]], positions[tri[1]], positions[tri[2]]);
    }
    // Outra malha (sem os grupos dela): facetado.
    if (m_cornerGroup.size() != count * 3) {
        for (std::size_t t = 0; t < count; ++t)
            for (std::size_t k = 0; k < 3; ++k) corners[t * 3 + k] = faces[t];
        return;
    }
    std::vector<Vec3> sum(m_groupCount, Vec3 {});
    for (std::size_t t = 0; t < count; ++t) {
        const Triangle& tri = triangles[t];
        const Vec3 a = positions[tri[0]], b = positions[tri[1]], c = positions[tri[2]];
        sum[m_cornerGroup[t * 3]] += faces[t] * cornerAngle(a, b, c);
        sum[m_cornerGroup[t * 3 + 1]] += faces[t] * cornerAngle(b, c, a);
        sum[m_cornerGroup[t * 3 + 2]] += faces[t] * cornerAngle(c, a, b);
    }
    const float mixFace = std::clamp(facet, 0.0f, 1.0f);
    for (std::size_t t = 0; t < count; ++t)
        for (std::size_t k = 0; k < 3; ++k) {
            const Vec3 group = sum[m_cornerGroup[t * 3 + k]];
            const float length = group.length();
            Vec3 normal = length > 1e-20f ? group / length : faces[t];
            if (mixFace > 0.0f) {
                const Vec3 blended = normal * (1.0f - mixFace) + faces[t] * mixFace;
                const float blendedLength = blended.length();
                normal = blendedLength > 1e-20f ? blended / blendedLength : faces[t];
            }
            corners[t * 3 + k] = normal;
        }
}

} // namespace MatterEngine::Modeling
