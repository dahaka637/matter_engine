#include "Engine/Modeling/Mesh/MeshRepair.hpp"

#include "Engine/Modeling/Geometry/ModelGeometry.hpp"
#include "Engine/Modeling/Mesh/MeshTopology.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <set>
#include <vector>

namespace MatterEngine::Modeling {

namespace {

// O menor angulo do triangulo (radianos).
float smallestAngle(Vec3 a, Vec3 b, Vec3 c) {
    const auto angle = [](Vec3 at, Vec3 p, Vec3 q) {
        const Vec3 u = p - at, v = q - at;
        const float lengths = u.length() * v.length();
        return lengths > 1e-20f ? std::acos(std::clamp(dot(u, v) / lengths, -1.0f, 1.0f)) : 0.0f;
    };
    return std::min({ angle(a, b, c), angle(b, c, a), angle(c, a, b) });
}

} // namespace

std::size_t flipFlatNeedles(Mesh& mesh, float flatness, float maximumHeight) {
    std::size_t flipped = 0;
    // Cada passada refaz as arestas; uma troca pode deixar outra agulha
    // (rara) para a passada seguinte.
    for (int pass = 0; pass < 8; ++pass) {
        const MeshEdges edges(mesh);
        std::vector<bool> touched(mesh.triangles.size(), false);
        std::size_t changed = 0;
        for (std::uint32_t t = 0; t < mesh.triangles.size(); ++t) {
            if (touched[t]) continue;
            const Triangle tri = mesh.triangles[t];
            // O vertice do meio: o oposto a aresta mais comprida.
            std::size_t middle = 0;
            float longest = -1.0f;
            for (std::size_t k = 0; k < 3; ++k) {
                const float length = (mesh.positions[tri[(k + 2) % 3]] - mesh.positions[tri[(k + 1) % 3]]).lengthSquared();
                if (length > longest) {
                    longest = length;
                    middle = k;
                }
            }
            const std::uint32_t m = tri[middle], b = tri[(middle + 1) % 3], a = tri[(middle + 2) % 3];
            // (a, m, b): a aresta comprida e b -> a nele.
            const Vec3 pa = mesh.positions[a], pb = mesh.positions[b], pm = mesh.positions[m];
            const Vec3 edge = pb - pa;
            const float edgeLength = edge.length();
            if (edgeLength < 1e-9f) continue;
            const float along = dot(pm - pa, edge) / (edgeLength * edgeLength);
            const float height = cross(edge, pm - pa).length() / edgeLength;
            if (along <= 0.0f || along >= 1.0f || height > flatness * edgeLength || height > maximumHeight) continue;
            const std::uint32_t u = edges.across(t, a, b);
            if (u == UINT32_MAX || touched[u]) continue;
            const MeshEdges::Use* use = edges.find(a, b);
            if (use == nullptr || use->count != 2) continue;
            // O vizinho (a, b, c): a mesma aresta no sentido contrario.
            const Triangle other = mesh.triangles[u];
            std::uint32_t c = UINT32_MAX;
            for (std::size_t k = 0; k < 3; ++k)
                if (other[k] == a && other[(k + 1) % 3] == b) c = other[(k + 2) % 3];
            if (c == UINT32_MAX || c == m || edges.find(m, c) != nullptr) continue;
            // So no plano do vizinho e do outro lado da aresta (numa quina, a
            // troca cortaria a quina).
            const Vec3 direction = edge / edgeLength;
            const Vec3 offset = (pm - pa) - direction * dot(pm - pa, direction);
            const Vec3 toward = (mesh.positions[c] - pa) - direction * dot(mesh.positions[c] - pa, direction);
            const Vec3 normal = safeNormalized(cross(edge, mesh.positions[c] - pa), {});
            if (dot(offset, toward) > 1e-12f || std::abs(dot(offset, normal)) > 0.35f * offset.length() + 1e-6f) continue;
            // Os dois novos nao podem ser agulhas nem dobrar em relacao ao
            // vizinho (o mesmo plano, a mesma frente).
            const Vec3 pc = mesh.positions[c];
            constexpr float MinimumAngle = 2.0f * DegreesToRadians;
            const float cosine = std::cos(MinimumAngle);
            if (smallestAngle(pa, pm, pc) < MinimumAngle || smallestAngle(pm, pb, pc) < MinimumAngle
                || dot(triangleNormal(pa, pm, pc), normal) < cosine || dot(triangleNormal(pm, pb, pc), normal) < cosine)
                continue;
            // A troca: (a, m, c) e (m, b, c), com a parte e o material do vizinho.
            mesh.triangles[t] = { a, m, c };
            mesh.triangles[u] = { m, b, c };
            mesh.trianglePart[t] = mesh.trianglePart[u];
            mesh.triangleMaterial[t] = mesh.triangleMaterial[u];
            touched[t] = touched[u] = true;
            ++changed;
        }
        flipped += changed;
        if (changed == 0) break;
    }
    return flipped;
}

std::size_t improveTriangles(Mesh& mesh, std::span<const int> parts, const TriangleFlipSettings& settings) {
    const float degrees = 3.14159265f / 180.0f;
    const float bendLimit = std::cos(settings.maximumBend * degrees);
    const auto inParts = [&](std::size_t t) {
        return parts.empty() || std::find(parts.begin(), parts.end(), mesh.partOf(t)) != parts.end();
    };
    std::size_t flips = 0;
    for (int pass = 0; pass < settings.passes; ++pass) {
        std::map<std::pair<std::uint32_t, std::uint32_t>, std::vector<std::size_t>> edges;
        for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
            for (int i = 0; i < 3; ++i) {
                const std::uint32_t a = mesh.triangles[t][static_cast<std::size_t>(i)];
                const std::uint32_t b = mesh.triangles[t][static_cast<std::size_t>((i + 1) % 3)];
                edges[{ std::min(a, b), std::max(a, b) }].push_back(t);
            }
        std::vector<bool> touched(mesh.triangles.size(), false);
        std::size_t passFlips = 0;
        for (const auto& [edge, triangles] : edges) {
            if (triangles.size() != 2) continue;
            const std::size_t t1 = triangles[0], t2 = triangles[1];
            if (touched[t1] || touched[t2] || !inParts(t1) || !inParts(t2) || mesh.partOf(t1) != mesh.partOf(t2)) continue;
            // t1 = (a, b, c) e t2 = (b, a, d) no sentido deles.
            const Triangle& first = mesh.triangles[t1];
            int k = 0;
            while (k < 3 && !(first[static_cast<std::size_t>(k)] != edge.first && first[static_cast<std::size_t>(k)] != edge.second)) ++k;
            const std::uint32_t c = first[static_cast<std::size_t>(k)];
            const std::uint32_t a = first[static_cast<std::size_t>((k + 1) % 3)], b = first[static_cast<std::size_t>((k + 2) % 3)];
            std::uint32_t d = 0;
            for (const std::uint32_t v : mesh.triangles[t2])
                if (v != a && v != b) d = v;
            if (c == d || edges.count({ std::min(c, d), std::max(c, d) }) != 0) continue;
            const Vec3 pa = mesh.positions[a], pb = mesh.positions[b], pc = mesh.positions[c], pd = mesh.positions[d];
            const Vec3 n1 = safeNormalized(cross(pb - pa, pc - pa), {}), n2 = safeNormalized(cross(pa - pb, pd - pb), {});
            if (dot(n1, n2) < bendLimit) continue;
            // Os novos (a, d, c) e (d, b, c): na mesma direcao, sem dobrar.
            const Vec3 m1 = cross(pd - pa, pc - pa), m2 = cross(pb - pd, pc - pd);
            const Vec3 facing = n1 + n2;
            if (dot(m1, facing) <= 0.0f || dot(m2, facing) <= 0.0f) continue;
            const float before = std::min(smallestAngle(pa, pb, pc), smallestAngle(pb, pa, pd));
            const float after = std::min(smallestAngle(pa, pd, pc), smallestAngle(pd, pb, pc));
            if (after < before + settings.gain * degrees) continue;
            mesh.triangles[t1] = { a, d, c };
            mesh.triangles[t2] = { d, b, c };
            touched[t1] = touched[t2] = true;
            ++passFlips;
        }
        flips += passFlips;
        if (passFlips == 0) break;
    }
    return flips;
}

namespace {

// Junta "gone" em "stays" se a malha continua uma variedade, nenhum
// triangulo vira e nenhum fica com menor angulo abaixo de "minimumAngle"
// (radianos). Verdadeiro se juntou.
bool tryCollapse(Mesh& mesh, std::uint32_t gone, std::uint32_t stays, const std::vector<std::set<std::uint32_t>>& neighbors,
    const std::vector<std::vector<std::size_t>>& around, float minimumAngle) {
    std::vector<std::size_t> shared;
    for (const std::size_t t : around[gone]) {
        const Triangle& tri = mesh.triangles[t];
        if (tri[0] == stays || tri[1] == stays || tri[2] == stays) shared.push_back(t);
    }
    if (shared.size() != 2) return false;
    std::set<std::uint32_t> opposite;
    for (const std::size_t t : shared)
        for (const std::uint32_t w : mesh.triangles[t])
            if (w != gone && w != stays) opposite.insert(w);
    std::size_t common = 0;
    for (const std::uint32_t w : neighbors[gone]) common += neighbors[stays].count(w);
    if (common != opposite.size()) return false;
    for (const std::size_t t : around[gone]) {
        if (std::find(shared.begin(), shared.end(), t) != shared.end()) continue;
        Triangle moved = mesh.triangles[t];
        for (std::uint32_t& w : moved)
            if (w == gone) w = stays;
        const Vec3 before = cross(mesh.positions[mesh.triangles[t][1]] - mesh.positions[mesh.triangles[t][0]],
            mesh.positions[mesh.triangles[t][2]] - mesh.positions[mesh.triangles[t][0]]);
        const Vec3 after = cross(mesh.positions[moved[1]] - mesh.positions[moved[0]], mesh.positions[moved[2]] - mesh.positions[moved[0]]);
        if (dot(safeNormalized(before, {}), safeNormalized(after, {})) < 0.7f
            || smallestAngle(mesh.positions[moved[0]], mesh.positions[moved[1]], mesh.positions[moved[2]]) < minimumAngle)
            return false;
    }
    for (const std::size_t t : around[gone])
        for (std::uint32_t& w : mesh.triangles[t])
            if (w == gone) w = stays;
    std::size_t kept = 0;
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
        const Triangle& tri = mesh.triangles[t];
        if (tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2]) continue;
        mesh.triangles[kept] = tri;
        mesh.trianglePart[kept] = mesh.trianglePart[t];
        mesh.triangleMaterial[kept] = mesh.triangleMaterial[t];
        if (!mesh.triangleUv.empty()) mesh.triangleUv[kept] = mesh.triangleUv[t];
        ++kept;
    }
    mesh.triangles.resize(kept);
    mesh.trianglePart.resize(kept);
    mesh.triangleMaterial.resize(kept);
    if (!mesh.triangleUv.empty()) mesh.triangleUv.resize(kept);
    return true;
}

void adjacency(const Mesh& mesh, std::vector<std::set<std::uint32_t>>& neighbors, std::vector<std::vector<std::size_t>>& around) {
    neighbors.assign(mesh.positions.size(), {});
    around.assign(mesh.positions.size(), {});
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
        for (int i = 0; i < 3; ++i) {
            const std::uint32_t a = mesh.triangles[t][static_cast<std::size_t>(i)];
            const std::uint32_t b = mesh.triangles[t][static_cast<std::size_t>((i + 1) % 3)];
            neighbors[a].insert(b);
            neighbors[b].insert(a);
            around[a].push_back(t);
        }
}

} // namespace

std::size_t collapseShortEdges(Mesh& mesh, float ratio, const std::function<bool(std::uint32_t)>& keep) {
    std::size_t collapses = 0;
    for (bool changed = true; changed;) {
        changed = false;
        std::vector<std::set<std::uint32_t>> neighbors;
        std::vector<std::vector<std::size_t>> around;
        adjacency(mesh, neighbors, around);
        const auto meanEdge = [&](std::uint32_t v) {
            float total = 0.0f;
            for (const std::uint32_t u : neighbors[v]) total += (mesh.positions[u] - mesh.positions[v]).length();
            return neighbors[v].empty() ? 0.0f : total / static_cast<float>(neighbors[v].size());
        };
        for (std::uint32_t u = 0; u < mesh.positions.size() && !changed; ++u)
            for (const std::uint32_t v : neighbors[u]) {
                const float length = (mesh.positions[u] - mesh.positions[v]).length();
                if (length >= ratio * 0.5f * (meanEdge(u) + meanEdge(v))) continue;
                std::uint32_t gone = u, stays = v;
                if (keep(gone)) std::swap(gone, stays);
                if (keep(gone)) continue;
                if (!tryCollapse(mesh, gone, stays, neighbors, around, 12.0f * 3.14159265f / 180.0f)) continue;
                ++collapses;
                changed = true;
                break;
            }
    }
    return collapses;
}

std::size_t collapseInto(Mesh& mesh, const std::function<bool(std::uint32_t)>& goes, const std::function<bool(std::uint32_t)>& into,
    float minimumAngle) {
    std::size_t collapses = 0;
    for (bool changed = true; changed;) {
        changed = false;
        std::vector<std::set<std::uint32_t>> neighbors;
        std::vector<std::vector<std::size_t>> around;
        adjacency(mesh, neighbors, around);
        for (std::uint32_t v = 0; v < mesh.positions.size() && !changed; ++v) {
            if (around[v].empty() || !goes(v)) continue;
            std::vector<std::pair<float, std::uint32_t>> targets;
            for (const std::uint32_t u : neighbors[v])
                if (into(u)) targets.push_back({ (mesh.positions[u] - mesh.positions[v]).length(), u });
            std::sort(targets.begin(), targets.end());
            for (const auto& [length, u] : targets)
                if (tryCollapse(mesh, v, u, neighbors, around, minimumAngle * 3.14159265f / 180.0f)) {
                    ++collapses;
                    changed = true;
                    break;
                }
        }
    }
    return collapses;
}

} // namespace MatterEngine::Modeling
