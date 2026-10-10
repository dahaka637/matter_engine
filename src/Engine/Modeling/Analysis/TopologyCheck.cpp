#include "Engine/Modeling/Analysis/TopologyCheck.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <numeric>
#include <unordered_map>

namespace MatterEngine::Modeling {

namespace {

struct EdgeUse {
    int count = 0;
    int forward = 0;
    int backward = 0;
    std::uint32_t triangle = 0;
};

std::uint64_t edgeKey(std::uint32_t a, std::uint32_t b) {
    return (static_cast<std::uint64_t>(std::min(a, b)) << 32) | std::max(a, b);
}

struct UnionFind {
    std::vector<std::uint32_t> parent;
    explicit UnionFind(std::size_t count) : parent(count) { std::iota(parent.begin(), parent.end(), 0u); }
    std::uint32_t find(std::uint32_t x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    }
    void unite(std::uint32_t a, std::uint32_t b) { parent[find(a)] = find(b); }
};

std::string partName(const Mesh& mesh, int part) {
    return part >= 0 && static_cast<std::size_t>(part) < mesh.parts.size() ? mesh.parts[static_cast<std::size_t>(part)].name
                                                                          : std::string();
}

std::string partRegion(const Mesh& mesh, int part) {
    return part >= 0 && static_cast<std::size_t>(part) < mesh.parts.size()
        ? mesh.parts[static_cast<std::size_t>(part)].region : std::string();
}

float minimumAngle(Vec3 a, Vec3 b, Vec3 c) {
    const auto angle = [](Vec3 at, Vec3 p, Vec3 q) {
        const Vec3 u = safeNormalized(p - at, {}), v = safeNormalized(q - at, {});
        return std::acos(std::clamp(dot(u, v), -1.0f, 1.0f));
    };
    return std::min({ angle(a, b, c), angle(b, c, a), angle(c, a, b) });
}

} // namespace

void checkTopology(const Mesh& mesh, Report& report, const TopologySettings& settings) {
    const std::size_t vertexCount = mesh.positions.size();
    const std::size_t triangleCount = mesh.triangles.size();
    report.statistic("vertices", std::to_string(vertexCount));
    report.statistic("triangulos", std::to_string(triangleCount));
    report.statistic("partes", std::to_string(mesh.parts.size()));
    report.statistic("materiais", std::to_string(mesh.materials.size()));
    if (triangleCount == 0) return;

    std::unordered_map<std::uint64_t, EdgeUse> edges;
    edges.reserve(triangleCount * 2);
    for (std::uint32_t t = 0; t < triangleCount; ++t) {
        const Triangle& tri = mesh.triangles[t];
        for (int i = 0; i < 3; ++i) {
            const std::uint32_t a = tri[static_cast<std::size_t>(i)], b = tri[static_cast<std::size_t>((i + 1) % 3)];
            EdgeUse& use = edges[edgeKey(a, b)];
            if (use.count == 0) use.triangle = t;
            ++use.count;
            if (a < b) ++use.forward;
            else ++use.backward;
        }
    }

    // As bordas abertas sao da checagem de conexao (ConnectivityCheck); aqui
    // elas so dizem quais pedacos sao abertos (sem volume para orientar).
    std::vector<std::pair<std::uint64_t, const EdgeUse*>> boundary, nonManifold, flipped;
    for (const auto& [key, use] : edges) {
        if (use.count == 1) boundary.push_back({ key, &use });
        else if (use.count > 2) nonManifold.push_back({ key, &use });
        else if (use.forward == 2 || use.backward == 2) flipped.push_back({ key, &use });
    }

    const auto edgeFinding = [&](const std::vector<std::pair<std::uint64_t, const EdgeUse*>>& list, Severity severity,
                                 const char* code, const std::string& message) {
        if (list.empty()) return;
        std::map<int, Finding> byPart;
        for (const auto& [key, use] : list) {
            const std::uint32_t a = static_cast<std::uint32_t>(key >> 32), b = static_cast<std::uint32_t>(key & 0xffffffffu);
            Finding& finding = byPart[mesh.partOf(use->triangle)];
            finding.segments.push_back({ mesh.positions[a], mesh.positions[b] });
            finding.triangles.push_back(use->triangle);
        }
        for (auto& [part, finding] : byPart) {
            finding.severity = severity;
            finding.code = code;
            finding.message = message + " (" + std::to_string(finding.segments.size()) + ")";
            finding.part = partName(mesh, part);
            finding.region = partRegion(mesh, part);
            finding.location = (finding.segments.front().first + finding.segments.front().second) * 0.5f;
            finding.hasLocation = true;
            report.add(std::move(finding));
        }
    };
    edgeFinding(nonManifold, Severity::Error, "aresta_nao_manifold", "aresta usada por mais de dois triangulos");
    edgeFinding(flipped, Severity::Error, "orientacao_trocada",
        "triangulos vizinhos com a frente para lados opostos");

    // Degenerados e agulhas, por parte. A agulha no plano de uma vizinha (um
    // pedaco de face cortada ou costurada) nao aparece: so observacao.
    std::unordered_map<std::uint64_t, std::array<std::uint32_t, 2>> edgeTriangles;
    edgeTriangles.reserve(triangleCount * 2);
    for (std::uint32_t t = 0; t < triangleCount; ++t)
        for (std::size_t i = 0; i < 3; ++i) {
            const auto [it, inserted] = edgeTriangles.try_emplace(edgeKey(mesh.triangles[t][i], mesh.triangles[t][(i + 1) % 3]),
                std::array<std::uint32_t, 2> { t, UINT32_MAX });
            if (!inserted) it->second[1] = t;
        }
    const float flatCosine = std::cos(settings.flatSliverDegrees * DegreesToRadians);
    // No plano de alguma vizinha: um pedaco da mesma face (corte, costura).
    const auto flatWithNeighbors = [&](std::uint32_t t) {
        const Vec3 normal = mesh.normal(t);
        for (std::size_t i = 0; i < 3; ++i) {
            const auto& pair = edgeTriangles.at(edgeKey(mesh.triangles[t][i], mesh.triangles[t][(i + 1) % 3]));
            const std::uint32_t other = pair[0] == t ? pair[1] : pair[0];
            if (other != UINT32_MAX && dot(normal, mesh.normal(other)) >= flatCosine) return true;
        }
        return false;
    };
    std::map<int, Finding> degenerate, slivers, flatSlivers;
    for (std::uint32_t t = 0; t < triangleCount; ++t) {
        const Triangle& tri = mesh.triangles[t];
        const Vec3 a = mesh.positions[tri[0]], b = mesh.positions[tri[1]], c = mesh.positions[tri[2]];
        if (triangleArea(a, b, c) < settings.minimumArea) {
            Finding& finding = degenerate[mesh.partOf(t)];
            finding.triangles.push_back(t);
            finding.points.push_back((a + b + c) / 3.0f);
        } else if (minimumAngle(a, b, c) * RadiansToDegrees < settings.minimumAngleDegrees) {
            Finding& finding = (flatWithNeighbors(t) ? flatSlivers : slivers)[mesh.partOf(t)];
            finding.triangles.push_back(t);
            finding.points.push_back((a + b + c) / 3.0f);
        }
    }
    const auto emit = [&](std::map<int, Finding>& groups, Severity severity, const char* code, const char* message) {
        for (auto& [part, finding] : groups) {
            finding.severity = severity;
            finding.code = code;
            finding.message = std::string(message) + " (" + std::to_string(finding.triangles.size()) + ")";
            finding.part = partName(mesh, part);
            finding.region = partRegion(mesh, part);
            finding.location = finding.points.front();
            finding.hasLocation = true;
            report.add(std::move(finding));
        }
    };
    emit(degenerate, Severity::Error, "triangulo_degenerado", "triangulo sem area");
    emit(slivers, Severity::Warning, "triangulo_agulha", "triangulo muito fino (agulha) dobrando a superficie");
    emit(flatSlivers, Severity::Info, "agulha_plana", "triangulo muito fino no plano de uma vizinha (nao aparece)");

    std::vector<int> used(vertexCount, 0);
    for (std::uint32_t t = 0; t < triangleCount; ++t)
        for (const std::uint32_t v : mesh.triangles[t]) ++used[v];
    const std::size_t unused = static_cast<std::size_t>(std::count(used.begin(), used.end(), 0));
    if (unused > 0) {
        Finding finding;
        finding.severity = Severity::Info;
        finding.code = "vertice_solto";
        finding.message = std::to_string(unused) + " vertice(s) fora de qualquer triangulo";
        report.add(std::move(finding));
    }

    // Os pedacos fechados (triangulos ligados por vertices, sem borda) com a
    // orientacao de cada um.
    UnionFind components(vertexCount);
    for (const Triangle& tri : mesh.triangles) {
        components.unite(tri[0], tri[1]);
        components.unite(tri[1], tri[2]);
    }
    std::map<std::uint32_t, std::vector<std::uint32_t>> componentTriangles;
    for (std::uint32_t t = 0; t < triangleCount; ++t) componentTriangles[components.find(mesh.triangles[t][0])].push_back(t);
    std::map<std::uint32_t, bool> componentOpen;
    for (const auto& [key, use] : boundary) componentOpen[components.find(static_cast<std::uint32_t>(key >> 32))] = true;
    for (const auto& [root, triangles] : componentTriangles) {
        if (componentOpen.count(root) != 0) continue;
        double volume = 0.0;
        // Origem local: pecas submilimetricas longe de (0,0,0) perdiam o
        // sinal por cancelamento dos produtos em float (ilhoses no pe).
        const Vec3 origin = mesh.positions[mesh.triangles[triangles.front()][0]];
        for (const std::uint32_t t : triangles) {
            const Triangle& tri = mesh.triangles[t];
            volume += dot(mesh.positions[tri[0]] - origin,
                cross(mesh.positions[tri[1]] - origin, mesh.positions[tri[2]] - origin));
        }
        if (volume < 0.0) {
            Finding finding;
            finding.severity = Severity::Error;
            finding.code = "normais_para_dentro";
            finding.message = "peca fechada com as normais para dentro (" + std::to_string(triangles.size())
                + " triangulos)";
            const int part = mesh.partOf(triangles.front());
            finding.part = partName(mesh, part);
            finding.region = partRegion(mesh, part);
            finding.triangles = triangles;
            finding.location = mesh.center(triangles.front());
            finding.hasLocation = true;
            report.add(std::move(finding));
        }
    }
}

} // namespace MatterEngine::Modeling
