#include "Engine/Modeling/Analysis/DeformationCheck.hpp"

#include "Engine/Modeling/Analysis/IntersectionCheck.hpp"
#include "Engine/Modeling/Geometry/Parallel.hpp"
#include "Engine/Modeling/Mesh/MeshSkinning.hpp"
#include "Engine/Modeling/Mesh/TriangleBvh.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <map>
#include <optional>
#include <set>

namespace MatterEngine::Modeling {

namespace {

// O avesso de um triangulo virado so aparece se um olhar de fora chega no
// verso dele: raios do centro, num cone em volta da normal do verso (a
// contraria a que ele tem na pose), saindo sem bater em nada; de onde se ve
// a frente dele nao e avesso. Virado dentro de uma prega (entre as duas peles
// que se apertam) nao aparece de nenhum lado. Os raios partem rente a ele: a
// outra pele de um vinco encosta a centesimos de milimetro e tampa a lasca
// virada (partindo de meio milimetro, o raio passava dela).
bool backVisibleFromOutside(const TriangleBvh& bvh, Vec3 center, Vec3 back) {
    constexpr std::array<float, 4> Polar { 0.0f, 25.0f, 50.0f, 70.0f };
    constexpr std::array<int, 4> Count { 1, 6, 8, 9 };
    const Vec3 u = anyPerpendicular(back), v = cross(back, u);
    int escaped = 0;
    for (std::size_t ring = 0; ring < Polar.size(); ++ring)
        for (int k = 0; k < Count[ring]; ++k) {
            const float polar = Polar[ring] * DegreesToRadians;
            const float azimuth = 2.0f * Pi * static_cast<float>(k) / static_cast<float>(Count[ring]) + 0.37f * static_cast<float>(ring);
            const Vec3 direction = back * std::cos(polar) + (u * std::cos(azimuth) + v * std::sin(azimuth)) * std::sin(polar);
            if (!bvh.raycast(center + back * 0.00001f, direction, 2.0f)) ++escaped;
        }
    // Um raio rasante saindo pela boca de uma prega nao basta.
    return escaped >= 2;
}

} // namespace

void checkDeformation(const Mesh& mesh, const Skeleton& skeleton, std::span<const PoseSample> poses, Report& report,
    const DeformationSettings& settings) {
    if (mesh.triangles.empty() || poses.empty()) return;
    if (!mesh.hasSkin()) {
        Finding finding;
        finding.severity = Severity::Info;
        finding.code = "deformacao_sem_skin";
        finding.message = "sem pesos de skin: as poses nao deformam a malha";
        report.add(std::move(finding));
        return;
    }
    const std::vector<int> dominant = dominantJoints(mesh);
    // Os vizinhos de aresta de cada triangulo (a dobra: a normal contra a de
    // um vizinho).
    std::vector<std::vector<std::uint32_t>> neighbors(mesh.triangles.size());
    {
        std::map<std::pair<std::uint32_t, std::uint32_t>, std::vector<std::uint32_t>> edges;
        for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
            for (int i = 0; i < 3; ++i) {
                const std::uint32_t a = mesh.triangles[t][static_cast<std::size_t>(i)];
                const std::uint32_t b = mesh.triangles[t][static_cast<std::size_t>((i + 1) % 3)];
                edges[{ std::min(a, b), std::max(a, b) }].push_back(static_cast<std::uint32_t>(t));
            }
        for (const auto& [edge, users] : edges)
            for (const std::uint32_t t : users)
                for (const std::uint32_t other : users)
                    if (other != t) neighbors[t].push_back(other);
    }
    // O verso que ja aparece na ligacao (uma borda aberta) nao e da pose;
    // medido so quando preciso (as poses correm em paralelo: duas threads que
    // medem o mesmo triangulo acham o mesmo valor).
    const TriangleBvh bindBvh(mesh.positions, mesh.triangles);
    std::vector<std::atomic<signed char>> backAtBind(mesh.triangles.size());
    for (std::atomic<signed char>& known : backAtBind) known.store(-1, std::memory_order_relaxed);
    const auto visibleAtBind = [&](std::size_t t) {
        signed char known = backAtBind[t].load(std::memory_order_relaxed);
        if (known < 0) {
            const Triangle& tri = mesh.triangles[t];
            const Vec3 center = (mesh.positions[tri[0]] + mesh.positions[tri[1]] + mesh.positions[tri[2]]) / 3.0f;
            known = backVisibleFromOutside(bindBvh, center, mesh.normal(t) * -1.0f) ? 1 : 0;
            backAtBind[t].store(known, std::memory_order_relaxed);
        }
        return known == 1;
    };
    // O que ja se cruza na ligacao nao e problema da pose.
    std::set<std::pair<std::string, std::string>> crossingAtBind;
    if (settings.intersections) {
        Report bindReport;
        checkIntersections(mesh, mesh.positions, bindReport);
        for (const Finding& finding : bindReport.findings()) crossingAtBind.insert({ finding.part, finding.otherPart });
    }
    // Cada pose no relatorio dela (em paralelo), juntados na ordem das poses.
    const auto check = [&](const PoseSample& sample, Report& report) {
        const std::vector<Transform> skinning = skinningTransforms(skeleton, sample.pose);
        const std::vector<Vec3> posed = mesh.posedPositions(skinning);
        std::vector<Vec3> normals(mesh.triangles.size());
        for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
            normals[t] = triangleNormal(posed[mesh.triangles[t][0]], posed[mesh.triangles[t][1]], posed[mesh.triangles[t][2]]);
        std::optional<TriangleBvh> bvh;
        struct Group {
            Finding finding;
            std::size_t count = 0;
        };
        std::map<std::pair<std::string, int>, Group> groups;
        for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
            const Triangle& tri = mesh.triangles[t];
            const float before = triangleArea(mesh.positions[tri[0]], mesh.positions[tri[1]], mesh.positions[tri[2]]);
            if (before < 1e-10f) continue;
            const float after = triangleArea(posed[tri[0]], posed[tri[1]], posed[tri[2]]);
            const float ratio = after / before;
            const char* code = nullptr;
            // Virou: a normal na pose contra a que o skin do triangulo pede
            // (a normal da ligacao girada pelas juntas dos tres vertices, nos
            // pesos deles). So o osso dominante nao basta: numa junta 50/50
            // dobrada a 100 graus o triangulo segue a mistura, nao o osso de
            // cima. Antes do tamanho: o lado de dentro de uma junta que fecha
            // amassa e vira junto, e o que aparece e o avesso.
            if (after > 1e-12f) {
                const Vec3 bindNormal = mesh.normal(t);
                Vec3 expected {};
                for (const std::uint32_t v : tri) {
                    const SkinWeights& weights = mesh.skin[v];
                    for (std::size_t i = 0; i < weights.weights.size(); ++i)
                        if (weights.weights[i] > 0.0f && weights.joints[i] < skinning.size())
                            expected += skinning[weights.joints[i]].applyVector(bindNormal) * weights.weights[i];
                }
                const int joint = dominant[tri[0]];
                if (expected.lengthSquared() < 1e-12f && joint >= 0 && static_cast<std::size_t>(joint) < skinning.size())
                    expected = skinning[static_cast<std::size_t>(joint)].applyVector(bindNormal);
                const Vec3 actual = normals[t];
                const Vec3 outward = safeNormalized(expected, actual);
                const bool flipped = expected.lengthSquared() > 1e-12f && dot(outward, actual) < -0.2f;
                // A dobra: a mistura dos giros pode virar a normal esperada
                // junto (na barra da camisa, a pelve e a coxa dobrada) - o
                // triangulo segue o skin e mesmo assim fica do avesso contra
                // os vizinhos.
                bool folded = false;
                if (!flipped)
                    for (const std::uint32_t other : neighbors[t]) folded = folded || dot(actual, normals[other]) < 0.0f;
                if (flipped || folded) {
                    // Virou: so e erro se aparece; dentro da prega, observacao.
                    if (!bvh) bvh.emplace(posed, mesh.triangles);
                    const Vec3 center = (posed[tri[0]] + posed[tri[1]] + posed[tri[2]]) / 3.0f;
                    const bool shows = backVisibleFromOutside(*bvh, center, actual * -1.0f);
                    if (flipped) code = shows ? "virou" : "prega";
                    else if (shows && !visibleAtBind(t)) code = "virou";
                }
            }
            if (code == nullptr) code = ratio > settings.stretchLimit ? "esticou" : ratio < settings.compressLimit ? "amassou" : nullptr;
            if (code == nullptr) continue;
            Group& group = groups[{ code, mesh.partOf(t) }];
            ++group.count;
            group.finding.triangles.push_back(static_cast<std::uint32_t>(t));
            const Vec3 at = (posed[tri[0]] + posed[tri[1]] + posed[tri[2]]) / 3.0f;
            if (group.finding.points.size() < 40) group.finding.points.push_back(at);
            if (!group.finding.hasLocation) {
                group.finding.location = at;
                group.finding.hasLocation = true;
            }
        }
        const std::map<std::string, std::string> messages {
            { "esticou", "triangulos esticados demais" },
            { "amassou", "triangulos amassados (perda de volume na junta)" },
            { "virou", "triangulos que viram do avesso a vista" },
            { "prega", "triangulos virados escondidos dentro de uma prega" },
        };
        for (auto& [key, group] : groups) {
            Finding& finding = group.finding;
            finding.severity = key.first == "virou" ? Severity::Error
                : key.first == "prega"              ? Severity::Info
                                                    : Severity::Warning;
            finding.code = key.first + "_pose";
            finding.message = messages.at(key.first) + " na pose " + sample.name + " (" + std::to_string(group.count) + ")";
            if (key.second >= 0 && static_cast<std::size_t>(key.second) < mesh.parts.size()) {
                finding.part = mesh.parts[static_cast<std::size_t>(key.second)].name;
                finding.region = mesh.parts[static_cast<std::size_t>(key.second)].region;
            }
            report.add(std::move(finding));
        }
        if (settings.intersections) {
            Report posedReport;
            checkIntersections(mesh, posed, posedReport, sample.name);
            // Na pose interessa so o que muda entre partes.
            for (const Finding& finding : posedReport.findings())
                if (crossingAtBind.count({ finding.part, finding.otherPart }) == 0) report.add(finding);
            for (const auto& [name, value] : posedReport.statistics()) report.statistic(name, value);
        }
    };
    std::vector<Report> reports(poses.size());
    parallelFor(poses.size(), 1, [&](std::size_t first, std::size_t last) {
        for (std::size_t i = first; i < last; ++i) check(poses[i], reports[i]);
    });
    for (const Report& posed : reports) {
        for (const Finding& finding : posed.findings()) report.add(finding);
        for (const auto& [name, value] : posed.statistics()) report.statistic(name, value);
    }
}

} // namespace MatterEngine::Modeling
