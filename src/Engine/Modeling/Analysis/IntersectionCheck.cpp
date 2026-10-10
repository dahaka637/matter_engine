#include "Engine/Modeling/Analysis/IntersectionCheck.hpp"

#include "Engine/Modeling/Mesh/TriangleBvh.hpp"

#include <algorithm>
#include <map>
#include <utility>

namespace MatterEngine::Modeling {

void checkIntersections(const Mesh& mesh, std::span<const Vec3> positions, Report& report, std::string_view context,
    const IntersectionSettings& settings) {
    if (mesh.triangles.empty() || positions.size() != mesh.positions.size()) return;
    const TriangleBvh bvh(positions, mesh.triangles);
    std::map<std::pair<int, int>, Finding> groups;
    std::size_t total = 0;
    bvh.overlappingPairs([&](std::uint32_t a, std::uint32_t b) {
        const Triangle& ta = mesh.triangles[a];
        const Triangle& tb = mesh.triangles[b];
        for (const std::uint32_t va : ta)
            for (const std::uint32_t vb : tb)
                if (va == vb || (positions[va] - positions[vb]).lengthSquared() < 1e-14f) return;
        if (!trianglesIntersect(positions[ta[0]], positions[ta[1]], positions[ta[2]], positions[tb[0]],
                positions[tb[1]], positions[tb[2]]))
            return;
        const int partA = mesh.partOf(a), partB = mesh.partOf(b);
        ++total;
        Finding& finding = groups[{ std::min(partA, partB), std::max(partA, partB) }];
        if (finding.triangles.size() < 4000) {
            finding.triangles.push_back(a);
            finding.triangles.push_back(b);
        }
        if (finding.points.size() < settings.maximumSamples) {
            const Vec3 ca = (positions[ta[0]] + positions[ta[1]] + positions[ta[2]]) / 3.0f;
            const Vec3 cb = (positions[tb[0]] + positions[tb[1]] + positions[tb[2]]) / 3.0f;
            finding.points.push_back((ca + cb) * 0.5f);
        }
    });
    const auto name = [&](int part) {
        return part >= 0 && static_cast<std::size_t>(part) < mesh.parts.size()
            ? mesh.parts[static_cast<std::size_t>(part)].name : std::string("?");
    };
    const std::string suffix = context.empty() ? std::string() : " na pose " + std::string(context);
    for (auto& [pair, finding] : groups) {
        const bool same = pair.first == pair.second;
        const std::size_t pairs = finding.triangles.size() / 2;
        finding.severity = context.empty() ? Severity::Error : Severity::Warning;
        finding.code = same ? "autointersecao" : "partes_se_cruzam";
        if (!context.empty()) finding.code += "_pose";
        finding.message = same ? "a parte cruza ela mesma" + suffix + " (" + std::to_string(pairs) + " pares)"
                               : name(pair.first) + " atravessa " + name(pair.second) + suffix + " ("
                + std::to_string(pairs) + " pares)";
        finding.part = name(pair.first);
        if (!same) finding.otherPart = name(pair.second);
        if (pair.first >= 0 && static_cast<std::size_t>(pair.first) < mesh.parts.size())
            finding.region = mesh.parts[static_cast<std::size_t>(pair.first)].region;
        finding.location = finding.points.front();
        finding.hasLocation = true;
        report.add(std::move(finding));
    }
    report.statistic(context.empty() ? std::string("pares de triangulos que se cruzam")
                                     : "pares que se cruzam na pose " + std::string(context),
        std::to_string(total));
}

} // namespace MatterEngine::Modeling
