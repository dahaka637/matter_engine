#include "Engine/Modeling/Analysis/SymmetryCheck.hpp"

#include "Engine/Modeling/Mesh/TriangleBvh.hpp"

#include <algorithm>
#include <map>

namespace MatterEngine::Modeling {

std::vector<float> mirrorDeviation(const Mesh& mesh, float searchRadius) {
    std::vector<float> deviation(mesh.positions.size(), 0.0f);
    if (mesh.triangles.empty()) return deviation;
    const TriangleBvh bvh(mesh.positions, mesh.triangles);
    for (std::size_t v = 0; v < mesh.positions.size(); ++v) {
        const Vec3 p = mesh.positions[v];
        const auto nearest = bvh.nearest({ p.x, -p.y, p.z }, searchRadius);
        deviation[v] = nearest ? nearest->distance : searchRadius;
    }
    return deviation;
}

void checkSymmetry(const Mesh& mesh, Report& report, const SymmetrySettings& settings) {
    if (mesh.triangles.empty()) return;
    const float search = std::max(settings.tolerance * 30.0f, 0.05f);
    const std::vector<float> deviation = mirrorDeviation(mesh, search);
    std::vector<int> vertexPart(mesh.positions.size(), -1);
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
        for (const std::uint32_t v : mesh.triangles[t])
            if (vertexPart[v] < 0) vertexPart[v] = mesh.partOf(t);
    struct PartStats {
        std::size_t vertices = 0;
        std::size_t off = 0;
        float worst = 0.0f;
        Vec3 worstAt;
        std::vector<Vec3> points;
    };
    std::map<int, PartStats> stats;
    float globalWorst = 0.0f;
    std::size_t globalOff = 0, considered = 0;
    for (std::size_t v = 0; v < mesh.positions.size(); ++v) {
        if (vertexPart[v] < 0) continue;
        ++considered;
        PartStats& part = stats[vertexPart[v]];
        ++part.vertices;
        globalWorst = std::max(globalWorst, deviation[v]);
        if (deviation[v] <= settings.tolerance) continue;
        ++part.off;
        ++globalOff;
        if (part.points.size() < 60) part.points.push_back(mesh.positions[v]);
        if (deviation[v] > part.worst) {
            part.worst = deviation[v];
            part.worstAt = mesh.positions[v];
        }
    }
    for (auto& [part, s] : stats) {
        if (s.off == 0) continue;
        Finding finding;
        finding.severity = Severity::Warning;
        finding.code = "assimetria";
        finding.message = std::to_string(s.off) + " de " + std::to_string(s.vertices)
            + " vertices sem par no espelho (ate " + formatLength(s.worst) + ")";
        if (part >= 0 && static_cast<std::size_t>(part) < mesh.parts.size()) {
            finding.part = mesh.parts[static_cast<std::size_t>(part)].name;
            finding.region = mesh.parts[static_cast<std::size_t>(part)].region;
        }
        finding.location = s.worstAt;
        finding.hasLocation = true;
        finding.points = std::move(s.points);
        report.add(std::move(finding));
    }
    report.statistic("simetria (vertices com par no espelho)",
        std::to_string(considered - globalOff) + " de " + std::to_string(considered) + ", pior "
            + formatLength(globalWorst));
}

} // namespace MatterEngine::Modeling
