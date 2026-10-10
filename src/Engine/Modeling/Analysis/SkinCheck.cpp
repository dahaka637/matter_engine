#include "Engine/Modeling/Analysis/SkinCheck.hpp"

#include <cmath>
#include <map>

namespace MatterEngine::Modeling {

bool regionsAdjacent(const Skeleton& skeleton, std::span<const int> jointRegion, int a, int b) {
    if (a == b) return true;
    for (std::size_t j = 0; j < skeleton.joints.size() && j < jointRegion.size(); ++j) {
        const int parent = skeleton.joints[j].parent;
        if (parent < 0 || static_cast<std::size_t>(parent) >= jointRegion.size()) continue;
        const int child = jointRegion[j], up = jointRegion[static_cast<std::size_t>(parent)];
        if ((child == a && up == b) || (child == b && up == a)) return true;
    }
    return false;
}

void checkSkin(const Mesh& mesh, const Skeleton& skeleton, const BodyRegions* regions,
    std::span<const int> jointRegion, Report& report) {
    if (!mesh.hasSkin()) {
        Finding finding;
        finding.severity = Severity::Info;
        finding.code = "sem_skin";
        finding.message = "a malha ainda nao tem pesos de skin (nao deforma nas poses)";
        report.add(std::move(finding));
        return;
    }
    std::vector<int> vertexPart(mesh.positions.size(), -1);
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
        for (const std::uint32_t v : mesh.triangles[t])
            if (vertexPart[v] < 0) vertexPart[v] = mesh.partOf(t);

    struct Group {
        Finding finding;
        std::size_t count = 0;
    };
    std::map<std::pair<std::string, int>, Group> groups;
    const auto flag = [&](const char* code, Severity severity, int part, Vec3 at) {
        Group& group = groups[{ code, part }];
        group.finding.severity = severity;
        group.finding.code = code;
        ++group.count;
        if (group.finding.points.size() < 60) group.finding.points.push_back(at);
        if (!group.finding.hasLocation) {
            group.finding.location = at;
            group.finding.hasLocation = true;
        }
    };

    const std::size_t jointCount = skeleton.joints.size();
    for (std::size_t v = 0; v < mesh.positions.size(); ++v) {
        if (vertexPart[v] < 0) continue;
        const SkinWeights& skin = mesh.skin[v];
        const Vec3 p = mesh.positions[v];
        const int part = vertexPart[v];
        float sum = 0.0f, best = 0.0f;
        int dominant = -1;
        bool invalid = false;
        for (std::size_t i = 0; i < 4; ++i) {
            const float weight = skin.weights[i];
            if (!std::isfinite(weight) || weight < 0.0f) invalid = true;
            if (weight <= 0.0f) continue;
            if (skin.joints[i] >= jointCount) {
                flag("peso_junta_inexistente", Severity::Error, part, p);
                continue;
            }
            if (skeleton.joints[skin.joints[i]].kind == JointKind::Tip) flag("peso_em_ponta", Severity::Warning, part, p);
            sum += weight;
            if (weight > best) {
                best = weight;
                dominant = skin.joints[i];
            }
        }
        if (invalid) flag("peso_invalido", Severity::Error, part, p);
        if (sum <= 1e-6f) {
            flag("sem_peso", Severity::Error, part, p);
            continue;
        }
        if (std::abs(sum - 1.0f) > 1e-3f) flag("pesos_nao_somam_1", Severity::Error, part, p);
        if (dominant < 0) continue;
        if (regions != nullptr && static_cast<std::size_t>(part) < mesh.parts.size()) {
            const int partRegion = regions->find(mesh.parts[static_cast<std::size_t>(part)].region);
            const int jointOwner = static_cast<std::size_t>(dominant) < jointRegion.size()
                ? jointRegion[static_cast<std::size_t>(dominant)] : -1;
            if (partRegion >= 0 && jointOwner >= 0 && !regionsAdjacent(skeleton, jointRegion, partRegion, jointOwner))
                flag("peso_de_outra_regiao", Severity::Warning, part, p);
        }
        // Longe do osso dominante (o segmento ate as filhas).
        float nearest = distance(p, skeleton.joints[static_cast<std::size_t>(dominant)].bind.position);
        for (std::size_t child = 0; child < jointCount; ++child)
            if (skeleton.joints[child].parent == dominant)
                nearest = std::min(nearest, pointSegmentDistance(p,
                    skeleton.joints[static_cast<std::size_t>(dominant)].bind.position,
                    skeleton.joints[child].bind.position));
        const float reach = std::max(0.06f, skeleton.joints[static_cast<std::size_t>(dominant)].radius * 3.0f);
        if (nearest > reach) flag("peso_longe_do_osso", Severity::Warning, part, p);
    }

    const std::map<std::string, std::string> messages {
        { "peso_junta_inexistente", "peso numa junta que nao existe" },
        { "peso_em_ponta", "peso numa ponta de osso (pontas nao deformam)" },
        { "peso_invalido", "peso negativo ou invalido" },
        { "sem_peso", "vertice sem peso (fica parado na pose)" },
        { "pesos_nao_somam_1", "pesos que nao somam 1" },
        { "peso_de_outra_regiao", "puxado por um osso de outra parte do corpo" },
        { "peso_longe_do_osso", "puxado por um osso distante" },
    };
    for (auto& [key, group] : groups) {
        Finding& finding = group.finding;
        const int part = key.second;
        finding.message = messages.at(finding.code) + " (" + std::to_string(group.count) + " vertices)";
        if (part >= 0 && static_cast<std::size_t>(part) < mesh.parts.size()) {
            finding.part = mesh.parts[static_cast<std::size_t>(part)].name;
            finding.region = mesh.parts[static_cast<std::size_t>(part)].region;
        }
        report.add(std::move(finding));
    }
}

} // namespace MatterEngine::Modeling
