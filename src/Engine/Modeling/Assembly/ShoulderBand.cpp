#include "Engine/Modeling/Assembly/ShoulderBand.hpp"

#include "Engine/Modeling/Geometry/ModelGeometry.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>
#include <vector>

namespace MatterEngine::Modeling {

namespace {

float smooth(float edge0, float edge1, float x) {
    const float t = std::clamp((x - edge0) / std::max(1e-6f, edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// Os "count" ossos mais fortes de uma distribuicao, somando 1.
std::vector<std::pair<int, float>> strongestOf(const std::map<int, float>& weights, std::size_t count) {
    std::vector<std::pair<int, float>> ranked(weights.begin(), weights.end());
    std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) { return a.second > b.second; });
    if (ranked.size() > count) ranked.resize(count);
    float total = 0.0f;
    for (const auto& item : ranked) total += item.second;
    for (auto& item : ranked) item.second = total > 0.0f ? item.second / total : 0.0f;
    return ranked;
}

} // namespace

std::size_t applyShoulderBand(Mesh& mesh, const Skeleton& skeleton, const ShoulderBand& band,
    const std::function<float(Vec3)>& collarHeight) {
    const int upper = skeleton.find("UpperChest"), helper = skeleton.find("RightShoulderHelper");
    const int arm = skeleton.find("RightUpperArm"), elbow = skeleton.find("RightForearm");
    if (!band.enabled() || !mesh.hasSkin() || upper < 0 || helper < 0 || arm < 0 || elbow < 0) return 0;
    // O referencial do braco na ligacao: ao longo dele, a frente e o alto/fora.
    const Vec3 joint = skeleton.joints[static_cast<std::size_t>(arm)].bind.position;
    const Vec3 along = safeNormalized(skeleton.joints[static_cast<std::size_t>(elbow)].bind.position - joint, { 0.0f, -1.0f, 0.0f });
    const Vec3 front = safeNormalized(Vec3 { 1.0f, 0.0f, 0.0f } - along * along.x, { 1.0f, 0.0f, 0.0f });
    const Vec3 up = cross(along, front);
    // O que e braco: ele e o que pende dele (os auxiliares pelo osso de cima);
    // a meia-junta do ombro nao e nem tronco nem braco (o campo da a dela).
    std::vector<char> limbJoint(skeleton.joints.size(), 0);
    for (std::size_t j = 0; j < skeleton.joints.size(); ++j) {
        const Joint& item = skeleton.joints[j];
        int k = item.kind == JointKind::Helper ? item.derivedFrom : static_cast<int>(j);
        for (; k >= 0; k = skeleton.joints[static_cast<std::size_t>(k)].parent)
            if (k == arm) {
                limbJoint[j] = 1;
                break;
            }
    }
    std::size_t changed = 0;
    for (std::size_t v = 0; v < mesh.positions.size(); ++v) {
        const Vec3 p = mesh.positions[v];
        const Vec3 offset = p - joint;
        const float s = dot(offset, along);
        if (s > band.rigidUntil) continue;
        const Vec3 radial = offset - along * s;
        const float distanceToAxis = radial.length();
        const Vec3 around = distanceToAxis > 1e-6f ? radial / distanceToAxis : up;
        // As quatro direcoes em volta do braco (a parte positiva de cada uma,
        // ao quadrado), como a faixa do quadril.
        const float f = dot(around, front), o = dot(around, up);
        const std::array<float, 4> share { f > 0.0f ? f * f : 0.0f, o > 0.0f ? o * o : 0.0f, f < 0.0f ? f * f : 0.0f, o < 0.0f ? o * o : 0.0f };
        const float total = std::max(1e-6f, share[0] + share[1] + share[2] + share[3]);
        const auto mix = [&](const std::array<float, 4>& value) {
            return (share[0] * value[0] + share[1] * value[1] + share[2] * value[2] + share[3] * value[3]) / total;
        };
        float reach = (1.0f - smooth(mix(band.full), mix(band.end), distanceToAxis)) * smooth(band.midline[0], band.midline[1], std::abs(p.y));
        if (collarHeight) reach *= smooth(band.collarGuard[0], band.collarGuard[1], collarHeight(p) - p.z);
        if (reach <= 0.0f) continue;
        const float t = std::clamp(0.5f + (s - mix(band.center)) / std::max(1e-3f, mix(band.width)), 0.0f, 1.0f);
        const float chest = std::max(1.0f - 2.0f * t, 0.0f), half = 1.0f - std::abs(2.0f * t - 1.0f), limb = std::max(2.0f * t - 1.0f, 0.0f);
        // O tronco e o braco que o vertice ja tem.
        std::map<int, float> torsoWeights, limbWeights;
        for (std::size_t i = 0; i < 4; ++i) {
            const float w = mesh.skin[v].weights[i];
            const int j = mesh.skin[v].joints[i];
            if (w <= 0.0f || j == helper) continue;
            (limbJoint[static_cast<std::size_t>(j)] ? limbWeights : torsoWeights)[j] += w;
        }
        if (limbWeights.empty() && reach * (half + limb) <= 0.0f) continue;
        if (torsoWeights.empty()) torsoWeights[upper] = 1.0f;
        if (limbWeights.empty()) limbWeights[arm] = 1.0f;
        // No maximo quatro: com o braco presente, dois do tronco; com a meia
        // tambem, um do braco.
        const bool withLimb = reach * (half + limb) > 0.0f;
        const auto torso = strongestOf(torsoWeights, withLimb ? 2 : 4);
        const auto limbPart = strongestOf(limbWeights, half > 0.0f && torso.size() >= 2 ? 1 : 2);
        std::map<int, float> weights;
        for (const auto& [j, w] : torso) weights[j] += (1.0f - reach + reach * chest) * w;
        if (reach * half > 0.0f) weights[helper] += reach * half;
        for (const auto& [j, w] : limbPart) weights[j] += reach * limb * w;
        const auto ranked = strongestOf(weights, 4);
        SkinWeights result {};
        for (std::size_t i = 0; i < ranked.size(); ++i) {
            result.joints[i] = static_cast<std::uint16_t>(ranked[i].first);
            result.weights[i] = ranked[i].second;
        }
        mesh.skin[v] = result;
        ++changed;
    }
    return changed;
}

} // namespace MatterEngine::Modeling
