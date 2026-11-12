#include "Engine/Tracking/TrackingClipGetUp3D.hpp"

#include "Engine/RL/Body/RLBody3D.hpp"

#include <algorithm>
#include <cmath>
#include <array>
#include <limits>
#include <string>
#include <vector>

namespace MatterEngine {

TrackingClipGetUp3D::Terms TrackingClipGetUp3D::score(const TrackingGetUpContract3D& contract, const RagdollProfile3D& profile,
    std::span<const PhysicsBodyState3D> body, const TrackingClipFrame3D& reference, Widths widths, float groundHeight) {
    const TrackingGetUpContract3D::Terms base = contract.reward(body, reference, widths.softness);
    Terms terms;
    terms.pose = base.pose;
    terms.velocity = base.velocity;
    terms.endEffector = base.endEffector;
    terms.centerOfMass = base.centerOfMass;
    const Vec3 v = rlBodyCenterOfMassVelocity3D(profile, body);
    const Vec3 vr = rlBodyCenterOfMassVelocity3D(profile, reference.links);
    const float width = std::max(0.05f, widths.comVelocity);
    terms.comVelocity = std::exp(-(v - vr).lengthSquared() / (width * width));
    if (widths.parts) {
        // A pose por parte (o tronco com a raiz, cada braco, cada perna) e maos e pes um a um - ver Widths::parts.
        const double k = 1.0 / std::max(1.0e-3f, widths.softness);
        thread_local std::vector<float> errors;
        errors.assign(profile.links.size(), 0.0f);
        contract.jointErrors(body, reference, errors);
        std::array<double, 5> sum {};
        for (std::size_t j = 0; j < profile.links.size(); ++j) {
            const std::string& id = profile.links[j].id;
            const bool left = id.rfind("Left", 0) == 0, right = id.rfind("Right", 0) == 0;
            const bool arm = id.find("Arm") != std::string::npos || id.find("Hand") != std::string::npos;
            const std::size_t part = left ? (arm ? 1 : 3) : right ? (arm ? 2 : 4) : 0;
            sum[part] += errors[j];
        }
        double pose = 0.0;
        for (const double s : sum) pose += std::exp(-2.0 * k * s);
        const double e = std::clamp(static_cast<double>(widths.emphasis), 0.0, 2.0);
        if (e > 0.0) {
            // (Widths::emphasis) O tronco (0) e os bracos (1, 2): a media geometrica das notas de pose deles elevada a emphasis.
            terms.emphasis = static_cast<float>(std::exp(-2.0 * k * (sum[0] + sum[1] + sum[2]) * e / 3.0));
        }
        // (17h40, a sonda: so por partes a nota afrouxava - o braco fora custava 0,18 contra 0,62 da do corpo inteiro e o atrasado
        // 0,2 s subia de 0,40 para 0,66. Metade a do corpo inteiro, rigida perto do clipe, metade por partes, o gradiente de cada uma.)
        terms.pose = 0.5f * base.pose + 0.5f * static_cast<float>(pose / sum.size());
        double end = 0.0;
        int count = 0;
        for (const char* name : { "LeftHand", "RightHand", "LeftFoot", "RightFoot" })
            for (std::size_t j = 0; j < profile.links.size(); ++j)
                if (profile.links[j].id == name && j < body.size() && j < reference.links.size()) {
                    const Vec3 d = rlLinkCenterOfMass3D(profile.links[j], reference.links[j]) - rlLinkCenterOfMass3D(profile.links[j], body[j]);
                    end += std::exp(-40.0 * k * d.lengthSquared());
                    ++count;
                }
        terms.endEffector = 0.5f * base.endEffector + 0.5f * (count ? static_cast<float>(end / count) : 0.0f);
    }
    // O pulo: os pes apoiados no clipe e o quanto os do corpo estao acima deles (o menor dos dois).
    const auto feet = contract.footLinks();
    float lifted = std::numeric_limits<float>::max();
    bool planted = false;
    for (const std::uint32_t f : feet) {
        if (f >= body.size() || f >= reference.links.size()) continue;
        const float clipFoot = rlLinkCenterOfMass3D(profile.links[f], reference.links[f]).z;
        if (clipFoot - groundHeight > PlantedFootHeight) continue;
        planted = true;
        lifted = std::min(lifted, rlLinkCenterOfMass3D(profile.links[f], body[f]).z - clipFoot);
    }
    if (planted) {
        const float x = std::max(0.0f, lifted - HopSlack) / HopWidth;
        terms.hop = std::exp(-x * x);
    }
    if (widths.parts) {
        // O tronco (TrunkWidth): a linha pelve -> peito alto contra a do clipe.
        const std::uint32_t chest = contract.chestLink(), root = contract.rootLink();
        if (chest < body.size() && root < body.size() && chest < reference.links.size() && root < reference.links.size()) {
            const Vec3 a = rlLinkCenterOfMass3D(profile.links[chest], body[chest]) - rlLinkCenterOfMass3D(profile.links[root], body[root]);
            const Vec3 b = rlLinkCenterOfMass3D(profile.links[chest], reference.links[chest])
                - rlLinkCenterOfMass3D(profile.links[root], reference.links[root]);
            const float la = a.length(), lb = b.length();
            if (la > 1.0e-4f && lb > 1.0e-4f) {
                const float angle = std::acos(std::clamp((a.x * b.x + a.y * b.y + a.z * b.z) / (la * lb), -1.0f, 1.0f)) / TrunkWidth;
                terms.trunk = std::exp(-angle * angle);
            }
        }
    }
    terms.total = (widths.parts
                          ? PartsPoseWeight * terms.pose + PartsVelocityWeight * terms.velocity + PartsEndEffectorWeight * terms.endEffector
                              + PartsCenterWeight * terms.centerOfMass + PartsComVelocityWeight * terms.comVelocity
                          : PoseWeight * terms.pose + VelocityWeight * terms.velocity + EndEffectorWeight * terms.endEffector
                              + CenterWeight * terms.centerOfMass + ComVelocityWeight * terms.comVelocity) * terms.hop * terms.trunk
        * terms.emphasis;
    return terms;
}

int TrackingClipGetUp3D::far(const TrackingGetUpContract3D& contract, const RagdollProfile3D& profile,
    std::span<const PhysicsBodyState3D> body, const TrackingClipFrame3D& reference, float facingLimit) {
    const Vec3 com = rlBodyCenterOfMass3D(profile, body);
    if ((com - reference.centerOfMass).length() > FarCenterMeters) return 1;
    const std::uint32_t head = contract.headLink();
    if (head < body.size() && head < reference.links.size() && body[head].position.z < reference.links[head].position.z - FarHeadMeters)
        return 1;
    // (11/11 18h10, ele: o rolando "parece estar imitando o levantar de costas normal" - a avaliacao levantava 16/16 com a
    // semelhanca 0,34: o longe de 0,5 m deixava levantar pelo outro caminho.) O peito virado ao contrario do clipe - de costas
    // com o clipe de brucos, ou o inverso - tambem e longe.
    const std::uint32_t chest = contract.chestLink();
    if (chest >= body.size() || chest >= reference.links.size()) return 0;
    return std::abs(chestFront(contract, profile, body) - chestFront(contract, profile, reference.links)) > facingLimit ? 2 : 0;
}

float TrackingClipGetUp3D::chestFront(const TrackingGetUpContract3D& contract, const RagdollProfile3D& profile,
    std::span<const PhysicsBodyState3D> body) {
    const std::uint32_t chest = contract.chestLink();
    if (chest >= body.size()) return 0.0f;
    return (body[chest].orientation * profile.links[chest].modelOrientation.conjugate()).normalized().rotate({ 1.0f, 0.0f, 0.0f }).z;
}

} // namespace MatterEngine
