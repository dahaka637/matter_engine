#include "Engine/Athlete/Physical/PhysicalCorrections3D.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace MatterEngine {

namespace {

float authorityOf(float value) { return std::isfinite(value) ? std::clamp(value, 0.0f, 1.0f) : 0.0f; }

float finiteOr(float value, float fallback) { return std::isfinite(value) ? value : fallback; }

} // namespace

bool PhysicalCorrections3D::active() const {
    return authorityOf(footworkAuthority) > 0.0f || authorityOf(balanceAuthority) > 0.0f
        || authorityOf(poseAuthority) > 0.0f || authorityOf(gainAuthority) > 0.0f;
}

int physicalGainGroup3D(const std::string& linkId) {
    const auto has = [&](const char* part) { return linkId.find(part) != std::string::npos; };
    if (has("Hand") || has("Arm") || has("arm")) return 2;
    if (has("Thigh") || has("Shin") || has("Foot") || has("Toe")) return 0;
    return 1;
}

Vec3 correctedFootGoal3D(Vec3 goal, bool planted, Quaternion heading, const PhysicalCorrections3D* corrections,
    std::size_t side) {
    if (corrections == nullptr || planted || side >= corrections->feet.size()) return goal;
    const float authority = authorityOf(corrections->footworkAuthority);
    if (authority <= 0.0f) return goal;
    const Vec3 offset = corrections->feet[side].touchdownOffset;
    if (!std::isfinite(offset.x) || !std::isfinite(offset.y) || !std::isfinite(offset.z)) return goal;
    return goal + heading.rotate(offset) * authority;
}

float correctedForwardLean3D(float lean, const PhysicalCorrections3D* corrections) {
    if (corrections == nullptr) return lean;
    const float authority = authorityOf(corrections->balanceAuthority);
    return authority > 0.0f ? lean + finiteOr(corrections->forwardLean, 0.0f) * authority : lean;
}

float correctedLateralLean3D(float lean, const PhysicalCorrections3D* corrections) {
    if (corrections == nullptr) return lean;
    const float authority = authorityOf(corrections->balanceAuthority);
    return authority > 0.0f ? lean + finiteOr(corrections->lateralLean, 0.0f) * authority : lean;
}

void correctDriveTargets3D(std::vector<RagdollDriveTarget3D>& targets, const RagdollProfile3D& profile,
    const PhysicalCorrections3D* corrections) {
    if (corrections == nullptr) return;
    const float pose = authorityOf(corrections->poseAuthority);
    const float gain = authorityOf(corrections->gainAuthority);
    if (pose <= 0.0f && gain <= 0.0f) return;
    for (RagdollDriveTarget3D& target : targets) {
        if (pose > 0.0f) {
            const std::size_t key = static_cast<std::size_t>(target.linkIndex) * 3
                + static_cast<std::size_t>(target.axis);
            if (key < corrections->jointResiduals.size())
                target.positionRadians += finiteOr(corrections->jointResiduals[key], 0.0f) * pose;
        }
        if (gain > 0.0f && target.linkIndex < profile.links.size()) {
            const int group = physicalGainGroup3D(profile.links[target.linkIndex].id);
            // (Uma faixa plausivel: ate 4x a rigidez calibrada; sem ela, uma
            // escala extrema virava infinito e 0 x infinito, NaN.)
            const float wanted = std::clamp(finiteOr(corrections->gainScale[static_cast<std::size_t>(group)], 1.0f),
                0.0f, 4.0f);
            const float scale = 1.0f + (wanted - 1.0f) * gain;
            target.stiffnessScale *= scale;
            target.dampingScale *= std::sqrt(scale);
            target.maximumTorqueScale *= scale;
        }
    }
}

} // namespace MatterEngine
