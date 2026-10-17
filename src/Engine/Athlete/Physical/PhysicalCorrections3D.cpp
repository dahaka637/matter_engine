#include "Engine/Athlete/Physical/PhysicalCorrections3D.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace MatterEngine {

namespace {

constexpr float Pi = 3.14159265359f;

float finiteOr(float value, float fallback) { return std::isfinite(value) ? value : fallback; }

bool finiteVec(Vec3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

// Um vetor do rumo no mundo, so no plano.
Vec3 planarWorld(const PhysicalCorrections3D& corrections, Vec3 local) {
    if (!finiteVec(local)) return {};
    return corrections.frame.rotate({ local.x, local.y, 0.0f });
}

const FootCorrection3D* footOf(const PhysicalCorrections3D* corrections, std::size_t side) {
    if (corrections == nullptr || !corrections->footwork || side >= corrections->feet.size()) return nullptr;
    return &corrections->feet[side];
}

} // namespace

float locomotionFrameYaw3D(const RagdollProfile3D& profile, Quaternion pelvisOrientation) {
    if (profile.links.empty()) return 0.0f;
    const Quaternion body = (pelvisOrientation * profile.links.front().modelOrientation.conjugate()).normalized();
    const Vec3 forward = body.rotate({ 1.0f, 0.0f, 0.0f });
    const Vec3 left = body.rotate({ 0.0f, 1.0f, 0.0f });
    // (De pe as duas concordam; tombando para a frente ou para tras a frente
    // fica vertical e o lado segura o rumo; de lado, o contrario. A
    // singularidade fica de cabeca para baixo, onde o rumo vira 0.)
    const float fx = forward.x, fy = forward.y;
    const float lx = left.y, ly = -left.x;
    const float fw = std::hypot(fx, fy), lw = std::hypot(lx, ly);
    const float x = fx * fw + lx * lw, y = fy * fw + ly * lw;
    if (!std::isfinite(x) || !std::isfinite(y) || x * x + y * y < 1e-12f) return 0.0f;
    return std::atan2(y, x);
}

int physicalGainGroup3D(const std::string& linkId) {
    const auto has = [&](const char* part) { return linkId.find(part) != std::string::npos; };
    if (has("Hand") || has("Arm") || has("arm")) return 2;
    if (has("Thigh") || has("Shin") || has("Foot") || has("Toe")) return 0;
    return 1;
}

Vec3 correctedFootGoal3D(Vec3 goal, bool planted, bool plannerOwned, float swingProgress,
    const PhysicalCorrections3D* corrections, std::size_t side) {
    const FootCorrection3D* foot = footOf(corrections, side);
    if (foot == nullptr || planted || !(swingProgress >= 0.0f)) return goal;
    const float progress = std::min(swingProgress, 1.0f);
    if (!plannerOwned) {
        // (A entrada do pouso: inteira a 35% do balanco.)
        const float x = std::clamp(progress / 0.35f, 0.0f, 1.0f);
        goal += planarWorld(*corrections, foot->touchdownOffset) * (x * x * (3.0f - 2.0f * x));
    }
    goal.z += finiteOr(foot->swingHeight, 0.0f) * std::sin(Pi * progress);
    return goal;
}

Vec3 plannerLandingOffset3D(const PhysicalCorrections3D* corrections, std::size_t side) {
    const FootCorrection3D* foot = footOf(corrections, side);
    return foot == nullptr ? Vec3 {} : planarWorld(*corrections, foot->touchdownOffset);
}

float cadenceFactor3D(const PhysicalCorrections3D* corrections) {
    if (corrections == nullptr || !corrections->footwork) return 1.0f;
    return std::exp2(std::clamp(finiteOr(corrections->cadenceOctaves, 0.0f), -1.0f, 1.0f));
}

float strideFactor3D(const PhysicalCorrections3D* corrections) {
    if (corrections == nullptr || !corrections->footwork) return 1.0f;
    return std::exp2(std::clamp(finiteOr(corrections->strideOctaves, 0.0f), -1.0f, 1.0f));
}

float legPhaseLead3D(const PhysicalCorrections3D* corrections, std::size_t side, bool waitingTouchdown) {
    const FootCorrection3D* foot = footOf(corrections, side);
    if (foot == nullptr) return 0.0f;
    const float lead = std::clamp(finiteOr(foot->phaseLead, 0.0f), -0.25f, 0.25f);
    return waitingTouchdown ? std::max(0.0f, lead) : lead;
}

float recoveryUrgency3D(const PhysicalCorrections3D* corrections, std::size_t side) {
    const FootCorrection3D* foot = footOf(corrections, side);
    return foot == nullptr ? 0.0f : std::clamp(finiteOr(foot->recoveryUrgency, 0.0f), 0.0f, 1.0f);
}

float correctedForwardLean3D(float lean, const PhysicalCorrections3D* corrections) {
    if (corrections == nullptr || !corrections->balance) return lean;
    return lean + finiteOr(corrections->forwardLean, 0.0f);
}

float correctedLateralLean3D(float lean, const PhysicalCorrections3D* corrections) {
    if (corrections == nullptr || !corrections->balance) return lean;
    return lean + finiteOr(corrections->lateralLean, 0.0f);
}

Vec3 comOffsetWorld3D(const PhysicalCorrections3D* corrections) {
    if (corrections == nullptr || !corrections->balance) return {};
    return planarWorld(*corrections, corrections->comOffset);
}

float pelvisHeightOffset3D(const PhysicalCorrections3D* corrections) {
    if (corrections == nullptr || !corrections->balance) return 0.0f;
    return finiteOr(corrections->pelvisHeight, 0.0f);
}

Vec3 contactAccelerationWorld3D(const PhysicalCorrections3D* corrections) {
    if (corrections == nullptr || !corrections->balance) return {};
    return planarWorld(*corrections, corrections->contactAcceleration);
}

void correctDriveTargets3D(std::vector<RagdollDriveTarget3D>& targets, const RagdollProfile3D& profile,
    const PhysicalCorrections3D* corrections) {
    if (corrections == nullptr || (!corrections->pose && !corrections->gain)) return;
    for (RagdollDriveTarget3D& target : targets) {
        if (corrections->pose) {
            const std::size_t key = static_cast<std::size_t>(target.linkIndex) * 3
                + static_cast<std::size_t>(target.axis);
            if (key < corrections->jointResiduals.size())
                target.positionRadians += finiteOr(corrections->jointResiduals[key], 0.0f);
        }
        if (corrections->gain && target.linkIndex < profile.links.size()) {
            const int group = physicalGainGroup3D(profile.links[target.linkIndex].id);
            // (Uma faixa plausivel: de 1/4 a 4x a rigidez calibrada; sem ela,
            // uma escala extrema virava infinito e 0 x infinito, NaN.)
            const float scale = std::clamp(finiteOr(corrections->gainScale[static_cast<std::size_t>(group)], 1.0f),
                0.25f, 4.0f);
            target.stiffnessScale *= scale;
            target.dampingScale *= std::sqrt(scale);
            target.maximumTorqueScale *= scale;
        }
    }
}

} // namespace MatterEngine
