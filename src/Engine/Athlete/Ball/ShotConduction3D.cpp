#include "Engine/Athlete/Ball/ShotConduction3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {

ShotConduction3D startShotConduction3D(float lookYaw, float bar, const BallOutcomeSettings3D& force) {
    constexpr float DegreesToRadians = 3.14159265f / 180.0f;
    const float b = std::clamp(bar, 0.0f, 1.0f);
    return { lookYaw, std::max(0.0f, force.conductionDegrees) * (1.0f - b * b) * DegreesToRadians, 0.0f };
}

ShotConductionStep3D stepShotConduction3D(ShotConduction3D& conduction, Vec3 velocity, Vec3 spin, float lookYaw,
    const BallOutcomeSettings3D& force) {
    const float wanted = std::clamp(std::remainder(lookYaw - conduction.lookYaw, 6.28318531f), -conduction.limit,
        conduction.limit);
    const float step = wanted - conduction.turned;
    conduction.turned = wanted;
    const float c = std::cos(step), s = std::sin(step);
    // (Giro +z leva para a esquerda: o mesmo sentido do rumo girado.)
    return { { velocity.x * c - velocity.y * s, velocity.x * s + velocity.y * c, velocity.z },
        spin + Vec3 { 0.0f, 0.0f, force.conductionSpin * step } };
}

} // namespace MatterEngine
