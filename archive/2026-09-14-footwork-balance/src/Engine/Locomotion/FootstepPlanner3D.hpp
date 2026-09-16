#pragma once

#include "Engine/Locomotion/FootworkTypes3D.hpp"

namespace MatterEngine {

struct FootstepPlanningContext3D {
    FootSide3D side = FootSide3D::Left;
    FootPose3D desiredHome;
    FootPose3D supportFoot;
    Vec3 desiredVelocity;
    float rootYawRadians = 0.0f;
    // Passadas normais nunca cruzam a linha sagital. Uma recuperação de
    // queda lateral, porém, pode exigir o crossover humano com o pé que já
    // está descarregado.
    bool allowCrossing = false;
};

[[nodiscard]] FootworkTerrainPatch3D sampleFootworkTerrainPatch3D(
    const FootworkTerrainProbe3D& terrain, Vec3 candidatePosition,
    float candidateYawRadians, const FootworkConfig3D& config);

[[nodiscard]] FootstepPlan3D planFootstep3D(
    const FootstepPlanningContext3D& context,
    const FootworkTerrainProbe3D& terrain,
    const FootworkConfig3D& config);

} // namespace MatterEngine
