#pragma once

#include "Engine/Athlete/Core/Kinematics3D.hpp"
#include "Engine/Athlete/Planning/FootstepPlanner3D.hpp"

#include <array>
#include <vector>

namespace MatterEngine {

struct LegSynthesisResult3D {
    // O quanto o tornozelo ficou longe do alvo (m): a perna nao alcancou.
    std::array<float, 2> reachError {};
};

// As pernas da pose do estilo levadas aos pes planejados (Etapa 3, v1): IK
// a partir da raiz do estilo, o joelho para onde o do estilo aponta, girado
// o que o pe girou em relacao ao do estilo. Refaz a pose no mundo. O pe
// livre ja no estilo nao mexe.
LegSynthesisResult3D synthesizeAthleteLegs3D(const RagdollProfile3D& profile, const AthleteLegs3D& legs,
    const AthleteFootGeometry3D& geometry, const std::array<FootTarget3D, 2>& feet, Vec3 rootPosition,
    Quaternion rootOrientation, std::vector<Vec3>& coordinates, RagdollAnimationPose3D& pose);

} // namespace MatterEngine
