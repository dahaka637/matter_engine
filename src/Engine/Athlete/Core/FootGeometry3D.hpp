#pragma once

#include "Engine/Athlete/Core/Kinematics3D.hpp"
#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <array>
#include <cstddef>

namespace MatterEngine {

// A sola de cada pe no referencial do link do pe (rework de 10/10/2026,
// Etapa 3): o calcanhar e a base dos dedos (toeRollMeters atras da frente da
// caixa - os dedos dobram ali), na face de baixo da caixa com o pe plano (a
// pose de ligacao do perfil tem os pes planos e o modelo olha para +x). O pe
// apoiado rola nesses dois pontos.
struct AthleteFootGeometry3D {
    bool valid = false;
    std::array<std::size_t, 2> link {};
    std::array<Vec3, 2> heel {};
    std::array<Vec3, 2> toeBase {};
    // A orientacao do link do pe plano, olhando para +x (a de ligacao).
    std::array<Quaternion, 2> flat {};
};

[[nodiscard]] AthleteFootGeometry3D resolveAthleteFootGeometry3D(const RagdollProfile3D& profile,
    const AthleteLegs3D& legs, float toeRollMeters);

// Para onde o pe aponta no plano (rad) e a rotacao em volta da vertical.
[[nodiscard]] float athleteFootYaw3D(const AthleteFootGeometry3D& geometry, int side, Quaternion orientation);
// O rumo do pe pelo eixo de lado dele (rad): firme com o pe muito inclinado -
// o calcanhar chutando para tras, a ponta para baixo -, onde o rumo da ponta
// gira a toa.
[[nodiscard]] float athleteFootHeading3D(const AthleteFootGeometry3D& geometry, int side, Quaternion orientation);
[[nodiscard]] Quaternion athleteYawRotation3D(float yawRadians);
[[nodiscard]] float athleteWrapAngle3D(float radians);

} // namespace MatterEngine
