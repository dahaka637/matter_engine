#pragma once

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <array>
#include <cstddef>
#include <vector>

namespace MatterEngine {

// Cinematica do corpo para o controle dos atletas: coordenadas de junta (o
// vetor de rotacao no referencial de cada junta, como os clipes e os
// motores), pose direta e o IK analitico da perna. Portado de
// RagdollPoseKinematics3D e BallGesturePose3D (rework de 10/10/2026); os
// antigos saem com o legado e fica esta copia.

[[nodiscard]] Quaternion athleteJointRotation3D(Vec3 coordinates);
// A variacao da rotacao para uma variacao das coordenadas.
[[nodiscard]] Vec3 athleteJointJacobian3D(Vec3 coordinates, Vec3 velocity);
// A pose de cada link no referencial da raiz e no mundo.
void athleteLocalPose3D(const RagdollProfile3D& profile, const std::vector<Vec3>& coordinates,
    RagdollAnimationPose3D& pose);
void athleteWorldPose3D(const RagdollProfile3D& profile, const std::vector<Vec3>& coordinates, Vec3 rootPosition,
    Quaternion rootOrientation, RagdollAnimationPose3D& pose);
// As coordenadas da junta do link `index` para a rotacao de mundo dele dado o
// pai (dentro dos limites).
[[nodiscard]] Vec3 athleteJointCoordinates3D(const RagdollProfile3D& profile, std::size_t index,
    Quaternion parentWorld, Quaternion childWorld);
// O vetor de rotacao (eixo x angulo) de um quaternion, pelo caminho curto.
[[nodiscard]] Vec3 athleteRotationVector3D(Quaternion q);
// A rotacao que leva a base (x, y, x cross y) de colunas dadas.
[[nodiscard]] Quaternion athleteFromBasis3D(Vec3 x, Vec3 y, Vec3 z);
// IK numerico (minimos quadrados amortecidos): leva um ponto preso ao link
// endIndex a um alvo no referencial da raiz, mexendo ate `depth` juntas.
void athleteSolveChainPoint3D(const RagdollProfile3D& profile, std::vector<Vec3>& coordinates,
    std::size_t endIndex, Vec3 pointInEnd, int depth, Vec3 targetRootLocal, float weight, int iterations = 7);

// As pernas do perfil (coxa, canela, pe; juntas e comprimentos).
struct AthleteLegs3D {
    bool valid = false;
    std::array<std::size_t, 2> thigh {};
    std::array<std::size_t, 2> shin {};
    std::array<std::size_t, 2> foot {};
    // Juntas no referencial do link: quadril na pelve, tornozelo no pe e na
    // canela.
    std::array<Vec3, 2> hipInPelvis {};
    std::array<Vec3, 2> ankleInFoot {};
    std::array<Vec3, 2> ankleInShin {};
    std::array<float, 2> thighLength {};
    std::array<float, 2> shinLength {};
};

[[nodiscard]] AthleteLegs3D resolveAthleteLegs3D(const RagdollProfile3D& profile);

// A perna pelo tornozelo: coxa, canela e pe para o centro da junta do
// tornozelo ir a ankleWorld, o joelho apontar para kneeWorld (direcao) e o pe
// ficar o mais perto de footWorld (o tornozelo tem dois eixos). Devolve onde o
// tornozelo ficou.
Vec3 solveAthleteLeg3D(const RagdollProfile3D& profile, const AthleteLegs3D& legs, int side,
    std::vector<Vec3>& coordinates, Vec3 rootPosition, Quaternion rootOrientation, Vec3 ankleWorld, Vec3 kneeWorld,
    Quaternion footWorld);

// O alcance macio (soft IK), em fracoes do comprimento da perna: ate `start`
// a distancia do quadril ao tornozelo fica; dai ela se aproxima de `end` sem
// nunca passar, numa curva sem quina (o joelho nao estala reto).
[[nodiscard]] float athleteSoftReach3D(float reach, float start, float end);

// Onde a perna esta: tornozelo, para onde o joelho aponta e o pe (mundo).
void athleteLegState3D(const RagdollProfile3D& profile, const AthleteLegs3D& legs, int side,
    const std::vector<Vec3>& coordinates, Vec3 rootPosition, Quaternion rootOrientation, Vec3& ankle, Vec3& knee,
    Quaternion& foot);

} // namespace MatterEngine
