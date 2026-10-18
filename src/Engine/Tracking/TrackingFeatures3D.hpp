#pragma once

#include "Engine/Physics/PhysicsTypes3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <span>
#include <vector>

namespace MatterEngine {

// O referencial de rumo do DReCon (F_kin / F_sim, secao 6.3): o rumo horizontal
// do lider, a gravidade e a origem num centro de massa; sem velocidade.
struct TrackingHeadingFrame3D {
    Vec3 origin;
    float cosYaw = 1.0f;
    float sinYaw = 0.0f;

    [[nodiscard]] static TrackingHeadingFrame3D at(Vec3 origin, float yawRadians);
    // Um ponto do mundo no referencial.
    [[nodiscard]] Vec3 point(Vec3 world) const;
    // Uma direcao do mundo no referencial (so a rotacao).
    [[nodiscard]] Vec3 direction(Vec3 world) const;
};

// A caixa que envolve o colisor de um link, no referencial do link (DReCon
// 6.5: os pontos de superficie nas faces da caixa orientada da geometria; aqui
// a geometria de colisao do perfil - capsula, caixa ou casca).
struct TrackingSurfaceBox3D {
    Vec3 center;
    Quaternion orientation;
    Vec3 halfExtents;
};

[[nodiscard]] std::vector<TrackingSurfaceBox3D> trackingSurfaceBoxes3D(const RagdollProfile3D& profile);

// Os 6 pontos de cada link (os centros das faces da caixa, na ordem dos links)
// no mundo e a velocidade de cada um (links na convencao do backend).
void trackingSurfacePoints3D(const RagdollProfile3D& profile, std::span<const TrackingSurfaceBox3D> boxes,
    std::span<const PhysicsBodyState3D> links, std::vector<Vec3>& positions, std::vector<Vec3>& velocities);

// A velocidade angular relativa da junta (no referencial do filho, a que o
// motor do Jolt recebe) para coordenadas que variam de coordinateVelocity
// perto de `target`, medida a partir da pose medida da junta (a mesma conta do
// modo fisico, CharacterLocomotion3DInternal::jointAngularVelocity).
[[nodiscard]] Vec3 trackingJointAngularVelocity3D(Vec3 target, Vec3 coordinateVelocity, Vec3 measured);

} // namespace MatterEngine
