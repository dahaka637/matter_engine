#pragma once

#include "Engine/Physics/RagdollProfile3D.hpp"

#include <span>
#include <string>
#include <utility>
#include <vector>

namespace MatterEngine {

enum class RagdollColliderWireframeSide3D {
    Center,
    Left,
    Right,
};

struct RagdollColliderWireframeLink3D {
    std::string linkId;
    RagdollColliderShape3D shape = RagdollColliderShape3D::Capsule;
    RagdollColliderWireframeSide3D side = RagdollColliderWireframeSide3D::Center;
    std::vector<std::pair<Vec3, Vec3>> segments;
    Vec3 linkOriginModel;
    Vec3 colliderCenterModel;
    Vec3 centerOfMassModel;
    Vec3 inboundJointModel;
    bool hasInboundJoint = false;
};

struct RagdollColliderWireframe3D {
    std::string profileId;
    std::vector<RagdollColliderWireframeLink3D> links;
};

// Converte as shapes do perfil em linhas no espaco de modelo. O resultado e
// neutro de renderer e pode alimentar tanto ferramentas headless quanto a
// inspecao visual. A capsula conica usa os aneis de tangencia das esferas
// quando eles existem, em vez de ligar seus equadores por uma aproximacao
// incorreta.
[[nodiscard]] RagdollColliderWireframe3D buildRagdollColliderWireframe3D(
    const RagdollProfile3D& profile, int radialSegments = 16, int capSegments = 4);

// Leva o wireframe de repouso para uma pose cinematica dos mesmos links. A
// transformacao acontece no frame de cada link, portanto tambem preserva os
// offsets locais de collider, COM e junta. Quando inPlace e verdadeiro, a
// raiz recebe a mesma remocao de locomocao horizontal usada pelo GameRig.
[[nodiscard]] RagdollColliderWireframe3D poseRagdollColliderWireframe3D(const RagdollProfile3D& profile,
    const RagdollColliderWireframe3D& restWireframe, std::span<const Vec3> linkPositions,
    std::span<const Quaternion> linkOrientations, bool inPlace = true);

} // namespace MatterEngine
