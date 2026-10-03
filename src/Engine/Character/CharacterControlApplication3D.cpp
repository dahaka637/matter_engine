#include "Engine/Character/CharacterControlApplication3D.hpp"

#include <algorithm>

namespace MatterEngine {

void applyCharacterControl3D(PhysicsScene3D& scene, RagdollHandle3D ragdoll,
    const CharacterLocomotionOutput3D& locomotion,
    const AdaptivePhysicalOutput3D* adaptive, bool down) {
    // Torque de antecipacao do controle do corpo inteiro (o peso pelas
    // pernas, etapa E4) somado aos alvos das juntas.
    const bool jointFeedforward = adaptive != nullptr && !down && adaptive->valid
        && adaptive->jointFeedforwardActive;
    if (jointFeedforward) {
        std::vector<RagdollDriveTarget3D> targets = locomotion.driveTargets;
        for (RagdollDriveTarget3D& target : targets) {
            if (target.linkIndex >= adaptive->jointFeedforward.size()) continue;
            target.feedforwardTorqueNewtonMeters +=
                adaptive->jointFeedforward[target.linkIndex][static_cast<std::size_t>(target.axis)];
        }
        scene.setRagdollActiveDriveTargets(ragdoll, targets, locomotion.gravityCompensationEnabled);
    } else {
        scene.setRagdollActiveDriveTargets(ragdoll, locomotion.driveTargets,
            locomotion.gravityCompensationEnabled);
    }
    scene.setRagdollAnimationConstraint(ragdoll, locomotion.guide);
    if (adaptive != nullptr && !down) {
        if (adaptive->valid) {
            scene.applyRagdollControlRootForce(ragdoll, adaptive->rootForceWorld,
                adaptive->rootTorqueWorld);
            for (std::size_t side = 0; side < 2; ++side) {
                if (adaptive->footLinkIndex[side] == RagdollDynamics3D::InvalidIndex) continue;
                scene.applyRagdollControlLinkForce(ragdoll, adaptive->footLinkIndex[side],
                    adaptive->footForceWorld[side], adaptive->footTorqueWorld[side]);
            }
            // Mao apoiada num obstaculo: a reacao da postura (empurra).
            for (std::size_t side = 0; side < 2; ++side) {
                if (adaptive->handLinkIndex[side] == RagdollDynamics3D::InvalidIndex) continue;
                scene.applyRagdollControlLinkForce(ragdoll, adaptive->handLinkIndex[side],
                    {}, adaptive->handTorqueWorld[side]);
            }
        }
    } else if (locomotion.rootControlTorqueWorld.lengthSquared() > 0.000001f
        || locomotion.rootControlForceWorld.lengthSquared() > 0.000001f) {
        scene.applyRagdollControlRootForce(ragdoll, locomotion.rootControlForceWorld,
            locomotion.rootControlTorqueWorld);
    }
    // Levantar: a ajuda no peito...
    if (locomotion.chestAssistLink != RagdollDynamics3D::InvalidIndex
        && (locomotion.chestAssistForceWorld.lengthSquared() > 0.000001f
            || locomotion.chestAssistTorqueWorld.lengthSquared() > 0.000001f)) {
        scene.applyRagdollControlLinkForce(ragdoll, locomotion.chestAssistLink,
            locomotion.chestAssistForceWorld, locomotion.chestAssistTorqueWorld);
    }
    // ...e a reacao do endireitar nos segmentos apoiados (par interno).
    const std::size_t reactions = std::min(locomotion.assistReactionLinks.size(),
        locomotion.assistReactionTorquesWorld.size());
    for (std::size_t i = 0; i < reactions; ++i) {
        scene.applyRagdollControlLinkForce(ragdoll, locomotion.assistReactionLinks[i],
            {}, locomotion.assistReactionTorquesWorld[i]);
    }
}

Vec3 navigationFollowVelocity3D(Vec3 body, Vec3 reference, float slack) {
    Vec3 away = body - reference;
    away.z = 0.0f;
    const float distance = away.length();
    if (distance <= slack || distance <= 0.0001f) return {};
    return away * (std::min(6.0f, (distance - slack) * 12.0f) / distance);
}

} // namespace MatterEngine
