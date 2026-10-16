#include "Engine/Athlete/AthleteRuntime3D.hpp"

#include "Engine/Athlete/Planning/MotionPlanner3D.hpp"

#include "Engine/Athlete/Legacy/LegacyAthleteStep3D.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace MatterEngine {

AthleteCommand3D athleteStickCommand3D(const AthleteStickInput3D& stick, const AthleteRuntimeSettings3D& settings,
    const CharacterGaitSpeeds3D& gaitSpeeds, float terrainSpeedLimit) {
    AthleteCommand3D command;
    CharacterIntent3D& intent = command.intent;
    intent.requestedFacingYawRadians = stick.cameraYawRadians;
    intent.effort = stick.sprint ? 1.0f : 0.0f;
    intent.crouch = stick.crouch;
    const Vec3 move { stick.moveWorld.x, stick.moveWorld.y, 0.0f };
    if (move.lengthSquared() > 1e-4f) {
        const Vec3 direction = move.normalized();
        const float travelRelative = std::atan2(direction.y, direction.x) - stick.cameraYawRadians;
        const float wanted = settings.physicalUsesDefaultSpeeds
            ? athleteDefaultTravelSpeed3D(settings, travelRelative, stick.sprint)
            : characterGaitSpeed3D(gaitSpeeds, travelRelative, stick.sprint);
        intent.requestedDirectionWorld = direction;
        const float speed = std::min(wanted * std::min(1.0f, move.length()), terrainSpeedLimit);
        intent.requestedVelocityWorld = direction * speed;
    }
    command.facingYawRadians = stick.cameraYawRadians;
    command.lookPitchRadians = stick.lookPitchRadians;
    command.sprinting = stick.sprint;
    return command;
}

void athleteStart3D(PhysicsScene3D& scene, RagdollHandle3D handle, const RagdollProfile3D& profile,
    const RagdollState3D& body, AthleteState3D& athlete, AthleteStartMode3D mode) {
    LegacyAthleteState3D& legacy = athlete.legacy;
    legacy.guideFacingYawRadians = 0.0f;
    legacy.locomotion.reset(profile, body);
    legacy.adaptivePhysics.reset(profile);
    if (mode == AthleteStartMode3D::Default) {
        legacy.defaultLocomotion.reset(0.0f);
        legacy.guided = true;
        legacy.bodyMode = LegacyBodyMode3D::Animated;
        scene.setRagdollGuided(handle, true);
    }
}

AthleteSnapshot3D athleteSnapshot3D(const AthleteState3D& athlete, const RagdollState3D& body) {
    return AthleteSnapshot3D { athlete, body };
}

void athleteRestore3D(PhysicsScene3D& scene, RagdollHandle3D handle, const AthleteSnapshot3D& snapshot,
    AthleteState3D& athlete, RagdollState3D& body) {
    scene.resetRagdollState(handle, snapshot.body.links);
    athlete = snapshot.athlete;
    // (O guiado e o unico estado da cena que o passo so escreve nas trocas;
    // os alvos, a guia e o atrito ele escreve a cada tick.)
    scene.setRagdollGuided(handle, athlete.legacy.guided);
    body = snapshot.body;
}

float athleteGroundHeight3D(const PhysicsScene3D& scene, const RagdollProfile3D& profile, Vec3 position) {
    PhysicsRayHit3D hit;
    if (scene.raycastStatic({ position + Vec3 { 0, 0, 0.3f }, { 0, 0, -1 } }, 2000, hit) && hit.normal.z > 0.7f)
        return hit.position.z;
    return position.z - profile.standingRootHeightMeters;
}

void AthleteRuntime3D::update(PhysicsScene3D& scene, const AthleteRuntimeAssets3D& assets,
    std::span<AthleteSlot3D> athletes, std::span<AthleteBall3D> balls, const AthleteRuntimeSettings3D& settings,
    const AthleteControlInput3D& control, float deltaTime, AthleteRuntimeOutput3D& output) {
    output = {};
    if (assets.profile == nullptr || assets.clips == nullptr || assets.defaultClips == nullptr) return;
    const RagdollProfile3D& profile = *assets.profile;
    // Dono de cada bola (pedido do usuario, 05/10 noite; BallOwnership3D):
    // quem ela tocou por ultimo (fora o dono, que nao a toca) passa a ser o
    // dono; a 5 m dele por 3 s (ou ele sumido), ela volta a colidir com
    // todos. A interacao do jogador o torna dono no manejo.
    for (AthleteBall3D& ball : balls) {
        if (!scene.contains(ball.handle) || ball.state == nullptr || ball.ownership == nullptr) continue;
        const AthleteSlot3D* owner = nullptr;
        for (const AthleteSlot3D& slot : athletes)
            if (slot.entityId == ball.ownership->owner() && slot.active && slot.state != nullptr
                && !slot.state->links.empty() && scene.contains(slot.handle))
                owner = &slot;
        const Vec3 pelvis = owner != nullptr ? owner->state->links.front().position : Vec3 {};
        ball.ownership->update(*ball.state, owner != nullptr ? &pelvis : nullptr, deltaTime);
        RagdollHandle3D handle {};
        for (const AthleteSlot3D& slot : athletes)
            if (slot.entityId == ball.ownership->owner() && slot.active) handle = slot.handle;
        scene.setInflatedBallOwner(ball.handle, handle);
    }
    // Quem esta dando carrinho (pedido do usuario, 05/10 noite): as pernas
    // dele neste tick. Um boneco com pancada de algo que se move perto delas
    // levou o carrinho - a ajuda dele cai (ImpactAssist3D, ate 20%) e volta.
    std::vector<std::pair<std::uint64_t, Vec3>> tacklerLegs;
    for (const AthleteSlot3D& other : athletes) {
        if (!other.active || other.state == nullptr || other.state->links.empty() || other.athlete == nullptr) continue;
        const LegacyAthleteState3D& legacy = other.athlete->legacy;
        const bool tackling = legacy.defaultTackling
            || legacy.locomotion.telemetry().state == CharacterLocomotionState3D::Tackling;
        if (!tackling) continue;
        for (std::size_t i = 0; i < profile.links.size() && i < other.state->links.size(); ++i) {
            const std::string& id = profile.links[i].id;
            if (id.find("Shin") != std::string::npos || id.find("Foot") != std::string::npos
                || id.find("Thigh") != std::string::npos)
                tacklerLegs.push_back({ other.entityId, other.state->links[i].position });
        }
    }
    AthleteControlInput3D tickControl = control;
    for (AthleteSlot3D& slot : athletes) {
        if (!slot.active || slot.frozen || slot.state == nullptr || slot.athlete == nullptr || slot.state->frozen
            || !scene.contains(slot.handle) || slot.state->links.empty())
            continue;
        stepLegacyAthlete3D(scene, assets, settings, tickControl, slot, balls, tacklerLegs, deltaTime, output);
    }
}

float athleteCapsuleWallImpact3D(Vec3 previousMoved, Vec3 moved, float braking, bool grounded, float rose) {
    const Vec3 before { previousMoved.x, previousMoved.y, 0.0f };
    const float speed = before.length();
    if (!grounded || speed <= 0.5f || rose > 0.01f) return 0.0f;
    const Vec3 now { moved.x, moved.y, 0.0f };
    return std::max(0.0f, dot(before - now, before * (1.0f / speed)) - braking);
}

float athleteDefaultTravelSpeed3D(const AthleteRuntimeSettings3D& settings, float travelRelativeToLookRadians,
    bool sprinting) {
    const float t = settings.defaultTrotSpeed;
    const std::array<float, 5> trot { t, t, t, t, t };
    const std::array<float, 5> sprint { settings.defaultSprintForwardSpeed, settings.defaultSprintForwardSpeed,
        settings.defaultSprintSideSpeed, settings.defaultSprintBackwardSpeed, settings.defaultSprintBackwardSpeed };
    return athleteTravelSpeed3D(trot, sprint, travelRelativeToLookRadians, sprinting);
}

} // namespace MatterEngine
