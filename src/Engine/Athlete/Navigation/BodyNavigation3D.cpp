#include "Engine/Athlete/Navigation/BodyNavigation3D.hpp"

#include "Engine/Athlete/Legacy/LegacyAthleteState3D.hpp"

#include <algorithm>

namespace MatterEngine {

BodyNavigationPoint3D bodyNavigationPoint3D(const PhysicsScene3D& scene, const RagdollProfile3D& profile, const RagdollState3D& body) {
    BodyNavigationPoint3D point;
    if (body.links.empty()) return point;
    const Vec3 pelvis = body.links.front().position;
    // (O chao estatico sob a pelve - nunca o proprio corpo nem a capsula.)
    const GroundProbeResult3D ground = scene.probeGround(pelvis + Vec3 { 0.0f, 0.0f, 0.3f }, 0.05f, 0.0f, 3.0f, 60.0f);
    point.feet = { pelvis.x, pelvis.y, ground.hasSurface ? ground.pointWorld.z : pelvis.z - profile.standingRootHeightMeters };
    Vec3 sum {};
    float mass = 0.0f;
    for (std::size_t i = 0; i < profile.links.size() && i < body.links.size(); ++i) {
        sum += body.links[i].linearVelocity * profile.links[i].massFraction;
        mass += profile.links[i].massFraction;
    }
    const Vec3 velocity = mass > 1.0e-6f ? sum * (1.0f / mass) : body.links.front().linearVelocity;
    point.velocity = { velocity.x, velocity.y, 0.0f };
    return point;
}

void placeBodyNavigation3D(PhysicsScene3D& scene, const CharacterMotorSettings3D& avatar, bool controlled,
    LegacyAthleteState3D& legacy, const BodyNavigationPoint3D& point) {
    // (A do jogador levada sem ser recriada: recriada, ela ficava "no ar" ate o proximo movimento e o padrao fazia a pose de pouso.)
    if (controlled && scene.hasCharacter()) scene.teleportCharacter(point.feet, point.velocity);
    legacy.virtualCapsule.position = { point.feet.x, point.feet.y, point.feet.z + 0.5f * avatar.standingHeight };
    legacy.virtualCapsule.velocity = point.velocity;
    legacy.virtualCapsule.grounded = true;
    legacy.virtualCapsule.crouched = false;
    legacy.virtualCapsuleValid = true;
    legacy.virtualFollowVelocity = {};
    legacy.virtualPushVelocity = {};
    // (A ancora do fisico recomeca da pelve no proximo tick dele - a do piloto com o embalo da capsula, que e o do corpo.)
    legacy.navigationAnchorValid = false;
    legacy.navigationAnchorVelocity = {};
}

} // namespace MatterEngine
