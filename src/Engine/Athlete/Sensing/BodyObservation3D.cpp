#include "Engine/Athlete/Sensing/BodyObservation3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {

void BodyObserver3D::prepare(const RagdollProfile3D& profile) {
    m_mass.assign(profile.links.size(), 0.0f);
    m_totalMass = 0.0f;
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        m_mass[i] = std::max(0.0f, profile.links[i].massFraction) * profile.totalMassKg;
        m_totalMass += m_mass[i];
        const std::string& id = profile.links[i].id;
        if (id == "LeftFoot") m_feet[0] = static_cast<std::uint32_t>(i);
        if (id == "RightFoot") m_feet[1] = static_cast<std::uint32_t>(i);
        if (id == "Chest") m_chest = static_cast<std::uint32_t>(i);
    }
}

BodyObservation3D BodyObserver3D::observe(const RagdollProfile3D& profile, const RagdollState3D& state,
    float deltaTime) const {
    BodyObservation3D observation;
    if (state.links.empty() || state.links.size() != profile.links.size() || m_totalMass <= 0.0f) return observation;
    observation.valid = true;
    observation.massKilograms = m_totalMass;
    for (std::size_t i = 0; i < state.links.size(); ++i) {
        const PhysicsBodyState3D& link = state.links[i];
        const Vec3 center = link.position + link.orientation.rotate(profile.links[i].centerOfMassLocal);
        observation.centerOfMass += center * m_mass[i];
        observation.centerOfMassVelocity += link.linearVelocity * m_mass[i];
    }
    observation.centerOfMass = observation.centerOfMass * (1.0f / m_totalMass);
    observation.centerOfMassVelocity = observation.centerOfMassVelocity * (1.0f / m_totalMass);
    const PhysicsBodyState3D& pelvis = state.links.front();
    observation.pelvisPosition = pelvis.position;
    observation.pelvisOrientation = pelvis.orientation;
    observation.pelvisVelocity = pelvis.linearVelocity;
    observation.pelvisAngularVelocity = pelvis.angularVelocity;
    const auto up = [&](std::size_t link) {
        return (state.links[link].orientation * profile.links[link].modelOrientation.conjugate())
            .rotate({ 0.0f, 0.0f, 1.0f });
    };
    observation.pelvisUp = up(0);
    observation.chestUp = up(m_chest);
    const float step = std::max(deltaTime, 1e-4f);
    for (int side = 0; side < 2; ++side) {
        FootObservation3D& foot = observation.feet[static_cast<std::size_t>(side)];
        const std::uint32_t index = m_feet[static_cast<std::size_t>(side)];
        if (index == 0 || index >= state.links.size()) continue;
        foot.valid = true;
        foot.link = index;
        const RagdollLinkDefinition3D& definition = profile.links[index];
        const PhysicsBodyState3D& link = state.links[index];
        const Quaternion collider = (link.orientation * definition.collider.localOrientation).normalized();
        const Vec3 center = link.position + link.orientation.rotate(definition.collider.localPosition);
        const Vec3 half = definition.collider.boxHalfExtents;
        const Vec3 down = collider.conjugate().rotate({ 0.0f, 0.0f, -1.0f });
        const float lowest = std::abs(down.x) * half.x + std::abs(down.y) * half.y + std::abs(down.z) * half.z;
        foot.soleHeight = center.z - lowest;
        foot.soleCenter = { center.x, center.y, foot.soleHeight };
        Vec3 forward = link.orientation.rotate(definition.modelOrientation.conjugate().rotate({ 1.0f, 0.0f, 0.0f }));
        forward.z = 0.0f;
        foot.forward = forward.lengthSquared() > 1e-6f ? forward.normalized() : Vec3 { 1.0f, 0.0f, 0.0f };
    }
    for (const RagdollInteraction3D& contact : state.interactions) {
        const bool groundLike = contact.normalWorld.z > 0.6f;
        int side = -1;
        if (contact.linkIndex == m_feet[0]) side = 0;
        if (contact.linkIndex == m_feet[1]) side = 1;
        if (side >= 0 && groundLike) {
            FootObservation3D& foot = observation.feet[static_cast<std::size_t>(side)];
            foot.supporting = true;
            foot.loadNewtons += contact.normalImpulseNewtonSeconds / step;
            continue;
        }
        // O chao embaixo de outro membro (joelho, mao) tambem nao e pancada.
        if (groundLike && contact.otherMotion == RagdollContactMotion3D::Static) continue;
        observation.externalImpulse += contact.normalImpulseNewtonSeconds;
        if (contact.otherMotion != RagdollContactMotion3D::Static)
            observation.movingImpulse += contact.normalImpulseNewtonSeconds;
        if (contact.normalImpulseNewtonSeconds > observation.strongestExternalImpulse) {
            observation.strongestExternalImpulse = contact.normalImpulseNewtonSeconds;
            observation.strongestExternalLink = contact.linkIndex;
        }
    }
    for (const FootObservation3D& foot : observation.feet)
        if (foot.supporting) ++observation.supportCount;
    return observation;
}

} // namespace MatterEngine
