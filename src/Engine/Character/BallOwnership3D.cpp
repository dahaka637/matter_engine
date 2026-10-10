#include "Engine/Character/BallOwnership3D.hpp"

#include <algorithm>

namespace MatterEngine {

void BallOwnership3D::interact(std::uint64_t entityId) {
    if (entityId == 0) return;
    if (entityId != m_owner) m_farSeconds = 0.0f;
    m_owner = entityId;
}

void BallOwnership3D::update(const InflatedBallState3D& ball, const Vec3* ownerPelvis, float deltaTime) {
    // (O dono nao gera contato com ela: so os outros bonecos aparecem aqui.)
    std::uint64_t toucher = 0;
    float strongest = -1.0f;
    const std::uint32_t count = std::min(ball.contactCount, InflatedBallState3D::MaximumContacts);
    for (std::uint32_t index = 0; index < count; ++index) {
        const InflatedBallContact3D& contact = ball.contacts[index];
        if (!contact.otherRagdoll || contact.otherEntityId == 0 || contact.otherEntityId == m_owner) continue;
        if (contact.normalForceNewtons > strongest) {
            strongest = contact.normalForceNewtons;
            toucher = contact.otherEntityId;
        }
    }
    if (toucher != 0) {
        m_owner = toucher;
        m_farSeconds = 0.0f;
        return;
    }
    if (m_owner == 0) return;
    if (ownerPelvis == nullptr) {
        release();
        return;
    }
    Vec3 apart = *ownerPelvis - ball.position;
    apart.z = 0.0f;
    m_farSeconds = apart.length() >= m_settings.releaseDistanceMeters ? m_farSeconds + deltaTime : 0.0f;
    if (m_farSeconds >= m_settings.releaseSeconds) release();
}

} // namespace MatterEngine
