#include "Engine/Control/ImpactAssist3D.hpp"

#include <algorithm>

namespace MatterEngine {

void ImpactAssist3D::reset() {
    m_floor = 1.0f;
    m_level = 1.0f;
    m_elapsed = 0.0f;
    m_seconds = 0.0f;
}

void ImpactAssist3D::update(float impulse, bool tackled, float tackleImpulse, float deltaTime) {
    const ImpactAssistSettings3D& s = m_settings;
    float drop = 0.0f;
    if (tackled) {
        const float x = std::clamp((tackleImpulse - s.tackleMinimumImpulse)
            / std::max(0.1f, s.tackleFullImpulse - s.tackleMinimumImpulse), 0.0f, 1.0f);
        drop = s.tackleBaseDrop + (s.tackleMaximumDrop - s.tackleBaseDrop) * x;
    }
    float common = 0.0f;
    if (s.kneeImpulse > s.minimumImpulse && s.fullImpulse > s.kneeImpulse) {
        if (impulse <= s.kneeImpulse) {
            common = s.kneeDrop * std::clamp((impulse - s.minimumImpulse)
                / std::max(0.1f, s.kneeImpulse - s.minimumImpulse), 0.0f, 1.0f);
        } else {
            common = s.kneeDrop + (s.maximumDrop - s.kneeDrop) * std::clamp((impulse - s.kneeImpulse)
                / std::max(0.1f, s.fullImpulse - s.kneeImpulse), 0.0f, 1.0f);
        }
    } else {
        common = s.maximumDrop * std::clamp((impulse - s.minimumImpulse)
            / std::max(0.1f, s.fullImpulse - s.minimumImpulse), 0.0f, 1.0f);
    }
    drop = std::max(drop, common);
    const float floor = 1.0f - drop;
    if (drop > 0.02f && floor < m_level - 0.01f) {
        // Um choque mais forte que o que ainda resta: recomeca dali.
        m_floor = floor;
        m_level = floor;
        m_elapsed = 0.0f;
        m_seconds = s.recoverBaseSeconds + s.recoverSecondsPerDrop * drop;
        return;
    }
    if (m_level >= 1.0f) return;
    m_elapsed += deltaTime;
    const float t = std::clamp(m_elapsed / std::max(0.05f, m_seconds), 0.0f, 1.0f);
    m_level = m_floor + (1.0f - m_floor) * t * t * (3.0f - 2.0f * t);
    if (t >= 1.0f) m_level = 1.0f;
}

float impactImpulse3D(const RagdollState3D& state) {
    float impulse = 0.0f;
    for (const RagdollInteraction3D& contact : state.interactions)
        if (contact.normalWorld.z <= 0.62f) impulse += contact.normalImpulseNewtonSeconds;
    return impulse;
}

} // namespace MatterEngine
