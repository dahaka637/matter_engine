#include "Engine/Character/DefaultLocomotion3DInternal.hpp"

namespace MatterEngine {

void DefaultLocomotion3D::stepTravel(Tick& t) {
    const DefaultLocomotionInput3D& input = t.input;
    const DefaultLocomotionSettings3D& s = m_settings;
    const float& dt = t.dt;
    const float& viewYaw = t.viewYaw;
    // O giro da camera (o carrinho vira o corpo para o lado dele).
    {
        const float rate = m_previousViewValid ? wrapAngle(viewYaw - m_previousViewYaw) / dt : 0.0f;
        m_viewTurnRate += (std::clamp(rate, -12.0f, 12.0f) - m_viewTurnRate) * (1.0f - std::exp(-dt / 0.12f));
        m_previousViewYaw = viewYaw;
        m_previousViewValid = true;
    }

    // A capsula: velocidade, aceleracao (filtrada) e o giro do caminho.
    t.velocity = planar(input.velocityWorld);
    const Vec3& velocity = t.velocity;
    t.speed = velocity.length();
    const float& speed = t.speed;
    if (!m_started) {
        m_previousVelocity = velocity;
        m_yaw = viewYaw;
    }
    {
        const Vec3 raw = (velocity - m_previousVelocity) * (1.0f / dt);
        m_acceleration += (raw - m_acceleration) * (1.0f - std::exp(-dt / std::max(0.005f, s.accelerationFilterSeconds)));
        m_previousVelocity = velocity;
        if (speed > 0.5f) {
            const float pathYaw = std::atan2(velocity.y, velocity.x);
            const float rate = m_pathYawValid ? wrapAngle(pathYaw - m_previousPathYaw) / dt : 0.0f;
            const float blend = 1.0f - std::exp(-dt / std::max(0.005f, s.pathTurnFilterSeconds));
            m_pathTurnRate += (std::clamp(rate, -8.0f, 8.0f) - m_pathTurnRate) * blend;
            m_previousPathYaw = pathYaw;
            m_pathYawValid = true;
        } else {
            m_pathTurnRate *= std::exp(-dt / 0.1f);
            m_pathYawValid = false;
        }
    }
    m_started = true;

}

} // namespace MatterEngine
