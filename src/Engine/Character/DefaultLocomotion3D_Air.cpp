#include "Engine/Character/DefaultLocomotion3DInternal.hpp"

namespace MatterEngine {

void DefaultLocomotion3D::stepAir(Tick& t) {
    const CharacterLocomotionAnimations3D& animations = t.animations;
    const DefaultLocomotionInput3D& input = t.input;
    const DefaultLocomotionSettings3D& s = m_settings;
    const float& dt = t.dt;
    const Vec3& velocity = t.velocity;
    const float& speed = t.speed;
    const Vec3& forward = t.forward;
    const Vec3& left = t.left;
    // No ar: o pulo pela fase do voo (da saida ao pouso previsto).
    if (!input.grounded) {
        if (m_airTime <= 0.0f) {
            m_takeoffSpeed = std::max(0.0f, input.velocityWorld.z);
            const float along = dot(velocity, forward), aside = dot(velocity, left);
            const AnimationClip3D* clip = animations.jumpStanding;
            if (speed > s.travelJumpSpeed) {
                if (std::abs(along) >= std::abs(aside))
                    clip = along >= 0.0f ? animations.jumpForward : animations.jumpBackward;
                else
                    clip = aside >= 0.0f ? animations.jumpLeft : animations.jumpRight;
            }
            m_airClip = clip != nullptr ? clip : animations.jumpStanding;
        }
        m_airTime += dt;
        const float flight = std::max(0.3f, 2.0f * m_takeoffSpeed / Gravity);
        // (Caindo de um degrau, sem pulo: o meio do voo.)
        m_airPhase = m_takeoffSpeed > 0.5f ? std::clamp(m_airTime / flight, 0.0f, 0.97f) : 0.5f;
        m_fallSpeed = std::max(0.0f, -input.velocityWorld.z);
    } else {
        // Toque: a aterrissagem pela velocidade da queda (um degrau ou um
        // tropeco de um tick no ar nao conta).
        if (m_airTime > 0.12f && s.landingAbsorb) {
            const float span = std::max(0.1f, s.landingSpeedFull - s.landingSpeedMinimum);
            m_landIntensity = std::clamp((m_fallSpeed - s.landingSpeedMinimum) / span, 0.0f, 1.0f);
            if (m_landIntensity > 0.0f) {
                m_landTime = 0.0f;
                m_landDuration = s.landingSecondsMinimum
                    + (s.landingSecondsMaximum - s.landingSecondsMinimum) * m_landIntensity;
                m_landFade = 1.0f;
                m_landPrepDrop = s.landingPrep ? m_landingPrep * s.landingPelvisDrop : 0.0f;
                m_landLeadSide = -1;
                if (m_feet[0] != 0 && m_feet[1] != 0 && m_output.pose.linkPositions.size() > std::max(m_feet[0], m_feet[1]))
                    m_landLeadSide = m_output.pose.linkPositions[m_feet[0]].z <= m_output.pose.linkPositions[m_feet[1]].z ? 0 : 1;
            }
        }
        m_airTime = 0.0f;
    }
    // A aterrissagem: a pelve, a coluna e a cabeca pela forma da referencia
    // (desce ate landingCompressShare do tempo e volta suave); no ar de novo
    // ela sai em ~0,05 s.
    t.landTrunk = 0.0f;
    t.landUpper = 0.0f;
    t.landHead = 0.0f;
    float& landTrunk = t.landTrunk;
    float& landUpper = t.landUpper;
    float& landHead = t.landHead;
    m_landingSink = 0.0f;
    m_landPull = 0.0f;
    m_landPrepCarry = 0.0f;
    if (m_landTime >= 0.0f) {
        m_landTime += dt;
        if (!input.grounded) m_landFade *= std::exp(-dt / 0.05f);
        const float compress = std::clamp(s.landingCompressShare, 0.05f, 0.9f);
        const auto shape = [&](float lag) {
            const float u = (m_landTime - lag) / std::max(0.05f, m_landDuration);
            if (u <= 0.0f || u >= 1.0f) return 0.0f;
            if (u < compress) return std::sin(0.5f * Pi * u / compress);
            return 0.5f * (1.0f + std::cos(Pi * (u - compress) / (1.0f - compress)));
        };
        const float running = 1.0f - (1.0f - s.landingRunningShare) * smoothStep((speed - 2.0f) / 3.0f);
        const float amount = m_landIntensity * m_landFade * running;
        m_landingSink = s.landingDepthMaximum * amount * shape(0.0f);
        m_landPrepCarry = m_landPrepDrop * (1.0f - smoothStep(m_landTime / std::max(0.05f, compress * m_landDuration)));
        // Os pes ao chao no toque (em 0,05 s; o preparo os deixava no ar ate
        // ~0,35 s depois) e soltos na volta.
        {
            const float u = m_landTime / std::max(0.05f, m_landDuration);
            m_landPull = input.grounded ? smoothStep(m_landTime / 0.05f) * (1.0f - smoothStep((u - 0.45f) / 0.35f))
                * m_landFade : 0.0f;
            m_landRunning = smoothStep((speed - 2.0f) / 3.0f);
        }
        landTrunk = s.landingTrunkMaximum * amount * shape(0.03f);
        landUpper = s.landingUpperMaximum * amount * shape(0.07f);
        landHead = s.landingHeadMaximum * amount * shape(0.08f);
        if (m_landTime > m_landDuration + 0.1f || m_landFade < 0.01f) m_landTime = -1.0f;
    }
    spring(m_airWeight, m_airRate, !input.grounded && m_airClip != nullptr ? 1.0f : 0.0f, s.airBlendSeconds, dt);
    m_airWeight = std::clamp(m_airWeight, 0.0f, 1.0f);

}

} // namespace MatterEngine
