// Matter Engine bounded residual get-up assistance.
// THEORY BOUNDARY: [R17] residual force as explicit dynamics-mismatch fallback.
// Forbidden in standing/walking/running/push recovery/airborne; vertical-only during eligible get-up stalls.

#include "Engine/Control/ResidualGetUpAssist3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {

void ResidualGetUpAssist3D::reset() {
    m_failureSeconds = 0.0f;
    m_activeSeconds = 0.0f;
    m_impulseNewtonSeconds = 0.0f;
}

Vec3 ResidualGetUpAssist3D::update(
    const ResidualGetUpAssistInput3D& input) {
    if (!m_settings.enabled
        || !input.inGetUp
        || !input.eligibleStage
        || input.manipulated
        || input.airborne
        || !std::isfinite(input.totalMassKg)
        || !std::isfinite(input.supportLoadNewtons)
        || !std::isfinite(input.actuatorDemandPeak)
        || !std::isfinite(input.centerOfMassVerticalVelocity)
        || !std::isfinite(input.desiredVerticalVelocity)
        || !std::isfinite(input.deltaTime)
        || input.totalMassKg <= 0.0f
        || input.deltaTime <= 0.0f) {
        m_failureSeconds = 0.0f;
        return {};
    }

    const float weight = input.totalMassKg*9.81f;
    if (input.supportLoadNewtons
            < weight*m_settings.minimumSupportLoadWeightFraction
        || input.actuatorDemandPeak
            < m_settings.minimumActuatorDemandRatio) {
        m_failureSeconds = 0.0f;
        return {};
    }

    const float progressDeficit =
        input.desiredVerticalVelocity
            -input.centerOfMassVerticalVelocity;
    if (progressDeficit
            <= m_settings.minimumProgressVelocityMetersPerSecond) {
        m_failureSeconds = 0.0f;
        return {};
    }

    m_failureSeconds += input.deltaTime;
    if (m_failureSeconds < m_settings.minimumFailureSeconds)
        return {};

    const float maximumImpulse =
        weight*m_settings.maximumImpulseWeightSeconds;
    if (m_impulseNewtonSeconds >= maximumImpulse)
        return {};

    const float maximumAcceleration =
        9.81f*m_settings.maximumResidualAccelerationGravityFraction;
    // The residual is proportional to a velocity deficit over the same
    // failure window that authorized it. It never contains position,
    // orientation, heading, or airborne correction.
    const float acceleration = std::clamp(
        progressDeficit/std::max(
            m_settings.minimumFailureSeconds, 0.05f),
        0.0f, maximumAcceleration);
    float force = input.totalMassKg*acceleration;

    const float impulseRemaining =
        std::max(0.0f, maximumImpulse-m_impulseNewtonSeconds);
    force = std::min(force,
        impulseRemaining/std::max(input.deltaTime, 1.0e-5f));
    if (!(force > 0.0f)) return {};

    m_activeSeconds += input.deltaTime;
    m_impulseNewtonSeconds += force*input.deltaTime;
    return {0.0f, 0.0f, force};
}

} // namespace MatterEngine
