#pragma once

#include "Engine/Control/BipedControlTypes3D.hpp"

namespace MatterEngine {

struct ResidualGetUpAssistSettings3D {
    bool enabled = true;
    float minimumActuatorDemandRatio = 0.82f;
    float minimumSupportLoadWeightFraction = 0.22f;
    float minimumFailureSeconds = 0.24f;
    float minimumProgressVelocityMetersPerSecond = 0.08f;
    float maximumResidualAccelerationGravityFraction = 0.30f;
    float maximumImpulseWeightSeconds = 0.75f;
};

struct ResidualGetUpAssistInput3D {
    bool inGetUp = false;
    bool eligibleStage = false;
    bool manipulated = false;
    bool airborne = false;
    float totalMassKg = 0.0f;
    float supportLoadNewtons = 0.0f;
    float actuatorDemandPeak = 0.0f;
    float centerOfMassVerticalVelocity = 0.0f;
    float desiredVerticalVelocity = 0.0f;
    float deltaTime = 0.0f;
};

class ResidualGetUpAssist3D {
public:
    void reset();
    [[nodiscard]] Vec3 update(
        const ResidualGetUpAssistInput3D& input);

    ResidualGetUpAssistSettings3D& settings() { return m_settings; }
    const ResidualGetUpAssistSettings3D& settings() const {
        return m_settings;
    }

    [[nodiscard]] float accumulatedImpulseNewtonSeconds() const {
        return m_impulseNewtonSeconds;
    }
    [[nodiscard]] float activeSeconds() const {
        return m_activeSeconds;
    }

private:
    ResidualGetUpAssistSettings3D m_settings;
    float m_failureSeconds = 0.0f;
    float m_activeSeconds = 0.0f;
    float m_impulseNewtonSeconds = 0.0f;
};

} // namespace MatterEngine
