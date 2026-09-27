#pragma once

#include "Engine/Control/BipedControlTypes3D.hpp"
#include "Engine/Control/BipedRig3D.hpp"

namespace MatterEngine {

struct BipedBalanceSettings3D {
    float captureSafeMarginLegFraction = 0.025f;
    float captureStepMarginLegFraction = -0.055f;

    // Cartwheel/GBWC reference implementation uses acceleration feedback
    // around 20 s^-2 for coronal position, 9 s^-1 for coronal velocity and
    // 30 s^-1 for sagittal velocity. These remain separately tunable.
    float standingPositionGainPerSecondSquared = 18.0f;
    float transferPositionGainPerSecondSquared = 20.0f;
    float sagittalVelocityGainPerSecond = 30.0f;
    float coronalVelocityGainPerSecond = 9.0f;
    float maximumHorizontalForceWeightFraction = 0.45f;

    // Dimensioned torso PD: Kp=I*wn^2, Kd=2*zeta*I*wn.
    float torsoNaturalFrequencyRadiansPerSecond = 6.0f;
    float torsoDampingRatio = 1.0f;
    float torsoInertiaMassHeightSquaredFraction = 0.12f;
    float maximumTorsoTorqueWeightLengthFraction = 0.35f;
};

class BipedBalanceController3D {
public:
    [[nodiscard]] BipedBalanceDecision3D assess(
        const BipedRig3D& rig,
        const BipedObservation3D& observation,
        const BipedIntent3D& intent) const;

    [[nodiscard]] BipedBalanceCommand3D command(
        const RagdollProfile3D& profile,
        const BipedRig3D& rig,
        const RagdollState3D& state,
        const BipedObservation3D& observation,
        const BipedIntent3D& intent,
        const BipedStepPlan3D& step,
        const BipedBalanceDecision3D& decision) const;

    BipedBalanceSettings3D& settings() { return m_settings; }
    const BipedBalanceSettings3D& settings() const {
        return m_settings;
    }

private:
    BipedBalanceSettings3D m_settings;
};

} // namespace MatterEngine
