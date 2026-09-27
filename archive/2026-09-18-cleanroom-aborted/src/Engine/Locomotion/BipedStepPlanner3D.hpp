#pragma once

#include "Engine/Control/BipedControlTypes3D.hpp"
#include "Engine/Control/BipedRig3D.hpp"

namespace MatterEngine {

struct BipedStepPlannerSettings3D {
    // GBWC §3.2 reports alpha=0.05 for desired-speed modification.
    float desiredVelocityAlphaSeconds = 0.05f;
    // GBWC limits the IP prediction to 0.6 leg lengths.
    float maximumStepReachLegFraction = 0.60f;
    float nominalClearanceLegFraction = 0.085f;
    float minimumStanceLoadRatioForLift = 0.72f;
    float maximumSwingLoadRatioForLift = 0.28f;
    float minimumTouchdownLoadWeightFraction = 0.035f;
    float transferTimeoutSeconds = 0.75f;
    float settleSeconds = 0.075f;
    float walkSwingSeconds = 0.36f;
    float runSwingSeconds = 0.24f;
    float recoverySwingSeconds = 0.18f;
    float minimumSwingSeconds = 0.14f;
    float maximumSwingSeconds = 0.48f;
    float landingReplanSpeedMetersPerSecond = 1.8f;
    float terrainProbeUpMeters = 0.45f;
    float terrainProbeDownMeters = 1.20f;
};

class BipedStepPlanner3D {
public:
    void reset();
    [[nodiscard]] BipedStepPlan3D update(
        const RagdollProfile3D& profile,
        const BipedRig3D& rig,
        const BipedObservation3D& observation,
        const BipedIntent3D& intent,
        const BipedBalanceDecision3D& balance,
        const BipedTerrainQuery3D& terrain,
        float deltaTime);

    BipedStepPlannerSettings3D& settings() { return m_settings; }
    const BipedStepPlannerSettings3D& settings() const {
        return m_settings;
    }

private:
    [[nodiscard]] int chooseSwingFoot(
        const BipedObservation3D& observation,
        const BipedIntent3D& intent,
        const BipedBalanceDecision3D& balance) const;
    [[nodiscard]] bool computeLanding(
        const RagdollProfile3D& profile,
        const BipedRig3D& rig,
        const BipedObservation3D& observation,
        const BipedIntent3D& intent,
        const BipedBalanceDecision3D& balance,
        const BipedTerrainQuery3D& terrain,
        int swingFoot,
        Vec3& landing,
        Quaternion& orientation) const;
    [[nodiscard]] bool canLift(
        const BipedObservation3D& observation) const;
    void startTransfer(int swingFoot, bool recovery);
    void startSwing(
        const RagdollProfile3D& profile,
        const BipedRig3D& rig,
        const BipedObservation3D& observation,
        const BipedIntent3D& intent,
        const BipedBalanceDecision3D& balance,
        const BipedTerrainQuery3D& terrain);

    BipedStepPlannerSettings3D m_settings;
    BipedStepPhase3D m_phase = BipedStepPhase3D::Idle;
    int m_stanceFoot = -1;
    int m_swingFoot = -1;
    int m_nextSwingFoot = 0;
    bool m_recovery = false;
    float m_phaseSeconds = 0.0f;
    float m_swingDurationSeconds = 0.0f;
    Vec3 m_swingStartSoleWorld;
    Vec3 m_landingSoleWorld;
    Quaternion m_landingOrientationWorld;
};

} // namespace MatterEngine
