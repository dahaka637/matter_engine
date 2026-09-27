#pragma once

#include "Engine/Control/BipedBalanceController3D.hpp"
#include "Engine/Control/BipedControlTypes3D.hpp"
#include "Engine/Control/BipedDynamicsAdapter3D.hpp"
#include "Engine/Control/BipedKinematics3D.hpp"
#include "Engine/Control/BipedRig3D.hpp"
#include "Engine/Control/BipedStateEstimator3D.hpp"
#include "Engine/Control/ProceduralGetUp3D.hpp"
#include "Engine/Control/ResidualGetUpAssist3D.hpp"
#include "Engine/Locomotion/BipedStepPlanner3D.hpp"

namespace MatterEngine {

struct PhysicalBipedControllerSettings3D {
    float muscleStrength = 1.0f;
    float walkSpeedMetersPerSecond = 1.35f;
    float runSpeedMetersPerSecond = 3.20f;
    float maximumDriveTargetSpeedRadiansPerSecond = 5.5f;
    float swingDriveTargetSpeedRadiansPerSecond = 8.0f;
    float getUpDriveTargetSpeedRadiansPerSecond = 7.0f;
    float fallConfirmationSeconds = 0.16f;
    float airborneFallConfirmationSeconds = 0.14f;
    float getUpStillSeconds = 0.35f;
    float uprightConfirmationSeconds = 0.20f;
    bool automaticGetUp = true;
    bool residualGetUpAssistEnabled = true;
};

class PhysicalBipedController3D {
public:
    void reset(const RagdollProfile3D& profile,
        const RagdollState3D& state);

    [[nodiscard]] bool requestWalk(const RagdollState3D& state,
        float seconds, Vec3 directionBody = {1.0f, 0.0f, 0.0f});
    [[nodiscard]] bool requestRun(const RagdollState3D& state,
        float seconds, Vec3 directionBody = {1.0f, 0.0f, 0.0f});
    void setIntent(const BipedIntent3D& intent);
    void stop();

    void update(const RagdollProfile3D& profile,
        const RagdollState3D& state,
        const RagdollDynamics3D& dynamics,
        const BipedTerrainQuery3D& terrain,
        float deltaTime,
        bool manipulated = false,
        std::uint32_t grabbedLink = RagdollDynamics3D::InvalidIndex);

    [[nodiscard]] const BipedControllerOutput3D& output() const {
        return m_output;
    }
    [[nodiscard]] const BipedTelemetry3D& telemetry() const {
        return m_telemetry;
    }
    [[nodiscard]] const BipedRig3D& rig() const { return m_rig; }

    PhysicalBipedControllerSettings3D& settings() { return m_settings; }
    BipedStateEstimator3D& stateEstimator() { return m_estimator; }
    BipedBalanceController3D& balanceController() { return m_balance; }
    BipedStepPlanner3D& stepPlanner() { return m_stepPlanner; }
    ProceduralGetUp3D& getUpController() { return m_getUp; }
    ResidualGetUpAssist3D& residualAssist() { return m_residual; }

private:
    void enterPhase(BipedControllerPhase3D phase);
    void buildDriveTargets(
        const RagdollProfile3D& profile,
        const RagdollState3D& state,
        std::span<const Vec3> desiredCoordinates,
        const BipedJointTorqueField3D& feedforward,
        float deltaTime,
        float stiffnessScale,
        float torqueScale);
    void updateNormalControl(
        const RagdollProfile3D& profile,
        const RagdollState3D& state,
        const RagdollDynamics3D& dynamics,
        const BipedTerrainQuery3D& terrain,
        const BipedObservation3D& observation,
        float deltaTime);
    void updateGetUpControl(
        const RagdollProfile3D& profile,
        const RagdollState3D& state,
        const RagdollDynamics3D& dynamics,
        const BipedTerrainQuery3D& terrain,
        const BipedObservation3D& observation,
        float deltaTime,
        bool manipulated);

    PhysicalBipedControllerSettings3D m_settings;
    BipedRig3D m_rig;
    BipedStateEstimator3D m_estimator;
    BipedBalanceController3D m_balance;
    BipedStepPlanner3D m_stepPlanner;
    BipedDynamicsAdapter3D m_dynamics;
    ProceduralGetUp3D m_getUp;
    ResidualGetUpAssist3D m_residual;

    BipedIntent3D m_intent;
    BipedControllerOutput3D m_output;
    BipedTelemetry3D m_telemetry;
    std::vector<Vec3> m_driveCoordinates;

    bool m_initialized = false;
    bool m_timedCommand = false;
    float m_commandSecondsRemaining = 0.0f;
    float m_fallenSeconds = 0.0f;
    float m_airborneSeconds = 0.0f;
    float m_stillSeconds = 0.0f;
    float m_uprightSeconds = 0.0f;
    Vec3 m_pelvisForwardLocal {1.0f, 0.0f, 0.0f};
};

} // namespace MatterEngine
