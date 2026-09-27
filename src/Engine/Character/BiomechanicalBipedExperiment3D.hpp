#pragma once

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Character/ProceduralBipedGait3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace MatterEngine {

enum class BiomechanicalBipedPhase3D : std::uint8_t {
    Settling,
    Standing,
    Walking,
    CaptureStep,
    GettingUp,
    Unsupported,
    Fallen
};

enum class ProceduralGetUpPhase3D : std::uint8_t {
    None,
    Resting,
    Assessing,
    SupineTuck,
    SupineSit,
    ProneBrace,
    PronePush,
    GatherFeet,
    Rise,
    Stabilize
};

enum class ProceduralFallOrientation3D : std::uint8_t {
    None,
    FaceUp,
    FaceDown
};

struct BiomechanicalBipedInput3D {
    Vec3 desiredVelocityWorld;
    float desiredFacingYawRadians = 0.0f;
    float muscleAuthority = 1.0f;
    float balanceAssistance = 1.0f;
    // Authored reference poses for the joint drive targets below. Sampled
    // curves only ever seed/blend desired joint coordinates that the PD
    // drives then chase through forces; they never gate contact, IK or the
    // balance wrench, and the floating base guide stays at zero authority.
    const AnimationClip3D* idleReferenceClip = nullptr;
    const AnimationClip3D* locomotionReferenceClip = nullptr;
};

struct BiomechanicalBipedOutput3D {
    std::vector<RagdollDriveTarget3D> driveTargets;
    // Deliberately left at zero authority. The floating base is never guided.
    RagdollAnimationConstraint3D guide;
    Vec3 balanceForceWorld;
    Vec3 balanceTorqueWorld;
    bool gravityCompensationEnabled = true;
};

struct BiomechanicalBipedTelemetry3D {
    BiomechanicalBipedPhase3D phase = BiomechanicalBipedPhase3D::Settling;
    Vec3 centerOfMassWorld;
    Vec3 centerOfMassVelocityWorld;
    Vec3 capturePointWorld;
    Vec3 supportCenterWorld;
    Vec3 requestedGroundReactionNewtons;
    Vec3 balanceForceWorld;
    Vec3 balanceTorqueWorld;
    std::array<bool, 2> footContact {};
    std::array<bool, 2> plannedFootContact { true, true };
    std::array<float, 2> contactLoadNewtons {};
    std::array<Vec3, 2> proceduralFootTargetWorld;
    float supportMarginMeters = 0.0f;
    float gaitPhase = 0.0f;
    float swingClearanceMeters = 0.0f;
    float desiredSpeedMetersPerSecond = 0.0f;
    float rootUpright = 1.0f;
    int swingFoot = -1;
    ProceduralContactPhase3D contactPhase =
        ProceduralContactPhase3D::DoubleSupport;
    ProceduralStepReason3D stepReason = ProceduralStepReason3D::None;
    ProceduralGetUpPhase3D getUpPhase = ProceduralGetUpPhase3D::None;
    ProceduralFallOrientation3D fallOrientation =
        ProceduralFallOrientation3D::None;
    float getUpProgress = 0.0f;
    std::uint32_t getUpAttempt = 0;
    bool predictiveBraking = false;
    bool dynamicsAvailable = false;
};

// Isolated floating-base experiment. It has no capsule trajectory or
// post-solver pose correction. Finite joint drives and real foot contacts do
// the primary work; an explicit bounded balance wrench can be calibrated.
class BiomechanicalBipedExperiment3D final {
public:
    void reset(const RagdollProfile3D& profile,
        const RagdollState3D& state,
        const RagdollDynamics3D& dynamics);
    void update(const RagdollProfile3D& profile,
        const RagdollState3D& state,
        const RagdollDynamics3D& dynamics,
        const BiomechanicalBipedInput3D& input,
        float deltaTime);

    [[nodiscard]] const BiomechanicalBipedOutput3D& output() const {
        return m_output;
    }
    [[nodiscard]] const BiomechanicalBipedTelemetry3D& telemetry() const {
        return m_telemetry;
    }

private:
    BiomechanicalBipedOutput3D m_output;
    BiomechanicalBipedTelemetry3D m_telemetry;
    std::vector<Vec3> m_targets;
    std::vector<Vec3> m_previousTargets;
    ProceduralBipedGait3D m_gait;
    std::array<std::size_t, 2> m_thighLinks {};
    std::array<std::size_t, 2> m_shinLinks {};
    std::array<std::size_t, 2> m_footLinks {};
    std::array<std::size_t, 2> m_handLinks {};
    std::size_t m_chestLink = 0;
    std::array<float, 2> m_contactConfidence {};
    std::array<Vec3, 2> m_contactPointWorld {};
    Vec3 m_previousCenterOfMass;
    Vec3 m_filteredCenterOfMassVelocity;
    float m_nominalCenterOfMassHeight = 0.92f;
    float m_idleClipSeconds = 0.0f;
    float m_locomotionClipSeconds = 0.0f;
    float m_elapsedSeconds = 0.0f;
    float m_unsupportedSeconds = 0.0f;
    ProceduralGetUpPhase3D m_getUpPhase = ProceduralGetUpPhase3D::None;
    ProceduralFallOrientation3D m_fallOrientation =
        ProceduralFallOrientation3D::None;
    float m_getUpPhaseSeconds = 0.0f;
    float m_recoveredStandingSeconds = 0.0f;
    std::uint32_t m_getUpAttempt = 0;
    bool m_initialized = false;
};

} // namespace MatterEngine
