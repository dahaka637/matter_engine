#pragma once

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

namespace MatterEngine {

enum class AnimatedRagdollPhase3D { Idle, Running, Stopping, Falling };

struct AnimatedRagdollClips3D {
    const AnimationClip3D* idle = nullptr;
    const AnimationClip3D* run = nullptr;
    const AnimationClip3D* stop = nullptr;
    [[nodiscard]] bool compatible(const RagdollProfile3D& profile) const;
};

struct AnimatedRagdollSettings3D {
    bool assistanceEnabled = true;
    // Zero derives cadence-compatible speed from the imported source.
    float runSpeedMetersPerSecond = 0.0f;
    // Residual help only: no vertical force, position spring or gravity lift.
    float maximumAssistWeightFraction = 0.12f;
    float maximumAssistTorqueWeightLengthFraction = 0.035f;
};

struct AnimatedRagdollForce3D {
    std::uint32_t linkIndex = 0;
    Vec3 forceNewtons;
    Vec3 torqueNewtonMeters;
};

struct AnimatedRagdollOutput3D {
    std::vector<RagdollDriveTarget3D> driveTargets;
    std::vector<AnimatedRagdollForce3D> assistance;
    RagdollAnimationPose3D targetPose;
    // Internal joint feedforward only; the six floating-base components
    // are never applied by PhysicsScene3D.
    bool gravityCompensationEnabled = false;
};

struct AnimatedRagdollTelemetry3D {
    AnimatedRagdollPhase3D phase = AnimatedRagdollPhase3D::Idle;
    float phaseSeconds = 0;
    float runSecondsRemaining = 0;
    float speedMetersPerSecond = 0;
    float travelledMeters = 0;
    float jointRmsDegrees = 0;
    float assistanceForceNewtons = 0;
    float assistanceTorqueNewtonMeters = 0;
    float assistanceAuthority = 0;
    float supportLoadNewtons = 0;
    float bodyUpDot = 1;
    bool blocked = false;
    bool manipulated = false;
};

// Animation is a bounded target. The ONLY outputs applied to the world are
// articulation drives and explicit external wrenches. No transforms/velocities
// are written to bodies. This controller has no dependency on archived WBC/IK.
class AnimatedRagdollController3D {
public:
    void reset(const RagdollProfile3D& profile, const RagdollState3D& state,
        float groundHeight);
    [[nodiscard]] bool requestRun(const RagdollState3D& state, float seconds = 5.0f);
    void update(const RagdollProfile3D& profile, const AnimatedRagdollClips3D& clips,
        const RagdollState3D& state, float deltaTime, float groundHeight,
        bool manipulated = false, bool obstacleAhead = false);
    [[nodiscard]] AnimatedRagdollSettings3D& settings() { return m_settings; }
    [[nodiscard]] const AnimatedRagdollOutput3D& output() const { return m_output; }
    [[nodiscard]] const AnimatedRagdollTelemetry3D& telemetry() const { return m_telemetry; }
private:
    void transition(AnimatedRagdollPhase3D phase);
    AnimatedRagdollSettings3D m_settings;
    AnimatedRagdollOutput3D m_output;
    AnimatedRagdollTelemetry3D m_telemetry;
    std::vector<Vec3> m_coordinates, m_blendFrom;
    // Diagnostic origin only. Never used to compute a force or motor target.
    Vec3 m_commandOrigin;
    Quaternion m_heading;
    float m_phaseTime = 0, m_blendTime = 0, m_requestedSeconds = 5;
    float m_speed = 0, m_stopRate = 1;
    bool m_initialized = false, m_wasManipulated = false;
};

} // namespace MatterEngine
