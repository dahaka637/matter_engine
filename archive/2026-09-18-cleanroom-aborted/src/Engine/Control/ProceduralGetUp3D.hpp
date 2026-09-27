#pragma once

#include "Engine/Control/BipedControlTypes3D.hpp"
#include "Engine/Control/BipedRig3D.hpp"

namespace MatterEngine {

struct ProceduralGetUpSettings3D {
    float establishSupportHeightFraction = 0.28f;
    float raiseTorsoHeightFraction = 0.50f;
    float bringFeetHeightFraction = 0.60f;
    float crouchHeightFraction = 0.72f;
    float standingHeightFraction = 0.92f;
    float stageTimeoutSeconds = 4.0f;
    float stableTransitionSeconds = 0.16f;
    float groundProbeUpMeters = 0.35f;
    float groundProbeDownMeters = 1.00f;
};

class ProceduralGetUp3D {
public:
    void reset();
    [[nodiscard]] BipedGetUpCommand3D update(
        const RagdollProfile3D& profile,
        const BipedRig3D& rig,
        const RagdollState3D& state,
        const BipedObservation3D& observation,
        const BipedTerrainQuery3D& terrain,
        float headingRadians,
        float deltaTime);

    [[nodiscard]] unsigned replans() const { return m_replans; }
    [[nodiscard]] BipedGetUpStage3D stage() const { return m_stage; }
    [[nodiscard]] BipedFallPose3D initialPose() const { return m_initialPose; }

    ProceduralGetUpSettings3D& settings() { return m_settings; }

private:
    [[nodiscard]] BipedFallPose3D classifyPose(
        const RagdollProfile3D& profile,
        const BipedRig3D& rig,
        const RagdollState3D& state) const;
    [[nodiscard]] Vec3 projectToGround(
        Vec3 pointWorld,
        const BipedTerrainQuery3D& terrain,
        float fallbackHeight,
        Vec3* normal = nullptr) const;
    [[nodiscard]] std::size_t usefulSupportCount(
        const BipedRig3D& rig,
        const BipedObservation3D& observation) const;
    void transition(BipedGetUpStage3D stage);

    ProceduralGetUpSettings3D m_settings;
    BipedGetUpStage3D m_stage = BipedGetUpStage3D::None;
    BipedFallPose3D m_initialPose = BipedFallPose3D::Unknown;
    float m_stageSeconds = 0.0f;
    float m_stableSeconds = 0.0f;
    unsigned m_replans = 0;
};

} // namespace MatterEngine
