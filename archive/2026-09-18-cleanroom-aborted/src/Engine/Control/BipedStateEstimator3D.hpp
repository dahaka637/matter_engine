#pragma once

#include "Engine/Control/BipedControlTypes3D.hpp"
#include "Engine/Control/BipedRig3D.hpp"

namespace MatterEngine {

struct BipedStateEstimatorSettings3D {
    float minimumFootLoadWeightFraction = 0.025f;
    float supportContactNormalMinimumDot = 0.20f;
    float slipSpeedMetersPerSecond = 0.22f;
    float uprightTorsoDot = 0.70f;
    float uprightComHeightFraction = 0.62f;
    float fallenTorsoDot = 0.55f;
    float fallenComHeightFraction = 0.55f;
};

class BipedStateEstimator3D {
public:
    [[nodiscard]] BipedObservation3D observe(
        const RagdollProfile3D& profile,
        const BipedRig3D& rig,
        const RagdollState3D& state,
        const RagdollDynamics3D& dynamics,
        float deltaTime) const;

    BipedStateEstimatorSettings3D& settings() { return m_settings; }
    const BipedStateEstimatorSettings3D& settings() const {
        return m_settings;
    }

private:
    BipedStateEstimatorSettings3D m_settings;
};

} // namespace MatterEngine
