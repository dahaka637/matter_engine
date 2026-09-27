#pragma once

#include "Engine/Math/Vec3.hpp"

#include <array>
#include <cstdint>

namespace MatterEngine {

enum class ProceduralContactPhase3D : std::uint8_t {
    DoubleSupport,
    WeightShift,
    Swing,
    Touchdown,
    Unsupported
};

enum class ProceduralStepReason3D : std::uint8_t {
    None,
    Locomotion,
    Turning,
    FootSeparation,
    BalanceRecovery
};

struct ProceduralBipedGaitInput3D {
    std::array<Vec3, 2> footPositionWorld;
    std::array<bool, 2> footContact {};
    std::array<float, 2> contactConfidence {};
    std::array<float, 2> contactLoadNewtons {};
    Vec3 centerOfMassWorld;
    Vec3 centerOfMassVelocityWorld;
    Vec3 capturePointWorld;
    Vec3 observedSupportCenterWorld;
    Vec3 desiredVelocityWorld;
    float desiredFacingYawRadians = 0.0f;
    float facingErrorRadians = 0.0f;
    float supportMarginMeters = 0.0f;
    float groundHeightWorld = 0.0f;
    float bodyMassKg = 75.0f;
    float deltaTime = 1.0f / 120.0f;
    bool allowStepping = false;
    bool fallen = false;
};

struct ProceduralBipedGaitOutput3D {
    std::array<Vec3, 2> footTargetWorld;
    std::array<bool, 2> plannedContact { true, true };
    Vec3 balanceTargetWorld;
    Vec3 landingTargetWorld;
    ProceduralContactPhase3D phase =
        ProceduralContactPhase3D::DoubleSupport;
    ProceduralStepReason3D reason = ProceduralStepReason3D::None;
    int swingFoot = -1;
    int stanceFoot = -1;
    float phaseProgress = 0.0f;
    float swingClearanceMeters = 0.0f;
};

// Contact scheduler and swing-foot planner for a free-base biped. Observed
// contacts are inputs only; planned contacts describe intent and never fake a
// physical touchdown. The generated trajectory is C2-continuous in the
// horizontal plane and has a real vertical apex so the foot cannot drag.
class ProceduralBipedGait3D final {
public:
    void reset(const std::array<Vec3, 2>& footPositionWorld);
    void update(const ProceduralBipedGaitInput3D& input);

    [[nodiscard]] const ProceduralBipedGaitOutput3D& output() const {
        return m_output;
    }

private:
    [[nodiscard]] int chooseSwingFoot(
        const ProceduralBipedGaitInput3D& input,
        ProceduralStepReason3D reason) const;
    [[nodiscard]] Vec3 planLanding(
        const ProceduralBipedGaitInput3D& input, int swingFoot,
        ProceduralStepReason3D reason) const;
    void beginWeightShift(const ProceduralBipedGaitInput3D& input,
        int swingFoot, ProceduralStepReason3D reason);
    void beginSwing(const ProceduralBipedGaitInput3D& input);
    void finishStep(const ProceduralBipedGaitInput3D& input);

    ProceduralBipedGaitOutput3D m_output;
    std::array<Vec3, 2> m_lockedFootWorld;
    Vec3 m_swingStartWorld;
    Vec3 m_landingWorld;
    float m_phaseSeconds = 0.0f;
    float m_phaseDurationSeconds = 0.42f;
    float m_doubleSupportSeconds = 0.0f;
    int m_lastSwingFoot = 1;
    bool m_initialized = false;
};

} // namespace MatterEngine
