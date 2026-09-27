#include "Engine/Character/ProceduralBipedGait3D.hpp"

#include "Engine/Math/Quaternion.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr float Gravity = 9.81f;

float smoothQuintic(float value) {
    value = std::clamp(value, 0.0f, 1.0f);
    const float squared = value * value;
    const float cubed = squared * value;
    return cubed * (10.0f + value * (-15.0f + value * 6.0f));
}

float wrapAngle(float radians) {
    while (radians > Pi) radians -= 2.0f * Pi;
    while (radians < -Pi) radians += 2.0f * Pi;
    return radians;
}

Vec3 clampPlanar(Vec3 value, float maximum) {
    value.z = 0.0f;
    const float length = value.length();
    return length > maximum && length > 0.000001f
        ? value * (maximum / length) : value;
}

} // namespace

void ProceduralBipedGait3D::reset(
    const std::array<Vec3, 2>& footPositionWorld) {
    m_output = {};
    m_output.phase = ProceduralContactPhase3D::DoubleSupport;
    m_output.footTargetWorld = footPositionWorld;
    m_output.balanceTargetWorld =
        (footPositionWorld[0] + footPositionWorld[1]) * 0.5f;
    m_lockedFootWorld = footPositionWorld;
    m_swingStartWorld = {};
    m_landingWorld = {};
    m_phaseSeconds = 0.0f;
    m_phaseDurationSeconds = 0.42f;
    m_doubleSupportSeconds = 0.0f;
    m_lastSwingFoot = 1;
    m_initialized = true;
}

int ProceduralBipedGait3D::chooseSwingFoot(
    const ProceduralBipedGaitInput3D& input,
    ProceduralStepReason3D reason) const {
    const Quaternion heading = Quaternion::fromAxisAngle(
        { 0.0f, 0.0f, 1.0f }, input.desiredFacingYawRadians);
    const Vec3 localVelocity = heading.conjugate().rotate(
        input.desiredVelocityWorld);
    const Vec3 captureError = heading.conjugate().rotate(
        input.capturePointWorld - input.observedSupportCenterWorld);

    int preferred = 1 - m_lastSwingFoot;
    if (reason == ProceduralStepReason3D::FootSeparation) {
        const Vec3 left = heading.conjugate().rotate(
            input.footPositionWorld[0] - input.centerOfMassWorld);
        const Vec3 right = heading.conjugate().rotate(
            input.footPositionWorld[1] - input.centerOfMassWorld);
        const float leftViolation = 0.085f - left.y;
        const float rightViolation = right.y + 0.085f;
        preferred = leftViolation >= rightViolation ? 0 : 1;
    } else if (reason == ProceduralStepReason3D::BalanceRecovery
        && std::abs(captureError.y) > 0.035f) {
        preferred = captureError.y >= 0.0f ? 0 : 1;
    } else if (std::abs(localVelocity.y)
            > std::abs(localVelocity.x) * 0.42f
        && std::abs(localVelocity.y) > 0.06f) {
        preferred = localVelocity.y >= 0.0f ? 0 : 1;
    } else if (reason == ProceduralStepReason3D::Turning) {
        preferred = wrapAngle(input.facingErrorRadians) >= 0.0f ? 0 : 1;
    }

    const int other = 1 - preferred;
    const bool preferredCanSupport = input.footContact[other]
        || input.contactConfidence[other] > 0.28f;
    if (preferredCanSupport) return preferred;
    if (input.footContact[preferred]
        || input.contactConfidence[preferred] > 0.28f) {
        return other;
    }
    return preferred;
}

Vec3 ProceduralBipedGait3D::planLanding(
    const ProceduralBipedGaitInput3D& input, int swingFoot,
    ProceduralStepReason3D reason) const {
    const Quaternion heading = Quaternion::fromAxisAngle(
        { 0.0f, 0.0f, 1.0f }, input.desiredFacingYawRadians);
    const Quaternion inverseHeading = heading.conjugate();
    const float height = std::max(0.42f,
        input.centerOfMassWorld.z - input.groundHeightWorld);
    const float omega = std::sqrt(Gravity / height);

    Vec3 velocity = input.centerOfMassVelocityWorld;
    velocity.z = 0.0f;
    Vec3 desired = input.desiredVelocityWorld;
    desired.z = 0.0f;
    const Vec3 captureLead = clampPlanar(
        input.capturePointWorld - input.centerOfMassWorld, 0.34f);

    Vec3 center = input.centerOfMassWorld
        + desired * 0.30f
        + (velocity - desired) * 0.11f
        + captureLead * (reason == ProceduralStepReason3D::BalanceRecovery
            ? 0.78f : 0.34f);
    center.z = input.groundHeightWorld;

    const float side = swingFoot == 0 ? 1.0f : -1.0f;
    const float speed = desired.length();
    const float stanceWidth = std::clamp(0.105f + speed * 0.012f,
        0.10f, 0.125f);
    Vec3 landing = center + heading.rotate({ 0.0f,
        side * stanceWidth, 0.0f });

    // Turning in place moves the free foot around the support foot instead of
    // twisting both planted feet against static friction.
    if (reason == ProceduralStepReason3D::FootSeparation) {
        // Repair uses a slightly wider stance and preserves the current
        // longitudinal coordinate. Repeating this step for the other foot
        // restores ordering even after a severe physical crossing.
        const Vec3 currentLocal = inverseHeading.rotate(
            input.footPositionWorld[swingFoot]
                - input.centerOfMassWorld);
        Vec3 repairLocal {
            std::clamp(currentLocal.x, -0.22f, 0.22f),
            side * 0.125f, 0.0f
        };
        landing = input.centerOfMassWorld + heading.rotate(repairLocal);
    } else if (reason == ProceduralStepReason3D::Turning && speed < 0.08f) {
        const int stanceFoot = 1 - swingFoot;
        const float stepYaw = std::clamp(input.facingErrorRadians,
            -0.24f, 0.24f);
        const Quaternion incremental = Quaternion::fromAxisAngle(
            { 0.0f, 0.0f, 1.0f }, stepYaw);
        Vec3 radius = m_lockedFootWorld[swingFoot]
            - m_lockedFootWorld[stanceFoot];
        radius.z = 0.0f;
        landing = m_lockedFootWorld[stanceFoot]
            + incremental.rotate(radius);
    }

    // Enforce anatomical side separation in heading coordinates. This is the
    // crossing constraint that keeps diagonal and lateral steps from tangling
    // the legs.
    Vec3 localFromCom = inverseHeading.rotate(
        landing - input.centerOfMassWorld);
    localFromCom.x = std::clamp(localFromCom.x, -0.36f, 0.43f);
    if (swingFoot == 0) {
        localFromCom.y = std::clamp(localFromCom.y, 0.075f, 0.25f);
    } else {
        localFromCom.y = std::clamp(localFromCom.y, -0.25f, -0.075f);
    }
    localFromCom.z = 0.0f;
    landing = input.centerOfMassWorld + heading.rotate(localFromCom);

    const int stanceFoot = 1 - swingFoot;
    Vec3 fromStance = landing - m_lockedFootWorld[stanceFoot];
    fromStance.z = 0.0f;
    const float maximumReach = reason == ProceduralStepReason3D::BalanceRecovery
        ? 0.50f : std::clamp(0.36f + speed * 0.08f, 0.36f, 0.46f);
    fromStance = clampPlanar(fromStance, maximumReach);
    landing = m_lockedFootWorld[stanceFoot] + fromStance;
    landing.z = input.groundHeightWorld + 0.036f;

    // The capture point already includes v/omega. Referencing omega here also
    // prevents dead code elimination of the physical time scale in reduced
    // builds and documents the planner's dynamic basis.
    (void)omega;
    return landing;
}

void ProceduralBipedGait3D::beginWeightShift(
    const ProceduralBipedGaitInput3D& input, int swingFoot,
    ProceduralStepReason3D reason) {
    m_output.phase = ProceduralContactPhase3D::WeightShift;
    m_output.reason = reason;
    m_output.swingFoot = swingFoot;
    m_output.stanceFoot = 1 - swingFoot;
    m_output.plannedContact = { true, true };
    m_phaseSeconds = 0.0f;
    m_phaseDurationSeconds = reason == ProceduralStepReason3D::BalanceRecovery
        ? 0.075f : 0.12f;
    m_swingStartWorld = input.footPositionWorld[swingFoot];
    m_landingWorld = planLanding(input, swingFoot, reason);
}

void ProceduralBipedGait3D::beginSwing(
    const ProceduralBipedGaitInput3D& input) {
    const int swing = m_output.swingFoot;
    if (swing < 0) return;
    m_output.phase = ProceduralContactPhase3D::Swing;
    m_output.plannedContact[swing] = false;
    m_phaseSeconds = 0.0f;
    m_swingStartWorld = input.footPositionWorld[swing];
    const float speed = input.desiredVelocityWorld.length();
    m_phaseDurationSeconds = m_output.reason
            == ProceduralStepReason3D::BalanceRecovery
        ? 0.27f : std::clamp(0.46f - speed * 0.085f, 0.34f, 0.46f);
}

void ProceduralBipedGait3D::finishStep(
    const ProceduralBipedGaitInput3D& input) {
    const int swing = m_output.swingFoot;
    if (swing >= 0) {
        m_lockedFootWorld[swing] = input.footContact[swing]
            ? input.footPositionWorld[swing] : m_landingWorld;
        m_lastSwingFoot = swing;
    }
    m_output.phase = ProceduralContactPhase3D::DoubleSupport;
    m_output.reason = ProceduralStepReason3D::None;
    m_output.swingFoot = -1;
    m_output.stanceFoot = -1;
    m_output.plannedContact = { true, true };
    m_phaseSeconds = 0.0f;
    m_doubleSupportSeconds = 0.0f;
}

void ProceduralBipedGait3D::update(
    const ProceduralBipedGaitInput3D& input) {
    if (!m_initialized) reset(input.footPositionWorld);
    const float dt = std::clamp(input.deltaTime,
        1.0f / 1000.0f, 1.0f / 30.0f);
    m_phaseSeconds += dt;
    m_doubleSupportSeconds += dt;

    const bool hasSupport = input.footContact[0] || input.footContact[1]
        || input.contactConfidence[0] > 0.18f
        || input.contactConfidence[1] > 0.18f;
    if (input.fallen || !hasSupport) {
        m_output.phase = ProceduralContactPhase3D::Unsupported;
        m_output.reason = ProceduralStepReason3D::None;
        m_output.swingFoot = -1;
        m_output.stanceFoot = -1;
        m_output.plannedContact = { false, false };
        m_output.footTargetWorld = input.footPositionWorld;
        m_output.balanceTargetWorld = input.centerOfMassWorld;
        m_output.phaseProgress = 0.0f;
        return;
    }
    if (m_output.phase == ProceduralContactPhase3D::Unsupported) {
        reset(input.footPositionWorld);
    }

    const float speed = input.desiredVelocityWorld.length();
    const Quaternion heading = Quaternion::fromAxisAngle(
        { 0.0f, 0.0f, 1.0f }, input.desiredFacingYawRadians);
    const Vec3 leftLocal = heading.conjugate().rotate(
        input.footPositionWorld[0] - input.centerOfMassWorld);
    const Vec3 rightLocal = heading.conjugate().rotate(
        input.footPositionWorld[1] - input.centerOfMassWorld);
    const bool feetMisordered = leftLocal.y - rightLocal.y < 0.115f
        || leftLocal.y < 0.035f || rightLocal.y > -0.035f;
    ProceduralStepReason3D requestedReason = ProceduralStepReason3D::None;
    if (feetMisordered && input.supportMarginMeters > -0.07f) {
        requestedReason = ProceduralStepReason3D::FootSeparation;
    } else if (input.supportMarginMeters < -0.018f) {
        requestedReason = ProceduralStepReason3D::BalanceRecovery;
    } else if (speed > 0.055f) {
        requestedReason = ProceduralStepReason3D::Locomotion;
    } else if (std::abs(input.facingErrorRadians) > 0.13f) {
        requestedReason = ProceduralStepReason3D::Turning;
    }

    switch (m_output.phase) {
    case ProceduralContactPhase3D::DoubleSupport: {
        for (int side = 0; side < 2; ++side) {
            if (input.footContact[side]
                && (m_lockedFootWorld[side]
                    - input.footPositionWorld[side]).length() > 0.12f) {
                // Reacquire only after a real discontinuity (spawn/reset or a
                // forced slide), never every tick: ordinary stance remains a
                // world-space lock and therefore resists skating.
                m_lockedFootWorld[side] = input.footPositionWorld[side];
            }
        }
        if (input.allowStepping && requestedReason
                != ProceduralStepReason3D::None
            && m_doubleSupportSeconds > 0.065f) {
            beginWeightShift(input,
                chooseSwingFoot(input, requestedReason), requestedReason);
        }
        break;
    }
    case ProceduralContactPhase3D::WeightShift: {
        const int stance = m_output.stanceFoot;
        const float minimumLoad = input.bodyMassKg * Gravity * 0.24f;
        const bool stanceReady = stance >= 0
            && (input.contactLoadNewtons[stance] > minimumLoad
                || input.contactConfidence[stance] > 0.52f);
        if ((stanceReady && m_phaseSeconds >= m_phaseDurationSeconds)
            || m_phaseSeconds > 0.24f) {
            beginSwing(input);
        }
        break;
    }
    case ProceduralContactPhase3D::Swing: {
        const int swing = m_output.swingFoot;
        const float progress = std::clamp(
            m_phaseSeconds / std::max(0.001f, m_phaseDurationSeconds),
            0.0f, 1.0f);
        if (swing >= 0 && progress < 0.68f) {
            const Vec3 replanned = planLanding(input, swing, m_output.reason);
            const float blend = 1.0f - std::exp(-7.5f * dt);
            m_landingWorld += (replanned - m_landingWorld) * blend;
        }
        if (swing >= 0 && input.footContact[swing] && progress > 0.58f) {
            m_output.phase = ProceduralContactPhase3D::Touchdown;
            m_output.plannedContact[swing] = true;
            m_phaseSeconds = 0.0f;
        } else if (progress >= 1.0f) {
            m_output.phase = ProceduralContactPhase3D::Touchdown;
            if (swing >= 0) m_output.plannedContact[swing] = true;
            m_phaseSeconds = 0.0f;
        }
        break;
    }
    case ProceduralContactPhase3D::Touchdown: {
        const int swing = m_output.swingFoot;
        if ((swing >= 0 && input.footContact[swing]
                && m_phaseSeconds > 0.025f)
            || m_phaseSeconds > 0.24f) {
            finishStep(input);
        }
        break;
    }
    case ProceduralContactPhase3D::Unsupported:
        break;
    }

    m_output.footTargetWorld = m_lockedFootWorld;
    m_output.landingTargetWorld = m_landingWorld;
    m_output.swingClearanceMeters = 0.0f;
    m_output.phaseProgress = 0.0f;

    const int swing = m_output.swingFoot;
    const int stance = m_output.stanceFoot;
    if (m_output.phase == ProceduralContactPhase3D::WeightShift
        && stance >= 0) {
        const float progress = smoothQuintic(std::clamp(
            m_phaseSeconds / std::max(0.001f, m_phaseDurationSeconds),
            0.0f, 1.0f));
        m_output.phaseProgress = progress;
        m_output.balanceTargetWorld = input.observedSupportCenterWorld
            + (m_lockedFootWorld[stance]
                - input.observedSupportCenterWorld) * (0.72f * progress);
    } else if (m_output.phase == ProceduralContactPhase3D::Swing
        && swing >= 0 && stance >= 0) {
        const float progress = std::clamp(
            m_phaseSeconds / std::max(0.001f, m_phaseDurationSeconds),
            0.0f, 1.0f);
        const float travel = smoothQuintic(progress);
        const float speedClearance = std::clamp(
            input.desiredVelocityWorld.length() * 0.030f, 0.0f, 0.040f);
        const float clearance = (m_output.reason
                == ProceduralStepReason3D::BalanceRecovery ? 0.16f : 0.125f)
            + speedClearance;
        Vec3 target = m_swingStartWorld
            + (m_landingWorld - m_swingStartWorld) * travel;
        target.z += clearance * std::pow(
            std::max(0.0f, std::sin(Pi * progress)), 0.82f);
        m_output.footTargetWorld[swing] = target;
        m_output.balanceTargetWorld = m_lockedFootWorld[stance];
        m_output.phaseProgress = progress;
        m_output.swingClearanceMeters = target.z
            - std::max(m_swingStartWorld.z, m_landingWorld.z);
    } else if (m_output.phase == ProceduralContactPhase3D::Touchdown
        && swing >= 0 && stance >= 0) {
        m_output.footTargetWorld[swing] = m_landingWorld;
        m_output.balanceTargetWorld = m_lockedFootWorld[stance] * 0.72f
            + m_landingWorld * 0.28f;
        m_output.phaseProgress = 1.0f;
    } else {
        m_output.balanceTargetWorld = input.observedSupportCenterWorld;
    }
    m_output.balanceTargetWorld.z = input.groundHeightWorld;
}

} // namespace MatterEngine
