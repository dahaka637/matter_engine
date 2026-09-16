#include "Engine/Locomotion/FootworkSystem3D.hpp"

#include "Engine/Locomotion/FootstepPlanner3D.hpp"
#include "Engine/Locomotion/FootworkTrajectory3D.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace MatterEngine {
namespace {

float horizontalLength(Vec3 value) noexcept {
    return std::sqrt(value.x * value.x + value.y * value.y);
}

Vec3 approachVector(Vec3 current, Vec3 target,
    float maximumDelta) noexcept {
    const Vec3 delta = target - current;
    const float distance = delta.length();
    if (distance <= maximumDelta || distance < 1.0e-6f) return target;
    return current + delta * (maximumDelta / distance);
}

float cross2D(Vec3 origin, Vec3 a, Vec3 b) noexcept {
    return (a.x - origin.x) * (b.y - origin.y)
        - (a.y - origin.y) * (b.x - origin.x);
}

FootPhase3D phaseForProgress(float progress,
    const FootworkConfig3D& config, bool aborting) noexcept {
    if (aborting) return FootPhase3D::Aborting;
    if (progress < config.liftOffFraction) return FootPhase3D::LiftOff;
    if (progress > 1.0f - config.touchDownFraction) {
        return FootPhase3D::TouchDown;
    }
    return FootPhase3D::Swing;
}

float swingDurationForDistance(float distanceMeters,
    const FootworkConfig3D& config, bool fast) noexcept {
    if (fast) {
        const float strideScale = std::clamp(
            distanceMeters / 0.52f, 0.82f, 1.18f);
        return std::clamp(
            config.fastSwingDurationSeconds * strideScale,
            0.12f, 0.30f);
    }
    const float distanceScale = std::clamp(
        distanceMeters
            / std::max(0.05f, config.stepTriggerDistanceMeters),
        0.70f, 1.42f);
    return std::clamp(
        config.swingDurationSeconds * distanceScale,
        config.minimumSwingDurationSeconds,
        config.maximumSwingDurationSeconds);
}

float swingPitchForProgress(float progress,
    const FootworkConfig3D& config, bool fast) noexcept {
    const float toeOff = fast
        ? config.fastToeOffPitchRadians
        : config.walkToeOffPitchRadians;
    const float touchDown = fast
        ? config.fastTouchDownPitchRadians
        : config.walkTouchDownPitchRadians;
    const float t = std::clamp(progress, 0.0f, 1.0f);
    const auto between = [](float value, float begin, float end) {
        return smoothStepUnit3D(
            (value - begin) / std::max(1.0e-5f, end - begin));
    };
    if (t < 0.18f) {
        return toeOff * between(t, 0.0f, 0.18f);
    }
    if (t < 0.48f) {
        return toeOff * (1.0f - between(t, 0.18f, 0.48f));
    }
    if (t < 0.78f) {
        return touchDown * between(t, 0.48f, 0.78f);
    }
    return touchDown * (1.0f - between(t, 0.78f, 1.0f));
}

void applySwingPitch(FootPose3D& pose, float pitchRadians,
    const FootworkConfig3D& config) noexcept {
    pose.orientation = pose.orientation
        * Quaternion::fromAxisAngle(
            { 0.0f, 1.0f, 0.0f }, pitchRadians);
    // O pivô visual fica no centro inferior do pé. Sem compensação, inclinar
    // faria ponta ou calcanhar atravessar o chão. Esta elevação mantém o
    // extremo mais baixo apoiado no plano local, produzindo toe-off real.
    pose.position += pose.groundNormal
        * (std::abs(std::sin(pitchRadians))
            * config.footLengthMeters * 0.5f);
}

} // namespace

FootworkSystem3D::FootworkSystem3D(FootworkConfig3D config)
    : m_config(config) {
    sanitizeConfig();
}

void FootworkSystem3D::sanitizeConfig() {
    m_config.footLengthMeters = std::clamp(
        m_config.footLengthMeters, 0.08f, 0.60f);
    m_config.footWidthMeters = std::clamp(
        m_config.footWidthMeters, 0.04f, 0.30f);
    m_config.footThicknessMeters = std::clamp(
        m_config.footThicknessMeters, 0.015f, 0.18f);
    m_config.stanceWidthMeters = std::clamp(
        m_config.stanceWidthMeters, m_config.footWidthMeters, 0.65f);
    m_config.walkSpeedMetersPerSecond = std::clamp(
        m_config.walkSpeedMetersPerSecond, 0.05f, 4.0f);
    m_config.fastSpeedMetersPerSecond = std::max(
        m_config.walkSpeedMetersPerSecond,
        std::clamp(m_config.fastSpeedMetersPerSecond, 0.05f, 6.0f));
    m_config.accelerationMetersPerSecondSquared = std::clamp(
        m_config.accelerationMetersPerSecondSquared, 0.1f, 30.0f);
    m_config.decelerationMetersPerSecondSquared = std::clamp(
        m_config.decelerationMetersPerSecondSquared, 0.1f, 40.0f);
    m_config.fastAccelerationMultiplier = std::clamp(
        m_config.fastAccelerationMultiplier, 1.0f, 4.0f);
    m_config.maximumRootYawRateRadiansPerSecond = std::clamp(
        m_config.maximumRootYawRateRadiansPerSecond, 0.05f, 12.0f);
    m_config.stepTriggerDistanceMeters = std::clamp(
        m_config.stepTriggerDistanceMeters, 0.02f, 0.80f);
    m_config.stepReleaseDistanceMeters = std::clamp(
        m_config.stepReleaseDistanceMeters, 0.01f,
        m_config.stepTriggerDistanceMeters * 0.95f);
    m_config.maximumStepReachMeters = std::max(
        m_config.stepTriggerDistanceMeters + 0.04f,
        std::clamp(m_config.maximumStepReachMeters, 0.10f, 1.50f));
    m_config.maximumStepYawRadians = std::clamp(
        m_config.maximumStepYawRadians, 0.05f, 1.40f);
    m_config.antiCrossingMarginMeters = std::clamp(
        m_config.antiCrossingMarginMeters, 0.005f, 0.20f);
    m_config.swingDurationSeconds = std::clamp(
        m_config.swingDurationSeconds, 0.08f, 1.50f);
    m_config.minimumSwingDurationSeconds = std::clamp(
        m_config.minimumSwingDurationSeconds, 0.06f,
        m_config.swingDurationSeconds);
    m_config.maximumSwingDurationSeconds = std::max(
        m_config.swingDurationSeconds,
        std::clamp(m_config.maximumSwingDurationSeconds,
            m_config.swingDurationSeconds, 2.0f));
    m_config.weightTransferDurationSeconds = std::clamp(
        m_config.weightTransferDurationSeconds, 0.0f, 1.0f);
    m_config.swingClearanceMeters = std::clamp(
        m_config.swingClearanceMeters, 0.0f, 0.65f);
    m_config.fastSwingDurationSeconds = std::clamp(
        m_config.fastSwingDurationSeconds, 0.12f, 0.60f);
    m_config.fastSwingClearanceMeters = std::clamp(
        m_config.fastSwingClearanceMeters, 0.02f, 0.50f);
    m_config.recoveryWeightTransferDurationSeconds = std::clamp(
        m_config.recoveryWeightTransferDurationSeconds, 0.0f, 0.30f);
    m_config.emergencyRecoveryWeightTransferDurationSeconds = std::clamp(
        m_config.emergencyRecoveryWeightTransferDurationSeconds,
        0.0f, m_config.recoveryWeightTransferDurationSeconds);
    m_config.fastVelocityLookAheadSeconds = std::clamp(
        m_config.fastVelocityLookAheadSeconds, 0.08f, 0.35f);
    m_config.walkToeOffPitchRadians = std::clamp(
        m_config.walkToeOffPitchRadians, -0.70f, 0.70f);
    m_config.walkTouchDownPitchRadians = std::clamp(
        m_config.walkTouchDownPitchRadians, -0.70f, 0.70f);
    m_config.fastToeOffPitchRadians = std::clamp(
        m_config.fastToeOffPitchRadians, -0.95f, 0.95f);
    m_config.fastTouchDownPitchRadians = std::clamp(
        m_config.fastTouchDownPitchRadians, -0.95f, 0.95f);
    m_config.liftOffFraction = std::clamp(
        m_config.liftOffFraction, 0.02f, 0.35f);
    m_config.touchDownFraction = std::clamp(
        m_config.touchDownFraction, 0.02f, 0.40f);
    m_config.targetChaseStrength = std::clamp(
        m_config.targetChaseStrength, 0.0f, 1.0f);
    m_config.abortDurationSeconds = std::clamp(
        m_config.abortDurationSeconds, 0.05f, 0.70f);
    m_config.maximumSlopeRadians = std::clamp(
        m_config.maximumSlopeRadians, 0.05f, 1.35f);
    m_config.maximumPatchHeightSpreadMeters = std::clamp(
        m_config.maximumPatchHeightSpreadMeters, 0.005f, 0.20f);
    m_config.rootSupportLimitMeters = std::clamp(
        m_config.rootSupportLimitMeters,
        m_config.stanceWidthMeters * 0.55f, 1.20f);
}

bool FootworkSystem3D::reset(Vec3 rootGroundPosition,
    float rootYawRadians, const FootworkTerrainProbe3D& terrain) {
    sanitizeConfig();
    m_rootGroundPosition = rootGroundPosition;
    m_rootVelocity = {};
    m_rootYawRadians = rootYawRadians;
    m_hasActiveStep = false;
    m_activeRecoveryStep = false;
    m_activeStepReleaseAllowed = true;
    m_hasPhysicalContactFeedback = false;
    m_physicalFootSupported = { false, false };
    m_hasPhysicalFootPose = false;
    m_inputWasMoving = false;
    m_completedStepCount = 0;
    m_targetChaseAccumulatorSeconds = 0.0f;
    m_activePlanVelocity = {};
    m_activePlanMovementLocal = {};
    m_activePlanYawRadians = rootYawRadians;
    m_lastCompletedSide = FootSide3D::Right;
    if (std::getenv("MATTERENGINE_FOOTWORK_EVENTS") != nullptr) {
        std::cerr << "[footwork] reset\n";
    }

    const Vec3 forward {
        std::cos(rootYawRadians), std::sin(rootYawRadians), 0.0f
    };
    const Vec3 left { -forward.y, forward.x, 0.0f };
    for (FootSide3D side : { FootSide3D::Left, FootSide3D::Right }) {
        const float sign = side == FootSide3D::Left ? 1.0f : -1.0f;
        const Vec3 candidate = rootGroundPosition
            + left * (sign * m_config.stanceWidthMeters * 0.5f)
            + forward * m_config.homeForwardOffsetMeters;
        const FootworkTerrainPatch3D patch =
            sampleFootworkTerrainPatch3D(
                terrain, candidate, rootYawRadians, m_config);
        if (!patch.valid) {
            clear();
            return false;
        }
        RuntimeFoot& foot = m_feet[footIndex3D(side)];
        foot.pose.position = patch.position;
        foot.pose.groundNormal = patch.normal;
        foot.pose.yawRadians = rootYawRadians;
        foot.pose.orientation = footOrientation3D(
            patch.normal, rootYawRadians);
        foot.home = foot.pose;
        foot.swingStart = foot.pose;
        foot.swingTarget = foot.pose;
        foot.phase = FootPhase3D::Planted;
        foot.progress = 0.0f;
        foot.weightTransferElapsedSeconds = 0.0f;
        foot.weightTransferDurationSeconds = 0.0f;
        foot.terrainQuality = patch.quality;
    }
    m_debug.initialized = true;
    updateDebugState();
    return true;
}

void FootworkSystem3D::clear() {
    m_feet = {};
    m_debug = {};
    m_rootGroundPosition = {};
    m_rootVelocity = {};
    m_rootYawRadians = 0.0f;
    m_hasActiveStep = false;
    m_activeRecoveryStep = false;
    m_activeStepReleaseAllowed = true;
    m_hasPhysicalContactFeedback = false;
    m_physicalFootSupported = { false, false };
    m_inputWasMoving = false;
    m_completedStepCount = 0;
    m_targetChaseAccumulatorSeconds = 0.0f;
    m_activePlanVelocity = {};
    m_activePlanMovementLocal = {};
    m_activePlanYawRadians = 0.0f;
}

void FootworkSystem3D::synchronizeRoot(Vec3 rootGroundPosition,
    Vec3 rootVelocity, float rootYawRadians) {
    if (!m_debug.initialized) return;
    m_rootGroundPosition = rootGroundPosition;
    m_rootVelocity = rootVelocity;
    m_rootYawRadians = rootYawRadians;
    updateHomes(rootVelocity, false);
    updateDebugState();
}

bool FootworkSystem3D::requestRecoveryStep(FootSide3D side,
    Vec3 captureTarget, bool emergency,
    const FootworkTerrainProbe3D& terrain) {
    if (!m_debug.initialized || (m_hasActiveStep
            && m_activeRecoveryStep)) {
        return false;
    }
    // Se já existe uma passada comum em voo, ela é a única perna que pode
    // reagir sem criar uma troca impossível de apoio no meio do movimento.
    // Redirecionamos esse mesmo pé ao novo ponto de captura.
    if (m_hasActiveStep) {
        side = m_activeSide;
    }
    RuntimeFoot& foot = m_feet[footIndex3D(side)];
    const RuntimeFoot& support =
        m_feet[footIndex3D(oppositeFoot3D(side))];
    FootstepPlanningContext3D context;
    context.side = side;
    context.desiredHome = foot.home;
    context.desiredHome.position = captureTarget;
    context.desiredHome.position.z = support.pose.position.z;
    context.desiredHome.yawRadians = m_rootYawRadians;
    context.desiredHome.orientation = footOrientation3D(
        context.desiredHome.groundNormal, m_rootYawRadians);
    context.supportFoot = support.pose;
    context.rootYawRadians = m_rootYawRadians;
    const FootstepPlan3D plan =
        planFootstep3D(context, terrain, m_config);
    if (!plan.valid) {
        m_debug.terrainBlocked = true;
        return false;
    }

    foot.swingStart = foot.pose;
    foot.swingTarget = plan.target;
    foot.terrainQuality = plan.terrainQuality;
    foot.progress = 0.0f;
    foot.phase = FootPhase3D::LiftOff;
    foot.fastStep = emergency;
    foot.physicallyReleased = false;
    foot.clearanceMeters = emergency
        ? m_config.fastSwingClearanceMeters
        : m_config.swingClearanceMeters;
    foot.durationSeconds = swingDurationForDistance(
        (foot.swingTarget.position - foot.swingStart.position).length(),
        m_config, emergency);
    const bool groundShuffle = emergency
        && foot.clearanceMeters <= 0.001f;
    if (groundShuffle) {
        // O passo interno lateral não possui uma fase balística: é uma
        // reposição reflexiva rente ao solo. Ele precisa terminar antes do
        // próximo passo externo, não no relógio de uma passada completa.
        foot.durationSeconds = std::min(foot.durationSeconds, 0.12f);
    }
    // Passo de captura nao espera a transferencia nominal da caminhada, mas
    // tambem nao pode arrancar instantaneamente do chao um pe carregado. A
    // janela reflexiva curta permite ao quadril deslocar o peso para o apoio.
    foot.weightTransferElapsedSeconds = 0.0f;
    foot.weightTransferDurationSeconds = emergency
        ? m_config.emergencyRecoveryWeightTransferDurationSeconds
        : m_config.recoveryWeightTransferDurationSeconds;
    m_activeSide = side;
    m_hasActiveStep = true;
    m_activeRecoveryStep = true;
    m_targetChaseAccumulatorSeconds = 0.0f;
    // O comando explícito já contém toda a intenção. Mantê-lo separado do
    // input de marcha evita abortar ou encadear a passada automaticamente.
    m_activePlanVelocity = {};
    m_activePlanMovementLocal = {};
    m_activePlanYawRadians = m_rootYawRadians;
    m_inputWasMoving = false;
    m_debug.terrainBlocked = false;
    if (std::getenv("MATTERENGINE_FOOTWORK_EVENTS") != nullptr) {
        std::cerr << "[footwork] recovery begin side="
                  << static_cast<int>(side) << " target=("
                  << foot.swingTarget.position.x << ","
                  << foot.swingTarget.position.y << ")\n";
    }
    buildSwingTrajectory();
    updateDebugState();
    return true;
}

bool FootworkSystem3D::recoverUnsupportedFoot(FootSide3D side,
    Vec3 captureTarget, bool emergency,
    const FootworkTerrainProbe3D& terrain) {
    if (!m_debug.initialized) return false;
    if (m_hasActiveStep && m_activeSide != side) {
        m_hasActiveStep = false;
        m_activeRecoveryStep = false;
        m_targetChaseAccumulatorSeconds = 0.0f;
    }
    if (m_hasActiveStep && m_activeSide == side
        && m_activeRecoveryStep) {
        return true;
    }
    return requestRecoveryStep(
        side, captureTarget, emergency, terrain);
}

bool FootworkSystem3D::retargetActiveStep(Vec3 captureTarget,
    bool allowCrossing, const FootworkTerrainProbe3D& terrain) {
    if (!m_debug.initialized || !m_hasActiveStep) return false;
    RuntimeFoot& foot = m_feet[footIndex3D(m_activeSide)];
    // Passadas normais congelam o pouso no fim da curva. Uma passada de
    // captura, porem, pode chegar ao touchdown com a bacia ainda viajando;
    // se o alvo ficou fora do alcance da perna, mantemos o horizonte
    // recedente ate haver contato real em vez de deixar a sola suspensa.
    if (foot.progress >= 0.84f && !m_activeRecoveryStep) return false;
    const RuntimeFoot& support =
        m_feet[footIndex3D(oppositeFoot3D(m_activeSide))];
    FootstepPlanningContext3D context;
    context.side = m_activeSide;
    context.desiredHome = foot.home;
    context.desiredHome.position = captureTarget;
    context.desiredHome.position.z = support.pose.position.z;
    context.desiredHome.yawRadians = m_rootYawRadians;
    context.desiredHome.orientation = footOrientation3D(
        context.desiredHome.groundNormal, m_rootYawRadians);
    context.supportFoot = support.pose;
    context.rootYawRadians = m_rootYawRadians;
    context.allowCrossing = allowCrossing;
    const FootstepPlan3D plan =
        planFootstep3D(context, terrain, m_config);
    if (!plan.valid) return false;

    // Receding-horizon foothold: o alvo acompanha o DCM, mas a curva não
    // reinicia e a passada continua contando uma única vez.
    const float remaining = 1.0f - foot.progress;
    const float blend = std::clamp(
        0.18f + remaining * 0.34f, 0.18f, 0.52f);
    foot.swingTarget.position +=
        (plan.target.position - foot.swingTarget.position) * blend;
    foot.swingTarget.groundNormal =
        (foot.swingTarget.groundNormal
            + (plan.target.groundNormal - foot.swingTarget.groundNormal)
                * blend).normalized();
    foot.swingTarget.yawRadians += shortestAngleDelta3D(
        foot.swingTarget.yawRadians,
        plan.target.yawRadians) * blend;
    foot.swingTarget.orientation = footOrientation3D(
        foot.swingTarget.groundNormal, foot.swingTarget.yawRadians);
    foot.terrainQuality = plan.terrainQuality;
    buildSwingTrajectory();
    updateDebugState();
    return true;
}

void FootworkSystem3D::updateHomes(
    Vec3 desiredVelocity, bool fast) {
    const Vec3 forward {
        std::cos(m_rootYawRadians), std::sin(m_rootYawRadians), 0.0f
    };
    const Vec3 left { -forward.y, forward.x, 0.0f };
    const Vec3 anticipation = desiredVelocity
        * (fast
            ? m_config.fastVelocityLookAheadSeconds
            : m_config.velocityLookAheadSeconds);
    for (FootSide3D side : { FootSide3D::Left, FootSide3D::Right }) {
        const float sign = side == FootSide3D::Left ? 1.0f : -1.0f;
        RuntimeFoot& foot = m_feet[footIndex3D(side)];
        foot.home.position = m_rootGroundPosition
            + left * (sign * m_config.stanceWidthMeters * 0.5f)
            + forward * m_config.homeForwardOffsetMeters
            + anticipation;
        foot.home.position.z = foot.pose.position.z;
        foot.home.groundNormal = foot.pose.groundNormal;
        foot.home.yawRadians = m_rootYawRadians;
        foot.home.orientation = footOrientation3D(
            foot.home.groundNormal, foot.home.yawRadians);
    }
}

bool FootworkSystem3D::beginStep(FootSide3D side,
    const FootworkTerrainProbe3D& terrain, Vec3 desiredVelocity,
    Vec2 movementLocal, bool fast) {
    RuntimeFoot& foot = m_feet[footIndex3D(side)];
    const RuntimeFoot& support =
        m_feet[footIndex3D(oppositeFoot3D(side))];
    FootstepPlanningContext3D context;
    context.side = side;
    context.desiredHome = foot.home;
    context.supportFoot = support.pose;
    context.desiredVelocity = desiredVelocity;
    context.rootYawRadians = m_rootYawRadians;
    const FootstepPlan3D plan =
        planFootstep3D(context, terrain, m_config);
    if (!plan.valid) {
        m_debug.terrainBlocked = true;
        return false;
    }

    foot.swingStart = foot.pose;
    foot.swingTarget = plan.target;
    foot.terrainQuality = plan.terrainQuality;
    foot.progress = 0.0f;
    foot.phase = FootPhase3D::LiftOff;
    foot.fastStep = fast;
    foot.physicallyReleased = false;
    foot.clearanceMeters = fast
        ? m_config.fastSwingClearanceMeters
        : m_config.swingClearanceMeters;
    const float distance = (foot.swingTarget.position
        - foot.swingStart.position).length();
    foot.durationSeconds = swingDurationForDistance(
        distance, m_config, fast);
    foot.weightTransferElapsedSeconds = 0.0f;
    foot.weightTransferDurationSeconds =
        fast ? 0.0f : m_config.weightTransferDurationSeconds;
    m_activeSide = side;
    m_hasActiveStep = true;
    m_activeRecoveryStep = false;
    m_targetChaseAccumulatorSeconds = 0.0f;
    m_activePlanVelocity = desiredVelocity;
    m_activePlanMovementLocal = movementLocal;
    m_activePlanYawRadians = m_rootYawRadians;
    m_debug.terrainBlocked = false;
    if (std::getenv("MATTERENGINE_FOOTWORK_EVENTS") != nullptr) {
        std::cerr << "[footwork] normal begin side="
                  << static_cast<int>(side) << " count="
                  << m_completedStepCount << " target=("
                  << foot.swingTarget.position.x << ","
                  << foot.swingTarget.position.y << ")\n";
    }
    buildSwingTrajectory();
    return true;
}

bool FootworkSystem3D::replanActiveStep(
    const FootworkTerrainProbe3D& terrain, Vec3 desiredVelocity,
    Vec2 movementLocal, bool fast) {
    if (!m_hasActiveStep) return false;
    RuntimeFoot& foot = m_feet[footIndex3D(m_activeSide)];
    const RuntimeFoot& support =
        m_feet[footIndex3D(oppositeFoot3D(m_activeSide))];
    FootstepPlanningContext3D context;
    context.side = m_activeSide;
    context.desiredHome = foot.home;
    context.supportFoot = support.pose;
    context.desiredVelocity = desiredVelocity;
    context.rootYawRadians = m_rootYawRadians;
    const FootstepPlan3D plan =
        planFootstep3D(context, terrain, m_config);
    if (!plan.valid) return false;

    foot.swingStart = foot.pose;
    foot.swingTarget = plan.target;
    foot.terrainQuality = plan.terrainQuality;
    foot.progress = 0.0f;
    foot.phase = FootPhase3D::LiftOff;
    foot.fastStep = fast;
    foot.physicallyReleased = false;
    foot.clearanceMeters = fast
        ? m_config.fastSwingClearanceMeters
        : m_config.swingClearanceMeters;
    foot.durationSeconds = swingDurationForDistance(
        (foot.swingTarget.position - foot.swingStart.position).length(),
        m_config, fast);
    foot.weightTransferElapsedSeconds = 0.0f;
    foot.weightTransferDurationSeconds =
        fast ? 0.0f : m_config.weightTransferDurationSeconds;
    m_targetChaseAccumulatorSeconds = 0.0f;
    m_activePlanVelocity = desiredVelocity;
    m_activePlanMovementLocal = movementLocal;
    m_activePlanYawRadians = m_rootYawRadians;
    m_debug.terrainBlocked = false;
    if (std::getenv("MATTERENGINE_FOOTWORK_EVENTS") != nullptr) {
        std::cerr << "[footwork] replan side="
                  << static_cast<int>(m_activeSide) << " count="
                  << m_completedStepCount << '\n';
    }
    buildSwingTrajectory();
    return true;
}

void FootworkSystem3D::enforceActiveFootSeparation(
    FootPose3D& pose) const {
    if (!m_hasActiveStep) return;
    const FootPose3D& support =
        m_feet[footIndex3D(oppositeFoot3D(m_activeSide))].pose;
    Vec3 separation = pose.position - support.position;
    separation.z = 0.0f;
    const float minimumDistance =
        m_config.footWidthMeters + m_config.antiCrossingMarginMeters;
    const float distance = horizontalLength(separation);
    if (distance >= minimumDistance) return;

    if (distance < 1.0e-5f) {
        const Vec3 forward {
            std::cos(m_activePlanYawRadians),
            std::sin(m_activePlanYawRadians), 0.0f
        };
        const Vec3 left { -forward.y, forward.x, 0.0f };
        separation = left * (
            m_activeSide == FootSide3D::Left ? 1.0f : -1.0f);
    } else {
        separation *= 1.0f / distance;
    }
    pose.position.x = support.position.x
        + separation.x * minimumDistance;
    pose.position.y = support.position.y
        + separation.y * minimumDistance;
}

void FootworkSystem3D::updateActiveStep(const FootworkInput3D& input,
    float deltaTime, const FootworkTerrainProbe3D& terrain,
    Vec3 desiredVelocity) {
    RuntimeFoot& foot = m_feet[footIndex3D(m_activeSide)];
    const RuntimeFoot& support =
        m_feet[footIndex3D(oppositeFoot3D(m_activeSide))];
    const bool inputMoving =
        input.movementLocal.x * input.movementLocal.x
            + input.movementLocal.y * input.movementLocal.y > 0.0025f;
    bool aborting = foot.phase == FootPhase3D::Aborting;

    const float desiredSpeed = std::sqrt(
        input.movementLocal.x * input.movementLocal.x
        + input.movementLocal.y * input.movementLocal.y);
    const float plannedSpeed = std::sqrt(
        m_activePlanMovementLocal.x * m_activePlanMovementLocal.x
        + m_activePlanMovementLocal.y * m_activePlanMovementLocal.y);
    float directionAgreement = 1.0f;
    if (desiredSpeed > 0.05f && plannedSpeed > 0.05f) {
        directionAgreement =
            (input.movementLocal.x * m_activePlanMovementLocal.x
                + input.movementLocal.y * m_activePlanMovementLocal.y)
            / (desiredSpeed * plannedSpeed);
    }
    const bool commandDirectionChanged =
        desiredSpeed > 0.05f
        && (plannedSpeed <= 0.05f || directionAgreement < 0.55f);

    // Uma mudança brusca de comando invalida a passada anterior. Replanejar
    // a partir da pose atual evita o "último passo para a direção velha".
    // Girar a câmera mantendo o mesmo WASD não reinicia o relógio da passada:
    // o alvo acompanha a curva gradualmente no bloco de target-chase abaixo.
    if (!m_activeRecoveryStep
        && inputMoving && foot.progress < 0.88f
        && commandDirectionChanged
        && replanActiveStep(terrain, desiredVelocity,
            input.movementLocal, input.fast)) {
        aborting = false;
    } else if (!m_activeRecoveryStep
        && !inputMoving && m_inputWasMoving
        && foot.progress < 0.82f) {
        FootworkTerrainPatch3D safePatch = sampleFootworkTerrainPatch3D(
            terrain, foot.pose.position, foot.pose.yawRadians, m_config);
        if (!safePatch.valid) {
            safePatch = sampleFootworkTerrainPatch3D(
                terrain, foot.swingTarget.position,
                foot.swingTarget.yawRadians, m_config);
        }
        if (safePatch.valid) {
            foot.swingStart = foot.pose;
            foot.swingTarget.position = safePatch.position;
            foot.swingTarget.groundNormal = safePatch.normal;
            foot.swingTarget.orientation = footOrientation3D(
                safePatch.normal, foot.swingTarget.yawRadians);
            foot.progress = 0.0f;
            foot.durationSeconds = m_config.abortDurationSeconds;
            foot.phase = FootPhase3D::Aborting;
            aborting = true;
            m_activePlanVelocity = {};
            m_activePlanMovementLocal = {};
            m_activePlanYawRadians = m_rootYawRadians;
            buildSwingTrajectory();
        }
    } else {
        m_targetChaseAccumulatorSeconds += deltaTime;
    }
    constexpr float TargetChaseIntervalSeconds = 1.0f / 30.0f;
    if (!aborting && !m_activeRecoveryStep
        && foot.progress > 0.02f && foot.progress < 0.64f
        && m_config.targetChaseStrength > 0.0f
        && m_targetChaseAccumulatorSeconds
            >= TargetChaseIntervalSeconds) {
        m_targetChaseAccumulatorSeconds = std::fmod(
            m_targetChaseAccumulatorSeconds,
            TargetChaseIntervalSeconds);
        // A casa continua andando com a raiz. Perseguimos só parte do erro e
        // o ganho desaparece ao aterrissar; não existe foot sliding depois
        // do contato.
        FootPose3D chasedHome = foot.home;
        chasedHome.position += desiredVelocity
            * ((input.fast
                    ? m_config.fastVelocityLookAheadSeconds
                    : m_config.velocityLookAheadSeconds)
                * (1.0f - foot.progress)
                * m_config.targetChaseStrength);
        FootstepPlanningContext3D context;
        context.side = m_activeSide;
        context.desiredHome = chasedHome;
        context.supportFoot = support.pose;
        context.desiredVelocity = desiredVelocity;
        context.rootYawRadians = m_rootYawRadians;
        const FootstepPlan3D chased =
            planFootstep3D(context, terrain, m_config);
        if (chased.valid) {
            const float chaseStrength = input.fast
                ? std::max(0.52f, m_config.targetChaseStrength)
                : m_config.targetChaseStrength;
            const float blend = std::clamp(
                deltaTime * 10.0f * chaseStrength,
                0.0f, input.fast ? 0.22f : 0.16f);
            foot.swingTarget.position +=
                (chased.target.position - foot.swingTarget.position) * blend;
            foot.swingTarget.groundNormal =
                (foot.swingTarget.groundNormal
                    + (chased.target.groundNormal
                        - foot.swingTarget.groundNormal) * blend).normalized();
            foot.swingTarget.yawRadians += shortestAngleDelta3D(
                foot.swingTarget.yawRadians,
                chased.target.yawRadians) * blend;
            foot.swingTarget.orientation = footOrientation3D(
                foot.swingTarget.groundNormal,
                foot.swingTarget.yawRadians);
            m_activePlanVelocity = desiredVelocity;
            m_activePlanYawRadians = m_rootYawRadians;
        }
    }

    if (input.fast && !foot.fastStep && !aborting) {
        foot.fastStep = true;
        foot.clearanceMeters = m_config.fastSwingClearanceMeters;
        const float fastDuration = swingDurationForDistance(
            (foot.swingTarget.position - foot.swingStart.position).length(),
            m_config, true);
        foot.durationSeconds = std::min(
            foot.durationSeconds, fastDuration);
    }

    if (!aborting && foot.weightTransferElapsedSeconds
            < foot.weightTransferDurationSeconds) {
        foot.weightTransferElapsedSeconds = std::min(
            foot.weightTransferDurationSeconds,
            foot.weightTransferElapsedSeconds + deltaTime);
        foot.progress = 0.0f;
        foot.phase = FootPhase3D::LiftOff;
        foot.pose = foot.swingStart;
        buildSwingTrajectory();
        updateDebugState();
        return;
    }
    if (!aborting && foot.weightTransferDurationSeconds > 1.0e-5f
        && foot.progress <= 1.0e-5f
        && !m_activeStepReleaseAllowed) {
        // A curva de transferência chegou ao fim, mas o corpo físico ainda
        // não está dinamicamente sobre o próximo apoio. Permanecer em apoio
        // duplo é a única resposta biomecanicamente válida; levantar o pé por
        // relógio cria uma queda que nenhum ganho posterior pode consertar.
        // Esta trava só vale antes do toe-off: depois que progress saiu de zero,
        // revogar a autorização teleportaria uma perna já em voo de volta ao
        // início da curva e destruiria a conservação de momento.
        foot.progress = 0.0f;
        foot.phase = FootPhase3D::LiftOff;
        foot.pose = foot.swingStart;
        buildSwingTrajectory();
        updateDebugState();
        return;
    }

    foot.progress = std::min(1.0f, foot.progress
        + deltaTime / std::max(0.001f, foot.durationSeconds));
    foot.phase = phaseForProgress(
        foot.progress, m_config, aborting);
    foot.pose = interpolateFootSwing3D(foot.swingStart,
        foot.swingTarget, foot.progress, foot.clearanceMeters);
    const bool groundShuffle = m_activeRecoveryStep
        && foot.clearanceMeters <= 0.001f;
    if (m_activeRecoveryStep && !aborting) {
        // Uma passada reflexiva primeiro intercepta horizontalmente a queda
        // e só então assenta a planta. A ciclóide nominal percorre apenas
        // ~65% do avanço aos 58% da fase; como a sola já está descendo nesse
        // instante, o primeiro contato acontecia 10--15 cm antes do ponto de
        // captura. Mantemos o arco vertical suave, mas comprimimos somente a
        // progressão XY. Não há teleporte: o WBC ainda acompanha esta curva
        // por torques e o contato real continua decidindo o heel-strike.
        const float horizontalArrival = groundShuffle ? 0.62f : 0.72f;
        const float horizontalProgress = cycloidProgress3D(
            std::clamp(foot.progress / horizontalArrival, 0.0f, 1.0f));
        foot.pose.position.x = foot.swingStart.position.x
            + (foot.swingTarget.position.x - foot.swingStart.position.x)
                * horizontalProgress;
        foot.pose.position.y = foot.swingStart.position.y
            + (foot.swingTarget.position.y - foot.swingStart.position.y)
                * horizontalProgress;
    }
    const std::size_t activeIndex = footIndex3D(m_activeSide);
    if (!groundShuffle && m_hasPhysicalContactFeedback) {
        const bool visiblyAboveLiftOff = m_hasPhysicalFootPose
            && m_physicalFootPose[activeIndex].position.z
                > foot.swingStart.position.z + 0.022f;
        foot.physicallyReleased = foot.physicallyReleased
            || !m_physicalFootSupported[activeIndex]
            || visiblyAboveLiftOff;
    }
    const float pitchRadians = groundShuffle ? 0.0f
        : swingPitchForProgress(
            foot.progress, m_config, foot.fastStep && !aborting);
    applySwingPitch(foot.pose, pitchRadians, m_config);
    enforceActiveFootSeparation(foot.pose);
    const float physicalLandingError = m_hasPhysicalFootPose
        ? horizontalLength(m_physicalFootPose[activeIndex].position
            - foot.swingTarget.position)
        : 0.0f;
    // Contato só encerra a passada quando a sola física chegou perto do
    // foothold. Antes, qualquer toque nos 20% finais fazia o planner
    // teleportar seu estado interno para o alvo e a passada seguinte era
    // calculada sobre um pé fantasma.
    // Em uma passada reflexiva o alvo continua recuando enquanto o tronco
    // viaja. A sola pode, portanto, criar um contato perfeitamente utilizavel
    // alguns centimetros antes do ponto de captura calculado no frame
    // anterior. Exigir a tolerancia estreita da marcha nominal mantinha um pe
    // que ja sustentava peso classificado como "em voo" ate o outro apoio
    // ceder. Aceitamos uma janela de um comprimento de sola apenas para a
    // recuperacao; a pose observada (e nao o alvo antigo) vira o novo apoio.
    const float landingToleranceMeters = m_activeRecoveryStep
        ? 0.18f : 0.14f;
    const bool physicalFootholdReached = !m_hasPhysicalFootPose
        || physicalLandingError <= landingToleranceMeters;
    const float minimumTouchdownProgress = m_activeRecoveryStep
        ? (groundShuffle ? 0.72f : 0.58f)
        : 1.0f - m_config.touchDownFraction;
    const bool completedPhysicalFlight = groundShuffle
        || !m_hasPhysicalContactFeedback
        || foot.physicallyReleased;
    const bool soleNearTerrain = !m_hasPhysicalFootPose
        || m_physicalFootPose[activeIndex].position.z
            <= foot.swingTarget.position.z + 0.038f;
    const bool confirmedEarlyTouchdown =
        m_hasPhysicalContactFeedback
        && m_physicalFootSupported[activeIndex]
        && completedPhysicalFlight
        && soleNearTerrain
        && physicalFootholdReached
        && foot.progress >= minimumTouchdownProgress;
    if (confirmedEarlyTouchdown) {
        // O contato físico tem prioridade sobre o relógio da curva. Continuar
        // tratando a sola já apoiada como tarefa cartesiana de voo fazia o WBC
        // empurrá-la contra o piso durante os últimos milissegundos, criando
        // impulso lateral e oscilação de apoio.
        foot.pose = m_hasPhysicalFootPose
            ? m_physicalFootPose[activeIndex] : foot.swingTarget;
        enforceActiveFootSeparation(foot.pose);
        foot.phase = FootPhase3D::Planted;
        foot.progress = 0.0f;
        m_hasActiveStep = false;
        m_activeRecoveryStep = false;
        m_targetChaseAccumulatorSeconds = 0.0f;
        m_lastCompletedSide = m_activeSide;
        ++m_completedStepCount;
        if (std::getenv("MATTERENGINE_FOOTWORK_EVENTS") != nullptr) {
            std::cerr << "[footwork] early touchdown side="
                      << static_cast<int>(m_activeSide) << " count="
                      << m_completedStepCount << '\n';
        }
        return;
    }
    if (foot.progress >= 1.0f) {
        foot.pose = foot.swingTarget;
        enforceActiveFootSeparation(foot.pose);
        if (m_hasPhysicalContactFeedback
            && (!m_physicalFootSupported[activeIndex]
                || !physicalFootholdReached)) {
            foot.progress = 1.0f;
            foot.phase = FootPhase3D::TouchDown;
            buildSwingTrajectory();
            updateDebugState();
            return;
        }
        if (m_hasPhysicalFootPose) {
            foot.pose = m_physicalFootPose[activeIndex];
        }
        foot.phase = FootPhase3D::Planted;
        foot.progress = 0.0f;
        m_hasActiveStep = false;
        m_activeRecoveryStep = false;
        m_targetChaseAccumulatorSeconds = 0.0f;
        m_lastCompletedSide = m_activeSide;
        ++m_completedStepCount;
        if (std::getenv("MATTERENGINE_FOOTWORK_EVENTS") != nullptr) {
            std::cerr << "[footwork] touchdown side="
                      << static_cast<int>(m_activeSide) << " count="
                      << m_completedStepCount << '\n';
        }
    }
}

void FootworkSystem3D::update(const FootworkInput3D& rawInput,
    float deltaTime, const FootworkTerrainProbe3D& terrain) {
    if (!m_debug.initialized || deltaTime <= 0.0f) return;
    sanitizeConfig();
    FootworkInput3D input = rawInput;
    const float inputLength = std::sqrt(
        input.movementLocal.x * input.movementLocal.x
        + input.movementLocal.y * input.movementLocal.y);
    if (inputLength > 1.0f) {
        input.movementLocal.x /= inputLength;
        input.movementLocal.y /= inputLength;
    }

    m_rootYawRadians = moveAngleTowards3D(m_rootYawRadians,
        input.lookYawRadians,
        m_config.maximumRootYawRateRadiansPerSecond * deltaTime);
    const Vec3 forward {
        std::cos(input.lookYawRadians), std::sin(input.lookYawRadians), 0.0f
    };
    const Vec3 right { forward.y, -forward.x, 0.0f };
    Vec3 desiredDirection = forward * input.movementLocal.y
        + right * input.movementLocal.x;
    if (desiredDirection.lengthSquared() > 1.0f) {
        desiredDirection = desiredDirection.normalized();
    }
    const float desiredSpeed = input.fast
        ? m_config.fastSpeedMetersPerSecond
        : m_config.walkSpeedMetersPerSecond;
    const Vec3 desiredVelocity = desiredDirection * desiredSpeed;
    float acceleration = desiredDirection.lengthSquared() > 0.0001f
        ? m_config.accelerationMetersPerSecondSquared
        : m_config.decelerationMetersPerSecondSquared;
    if (input.fast && desiredDirection.lengthSquared() > 0.0001f) {
        acceleration *= m_config.fastAccelerationMultiplier;
    }
    m_rootVelocity = approachVector(m_rootVelocity, desiredVelocity,
        acceleration * deltaTime);

    const Vec3 supportCenter = (m_feet[0].pose.position
        + m_feet[1].pose.position) * 0.5f;
    const Vec3 supportAnchor = m_hasActiveStep
        ? m_feet[footIndex3D(
            oppositeFoot3D(m_activeSide))].pose.position
        : supportCenter;
    Vec3 proposedRoot = m_rootGroundPosition
        + m_rootVelocity * deltaTime;
    Vec3 supportOffset = proposedRoot - supportAnchor;
    supportOffset.z = 0.0f;
    const float supportDistance = horizontalLength(supportOffset);
    if (supportDistance > m_config.rootSupportLimitMeters) {
        const Vec3 supportDirection =
            supportOffset * (1.0f / supportDistance);
        proposedRoot.x = supportAnchor.x
            + supportDirection.x * m_config.rootSupportLimitMeters;
        proposedRoot.y = supportAnchor.y
            + supportDirection.y * m_config.rootSupportLimitMeters;
        const float outwardVelocity =
            dot(m_rootVelocity, supportDirection);
        if (outwardVelocity > 0.0f) {
            m_rootVelocity -= supportDirection * outwardVelocity;
        }
    }
    m_rootGroundPosition = proposedRoot;
    m_rootGroundPosition.z = supportCenter.z;
    updateHomes(desiredVelocity, input.fast);

    if (m_hasActiveStep) {
        const FootSide3D completingSide = m_activeSide;
        updateActiveStep(input, deltaTime, terrain, desiredVelocity);
        // Corrida usa apoio fugaz: no mesmo tick em que um pé assenta, o
        // contralateral inicia a próxima passada. Continua havendo sempre um
        // contato válido, mas não há a pausa artificial da marcha.
        if (!m_hasActiveStep && m_automaticSteppingEnabled
            && input.fast && inputLength > 0.05f) {
            static_cast<void>(beginStep(
                oppositeFoot3D(completingSide), terrain,
                desiredVelocity, input.movementLocal, true));
        }
    } else if (m_automaticSteppingEnabled) {
        std::array<float, 2> scores {};
        for (FootSide3D side : { FootSide3D::Left, FootSide3D::Right }) {
            RuntimeFoot& foot = m_feet[footIndex3D(side)];
            const Vec3 error = foot.home.position - foot.pose.position;
            const float distanceError = horizontalLength(error);
            const float yawError = std::abs(shortestAngleDelta3D(
                foot.pose.yawRadians, foot.home.yawRadians));
            scores[footIndex3D(side)] =
                distanceError / std::max(
                    0.01f, m_config.stepTriggerDistanceMeters)
                + yawError / std::max(
                    0.01f, m_config.stepTriggerYawRadians);
            if (side == m_lastCompletedSide) scores[footIndex3D(side)] -= 0.36f;
        }
        const float turnDelta = shortestAngleDelta3D(
            (m_feet[0].pose.yawRadians + m_feet[1].pose.yawRadians) * 0.5f,
            m_rootYawRadians);
        if (turnDelta > 0.0f) {
            // Virada à esquerda: o pé direito é o contralateral.
            scores[footIndex3D(FootSide3D::Right)] +=
                std::min(0.55f, std::abs(turnDelta));
        } else {
            scores[footIndex3D(FootSide3D::Left)] +=
                std::min(0.55f, std::abs(turnDelta));
        }
        const FootSide3D candidate =
            scores[0] >= scores[1] ? FootSide3D::Left : FootSide3D::Right;
        RuntimeFoot& foot = m_feet[footIndex3D(candidate)];
        const float distanceError = horizontalLength(
            foot.home.position - foot.pose.position);
        const float yawError = std::abs(shortestAngleDelta3D(
            foot.pose.yawRadians, foot.home.yawRadians));
        const bool moving = inputLength > 0.05f
            || m_rootVelocity.lengthSquared() > 0.0025f;
        const bool needsStep =
            (distanceError > m_config.stepTriggerDistanceMeters
                && moving)
            || yawError > m_config.stepTriggerYawRadians;
        if (needsStep) {
            static_cast<void>(beginStep(
                candidate, terrain, desiredVelocity,
                input.movementLocal, input.fast));
        } else if (distanceError < m_config.stepReleaseDistanceMeters
            && yawError < m_config.stepTriggerYawRadians * 0.55f) {
            m_debug.terrainBlocked = false;
        }
    } else {
        m_debug.terrainBlocked = false;
    }
    m_inputWasMoving = inputLength > 0.05f;
    updateDebugState();
}

void FootworkSystem3D::buildSwingTrajectory() {
    m_debug.swingTrajectoryCount = 0;
    if (!m_hasActiveStep) return;
    const RuntimeFoot& foot = m_feet[footIndex3D(m_activeSide)];
    for (std::size_t index = 0;
            index < FootworkTrajectorySampleCount3D; ++index) {
        const float progress = static_cast<float>(index)
            / static_cast<float>(FootworkTrajectorySampleCount3D - 1);
        FootPose3D trajectoryPose = interpolateFootSwing3D(
            foot.swingStart, foot.swingTarget, progress,
            foot.clearanceMeters);
        applySwingPitch(trajectoryPose,
            swingPitchForProgress(
                progress, m_config, foot.fastStep),
            m_config);
        enforceActiveFootSeparation(trajectoryPose);
        m_debug.swingTrajectory[index] = trajectoryPose.position;
    }
    m_debug.swingTrajectoryCount = FootworkTrajectorySampleCount3D;
}

void FootworkSystem3D::rebuildSupportPolygon() {
    std::array<Vec3, FootworkMaximumSupportVertices3D> points {};
    std::size_t pointCount = 0;
    for (FootSide3D side : { FootSide3D::Left, FootSide3D::Right }) {
        if (m_hasActiveStep && side == m_activeSide
            && m_feet[footIndex3D(side)].phase
                != FootPhase3D::TouchDown) {
            continue;
        }
        const FootPose3D& pose = m_feet[footIndex3D(side)].pose;
        const Vec3 forward = pose.orientation.rotate({ 1.0f, 0.0f, 0.0f });
        const Vec3 left = pose.orientation.rotate({ 0.0f, 1.0f, 0.0f });
        const float halfLength = m_config.footLengthMeters * 0.5f;
        const float halfWidth = m_config.footWidthMeters * 0.5f;
        points[pointCount++] = pose.position
            + forward * halfLength + left * halfWidth;
        points[pointCount++] = pose.position
            + forward * halfLength - left * halfWidth;
        points[pointCount++] = pose.position
            - forward * halfLength - left * halfWidth;
        points[pointCount++] = pose.position
            - forward * halfLength + left * halfWidth;
    }
    if (pointCount <= 2) {
        m_debug.supportPolygonCount = pointCount;
        std::copy_n(points.begin(), pointCount,
            m_debug.supportPolygon.begin());
        return;
    }

    std::sort(points.begin(), points.begin() + pointCount,
        [](Vec3 a, Vec3 b) {
            return a.x < b.x || (a.x == b.x && a.y < b.y);
        });
    std::array<Vec3, FootworkMaximumSupportVertices3D * 2> hull {};
    std::size_t count = 0;
    for (std::size_t index = 0; index < pointCount; ++index) {
        const Vec3 point = points[index];
        while (count >= 2
            && cross2D(hull[count - 2], hull[count - 1], point) <= 0.0f) {
            --count;
        }
        hull[count++] = point;
    }
    const std::size_t lowerCount = count;
    for (std::size_t reverseIndex = pointCount - 1;
            reverseIndex > 0; --reverseIndex) {
        const Vec3 point = points[reverseIndex - 1];
        while (count > lowerCount
            && cross2D(hull[count - 2], hull[count - 1], point) <= 0.0f) {
            --count;
        }
        hull[count++] = point;
    }
    if (count > 1) --count;
    m_debug.supportPolygonCount = std::min(
        count, FootworkMaximumSupportVertices3D);
    std::copy_n(hull.begin(), m_debug.supportPolygonCount,
        m_debug.supportPolygon.begin());
}

void FootworkSystem3D::updateDebugState() {
    m_debug.initialized = true;
    m_debug.rootGroundPosition = m_rootGroundPosition;
    m_debug.rootVelocity = m_rootVelocity;
    m_debug.rootYawRadians = m_rootYawRadians;
    m_debug.activeSwingSide = m_activeSide;
    m_debug.hasActiveSwing = m_hasActiveStep;
    m_debug.completedStepCount = m_completedStepCount;
    for (FootSide3D side : { FootSide3D::Left, FootSide3D::Right }) {
        const RuntimeFoot& source = m_feet[footIndex3D(side)];
        FootworkFootState3D& destination =
            m_debug.feet[footIndex3D(side)];
        destination.pose = source.pose;
        destination.home = source.home;
        destination.target = source.swingTarget;
        destination.phase = source.phase;
        destination.phaseProgress = source.progress;
        destination.weightTransferProgress =
            source.weightTransferDurationSeconds <= 1.0e-5f
            ? 1.0f
            : std::clamp(
                source.weightTransferElapsedSeconds
                    / source.weightTransferDurationSeconds,
                0.0f, 1.0f);
        destination.homeErrorMeters = horizontalLength(
            source.home.position - source.pose.position);
        destination.homeYawErrorRadians = std::abs(
            shortestAngleDelta3D(
                source.pose.yawRadians, source.home.yawRadians));
        destination.terrainQuality = source.terrainQuality;
    }
    if (m_hasActiveStep) buildSwingTrajectory();
    else m_debug.swingTrajectoryCount = 0;
    rebuildSupportPolygon();
}

} // namespace MatterEngine
