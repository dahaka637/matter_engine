// Matter Engine clean-room physical biped controller.
// THEORY: [R01] SIMBICON torso/COM feedback; [R02/R04] GBWC/VMC velocity tuning.
// This module emits task-space intents only; it does not apply external balance forces.

#include "Engine/Control/BipedBalanceController3D.hpp"

#include "Engine/Control/BipedMath3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {

BipedBalanceDecision3D BipedBalanceController3D::assess(
    const BipedRig3D& rig,
    const BipedObservation3D& observation,
    const BipedIntent3D& intent) const {
    BipedBalanceDecision3D result;
    if (!rig.valid || !observation.valid) {
        result.strategy = BipedBalanceStrategy3D::Fall;
        result.urgency = 1.0f;
        return result;
    }

    if (observation.support.mode == BipedSupportMode3D::Airborne
        || observation.support.mode == BipedSupportMode3D::MultiContactBody) {
        result.strategy = BipedBalanceStrategy3D::Fall;
        result.urgency = 1.0f;
        return result;
    }

    const float leg = std::max(rig.legLengthMeters, 0.25f);
    const float safe =
        leg*m_settings.captureSafeMarginLegFraction;
    const float stepBoundary =
        leg*m_settings.captureStepMarginLegFraction;
    const float margin = observation.captureMarginMeters;

    if (margin >= safe) {
        result.strategy = BipedBalanceStrategy3D::AnkleHip;
        result.urgency = 0.0f;
    } else if (margin >= stepBoundary) {
        result.strategy = BipedBalanceStrategy3D::AnkleHip;
        result.urgency = std::clamp(
            (safe-margin)/std::max(safe-stepBoundary, 0.001f),
            0.0f, 1.0f);
    } else if (intent.allowStepping) {
        result.strategy = BipedBalanceStrategy3D::Step;
        result.mustStep = true;
        result.urgency = std::clamp(
            0.65f+(-margin+std::abs(stepBoundary))/leg,
            0.0f, 1.0f);
    } else {
        result.strategy = BipedBalanceStrategy3D::Fall;
        result.urgency = 1.0f;
    }

    if (observation.torsoUpDot < 0.50f
        && observation.bodyGroundContact) {
        result.strategy = BipedBalanceStrategy3D::Fall;
        result.mustStep = false;
        result.urgency = 1.0f;
    }
    return result;
}

BipedBalanceCommand3D BipedBalanceController3D::command(
    const RagdollProfile3D& profile,
    const BipedRig3D& rig,
    const RagdollState3D& state,
    const BipedObservation3D& observation,
    const BipedIntent3D& intent,
    const BipedStepPlan3D& step,
    const BipedBalanceDecision3D& decision) const {
    BipedBalanceCommand3D result;
    result.strategy = decision.strategy;
    result.urgency = decision.urgency;
    if (!observation.valid || !rig.valid
        || rig.torso >= state.links.size()) {
        return result;
    }

    const Quaternion heading =
        yawQuaternionBiped(intent.desiredHeadingRadians);
    Vec3 desiredVelocityWorld =
        heading.rotate(intent.desiredVelocityBody);
    desiredVelocityWorld.z = 0.0f;
    if (!intent.locomotionEnabled) desiredVelocityWorld = {};
    // MATTER ADAPTATION: keep the COM over the measured stance contact
    // during weight transfer. Forward velocity tuning begins after physical
    // liftoff; otherwise the body can accelerate beyond its support polygon
    // while both feet still carry half the load.
    if (step.phase == BipedStepPhase3D::Transfer)
        desiredVelocityWorld = {};

    Vec3 targetCom = observation.support.centerWorld;
    targetCom.z = observation.centerOfMassWorld.z;
    if (step.phase == BipedStepPhase3D::Transfer
        || step.phase == BipedStepPhase3D::Swing) {
        targetCom = step.desiredComWorld;
        targetCom.z = observation.centerOfMassWorld.z;
    }

    Vec3 positionErrorBody =
        heading.conjugate().rotate(
            targetCom-observation.centerOfMassWorld);
    Vec3 velocityErrorBody =
        heading.conjugate().rotate(
            desiredVelocityWorld-observation.centerOfMassVelocityWorld);
    positionErrorBody.z = 0.0f;
    velocityErrorBody.z = 0.0f;

    Vec3 accelerationBody;
    const bool transferring =
        step.phase == BipedStepPhase3D::Transfer
        || step.phase == BipedStepPhase3D::Swing;
    const float positionGain = transferring
        ? m_settings.transferPositionGainPerSecondSquared
        : m_settings.standingPositionGainPerSecondSquared;

    accelerationBody.x =
        m_settings.sagittalVelocityGainPerSecond*velocityErrorBody.x;
    if (!intent.locomotionEnabled)
        accelerationBody.x += positionGain*positionErrorBody.x;
    else if (transferring)
        accelerationBody.x += positionGain*0.25f*positionErrorBody.x;

    accelerationBody.y =
        positionGain*positionErrorBody.y
        +m_settings.coronalVelocityGainPerSecond*velocityErrorBody.y;

    Vec3 forceWorld =
        heading.rotate(accelerationBody)*profile.totalMassKg;
    const float maximumForce =
        profile.totalMassKg*9.81f
        *m_settings.maximumHorizontalForceWeightFraction;
    result.desiredComForceWorld =
        clampLengthBiped(forceWorld, maximumForce);

    const auto& torsoDef = profile.links[rig.torso];
    const auto& torsoState = state.links[rig.torso];
    result.desiredTorsoOrientationWorld =
        bipedNeutralLinkOrientationWorld3D(
            torsoDef, intent.desiredHeadingRadians,
            {0.0f, 0.0f, 1.0f});
    const Vec3 error = orientationErrorBiped(
        result.desiredTorsoOrientationWorld,
        torsoState.orientation);

    const float height = std::max(
        profile.standingRootHeightMeters, 0.40f);
    const float inertia =
        std::max(0.01f,
            m_settings.torsoInertiaMassHeightSquaredFraction
            *profile.totalMassKg*height*height);
    const float wn =
        std::max(0.1f,
            m_settings.torsoNaturalFrequencyRadiansPerSecond);
    const float kp = inertia*wn*wn;
    const float kd =
        2.0f*std::max(0.0f, m_settings.torsoDampingRatio)
        *inertia*wn;
    Vec3 torque =
        error*kp-torsoState.angularVelocity*kd;
    const float maximumTorque =
        profile.totalMassKg*9.81f
        *std::max(rig.legLengthMeters, 0.25f)
        *m_settings.maximumTorsoTorqueWeightLengthFraction;
    result.desiredTorsoTorqueWorld =
        clampLengthBiped(torque, maximumTorque);
    return result;
}

} // namespace MatterEngine
