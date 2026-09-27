// Matter Engine production integration of the clean-room physical biped stack.
// Foundations: [R01] SIMBICON, [R02] GBWC, [R04] VMC, [R05/R06] capturability, [R13] PhysX.
// The retired balance/footwork/get-up/assist controller is not used as an algorithmic source.

#include "Engine/Control/PhysicalBipedController3D.hpp"

#include "Engine/Control/BipedMath3D.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace MatterEngine {
namespace {

Vec3 horizontal(Vec3 value) {
    value.z = 0.0f;
    return value;
}

float rootHeading(
    const RagdollProfile3D& profile,
    const BipedRig3D& rig,
    const RagdollState3D& state) {
    if (rig.pelvis >= profile.links.size()
        || rig.pelvis >= state.links.size()) return 0.0f;
    Vec3 forward = bipedAnatomicalAxisWorld3D(
        profile.links[rig.pelvis],
        state.links[rig.pelvis],
        {1.0f, 0.0f, 0.0f});
    forward.z = 0.0f;
    if (forward.lengthSquared() < 1.0e-8f) return 0.0f;
    return yawFromForwardBiped(forward);
}

Vec3 endEffectorOriginForSole(
    const RagdollLinkDefinition3D& link,
    Quaternion orientationWorld,
    Vec3 soleWorld) {
    const Vec3 localModelUp =
        link.modelOrientation.conjugate().rotate({0.0f, 0.0f, 1.0f});
    Vec3 groundNormal =
        orientationWorld.rotate(localModelUp).normalized();
    if (groundNormal.lengthSquared() < 1.0e-8f)
        groundNormal = {0.0f, 0.0f, 1.0f};
    const Vec3 offset = bipedColliderSupportOffsetLocal3D(
        link, orientationWorld, groundNormal);
    return soleWorld-orientationWorld.rotate(offset);
}

std::vector<BipedSupportContact3D> footContacts(
    const BipedRig3D& rig,
    const BipedObservation3D& observation) {
    std::vector<BipedSupportContact3D> result;
    for (const auto& contact : observation.support.contacts) {
        if (contact.linkIndex == rig.foot[0]
            || contact.linkIndex == rig.foot[1]) {
            result.push_back(contact);
        }
    }
    return result;
}

void disableLeg(std::vector<std::uint8_t>& enabled,
    const BipedRig3D& rig, int side) {
    if (side < 0 || side > 1) return;
    for (std::size_t link : {
            rig.foot[static_cast<std::size_t>(side)],
            rig.lowerLeg[static_cast<std::size_t>(side)],
            rig.upperLeg[static_cast<std::size_t>(side)]}) {
        if (link < enabled.size()) enabled[link] = 0;
    }
}

Vec3 getUpComForce(
    const RagdollProfile3D& profile,
    const BipedObservation3D& observation,
    const BipedGetUpCommand3D& command) {
    const Vec3 horizontalError = {
        observation.support.centerWorld.x-observation.centerOfMassWorld.x,
        observation.support.centerWorld.y-observation.centerOfMassWorld.y,
        0.0f
    };
    Vec3 acceleration =
        horizontalError*10.0f
        -horizontal(observation.centerOfMassVelocityWorld)*5.0f;
    const float zError =
        command.desiredComHeightWorld-observation.centerOfMassWorld.z;
    acceleration.z =
        10.0f*zError
        +5.0f*(command.desiredComVerticalVelocity
            -observation.centerOfMassVelocityWorld.z);
    acceleration.z = std::clamp(acceleration.z, -2.5f, 7.0f);
    acceleration = clampLengthBiped(acceleration, 8.0f);
    return acceleration*profile.totalMassKg;
}

Vec3 getUpTorsoTorque(
    const RagdollProfile3D& profile,
    const BipedRig3D& rig,
    const RagdollState3D& state,
    const BipedGetUpCommand3D& command) {
    if (rig.torso >= state.links.size()) return {};
    const auto& body = state.links[rig.torso];
    const Vec3 error = orientationErrorBiped(
        command.desiredTorsoOrientationWorld, body.orientation);
    const float h =
        std::max(profile.standingRootHeightMeters, 0.4f);
    const float inertia =
        std::max(0.01f, 0.12f*profile.totalMassKg*h*h);
    const float wn = 5.0f;
    const float kp = inertia*wn*wn;
    const float kd = 2.0f*inertia*wn;
    const float maxTorque =
        profile.totalMassKg*9.81f
        *std::max(rig.legLengthMeters, 0.25f)*0.40f;
    return clampLengthBiped(
        (error*kp-body.angularVelocity*kd)
            *command.desiredTorsoWeight,
        maxTorque);
}

float estimateActuatorDemandPeak(
    const RagdollProfile3D& profile,
    const RagdollState3D& state,
    const RagdollDynamics3D& dynamics,
    std::span<const RagdollDriveTarget3D> targets,
    bool gravityCompensationEnabled) {
    float peak = 0.0f;
    for (const auto& target : targets) {
        const std::size_t link = target.linkIndex;
        const std::size_t axis = static_cast<std::size_t>(target.axis);
        if (link == 0 || link >= profile.links.size()
            || link >= state.joints.size() || axis >= 3) continue;
        const auto& definition = profile.links[link].inboundJoint.axes[axis];
        if (!definition.enabled || definition.maximumTorque <= 1.0e-5f)
            continue;

        const float limit = definition.maximumTorque
            *std::max(0.0f, target.maximumTorqueScale);
        if (limit <= 1.0e-5f) continue;

        const float qError = target.positionRadians
            -state.joints[link].positionRadians[axis];
        const float vError = target.velocityRadiansPerSecond
            -state.joints[link].velocityRadiansPerSecond[axis];
        float demand = definition.stiffness
                *std::max(0.0f, target.stiffnessScale)
                *RagdollActiveDriveStiffnessMultiplier3D*qError
            +definition.damping
                *std::max(0.0f, target.dampingScale)
                *RagdollActiveDriveDampingMultiplier3D*vError
            +target.feedforwardTorqueNewtonMeters;

        if (gravityCompensationEnabled && dynamics.valid
            && link < dynamics.jointGeneralizedDof.size()) {
            const std::uint32_t gdof = dynamics.jointGeneralizedDof[link][axis];
            if (gdof != RagdollDynamics3D::InvalidIndex
                && gdof < dynamics.gravityCompensationForce.size()
                && std::isfinite(dynamics.gravityCompensationForce[gdof])) {
                demand += dynamics.gravityCompensationForce[gdof];
            }
        }
        peak = std::max(peak, std::abs(demand)/limit);
    }
    return peak;

}

}

void PhysicalBipedController3D::reset(
    const RagdollProfile3D& profile,
    const RagdollState3D& state) {
    m_rig = resolveBipedRig3D(profile);
    m_initialized = m_rig.valid
        && state.links.size() == profile.links.size()
        && state.joints.size() == profile.links.size()
        && !state.links.empty();
    m_output = {};
    m_telemetry = {};
    m_stepPlanner.reset();
    m_getUp.reset();
    m_residual.reset();
    m_fallenSeconds = 0.0f;
    m_airborneSeconds = 0.0f;
    m_stillSeconds = 0.0f;
    m_uprightSeconds = 0.0f;
    m_timedCommand = false;
    m_commandSecondsRemaining = 0.0f;
    m_intent = {};
    m_driveCoordinates.clear();
    if (!m_initialized) return;

    m_driveCoordinates =
        bipedJointCoordinatesFromState3D(profile, state);
    m_pelvisForwardLocal = profile.links[m_rig.pelvis].modelOrientation
        .conjugate().rotate({1.0f, 0.0f, 0.0f});
    m_intent.desiredHeadingRadians =
        rootHeading(profile, m_rig, state);
    m_telemetry.phase = BipedControllerPhase3D::Standing;
}

bool PhysicalBipedController3D::requestWalk(
    const RagdollState3D& state, float seconds, Vec3 directionBody) {
    if (!m_initialized
        || !std::isfinite(seconds) || seconds <= 0.0f
        || !finiteBiped(directionBody)
        || m_telemetry.phase == BipedControllerPhase3D::GetUp
        || m_telemetry.phase == BipedControllerPhase3D::Falling) {
        return false;
    }
    directionBody.z = 0.0f;
    if (directionBody.lengthSquared() < 1.0e-8f) return false;
    directionBody = directionBody.normalized();
    if (m_rig.pelvis < state.links.size()) {
        Vec3 forward = state.links[m_rig.pelvis].orientation
            .rotate(m_pelvisForwardLocal);
        forward.z = 0.0f;
        if (forward.lengthSquared() > 1.0e-8f)
            m_intent.desiredHeadingRadians = yawFromForwardBiped(forward);
    }
    m_intent.desiredVelocityBody =
        directionBody*m_settings.walkSpeedMetersPerSecond;
    m_intent.locomotionEnabled = true;
    m_intent.sprint = false;
    m_intent.allowStepping = true;
    m_timedCommand = true;
    m_commandSecondsRemaining =
        std::clamp(seconds, 0.05f, 120.0f);
    return true;
}

bool PhysicalBipedController3D::requestRun(
    const RagdollState3D& state, float seconds, Vec3 directionBody) {
    if (!requestWalk(state, seconds, directionBody)) return false;
    directionBody.z = 0.0f;
    directionBody = directionBody.normalized();
    m_intent.desiredVelocityBody =
        directionBody*m_settings.runSpeedMetersPerSecond;
    m_intent.sprint = true;
    return true;
}

void PhysicalBipedController3D::setIntent(
    const BipedIntent3D& intent) {
    if (!finiteBiped(intent.desiredVelocityBody)
        || !std::isfinite(intent.desiredHeadingRadians)) return;
    m_intent = intent;
    m_intent.desiredVelocityBody.z = 0.0f;
    m_timedCommand = false;
    m_commandSecondsRemaining = 0.0f;
}

void PhysicalBipedController3D::stop() {
    m_intent.desiredVelocityBody = {};
    m_intent.locomotionEnabled = false;
    m_intent.sprint = false;
    m_timedCommand = false;
    m_commandSecondsRemaining = 0.0f;
}

void PhysicalBipedController3D::enterPhase(
    BipedControllerPhase3D phase) {
    if (m_telemetry.phase == phase) return;
    m_telemetry.phase = phase;
    if (phase == BipedControllerPhase3D::Falling) {
        stop();
        m_stepPlanner.reset();
    }
    if (phase == BipedControllerPhase3D::GetUp) {
        stop();
        m_stepPlanner.reset();
        m_getUp.reset();
        m_residual.reset();
    }
}

void PhysicalBipedController3D::buildDriveTargets(
    const RagdollProfile3D& profile,
    const RagdollState3D& state,
    std::span<const Vec3> desiredCoordinates,
    const BipedJointTorqueField3D& feedforward,
    float deltaTime,
    float stiffnessScale,
    float torqueScale) {
    m_output.driveTargets.clear();
    if (m_driveCoordinates.size() != profile.links.size())
        m_driveCoordinates =
            bipedJointCoordinatesFromState3D(profile, state);

    float maxTargetSpeed =
        m_settings.maximumDriveTargetSpeedRadiansPerSecond;
    if (m_telemetry.phase == BipedControllerPhase3D::Swing)
        maxTargetSpeed =
            m_settings.swingDriveTargetSpeedRadiansPerSecond;
    else if (m_telemetry.phase == BipedControllerPhase3D::GetUp)
        maxTargetSpeed =
            m_settings.getUpDriveTargetSpeedRadiansPerSecond;

    const float muscle =
        std::clamp(m_settings.muscleStrength, 0.25f, 2.0f);
    const float stiffness =
        std::clamp(stiffnessScale*muscle, 0.0f, 2.0f);
    const float damping =
        std::clamp(
            std::sqrt(std::max(stiffnessScale*muscle, 0.0f)),
            0.0f, 2.5f);
    const float maximumTorqueScale =
        std::clamp(torqueScale*muscle, 0.0f, 3.0f);

    for (std::size_t link = 1; link < profile.links.size(); ++link) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const auto& definition =
                profile.links[link].inboundJoint.axes[axis];
            if (!definition.enabled) continue;
            const float desired =
                link < desiredCoordinates.size()
                    ? std::clamp(
                        componentBiped(desiredCoordinates[link], axis),
                        definition.minimumRadians,
                        definition.maximumRadians)
                    : 0.0f;
            float currentTarget =
                componentBiped(m_driveCoordinates[link], axis);
            const float delta = desired-currentTarget;
            const float maximumDelta =
                std::max(0.0f, maxTargetSpeed)*deltaTime;
            const float appliedDelta =
                std::clamp(delta, -maximumDelta, maximumDelta);
            currentTarget = std::clamp(
                currentTarget+appliedDelta,
                definition.minimumRadians,
                definition.maximumRadians);
            setComponentBiped(
                m_driveCoordinates[link], axis, currentTarget);

            float ff = 0.0f;
            if (link < feedforward.torqueNewtonMeters.size())
                ff = feedforward.torqueNewtonMeters[link][axis];
            if (!std::isfinite(ff)) ff = 0.0f;
            const float ffLimit =
                definition.maximumTorque*maximumTorqueScale;
            ff = std::clamp(ff, -ffLimit, ffLimit);

            RagdollDriveTarget3D target;
            target.linkIndex = static_cast<std::uint32_t>(link);
            target.axis = static_cast<RagdollAxis3D>(axis);
            target.positionRadians = currentTarget;
            target.velocityRadiansPerSecond =
                deltaTime > 0.0f ? appliedDelta/deltaTime : 0.0f;
            target.feedforwardTorqueNewtonMeters = ff;
            target.stiffnessScale = stiffness;
            target.dampingScale = damping;
            target.maximumTorqueScale = maximumTorqueScale;
            m_output.driveTargets.push_back(target);
        }
    }
}

void PhysicalBipedController3D::updateNormalControl(
    const RagdollProfile3D& profile,
    const RagdollState3D& state,
    const RagdollDynamics3D& dynamics,
    const BipedTerrainQuery3D& terrain,
    const BipedObservation3D& observation,
    float deltaTime) {
    const BipedBalanceDecision3D decision =
        m_balance.assess(m_rig, observation, m_intent);
    BipedStepPlan3D step = m_stepPlanner.update(
        profile, m_rig, observation, m_intent,
        decision, terrain, deltaTime);
    const BipedBalanceCommand3D balance =
        m_balance.command(profile, m_rig, state,
            observation, m_intent, step, decision);

    if (step.phase == BipedStepPhase3D::Transfer)
        enterPhase(BipedControllerPhase3D::Transfer);
    else if (step.phase == BipedStepPhase3D::Swing)
        enterPhase(BipedControllerPhase3D::Swing);
    else if (step.phase == BipedStepPhase3D::Settle)
        enterPhase(BipedControllerPhase3D::DoubleSupport);
    else
        enterPhase(BipedControllerPhase3D::Standing);

    std::vector<Vec3> desired(profile.links.size());
    clampBipedJointCoordinates3D(profile, desired);

    if (step.phase == BipedStepPhase3D::Swing
        && step.swingFoot >= 0 && step.swingFoot < 2) {
        const std::size_t foot =
            m_rig.foot[static_cast<std::size_t>(step.swingFoot)];
        if (foot < profile.links.size()) {
            const Vec3 origin = endEffectorOriginForSole(
                profile.links[foot],
                step.swingFootOrientationWorld,
                step.swingSoleTargetWorld);
            BipedIkSettings3D ik;
            ik.iterations = 12;
            ik.maximumChainDepth = 3;
            ik.positionWeight = 1.0f;
            ik.orientationWeight = 0.18f;
            const bool ikSolved = solveBipedEndEffectorDls3D(
                profile, desired,
                state.links[m_rig.pelvis].position,
                state.links[m_rig.pelvis].orientation,
                foot, origin,
                step.swingFootOrientationWorld,
                true, 1.0f, ik);
            (void)ikSolved;
        }
    }

    BipedJointTorqueField3D torques;
    torques.reset(profile.links.size());
    if (decision.strategy != BipedBalanceStrategy3D::Fall
        && m_dynamics.validFor(profile, dynamics)) {
        auto contacts = footContacts(m_rig, observation);
        std::vector<std::uint8_t> enabled(profile.links.size(), 1);
        if (step.phase == BipedStepPhase3D::Swing)
            disableLeg(enabled, m_rig, step.swingFoot);

        m_dynamics.addComForceThroughContacts(
            profile, state, dynamics, contacts,
            balance.desiredComForceWorld, torques, enabled);
        m_dynamics.addAngularTaskThroughContacts(
            profile, state, dynamics, m_rig.torso,
            contacts, balance.desiredTorsoTorqueWorld,
            torques, enabled);
    }

    buildDriveTargets(profile, state, desired, torques,
        deltaTime, 1.0f,
        decision.strategy == BipedBalanceStrategy3D::Step
            ? 1.15f : 1.0f);
    m_telemetry.actuatorDemandPeak = estimateActuatorDemandPeak(
        profile, state, dynamics, m_output.driveTargets,
        m_output.gravityCompensationEnabled);

    m_telemetry.balanceStrategy = decision.strategy;
    m_telemetry.balanceUrgency = decision.urgency;
    m_telemetry.stepPhase = step.phase;
    m_telemetry.stanceFoot = step.stanceFoot;
    m_telemetry.swingFoot = step.swingFoot;
    m_telemetry.recoveryStep = step.recoveryStep;
    m_telemetry.plannedLandingWorld =
        step.landingSoleWorld;
}

void PhysicalBipedController3D::updateGetUpControl(
    const RagdollProfile3D& profile,
    const RagdollState3D& state,
    const RagdollDynamics3D& dynamics,
    const BipedTerrainQuery3D& terrain,
    const BipedObservation3D& observation,
    float deltaTime,
    bool manipulated) {
    enterPhase(BipedControllerPhase3D::GetUp);
    BipedGetUpCommand3D command =
        m_getUp.update(profile, m_rig, state,
            observation, terrain,
            m_intent.desiredHeadingRadians, deltaTime);

    if (command.complete) {
        enterPhase(BipedControllerPhase3D::Standing);
        m_stepPlanner.reset();
        m_residual.reset();
        m_driveCoordinates =
            bipedJointCoordinatesFromState3D(profile, state);
        return;
    }

    std::vector<Vec3> desired =
        bipedJointCoordinatesFromState3D(profile, state);
    const float regularization =
        1.0f-std::exp(-1.6f*deltaTime);
    for (std::size_t link = 1; link < desired.size(); ++link) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            if (!profile.links[link].inboundJoint.axes[axis].enabled)
                continue;
            setComponentBiped(desired[link], axis,
                componentBiped(desired[link], axis)
                    *(1.0f-regularization));
        }
    }

    BipedIkSettings3D ik;
    ik.iterations = 12;
    ik.maximumChainDepth = 4;
    ik.positionWeight = 1.0f;
    ik.orientationWeight = 0.18f;
    for (const auto& task : command.endEffectors) {
        if (task.linkIndex >= profile.links.size()) continue;
        ik.positionWeight =
            std::max(0.05f, task.positionWeight);
        ik.orientationWeight =
            std::max(0.0f, task.orientationWeight);
        const bool ikSolved = solveBipedEndEffectorDls3D(
            profile, desired,
            state.links[m_rig.pelvis].position,
            state.links[m_rig.pelvis].orientation,
            task.linkIndex,
            task.positionWorld,
            task.orientationWorld,
            task.constrainOrientation,
            task.positionWeight, ik);
        (void)ikSolved;
    }

    BipedJointTorqueField3D torques;
    torques.reset(profile.links.size());
    if (m_dynamics.validFor(profile, dynamics)
        && !observation.support.contacts.empty()) {
        const Vec3 comForce =
            getUpComForce(profile, observation, command);
        const Vec3 torsoTorque =
            getUpTorsoTorque(profile, m_rig, state, command);
        m_dynamics.addComForceThroughContacts(
            profile, state, dynamics,
            observation.support.contacts,
            comForce, torques);
        m_dynamics.addAngularTaskThroughContacts(
            profile, state, dynamics, m_rig.torso,
            observation.support.contacts,
            torsoTorque, torques);
    }

    buildDriveTargets(profile, state, desired, torques,
        deltaTime, 1.25f, 1.45f);

    m_residual.settings().enabled =
        m_settings.residualGetUpAssistEnabled;
    ResidualGetUpAssistInput3D residualInput;
    residualInput.inGetUp = true;
    residualInput.eligibleStage = command.residualEligible;
    residualInput.manipulated = manipulated;
    residualInput.airborne =
        observation.support.mode == BipedSupportMode3D::Airborne;
    residualInput.totalMassKg = profile.totalMassKg;
    residualInput.supportLoadNewtons =
        observation.support.totalNormalLoadNewtons;
    const float actuatorDemandPeak = estimateActuatorDemandPeak(
        profile, state, dynamics, m_output.driveTargets,
        m_output.gravityCompensationEnabled);
    residualInput.actuatorDemandPeak = actuatorDemandPeak;
    m_telemetry.actuatorDemandPeak = actuatorDemandPeak;
    residualInput.centerOfMassVerticalVelocity =
        observation.centerOfMassVelocityWorld.z;
    residualInput.desiredVerticalVelocity =
        command.desiredComVerticalVelocity;
    residualInput.deltaTime = deltaTime;
    const Vec3 residual = m_residual.update(residualInput);
    if (residual.lengthSquared() > 0.0f) {
        m_output.externalWrenches.push_back({
            static_cast<std::uint32_t>(m_rig.pelvis),
            residual, {}});
    }

    m_telemetry.getUpStage = command.stage;
    m_telemetry.fallPose = command.initialPose;
    m_telemetry.getUpReplans = m_getUp.replans();
    m_telemetry.residualForceNewtons = residual.length();
    m_telemetry.residualImpulseNewtonSeconds =
        m_residual.accumulatedImpulseNewtonSeconds();
    m_telemetry.balanceStrategy =
        BipedBalanceStrategy3D::None;
    m_telemetry.stepPhase = BipedStepPhase3D::Idle;
}

void PhysicalBipedController3D::update(
    const RagdollProfile3D& profile,
    const RagdollState3D& state,
    const RagdollDynamics3D& dynamics,
    const BipedTerrainQuery3D& terrain,
    float deltaTime,
    bool manipulated,
    std::uint32_t) {
    m_output.driveTargets.clear();
    m_output.externalWrenches.clear();
    m_output.gravityCompensationEnabled = true;
    if (!m_initialized || !m_rig.valid
        || !std::isfinite(deltaTime)
        || deltaTime <= 0.0f || deltaTime > 0.05f) {
        return;
    }

    if (m_timedCommand) {
        m_commandSecondsRemaining =
            std::max(0.0f, m_commandSecondsRemaining-deltaTime);
        if (m_commandSecondsRemaining <= 0.0f) stop();
    }

    const BipedObservation3D observation =
        m_estimator.observe(
            profile, m_rig, state, dynamics, deltaTime);
    if (!observation.valid) return;

    m_telemetry.manipulated = manipulated;
    m_telemetry.commandActive = m_intent.locomotionEnabled;
    m_telemetry.commandSecondsRemaining =
        m_commandSecondsRemaining;
    m_telemetry.desiredSpeedMetersPerSecond =
        horizontal(m_intent.desiredVelocityBody).length();
    m_telemetry.measuredSpeedMetersPerSecond =
        horizontal(observation.centerOfMassVelocityWorld).length();
    m_telemetry.centerOfMassHeightMeters =
        observation.centerOfMassHeightMeters;
    m_telemetry.centerOfMassWorld =
        observation.centerOfMassWorld;
    m_telemetry.centerOfMassVelocityWorld =
        observation.centerOfMassVelocityWorld;
    m_telemetry.capturePointWorld =
        observation.capturePointWorld;
    m_telemetry.captureMarginMeters =
        observation.captureMarginMeters;
    m_telemetry.supportLoadNewtons =
        observation.support.totalNormalLoadNewtons;
    m_telemetry.leftFootLoadRatio =
        observation.feet[0].loadRatio;
    m_telemetry.rightFootLoadRatio =
        observation.feet[1].loadRatio;
    m_telemetry.supportMode = observation.support.mode;
    m_telemetry.jointReactionTorqueRatioPeak =
        observation.jointReactionTorqueRatioPeak;
    m_telemetry.residualForceNewtons = 0.0f;
    m_telemetry.actuatorDemandPeak = 0.0f;

    if (manipulated) {
        m_fallenSeconds = m_airborneSeconds =
            m_stillSeconds = m_uprightSeconds = 0.0f;
        BipedJointTorqueField3D none;
        none.reset(profile.links.size());
        const auto current =
            bipedJointCoordinatesFromState3D(profile, state);
        buildDriveTargets(profile, state, current, none,
            deltaTime, 0.55f, 0.65f);
        return;
    }

    if (observation.fallenCandidate)
        m_fallenSeconds += deltaTime;
    else
        m_fallenSeconds = 0.0f;

    if (observation.support.mode == BipedSupportMode3D::Airborne)
        m_airborneSeconds += deltaTime;
    else
        m_airborneSeconds = 0.0f;

    if (observation.uprightCandidate)
        m_uprightSeconds += deltaTime;
    else
        m_uprightSeconds = 0.0f;

    if (m_telemetry.phase != BipedControllerPhase3D::GetUp
        && (m_fallenSeconds >= m_settings.fallConfirmationSeconds
            || m_airborneSeconds
                >= m_settings.airborneFallConfirmationSeconds)) {
        enterPhase(BipedControllerPhase3D::Falling);
    }

    if (m_telemetry.phase == BipedControllerPhase3D::Falling) {
        if (m_uprightSeconds >= m_settings.uprightConfirmationSeconds) {
            enterPhase(BipedControllerPhase3D::Standing);
            m_stillSeconds = 0.0f;
        } else {
            const auto& root = state.links[m_rig.pelvis];
            const bool still =
                observation.bodyGroundContact
                && root.linearVelocity.length() < 1.15f
                && root.angularVelocity.length() < 2.4f;
            m_stillSeconds = still
                ? m_stillSeconds+deltaTime : 0.0f;
            if (m_settings.automaticGetUp
                && m_stillSeconds >= m_settings.getUpStillSeconds) {
                enterPhase(BipedControllerPhase3D::GetUp);
            }
        }
    }

    if (m_telemetry.phase == BipedControllerPhase3D::GetUp) {
        updateGetUpControl(profile, state, dynamics,
            terrain, observation, deltaTime, false);
        return;
    }

    if (m_telemetry.phase == BipedControllerPhase3D::Falling) {
        BipedJointTorqueField3D none;
        none.reset(profile.links.size());
        auto posture =
            bipedJointCoordinatesFromState3D(profile, state);
        // Falling posture is internal only. It deliberately does not chase
        // world orientation or use any external balance wrench.
        buildDriveTargets(profile, state, posture, none,
            deltaTime, 0.40f, 0.55f);
        m_telemetry.balanceStrategy =
            BipedBalanceStrategy3D::Fall;
        return;
    }

    updateNormalControl(profile, state, dynamics,
        terrain, observation, deltaTime);
}

} // namespace MatterEngine
