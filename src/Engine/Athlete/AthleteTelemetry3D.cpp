#include "Engine/Athlete/AthleteTelemetry3D.hpp"

#include "Engine/Athlete/Legacy/LegacyReference3D.hpp"

#include <algorithm>
#include <cstring>

namespace MatterEngine {

AthleteTelemetry3D athleteTelemetry3D(const AthleteState3D& athlete) {
    const LegacyAthleteState3D& legacy = athlete.legacy;
    AthleteTelemetry3D out;
    switch (legacy.bodyMode) {
    case LegacyBodyMode3D::Animated: out.mode = BodyModeKind3D::Animated; break;
    case LegacyBodyMode3D::Physical: out.mode = BodyModeKind3D::Physical; break;
    case LegacyBodyMode3D::Returning: out.mode = BodyModeKind3D::Returning; break;
    case LegacyBodyMode3D::Active: out.mode = legacy.bridge ? BodyModeKind3D::Bridge : BodyModeKind3D::Active; break;
    }
    out.modeSeconds = legacy.bodyModeSeconds;
    out.guided = legacy.guided;
    out.assistLevel = legacy.impactAssist.level();
    out.calmSeconds = legacy.calmSeconds;
    const CharacterLocomotionTelemetry3D& locomotion = legacy.locomotion.telemetry();
    const bool physical = out.mode == BodyModeKind3D::Physical;
    out.fallen = physical && locomotion.state == CharacterLocomotionState3D::Fallen;
    out.gettingUp = physical && locomotion.state == CharacterLocomotionState3D::GettingUp;
    out.tackling = (physical && locomotion.state == CharacterLocomotionState3D::Tackling) || legacy.defaultTackling;
    out.stagger = physical ? locomotion.stagger : 0.0f;
    out.perturbation = physical ? legacy.adaptivePhysics.output().perturbation : 0.0f;
    out.terrainSpeedLimit = locomotion.terrainSpeedLimit;
    out.contacts = athlete.lastContacts;
    out.lastDecision = athlete.lastDecision;
    out.lastDecisionFrom = athlete.lastDecisionFrom;
    return out;
}

AthleteControlView3D athleteControlView3D(const AthleteState3D& athlete) {
    const LegacyAthleteState3D& legacy = athlete.legacy;
    AthleteControlView3D view;
    view.groundHeight = athlete.groundHeight;
    view.impactLevel = legacy.impactAssist.level();
    const AdaptivePhysicalCharacter3D& adaptive = legacy.adaptivePhysics;
    view.walkingScale = adaptive.settings().walkingAssistScale;
    view.standingScale = adaptive.settings().legsAssistRetained;
    view.pelvisForceWorld = adaptive.output().rootForceWorld;
    view.pelvisTorqueWorld = adaptive.output().rootTorqueWorld;
    view.chosenStanding = adaptive.settings().standingWeightAssistScale >= 0.0f
        ? adaptive.settings().standingWeightAssistScale : view.standingScale;
    view.chosenWalking = adaptive.settings().walkingWeightAssistScale >= 0.0f
        ? adaptive.settings().walkingWeightAssistScale : view.walkingScale;
    view.impactFloor = legacy.impactAssist.floorLevel();
    view.impactProgress = legacy.impactAssist.recoverProgress();
    view.impactSeconds = legacy.impactAssist.recoverSeconds();
    view.stagger = legacy.locomotion.telemetry().stagger;
    {
        const std::vector<float>& strength = adaptive.output().jointStrength;
        if (!strength.empty()) {
            float sum = 0.0f, low = 1.0f;
            for (float s : strength) {
                sum += s;
                low = std::min(low, s);
            }
            view.muscleMean = sum / static_cast<float>(strength.size());
            view.muscleMinimum = low;
        }
    }
    view.policyApplied = athlete.policyApplied;
    const float acceleration = physicalNavigationAcceleration3D(view.walkingScale);
    view.navigation = { acceleration, acceleration, acceleration, 10.0f };
    const CharacterContactPlan3D& plan = legacy.locomotion.contactPlan();
    for (std::size_t side = 0; side < 2 && plan.valid; ++side) {
        const FootPlan3D& foot = plan.feet[side];
        AthleteControlView3D::ClassicFoot& out = view.classicFeet[side];
        out.swinging = foot.phase == FootPhase3D::Swing;
        out.goalWorld = foot.goalWorld;
        out.swingLeft = foot.swingDurationSeconds > 1e-4f
            ? std::clamp(1.0f - foot.swingSeconds / foot.swingDurationSeconds, 0.0f, 1.0f) : 0.0f;
    }
    view.classicPose = &legacy.locomotion.output().targetPose;
    if (legacy.referenceShadowActive) view.reference = legacyReferenceMotion3D(legacy.referenceShadow.output());
    {
        const TurnMoveOutput3D& move = legacy.turnMove.output();
        if (move.weight > 0.0f) {
            const PhysicalCorrections3D& prior = legacy.turnMove.corrections();
            view.turn.active = true;
            view.turn.state = static_cast<int>(move.state);
            view.turn.side = move.side;
            view.turn.remainingDegrees = move.remainingDegrees;
            view.turn.progress = move.progress;
            view.turn.entrySpeed = move.entrySpeed;
            view.turn.priorLateralLean = prior.lateralLean;
            view.turn.priorForwardLean = prior.forwardLean;
            view.turn.priorPelvisHeight = prior.pelvisHeight;
            view.turn.priorBrake = 1.0f - move.speedScale;
        }
    }
    const CharacterLocomotionTelemetry3D& telemetry = legacy.locomotion.telemetry();
    AthleteControlView3D::Executor& executor = view.executor;
    executor.state = static_cast<int>(telemetry.state);
    executor.gaitDirection = static_cast<int>(telemetry.gaitDirection);
    executor.cyclePhase = telemetry.cyclePhase;
    executor.legPhase = telemetry.legPhase;
    executor.swingProgress = telemetry.footSwingProgress;
    executor.clipContact = telemetry.clipFootContact;
    executor.classicLead = telemetry.legPhaseLead;
    executor.waitingLeg = telemetry.waitingLeg;
    executor.playbackRate = telemetry.playbackRate;
    executor.strideScale = telemetry.strideScale;
    for (std::size_t side = 0; side < 2; ++side) {
        executor.plannerPhase[side] = static_cast<int>(plan.feet[side].phase);
        executor.plannerOwns[side] = !plan.feet[side].followingGait;
    }
    executor.recoveryUrgency = plan.recoveryUrgency;
    executor.capturePointWorld = telemetry.capturePointWorld;
    executor.captureOutside = telemetry.captureOutsideMeters;
    executor.referenceRootWorld = legacy.locomotion.output().guide.rootPositionWorld;
    executor.legShare = adaptive.output().legShare;
    executor.balanceShare = adaptive.output().balanceShare;
    executor.comAccelerationRequested = adaptive.output().requestedComAccelerationWorld;
    executor.comAccelerationAllocated = adaptive.output().allocatedComAccelerationWorld;
    executor.comAccelerationResidual = adaptive.output().allocationResidualWorld;
    executor.balanceMargin = adaptive.output().balanceMargin;
    return view;
}

namespace {

// FNV-1a de 64 bits sobre os bits dos numeros (bit a bit: -0 e 0 diferem,
// como deve ser numa prova de igualdade).
struct Digest {
    std::uint64_t value = 1469598103934665603ull;
    void bytes(const void* data, std::size_t size) {
        const auto* p = static_cast<const unsigned char*>(data);
        for (std::size_t i = 0; i < size; ++i) {
            value ^= p[i];
            value *= 1099511628211ull;
        }
    }
    void add(float v) {
        std::uint32_t bits = 0;
        std::memcpy(&bits, &v, sizeof bits);
        bytes(&bits, sizeof bits);
    }
    void add(std::uint32_t v) { bytes(&v, sizeof v); }
    void add(Vec3 v) { add(v.x); add(v.y); add(v.z); }
    void add(Quaternion q) { add(q.x); add(q.y); add(q.z); add(q.w); }
};

} // namespace

AthleteTickDigest3D athleteTickDigest3D(const AthleteState3D& athlete, const RagdollState3D& body) {
    AthleteTickDigest3D out;
    Digest links;
    for (const PhysicsBodyState3D& link : body.links) {
        links.add(link.position);
        links.add(link.orientation);
        links.add(link.linearVelocity);
        links.add(link.angularVelocity);
    }
    for (const RagdollInteraction3D& contact : body.interactions) {
        links.add(contact.linkIndex);
        links.add(static_cast<std::uint32_t>(contact.otherMotion));
        links.add(contact.normalWorld);
        links.add(contact.normalImpulseNewtonSeconds);
    }
    for (const RagdollJointState3D& joint : body.joints)
        for (float angle : joint.positionRadians) links.add(angle);
    links.add(body.externalInterference);
    links.add(body.externalPushNewtonSeconds);
    out.body = links.value;
    const LegacyAthleteState3D& legacy = athlete.legacy;
    const CharacterLocomotionOutput3D& locomotion = legacy.locomotion.output();
    Digest targets;
    for (const RagdollDriveTarget3D& target : locomotion.driveTargets) {
        targets.add(target.linkIndex);
        targets.add(static_cast<std::uint32_t>(target.axis));
        targets.add(target.positionRadians);
        targets.add(target.velocityRadiansPerSecond);
        targets.add(target.feedforwardTorqueNewtonMeters);
        targets.add(target.stiffnessScale);
        targets.add(target.dampingScale);
        targets.add(target.maximumTorqueScale);
    }
    targets.add(locomotion.guide.rootPositionWorld);
    targets.add(locomotion.guide.rootOrientationWorld);
    targets.add(locomotion.rootControlForceWorld);
    targets.add(locomotion.rootControlTorqueWorld);
    out.targets = targets.value;
    const AdaptivePhysicalOutput3D& adaptive = legacy.adaptivePhysics.output();
    Digest assist;
    assist.add(adaptive.rootForceWorld);
    assist.add(adaptive.rootTorqueWorld);
    for (std::size_t side = 0; side < 2; ++side) {
        assist.add(adaptive.footForceWorld[side]);
        assist.add(adaptive.footTorqueWorld[side]);
        assist.add(adaptive.handTorqueWorld[side]);
    }
    for (const LinkWrench3D& wrench : athlete.feetAssistOutput.wrenches) {
        assist.add(wrench.link);
        assist.add(wrench.force);
        assist.add(wrench.torque);
    }
    assist.add(legacy.impactAssist.level());
    out.assist = assist.value;
    Digest decisions;
    const AthleteTelemetry3D telemetry = athleteTelemetry3D(athlete);
    decisions.add(static_cast<std::uint32_t>(telemetry.mode));
    decisions.add(telemetry.modeSeconds);
    decisions.add(static_cast<std::uint32_t>(telemetry.fallen) | static_cast<std::uint32_t>(telemetry.gettingUp) << 1
        | static_cast<std::uint32_t>(telemetry.tackling) << 2);
    decisions.add(static_cast<std::uint32_t>(athlete.lastDecision.change));
    decisions.add(static_cast<std::uint32_t>(athlete.lastDecision.to));
    decisions.add(static_cast<std::uint32_t>(athlete.lastDecision.event));
    decisions.add(athlete.lastDecision.value);
    decisions.add(static_cast<std::uint32_t>(legacy.bridge) | static_cast<std::uint32_t>(legacy.guided) << 1);
    out.decisions = decisions.value;
    Digest navigation;
    navigation.add(legacy.navigationAnchor);
    navigation.add(athlete.commandPlanVelocity);
    navigation.add(athlete.commandTurnRate);
    navigation.add(athlete.pushWindow.value);
    navigation.add(athlete.impactWindow.value);
    navigation.add(legacy.returnBoost);
    navigation.add(legacy.returnRampLeft);
    out.navigation = navigation.value;
    return out;
}

int athleteDumpModeCode3D(BodyModeKind3D mode) {
    switch (mode) {
    case BodyModeKind3D::Animated: return 0;
    case BodyModeKind3D::Physical: return 1;
    case BodyModeKind3D::Returning: return 2;
    case BodyModeKind3D::Bridge:
    case BodyModeKind3D::Active: return 3;
    }
    return 0;
}

} // namespace MatterEngine
