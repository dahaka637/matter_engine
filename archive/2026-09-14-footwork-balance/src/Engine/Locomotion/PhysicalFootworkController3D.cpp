#include "Engine/Locomotion/PhysicalFootworkController3D.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <string_view>

namespace MatterEngine {
namespace {

constexpr float Pi = 3.14159265358979323846f;

std::size_t findLink(const RagdollProfile3D& profile,
    std::string_view id, std::size_t fallback) {
    for (std::size_t index = 0; index < profile.links.size(); ++index) {
        if (profile.links[index].id == id) return index;
    }
    return fallback;
}

Vec3 clampLength(Vec3 value, float maximumLength) {
    const float length = value.length();
    if (length <= maximumLength || length <= 1.0e-6f) return value;
    return value * (maximumLength / length);
}

float smoothStep(float value) {
    value = std::clamp(value, 0.0f, 1.0f);
    return value * value * (3.0f - 2.0f * value);
}

float yawFromOrientation(Quaternion orientation) {
    const Vec3 forward = orientation.rotate({ 1.0f, 0.0f, 0.0f });
    return std::atan2(forward.y, forward.x);
}

float shortestAngle(float from, float to) {
    float delta = std::fmod(to - from + Pi, 2.0f * Pi);
    if (delta < 0.0f) delta += 2.0f * Pi;
    return delta - Pi;
}

RagdollDriveTarget3D* findTarget(
    std::vector<RagdollDriveTarget3D>& targets,
    std::size_t linkIndex, RagdollAxis3D axis) {
    const auto found = std::find_if(targets.begin(), targets.end(),
        [&](const RagdollDriveTarget3D& target) {
            return target.linkIndex == linkIndex && target.axis == axis;
        });
    return found != targets.end() ? &*found : nullptr;
}

} // namespace

bool PhysicalFootworkController3D::reset(
    const RagdollProfile3D& profile, const RagdollState3D& state,
    float facingYawRadians, const FootworkTerrainProbe3D& terrain,
    PhysicalFootworkMode3D mode) {
    clear();
    if (profile.links.size() < 7
        || state.links.size() != profile.links.size()
        || state.joints.size() != profile.links.size()) {
        return false;
    }
    m_links.pelvis = findLink(profile, "Pelvis", 0);
    m_links.thigh = {
        findLink(profile, "LeftThigh", 1),
        findLink(profile, "RightThigh", 4)
    };
    m_links.shin = {
        findLink(profile, "LeftShin", 2),
        findLink(profile, "RightShin", 5)
    };
    m_links.foot = {
        findLink(profile, "LeftFoot", 3),
        findLink(profile, "RightFoot", 6)
    };
    const FootPose3D left =
        physicalSolePose(profile, state, m_links.foot[0]);
    const FootPose3D right =
        physicalSolePose(profile, state, m_links.foot[1]);
    const Vec3 groundCenter = (left.position + right.position) * 0.5f;
    if (!m_planner.reset(groundCenter, facingYawRadians, terrain)) {
        return false;
    }
    m_planner.setAutomaticSteppingEnabled(
        mode != PhysicalFootworkMode3D::ReactiveBalance);
    FootworkConfig3D& planning = m_planner.config();
    planning.footLengthMeters =
        profile.links[m_links.foot[0]].collider.boxHalfExtents.x * 2.0f;
    planning.footWidthMeters =
        profile.links[m_links.foot[0]].collider.boxHalfExtents.y * 2.0f;
    planning.footThicknessMeters =
        profile.links[m_links.foot[0]].collider.boxHalfExtents.z * 2.0f;
    planning.stanceWidthMeters = 0.27f;
    planning.rootSupportLimitMeters = 0.48f;
    m_mode = mode;
    m_input.lookYawRadians = facingYawRadians;
    m_supportConfidence = 0.20f;
    m_telemetry.initialized = true;
    m_telemetry.mode = mode;
    return true;
}

void PhysicalFootworkController3D::clear() {
    m_planner.clear();
    m_output = {};
    m_telemetry = {};
    m_input = {};
    m_hasPreviousDesiredFootPosition = {};
    m_supportConfidence = 0.0f;
    m_elapsedSeconds = 0.0f;
    m_pathOrigin = {};
    m_pathForward = { 1.0f, 0.0f, 0.0f };
    m_pathMaximumLateralDeviationMeters = 0.04f;
    m_pathConstraintEnabled = false;
    m_paused = false;
}

void PhysicalFootworkController3D::setInput(
    const FootworkInput3D& input) {
    m_input = input;
}

void PhysicalFootworkController3D::setPaused(bool paused) {
    m_paused = paused;
}

void PhysicalFootworkController3D::setLocomotionPath(
    bool enabled, Vec3 origin, Vec3 forward,
    float maximumLateralDeviationMeters) {
    forward.z = 0.0f;
    if (forward.lengthSquared() <= 1.0e-8f) {
        enabled = false;
        forward = { 1.0f, 0.0f, 0.0f };
    } else {
        forward = forward.normalized();
    }
    m_pathConstraintEnabled = enabled;
    m_pathOrigin = origin;
    m_pathForward = forward;
    m_pathMaximumLateralDeviationMeters = std::clamp(
        maximumLateralDeviationMeters, 0.0f, 0.25f);
}

bool PhysicalFootworkController3D::requestRecoveryStep(
    FootSide3D side, Vec3 captureTarget, bool emergency,
    const FootworkTerrainProbe3D& terrain) {
    if (!m_telemetry.initialized || m_paused) return false;
    return m_planner.requestRecoveryStep(
        side, captureTarget, emergency, terrain);
}

bool PhysicalFootworkController3D::recoverUnsupportedFoot(
    FootSide3D side, Vec3 captureTarget, bool emergency,
    const FootworkTerrainProbe3D& terrain) {
    if (!m_telemetry.initialized || m_paused) return false;
    return m_planner.recoverUnsupportedFoot(
        side, captureTarget, emergency, terrain);
}

bool PhysicalFootworkController3D::retargetActiveStep(
    Vec3 captureTarget, bool allowCrossing,
    const FootworkTerrainProbe3D& terrain) {
    if (!m_telemetry.initialized || m_paused) return false;
    return m_planner.retargetActiveStep(
        captureTarget, allowCrossing, terrain);
}

FootPose3D PhysicalFootworkController3D::physicalSolePose(
    const RagdollProfile3D& profile, const RagdollState3D& state,
    std::size_t footIndex) const {
    FootPose3D pose;
    const PhysicsBodyState3D& body = state.links[footIndex];
    const RagdollCapsuleDefinition3D& collider =
        profile.links[footIndex].collider;
    pose.position = body.position
        + body.orientation.rotate(collider.localPosition)
        - body.orientation.rotate({
            0.0f, 0.0f, collider.boxHalfExtents.z });
    pose.orientation = body.orientation;
    pose.groundNormal =
        body.orientation.rotate({ 0.0f, 0.0f, 1.0f }).normalized();
    pose.yawRadians = yawFromOrientation(body.orientation);
    return pose;
}

void PhysicalFootworkController3D::buildNeutralTargets(
    const RagdollProfile3D& profile) {
    m_output.driveTargets.clear();
    for (std::size_t linkIndex = 0;
            linkIndex < profile.links.size(); ++linkIndex) {
        const RagdollJointDefinition3D& joint =
            profile.links[linkIndex].inboundJoint;
        for (std::size_t axisIndex = 0; axisIndex < 3; ++axisIndex) {
            if (!joint.axes[axisIndex].enabled) continue;
            RagdollDriveTarget3D target;
            target.linkIndex = static_cast<std::uint32_t>(linkIndex);
            target.axis = static_cast<RagdollAxis3D>(axisIndex);
            target.stiffnessScale = m_config.jointStiffnessScale;
            target.dampingScale = m_config.jointDampingScale;
            target.maximumTorqueScale = m_config.jointMaximumTorqueScale;
            m_output.driveTargets.push_back(target);
        }
    }
}

void PhysicalFootworkController3D::applyLegTask(
    const RagdollProfile3D& profile, const RagdollState3D& state,
    FootSide3D side, const FootPose3D& desiredSole, float deltaTime) {
    const std::size_t sideIndex = footIndex3D(side);
    const std::array<std::size_t, 3> chain {
        m_links.thigh[sideIndex],
        m_links.shin[sideIndex],
        m_links.foot[sideIndex]
    };
    const std::size_t footIndex = chain[2];
    const FootworkDebugState3D& planning = m_planner.state();
    // `hasActiveSwing` significa que existe uma passada planejada, não que a
    // sola já saiu do chão. Durante a transferência de peso phaseProgress é
    // zero e os dois pés ainda precisam de pré-carga/orientação plantar. A
    // versão antiga retirava o apoio do futuro pé de swing no instante da
    // decisão e tentava transferir peso sobre uma base que ela própria havia
    // desmontado.
    const bool swinging = planning.hasActiveSwing
        && planning.activeSwingSide == side
        && planning.feet[sideIndex].phaseProgress > 1.0e-4f;
    const RagdollCapsuleDefinition3D& footCollider =
        profile.links[footIndex].collider;
    Vec3 loadedSolePosition = desiredSole.position;
    if (!swinging) {
        const Vec3 groundNormal =
            desiredSole.groundNormal.lengthSquared() > 1.0e-8f
            ? desiredSole.groundNormal.normalized()
            : desiredSole.orientation.rotate(
                { 0.0f, 0.0f, 1.0f }).normalized();
        loadedSolePosition -= groundNormal
            * m_config.plantedSolePreloadMeters;
    }
    const Vec3 desiredColliderCenter = loadedSolePosition
        + desiredSole.orientation.rotate({
            0.0f, 0.0f, footCollider.boxHalfExtents.z });
    const Vec3 desiredLinkPosition = desiredColliderCenter
        - desiredSole.orientation.rotate(footCollider.localPosition);
    Vec3 desiredVelocity;
    if (m_hasPreviousDesiredFootPosition[sideIndex]
        && deltaTime > 1.0e-5f) {
        desiredVelocity = (desiredLinkPosition
            - m_previousDesiredFootPosition[sideIndex]) / deltaTime;
    }
    m_previousDesiredFootPosition[sideIndex] = desiredLinkPosition;
    m_hasPreviousDesiredFootPosition[sideIndex] = true;

    const RagdollLinkDefinition3D& thighDefinition =
        profile.links[chain[0]];
    const RagdollLinkDefinition3D& pelvisDefinition =
        profile.links[m_links.pelvis];
    const PhysicsBodyState3D& pelvisState = state.links[m_links.pelvis];
    const Vec3 parentFramePosition =
        pelvisDefinition.modelOrientation.conjugate().rotate(
            thighDefinition.inboundJoint.anchorModelPosition
                - pelvisDefinition.modelPosition);
    const Vec3 hipAnchor = pelvisState.position
        + pelvisState.orientation.rotate(parentFramePosition);
    const Vec3 legVector = desiredLinkPosition - hipAnchor;
    const Vec3 desiredForward =
        desiredSole.orientation.rotate({ 1.0f, 0.0f, 0.0f });
    const Vec3 desiredLateral =
        desiredSole.orientation.rotate({ 0.0f, 1.0f, 0.0f });
    const float verticalDrop = std::max(0.20f, -legVector.z);
    const float forwardOffset = dot(legVector, desiredForward);
    const float lateralOffset = dot(legVector, desiredLateral);
    const float thighLength =
        std::max(0.05f, profile.links[chain[0]].collider.lengthMeters);
    const float shinLength =
        std::max(0.05f, profile.links[chain[1]].collider.lengthMeters);
    // A extensão do joelho depende da distância 3D quadril-pé. A versão
    // anterior ignorava lateralOffset: num side-step de 40--60 cm ela
    // calculava o joelho como se o pé ainda estivesse sob a bacia, dobrava a
    // perna demais e deixava a sola 15--25 cm acima do chão. Hip roll cuida
    // da direção lateral; a lei dos cossenos precisa do comprimento total.
    const float legDistance = std::sqrt(
        forwardOffset * forwardOffset
        + lateralOffset * lateralOffset
        + verticalDrop * verticalDrop);
    const float clampedDistance = std::clamp(legDistance,
        std::abs(thighLength - shinLength) + 0.01f,
        thighLength + shinLength - 0.01f);
    const float kneeInteriorCosine = std::clamp(
        (thighLength * thighLength + shinLength * shinLength
            - clampedDistance * clampedDistance)
            / (2.0f * thighLength * shinLength),
        -1.0f, 1.0f);
    const float kneeFlexion = Pi - std::acos(kneeInteriorCosine);
    const float hipOffsetCosine = std::clamp(
        (thighLength * thighLength + clampedDistance * clampedDistance
            - shinLength * shinLength)
            / (2.0f * thighLength * clampedDistance),
        -1.0f, 1.0f);
    const float hipOffsetAngle = std::acos(hipOffsetCosine);
    const float hipPitch = std::clamp(
        -std::atan2(forwardOffset, verticalDrop) - hipOffsetAngle,
        -2.0f, 0.5f);
    const float hipRoll = std::clamp(
        -std::atan2(lateralOffset, verticalDrop), -0.66f, 0.66f);

    const auto setTarget = [&](std::size_t linkIndex,
            RagdollAxis3D axis, float position) {
        RagdollDriveTarget3D* target =
            findTarget(m_output.driveTargets, linkIndex, axis);
        if (target == nullptr) return;
        const std::size_t axisIndex = static_cast<std::size_t>(axis);
        const float minimum =
            profile.links[linkIndex].inboundJoint.axes[axisIndex]
                .minimumRadians;
        const float maximum =
            profile.links[linkIndex].inboundJoint.axes[axisIndex]
                .maximumRadians;
        target->positionRadians = std::clamp(position, minimum, maximum);
        target->velocityRadiansPerSecond = std::clamp(
            (target->positionRadians
                - state.joints[linkIndex].positionRadians[axisIndex])
                * m_config.jointPositionGain,
            -m_config.jointMaximumVelocityRadiansPerSecond,
            m_config.jointMaximumVelocityRadiansPerSecond);
    };
    setTarget(chain[0], RagdollAxis3D::Swing1, hipPitch);
    setTarget(chain[0], RagdollAxis3D::Swing2, hipRoll);
    setTarget(chain[1], RagdollAxis3D::Twist, kneeFlexion);
    setTarget(chain[2], RagdollAxis3D::Twist,
        -(hipPitch + kneeFlexion));
    setTarget(chain[2], RagdollAxis3D::Swing1, -hipRoll);
    if (!swinging) {
        for (RagdollAxis3D axis :
                { RagdollAxis3D::Twist,
                    RagdollAxis3D::Swing1,
                    RagdollAxis3D::Swing2 }) {
            if (RagdollDriveTarget3D* target = findTarget(
                    m_output.driveTargets, footIndex, axis)) {
                target->stiffnessScale *=
                    m_config.plantedFootDriveStiffnessScale;
                target->dampingScale *=
                    m_config.plantedFootDriveDampingScale;
                target->maximumTorqueScale *=
                    m_config.plantedFootDriveTorqueScale;
            }
        }
    }

    // Força virtual de tarefa convertida em torques articulares por J^T.
    // O pé continua sendo movido exclusivamente pelos motores das juntas.
    const PhysicsBodyState3D& footState = state.links[footIndex];
    Vec3 taskForceRequest =
        (desiredLinkPosition - footState.position)
                * m_config.footTaskPositionGainNewtonsPerMeter
            + (desiredVelocity - footState.linearVelocity)
                * m_config.footTaskVelocityGainNewtonSecondsPerMeter;
    if (!swinging) {
        const Vec3 plantedNormal =
            desiredSole.groundNormal.lengthSquared() > 1.0e-8f
            ? desiredSole.groundNormal.normalized()
            : desiredSole.orientation.rotate(
                { 0.0f, 0.0f, 1.0f }).normalized();
        taskForceRequest -= plantedNormal
            * m_config.plantedContactPreloadNewtons;
    }
    const Vec3 taskForce = clampLength(
        taskForceRequest,
        m_config.maximumFootTaskForceNewtons);
    const Vec3 currentUp = footState.orientation.rotate(
        { 0.0f, 0.0f, 1.0f }).normalized();
    const Vec3 desiredUp = desiredSole.orientation.rotate(
        { 0.0f, 0.0f, 1.0f }).normalized();
    const Vec3 currentFootForward = footState.orientation.rotate(
        { 1.0f, 0.0f, 0.0f }).normalized();
    const Vec3 desiredFootForward = desiredSole.orientation.rotate(
        { 1.0f, 0.0f, 0.0f }).normalized();
    const float orientationScale = swinging
        ? m_config.plantedFootOrientationScale * 0.14f
        : m_config.plantedFootOrientationScale;
    // cross(atual, alvo) é um vetor-erro angular no mundo. A componente
    // dominante mantém a planta paralela ao terreno; a componente menor de
    // heading evita pés torcidos sem transformar o quadril numa dobradiça.
    const Vec3 orientationTorque = clampLength(
        (cross(currentUp, desiredUp)
                + cross(currentFootForward, desiredFootForward) * 0.18f)
                * (m_config.footOrientationStiffnessNewtonMetersPerRadian
                    * orientationScale)
            - footState.angularVelocity
                * (m_config.footOrientationDampingNewtonMeterSeconds
                    * orientationScale),
        m_config.maximumFootOrientationTorqueNewtonMeters);
    for (std::size_t linkIndex : chain) {
        const RagdollLinkDefinition3D& link = profile.links[linkIndex];
        const std::size_t parentIndex =
            static_cast<std::size_t>(link.parentIndex);
        const RagdollLinkDefinition3D& parent = profile.links[parentIndex];
        const PhysicsBodyState3D& parentState = state.links[parentIndex];
        const Vec3 localAnchor =
            parent.modelOrientation.conjugate().rotate(
                link.inboundJoint.anchorModelPosition
                    - parent.modelPosition);
        const Quaternion localFrame =
            (parent.modelOrientation.conjugate()
                * link.inboundJoint.frameModelOrientation).normalized();
        const Vec3 anchor = parentState.position
            + parentState.orientation.rotate(localAnchor);
        const Quaternion frame =
            (parentState.orientation * localFrame).normalized();
        // A força linear usa a cadeia inteira. Já o momento plantar fica no
        // tornozelo: projetá-lo também no quadril satisfazia a orientação da
        // sola à custa de inclinar a pelve inteira, exatamente a postura
        // desengonçada que este controle pretende eliminar.
        const Vec3 moment =
            cross(footState.position - anchor, taskForce)
            + (linkIndex == footIndex ? orientationTorque : Vec3 {});
        const std::array<Vec3, 3> axes {
            frame.rotate({ 1.0f, 0.0f, 0.0f }),
            frame.rotate({ 0.0f, 1.0f, 0.0f }),
            frame.rotate({ 0.0f, 0.0f, 1.0f })
        };
        for (std::size_t axisIndex = 0; axisIndex < 3; ++axisIndex) {
            if (!link.inboundJoint.axes[axisIndex].enabled) continue;
            if (RagdollDriveTarget3D* target = findTarget(
                    m_output.driveTargets, linkIndex,
                    static_cast<RagdollAxis3D>(axisIndex))) {
                if (linkIndex == footIndex) {
                    target->maximumTorqueScale *=
                        m_config.footJointTorqueAuthorityScale;
                }
                target->feedforwardTorqueNewtonMeters +=
                    dot(axes[axisIndex], moment);
            }
        }
    }
}

void PhysicalFootworkController3D::buildRootAssistance(
    const RagdollProfile3D& profile, const RagdollState3D& state,
    float deltaTime) {
    const PhysicsBodyState3D& pelvis = state.links[m_links.pelvis];
    const FootworkDebugState3D& planning = m_planner.state();
    const Vec3 pelvisUp =
        pelvis.orientation.rotate({ 0.0f, 0.0f, 1.0f }).normalized();
    const float uprightDot = dot(pelvisUp, { 0.0f, 0.0f, 1.0f });
    std::array<bool, 2> supportedFoot { false, false };
    for (const RagdollContactPoint3D& contact : state.contacts) {
        if (contact.normal.z <= 0.35f) {
            continue;
        }
        if (contact.linkIndex == m_links.foot[0]) {
            supportedFoot[0] = true;
        } else if (contact.linkIndex == m_links.foot[1]) {
            supportedFoot[1] = true;
        }
    }
    const std::uint32_t supportedFeet =
        static_cast<std::uint32_t>(supportedFoot[0])
        + static_cast<std::uint32_t>(supportedFoot[1]);
    const FootPose3D leftSole =
        physicalSolePose(profile, state, m_links.foot[0]);
    const FootPose3D rightSole =
        physicalSolePose(profile, state, m_links.foot[1]);
    Vec3 stanceDelta = leftSole.position - rightSole.position;
    stanceDelta.z = 0.0f;
    m_telemetry.leftSoleUpDot = dot(leftSole.groundNormal,
        { 0.0f, 0.0f, 1.0f });
    m_telemetry.rightSoleUpDot = dot(rightSole.groundNormal,
        { 0.0f, 0.0f, 1.0f });
    m_telemetry.stanceWidthMeters = stanceDelta.length();
    const float contactTarget = supportedFeet > 0
        ? 1.0f : (m_elapsedSeconds < 0.45f ? 0.55f : 0.0f);
    const float contactRate = contactTarget > m_supportConfidence
        ? 10.0f : 5.0f;
    m_supportConfidence += (contactTarget - m_supportConfidence)
        * std::clamp(deltaTime * contactRate, 0.0f, 1.0f);
    const float postureAuthority = smoothStep((uprightDot - 0.18f) / 0.62f);
    const float minimumStandingHeight =
        std::max(0.25f, m_config.pelvisHeightMeters * 0.48f);
    const float heightAuthority = smoothStep(
        (pelvis.position.z - planning.rootGroundPosition.z
            - minimumStandingHeight)
        / std::max(0.10f,
            m_config.pelvisHeightMeters - minimumStandingHeight));
    const float authority = std::clamp(
        m_supportConfidence * postureAuthority
            * (0.18f + 0.82f * heightAuthority),
        0.0f, 1.0f);

    const Vec3 desiredPelvis {
        m_mode == PhysicalFootworkMode3D::Manual
            ? planning.rootGroundPosition.x : pelvis.position.x,
        m_mode == PhysicalFootworkMode3D::Manual
            ? planning.rootGroundPosition.y : pelvis.position.y,
        planning.rootGroundPosition.z + m_config.pelvisHeightMeters
    };
    Vec3 force;
    force.z = (desiredPelvis.z - pelvis.position.z)
            * m_config.verticalStiffnessNewtonsPerMeter
        - pelvis.linearVelocity.z
            * m_config.verticalDampingNewtonSecondsPerMeter
        + profile.totalMassKg * 9.81f
            * m_config.gravitySupportFraction;
    if (m_mode == PhysicalFootworkMode3D::Manual) {
        force.x = (desiredPelvis.x - pelvis.position.x)
                * m_config.horizontalStiffnessNewtonsPerMeter
            + (planning.rootVelocity.x - pelvis.linearVelocity.x)
                * m_config.horizontalDampingNewtonSecondsPerMeter;
        force.y = (desiredPelvis.y - pelvis.position.y)
                * m_config.horizontalStiffnessNewtonsPerMeter
            + (planning.rootVelocity.y - pelvis.linearVelocity.y)
                * m_config.horizontalDampingNewtonSecondsPerMeter;
    }
    force = clampLength(force * authority,
        m_config.maximumRootForceNewtons);

    Vec3 torque = cross(pelvisUp, { 0.0f, 0.0f, 1.0f })
            * m_config.uprightStiffnessNewtonMeters
        - Vec3 { pelvis.angularVelocity.x, pelvis.angularVelocity.y, 0.0f }
            * m_config.uprightDampingNewtonMeterSeconds;
    if (m_mode == PhysicalFootworkMode3D::Manual) {
        const float currentYaw = yawFromOrientation(pelvis.orientation);
        torque.z = shortestAngle(currentYaw, planning.rootYawRadians)
                * m_config.yawStiffnessNewtonMeters
            - pelvis.angularVelocity.z
                * m_config.yawDampingNewtonMeterSeconds;
    } else {
        torque.z = -pelvis.angularVelocity.z
            * m_config.yawDampingNewtonMeterSeconds * 0.35f;
    }
    torque = clampLength(torque * authority,
        m_config.maximumRootTorqueNewtonMeters);

    m_output.rootForceNewtons = force;
    m_output.rootTorqueNewtonMeters = torque;
    m_output.applyRootForce = authority > 0.015f
        && (force.lengthSquared() > 0.01f
            || torque.lengthSquared() > 0.01f);
    m_telemetry.supportConfidence = authority;
    m_telemetry.uprightDot = uprightDot;
    m_telemetry.pelvisHeightErrorMeters =
        desiredPelvis.z - pelvis.position.z;
    m_telemetry.rootForceNewtons = force.length();
    m_telemetry.rootTorqueNewtonMeters = torque.length();
    m_telemetry.supportedFootCount = supportedFeet;
    m_telemetry.assistanceActive = m_output.applyRootForce;
    m_telemetry.fallen = postureAuthority < 0.15f
        || heightAuthority < 0.10f;
}

void PhysicalFootworkController3D::update(
    const RagdollProfile3D& profile, const RagdollState3D& state,
    float deltaTime, const FootworkTerrainProbe3D& terrain) {
    if (!m_telemetry.initialized || deltaTime <= 0.0f
        || state.links.size() != profile.links.size()
        || state.joints.size() != profile.links.size()) {
        return;
    }
    m_elapsedSeconds += deltaTime;
    if (m_mode == PhysicalFootworkMode3D::AutonomousFollow
        || m_mode == PhysicalFootworkMode3D::ReactiveBalance) {
        const PhysicsBodyState3D& pelvis = state.links[m_links.pelvis];
        const FootPose3D leftSole =
            physicalSolePose(profile, state, m_links.foot[0]);
        const FootPose3D rightSole =
            physicalSolePose(profile, state, m_links.foot[1]);
        Vec3 actualRootGround {
            pelvis.position.x,
            pelvis.position.y,
            (leftSole.position.z + rightSole.position.z) * 0.5f
        };
        Vec3 plannerRootVelocity {
            pelvis.linearVelocity.x, pelvis.linearVelocity.y, 0.0f
        };
        if (m_mode == PhysicalFootworkMode3D::ReactiveBalance
            && m_pathConstraintEnabled) {
            const Vec3 pathLateral {
                -m_pathForward.y, m_pathForward.x, 0.0f
            };
            const Vec3 fromOrigin = actualRootGround - m_pathOrigin;
            const float progress = dot(fromOrigin, m_pathForward);
            const float lateral = std::clamp(
                dot(fromOrigin, pathLateral),
                -m_pathMaximumLateralDeviationMeters,
                m_pathMaximumLateralDeviationMeters);
            actualRootGround = m_pathOrigin
                + m_pathForward * progress
                + pathLateral * lateral;
            actualRootGround.z =
                (leftSole.position.z + rightSole.position.z) * 0.5f;
            plannerRootVelocity =
                m_pathForward * dot(plannerRootVelocity, m_pathForward);
        }
        FootworkInput3D inferred = m_input;
        inferred.lookYawRadians =
            m_mode == PhysicalFootworkMode3D::AutonomousFollow
            ? yawFromOrientation(pelvis.orientation)
            : m_input.lookYawRadians;
        m_planner.synchronizeRoot(actualRootGround,
            plannerRootVelocity,
            inferred.lookYawRadians);
        if (m_mode == PhysicalFootworkMode3D::AutonomousFollow) {
            const Vec3 forward {
                std::cos(inferred.lookYawRadians),
                std::sin(inferred.lookYawRadians), 0.0f
            };
            const Vec3 right { forward.y, -forward.x, 0.0f };
            const float speed = std::sqrt(
                pelvis.linearVelocity.x * pelvis.linearVelocity.x
                    + pelvis.linearVelocity.y * pelvis.linearVelocity.y);
            inferred.movementLocal = {};
            if (speed > 0.08f) {
                inferred.movementLocal.y =
                    dot(pelvis.linearVelocity, forward)
                        / std::max(0.25f,
                            m_planner.config().walkSpeedMetersPerSecond);
                inferred.movementLocal.x =
                    dot(pelvis.linearVelocity, right)
                        / std::max(0.25f,
                            m_planner.config().walkSpeedMetersPerSecond);
            }
            inferred.fast =
                speed > m_planner.config().walkSpeedMetersPerSecond * 1.35f;
        }
        m_input = inferred;
    }
    if (!m_paused) {
        m_planner.update(m_input, deltaTime, terrain);
    }
    buildNeutralTargets(profile);
    const FootworkDebugState3D& planning = m_planner.state();
    applyLegTask(profile, state, FootSide3D::Left,
        planning.feet[0].pose, deltaTime);
    applyLegTask(profile, state, FootSide3D::Right,
        planning.feet[1].pose, deltaTime);
    buildRootAssistance(profile, state, deltaTime);
    m_output.gravityCompensationEnabled = true;
}

} // namespace MatterEngine
