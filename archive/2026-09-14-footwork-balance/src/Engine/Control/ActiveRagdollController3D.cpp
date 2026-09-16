#include "Engine/Control/ActiveRagdollController3D.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <string_view>

namespace MatterEngine {
namespace {

std::size_t findLink(const RagdollProfile3D& profile,
    std::string_view id, std::size_t fallback) {
    for (std::size_t index = 0; index < profile.links.size(); ++index) {
        if (profile.links[index].id == id) return index;
    }
    return fallback;
}

float yawFromOrientation(Quaternion orientation) {
    const Vec3 forward = orientation.rotate({ 1.0f, 0.0f, 0.0f });
    return std::atan2(forward.y, forward.x);
}

Vec3 approachVector(Vec3 current, Vec3 target,
    float maximumDelta) noexcept {
    const Vec3 delta = target - current;
    const float distance = delta.length();
    if (distance <= maximumDelta || distance < 1.0e-6f) return target;
    return current + delta * (maximumDelta / distance);
}

Vec3 physicalSoleCenter(const RagdollProfile3D& profile,
    const RagdollState3D& state, std::size_t footLink) {
    if (footLink >= profile.links.size()
        || footLink >= state.links.size()) {
        return {};
    }
    const PhysicsBodyState3D& foot = state.links[footLink];
    const RagdollCapsuleDefinition3D& collider =
        profile.links[footLink].collider;
    return foot.position
        + foot.orientation.rotate(collider.localPosition)
        - foot.orientation.rotate({
            0.0f, 0.0f, collider.boxHalfExtents.z });
}

FootPose3D physicalSolePose(const RagdollProfile3D& profile,
    const RagdollState3D& state, std::size_t footLink) {
    FootPose3D pose;
    if (footLink >= profile.links.size()
        || footLink >= state.links.size()) {
        return pose;
    }
    const PhysicsBodyState3D& foot = state.links[footLink];
    pose.position = physicalSoleCenter(profile, state, footLink);
    pose.orientation = foot.orientation;
    pose.groundNormal = foot.orientation.rotate(
        { 0.0f, 0.0f, 1.0f }).normalized();
    pose.yawRadians = yawFromOrientation(foot.orientation);
    return pose;
}

} // namespace

void ActiveRagdollController3D::reset(
    const RagdollProfile3D& profile, Quaternion worldOrientation) {
    m_links.pelvis = findLink(profile, "Pelvis", 0);
    m_links.abdomen = findLink(profile, "Abdomen", 1);
    m_links.chest = findLink(profile, "Chest", 2);
    m_links.upperChest = findLink(profile, "UpperChest", 3);
    m_links.neck = findLink(profile, "Neck", 4);
    m_links.head = findLink(profile, "Head", 5);
    m_links.leftUpperArm = findLink(profile, "LeftUpperArm", 6);
    m_links.leftForearm = findLink(profile, "LeftForearm", 7);
    m_links.leftHand = findLink(profile, "LeftHand", 8);
    m_links.rightUpperArm = findLink(profile, "RightUpperArm", 9);
    m_links.rightForearm = findLink(profile, "RightForearm", 10);
    m_links.rightHand = findLink(profile, "RightHand", 11);
    m_links.leftThigh = findLink(profile, "LeftThigh", 12);
    m_links.leftShin = findLink(profile, "LeftShin", 13);
    m_links.leftFoot = findLink(profile, "LeftFoot", 14);
    m_links.rightThigh = findLink(profile, "RightThigh", 15);
    m_links.rightShin = findLink(profile, "RightShin", 16);
    m_links.rightFoot = findLink(profile, "RightFoot", 17);
    m_spawnOrientation = worldOrientation.normalized();
    m_forward = m_spawnOrientation.rotate({ 1.0f, 0.0f, 0.0f });
    m_forward.z = 0.0f;
    m_forward = m_forward.lengthSquared() > 1.0e-8f
        ? m_forward.normalized() : Vec3 { 1.0f, 0.0f, 0.0f };
    m_right = { m_forward.y, -m_forward.x, 0.0f };
    m_balance.reset(m_spawnOrientation);
    m_wholeBody.reset(m_spawnOrientation);
    m_footwork.clear();
    m_output = {};
    m_telemetry = {};
    m_ageSeconds = 0.0f;
    m_walkStart = {};
    m_recoveryVelocityReference = {};
    m_walkRequestedMeters = 0.0f;
    m_lastWalkDistanceMeters = 0.0f;
    m_walkPreparationSeconds = 0.0f;
    m_recoveryCooldownSeconds = 0.0f;
    m_outsideSupportSeconds = 0.0f;
    m_narrowStanceSeconds = 0.0f;
    m_leftUnsupportedSeconds = 0.0f;
    m_rightUnsupportedSeconds = 0.0f;
    m_leftSupportStableSeconds = 0.0f;
    m_rightSupportStableSeconds = 0.0f;
    m_stanceAdjustmentCooldownSeconds = 0.0f;
    m_confirmedLandingSeconds = 0.0f;
    m_lastCompletedFootworkSteps = 0;
    m_walkStartCompletedFootworkSteps = 0;
    m_lastCaptureRedirectedStep =
        std::numeric_limits<std::uint64_t>::max();
    m_footworkInitialized = false;
    m_reactiveRecoveryStep = false;
    m_reactiveRecoveryTrailingStep = false;
    m_lastReactiveRecoverySide = FootSide3D::Left;
    m_hasLastReactiveRecoverySide = false;
    m_externalManipulationActive = false;
    m_externalManipulatedLinkIndex =
        std::numeric_limits<std::uint32_t>::max();
    m_walkPending = false;
    m_walking = false;
}

void ActiveRagdollController3D::requestWalkDistance(
    float distanceMeters) {
    if (!std::isfinite(distanceMeters) || distanceMeters <= 0.01f) {
        cancelWalk();
        return;
    }
    m_walkRequestedMeters = distanceMeters;
    m_lastWalkDistanceMeters = 0.0f;
    m_walkPreparationSeconds = 0.0f;
    m_walkPending = true;
    m_walking = false;
}

void ActiveRagdollController3D::cancelWalk() {
    m_walkRequestedMeters = 0.0f;
    m_walkPending = false;
    m_walkPreparationSeconds = 0.0f;
    m_walking = false;
    m_telemetry.walking = false;
    m_telemetry.walkRequestedMeters = 0.0f;
    m_telemetry.walkRemainingMeters = 0.0f;
}

void ActiveRagdollController3D::transitionTo(
    ActiveRagdollPhase3D phase) {
    if (m_telemetry.phase == phase) return;
    m_telemetry.phase = phase;
    m_telemetry.timeInPhaseSeconds = 0.0f;
}

void ActiveRagdollController3D::configurePhysicalFootwork() {
    PhysicalFootworkConfig3D& physical = m_footwork.config();
    physical.pelvisHeightMeters = 0.98f;
    physical.gravitySupportFraction = 0.0f;
    physical.jointPositionGain = m_config.jointPositionGain;
    physical.jointMaximumVelocityRadiansPerSecond =
        m_config.maximumJointVelocityRadiansPerSecond;
    physical.jointStiffnessScale =
        m_config.standingStiffnessScale * 1.16f;
    physical.jointDampingScale =
        m_config.standingDampingScale;
    physical.jointMaximumTorqueScale =
        m_config.legTorqueAuthorityScale;
    // O conjunto de pernas do corpo inteiro move mais massa que o protótipo
    // inferior. Esta autoridade evita passos "moles", mas continua limitada
    // pelos torques anatômicos declarados no perfil e pela J^T.
    physical.footTaskPositionGainNewtonsPerMeter = 1'480.0f;
    physical.footTaskVelocityGainNewtonSecondsPerMeter = 180.0f;
    physical.maximumFootTaskForceNewtons = 1'050.0f;
    physical.footOrientationStiffnessNewtonMetersPerRadian = 235.0f;
    physical.footOrientationDampingNewtonMeterSeconds = 38.0f;
    physical.maximumFootOrientationTorqueNewtonMeters = 205.0f;
    physical.plantedFootOrientationScale = 0.0f;
    physical.footJointTorqueAuthorityScale = 1.0f;
    physical.plantedSolePreloadMeters = 0.0040f;
    physical.plantedContactPreloadNewtons = 0.0f;

    FootworkConfig3D& gait = m_footwork.planner().config();
    gait.stanceWidthMeters = 0.26f;
    gait.homeForwardOffsetMeters = 0.04f;
    gait.walkSpeedMetersPerSecond =
        m_config.walkSpeedMetersPerSecond;
    gait.fastSpeedMetersPerSecond = 3.10f;
    gait.accelerationMetersPerSecondSquared = 10.0f;
    gait.decelerationMetersPerSecondSquared = 13.0f;
    gait.stepTriggerDistanceMeters = 0.20f;
    gait.stepReleaseDistanceMeters = 0.11f;
    // O alcance do planejador precisa respeitar o alcance horizontal real
    // da cadeia quadril-joelho-tornozelo nesta altura de COM. Aceitar 92 cm
    // produzia footholds matematicamente perfeitos, porém inalcançáveis; a
    // perna parava a ~20 cm do alvo e nunca liberava a passada seguinte.
    // Impactos grandes são recuperados por uma sequência de passos rápidos.
    gait.maximumStepReachMeters = 0.68f;
    gait.antiCrossingMarginMeters = 0.050f;
    gait.swingDurationSeconds = 0.36f;
    gait.minimumSwingDurationSeconds = 0.31f;
    gait.maximumSwingDurationSeconds = 0.46f;
    gait.weightTransferDurationSeconds = 0.68f;
    gait.swingClearanceMeters = 0.115f;
    gait.fastSwingDurationSeconds = 0.12f;
    gait.fastSwingClearanceMeters = 0.10f;
    gait.emergencyRecoveryWeightTransferDurationSeconds = 0.0f;
    // O alvo fica cerca de 35 cm adiante da bacia para uma caminhada de
    // 0,84 m/s. Isso produz passadas humanas de 38--48 cm sem pedir a
    // extensão máxima das pernas; o horizonte antigo de 0,28 s gerava
    // passinhos de aproximadamente 20 cm.
    gait.velocityLookAheadSeconds = 0.42f;
    gait.fastVelocityLookAheadSeconds = 0.27f;
    gait.targetChaseStrength = 0.08f;
    gait.rootSupportLimitMeters = 0.66f;
}

void ActiveRagdollController3D::buildFallbackTargets(
    const RagdollProfile3D& profile) {
    m_output.driveTargets.clear();
    for (std::size_t linkIndex = 1;
            linkIndex < profile.links.size(); ++linkIndex) {
        const RagdollJointDefinition3D& joint =
            profile.links[linkIndex].inboundJoint;
        for (std::size_t axisIndex = 0; axisIndex < 3; ++axisIndex) {
            if (!joint.axes[axisIndex].enabled) continue;
            RagdollDriveTarget3D target;
            target.linkIndex = static_cast<std::uint32_t>(linkIndex);
            target.axis = static_cast<RagdollAxis3D>(axisIndex);
            target.stiffnessScale = m_config.standingStiffnessScale;
            target.dampingScale = m_config.standingDampingScale;
            target.maximumTorqueScale = 1.0f;
            m_output.driveTargets.push_back(target);
        }
    }
}

RagdollDriveTarget3D* ActiveRagdollController3D::findTarget(
    std::size_t linkIndex, RagdollAxis3D axis) {
    const auto found = std::find_if(m_output.driveTargets.begin(),
        m_output.driveTargets.end(),
        [&](const RagdollDriveTarget3D& target) {
            return target.linkIndex == linkIndex
                && target.axis == axis;
        });
    return found != m_output.driveTargets.end() ? &*found : nullptr;
}

void ActiveRagdollController3D::applyWorldTorque(
    const RagdollProfile3D& profile, const RagdollState3D& state,
    std::size_t linkIndex, Vec3 torqueWorld, float scale) {
    if (linkIndex >= profile.links.size()
        || linkIndex >= state.links.size()) {
        return;
    }
    const RagdollLinkDefinition3D& link = profile.links[linkIndex];
    if (link.parentIndex < 0) return;
    const std::size_t parentIndex =
        static_cast<std::size_t>(link.parentIndex);
    const RagdollLinkDefinition3D& parent = profile.links[parentIndex];
    const Quaternion localFrame =
        (parent.modelOrientation.conjugate()
            * link.inboundJoint.frameModelOrientation).normalized();
    const Quaternion frame =
        (state.links[parentIndex].orientation * localFrame).normalized();
    const std::array<Vec3, 3> axes {
        frame.rotate({ 1.0f, 0.0f, 0.0f }),
        frame.rotate({ 0.0f, 1.0f, 0.0f }),
        frame.rotate({ 0.0f, 0.0f, 1.0f })
    };
    for (std::size_t axisIndex = 0; axisIndex < 3; ++axisIndex) {
        if (!link.inboundJoint.axes[axisIndex].enabled) continue;
        if (RagdollDriveTarget3D* target = findTarget(
                linkIndex, static_cast<RagdollAxis3D>(axisIndex))) {
            target->feedforwardTorqueNewtonMeters +=
                dot(axes[axisIndex], torqueWorld) * scale;
        }
    }
}

void ActiveRagdollController3D::applyLegEndpointForce(
    const RagdollProfile3D& profile, const RagdollState3D& state,
    bool left, Vec3 forceWorld, float scale) {
    const std::array<std::size_t, 3> chain = left
        ? std::array<std::size_t, 3> {
            m_links.leftThigh, m_links.leftShin, m_links.leftFoot
        }
        : std::array<std::size_t, 3> {
            m_links.rightThigh, m_links.rightShin, m_links.rightFoot
        };
    const Vec3 endpoint = state.links[chain[2]].position;
    for (std::size_t linkIndex : chain) {
        const RagdollLinkDefinition3D& link = profile.links[linkIndex];
        if (link.parentIndex < 0) continue;
        const std::size_t parentIndex =
            static_cast<std::size_t>(link.parentIndex);
        const RagdollLinkDefinition3D& parent =
            profile.links[parentIndex];
        const PhysicsBodyState3D& parentState =
            state.links[parentIndex];
        const Vec3 localAnchor =
            parent.modelOrientation.conjugate().rotate(
                link.inboundJoint.anchorModelPosition
                    - parent.modelPosition);
        const Vec3 anchor = parentState.position
            + parentState.orientation.rotate(localAnchor);
        applyWorldTorque(profile, state, linkIndex,
            cross(endpoint - anchor, forceWorld), scale);
    }
}

void ActiveRagdollController3D::buildWholeBodyTargets(
    const RagdollProfile3D& profile, const RagdollState3D& state) {
    const NaturalBalanceState3D& balance = m_balance.state();
    const NaturalBalanceResponse3D& reaction = m_balance.response();
    const float urgency = balance.recoveryUrgency;
    const float horizontalSpeed = std::sqrt(
        balance.centerOfMassVelocity.x * balance.centerOfMassVelocity.x
            + balance.centerOfMassVelocity.y
                * balance.centerOfMassVelocity.y);
    const bool fallProtection =
        (balance.bodyUpDot < m_config.fallProtectionBodyUpDot
            || balance.torsoUpDot < m_config.fallProtectionBodyUpDot
            || (balance.captureMarginMeters
                    < m_config.fallProtectionCaptureMarginMeters
                && std::min(balance.bodyUpDot, balance.torsoUpDot)
                    < 0.78f))
        && horizontalSpeed
            >= m_config.fallProtectionMinimumSpeedMetersPerSecond;
    const float protectionWeight = fallProtection
        ? std::clamp(
            (m_config.fallProtectionBodyUpDot
                - std::min(balance.bodyUpDot, balance.torsoUpDot)) / 0.42f
                + horizontalSpeed * 0.18f,
            0.0f, 1.0f)
        : 0.0f;
    const auto poseTarget = [&](std::size_t linkIndex,
            RagdollAxis3D axis, float position, float stiffness,
            float damping, float torqueScale) {
        RagdollDriveTarget3D* target = findTarget(linkIndex, axis);
        if (target == nullptr || linkIndex >= state.joints.size()) return;
        const std::size_t axisIndex = static_cast<std::size_t>(axis);
        const RagdollAxisDefinition3D& definition =
            profile.links[linkIndex].inboundJoint.axes[axisIndex];
        target->positionRadians = std::clamp(
            position, definition.minimumRadians, definition.maximumRadians);
        target->velocityRadiansPerSecond = std::clamp(
            (target->positionRadians
                - state.joints[linkIndex].positionRadians[axisIndex])
                * m_config.jointPositionGain,
            -m_config.maximumJointVelocityRadiansPerSecond,
            m_config.maximumJointVelocityRadiansPerSecond);
        target->stiffnessScale = stiffness;
        target->dampingScale = damping;
        target->maximumTorqueScale = torqueScale;
    };

    // Tronco elástico, mas persistentemente posturado. Em repouso a cadeia
    // inteira converge com mais firmeza ao neutro; durante uma perturbação
    // parte dessa rigidez cede espaço ao torque dinâmico de contrarrotação.
    // É controle articular interno e permanece ativo sem auxílio na raiz.
    const float quietPostureWeight = std::clamp(
        (0.62f - urgency) / 0.54f, 0.0f, 1.0f);
    const float spineStiffness =
        m_config.standingStiffnessScale
        * (0.98f + 0.30f * quietPostureWeight);
    const float spineDamping =
        m_config.standingDampingScale
        * (1.24f + 0.16f * quietPostureWeight);
    for (const auto& [linkIndex, weight] :
            { std::pair { m_links.abdomen, 0.46f },
              std::pair { m_links.chest, 0.33f },
              std::pair { m_links.upperChest, 0.21f } }) {
        for (RagdollAxis3D axis :
                { RagdollAxis3D::Twist, RagdollAxis3D::Swing1,
                    RagdollAxis3D::Swing2 }) {
            poseTarget(linkIndex, axis, 0.0f,
                spineStiffness,
                spineDamping,
                m_config.spineTorqueAuthorityScale);
        }
        applyWorldTorque(profile, state, linkIndex,
            reaction.internalTorsoTorqueWorld, weight);
    }

    // Cadeia cervical ativa. Antes, pescoço e cabeça caíam no grupo
    // genérico de extremidades e recebiam apenas uma fração pequena da
    // autoridade postural. Isso deixava a cabeça pendurada para trás e ainda
    // deslocava o COM que o restante do corpo tentava corrigir. Os alvos
    // relativos continuam neutros; a referência vestibular em espaço global
    // é resolvida pelo WBC mais abaixo.
    for (RagdollAxis3D axis :
            { RagdollAxis3D::Twist, RagdollAxis3D::Swing1,
                RagdollAxis3D::Swing2 }) {
        poseTarget(m_links.neck, axis, 0.0f,
            m_config.neckPostureStiffnessScale,
            m_config.neckPostureDampingScale, 1.06f);
        poseTarget(m_links.head, axis, 0.0f,
            m_config.headPostureStiffnessScale,
            m_config.headPostureDampingScale, 1.02f);
    }

    // Em repouso os braços ficam abaixados. Sob perturbação eles abrem um
    // pouco e recebem contratorque, oferecendo inércia sem fazer uma pose
    // teatral ou instantânea.
    const float armOpening = std::clamp(
        0.52f * urgency + 0.92f * protectionWeight,
        0.0f, 1.02f);
    poseTarget(m_links.leftUpperArm, RagdollAxis3D::Swing1,
        1.20f - armOpening,
        m_config.standingStiffnessScale * 0.54f,
        m_config.standingDampingScale * 1.10f,
        m_config.armTorqueAuthorityScale);
    poseTarget(m_links.rightUpperArm, RagdollAxis3D::Swing1,
        -1.20f + armOpening,
        m_config.standingStiffnessScale * 0.54f,
        m_config.standingDampingScale * 1.10f,
        m_config.armTorqueAuthorityScale);
    if (fallProtection) {
        // Ao ultrapassar a região recuperável, abre os dois braços e estende
        // os cotovelos antes do contato. Não teletransporta a mão ao chão:
        // são somente alvos/torques articulares limitados, capazes de ceder
        // diante de uma colisão real.
        poseTarget(m_links.leftForearm, RagdollAxis3D::Twist,
            0.04f, 0.72f, 1.20f,
            m_config.armTorqueAuthorityScale * 1.18f);
        poseTarget(m_links.rightForearm, RagdollAxis3D::Twist,
            0.04f, 0.72f, 1.20f,
            m_config.armTorqueAuthorityScale * 1.18f);
        // Flexão curta dos joelhos absorve parte da energia e preserva
        // alcance para uma última passada, em vez de cair com pernas travadas.
        const float protectiveKneeFlexion =
            0.20f + protectionWeight * 0.34f;
        poseTarget(m_links.leftShin, RagdollAxis3D::Twist,
            protectiveKneeFlexion, 0.86f, 1.28f,
            m_config.legTorqueAuthorityScale);
        poseTarget(m_links.rightShin, RagdollAxis3D::Twist,
            protectiveKneeFlexion, 0.86f, 1.28f,
            m_config.legTorqueAuthorityScale);
    }
    applyWorldTorque(profile, state, m_links.leftUpperArm,
        reaction.internalArmTorqueWorld,
        0.5f * m_config.armCounterRotationResponse);
    applyWorldTorque(profile, state, m_links.rightUpperArm,
        reaction.internalArmTorqueWorld,
        0.5f * m_config.armCounterRotationResponse);

    // Tornozelos transformam o erro do ponto de captura em torque físico
    // somente nas solas realmente apoiadas.
    bool leftStance = balance.leftFootSupported;
    bool rightStance = balance.rightFootSupported;
    if (m_footworkInitialized) {
        const FootworkDebugState3D& footwork =
            m_footwork.planner().state();
        if (footwork.hasActiveSwing) {
            const std::size_t swing =
                footIndex3D(footwork.activeSwingSide);
            const bool released =
                footwork.feet[swing].phaseProgress > 1.0e-4f;
            if (released) {
                if (footwork.activeSwingSide == FootSide3D::Left) {
                    leftStance = false;
                } else {
                    rightStance = false;
                }
            }
        }
    }
    const int supportCount =
        static_cast<int>(leftStance)
        + static_cast<int>(rightStance);
    if (supportCount > 0) {
        const float supportWeight =
            1.0f / static_cast<float>(supportCount);
        const float hipStrategyScale =
            m_config.useMagicPelvisStabilization ? 0.34f : 1.45f;
        if (leftStance) {
            applyWorldTorque(profile, state, m_links.leftFoot,
                -reaction.internalSupportTorqueWorld, supportWeight);
            applyWorldTorque(profile, state, m_links.leftThigh,
                -reaction.internalPelvisTorqueWorld,
                supportWeight * hipStrategyScale);
            applyLegEndpointForce(profile, state, true,
                reaction.stancePushForceWorld, supportWeight);
        }
        if (rightStance) {
            applyWorldTorque(profile, state, m_links.rightFoot,
                -reaction.internalSupportTorqueWorld, supportWeight);
            applyWorldTorque(profile, state, m_links.rightThigh,
                -reaction.internalPelvisTorqueWorld,
                supportWeight * hipStrategyScale);
            applyLegEndpointForce(profile, state, false,
                reaction.stancePushForceWorld, supportWeight);
        }
    }

    // Mãos não devem herdar a rigidez alta das pernas. Cabeça e pescoço
    // possuem controle axial explícito acima e no WBC.
    for (RagdollDriveTarget3D& target : m_output.driveTargets) {
        const std::size_t index = target.linkIndex;
        const bool externallyManipulatedLeg =
            m_externalManipulationActive
            && (index == m_externalManipulatedLinkIndex
                || (m_externalManipulatedLinkIndex == m_links.leftFoot
                    && (index == m_links.leftThigh
                        || index == m_links.leftShin
                        || index == m_links.leftFoot))
                || (m_externalManipulatedLinkIndex == m_links.rightFoot
                    && (index == m_links.rightThigh
                        || index == m_links.rightShin
                        || index == m_links.rightFoot)));
        if (externallyManipulatedLeg) {
            // Em fallback (sem dinâmica inversa) os drives do PhysX também
            // precisam ceder. Zerar toda a cadeia evita que o joelho/quadril
            // transmitam a disputa do handle para a bacia.
            target.stiffnessScale = 0.0f;
            target.dampingScale = 0.0f;
            target.maximumTorqueScale = 0.0f;
            target.feedforwardTorqueNewtonMeters = 0.0f;
            continue;
        }
        const bool leg = index == m_links.leftThigh
            || index == m_links.leftShin || index == m_links.leftFoot
            || index == m_links.rightThigh
            || index == m_links.rightShin || index == m_links.rightFoot;
        const bool spine = index == m_links.abdomen
            || index == m_links.chest || index == m_links.upperChest;
        const bool arm = index == m_links.leftUpperArm
            || index == m_links.rightUpperArm;
        if (leg && !m_config.useMagicPelvisStabilization) {
            // Sem força na raiz, toda a autoridade precisa atravessar a
            // cadeia quadril-joelho-tornozelo. Aumentamos apenas o limite dos
            // motores; os alvos, forças e colisões continuam os mesmos.
            target.maximumTorqueScale *= 1.55f;
            target.dampingScale *= 1.12f;
        }
        const bool headChain = index == m_links.neck
            || index == m_links.head;
        if (!leg && !spine && !arm && !headChain) {
            target.stiffnessScale =
                m_config.standingStiffnessScale * 0.58f;
            target.dampingScale =
                m_config.standingDampingScale * 1.05f;
            target.maximumTorqueScale = 0.78f;
        }
    }
}

void ActiveRagdollController3D::update(
    const RagdollProfile3D& profile, const RagdollState3D& state,
    float deltaTime, const FootworkTerrainProbe3D* terrain,
    const RagdollDynamics3D* dynamics) {
    if (deltaTime <= 0.0f || !std::isfinite(deltaTime)
        || state.links.size() != profile.links.size()
        || state.joints.size() != profile.links.size()) {
        m_output = {};
        return;
    }
    m_ageSeconds += deltaTime;
    m_recoveryCooldownSeconds = std::max(
        0.0f, m_recoveryCooldownSeconds - deltaTime);
    m_stanceAdjustmentCooldownSeconds = std::max(
        0.0f, m_stanceAdjustmentCooldownSeconds - deltaTime);
    m_telemetry.timeInPhaseSeconds += deltaTime;

    NaturalBalanceConfig3D& balanceConfig = m_balance.config();
    balanceConfig.assistanceEnabled =
        m_config.useMagicPelvisStabilization;
    const float unassistedJointAuthority =
        m_config.useMagicPelvisStabilization ? 1.0f : 2.35f;
    balanceConfig.supportTorqueNewtonMetersPerMeter =
        m_config.ankleTorqueResponseNewtonMetersPerMeter
        * unassistedJointAuthority;
    balanceConfig.maximumSupportTorqueNewtonMeters =
        m_config.maximumAnkleBalanceTorqueNewtonMeters
        * (m_config.useMagicPelvisStabilization ? 1.0f : 2.0f);
    balanceConfig.stepCaptureMarginMeters =
        m_config.recoveryStepMinimumCaptureMarginMeters;
    balanceConfig.emergencyCaptureMarginMeters =
        m_config.recoveryStepEmergencyCaptureMarginMeters;
    balanceConfig.torsoInternalUprightStiffnessNewtonMeters =
        m_config.torsoUprightTorqueNewtonMetersPerRadian;
    balanceConfig.pelvisUprightStiffnessNewtonMeters =
        m_config.pelvisUprightTorqueNewtonMetersPerRadian;
    balanceConfig.pelvisInternalUprightStiffnessNewtonMeters =
        m_config.pelvisUprightTorqueNewtonMetersPerRadian;
    balanceConfig.angularMomentumDampingPerSecond =
        m_config.angularMomentumDampingGainPerSecond;
    // Apoio simples só recebe a margem vertical adicional quando há uma
    // passada de locomoção deliberada. Perder contato por estar na ponta do
    // pé não pode aumentar a "força mágica" e perpetuar o pé no ar.
    const NaturalBalanceState3D& previousBalance =
        m_balance.state();
    const bool plannedFootTransfer =
        m_footworkInitialized
        && m_footwork.planner().state().hasActiveSwing;
    const bool quietOrAccidentalRest =
        !m_walking && !m_walkPending
        && !plannedFootTransfer
        && (previousBalance.leftFootSupported
            || previousBalance.rightFootSupported)
        && previousBalance.bodyUpDot > 0.86f
        && previousBalance.centerOfMassVelocity.lengthSquared() < 0.09f;
    const float verticalAssistScale = quietOrAccidentalRest
        ? m_config.restingVerticalAssistScale : 1.0f;
    balanceConfig.bilateralSupportVerticalAssistScale =
        verticalAssistScale;
    balanceConfig.singleSupportVerticalAssistScale =
        verticalAssistScale;
    // Depois de cada pouso, a próxima transferência só começa quando os
    // dois pés realmente absorveram o momento da passada anterior. Durante
    // essa curta fase bilateral não continuamos puxando a bacia para frente:
    // fazê-lo aumentava artificialmente o próximo alcance e acumulava uma
    // queda até a terceira/quarta passada.
    const FootworkDebugState3D* balancePhaseFootwork =
        m_footworkInitialized ? &m_footwork.planner().state() : nullptr;
    const bool settlingBetweenWalkingSteps =
        m_walking && balancePhaseFootwork != nullptr
        && !balancePhaseFootwork->hasActiveSwing
        && balancePhaseFootwork->completedStepCount
            > m_walkStartCompletedFootworkSteps;
    const Vec3 balanceDesiredVelocity =
        m_walking && !settlingBetweenWalkingSteps
        ? m_forward
            * (m_config.walkSpeedMetersPerSecond * 0.40f)
        : m_walking ? Vec3 {} : m_recoveryVelocityReference;
    m_balance.setDesiredHorizontalVelocity(balanceDesiredVelocity);
    m_balance.update(profile, state, m_links, deltaTime);
    const NaturalBalanceState3D& balance = m_balance.state();
    const NaturalBalanceResponse3D& reaction = m_balance.response();
    const float horizontalCenterOfMassSpeed = std::sqrt(
        balance.centerOfMassVelocity.x * balance.centerOfMassVelocity.x
            + balance.centerOfMassVelocity.y
                * balance.centerOfMassVelocity.y);
    Vec3 horizontalCenterOfMassVelocity =
        balance.centerOfMassVelocity;
    horizontalCenterOfMassVelocity.z = 0.0f;
    if (!m_walking && !balance.fallen
        && (balance.recoveryUrgency > 0.12f
            || m_reactiveRecoveryStep)) {
        Vec3 travelTarget = horizontalCenterOfMassVelocity
            * m_config.impactTravelVelocityFraction;
        if (travelTarget.length()
                > m_config.maximumImpactTravelSpeedMetersPerSecond) {
            travelTarget = travelTarget.normalized()
                * m_config.maximumImpactTravelSpeedMetersPerSecond;
        }
        m_recoveryVelocityReference = approachVector(
            m_recoveryVelocityReference, travelTarget,
            m_config.impactTravelAccelerationMetersPerSecondSquared
                * deltaTime);
    } else {
        m_recoveryVelocityReference = approachVector(
            m_recoveryVelocityReference, {},
            m_config.impactTravelBrakingMetersPerSecondSquared
                * deltaTime);
    }
    if (balance.captureMarginMeters
            <= m_config.recoveryStepMinimumCaptureMarginMeters) {
        m_outsideSupportSeconds += deltaTime;
    } else if (balance.captureMarginMeters > -0.02f) {
        m_outsideSupportSeconds = 0.0f;
    }

    if (!m_footworkInitialized && terrain != nullptr) {
        m_footworkInitialized = m_footwork.reset(profile, state,
            yawFromOrientation(m_spawnOrientation), *terrain,
            PhysicalFootworkMode3D::ReactiveBalance);
        if (m_footworkInitialized) {
            configurePhysicalFootwork();
            m_lastCompletedFootworkSteps =
                m_footwork.planner().state().completedStepCount;
        }
    }

    const Vec3 leftFootPosition =
        state.links[m_links.leftFoot].position;
    const Vec3 rightFootPosition =
        state.links[m_links.rightFoot].position;
    const float signedStanceWidth = dot(
        leftFootPosition - rightFootPosition, m_balance.lateral());
    const FootworkDebugState3D* preStepFootwork =
        m_footworkInitialized ? &m_footwork.planner().state() : nullptr;
    const bool calmPostureRepair =
        balance.recoveryUrgency < 0.055f
        && horizontalCenterOfMassSpeed < 0.12f
        && balance.captureMarginMeters > 0.055f;
    const bool idlePostureWindow =
        terrain != nullptr && preStepFootwork != nullptr
        && !preStepFootwork->hasActiveSwing
        && !m_walking && !m_walkPending
        && !m_externalManipulationActive
        && calmPostureRepair
        && !balance.fallen
        && balance.bodyUpDot > 0.68f
        && balance.supportConfidence > 0.60f
        && m_stanceAdjustmentCooldownSeconds <= 0.0f;

    // A base tem sinal: esquerda deve permanecer do lado esquerdo da
    // direita. Usar abs() aqui classificava uma base completamente cruzada
    // como "larga e estável". A verificação permanece viva durante toda a
    // existência do ragdoll; equilíbrio não é uma conquista descartável.
    const bool bilateralSupport =
        balance.leftFootSupported && balance.rightFootSupported;
    const float leftSoleHeight = physicalSoleCenter(
        profile, state, m_links.leftFoot).z - balance.supportCenter.z;
    const float rightSoleHeight = physicalSoleCenter(
        profile, state, m_links.rightFoot).z - balance.supportCenter.z;
    const auto updateStableSupport = [deltaTime](
            float& elapsed, bool supported, float soleHeight) {
        if (supported && soleHeight < 0.055f) {
            elapsed += deltaTime;
        } else {
            elapsed = 0.0f;
        }
    };
    updateStableSupport(m_leftSupportStableSeconds,
        balance.leftFootSupported, leftSoleHeight);
    updateStableSupport(m_rightSupportStableSeconds,
        balance.rightFootSupported, rightSoleHeight);
    // O estimador conserva contato por alguns frames para filtrar ruído do
    // PhysX. Essa memória não pode fazer um pé erguido pela Physgun parecer
    // apoio bilateral válido e disparar reparo da base durante o grab.
    const bool bilateralFeetNearGround =
        leftSoleHeight < 0.045f && rightSoleHeight < 0.045f;
    const bool crossedStance = signedStanceWidth <= 0.0f;
    const bool weakStance =
        signedStanceWidth < m_config.minimumStableStanceWidthMeters;
    if (idlePostureWindow && bilateralSupport
            && bilateralFeetNearGround && weakStance) {
        m_narrowStanceSeconds += deltaTime;
    } else if (signedStanceWidth
            >= m_config.stableStanceReleaseWidthMeters
        || !bilateralSupport || m_walking || m_walkPending) {
        m_narrowStanceSeconds = std::max(
            0.0f, m_narrowStanceSeconds - deltaTime * 4.0f);
    }

    const bool idleWithoutSwing = terrain != nullptr
        && preStepFootwork != nullptr
        && !preStepFootwork->hasActiveSwing
        && !m_walking && !m_walkPending && !balance.fallen
        && !m_externalManipulationActive
        && calmPostureRepair;
    if (idleWithoutSwing && !balance.leftFootSupported
        && balance.rightFootSupported) {
        m_leftUnsupportedSeconds += deltaTime;
    } else {
        m_leftUnsupportedSeconds = 0.0f;
    }
    if (idleWithoutSwing && balance.leftFootSupported
        && !balance.rightFootSupported) {
        m_rightUnsupportedSeconds += deltaTime;
    } else {
        m_rightUnsupportedSeconds = 0.0f;
    }

    const bool leftNeedsReplant =
        m_leftUnsupportedSeconds
            >= m_config.unsupportedFootReplantDelaySeconds;
    const bool rightNeedsReplant =
        m_rightUnsupportedSeconds
            >= m_config.unsupportedFootReplantDelaySeconds;
    const float stanceConfirmation = crossedStance
        ? std::min(0.055f, m_config.narrowStanceConfirmationSeconds)
        : m_config.narrowStanceConfirmationSeconds;
    const bool baseNeedsRepair =
        idlePostureWindow && bilateralSupport && bilateralFeetNearGround
        && m_narrowStanceSeconds >= stanceConfirmation;
    bool postureRepairStarted = false;
    if (idlePostureWindow
        && (leftNeedsReplant || rightNeedsReplant || baseNeedsRepair)) {
        const Vec3 pelvisPosition =
            state.links[m_links.pelvis].position;
        const float leftOffset = dot(
            leftFootPosition - pelvisPosition, m_balance.lateral());
        const float rightOffset = dot(
            rightFootPosition - pelvisPosition, m_balance.lateral());
        const float desiredHalf =
            m_config.desiredStandingStanceWidthMeters * 0.5f;
        const float leftError = std::abs(desiredHalf - leftOffset);
        const float rightError = std::abs(-desiredHalf - rightOffset);
        const FootSide3D swingSide = leftNeedsReplant
            ? FootSide3D::Left
            : rightNeedsReplant
                ? FootSide3D::Right
                : leftError >= rightError
                    ? FootSide3D::Left : FootSide3D::Right;
        const Vec3 supportPosition = swingSide == FootSide3D::Left
            ? rightFootPosition : leftFootPosition;
        const float lateralSign =
            swingSide == FootSide3D::Left ? 1.0f : -1.0f;
        const float sagittalSign =
            swingSide == FootSide3D::Left ? 1.0f : -1.0f;
        Vec3 stanceTarget = supportPosition
            + m_balance.lateral()
                * (lateralSign
                    * m_config.desiredStandingStanceWidthMeters)
            + m_balance.forward()
                * (sagittalSign
                    * m_config.stanceSagittalOffsetMeters);
        stanceTarget.z = balance.supportCenter.z;
        if (m_footwork.requestRecoveryStep(
                swingSide, stanceTarget, false, *terrain)) {
            m_reactiveRecoveryStep = false;
            m_reactiveRecoveryTrailingStep = false;
            postureRepairStarted = true;
            m_narrowStanceSeconds = 0.0f;
            m_leftUnsupportedSeconds = 0.0f;
            m_rightUnsupportedSeconds = 0.0f;
            m_stanceAdjustmentCooldownSeconds =
                m_config.stanceAdjustmentCooldownSeconds;
        }
    }

    const bool readyToWalk =
        !balance.fallen && balance.supportConfidence > 0.80f
        && balance.bodyUpDot > 0.88f && balance.torsoUpDot > 0.86f
        && m_ageSeconds >= m_config.minimumStandingBeforeWalkSeconds;
    if (m_walkPending && readyToWalk) {
        m_walkPreparationSeconds += deltaTime;
    } else if (m_walkPending) {
        m_walkPreparationSeconds = 0.0f;
    }
    if (m_walkPending && readyToWalk
        && m_walkPreparationSeconds
            >= m_config.walkPreparationSeconds) {
        m_walkStart = state.links[m_links.pelvis].position;
        m_walkStartCompletedFootworkSteps =
            m_footworkInitialized
            ? m_footwork.planner().state().completedStepCount
            : 0;
        m_lastWalkDistanceMeters = 0.0f;
        m_walkPending = false;
        m_walkPreparationSeconds = 0.0f;
        m_walking = true;
    }

    Vec3 desiredVelocity = m_walking
        ? m_forward * m_config.walkSpeedMetersPerSecond
        : m_recoveryVelocityReference;
    const bool recoveryReady =
        !balance.fallen
        && m_ageSeconds >= m_config.recoverySettlingSeconds
        && balance.supportConfidence
            >= m_config.minimumRecoverySupportConfidence
        && m_recoveryCooldownSeconds <= 0.0f;
    const Vec3 commandedHorizontalVelocity =
        m_walking
            ? m_forward * m_config.walkSpeedMetersPerSecond
            : m_recoveryVelocityReference;
    const Vec3 unexpectedVelocity =
        balance.centerOfMassVelocity - commandedHorizontalVelocity;
    const float unexpectedLateralSpeed =
        std::abs(dot(unexpectedVelocity, m_balance.lateral()));
    const float unexpectedForwardSpeed =
        dot(unexpectedVelocity, m_balance.forward());
    const bool walkingDisturbance =
        !m_walking
        || unexpectedLateralSpeed > 0.16f
        || unexpectedForwardSpeed > 0.32f
        || (unexpectedForwardSpeed < -0.48f
            && balance.bodyUpDot < 0.95f);
    // O passo precisa começar ANTES de o corpo depender da assistência. A
    // versão anterior fazia justamente o inverso quando a assistência estava
    // desligada: aguardava a margem chegar a -18/-28 cm, instante em que a
    // perna humana já não alcança o ponto de captura a tempo. O mesmo limiar
    // físico vale nos dois modos; a assistência apenas acrescenta robustez
    // residual depois das estratégias articulares.
    const float recoveryCaptureThreshold =
        m_config.recoveryStepMinimumCaptureMarginMeters;
    const float emergencyCaptureThreshold =
        m_config.recoveryStepEmergencyCaptureMarginMeters;
    std::optional<FootSide3D> requestedRecoverySide;
    bool requestedTrailingRecoveryStep = false;
    bool requestedSupportReacquisition = false;
    const float recoveryCenterOfMassHeight = std::max(
        0.35f, balance.centerOfMass.z - balance.supportCenter.z);
    const float recoveryCaptureOmega =
        std::sqrt(9.81f / recoveryCenterOfMassHeight);
    // O estimador de risco remove a velocidade que decidimos acompanhar
    // depois de um impacto. Isso impede uma frenagem artificial, mas essa
    // versão relativa NÃO é um lugar do mundo onde o pé possa pousar. O alvo
    // físico usa sempre a velocidade absoluta do COM.
    Vec3 absoluteCapturePoint = balance.centerOfMass
        + horizontalCenterOfMassVelocity
            * (1.0f / recoveryCaptureOmega);
    absoluteCapturePoint.z = balance.supportCenter.z;
    // O primeiro pé externo cria uma base larga e, por alguns frames, a
    // margem volta a ser positiva. Isso não significa que a perturbação
    // acabou: com velocidade residual alta, esperar a margem sair novamente
    // desperdiça quase metade do tempo de voo do pé interno. Encadeamos o
    // shuffle assim que o primeiro heel-strike está confirmado.
    const FootSide3D velocityExternalSide =
        dot(horizontalCenterOfMassVelocity, m_balance.lateral()) >= 0.0f
        ? FootSide3D::Left : FootSide3D::Right;
    const bool lastStepWasExternal =
        m_hasLastReactiveRecoverySide
        && m_lastReactiveRecoverySide == velocityExternalSide;
    const float lastSupportStableSeconds =
        m_lastReactiveRecoverySide == FootSide3D::Left
        ? m_leftSupportStableSeconds : m_rightSupportStableSeconds;
    const bool predictiveRecoveryCadence =
        !m_walking && m_hasLastReactiveRecoverySide
        && balance.leftFootSupported && balance.rightFootSupported
        // Depois da primeira captura o pé interno precisa partir já; o pé
        // externo que acabou de tocar ainda é o próprio novo apoio. Depois
        // do shuffle, porém, só iniciamos outro passo externo quando esse pé
        // interno demonstrou contato persistente.
        && (lastStepWasExternal
            || lastSupportStableSeconds >= 0.035f)
        && horizontalCenterOfMassSpeed > 0.62f
        && signedStanceWidth
            > m_config.desiredStandingStanceWidthMeters * 1.38f;
    const bool reactiveSupportReady =
        !m_hasLastReactiveRecoverySide
        || lastStepWasExternal
        || lastSupportStableSeconds >= 0.12f;
    const bool requestRecovery =
        recoveryReady && terrain != nullptr && !postureRepairStarted
        && !m_externalManipulationActive
        && walkingDisturbance
        // A marcha já é uma sequência de passos de captura. Injetar uma
        // segunda "passada de recuperação" entre dois apoios quebrava a
        // alternância e fazia o mesmo pé avançar repetidamente. Durante a
        // marcha só tomamos o controle em uma queda realmente extrema.
        && (!m_walking || balance.bodyUpDot < 0.72f)
        && (balance.leftFootSupported || balance.rightFootSupported)
        && reactiveSupportReady
        && (predictiveRecoveryCadence
            || balance.recoveryUrgency
                >= m_config.recoveryStepTriggerUrgency)
        && (predictiveRecoveryCadence
            || balance.captureMarginMeters
                <= recoveryCaptureThreshold)
        && (horizontalCenterOfMassSpeed
                >= m_config
                    .recoveryStepMinimumHorizontalSpeedMetersPerSecond
            || balance.captureMarginMeters
                <= emergencyCaptureThreshold
            || m_outsideSupportSeconds
                >= m_config.recoveryStepStaticOutsideDelaySeconds)
        && (predictiveRecoveryCadence
            || reaction.recoveryVelocityWorld.lengthSquared() > 1.0e-6f);
    if (requestRecovery) {
        const Vec3 captureOffset =
            balance.capturePoint - balance.supportCenter;
        // A perna externa deve ser escolhida pela direcao em que o corpo
        // realmente esta escapando. Usar apenas capturePoint-supportCenter
        // introduz o deslocamento do unico pe que ainda esta apoiado: numa
        // perturbacao lateral esse termo podia parecer sagital e selecionar
        // justamente o pe interno/descarregado. A velocidade inesperada e a
        // melhor estimativa enquanto ha movimento; perto da velocidade zero
        // voltamos ao erro de captura para tambem corrigir quedas quase
        // estaticas.
        Vec3 sideSelectionDirection = unexpectedVelocity;
        sideSelectionDirection.z = 0.0f;
        if (sideSelectionDirection.lengthSquared() < 0.0025f) {
            sideSelectionDirection = captureOffset;
            sideSelectionDirection.z = 0.0f;
        }
        const float lateralEscape =
            dot(sideSelectionDirection, m_balance.lateral());
        const float forwardEscape =
            dot(sideSelectionDirection, m_balance.forward());
        const FootworkDebugState3D& plannedFootwork =
            m_footwork.planner().state();
        if (plannedFootwork.hasActiveSwing) {
            const std::size_t activeSwing = footIndex3D(
                plannedFootwork.activeSwingSide);
            const std::size_t activeSupport = 1u - activeSwing;
            const bool swingHasLanded = activeSwing == 0
                ? balance.leftFootSupported
                : balance.rightFootSupported;
            Vec3 swingFootholdError = physicalSoleCenter(
                profile, state,
                activeSwing == 0
                    ? m_links.leftFoot : m_links.rightFoot)
                - plannedFootwork.feet[activeSwing].target.position;
            swingFootholdError.z = 0.0f;
            const bool swingReachedPlannedFoothold =
                swingFootholdError.length() <= 0.10f;
            const bool oldSupportWasLost = activeSupport == 0
                ? !balance.leftFootSupported
                : !balance.rightFootSupported;
            if (swingHasLanded && swingReachedPlannedFoothold
                && oldSupportWasLost
                && plannedFootwork.feet[activeSwing].phaseProgress
                    >= 0.58f) {
                // A perna de captura chegou ao chão, mas a rajada já retirou
                // a antiga perna de apoio. Insistir no alvo ideal da primeira
                // passada deixava o WBC sem nenhum contato modelado: o pé
                // novo era excluído por ser "swing" e o antigo já estava no
                // ar. Aceitamos a topologia física real e reclassificamos o
                // antigo apoio como o novo pé de recuperação. Não há troca
                // cinematica de transforms; o novo passo ainda é realizado
                // integralmente por torques e colisão.
                requestedRecoverySide =
                    oppositeFoot3D(plannedFootwork.activeSwingSide);
                requestedTrailingRecoveryStep = true;
                requestedSupportReacquisition = true;
            } else {
                // Enquanto o apoio antigo ainda existe, a única perna que
                // pode reagir sem uma troca impossível é a já descarregada.
                requestedRecoverySide =
                    plannedFootwork.activeSwingSide;
            }
        } else if (std::abs(lateralEscape)
                > std::abs(forwardEscape) * 0.55f) {
            // Estrategia lateral sem cruzamento: ao cair para a esquerda, o
            // pe esquerdo (externo) amplia a base naquela direcao. Escolher o
            // pe interno so porque ele descarregou momentaneamente produz uma
            // passada cruzada e reduz ainda mais o poligono de apoio.
            const FootSide3D externalSide = lateralEscape >= 0.0f
                ? FootSide3D::Left : FootSide3D::Right;
            // Uma rajada continua nao pode mandar o mesmo pe externo cada
            // vez mais longe. Depois da primeira passada de captura a perna
            // interna precisa acompanhar o corpo e reconstruir a largura da
            // base; na passada seguinte o pe externo volta a liderar. Isso
            // produz o padrao humano side-step (externo, interno, externo)
            // sem cruzar as pernas.
            const bool wideAfterExternalCatch =
                m_hasLastReactiveRecoverySide
                && m_lastReactiveRecoverySide == externalSide
                && signedStanceWidth
                    > m_config.desiredStandingStanceWidthMeters * 1.42f;
            requestedRecoverySide = wideAfterExternalCatch
                ? oppositeFoot3D(externalSide) : externalSide;
            requestedTrailingRecoveryStep = wideAfterExternalCatch;
        } else if (balance.leftFootSupported
                != balance.rightFootSupported) {
            // No eixo sagital, o pe ja descarregado continua sendo a opcao
            // mais rapida e nao cria cruzamento lateral.
            requestedRecoverySide = balance.leftFootSupported
                ? FootSide3D::Right : FootSide3D::Left;
        } else {
            // No eixo da marcha, avança o pé que ficou para trás na direção
            // da queda. Isso evita alternância arbitrária e cruzamento.
            const Vec3 direction =
                reaction.recoveryVelocityWorld.normalized();
            const float leftProgress =
                dot(state.links[m_links.leftFoot].position, direction);
            const float rightProgress =
                dot(state.links[m_links.rightFoot].position, direction);
            requestedRecoverySide = leftProgress <= rightProgress
                ? FootSide3D::Left : FootSide3D::Right;
        }
        Vec3 direction = horizontalCenterOfMassVelocity;
        if (direction.lengthSquared() < 0.0144f) {
            direction = reaction.recoveryVelocityWorld;
        }
        direction = direction.normalized();
        const float predictiveOvershoot = std::clamp(
            horizontalCenterOfMassSpeed * 0.085f, 0.0f, 0.11f);
        Vec3 target = absoluteCapturePoint
            + direction * (m_config.recoveryLandingOvershootMeters
                + predictiveOvershoot);
        target.z = balance.supportCenter.z;
        const bool emergencyStep =
            balance.recoveryUrgency
                >= m_config.emergencyFastStepUrgency
            || horizontalCenterOfMassSpeed >= 0.24f
            || balance.captureMarginMeters
                <= m_config.recoveryStepEmergencyCaptureMarginMeters;
        // A perna externa cruza uma distancia grande e precisa de folga para
        // nao raspar no terreno. A perna interna de acompanhamento faz um
        // shuffle: levantar 10 cm desperdicava toda a janela recuperavel e o
        // COM passava do apoio antes do touchdown. A configuracao e copiada
        // para a passada no request e restaurada em seguida, sem alterar a
        // marcha nominal nem a proxima passada externa.
        FootworkConfig3D& recoveryGait =
            m_footwork.planner().config();
        const float savedFastClearance =
            recoveryGait.fastSwingClearanceMeters;
        if (requestedTrailingRecoveryStep) {
            // O pe interno nao executa uma passada alta: ele desliza a sola
            // rente ao terreno para reconstruir a base antes que o COM
            // ultrapasse o pe externo. Qualquer arco, mesmo de 2--3 cm no
            // alvo, vira quase 10 cm na perna fisica sob aceleracao lateral
            // e faz o contato chegar tarde demais.
            recoveryGait.fastSwingClearanceMeters = 0.0f;
        }
        const bool started = requestedSupportReacquisition
            ? m_footwork.recoverUnsupportedFoot(
                *requestedRecoverySide, target, emergencyStep, *terrain)
            : m_footwork.requestRecoveryStep(
                *requestedRecoverySide, target, emergencyStep, *terrain);
        recoveryGait.fastSwingClearanceMeters = savedFastClearance;
        if (started) {
            m_reactiveRecoveryStep = true;
            m_reactiveRecoveryTrailingStep =
                requestedTrailingRecoveryStep;
            m_lastReactiveRecoverySide = *requestedRecoverySide;
            m_hasLastReactiveRecoverySide = true;
            m_outsideSupportSeconds = 0.0f;
        }
    }

    // O ponto de captura continua se movendo durante o voo. Em equilibrio
    // reativo congelar o alvo no lift-off persegue a queda de centenas de
    // milissegundos atras; atualizamos o mesmo passo sem reiniciar a curva.
    if (!m_walking && terrain != nullptr && m_footworkInitialized) {
        const FootworkDebugState3D& plan =
            m_footwork.planner().state();
        if (m_reactiveRecoveryStep && plan.hasActiveSwing
            && (balance.recoveryUrgency > 0.075f
                || balance.captureMarginMeters < 0.045f)) {
            // O risco é calculado no referencial de velocidade acompanhada,
            // porém o pé em voo precisa continuar perseguindo o movimento
            // absoluto do corpo. Usar a direção residual aqui fazia o alvo
            // recuar assim que o controlador reconhecia parte do impulso.
            Vec3 recoveryDirection = horizontalCenterOfMassVelocity;
            if (recoveryDirection.lengthSquared() < 0.0144f) {
                recoveryDirection = balance.recoveryDirectionWorld;
            }
            recoveryDirection.z = 0.0f;
            if (recoveryDirection.lengthSquared() > 1.0e-8f) {
                recoveryDirection = recoveryDirection.normalized();
                const float liveOvershoot = std::clamp(
                    horizontalCenterOfMassSpeed * 0.12f,
                    0.055f, 0.18f);
                Vec3 liveTarget = absoluteCapturePoint
                    + recoveryDirection * liveOvershoot;
                liveTarget.z = balance.supportCenter.z;
                // planFootstep3D já projeta alcance e impede cruzamento em
                // relação à sola de apoio. Uma segunda correção lateral aqui
                // usava o frame do corpo em rotação e podia espelhar o alvo
                // para trás durante um impacto diagonal.
                static_cast<void>(m_footwork.retargetActiveStep(
                    liveTarget, false, *terrain));
            }
        }
    }

    // Durante a marcha, a posição de pouso é uma variável de horizonte
    // recedente. O DCM muda enquanto a perna está no ar; congelar o alvo no
    // lift-off fazia o pé chegar ao lugar onde a queda estava 300 ms atrás.
    // Retarget não reinicia a curva e não cria passos extras.
    if (m_walking && terrain != nullptr && m_footworkInitialized) {
        const FootworkDebugState3D& plan =
            m_footwork.planner().state();
        if (plan.hasActiveSwing) {
            const std::size_t swing =
                footIndex3D(plan.activeSwingSide);
            const bool physicallyReleased =
                plan.feet[swing].weightTransferProgress >= 0.999f;
            if (physicallyReleased
                && plan.feet[swing].phaseProgress > 0.08f
                && plan.feet[swing].phaseProgress < 0.62f
                && balance.captureMarginMeters < -0.12f) {
                const float centerOfMassHeight = std::max(
                    0.35f, balance.centerOfMass.z
                        - balance.supportCenter.z);
                const float captureOmega =
                    std::sqrt(9.81f / centerOfMassHeight);
                // Para posicionar o pé usamos o ponto de captura absoluto.
                // A estimativa de estabilidade do NaturalBalance remove a
                // velocidade comandada (correto para não classificar marcha
                // nominal como queda), mas essa versão relativa colocaria o
                // pé atrás do corpo justamente quando ele precisa interceptar
                // o momento linear real.
                Vec3 captureTarget = balance.centerOfMass
                    + balance.centerOfMassVelocity
                        * (1.0f / captureOmega)
                    + m_forward * 0.055f;
                // Uma correção de horizonte recedente pode alongar/encurtar
                // a passada, mas não transformar uma perna já em voo para
                // frente numa passada para trás por causa de um único pico
                // do estimador. Mantemos o pouso adiante do pé de apoio e
                // limitamos a correção lateral em torno da faixa planejada.
                const std::size_t support = 1u - swing;
                const std::size_t supportLink = support == 0
                    ? m_links.leftFoot : m_links.rightFoot;
                const Vec3 supportSole =
                    physicalSoleCenter(profile, state, supportLink);
                const float supportForward =
                    dot(supportSole, m_forward);
                const float boundedForward = std::clamp(
                    dot(captureTarget, m_forward),
                    supportForward + 0.14f,
                    supportForward + 0.58f);
                const float plannedLateral = dot(
                    plan.feet[swing].target.position,
                    m_balance.lateral());
                const float boundedLateral = std::clamp(
                    dot(captureTarget, m_balance.lateral()),
                    plannedLateral - 0.075f,
                    plannedLateral + 0.075f);
                captureTarget =
                    m_forward * boundedForward
                    + m_balance.lateral() * boundedLateral
                    + Vec3 { 0.0f, 0.0f, balance.supportCenter.z };
                captureTarget.z = balance.supportCenter.z;
                static_cast<void>(m_footwork.retargetActiveStep(
                    captureTarget,
                    balance.captureMarginMeters < -0.18f, *terrain));
            }
        }
    }

    if (m_footworkInitialized && terrain != nullptr) {
        m_footwork.planner().setPhysicalContactFeedback(
            balance.leftFootSupported,
            balance.rightFootSupported,
            std::array<FootPose3D, 2> {
                physicalSolePose(profile, state, m_links.leftFoot),
                physicalSolePose(profile, state, m_links.rightFoot)
            });
        const FootworkDebugState3D& preUpdatePlan =
            m_footwork.planner().state();
        // Uma perda momentânea do contato de apoio durante o touchdown não
        // cancela a passada que já está no ar. O contato da sola que pousa
        // precisa ser aceito primeiro; se a outra sola continuar sem apoio, o
        // bloco de single-support abaixo inicia uma passada corretiva no tick
        // seguinte. Trocar imediatamente de perna aqui fazia duas oscilações de
        // contato descartarem a passada sem contá-la e injetava alvos opostos.
        const bool plannerBetweenSteps = !preUpdatePlan.hasActiveSwing;
        const bool landingConfirmed =
            balance.leftFootSupported && balance.rightFootSupported;
        const bool singleSupport =
            balance.leftFootSupported != balance.rightFootSupported;
        m_footwork.planner().config().weightTransferDurationSeconds =
            0.68f;
        bool activeStepReleaseAllowed = true;
        if (preUpdatePlan.hasActiveSwing) {
            const bool supportFootContact =
                preUpdatePlan.activeSwingSide == FootSide3D::Left
                ? balance.rightFootSupported
                : balance.leftFootSupported;
            const float supportStableSeconds =
                preUpdatePlan.activeSwingSide == FootSide3D::Left
                ? m_rightSupportStableSeconds
                : m_leftSupportStableSeconds;
            // Nunca retirar do chao o pe externo carregado antes de a perna
            // interna realmente aceitar peso. Isso vale tambem para passos
            // reflexivos, nao apenas para locomocao planejada.
            // Nunca liberar o único contato real. A regra balística antiga
            // aceitava "qualquer pé" apoiado, inclusive o próprio pé que
            // seria levantado; após um heel-strike breve isso produzia um
            // frame sem contato no QP e toda a atuação desaparecia. Se a
            // perna de stance perdeu o chão, urgentPlantFoot a readquire
            // antes do próximo toe-off.
            // O shuffle interno acontece enquanto o pé externo recém-
            // plantado recebe carga e precisa partir imediatamente. Já o
            // próximo passo externo só pode retirar esse pé depois de o
            // shuffle ter sustentado peso por uma janela real; um único
            // contato de colisão não constitui apoio.
            const float requiredStableSupportSeconds =
                m_reactiveRecoveryTrailingStep ? 0.0f : 0.075f;
            activeStepReleaseAllowed = supportFootContact
                && supportStableSeconds
                    >= requiredStableSupportSeconds;
        }
        if (m_walking && preUpdatePlan.hasActiveSwing) {
            const std::size_t swing =
                footIndex3D(preUpdatePlan.activeSwingSide);
            const std::size_t stance = 1u - swing;
            const float transferProgress =
                preUpdatePlan.feet[swing].weightTransferProgress;
            if (transferProgress < 0.999f) {
                activeStepReleaseAllowed = false;
            } else if (balance.leftFootSupported
                && balance.rightFootSupported) {
                const std::size_t stanceLink = stance == 0
                    ? m_links.leftFoot : m_links.rightFoot;
                const Vec3 stancePosition =
                    physicalSoleCenter(profile, state, stanceLink);
                const float centerOfMassHeight = std::max(
                    0.35f, balance.centerOfMass.z
                        - balance.supportCenter.z);
                const float omega =
                    std::sqrt(9.81f / centerOfMassHeight);
                const Vec3 divergentComponent =
                    balance.centerOfMass
                    + balance.centerOfMassVelocity * (1.0f / omega);
                const Vec3 stanceError =
                    divergentComponent - stancePosition;
                const float lateralError =
                    std::abs(dot(stanceError, m_balance.lateral()));
                const float signedSagittalError =
                    dot(stanceError, m_balance.forward());
                m_telemetry.toeOffLateralErrorMeters = lateralError;
                m_telemetry.toeOffSagittalErrorMeters =
                    signedSagittalError;
                // Lateralmente o DCM permanece dentro da sola de apoio.
                // Longitudinalmente a janela é assimétrica: caminhar é uma
                // queda controlada e o ponto de captura deve poder avançar
                // rumo ao próximo pouso. Exigir |erro| pequeno transformava
                // a preparação em uma pose parada e perdia o instante de
                // toe-off assim que o push-off começava.
                activeStepReleaseAllowed =
                    lateralError <= 0.060f
                    // O DCM precisa estar alguns centímetros dentro da
                    // borda traseira efetivamente utilizável. Liberar com o
                    // COM atrás da sola faz a reação do solo acelerar a
                    // queda para trás (c_ddot = w²(c-zmp)).
                    && signedSagittalError >= -0.075f
                    && signedSagittalError <= 0.330f
                    && balance.bodyUpDot > 0.94f
                    && balance.torsoUpDot > 0.84f;
            }
        }
        m_footwork.planner().setActiveStepReleaseAllowed(
            activeStepReleaseAllowed);
        const Vec3 worldUp { 0.0f, 0.0f, 1.0f };
        const float leftSoleUp = dot(
            state.links[m_links.leftFoot].orientation.rotate(worldUp),
            worldUp);
        const float rightSoleUp = dot(
            state.links[m_links.rightFoot].orientation.rotate(worldUp),
            worldUp);
        const Vec3 landingVelocityReference =
            settlingBetweenWalkingSteps ? Vec3 {} : desiredVelocity;
        const float lateralVelocityError = std::abs(dot(
            balance.centerOfMassVelocity - landingVelocityReference,
            m_balance.lateral()));
        const bool dynamicallyReadyForNextStep =
            balance.bodyUpDot > 0.94f
            && balance.torsoUpDot > 0.90f
            && balance.captureMarginMeters > 0.045f
            && lateralVelocityError < 0.12f
            && leftSoleUp > 0.975f
            && rightSoleUp > 0.975f;
        if (plannerBetweenSteps && landingConfirmed
            && dynamicallyReadyForNextStep) {
            m_confirmedLandingSeconds += deltaTime;
        } else {
            m_confirmedLandingSeconds = 0.0f;
        }
        bool explicitCatchStarted = false;
        if (m_walking && plannerBetweenSteps && landingConfirmed
            && (balance.captureMarginMeters < 0.045f
                || std::abs(dot(
                    balance.centerOfMassVelocity
                        - landingVelocityReference,
                    m_balance.lateral())) > 0.18f)) {
            const float centerOfMassHeight = std::max(
                0.35f, balance.centerOfMass.z
                    - balance.supportCenter.z);
            const float omega =
                std::sqrt(9.81f / centerOfMassHeight);
            Vec3 target = balance.centerOfMass
                + balance.centerOfMassVelocity * (1.0f / omega)
                + m_forward * 0.055f;
            target.z = balance.supportCenter.z;
            const float lateralEscape = dot(
                target - balance.supportCenter, m_balance.lateral());
            const FootSide3D captureSide = lateralEscape >= 0.0f
                ? FootSide3D::Left : FootSide3D::Right;
            explicitCatchStarted = m_footwork.requestRecoveryStep(
                captureSide, target, true, *terrain);
        }
        if (m_walking && plannerBetweenSteps && singleSupport) {
            const FootSide3D unsupported =
                balance.leftFootSupported
                ? FootSide3D::Right : FootSide3D::Left;
            // Com feedback físico, o planejador só encerra TouchDown depois
            // de contato confirmado. Portanto, chegar aqui com um pé solto
            // representa uma nova perda real de apoio (não uma curva que
            // terminou cedo) e esse mesmo pé precisa ser recolocado.
            const float centerOfMassHeight = std::max(
                0.35f, balance.centerOfMass.z
                    - balance.supportCenter.z);
            const float omega =
                std::sqrt(9.81f / centerOfMassHeight);
            Vec3 target = balance.centerOfMass
                + balance.centerOfMassVelocity * (1.0f / omega)
                + m_forward * 0.04f;
            target.z = balance.supportCenter.z;
            explicitCatchStarted = explicitCatchStarted
                ||
                m_footwork.requestRecoveryStep(
                    unsupported, target, true, *terrain);
        }
        m_footwork.planner().setAutomaticSteppingEnabled(
            m_walking && plannerBetweenSteps
                && !explicitCatchStarted && landingConfirmed
                && dynamicallyReadyForNextStep
                && m_confirmedLandingSeconds >= 0.14f);
        // A marcha já possui dorsiflexão/toe-off próprios nos alvos
        // articulares. O bloqueio plantar adicional é uma tarefa de postura
        // parada e estável; mantê-lo durante uma passada ou recuperação
        // transformava o tornozelo numa trava e fazia as pernas tropeçarem.
        const FootworkDebugState3D& currentFootwork =
            m_footwork.planner().state();
        const bool plantarPostureActive =
            ((!m_walking && !m_walkPending)
                || settlingBetweenWalkingSteps)
            && !currentFootwork.hasActiveSwing
            && m_config.standingStiffnessScale > 0.10f
            && balance.bodyUpDot > 0.90f;
        // Antes de o pe externo poder dar a passada de captura, o pe interno
        // precisa aceitar carga de verdade. Um contato perdido por poucos
        // milimetros nao pode deixar o planner esperando quase um segundo.
        // A pre-carga abaixo atua somente no pe que NAO esta em swing; nao
        // trava o tornozelo nem disputa a trajetoria da perna externa.
        const bool recoveryStancePreload =
            currentFootwork.hasActiveSwing && m_reactiveRecoveryStep;
        m_footwork.config().plantedFootOrientationScale =
            plantarPostureActive ? 0.58f
                : recoveryStancePreload ? 0.34f : 0.0f;
        m_footwork.config().plantedSolePreloadMeters =
            plantarPostureActive ? 0.0040f
                : recoveryStancePreload ? 0.0070f : 0.0f;
        m_footwork.config().plantedContactPreloadNewtons =
            plantarPostureActive ? 38.0f
                : recoveryStancePreload ? 72.0f : 0.0f;
        m_footwork.config().plantedFootDriveStiffnessScale =
            plantarPostureActive ? 1.85f
                : recoveryStancePreload ? 1.32f : 1.0f;
        m_footwork.config().plantedFootDriveDampingScale =
            plantarPostureActive ? 1.38f
                : recoveryStancePreload ? 1.18f : 1.0f;
        m_footwork.config().plantedFootDriveTorqueScale =
            plantarPostureActive ? 1.34f
                : recoveryStancePreload ? 1.12f : 1.0f;
        FootworkInput3D input;
        input.lookYawRadians = yawFromOrientation(m_spawnOrientation);
        input.fast = !m_walking
            && m_recoveryVelocityReference.length() > 0.82f;
        const float referenceSpeed = input.fast
            ? m_footwork.planner().config().fastSpeedMetersPerSecond
            : m_footwork.planner().config().walkSpeedMetersPerSecond;
        input.movementLocal.x =
            dot(desiredVelocity, m_right)
            / std::max(0.20f, referenceSpeed);
        input.movementLocal.y =
            dot(desiredVelocity, m_forward)
            / std::max(0.20f, referenceSpeed);
        m_footwork.setInput(input);
        // Parado, somente o controlador de equilibrio pode pedir passos. O
        // auto-step do prototipo perseguia homes ligadas a bacia e desfazia
        // imediatamente a base larga criada por uma passada de recuperacao.
        // Durante caminhada deliberada ele volta a ser o gerador nominal.
        // Recuperação não usa a marcha automática comum: cada pouso deve ser
        // recalculado a partir do ponto de captura atual. Reaproveitar passos
        // nominais de caminhada após o primeiro reflexo produzia alvos curtos
        // demais para a velocidade adquirida num impacto de 1000 N.
        m_footwork.planner().setAutomaticSteppingEnabled(m_walking);
        m_footwork.setLocomotionPath(
            m_walking, m_walkStart, m_forward, 0.035f);
        m_footwork.setPaused(balance.fallen);
        m_footwork.update(profile, state, deltaTime, *terrain);
        m_output.driveTargets = m_footwork.output().driveTargets;
    } else {
        buildFallbackTargets(profile);
    }
    buildWholeBodyTargets(profile, state);
    bool wholeBodySolved = false;
    WholeBodyMotionIntent3D wholeBodyIntent;
    wholeBodyIntent.locomotionActive = m_walking;
    wholeBodyIntent.balanceRecoveryUrgency = balance.recoveryUrgency;
    const bool fallProtectionActive =
        (balance.bodyUpDot < m_config.fallProtectionBodyUpDot
            || balance.torsoUpDot < m_config.fallProtectionBodyUpDot
            || (balance.captureMarginMeters
                    < m_config.fallProtectionCaptureMarginMeters
                && std::min(balance.bodyUpDot, balance.torsoUpDot)
                    < 0.78f))
        && horizontalCenterOfMassSpeed
            >= m_config.fallProtectionMinimumSpeedMetersPerSecond;
    wholeBodyIntent.fallProtectionActive = fallProtectionActive;
    Vec3 counterLean = balance.recoveryDirectionWorld;
    counterLean.z = 0.0f;
    if (counterLean.lengthSquared() > 1.0e-8f) {
        counterLean = counterLean.normalized()
            * -(0.035f + balance.recoveryUrgency * 0.105f);
    }
    wholeBodyIntent.desiredTorsoUpWorld =
        (Vec3 { 0.0f, 0.0f, 1.0f } + counterLean).normalized();
    wholeBodyIntent.desiredHeadUpWorld =
        (Vec3 { 0.0f, 0.0f, 1.0f }
            + counterLean * m_config.headCounterLeanScale).normalized();
    wholeBodyIntent.desiredHorizontalVelocity =
        settlingBetweenWalkingSteps
        ? Vec3 {}
        : desiredVelocity * (m_walking ? 0.40f : 1.0f);
    const float inverseMass = 1.0f / std::max(1.0f, profile.totalMassKg);
    wholeBodyIntent.desiredContactAccelerationWorld = {
        -reaction.stancePushForceWorld.x * inverseMass,
        -reaction.stancePushForceWorld.y * inverseMass,
        0.0f
    };
    bool wholeBodyLeftSupport = balance.leftFootSupported;
    bool wholeBodyRightSupport = balance.rightFootSupported;
    Vec3 wholeBodySupportCenter = balance.supportCenter;
    const auto externallyManipulatedLeg = [&](bool left) {
        if (!m_externalManipulationActive) return false;
        return m_externalManipulatedLinkIndex
                == (left ? m_links.leftThigh : m_links.rightThigh)
            || m_externalManipulatedLinkIndex
                == (left ? m_links.leftShin : m_links.rightShin)
            || m_externalManipulatedLinkIndex
                == (left ? m_links.leftFoot : m_links.rightFoot);
    };
    const bool externalLeftLeg = externallyManipulatedLeg(true);
    const bool externalRightLeg = externallyManipulatedLeg(false);
    wholeBodyIntent.externallyManipulatedLeg = {
        externalLeftLeg, externalRightLeg
    };
    if (externalLeftLeg && !externalRightLeg
        && balance.rightFootSupported) {
        wholeBodyLeftSupport = false;
        wholeBodySupportCenter = physicalSoleCenter(
            profile, state, m_links.rightFoot);
    } else if (externalRightLeg && !externalLeftLeg
        && balance.leftFootSupported) {
        wholeBodyRightSupport = false;
        wholeBodySupportCenter = physicalSoleCenter(
            profile, state, m_links.leftFoot);
    }
    if (m_footworkInitialized) {
        const FootworkDebugState3D& plan =
            m_footwork.planner().state();
        wholeBodyIntent.desiredFootSoles[0] = plan.feet[0].pose;
        wholeBodyIntent.desiredFootSoles[1] = plan.feet[1].pose;
        wholeBodyIntent.hasDesiredFootSole = { true, true };
        wholeBodyIntent.trackSwingFoot[0] =
            !balance.leftFootSupported;
        wholeBodyIntent.trackSwingFoot[1] =
            !balance.rightFootSupported;
        if (m_externalManipulationActive) {
            // Não criar uma disputa cartesiana entre o WBC e a Physgun. O
            // membro agarrado cede; coluna, braços e perna oposta continuam
            // controlados e o pé volta a ser adquirido após a soltura.
            if (externalLeftLeg) {
                wholeBodyIntent.trackSwingFoot[0] = false;
            }
            if (externalRightLeg) {
                wholeBodyIntent.trackSwingFoot[1] = false;
            }
        }
        if (plan.hasActiveSwing) {
            // Uma passada de captura e locomocao dinamica mesmo quando nao
            // existe um comando de caminhada. Isso ativa a hierarquia
            // centroidal/ZMP e reduz a prioridade cartesiana do pe em voo;
            // trata-la como "repouso" fazia a perna arrastar o corpo inteiro
            // na direcao da queda.
            wholeBodyIntent.locomotionActive = true;
            const std::size_t swing =
                footIndex3D(plan.activeSwingSide);
            const std::size_t support = 1u - swing;
            // Transferência de peso antes do toe-off: durante o começo da
            // trajetória os dois contatos continuam válidos e o objetivo
            // centroidal migra para o pé de apoio. Só então o pé em balanço
            // é liberado. Isso evita exigir equilíbrio monopodal antes de o
            // corpo chegar sobre a nova base.
            const float transferLinear =
                plan.feet[swing].weightTransferProgress;
            const float transfer = transferLinear * transferLinear
                * (3.0f - 2.0f * transferLinear);
            // O planejador guarda o pouso desejado, não o pouso que a física
            // conseguiu realizar. Depois do primeiro contato toda decisão de
            // equilíbrio deve usar a sola observada; perseguir a pose ideal
            // antiga equivale a equilibrar sobre um apoio virtual.
            const Vec3 physicalLeftSole =
                physicalSoleCenter(profile, state, m_links.leftFoot);
            const Vec3 physicalRightSole =
                physicalSoleCenter(profile, state, m_links.rightFoot);
            const Vec3 doubleSupportCenter =
                (physicalLeftSole + physicalRightSole) * 0.5f;
            const Vec3 stanceCenter = support == 0
                ? physicalLeftSole : physicalRightSole;
            // Em apoio simples, o alvo do COM só pode depender do pé que
            // realmente sustenta o corpo. Usar o ponto médio com o pé em voo
            // movia o alvo para fora da sola sempre que uma passada de
            // captura era mais larga, produzindo exatamente a aceleração
            // lateral que derrubava o ragdoll. Mantemos o alvo alguns
            // centímetros para o lado medial, ainda dentro da área real de
            // centro de pressão do pé plantado.
            const float medialSign =
                plan.activeSwingSide == FootSide3D::Left ? 1.0f : -1.0f;
            const Vec3 singleSupportCenter = stanceCenter
                + m_balance.lateral() * (medialSign * 0.028f)
                // Durante a transferência ainda há dois contatos. Mirar a
                // região posterior (mas dentro) do pé que ficará em apoio
                // produz heel-strike/toe-off plausível e evita pedir que a
                // bacia se teleporte ao centro da passada seguinte.
                - m_forward * (m_walking ? 0.118f : 0.0f);
            wholeBodySupportCenter =
                doubleSupportCenter
                + (singleSupportCenter - doubleSupportCenter) * transfer;
            // weightTransferProgress==1 também representa o estado "pronto,
            // mas aguardando o DCM entrar na sola". O contato só é removido do
            // modelo quando a trajetória física realmente começou.
            const bool released = transferLinear >= 0.999f
                && plan.feet[swing].phaseProgress > 1.0e-4f;
            if (released) {
                // O contato bruto do PhysX pode continuar verdadeiro por
                // alguns frames enquanto a ponta/calcanhar ainda raspa o
                // chao. Depois do toe-off planejado essa perna ja e voo:
                // mante-la simultaneamente como contato rigido e nao ativar
                // a tarefa cartesiana prendia a sola no ponto antigo. O
                // planner concluia a curva, mas o pe fisico mal se deslocava
                // lateralmente. O heel-strike validado abaixo readmite o
                // contato assim que a sola realmente chega ao novo alvo.
                if (swing == 0) {
                    wholeBodyLeftSupport = false;
                } else {
                    wholeBodyRightSupport = false;
                }
                if (!(swing == 0 ? externalLeftLeg : externalRightLeg)) {
                    wholeBodyIntent.trackSwingFoot[swing] = true;
                }
                // No apoio simples o WBC precisa do centro geométrico da sola
                // para limitar o ZMP ao retângulo real. O deslocamento medial/
                // posterior acima é somente a referência de transferência.
                wholeBodySupportCenter = stanceCenter;
            }
            m_telemetry.footworkWeightTransferProgress =
                transferLinear;
            m_telemetry.wholeBodySwingReleased = released;
            const bool physicalSwingSupported = swing == 0
                ? balance.leftFootSupported
                : balance.rightFootSupported;
            const bool physicalStanceSupported = support == 0
                ? balance.leftFootSupported
                : balance.rightFootSupported;
            const Vec3 physicalSwingSole = swing == 0
                ? physicalLeftSole : physicalRightSole;
            Vec3 heelStrikeError = physicalSwingSole
                - plan.feet[swing].target.position;
            heelStrikeError.z = 0.0f;
            const bool physicalHeelStrike = released
                && physicalSwingSupported
                && plan.feet[swing].phaseProgress >= 0.58f
                && heelStrikeError.length() <= 0.18f
                && physicalSwingSole.z
                    <= plan.feet[swing].target.position.z + 0.040f;
            if (physicalHeelStrike) {
                // O contato físico passa a integrar o modelo no mesmo tick
                // do heel-strike. Esperar o planner encerrar a curva deixava
                // um frame inteiro equilibrado apenas sobre o pé antigo,
                // atrás do COM, apesar de a nova planta já poder produzir
                // reação do solo. Isso invertia a aceleração exatamente no
                // instante mais importante de uma passada de captura.
                wholeBodyLeftSupport = balance.leftFootSupported;
                wholeBodyRightSupport = balance.rightFootSupported;
                if (wholeBodyLeftSupport && wholeBodyRightSupport) {
                    wholeBodySupportCenter = doubleSupportCenter;
                } else {
                    wholeBodySupportCenter = physicalSwingSole;
                }
            }
            if (released && !physicalSwingSupported
                && (plan.feet[swing].phaseProgress
                    >= 0.72f
                    || (m_reactiveRecoveryStep
                        && balance.recoveryUrgency >= 0.75f))) {
                // O planner pode terminar a curva antes de a sola física
                // tocar o terreno. Sem elevar este estado a uma aquisição
                // urgente, a tarefa secundária do pé perde para COM/coluna e
                // a perna permanece suspensa -- deixando o corpo inteiro em
                // apoio simples até cair. O alvo continua sendo a superfície
                // amostrada pelo footwork; apenas sua prioridade muda durante
                // o heel-strike atrasado.
                wholeBodyIntent.urgentPlantFoot[swing] = true;
            }
            if (!released && !physicalStanceSupported) {
                // A passada externa só pode sair depois que o pé interno
                // aceitar carga. Se esse contato se perdeu durante a rajada,
                // o WBC recebe uma tarefa explícita de replantio; sem ela o
                // QP priorizava o COM e deixava a perna interna flutuar.
                wholeBodyIntent.urgentPlantFoot[support] = true;
                wholeBodyIntent.trackSwingFoot[support] = true;
            }
            // A transferência possui uma fase propulsiva real. Nos 65%
            // iniciais, os dois apoios levam o COM para a perna de stance.
            // No trecho final, ainda com as duas solas no chão, os tornozelos
            // e quadris criam o momento frontal necessário à passada. Sem
            // isso o toe-off acontecia quase parado e o controlador tentava
            // puxar o corpo pelo pé em voo.
            // Caminhar não é uma sequência de poses paradas seguidas por um
            // tranco. O COM começa a avançar cedo enquanto o peso migra
            // lateralmente, e atinge a velocidade nominal perto do toe-off.
            // A rampa anterior só começava nos 35% finais, acumulando
            // inclinação do tronco e limitando cada apoio a ~20 cm.
            const float pushOffLinear = std::clamp(
                (transferLinear - 0.16f) / 0.84f, 0.0f, 1.0f);
            const float pushOff = pushOffLinear * pushOffLinear
                * (3.0f - 2.0f * pushOffLinear);
            // Se a transferência terminou mas a condição física de toe-off
            // ainda não foi satisfeita, os dois contatos continuam presos.
            // Exigir velocidade de marcha plena nesse estado satura quadril
            // e tornozelo contra o chão e impede o próprio tórax de se
            // recompor. Conservamos apenas um pequeno viés frontal enquanto
            // o DCM/postura entra na janela; ao liberar, a marcha retoma.
            const float forwardScale = released
                ? 0.40f
                : transferLinear >= 0.999f
                    ? 0.10f
                    : 0.40f * pushOff;
            wholeBodyIntent.desiredHorizontalVelocity =
                desiredVelocity * forwardScale;
            wholeBodyIntent.capturePointSupport = released;
            wholeBodyIntent.unloadingFoot[swing] = released;
            wholeBodyIntent.trackSwingFoot[swing] =
                !physicalHeelStrike
                && (released || !physicalSwingSupported);
            if (m_reactiveRecoveryTrailingStep) {
                // A perna interna apenas recompõe a largura da base. Fazer
                // esse deslocamento ganhar da tarefa centroidal empurrava o
                // tronco no mesmo sentido do impacto; a impedância local e
                // esta tarefa reduzida bastam para o shuffle curto.
                wholeBodyIntent.swingFootTaskWeightScale[swing] = 1.0f;
            }
            if (released && !physicalHeelStrike) {
                if (swing == 0) {
                    wholeBodyLeftSupport = false;
                } else {
                    wholeBodyRightSupport = false;
                }
            }
        }
    }
    if (m_config.useWholeBodyDynamics && dynamics != nullptr
        && dynamics->valid && !balance.fallen) {
        wholeBodySolved = m_wholeBody.update(profile, state, *dynamics,
            wholeBodyLeftSupport, wholeBodyRightSupport,
            m_links.leftFoot, m_links.rightFoot,
            wholeBodySupportCenter, wholeBodyIntent, deltaTime,
            m_output.driveTargets);
    }

    if (!m_reactiveRecoveryStep && !m_walking
        && balance.recoveryUrgency < 0.045f
        && horizontalCenterOfMassSpeed < 0.09f
        && balance.captureMarginMeters > 0.060f) {
        m_hasLastReactiveRecoverySide = false;
    }

    const FootworkDebugState3D* footworkState =
        m_footworkInitialized ? &m_footwork.planner().state() : nullptr;
    if (footworkState != nullptr
        && footworkState->completedStepCount
            > m_lastCompletedFootworkSteps) {
        m_lastCompletedFootworkSteps =
            footworkState->completedStepCount;
        if (!m_walking) {
            m_recoveryCooldownSeconds =
                balance.recoveryUrgency > 0.35f
                    || horizontalCenterOfMassSpeed > 0.22f
                    ? 0.0f : m_config.recoveryStepCooldownSeconds;
        }
        m_reactiveRecoveryStep = false;
        m_reactiveRecoveryTrailingStep = false;
    }
    if (m_walking) {
        m_lastWalkDistanceMeters = std::max(
            m_lastWalkDistanceMeters,
            dot(state.links[m_links.pelvis].position - m_walkStart,
                m_forward));
        if (m_lastWalkDistanceMeters
                >= m_walkRequestedMeters
                    - m_config.walkCompletionToleranceMeters
            && (footworkState == nullptr
                || !footworkState->hasActiveSwing)) {
            m_walking = false;
        }
    }
    if (balance.fallen) {
        m_walking = false;
        m_walkPending = false;
    }

    if (balance.fallen) {
        transitionTo(ActiveRagdollPhase3D::Falling);
    } else if (m_walking) {
        transitionTo(ActiveRagdollPhase3D::Walking);
    } else if (footworkState != nullptr
            && footworkState->hasActiveSwing) {
        transitionTo(ActiveRagdollPhase3D::RecoveryStep);
    } else if (balance.recoveryUrgency > 0.12f) {
        transitionTo(ActiveRagdollPhase3D::Balancing);
    } else if (m_ageSeconds < 0.30f) {
        transitionTo(ActiveRagdollPhase3D::Settling);
    } else {
        transitionTo(ActiveRagdollPhase3D::Standing);
    }

    m_telemetry.centerOfMass = balance.centerOfMass;
    m_telemetry.centerOfMassVelocity = balance.centerOfMassVelocity;
    m_telemetry.centerOfPressure = balance.centerOfPressure;
    m_telemetry.capturePoint = balance.capturePoint;
    m_telemetry.supportMinimum = balance.supportMinimum;
    m_telemetry.supportMaximum = balance.supportMaximum;
    m_telemetry.captureMarginMeters = balance.captureMarginMeters;
    m_telemetry.bodyUpDot = balance.bodyUpDot;
    m_telemetry.torsoUpDot = balance.torsoUpDot;
    const Vec3 worldUp { 0.0f, 0.0f, 1.0f };
    m_telemetry.neckUpDot = dot(
        state.links[m_links.neck].orientation.rotate(worldUp), worldUp);
    m_telemetry.headUpDot = dot(
        state.links[m_links.head].orientation.rotate(worldUp), worldUp);
    m_telemetry.centroidalAngularMomentum =
        balance.centroidalAngularMomentum;
    m_telemetry.hipStrategyWeight = balance.recoveryUrgency;
    m_telemetry.supportContactCount =
        balance.supportContactCount;
    m_telemetry.leftFootSupported = balance.leftFootSupported;
    m_telemetry.rightFootSupported = balance.rightFootSupported;
    m_telemetry.recoveryUrgency = balance.recoveryUrgency;
    m_telemetry.assistanceAuthority =
        balance.assistanceAuthority;
    const WholeBodyControllerTelemetry3D& wholeBodyTelemetry =
        m_wholeBody.telemetry();
    m_telemetry.wholeBodySolved = wholeBodySolved;
    m_telemetry.wholeBodySolverOptimal =
        wholeBodyTelemetry.solverOptimal;
    m_telemetry.wholeBodySolverIterations =
        wholeBodyTelemetry.solverIterations;
    m_telemetry.wholeBodyDynamicsResidual =
        wholeBodyTelemetry.dynamicsResidual;
    m_telemetry.wholeBodySolverPrimalResidual =
        wholeBodyTelemetry.solverPrimalResidual;
    m_telemetry.wholeBodySolverDualResidual =
        wholeBodyTelemetry.solverDualResidual;
    m_telemetry.wholeBodyMaximumTorqueNewtonMeters =
        wholeBodyTelemetry.maximumTorqueNewtonMeters;
    m_telemetry.leftGroundReactionNewtons =
        wholeBodyTelemetry.leftNormalForceNewtons;
    m_telemetry.rightGroundReactionNewtons =
        wholeBodyTelemetry.rightNormalForceNewtons;
    m_telemetry.desiredRootLinearAcceleration =
        wholeBodyTelemetry.desiredRootLinearAcceleration;
    m_telemetry.desiredCenterOfMass =
        wholeBodyTelemetry.desiredCenterOfMass;
    m_telemetry.dynamicsCenterOfMass =
        wholeBodyTelemetry.measuredCenterOfMass;
    m_telemetry.dynamicsCenterOfMassVelocity =
        wholeBodyTelemetry.measuredCenterOfMassVelocity;
    m_telemetry.wholeBodyDesiredHorizontalVelocity =
        wholeBodyTelemetry.desiredHorizontalVelocity;
    m_telemetry.desiredContactAcceleration =
        wholeBodyIntent.desiredContactAccelerationWorld;
    m_telemetry.desiredSwingFootAcceleration =
        wholeBodyTelemetry.desiredSwingFootAcceleration;
    m_telemetry.solvedSwingFootAcceleration =
        wholeBodyTelemetry.solvedSwingFootAcceleration;
    m_telemetry.leftContactForceNewtons =
        wholeBodyTelemetry.leftContactForceNewtons;
    m_telemetry.rightContactForceNewtons =
        wholeBodyTelemetry.rightContactForceNewtons;
    m_telemetry.solvedRootLinearAcceleration =
        wholeBodyTelemetry.solvedRootLinearAcceleration;
    m_telemetry.solvedCenterOfMassAcceleration =
        wholeBodyTelemetry.solvedCenterOfMassAcceleration;
    m_telemetry.fallen = balance.fallen;
    m_telemetry.recoveryStepActive =
        footworkState != nullptr && footworkState->hasActiveSwing;
    if (footworkState != nullptr) {
        m_telemetry.recoverySwingSide =
            footworkState->activeSwingSide;
        const std::size_t side =
            footIndex3D(footworkState->activeSwingSide);
        m_telemetry.recoveryStepProgress =
            footworkState->feet[side].phaseProgress;
        m_telemetry.recoveryStepTarget =
            footworkState->feet[side].target;
        m_telemetry.recoveryStepPose =
            footworkState->feet[side].pose;
        m_telemetry.completedWalkSteps =
            static_cast<std::uint32_t>(
                footworkState->completedStepCount
                    >= m_walkStartCompletedFootworkSteps
                ? footworkState->completedStepCount
                    - m_walkStartCompletedFootworkSteps
                : 0);
    }
    m_telemetry.walking = m_walking;
    m_telemetry.walkPending = m_walkPending;
    m_telemetry.walkRequestedMeters = m_walkRequestedMeters;
    m_telemetry.walkDistanceMeters =
        std::max(0.0f, m_lastWalkDistanceMeters);
    m_telemetry.walkRemainingMeters = m_walking
        ? std::max(0.0f,
            m_walkRequestedMeters - m_lastWalkDistanceMeters)
        : 0.0f;
    m_telemetry.fallProtectionActive = fallProtectionActive;
    m_telemetry.recoveryDirectionWorld =
        balance.recoveryDirectionWorld;

    m_output.gravityCompensationEnabled =
        m_config.enableFreeBaseGravityCompensation
        && !balance.fallen && !wholeBodySolved;
    // Quando o QP está ativo, qualquer auxílio opcional é deliberadamente
    // residual. Ele continua disponível como proteção artística, mas nunca
    // possui autoridade suficiente para sustentar uma postura ruim.
    const float residualAssistanceScale =
        wholeBodySolved ? 0.12f : 1.0f;
    m_output.rootForceNewtons =
        reaction.pelvisAssistForceNewtons * residualAssistanceScale;
    m_output.rootTorqueNewtonMeters =
        reaction.pelvisAssistTorqueNewtonMeters
            * residualAssistanceScale;
    m_output.applyRootForce =
        m_config.useMagicPelvisStabilization
        && reaction.assistanceAuthority > 0.01f
        && (m_output.rootForceNewtons.lengthSquared() > 0.01f
            || m_output.rootTorqueNewtonMeters.lengthSquared() > 0.01f);
    m_output.spineLinkIndex =
        static_cast<std::uint32_t>(m_links.upperChest);
    m_output.spineForceNewtons =
        reaction.spineAssistForceNewtons * residualAssistanceScale;
    m_output.spineTorqueNewtonMeters =
        reaction.spineAssistTorqueNewtonMeters
            * residualAssistanceScale;
    m_output.applySpineForce =
        m_config.useMagicPelvisStabilization
        && reaction.assistanceAuthority > 0.01f
        && (m_output.spineForceNewtons.lengthSquared() > 0.01f
            || m_output.spineTorqueNewtonMeters.lengthSquared() > 0.01f);
    m_telemetry.pelvisAssistForceNewtons =
        m_output.rootForceNewtons.length();
    m_telemetry.spineAssistForceNewtons =
        m_output.spineForceNewtons.length();
    if (m_footworkInitialized) {
        const PhysicalFootworkTelemetry3D& footTelemetry =
            m_footwork.telemetry();
        m_telemetry.leftSoleUpDot = footTelemetry.leftSoleUpDot;
        m_telemetry.rightSoleUpDot = footTelemetry.rightSoleUpDot;
        m_telemetry.stanceWidthMeters =
            footTelemetry.stanceWidthMeters;
    }
}

} // namespace MatterEngine
