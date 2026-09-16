#include "Engine/Control/NaturalBalanceSystem3D.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace MatterEngine {
namespace {

constexpr float GravityMetersPerSecondSquared = 9.81f;

float smoothStep(float value) {
    value = std::clamp(value, 0.0f, 1.0f);
    return value * value * (3.0f - 2.0f * value);
}

Vec3 clampLength(Vec3 value, float maximumLength) {
    const float length = value.length();
    if (length <= maximumLength || length <= 1.0e-6f) return value;
    return value * (maximumLength / length);
}

Vec3 horizontal(Vec3 value) {
    value.z = 0.0f;
    return value;
}

} // namespace

void NaturalBalanceSystem3D::reset(Quaternion worldOrientation) {
    m_forward = horizontal(
        worldOrientation.rotate({ 1.0f, 0.0f, 0.0f }));
    m_forward = m_forward.lengthSquared() > 1.0e-8f
        ? m_forward.normalized() : Vec3 { 1.0f, 0.0f, 0.0f };
    m_lateral = { -m_forward.y, m_forward.x, 0.0f };
    m_state = {};
    m_response = {};
    m_ageSeconds = 0.0f;
    m_supportConfidence = 0.0f;
    m_assistanceAuthority = 0.0f;
    m_fallenSeconds = 0.0f;
    m_leftFootContactGraceSeconds = 0.0f;
    m_rightFootContactGraceSeconds = 0.0f;
    m_desiredHorizontalVelocityWorld = {};
}

void NaturalBalanceSystem3D::setDesiredHorizontalVelocity(
    Vec3 velocityWorld) {
    velocityWorld.z = 0.0f;
    m_desiredHorizontalVelocityWorld =
        std::isfinite(velocityWorld.x)
            && std::isfinite(velocityWorld.y)
        ? velocityWorld : Vec3 {};
}

void NaturalBalanceSystem3D::update(
    const RagdollProfile3D& profile, const RagdollState3D& state,
    const NaturalBalanceLinkMap3D& links, float deltaTime) {
    m_response = {};
    if (deltaTime <= 0.0f || !std::isfinite(deltaTime)
        || state.links.size() != profile.links.size()
        || links.rightFoot >= state.links.size()
        || links.upperChest >= state.links.size()) {
        return;
    }
    m_ageSeconds += deltaTime;

    Vec3 centerOfMass;
    Vec3 centerOfMassVelocity;
    float totalFraction = 0.0f;
    for (std::size_t index = 0; index < state.links.size(); ++index) {
        const float fraction = profile.links[index].massFraction;
        if (fraction <= 0.0f) continue;
        const PhysicsBodyState3D& body = state.links[index];
        const Vec3 linkCenter = body.position
            + body.orientation.rotate(profile.links[index].centerOfMassLocal);
        centerOfMass += linkCenter * fraction;
        centerOfMassVelocity += body.linearVelocity * fraction;
        totalFraction += fraction;
    }
    if (totalFraction > 1.0e-6f) {
        centerOfMass *= 1.0f / totalFraction;
        centerOfMassVelocity *= 1.0f / totalFraction;
    }

    Vec3 angularMomentum;
    for (std::size_t index = 0; index < state.links.size(); ++index) {
        const float mass = profile.links[index].massFraction
            * profile.totalMassKg;
        if (mass <= 0.0f) continue;
        const PhysicsBodyState3D& body = state.links[index];
        const Vec3 linkCenter = body.position
            + body.orientation.rotate(profile.links[index].centerOfMassLocal);
        angularMomentum += cross(linkCenter - centerOfMass,
            body.linearVelocity * mass);
    }

    bool leftSupported = false;
    bool rightSupported = false;
    std::size_t contactCount = 0;
    float contactHeightSum = 0.0f;
    Vec3 weightedCenterOfPressure;
    float totalNormalImpulse = 0.0f;
    for (const RagdollContactPoint3D& contact : state.contacts) {
        if (contact.normal.z < 0.40f
            || contact.normalImpulseNewtonSeconds
                < m_config.contactImpulseThresholdNewtonSeconds) {
            continue;
        }
        if (contact.linkIndex != links.leftFoot
            && contact.linkIndex != links.rightFoot) {
            continue;
        }
        leftSupported |= contact.linkIndex == links.leftFoot;
        rightSupported |= contact.linkIndex == links.rightFoot;
        const float weight = std::max(
            contact.normalImpulseNewtonSeconds, 1.0e-5f);
        weightedCenterOfPressure += contact.position * weight;
        totalNormalImpulse += weight;
        contactHeightSum += contact.position.z;
        ++contactCount;
    }
    if (leftSupported) {
        m_leftFootContactGraceSeconds =
            m_config.footContactGraceSeconds;
    } else {
        m_leftFootContactGraceSeconds = std::max(
            0.0f, m_leftFootContactGraceSeconds - deltaTime);
        leftSupported = m_leftFootContactGraceSeconds > 0.0f;
    }
    if (rightSupported) {
        m_rightFootContactGraceSeconds =
            m_config.footContactGraceSeconds;
    } else {
        m_rightFootContactGraceSeconds = std::max(
            0.0f, m_rightFootContactGraceSeconds - deltaTime);
        rightSupported = m_rightFootContactGraceSeconds > 0.0f;
    }

    const auto soleCenter = [&](std::size_t footIndex) {
        const PhysicsBodyState3D& foot = state.links[footIndex];
        const RagdollCapsuleDefinition3D& collider =
            profile.links[footIndex].collider;
        return foot.position
            + foot.orientation.rotate(collider.localPosition)
            - foot.orientation.rotate(
                { 0.0f, 0.0f, collider.boxHalfExtents.z });
    };
    const Vec3 leftSole = soleCenter(links.leftFoot);
    const Vec3 rightSole = soleCenter(links.rightFoot);
    const float supportHeight = contactCount > 0
        ? contactHeightSum / static_cast<float>(contactCount)
        : (leftSole.z + rightSole.z) * 0.5f;

    float minimumForward = std::numeric_limits<float>::max();
    float maximumForward = -std::numeric_limits<float>::max();
    float minimumLateral = std::numeric_limits<float>::max();
    float maximumLateral = -std::numeric_limits<float>::max();
    Vec3 supportMinimum {
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(), supportHeight
    };
    Vec3 supportMaximum {
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max(), supportHeight
    };
    Vec3 supportCenter;
    std::size_t supportedFootCount = 0;
    const auto includeFoot = [&](std::size_t footIndex, bool supported) {
        if (!supported) return;
        const PhysicsBodyState3D& foot = state.links[footIndex];
        const RagdollCapsuleDefinition3D& collider =
            profile.links[footIndex].collider;
        const Vec3 center = soleCenter(footIndex);
        Vec3 footForward = horizontal(
            foot.orientation.rotate({ 1.0f, 0.0f, 0.0f }));
        footForward = footForward.lengthSquared() > 1.0e-8f
            ? footForward.normalized() : m_forward;
        const Vec3 footLateral {
            -footForward.y, footForward.x, 0.0f
        };
        supportCenter += center;
        ++supportedFootCount;
        for (float xSign : { -1.0f, 1.0f }) {
            for (float ySign : { -1.0f, 1.0f }) {
                Vec3 corner = center
                    + footForward
                        * (collider.boxHalfExtents.x * xSign)
                    + footLateral
                        * (collider.boxHalfExtents.y * ySign);
                corner.z = supportHeight;
                minimumForward = std::min(
                    minimumForward, dot(corner, m_forward));
                maximumForward = std::max(
                    maximumForward, dot(corner, m_forward));
                minimumLateral = std::min(
                    minimumLateral, dot(corner, m_lateral));
                maximumLateral = std::max(
                    maximumLateral, dot(corner, m_lateral));
                supportMinimum.x = std::min(supportMinimum.x, corner.x);
                supportMinimum.y = std::min(supportMinimum.y, corner.y);
                supportMaximum.x = std::max(supportMaximum.x, corner.x);
                supportMaximum.y = std::max(supportMaximum.y, corner.y);
            }
        }
    };
    includeFoot(links.leftFoot, leftSupported);
    includeFoot(links.rightFoot, rightSupported);
    if (supportedFootCount == 0) {
        // Referência geométrica somente para estimativa de queda. A
        // autoridade de controle continua zero sem contato confirmado.
        supportCenter = (leftSole + rightSole) * 0.5f;
        minimumForward = dot(supportCenter, m_forward) - 0.15f;
        maximumForward = dot(supportCenter, m_forward) + 0.15f;
        minimumLateral = dot(supportCenter, m_lateral) - 0.13f;
        maximumLateral = dot(supportCenter, m_lateral) + 0.13f;
        supportMinimum = supportCenter - Vec3 { 0.15f, 0.13f, 0.0f };
        supportMaximum = supportCenter + Vec3 { 0.15f, 0.13f, 0.0f };
    } else {
        supportCenter *= 1.0f / static_cast<float>(supportedFootCount);
    }
    supportCenter.z = supportHeight;

    const float comHeight = std::max(
        centerOfMass.z - supportHeight, 0.35f);
    const float omega = std::sqrt(
        GravityMetersPerSecondSquared / comHeight);
    // A captura é calculada em relação ao movimento solicitado. Parado, a
    // referência é zero e qualquer deslocamento exige reação. Caminhando,
    // a velocidade prevista não é tratada como uma queda a ser freada.
    const Vec3 velocityError = horizontal(
        centerOfMassVelocity - m_desiredHorizontalVelocityWorld);
    Vec3 capturePoint = centerOfMass + velocityError / omega;
    capturePoint.z = supportHeight;
    const float captureForward = dot(capturePoint, m_forward);
    const float captureLateral = dot(capturePoint, m_lateral);
    const float clampedForward = std::clamp(
        captureForward, minimumForward, maximumForward);
    const float clampedLateral = std::clamp(
        captureLateral, minimumLateral, maximumLateral);
    const bool captureInside =
        captureForward >= minimumForward
        && captureForward <= maximumForward
        && captureLateral >= minimumLateral
        && captureLateral <= maximumLateral;
    float captureMargin;
    if (captureInside) {
        captureMargin = std::min({
            captureForward - minimumForward,
            maximumForward - captureForward,
            captureLateral - minimumLateral,
            maximumLateral - captureLateral
        });
    } else {
        const float forwardOutside = captureForward - clampedForward;
        const float lateralOutside = captureLateral - clampedLateral;
        captureMargin = -std::sqrt(
            forwardOutside * forwardOutside
                + lateralOutside * lateralOutside);
    }
    Vec3 recoveryDirection =
        m_forward * (captureForward
            - dot(supportCenter, m_forward))
        + m_lateral * (captureLateral
            - dot(supportCenter, m_lateral));
    if (recoveryDirection.lengthSquared() < 1.0e-6f) {
        recoveryDirection = velocityError;
    }
    if (recoveryDirection.lengthSquared() > 1.0e-6f) {
        recoveryDirection = recoveryDirection.normalized();
    }
    const float recoveryUrgency = smoothStep(
        (m_config.safeCaptureMarginMeters - captureMargin)
        / std::max(0.001f,
            m_config.safeCaptureMarginMeters
                - m_config.emergencyCaptureMarginMeters));

    const PhysicsBodyState3D& pelvis = state.links[links.pelvis];
    const PhysicsBodyState3D& torso = state.links[links.upperChest];
    const Vec3 pelvisUp =
        pelvis.orientation.rotate({ 0.0f, 0.0f, 1.0f }).normalized();
    const Vec3 torsoUp =
        torso.orientation.rotate({ 0.0f, 0.0f, 1.0f }).normalized();
    const float bodyUpDot = dot(pelvisUp, { 0.0f, 0.0f, 1.0f });
    const float torsoUpDot = dot(torsoUp, { 0.0f, 0.0f, 1.0f });
    const float pelvisHeight = pelvis.position.z - supportHeight;

    const float supportTarget = supportedFootCount > 0 ? 1.0f : 0.0f;
    const float supportRate = supportTarget > m_supportConfidence
        ? 16.0f : 12.0f;
    m_supportConfidence += (supportTarget - m_supportConfidence)
        * std::clamp(deltaTime * supportRate, 0.0f, 1.0f);
    const float postureAuthority = smoothStep(
        (std::min(bodyUpDot, torsoUpDot) - 0.28f) / 0.60f);
    const float heightAuthority = smoothStep(
        (pelvisHeight - m_config.fallenPelvisHeightMeters)
        / std::max(0.10f,
            m_config.nominalPelvisHeightMeters
                - m_config.fallenPelvisHeightMeters));
    const float requestedAuthority =
        m_config.assistanceEnabled
        ? m_supportConfidence * postureAuthority * heightAuthority
        : 0.0f;
    const float authorityRate =
        requestedAuthority > m_assistanceAuthority ? 8.0f : 20.0f;
    m_assistanceAuthority +=
        (requestedAuthority - m_assistanceAuthority)
        * std::clamp(deltaTime * authorityRate, 0.0f, 1.0f);

    const bool fallenCandidate =
        pelvisHeight < m_config.fallenPelvisHeightMeters
        || std::min(bodyUpDot, torsoUpDot) < m_config.fallenUpDot;
    m_fallenSeconds = fallenCandidate
        ? m_fallenSeconds + deltaTime
        : std::max(0.0f, m_fallenSeconds - deltaTime * 2.5f);
    const bool fallen =
        m_fallenSeconds >= m_config.fallenConfirmationSeconds;
    if (fallen) {
        m_assistanceAuthority = 0.0f;
    }

    m_state.centerOfMass = centerOfMass;
    m_state.centerOfMassVelocity = centerOfMassVelocity;
    m_state.centerOfPressure = totalNormalImpulse > 1.0e-6f
        ? weightedCenterOfPressure / totalNormalImpulse
        : supportCenter;
    m_state.capturePoint = capturePoint;
    m_state.supportCenter = supportCenter;
    m_state.supportMinimum = supportMinimum;
    m_state.supportMaximum = supportMaximum;
    m_state.centroidalAngularMomentum = angularMomentum;
    m_state.recoveryDirectionWorld = recoveryDirection;
    m_state.captureMarginMeters = captureMargin;
    m_state.recoveryUrgency = recoveryUrgency;
    m_state.bodyUpDot = bodyUpDot;
    m_state.torsoUpDot = torsoUpDot;
    m_state.pelvisHeightMeters = pelvisHeight;
    m_state.supportConfidence = m_supportConfidence;
    m_state.assistanceAuthority = m_assistanceAuthority;
    m_state.supportContactCount = contactCount;
    m_state.leftFootSupported = leftSupported;
    m_state.rightFootSupported = rightSupported;
    m_state.fallen = fallen;

    if (fallen) {
        m_state.regime = NaturalBalanceRegime3D::Fallen;
        return;
    }
    if (m_ageSeconds < 0.30f) {
        m_state.regime = NaturalBalanceRegime3D::Settling;
    } else if (captureMargin < m_config.stepCaptureMarginMeters) {
        m_state.regime = NaturalBalanceRegime3D::RecoveryStep;
    } else if (recoveryUrgency > 0.12f) {
        m_state.regime = NaturalBalanceRegime3D::CounterRotating;
    } else {
        m_state.regime = NaturalBalanceRegime3D::Stable;
    }

    if (captureMargin < m_config.stepCaptureMarginMeters
        && recoveryDirection.lengthSquared() > 1.0e-6f) {
        const float stepWeight = smoothStep(
            (m_config.stepCaptureMarginMeters - captureMargin)
            / std::max(0.001f,
                m_config.stepCaptureMarginMeters
                    - m_config.emergencyCaptureMarginMeters));
        const float recoverySpeed =
            m_config.minimumRecoverySpeedMetersPerSecond
            + (m_config.maximumRecoverySpeedMetersPerSecond
                - m_config.minimumRecoverySpeedMetersPerSecond)
                * stepWeight;
        m_response.recoveryVelocityWorld =
            recoveryDirection * recoverySpeed;
    }

    const Vec3 captureError = horizontal(
        supportCenter - capturePoint);
    m_response.internalSupportTorqueWorld = clampLength(
        cross({ 0.0f, 0.0f, 1.0f }, captureError)
            * m_config.supportTorqueNewtonMetersPerMeter,
        m_config.maximumSupportTorqueNewtonMeters);
    // Força de tarefa aplicada ao pé apoiado. O sinal é intencional:
    // ao cair para +X o pé empurra o chão para +X, então a reação do solo
    // empurra o restante do corpo para -X.
    const Vec3 captureOutside = horizontal(
        capturePoint
        - (m_forward * clampedForward + m_lateral * clampedLateral));
    const Vec3 captureFromSupportCenter =
        horizontal(capturePoint - supportCenter);
    const float centeringScale =
        m_desiredHorizontalVelocityWorld.lengthSquared() > 0.01f
        ? m_config.movingStanceCenteringScale : 1.0f;
    m_response.stancePushForceWorld = clampLength(
        (captureOutside * m_config.stancePushNewtonsPerMeter
            + velocityError
                * m_config.stancePushDampingNewtonSecondsPerMeter
            + captureFromSupportCenter
                * (m_config.stanceCenteringNewtonsPerMeter
                    * centeringScale))
            * m_supportConfidence,
        m_config.maximumStancePushNewtons);
    // Sustentação física restante: a assistência externa retira apenas uma
    // fração do peso. O restante vira força que a perna apoiada exerce para
    // baixo no solo; a reação normal sustenta o corpo. Quando há um só pé
    // de apoio, ele recebe toda essa tarefa, realizando a transferência de
    // carga necessária antes/durante uma passada.
    const float verticalAssistScale = supportedFootCount == 1
        ? m_config.singleSupportVerticalAssistScale
        : m_config.bilateralSupportVerticalAssistScale;
    const float effectivePelvisWeightAssistFraction =
        m_config.pelvisWeightAssistFraction * verticalAssistScale;
    const float effectiveSpineWeightAssistFraction =
        m_config.spineWeightAssistFraction * verticalAssistScale;
    const float assistedWeightFraction = std::clamp(
        (effectivePelvisWeightAssistFraction
            + effectiveSpineWeightAssistFraction)
            * m_assistanceAuthority,
        0.0f, 0.65f);
    m_response.stancePushForceWorld.z =
        -profile.totalMassKg * GravityMetersPerSecondSquared
        * (1.0f - assistedWeightFraction)
        * m_supportConfidence;
    Vec3 torsoTorque =
        cross(torsoUp, { 0.0f, 0.0f, 1.0f })
            * m_config.torsoInternalUprightStiffnessNewtonMeters
        - horizontal(torso.angularVelocity)
            * m_config.torsoInternalUprightDampingNewtonMeterSeconds;
    Vec3 momentumTorque {
        -angularMomentum.x, -angularMomentum.y, 0.0f
    };
    momentumTorque *=
        m_config.angularMomentumDampingPerSecond
        * (0.20f + 0.80f * recoveryUrgency);
    m_response.internalTorsoTorqueWorld = clampLength(
        torsoTorque + momentumTorque,
        m_config.maximumInternalTorsoTorqueNewtonMeters);
    // Estratégia de quadril puramente interna: o torque é aplicado nas
    // articulações das coxas e a reação endireita a pelve através das pernas
    // apoiadas. Continua funcionando com toda assistência externa desligada.
    m_response.internalPelvisTorqueWorld = clampLength(
        cross(pelvisUp, { 0.0f, 0.0f, 1.0f })
                * m_config.pelvisInternalUprightStiffnessNewtonMeters
            - horizontal(pelvis.angularVelocity)
                * m_config.pelvisInternalUprightDampingNewtonMeterSeconds
            + momentumTorque * 0.34f,
        m_config.maximumInternalPelvisTorqueNewtonMeters);
    m_response.internalArmTorqueWorld = clampLength(
        momentumTorque + m_response.internalSupportTorqueWorld * 0.22f,
        m_config.maximumInternalArmTorqueNewtonMeters);

    const float authority = fallen ? 0.0f : m_assistanceAuthority;
    const float targetPelvisZ =
        supportHeight + m_config.nominalPelvisHeightMeters;
    const float heightError = std::clamp(
        targetPelvisZ - pelvis.position.z, -0.08f, 0.08f);
    const float heightGainScale = supportedFootCount == 1
        ? m_config.singleSupportHeightGainScale : 1.0f;
    float pelvisForceZ =
        profile.totalMassKg * GravityMetersPerSecondSquared
            * effectivePelvisWeightAssistFraction
        + heightError
            * (m_config.pelvisHeightStiffnessNewtonsPerMeter
                * heightGainScale)
        - pelvis.linearVelocity.z
            * (m_config.pelvisHeightDampingNewtonSecondsPerMeter
                * heightGainScale);
    const float maximumVerticalAssistScale = supportedFootCount == 1
        ? m_config.singleSupportMaximumAssistScale : 1.0f;
    pelvisForceZ = std::clamp(pelvisForceZ, 0.0f,
        m_config.pelvisMaximumAssistForceNewtons
            * maximumVerticalAssistScale);
    const Vec3 horizontalAssist = clampLength(
        -(captureOutside
                * m_config.pelvisHorizontalAssistNewtonsPerMeter
            + velocityError
                * m_config
                    .pelvisHorizontalAssistDampingNewtonSecondsPerMeter),
        m_config.pelvisMaximumHorizontalAssistNewtons);
    m_response.pelvisAssistForceNewtons =
        (horizontalAssist + Vec3 { 0.0f, 0.0f, pelvisForceZ })
        * authority;
    m_response.pelvisAssistTorqueNewtonMeters = clampLength(
        (cross(pelvisUp, { 0.0f, 0.0f, 1.0f })
                * m_config.pelvisUprightStiffnessNewtonMeters
            - horizontal(pelvis.angularVelocity)
                * m_config.pelvisUprightDampingNewtonMeterSeconds)
            * authority,
        m_config.pelvisMaximumAssistTorqueNewtonMeters);

    const float spineForceZ = std::min(
        profile.totalMassKg * GravityMetersPerSecondSquared
            * effectiveSpineWeightAssistFraction,
        m_config.spineMaximumAssistForceNewtons
            * maximumVerticalAssistScale);
    m_response.spineAssistForceNewtons =
        (horizontalAssist * m_config.spineHorizontalAssistFraction
            + Vec3 { 0.0f, 0.0f, spineForceZ }) * authority;
    m_response.spineAssistTorqueNewtonMeters = clampLength(
        (cross(torsoUp, { 0.0f, 0.0f, 1.0f })
                * m_config.spineUprightStiffnessNewtonMeters
            - horizontal(torso.angularVelocity)
                * m_config.spineUprightDampingNewtonMeterSeconds)
            * authority,
        m_config.spineMaximumAssistTorqueNewtonMeters);
    m_response.assistanceAuthority = authority;
}

} // namespace MatterEngine
