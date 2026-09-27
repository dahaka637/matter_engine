// Matter Engine clean-room physical biped controller.
// THEORY: [R02] GBWC inverted-pendulum foot placement; [R05/R06] capture recovery.
// Core invariant: swing liftoff requires measured load transfer; a timer can never bypass contact physics.

#include "Engine/Locomotion/BipedStepPlanner3D.hpp"

#include "Engine/Control/BipedMath3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

float orbitalStopDistance(float velocity, float height) {
    constexpr float Gravity = 9.81f;
    const float h = std::max(height, 0.05f);
    const float inside =
        h/Gravity+(velocity*velocity)/(4.0f*Gravity*Gravity);
    return velocity*std::sqrt(std::max(inside, 0.0f));
}

float sideSign(int foot) {
    return foot == 0 ? 1.0f : -1.0f;
}

Vec3 horizontal(Vec3 value) {
    value.z = 0.0f;
    return value;
}

}

void BipedStepPlanner3D::reset() {
    m_phase = BipedStepPhase3D::Idle;
    m_stanceFoot = -1;
    m_swingFoot = -1;
    m_nextSwingFoot = 0;
    m_recovery = false;
    m_phaseSeconds = 0.0f;
    m_swingDurationSeconds = 0.0f;
    m_swingStartSoleWorld = {};
    m_landingSoleWorld = {};
    m_landingOrientationWorld = {};
}

int BipedStepPlanner3D::chooseSwingFoot(
    const BipedObservation3D& observation,
    const BipedIntent3D& intent,
    const BipedBalanceDecision3D& balance) const {
    const bool left = observation.feet[0].contact;
    const bool right = observation.feet[1].contact;
    if (left && !right) return 1;
    if (right && !left) return 0;

    if (balance.mustStep) {
        const Quaternion heading =
            yawQuaternionBiped(intent.desiredHeadingRadians);
        const Vec3 captureBody = heading.conjugate().rotate(
            observation.capturePointWorld-observation.centerOfMassWorld);
        if (std::abs(captureBody.y) > 0.04f)
            return captureBody.y >= 0.0f ? 0 : 1;
    }

    if (observation.feet[0].loadRatio
            < observation.feet[1].loadRatio-0.12f)
        return 0;
    if (observation.feet[1].loadRatio
            < observation.feet[0].loadRatio-0.12f)
        return 1;
    return m_nextSwingFoot;
}

bool BipedStepPlanner3D::computeLanding(
    const RagdollProfile3D& profile,
    const BipedRig3D& rig,
    const BipedObservation3D& observation,
    const BipedIntent3D& intent,
    const BipedBalanceDecision3D& balance,
    const BipedTerrainQuery3D& terrain,
    int swingFoot,
    Vec3& landing,
    Quaternion& orientation) const {
    if (swingFoot < 0 || swingFoot > 1) return false;
    const Quaternion body =
        yawQuaternionBiped(intent.desiredHeadingRadians);
    const Vec3 velocityBody =
        body.conjugate().rotate(observation.centerOfMassVelocityWorld);
    Vec3 stepBody {
        orbitalStopDistance(
            velocityBody.x, observation.centerOfMassHeightMeters)
            -m_settings.desiredVelocityAlphaSeconds
                *intent.desiredVelocityBody.x,
        orbitalStopDistance(
            velocityBody.y, observation.centerOfMassHeightMeters)
            -m_settings.desiredVelocityAlphaSeconds
                *intent.desiredVelocityBody.y,
        0.0f
    };

    if (balance.mustStep) {
        Vec3 captureBody = body.conjugate().rotate(
            observation.capturePointWorld-observation.centerOfMassWorld);
        captureBody.z = 0.0f;
        const float blend = std::clamp(
            0.55f+0.45f*balance.urgency, 0.0f, 1.0f);
        stepBody = stepBody*(1.0f-blend)+captureBody*blend;
    }

    const float halfWidth =
        std::max(0.06f, rig.nominalStepWidthMeters*0.5f);
    const float calmLateral = std::clamp(
        1.0f-std::abs(velocityBody.y)*1.5f
            -balance.urgency*0.65f, 0.0f, 1.0f);
    stepBody.y = stepBody.y*(1.0f-calmLateral)
        +sideSign(swingFoot)*halfWidth*calmLateral;

    const float reach = std::max(0.20f,
        rig.legLengthMeters*m_settings.maximumStepReachLegFraction);
    const float horizontalLength =
        std::sqrt(stepBody.x*stepBody.x+stepBody.y*stepBody.y);
    if (horizontalLength > reach && horizontalLength > 1.0e-6f) {
        const float scale = reach/horizontalLength;
        stepBody.x *= scale;
        stepBody.y *= scale;
    }

    Vec3 desired =
        observation.centerOfMassWorld+body.rotate(stepBody);
    BipedTerrainHit3D hit;
    if (!terrain.sampleGround
        || !terrain.sampleGround(desired,
            m_settings.terrainProbeUpMeters,
            m_settings.terrainProbeDownMeters, hit)
        || !hit.valid || hit.normal.z < 0.35f) {
        return false;
    }

    landing = hit.position;
    const auto& footDef = profile.links[rig.foot[
        static_cast<std::size_t>(swingFoot)]];
    orientation = bipedNeutralLinkOrientationWorld3D(
        footDef, intent.desiredHeadingRadians, hit.normal);
    return finiteBiped(landing) && finiteBiped(orientation);
}

bool BipedStepPlanner3D::canLift(
    const BipedObservation3D& observation) const {
    if (m_stanceFoot < 0 || m_swingFoot < 0) return false;
    const auto& stance =
        observation.feet[static_cast<std::size_t>(m_stanceFoot)];
    const auto& swing =
        observation.feet[static_cast<std::size_t>(m_swingFoot)];
    return stance.contact && !stance.slipping
        && stance.loadRatio >= m_settings.minimumStanceLoadRatioForLift
        && swing.loadRatio <= m_settings.maximumSwingLoadRatioForLift;
}

void BipedStepPlanner3D::startTransfer(
    int swingFoot, bool recovery) {
    m_swingFoot = swingFoot;
    m_stanceFoot = 1-swingFoot;
    m_recovery = recovery;
    m_phase = BipedStepPhase3D::Transfer;
    m_phaseSeconds = 0.0f;
}

void BipedStepPlanner3D::startSwing(
    const RagdollProfile3D& profile,
    const BipedRig3D& rig,
    const BipedObservation3D& observation,
    const BipedIntent3D& intent,
    const BipedBalanceDecision3D& balance,
    const BipedTerrainQuery3D& terrain) {
    Vec3 landing;
    Quaternion orientation;
    if (!computeLanding(profile, rig, observation,
            intent, balance, terrain, m_swingFoot,
            landing, orientation)) {
        return;
    }
    m_landingSoleWorld = landing;
    m_landingOrientationWorld = orientation;
    m_swingStartSoleWorld =
        observation.feet[static_cast<std::size_t>(m_swingFoot)]
            .solePositionWorld;

    const float base = m_recovery
        ? m_settings.recoverySwingSeconds
        : intent.sprint
            ? m_settings.runSwingSeconds
            : m_settings.walkSwingSeconds;
    const float distance =
        horizontal(m_landingSoleWorld-m_swingStartSoleWorld).length();
    const float distanceScale = std::clamp(
        distance/std::max(0.20f, rig.legLengthMeters*0.45f),
        0.65f, 1.25f);
    m_swingDurationSeconds = std::clamp(
        base*distanceScale,
        m_settings.minimumSwingSeconds,
        m_settings.maximumSwingSeconds);
    m_phase = BipedStepPhase3D::Swing;
    m_phaseSeconds = 0.0f;
}

BipedStepPlan3D BipedStepPlanner3D::update(
    const RagdollProfile3D& profile,
    const BipedRig3D& rig,
    const BipedObservation3D& observation,
    const BipedIntent3D& intent,
    const BipedBalanceDecision3D& balance,
    const BipedTerrainQuery3D& terrain,
    float deltaTime) {
    BipedStepPlan3D output;
    if (!rig.valid || !observation.valid
        || !std::isfinite(deltaTime) || deltaTime <= 0.0f) {
        return output;
    }

    const float desiredSpeed =
        horizontal(intent.desiredVelocityBody).length();
    const bool wantsLocomotion =
        intent.locomotionEnabled && desiredSpeed > 0.05f;
    const bool wantsStep =
        intent.allowStepping && (wantsLocomotion || balance.mustStep);

    m_phaseSeconds += deltaTime;

    if (m_phase == BipedStepPhase3D::Idle && wantsStep) {
        startTransfer(
            chooseSwingFoot(observation, intent, balance),
            balance.mustStep);
    }

    if (m_phase == BipedStepPhase3D::Transfer) {
        if (m_stanceFoot < 0 || m_swingFoot < 0) {
            reset();
        } else if (canLift(observation)) {
            startSwing(profile, rig, observation,
                intent, balance, terrain);
        } else if (m_phaseSeconds > m_settings.transferTimeoutSeconds) {
            // Never violate load transfer. Re-evaluate which foot should move
            // instead of authorizing liftoff from a 50/50 support state.
            const int candidate =
                chooseSwingFoot(observation, intent, balance);
            if (candidate != m_swingFoot) startTransfer(
                candidate, balance.mustStep);
            else m_phaseSeconds =
                m_settings.transferTimeoutSeconds*0.5f;
        }
    }

    if (m_phase == BipedStepPhase3D::Swing) {
        Vec3 updatedLanding;
        Quaternion updatedOrientation;
        if (computeLanding(profile, rig, observation,
                intent, balance, terrain, m_swingFoot,
                updatedLanding, updatedOrientation)) {
            Vec3 delta = updatedLanding-m_landingSoleWorld;
            delta.z = 0.0f;
            const float maxMove =
                m_settings.landingReplanSpeedMetersPerSecond*deltaTime;
            delta = clampLengthBiped(delta, maxMove);
            m_landingSoleWorld += delta;
            m_landingSoleWorld.z = updatedLanding.z;
            m_landingOrientationWorld = updatedOrientation;
        }

        const float phase = std::clamp(
            m_phaseSeconds/std::max(m_swingDurationSeconds, 0.001f),
            0.0f, 1.0f);
        const float u = smoothstepBiped(phase);
        const float clearance = std::max(
            0.05f,
            rig.legLengthMeters*m_settings.nominalClearanceLegFraction);
        Vec3 target =
            m_swingStartSoleWorld*(1.0f-u)+m_landingSoleWorld*u;
        target.z += clearance*4.0f*phase*(1.0f-phase);

        const auto& foot =
            observation.feet[static_cast<std::size_t>(m_swingFoot)];
        const float weight = profile.totalMassKg*9.81f;
        const bool touchdown =
            phase > 0.30f && foot.contact
            && foot.normalLoadNewtons
                >= weight*m_settings.minimumTouchdownLoadWeightFraction;
        if (touchdown) {
            m_phase = BipedStepPhase3D::Settle;
            m_phaseSeconds = 0.0f;
            m_nextSwingFoot = m_stanceFoot;
        }

        output.swingSoleTargetWorld = target;
        output.swingFootOrientationWorld =
            m_landingOrientationWorld;
    }

    if (m_phase == BipedStepPhase3D::Settle
        && m_phaseSeconds >= m_settings.settleSeconds) {
        if (wantsStep) {
            startTransfer(
                chooseSwingFoot(observation, intent, balance),
                balance.mustStep);
        } else {
            reset();
        }
    }

    output.phase = m_phase;
    output.stanceFoot = m_stanceFoot;
    output.swingFoot = m_swingFoot;
    output.recoveryStep = m_recovery;
    output.validLanding =
        m_phase == BipedStepPhase3D::Swing
        || m_phase == BipedStepPhase3D::Settle;
    output.landingSoleWorld = m_landingSoleWorld;
    output.landingFootOrientationWorld =
        m_landingOrientationWorld;
    output.durationSeconds = m_swingDurationSeconds;
    if (m_phase == BipedStepPhase3D::Swing)
        output.phaseProgress = std::clamp(
            m_phaseSeconds/std::max(m_swingDurationSeconds, 0.001f),
            0.0f, 1.0f);

    if (m_stanceFoot >= 0 && m_stanceFoot < 2) {
        output.desiredComWorld =
            observation.feet[static_cast<std::size_t>(m_stanceFoot)]
                .contactCenterWorld;
        output.desiredComWorld.z =
            observation.centerOfMassWorld.z;
    } else {
        output.desiredComWorld =
            observation.support.centerWorld;
        output.desiredComWorld.z =
            observation.centerOfMassWorld.z;
    }
    return output;
}

} // namespace MatterEngine
