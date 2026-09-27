// Matter Engine clean-room procedural get-up.
// THEORY: [R19-R24] contact-rich/full-body get-up and contact-state transitions.
// No stand-up animation, clip phase, root alignment, or world-space pose spring is used.

#include "Engine/Control/ProceduralGetUp3D.hpp"

#include "Engine/Control/BipedMath3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

bool isIndex(std::size_t value, std::size_t candidate) {
    return candidate != InvalidBipedLink3D && value == candidate;
}

Vec3 endEffectorOriginForSupportPoint(
    const RagdollLinkDefinition3D& link,
    Quaternion orientationWorld,
    Vec3 supportPointWorld,
    Vec3 supportNormalWorld) {
    const Vec3 offset = bipedColliderSupportOffsetLocal3D(
        link, orientationWorld, supportNormalWorld);
    return supportPointWorld-orientationWorld.rotate(offset);
}

}

void ProceduralGetUp3D::reset() {
    m_stage = BipedGetUpStage3D::None;
    m_initialPose = BipedFallPose3D::Unknown;
    m_stageSeconds = 0.0f;
    m_stableSeconds = 0.0f;
    m_replans = 0;
}

void ProceduralGetUp3D::transition(BipedGetUpStage3D stage) {
    m_stage = stage;
    m_stageSeconds = 0.0f;
    m_stableSeconds = 0.0f;
}

BipedFallPose3D ProceduralGetUp3D::classifyPose(
    const RagdollProfile3D& profile,
    const BipedRig3D& rig,
    const RagdollState3D& state) const {
    if (rig.torso >= profile.links.size()
        || rig.torso >= state.links.size()) {
        return BipedFallPose3D::Unknown;
    }
    const auto& definition = profile.links[rig.torso];
    const auto& body = state.links[rig.torso];
    const Vec3 anterior = bipedAnatomicalAxisWorld3D(
        definition, body, {1.0f, 0.0f, 0.0f});
    const float anteriorUp =
        dot(anterior, Vec3{0.0f, 0.0f, 1.0f});
    if (anteriorUp > 0.40f) return BipedFallPose3D::Supine;
    if (anteriorUp < -0.40f) return BipedFallPose3D::Prone;
    const Vec3 left = bipedAnatomicalAxisWorld3D(
        definition, body, {0.0f, 1.0f, 0.0f});
    return left.z >= 0.0f
        ? BipedFallPose3D::RightSide
        : BipedFallPose3D::LeftSide;
}

Vec3 ProceduralGetUp3D::projectToGround(
    Vec3 pointWorld,
    const BipedTerrainQuery3D& terrain,
    float fallbackHeight,
    Vec3* normal) const {
    BipedTerrainHit3D hit;
    if (terrain.sampleGround
        && terrain.sampleGround(pointWorld,
            m_settings.groundProbeUpMeters,
            m_settings.groundProbeDownMeters, hit)
        && hit.valid && hit.normal.z > 0.25f) {
        if (normal) *normal = hit.normal.normalized();
        return hit.position;
    }
    if (normal) *normal = {0.0f, 0.0f, 1.0f};
    pointWorld.z = fallbackHeight;
    return pointWorld;
}

std::size_t ProceduralGetUp3D::usefulSupportCount(
    const BipedRig3D& rig,
    const BipedObservation3D& observation) const {
    std::size_t count = 0;
    for (const auto& contact : observation.support.contacts) {
        const std::size_t link = contact.linkIndex;
        bool useful = false;
        for (std::size_t side = 0; side < 2; ++side) {
            useful = useful
                || isIndex(link, rig.foot[side])
                || isIndex(link, rig.hand[side])
                || isIndex(link, rig.lowerLeg[side])
                || isIndex(link, rig.lowerArm[side]);
        }
        if (useful) ++count;
    }
    return count;
}

BipedGetUpCommand3D ProceduralGetUp3D::update(
    const RagdollProfile3D& profile,
    const BipedRig3D& rig,
    const RagdollState3D& state,
    const BipedObservation3D& observation,
    const BipedTerrainQuery3D& terrain,
    float headingRadians,
    float deltaTime) {
    BipedGetUpCommand3D output;
    if (!rig.valid || !observation.valid
        || !std::isfinite(deltaTime) || deltaTime <= 0.0f) {
        return output;
    }

    if (m_stage == BipedGetUpStage3D::None
        || m_stage == BipedGetUpStage3D::Complete) {
        m_initialPose = classifyPose(profile, rig, state);
        transition(BipedGetUpStage3D::EstablishSupport);
    }
    m_stageSeconds += deltaTime;

    const float ground = observation.support.centerWorld.z;
    const float standingHeight =
        std::max(profile.standingRootHeightMeters, 0.50f);
    const Quaternion heading = yawQuaternionBiped(headingRadians);
    output.stage = m_stage;
    output.initialPose = m_initialPose;
    output.desiredTorsoOrientationWorld =
        bipedNeutralLinkOrientationWorld3D(
            profile.links[rig.torso], headingRadians,
            {0.0f, 0.0f, 1.0f});

    auto addGroundTask = [&](std::size_t linkIndex, Vec3 desiredXY,
                              bool orient, float weight) {
        if (linkIndex == InvalidBipedLink3D
            || linkIndex >= profile.links.size()
            || linkIndex >= state.links.size()) return;
        Vec3 normal;
        const Vec3 contactPoint = projectToGround(
            desiredXY, terrain, ground, &normal);
        Quaternion orientation = orient
            ? bipedNeutralLinkOrientationWorld3D(
                profile.links[linkIndex], headingRadians, normal)
            : state.links[linkIndex].orientation;
        const Vec3 origin = endEffectorOriginForSupportPoint(
            profile.links[linkIndex], orientation,
            contactPoint, normal);
        BipedEndEffectorTask3D task;
        task.linkIndex = linkIndex;
        task.positionWorld = origin;
        task.orientationWorld = orientation;
        task.constrainOrientation = orient;
        task.positionWeight = weight;
        task.orientationWeight = orient ? 0.25f*weight : 0.0f;
        output.endEffectors.push_back(task);
    };

    const Vec3 forward = heading.rotate({1.0f, 0.0f, 0.0f});
    const Vec3 left = heading.rotate({0.0f, 1.0f, 0.0f});
    const float halfStepWidth =
        std::max(0.06f, rig.nominalStepWidthMeters*0.5f);

    auto addCurrentHands = [&](float weight) {
        for (std::size_t side = 0; side < 2; ++side) {
            const std::size_t hand = rig.hand[side];
            if (hand == InvalidBipedLink3D
                || hand >= state.links.size()) continue;
            Vec3 p = state.links[hand].position;
            p += forward*(m_initialPose == BipedFallPose3D::Prone
                ? 0.08f : -0.02f);
            addGroundTask(hand, p, false, weight);
        }
    };

    auto addCurrentFeet = [&](float weight) {
        for (std::size_t side = 0; side < 2; ++side) {
            const std::size_t foot = rig.foot[side];
            Vec3 p = observation.feet[side].solePositionWorld;
            addGroundTask(foot, p, true, weight);
        }
    };

    auto addFeetUnderCom = [&](float weight) {
        for (std::size_t side = 0; side < 2; ++side) {
            Vec3 p = observation.centerOfMassWorld
                +left*(side == 0 ? halfStepWidth : -halfStepWidth);
            // A small forward offset gives the crouched COM room to extend
            // without placing both ankles directly under the pelvis axis.
            p += forward*0.04f;
            addGroundTask(rig.foot[side], p, true, weight);
        }
    };

    bool stageCondition = false;
    const bool bothFeet =
        observation.feet[0].contact && observation.feet[1].contact;
    const std::size_t usefulSupports =
        usefulSupportCount(rig, observation);

    switch (m_stage) {
    case BipedGetUpStage3D::EstablishSupport:
        addCurrentHands(1.0f);
        addCurrentFeet(0.75f);
        output.desiredComHeightWorld =
            ground+standingHeight
                *m_settings.establishSupportHeightFraction;
        output.desiredTorsoWeight = 0.30f;
        output.desiredComVerticalVelocity = 0.15f;
        stageCondition = usefulSupports >= 2
            && m_stageSeconds > 0.22f;
        break;

    case BipedGetUpStage3D::RaiseTorso:
        addCurrentHands(1.0f);
        addCurrentFeet(0.85f);
        output.desiredComHeightWorld =
            ground+standingHeight*m_settings.raiseTorsoHeightFraction;
        output.desiredTorsoWeight = 0.70f;
        output.desiredComVerticalVelocity = 0.35f;
        output.residualEligible = usefulSupports >= 2;
        stageCondition =
            observation.centerOfMassHeightMeters
                > standingHeight*0.40f
            && observation.torsoUpDot > 0.10f;
        break;

    case BipedGetUpStage3D::BringFeetUnderCom:
        addCurrentHands(0.85f);
        addFeetUnderCom(1.0f);
        output.desiredComHeightWorld =
            ground+standingHeight*m_settings.bringFeetHeightFraction;
        output.desiredTorsoWeight = 0.88f;
        output.desiredComVerticalVelocity = 0.30f;
        output.residualEligible = usefulSupports >= 2;
        stageCondition = bothFeet
            && observation.feet[0].normalLoadNewtons
                +observation.feet[1].normalLoadNewtons
                > profile.totalMassKg*9.81f*0.30f;
        break;

    case BipedGetUpStage3D::Crouch:
        addCurrentHands(0.35f);
        addFeetUnderCom(1.0f);
        output.desiredComHeightWorld =
            ground+standingHeight*m_settings.crouchHeightFraction;
        output.desiredTorsoWeight = 1.0f;
        output.desiredComVerticalVelocity = 0.25f;
        output.residualEligible = usefulSupports >= 2;
        stageCondition = bothFeet
            && observation.torsoUpDot > 0.55f
            && observation.centerOfMassHeightMeters
                > standingHeight*0.57f;
        break;

    case BipedGetUpStage3D::Extend:
        addFeetUnderCom(1.0f);
        output.desiredComHeightWorld =
            ground+standingHeight*m_settings.standingHeightFraction;
        output.desiredTorsoWeight = 1.0f;
        output.desiredComVerticalVelocity = 0.18f;
        output.residualEligible = bothFeet;
        stageCondition = observation.uprightCandidate
            && bothFeet
            && observation.centerOfMassVelocityWorld.length() < 0.85f;
        break;

    case BipedGetUpStage3D::Complete:
        output.complete = true;
        return output;

    case BipedGetUpStage3D::None:
        break;
    }

    if (stageCondition)
        m_stableSeconds += deltaTime;
    else
        m_stableSeconds = 0.0f;

    if (m_stableSeconds >= m_settings.stableTransitionSeconds) {
        switch (m_stage) {
        case BipedGetUpStage3D::EstablishSupport:
            transition(BipedGetUpStage3D::RaiseTorso); break;
        case BipedGetUpStage3D::RaiseTorso:
            transition(BipedGetUpStage3D::BringFeetUnderCom); break;
        case BipedGetUpStage3D::BringFeetUnderCom:
            transition(BipedGetUpStage3D::Crouch); break;
        case BipedGetUpStage3D::Crouch:
            transition(BipedGetUpStage3D::Extend); break;
        case BipedGetUpStage3D::Extend:
            transition(BipedGetUpStage3D::Complete); break;
        default: break;
        }
        output.stage = m_stage;
        if (m_stage == BipedGetUpStage3D::Complete)
            output.complete = true;
    } else if (m_stageSeconds > m_settings.stageTimeoutSeconds) {
        ++m_replans;
        m_initialPose = classifyPose(profile, rig, state);
        transition(BipedGetUpStage3D::EstablishSupport);
        output.stage = m_stage;
        output.initialPose = m_initialPose;
        output.residualEligible = false;
    }

    return output;
}

} // namespace MatterEngine
