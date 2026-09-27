// Matter Engine clean-room physical biped controller.
// THEORY: [R05/R06] capture point/capturability; [R10] centroidal momentum.
// MATTER ADAPTATION: observation only; no control law from the retired system.

#include "Engine/Control/BipedStateEstimator3D.hpp"

#include "Engine/Control/BipedMath3D.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace MatterEngine {
namespace {

float linkMass(const RagdollProfile3D& profile, std::size_t index) {
    if (index >= profile.links.size()) return 0.0f;
    return std::max(0.0f,
        profile.totalMassKg*profile.links[index].massFraction);
}

Vec3 linkCenterOfMassWorld(const RagdollProfile3D& profile,
    const RagdollState3D& state, std::size_t index) {
    const auto& link = profile.links[index];
    const auto& body = state.links[index];
    return body.position
        + body.orientation.rotate(link.centerOfMassLocal);
}

Vec3 linkCenterOfMassVelocityWorld(const RagdollProfile3D& profile,
    const RagdollState3D& state, std::size_t index) {
    const auto& link = profile.links[index];
    const auto& body = state.links[index];
    const Vec3 r = body.orientation.rotate(link.centerOfMassLocal);
    return body.linearVelocity + cross(body.angularVelocity, r);
}

void appendFootBoxProjection(const RagdollProfile3D& profile,
    const RagdollState3D& state, std::size_t foot,
    std::vector<Vec3>& points) {
    if (foot >= profile.links.size() || foot >= state.links.size()) return;
    const auto& link = profile.links[foot];
    const auto& collider = link.collider;
    if (collider.shape != RagdollColliderShape3D::Box) return;

    const auto& body = state.links[foot];
    const Quaternion q =
        (body.orientation*collider.localOrientation).normalized();
    const Vec3 center =
        body.position+body.orientation.rotate(collider.localPosition);
    const Vec3 h = collider.boxHalfExtents;

    std::array<Vec3, 8> corners{};
    std::size_t cursor = 0;
    float minimumZ = std::numeric_limits<float>::infinity();
    for (int sx : {-1, 1}) {
        for (int sy : {-1, 1}) {
            for (int sz : {-1, 1}) {
                const Vec3 p = center+q.rotate({
                    static_cast<float>(sx)*h.x,
                    static_cast<float>(sy)*h.y,
                    static_cast<float>(sz)*h.z});
                corners[cursor++] = p;
                minimumZ = std::min(minimumZ, p.z);
            }
        }
    }
    const float tolerance =
        std::max(0.015f, std::min({h.x, h.y, h.z})*0.35f);
    for (const Vec3 p : corners) {
        if (p.z <= minimumZ+tolerance) points.push_back(p);
    }
}

float jointSaturation(const RagdollProfile3D& profile,
    const RagdollState3D& state) {
    float peak = 0.0f;
    const std::size_t count =
        std::min(profile.links.size(), state.joints.size());
    for (std::size_t i = 1; i < count; ++i) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const auto& definition = profile.links[i].inboundJoint.axes[axis];
            if (!definition.enabled || definition.maximumTorque <= 1.0e-4f)
                continue;
            const float measured =
                std::abs(state.joints[i].transmittedTorqueNewtonMeters[axis]);
            peak = std::max(peak, measured/definition.maximumTorque);
        }
    }
    return peak;
}

}

BipedObservation3D BipedStateEstimator3D::observe(
    const RagdollProfile3D& profile,
    const BipedRig3D& rig,
    const RagdollState3D& state,
    const RagdollDynamics3D& dynamics,
    float deltaTime) const {
    BipedObservation3D result;
    if (!rig.valid || !std::isfinite(deltaTime)
        || deltaTime <= 0.0f
        || state.links.size() != profile.links.size()
        || state.joints.size() != profile.links.size()
        || state.links.empty()) {
        return result;
    }
    for (const auto& body : state.links) {
        if (!finiteBiped(body.position)
            || !finiteBiped(body.orientation)
            || !finiteBiped(body.linearVelocity)
            || !finiteBiped(body.angularVelocity)) {
            return result;
        }
    }

    const float totalMass = std::max(profile.totalMassKg, 0.001f);
    if (dynamics.valid && finiteBiped(dynamics.centerOfMass)) {
        result.centerOfMassWorld = dynamics.centerOfMass;
    } else {
        Vec3 weighted;
        float measuredMass = 0.0f;
        for (std::size_t i = 0; i < profile.links.size(); ++i) {
            const float m = linkMass(profile, i);
            weighted += linkCenterOfMassWorld(profile, state, i)*m;
            measuredMass += m;
        }
        if (measuredMass > 1.0e-6f)
            result.centerOfMassWorld = weighted/measuredMass;
        else
            result.centerOfMassWorld = state.links.front().position;
    }

    Vec3 weightedVelocity;
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        weightedVelocity += linkCenterOfMassVelocityWorld(
            profile, state, i)*linkMass(profile, i);
    }
    result.centerOfMassVelocityWorld = weightedVelocity/totalMass;
    result.linearMomentumWorld =
        result.centerOfMassVelocityWorld*totalMass;

    if (dynamics.valid
        && dynamics.centroidalMomentumMatrix.size()
            >= static_cast<std::size_t>(6u)
                *dynamics.generalizedDofCount
        && dynamics.generalizedVelocity.size()
            >= dynamics.generalizedDofCount) {
        std::array<float, 6> h{};
        for (std::size_t row = 0; row < 6; ++row) {
            float value = 0.0f;
            for (std::size_t column = 0;
                    column < dynamics.generalizedDofCount; ++column) {
                value += dynamics.centroidalMomentumMatrix[
                    row*dynamics.generalizedDofCount+column]
                    *dynamics.generalizedVelocity[column];
            }
            h[row] = value;
        }
        result.linearMomentumWorld = {h[0], h[1], h[2]};
        result.angularMomentumWorld = {h[3], h[4], h[5]};
    }

    const auto& pelvisDef = profile.links[rig.pelvis];
    const auto& pelvisState = state.links[rig.pelvis];
    result.pelvisUpWorld = bipedAnatomicalAxisWorld3D(
        pelvisDef, pelvisState, {0.0f, 0.0f, 1.0f});
    const auto& torsoDef = profile.links[rig.torso];
    const auto& torsoState = state.links[rig.torso];
    result.torsoUpWorld = bipedAnatomicalAxisWorld3D(
        torsoDef, torsoState, {0.0f, 0.0f, 1.0f});
    result.torsoUpDot =
        dot(result.torsoUpWorld, Vec3{0.0f, 0.0f, 1.0f});

    result.forwardWorld = bipedAnatomicalAxisWorld3D(
        pelvisDef, pelvisState, {1.0f, 0.0f, 0.0f});
    result.forwardWorld.z = 0.0f;
    if (result.forwardWorld.lengthSquared() < 1.0e-8f) {
        result.forwardWorld = bipedAnatomicalAxisWorld3D(
            torsoDef, torsoState, {1.0f, 0.0f, 0.0f});
        result.forwardWorld.z = 0.0f;
    }
    if (result.forwardWorld.lengthSquared() < 1.0e-8f)
        result.forwardWorld = {1.0f, 0.0f, 0.0f};
    else
        result.forwardWorld = result.forwardWorld.normalized();
    result.leftWorld = {
        -result.forwardWorld.y, result.forwardWorld.x, 0.0f};

    const float weight = totalMass*9.81f;
    const float minimumFootLoad =
        weight*std::max(0.0f, m_settings.minimumFootLoadWeightFraction);
    std::array<Vec3, 2> weightedFootCenter{};
    std::array<Vec3, 2> weightedFootNormal{};
    std::array<float, 2> contactWeight{};

    for (std::size_t side = 0; side < 2; ++side) {
        auto& foot = result.feet[side];
        foot.linkIndex = rig.foot[side];
        if (foot.linkIndex < profile.links.size()) {
            foot.solePositionWorld = bipedSoleWorld3D(
                profile.links[foot.linkIndex],
                state.links[foot.linkIndex],
                {0.0f, 0.0f, 1.0f});
        }
    }

    for (const auto& contact : state.contacts) {
        if (contact.linkIndex >= state.links.size()
            || !finiteBiped(contact.position)
            || !finiteBiped(contact.normal)
            || !std::isfinite(contact.normalImpulseNewtonSeconds)
            || !std::isfinite(contact.tangentialSpeedMetersPerSecond)) {
            continue;
        }
        const float normalDot =
            dot(contact.normal, Vec3{0.0f, 0.0f, 1.0f});
        if (normalDot < m_settings.supportContactNormalMinimumDot) continue;
        const float normalLoad = std::max(
            0.0f, contact.normalImpulseNewtonSeconds/deltaTime);
        if (normalLoad <= 0.0f) continue;

        bool isFoot = false;
        for (std::size_t side = 0; side < 2; ++side) {
            auto& foot = result.feet[side];
            if (contact.linkIndex != foot.linkIndex) continue;
            isFoot = true;
            foot.normalLoadNewtons += normalLoad;
            foot.tangentialSpeedMetersPerSecond = std::max(
                foot.tangentialSpeedMetersPerSecond,
                contact.tangentialSpeedMetersPerSecond);
            foot.contactPointsWorld.push_back(contact.position);
            weightedFootCenter[side] += contact.position*normalLoad;
            weightedFootNormal[side] += contact.normal*normalLoad;
            contactWeight[side] += normalLoad;
        }

        BipedSupportContact3D support;
        support.linkIndex = contact.linkIndex;
        support.positionWorld = contact.position;
        support.normalWorld = contact.normal.normalized();
        support.normalLoadNewtons = normalLoad;
        support.tangentialSpeedMetersPerSecond =
            contact.tangentialSpeedMetersPerSecond;
        result.support.contacts.push_back(support);
        if (!isFoot) result.bodyGroundContact = true;
    }

    float footLoadTotal = 0.0f;
    for (std::size_t side = 0; side < 2; ++side) {
        auto& foot = result.feet[side];
        foot.contact = foot.normalLoadNewtons >= minimumFootLoad;
        foot.slipping = foot.contact
            && foot.tangentialSpeedMetersPerSecond
                > m_settings.slipSpeedMetersPerSecond;
        if (contactWeight[side] > 1.0e-5f) {
            foot.contactCenterWorld =
                weightedFootCenter[side]/contactWeight[side];
            foot.contactNormalWorld =
                weightedFootNormal[side].normalized();
        } else {
            foot.contactCenterWorld = foot.solePositionWorld;
            foot.contactNormalWorld = {0.0f, 0.0f, 1.0f};
        }
        footLoadTotal += foot.normalLoadNewtons;
    }
    if (footLoadTotal > 1.0e-5f) {
        result.feet[0].loadRatio =
            result.feet[0].normalLoadNewtons/footLoadTotal;
        result.feet[1].loadRatio =
            result.feet[1].normalLoadNewtons/footLoadTotal;
    }

    const bool left = result.feet[0].contact;
    const bool right = result.feet[1].contact;
    if (left && right)
        result.support.mode = BipedSupportMode3D::DoubleSupport;
    else if (left)
        result.support.mode = BipedSupportMode3D::LeftSupport;
    else if (right)
        result.support.mode = BipedSupportMode3D::RightSupport;
    else if (result.bodyGroundContact)
        result.support.mode = BipedSupportMode3D::MultiContactBody;
    else
        result.support.mode = BipedSupportMode3D::Airborne;

    std::vector<Vec3> footPoints;
    for (std::size_t side = 0; side < 2; ++side) {
        const auto& foot = result.feet[side];
        if (!foot.contact) continue;
        footPoints.insert(footPoints.end(),
            foot.contactPointsWorld.begin(), foot.contactPointsWorld.end());
        if (foot.contactPointsWorld.size() < 3) {
            appendFootBoxProjection(
                profile, state, foot.linkIndex, footPoints);
        }
    }
    result.support.footSupportPolygonWorld =
        convexHullXYBiped(footPoints);

    std::vector<Vec3> allPoints;
    allPoints.reserve(result.support.contacts.size()+footPoints.size());
    for (const auto& contact : result.support.contacts)
        allPoints.push_back(contact.positionWorld);
    allPoints.insert(allPoints.end(), footPoints.begin(), footPoints.end());
    result.support.allSupportPolygonWorld =
        convexHullXYBiped(allPoints);

    Vec3 supportCenter;
    float supportWeight = 0.0f;
    for (const auto& contact : result.support.contacts) {
        supportCenter +=
            contact.positionWorld*contact.normalLoadNewtons;
        supportWeight += contact.normalLoadNewtons;
    }
    if (supportWeight > 1.0e-5f) {
        supportCenter = supportCenter/supportWeight;
    } else {
        supportCenter = result.centerOfMassWorld;
        supportCenter.z -= profile.standingRootHeightMeters;
    }
    result.support.centerWorld = supportCenter;
    result.support.referenceHeightMeters = supportCenter.z;
    result.support.totalNormalLoadNewtons = supportWeight;
    result.centerOfMassHeightMeters =
        result.centerOfMassWorld.z-supportCenter.z;

    const float h = std::max(result.centerOfMassHeightMeters, 0.15f);
    const float omega = std::sqrt(9.81f/h);
    result.capturePointWorld =
        result.centerOfMassWorld+result.centerOfMassVelocityWorld/omega;
    result.capturePointWorld.z = supportCenter.z;
    result.captureMarginMeters =
        signedDistanceToConvexPolygonXYBiped(
            result.capturePointWorld,
            result.support.footSupportPolygonWorld);

    result.jointReactionTorqueRatioPeak = jointSaturation(profile, state);
    result.uprightCandidate =
        (left || right)
        && result.torsoUpDot >= m_settings.uprightTorsoDot
        && result.centerOfMassHeightMeters
            >= profile.standingRootHeightMeters
                *m_settings.uprightComHeightFraction;
    result.fallenCandidate =
        result.bodyGroundContact
        && (result.torsoUpDot < m_settings.fallenTorsoDot
            || result.centerOfMassHeightMeters
                < profile.standingRootHeightMeters
                    *m_settings.fallenComHeightFraction);

    result.valid = true;
    return result;
}

} // namespace MatterEngine
