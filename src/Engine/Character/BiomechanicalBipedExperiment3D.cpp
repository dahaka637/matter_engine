#include "Engine/Character/BiomechanicalBipedExperiment3D.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <string_view>

namespace MatterEngine {
namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr float Gravity = 9.81f;

bool finite(Vec3 value) {
    return std::isfinite(value.x) && std::isfinite(value.y)
        && std::isfinite(value.z);
}

float component(Vec3 value, std::size_t axis) {
    return axis == 0 ? value.x : axis == 1 ? value.y : value.z;
}

void setComponent(Vec3& value, std::size_t axis, float result) {
    if (axis == 0) value.x = result;
    else if (axis == 1) value.y = result;
    else value.z = result;
}

float wrapAngle(float radians) {
    while (radians > Pi) radians -= 2.0f * Pi;
    while (radians < -Pi) radians += 2.0f * Pi;
    return radians;
}

float smoothStep(float value) {
    value = std::clamp(value, 0.0f, 1.0f);
    return value * value * (3.0f - 2.0f * value);
}

float getUpPhaseDuration(ProceduralGetUpPhase3D phase) {
    switch (phase) {
    case ProceduralGetUpPhase3D::Resting: return 0.90f;
    case ProceduralGetUpPhase3D::Assessing: return 0.28f;
    case ProceduralGetUpPhase3D::SupineTuck: return 0.95f;
    case ProceduralGetUpPhase3D::SupineSit: return 1.10f;
    case ProceduralGetUpPhase3D::ProneBrace: return 0.85f;
    case ProceduralGetUpPhase3D::PronePush: return 1.05f;
    case ProceduralGetUpPhase3D::GatherFeet: return 1.15f;
    case ProceduralGetUpPhase3D::Rise: return 1.25f;
    case ProceduralGetUpPhase3D::Stabilize: return 1.20f;
    case ProceduralGetUpPhase3D::None: return 1.0f;
    }
    return 1.0f;
}

Vec3 clampMagnitude(Vec3 value, float maximum) {
    const float length = value.length();
    return length > maximum && length > 0.000001f
        ? value * (maximum / length) : value;
}

Vec3 uprightOrientationError(Quaternion current, float desiredYaw,
    ProceduralFallOrientation3D fallOrientation) {
    const Vec3 currentForward = current.rotate({ 1.0f, 0.0f, 0.0f });
    const Vec3 currentLeft = current.rotate({ 0.0f, 1.0f, 0.0f });
    const Vec3 currentUp = current.rotate({ 0.0f, 0.0f, 1.0f });
    const Vec3 desiredForward {
        std::cos(desiredYaw), std::sin(desiredYaw), 0.0f
    };
    const Vec3 desiredLeft {
        -std::sin(desiredYaw), std::cos(desiredYaw), 0.0f
    };
    Vec3 error = (cross(currentForward, desiredForward)
        + cross(currentLeft, desiredLeft)
        + cross(currentUp, Vec3 { 0.0f, 0.0f, 1.0f })) * 0.5f;
    if (error.lengthSquared() < 0.0025f && currentUp.z < -0.65f) {
        error = desiredLeft * (fallOrientation
                == ProceduralFallOrientation3D::FaceDown ? -1.0f : 1.0f);
    }
    return error;
}

std::size_t findLink(const RagdollProfile3D& profile,
    std::string_view id) {
    for (std::size_t index = 0; index < profile.links.size(); ++index) {
        if (profile.links[index].id == id) return index;
    }
    return profile.links.size();
}

void setCoordinate(const RagdollProfile3D& profile,
    std::vector<Vec3>& targets, std::string_view linkId,
    std::size_t axis, float value) {
    const std::size_t link = findLink(profile, linkId);
    if (link >= targets.size() || axis >= 3) return;
    const auto& definition = profile.links[link].inboundJoint.axes[axis];
    if (!definition.enabled) return;
    setComponent(targets[link], axis, std::clamp(value,
        definition.minimumRadians, definition.maximumRadians));
}

void addCoordinate(const RagdollProfile3D& profile,
    std::vector<Vec3>& targets, std::string_view linkId,
    std::size_t axis, float amount) {
    const std::size_t link = findLink(profile, linkId);
    if (link >= targets.size() || axis >= 3) return;
    const auto& definition = profile.links[link].inboundJoint.axes[axis];
    if (!definition.enabled) return;
    setComponent(targets[link], axis, std::clamp(
        component(targets[link], axis) + amount,
        definition.minimumRadians, definition.maximumRadians));
}

Quaternion exponentialRotation(Vec3 coordinates) {
    const float angle = coordinates.length();
    return angle > 0.000001f
        ? Quaternion::fromAxisAngle(coordinates / angle, angle)
        : Quaternion {};
}

struct ProceduralLocalPose {
    std::vector<Vec3> linkPositions;
    std::vector<Quaternion> linkOrientations;
};

Vec3 exponentialJacobian(Vec3 coordinates, Vec3 velocity) {
    const float squared = coordinates.lengthSquared();
    const float angle = std::sqrt(squared);
    const float first = squared < 0.0001f
        ? 0.5f - squared / 24.0f
        : (1.0f - std::cos(angle)) / squared;
    const float second = squared < 0.0001f
        ? 1.0f / 6.0f - squared / 120.0f
        : (angle - std::sin(angle)) / (squared * angle);
    return velocity + cross(coordinates, velocity) * first
        + cross(coordinates, cross(coordinates, velocity)) * second;
}

void buildLocalPose(const RagdollProfile3D& profile,
    const std::vector<Vec3>& coordinates, ProceduralLocalPose& pose) {
    pose.linkPositions.assign(profile.links.size(), {});
    pose.linkOrientations.assign(profile.links.size(), {});
    for (std::size_t index = 1; index < profile.links.size(); ++index) {
        const RagdollLinkDefinition3D& link = profile.links[index];
        if (link.parentIndex < 0) continue;
        const std::size_t parent = static_cast<std::size_t>(link.parentIndex);
        const RagdollLinkDefinition3D& parentLink = profile.links[parent];
        Vec3 angles = index < coordinates.size()
            ? coordinates[index] : Vec3 {};
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const RagdollAxisDefinition3D& limit =
                link.inboundJoint.axes[axis];
            setComponent(angles, axis, limit.enabled
                ? std::clamp(component(angles, axis),
                    limit.minimumRadians, limit.maximumRadians)
                : 0.0f);
        }
        const Quaternion frame = link.inboundJoint.frameModelOrientation;
        pose.linkOrientations[index] =
            (pose.linkOrientations[parent]
                * parentLink.modelOrientation.conjugate()
                * frame * exponentialRotation(angles)
                * (link.modelOrientation.conjugate() * frame).conjugate())
                    .normalized();
        const Vec3 anchor = link.inboundJoint.anchorModelPosition;
        pose.linkPositions[index] = pose.linkPositions[parent]
            + pose.linkOrientations[parent].rotate(
                parentLink.modelOrientation.conjugate().rotate(
                    anchor - parentLink.modelPosition))
            - pose.linkOrientations[index].rotate(
                link.modelOrientation.conjugate().rotate(
                    anchor - link.modelPosition));
    }
}

// Damped least squares for the hip-knee-ankle chain. The target comes only
// from the procedural contact planner; no authored pose is sampled here.
void solveFootPosition(const RagdollProfile3D& profile,
    std::vector<Vec3>& coordinates, std::size_t footIndex,
    Vec3 targetRootLocal, float weight) {
    if (footIndex == 0 || footIndex >= profile.links.size()
        || coordinates.size() != profile.links.size()
        || !finite(targetRootLocal) || weight <= 0.0f) {
        return;
    }
    struct DegreeOfFreedom {
        std::size_t link = 0;
        std::size_t axis = 0;
    };
    std::array<DegreeOfFreedom, 9> degrees;
    std::size_t count = 0;
    int linkIndex = static_cast<int>(footIndex);
    for (int depth = 0; depth < 3 && linkIndex > 0; ++depth) {
        const auto& joint = profile.links[static_cast<std::size_t>(linkIndex)]
            .inboundJoint;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            if (joint.axes[axis].enabled && count < degrees.size()) {
                degrees[count++] = {
                    static_cast<std::size_t>(linkIndex), axis
                };
            }
        }
        linkIndex = profile.links[static_cast<std::size_t>(linkIndex)]
            .parentIndex;
    }
    if (count == 0) return;

    const std::vector<Vec3> original = coordinates;
    ProceduralLocalPose pose;
    for (int iteration = 0; iteration < 9; ++iteration) {
        buildLocalPose(profile, coordinates, pose);
        const Vec3 error = targetRootLocal - pose.linkPositions[footIndex];
        if (error.lengthSquared() < 0.00000016f) break;

        float jacobian[3][9] {};
        for (std::size_t column = 0; column < count; ++column) {
            const DegreeOfFreedom degree = degrees[column];
            const RagdollLinkDefinition3D& link = profile.links[degree.link];
            const std::size_t parent = static_cast<std::size_t>(
                link.parentIndex);
            const RagdollLinkDefinition3D& parentLink = profile.links[parent];
            const Quaternion parentModelFrame = pose.linkOrientations[parent]
                * parentLink.modelOrientation.conjugate();
            const Vec3 basis = degree.axis == 0
                ? Vec3 { 1.0f, 0.0f, 0.0f }
                : degree.axis == 1 ? Vec3 { 0.0f, 1.0f, 0.0f }
                : Vec3 { 0.0f, 0.0f, 1.0f };
            const Vec3 angular = (parentModelFrame
                * link.inboundJoint.frameModelOrientation).rotate(
                    exponentialJacobian(coordinates[degree.link], basis));
            const Vec3 anchor = pose.linkPositions[parent]
                + parentModelFrame.rotate(
                    link.inboundJoint.anchorModelPosition
                        - parentLink.modelPosition);
            const Vec3 linear = cross(angular,
                pose.linkPositions[footIndex] - anchor);
            jacobian[0][column] = linear.x;
            jacobian[1][column] = linear.y;
            jacobian[2][column] = linear.z;
        }

        float system[9][10] {};
        for (std::size_t row = 0; row < count; ++row) {
            for (std::size_t column = 0; column < count; ++column) {
                for (std::size_t dimension = 0;
                        dimension < 3; ++dimension) {
                    system[row][column] += jacobian[dimension][row]
                        * jacobian[dimension][column];
                }
                if (row == column) system[row][column] += 0.018f;
            }
            system[row][count] = jacobian[0][row] * error.x
                + jacobian[1][row] * error.y
                + jacobian[2][row] * error.z;
        }
        for (std::size_t pivot = 0; pivot < count; ++pivot) {
            std::size_t best = pivot;
            for (std::size_t row = pivot + 1; row < count; ++row) {
                if (std::abs(system[row][pivot])
                    > std::abs(system[best][pivot])) best = row;
            }
            if (best != pivot) {
                for (std::size_t column = pivot;
                        column <= count; ++column) {
                    std::swap(system[pivot][column], system[best][column]);
                }
            }
            const float divisor = system[pivot][pivot];
            if (std::abs(divisor) < 0.000001f) continue;
            for (std::size_t column = pivot; column <= count; ++column) {
                system[pivot][column] /= divisor;
            }
            for (std::size_t row = 0; row < count; ++row) {
                if (row == pivot) continue;
                const float factor = system[row][pivot];
                for (std::size_t column = pivot;
                        column <= count; ++column) {
                    system[row][column] -=
                        factor * system[pivot][column];
                }
            }
        }
        for (std::size_t degreeIndex = 0;
                degreeIndex < count; ++degreeIndex) {
            const DegreeOfFreedom degree = degrees[degreeIndex];
            const RagdollAxisDefinition3D& limit =
                profile.links[degree.link].inboundJoint.axes[degree.axis];
            const float next = component(
                coordinates[degree.link], degree.axis)
                + std::clamp(system[degreeIndex][count], -0.14f, 0.14f);
            setComponent(coordinates[degree.link], degree.axis,
                std::clamp(next,
                    limit.minimumRadians, limit.maximumRadians));
        }
    }

    weight = std::clamp(weight, 0.0f, 1.0f);
    for (std::size_t degreeIndex = 0;
            degreeIndex < count; ++degreeIndex) {
        const DegreeOfFreedom degree = degrees[degreeIndex];
        const float result = component(
            coordinates[degree.link], degree.axis);
        const float before = component(original[degree.link], degree.axis);
        setComponent(coordinates[degree.link], degree.axis,
            before + (result - before) * weight);
    }
}

float supportMargin(Vec3 capture, Vec3 minimum, Vec3 maximum) {
    if (capture.x < minimum.x) return capture.x - minimum.x;
    if (capture.x > maximum.x) return maximum.x - capture.x;
    if (capture.y < minimum.y) return capture.y - minimum.y;
    if (capture.y > maximum.y) return maximum.y - capture.y;
    return std::min({ capture.x - minimum.x, maximum.x - capture.x,
        capture.y - minimum.y, maximum.y - capture.y });
}

float footSoleOffset(const RagdollLinkDefinition3D& link) {
    return link.collider.shape == RagdollColliderShape3D::Box
        ? link.collider.boxHalfExtents.z : link.collider.radiusMeters;
}

// Blends an authored clip's joint coordinates into the desired-coordinate
// buffer. Root tracks are skipped: the floating base is never guided here.
// weight 1 replaces the current target outright; weight 0 leaves it alone.
void applyClipPose(const RagdollProfile3D& profile,
    const AnimationClip3D& clip, float timeSeconds,
    std::vector<Vec3>& targets, float weight) {
    if (weight <= 0.0f) return;
    weight = std::min(weight, 1.0f);
    for (const AnimationTrack3D& track : clip.tracks) {
        if (track.space != AnimationTrackSpace3D::Joint) continue;
        const std::size_t link = findLink(profile, track.targetLinkId);
        if (link >= targets.size()) continue;
        const Vec3 sampled = sampleAnimationTrack3D(track, timeSeconds,
            clip.durationSeconds, clip.loops).jointPositionRadians;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const RagdollAxisDefinition3D& definition =
                profile.links[link].inboundJoint.axes[axis];
            if (!definition.enabled) continue;
            const float value = std::clamp(component(sampled, axis),
                definition.minimumRadians, definition.maximumRadians);
            setComponent(targets[link], axis, component(targets[link], axis)
                + (value - component(targets[link], axis)) * weight);
        }
    }
}

// A foot's largest contiguous low-contact run in an authored clip, i.e. the
// window its swing actually occupies. Used to phase-lock clip sampling to
// the physical planner's own touchdown/liftoff instead of a free-running
// clock that would drift out of sync with real contacts.
struct ClipSwingWindow {
    float startSeconds = 0.0f;
    float durationSeconds = 0.0f;
    bool valid = false;
};

ClipSwingWindow findClipSwingWindow(const AnimationScalarTrack3D& contact,
    float clipDurationSeconds) {
    ClipSwingWindow window;
    const std::size_t count = contact.keyframes.size();
    if (count < 2 || clipDurationSeconds <= 0.0f) return window;
    constexpr float SwingThreshold = 0.5f;
    std::size_t bestStart = count;
    std::size_t bestLength = 0;
    for (std::size_t start = 0; start < count; ++start) {
        if (contact.keyframes[start].value >= SwingThreshold) continue;
        std::size_t length = 0;
        while (length < count && contact.keyframes[(start + length) % count]
                .value < SwingThreshold) {
            ++length;
        }
        if (length > bestLength) {
            bestLength = length;
            bestStart = start;
        }
    }
    if (bestLength == 0 || bestStart == count) return window;
    const std::size_t endIndex = (bestStart + bestLength) % count;
    float endTime = contact.keyframes[endIndex].timeSeconds;
    if (endIndex <= bestStart) endTime += clipDurationSeconds;
    window.startSeconds = contact.keyframes[bestStart].timeSeconds;
    window.durationSeconds = std::max(0.01f,
        endTime - window.startSeconds);
    window.valid = true;
    return window;
}

float mapSwingPhaseToClipTime(const ClipSwingWindow& window,
    float clipDurationSeconds, float phaseProgress) {
    if (!window.valid || clipDurationSeconds <= 0.0f) return 0.0f;
    float time = window.startSeconds
        + std::clamp(phaseProgress, 0.0f, 1.0f) * window.durationSeconds;
    time = std::fmod(time, clipDurationSeconds);
    if (time < 0.0f) time += clipDurationSeconds;
    return time;
}

// Height of one foot above its own liftoff point, purely from the clip's
// geometry (no root motion, no physical stride). This is what makes a
// running gait read as a run rather than a walk: how high the swing knee
// actually lifts, independent of how far the physical planner sends it.
float sampleClipFootHeightAbovePelvis(const RagdollProfile3D& profile,
    const AnimationClip3D& clip, float timeSeconds,
    std::size_t footLink) {
    std::vector<Vec3> coordinates(profile.links.size(), Vec3 {});
    applyClipPose(profile, clip, timeSeconds, coordinates, 1.0f);
    ProceduralLocalPose pose;
    buildLocalPose(profile, coordinates, pose);
    return footLink < pose.linkPositions.size()
        ? pose.linkPositions[footLink].z : 0.0f;
}

} // namespace

void BiomechanicalBipedExperiment3D::reset(
    const RagdollProfile3D& profile, const RagdollState3D& state,
    const RagdollDynamics3D& dynamics) {
    m_output = {};
    m_telemetry = {};
    m_targets.assign(profile.links.size(), Vec3 {});
    m_previousTargets.assign(profile.links.size(), Vec3 {});
    for (std::size_t link = 1;
            link < profile.links.size() && link < state.joints.size(); ++link) {
        m_targets[link] = {
            state.joints[link].positionRadians[0],
            state.joints[link].positionRadians[1],
            state.joints[link].positionRadians[2]
        };
    }
    m_previousTargets = m_targets;
    m_thighLinks = { findLink(profile, "LeftThigh"),
        findLink(profile, "RightThigh") };
    m_shinLinks = { findLink(profile, "LeftShin"),
        findLink(profile, "RightShin") };
    m_footLinks = { findLink(profile, "LeftFoot"),
        findLink(profile, "RightFoot") };
    m_handLinks = { findLink(profile, "LeftHand"),
        findLink(profile, "RightHand") };
    m_chestLink = findLink(profile, "Chest");
    m_contactConfidence = {};
    m_contactPointWorld = {};
    m_previousCenterOfMass = dynamics.valid
        && dynamics.centerOfMass.lengthSquared() > 0.0f
        ? dynamics.centerOfMass
        : state.links.empty() ? Vec3 {} : state.links.front().position;
    m_filteredCenterOfMassVelocity = {};
    float groundHeight = m_previousCenterOfMass.z
        - profile.standingRootHeightMeters;
    std::array<Vec3, 2> feet {};
    for (std::size_t side = 0; side < 2; ++side) {
        const std::size_t foot = m_footLinks[side];
        if (foot < state.links.size()) {
            feet[side] = state.links[foot].position;
            groundHeight = std::min(groundHeight,
                feet[side].z - footSoleOffset(profile.links[foot]));
            m_contactPointWorld[side] = {
                feet[side].x, feet[side].y, groundHeight
            };
        }
    }
    m_gait.reset(feet);
    m_nominalCenterOfMassHeight = std::clamp(
        m_previousCenterOfMass.z - groundHeight, 0.72f, 1.12f);
    m_idleClipSeconds = 0.0f;
    m_locomotionClipSeconds = 0.0f;
    m_elapsedSeconds = 0.0f;
    m_unsupportedSeconds = 0.0f;
    m_getUpPhase = ProceduralGetUpPhase3D::None;
    m_fallOrientation = ProceduralFallOrientation3D::None;
    m_getUpPhaseSeconds = 0.0f;
    m_recoveredStandingSeconds = 0.0f;
    m_getUpAttempt = 0;
    m_initialized = true;
}

void BiomechanicalBipedExperiment3D::update(
    const RagdollProfile3D& profile,
    const RagdollState3D& state,
    const RagdollDynamics3D& dynamics,
    const BiomechanicalBipedInput3D& input,
    float deltaTime) {
    deltaTime = std::clamp(deltaTime, 1.0f / 1000.0f, 1.0f / 30.0f);
    if (!m_initialized || m_targets.size() != profile.links.size()) {
        reset(profile, state, dynamics);
    }
    m_output = {};
    m_output.gravityCompensationEnabled = true;
    m_telemetry = {};
    m_telemetry.dynamicsAvailable = dynamics.valid;
    m_elapsedSeconds += deltaTime;

    const Vec3 centerOfMass = dynamics.valid
        ? dynamics.centerOfMass
        : state.links.empty() ? Vec3 {} : state.links.front().position;
    Vec3 measuredCenterVelocity =
        (centerOfMass - m_previousCenterOfMass) / deltaTime;
    if (!finite(measuredCenterVelocity)) measuredCenterVelocity = {};
    m_previousCenterOfMass = centerOfMass;
    const float velocityBlend = 1.0f - std::exp(-16.0f * deltaTime);
    m_filteredCenterOfMassVelocity +=
        (measuredCenterVelocity - m_filteredCenterOfMassVelocity)
            * velocityBlend;
    const Vec3 centerVelocity = m_filteredCenterOfMassVelocity;

    std::array<bool, 2> touched {};
    std::array<float, 2> impulses {};
    std::array<Vec3, 2> contactSums {};
    bool anyGroundContact = false;
    float anyGroundHeight = std::numeric_limits<float>::max();
    std::array<bool, 2> handGroundContact {};
    std::array<bool, 2> kneeGroundContact {};
    for (const RagdollContactPoint3D& contact : state.contacts) {
        if (contact.normal.z >= 0.40f
            && contact.normalImpulseNewtonSeconds > 0.00001f) {
            anyGroundContact = true;
            anyGroundHeight = std::min(anyGroundHeight, contact.position.z);
            for (std::size_t side = 0; side < 2; ++side) {
                handGroundContact[side] = handGroundContact[side]
                    || contact.linkIndex == m_handLinks[side];
                kneeGroundContact[side] = kneeGroundContact[side]
                    || contact.linkIndex == m_shinLinks[side];
            }
        }
        for (std::size_t side = 0; side < 2; ++side) {
            if (contact.linkIndex != m_footLinks[side]
                || contact.normal.z < 0.45f
                || contact.normalImpulseNewtonSeconds <= 0.00001f) {
                continue;
            }
            touched[side] = true;
            impulses[side] += contact.normalImpulseNewtonSeconds;
            contactSums[side] += contact.position
                * contact.normalImpulseNewtonSeconds;
        }
    }
    for (std::size_t side = 0; side < 2; ++side) {
        if (touched[side] && impulses[side] > 0.00001f) {
            m_contactPointWorld[side] = contactSums[side] / impulses[side];
            m_contactConfidence[side] = std::min(1.0f,
                m_contactConfidence[side] + deltaTime * 24.0f);
        } else {
            m_contactConfidence[side] = std::max(0.0f,
                m_contactConfidence[side] - deltaTime * 9.0f);
        }
        m_telemetry.footContact[side] =
            m_contactConfidence[side] > 0.18f;
        m_telemetry.contactLoadNewtons[side] = impulses[side] / deltaTime;
    }

    float rootUpright = 0.0f;
    float actualYaw = input.desiredFacingYawRadians;
    Quaternion rootOrientation;
    Vec3 rootPosition = centerOfMass;
    Vec3 rootAngularVelocity;
    if (!state.links.empty()) {
        rootOrientation = state.links.front().orientation;
        rootPosition = state.links.front().position;
        rootAngularVelocity = state.links.front().angularVelocity;
        const Vec3 up = rootOrientation.rotate({ 0.0f, 0.0f, 1.0f });
        const Vec3 forward = rootOrientation.rotate({ 1.0f, 0.0f, 0.0f });
        rootUpright = up.z;
        actualYaw = std::atan2(forward.y, forward.x);
    }
    const Quaternion heading = Quaternion::fromAxisAngle(
        { 0.0f, 0.0f, 1.0f }, actualYaw);

    float supportWeight = 0.0f;
    Vec3 supportCenter;
    const float fallbackGround =
        centerOfMass.z - m_nominalCenterOfMassHeight;
    float groundHeight = 0.0f;
    float groundWeight = 0.0f;
    Vec3 supportMinimum {
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(), 0.0f
    };
    Vec3 supportMaximum {
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max(), 0.0f
    };
    std::array<Vec3, 2> footPositions {};
    for (std::size_t side = 0; side < 2; ++side) {
        const std::size_t foot = m_footLinks[side];
        footPositions[side] = foot < state.links.size()
            ? state.links[foot].position : m_contactPointWorld[side];
        if (!m_telemetry.footContact[side]) continue;
        const float weight = std::max(0.05f, m_contactConfidence[side]);
        supportCenter += footPositions[side] * weight;
        supportWeight += weight;
        groundHeight += m_contactPointWorld[side].z * weight;
        groundWeight += weight;
        const Vec3 localFoot = heading.conjugate().rotate(
            footPositions[side]);
        supportMinimum.x = std::min(supportMinimum.x,
            localFoot.x - 0.095f);
        supportMinimum.y = std::min(supportMinimum.y,
            localFoot.y - 0.036f);
        supportMaximum.x = std::max(supportMaximum.x,
            localFoot.x + 0.095f);
        supportMaximum.y = std::max(supportMaximum.y,
            localFoot.y + 0.036f);
    }
    if (supportWeight > 0.0f) supportCenter *= 1.0f / supportWeight;
    else supportCenter = { centerOfMass.x, centerOfMass.y, fallbackGround };
    groundHeight = groundWeight > 0.0f
        ? groundHeight / groundWeight
        : anyGroundContact ? anyGroundHeight : fallbackGround;

    const float measuredComHeight = std::max(0.08f,
        centerOfMass.z - groundHeight);
    const float comHeight = std::max(0.35f, measuredComHeight);
    const float omega = std::sqrt(Gravity / comHeight);
    Vec3 capturePoint = centerOfMass + centerVelocity / omega;
    capturePoint.z = groundHeight;
    const Vec3 captureLocal = heading.conjugate().rotate(capturePoint);
    const float margin = supportWeight > 0.0f
        ? supportMargin(captureLocal, supportMinimum, supportMaximum) : -1.0f;

    if (supportWeight <= 0.0f) m_unsupportedSeconds += deltaTime;
    else m_unsupportedSeconds = 0.0f;
    const bool fallenObservation = m_elapsedSeconds > 0.45f
        && anyGroundContact
        && (rootUpright < 0.45f || measuredComHeight < 0.52f);

    const auto enterGetUpPhase = [&](ProceduralGetUpPhase3D phase) {
        m_getUpPhase = phase;
        m_getUpPhaseSeconds = 0.0f;
    };
    const auto classifyFallOrientation = [&]() {
        const std::size_t reference = m_chestLink < state.links.size()
            ? m_chestLink : 0;
        const Vec3 bodyForward = state.links[reference]
            .orientation.rotate({ 1.0f, 0.0f, 0.0f });
        m_fallOrientation = bodyForward.z < 0.0f
            ? ProceduralFallOrientation3D::FaceDown
            : ProceduralFallOrientation3D::FaceUp;
    };
    const bool feetReady = m_telemetry.footContact[0]
        || m_telemetry.footContact[1];
    const bool handsReady = handGroundContact[0] || handGroundContact[1];
    const bool kneesReady = kneeGroundContact[0] || kneeGroundContact[1];
    if (m_getUpPhase == ProceduralGetUpPhase3D::None) {
        if (fallenObservation) {
            classifyFallOrientation();
            enterGetUpPhase(ProceduralGetUpPhase3D::Resting);
            ++m_getUpAttempt;
            m_recoveredStandingSeconds = 0.0f;
            m_gait.reset(footPositions);
        }
    } else {
        m_getUpPhaseSeconds += deltaTime;
        const float duration = getUpPhaseDuration(m_getUpPhase);
        switch (m_getUpPhase) {
        case ProceduralGetUpPhase3D::Resting:
            if (m_getUpPhaseSeconds >= duration) {
                enterGetUpPhase(ProceduralGetUpPhase3D::Assessing);
            }
            break;
        case ProceduralGetUpPhase3D::Assessing:
            if (m_getUpPhaseSeconds >= duration) {
                classifyFallOrientation();
                enterGetUpPhase(m_fallOrientation
                        == ProceduralFallOrientation3D::FaceDown
                    ? ProceduralGetUpPhase3D::ProneBrace
                    : ProceduralGetUpPhase3D::SupineTuck);
            }
            break;
        case ProceduralGetUpPhase3D::SupineTuck:
            if ((feetReady && m_getUpPhaseSeconds > 0.62f)
                || m_getUpPhaseSeconds >= duration) {
                enterGetUpPhase(ProceduralGetUpPhase3D::SupineSit);
            }
            break;
        case ProceduralGetUpPhase3D::SupineSit:
            if ((feetReady && rootUpright > 0.10f
                    && m_getUpPhaseSeconds > 0.72f)
                || m_getUpPhaseSeconds >= duration) {
                enterGetUpPhase(ProceduralGetUpPhase3D::GatherFeet);
            }
            break;
        case ProceduralGetUpPhase3D::ProneBrace:
            if ((handsReady && m_getUpPhaseSeconds > 0.48f)
                || m_getUpPhaseSeconds >= duration) {
                enterGetUpPhase(ProceduralGetUpPhase3D::PronePush);
            }
            break;
        case ProceduralGetUpPhase3D::PronePush:
            if ((handsReady && (kneesReady || feetReady)
                    && m_getUpPhaseSeconds > 0.68f)
                || m_getUpPhaseSeconds >= duration) {
                enterGetUpPhase(ProceduralGetUpPhase3D::GatherFeet);
            }
            break;
        case ProceduralGetUpPhase3D::GatherFeet:
            if ((feetReady && m_getUpPhaseSeconds > 0.72f)
                || m_getUpPhaseSeconds >= duration) {
                enterGetUpPhase(ProceduralGetUpPhase3D::Rise);
            }
            break;
        case ProceduralGetUpPhase3D::Rise:
            if ((rootUpright > 0.68f && measuredComHeight > 0.64f
                    && m_getUpPhaseSeconds > 0.78f)
                || m_getUpPhaseSeconds >= duration) {
                enterGetUpPhase(ProceduralGetUpPhase3D::Stabilize);
            }
            break;
        case ProceduralGetUpPhase3D::Stabilize: {
            const bool recovered = rootUpright > 0.78f
                && measuredComHeight > 0.68f && supportWeight > 0.20f;
            m_recoveredStandingSeconds = recovered
                ? m_recoveredStandingSeconds + deltaTime : 0.0f;
            if (m_recoveredStandingSeconds > 0.28f) {
                m_getUpPhase = ProceduralGetUpPhase3D::None;
                m_fallOrientation = ProceduralFallOrientation3D::None;
                m_getUpPhaseSeconds = 0.0f;
                m_recoveredStandingSeconds = 0.0f;
                m_getUpAttempt = 0;
                m_gait.reset(footPositions);
            } else if (m_getUpPhaseSeconds >= duration) {
                classifyFallOrientation();
                enterGetUpPhase(ProceduralGetUpPhase3D::Resting);
                ++m_getUpAttempt;
            }
            break;
        }
        case ProceduralGetUpPhase3D::None:
            break;
        }
    }
    const bool getUpActive =
        m_getUpPhase != ProceduralGetUpPhase3D::None;
    const bool fallen = fallenObservation && !getUpActive;

    Vec3 desiredVelocity = input.desiredVelocityWorld;
    desiredVelocity.z = 0.0f;
    desiredVelocity = clampMagnitude(desiredVelocity, 1.15f);
    const float yawError = wrapAngle(
        input.desiredFacingYawRadians - actualYaw);

    ProceduralBipedGaitInput3D gaitInput;
    gaitInput.footPositionWorld = footPositions;
    gaitInput.footContact = m_telemetry.footContact;
    gaitInput.contactConfidence = m_contactConfidence;
    gaitInput.contactLoadNewtons = m_telemetry.contactLoadNewtons;
    gaitInput.centerOfMassWorld = centerOfMass;
    gaitInput.centerOfMassVelocityWorld = centerVelocity;
    gaitInput.capturePointWorld = capturePoint;
    gaitInput.observedSupportCenterWorld = supportCenter;
    gaitInput.desiredVelocityWorld = desiredVelocity;
    gaitInput.desiredFacingYawRadians = input.desiredFacingYawRadians;
    gaitInput.facingErrorRadians = yawError;
    gaitInput.supportMarginMeters = margin;
    gaitInput.groundHeightWorld = groundHeight;
    gaitInput.bodyMassKg = profile.totalMassKg;
    gaitInput.deltaTime = deltaTime;
    gaitInput.allowStepping = m_elapsedSeconds > 0.68f
        && rootUpright > 0.66f && !fallenObservation && !getUpActive;
    gaitInput.fallen = fallenObservation || getUpActive;
    m_gait.update(gaitInput);
    const ProceduralBipedGaitOutput3D& gait = m_gait.output();

    // Base pose in this rig's joint frames. The authored idle reference sets
    // the full-body seed when available; a fixed fallback keeps the rig
    // sane before the catalog finishes loading or when no clip is wired.
    m_idleClipSeconds += deltaTime;
    std::vector<Vec3> desiredCoordinates(profile.links.size(), Vec3 {});
    if (input.idleReferenceClip != nullptr) {
        applyClipPose(profile, *input.idleReferenceClip, m_idleClipSeconds,
            desiredCoordinates, 1.0f);
    } else {
        setCoordinate(profile, desiredCoordinates, "LeftUpperArm", 1, 1.20f);
        setCoordinate(profile, desiredCoordinates, "RightUpperArm", 1, -1.20f);
        setCoordinate(profile, desiredCoordinates, "LeftForearm", 0, 0.28f);
        setCoordinate(profile, desiredCoordinates, "RightForearm", 0, 0.28f);
        setCoordinate(profile, desiredCoordinates, "LeftShin", 0, 0.015f);
        setCoordinate(profile, desiredCoordinates, "RightShin", 0, 0.015f);
    }

    if (getUpActive) {
        const float phaseProgress = smoothStep(std::clamp(
            m_getUpPhaseSeconds
                / getUpPhaseDuration(m_getUpPhase), 0.0f, 1.0f));
        const auto setLegs = [&](float hipFlexion, float kneeFlexion,
                float anklePitch) {
            setCoordinate(profile, desiredCoordinates,
                "LeftThigh", 1, hipFlexion);
            setCoordinate(profile, desiredCoordinates,
                "RightThigh", 1, hipFlexion);
            setCoordinate(profile, desiredCoordinates,
                "LeftShin", 0, kneeFlexion);
            setCoordinate(profile, desiredCoordinates,
                "RightShin", 0, kneeFlexion);
            setCoordinate(profile, desiredCoordinates,
                "LeftFoot", 0, anklePitch);
            setCoordinate(profile, desiredCoordinates,
                "RightFoot", 0, anklePitch);
        };
        const auto setSpine = [&](float bend) {
            setCoordinate(profile, desiredCoordinates,
                "Abdomen", 1, bend * 0.48f);
            setCoordinate(profile, desiredCoordinates,
                "Chest", 1, bend * 0.34f);
            setCoordinate(profile, desiredCoordinates,
                "UpperChest", 1, bend * 0.22f);
            setCoordinate(profile, desiredCoordinates,
                "Neck", 1, -bend * 0.18f);
        };
        switch (m_getUpPhase) {
        case ProceduralGetUpPhase3D::Resting:
            for (std::size_t link = 1;
                    link < desiredCoordinates.size()
                    && link < state.joints.size(); ++link) {
                desiredCoordinates[link] = {
                    state.joints[link].positionRadians[0],
                    state.joints[link].positionRadians[1],
                    state.joints[link].positionRadians[2]
                };
            }
            break;
        case ProceduralGetUpPhase3D::Assessing:
            setLegs(-0.30f, 0.42f, -0.08f);
            setSpine(0.24f);
            setCoordinate(profile, desiredCoordinates,
                "LeftForearm", 0, 0.72f);
            setCoordinate(profile, desiredCoordinates,
                "RightForearm", 0, 0.72f);
            break;
        case ProceduralGetUpPhase3D::SupineTuck:
            setLegs(-0.55f - phaseProgress * 0.70f,
                0.65f + phaseProgress * 1.02f, -0.24f);
            setSpine(0.42f + phaseProgress * 0.48f);
            setCoordinate(profile, desiredCoordinates,
                "LeftUpperArm", 1, 0.72f);
            setCoordinate(profile, desiredCoordinates,
                "RightUpperArm", 1, -0.72f);
            setCoordinate(profile, desiredCoordinates,
                "LeftForearm", 0, 1.12f);
            setCoordinate(profile, desiredCoordinates,
                "RightForearm", 0, 1.12f);
            break;
        case ProceduralGetUpPhase3D::SupineSit:
            setLegs(-1.28f, 1.72f, -0.18f);
            setSpine(0.82f - phaseProgress * 0.20f);
            setCoordinate(profile, desiredCoordinates,
                "LeftUpperArm", 0, -0.52f);
            setCoordinate(profile, desiredCoordinates,
                "RightUpperArm", 0, -0.52f);
            setCoordinate(profile, desiredCoordinates,
                "LeftUpperArm", 1, 0.86f);
            setCoordinate(profile, desiredCoordinates,
                "RightUpperArm", 1, -0.86f);
            setCoordinate(profile, desiredCoordinates,
                "LeftForearm", 0, 1.02f);
            setCoordinate(profile, desiredCoordinates,
                "RightForearm", 0, 1.02f);
            break;
        case ProceduralGetUpPhase3D::ProneBrace:
            setLegs(-0.48f - phaseProgress * 0.32f,
                0.58f + phaseProgress * 0.52f, -0.12f);
            setSpine(0.16f);
            setCoordinate(profile, desiredCoordinates,
                "LeftUpperArm", 0, -0.88f);
            setCoordinate(profile, desiredCoordinates,
                "RightUpperArm", 0, -0.88f);
            setCoordinate(profile, desiredCoordinates,
                "LeftUpperArm", 1, 0.62f);
            setCoordinate(profile, desiredCoordinates,
                "RightUpperArm", 1, -0.62f);
            setCoordinate(profile, desiredCoordinates,
                "LeftForearm", 0, 1.34f);
            setCoordinate(profile, desiredCoordinates,
                "RightForearm", 0, 1.34f);
            break;
        case ProceduralGetUpPhase3D::PronePush:
            setLegs(-0.82f - phaseProgress * 0.40f,
                1.10f + phaseProgress * 0.55f, -0.20f);
            setSpine(0.28f + phaseProgress * 0.32f);
            setCoordinate(profile, desiredCoordinates,
                "LeftUpperArm", 0, -0.52f * (1.0f - phaseProgress));
            setCoordinate(profile, desiredCoordinates,
                "RightUpperArm", 0, -0.52f * (1.0f - phaseProgress));
            setCoordinate(profile, desiredCoordinates,
                "LeftUpperArm", 1, 0.86f);
            setCoordinate(profile, desiredCoordinates,
                "RightUpperArm", 1, -0.86f);
            setCoordinate(profile, desiredCoordinates,
                "LeftForearm", 0, 1.22f - phaseProgress * 0.42f);
            setCoordinate(profile, desiredCoordinates,
                "RightForearm", 0, 1.22f - phaseProgress * 0.42f);
            break;
        case ProceduralGetUpPhase3D::GatherFeet:
            setLegs(-1.22f, 1.68f, -0.18f);
            setSpine(0.62f);
            setCoordinate(profile, desiredCoordinates,
                "LeftUpperArm", 1, 0.82f);
            setCoordinate(profile, desiredCoordinates,
                "RightUpperArm", 1, -0.82f);
            setCoordinate(profile, desiredCoordinates,
                "LeftForearm", 0, 0.72f);
            setCoordinate(profile, desiredCoordinates,
                "RightForearm", 0, 0.72f);
            break;
        case ProceduralGetUpPhase3D::Rise: {
            const float crouch = 1.0f - phaseProgress;
            setLegs(-1.12f * crouch,
                1.58f * crouch + 0.015f * phaseProgress,
                -0.16f * crouch);
            setSpine(0.55f * crouch);
            setCoordinate(profile, desiredCoordinates,
                "LeftUpperArm", 1, 0.82f + phaseProgress * 0.38f);
            setCoordinate(profile, desiredCoordinates,
                "RightUpperArm", 1, -0.82f - phaseProgress * 0.38f);
            setCoordinate(profile, desiredCoordinates,
                "LeftForearm", 0, 0.58f - phaseProgress * 0.30f);
            setCoordinate(profile, desiredCoordinates,
                "RightForearm", 0, 0.58f - phaseProgress * 0.30f);
            break;
        }
        case ProceduralGetUpPhase3D::Stabilize:
            setLegs(-0.02f, 0.025f, 0.0f);
            setSpine(0.05f * (1.0f - phaseProgress));
            break;
        case ProceduralGetUpPhase3D::None:
            break;
        }
    } else {
    // Blend the authored locomotion reference over the idle base as soon as
    // movement is requested; there is no separate walk clip, only run.
    // While a foot is actually swinging, the clip clock is phase-locked to
    // that foot's own authored swing window (from its contact curve) so
    // liftoff/touchdown line up with the real contacts below; otherwise it
    // free-wheels at a rate scaled by the requested speed so it does not
    // freeze during double support.
    const float runBlend = smoothStep(
        desiredVelocity.length() / 0.5f);
    if (input.locomotionReferenceClip != nullptr && runBlend > 0.0f) {
        const AnimationClip3D& locomotionClip =
            *input.locomotionReferenceClip;
        if (gait.swingFoot == 0 || gait.swingFoot == 1) {
            const ClipSwingWindow window = findClipSwingWindow(
                gait.swingFoot == 0 ? locomotionClip.leftFootContact
                                    : locomotionClip.rightFootContact,
                locomotionClip.durationSeconds);
            m_locomotionClipSeconds = mapSwingPhaseToClipTime(window,
                locomotionClip.durationSeconds, gait.phaseProgress);
        } else if (locomotionClip.nominalSpeedMetersPerSecond > 0.01f) {
            m_locomotionClipSeconds += deltaTime * desiredVelocity.length()
                / locomotionClip.nominalSpeedMetersPerSecond;
            m_locomotionClipSeconds = std::fmod(
                m_locomotionClipSeconds, locomotionClip.durationSeconds);
            if (m_locomotionClipSeconds < 0.0f) {
                m_locomotionClipSeconds += locomotionClip.durationSeconds;
            }
        }
        applyClipPose(profile, locomotionClip, m_locomotionClipSeconds,
            desiredCoordinates, runBlend);
    }

    const Vec3 localVelocityError = heading.conjugate().rotate(
        desiredVelocity - centerVelocity);
    addCoordinate(profile, desiredCoordinates, "Abdomen", 1,
        std::clamp(localVelocityError.x * 0.040f, -0.09f, 0.09f));
    addCoordinate(profile, desiredCoordinates, "Chest", 1,
        std::clamp(localVelocityError.x * 0.028f, -0.065f, 0.065f));
    addCoordinate(profile, desiredCoordinates, "Abdomen", 2,
        std::clamp(-localVelocityError.y * 0.035f, -0.07f, 0.07f));
    addCoordinate(profile, desiredCoordinates, "Chest", 2,
        std::clamp(-localVelocityError.y * 0.020f, -0.045f, 0.045f));
    addCoordinate(profile, desiredCoordinates, "Abdomen", 0,
        std::clamp(yawError * 0.30f, -0.14f, 0.14f));
    addCoordinate(profile, desiredCoordinates, "Chest", 0,
        std::clamp(yawError * 0.22f, -0.10f, 0.10f));
    addCoordinate(profile, desiredCoordinates, "UpperChest", 0,
        std::clamp(yawError * 0.12f, -0.055f, 0.055f));
    addCoordinate(profile, desiredCoordinates, "Neck", 0,
        std::clamp(yawError * 0.10f, -0.07f, 0.07f));

    const float armOpen = std::clamp((-margin - 0.005f) * 2.6f,
        0.0f, 0.38f);
    addCoordinate(profile, desiredCoordinates,
        "LeftUpperArm", 1, -armOpen);
    addCoordinate(profile, desiredCoordinates,
        "RightUpperArm", 1, armOpen);
    addCoordinate(profile, desiredCoordinates,
        "LeftForearm", 0, armOpen * 0.20f);
    addCoordinate(profile, desiredCoordinates,
        "RightForearm", 0, armOpen * 0.20f);

    if (!state.links.empty() && !fallen
        && gait.phase != ProceduralContactPhase3D::Unsupported) {
        Vec3 taskRootPosition = rootPosition;
        const float desiredPelvisHeight = groundHeight
            + profile.standingRootHeightMeters;
        taskRootPosition.z += std::clamp(
            desiredPelvisHeight - rootPosition.z, -0.03f, 0.18f);
        for (std::size_t side = 0; side < 2; ++side) {
            const std::size_t foot = m_footLinks[side];
            if (foot >= profile.links.size()) continue;
            Vec3 plannedFootTarget = gait.footTargetWorld[side];
            // The physical planner already guarantees a safe, reachable
            // landing spot; only its swing-height shape is reinforced here
            // from the locomotion clip, so a running command visibly lifts
            // the knee instead of tracing the same clearance as before.
            if (runBlend > 0.0f && input.locomotionReferenceClip != nullptr
                && static_cast<int>(side) == gait.swingFoot) {
                const AnimationClip3D& locomotionClip =
                    *input.locomotionReferenceClip;
                const ClipSwingWindow window = findClipSwingWindow(
                    side == 0 ? locomotionClip.leftFootContact
                              : locomotionClip.rightFootContact,
                    locomotionClip.durationSeconds);
                if (window.valid) {
                    const float liftoffHeight =
                        sampleClipFootHeightAbovePelvis(profile,
                            locomotionClip, window.startSeconds, foot);
                    const float currentHeight =
                        sampleClipFootHeightAbovePelvis(profile,
                            locomotionClip, m_locomotionClipSeconds, foot);
                    const float clipLift = std::max(0.0f,
                        currentHeight - liftoffHeight);
                    plannedFootTarget.z += clipLift * runBlend;
                }
            }
            const Vec3 targetRootLocal = rootOrientation.conjugate().rotate(
                plannedFootTarget - taskRootPosition);
            solveFootPosition(profile, desiredCoordinates, foot,
                targetRootLocal, 1.0f);
        }
    }

    // Actual sole orientation closes the ankle loop. Swing dorsiflexion at
    // the apex gives toe clearance independently from vertical translation.
    for (std::size_t side = 0; side < 2; ++side) {
        const std::size_t foot = m_footLinks[side];
        if (foot >= state.links.size() || foot >= profile.links.size()) continue;
        const Quaternion footOrientation = state.links[foot].orientation;
        const Vec3 footForward = footOrientation.rotate({ 1, 0, 0 });
        const Vec3 footLeft = footOrientation.rotate({ 0, 1, 0 });
        const Vec3 footUp = footOrientation.rotate({ 0, 0, 1 });
        const float footPitch = std::atan2(-footForward.z,
            std::hypot(footForward.x, footForward.y));
        const float footRoll = std::atan2(footLeft.z,
            std::max(0.001f, footUp.z));
        addCoordinate(profile, desiredCoordinates,
            profile.links[foot].id, 0,
            std::clamp(-footPitch * 0.86f, -0.28f, 0.28f));
        addCoordinate(profile, desiredCoordinates,
            profile.links[foot].id, 1,
            std::clamp(footRoll * 0.86f, -0.22f, 0.22f));
        if (gait.swingFoot == static_cast<int>(side)
            && gait.phase == ProceduralContactPhase3D::Swing) {
            addCoordinate(profile, desiredCoordinates,
                profile.links[foot].id, 0,
                -0.17f * std::sin(Pi * gait.phaseProgress));
        }
    }
    }

    const float targetRate = m_getUpPhase == ProceduralGetUpPhase3D::Resting
        ? 4.0f : getUpActive ? 8.5f
        : gait.phase == ProceduralContactPhase3D::Swing ? 38.0f : 20.0f;
    const float targetBlend = 1.0f - std::exp(-targetRate * deltaTime);
    for (std::size_t link = 1; link < m_targets.size(); ++link) {
        m_targets[link] += (desiredCoordinates[link] - m_targets[link])
            * targetBlend;
    }

    Vec3 desiredAcceleration = (desiredVelocity - centerVelocity) * 3.6f;
    desiredAcceleration += Vec3 {
        gait.balanceTargetWorld.x - centerOfMass.x,
        gait.balanceTargetWorld.y - centerOfMass.y, 0.0f
    } * (gait.reason == ProceduralStepReason3D::BalanceRecovery
        ? 10.5f : 6.8f);
    desiredAcceleration.z = std::clamp(
        (groundHeight + m_nominalCenterOfMassHeight - centerOfMass.z) * 15.0f
            - centerVelocity.z * 5.5f,
        -2.0f, 3.2f);
    Vec3 horizontalAcceleration {
        desiredAcceleration.x, desiredAcceleration.y, 0.0f
    };
    horizontalAcceleration = clampMagnitude(horizontalAcceleration,
        gait.reason == ProceduralStepReason3D::BalanceRecovery ? 3.8f : 2.8f);
    desiredAcceleration.x = horizontalAcceleration.x;
    desiredAcceleration.y = horizontalAcceleration.y;
    Vec3 totalReaction = desiredAcceleration * profile.totalMassKg;
    totalReaction.z += profile.totalMassKg * Gravity;
    const float frictionLimit = std::max(0.0f, totalReaction.z * 0.68f);
    Vec3 tangential { totalReaction.x, totalReaction.y, 0.0f };
    tangential = clampMagnitude(tangential, frictionLimit);
    totalReaction.x = tangential.x;
    totalReaction.y = tangential.y;

    const float assistance = std::clamp(
        input.balanceAssistance, 0.0f, 1.0f);
    if (getUpActive && m_getUpPhase != ProceduralGetUpPhase3D::Resting
        && anyGroundContact && assistance > 0.0f) {
        const float progress = smoothStep(std::clamp(
            m_getUpPhaseSeconds / getUpPhaseDuration(m_getUpPhase),
            0.0f, 1.0f));
        float desiredHeight = 0.30f;
        float maximumLiftShare = 0.30f;
        float uprightShare = 0.22f;
        switch (m_getUpPhase) {
        case ProceduralGetUpPhase3D::Assessing:
            desiredHeight = 0.30f;
            maximumLiftShare = 0.16f;
            uprightShare = 0.15f;
            break;
        case ProceduralGetUpPhase3D::SupineTuck:
            desiredHeight = 0.31f + progress * 0.09f;
            maximumLiftShare = 0.34f;
            uprightShare = 0.30f + progress * 0.16f;
            break;
        case ProceduralGetUpPhase3D::SupineSit:
            desiredHeight = 0.52f;
            maximumLiftShare = 0.58f;
            uprightShare = 0.70f;
            break;
        case ProceduralGetUpPhase3D::ProneBrace:
            desiredHeight = 0.34f;
            maximumLiftShare = 0.26f;
            uprightShare = 0.34f;
            break;
        case ProceduralGetUpPhase3D::PronePush:
            desiredHeight = 0.48f;
            maximumLiftShare = 0.58f;
            uprightShare = 0.72f;
            break;
        case ProceduralGetUpPhase3D::GatherFeet:
            desiredHeight = 0.60f;
            maximumLiftShare = 0.68f;
            uprightShare = 0.88f;
            break;
        case ProceduralGetUpPhase3D::Rise:
            desiredHeight = 0.60f
                + (m_nominalCenterOfMassHeight - 0.60f) * progress;
            maximumLiftShare = 0.72f;
            uprightShare = 0.92f + progress * 0.08f;
            break;
        case ProceduralGetUpPhase3D::Stabilize:
            desiredHeight = m_nominalCenterOfMassHeight;
            maximumLiftShare = 0.42f;
            uprightShare = 1.0f;
            break;
        case ProceduralGetUpPhase3D::Resting:
        case ProceduralGetUpPhase3D::None:
            break;
        }

        Vec3 feetCenter = (footPositions[0] + footPositions[1]) * 0.5f;
        Vec3 planarError = feetCenter - centerOfMass;
        planarError.z = 0.0f;
        Vec3 planarVelocity = centerVelocity;
        planarVelocity.z = 0.0f;
        Vec3 guideForce = planarError * (profile.totalMassKg * 5.8f)
            - planarVelocity * (profile.totalMassKg * 1.15f);
        guideForce = clampMagnitude(guideForce,
            profile.totalMassKg * Gravity
                * (0.20f + std::min(3u, m_getUpAttempt) * 0.025f));

        float supportReadiness = 1.0f;
        if ((m_getUpPhase == ProceduralGetUpPhase3D::ProneBrace
                || m_getUpPhase == ProceduralGetUpPhase3D::PronePush)
            && !handsReady) {
            supportReadiness = 0.42f;
        }
        if ((m_getUpPhase == ProceduralGetUpPhase3D::GatherFeet
                || m_getUpPhase == ProceduralGetUpPhase3D::Rise
                || m_getUpPhase == ProceduralGetUpPhase3D::Stabilize)
            && !feetReady) {
            supportReadiness = 0.38f;
        }
        if (m_getUpPhase == ProceduralGetUpPhase3D::PronePush
            && !kneesReady && !feetReady) {
            supportReadiness *= 0.72f;
        }
        const float attemptBoost = 1.0f
            + std::min(3u, m_getUpAttempt > 0 ? m_getUpAttempt - 1 : 0u)
                * 0.12f;
        const float heightError = desiredHeight - measuredComHeight;
        guideForce.z = profile.totalMassKg
            * (heightError * 11.0f - centerVelocity.z * 2.8f);
        const float maximumLift = profile.totalMassKg * Gravity
            * std::min(0.88f,
                maximumLiftShare * attemptBoost * supportReadiness);
        guideForce.z = std::clamp(guideForce.z,
            -profile.totalMassKg * Gravity * 0.08f, maximumLift);
        m_output.balanceForceWorld = guideForce * assistance;

        const Vec3 orientationError = uprightOrientationError(
            rootOrientation, input.desiredFacingYawRadians,
            m_fallOrientation);
        Vec3 guideTorque = orientationError
                * (profile.totalMassKg * 22.0f
                    * uprightShare * attemptBoost)
            - rootAngularVelocity
                * (profile.totalMassKg * (1.05f + uprightShare * 0.72f));
        guideTorque = clampMagnitude(guideTorque,
            profile.totalMassKg
                * (1.55f + uprightShare * 2.15f) * attemptBoost);
        m_output.balanceTorqueWorld = guideTorque * assistance;
    } else if (!getUpActive && supportWeight > 0.0f
        && !fallen && assistance > 0.0f) {
        Vec3 planarVelocity = centerVelocity;
        planarVelocity.z = 0.0f;
        const bool reversing = desiredVelocity.lengthSquared() > 0.01f
            && dot(desiredVelocity, planarVelocity) < -0.035f;
        const bool stopping = desiredVelocity.lengthSquared() <= 0.01f
            && planarVelocity.lengthSquared() > 0.10f;
        const bool recovery = gait.reason
            == ProceduralStepReason3D::BalanceRecovery;
        m_telemetry.predictiveBraking = reversing || stopping || recovery;

        Vec3 desiredCapture = gait.balanceTargetWorld
            + desiredVelocity / omega;
        desiredCapture.z = groundHeight;
        Vec3 assistanceForce = (desiredCapture - capturePoint)
                * (profile.totalMassKg * omega * 1.72f)
            + (desiredVelocity - planarVelocity)
                * (profile.totalMassKg * 0.72f);
        assistanceForce.z = 0.0f;
        assistanceForce = clampMagnitude(assistanceForce,
            profile.totalMassKg * Gravity
                * (m_telemetry.predictiveBraking ? 0.36f : 0.28f))
            * assistance;
        const float heightForce = std::clamp(
            profile.totalMassKg
                * ((m_nominalCenterOfMassHeight - comHeight) * 18.0f
                    - centerVelocity.z * 3.2f),
            -profile.totalMassKg * Gravity * 0.10f,
            profile.totalMassKg * Gravity * 0.20f);
        m_output.balanceForceWorld = assistanceForce;
        m_output.balanceForceWorld.z = heightForce * assistance;

        if (!state.links.empty()) {
            const Vec3 rootUp = rootOrientation.rotate({ 0, 0, 1 });
            const Vec3 tiltError = cross(rootUp, Vec3 { 0, 0, 1 });
            Vec3 planarAngularVelocity = rootAngularVelocity;
            planarAngularVelocity.z = 0.0f;
            Vec3 tiltTorque = tiltError
                    * (profile.totalMassKg
                        * (m_telemetry.predictiveBraking ? 21.5f : 19.5f))
                - planarAngularVelocity * (profile.totalMassKg * 1.45f);
            tiltTorque = clampMagnitude(tiltTorque,
                profile.totalMassKg
                    * (m_telemetry.predictiveBraking ? 2.30f : 2.02f));
            const float yawTorque = std::clamp(
                yawError * profile.totalMassKg * 1.75f
                    - rootAngularVelocity.z * profile.totalMassKg * 0.42f,
                -profile.totalMassKg * 0.72f,
                profile.totalMassKg * 0.72f);
            m_output.balanceTorqueWorld = tiltTorque
                + Vec3 { 0.0f, 0.0f, yawTorque };
            m_output.balanceTorqueWorld = clampMagnitude(
                m_output.balanceTorqueWorld,
                profile.totalMassKg
                    * (m_telemetry.predictiveBraking ? 2.42f : 2.14f))
                * assistance;
        }
    }

    std::vector<float> jointFeedforward(
        dynamics.generalizedDofCount, 0.0f);
    if (dynamics.valid && supportWeight > 0.0f && !fallen
        && !getUpActive) {
        std::array<float, 2> forceWeights {};
        float forceWeightSum = 0.0f;
        for (std::size_t side = 0; side < 2; ++side) {
            if (!m_telemetry.footContact[side]
                || !gait.plannedContact[side]) continue;
            forceWeights[side] = m_contactConfidence[side]
                * std::max(0.12f,
                    m_telemetry.contactLoadNewtons[side]
                        / std::max(1.0f, profile.totalMassKg * Gravity));
            forceWeightSum += forceWeights[side];
        }
        if (forceWeightSum <= 0.0001f) {
            for (std::size_t side = 0; side < 2; ++side) {
                if (gait.plannedContact[side]) {
                    forceWeights[side] = 1.0f;
                    forceWeightSum += 1.0f;
                }
            }
        }
        Vec3 captureErrorLocal = heading.conjugate().rotate(
            capturePoint - gait.balanceTargetWorld);
        captureErrorLocal.z = 0.0f;
        constexpr float AnkleStrategyShare = 0.16f;
        const float pitchEffort = std::clamp(
            profile.totalMassKg * Gravity * captureErrorLocal.x
                * AnkleStrategyShare, -22.0f, 22.0f);
        const float rollEffort = std::clamp(
            profile.totalMassKg * Gravity * captureErrorLocal.y
                * AnkleStrategyShare, -17.0f, 17.0f);
        for (std::size_t side = 0; side < 2; ++side) {
            const std::size_t foot = m_footLinks[side];
            if (forceWeights[side] <= 0.0f
                || foot >= dynamics.jointGeneralizedDof.size()) continue;
            const float weight = forceWeights[side]
                / std::max(0.0001f, forceWeightSum);
            const std::uint32_t pitchDof =
                dynamics.jointGeneralizedDof[foot][0];
            const std::uint32_t rollDof =
                dynamics.jointGeneralizedDof[foot][1];
            if (pitchDof != RagdollDynamics3D::InvalidIndex
                && pitchDof < jointFeedforward.size()) {
                jointFeedforward[pitchDof] += pitchEffort * weight;
            }
            if (rollDof != RagdollDynamics3D::InvalidIndex
                && rollDof < jointFeedforward.size()) {
                jointFeedforward[rollDof] += rollEffort * weight;
            }
        }
    }

    const float muscle = std::clamp(input.muscleAuthority, 0.0f, 1.0f);
    for (std::size_t link = 1;
            link < profile.links.size() && link < state.joints.size(); ++link) {
        const std::string& id = profile.links[link].id;
        const bool isArm = id.find("Arm") != std::string::npos
            || id.find("Forearm") != std::string::npos
            || id.find("Hand") != std::string::npos;
        const bool isTorso = id == "Abdomen" || id == "Chest"
            || id == "UpperChest" || id == "Neck" || id == "Head";
        const bool isLeftLeg = link == m_thighLinks[0]
            || link == m_shinLinks[0] || link == m_footLinks[0];
        const bool isRightLeg = link == m_thighLinks[1]
            || link == m_shinLinks[1] || link == m_footLinks[1];
        const bool isKnee = link == m_shinLinks[0]
            || link == m_shinLinks[1];
        const bool isAnkle = link == m_footLinks[0]
            || link == m_footLinks[1];
        const int legSide = isLeftLeg ? 0 : isRightLeg ? 1 : -1;
        const bool stanceLeg = legSide >= 0
            && gait.plannedContact[static_cast<std::size_t>(legSide)];
        float stiffness = isArm ? 0.52f : isTorso ? 1.18f
            : stanceLeg ? 1.48f : 1.26f;
        float damping = isArm ? 0.66f : isTorso ? 1.08f
            : stanceLeg ? 1.22f : 1.04f;
        float torque = isArm ? 0.82f : isTorso ? 1.42f
            : stanceLeg ? 2.20f : 1.92f;
        if (isKnee && !getUpActive) {
            stiffness += stanceLeg ? 0.20f : 0.12f;
            damping += 0.10f;
            torque += stanceLeg ? 0.28f : 0.18f;
        } else if (isAnkle && !getUpActive) {
            stiffness += 0.10f;
            damping += 0.08f;
            torque += 0.12f;
        }
        if (m_getUpPhase == ProceduralGetUpPhase3D::Resting) {
            stiffness = 0.16f;
            damping = 0.88f;
            torque = 0.38f;
        } else if (getUpActive) {
            stiffness = isArm ? 1.16f : isTorso ? 1.38f : 1.58f;
            damping = isArm ? 1.02f : isTorso ? 1.18f : 1.22f;
            torque = isArm ? 1.82f : isTorso ? 2.08f : 2.55f;
            if (isKnee) {
                stiffness += 0.16f;
                torque += 0.22f;
            }
        } else if (fallen) {
            stiffness *= 0.38f;
            damping *= 0.70f;
            torque *= 0.52f;
        }
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const auto& definition =
                profile.links[link].inboundJoint.axes[axis];
            if (!definition.enabled) continue;
            RagdollDriveTarget3D target;
            target.linkIndex = static_cast<std::uint32_t>(link);
            target.axis = static_cast<RagdollAxis3D>(axis);
            target.positionRadians = component(m_targets[link], axis);
            target.velocityRadiansPerSecond = std::clamp(
                (component(m_targets[link], axis)
                    - component(m_previousTargets[link], axis)) / deltaTime,
                -11.0f, 11.0f);
            target.stiffnessScale = stiffness * muscle;
            target.dampingScale = damping * std::sqrt(muscle);
            target.maximumTorqueScale = torque * muscle;
            if (link < dynamics.jointGeneralizedDof.size()) {
                const std::uint32_t dof =
                    dynamics.jointGeneralizedDof[link][axis];
                if (dof != RagdollDynamics3D::InvalidIndex
                    && dof < jointFeedforward.size()) {
                    target.feedforwardTorqueNewtonMeters =
                        jointFeedforward[dof] * muscle;
                }
            }
            m_output.driveTargets.push_back(target);
        }
    }
    m_previousTargets = m_targets;

    BiomechanicalBipedPhase3D phase = BiomechanicalBipedPhase3D::Standing;
    if (getUpActive) phase = BiomechanicalBipedPhase3D::GettingUp;
    else if (fallenObservation) phase = BiomechanicalBipedPhase3D::Fallen;
    else if (m_elapsedSeconds < 0.68f)
        phase = BiomechanicalBipedPhase3D::Settling;
    else if (m_unsupportedSeconds > 0.10f)
        phase = BiomechanicalBipedPhase3D::Unsupported;
    else if (gait.reason == ProceduralStepReason3D::BalanceRecovery)
        phase = BiomechanicalBipedPhase3D::CaptureStep;
    else if (gait.phase != ProceduralContactPhase3D::DoubleSupport)
        phase = BiomechanicalBipedPhase3D::Walking;

    m_telemetry.phase = phase;
    m_telemetry.centerOfMassWorld = centerOfMass;
    m_telemetry.centerOfMassVelocityWorld = centerVelocity;
    m_telemetry.capturePointWorld = capturePoint;
    m_telemetry.supportCenterWorld = supportCenter;
    m_telemetry.requestedGroundReactionNewtons = totalReaction;
    m_telemetry.balanceForceWorld = m_output.balanceForceWorld;
    m_telemetry.balanceTorqueWorld = m_output.balanceTorqueWorld;
    m_telemetry.plannedFootContact = gait.plannedContact;
    m_telemetry.proceduralFootTargetWorld = gait.footTargetWorld;
    m_telemetry.supportMarginMeters = margin;
    m_telemetry.gaitPhase = gait.phaseProgress;
    m_telemetry.swingClearanceMeters = gait.swingClearanceMeters;
    m_telemetry.desiredSpeedMetersPerSecond = desiredVelocity.length();
    m_telemetry.rootUpright = rootUpright;
    m_telemetry.swingFoot = gait.swingFoot;
    m_telemetry.contactPhase = gait.phase;
    m_telemetry.stepReason = gait.reason;
    m_telemetry.getUpPhase = m_getUpPhase;
    m_telemetry.fallOrientation = m_fallOrientation;
    m_telemetry.getUpProgress = std::clamp(
        m_getUpPhaseSeconds / getUpPhaseDuration(m_getUpPhase),
        0.0f, 1.0f);
    m_telemetry.getUpAttempt = m_getUpAttempt;
}

} // namespace MatterEngine
