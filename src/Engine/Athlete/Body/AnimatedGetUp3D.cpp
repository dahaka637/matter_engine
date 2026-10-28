#include "Engine/Athlete/Body/AnimatedGetUp3D.hpp"

#include "Engine/Character/RagdollPoseKinematics3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {

namespace {

// (A inercia da entrada: a pose fisica deitada vai ao clipe nisto - com 0,12
// s a cabeca girava a 666 graus/s.)
constexpr float EntryHalfLifeSeconds = 0.16f;
constexpr float EntrySeconds = 0.55f;

void worldPose(const RagdollProfile3D& profile, Vec3 rootPosition, Quaternion rootOrientation,
    const std::vector<Vec3>& coordinates, RagdollAnimationPose3D& pose) {
    RagdollAnimationPose3D local;
    buildRagdollLocalPose3D(profile, coordinates, local);
    pose.linkPositions.resize(profile.links.size());
    pose.linkOrientations.resize(profile.links.size());
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        pose.linkPositions[i] = rootPosition + rootOrientation.rotate(local.linkPositions[i]);
        pose.linkOrientations[i] = (rootOrientation * local.linkOrientations[i]).normalized();
    }
}

} // namespace

void AnimatedGetUp3D::clipRoot(const RagdollProfile3D& profile, float timeSeconds, std::vector<Vec3>& coordinates,
    Vec3& rootPosition, Quaternion& rootOrientation) const {
    const AnimationClip3D& clip = *m_clip;
    const float t = std::clamp(timeSeconds, 0.0f, clip.durationSeconds);
    coordinates.assign(profile.links.size(), {});
    for (std::size_t i = 1; i < profile.links.size(); ++i)
        if (const AnimationTrack3D* track = findAnimationTrack3D(clip, profile.links[i].id))
            coordinates[i] = sampleAnimationTrack3D(*track, t, clip.durationSeconds, false).jointPositionRadians;
    AnimationTransformSample3D root;
    if (const AnimationTrack3D* track = findAnimationTrack3D(clip, profile.links.front().id))
        root = sampleAnimationTrack3D(*track, t, clip.durationSeconds, false);
    const RagdollLinkDefinition3D& link = profile.links.front();
    rootPosition = m_base + m_turn.rotate(link.modelPosition + root.translationOffsetMeters - m_pivot);
    rootOrientation = (m_turn * link.modelOrientation * root.rotationDelta).normalized();
}

void AnimatedGetUp3D::start(const RagdollProfile3D& profile, const AnimationClip3D& clip, const Body& body) {
    m_clip = &clip;
    m_time = 0.0f;
    m_active = !profile.links.empty();
    if (!m_active) return;
    // O primeiro quadro no modelo: o eixo da pelve a cabeca e a pelve.
    const RagdollAnimationPose3D first = sampleRagdollAnimationPose3D(profile, &clip, 0.0f, false);
    std::size_t head = 0;
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        if (profile.links[i].id == "Head") head = i;
    const Vec3 clipAxis = first.linkPositions[head] - first.linkPositions.front();
    const Vec3 bodyAxis = body.headPosition - body.pelvisPosition;
    const float turn = std::atan2(bodyAxis.y, bodyAxis.x) - std::atan2(clipAxis.y, clipAxis.x);
    m_turn = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, turn);
    const RagdollLinkDefinition3D& root = profile.links.front();
    m_pivot = { first.linkPositions.front().x, first.linkPositions.front().y,
        root.modelPosition.z - profile.standingRootHeightMeters };
    m_base = { body.pelvisPosition.x, body.pelvisPosition.y, body.groundHeight };
    if (body.coordinates != nullptr && body.coordinateVelocities != nullptr)
        m_inertia.start(*body.coordinates, *body.coordinateVelocities, body.pelvisPosition, body.pelvisOrientation,
            body.pelvisVelocity, body.pelvisAngularVelocity, EntryHalfLifeSeconds, EntrySeconds);
    else
        m_inertia.stop();
}

void AnimatedGetUp3D::poseAt(const RagdollProfile3D& profile, float timeSeconds, RagdollAnimationPose3D& pose) const {
    if (m_clip == nullptr) return;
    std::vector<Vec3> coordinates;
    Vec3 rootPosition;
    Quaternion rootOrientation;
    clipRoot(profile, timeSeconds, coordinates, rootPosition, rootOrientation);
    worldPose(profile, rootPosition, rootOrientation, coordinates, pose);
}

void AnimatedGetUp3D::step(const RagdollProfile3D& profile, float deltaTime, RagdollAnimationPose3D& pose) {
    if (!m_active || m_clip == nullptr) return;
    m_time = std::min(m_time + std::max(0.0f, deltaTime), m_clip->durationSeconds);
    std::vector<Vec3> coordinates;
    Vec3 rootPosition;
    Quaternion rootOrientation;
    clipRoot(profile, m_time, coordinates, rootPosition, rootOrientation);
    if (m_inertia.active()) m_inertia.apply(coordinates, rootPosition, rootOrientation, deltaTime);
    worldPose(profile, rootPosition, rootOrientation, coordinates, pose);
}

} // namespace MatterEngine
