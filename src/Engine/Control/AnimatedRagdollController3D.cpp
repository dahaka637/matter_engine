#include "Engine/Control/AnimatedRagdollController3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {
constexpr float Pi = 3.14159265358979323846f;
Vec3 limited(Vec3 value, float maximum) {
  const float length = value.length();
  return length > maximum ? value * (maximum / length) : value;
}
float smooth(float t) {
  t = std::clamp(t, 0.0f, 1.0f);
  return t * t * (3 - 2 * t);
}
Quaternion rotation(Vec3 v) {
  const float n = v.length();
  return n < 1e-6f ? Quaternion{} : Quaternion::fromAxisAngle(v / n, n);
}
Quaternion heading(Quaternion q) {
  const Vec3 forward = q.rotate({1, 0, 0});
  return Quaternion::fromAxisAngle({0, 0, 1}, std::atan2(forward.y, forward.x));
}
float component(Vec3 v, std::size_t i) {
  return i == 0 ? v.x : i == 1 ? v.y : v.z;
}
void setComponent(Vec3 &v, std::size_t i, float f) {
  if (i == 0)
    v.x = f;
  else if (i == 1)
    v.y = f;
  else
    v.z = f;
}
} // namespace

bool AnimatedRagdollClips3D::compatible(const RagdollProfile3D &profile) const {
  return idle && run && stop && idle->loops && run->loops && !stop->loops &&
         validateAnimationClipForRagdoll3D(*idle, profile).empty() &&
         validateAnimationClipForRagdoll3D(*run, profile).empty() &&
         validateAnimationClipForRagdoll3D(*stop, profile).empty();
}

void AnimatedRagdollController3D::reset(const RagdollProfile3D &profile,
                                        const RagdollState3D &state,
                                        float /*groundHeight*/) {
  m_initialized = state.links.size() == profile.links.size() &&
                  state.joints.size() == profile.links.size() &&
                  !state.links.empty();
  m_output = {};
  m_telemetry = {};
  m_phaseTime = 0;
  m_blendTime = 0;
  m_speed = 0;
  m_wasManipulated = false;
  m_previousBodyPositionsValid = false;
  if (!m_initialized)
    return;
  m_commandOrigin = state.links.front().position;
  m_heading = heading(state.links.front().orientation);
  m_coordinates.assign(profile.links.size(), {});
  for (std::size_t i = 1; i < state.joints.size(); ++i)
    m_coordinates[i] = {state.joints[i].positionRadians[0],
                        state.joints[i].positionRadians[1],
                        state.joints[i].positionRadians[2]};
  m_blendFrom = m_coordinates;
}

void AnimatedRagdollController3D::transition(AnimatedRagdollPhase3D phase) {
  m_telemetry.phase = phase;
  m_phaseTime = 0;
  m_blendTime = 0;
  m_blendFrom = m_coordinates;
}

bool AnimatedRagdollController3D::requestRun(const RagdollState3D &state,
                                             float seconds) {
  if (!m_initialized || state.links.empty() ||
      m_telemetry.phase != AnimatedRagdollPhase3D::Idle ||
      m_telemetry.manipulated || !std::isfinite(seconds) || seconds <= 0)
    return false;
  m_requestedSeconds = std::clamp(seconds, 0.1f, 30.0f);
  m_heading = heading(state.links.front().orientation);
  m_commandOrigin = state.links.front().position;
  m_speed = 0;
  m_telemetry.blocked = false;
  transition(AnimatedRagdollPhase3D::Running);
  return true;
}

void AnimatedRagdollController3D::update(const RagdollProfile3D &profile,
                                         const AnimatedRagdollClips3D &clips,
                                         const RagdollState3D &state, float dt,
                                         float groundHeight, bool manipulated,
                                         bool obstacleAhead) {
  m_output.driveTargets.clear();
  m_output.assistance.clear();
  m_output.gravityCompensationEnabled = false;
  m_telemetry.assistanceForceNewtons = 0;
  m_telemetry.assistanceTorqueNewtonMeters = 0;
  m_telemetry.assistanceNetForceWorld = {};
  m_telemetry.assistanceAuthority = 0;
  m_telemetry.supportLoadNewtons = 0;
  if (!std::isfinite(dt) || dt <= 0 || dt > 0.05f ||
      state.links.size() != profile.links.size() ||
      state.joints.size() != profile.links.size() || state.links.empty() ||
      !std::isfinite(groundHeight) || !clips.idle || !clips.run || !clips.stop)
    return;
  if (!m_initialized)
    reset(profile, state, groundHeight);
  m_telemetry.manipulated = manipulated;
  if (manipulated) {
    // Yield to the Physgun, including internal gravity feedforward.
    // On release, blend from the actual pose at the actual location.
    m_wasManipulated = true;
    m_previousBodyPositionsValid = false;
    for (std::size_t i = 1; i < profile.links.size(); ++i)
      for (std::size_t a = 0; a < 3; ++a)
        if (profile.links[i].inboundJoint.axes[a].enabled) {
          RagdollDriveTarget3D drive;
          drive.linkIndex = static_cast<std::uint32_t>(i);
          drive.axis = static_cast<RagdollAxis3D>(a);
          drive.stiffnessScale = drive.dampingScale = drive.maximumTorqueScale =
              0;
          m_output.driveTargets.push_back(drive);
        }
    return;
  }
  if (m_wasManipulated)
    reset(profile, state, groundHeight);
  m_phaseTime += dt;
  m_blendTime += dt;
  if (m_telemetry.phase == AnimatedRagdollPhase3D::Running &&
      (m_phaseTime >= m_requestedSeconds || obstacleAhead)) {
    m_telemetry.blocked = obstacleAhead;
    const float stopStartSpeed = std::max(
        0.0f, dot(state.links.front().linearVelocity,
                  heading(state.links.front().orientation).rotate({1, 0, 0})));
    const auto *root =
        findAnimationTrack3D(*clips.stop, profile.links.front().id);
    const float interval = 1.0f / clips.stop->sourceSampleRateHz;
    const float sourceSpeed =
        root ? (sampleAnimationTrack3D(*root, interval,
                                       clips.stop->durationSeconds, false)
                    .translationOffsetMeters.x -
                sampleAnimationTrack3D(*root, 0, clips.stop->durationSeconds,
                                       false)
                    .translationOffsetMeters.x) /
                   interval
             : 0;
    m_stopRate = sourceSpeed > 0.1f
                     ? std::clamp(stopStartSpeed / sourceSpeed, 0.5f, 2.5f)
                     : 1;
    transition(AnimatedRagdollPhase3D::Stopping);
  }
  if (m_telemetry.phase == AnimatedRagdollPhase3D::Stopping &&
      m_phaseTime * m_stopRate >= clips.stop->durationSeconds)
    transition(AnimatedRagdollPhase3D::Idle);
  const AnimationClip3D *clip = clips.idle;
  if (m_telemetry.phase == AnimatedRagdollPhase3D::Running)
    clip = clips.run;
  if (m_telemetry.phase == AnimatedRagdollPhase3D::Stopping)
    clip = clips.stop;
  const float sourceRunSpeed =
      std::max(0.5f, clips.run->sourceRootDisplacementMeters.x /
                         clips.run->durationSeconds);
  const float runSpeed = m_settings.runSpeedMetersPerSecond > 0
                             ? m_settings.runSpeedMetersPerSecond
                             : sourceRunSpeed;
  m_speed = m_telemetry.phase == AnimatedRagdollPhase3D::Running
                ? std::clamp(runSpeed, 0.5f, 8.0f) * smooth(m_phaseTime / 0.35f)
                : 0;
  const float sampleTime =
      m_phaseTime * (m_telemetry.phase == AnimatedRagdollPhase3D::Stopping
                         ? m_stopRate
                     : m_telemetry.phase == AnimatedRagdollPhase3D::Running
                         ? runSpeed / sourceRunSpeed
                         : 1.0f);
  if (m_telemetry.phase == AnimatedRagdollPhase3D::Stopping) {
    const auto *root =
        findAnimationTrack3D(*clips.stop, profile.links.front().id);
    // Differentiate the source curve; it is a speed reference, never a
    // desired world position. No positional catch-up when displaced.
    m_speed =
        root && !m_telemetry.blocked
            ? std::max(
                  0.0f,
                  (sampleAnimationTrack3D(*root, sampleTime,
                                          clips.stop->durationSeconds, false)
                       .translationOffsetMeters.x -
                   sampleAnimationTrack3D(
                       *root, std::max(0.0f, sampleTime - dt * m_stopRate),
                       clips.stop->durationSeconds, false)
                       .translationOffsetMeters.x) /
                      dt)
            : 0;
  }
  const auto &rootState = state.links.front();
  const Vec3 up = rootState.orientation.rotate({0, 0, 1});
  const auto *rootTrack = findAnimationTrack3D(*clip, profile.links.front().id);
  const auto sourceRoot =
      rootTrack ? sampleAnimationTrack3D(*rootTrack, sampleTime,
                                         clip->durationSeconds, clip->loops)
                      .rotationDelta
                : Quaternion{};
  Vec3 forward = heading(rootState.orientation).rotate({1, 0, 0});
  const Vec3 commandForward = m_heading.rotate({1, 0, 0});
  const float bodyWeight = profile.totalMassKg * 9.81f;
  float supportLoad = 0;
  for (const auto &contact : state.contacts) {
    if (contact.linkIndex >= profile.links.size() ||
        !profile.links[contact.linkIndex].collider.contactSensor ||
        contact.normal.z < 0.8f ||
        !std::isfinite(contact.normalImpulseNewtonSeconds))
      continue;
    const float load = std::max(0.0f, contact.normalImpulseNewtonSeconds / dt);
    supportLoad += load;
  }
  const float authority =
      smooth((up.z - 0.65f) / 0.3f) *
      (1 - smooth((rootState.angularVelocity.length() - 1.5f) / 2.0f));
  if (m_telemetry.phase != AnimatedRagdollPhase3D::Falling &&
      (up.z < 0.65f || (rootState.position.z - groundHeight) <
                           profile.standingRootHeightMeters * 0.5f))
    transition(AnimatedRagdollPhase3D::Falling);
  m_telemetry.bodyUpDot = up.z;
  m_telemetry.supportLoadNewtons = supportLoad;
  m_telemetry.assistanceAuthority =
      m_telemetry.phase == AnimatedRagdollPhase3D::Falling ? 0 : authority;

  m_output.targetPose.linkPositions.resize(profile.links.size());
  m_output.targetPose.linkOrientations.resize(profile.links.size());
  auto &positions = m_output.targetPose.linkPositions;
  auto &orientations = m_output.targetPose.linkOrientations;
  const float blend =
      smooth(m_blendTime /
             (m_telemetry.phase == AnimatedRagdollPhase3D::Stopping ? 0.12f
              : m_telemetry.phase == AnimatedRagdollPhase3D::Idle   ? 1.2f
                                                                    : 0.35f));
  float jointError = 0;
  std::size_t dofs = 0;
  for (std::size_t i = 0; i < profile.links.size(); ++i) {
    const auto &link = profile.links[i];
    const auto *track = findAnimationTrack3D(*clip, link.id);
    const auto sample =
        track ? sampleAnimationTrack3D(*track, sampleTime,
                                       clip->durationSeconds, clip->loops)
              : AnimationTransformSample3D{};
    if (i == 0) {
      // Build directly in body space. Differentiating this pose must
      // never subtract world positions from different simulation ticks.
      positions[i] = {};
      orientations[i] = {};
      continue;
    }
    Vec3 coordinates;
    for (std::size_t a = 0; a < 3; ++a) {
      const auto &axis = link.inboundJoint.axes[a];
      const float old = component(m_coordinates[i], a);
      float wanted = component(m_blendFrom[i], a) * (1 - blend) +
                     component(sample.jointPositionRadians, a) * blend;
      if (m_telemetry.phase == AnimatedRagdollPhase3D::Falling)
        wanted = component(m_blendFrom[i], a);
      const float value = axis.enabled ? std::clamp(wanted, axis.minimumRadians,
                                                    axis.maximumRadians)
                                       : 0;
      setComponent(coordinates, a, value);
      if (!axis.enabled)
        continue;
      RagdollDriveTarget3D drive;
      drive.linkIndex = static_cast<std::uint32_t>(i);
      drive.axis = static_cast<RagdollAxis3D>(a);
      drive.positionRadians = value;
      drive.velocityRadiansPerSecond =
          std::clamp((value - old) / dt, -20.0f, 20.0f);
      drive.stiffnessScale = 1;
      drive.dampingScale = 1;
      drive.maximumTorqueScale = 1;
      if (m_telemetry.phase == AnimatedRagdollPhase3D::Falling) {
        drive.stiffnessScale = 0.1f;
        drive.dampingScale = 0.25f;
        drive.maximumTorqueScale = 0.12f;
        drive.feedforwardTorqueNewtonMeters = 0;
      }
      m_output.driveTargets.push_back(drive);
      if (i < state.joints.size())
        jointError +=
            std::pow(value - state.joints[i].positionRadians[a], 2.0f);
      ++dofs;
    }
    m_coordinates[i] = coordinates;
    const auto parent = static_cast<std::size_t>(link.parentIndex);
    const auto &parentLink = profile.links[parent];
    const auto frame = link.inboundJoint.frameModelOrientation;
    const auto parentFrame = parentLink.modelOrientation.conjugate() * frame;
    const auto childFrame = link.modelOrientation.conjugate() * frame;
    orientations[i] = (orientations[parent] * parentFrame *
                       rotation(coordinates) * childFrame.conjugate())
                          .normalized();
    const auto anchor = link.inboundJoint.anchorModelPosition;
    positions[i] =
        positions[parent] +
        orientations[parent].rotate(
            parentLink.modelOrientation.conjugate().rotate(
                anchor - parentLink.modelPosition)) -
        orientations[i].rotate(link.modelOrientation.conjugate().rotate(
            anchor - link.modelPosition));
  }
  m_output.gravityCompensationEnabled =
      m_telemetry.phase != AnimatedRagdollPhase3D::Falling;
  BodyRelativeRagdollReference3D reference;
  reference.positionsBody.resize(profile.links.size());
  reference.velocitiesBody.resize(profile.links.size());
  for (std::size_t i = 0; i < profile.links.size(); ++i) {
    reference.positionsBody[i] =
        positions[i] +
        orientations[i].rotate(profile.links[i].centerOfMassLocal) -
        profile.links.front().centerOfMassLocal;
    if (m_previousBodyPositionsValid &&
        m_previousBodyPositions.size() == profile.links.size())
      reference.velocitiesBody[i] = limited(
          (reference.positionsBody[i] - m_previousBodyPositions[i]) / dt,
          20.0f);
    // World pose is an output for inspection only, anchored to the actual
    // body on this tick. It is never fed back as a future world target.
    positions[i] =
        rootState.position + rootState.orientation.rotate(positions[i]);
    orientations[i] = (rootState.orientation * orientations[i]).normalized();
  }
  m_previousBodyPositions = reference.positionsBody;
  m_previousBodyPositionsValid = true;
  const float playbackRate =
      m_telemetry.phase == AnimatedRagdollPhase3D::Stopping ? m_stopRate
      : m_telemetry.phase == AnimatedRagdollPhase3D::Running
          ? runSpeed / sourceRunSpeed
          : 1.0f;
  const float verticalVelocity =
      rootTrack
          ? (sampleAnimationTrack3D(*rootTrack, sampleTime,
                                    clip->durationSeconds, clip->loops)
                 .translationOffsetMeters.z -
             sampleAnimationTrack3D(*rootTrack, sampleTime - dt * playbackRate,
                                    clip->durationSeconds, clip->loops)
                 .translationOffsetMeters.z) /
                dt
          : 0;
  reference.velocityHeading = {m_speed, 0, verticalVelocity};
  reference.upHeading =
      (heading(sourceRoot).conjugate() * sourceRoot).rotate({0, 0, 1});
  reference.poseAuthority = m_telemetry.assistanceAuthority;
  reference.balanceAuthority = m_telemetry.assistanceAuthority;
  // This first locomotion experiment gates NET propulsion by loaded feet.
  // Shape assistance is still XYZ in flight, with zero net force/moment.
  reference.movementAuthority = m_telemetry.assistanceAuthority *
                                smooth(supportLoad / (bodyWeight * 0.5f));
  auto assistance =
      computeBodyRelativeRagdollAssist3D(profile, state, reference, m_settings);
  m_telemetry.assistanceForceNewtons = assistance.forceSumNewtons;
  m_telemetry.assistanceTorqueNewtonMeters = assistance.torqueSumNewtonMeters;
  m_telemetry.assistanceNetForceWorld = assistance.netForceWorld;
  m_output.assistance = std::move(assistance.wrenches);
  m_telemetry.phaseSeconds = m_phaseTime;
  m_telemetry.runSecondsRemaining =
      m_telemetry.phase == AnimatedRagdollPhase3D::Running
          ? std::max(0.0f, m_requestedSeconds - m_phaseTime)
          : 0;
  m_telemetry.speedMetersPerSecond =
      dot(state.links.front().linearVelocity, forward);
  m_telemetry.travelledMeters =
      dot(state.links.front().position - m_commandOrigin, commandForward);
  m_telemetry.jointRmsDegrees =
      std::sqrt(jointError /
                static_cast<float>(std::max(std::size_t{1}, dofs))) *
      180 / Pi;
}
} // namespace MatterEngine
