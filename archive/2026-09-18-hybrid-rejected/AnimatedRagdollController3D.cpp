#include "Engine/Control/AnimatedRagdollController3D.hpp"
#include "Engine/Control/RagdollPoseMotor3D.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <string_view>

namespace MatterEngine {
namespace {
constexpr float Pi = 3.14159265358979323846f;
using Phase = AnimatedRagdollPhase3D;
Vec3 limited(Vec3 v, float maximum) {
  const float n = v.length();
  return n > maximum ? v * (maximum / n) : v;
}
float smooth(float t) {
  t = std::clamp(t, 0.0f, 1.0f);
  return t * t * (3 - 2 * t);
}
float smoothRange(float value, float begin, float end) {
  if (!std::isfinite(value) || !std::isfinite(begin) || !std::isfinite(end) ||
      end <= begin)
    return value >= end ? 1.0f : 0.0f;
  return smooth((value - begin) / (end - begin));
}
float lerp(float a, float b, float t) {
  return a + (b - a) * std::clamp(t, 0.0f, 1.0f);
}
float component(Vec3 v, std::size_t a) {
  return a == 0 ? v.x : a == 1 ? v.y : v.z;
}
void setComponent(Vec3 &v, std::size_t a, float f) {
  if (a == 0)
    v.x = f;
  else if (a == 1)
    v.y = f;
  else
    v.z = f;
}
Quaternion heading(Quaternion q) {
  auto forward = q.rotate({1, 0, 0});
  if (forward.x * forward.x + forward.y * forward.y < 0.0001f) {
    const auto left = q.rotate({0, 1, 0});
    forward = {left.y, -left.x, 0};
  }
  return Quaternion::fromAxisAngle({0, 0, 1}, std::atan2(forward.y, forward.x));
}
bool recovering(Phase p) {
  return p == Phase::GettingUpFront || p == Phase::GettingUpBack;
}
bool moving(Phase p) { return p == Phase::Walking || p == Phase::Running; }
bool finite(Vec3 v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
bool finite(Quaternion q) {
  return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
         std::isfinite(q.w) &&
         q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 0.0001f;
}
bool containsInsensitive(std::string_view text, std::string_view needle) {
  if (needle.empty() || needle.size() > text.size())
    return false;
  for (std::size_t offset = 0; offset + needle.size() <= text.size(); ++offset) {
    bool equal = true;
    for (std::size_t i = 0; i < needle.size(); ++i) {
      const auto a = static_cast<unsigned char>(text[offset + i]);
      const auto b = static_cast<unsigned char>(needle[i]);
      if (std::tolower(a) != std::tolower(b)) {
        equal = false;
        break;
      }
    }
    if (equal)
      return true;
  }
  return false;
}

Vec3 anatomicalAxisWorld(const RagdollLinkDefinition3D &definition,
                         const PhysicsBodyState3D &state, Vec3 modelAxis) {
  const Vec3 localAxis = definition.modelOrientation.conjugate().rotate(modelAxis).normalized();
  return state.orientation.rotate(localAxis).normalized();
}

bool isFootLink(const RagdollLinkDefinition3D &definition) {
  return containsInsensitive(definition.id, "foot") ||
         containsInsensitive(definition.id, "ankle");
}

float colliderLowestWorldZ(const RagdollLinkDefinition3D &definition,
                           const PhysicsBodyState3D &state) {
  const auto &collider = definition.collider;
  const Quaternion colliderWorldOrientation =
      (state.orientation * collider.localOrientation).normalized();
  const Vec3 centerWorld =
      state.position + state.orientation.rotate(collider.localPosition);

  float verticalExtent = 0.0f;
  if (collider.shape == RagdollColliderShape3D::Box) {
    const Vec3 x = colliderWorldOrientation.rotate({1, 0, 0});
    const Vec3 y = colliderWorldOrientation.rotate({0, 1, 0});
    const Vec3 z = colliderWorldOrientation.rotate({0, 0, 1});
    verticalExtent = std::abs(x.z) * collider.boxHalfExtents.x +
                     std::abs(y.z) * collider.boxHalfExtents.y +
                     std::abs(z.z) * collider.boxHalfExtents.z;
  } else {
    // PhysX capsules are longitudinal along local X. The hemispherical radius
    // contributes in every direction; only the cylindrical half-height must be
    // projected onto world Z.
    const float radius = std::max(0.0f, collider.radiusMeters);
    const float halfHeight =
        std::max(0.0f, collider.lengthMeters * 0.5f - radius);
    const Vec3 axis = colliderWorldOrientation.rotate({1, 0, 0});
    verticalExtent = radius + halfHeight * std::abs(axis.z);
  }
  return centerWorld.z - verticalExtent;
}
std::size_t torsoIndex(const RagdollProfile3D &profile) {
  for (std::size_t i = 0; i < profile.links.size(); ++i)
    if (containsInsensitive(profile.links[i].id, "chest") ||
        containsInsensitive(profile.links[i].id, "torso"))
      return i;
  for (std::size_t i = 0; i < profile.links.size(); ++i)
    if (containsInsensitive(profile.links[i].id, "spine"))
      return i;
  return 0;
}
bool validClip(const AnimationClip3D *c, const RagdollProfile3D &p) {
  return c && c->targetRigId == p.id && std::isfinite(c->durationSeconds) &&
         c->durationSeconds > 0;
}
} // namespace

bool AnimatedRagdollClips3D::compatible(const RagdollProfile3D &profile) const {
  if (!idle || !run || !stop || !idle->loops || !run->loops || stop->loops)
    return false;
  for (const auto *c : {idle, run, stop, walkForward, walkBackward, walkLeft,
                        walkRight, runBackward, standUpFront, standUpBack})
    if (c && (!validClip(c, profile) ||
              !validateAnimationClipForRagdoll3D(*c, profile).empty()))
      return false;
  return true;
}
void AnimatedRagdollController3D::reset(const RagdollProfile3D &profile,
                                        const RagdollState3D &state, float) {
  m_initialized = state.links.size() == profile.links.size() &&
                  state.joints.size() == profile.links.size() &&
                  !state.links.empty();
  m_output = {};
  m_telemetry = {};
  m_phaseTime = m_blendTime = m_speed = 0;
  m_stillSeconds = m_fallSeconds = m_recoveryClock = m_gaitClock = 0;
  m_unsupportedRecoverySeconds = 0;
  m_recoveryStableSeconds = m_recoveryClipFinishedSeconds = 0;
  m_recoveryRetryCooldownSeconds = m_recoverySupportAuthority = 0;
  m_recoveryBestHeightMeters = 0;
  m_recoveryBestUpDot = -1;
  m_recoveryFailures = 0;
  m_gaitSource = nullptr;
  m_wasManipulated = m_previousBodyPositionsValid = m_stoppingWalk = false;
  m_footwork.reset(profile);
  m_hybridAssist.reset();
  m_directionBody = {1, 0, 0};
  if (!m_initialized)
    return;
  m_commandOrigin = state.links.front().position;
  m_heading = heading(state.links.front().orientation);
  m_coordinates.assign(profile.links.size(), {});
  m_coordinateVelocities.assign(profile.links.size(), {});
  for (std::size_t i = 1; i < state.joints.size(); ++i)
    m_coordinates[i] = {state.joints[i].positionRadians[0],
                        state.joints[i].positionRadians[1],
                        state.joints[i].positionRadians[2]};
  m_blendFrom = m_coordinates;
}
void AnimatedRagdollController3D::transition(Phase phase) {
  m_telemetry.phase = phase;
  m_phaseTime = m_blendTime = 0;
  m_blendFrom = m_coordinates;
  m_previousBodyPositionsValid = false;
}
bool AnimatedRagdollController3D::requestRun(const RagdollState3D &state,
                                             float seconds) {
  if (!m_initialized || state.links.empty() ||
      m_telemetry.phase != Phase::Idle || m_telemetry.manipulated ||
      !std::isfinite(seconds) || seconds <= 0)
    return false;
  m_requestedSeconds = std::clamp(seconds, 0.1f, 30.0f);
  m_heading = heading(state.links.front().orientation);
  m_commandOrigin = state.links.front().position;
  m_directionBody = {1, 0, 0};
  m_speed = m_gaitClock = 0;
  m_telemetry.blocked = false;
  m_stoppingWalk = false;
  transition(Phase::Running);
  return true;
}
bool AnimatedRagdollController3D::requestWalk(const RagdollState3D &state,
                                              float seconds, Vec3 direction) {
  if (!finite(direction))
    return false;
  direction.z = 0;
  if (direction.length() < 0.001f)
    return false;
  if (!requestRun(state, seconds))
    return false;
  m_directionBody = direction.normalized();
  transition(Phase::Walking);
  return true;
}
void AnimatedRagdollController3D::update(const RagdollProfile3D &profile,
                                         const AnimatedRagdollClips3D &clips,
                                         const RagdollState3D &state, float dt,
                                         float groundHeight, bool manipulated,
                                         bool obstacleAhead,
                                         std::uint32_t grabbedLink) {
  m_output.driveTargets.clear();
  m_output.assistance.clear();
  m_output.gravityCompensationEnabled = false;
  m_output.targetPose = {};
  m_telemetry.assistanceForceNewtons =
      m_telemetry.assistanceTorqueNewtonMeters = 0;
  m_telemetry.assistanceNetForceWorld = {};
  m_telemetry.assistanceAuthority = 0;
  m_telemetry.recoveryLiftForceNewtons = 0;
  m_telemetry.balanceUrgency = 0;
  m_telemetry.internalBalanceTorqueNewtonMeters = 0;
  m_telemetry.residualSupportAuthority = 0;
  m_telemetry.standingPostureAuthority = 0;
  m_telemetry.bodyTiltRadians = 0;
  m_telemetry.tiltAngularSpeedRadiansPerSecond = 0;
  if (!std::isfinite(dt) || dt <= 0 || dt > 0.05f ||
      !std::isfinite(groundHeight) || state.links.empty() ||
      state.links.size() != profile.links.size() ||
      state.joints.size() != profile.links.size() ||
      !validClip(clips.idle, profile))
    return;
  for (const auto &link : state.links)
    if (!finite(link.position) || !finite(link.orientation) ||
        !finite(link.linearVelocity) || !finite(link.angularVelocity))
      return;
  for (const auto &joint : state.joints)
    for (std::size_t a = 0; a < 3; ++a)
      if (!std::isfinite(joint.positionRadians[a]) ||
          !std::isfinite(joint.velocityRadiansPerSecond[a]))
        return;
  if (!m_initialized)
    reset(profile, state, groundHeight);
  m_telemetry.manipulated = manipulated;
  const auto &root = state.links.front();
  const auto facing = heading(root.orientation);
  const float weight = profile.totalMassKg * 9.81f;
  const float up = anatomicalAxisWorld(profile.links.front(), root, {0, 0, 1}).z;
  const float height = root.position.z - groundHeight;
  m_phaseTime += dt;
  m_blendTime += dt;
  // Keep internal impedance during a grab; stop voluntary propulsion.
  // Release blends from measured joints at the new location.
  if (manipulated != m_wasManipulated) {
    for (std::size_t i = 1; i < state.joints.size(); ++i)
      m_coordinates[i] = {state.joints[i].positionRadians[0],
                          state.joints[i].positionRadians[1],
                          state.joints[i].positionRadians[2]};
    m_coordinateVelocities.assign(profile.links.size(), {});
    transition(Phase::Idle);
    m_speed = 0;
    m_footwork.reset(profile);
  m_hybridAssist.reset();
    m_telemetry.recoveryAttempts = 0;
    m_previousBodyPositionsValid = false;
    m_wasManipulated = manipulated;
  }
  if (moving(m_telemetry.phase) &&
      (m_phaseTime >= m_requestedSeconds || obstacleAhead)) {
    m_stoppingWalk = m_telemetry.phase == Phase::Walking;
    m_telemetry.blocked = obstacleAhead;
    transition(Phase::Stopping);
  }
  if (m_telemetry.phase == Phase::Stopping && m_phaseTime > 1.0f)
    transition(Phase::Idle);
  const float sourceRunSpeed =
      validClip(clips.run, profile)
          ? clips.run->sourceRootDisplacementMeters.length() /
                clips.run->durationSeconds
          : 2.6f;
  const float commandSpeed = m_telemetry.phase == Phase::Walking
                                 ? m_settings.walkSpeedMetersPerSecond
                             : m_telemetry.phase == Phase::Running
                                 ? (m_settings.runSpeedMetersPerSecond > 0
                                        ? m_settings.runSpeedMetersPerSecond
                                        : sourceRunSpeed)
                                 : 0;
  // Desired velocity must react like an intentional human command, not like a
  // tired root-speed filter. The feet still have to create this velocity
  // physically; this only changes how fast the controller asks for it.
  const float speedError = std::clamp(commandSpeed, 0.0f, 6.0f) - m_speed;
  const float acceleration = commandSpeed > m_speed
      ? (m_telemetry.phase == Phase::Running
             ? std::max(0.5f, m_settings.runAccelerationMetersPerSecondSquared)
             : std::max(0.5f, m_settings.walkAccelerationMetersPerSecondSquared))
      : std::max(0.5f, m_settings.stoppingDecelerationMetersPerSecondSquared);
  m_speed += std::clamp(speedError, -acceleration * dt, acceleration * dt);
  if (manipulated)
    m_speed = 0.0f;

  ContactFootworkIntent3D footworkIntent;
  footworkIntent.desiredVelocityHeading = m_directionBody * m_speed;
  footworkIntent.locomotionEnabled = moving(m_telemetry.phase) ||
                                     m_telemetry.phase == Phase::Stopping;
  footworkIntent.balanceRecoveryEnabled = true;
  // Recovery stepping is allowed even with zero player input and remains alive
  // in early Falling. ContactFootwork itself refuses to step once the body is
  // actually down/unsupported.
  footworkIntent.allowSteps = m_settings.footworkEnabled &&
                              m_settings.steppingEnabled && !manipulated &&
                              !recovering(m_telemetry.phase) &&
                              m_telemetry.phase != Phase::Fallen;
  m_footwork.update(profile, state, dt, footworkIntent);
  const auto &feet = m_footwork.output();
  m_telemetry.supportFootCount = feet.supportCount;
  m_telemetry.footworkPhase = feet.phase;
  m_telemetry.swingClearanceMeters = feet.swingClearanceMeters;
  m_telemetry.desiredSwingClearanceMeters = feet.desiredSwingClearanceMeters;
  m_telemetry.waitingForClearance = feet.waitingForClearance;
  m_telemetry.swingFoot = feet.swingFoot;
  m_telemetry.centerOfMassHeading = feet.centerOfMassHeading;
  m_telemetry.supportCenterHeading = feet.supportCenterHeading;
  m_telemetry.captureErrorMeters = feet.supportErrorHeading.length();
  m_telemetry.footstepReason = feet.stepReason;
  m_telemetry.footworkStepUrgency = feet.stepUrgency;
  m_telemetry.plannedSwingDurationSeconds = feet.plannedSwingDurationSeconds;
  m_telemetry.plannedStepDirectionHeading = feet.plannedStepDirectionHeading;
  m_telemetry.supportLoadNewtons =
      feet.feet[0].normalForceNewtons + feet.feet[1].normalForceNewtons;
  // ---------------------------------------------------------------------
  // CONTATOS, QUEDA E ESTADO DE RECUPERACAO
  // ---------------------------------------------------------------------
  // Separamos carga nos pes da carga em joelhos/maos/torso. Para concluir um
  // get-up nao basta "ter algum contato": queremos o corpo alto, orientado e
  // sustentado predominantemente pelos pes. Durante a subida, entretanto,
  // contato em qualquer parte do corpo e legitimo e autoriza a descarga de
  // peso da forca auxiliar.
  float externalLoad = 0.0f;
  float footLoad = 0.0f;
  float nonFootLoad = 0.0f;
  bool hasGroundContact = false;
  for (const auto &c : state.contacts) {
    if (c.linkIndex >= profile.links.size() || c.normal.z <= 0.35f)
      continue;

    // For GET-UP, existence of a plausible upward-facing contact matters more
    // than the solver impulse of this exact frame. A hand/knee/chest may be in
    // valid support while its normal impulse briefly approaches zero during a
    // weight transfer. Requiring a large measured impulse made the V6 lift
    // disappear precisely while the character was trying to rise.
    hasGroundContact = true;

    float load = 0.0f;
    if (std::isfinite(c.normalImpulseNewtonSeconds) &&
        c.normalImpulseNewtonSeconds > 0.0f)
      load = std::max(0.0f, c.normalImpulseNewtonSeconds / dt);

    externalLoad += load;
    if (isFootLink(profile.links[c.linkIndex]))
      footLoad += load;
    else
      nonFootLoad += load;
  }
  // Contact telemetry is opt-in per link (historically mostly feet). A get-up
  // starts while hands/knees/torso may be the only support, so relying only on
  // state.contacts can falsely report "airborne". We therefore also test the
  // actual ragdoll collider geometry against the ground plane supplied to this
  // controller. This is a support PRESENCE test only; it never creates a target
  // position or pushes the body toward groundHeight.
  float minimumGroundClearance = std::numeric_limits<float>::infinity();
  for (std::size_t i = 0; i < profile.links.size(); ++i) {
    const float lowest = colliderLowestWorldZ(profile.links[i], state.links[i]);
    if (std::isfinite(lowest))
      minimumGroundClearance =
          std::min(minimumGroundClearance, lowest - groundHeight);
  }
  const bool geometricGroundSupport =
      std::isfinite(minimumGroundClearance) &&
      minimumGroundClearance <=
          std::max(0.005f, m_settings.recoveryGroundClearanceMeters);

  const bool grounded = geometricGroundSupport || hasGroundContact ||
                        externalLoad > weight * 0.025f;
  const float measuredFootLoad = std::max(footLoad, m_telemetry.supportLoadNewtons);
  const bool feetBearing = feet.supportCount > 0 || measuredFootLoad > weight * 0.035f;

  // GET-UP SUPPORT CONTRACT:
  // - any genuine ground contact may react the vertical lift;
  // - measured load increases confidence, but does not have to be large;
  // - absolutely no contact means zero authority immediately (no airborne magic).
  const float measuredBodySupport = smoothRange(
      externalLoad / std::max(1.0f, weight), 0.005f, 0.30f);
  const bool getUpSupportPresent = hasGroundContact || geometricGroundSupport;
  const float rawBodySupportAuthority = getUpSupportPresent
      ? std::max(0.78f, measuredBodySupport)
      : 0.0f;
  const float standingSupportAuthority = feetBearing
      ? smoothRange(measuredFootLoad / std::max(1.0f, weight), 0.025f, 0.70f)
      : 0.0f;
  m_telemetry.footSupportLoadNewtons = measuredFootLoad;
  m_telemetry.nonFootSupportLoadNewtons = nonFootLoad;
  m_telemetry.residualSupportAuthority = standingSupportAuthority;
  m_telemetry.recoveryMinimumGroundClearanceMeters =
      std::isfinite(minimumGroundClearance) ? minimumGroundClearance : 999.0f;
  m_telemetry.recoveryGeometricSupport = geometricGroundSupport;

  // ---------------------------------------------------------------------
  // STANDING BALANCE OBSERVATION
  // ---------------------------------------------------------------------
  // Instability drives the procedural step timing and reference responsiveness.
  // Distributed hybrid assistance supplies posture and COM velocity control.
  const Vec3 worldUp{0, 0, 1};
  const Vec3 rootUpWorld =
      anatomicalAxisWorld(profile.links.front(), root, worldUp);
  const std::size_t torso = torsoIndex(profile);
  const Vec3 torsoUpWorld =
      anatomicalAxisWorld(profile.links[torso], state.links[torso], worldUp);
  Vec3 coreUpWorld = rootUpWorld * 0.45f + torsoUpWorld * 0.55f;
  if (coreUpWorld.lengthSquared() < 1.0e-6f)
    coreUpWorld = rootUpWorld;
  coreUpWorld = coreUpWorld.normalized();

  const float bodyTilt = std::acos(
      std::clamp(dot(coreUpWorld, worldUp), -1.0f, 1.0f));
  const Vec3 coreOmegaWorld =
      root.angularVelocity * 0.45f + state.links[torso].angularVelocity * 0.55f;
  const Vec3 tiltOmegaWorld =
      coreOmegaWorld - worldUp * dot(coreOmegaWorld, worldUp);
  const float tiltAngularSpeed = tiltOmegaWorld.length();
  const float captureError = std::sqrt(
      feet.supportErrorHeading.x * feet.supportErrorHeading.x +
      feet.supportErrorHeading.y * feet.supportErrorHeading.y);

  const float tiltUrgency = smoothRange(
      bodyTilt, 0.04f,
      std::max(0.08f, m_settings.standingFullAuthorityTiltRadians));
  const float angularUrgency = smoothRange(
      tiltAngularSpeed, 0.18f,
      std::max(0.35f, m_settings.standingFullAuthorityAngularSpeed));
  const float captureUrgency = feet.supportCount > 0
      ? smoothRange(captureError, 0.015f,
            std::max(0.04f,
                     m_settings.standingFullAuthorityCaptureErrorMeters))
      : 0.0f;
  // ContactFootwork already combines capture point, tilt and angular motion
  // into a stepping urgency. Reuse it here so muscles, posture control and the
  // feet all escalate together instead of reacting as disconnected systems.
  const float balanceUrgency =
      std::max({tiltUrgency, angularUrgency, captureUrgency, feet.stepUrgency});

  // External posture is RESIDUAL. Stable standing has zero residual authority;
  // ankle/hip/spine joints remain responsible for ordinary balance.
  const float standingResidualAuthority =
      std::clamp(m_settings.standingResidualMaximumAuthority, 0.0f, 1.0f) *
      smoothRange(balanceUrgency,
                  std::clamp(m_settings.standingResidualActivationUrgency,
                             0.0f, 0.95f),
                  1.0f) *
      standingSupportAuthority;

  m_telemetry.balanceUrgency = balanceUrgency;
  m_telemetry.standingPostureAuthority = standingResidualAuthority;
  m_telemetry.bodyTiltRadians = bodyTilt;
  m_telemetry.tiltAngularSpeedRadiansPerSecond = tiltAngularSpeed;

  const bool geometricFall = up < 0.42f ||
      height < profile.standingRootHeightMeters * 0.50f;
  const bool fallen = geometricFall || feet.likelyFallen;
  m_fallSeconds = fallen ? m_fallSeconds + dt : 0.0f;
  m_recoveryRetryCooldownSeconds =
      std::max(0.0f, m_recoveryRetryCooldownSeconds - dt);

  if (!manipulated && !recovering(m_telemetry.phase) &&
      m_telemetry.phase != Phase::Fallen &&
      m_telemetry.phase != Phase::Falling && m_fallSeconds > 0.12f) {
    transition(Phase::Falling);
    m_stillSeconds = 0.0f;
    m_speed = 0.0f;
  }

  if (m_telemetry.phase == Phase::Falling) {
    const bool recoveredStanding = up > 0.82f &&
        height > profile.standingRootHeightMeters * 0.74f && feetBearing;
    const bool physicallyDown = fallen &&
        (up < m_settings.fallingMinimumRecoveryUpDot ||
         height < profile.standingRootHeightMeters *
                      m_settings.fallingMinimumRecoveryHeightFraction ||
         feet.likelyFallen);
    if (recoveredStanding) {
      transition(Phase::Idle);
      m_telemetry.recoveryAttempts = 0;
    } else if (grounded && physicallyDown &&
               m_phaseTime > m_settings.fallingToFallenDelaySeconds) {
      // So relaxamos quando a queda realmente foi perdida. Enquanto ainda ha
      // altura/orientacao recuperavel, Falling continua com musculos e Posture
      // Assist fortes tentando voltar para cima.
      transition(Phase::Fallen);
      m_stillSeconds = 0.0f;
    }
  }

  if (m_telemetry.phase == Phase::Fallen) {
    m_speed = 0.0f;
    const bool calm = grounded && root.linearVelocity.length() < 1.15f &&
                      root.angularVelocity.length() < 2.6f;
    m_stillSeconds = calm ? m_stillSeconds + dt : 0.0f;

    const bool alreadyStanding = up > m_settings.recoverySuccessUpDot &&
        height > profile.standingRootHeightMeters *
                     m_settings.recoverySuccessHeightFraction &&
        feetBearing;
    if (alreadyStanding) {
      transition(Phase::Idle);
      m_telemetry.recoveryAttempts = 0;
      m_recoveryFailures = 0;
      m_recoveryRetryCooldownSeconds = 0.0f;
    } else if (m_settings.automaticRecoveryEnabled && !manipulated &&
               m_recoveryRetryCooldownSeconds <= 0.0f &&
               m_stillSeconds > m_settings.fallenSettleSeconds) {
      const std::size_t torso = torsoIndex(profile);
      const float frontUp = anatomicalAxisWorld(
          profile.links[torso], state.links[torso], {1, 0, 0}).z;

      // Mesmo quando o corpo esta quase de lado, nao ficamos eternamente
      // esperando uma classificacao perfeita. Escolhemos a opcao fisicamente
      // mais proxima e, em poses ambiguas, alternamos entre as duas tentativas.
      const AnimationClip3D *recovery = nullptr;
      bool useFront = frontUp < 0.0f;
      if (std::abs(frontUp) < 0.15f)
        useFront = (m_telemetry.recoveryAttempts % 2u) == 0u;
      recovery = useFront ? clips.standUpFront : clips.standUpBack;
      if (!validClip(recovery, profile)) {
        recovery = useFront ? clips.standUpBack : clips.standUpFront;
        useFront = recovery == clips.standUpFront;
      }

      if (validClip(recovery, profile)) {
        if (m_telemetry.recoveryAttempts <
            std::numeric_limits<unsigned>::max())
          ++m_telemetry.recoveryAttempts;

        m_recoveryClock = 0.0f;
        m_unsupportedRecoverySeconds = 0.0f;
        m_recoveryStableSeconds = 0.0f;
        m_recoveryClipFinishedSeconds = 0.0f;
        m_recoverySupportAuthority = 0.0f;
        m_recoveryBestHeightMeters = height;
        m_recoveryBestUpDot = up;
        m_stillSeconds = 0.0f;
        m_fallSeconds = 0.0f;
        // Get-up has no root/world heading target. The animation drives only
        // articulated joint coordinates. Global pelvis orientation is an outcome
        // of contacts, joint torques and physics.


        transition(useFront ? Phase::GettingUpFront : Phase::GettingUpBack);
      }
    }
  }
  const AnimationClip3D *clip = clips.idle;
  if (m_telemetry.phase == Phase::Running && validClip(clips.run, profile))
    clip = clips.run;
  if (m_telemetry.phase == Phase::Walking &&
      validClip(clips.walkForward, profile)) {
    // Cardinal source poses feed the bounded procedural foot targets.
    // Diagonal stride direction is resolved by the planner, not by
    // mixing mismatched source timestamps from unrelated gait cycles.
    if (std::abs(m_directionBody.y) > std::abs(m_directionBody.x))
      clip = m_directionBody.y > 0 ? clips.walkLeft : clips.walkRight;
    else
      clip = m_directionBody.x >= 0 ? clips.walkForward : clips.walkBackward;
    if (!validClip(clip, profile))
      clip = clips.walkForward;
  }
  if (m_telemetry.phase == Phase::Stopping && !m_stoppingWalk &&
      validClip(clips.stop, profile))
    clip = clips.stop;
  if (m_telemetry.phase == Phase::GettingUpFront)
    clip = clips.standUpFront;
  if (m_telemetry.phase == Phase::GettingUpBack)
    clip = clips.standUpBack;
  if (!validClip(clip, profile)) {
    transition(Phase::Fallen);
    clip = clips.idle;
  }
  bool recovery = recovering(m_telemetry.phase);
  if (recovery) {
    // This smoothed value is state/playback telemetry only. The actual lift
    // uses rawBodySupportAuthority below, so it still becomes exactly zero in
    // the first fully airborne frame.
    const float supportDelta = rawBodySupportAuthority - m_recoverySupportAuthority;
    const float supportRate = supportDelta >= 0.0f ? 8.0f : 2.5f;
    m_recoverySupportAuthority += std::clamp(
        supportDelta, -supportRate * dt, supportRate * dt);
    m_recoverySupportAuthority =
        std::clamp(m_recoverySupportAuthority, 0.0f, 1.0f);

    // Nunca congelamos o clipe simplesmente porque um contato sumiu por um
    // frame. Tracking ruim reduz moderadamente o playback; perda prolongada
    // de suporte e tratada como falha explicita mais abaixo.
    const float trackingRate = std::clamp(
        1.0f - m_telemetry.jointRmsDegrees / 110.0f, 0.55f, 1.0f);
    const float supportPlaybackRate = 0.65f + 0.35f * m_recoverySupportAuthority;
    if (m_recoveryClock < clip->durationSeconds) {
      m_recoveryClock = std::min(
          clip->durationSeconds,
          m_recoveryClock + dt * trackingRate * supportPlaybackRate);
    }

    const float progress = clip->durationSeconds > 0.0f
        ? std::clamp(m_recoveryClock / clip->durationSeconds, 0.0f, 1.0f)
        : 1.0f;
    m_recoveryBestHeightMeters = std::max(m_recoveryBestHeightMeters, height);
    m_recoveryBestUpDot = std::max(m_recoveryBestUpDot, up);

    const bool standingCandidate =
        up > m_settings.recoverySuccessUpDot &&
        height > profile.standingRootHeightMeters *
                     m_settings.recoverySuccessHeightFraction &&
        feetBearing &&
        nonFootLoad < weight * 0.30f &&
        root.linearVelocity.length() <
            m_settings.recoverySuccessMaximumLinearSpeed &&
        root.angularVelocity.length() <
            m_settings.recoverySuccessMaximumAngularSpeed;
    m_recoveryStableSeconds = standingCandidate
        ? m_recoveryStableSeconds + dt
        : 0.0f;

    if (m_recoveryClock >= clip->durationSeconds - 0.0001f)
      m_recoveryClipFinishedSeconds += dt;
    else
      m_recoveryClipFinishedSeconds = 0.0f;

    m_unsupportedRecoverySeconds = m_recoverySupportAuthority < 0.10f
        ? m_unsupportedRecoverySeconds + dt
        : 0.0f;

    auto failRecovery = [&] {
      if (m_recoveryFailures < std::numeric_limits<unsigned>::max())
        ++m_recoveryFailures;
      m_recoveryRetryCooldownSeconds =
          std::max(0.0f, m_settings.recoveryRetryDelaySeconds);
      m_recoveryStableSeconds = 0.0f;
      m_recoveryClipFinishedSeconds = 0.0f;
      m_recoverySupportAuthority = 0.0f;
      m_unsupportedRecoverySeconds = 0.0f;
      m_stillSeconds = 0.0f;
      m_fallSeconds = 0.0f;
      m_footwork.reset(profile);
  m_hybridAssist.reset();
      transition(Phase::Fallen);
    };

    if (m_recoveryStableSeconds >= m_settings.recoverySuccessHoldSeconds) {
      transition(Phase::Idle);
      m_recoverySupportAuthority = 0.0f;
      m_recoveryRetryCooldownSeconds = 0.0f;
      m_recoveryFailures = 0;
      m_telemetry.recoveryAttempts = 0;
      m_footwork.reset(profile);
  m_hybridAssist.reset();
    } else if (m_unsupportedRecoverySeconds >
                   m_settings.recoveryUnsupportedTimeoutSeconds ||
               m_recoveryClipFinishedSeconds >
                   m_settings.recoveryCompletionGraceSeconds ||
               m_phaseTime > clip->durationSeconds * 2.2f + 2.0f) {
      failRecovery();
    }

    m_telemetry.recoveryProgress = progress;
    m_telemetry.recoverySupportAuthority = m_recoverySupportAuthority;
    m_telemetry.recoveryStableSeconds = m_recoveryStableSeconds;
    m_telemetry.recoveryBestHeightMeters = m_recoveryBestHeightMeters;
    m_telemetry.recoveryBestUpDot = m_recoveryBestUpDot;
    m_telemetry.recoveryFailures = m_recoveryFailures;
    m_telemetry.recoveryStandingCandidate = standingCandidate;
  } else {
    m_telemetry.recoveryProgress = 0.0f;
    m_telemetry.recoverySupportAuthority = 0.0f;
    m_telemetry.recoveryStableSeconds = 0.0f;
    m_telemetry.recoveryBestHeightMeters = m_recoveryBestHeightMeters;
    m_telemetry.recoveryBestUpDot = m_recoveryBestUpDot;
    m_telemetry.recoveryFailures = m_recoveryFailures;
    m_telemetry.recoveryStandingCandidate = false;
  }

  if (recovery && !recovering(m_telemetry.phase)) {
    recovery = false;
    clip = clips.idle;
  }
  const float sourceSpeed =
      std::max(0.4f, clip->sourceRootDisplacementMeters.length() /
                         clip->durationSeconds);
  const float cadence = moving(m_telemetry.phase)
                            ? std::clamp(m_speed / sourceSpeed, 0.25f, 1.5f)
                            : 1.0f;
  if (feet.phase != ContactFootworkPhase3D::Touchdown ||
      !moving(m_telemetry.phase))
    m_gaitClock += dt * cadence;
  if (moving(m_telemetry.phase) && m_gaitSource != clip) {
    m_gaitSource = clip;
    std::array<float, 2> peakHeight{-1000, -1000};
    for (int sample = 0; sample < 48; ++sample) {
      const float time =
          clip->durationSeconds * static_cast<float>(sample) / 48;
      const auto pose =
          sampleRagdollAnimationPose3D(profile, clip, time, false);
      for (int foot = 0; foot < 2; ++foot) {
        const auto i = feet.feet[foot].linkIndex;
        if (i < pose.linkPositions.size() &&
            pose.linkPositions[i].z > peakHeight[foot]) {
          peakHeight[foot] = pose.linkPositions[i].z;
          m_swingPeakSeconds[foot] = time;
        }
      }
    }
  }
  const bool footInFlight = feet.phase == ContactFootworkPhase3D::Swing ||
                            feet.phase == ContactFootworkPhase3D::Touchdown;
  if (moving(m_telemetry.phase) && feet.swingFoot >= 0 && footInFlight) {
    const float progress = feet.phaseProgress;
    m_gaitClock =
        std::fmod(m_swingPeakSeconds[feet.swingFoot] +
                      (progress - 0.5f) * clip->durationSeconds * 0.5f +
                      clip->durationSeconds,
                  clip->durationSeconds);
  }
  const float sampleTime = recovery                    ? m_recoveryClock
                           : moving(m_telemetry.phase) ? m_gaitClock
                                                       : m_phaseTime;
  const float recoveryBlendSeconds = std::max(0.05f, m_settings.recoveryBlendSeconds);
  const float blend = smooth(m_blendTime / (recovery ? recoveryBlendSeconds : 0.65f));
  std::vector<Vec3> wanted(profile.links.size());
  for (std::size_t i = 1; i < profile.links.size(); ++i) {
    const auto *track = findAnimationTrack3D(*clip, profile.links[i].id);
    const auto sample =
        track ? sampleAnimationTrack3D(*track, sampleTime,
                                       clip->durationSeconds, clip->loops)
              : AnimationTransformSample3D{};
    wanted[i] =
        m_blendFrom[i] * (1 - blend) + sample.jointPositionRadians * blend;
    // Falling continua ativo: o idle vira uma referencia articular de retorno
    // enquanto Posture Assist tenta recuperar a base. Somente Fallen congela a
    // pose medida e relaxa para evitar que o boneco lute contra o chao.
    if (m_telemetry.phase == Phase::Fallen)
      wanted[i] = m_blendFrom[i];
  }

  // Ground-adapted feet and COM strategy only when upright. The IK outputs
  // legal joint angles; it cannot move the root or apply a position spring.
  // Keep procedural stepping alive deep into a recoverable lean. The previous
  // 0.55 up-dot gate disabled the feet exactly when a recovery step became most
  // valuable. ContactFootwork owns the final fallen/unsupported decision.
  if (m_settings.footworkEnabled && !manipulated && !recovery && up > 0.28f &&
      feet.valid && feet.supportCount > 0 && !feet.likelyFallen) {
    for (std::size_t footIndex = 0; footIndex < feet.feet.size(); ++footIndex) {
      const auto &foot = feet.feet[footIndex];
      if (foot.linkIndex >= profile.links.size())
        continue;
      Vec3 target = foot.targetPositionHeading;
      // The support leg's extension is relative to current support,
      // without a saved global height. Upright posture uses nearly
      // straight, not locked, knees.
      // A selected future swing foot is still a stance foot while
      // transferring weight. Only measured support authorizes liftoff.
      const bool stance =
          !footInFlight || static_cast<int>(footIndex) != feet.swingFoot;
      if (stance) {
        // Preserve the measured support location relative to the current pelvis.
        // Auxiliary balance handles the COM; the leg must not drag the support foot.
        target.z = feet.supportCenterHeading.z +
            (profile.links[foot.linkIndex].modelPosition.z - profile.links.front().modelPosition.z +
             profile.standingRootHeightMeters);
      }
      // Solve a coherent desired upright pelvis pose, not the measured
      // lean. Joint drives and current contacts must realize it; no
      // root transform is written, and no old world position is held.
      fitRagdollFootTarget3D(profile, wanted, foot.linkIndex, target,
                             profile.links[foot.linkIndex].modelOrientation,
                             (stance ? 1.0f : foot.targetWeight) * blend);
    }
  }
  float jointError = 0;
  std::size_t dofs = 0;

  const bool passiveFall = m_telemetry.phase == Phase::Fallen;
  const float muscleAuthority = std::isfinite(m_settings.muscleAuthority)
      ? std::clamp(m_settings.muscleAuthority, 0.0f, 1.0f) : 0.0f;
  const float muscle = 3.0f * muscleAuthority;
  m_telemetry.muscleTorqueScale = muscle;
  m_telemetry.internalBalanceTorqueNewtonMeters = 0;

  for (std::size_t i = 1; i < profile.links.size(); ++i) {
    const auto &link = profile.links[i];
    const float maximumSpeed = recovery
        ? std::clamp(m_settings.recoveryMaximumJointSpeedRadiansPerSecond, 1.0f, 8.0f)
        : std::clamp(m_settings.locomotionMaximumJointSpeedRadiansPerSecond *
                         (1.0f + 0.18f * balanceUrgency),
                     3.0f, 10.0f);

    for (std::size_t a = 0; a < 3; ++a) {
      const auto &axis = link.inboundJoint.axes[a];
      if (!axis.enabled) {
        setComponent(m_coordinates[i], a, 0);
        setComponent(m_coordinateVelocities[i], a, 0);
        continue;
      }
      const float old = component(m_coordinates[i], a);
      const float baseOmega = std::max(8.0f, m_settings.locomotionReferenceOmega);
      const float emergencyOmega = std::max(baseOmega, m_settings.emergencyReferenceOmega);
      const float omega = recovery
          ? std::clamp(m_settings.recoveryReferenceOmega, 6.0f, 24.0f)
          : lerp(baseOmega, emergencyOmega, balanceUrgency);
      const float previousSpeed = component(m_coordinateVelocities[i], a);
      const float desired = std::clamp(
          component(wanted[i], a), axis.minimumRadians, axis.maximumRadians);
      const float maximumReferenceAcceleration =
          recovery ? 90.0f : lerp(95.0f, 145.0f, balanceUrgency);
      const float targetAcceleration = std::clamp(
          omega * omega * (desired - old) - 2.0f * omega * previousSpeed,
          -maximumReferenceAcceleration, maximumReferenceAcceleration);
      const float targetSpeed = std::clamp(
          previousSpeed + targetAcceleration * dt, -maximumSpeed, maximumSpeed);
      const float value = std::clamp(
          old + targetSpeed * dt, axis.minimumRadians, axis.maximumRadians);
      setComponent(m_coordinates[i], a, value);
      setComponent(m_coordinateVelocities[i], a, (value - old) / dt);
    }

    const auto &measured = state.joints[i].positionRadians;
    const auto angularVelocity = ragdollJointTargetVelocity3D(
        m_coordinates[i], m_coordinateVelocities[i],
        {measured[0], measured[1], measured[2]});

    for (std::size_t a = 0; a < 3; ++a) {
      const auto &axis = link.inboundJoint.axes[a];
      if (!axis.enabled)
        continue;

      const float value = component(m_coordinates[i], a);
      RagdollDriveTarget3D drive;
      drive.linkIndex = static_cast<std::uint32_t>(i);
      drive.axis = static_cast<RagdollAxis3D>(a);
      drive.positionRadians = value;
      drive.velocityRadiansPerSecond = std::clamp(
          component(angularVelocity, a), -maximumSpeed, maximumSpeed);

      // Muscle authority is independent of auxiliary authority, including in flight.
      // At zero every motor channel and gravity feed-forward are really disabled.
      drive.stiffnessScale = 2.0f * muscleAuthority;
      drive.dampingScale = 2.0f * std::sqrt(muscleAuthority);
      drive.maximumTorqueScale = muscle;
      drive.feedforwardTorqueNewtonMeters = 0;
      if (manipulated && i == grabbedLink) {
        drive.stiffnessScale *= 0.25f;
        drive.maximumTorqueScale *= 0.25f;
      }

      m_output.driveTargets.push_back(drive);
      const float e = value - state.joints[i].positionRadians[a];
      jointError += e * e;
      ++dofs;
    }
  }
  buildRagdollJointPose3D(profile, m_coordinates, m_output.targetPose);
  std::vector<Vec3> referenceVelocities(profile.links.size());
  std::vector<Vec3> bodyPositions(profile.links.size());
  for (std::size_t i = 0; i < profile.links.size(); ++i) {
    bodyPositions[i] = m_output.targetPose.linkPositions[i] +
        m_output.targetPose.linkOrientations[i].rotate(profile.links[i].centerOfMassLocal);
    if (m_previousBodyPositionsValid && m_previousBodyPositions.size() == profile.links.size())
      referenceVelocities[i] = limited((bodyPositions[i] - m_previousBodyPositions[i]) / dt, 4.0f);
  }
  m_previousBodyPositions = std::move(bodyPositions);
  m_previousBodyPositionsValid = true;
  m_output.gravityCompensationEnabled = muscleAuthority > 0.0f && !passiveFall;
  const float requestedAux = m_settings.assistanceEnabled ? m_settings.auxiliaryAuthority : 0.0f;
  auto assistance = m_hybridAssist.update(profile, state, m_output.targetPose,
      referenceVelocities, feet, m_directionBody * m_speed, requestedAux, dt,
      manipulated || passiveFall || recovery);
  m_output.assistance = std::move(assistance.wrenches);
  m_telemetry.assistanceForceNewtons = assistance.forceSumNewtons;
  m_telemetry.assistanceTorqueNewtonMeters = assistance.torqueSumNewtonMeters;
  m_telemetry.assistanceNetForceWorld = assistance.netForceWorld;
  m_telemetry.assistanceAuthority = m_hybridAssist.authority();
  m_telemetry.standingPostureAuthority = m_hybridAssist.authority();
  m_telemetry.assistanceBudgetWeightFraction = 4.0f * m_hybridAssist.authority();
  m_telemetry.recoveryLiftForceNewtons = 0;
  for (std::size_t i = 0; i < profile.links.size(); ++i) {
    m_output.targetPose.linkPositions[i] = root.position + facing.rotate(m_output.targetPose.linkPositions[i]);
    m_output.targetPose.linkOrientations[i] = (facing * m_output.targetPose.linkOrientations[i]).normalized();
  }
  m_telemetry.bodyUpDot = up;
  m_telemetry.phaseSeconds = m_phaseTime;
  m_telemetry.movementSecondsRemaining =
      moving(m_telemetry.phase)
          ? std::max(0.0f, m_requestedSeconds - m_phaseTime)
          : 0;
  m_telemetry.runSecondsRemaining = m_telemetry.phase == Phase::Running
                                        ? m_telemetry.movementSecondsRemaining
                                        : 0;
  m_telemetry.speedMetersPerSecond =
      dot(root.linearVelocity, facing.rotate(m_directionBody));
  m_telemetry.travelledMeters =
      dot(root.position - m_commandOrigin, m_heading.rotate(m_directionBody));
  m_telemetry.jointRmsDegrees =
      std::sqrt(jointError /
                static_cast<float>(std::max(std::size_t{1}, dofs))) *
      180 / Pi;
}
} // namespace MatterEngine
