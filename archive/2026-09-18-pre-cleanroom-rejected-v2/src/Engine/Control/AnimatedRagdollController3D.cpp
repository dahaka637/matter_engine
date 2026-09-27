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

bool isThighLink(const RagdollLinkDefinition3D &definition) {
  return containsInsensitive(definition.id, "thigh") ||
         containsInsensitive(definition.id, "upperleg");
}

bool isSpineBalanceLink(const RagdollLinkDefinition3D &definition) {
  return containsInsensitive(definition.id, "spine") ||
         containsInsensitive(definition.id, "lumbar") ||
         containsInsensitive(definition.id, "chest") ||
         containsInsensitive(definition.id, "torso");
}

int sideIndex(const RagdollLinkDefinition3D &definition) {
  if (containsInsensitive(definition.id, "left"))
    return 0;
  if (containsInsensitive(definition.id, "right"))
    return 1;
  return -1;
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
  unsigned groundContactCount = 0;
  for (const auto &c : state.contacts) {
    if (c.linkIndex >= profile.links.size() || c.normal.z <= 0.35f)
      continue;

    // For GET-UP, existence of a plausible upward-facing contact matters more
    // than the solver impulse of this exact frame. A hand/knee/chest may be in
    // valid support while its normal impulse briefly approaches zero during a
    // weight transfer. Requiring a large measured impulse made the V6 lift
    // disappear precisely while the character was trying to rise.
    hasGroundContact = true;
    ++groundContactCount;

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
  // Instability is measured here, but the NORMAL corrective work will be done
  // by internal joint feed-forward torques farther below. The external residual
  // layer receives only a small authority derived from this urgency and only
  // when real support exists.
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
      transition(Phase::Fallen);
    };

    if (m_recoveryStableSeconds >= m_settings.recoverySuccessHoldSeconds) {
      transition(Phase::Idle);
      m_recoverySupportAuthority = 0.0f;
      m_recoveryRetryCooldownSeconds = 0.0f;
      m_recoveryFailures = 0;
      m_telemetry.recoveryAttempts = 0;
      m_footwork.reset(profile);
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
        const Vec3 neutral = profile.links[foot.linkIndex].modelPosition -
                             profile.links.front().modelPosition;
        const Vec3 error = feet.centerOfMassHeading - feet.supportCenterHeading;
        const Vec3 velocity = feet.centerOfMassVelocityHeading - m_directionBody * m_speed;
        const float reaction = 0.65f + 0.75f * balanceUrgency;
        target.z = neutral.z + 0.006f;
        target.x += std::clamp((error.x + velocity.x * 0.16f) * reaction, -0.16f, 0.16f);
        target.y += std::clamp((error.y + velocity.y * 0.16f) * reaction, -0.12f, 0.12f);
        if (feet.phase == ContactFootworkPhase3D::WeightShift)
          target -= feet.desiredComCorrectionHeading *
                    (0.50f + 0.35f * feet.stepUrgency);
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

  // ---------------------------------------------------------------------
  // INTERNAL BALANCE CONTROLLER
  // ---------------------------------------------------------------------
  // This is the normal standing controller. We compute the virtual corrective
  // torque we would like at the pelvis/torso, then realize it through joint
  // feed-forward on the support chain. That keeps the six floating-root DOFs
  // unactuated: the world can only rotate/translate the character through real
  // contacts, exactly as an active ragdoll should.
  const bool passiveFall = m_telemetry.phase == Phase::Fallen;
  const bool activeFalling = m_telemetry.phase == Phase::Falling;
  const bool recoveryNearStanding = recovery && feetBearing &&
      up > m_settings.recoveryResidualPostureStartUpDot &&
      height > profile.standingRootHeightMeters *
                   m_settings.recoveryResidualPostureStartHeightFraction;
  const bool internalBalanceActive = !passiveFall && !manipulated &&
      standingSupportAuthority > 0.03f && feet.supportCount > 0 &&
      (!recovery || recoveryNearStanding);

  Vec3 desiredBalanceTorqueWorld;
  if (internalBalanceActive) {
    const float balanceLever =
        std::max(0.35f, profile.standingRootHeightMeters);
    const Vec3 captureErrorWorld = facing.rotate(
        {feet.supportErrorHeading.x, feet.supportErrorHeading.y, 0});
    const Vec3 comVelocityErrorHeading =
        feet.centerOfMassVelocityHeading - m_directionBody * m_speed;
    const Vec3 comVelocityErrorWorld = facing.rotate(
        {comVelocityErrorHeading.x, comVelocityErrorHeading.y, 0});

    // Torso/hip strategy. The first term restores structural UP, the second
    // reacts to capture-point error, the third damps COM travel and the fourth
    // damps tilt angular velocity. All are torques to be realized internally.
    desiredBalanceTorqueWorld =
        cross(coreUpWorld, worldUp) *
            (weight * balanceLever *
             m_settings.standingInternalBalanceTiltGain *
             (0.65f + 0.35f * balanceUrgency)) -
        cross(worldUp, captureErrorWorld) *
            (weight * m_settings.standingInternalBalanceCaptureGain) -
        cross(worldUp, comVelocityErrorWorld) *
            (profile.totalMassKg * balanceLever *
             m_settings.standingInternalBalanceVelocityGain) -
        tiltOmegaWorld *
            (weight * balanceLever *
             m_settings.standingInternalBalanceAngularDamping);
    desiredBalanceTorqueWorld.z = 0.0f; // Never servo yaw.

    const float maximumBalanceTorque =
        weight * balanceLever *
        std::clamp(
            m_settings.standingInternalBalanceMaximumWeightLengthFraction,
            0.05f, 0.80f) *
        (0.65f + 0.35f * balanceUrgency);
    desiredBalanceTorqueWorld =
        limited(desiredBalanceTorqueWorld, maximumBalanceTorque);
  }
  m_telemetry.internalBalanceTorqueNewtonMeters =
      desiredBalanceTorqueWorld.length();

  // Convert measured support load into left/right shares. If contact force is
  // temporarily unavailable but ContactFootwork still reports a valid support,
  // fall back to an equal share instead of deleting balance for one frame.
  std::array<float, 2> supportShare{0.0f, 0.0f};
  float supportForceSum = 0.0f;
  for (int side = 0; side < 2; ++side) {
    if (feet.feet[side].supported) {
      supportShare[side] = std::max(0.0f, feet.feet[side].normalForceNewtons);
      supportForceSum += supportShare[side];
    }
  }
  if (feet.supportCount > 0) {
    if (supportForceSum > 1.0f) {
      supportShare[0] /= supportForceSum;
      supportShare[1] /= supportForceSum;
    } else {
      const float equal = 1.0f / static_cast<float>(feet.supportCount);
      for (int side = 0; side < 2; ++side)
        supportShare[side] = feet.feet[side].supported ? equal : 0.0f;
    }
  }

  std::size_t spineBalanceLinkCount = 0;
  for (std::size_t i = 1; i < profile.links.size(); ++i)
    if (isSpineBalanceLink(profile.links[i]))
      ++spineBalanceLinkCount;

  const float requestedShareSum = std::max(
      0.001f,
      std::max(0.0f, m_settings.standingHipBalanceShare) +
          std::max(0.0f, m_settings.standingAnkleBalanceShare) +
          std::max(0.0f, m_settings.standingSpineBalanceShare));
  const float hipGroupShare =
      std::max(0.0f, m_settings.standingHipBalanceShare) / requestedShareSum;
  const float ankleGroupShare =
      std::max(0.0f, m_settings.standingAnkleBalanceShare) / requestedShareSum;
  const float spineGroupShare =
      std::max(0.0f, m_settings.standingSpineBalanceShare) / requestedShareSum;

  const float muscleMultiplier = recovery
      ? m_settings.recoveryMuscleStrengthMultiplier
      : activeFalling ? m_settings.fallingMuscleStrengthMultiplier
                      : m_settings.standingMuscleStrengthMultiplier;
  const float reactiveMuscle = 1.0f +
      std::max(0.0f, m_settings.reactiveMuscleBoost) * balanceUrgency;
  const float muscle = std::clamp(
      m_settings.muscleStrength * muscleMultiplier * reactiveMuscle, 0.3f, 5.8f);
  m_telemetry.muscleTorqueScale = passiveFall ? 0.35f : muscle;

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

    // Determine what fraction of the virtual balance torque this joint should
    // realize. Spine acts directly on the torso; hip/ankle chains use the
    // opposite sign so their reaction is transmitted into the pelvis/body.
    float balanceShare = 0.0f;
    float balanceSign = 0.0f;
    if (internalBalanceActive) {
      if (isSpineBalanceLink(link) && spineBalanceLinkCount > 0) {
        balanceShare =
            spineGroupShare / static_cast<float>(spineBalanceLinkCount);
        balanceSign = 1.0f;
      } else if (isThighLink(link)) {
        const int side = sideIndex(link);
        if (side >= 0 && feet.feet[side].supported) {
          balanceShare = hipGroupShare * supportShare[side];
          balanceSign = -1.0f;
        }
      } else if (isFootLink(link)) {
        const int side = sideIndex(link);
        if (side >= 0 && feet.feet[side].supported) {
          balanceShare = ankleGroupShare * supportShare[side];
          balanceSign = -1.0f;
        }
      }
    }

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

      if (passiveFall) {
        drive.stiffnessScale = 0.22f;
        drive.dampingScale = 0.60f;
        drive.maximumTorqueScale = 0.35f;
      } else if (manipulated) {
        drive.stiffnessScale = 0.70f;
        drive.dampingScale = 1.10f;
        drive.maximumTorqueScale = std::min(1.0f, muscle);
      } else if (recovery) {
        drive.stiffnessScale = std::min(2.8f, 0.68f * muscle);
        drive.dampingScale =
            1.55f * std::sqrt(std::max(0.2f, muscle / 2.2f));
        drive.maximumTorqueScale = muscle;
      } else if (activeFalling) {
        drive.stiffnessScale = std::min(3.25f, 0.76f * muscle + 0.55f +
            m_settings.reactiveDriveStiffnessBoost * balanceUrgency);
        drive.dampingScale =
            (1.62f + m_settings.reactiveDriveDampingBoost * balanceUrgency) *
            std::sqrt(std::max(0.2f, muscle / 2.4f));
        drive.maximumTorqueScale = muscle;
      } else {
        drive.stiffnessScale = std::min(3.10f, 0.76f * muscle + 0.38f +
            m_settings.reactiveDriveStiffnessBoost * balanceUrgency);
        drive.dampingScale =
            (1.48f + m_settings.reactiveDriveDampingBoost * balanceUrgency) *
            std::sqrt(std::max(0.2f, muscle / 2.4f));
        drive.maximumTorqueScale = muscle;
      }

      if (manipulated && i == grabbedLink) {
        drive.stiffnessScale = 0.30f;
        drive.maximumTorqueScale = 0.50f;
      }

      if (internalBalanceActive && balanceShare > 0.0f) {
        const Quaternion jointFrameWorld =
            state.links[i].orientation * link.modelOrientation.conjugate() *
            link.inboundJoint.frameModelOrientation;
        const Vec3 basis = a == 0   ? Vec3{1, 0, 0}
                           : a == 1 ? Vec3{0, 1, 0}
                                    : Vec3{0, 0, 1};
        const Vec3 axisWorld = jointFrameWorld.rotate(basis);
        float feedForward = dot(
            axisWorld,
            desiredBalanceTorqueWorld * (balanceSign * balanceShare));

        // Keep feed-forward subordinate to the joint's real actuator budget.
        if (axis.maximumTorque > 0.0f) {
          const float limit = axis.maximumTorque *
              std::max(0.1f, drive.maximumTorqueScale) * 0.85f;
          feedForward = std::clamp(feedForward, -limit, limit);
        }
        drive.feedforwardTorqueNewtonMeters += feedForward;
      }

      m_output.driveTargets.push_back(drive);
      const float e = value - state.joints[i].positionRadians[a];
      jointError += e * e;
      ++dofs;
    }
  }
  buildRagdollJointPose3D(profile, m_coordinates, m_output.targetPose);
  auto &positions = m_output.targetPose.linkPositions;
  auto &orientations = m_output.targetPose.linkOrientations;
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
          (reference.positionsBody[i] - m_previousBodyPositions[i]) / dt, 10);
    positions[i] = root.position + root.orientation.rotate(positions[i]);
    orientations[i] = (root.orientation * orientations[i]).normalized();
  }
  m_previousBodyPositions = reference.positionsBody;
  m_previousBodyPositionsValid = true;
  m_output.gravityCompensationEnabled = !passiveFall;

  // ---------------------------------------------------------------------
  // RESIDUAL ASSISTANCE REQUEST
  // ---------------------------------------------------------------------
  // From this point on we are explicitly NOT doing the main balance work. Joint
  // drives and internal feed-forward above are primary. The residual layer is
  // support-gated and small; with zero support it must produce exactly zero
  // external righting/lift force.
  const float controllerAuthority = passiveFall || manipulated ? 0.0f : 1.0f;
  reference.poseAuthority = 0.0f; // Animation pose belongs to joint drives.
  reference.velocityHeading = m_directionBody * m_speed;
  reference.accelerationHeading = {};
  reference.upHeading = {0, 0, 1};

  // Separate support channels are intentional. Locomotion/posture require feet;
  // get-up lift may use hands/knees/body contacts. No hysteresis is fed into
  // these values, so throwing the character into the air immediately disables
  // every external residual.
  reference.shapeSupportAuthority = 0.0f;
  reference.movementSupportAuthority = standingSupportAuthority;
  reference.postureSupportAuthority = standingSupportAuthority;
  reference.liftSupportAuthority = recovery ? rawBodySupportAuthority : 0.0f;

  const bool locomotionPhase = moving(m_telemetry.phase) ||
                               m_telemetry.phase == Phase::Stopping;
  reference.movementAuthority =
      controllerAuthority * (locomotionPhase ? 1.0f : 0.0f);

  float residualPostureAuthority = standingResidualAuthority;
  if (activeFalling) {
    // Still try hard while a real foot support exists, but this remains a
    // residual. Once feet leave the environment the support gate makes it zero.
    residualPostureAuthority = std::max(
        residualPostureAuthority,
        std::clamp(m_settings.fallingResidualMaximumAuthority, 0.0f, 1.0f) *
            smoothRange(balanceUrgency, 0.15f, 1.0f));
  }
  if (recovery) {
    // During early/mid get-up there is NO external orientation servo. The clip
    // and joint motors must physically create the movement. Residual posture is
    // introduced only after the body is already near standing on its feet.
    const float upRamp = smoothRange(
        up, m_settings.recoveryResidualPostureStartUpDot, 0.86f);
    const float heightRamp = smoothRange(
        height / std::max(0.1f, profile.standingRootHeightMeters),
        m_settings.recoveryResidualPostureStartHeightFraction, 0.78f);
    residualPostureAuthority = recoveryNearStanding
        ? std::clamp(m_settings.recoveryResidualPostureMaximumAuthority,
                     0.0f, 1.0f) * std::min(upRamp, heightRamp)
        : 0.0f;
  }
  reference.balanceAuthority = controllerAuthority * residualPostureAuthority;

  const auto *rootTrack =
      findAnimationTrack3D(*clip, profile.links.front().id);
  const auto source = rootTrack
      ? sampleAnimationTrack3D(*rootTrack, sampleTime,
                               clip->durationSeconds, clip->loops)
      : AnimationTransformSample3D{};

  // For ordinary locomotion, residual posture may respect the authored root
  // lean relative to CURRENT heading. This still has no absolute yaw/world
  // orientation. Get-up intentionally ignores authored root orientation.
  if (!recovery) {
    reference.upHeading =
        (heading(source.rotationDelta).conjugate() * source.rotationDelta)
            .rotate({0, 0, 1});
  } else {
    reference.upHeading = {0, 0, 1};
    reference.recoveryMode = true;
    reference.velocityHeading = {};
    reference.accelerationHeading = {};
    reference.movementAuthority = 0.0f;

    // GET-UP LIFT V6
    // ----------------
    // The clip/joint motors decide HOW to stand up. This channel only makes the
    // body lighter in gravity-up so those muscles can actually perform it.
    // Crucially, the envelope comes from CURRENT PHYSICAL STATE, not animation
    // progress and not a target world height. If the attempt lags behind the
    // clip, the helper does not vanish just because the clip reached its end.
    const float standingHeight =
        std::max(0.10f, profile.standingRootHeightMeters);
    const float heightFraction = height / standingHeight;
    const float heightNeed = 1.0f - smoothRange(
        heightFraction, m_settings.recoveryLiftFadeStartHeightFraction,
        m_settings.recoveryLiftFadeEndHeightFraction);
    const float uprightNeed = 1.0f - smoothRange(
        up, m_settings.recoveryLiftFadeStartUpDot,
        m_settings.recoveryLiftFadeEndUpDot);

    // Keep assistance until BOTH height and orientation are convincingly close
    // to standing. A character that is high but still horizontal, or upright but
    // still sitting low, still needs help.
    const float physicalLiftNeed =
        std::clamp(std::max(heightNeed, uprightNeed), 0.0f, 1.0f);

    // At full need, unload roughly 80% of gravity before velocity feedback. The
    // hard force budget below prevents the helper from exceeding the configured
    // fraction of body weight. Near standing this term smoothly fades to zero.
    const float gravityFraction =
        std::max(0.0f, m_settings.recoveryLiftBaseGravityFraction) *
            physicalLiftNeed +
        std::max(0.0f, m_settings.recoveryLiftExtraGravityFraction) *
            physicalLiftNeed * physicalLiftNeed;

    reference.liftVelocityUpMetersPerSecond =
        std::max(0.0f, m_settings.recoveryRiseVelocityMetersPerSecond) *
        physicalLiftNeed;
    reference.liftAccelerationUpMetersPerSecondSquared =
        9.81f * gravityFraction;
    reference.liftAuthority = controllerAuthority;
  }

  auto assistSettings =
      static_cast<BodyRelativeRagdollAssistSettings3D>(m_settings);

  // Shape force is intentionally disabled in the final controller. Keeping the
  // implementation in BodyRelativeRagdollAssist3D is useful for diagnostics,
  // but production animation tracking is done by joint drives only.
  assistSettings.shapeAssistEnabled = false;
  assistSettings.maximumShapeAssistWeightFraction = 0.0f;
  assistSettings.maximumShapeAssistTorqueWeightLengthFraction = 0.0f;

  // Locomotion residual is small. The footwork rewrite will eventually reduce
  // dependence on this even further.
  assistSettings.maximumAssistWeightFraction = std::clamp(
      m_settings.walkingAssistWeightFraction, 0.0f, 0.18f);
  assistSettings.velocityResponsePerSecond = std::clamp(
      assistSettings.velocityResponsePerSecond, 0.2f, 1.2f);

  // Support-gated posture residual. The substantial standing authority is in
  // the internal joint controller above, not here.
  assistSettings.maximumAssistTorqueWeightLengthFraction = std::clamp(
      recovery ? m_settings.recoveryResidualPostureTorqueWeightLengthFraction
               : m_settings.standingAssistTorqueWeightLengthFraction,
      0.02f, 0.22f);
  assistSettings.maximumPostureTorquePerLinkWeightLengthFraction = std::clamp(
      m_settings.standingPostureTorquePerLinkWeightLengthFraction,
      0.02f, 0.12f);
  assistSettings.postureResidualActivationUrgency = std::clamp(
      m_settings.standingResidualActivationUrgency, 0.05f, 0.90f);

  if (recovery) {
    assistSettings.maximumAssistWeightFraction = 0.0f;
    assistSettings.maximumLiftAssistWeightFraction = std::clamp(
        m_settings.recoveryAssistWeightFraction, 0.0f, 1.10f);
    assistSettings.liftVelocityResponsePerSecond = std::clamp(
        m_settings.recoveryLiftVelocityResponsePerSecond, 0.5f, 6.0f);
    assistSettings.maximumLiftAccelerationMetersPerSecondSquared = std::clamp(
        m_settings.recoveryMaximumLiftAccelerationMetersPerSecondSquared,
        1.0f, 14.0f);
  } else {
    reference.liftAuthority = 0.0f;
  }
  m_telemetry.assistanceBudgetWeightFraction = m_settings.assistanceEnabled
      ? std::max(assistSettings.maximumAssistWeightFraction,
                 recovery ? assistSettings.maximumLiftAssistWeightFraction : 0.0f)
      : 0.0f;
  auto assistance = computeBodyRelativeRagdollAssist3D(
      profile, state, reference, assistSettings);
  m_output.assistance = std::move(assistance.wrenches);
  m_telemetry.assistanceForceNewtons = assistance.forceSumNewtons;
  m_telemetry.assistanceTorqueNewtonMeters = assistance.torqueSumNewtonMeters;
  m_telemetry.assistanceNetForceWorld = assistance.netForceWorld;
  m_telemetry.assistanceAuthority = reference.balanceAuthority;
  m_telemetry.recoveryLiftForceNewtons = assistance.liftForceWorld.length();
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
