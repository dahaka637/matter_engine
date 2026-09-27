#include "Engine/Control/BodyRelativeRagdollAssist3D.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <string_view>
#include <vector>

namespace MatterEngine {
namespace {

constexpr float GravityMetersPerSecondSquared = 9.81f;
constexpr float Epsilon = 1.0e-6f;

bool finite(Vec3 v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool finite(Quaternion q) {
  return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
         std::isfinite(q.w);
}

float bounded(float value, float maximum) {
  return std::isfinite(value) ? std::clamp(value, 0.0f, maximum) : 0.0f;
}

float safePositive(float value, float fallback) {
  return std::isfinite(value) && value > 0.0f ? value : fallback;
}

float smooth01(float value) {
  value = std::clamp(value, 0.0f, 1.0f);
  return value * value * (3.0f - 2.0f * value);
}

float smoothRange(float value, float begin, float end) {
  if (!std::isfinite(value) || !std::isfinite(begin) || !std::isfinite(end) ||
      end <= begin)
    return value >= end ? 1.0f : 0.0f;
  return smooth01((value - begin) / (end - begin));
}

float safeVectorScale(float magnitude, float budget) {
  if (!std::isfinite(magnitude) || magnitude <= Epsilon)
    return 1.0f;
  if (!std::isfinite(budget) || budget <= 0.0f)
    return 0.0f;
  return magnitude > budget ? budget / magnitude : 1.0f;
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

float postureWeightForLink(const RagdollProfile3D &profile, std::size_t index,
                           const BodyRelativeRagdollAssistSettings3D &settings) {
  const auto &link = profile.links[index];
  const std::string_view id = link.id;
  if (link.parentIndex < 0 || containsInsensitive(id, "pelvis") ||
      containsInsensitive(id, "hip"))
    return bounded(settings.pelvisPostureWeight, 4.0f);
  if (containsInsensitive(id, "spine") || containsInsensitive(id, "lumbar"))
    return bounded(settings.spinePostureWeight, 4.0f);
  if (containsInsensitive(id, "chest") || containsInsensitive(id, "torso"))
    return bounded(settings.chestPostureWeight, 4.0f);
  if (containsInsensitive(id, "neck"))
    return bounded(settings.neckPostureWeight, 4.0f);
  if (containsInsensitive(id, "head"))
    return bounded(settings.headPostureWeight, 4.0f);
  if (containsInsensitive(id, "thigh") || containsInsensitive(id, "leg") ||
      containsInsensitive(id, "shin") || containsInsensitive(id, "calf") ||
      containsInsensitive(id, "foot") || containsInsensitive(id, "ankle"))
    return bounded(settings.legPostureWeight, 4.0f);
  if (containsInsensitive(id, "arm") || containsInsensitive(id, "forearm") ||
      containsInsensitive(id, "hand"))
    return bounded(settings.armPostureWeight, 4.0f);
  return bounded(settings.otherPostureWeight, 4.0f);
}

float liftWeightForLink(const RagdollProfile3D &profile, std::size_t index,
                        const BodyRelativeRagdollAssistSettings3D &settings) {
  const auto &link = profile.links[index];
  const std::string_view id = link.id;
  if (link.parentIndex < 0 || containsInsensitive(id, "pelvis") ||
      containsInsensitive(id, "hip"))
    return bounded(settings.pelvisLiftWeight, 4.0f);
  if (containsInsensitive(id, "spine") || containsInsensitive(id, "lumbar"))
    return bounded(settings.spineLiftWeight, 4.0f);
  if (containsInsensitive(id, "chest") || containsInsensitive(id, "torso"))
    return bounded(settings.chestLiftWeight, 4.0f);
  if (containsInsensitive(id, "thigh") || containsInsensitive(id, "upperleg"))
    return bounded(settings.thighLiftWeight, 4.0f);
  return bounded(settings.otherLiftWeight, 4.0f);
}

// Axis-angle error that changes only the direction of an anatomical UP vector.
// There is intentionally no yaw target in this representation.
Vec3 directionRotationError(Vec3 current, Vec3 desired) {
  current = current.normalized();
  desired = desired.normalized();
  if (current.lengthSquared() <= Epsilon || desired.lengthSquared() <= Epsilon)
    return {};

  const Vec3 axisRaw = cross(current, desired);
  const float sine = axisRaw.length();
  const float cosine = std::clamp(dot(current, desired), -1.0f, 1.0f);
  if (sine <= Epsilon) {
    // For the exact antiparallel case choose a deterministic TILT axis. This is
    // only relevant if the caller explicitly authorizes posture residual while
    // upside-down; normal Fallen/get-up startup keeps that authority at zero.
    if (cosine > 0.0f)
      return {};
    const Vec3 helper = std::abs(current.z) < 0.75f ? Vec3{0, 0, 1}
                                                    : Vec3{1, 0, 0};
    Vec3 axis = cross(current, helper).normalized();
    if (axis.lengthSquared() <= Epsilon)
      axis = cross(current, Vec3{0, 1, 0}).normalized();
    return axis * 3.14159265358979323846f;
  }
  return axisRaw * (std::atan2(sine, cosine) / sine);
}

Vec3 anatomicalUpWorld(const RagdollLinkDefinition3D &definition,
                       const PhysicsBodyState3D &state) {
  const Vec3 neutralUpLocal =
      definition.modelOrientation.conjugate().rotate({0, 0, 1}).normalized();
  return state.orientation.rotate(neutralUpLocal).normalized();
}

} // namespace

BodyRelativeRagdollAssist3D computeBodyRelativeRagdollAssist3D(
    const RagdollProfile3D &profile, const RagdollState3D &state,
    const BodyRelativeRagdollReference3D &reference,
    const BodyRelativeRagdollAssistSettings3D &settings) {
  BodyRelativeRagdollAssist3D result;
  const std::size_t count = profile.links.size();

  // -----------------------------------------------------------------------
  // 0) CONTRACT VALIDATION
  // -----------------------------------------------------------------------
  if (!settings.assistanceEnabled || count == 0 || state.links.size() != count ||
      reference.positionsBody.size() != count ||
      reference.velocitiesBody.size() != count ||
      !finite(reference.velocityHeading) ||
      !finite(reference.accelerationHeading) || !finite(reference.upHeading) ||
      !std::isfinite(reference.liftVelocityUpMetersPerSecond) ||
      !std::isfinite(reference.liftAccelerationUpMetersPerSecondSquared) ||
      !std::isfinite(reference.supportAuthority) ||
      !std::isfinite(reference.shapeSupportAuthority) ||
      !std::isfinite(reference.movementSupportAuthority) ||
      !std::isfinite(reference.liftSupportAuthority) ||
      !std::isfinite(reference.postureSupportAuthority) ||
      !std::isfinite(profile.totalMassKg) || profile.totalMassKg <= 0.0f)
    return result;

  if (!reference.postureUpHeading.empty() &&
      reference.postureUpHeading.size() != count)
    return result;
  if (!reference.postureUpWorld.empty() &&
      reference.postureUpWorld.size() != count)
    return result;

  float massFractionSum = 0.0f;
  for (std::size_t i = 0; i < count; ++i) {
    const auto &link = state.links[i];
    if (!finite(link.position) || !finite(link.orientation) ||
        !finite(link.linearVelocity) || !finite(link.angularVelocity) ||
        !finite(reference.positionsBody[i]) ||
        !finite(reference.velocitiesBody[i]) ||
        !std::isfinite(profile.links[i].massFraction) ||
        profile.links[i].massFraction < 0.0f)
      return result;
    if (!reference.postureUpHeading.empty() &&
        !finite(reference.postureUpHeading[i]))
      return result;
    if (!reference.postureUpWorld.empty() &&
        !finite(reference.postureUpWorld[i]))
      return result;
    massFractionSum += profile.links[i].massFraction;
  }
  if (!std::isfinite(massFractionSum) || massFractionSum <= Epsilon)
    return result;

  const float supportAuthority = bounded(reference.supportAuthority, 1.0f);
  const float shapeSupportAuthority = std::max(
      supportAuthority, bounded(reference.shapeSupportAuthority, 1.0f));
  const float movementSupportAuthority = std::max(
      supportAuthority, bounded(reference.movementSupportAuthority, 1.0f));
  const float liftSupportAuthority = std::max(
      supportAuthority, bounded(reference.liftSupportAuthority, 1.0f));
  const float postureSupportAuthority = std::max(
      supportAuthority, bounded(reference.postureSupportAuthority, 1.0f));
  result.supportAuthority = std::max(
      {shapeSupportAuthority, movementSupportAuthority,
       liftSupportAuthority, postureSupportAuthority});

  const float poseAuthority = bounded(reference.poseAuthority, 1.0f);
  const float movementAuthority = bounded(reference.movementAuthority, 1.0f);
  const float liftAuthority = bounded(reference.liftAuthority, 1.0f);
  const float balanceAuthority = bounded(reference.balanceAuthority, 1.0f);

  // -----------------------------------------------------------------------
  // 1) CURRENT BODY-RELATIVE FRAME AND MASS OBSERVATIONS
  // -----------------------------------------------------------------------
  const auto &root = state.links.front();
  const Quaternion bodyRotation = root.orientation.normalized();
  const Quaternion inverseBody = bodyRotation.conjugate();
  if (!finite(bodyRotation))
    return result;

  const Vec3 rootComWorld =
      root.position + bodyRotation.rotate(profile.links.front().centerOfMassLocal);
  const Vec3 angularVelocityBody = inverseBody.rotate(root.angularVelocity);

  // Heading is used only by derivative-level movement intent. No heading is
  // retained between frames and no posture torque ever tries to hold it.
  Vec3 forward = bodyRotation.rotate({1, 0, 0});
  if (forward.x * forward.x + forward.y * forward.y < 0.0001f) {
    const Vec3 left = bodyRotation.rotate({0, 1, 0});
    forward = {left.y, -left.x, 0};
  }
  const Quaternion headingRotation = Quaternion::fromAxisAngle(
      {0, 0, 1}, std::atan2(forward.y, forward.x));
  const Quaternion inverseHeading = headingRotation.conjugate();

  std::vector<float> massShares(count, 0.0f);
  std::vector<Vec3> linkComWorld(count), offsetsBody(count);
  Vec3 wholeComWorld;
  Vec3 wholeComVelocityWorld;
  for (std::size_t i = 0; i < count; ++i) {
    const float share = profile.links[i].massFraction / massFractionSum;
    massShares[i] = share;
    linkComWorld[i] = state.links[i].position +
        state.links[i].orientation.rotate(profile.links[i].centerOfMassLocal);
    wholeComWorld += linkComWorld[i] * share;
    wholeComVelocityWorld += state.links[i].linearVelocity * share;
    offsetsBody[i] = inverseBody.rotate(linkComWorld[i] - rootComWorld);
  }

  const float totalWeight =
      profile.totalMassKg * GravityMetersPerSecondSquared;
  const float standingHeight =
      safePositive(profile.standingRootHeightMeters, 1.0f);
  const float weightLength = totalWeight * standingHeight;

  // -----------------------------------------------------------------------
  // 2) SHAPE RESIDUAL -- OFF BY DEFAULT, ZERO WRENCH BY CONTRACT
  // -----------------------------------------------------------------------
  std::vector<Vec3> shapeForcesBody(count), shapeReactionTorquesBody(count);
  const float shapeSupport = settings.shapeRequiresSupport ? shapeSupportAuthority : 1.0f;
  if (settings.shapeAssistEnabled && poseAuthority > 0.0f &&
      shapeSupport > Epsilon) {
    const float kp = bounded(settings.poseStiffnessPerSecondSquared, 40.0f);
    const float kd = bounded(settings.poseDampingPerSecond, 10.0f);
    Vec3 sumShapeForceBody;
    Vec3 comBody;

    for (std::size_t i = 0; i < count; ++i) {
      const Vec3 relativeVelocityBody =
          inverseBody.rotate(state.links[i].linearVelocity - root.linearVelocity) -
          cross(angularVelocityBody, offsetsBody[i]);
      const Vec3 positionError = reference.positionsBody[i] - offsetsBody[i];
      const Vec3 velocityError =
          reference.velocitiesBody[i] - relativeVelocityBody;
      shapeForcesBody[i] =
          (positionError * kp + velocityError * kd) *
          (profile.totalMassKg * massShares[i] * poseAuthority * shapeSupport);
      sumShapeForceBody += shapeForcesBody[i];
      comBody += offsetsBody[i] * massShares[i];
    }

    Vec3 shapeMomentBody;
    for (std::size_t i = 0; i < count; ++i) {
      shapeForcesBody[i] -= sumShapeForceBody * massShares[i];
      shapeMomentBody +=
          cross(offsetsBody[i] - comBody, shapeForcesBody[i]);
    }
    for (std::size_t i = 0; i < count; ++i)
      shapeReactionTorquesBody[i] = -shapeMomentBody * massShares[i];

    float forceMagnitude = 0.0f;
    float torqueMagnitude = 0.0f;
    for (std::size_t i = 0; i < count; ++i) {
      forceMagnitude += shapeForcesBody[i].length();
      torqueMagnitude += shapeReactionTorquesBody[i].length();
    }
    result.shapeForceScale = safeVectorScale(
        forceMagnitude,
        totalWeight * bounded(settings.maximumShapeAssistWeightFraction, 0.5f));
    result.shapeTorqueScale = safeVectorScale(
        torqueMagnitude,
        weightLength * bounded(
            settings.maximumShapeAssistTorqueWeightLengthFraction, 0.25f));
  }

  // -----------------------------------------------------------------------
  // 3) MOVEMENT RESIDUAL -- SMALL, SUPPORT-GATED, VELOCITY ONLY
  // -----------------------------------------------------------------------
  Vec3 movementForceWorld;
  const float movementSupport =
      settings.movementRequiresSupport ? movementSupportAuthority : 1.0f;
  if (settings.movementAssistEnabled && movementAuthority > 0.0f &&
      movementSupport > Epsilon) {
    const Vec3 comVelocityHeading = inverseHeading.rotate(wholeComVelocityWorld);
    const Vec3 velocityError = reference.velocityHeading - comVelocityHeading;
    const float response = bounded(settings.velocityResponsePerSecond, 5.0f);
    Vec3 acceleration = velocityError * response + reference.accelerationHeading;
    const float maxAcceleration = bounded(
        settings.maximumMovementAccelerationMetersPerSecondSquared, 12.0f);
    const float accelerationMagnitude = acceleration.length();
    if (maxAcceleration > 0.0f && accelerationMagnitude > maxAcceleration)
      acceleration *= maxAcceleration / accelerationMagnitude;

    movementForceWorld = headingRotation.rotate(acceleration) *
        (profile.totalMassKg * movementAuthority * movementSupport);
    const float budget = totalWeight *
        bounded(settings.maximumAssistWeightFraction, 0.5f);
    result.movementForceScale =
        safeVectorScale(movementForceWorld.length(), budget);
    movementForceWorld *= result.movementForceScale;
  }

  // -----------------------------------------------------------------------
  // 4) GET-UP LIFT -- SUPPORT-GATED GRAVITY UNLOADING, NEVER A POSITION SERVO
  // -----------------------------------------------------------------------
  std::vector<Vec3> liftForcesWorld(count), liftReactionTorquesWorld(count);
  const float liftSupport = settings.liftRequiresSupport ? liftSupportAuthority : 1.0f;
  if (settings.liftAssistEnabled && liftAuthority > 0.0f &&
      liftSupport > Epsilon) {
    // Two deliberately separate terms are accepted:
    //  - acceleration bias: explicit gravity unloading requested by get-up;
    //  - velocity deficit: extra help if the COM is not rising as requested.
    // Neither term knows or pursues a position. The support gate is what prevents
    // this from becoming a hover controller when the ragdoll leaves the ground.
    const float targetUpVelocity =
        std::max(0.0f, reference.liftVelocityUpMetersPerSecond);
    const float currentUpVelocity = wholeComVelocityWorld.z;
    const float velocityDeficit =
        std::max(0.0f, targetUpVelocity - currentUpVelocity);
    const float response =
        bounded(settings.liftVelocityResponsePerSecond, 8.0f);
    float liftAcceleration =
        std::max(0.0f, reference.liftAccelerationUpMetersPerSecondSquared) +
        velocityDeficit * response;
    const float maxAcceleration = bounded(
        settings.maximumLiftAccelerationMetersPerSecondSquared, 10.0f);
    if (maxAcceleration > 0.0f)
      liftAcceleration = std::min(liftAcceleration, maxAcceleration);

    float liftForceMagnitude = profile.totalMassKg * liftAcceleration *
                               liftAuthority * liftSupport;
    const float liftBudget = totalWeight *
        bounded(settings.maximumLiftAssistWeightFraction, 1.25f);
    result.liftForceScale = safeVectorScale(liftForceMagnitude, liftBudget);
    liftForceMagnitude *= result.liftForceScale;

    std::vector<float> liftWeights(count, 0.0f);
    float liftWeightSum = 0.0f;
    for (std::size_t i = 0; i < count; ++i) {
      liftWeights[i] = liftWeightForLink(profile, i, settings);
      liftWeightSum += liftWeights[i];
    }

    if (liftWeightSum > Epsilon && liftForceMagnitude > Epsilon) {
      Vec3 generatedMoment;
      for (std::size_t i = 0; i < count; ++i) {
        if (liftWeights[i] <= 0.0f)
          continue;
        const float share = liftWeights[i] / liftWeightSum;
        liftForcesWorld[i] = {0, 0, liftForceMagnitude * share};
        generatedMoment +=
            cross(linkComWorld[i] - wholeComWorld, liftForcesWorld[i]);
      }

      // Applying the lift at several core links can accidentally create a
      // rotation. Cancel that *parasitic* moment only; do not use this torque to
      // align the body to any pose.
      const float momentBudget = weightLength * bounded(
          settings.maximumLiftMomentCancellationWeightLengthFraction, 0.5f);
      result.liftMomentScale =
          safeVectorScale(generatedMoment.length(), momentBudget);
      const Vec3 cancellation = -generatedMoment * result.liftMomentScale;
      for (std::size_t i = 0; i < count; ++i) {
        if (liftWeights[i] > 0.0f)
          liftReactionTorquesWorld[i] =
              cancellation * (liftWeights[i] / liftWeightSum);
      }
    }
  }

  // -----------------------------------------------------------------------
  // 5) POSTURE RESIDUAL -- SUPPORT-GATED LAST-MILE CORRECTION
  // -----------------------------------------------------------------------
  std::vector<Vec3> postureTorquesWorld(count);
  const float postureSupport =
      settings.postureRequiresSupport ? postureSupportAuthority : 1.0f;
  const float postureAuthority = settings.postureAssistEnabled
      ? balanceAuthority * postureSupport
      : 0.0f;

  if (postureAuthority > Epsilon) {
    std::vector<float> postureWeights(count, 0.0f);
    float postureWeightSum = 0.0f;
    for (std::size_t i = 0; i < count; ++i) {
      postureWeights[i] = postureWeightForLink(profile, i, settings);
      postureWeightSum += postureWeights[i];
    }

    if (postureWeightSum > Epsilon) {
      const float kp = bounded(
          settings.postureStiffnessWeightLengthPerRadian, 1.5f);
      const float kd = bounded(
          settings.postureDampingWeightLengthSeconds, 0.5f);
      const float responseScale = bounded(settings.balanceResponseScale, 3.0f);
      const float recoveryGain = reference.recoveryMode
          ? bounded(settings.recoveryPostureGainMultiplier, 1.5f)
          : 1.0f;
      const float deadZone = bounded(settings.postureDeadZoneRadians, 0.3f);
      const float fullAngle = std::max(
          deadZone + 0.001f,
          bounded(settings.postureFullAuthorityRadians, 1.2f));
      const float fullSpeed = std::max(
          0.05f,
          bounded(settings.postureFullAuthorityAngularSpeed, 10.0f));
      const float activation = bounded(
          settings.postureResidualActivationUrgency, 0.95f);

      float torqueMagnitudeSum = 0.0f;
      for (std::size_t i = 0; i < count; ++i) {
        if (postureWeights[i] <= 0.0f)
          continue;

        Vec3 desiredUpWorld;
        if (!reference.postureUpWorld.empty()) {
          desiredUpWorld = reference.postureUpWorld[i].normalized();
        } else {
          Vec3 desiredUpHeading = reference.postureUpHeading.empty()
              ? reference.upHeading
              : reference.postureUpHeading[i];
          desiredUpHeading = desiredUpHeading.normalized();
          if (desiredUpHeading.lengthSquared() <= Epsilon)
            desiredUpHeading = {0, 0, 1};
          desiredUpWorld =
              headingRotation.rotate(desiredUpHeading).normalized();
        }
        if (desiredUpWorld.lengthSquared() <= Epsilon)
          desiredUpWorld = {0, 0, 1};

        const Vec3 currentUpWorld =
            anatomicalUpWorld(profile.links[i], state.links[i]);
        const Vec3 angularError =
            directionRotationError(currentUpWorld, desiredUpWorld);
        const float tilt = angularError.length();
        result.maximumPostureTiltRadians =
            std::max(result.maximumPostureTiltRadians, tilt);

        // Damping excludes angular velocity around desiredUpWorld; yaw remains
        // unconstrained even while the residual posture channel is active.
        const Vec3 omega = state.links[i].angularVelocity;
        const Vec3 tiltOmega =
            omega - desiredUpWorld * dot(omega, desiredUpWorld);
        const float angleUrgency = smoothRange(tilt, deadZone, fullAngle);
        const float speedUrgency = smoothRange(
            tiltOmega.length(), 0.12f, fullSpeed);
        const float urgency = std::max(angleUrgency, speedUrgency * 0.75f);
        result.postureUrgency = std::max(result.postureUrgency, urgency);

        // Residual means residual: below the activation threshold it contributes
        // nothing. Internal joints remain fully responsible for normal stance.
        const float residualGain =
            smoothRange(urgency, activation, 1.0f);
        if (residualGain <= Epsilon)
          continue;

        const float regionShare = postureWeights[i] / postureWeightSum;
        Vec3 torque = (angularError * kp - tiltOmega * kd) *
            (weightLength * responseScale * recoveryGain * postureAuthority *
             residualGain * regionShare);

        const float perLinkBudget = weightLength * bounded(
            settings.maximumPostureTorquePerLinkWeightLengthFraction, 0.30f);
        const float magnitude = torque.length();
        if (perLinkBudget > 0.0f && magnitude > perLinkBudget)
          torque *= perLinkBudget / magnitude;
        postureTorquesWorld[i] = torque;
        torqueMagnitudeSum += torque.length();
      }

      const float totalBudget = weightLength * bounded(
          settings.maximumAssistTorqueWeightLengthFraction, 0.40f);
      result.postureTorqueScale =
          safeVectorScale(torqueMagnitudeSum, totalBudget);
    }
  }

  result.budgetScale = std::min(
      {result.shapeForceScale, result.shapeTorqueScale,
       result.movementForceScale, result.liftForceScale,
       result.liftMomentScale, result.postureTorqueScale});

  // -----------------------------------------------------------------------
  // 6) COMPOSE EXPLICIT WRENCHES
  // -----------------------------------------------------------------------
  result.wrenches.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const Vec3 shapeForceWorld = bodyRotation.rotate(
        shapeForcesBody[i] * result.shapeForceScale);
    const Vec3 movementShare = movementForceWorld * massShares[i];
    const Vec3 forceWorld =
        shapeForceWorld + movementShare + liftForcesWorld[i];

    const Vec3 shapeTorqueWorld = bodyRotation.rotate(
        shapeReactionTorquesBody[i] * result.shapeTorqueScale);
    const Vec3 postureTorqueWorld =
        postureTorquesWorld[i] * result.postureTorqueScale;
    const Vec3 torqueWorld =
        shapeTorqueWorld + postureTorqueWorld + liftReactionTorquesWorld[i];

    if (!finite(forceWorld) || !finite(torqueWorld))
      return {};

    result.forceSumNewtons += forceWorld.length();
    result.torqueSumNewtonMeters += torqueWorld.length();
    result.netForceWorld += forceWorld;
    result.netTorqueWorld +=
        torqueWorld + cross(linkComWorld[i] - wholeComWorld, forceWorld);
    result.movementForceWorld += movementShare;
    result.liftForceWorld += liftForcesWorld[i];
    result.postureTorqueWorld += postureTorqueWorld;
    result.wrenches.push_back(
        {static_cast<std::uint32_t>(i), forceWorld, torqueWorld});
  }

  if (!finite(result.netForceWorld) || !finite(result.netTorqueWorld) ||
      !std::isfinite(result.forceSumNewtons) ||
      !std::isfinite(result.torqueSumNewtonMeters))
    return {};

  return result;
}

} // namespace MatterEngine
