#include "Engine/Locomotion/ContactFootwork3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {
constexpr std::size_t MaximumContactPoints = 16;
constexpr std::size_t MaximumContactsObserved = 256;
constexpr float Pi = 3.14159265358979323846f;
constexpr float Epsilon = 1.0e-6f;

bool finite(Vec3 p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }
bool finite(Quaternion q) {
  return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
         q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 0.0001f;
}
float safe(float value, float fallback, float low, float high) {
  return std::isfinite(value) ? std::clamp(value, low, high) : fallback;
}
float smooth(float t) {
  t = std::clamp(t, 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}
float smoothRange(float value, float begin, float end) {
  if (!std::isfinite(value) || !std::isfinite(begin) || !std::isfinite(end) || end <= begin)
    return value >= end ? 1.0f : 0.0f;
  return smooth((value - begin) / (end - begin));
}
Vec3 horizontal(Vec3 v) { return {v.x, v.y, 0}; }
Vec3 limited(Vec3 v, float maximum) {
  const float squared = v.lengthSquared();
  if (maximum <= 0.0f)
    return {};
  return squared > maximum * maximum ? v * (maximum / std::sqrt(squared)) : v;
}
float cross2(Vec3 a, Vec3 b) { return a.x * b.y - a.y * b.x; }
float lerp(float a, float b, float t) { return a + (b - a) * std::clamp(t, 0.0f, 1.0f); }

struct ContactPoints {
  std::array<Vec3, MaximumContactPoints> points{};
  std::size_t count = 0;
};

Vec3 anatomicalUpWorld(const RagdollLinkDefinition3D &definition, const PhysicsBodyState3D &state) {
  const Vec3 neutralUpLocal = definition.modelOrientation.conjugate().rotate({0, 0, 1}).normalized();
  return state.orientation.rotate(neutralUpLocal).normalized();
}

// Nearest point on the convex hull of ACTUAL support contacts. This is what
// makes the capture-point error meaningful: a toe contact is not silently
// expanded into a full rectangular foot support polygon.
Vec3 nearestSupportPoint(const std::array<ContactPoints, 2> &feet,
                         const ContactFootworkOutput3D &observation, Vec3 p,
                         Vec3 &geometricCenter) {
  std::array<Vec3, MaximumContactPoints * 2> points{};
  std::size_t count = 0;
  for (int foot = 0; foot < 2; ++foot) {
    if (!observation.feet[foot].supported)
      continue;
    for (std::size_t i = 0; i < feet[foot].count; ++i)
      points[count++] = horizontal(feet[foot].points[i]);
  }
  if (count == 0)
    return p;

  std::sort(points.begin(), points.begin() + count, [](Vec3 a, Vec3 b) {
    return a.x < b.x || (a.x == b.x && a.y < b.y);
  });

  std::array<Vec3, MaximumContactPoints * 4> hull{};
  std::size_t size = 0;
  for (std::size_t i = 0; i < count; ++i) {
    while (size >= 2 && cross2(hull[size - 1] - hull[size - 2], points[i] - hull[size - 1]) <= 0.0f)
      --size;
    hull[size++] = points[i];
  }
  const std::size_t lowerSize = size;
  for (std::size_t i = count - 1; i > 0; --i) {
    while (size > lowerSize && cross2(hull[size - 1] - hull[size - 2], points[i - 1] - hull[size - 1]) <= 0.0f)
      --size;
    hull[size++] = points[i - 1];
  }
  if (size > 1)
    --size;

  float twiceArea = 0.0f;
  Vec3 centroid;
  for (std::size_t i = 0; i < size; ++i) {
    const Vec3 a = hull[i], b = hull[(i + 1) % size];
    const float area = cross2(a, b);
    twiceArea += area;
    centroid += (a + b) * area;
  }
  centroid = std::abs(twiceArea) > 0.0000001f ? centroid / (3.0f * twiceArea)
                                                : (hull[0] + hull[size - 1]) * 0.5f;
  geometricCenter.x = centroid.x;
  geometricCenter.y = centroid.y;

  bool inside = size >= 3;
  Vec3 nearest = hull[0];
  float bestSquared = (p - nearest).lengthSquared();
  for (std::size_t i = 0; i < size; ++i) {
    const Vec3 a = hull[i], edge = hull[(i + 1) % size] - a;
    inside = inside && cross2(edge, p - a) >= -0.000001f;
    const float denominator = edge.lengthSquared();
    const float t = denominator > 0.0000001f ? std::clamp(dot(p - a, edge) / denominator, 0.0f, 1.0f) : 0.0f;
    const Vec3 point = a + edge * t;
    const float squared = (p - point).lengthSquared();
    if (squared < bestSquared) {
      bestSquared = squared;
      nearest = point;
    }
  }
  return inside ? p : nearest;
}
} // namespace

void ContactFootwork3D::reset(const RagdollProfile3D &profile) {
  m_output = {};
  m_linkCount = profile.links.size();
  m_footIndices.fill(RagdollDynamics3D::InvalidIndex);
  m_neutralFeet = {};
  m_soleHeight = {};
  m_phase = ContactFootworkPhase3D::Unsupported;
  m_stepReason = FootstepReason3D::None;
  m_phaseSeconds = m_withoutSupportSeconds = 0.0f;
  m_swingDuration = 0.31f;
  m_swingFoot = -1;
  m_lastSwingFoot = 1;
  m_swingReleased = false;
  m_swingStartHeading = m_swingGoalHeading = m_stepDirectionHeading = {};

  for (std::size_t i = 0; i < profile.links.size(); ++i) {
    const auto &link = profile.links[i];
    const int foot = link.id == "LeftFoot" ? 0 : link.id == "RightFoot" ? 1 : -1;
    if (foot < 0)
      continue;

    m_footIndices[foot] = static_cast<std::uint32_t>(i);
    m_neutralFeet[foot] = link.modelPosition - profile.links.front().modelPosition;
    const auto &shape = link.collider;
    if (shape.shape == RagdollColliderShape3D::Box) {
      const Quaternion rotation = link.modelOrientation * shape.localOrientation;
      m_soleHeight[foot] =
          std::abs(rotation.rotate({1, 0, 0}).z) * shape.boxHalfExtents.x +
          std::abs(rotation.rotate({0, 1, 0}).z) * shape.boxHalfExtents.y +
          std::abs(rotation.rotate({0, 0, 1}).z) * shape.boxHalfExtents.z -
          link.modelOrientation.rotate(shape.localPosition).z;
    } else {
      m_soleHeight[foot] = shape.radiusMeters;
    }
    m_soleHeight[foot] = safe(m_soleHeight[foot], 0.035f, 0.005f, 0.25f);
  }
}

void ContactFootwork3D::update(const RagdollProfile3D &profile, const RagdollState3D &state,
                               float dt, const ContactFootworkIntent3D &intent) {
  m_output = {};
  auto &out = m_output;
  for (int foot = 0; foot < 2; ++foot)
    out.feet[foot].linkIndex = m_footIndices[foot];

  if (profile.links.empty() || profile.links.size() != m_linkCount || state.links.size() != m_linkCount ||
      m_linkCount > 256 || m_footIndices[0] >= m_linkCount || m_footIndices[1] >= m_linkCount ||
      !std::isfinite(dt) || dt <= 0.0f || !finite(intent.desiredVelocityHeading)) {
    m_phase = ContactFootworkPhase3D::Unsupported;
    m_stepReason = FootstepReason3D::None;
    m_swingFoot = -1;
    return;
  }

  const float gravity = safe(m_settings.gravityMetersPerSecondSquared, 9.81f, 0.1f, 50.0f);
  const float mass = safe(profile.totalMassKg, 75.0f, 0.1f, 10000.0f);
  const float bodyHeight = safe(profile.standingRootHeightMeters, 0.9f, 0.2f, 4.0f);
  const float phaseDt = std::min(dt, 0.05f);
  const Vec3 desiredVelocityHeading = limited(horizontal(intent.desiredVelocityHeading), 8.0f);

  float massSum = 0.0f;
  Vec3 centerRelativeWorld, velocityWorld;
  for (std::size_t i = 0; i < m_linkCount; ++i) {
    const auto &link = state.links[i];
    const auto &definition = profile.links[i];
    if (!finite(link.position) || !finite(link.orientation) || !finite(link.linearVelocity) ||
        !finite(definition.centerOfMassLocal) || !std::isfinite(definition.massFraction) ||
        definition.massFraction < 0.0f) {
      m_phase = ContactFootworkPhase3D::Unsupported;
      m_stepReason = FootstepReason3D::None;
      m_swingFoot = -1;
      return;
    }
    centerRelativeWorld +=
        (link.position - state.links.front().position +
         link.orientation.normalized().rotate(definition.centerOfMassLocal)) *
        definition.massFraction;
    velocityWorld += link.linearVelocity * definition.massFraction;
    massSum += definition.massFraction;
  }
  if (!std::isfinite(massSum) || massSum <= 0.0f || !finite(centerRelativeWorld) || !finite(velocityWorld))
    return;

  const auto &root = state.links.front();
  const Quaternion rootRotation = root.orientation.normalized();
  Vec3 forward = rootRotation.rotate({1, 0, 0});
  if (horizontal(forward).lengthSquared() < 0.0001f) {
    const Vec3 left = rootRotation.rotate({0, 1, 0});
    forward = {left.y, -left.x, 0};
  }
  out.heading = Quaternion::fromAxisAngle({0, 0, 1}, std::atan2(forward.y, forward.x));
  const Quaternion inverseHeading = out.heading.conjugate();
  out.centerOfMassHeading = inverseHeading.rotate(centerRelativeWorld / massSum);
  out.centerOfMassVelocityHeading = inverseHeading.rotate(velocityWorld / massSum);

  Vec3 rootUp = anatomicalUpWorld(profile.links.front(), root);
  if (rootUp.lengthSquared() <= Epsilon)
    rootUp = {0, 0, 1};
  out.uprightDot = std::clamp(rootUp.z, -1.0f, 1.0f);
  out.bodyTiltRadians = std::acos(out.uprightDot);
  const Vec3 omegaTilt = root.angularVelocity - Vec3{0, 0, 1} * root.angularVelocity.z;
  out.tiltAngularSpeedRadiansPerSecond = omegaTilt.length();
  out.valid = true;

  // -----------------------------------------------------------------------
  // Observe real foot support.
  // -----------------------------------------------------------------------
  std::array<ContactPoints, 2> contactPoints;
  std::array<Vec3, 2> contactPositionSum{};
  std::array<float, 2> verticalImpulse{};
  const float minimumNormalZ = safe(m_settings.minimumSupportNormalZ, 0.62f, 0.2f, 0.99f);

  for (std::size_t i = 0; i < std::min(state.contacts.size(), MaximumContactsObserved); ++i) {
    const auto &contact = state.contacts[i];
    if (contact.linkIndex >= m_linkCount || !finite(contact.position) || !finite(contact.normal) ||
        !std::isfinite(contact.normalImpulseNewtonSeconds) || contact.normalImpulseNewtonSeconds <= 0.0f)
      continue;

    const Vec3 normal = contact.normal.normalized();
    if (normal.z < minimumNormalZ)
      continue;

    const int foot = contact.linkIndex == m_footIndices[0] ? 0 : contact.linkIndex == m_footIndices[1] ? 1 : -1;
    if (foot < 0) {
      out.nonFootGroundContact = true;
      continue;
    }

    const Vec3 p = inverseHeading.rotate(contact.position - root.position);
    const float impulse = std::min(contact.normalImpulseNewtonSeconds * normal.z, mass * gravity * dt * 5.0f);
    verticalImpulse[foot] += impulse;
    contactPositionSum[foot] += p * impulse;
    auto &points = contactPoints[foot];
    if (points.count < points.points.size())
      points.points[points.count++] = p;
  }

  float supportedImpulse = 0.0f;
  const float minimumLoad = safe(m_settings.minimumSupportWeightFraction, 0.010f, 0.001f, 0.25f);
  for (int foot = 0; foot < 2; ++foot) {
    auto &f = out.feet[foot];
    f.normalForceNewtons = verticalImpulse[foot] / dt;
    f.supportLoad = f.normalForceNewtons / std::max(1.0f, mass * gravity);
    f.supported = f.supportLoad >= minimumLoad && contactPoints[foot].count > 0;
    f.targetPositionHeading = inverseHeading.rotate(state.links[m_footIndices[foot]].position - root.position);
    if (verticalImpulse[foot] > 0.0f)
      f.contactPointHeading = contactPositionSum[foot] / verticalImpulse[foot];
    if (f.supported) {
      ++out.supportCount;
      out.centerOfPressureHeading += contactPositionSum[foot];
      supportedImpulse += verticalImpulse[foot];
    }
  }

  if (out.supportCount > 0) {
    out.centerOfPressureHeading = out.centerOfPressureHeading / std::max(supportedImpulse, Epsilon);
    std::size_t pointCount = 0;
    for (int foot = 0; foot < 2; ++foot) {
      if (!out.feet[foot].supported)
        continue;
      for (std::size_t point = 0; point < contactPoints[foot].count; ++point) {
        out.supportCenterHeading += contactPoints[foot].points[point];
        ++pointCount;
      }
    }
    if (pointCount > 0)
      out.supportCenterHeading = out.supportCenterHeading / static_cast<float>(pointCount);
    out.rootHeightAboveSupport = -out.supportCenterHeading.z;
    m_withoutSupportSeconds = 0.0f;
  } else {
    m_withoutSupportSeconds = std::min(m_withoutSupportSeconds + phaseDt, 60.0f);
  }
  out.secondsWithoutSupport = m_withoutSupportSeconds;

  out.likelyFallen =
      (out.nonFootGroundContact && out.uprightDot < 0.55f) ||
      (out.supportCount > 0 && out.rootHeightAboveSupport < bodyHeight * 0.44f && out.uprightDot < 0.55f) ||
      (out.uprightDot < 0.12f && m_withoutSupportSeconds > 0.25f);

  // -----------------------------------------------------------------------
  // Capture point and balance urgency.
  // -----------------------------------------------------------------------
  if (out.supportCount > 0) {
    const float pendulumHeight = std::clamp(out.centerOfMassHeading.z - out.supportCenterHeading.z,
                                            bodyHeight * 0.25f, bodyHeight * 1.5f);
    const float omega = std::sqrt(gravity / std::max(0.05f, pendulumHeight));
    out.capturePointHeading = horizontal(out.centerOfMassHeading) +
                              horizontal(out.centerOfMassVelocityHeading - desiredVelocityHeading) / omega;
    const Vec3 nearest = nearestSupportPoint(contactPoints, out, out.capturePointHeading, out.supportCenterHeading);
    out.supportErrorHeading = horizontal(out.capturePointHeading - nearest);
    out.desiredComCorrectionHeading = limited(
        -out.supportErrorHeading,
        safe(m_settings.maximumComCorrectionMeters, 0.18f, 0.01f, 0.5f));
  } else {
    out.capturePointHeading = horizontal(out.centerOfMassHeading);
  }

  const float captureError = horizontal(out.supportErrorHeading).length();
  const float captureMargin = safe(m_settings.captureMarginMeters, 0.025f, 0.003f, 0.2f);
  const float captureUrgency = out.supportCount > 0
      ? smoothRange(captureError, captureMargin * 0.55f,
                    std::max(captureMargin + 0.01f, m_settings.fullRecoveryCaptureErrorMeters))
      : 0.0f;
  const float tiltUrgency = smoothRange(out.bodyTiltRadians, m_settings.recoveryTiltStartRadians,
                                        std::max(m_settings.recoveryTiltStartRadians + 0.01f,
                                                 m_settings.recoveryTiltFullRadians));
  const float angularUrgency = smoothRange(
      out.tiltAngularSpeedRadiansPerSecond, m_settings.recoveryAngularSpeedStartRadiansPerSecond,
      std::max(m_settings.recoveryAngularSpeedStartRadiansPerSecond + 0.1f,
               m_settings.recoveryAngularSpeedFullRadiansPerSecond));
  out.stepUrgency = std::max({captureUrgency, tiltUrgency * 0.75f, angularUrgency * 0.70f});

  // A tilt by itself can normally be recovered with ankle/hip strategy. A step
  // becomes mandatory when the capture point leaves support, or when a fast
  // tilt is already developing and waiting would make the later step sluggish.
  const bool balanceNeedsStep = intent.balanceRecoveryEnabled && out.supportCount > 0 &&
      (captureError > captureMargin ||
       (tiltUrgency > 0.68f && angularUrgency > 0.35f));
  const float desiredSpeed = desiredVelocityHeading.length();
  const bool locomotionNeedsStep = intent.locomotionEnabled &&
      desiredSpeed > safe(m_settings.locomotionStartSpeedMetersPerSecond, 0.08f, 0.01f, 1.0f);

  const float lossTimeout = safe(m_settings.supportLossTimeoutSeconds, 0.14f, 0.02f, 0.5f);
  const bool canPlanSteps = intent.allowSteps && !out.likelyFallen &&
                            out.supportCount > 0 && m_withoutSupportSeconds < lossTimeout;
  if (!canPlanSteps) {
    m_phase = out.supportCount > 0 ? ContactFootworkPhase3D::Supported : ContactFootworkPhase3D::Unsupported;
    m_stepReason = FootstepReason3D::None;
    m_swingFoot = -1;
    m_phaseSeconds = 0.0f;
  } else if (m_phase == ContactFootworkPhase3D::Unsupported) {
    m_phase = ContactFootworkPhase3D::Supported;
    m_stepReason = FootstepReason3D::None;
  }

  m_phaseSeconds += phaseDt;

  // -----------------------------------------------------------------------
  // Start a new step. Balance recovery has priority over locomotion.
  // -----------------------------------------------------------------------
  if (canPlanSteps && m_phase == ContactFootworkPhase3D::Supported &&
      (balanceNeedsStep || locomotionNeedsStep)) {
    m_stepReason = balanceNeedsStep ? FootstepReason3D::BalanceRecovery : FootstepReason3D::Locomotion;

    Vec3 stepDirection = m_stepReason == FootstepReason3D::BalanceRecovery
        ? horizontal(out.supportErrorHeading + desiredVelocityHeading * 0.04f)
        : horizontal(desiredVelocityHeading);
    if (stepDirection.lengthSquared() < 0.0001f)
      stepDirection = horizontal(out.centerOfMassVelocityHeading);
    m_stepDirectionHeading = stepDirection.normalized();
    out.plannedStepDirectionHeading = m_stepDirectionHeading;

    // Sideways recovery should use the foot on the side of the fall. For
    // forward/backward motion, alternate unless one foot is clearly unloaded.
    if (std::abs(stepDirection.y) > std::max(0.04f, std::abs(stepDirection.x) * 0.45f)) {
      m_swingFoot = stepDirection.y > 0.0f ? 0 : 1;
    } else if (out.supportCount == 2 && std::abs(out.feet[0].supportLoad - out.feet[1].supportLoad) > 0.12f) {
      m_swingFoot = out.feet[0].supportLoad < out.feet[1].supportLoad ? 0 : 1;
    } else {
      m_swingFoot = 1 - m_lastSwingFoot;
    }

    // If only one foot is loaded, the unloaded foot is the only legal swing
    // candidate. Never lift the sole remaining stance foot.
    if (!out.feet[1 - m_swingFoot].supported && out.feet[m_swingFoot].supported)
      m_swingFoot = 1 - m_swingFoot;
    if (!out.feet[1 - m_swingFoot].supported) {
      m_swingFoot = -1;
      m_stepReason = FootstepReason3D::None;
    } else {
      const float urgency = m_stepReason == FootstepReason3D::BalanceRecovery ? out.stepUrgency : 0.0f;
      const float normalDuration = safe(m_settings.normalSwingDurationSeconds, 0.31f, 0.16f, 0.8f);
      const float minimumDuration = safe(m_settings.minimumRecoverySwingDurationSeconds, 0.14f, 0.08f, normalDuration);
      const float maximumDuration = safe(m_settings.maximumSwingDurationSeconds, 0.46f, normalDuration, 0.9f);
      const float speedFactor = std::clamp(desiredSpeed / 3.0f, 0.0f, 1.0f);
      m_swingDuration = m_stepReason == FootstepReason3D::BalanceRecovery
          ? lerp(normalDuration, minimumDuration, urgency)
          : lerp(maximumDuration, normalDuration * 0.82f, speedFactor);
      m_swingDuration = std::clamp(m_swingDuration, minimumDuration, maximumDuration);

      m_phase = ContactFootworkPhase3D::WeightShift;
      m_phaseSeconds = 0.0f;
      m_swingStartHeading = out.feet[m_swingFoot].targetPositionHeading;
      m_swingReleased = false;
    }
  }

  auto computeStepGoal = [&]() {
    if (m_swingFoot < 0)
      return Vec3{};

    const bool recoveryStep = m_stepReason == FootstepReason3D::BalanceRecovery;
    Vec3 goal;
    if (recoveryStep) {
      // Capture point is the primary landing target. A short velocity lead keeps
      // the foot from always arriving one frame behind an accelerating fall.
      goal = horizontal(out.capturePointHeading) +
             horizontal(out.centerOfMassVelocityHeading) *
                 safe(m_settings.recoveryLeadSeconds, 0.10f, 0.0f, 0.4f);
    } else {
      const float lead = safe(m_settings.locomotionLeadSeconds, 0.23f, 0.05f, 0.6f);
      const float errorLead = safe(m_settings.velocityErrorLeadSeconds, 0.10f, 0.0f, 0.4f);
      goal = horizontal(out.centerOfMassHeading) + desiredVelocityHeading * lead +
             horizontal(out.centerOfMassVelocityHeading - desiredVelocityHeading) * errorLead;
    }

    const float side = m_swingFoot == 0 ? 1.0f : -1.0f;
    const float halfSeparation = safe(m_settings.minimumFootSeparationMeters, 0.10f, 0.04f, bodyHeight * 0.4f) * 0.5f;
    goal.y += m_neutralFeet[m_swingFoot].y;
    goal.y = side * std::max(goal.y * side, halfSeparation);

    const float reach = bodyHeight * safe(m_settings.maximumStepReachHeightFraction, 0.68f, 0.15f, 0.90f);
    goal.y = std::clamp(goal.y, -reach * 0.82f, reach * 0.82f);
    const float xReach = std::sqrt(std::max(0.0f, reach * reach - goal.y * goal.y));
    goal.x = std::clamp(goal.x, -xReach, xReach);
    goal.z = out.supportCenterHeading.z + m_soleHeight[m_swingFoot];
    return goal;
  };

  // -----------------------------------------------------------------------
  // Execute step state machine.
  // -----------------------------------------------------------------------
  if (m_swingFoot >= 0) {
    auto &swing = out.feet[m_swingFoot];
    const auto &stance = out.feet[1 - m_swingFoot];
    out.plannedSwingDurationSeconds = m_swingDuration;
    out.plannedStepDirectionHeading = m_stepDirectionHeading;
    out.stepReason = m_stepReason;

    if (!stance.supported) {
      m_phase = ContactFootworkPhase3D::Supported;
      m_stepReason = FootstepReason3D::None;
      m_swingFoot = -1;
      m_phaseSeconds = 0.0f;
      out.stepTimedOut = true;
    } else if (m_phase == ContactFootworkPhase3D::WeightShift) {
      const bool emergency = m_stepReason == FootstepReason3D::BalanceRecovery && out.stepUrgency > 0.55f;
      const float normalMinShift = safe(m_settings.minimumWeightShiftSeconds, 0.055f, 0.01f, 0.35f);
      const float emergencyShift = safe(m_settings.emergencyWeightShiftSeconds, 0.018f, 0.0f, normalMinShift);
      const float minShift = emergency ? lerp(normalMinShift, emergencyShift, out.stepUrgency) : normalMinShift;
      const float maxShift = safe(m_settings.maximumWeightShiftSeconds, 0.24f, normalMinShift, 0.7f);
      const float requiredLoad = emergency
          ? lerp(safe(m_settings.minimumStanceWeightFraction, 0.56f, 0.30f, 0.95f),
                 safe(m_settings.emergencyMinimumStanceWeightFraction, 0.34f, 0.10f, 0.80f), out.stepUrgency)
          : safe(m_settings.minimumStanceWeightFraction, 0.56f, 0.30f, 0.95f);

      out.phaseProgress = minShift > Epsilon ? std::clamp(m_phaseSeconds / minShift, 0.0f, 1.0f) : 1.0f;
      const Vec3 transfer = horizontal(stance.contactPointHeading - out.centerOfMassHeading);
      out.desiredComCorrectionHeading = limited(
          transfer, safe(m_settings.maximumComCorrectionMeters, 0.18f, 0.01f, 0.5f));

      const bool loadReady = stance.supportLoad >= requiredLoad;
      const bool emergencyReady = emergency && m_phaseSeconds >= minShift && stance.supported &&
                                  (stance.supportLoad >= m_settings.emergencyMinimumStanceWeightFraction || out.supportCount == 1);
      if ((m_phaseSeconds >= minShift && loadReady) || emergencyReady ||
          (emergency && out.stepUrgency > 0.90f && m_phaseSeconds >= emergencyShift)) {
        m_phase = ContactFootworkPhase3D::Swing;
        m_phaseSeconds = 0.0f;
        out.phaseProgress = 0.0f;
        m_swingStartHeading = swing.targetPositionHeading;
        m_swingGoalHeading = computeStepGoal();
      } else if (m_phaseSeconds >= maxShift) {
        // For a normal walk, abort a bad transfer. For an urgent recovery, if
        // there is still a stance foot, release anyway rather than freezing.
        if (emergency && stance.supported) {
          m_phase = ContactFootworkPhase3D::Swing;
          m_phaseSeconds = 0.0f;
          m_swingStartHeading = swing.targetPositionHeading;
          m_swingGoalHeading = computeStepGoal();
        } else {
          m_phase = ContactFootworkPhase3D::Supported;
          m_stepReason = FootstepReason3D::None;
          m_swingFoot = -1;
          m_phaseSeconds = 0.0f;
          out.stepTimedOut = true;
        }
      }
    } else if (m_phase == ContactFootworkPhase3D::Swing ||
               m_phase == ContactFootworkPhase3D::Touchdown) {
      m_swingReleased = m_swingReleased || !swing.supported;

      if (out.supportCount > 0) {
        const Vec3 newGoal = computeStepGoal();
        const float replanSpeed = m_stepReason == FootstepReason3D::BalanceRecovery
            ? safe(m_settings.recoveryGoalReplanSpeedMetersPerSecond, 4.5f, 0.2f, 12.0f)
            : safe(m_settings.goalReplanSpeedMetersPerSecond, 2.2f, 0.1f, 8.0f);
        m_swingGoalHeading += limited(newGoal - m_swingGoalHeading, phaseDt * replanSpeed);
      }

      const float t = std::clamp(m_phaseSeconds / std::max(0.05f, m_swingDuration), 0.0f, 1.0f);
      out.phaseProgress = t;
      const float blend = smooth(t);
      swing.targetPositionHeading = m_swingStartHeading * (1.0f - blend) + m_swingGoalHeading * blend;

      // Recovery lifts the foot more aggressively so it does not catch the
      // ground while the body is already falling. The arc starts immediately.
      const float recoveryHeight = m_stepReason == FootstepReason3D::BalanceRecovery
          ? safe(m_settings.recoveryExtraStepHeightMeters, 0.035f, 0.0f, bodyHeight * 0.15f) * out.stepUrgency
          : 0.0f;
      const float arc = std::sin(Pi * t);
      swing.targetPositionHeading.z += arc *
          (safe(m_settings.stepHeightMeters, 0.075f, 0.015f, bodyHeight * 0.25f) + recoveryHeight);
      swing.targetWeight = 1.0f;
      swing.swingWeight = arc;

      if (m_swingReleased && swing.supported && t >= 0.32f) {
        m_phase = ContactFootworkPhase3D::Supported;
        m_lastSwingFoot = m_swingFoot;
        m_stepReason = FootstepReason3D::None;
        m_swingFoot = -1;
        m_phaseSeconds = 0.0f;
        swing.targetWeight = swing.swingWeight = 0.0f;
      } else if (t >= 1.0f) {
        m_phase = ContactFootworkPhase3D::Touchdown;
        // Search slightly downward after the planned touchdown. This is not a
        // terrain servo; it simply prevents a millimetric miss from holding the
        // foot in the air until timeout.
        const float overtime = std::max(0.0f, m_phaseSeconds - m_swingDuration);
        swing.targetPositionHeading = m_swingGoalHeading;
        swing.targetPositionHeading.z -= std::min(0.035f, overtime * 0.18f);
        swing.targetWeight = 1.0f;
        swing.swingWeight = 0.0f;

        const float timeout = safe(m_settings.touchdownTimeoutSeconds, 0.16f, 0.04f, 0.6f);
        if (overtime >= timeout) {
          m_phase = out.supportCount > 0 ? ContactFootworkPhase3D::Supported
                                         : ContactFootworkPhase3D::Unsupported;
          m_stepReason = FootstepReason3D::None;
          m_swingFoot = -1;
          m_phaseSeconds = 0.0f;
          out.stepTimedOut = true;
          swing.targetWeight = swing.swingWeight = 0.0f;
        }
      }
    }
  }

  out.phase = m_phase;
  out.stepReason = m_stepReason;
  out.swingFoot = m_swingFoot;
  if (m_swingFoot < 0) {
    out.phaseProgress = 0.0f;
    out.plannedSwingDurationSeconds = 0.0f;
  }
}

} // namespace MatterEngine
