#include "Engine/Control/BodyRelativeRagdollAssist3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {
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
} // namespace

BodyRelativeRagdollAssist3D computeBodyRelativeRagdollAssist3D(
    const RagdollProfile3D &profile, const RagdollState3D &state,
    const BodyRelativeRagdollReference3D &reference,
    const BodyRelativeRagdollAssistSettings3D &settings) {
  BodyRelativeRagdollAssist3D result;
  const std::size_t count = profile.links.size();
  if (!settings.assistanceEnabled || count == 0 ||
      state.links.size() != count || reference.positionsBody.size() != count ||
      reference.velocitiesBody.size() != count ||
      !finite(reference.velocityHeading) || !finite(reference.upHeading) ||
      !std::isfinite(profile.totalMassKg) || profile.totalMassKg <= 0)
    return result;
  if (bounded(reference.poseAuthority, 1.0f) == 0 &&
      bounded(reference.movementAuthority, 1.0f) == 0 &&
      bounded(reference.balanceAuthority, 1.0f) == 0)
    return result;

  float fractionSum = 0;
  for (std::size_t i = 0; i < count; ++i) {
    const auto &link = state.links[i];
    if (!finite(link.position) || !finite(link.orientation) ||
        !finite(link.linearVelocity) || !finite(link.angularVelocity) ||
        !finite(reference.positionsBody[i]) ||
        !finite(reference.velocitiesBody[i]) ||
        !std::isfinite(profile.links[i].massFraction) ||
        profile.links[i].massFraction < 0)
      return result;
    fractionSum += profile.links[i].massFraction;
  }
  if (!std::isfinite(fractionSum) || fractionSum <= 0)
    return result;

  const auto &root = state.links.front();
  const auto bodyRotation = root.orientation.normalized();
  const auto inverseBody = bodyRotation.conjugate();
  const Vec3 origin =
      root.position +
      bodyRotation.rotate(profile.links.front().centerOfMassLocal);
  const Vec3 forward = bodyRotation.rotate({1, 0, 0});
  const auto headingRotation =
      Quaternion::fromAxisAngle({0, 0, 1}, std::atan2(forward.y, forward.x));
  const Vec3 angularVelocityBody = inverseBody.rotate(root.angularVelocity);

  std::vector<Vec3> offsets(count), forces(count), torques(count);
  std::vector<float> shares(count);
  Vec3 comBody, comVelocityWorld, sumShapeForce;
  const float kp = bounded(settings.poseStiffnessPerSecondSquared, 50.0f);
  const float kd = bounded(settings.poseDampingPerSecond, 10.0f);
  const float poseAuthority = bounded(reference.poseAuthority, 1.0f);
  for (std::size_t i = 0; i < count; ++i) {
    const auto &link = state.links[i];
    shares[i] = profile.links[i].massFraction / fractionSum;
    offsets[i] = inverseBody.rotate(
        link.position +
        link.orientation.rotate(profile.links[i].centerOfMassLocal) - origin);
    // PhysX snapshots contain COM linear velocities. Subtract rigid-body
    // translation AND rotation before comparing articulation motion.
    const Vec3 relativeVelocity =
        inverseBody.rotate(link.linearVelocity - root.linearVelocity) -
        cross(angularVelocityBody, offsets[i]);
    forces[i] = ((reference.positionsBody[i] - offsets[i]) * kp +
                 (reference.velocitiesBody[i] - relativeVelocity) * kd) *
                (profile.totalMassKg * shares[i] * poseAuthority);
    sumShapeForce += forces[i];
    comBody += offsets[i] * shares[i];
    comVelocityWorld += link.linearVelocity * shares[i];
  }

  // Remove the rigid translation mode from shape assistance. Then cancel
  // the resultant moment, including force lever arms about the actual COM.
  // The shape helper cannot secretly become a whole-body lift or thruster.
  Vec3 shapeMoment;
  for (std::size_t i = 0; i < count; ++i) {
    forces[i] -= sumShapeForce * shares[i];
    shapeMoment += cross(offsets[i] - comBody, forces[i]);
  }
  for (std::size_t i = 0; i < count; ++i)
    torques[i] = -shapeMoment * shares[i];

  const float weight = profile.totalMassKg * 9.81f;
  const Vec3 velocityError =
      reference.velocityHeading -
      headingRotation.conjugate().rotate(comVelocityWorld);
  // Velocity intent in XYZ, not accumulated displacement. In particular,
  // vertical aid is allowed; no absolute-height target exists here.
  const Vec3 movementForceBody =
      inverseBody.rotate(headingRotation.rotate(velocityError)) *
      (profile.totalMassKg * bounded(settings.velocityResponsePerSecond, 2.0f) *
       bounded(reference.movementAuthority, 1.0f));
  const Vec3 upHeading =
      headingRotation.conjugate().rotate(bodyRotation.rotate({0, 0, 1}));
  const Vec3 omegaHeading =
      headingRotation.conjugate().rotate(root.angularVelocity);
  Vec3 balanceTorqueHeading =
      cross(upHeading, reference.upHeading.normalized()) * (weight * 0.1f) -
      Vec3{omegaHeading.x, omegaHeading.y, 0} * 2.0f;
  balanceTorqueHeading.z = 0; // No servo holding an old world heading.
  const Vec3 balanceTorqueBody =
      inverseBody.rotate(headingRotation.rotate(balanceTorqueHeading)) *
      bounded(reference.balanceAuthority, 1.0f);

  for (std::size_t i = 0; i < count; ++i) {
    forces[i] += movementForceBody * shares[i];
    torques[i] += balanceTorqueBody * shares[i];
    result.forceSumNewtons += forces[i].length();
    result.torqueSumNewtonMeters += torques[i].length();
  }
  const float forceBudget =
      weight * bounded(settings.maximumAssistWeightFraction, 0.15f);
  const float torqueBudget =
      weight * bounded(profile.standingRootHeightMeters, 10.0f) *
      bounded(settings.maximumAssistTorqueWeightLengthFraction, 0.05f);
  if (!std::isfinite(result.forceSumNewtons) ||
      !std::isfinite(result.torqueSumNewtonMeters))
    return {};
  float scale = 1;
  if (result.forceSumNewtons > forceBudget)
    scale = std::min(scale, forceBudget / result.forceSumNewtons);
  if (result.torqueSumNewtonMeters > torqueBudget)
    scale = std::min(scale, torqueBudget / result.torqueSumNewtonMeters);
  result.budgetScale = scale;
  result.forceSumNewtons *= scale;
  result.torqueSumNewtonMeters *= scale;
  result.wrenches.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const Vec3 force = bodyRotation.rotate(forces[i] * scale);
    result.wrenches.push_back({static_cast<std::uint32_t>(i), force,
                               bodyRotation.rotate(torques[i] * scale)});
    result.netForceWorld += force;
  }
  return result;
}

} // namespace MatterEngine
