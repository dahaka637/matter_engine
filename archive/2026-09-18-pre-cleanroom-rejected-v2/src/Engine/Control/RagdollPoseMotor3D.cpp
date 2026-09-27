#include "Engine/Control/RagdollPoseMotor3D.hpp"
#include <algorithm>
#include <array>
#include <cmath>

namespace MatterEngine {
namespace {
float &axis(Vec3 &v, int a) { return a == 0 ? v.x : a == 1 ? v.y : v.z; }
float axis(const Vec3 &v, int a) { return a == 0 ? v.x : a == 1 ? v.y : v.z; }
bool finite(Vec3 v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
Vec3 exponentialJacobian(Vec3 q, Vec3 v) {
  const float squared = q.lengthSquared();
  const float angle = std::sqrt(squared);
  const float a =
      squared < 0.0001f ? 0.5f - squared / 24 : (1 - std::cos(angle)) / squared;
  const float b = squared < 0.0001f
                      ? 1.0f / 6 - squared / 120
                      : (angle - std::sin(angle)) / (squared * angle);
  return v + cross(q, v) * a + cross(q, cross(q, v)) * b;
}
Quaternion rotation(Vec3 v) {
  const float n = v.length();
  return n < 1e-6f ? Quaternion{} : Quaternion::fromAxisAngle(v / n, n);
}
Vec3 rotationError(Quaternion desired, Quaternion current) {
  auto d = (desired * current.conjugate()).normalized();
  if (d.w < 0)
    d = {-d.x, -d.y, -d.z, -d.w};
  const Vec3 v{d.x, d.y, d.z};
  const float n = v.length();
  return n < 1e-6f ? Vec3{} : v * (2 * std::atan2(n, d.w) / n);
}
} // namespace
Vec3 ragdollJointTargetVelocity3D(Vec3 target, Vec3 velocity, Vec3 measured) {
  if (!finite(target) || !finite(velocity) || !finite(measured))
    return {};
  return rotation(measured).conjugate().rotate(
      exponentialJacobian(target, velocity));
}
void buildRagdollJointPose3D(const RagdollProfile3D &p, std::span<const Vec3> q,
                             RagdollAnimationPose3D &pose) {
  pose.linkPositions.resize(p.links.size());
  pose.linkOrientations.resize(p.links.size());
  for (std::size_t i = 0; i < p.links.size(); ++i) {
    const auto &l = p.links[i];
    if (l.parentIndex < 0) {
      pose.linkPositions[i] = {};
      pose.linkOrientations[i] = {};
      continue;
    }
    const auto parent = static_cast<std::size_t>(l.parentIndex);
    const auto &pl = p.links[parent];
    auto angles = i < q.size() ? q[i] : Vec3{};
    for (int a = 0; a < 3; ++a) {
      const auto &limit = l.inboundJoint.axes[a];
      axis(angles, a) = limit.enabled
                            ? std::clamp(axis(angles, a), limit.minimumRadians,
                                         limit.maximumRadians)
                            : 0;
    }
    const auto frame = l.inboundJoint.frameModelOrientation;
    pose.linkOrientations[i] =
        (pose.linkOrientations[parent] * pl.modelOrientation.conjugate() *
         frame * rotation(angles) *
         (l.modelOrientation.conjugate() * frame).conjugate())
            .normalized();
    const auto anchor = l.inboundJoint.anchorModelPosition;
    pose.linkPositions[i] =
        pose.linkPositions[parent] +
        pose.linkOrientations[parent].rotate(
            pl.modelOrientation.conjugate().rotate(anchor - pl.modelPosition)) -
        pose.linkOrientations[i].rotate(
            l.modelOrientation.conjugate().rotate(anchor - l.modelPosition));
  }
}
void fitRagdollFootTarget3D(const RagdollProfile3D &p, std::vector<Vec3> &q,
                            std::size_t foot, Vec3 target,
                            Quaternion orientation, float weight) {
  if (foot >= p.links.size() || q.size() != p.links.size() ||
      !std::isfinite(weight) || weight <= 0 || !finite(target) ||
      !std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
      !std::isfinite(orientation.z) || !std::isfinite(orientation.w))
    return;
  for (const auto angles : q)
    if (!finite(angles))
      return;
  struct Dof {
    std::size_t link;
    int a;
  };
  std::array<Dof, 9> dofs{};
  int count = 0;
  int link = static_cast<int>(foot);
  for (int depth = 0; depth < 3 && link > 0; ++depth) {
    for (int a = 0; a < 3; ++a)
      if (p.links[link].inboundJoint.axes[a].enabled)
        dofs[count++] = {static_cast<std::size_t>(link), a};
    link = p.links[link].parentIndex;
  }
  const auto original = q;
  RagdollAnimationPose3D base;
  // A fixed eight iterations with <=9 variables, independent of error.
  // Damped least squares remains well-conditioned near a straight knee.
  for (int iteration = 0; iteration < 8; ++iteration) {
    buildRagdollJointPose3D(p, q, base);
    const auto dp = (target - base.linkPositions[foot]) * 4;
    const auto dr =
        rotationError(orientation, base.linkOrientations[foot]) * 0.55f;
    const std::array<float, 6> error{dp.x, dp.y, dp.z, dr.x, dr.y, dr.z};
    float jac[6][9]{};
    for (int j = 0; j < count; ++j) {
      const auto d = dofs[j];
      const auto &definition = p.links[d.link];
      const auto parent = static_cast<std::size_t>(definition.parentIndex);
      const auto &pd = p.links[parent];
      const auto parentFrame =
          base.linkOrientations[parent] * pd.modelOrientation.conjugate();
      const Vec3 basis = d.a == 0   ? Vec3{1, 0, 0}
                         : d.a == 1 ? Vec3{0, 1, 0}
                                    : Vec3{0, 0, 1};
      // q is an exponential rotation vector, not three Euler angles.
      // Its analytic left Jacobian avoids differencing float FK poses.
      const auto angular =
          (parentFrame * definition.inboundJoint.frameModelOrientation)
              .rotate(exponentialJacobian(q[d.link], basis));
      const auto anchor =
          base.linkPositions[parent] +
          parentFrame.rotate(definition.inboundJoint.anchorModelPosition -
                             pd.modelPosition);
      const auto jp = cross(angular, base.linkPositions[foot] - anchor) * 4;
      const auto jr = angular * 0.55f;
      jac[0][j] = jp.x;
      jac[1][j] = jp.y;
      jac[2][j] = jp.z;
      jac[3][j] = jr.x;
      jac[4][j] = jr.y;
      jac[5][j] = jr.z;
    }
    float system[9][10]{};
    for (int j = 0; j < count; ++j) {
      for (int k = 0; k < count; ++k) {
        for (int r = 0; r < 6; ++r)
          system[j][k] += jac[r][j] * jac[r][k];
        if (j == k)
          system[j][k] += 0.035f;
      }
      for (int r = 0; r < 6; ++r)
        system[j][count] += jac[r][j] * error[r];
    }
    for (int pivot = 0; pivot < count; ++pivot) {
      const float divisor = system[pivot][pivot];
      for (int c = pivot; c <= count; ++c)
        system[pivot][c] /= divisor;
      for (int r = 0; r < count; ++r)
        if (r != pivot) {
          const float f = system[r][pivot];
          for (int c = pivot; c <= count; ++c)
            system[r][c] -= f * system[pivot][c];
        }
    }
    for (int j = 0; j < count; ++j) {
      const auto d = dofs[j];
      const auto &limit = p.links[d.link].inboundJoint.axes[d.a];
      axis(q[d.link], d.a) = std::clamp(
          axis(q[d.link], d.a) + std::clamp(system[j][count], -0.15f, 0.15f),
          limit.minimumRadians, limit.maximumRadians);
    }
  }
  for (int j = 0; j < count; ++j) {
    const auto d = dofs[j];
    axis(q[d.link], d.a) =
        axis(original[d.link], d.a) * (1 - std::clamp(weight, 0.0f, 1.0f)) +
        axis(q[d.link], d.a) * std::clamp(weight, 0.0f, 1.0f);
  }
}
} // namespace MatterEngine
