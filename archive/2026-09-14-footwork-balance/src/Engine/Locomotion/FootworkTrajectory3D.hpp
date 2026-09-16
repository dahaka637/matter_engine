#pragma once

#include "Engine/Locomotion/FootworkTypes3D.hpp"

namespace MatterEngine {

[[nodiscard]] float smoothStepUnit3D(float value) noexcept;
[[nodiscard]] float cycloidProgress3D(float value) noexcept;
[[nodiscard]] float shortestAngleDelta3D(
    float fromRadians, float toRadians) noexcept;
[[nodiscard]] float moveAngleTowards3D(float currentRadians,
    float targetRadians, float maximumDeltaRadians) noexcept;
[[nodiscard]] Quaternion footOrientation3D(
    Vec3 groundNormal, float yawRadians) noexcept;
[[nodiscard]] FootPose3D interpolateFootSwing3D(
    const FootPose3D& start, const FootPose3D& target,
    float normalizedProgress, float clearanceMeters) noexcept;

} // namespace MatterEngine
