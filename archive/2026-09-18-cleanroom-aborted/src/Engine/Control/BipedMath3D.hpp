#pragma once

#include "Engine/Math/Quaternion.hpp"

#include <span>
#include <vector>

namespace MatterEngine {

[[nodiscard]] bool finiteBiped(Vec3 value);
[[nodiscard]] bool finiteBiped(Quaternion value);
[[nodiscard]] float clamp01Biped(float value);
[[nodiscard]] float smoothstepBiped(float value);
[[nodiscard]] Vec3 clampLengthBiped(Vec3 value, float maximumLength);
[[nodiscard]] float componentBiped(Vec3 value, std::size_t axis);
void setComponentBiped(Vec3& value, std::size_t axis, float component);
[[nodiscard]] Quaternion yawQuaternionBiped(float radians);
[[nodiscard]] float yawFromForwardBiped(Vec3 forwardWorld);
[[nodiscard]] Vec3 orientationErrorBiped(
    Quaternion desired, Quaternion current);
[[nodiscard]] Quaternion quaternionFromToBiped(Vec3 from, Vec3 to);
[[nodiscard]] Quaternion alignOrientationUpBiped(
    Quaternion baseOrientation, Vec3 localUp, Vec3 desiredWorldUp);
[[nodiscard]] std::vector<Vec3> convexHullXYBiped(
    std::span<const Vec3> points);
[[nodiscard]] float signedDistanceToConvexPolygonXYBiped(
    Vec3 point, std::span<const Vec3> polygon);
[[nodiscard]] Vec3 closestPointOnSegmentXYBiped(
    Vec3 point, Vec3 a, Vec3 b);

} // namespace MatterEngine
