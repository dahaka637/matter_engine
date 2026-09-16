#include "Engine/Locomotion/FootworkTrajectory3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr float TwoPi = Pi * 2.0f;

Quaternion quaternionFromBasis(Vec3 xAxis, Vec3 yAxis, Vec3 zAxis) noexcept {
    // Matriz coluna-major conceitual: os eixos locais X/Y/Z expressos no
    // mundo. A conversão evita Euler e continua estável em qualquer inclinação.
    const float m00 = xAxis.x;
    const float m01 = yAxis.x;
    const float m02 = zAxis.x;
    const float m10 = xAxis.y;
    const float m11 = yAxis.y;
    const float m12 = zAxis.y;
    const float m20 = xAxis.z;
    const float m21 = yAxis.z;
    const float m22 = zAxis.z;
    const float trace = m00 + m11 + m22;
    Quaternion result;
    if (trace > 0.0f) {
        const float scale = std::sqrt(trace + 1.0f) * 2.0f;
        result.w = 0.25f * scale;
        result.x = (m21 - m12) / scale;
        result.y = (m02 - m20) / scale;
        result.z = (m10 - m01) / scale;
    } else if (m00 > m11 && m00 > m22) {
        const float scale = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f;
        result.w = (m21 - m12) / scale;
        result.x = 0.25f * scale;
        result.y = (m01 + m10) / scale;
        result.z = (m02 + m20) / scale;
    } else if (m11 > m22) {
        const float scale = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f;
        result.w = (m02 - m20) / scale;
        result.x = (m01 + m10) / scale;
        result.y = 0.25f * scale;
        result.z = (m12 + m21) / scale;
    } else {
        const float scale = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f;
        result.w = (m10 - m01) / scale;
        result.x = (m02 + m20) / scale;
        result.y = (m12 + m21) / scale;
        result.z = 0.25f * scale;
    }
    return result.normalized();
}

} // namespace

float smoothStepUnit3D(float value) noexcept {
    const float t = std::clamp(value, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

float cycloidProgress3D(float value) noexcept {
    const float t = std::clamp(value, 0.0f, 1.0f);
    return t - std::sin(TwoPi * t) / TwoPi;
}

float shortestAngleDelta3D(
    float fromRadians, float toRadians) noexcept {
    return std::remainder(toRadians - fromRadians, TwoPi);
}

float moveAngleTowards3D(float currentRadians, float targetRadians,
    float maximumDeltaRadians) noexcept {
    const float delta = shortestAngleDelta3D(currentRadians, targetRadians);
    return currentRadians + std::clamp(
        delta, -maximumDeltaRadians, maximumDeltaRadians);
}

Quaternion footOrientation3D(
    Vec3 groundNormal, float yawRadians) noexcept {
    if (groundNormal.lengthSquared() < 1.0e-8f) {
        groundNormal = { 0.0f, 0.0f, 1.0f };
    } else {
        groundNormal = groundNormal.normalized();
    }
    const Vec3 desiredForward {
        std::cos(yawRadians), std::sin(yawRadians), 0.0f
    };
    Vec3 forward = desiredForward
        - groundNormal * dot(desiredForward, groundNormal);
    if (forward.lengthSquared() < 1.0e-8f) {
        forward = cross({ 0.0f, 1.0f, 0.0f }, groundNormal);
    }
    forward = forward.normalized();
    const Vec3 left = cross(groundNormal, forward).normalized();
    forward = cross(left, groundNormal).normalized();
    return quaternionFromBasis(forward, left, groundNormal);
}

FootPose3D interpolateFootSwing3D(const FootPose3D& start,
    const FootPose3D& target, float normalizedProgress,
    float clearanceMeters) noexcept {
    const float t = std::clamp(normalizedProgress, 0.0f, 1.0f);
    const float horizontal = cycloidProgress3D(t);
    // sin(pi*t) chega ao chão com derivada -pi: uma passada de 11,5 cm em
    // 360 ms comandava aproximadamente 1 m/s para baixo no touchdown. Isso
    // transformava cada pouso em um impulso e contaminava a transferência de
    // peso seguinte. sin² preserva a mesma altura máxima, mas zera velocidade
    // vertical tanto no toe-off quanto no pouso.
    const float verticalSine = std::sin(Pi * t);
    const float verticalArc = verticalSine * verticalSine;
    FootPose3D pose;
    pose.position = start.position
        + (target.position - start.position) * horizontal;
    pose.position.z += std::max(0.0f, clearanceMeters) * verticalArc;
    pose.groundNormal = (start.groundNormal
        + (target.groundNormal - start.groundNormal)
            * smoothStepUnit3D(t)).normalized();
    pose.yawRadians = start.yawRadians
        + shortestAngleDelta3D(start.yawRadians, target.yawRadians)
            * smoothStepUnit3D(t);
    pose.orientation = footOrientation3D(
        pose.groundNormal, pose.yawRadians);
    return pose;
}

} // namespace MatterEngine
