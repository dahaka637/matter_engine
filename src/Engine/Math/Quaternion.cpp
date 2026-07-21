#include "Engine/Math/Quaternion.hpp"

#include <cmath>

namespace MatterEngine {

Quaternion Quaternion::fromAxisAngle(Vec3 axis, float radians) {
    axis = axis.normalized();
    const float halfAngle = radians * 0.5f;
    const float sine = std::sin(halfAngle);
    return { axis.x * sine, axis.y * sine, axis.z * sine, std::cos(halfAngle) };
}

Quaternion Quaternion::normalized() const {
    const float magnitude = std::sqrt(x * x + y * y + z * z + w * w);
    if (magnitude <= 0.000001f) return {};
    return { x / magnitude, y / magnitude, z / magnitude, w / magnitude };
}

Vec3 Quaternion::rotate(Vec3 vector) const {
    const Quaternion q = normalized();
    const Vec3 imaginary { q.x, q.y, q.z };
    const Vec3 twiceCross = cross(imaginary, vector) * 2.0f;
    return vector + twiceCross * q.w + cross(imaginary, twiceCross);
}

Quaternion operator*(Quaternion a, Quaternion b) {
    return {
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
}

} // namespace MatterEngine
