#pragma once

#include "Engine/Math/Vec3.hpp"

namespace MatterEngine {

struct Quaternion {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float w = 1.0f;

    constexpr Quaternion() = default;
    constexpr Quaternion(float xValue, float yValue, float zValue, float wValue)
        : x(xValue), y(yValue), z(zValue), w(wValue) {}

    [[nodiscard]] static Quaternion fromAxisAngle(Vec3 axis, float radians);
    [[nodiscard]] Quaternion normalized() const;
    [[nodiscard]] Quaternion conjugate() const { return { -x, -y, -z, w }; }
    [[nodiscard]] Vec3 rotate(Vec3 vector) const;
};

Quaternion operator*(Quaternion a, Quaternion b);

} // namespace MatterEngine
