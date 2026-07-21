#include "Engine/Math/Vec3.hpp"

#include <cmath>

namespace MatterEngine {

float Vec3::length() const { return std::sqrt(lengthSquared()); }

Vec3 Vec3::normalized() const {
    const float magnitude = length();
    return magnitude > 0.000001f ? *this / magnitude : Vec3 {};
}

Vec3& operator+=(Vec3& a, Vec3 b) { a = a + b; return a; }
Vec3& operator-=(Vec3& a, Vec3 b) { a = a - b; return a; }
Vec3& operator*=(Vec3& value, float scalar) { value = value * scalar; return value; }

} // namespace MatterEngine
