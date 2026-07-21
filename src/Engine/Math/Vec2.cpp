#include "Engine/Math/Vec2.hpp"
#include <cmath>

namespace MatterEngine {

float Vec2::length() const { return std::sqrt(x * x + y * y); }
float Vec2::lengthSquared() const { return x * x + y * y; }

Vec2 Vec2::normalized() const {
    const float len = length();
    if (len <= 0.00001f) return {};
    return { x / len, y / len };
}

Vec2 operator+(Vec2 a, Vec2 b) { return { a.x + b.x, a.y + b.y }; }
Vec2 operator-(Vec2 a, Vec2 b) { return { a.x - b.x, a.y - b.y }; }
Vec2 operator-(Vec2 v) { return { -v.x, -v.y }; }
Vec2 operator*(Vec2 v, float scalar) { return { v.x * scalar, v.y * scalar }; }
Vec2 operator/(Vec2 v, float scalar) { return { v.x / scalar, v.y / scalar }; }

Vec2& operator+=(Vec2& a, Vec2 b) {
    a.x += b.x;
    a.y += b.y;
    return a;
}

Vec2& operator-=(Vec2& a, Vec2 b) {
    a.x -= b.x;
    a.y -= b.y;
    return a;
}

Vec2& operator*=(Vec2& v, float scalar) {
    v.x *= scalar;
    v.y *= scalar;
    return v;
}

float dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
float cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }

Vec2 rotate(Vec2 v, float radians) {
    const float c = std::cos(radians);
    const float s = std::sin(radians);
    return { v.x * c - v.y * s, v.x * s + v.y * c };
}

}
