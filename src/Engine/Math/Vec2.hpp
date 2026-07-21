#pragma once

namespace MatterEngine {

struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;

    Vec2() = default;
    Vec2(float xValue, float yValue) : x(xValue), y(yValue) {}

    [[nodiscard]] float length() const;
    [[nodiscard]] float lengthSquared() const;
    [[nodiscard]] Vec2 normalized() const;
};

Vec2 operator+(Vec2 a, Vec2 b);
Vec2 operator-(Vec2 a, Vec2 b);
Vec2 operator-(Vec2 v);
Vec2 operator*(Vec2 v, float scalar);
Vec2 operator/(Vec2 v, float scalar);
Vec2& operator+=(Vec2& a, Vec2 b);
Vec2& operator-=(Vec2& a, Vec2 b);
Vec2& operator*=(Vec2& v, float scalar);
float dot(Vec2 a, Vec2 b);
float cross(Vec2 a, Vec2 b);
Vec2 rotate(Vec2 v, float radians);

}
