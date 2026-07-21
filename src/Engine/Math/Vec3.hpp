#pragma once

namespace MatterEngine {

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    constexpr Vec3() = default;
    constexpr Vec3(float xValue, float yValue, float zValue) : x(xValue), y(yValue), z(zValue) {}

    [[nodiscard]] float length() const;
    [[nodiscard]] constexpr float lengthSquared() const { return x * x + y * y + z * z; }
    [[nodiscard]] Vec3 normalized() const;
};

constexpr Vec3 operator+(Vec3 a, Vec3 b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
constexpr Vec3 operator-(Vec3 a, Vec3 b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
constexpr Vec3 operator-(Vec3 value) { return { -value.x, -value.y, -value.z }; }
constexpr Vec3 operator*(Vec3 value, float scalar) { return { value.x * scalar, value.y * scalar, value.z * scalar }; }
constexpr Vec3 operator*(float scalar, Vec3 value) { return value * scalar; }
constexpr Vec3 operator/(Vec3 value, float scalar) { return { value.x / scalar, value.y / scalar, value.z / scalar }; }
constexpr float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
constexpr Vec3 cross(Vec3 a, Vec3 b) {
    return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}

Vec3& operator+=(Vec3& a, Vec3 b);
Vec3& operator-=(Vec3& a, Vec3 b);
Vec3& operator*=(Vec3& value, float scalar);

} // namespace MatterEngine
