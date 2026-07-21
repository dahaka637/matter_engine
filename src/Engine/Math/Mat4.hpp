#pragma once

#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec2.hpp"
#include <array>

namespace MatterEngine {

// Column-major, column-vector matrix. The engine uses a right-handed world,
// -Z forward, meters, radians, and Vulkan's 0..1 clip-space depth.
// perspective()/perspectiveJittered() produce REVERSED depth (near=1,
// far=0, ver Mat4.cpp) - orthographic() continues with standard depth
// (near=0, far=1), used only by the shadow-map light camera today.
struct Mat4 {
    std::array<float, 16> values {};

    [[nodiscard]] static Mat4 identity();
    [[nodiscard]] static Mat4 translation(Vec3 position);
    [[nodiscard]] static Mat4 scale(Vec3 scale);
    [[nodiscard]] static Mat4 rotation(Quaternion rotation);
    [[nodiscard]] static Mat4 perspective(float verticalFovRadians, float aspect, float nearPlane, float farPlane);
    // Identica a perspective(), mas desloca o resultado em espaco NDC (nao
    // pixels - quem chama converte a amostra de jitter num offset NDC usando
    // o extent alvo, ver JitterSequence.hpp) para o TAA (Fase 6) sobrepor
    // amostras sub-pixel entre quadros. ndcJitter={0,0} produz exatamente a
    // mesma matriz que perspective().
    [[nodiscard]] static Mat4 perspectiveJittered(float verticalFovRadians, float aspect,
        float nearPlane, float farPlane, Vec2 ndcJitter);
    [[nodiscard]] static Mat4 orthographic(float left, float right, float bottom, float top,
        float nearPlane, float farPlane);
    [[nodiscard]] static Mat4 lookAt(Vec3 eye, Vec3 target, Vec3 up);
    [[nodiscard]] Mat4 inverse() const;

    [[nodiscard]] float& at(int row, int column) { return values[static_cast<std::size_t>(column * 4 + row)]; }
    [[nodiscard]] float at(int row, int column) const { return values[static_cast<std::size_t>(column * 4 + row)]; }
    [[nodiscard]] const float* data() const { return values.data(); }
};

Mat4 operator*(const Mat4& a, const Mat4& b);

struct Transform3D {
    Vec3 position;
    Quaternion rotation;
    Vec3 scale { 1.0f, 1.0f, 1.0f };

    [[nodiscard]] Mat4 matrix() const;
};

} // namespace MatterEngine
