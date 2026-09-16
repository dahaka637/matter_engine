#pragma once

#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/Vec3.hpp"

#include <array>
#include <cmath>

namespace MatterEngine {

// Espectro compacto do novo oceano procedural. Ele descreve apenas as ondas
// que deslocam a superfície e afetam a física; detalhes muito pequenos ficam
// exclusivamente no shader e nunca fingem alterar a colisão.
struct OceanBand3D {
    Vec2 direction;
    float wavelengthMeters = 1.0f;
    float amplitudeMeters = 0.0f;
    float phaseRadians = 0.0f;
    float speedScale = 1.0f;
};

inline const std::array<OceanBand3D, 4> OceanSpectrum {{
    { { 0.894427f, 0.447214f }, 36.0f, 0.155f, 0.2f, 0.82f },
    { { 0.316228f, -0.948683f }, 19.0f, 0.072f, 2.1f, 0.95f },
    { { -0.780869f, 0.624695f }, 9.5f, 0.031f, 4.0f, 1.08f },
    { { 0.980581f, -0.196116f }, 4.8f, 0.012f, 5.3f, 1.18f }
}};

inline constexpr float OceanMaximumDisplacementMeters = 0.270f;

struct OceanSurfaceSample3D {
    float heightMeters = 0.0f;
    float verticalSpeedMetersPerSecond = 0.0f;
    Vec3 normal { 0.0f, 0.0f, 1.0f };
};

[[nodiscard]] inline OceanSurfaceSample3D evaluateOceanSurface(
    Vec2 position, float meanSeaLevelMeters, float timeSeconds) {
    constexpr float Tau = 6.28318530717958647692f;
    constexpr float Gravity = 9.81f;
    OceanSurfaceSample3D result;
    result.heightMeters = meanSeaLevelMeters;
    float derivativeX = 0.0f;
    float derivativeY = 0.0f;
    for (const OceanBand3D& band : OceanSpectrum) {
        const float waveNumber = Tau / band.wavelengthMeters;
        const float angularSpeed =
            std::sqrt(Gravity * waveNumber) * band.speedScale;
        const float phase = waveNumber
                * (band.direction.x * position.x
                    + band.direction.y * position.y)
            - angularSpeed * timeSeconds + band.phaseRadians;
        const float sine = std::sin(phase);
        const float cosine = std::cos(phase);
        result.heightMeters += band.amplitudeMeters * sine;
        result.verticalSpeedMetersPerSecond -=
            band.amplitudeMeters * angularSpeed * cosine;
        const float derivative =
            band.amplitudeMeters * waveNumber * cosine;
        derivativeX += band.direction.x * derivative;
        derivativeY += band.direction.y * derivative;
    }
    result.normal =
        Vec3 { -derivativeX, -derivativeY, 1.0f }.normalized();
    return result;
}

} // namespace MatterEngine
