#pragma once

#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec2.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace MatterEngine {

enum class FootSide3D : std::uint8_t {
    Left = 0,
    Right = 1
};

enum class FootPhase3D : std::uint8_t {
    Planted,
    LiftOff,
    Swing,
    TouchDown,
    Aborting
};

struct FootPose3D {
    Vec3 position;
    Quaternion orientation;
    Vec3 groundNormal { 0.0f, 0.0f, 1.0f };
    float yawRadians = 0.0f;
};

struct FootworkTerrainHit3D {
    Vec3 position;
    Vec3 normal { 0.0f, 0.0f, 1.0f };
    float distanceMeters = 0.0f;
};

// A locomocao conhece somente esta consulta semantica. PhysX, heightfields,
// navmeshes e terrenos procedurais podem implementá-la sem vazar seus tipos
// para o planejador.
class FootworkTerrainProbe3D {
public:
    virtual ~FootworkTerrainProbe3D() = default;

    [[nodiscard]] virtual bool raycastGround(Vec3 candidatePosition,
        float distanceAboveMeters, float distanceBelowMeters,
        FootworkTerrainHit3D& hit) const = 0;
};

struct FootworkConfig3D {
    float footLengthMeters = 0.28f;
    float footWidthMeters = 0.115f;
    float footThicknessMeters = 0.055f;
    float stanceWidthMeters = 0.27f;
    float homeForwardOffsetMeters = 0.055f;
    float velocityLookAheadSeconds = 0.16f;

    float walkSpeedMetersPerSecond = 1.05f;
    float fastSpeedMetersPerSecond = 3.15f;
    float accelerationMetersPerSecondSquared = 4.8f;
    float decelerationMetersPerSecondSquared = 7.2f;
    float fastAccelerationMultiplier = 1.65f;
    float maximumRootYawRateRadiansPerSecond = 3.8f;

    float stepTriggerDistanceMeters = 0.19f;
    float stepReleaseDistanceMeters = 0.115f;
    float stepTriggerYawRadians = 0.22f;
    float maximumStepReachMeters = 0.78f;
    float maximumStepYawRadians = 0.58f;
    float antiCrossingMarginMeters = 0.025f;

    float swingDurationSeconds = 0.34f;
    float minimumSwingDurationSeconds = 0.24f;
    float maximumSwingDurationSeconds = 0.52f;
    // Fase bilateral anterior ao toe-off. O pé permanece imóvel enquanto o
    // controlador desloca o COM para a perna de apoio.
    float weightTransferDurationSeconds = 0.0f;
    float swingClearanceMeters = 0.105f;
    float fastSwingDurationSeconds = 0.215f;
    float fastSwingClearanceMeters = 0.155f;
    // Passadas reflexivas ainda transferem carga para a perna de apoio. A
    // duracao curta preserva reatividade sem levantar instantaneamente o pe
    // que estava sustentando o corpo.
    float recoveryWeightTransferDurationSeconds = 0.10f;
    float emergencyRecoveryWeightTransferDurationSeconds = 0.045f;
    float fastVelocityLookAheadSeconds = 0.205f;
    float walkToeOffPitchRadians = 0.28f;
    float walkTouchDownPitchRadians = -0.22f;
    float fastToeOffPitchRadians = 0.44f;
    float fastTouchDownPitchRadians = -0.34f;
    float liftOffFraction = 0.13f;
    float touchDownFraction = 0.20f;
    float targetChaseStrength = 0.28f;
    float abortDurationSeconds = 0.19f;

    float probeDistanceAboveMeters = 0.75f;
    float probeDistanceBelowMeters = 1.60f;
    float maximumSlopeRadians = 0.872664626f; // 50 graus
    // Desvio máximo em relação ao plano local, não a diferença de altura
    // natural entre a ponta e o calcanhar numa rampa.
    float maximumPatchHeightSpreadMeters = 0.035f;
    float rootSupportLimitMeters = 0.42f;
};

struct FootworkInput3D {
    // x: direita/esquerda; y: frente/trás, ambos relativos ao lookYaw.
    Vec2 movementLocal;
    float lookYawRadians = 0.0f;
    bool fast = false;
};

struct FootworkTerrainPatch3D {
    bool valid = false;
    Vec3 position;
    Vec3 normal { 0.0f, 0.0f, 1.0f };
    float quality = 0.0f;
    float heightSpreadMeters = 0.0f;
    std::uint8_t hitCount = 0;
};

struct FootstepPlan3D {
    bool valid = false;
    FootSide3D side = FootSide3D::Left;
    FootPose3D target;
    float terrainQuality = 0.0f;
};

struct FootworkFootState3D {
    FootPose3D pose;
    FootPose3D home;
    FootPose3D target;
    FootPhase3D phase = FootPhase3D::Planted;
    float phaseProgress = 0.0f;
    float weightTransferProgress = 1.0f;
    float homeErrorMeters = 0.0f;
    float homeYawErrorRadians = 0.0f;
    float terrainQuality = 1.0f;
};

constexpr std::size_t FootworkTrajectorySampleCount3D = 25;
constexpr std::size_t FootworkMaximumSupportVertices3D = 8;

struct FootworkDebugState3D {
    bool initialized = false;
    Vec3 rootGroundPosition;
    Vec3 rootVelocity;
    float rootYawRadians = 0.0f;
    std::array<FootworkFootState3D, 2> feet;
    std::array<Vec3, FootworkTrajectorySampleCount3D> swingTrajectory;
    std::size_t swingTrajectoryCount = 0;
    std::array<Vec3, FootworkMaximumSupportVertices3D> supportPolygon;
    std::size_t supportPolygonCount = 0;
    FootSide3D activeSwingSide = FootSide3D::Left;
    bool hasActiveSwing = false;
    bool terrainBlocked = false;
    std::uint64_t completedStepCount = 0;
};

[[nodiscard]] constexpr std::size_t footIndex3D(FootSide3D side) {
    return side == FootSide3D::Left ? 0u : 1u;
}

[[nodiscard]] constexpr FootSide3D oppositeFoot3D(FootSide3D side) {
    return side == FootSide3D::Left
        ? FootSide3D::Right : FootSide3D::Left;
}

} // namespace MatterEngine
