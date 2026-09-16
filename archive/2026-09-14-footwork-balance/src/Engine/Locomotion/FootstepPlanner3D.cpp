#include "Engine/Locomotion/FootstepPlanner3D.hpp"

#include "Engine/Locomotion/FootworkTrajectory3D.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace MatterEngine {
namespace {

} // namespace

FootworkTerrainPatch3D sampleFootworkTerrainPatch3D(
    const FootworkTerrainProbe3D& terrain, Vec3 candidatePosition,
    float candidateYawRadians, const FootworkConfig3D& config) {
    const Vec3 forward {
        std::cos(candidateYawRadians), std::sin(candidateYawRadians), 0.0f
    };
    const Vec3 left { -forward.y, forward.x, 0.0f };
    const float halfLength = std::max(0.02f,
        config.footLengthMeters * 0.37f);
    const float halfWidth = std::max(0.015f,
        config.footWidthMeters * 0.34f);
    const std::array<Vec3, 5> offsets {
        Vec3 {},
        forward * halfLength + left * halfWidth,
        forward * halfLength - left * halfWidth,
        -forward * halfLength + left * halfWidth,
        -forward * halfLength - left * halfWidth
    };

    FootworkTerrainPatch3D result;
    std::array<FootworkTerrainHit3D, offsets.size()> hits {};
    std::array<bool, offsets.size()> hasHit {};
    Vec3 normalSum;
    float centerHeight = candidatePosition.z;
    float minimumHeight = std::numeric_limits<float>::max();
    float maximumHeight = -std::numeric_limits<float>::max();
    for (std::size_t index = 0; index < offsets.size(); ++index) {
        FootworkTerrainHit3D hit;
        if (!terrain.raycastGround(candidatePosition + offsets[index],
                config.probeDistanceAboveMeters,
                config.probeDistanceBelowMeters, hit)) {
            continue;
        }
        hits[index] = hit;
        hasHit[index] = true;
        if (index == 0) {
            centerHeight = hit.position.z;
            result.position = hit.position;
        }
        normalSum += hit.normal.normalized();
        minimumHeight = std::min(minimumHeight, hit.position.z);
        maximumHeight = std::max(maximumHeight, hit.position.z);
        ++result.hitCount;
    }
    // Sem a amostra central não existe uma posição de contato inequívoca.
    if (!hasHit[0] || result.hitCount < 4) return result;

    result.normal = normalSum.normalized();
    result.position.x = candidatePosition.x;
    result.position.y = candidatePosition.y;
    result.position.z = centerHeight;
    result.heightSpreadMeters = maximumHeight - minimumHeight;
    const float slopeRadians = std::acos(
        std::clamp(result.normal.z, -1.0f, 1.0f));
    if (slopeRadians > config.maximumSlopeRadians
        || result.normal.z < 0.08f) {
        return result;
    }

    // Uma rampa plana naturalmente tem grande variação de altura entre
    // calcanhar e ponta. O código antigo confundia isso com irregularidade e
    // rejeitava qualquer inclinação mais séria. Medimos agora o desvio das
    // amostras em relação ao plano local; só degraus/bordas são penalizados.
    float maximumPlaneResidual = 0.0f;
    for (std::size_t index = 1; index < offsets.size(); ++index) {
        if (!hasHit[index]) continue;
        const Vec3 delta = hits[index].position - hits[0].position;
        const float planeResidual = std::abs(dot(delta, result.normal));
        maximumPlaneResidual = std::max(
            maximumPlaneResidual, planeResidual);
    }
    if (maximumPlaneResidual
        > config.maximumPatchHeightSpreadMeters) {
        return result;
    }

    const float completeness = static_cast<float>(result.hitCount) / 5.0f;
    const float slopeQuality = 1.0f - std::clamp(
        slopeRadians / std::max(0.01f, config.maximumSlopeRadians),
        0.0f, 1.0f);
    const float roughnessQuality = 1.0f - std::clamp(
        maximumPlaneResidual
            / std::max(0.001f, config.maximumPatchHeightSpreadMeters),
        0.0f, 1.0f);
    result.quality = completeness
        * (0.48f + 0.30f * slopeQuality + 0.22f * roughnessQuality);
    result.valid = true;
    return result;
}

FootstepPlan3D planFootstep3D(
    const FootstepPlanningContext3D& context,
    const FootworkTerrainProbe3D& terrain,
    const FootworkConfig3D& config) {
    FootstepPlan3D result;
    result.side = context.side;

    // Decompomos o alcance no referencial do corpo antes de limitá-lo.
    // O clamp radial antigo encurtava também a largura da base em passadas
    // longas; depois o anti-crossing empurrava o alvo de lado, criando uma
    // diagonal/curva visível sobretudo na corrida.
    const Vec3 forward {
        std::cos(context.rootYawRadians),
        std::sin(context.rootYawRadians), 0.0f
    };
    const Vec3 left { -forward.y, forward.x, 0.0f };
    const Vec3 desiredOffset =
        context.desiredHome.position - context.supportFoot.position;
    const float maximumReach =
        std::max(0.05f, config.maximumStepReachMeters);
    float lateral = dot(desiredOffset, left);
    const float requiredSign =
        context.side == FootSide3D::Left ? 1.0f : -1.0f;
    // A margem antiga era aplicada aos centros, permitindo que duas caixas de
    // pé ocupassem praticamente o mesmo espaço. A largura inteira separa as
    // geometrias; a margem configura apenas a folga adicional.
    const float minimumLateralSeparation =
        config.footWidthMeters + config.antiCrossingMarginMeters;
    if (!context.allowCrossing
        && lateral * requiredSign < minimumLateralSeparation) {
        lateral = requiredSign * minimumLateralSeparation;
    }
    lateral = std::clamp(lateral, -maximumReach, maximumReach);
    const float remainingForwardReach = std::sqrt(std::max(
        0.0f, maximumReach * maximumReach - lateral * lateral));
    const float sagittal = std::clamp(
        dot(desiredOffset, forward),
        -remainingForwardReach, remainingForwardReach);
    Vec3 candidate = context.supportFoot.position
        + forward * sagittal + left * lateral;
    candidate.z = context.desiredHome.position.z;

    const float supportYaw = context.supportFoot.yawRadians;
    const float targetYaw = supportYaw + std::clamp(
        shortestAngleDelta3D(supportYaw,
            context.desiredHome.yawRadians),
        -config.maximumStepYawRadians,
        config.maximumStepYawRadians);
    const FootworkTerrainPatch3D patch =
        sampleFootworkTerrainPatch3D(
            terrain, candidate, targetYaw, config);
    if (!patch.valid) return result;

    result.target.position = patch.position;
    result.target.groundNormal = patch.normal;
    result.target.yawRadians = targetYaw;
    result.target.orientation = footOrientation3D(
        patch.normal, targetYaw);
    result.terrainQuality = patch.quality;
    result.valid = true;
    return result;
}

} // namespace MatterEngine
