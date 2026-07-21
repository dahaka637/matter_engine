#include "Engine/Audio/AcousticOcclusion3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

constexpr float Pi = 3.14159265358979323846f;
// Curto o suficiente para nao acertar a propria fonte/ouvinte, longo o
// bastante para nao vazar atraves de paredes finas por erro de ponto
// flutuante — o mesmo valor de folga ja usado antes desta funcao existir.
constexpr float RaycastEndSkinMeters = 0.08f;

bool segmentBlocked(const PhysicsScene3D& scene, Vec3 from, Vec3 to) {
    const Vec3 offset = to - from;
    const float distance = offset.length();
    if (distance < 0.0001f) return false;
    const Vec3 direction = offset / distance;
    PhysicsRayHit3D hit;
    return scene.raycastStatic({ from, direction },
        std::max(0.0f, distance - RaycastEndSkinMeters), hit);
}

} // namespace

float computeOcclusionFactor3D(const PhysicsScene3D& scene,
    Vec3 listenerPosition, Vec3 sourcePosition,
    const AcousticOcclusionSettings3D& settings) {
    const Vec3 mainOffset = sourcePosition - listenerPosition;
    const float mainDistance = mainOffset.length();
    if (mainDistance < 0.0001f) return 0.0f;
    const Vec3 mainDirection = mainOffset / mainDistance;

    // Base perpendicular ao caminho direto, para dispor as amostras extras
    // em um circulo ao redor dele (nao ao redor de um eixo do mundo).
    const Vec3 worldUp = std::abs(mainDirection.z) < 0.99f
        ? Vec3 { 0.0f, 0.0f, 1.0f } : Vec3 { 1.0f, 0.0f, 0.0f };
    const Vec3 right = cross(mainDirection, worldUp).normalized();
    const Vec3 up = cross(right, mainDirection).normalized();

    int blockedSamples = segmentBlocked(scene, listenerPosition,
        sourcePosition) ? 1 : 0;
    int totalSamples = 1;

    const int crossSampleCount = std::max(0, settings.crossSampleCount);
    for (int index = 0; index < crossSampleCount; ++index) {
        const float angle = (2.0f * Pi * static_cast<float>(index))
            / static_cast<float>(crossSampleCount);
        const Vec3 perpendicular = (right * std::cos(angle)
            + up * std::sin(angle)) * settings.crossSampleRadiusMeters;
        ++totalSamples;
        if (segmentBlocked(scene, listenerPosition + perpendicular,
                sourcePosition + perpendicular)) {
            ++blockedSamples;
        }
    }

    return static_cast<float>(blockedSamples)
        / static_cast<float>(totalSamples);
}

} // namespace MatterEngine
