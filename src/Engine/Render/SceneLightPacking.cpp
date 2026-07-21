#include "Engine/Render/SceneLightPacking.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {

namespace {

constexpr float DegreesToRadians = 3.14159265358979323846f / 180.0f;
// Cosseno de um angulo de 180 graus - "qualquer direcao passa". Usado nos
// dois cossenos de cone quando o tipo da luz nao usa cone algum
// (Directional/Point), para o dado empacotado deixar explicito que esses
// campos nao se aplicam, em vez de carregar o angulo default da struct.
constexpr float NoConeCosine = -1.0f;

} // namespace

std::vector<GpuLightData3D> packSceneLights(std::span<const LightRender3D> lights) {
    std::vector<GpuLightData3D> packed;
    packed.reserve(lights.size());
    for (const LightRender3D& light : lights) {
        GpuLightData3D gpu;
        const float intensity = std::max(light.intensity, 0.0f);
        const Vec3 direction = light.direction.normalized();
        gpu.colorIntensity = { light.color.x, light.color.y, light.color.z, intensity };
        gpu.parameters[1] = static_cast<float>(static_cast<int>(light.type));
        gpu.parameters[2] = light.castsShadow ? 1.0f : 0.0f;

        if (light.type == LightType3D::Spot) {
            const float range = std::max(light.range, 0.0f);
            const float outerHalfAngle = std::clamp(
                light.coneOuterDegrees * 0.5f, 1.0f, 89.0f) * DegreesToRadians;
            const float innerHalfAngle = outerHalfAngle * 0.72f;
            gpu.positionRange = { light.position.x, light.position.y,
                light.position.z, range };
            gpu.directionOuterCosine = { direction.x, direction.y, direction.z,
                std::cos(outerHalfAngle) };
            gpu.parameters[0] = std::cos(innerHalfAngle);
        } else if (light.type == LightType3D::Point) {
            const float range = std::max(light.range, 0.0f);
            gpu.positionRange = { light.position.x, light.position.y,
                light.position.z, range };
            gpu.directionOuterCosine = { direction.x, direction.y, direction.z,
                NoConeCosine };
            gpu.parameters[0] = NoConeCosine;
        } else { // Directional - nem posicao nem alcance/cone se aplicam.
            gpu.positionRange = { light.position.x, light.position.y,
                light.position.z, 0.0f };
            gpu.directionOuterCosine = { direction.x, direction.y, direction.z,
                NoConeCosine };
            gpu.parameters[0] = NoConeCosine;
        }
        packed.push_back(gpu);
    }
    return packed;
}

} // namespace MatterEngine
