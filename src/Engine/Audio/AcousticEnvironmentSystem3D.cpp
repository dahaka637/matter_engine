#include "Engine/Audio/AcousticEnvironmentSystem3D.hpp"

#include <algorithm>

namespace MatterEngine {
namespace {

struct WeightedZone3D {
    const AcousticZoneDefinition3D* zone = nullptr;
    // Peso geometrico bruto (1 no centro, cai linearmente ate 0 na borda
    // externa raio+blend_distance) - ainda nao inclui wetSendGain nem a
    // normalizacao entre zonas sobrepostas.
    float weight = 0.0f;
};

} // namespace

AcousticEnvironmentBlend3D AcousticEnvironmentSystem3D::evaluate(
    Vec3 listenerPosition) const {
    std::vector<WeightedZone3D> candidates;
    candidates.reserve(m_zones.size());
    for (const AcousticZoneDefinition3D& zone : m_zones) {
        const float distance = (listenerPosition - zone.center).length();
        const float outerRadius = zone.radiusMeters + zone.blendDistanceMeters;
        if (distance >= outerRadius) continue;
        float weight = 1.0f;
        if (distance > zone.radiusMeters
            && zone.blendDistanceMeters > 0.0001f) {
            weight = 1.0f
                - (distance - zone.radiusMeters) / zone.blendDistanceMeters;
        }
        candidates.push_back({ &zone, std::clamp(weight, 0.0f, 1.0f) });
    }

    // As zonas com maior peso (mais proximas/mais dentro) ganham os slots
    // disponiveis; o resto simplesmente nao soa (sem um terceiro slot para
    // colocar).
    std::sort(candidates.begin(), candidates.end(),
        [](const WeightedZone3D& left, const WeightedZone3D& right) {
            return left.weight > right.weight;
        });

    AcousticEnvironmentBlend3D blend;
    const std::size_t activeCount = std::min(candidates.size(),
        blend.slots.size());

    // Normaliza so quando a soma bruta ultrapassa 1 (duas zonas fortes
    // sobrepostas) - uma unica zona isolada preserva seu proprio
    // desvanecimento por distancia sem ser "esticada" de volta para 1.
    float totalWeight = 0.0f;
    for (std::size_t index = 0; index < activeCount; ++index) {
        totalWeight += candidates[index].weight;
    }
    const float scale = totalWeight > 1.0f ? 1.0f / totalWeight : 1.0f;

    for (std::size_t index = 0; index < activeCount; ++index) {
        AcousticEnvironmentBlend3D::Slot& slot = blend.slots[index];
        slot.active = true;
        slot.preset = candidates[index].zone->preset;
        slot.wetSendGain = candidates[index].weight * scale
            * candidates[index].zone->wetSendGain;
    }
    return blend;
}

} // namespace MatterEngine
